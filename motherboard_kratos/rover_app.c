/*
 * rover_app.c : micro-ROS + FreeRTOS (CMSIS-RTOS v2) skeleton for STM32
 *
 * Priorities: Drive > Arm > micro-ROS > Life detection
 * Rule: ONLY microros_task touches rcl/rclc objects.
 *       Other tasks block on queues / delays, never busy-loop.
 *
 * Setup:
 *  - Add micro_ros_stm32cubemx_utils to the project (provides the
 *    cubemx_transport_* and microros_* allocator functions).
 *  - Call rover_app_init() from main() after MX_FREERTOS_Init() or inside
 *    a USER CODE block, BEFORE osKernelStart().
 *  - Do NOT also let CubeMX create tasks with the same names.
 */

#include "main.h"
#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include <string.h>
#include <stdbool.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>

#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <std_msgs/msg/bool.h>

/* ------------------------------------------------------------------ */
/* Config                                                             */
/* ------------------------------------------------------------------ */
#define ARM_JOINTS          6
#define DRIVE_TIMEOUT_MS    500     /* no cmd_vel for this long -> stop   */
#define LIFE_PERIOD_MS      100
#define SPIN_TIMEOUT_MS     10

extern UART_HandleTypeDef huart2;   /* transport UART (change if needed)  */
extern IWDG_HandleTypeDef hiwdg;    /* enable IWDG in CubeMX (~3 s timeout) */

/* Fed ONLY from microros_task, and ONLY while the agent link is healthy.
 * If the link drops, or microros_task hangs, nobody feeds it -> MCU resets. */
#define WDG_FEED()  HAL_IWDG_Refresh(&hiwdg)

/* from micro_ros_stm32cubemx_utils */
bool   cubemx_transport_open(struct uxrCustomTransport *t);
bool   cubemx_transport_close(struct uxrCustomTransport *t);
size_t cubemx_transport_write(struct uxrCustomTransport *t, const uint8_t *buf, size_t len, uint8_t *err);
size_t cubemx_transport_read(struct uxrCustomTransport *t, uint8_t *buf, size_t len, int timeout, uint8_t *err);
void  *microros_allocate(size_t size, void *state);
void   microros_deallocate(void *ptr, void *state);
void  *microros_reallocate(void *ptr, size_t size, void *state);
void  *microros_zero_allocate(size_t n, size_t size, void *state);

/* ------------------------------------------------------------------ */
/* Your hardware layer (implement these; plain functions, no ROS)      */
/* ------------------------------------------------------------------ */
void drive_apply(float lin, float ang);        /* set PWM / direction     */
void arm_apply(const float q[ARM_JOINTS]);     /* set servo/stepper goals */
bool life_sensor_read(void);                   /* read the sensor         */

/* ------------------------------------------------------------------ */
/* Shared data (queues of length 1 = "latest value wins")              */
/* ------------------------------------------------------------------ */
typedef struct { float lin, ang; } DriveCmd_t;
typedef struct { float q[ARM_JOINTS]; } ArmCmd_t;

static QueueHandle_t driveQ, armQ, lifeQ;

/* ------------------------------------------------------------------ */
/* ROS objects (used ONLY inside microros_task)                        */
/* ------------------------------------------------------------------ */
static rcl_allocator_t  allocator;
static rclc_support_t   support;
static rcl_node_t       node;
static rclc_executor_t  executor;

static rcl_subscription_t drive_sub, arm_sub;
static rcl_publisher_t    life_pub;
static rcl_timer_t        life_timer;

static geometry_msgs__msg__Twist        drive_msg;
static std_msgs__msg__Float32MultiArray arm_msg;
static std_msgs__msg__Bool              life_msg;
static float arm_buf[ARM_JOINTS];       /* static storage, no malloc in cb */

/* ------------------------------------------------------------------ */
/* DRIVE task  (highest priority)                                      */
/* ------------------------------------------------------------------ */
static void drive_task(void *arg)
{
    (void)arg;
    DriveCmd_t cmd;
    for (;;) {
        if (xQueueReceive(driveQ, &cmd, pdMS_TO_TICKS(DRIVE_TIMEOUT_MS)) == pdTRUE) {
            drive_apply(cmd.lin, cmd.ang);
        } else {
            drive_apply(0.0f, 0.0f);    /* failsafe: link lost / no cmd */
        }
    }
}

/* ------------------------------------------------------------------ */
/* ARM task                                                            */
/* ------------------------------------------------------------------ */
static void arm_task(void *arg)
{
    (void)arg;
    ArmCmd_t cmd;
    for (;;) {
        xQueueReceive(armQ, &cmd, portMAX_DELAY);   /* sleeps until new target */
        arm_apply(cmd.q);
    }
}

/* ------------------------------------------------------------------ */
/* LIFE DETECTION task  (lowest priority)                              */
/* ------------------------------------------------------------------ */
static void life_task(void *arg)
{
    (void)arg;
    TickType_t last = xTaskGetTickCount();
    for (;;) {
        bool detected = life_sensor_read();
        xQueueOverwrite(lifeQ, &detected);
        vTaskDelayUntil(&last, pdMS_TO_TICKS(LIFE_PERIOD_MS));
    }
}

/* ------------------------------------------------------------------ */
/* ROS callbacks (run inside microros_task, keep them short)           */
/* ------------------------------------------------------------------ */
static void drive_cb(const void *msgin)
{
    const geometry_msgs__msg__Twist *m = msgin;
    DriveCmd_t c = { (float)m->linear.x, (float)m->angular.z };
    xQueueOverwrite(driveQ, &c);
}

static void arm_cb(const void *msgin)
{
    const std_msgs__msg__Float32MultiArray *m = msgin;
    ArmCmd_t c;
    memset(&c, 0, sizeof(c));
    size_t n = (m->data.size < ARM_JOINTS) ? m->data.size : ARM_JOINTS;
    memcpy(c.q, m->data.data, n * sizeof(float));
    xQueueOverwrite(armQ, &c);
}

static void life_timer_cb(rcl_timer_t *t, int64_t last_call)
{
    (void)last_call;
    if (t == NULL) return;
    bool detected = false;
    xQueuePeek(lifeQ, &detected, 0);            /* non-blocking */
    life_msg.data = detected;
    (void)rcl_publish(&life_pub, &life_msg, NULL);
}

/* ------------------------------------------------------------------ */
/* ROS entity create / destroy                                         */
/* ------------------------------------------------------------------ */
static bool ros_create(void)
{
    allocator = rcl_get_default_allocator();

    if (rclc_support_init(&support, 0, NULL, &allocator) != RCL_RET_OK) return false;
    if (rclc_node_init_default(&node, "rover_node", "", &support) != RCL_RET_OK) return false;

    if (rclc_subscription_init_default(&drive_sub, &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "cmd_vel") != RCL_RET_OK) return false;

    if (rclc_subscription_init_default(&arm_sub, &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray), "arm_cmd") != RCL_RET_OK) return false;

    if (rclc_publisher_init_default(&life_pub, &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), "life_detected") != RCL_RET_OK) return false;

    /* On newer rclc (Humble+) use rclc_timer_init_default2(..., true) */
    if (rclc_timer_init_default(&life_timer, &support,
            RCL_MS_TO_NS(LIFE_PERIOD_MS), life_timer_cb) != RCL_RET_OK) return false;

    /* handles = 2 subscriptions + 1 timer */
    if (rclc_executor_init(&executor, &support.context, 3, &allocator) != RCL_RET_OK) return false;
    if (rclc_executor_add_subscription(&executor, &drive_sub, &drive_msg, drive_cb, ON_NEW_DATA) != RCL_RET_OK) return false;
    if (rclc_executor_add_subscription(&executor, &arm_sub, &arm_msg, arm_cb, ON_NEW_DATA) != RCL_RET_OK) return false;
    if (rclc_executor_add_timer(&executor, &life_timer) != RCL_RET_OK) return false;

    return true;
}

static void ros_destroy(void)
{
    rmw_context_t *rmw_ctx = rcl_context_get_rmw_context(&support.context);
    (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_ctx, 0);

    (void)rclc_executor_fini(&executor);
    (void)rcl_timer_fini(&life_timer);
    (void)rcl_publisher_fini(&life_pub, &node);
    (void)rcl_subscription_fini(&arm_sub, &node);
    (void)rcl_subscription_fini(&drive_sub, &node);
    (void)rcl_node_fini(&node);
    (void)rclc_support_fini(&support);
}

/* ------------------------------------------------------------------ */
/* micro-ROS task: the ONLY loop that handles ROS                      */
/* ------------------------------------------------------------------ */
typedef enum {
    WAITING_AGENT,
    AGENT_AVAILABLE,
    AGENT_CONNECTED,
    AGENT_DISCONNECTED
} AgentState_t;

static void microros_task(void *arg)
{
    (void)arg;

    /* transport */
    rmw_uros_set_custom_transport(true, (void *)&huart2,
        cubemx_transport_open, cubemx_transport_close,
        cubemx_transport_write, cubemx_transport_read);

    /* FreeRTOS-friendly allocators */
    rcl_allocator_t fa = rcutils_get_zero_initialized_allocator();
    fa.allocate        = microros_allocate;
    fa.deallocate      = microros_deallocate;
    fa.reallocate      = microros_reallocate;
    fa.zero_allocate   = microros_zero_allocate;
    if (!rcutils_set_default_allocator(&fa)) {
        for (;;) { osDelay(1000); }     /* allocator setup failed */
    }

    /* preallocate array message storage once */
    arm_msg.data.data     = arm_buf;
    arm_msg.data.size     = 0;
    arm_msg.data.capacity = ARM_JOINTS;

    AgentState_t state = WAITING_AGENT;

    /* THE main loop */
    for (;;) {
        switch (state) {
        case WAITING_AGENT:
            state = (rmw_uros_ping_agent(100, 1) == RMW_RET_OK)
                    ? AGENT_AVAILABLE : WAITING_AGENT;
            break;

        case AGENT_AVAILABLE:
            if (ros_create()) {
                state = AGENT_CONNECTED;
            } else {
                ros_destroy();
                state = WAITING_AGENT;
            }
            break;

        case AGENT_CONNECTED:
            /* check the link about once per second */
            {
                static uint32_t last_ping = 0;
                uint32_t now = osKernelGetTickCount();
                if (now - last_ping > 1000) {
                    last_ping = now;
                    if (rmw_uros_ping_agent(100, 1) != RMW_RET_OK) {
                        state = AGENT_DISCONNECTED;
                        break;
                    }
                }
            }
            rclc_executor_spin_some(&executor, RCL_MS_TO_NS(SPIN_TIMEOUT_MS));
            WDG_FEED();     /* reached only if the last ping succeeded */
            break;

        case AGENT_DISCONNECTED:
            ros_destroy();
            state = WAITING_AGENT;
            break;
        }
        osDelay(1);     /* let lower-priority tasks run */
    }
}

/* ------------------------------------------------------------------ */
/* Init: call before osKernelStart()                                   */
/* ------------------------------------------------------------------ */
void rover_app_init(void)
{
    driveQ = xQueueCreate(1, sizeof(DriveCmd_t));
    armQ   = xQueueCreate(1, sizeof(ArmCmd_t));
    lifeQ  = xQueueCreate(1, sizeof(bool));

    /* CMSIS v2: stack_size is in BYTES */
    const osThreadAttr_t driveAttr = { .name = "drive", .priority = (osPriority_t)osPriorityHigh,        .stack_size = 256  * 4 };
    const osThreadAttr_t armAttr   = { .name = "arm",   .priority = (osPriority_t)osPriorityAboveNormal, .stack_size = 256  * 4 };
    const osThreadAttr_t rosAttr   = { .name = "uROS",  .priority = (osPriority_t)osPriorityNormal,      .stack_size = 3000 * 4 };
    const osThreadAttr_t lifeAttr  = { .name = "life",  .priority = (osPriority_t)osPriorityBelowNormal, .stack_size = 256  * 4 };

    osThreadNew(drive_task,    NULL, &driveAttr);
    osThreadNew(arm_task,      NULL, &armAttr);
    osThreadNew(microros_task, NULL, &rosAttr);
    osThreadNew(life_task,     NULL, &lifeAttr);
}
