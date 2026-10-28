#include "microros_main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os2.h"

/* --- Micro-ROS & Standard Includes --- */
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include "iwdg.h"
#include <rcutils/allocator.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <uxr/client/transport.h>
#include <rmw_microxrcedds_c/config.h>
#include <rmw_microros/rmw_microros.h>
#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <geometry_msgs/msg/twist.h>
#include <kratos_teerth_rohan_pandya_msgs/msg/rover_status.h>
/* --- Macros --- */
#define SPIN_TIMEOUT_MS   10
#define PING_PERIOD_MS    1000
extern IWDG_HandleTypeDef hiwdg1; 
#define WDG_FEED()        HAL_IWDG_Refresh(&hiwdg1)
#define RCCHECK(fn)  do { WDG_FEED(); if ((fn) != RCL_RET_OK) return false; } while (0)

/* --- Types --- */
typedef enum states {
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
} state_t;

/* --- Externs & Transport Prototypes --- */
extern UART_HandleTypeDef huart3;

bool cubemx_transport_open(struct uxrCustomTransport * transport);
bool cubemx_transport_close(struct uxrCustomTransport * transport);
size_t cubemx_transport_write(struct uxrCustomTransport* transport, const uint8_t * buf, size_t len, uint8_t * err);
size_t cubemx_transport_read(struct uxrCustomTransport* transport, uint8_t * buf, size_t len, int timeout, uint8_t * err);

void * microros_allocate(size_t size, void * state);
void microros_deallocate(void * pointer, void * state);
void * microros_reallocate(void * pointer, size_t size, void * state);
void * microros_zero_allocate(size_t number_of_elements, size_t size_of_element, void * state);

/* --- ROS Entities & Message Buffers --- */
static rcl_allocator_t  allocator;
static rclc_support_t   support;
static rcl_node_t       node;
static rclc_executor_t  executor;

rcl_subscription_t arm_message_subscriber;
rcl_subscription_t drive_message_subscriber;
rcl_subscription_t misc_message_subscriber;


std_msgs__msg__Bool misc_msg;

kratos_teerth_rohan_pandya_msgs__msg__RoverStatus custom_status_msg;
volatile HAL_StatusTypeDef tx_ret;   // global, so Live Expressions can read it
#define ARM_DATA_CAPACITY 8   // set to your number of joints/values

/* --- Subscriber Callbacks --- */
void arm_cb(const void * msgin) {
  (void)msgin;
  //  custom_status_msg.arm_cmd[x]; for array
  // Add your arm callback logic here
}

void drive_cb(const void * msgin) {
  (void)msgin;
  //custom_status_msg.wheel_info[x]; for array
  // Add your drive callback logic here
}

void misc_cb(const void * msgin) {
  (void)msgin;
  //  for array
  // Add your misc callback logic here
}

/* --- Lifecycle Functions --- */
static bool ros_create(void)
{
  memset(&support, 0, sizeof(support));
  node = rcl_get_zero_initialized_node();
  arm_message_subscriber   = rcl_get_zero_initialized_subscription();
  drive_message_subscriber = rcl_get_zero_initialized_subscription();
  misc_message_subscriber  = rcl_get_zero_initialized_subscription();
  executor = rclc_executor_get_zero_initialized_executor();
  allocator = rcl_get_default_allocator();

  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "rover_node", "", &support));

  RCCHECK(rclc_subscription_init_best_effort(&arm_message_subscriber, &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray), "arm_cmd"));
  RCCHECK(rclc_subscription_init_best_effort(&drive_message_subscriber, &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "cmd_vel"));
  RCCHECK(rclc_subscription_init_best_effort(&misc_message_subscriber, &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), "misc_topic"));

  /* 3 handles for the 3 subscribers */
  RCCHECK(rclc_executor_init(&executor, &support.context, 3, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &arm_message_subscriber, &custom_status_msg.arm_cmd, &arm_cb, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &drive_message_subscriber, &custom_status_msg.wheel_info, &drive_cb, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &misc_message_subscriber, &misc_msg, &misc_cb, ON_NEW_DATA));

  return true;
}

static void ros_destroy(void)
{
  if (!rcl_context_is_valid(&support.context)) return;

  rmw_context_t *rmw_ctx = rcl_context_get_rmw_context(&support.context);
  (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_ctx, 0);

  rcl_ret_t rc;
  rc = rclc_executor_fini(&executor);
  rc = rcl_subscription_fini(&arm_message_subscriber, &node);
  rc = rcl_subscription_fini(&drive_message_subscriber, &node);
  rc = rcl_subscription_fini(&misc_message_subscriber, &node);
  rc = rcl_node_fini(&node);
  rc = rclc_support_fini(&support);
  (void)rc; // Silence unused result warnings
}

/* --- Main Task Loop --- */
/* static buffers for the array message so the executor never mallocs */


void microros_main_Run(void *argument)
{
	      // add to Live Expressions
//for debugging uart
//	for (;;) {
//	    WDG_FEED();
//	    tx_ret = HAL_UART_Transmit(&huart3, (uint8_t*)"hello\r\n", 7, 100);
//	    osDelay(500);
//	}
	WDG_FEED();
  (void)argument;
  /* 2. transport */
  rmw_uros_set_custom_transport(true, (void *)&huart3,
            cubemx_transport_open, cubemx_transport_close,
            cubemx_transport_write, cubemx_transport_read);

//uncomment if no refresh
//  if ( rmw_uros_ping_agent(100, 1) != RMW_RET_OK) {
//      for (;;) { osDelay(100); }   // or Error_Handler()
//    }

  /* 1. allocators FIRST, before any rcl/rmw call */
  rcl_allocator_t fa = rcutils_get_zero_initialized_allocator();
   fa.allocate      = microros_allocate;
   fa.deallocate    = microros_deallocate;
   fa.reallocate    = microros_reallocate;
   fa.zero_allocate = microros_zero_allocate;
   if (!rcutils_set_default_allocator(&fa)) {
     for (;;) {  osDelay(1000); }
   }



  state_t agent_state = WAITING_AGENT;
    uint32_t last_ping  = osKernelGetTickCount();
    int missed_spins    = 0;

  /* 3. static message storage (Float32MultiArray needs it) */




  for (;;)
  {
    switch (agent_state)
    {
    case WAITING_AGENT:
            agent_state = (rmw_uros_ping_agent(100, 1) == RMW_RET_OK)
                          ? AGENT_AVAILABLE : WAITING_AGENT;
            break;

      case AGENT_AVAILABLE:
        if (ros_create()) {
          last_ping    = osKernelGetTickCount();
          missed_spins = 0;
          agent_state  = AGENT_CONNECTED;
        } else {
          ros_destroy();
          agent_state = WAITING_AGENT;
        }
        break;

      case AGENT_CONNECTED:
      {
        uint32_t now = osKernelGetTickCount();
        if ((uint32_t)(now - last_ping) >= PING_PERIOD_MS) {
          last_ping = now;
          if (rmw_uros_ping_agent(100, 1) != RMW_RET_OK) {
            agent_state = AGENT_DISCONNECTED;
            break;
          }
        }
        rcl_ret_t r = rclc_executor_spin_some(&executor, RCL_MS_TO_NS(SPIN_TIMEOUT_MS));
        if (r != RCL_RET_OK && r != RCL_RET_TIMEOUT) {
          if (++missed_spins > 20) { agent_state = AGENT_DISCONNECTED; }
        } else {
          missed_spins = 0;
        }
        break;
      }

      case AGENT_DISCONNECTED:
        ros_destroy();
        agent_state = WAITING_AGENT;
        break;
    }
    WDG_FEED();
    osDelay(1);
  }
}
//backup code if smtg breaks


//void microros_main_Run(void *argument)
//{
//  (void)argument;
//
//  rmw_uros_set_custom_transport(true, (void *)&huart3,
//      cubemx_transport_open, cubemx_transport_close,
//      cubemx_transport_write, cubemx_transport_read);
//
//  for(;;)
//   {
//       HAL_IWDG_Refresh(&hiwdg1);
// 	  rcl_allocator_t allocator = rcl_get_default_allocator();
// 	  rclc_support_t support;
// 	  rclc_support_init(&support, 0, NULL, &allocator);
//
//       // 1. Wait until the laptop agent is actually listening
//       while (rmw_uros_ping_agent(100, 1) != RMW_RET_OK) {
// //          HAL_IWDG_Refresh(&hiwdg);
//           osDelay(100);
//       }
//       ros_create();
//       rcl_ret_t spin_ret = rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
//       for(;;)
//             {
//                 while (rmw_uros_ping_agent(100, 1) != RMW_RET_OK) {
//
//
//               	  osDelay(500);
//
//                 }
//                 HAL_IWDG_Refresh(&hiwdg1);
//                 osDelay(1);
//             }
//   }
//
//  rcl_allocator_t fa = rcutils_get_zero_initialized_allocator();
//  fa.allocate      = microros_allocate;
//  fa.deallocate    = microros_deallocate;
//  fa.reallocate    = microros_reallocate;
//  fa.zero_allocate = microros_zero_allocate;
//  if (!rcutils_set_default_allocator(&fa)) {
//    for (;;) { osDelay(1000); }
//  }
//
//  state_t agent_state = WAITING_AGENT;
//  uint32_t last_ping  = osKernelGetTickCount();
//  int missed_spins    = 0;
//
//  for (;;)
//  {
//    switch (agent_state)
//    {
//      case WAITING_AGENT:
//        agent_state = (rmw_uros_ping_agent(100, 1) == RMW_RET_OK) ? AGENT_AVAILABLE : WAITING_AGENT;
//        break;
//
//      case AGENT_AVAILABLE:
//        if (ros_create()) {
//          last_ping    = osKernelGetTickCount();
//          missed_spins = 0;
//          agent_state  = AGENT_CONNECTED;
//        } else {
//          ros_destroy();
//          agent_state = WAITING_AGENT;
//        }
//        break;
//
//      case AGENT_CONNECTED:
//      {
//        uint32_t now = osKernelGetTickCount();
//        if ((uint32_t)(now - last_ping) >= PING_PERIOD_MS) {
//          last_ping = now;
//          if (rmw_uros_ping_agent(100, 1) != RMW_RET_OK) {
//            agent_state = AGENT_DISCONNECTED;
//            break;
//          }
//        }
//        rcl_ret_t r = rclc_executor_spin_some(&executor, RCL_MS_TO_NS(SPIN_TIMEOUT_MS));
//        if (r != RCL_RET_OK && r != RCL_RET_TIMEOUT) {
//          if (++missed_spins > 20) {
//            agent_state = AGENT_DISCONNECTED;
//            break;
//          }
//        } else {
//          missed_spins = 0;
//        }
//        break;
//      }
//
//      case AGENT_DISCONNECTED:
//        ros_destroy();
//        agent_state = WAITING_AGENT;
//        break;
//    }
//    WDG_FEED();
//    osDelay(1);
//  }
//}
