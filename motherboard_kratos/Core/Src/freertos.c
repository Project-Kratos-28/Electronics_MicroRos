/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "FreeRTOS.h"
#include "cmsis_os2.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include <stdbool.h>
#include "queue.h"
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
#include <sys/time.h>
#include "motor_control.h"
#include "sensor_polling.h"
#include "ld_sensors.h"
#include "oled_battery.h"
#include "microros_main.h"
#include <kratos_teerth_rohan_pandya_msgs/msg/rover_status.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum states {
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
} state_t;


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define SPIN_TIMEOUT_MS   10
#define PING_PERIOD_MS    1000
#define WDG_FEED()        HAL_IWDG_Refresh(&hiwdg1)

/* feed the watchdog, then bail out of the calling function on failure */
#define RCCHECK(fn)  do { WDG_FEED(); if ((fn) != RCL_RET_OK) return false; } while (0)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
extern UART_HandleTypeDef huart3;   /* transport UART */

bool cubemx_transport_open(struct uxrCustomTransport * transport);
bool cubemx_transport_close(struct uxrCustomTransport * transport);
size_t cubemx_transport_write(struct uxrCustomTransport* transport, const uint8_t * buf, size_t len, uint8_t * err);
size_t cubemx_transport_read(struct uxrCustomTransport* transport, uint8_t * buf, size_t len, int timeout, uint8_t * err);

void * microros_allocate(size_t size, void * state);
void microros_deallocate(void * pointer, void * state);
void * microros_reallocate(void * pointer, size_t size, void * state);
void * microros_zero_allocate(size_t number_of_elements, size_t size_of_element, void * state);

/* hardware hooks: implement these in your drive / arm / sensor files */

static rcl_allocator_t  allocator;
static rclc_support_t   support;
static rcl_node_t       node;
static rclc_executor_t  executor;
std_msgs__msg__Bool incoming_msg;



/* static storage for the array message, no malloc inside the callback */

/* USER CODE END Variables */
/* Definitions for microros_task */
osThreadId_t microros_taskHandle;
const osThreadAttr_t microros_task_attributes = {
  .name = "microros_task",
  .stack_size = 3000 * 4,
  .priority = (osPriority_t) osPriorityNormal4,
};
/* Definitions for motor_control */
osThreadId_t motor_controlHandle;
const osThreadAttr_t motor_control_attributes = {
  .name = "motor_control",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh7,
};
/* Definitions for LD_task */
osThreadId_t LD_taskHandle;
const osThreadAttr_t LD_task_attributes = {
  .name = "LD_task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal1,
};
/* Definitions for oled_battery */
osThreadId_t oled_batteryHandle;
const osThreadAttr_t oled_battery_attributes = {
  .name = "oled_battery",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void custom_subscriber_callback(const void * msgin);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartTask02(void *argument);
void StartTask04(void *argument);
void StartTask07(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* length 1 on purpose: xQueueOverwrite() only works on 1-item queues */

  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of microros_task */
  microros_taskHandle = osThreadNew(StartDefaultTask, NULL, &microros_task_attributes);

  /* creation of motor_control */
  motor_controlHandle = osThreadNew(StartTask02, NULL, &motor_control_attributes);

  /* creation of LD_task */
  LD_taskHandle = osThreadNew(StartTask04, NULL, &LD_task_attributes);

  /* creation of oled_battery */
  oled_batteryHandle = osThreadNew(StartTask07, NULL, &oled_battery_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/* ---- callbacks (run inside the micro-ROS task, only push to queues) ---- */


/* ---- create / destroy all ROS entities ---- */
/**
  * @brief  Function implementing the micro-ROS thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
	microros_main_Run(argument);
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartTask02 */
/**
* @brief Function implementing the Drive_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask02 */
void StartTask02(void *argument)
{
  /* USER CODE BEGIN StartTask02 */
  (void)argument;

  for (;;)
  {
	  motor_control_Run(argument);
	  osDelay(50);
  }
  /* USER CODE END StartTask02 */
}

/* USER CODE BEGIN Header_StartTask04 */
/**
* @brief Function implementing the LD_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask04 */
void StartTask04(void *argument)
{
  /* USER CODE BEGIN StartTask04 */
  (void)argument;

  for (;;)
  {
	  ld_sensors_Run(argument);
	  osDelay(100);
  }
  /* USER CODE END StartTask04 */
}

/* USER CODE BEGIN Header_StartTask07 */
/**
* @brief Function implementing the LD thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask07 */
void StartTask07(void *argument)
{
  /* USER CODE BEGIN StartTask07 */
  /* Infinite loop */
  for(;;)
  {
	  oled_battery_Run(argument);
    osDelay(10);
  }
  /* USER CODE END StartTask07 */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
int _gettimeofday(struct timeval *tv, void *tzvp)
{
    (void)tzvp; // Timezone is not supported
    if (tv != NULL) {
        // Get the FreeRTOS tick count (Assuming configTICK_RATE_HZ is 1000 / 1ms tick)
        uint32_t tick = xTaskGetTickCount();
        tv->tv_sec = tick / 1000;
        tv->tv_usec = (tick % 1000) * 1000;
    }
    return 0;
}

void custom_subscriber_callback(const void * msgin)
{
  const std_msgs__msg__Bool * msg = (const std_msgs__msg__Bool *)msgin;

  if (msg->data == true) {
      // Do something, e.g., turn on an LED or set a flag for motor_control
  }
}
/* USER CODE END Application */

