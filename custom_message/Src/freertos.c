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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <uxr/client/transport.h>
#include <rmw_microxrcedds_c/config.h>
#include <rmw_microros/rmw_microros.h>
#include <std_msgs/msg/string.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/bool.h>
#include "sys/time.h"
#include "iwdg.h"
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

state_t state = WAITING_AGENT;
uint32_t last_ping_time = 0;
const uint32_t HEARTBEAT_INTERVAL_MS = 1000;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
extern UART_HandleTypeDef huart2;
extern TIM_HandleTypeDef htim1;

bool cubemx_transport_open(struct uxrCustomTransport * transport);
bool cubemx_transport_close(struct uxrCustomTransport * transport);
size_t cubemx_transport_write(struct uxrCustomTransport* transport, const uint8_t * buf, size_t len, uint8_t * err);
size_t cubemx_transport_read(struct uxrCustomTransport* transport, uint8_t * buf, size_t len, int timeout, uint8_t * err);

#define LED_PORT GPIOA
#define LED_PIN GPIO_PIN_5

rcl_subscription_t led_control_subscriber;
rcl_subscription_t fade_speed_subscriber;
rcl_subscription_t custom_message_subscriber;

std_msgs__msg__Bool   led_control_msg;
std_msgs__msg__Int32  fade_speed_msg;
std_msgs__msg__String custom_message_msg;
char custom_message_buffer[50];


rcl_publisher_t counter_publisher;
rcl_publisher_t string_publisher;
std_msgs__msg__Int32 counter_msg;
std_msgs__msg__String string_msg;
char string_buffer[50];
rcl_timer_t timer;

void * microros_allocate(size_t size, void * state);
void microros_deallocate(void * pointer, void * state);
void * microros_reallocate(void * pointer, size_t size, void * state);
void * microros_zero_allocate(size_t number_of_elements, size_t size_of_element, void * state);

volatile bool led_manual_override = false;
volatile bool led_manual_state = false;
volatile int32_t fade_step = 10;          // default step size, same as before
volatile bool custom_message_active = false;
char custom_message_storage[50];
rcl_publisher_t custom_status_publisher;
kratos_teerth_rohan_pandya_msgs__msg__RoverStatus custom_status_msg;
// Add near your other globals
volatile uint32_t ore_error_count = 0;
volatile uint32_t spin_error_count = 0;
size_t min_free_heap = SIZE_MAX;
volatile uint32_t p_error = 0;
/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 3000 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Fade */
osThreadId_t FadeHandle;
const osThreadAttr_t Fade_attributes = {
  .name = "Fade",
  .stack_size = 200 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for connMutex */
osMutexId_t connMutexHandle;
const osMutexAttr_t connMutex_attributes = {
  .name = "connMutex"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartTask02(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */
  /* Create the mutex(es) */
  /* creation of connMutex */
  connMutexHandle = osMutexNew(&connMutex_attributes);

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
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of Fade */
  FadeHandle = osThreadNew(StartTask02, NULL, &Fade_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
__attribute__((section(".noinit"))) volatile uint32_t reset_count;
__attribute__((section(".noinit"))) volatile uint32_t last_reset_was_iwdg;

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
  (void) last_call_time;
  if (timer != NULL) {

    if (led_manual_override) {
      HAL_GPIO_WritePin(LED_PORT, LED_PIN, led_manual_state ? GPIO_PIN_SET : GPIO_PIN_RESET);
    } else {
      HAL_GPIO_TogglePin(LED_PORT, LED_PIN);
    }
}

    rcl_ret_t ret1 = rcl_publish(&counter_publisher, &counter_msg, NULL);
    (void)ret1;
    counter_msg.data++;

    if (custom_message_active) {
      snprintf(string_buffer, sizeof(string_buffer), "%s", custom_message_storage);
    } else {
      snprintf(string_buffer, sizeof(string_buffer), "Hello from Nucleo! count=%ld", (long)counter_msg.data);
      snprintf(string_buffer, sizeof(string_buffer), "cnt=%ld ore=%lu spin_err=%lu heap=%lu, ping = %lu",
          (long)counter_msg.data, ore_error_count, spin_error_count, (unsigned long)min_free_heap,p_error);
    }
    string_msg.data.data = string_buffer;
    string_msg.data.size = strlen(string_buffer);
    string_msg.data.capacity = sizeof(string_buffer);
    rcl_ret_t ret2 = rcl_publish(&string_publisher, &string_msg, NULL);
    (void)ret2;
  }


int _gettimeofday(struct timeval *tv, void *tzvp) {
  (void)tzvp;
  uint32_t tick = HAL_GetTick();
  tv->tv_sec = tick / 1000;
  tv->tv_usec = (tick % 1000) * 1000;
  return 0;
}

void led_control_callback(const void *msgin) {
  const std_msgs__msg__Bool * msg = (const std_msgs__msg__Bool *) msgin;
  led_manual_override = true;
  led_manual_state = msg->data;
}

void fade_speed_callback(const void *msgin) {
  const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *) msgin;
  if (msg->data >= 0 && msg->data <= 999) {
    fade_step = msg->data;
  }
}

void custom_message_callback(const void * msgin) {
  const std_msgs__msg__String * msg = (const std_msgs__msg__String *) msgin;
  size_t len = msg->data.size;
  if (len >= sizeof(custom_message_storage)) {
    len = sizeof(custom_message_storage) - 1;
  }
  memcpy(custom_message_storage, msg->data.data, len);
  custom_message_storage[len] = '\0';
  custom_message_active = true;
}
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
	rmw_uros_set_custom_transport(
	    true,
	    (void *) &huart2,
	    cubemx_transport_open,
	    cubemx_transport_close,
	    cubemx_transport_write,
	    cubemx_transport_read);



  /* Infinite Outer Loop: Connection & Initialization Manager */

      HAL_IWDG_Refresh(&hiwdg);
	  rcl_allocator_t allocator = rcl_get_default_allocator();
	  rclc_support_t support;
	  rclc_support_init(&support, 0, NULL, &allocator);

//      // 1. Wait until the laptop agent is actually listening

      // 2. Agent found! Initialize everything
//	  rcl_allocator_t allocator = rcl_get_default_allocator();
//	  rclc_support_t support;
//	  rclc_support_init(&support, 0, NULL, &allocator);

	  rcl_node_t node;
	  rclc_node_init_default(&node, "nucleo_led_node", "", &support);

	  rclc_publisher_init_default(&counter_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "led_blink_count");
	  rclc_publisher_init_default(&string_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "nucleo_chatter");
	  rclc_publisher_init_default(&custom_status_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(kratos_teerth_rohan_pandya_msgs, msg, RoverStatus), "nucleo_status_topic");
	  rclc_subscription_init_best_effort(&led_control_subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), "led_control");
	  rclc_subscription_init_best_effort(&fade_speed_subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "fade_speed");
	  rclc_subscription_init_best_effort(&custom_message_subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "custom_message");

	  rclc_timer_init_default2(&timer, &support, RCL_MS_TO_NS(10), timer_callback, true);

	  rclc_executor_t executor;
	  rclc_executor_init(&executor, &support.context, 4, &allocator);
	  rclc_executor_add_timer(&executor, &timer);
	  rclc_executor_add_subscription(&executor, &led_control_subscriber, &led_control_msg, &led_control_callback, ON_NEW_DATA);
	  rclc_executor_add_subscription(&executor, &fade_speed_subscriber, &fade_speed_msg, &fade_speed_callback, ON_NEW_DATA);

	  custom_message_msg.data.data = custom_message_buffer;
	  custom_message_msg.data.size = 0;
	  custom_message_msg.data.capacity = sizeof(custom_message_buffer);
	  rclc_executor_add_subscription(&executor, &custom_message_subscriber, &custom_message_msg, &custom_message_callback, ON_NEW_DATA);
	  rcl_ret_t spin_ret = rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
	  counter_msg.data = 0;
      uint32_t last_ping_time = HAL_GetTick();
      int missed_spins = 0;
      // 3. Infinite Inner Loop: Normal Operation
      for(;;)
      {
          while (rmw_uros_ping_agent(100, 1) != RMW_RET_OK) {
//        	  osThreadSuspend(FadeHandle);
        	  HAL_GPIO_TogglePin(LED_PORT, LED_PIN);
        	  osDelay(5);
//        	  osThreadSuspend(FadeHandle);   // stop the other task from touching HW mid-reset
//        	  NVIC_SystemReset();
          }

	      rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
	      if (spin_ret != RCL_RET_OK && spin_ret != RCL_RET_TIMEOUT) {
	          spin_error_count++;
	          missed_spins++;
	                 if (missed_spins > 20) {
	                     break;
	                 }
	              else {
	                 missed_spins = 0;
	             }
	      }

	      size_t current_free = xPortGetFreeHeapSize();
	      if (current_free < min_free_heap) {
	          min_free_heap = current_free;
	      }
	      // --- 1. Populate the 6-element arm_cmd array ---
	      custom_status_msg.arm_cmd[0] = 1.54;
	      custom_status_msg.arm_cmd[1] = -0.23;
	      custom_status_msg.arm_cmd[2] = 3.14;
	      custom_status_msg.arm_cmd[3] = 0.0;
	      custom_status_msg.arm_cmd[4] = 1.0;
	      custom_status_msg.arm_cmd[5] = -1.0;

	      // --- 2. Populate the 4-element wheel_vel array ---
	      custom_status_msg.wheel_vel[0] = 2.5;  // Front Left
	      custom_status_msg.wheel_vel[1] = 2.5;  // Front Right
	      custom_status_msg.wheel_vel[2] = 2.5;  // Rear Left
	      custom_status_msg.wheel_vel[3] = 2.5;  // Rear Right

	      // --- 3. Populate the 4-element wheel_angle array ---
	      custom_status_msg.wheel_angle[0] = 0.785;  // ~45 degrees in radians
	      custom_status_msg.wheel_angle[1] = -0.785;
	      custom_status_msg.wheel_angle[2] = 0.0;
	      custom_status_msg.wheel_angle[3] = 0.0;

	      // --- 4. Publish the fully populated message ---
	      rcl_publish(&custom_status_publisher, &custom_status_msg, NULL);
	      HAL_IWDG_Refresh(&hiwdg);
	      osDelay(1);
      } // End of Inner Loop



      // 4. Teardown: Free memory cleanly before restarting
      rclc_executor_fini(&executor);
      rcl_publisher_fini(&counter_publisher, &node);
      rcl_publisher_fini(&string_publisher, &node);
      rcl_subscription_fini(&led_control_subscriber, &node);
      rcl_subscription_fini(&fade_speed_subscriber, &node);
      rcl_subscription_fini(&custom_message_subscriber, &node);
      rcl_timer_fini(&timer);
      rcl_node_fini(&node);
      rclc_support_fini(&support);

      // Reset the transport layer
      cubemx_transport_close(NULL);
      cubemx_transport_open(NULL);

//} // Outer loop repeats, sending it back to "Wait for Agent"
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartTask02 */
/**
* @brief Function implementing the Fade thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask02 */
void StartTask02(void *argument)
{
  /* USER CODE BEGIN StartTask02 */
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);

	  uint32_t max_duty = 999;  // 999, from ARR
	  int32_t duty = 0;
	  int8_t direction = 1;
  /* Infinite loop */
  for(;;)
  {
//	  if (rmw_uros_ping_agent(100, 1) != RMW_RET_OK) {
//
//	  osThreadSuspend(StartDefaultTask);
//	  HAL_GPIO_TogglePin(LED_PORT, LED_PIN);
//	  osDelay(500);
//
//	            }
//	  else{
//		  osThreadSuspend(defaultTaskHandle);
//		  NVIC_SystemReset();
//	  }
	  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, duty);

	      duty += direction * 10;
	      if (duty >= (int32_t)max_duty) {
	        duty = max_duty;
	        direction = -1;
	      } else if (duty <= 0) {
	        duty = 0;
	        direction = 1;
	      }

	      osDelay(5);
  }
  /* USER CODE END StartTask02 */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

