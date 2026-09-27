# Adding Multiple Publishers & Subscribers to micro-ROS (STM32 Nucleo-F446RE)

Extension of the base [LED Blink project](./README.md): two FreeRTOS tasks
(`StartDefaultTask` for micro-ROS + LD2, `StartTask02` for a PWM fade on
PA8), with **2 publishers** and **3 subscribers** added to `StartDefaultTask`.

---

## Table of Contents

- [Summary of findings](#summary-of-findings)
- [Final topic list](#final-topic-list)
- [Full working freertos.c](#full-working-freertosc)
- [Bugs hit while wiring this up](#bugs-hit-while-wiring-this-up)
- [Key concepts](#key-concepts)
- [Test commands](#test-commands)

---

## Summary of findings

**Multiple publishers:** no issues. Both publishers (`counter_publisher`,
`string_publisher`) run fine from the same task, publishing back-to-back
inside the same `timer_callback`. Behavior/timing is governed by the
FreeRTOS task priority set in CubeMX (`osPriorityNormal` for the micro-ROS
task vs `osPriorityLow` for the fade task) — not by anything
publisher-specific.

**Multiple subscribers:** more fragile. Adding several subscribers to one
executor works, but ran into repeated build/runtime issues while wiring
it up (see below) that individual publishers didn't hit — largely because
each subscriber needs its own message buffer pre-allocated and its own
executor handle slot correctly counted, and because a String subscriber
specifically needs internal buffer setup that a String *publisher* does
not.

**Open question / limitation noted:** if subscriber issues persist at
scale, running each subscriber's handling in its **own separate FreeRTOS
thread** (rather than all sharing one executor loop) may be more robust
than trying to route around it via environment variables inside the
Docker-based library build — this hasn't been implemented yet, just noted
as the likely next step if the current single-executor approach doesn't
scale further (e.g. beyond 3 subscribers, or under higher message rates).

---

## Final topic list

| Topic | Type | Direction | Purpose |
|---|---|---|---|
| `/led_blink_count` | `std_msgs/Int32` | Publish | Increments every 500 ms |
| `/nucleo_chatter` | `std_msgs/String` | Publish | Status string, printed every 500 ms |
| `/led_control` | `std_msgs/Bool` | Subscribe | `true` = force LD2 on, `false` = resume auto-blink |
| `/fade_speed` | `std_msgs/Int32` | Subscribe | Sets PA8 fade step size (0–999) |
| `/custom_message` | `std_msgs/String` | Subscribe | Overrides the text sent on `/nucleo_chatter` |

---

## Full working `freertos.c`

```c
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
/* USER CODE END Includes */

/* USER CODE BEGIN Variables */
extern UART_HandleTypeDef huart2;
extern TIM_HandleTypeDef htim1;

bool cubemx_transport_open(struct uxrCustomTransport * transport);
bool cubemx_transport_close(struct uxrCustomTransport * transport);
size_t cubemx_transport_write(struct uxrCustomTransport* transport, const uint8_t * buf, size_t len, uint8_t * err);
size_t cubemx_transport_read(struct uxrCustomTransport* transport, uint8_t * buf, size_t len, int timeout, uint8_t * err);

#define LED_PORT GPIOA
#define LED_PIN GPIO_PIN_5

// Subscribers + their message buffers
rcl_subscription_t led_control_subscriber;
rcl_subscription_t fade_speed_subscriber;
rcl_subscription_t custom_message_subscriber;

std_msgs__msg__Bool   led_control_msg;
std_msgs__msg__Int32  fade_speed_msg;
std_msgs__msg__String custom_message_msg;
char custom_message_buffer[50];

// Publishers + their message buffers
rcl_publisher_t counter_publisher;
rcl_publisher_t string_publisher;
std_msgs__msg__Int32 counter_msg;
std_msgs__msg__String string_msg;
char string_buffer[50];
rcl_timer_t timer;

// Shared state, written by subscriber callbacks (Task 1), read by Task 2
volatile bool led_manual_override = false;
volatile bool led_manual_state = false;
volatile int32_t fade_step = 10;
volatile bool custom_message_active = false;
char custom_message_storage[50];
/* USER CODE END Variables */

// One handle + one attributes struct per task — no duplicates
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 3000 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

osThreadId_t fadeTaskHandle;
const osThreadAttr_t fadeTask_attributes = {
  .name = "Fade",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};

void StartDefaultTask(void *argument);
void StartTask02(void *argument);

void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);
  fadeTaskHandle = osThreadNew(StartTask02, NULL, &fadeTask_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* USER CODE END RTOS_MUTEX */
  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* USER CODE END RTOS_SEMAPHORES */
  /* USER CODE BEGIN RTOS_TIMERS */
  /* USER CODE END RTOS_TIMERS */
  /* USER CODE BEGIN RTOS_QUEUES */
  /* USER CODE END RTOS_QUEUES */
  /* USER CODE BEGIN RTOS_THREADS */
  /* USER CODE END RTOS_THREADS */
  /* USER CODE BEGIN RTOS_EVENTS */
  /* USER CODE END RTOS_EVENTS */
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
  (void) last_call_time;
  if (timer != NULL) {

    if (led_manual_override) {
      HAL_GPIO_WritePin(LED_PORT, LED_PIN, led_manual_state ? GPIO_PIN_SET : GPIO_PIN_RESET);
    } else {
      HAL_GPIO_TogglePin(LED_PORT, LED_PIN);
    }

    rcl_ret_t ret1 = rcl_publish(&counter_publisher, &counter_msg, NULL);
    (void)ret1;
    counter_msg.data++;

    if (custom_message_active) {
      snprintf(string_buffer, sizeof(string_buffer), "%s", custom_message_storage);
    } else {
      snprintf(string_buffer, sizeof(string_buffer), "Hello from Nucleo! count=%ld", (long)counter_msg.data);
    }
    string_msg.data.data = string_buffer;
    string_msg.data.size = strlen(string_buffer);
    string_msg.data.capacity = sizeof(string_buffer);
    rcl_ret_t ret2 = rcl_publish(&string_publisher, &string_msg, NULL);
    (void)ret2;
  }
}

// Needed because rcutils/rmw internals call gettimeofday(), which newlib-nano
// doesn't implement on bare-metal targets by default
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

  rcl_allocator_t allocator = rcl_get_default_allocator();
  rclc_support_t support;
  rclc_support_init(&support, 0, NULL, &allocator);

  rcl_node_t node;
  rclc_node_init_default(&node, "nucleo_led_node", "", &support);

  rclc_publisher_init_default(
    &counter_publisher, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "led_blink_count");

  rclc_publisher_init_default(
    &string_publisher, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
    "nucleo_chatter");

  rclc_subscription_init_default(
    &led_control_subscriber, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
    "led_control");

  rclc_subscription_init_default(
    &fade_speed_subscriber, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "fade_speed");

  rclc_subscription_init_default(
    &custom_message_subscriber, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
    "custom_message");

  const unsigned int timer_period_ms = 500;
  rclc_timer_init_default2(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_period_ms),
    timer_callback,
    true);

  // 1 timer + 3 subscriptions = 4 handles
  rclc_executor_t executor;
  rclc_executor_init(&executor, &support.context, 4, &allocator);
  rclc_executor_add_timer(&executor, &timer);

  rclc_executor_add_subscription(
    &executor, &led_control_subscriber, &led_control_msg,
    &led_control_callback, ON_NEW_DATA);

  rclc_executor_add_subscription(
    &executor, &fade_speed_subscriber, &fade_speed_msg,
    &fade_speed_callback, ON_NEW_DATA);

  // String subscription needs its buffer set up before the executor
  // can receive into it — a String publisher does not need this.
  custom_message_msg.data.data = custom_message_buffer;
  custom_message_msg.data.size = 0;
  custom_message_msg.data.capacity = sizeof(custom_message_buffer);

  rclc_executor_add_subscription(
    &executor, &custom_message_subscriber, &custom_message_msg,
    &custom_message_callback, ON_NEW_DATA);

  counter_msg.data = 0;

  for (;;)
  {
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
    osDelay(10);
  }
  /* USER CODE END StartDefaultTask */
}

void StartTask02(void *argument)
{
  /* USER CODE BEGIN StartTask02 */
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);

  uint32_t max_duty = 999;
  int32_t duty = 0;
  int8_t direction = 1;

  for (;;)
  {
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, duty);

    duty += direction * fade_step;
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
```

---

## Bugs hit while wiring this up

All of these were caught by the compiler/linker, not silent runtime bugs
— every one showed up as a build error.

| Error | Cause | Fix |
|---|---|---|
| `incompatible types when assigning to type 'std_msgs__msg__Bool' from type 'int'` | Wrote `led_control_msg = true;` — assigning a bool straight to a whole message struct | Use `led_manual_state = msg->data;` inside the callback instead |
| `'led_manual_stage' undeclared` | Typo: `stage` instead of `state` | Rename to `led_manual_state` |
| `'fade_speed_msg' redeclared as different kind of symbol` | Callback function accidentally given the same name as the message-struct global `fade_speed_msg` | Name the callback `fade_speed_callback` — must differ from the variable name |
| `unknown type name 'std_msgs__msg___Int32'` | Triple underscore typo (`msg___Int32` instead of `msg__Int32`) | Exactly two underscores in each gap: `std_msgs__msg__Int32` |
| `implicit declaration of function 'rclc_subscriber_init_default'` | Wrong function name — "subscriber" instead of "subscription" | `rclc_subscription_init_default` |
| `expected declaration or statement at end of input` | Missing closing `}` in `fade_speed_callback` (opened both the function's brace and the `if`'s brace, only closed one) | Add the missing `}`; also fixes cascading errors later in the file, since the compiler had been mis-parsing everything after the leak as still "inside" that function |
| `'StartTask2' undeclared (first use in this function)` / `'StartTask02' undeclared` (recurring) | Function name spelled inconsistently across: forward declaration, `osThreadNew(...)` call, and the actual `void StartTaskXX(void *argument) { ... }` definition | Pick **one** spelling and use it in all three places — grep the whole file to confirm zero stray occurrences of the other spelling |
| `'fadeTaskHandle' undeclared` | A leftover duplicate block of `MX_FREERTOS_Init` code (old CubeMX-generated thread creation + a hand-added second copy) referenced a handle that only existed inside the deleted half | Keep exactly **one** `osThreadId_t` handle and one `osThreadAttr_t` struct per task, declared once at file scope; call `osThreadNew` for each task exactly **once**, inside `MX_FREERTOS_Init` |
| Two declarations of `osThreadId_t defaultTaskHandle;` | CubeMX's auto-generated declaration plus a duplicate manually added one | Delete the duplicate, keep one |

**Root pattern behind most of these:** hand-editing generated FreeRTOS
boilerplate is easy to duplicate accidentally (CubeMX already declares
`defaultTaskHandle`/`defaultTask_attributes` and creates the thread once;
adding a second task by copy-pasting that pattern risks leaving both the
old and new blocks in place). Before building, it's worth searching the
file for each task name and each handle name to confirm each appears
exactly where expected and nowhere else.

---

## Key concepts

**Multiple publishers in one task:** trivial — just call
`rclc_publisher_init_default` once per publisher, and call `rcl_publish`
on each inside whichever callback/loop should trigger it. No executor
slot is consumed by a publisher (only timers, subscriptions, services,
and clients count toward `rclc_executor_init`'s handle count).

**Multiple subscribers in one task:**
- Each needs its own `rcl_subscription_t` + message struct, created via
  `rclc_subscription_init_default`.
- Each consumes **one** executor handle slot — `rclc_executor_init`'s
  count must equal (timers + subscriptions + services + clients)
  combined, or the executor will misbehave.
- Each needs `rclc_executor_add_subscription(...)` with a callback
  matching `void callback(const void * msgin)`.
- A **String** subscription specifically needs its message's internal
  buffer (`.data`, `.size`, `.capacity`) manually pointed at real storage
  *before* being added to the executor — a String publisher does not
  need this since you set those fields fresh each time you publish.
- `ON_NEW_DATA` vs `ALWAYS` (the `rclc_executor_handle_invocation_t`
  parameter): `ON_NEW_DATA` only fires the callback when a fresh message
  actually arrives — use this for anything event-driven like these three.

**Cross-task shared state:** subscriber callbacks run inside
`StartDefaultTask` (during `rclc_executor_spin_some`), but their effects
(LED override, fade speed) need to be read by `StartTask02`. Any variable
touched by both tasks is declared `volatile` so the compiler always
re-reads it from memory instead of caching a stale copy in a register.

**Task priority governs preemption, not publish reliability:** the note
in the original findings about "priorities were given based on task
priority" reflects that `osPriorityNormal` (micro-ROS task) will preempt
`osPriorityLow` (fade task) if both are ready to run — this affects
*timing/smoothness* of the fade under load, not whether publishes
themselves succeed.

---

## Test commands

```bash
# Force LED on
ros2 topic pub --once /led_control std_msgs/msg/Bool "{data: true}"

# Resume auto-blink
ros2 topic pub --once /led_control std_msgs/msg/Bool "{data: false}"

# Speed up the fade
ros2 topic pub --once /fade_speed std_msgs/msg/Int32 "{data: 50}"

# Override the chatter message
ros2 topic pub --once /custom_message std_msgs/msg/String "{data: 'Testing subscribers!'}"

# Watch the counter and chatter
ros2 topic echo /led_blink_count
ros2 topic echo /nucleo_chatter

# List everything the board is exposing
ros2 topic list
ros2 node list
```
