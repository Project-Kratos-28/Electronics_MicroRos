# Two-Loop Reconnect Architecture — micro-ROS on STM32

Continuation of [WATCHDOG_RECONNECT.md](./WATCHDOG_RECONNECT.md). Documents
why `StartDefaultTask` is structured as **two nested infinite loops**
instead of one, how that structure handles the agent being killed
(Ctrl+C) and restarted without a physical reset, the UART overrun bug
found along the way, and a real bug found in the reconnect logic itself
that is fixed here.

---

## Table of Contents

- [Why two loops instead of one](#why-two-loops-instead-of-one)
- [The structure, at a glance](#the-structure-at-a-glance)
- [How IWDG refreshing works across both loops](#how-iwdg-refreshing-works-across-both-loops)
- [How reconnect after Ctrl+C actually works](#how-reconnect-after-ctrl-c-actually-works)
- [Bug found and fixed: stale `spin_ret`](#bug-found-and-fixed-stale-spin_ret)
- [UART overrun (ORE) errors: cause and fix](#uart-overrun-ore-errors-cause-and-fix)
- [Open mystery: periodic full reset around msg ~500](#open-mystery-periodic-full-reset-around-msg-500)
- [Full current code](#full-current-code)
- [Major segments explained](#major-segments-explained)

---

## Why two loops instead of one

A single `for(;;)` loop can't cleanly represent two different jobs that
have to happen at different times:

1. **"Do I have a session yet?"** — a one-time (per connection attempt)
   sequence: wait for the agent, then build every micro-ROS entity
   (node, publishers, subscribers, timer, executor).
2. **"Keep the session alive"** — a repeating, per-message sequence: spin
   the executor, watch for errors, feed the watchdog.

Trying to cram both into one loop means either re-checking "am I
connected?" on every single iteration (wasteful, and exactly the kind of
check that caused false disconnects earlier — see
[WATCHDOG_RECONNECT.md Part 8](./WATCHDOG_RECONNECT.md#️-part-8--known-unresolved-issue-reconnect-fails-after-a-few-cycles)),
or duplicating the entity-creation code at multiple call sites. Splitting
into two nested loops keeps each job's logic in exactly one place and
makes it obvious when each part runs:

- **Outer loop** = "manage the connection lifecycle" — runs once per
  connection attempt (wait → create → run inner loop → teardown → repeat).
- **Inner loop** = "normal operation" — runs once per ~1-10ms tick while
  connected, doing the actual work (spin, LED, publish, ping heartbeat
  where applicable).

## The structure, at a glance

```c
void StartDefaultTask(void *argument)
{
  rmw_uros_set_custom_transport(...);   // set once, never repeated

  for (;;)   // OUTER LOOP — connection lifecycle
  {
    HAL_IWDG_Refresh(&hiwdg);

    // block here until the agent responds
    while (rmw_uros_ping_agent(100, 1) != RMW_RET_OK) {
      HAL_IWDG_Refresh(&hiwdg);
      osDelay(100);
    }

    // agent found — build every entity fresh
    rclc_support_init(...);
    rclc_node_init_default(...);
    rclc_publisher_init_default(...);      // x2
    rclc_subscription_init_best_effort(...); // x3
    rclc_timer_init_default2(...);
    rclc_executor_init(...);
    rclc_executor_add_timer(...);
    rclc_executor_add_subscription(...);   // x3

    for (;;)   // INNER LOOP — normal operation
    {
      // clear UART overruns if any occur
      // spin the executor, capture its return value
      // count consecutive failures; break out if too many
      HAL_IWDG_Refresh(&hiwdg);
      osDelay(1);
    }

    // reached only when the inner loop breaks (real failure detected)
    rclc_executor_fini(...);
    rcl_publisher_fini(...);   // x2
    rcl_subscription_fini(...); // x3
    rcl_timer_fini(...);
    rcl_node_fini(...);
    rclc_support_fini(...);
    cubemx_transport_close(NULL);
    cubemx_transport_open(NULL);

    // outer loop repeats — back to "wait for agent"
  }
}
```

## How IWDG refreshing works across both loops

The watchdog is fed in **three distinct places**, each covering a
different kind of wait/work so nothing goes more than a bounded time
without a refresh:

1. **Top of the outer loop** — before anything else happens each
   connection attempt.
2. **Inside the agent-wait `while`** — since this can legitimately block
   for a long time if the agent isn't running yet, it needs its own
   refresh on every retry (`osDelay(100)` between checks).
3. **Bottom of the inner loop** — once per normal-operation tick, so
   sustained running never starves the watchdog.

This three-point coverage is what lets the watchdog do its actual job —
catching genuine hangs (e.g. a call that blocks forever unexpectedly) —
without ever firing during expected, legitimate waiting (like patiently
waiting for the agent to start).

## How reconnect after Ctrl+C actually works

1. You Ctrl+C the agent on the host. The board has no direct way to know
   this — UART gives no "connection closed" signal the way TCP would.
2. Inside the inner loop, `rclc_executor_spin_some()` starts returning a
   non-OK, non-timeout result once writes genuinely start failing at the
   transport level (the agent process is gone, so nothing acknowledges
   reliable-stream traffic, and any custom-transport writes to a dead
   remote eventually surface as errors here).
3. `missed_spins` counts these consecutive bad results. Once it exceeds
   the threshold (20, ≈2 seconds at 100ms-timeout spins), the inner loop
   `break`s.
4. Execution falls through to the teardown block — every `rcl`/`rclc`
   entity is `_fini`'d, and the transport itself is closed and reopened
   to clear any stale DMA/session state.
5. The outer `for(;;)` loops back to the top, refreshes the watchdog, and
   re-enters the `while (rmw_uros_ping_agent(...) != RMW_RET_OK)` wait.
6. The moment you restart the agent on the host, the next ping succeeds,
   and the outer loop rebuilds every entity from scratch — a completely
   fresh session, no physical reset needed.

This is the mechanism that replaced the original "just let the whole
chip hard-reset via IWDG" approach from earlier in the project — that
approach worked, but wiped every RAM variable on each reconnect and was
slower/noisier (it also interrupted the independent PWM fade task
running on PA8, since a full chip reset restarts everything, not just
the micro-ROS session).

**Important nuance**: pinging the agent is only done in the **outer**
loop's wait step, never inside the inner loop while a session is active.
An earlier version of this code did ping periodically while connected as
a "heartbeat," which turned out to be a documented anti-pattern — see
[WATCHDOG_RECONNECT.md](./WATCHDOG_RECONNECT.md), the ping call itself
disturbs an active reliable session, causing false self-inflicted
disconnects roughly every 20 seconds. Relying purely on
`rclc_executor_spin_some`'s own return value avoids this entirely.

## Bug found and fixed: stale `spin_ret`

An earlier version of the inner loop had this shape:

```c
rcl_ret_t spin_ret = rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));  // captured ONCE, before the loop
...
for(;;)
{
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));   // return value discarded!
    if (spin_ret != RCL_RET_OK && spin_ret != RCL_RET_TIMEOUT) {   // checks the SAME stale value every time
        spin_error_count++;
    }
    ...
}
```

`spin_ret` was assigned once, immediately before the inner loop started,
and the actual `rclc_executor_spin_some()` call *inside* the loop threw
its return value away every iteration. Every subsequent check was
testing a single frozen result from before the loop even began — meaning
the entire disconnect-detection path (`missed_spins`, the `break`) never
actually reacted to real, ongoing spin results.

**Fix** — capture the return value fresh, inside the loop, every
iteration:

```c
for(;;)
{
    if (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_ORE) != RESET) {
        __HAL_UART_CLEAR_OREFLAG(&huart2);
        huart2.ErrorCode = HAL_UART_ERROR_NONE;
        ore_error_count++;
    }

    rcl_ret_t spin_ret = rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));  // fresh, every time

    if (spin_ret != RCL_RET_OK && spin_ret != RCL_RET_TIMEOUT) {
        spin_error_count++;
        missed_spins++;
        if (missed_spins > 20) {
            break;
        }
    } else {
        missed_spins = 0;
    }

    size_t current_free = xPortGetFreeHeapSize();
    if (current_free < min_free_heap) {
        min_free_heap = current_free;
    }

    HAL_IWDG_Refresh(&hiwdg);
    osDelay(1);
}
```

This also merges what had become two separate, overlapping
`if (spin_ret != OK...)` checks in the broken version into one.

## UART overrun (ORE) errors: cause and fix

Separately from the reconnect logic, sustained operation was showing
UART overrun errors (`ore` counter climbing) starting consistently around
message ~35, correlating with the LED blink rate slowing down and
`/custom_message` becoming unresponsive. Root cause: a new byte arriving
before the previous one had been read out of the UART's receive register
— a hardware-level condition, not a software timing bug.

**Fix**: disable overrun detection at the hardware level, since DMA
continuously drains the receive buffer and the transport layer already
tolerates occasional dropped bytes (frames simply fail a checksum and
get skipped). On STM32F4, this bit isn't exposed via `AdvancedInit` (that
struct-based API only exists on newer families) — it must be set directly
on the `CR3` register, since the F4 CMSIS header doesn't even define a
named macro for it (`USART_CR3_OVRDIS` isn't present; the F1/F3/L4
headers do have it):

```c
// In Src/usart.c, inside MX_USART2_UART_Init(), in USER CODE BEGIN USART2_Init 2:
huart2.Instance->CR3 |= (1 << 12);   // OVRDIS: disable overrun detection at the hardware level
```
Bit 12 of `CR3` is confirmed as the OVRDIS bit position across STM32
families in ST's reference manuals, even where the CMSIS macro name
itself is missing for a given family's header.

Two allocation-related changes were also tried while chasing this
symptom before the actual UART cause was found — kept in place, though
they turned out not to be the root cause:
```json
// colcon.meta, rmw_microxrcedds block:
"-DRMW_UXRCE_ALLOW_DYNAMIC_ALLOCATIONS=ON"
// colcon.meta, rcutils block:
"-DRCUTILS_AVOID_DYNAMIC_ALLOCATION=OFF"   // was ON
```
Instrumented heap tracking (`min_free_heap`, via `xPortGetFreeHeapSize()`)
showed a completely flat heap throughout the slowdown, ruling out a
memory leak as the cause — the ORE fix above is what actually mattered.

## Open mystery: periodic full reset around msg ~500

With the `spin_ret` bug fixed and the ORE fix applied, the board still
resets — but the counter goes from ~520 back to 1, and `ore`, `spin_err`
stay at 0 the entire time up to the reset. Since the graceful
disconnect/teardown path requires `spin_error_count` to be incrementing
(it never does), this reset is **not** going through the software
reconnect path at all — it's most consistent with a genuine full chip
reset (IWDG expiry from an undiscovered hang, or a fault), which also
explains why every RAM counter goes back to its initial value: a real
reset wipes RAM, so "software reconnect" and "hard reset" look identical
from the counters alone.

**Next diagnostic step (not yet done):** persist a counter and the IWDG
reset-flag across resets using a `.noinit` linker section, so the next
occurrence can be definitively attributed:

```c
// outside any function, in freertos.c
__attribute__((section(".noinit"))) volatile uint32_t reset_count;
__attribute__((section(".noinit"))) volatile uint32_t last_reset_was_iwdg;
```
```c
// top of main(), in USER CODE BEGIN 1, before HAL_Init()
last_reset_was_iwdg = __HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST) ? 1 : 0;
__HAL_RCC_CLEAR_RESET_FLAGS();
reset_count++;
```
Publish both alongside the existing diagnostics string. If the linker
script doesn't already define a `.noinit` section, one needs to be added
manually before this will behave as intended — worth checking
`STM32F446RETX_RAM.ld` first.

---

## Full current code

```c
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

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

typedef enum states {
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
} state_t;

state_t state = WAITING_AGENT;
uint32_t last_ping_time = 0;
const uint32_t HEARTBEAT_INTERVAL_MS = 1000;

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

volatile bool led_manual_override = false;
volatile bool led_manual_state = false;
volatile int32_t fade_step = 10;
volatile bool custom_message_active = false;
char custom_message_storage[50];

volatile uint32_t ore_error_count = 0;
volatile uint32_t spin_error_count = 0;
size_t min_free_heap = SIZE_MAX;

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
      snprintf(string_buffer, sizeof(string_buffer), "cnt=%ld ore=%lu spin_err=%lu heap=%lu",
          (long)counter_msg.data, ore_error_count, spin_error_count, (unsigned long)min_free_heap);
    }
    string_msg.data.data = string_buffer;
    string_msg.data.size = strlen(string_buffer);
    string_msg.data.capacity = sizeof(string_buffer);
    rcl_ret_t ret2 = rcl_publish(&string_publisher, &string_msg, NULL);
    (void)ret2;
  }
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

void StartDefaultTask(void *argument)
{
  rmw_uros_set_custom_transport(
      true,
      (void *) &huart2,
      cubemx_transport_open,
      cubemx_transport_close,
      cubemx_transport_write,
      cubemx_transport_read);

  for (;;)   // OUTER LOOP
  {
    HAL_IWDG_Refresh(&hiwdg);

    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;
    rclc_support_init(&support, 0, NULL, &allocator);

    while (rmw_uros_ping_agent(100, 1) != RMW_RET_OK) {
      HAL_IWDG_Refresh(&hiwdg);
      osDelay(100);
    }

    rcl_node_t node;
    rclc_node_init_default(&node, "nucleo_led_node", "", &support);

    rclc_publisher_init_default(&counter_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "led_blink_count");
    rclc_publisher_init_default(&string_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "nucleo_chatter");

    rclc_subscription_init_best_effort(&led_control_subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), "led_control");
    rclc_subscription_init_best_effort(&fade_speed_subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "fade_speed");
    rclc_subscription_init_best_effort(&custom_message_subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "custom_message");

    rclc_timer_init_default2(&timer, &support, RCL_MS_TO_NS(500), timer_callback, true);

    rclc_executor_t executor;
    rclc_executor_init(&executor, &support.context, 4, &allocator);
    rclc_executor_add_timer(&executor, &timer);
    rclc_executor_add_subscription(&executor, &led_control_subscriber, &led_control_msg, &led_control_callback, ON_NEW_DATA);
    rclc_executor_add_subscription(&executor, &fade_speed_subscriber, &fade_speed_msg, &fade_speed_callback, ON_NEW_DATA);

    custom_message_msg.data.data = custom_message_buffer;
    custom_message_msg.data.size = 0;
    custom_message_msg.data.capacity = sizeof(custom_message_buffer);
    rclc_executor_add_subscription(&executor, &custom_message_subscriber, &custom_message_msg, &custom_message_callback, ON_NEW_DATA);

    counter_msg.data = 0;
    int missed_spins = 0;

    for (;;)   // INNER LOOP
    {
      if (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_ORE) != RESET) {
        __HAL_UART_CLEAR_OREFLAG(&huart2);
        huart2.ErrorCode = HAL_UART_ERROR_NONE;
        ore_error_count++;
      }

      rcl_ret_t spin_ret = rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));

      if (spin_ret != RCL_RET_OK && spin_ret != RCL_RET_TIMEOUT) {
        spin_error_count++;
        missed_spins++;
        if (missed_spins > 20) {
          break;
        }
      } else {
        missed_spins = 0;
      }

      size_t current_free = xPortGetFreeHeapSize();
      if (current_free < min_free_heap) {
        min_free_heap = current_free;
      }

      HAL_IWDG_Refresh(&hiwdg);
      osDelay(1);
    }

    rclc_executor_fini(&executor);
    rcl_publisher_fini(&counter_publisher, &node);
    rcl_publisher_fini(&string_publisher, &node);
    rcl_subscription_fini(&led_control_subscriber, &node);
    rcl_subscription_fini(&fade_speed_subscriber, &node);
    rcl_subscription_fini(&custom_message_subscriber, &node);
    rcl_timer_fini(&timer);
    rcl_node_fini(&node);
    rclc_support_fini(&support);

    cubemx_transport_close(NULL);
    cubemx_transport_open(NULL);
  }
}
```

---

## Major segments explained

| Segment | Purpose |
|---|---|
| `rmw_uros_set_custom_transport(...)` | Set **once**, outside both loops — wires the four `dma_transport.c` functions into micro-ROS; never repeated per-reconnect |
| Outer `for(;;)` | Owns the connection lifecycle: wait → build entities → run inner loop → teardown → repeat |
| `while (rmw_uros_ping_agent(...) != RMW_RET_OK)` | The **only** place pinging happens — safe here because no active session exists yet to disturb |
| `rclc_support_init` → `rclc_executor_add_subscription` block | Entity creation, run fresh every time the outer loop restarts — this is what makes a clean reconnect possible without a chip reset |
| Inner `for(;;)` | Owns normal operation: UART overrun handling, executor spin, error counting, heap tracking, IWDG refresh |
| `spin_ret` capture (now fixed) | The signal that actually detects a dead agent — must be re-read every iteration, not cached once |
| `missed_spins` / threshold `break` | Debounces transient spin errors (avoids tearing down on one bad tick) while still reacting to a real, sustained failure within ~2 seconds |
| Teardown block (`*_fini` calls + transport close/reopen) | Runs only after the inner loop breaks for a real reason; returns every resource to a clean state so the next `rclc_support_init` starts from scratch, not leaked state |
| `ore_error_count`, `spin_error_count`, `min_free_heap` | Diagnostic counters added specifically to distinguish UART-level, executor-level, and memory-level failure modes from each other — instrumental in finding the ORE bug and ruling out a heap leak |
