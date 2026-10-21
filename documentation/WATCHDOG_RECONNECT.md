# IWDG Watchdog & Automatic Agent Reconnection — micro-ROS on STM32

Documents the full journey of adding an Independent Watchdog (IWDG) to the
micro-ROS STM32 project to eliminate manual reset-button presses, and the
subsequent work to make the board automatically reconnect when the
micro-ROS Agent is restarted — including the bugs hit, fixes applied, and
**one issue still open at time of writing** (flagged clearly at the end).

---

## Table of Contents

- [Why we needed this](#why-we-needed-this)
- [Part 1 — IWDG basics & CubeMX setup](#part-1--iwdg-basics--cubemx-setup)
- [Part 2 — First attempt: reset-on-failed-handshake](#part-2--first-attempt-reset-on-failed-handshake)
- [Part 3 — Bug: unreachable code in main.c](#part-3--bug-unreachable-code-in-mainc)
- [Part 4 — Porting to STM32H753ZI: wrong UART instance](#part-4--porting-to-stm32h753zi-wrong-uart-instance)
- [Part 5 — Multiple subscribers on a single-core chip](#part-5--multiple-subscribers-on-a-single-core-chip)
- [Part 6 — The real goal: reconnect without any reset](#part-6--the-real-goal-reconnect-without-any-reset)
- [Part 7 — Final state-machine code](#part-7--final-state-machine-code)
- [⚠️ Part 8 — Known unresolved issue: reconnect fails after a few cycles](#️-part-8--known-unresolved-issue-reconnect-fails-after-a-few-cycles)
- [Quick reference: all code in one place](#quick-reference-all-code-in-one-place)

---

## Why we needed this

Without a watchdog, the board only attempts the micro-ROS agent handshake
once (right after boot / firmware flash). If the agent isn't running yet
at that exact moment, `rclc_support_init()` blocks forever, and the only
way to retry is a **manual physical reset** (pressing the black button on
the Nucleo). The goal of this work was to eliminate that manual step
entirely — first by having the board auto-reset itself until the agent
appears, and later by avoiding resets altogether in favor of a pure
software reconnect loop.

---

## Part 1 — IWDG basics & CubeMX setup

**What IWDG is:** a separate hardware timer clocked by the internal LSI
oscillator (~32 kHz), completely independent of the main clock/FreeRTOS
tick. It counts down from a reload value; if the count reaches zero
without being refreshed ("kicked") first, the chip **force-resets**
immediately, regardless of what the CPU was doing. Once started it cannot
be stopped or reconfigured except by a full reset — this is intentional.

**CubeMX configuration** (`.ioc` → System Core → IWDG):
- **Activated**: checked
- **Prescaler**: `/256`
- **Down-counter reload value**: `4095`

This combination gives the maximum possible timeout at this prescaler:
≈32.7 seconds — enough margin for a normal agent handshake (which usually
takes 1-3 seconds), while still catching a genuinely stuck board.

This generates `Src/iwdg.c`, a global `IWDG_HandleTypeDef hiwdg`, and a
call to `MX_IWDG_Init()` in `main.c`'s init sequence, before the scheduler
starts.

---

## Part 2 — First attempt: reset-on-failed-handshake

The initial plan: don't refresh the watchdog while waiting on the agent.
If the agent isn't up, the watchdog naturally expires and resets the
board — repeating automatically every ~15-30 seconds until the agent
happens to be running at connection time.

```c
uint32_t start_time = HAL_GetTick();
uint8_t connected = 0;
rmw_ret_t state;

while ((HAL_GetTick() - start_time) < 5000)
{
    HAL_IWDG_Refresh(&hiwdg);
    state = (RMW_RET_OK == rmw_uros_ping_agent(100, 3)) ? AGENT_AVAILABLE : WAITING_AGENT;
    if (state == AGENT_AVAILABLE)
    {
        connected = 1;
        break;
    }
    HAL_Delay(10);
}

if (!connected)
{
    // Stop refreshing — IWDG will hit zero and force a reset
    while(1) { }
}
```

This is a reasonable one-shot pre-check pattern — the problem was **where
it was placed**, covered next.

---

## Part 3 — Bug: unreachable code in main.c

The pre-check block above was placed in `main.c` **after** `osKernelStart()`:

```c
osKernelStart();

/* We should never get here as control is now taken by the scheduler */
uint32_t start_time = HAL_GetTick();
... // the entire pre-check block
```

`osKernelStart()` **never returns** once FreeRTOS starts successfully —
that comment is literally true, and everything after it is dead code that
never executes. Control jumps straight into `StartDefaultTask` in
`freertos.c`. So none of the pre-check/reset logic was actually running;
what was really happening was governed entirely by `freertos.c`, where
`rclc_support_init()` blocked with **no** IWDG refresh happening during
it, and the *only* refresh was inside the `for(;;)` loop, which only runs
**after** setup completes.

**Fix:** move the connection-check logic into `USER CODE BEGIN 2` in
`main.c` — a section that runs once, before `MX_FREERTOS_Init()` and
`osKernelStart()`:

```c
/* USER CODE BEGIN 2 */

rmw_uros_set_custom_transport(
    true, (void *) &huart2,
    cubemx_transport_open, cubemx_transport_close,
    cubemx_transport_write, cubemx_transport_read);

uint8_t was_watchdog_reset = __HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST) ? 1 : 0;
__HAL_RCC_CLEAR_RESET_FLAGS();

uint32_t start_time = HAL_GetTick();
uint8_t connected = 0;
rmw_ret_t ping_state;

while ((HAL_GetTick() - start_time) < 5000)
{
    HAL_IWDG_Refresh(&hiwdg);
    ping_state = rmw_uros_ping_agent(100, 3);
    if (ping_state == RMW_RET_OK)
    {
        connected = 1;
        break;
    }
    HAL_Delay(10);
}

if (!connected)
{
    while (1) { } // let IWDG expire and reset
}

/* USER CODE END 2 */
```

This ran once per boot as intended — but a second, deeper issue then
surfaced (see Part 6): the watchdog kept resetting **inside**
`rclc_support_init()` itself, because that call is heavier than a plain
ping and nothing refreshed the watchdog *during* it. The fix there was to
bracket the real handshake call with refreshes too:

```c
HAL_IWDG_Refresh(&hiwdg);
rclc_support_init(&support, 0, NULL, &allocator);
HAL_IWDG_Refresh(&hiwdg);
```

---

## Part 4 — Porting to STM32H753ZI: wrong UART instance

When reusing this code on a **NUCLEO-H753ZI** board (different from the
F446RE used up to this point), the board appeared completely unresponsive
— the agent would open the serial port fine but never see any handshake
traffic at all, no matter how the IWDG/ping logic was tuned.

**Root cause:** the NUCLEO-H753ZI's ST-LINK Virtual COM Port is wired to
**USART3** (PD8/PD9) — not USART2, which is what the F446RE code was
still referencing (`&huart2`). USART2 on the H753ZI is just an ordinary
GPIO-capable peripheral with nothing physically connected to it. The
board was faithfully sending data into the void.

**Fix:**
1. In CubeMX, enable **USART3** (Asynchronous), configure its DMA (RX
   Circular, TX Normal, Very High priority) and NVIC interrupt — same
   pattern as USART2 on the F446RE.
2. In code, replace every `huart2` reference with `huart3`:
```c
extern UART_HandleTypeDef huart3;
...
rmw_uros_set_custom_transport(true, (void *) &huart3, ...);
```

**Lesson for future ports:** the VCP UART peripheral number is **not**
standardized across the Nucleo family — it depends on the specific
board's ST-LINK wiring. Always verify against that board's user manual or
CubeMX's own board-template pin highlighting, rather than assuming the
previous board's UART number carries over.

---

## Part 5 — Multiple subscribers on a single-core chip

When planning a drive + arm robot control setup on the H753ZI, the
question was whether "only 1 thread" limits how many subscribers can run
concurrently. It doesn't — a single FreeRTOS task with one executor can
hold arbitrarily many subscriptions (proven earlier with 3 subscribers in
one task). The real constraint is different: **only one task should own
the UART transport / micro-ROS session at a time** — running two separate
`rclc_support_t`/executor pairs concurrently would race on the same
physical UART.

**Recommended pattern:** one dedicated comms task owns the *only*
executor and *only* session; its subscription callbacks do minimal work
(just forward the received command into a FreeRTOS queue) and return
immediately; separate independent tasks (`DriveTask`, `ArmTask`) block on
their own queues and do the actual control logic, achieving real
concurrency via FreeRTOS preemption without ever touching the transport
directly. `xQueueOverwrite` on length-1 queues keeps only the latest
command, appropriate for control loops where stale commands shouldn't
pile up. See the [drive/arm example code](#quick-reference-all-code-in-one-place)
below for the full pattern.

---

## Part 6 — The real goal: reconnect without any reset

Even with IWDG-triggered resets working, this meant a full reboot cycle
every time the agent needed restarting — noisy (interrupts any other
running task, e.g. an LED fade) and slow. The actual goal: **detect that
the agent is gone and rebuild the micro-ROS session in software**, with
no reset at all.

**First blocker:** the connection-check code only ran once, at the very
top of `StartDefaultTask`. Once it succeeded, the board created its node/
publishers/subscribers and moved permanently into the `for(;;)` spin
loop — it never revisited the connection logic. Killing the agent
(Ctrl+C) and restarting it did nothing on the board's side, since nothing
was watching for that.

**Fix:** implement the canonical micro-ROS state machine
(`WAITING_AGENT` → `AGENT_AVAILABLE` → `AGENT_CONNECTED` →
`AGENT_DISCONNECTED` → back to `WAITING_AGENT`), confirmed against the
official Vulcanexus/micro-ROS reconnection documentation. Key points from
that reference implementation:
- `WAITING_AGENT`: periodically call `rmw_uros_ping_agent()` (e.g. every
  500ms via an `EXECUTE_EVERY_N_MS`-style throttle) until it succeeds.
- `AGENT_AVAILABLE`: create all entities (node, publishers, subscribers,
  timer, executor) fresh.
- `AGENT_CONNECTED`: **periodically ping the agent even while connected**
  (this is the officially documented pattern, not a bug — pinging while
  connected is how the client notices the agent died, since a dead serial
  process gives no OS-level "connection closed" signal over plain UART).
  If the ping fails, move to `AGENT_DISCONNECTED`; otherwise spin the
  executor.
- `AGENT_DISCONNECTED`: tear down every entity (`rclc_executor_fini`,
  `rcl_publisher_fini`, `rcl_subscription_fini`, `rcl_timer_fini`,
  `rcl_node_fini`, `rclc_support_fini`), then return to `WAITING_AGENT`.

---

## Part 7 — Final state-machine code

```c
void StartDefaultTask(void *argument)
{
  rmw_uros_set_custom_transport(
      true, (void *) &huart2,   // huart3 if on H753ZI — see Part 4
      cubemx_transport_open, cubemx_transport_close,
      cubemx_transport_write, cubemx_transport_read);

  rcl_allocator_t allocator = rcl_get_default_allocator();
  rclc_support_t support;
  rcl_node_t node;
  rclc_executor_t executor;
  bool entities_created = false;

  state = WAITING_AGENT;

  for (;;)
  {
    HAL_IWDG_Refresh(&hiwdg);

    switch (state)
    {
      case WAITING_AGENT:
        if ((HAL_GetTick() - last_ping_time) > HEARTBEAT_INTERVAL_MS)
        {
          last_ping_time = HAL_GetTick();
          state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;
        }
        break;

      case AGENT_AVAILABLE:
        rclc_support_init(&support, 0, NULL, &allocator);
        rclc_node_init_default(&node, "nucleo_led_node", "", &support);

        rclc_publisher_init_default(&counter_publisher, &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "led_blink_count");
        rclc_publisher_init_default(&string_publisher, &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "nucleo_chatter");

        rclc_subscription_init_default(&led_control_subscriber, &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), "led_control");
        rclc_subscription_init_default(&fade_speed_subscriber, &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "fade_speed");
        rclc_subscription_init_default(&custom_message_subscriber, &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "custom_message");

        custom_message_msg.data.data = custom_message_buffer;
        custom_message_msg.data.size = 0;
        custom_message_msg.data.capacity = sizeof(custom_message_buffer);

        rclc_timer_init_default2(&timer, &support, RCL_MS_TO_NS(500), timer_callback, true);

        rclc_executor_init(&executor, &support.context, 4, &allocator);
        rclc_executor_add_timer(&executor, &timer);
        rclc_executor_add_subscription(&executor, &led_control_subscriber, &led_control_msg, &led_control_callback, ON_NEW_DATA);
        rclc_executor_add_subscription(&executor, &fade_speed_subscriber, &fade_speed_msg, &fade_speed_callback, ON_NEW_DATA);
        rclc_executor_add_subscription(&executor, &custom_message_subscriber, &custom_message_msg, &custom_message_callback, ON_NEW_DATA);

        counter_msg.data = 0;
        entities_created = true;
        state = AGENT_CONNECTED;
        break;

      case AGENT_CONNECTED:
        if ((HAL_GetTick() - last_ping_time) > HEARTBEAT_INTERVAL_MS)
        {
          last_ping_time = HAL_GetTick();
          if (RMW_RET_OK != rmw_uros_ping_agent(50, 1))
          {
            state = AGENT_DISCONNECTED;
            break;
          }
        }
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
        break;

      case AGENT_DISCONNECTED:
        if (entities_created)
        {
          rclc_executor_fini(&executor);
          rcl_publisher_fini(&counter_publisher, &node);
          rcl_publisher_fini(&string_publisher, &node);
          rcl_subscription_fini(&led_control_subscriber, &node);
          rcl_subscription_fini(&fade_speed_subscriber, &node);
          rcl_subscription_fini(&custom_message_subscriber, &node);
          rcl_timer_fini(&timer);
          rcl_node_fini(&node);
          rclc_support_fini(&support);
          entities_created = false;
        }
        state = WAITING_AGENT;
        break;

      default:
        break;
    }

    osDelay(10);
  }
}
```

Required near the top of `freertos.c`:
```c
typedef enum states {
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
} state_t;

state_t state = WAITING_AGENT;
uint32_t last_ping_time = 0;
const uint32_t HEARTBEAT_INTERVAL_MS = 1000;
```

---

## ⚠️ Part 8 — Known unresolved issue: reconnect fails after a few cycles

**Status: NOT YET CONFIRMED FIXED as of the last debugging session.**

With the state machine above running, the very first Ctrl+C + agent
restart worked correctly — the board detected the disconnect, tore down
its entities, and reconnected cleanly with no physical reset. However,
**on a subsequent attempt, the reconnect stopped working entirely** — the
board got stuck at `WAITING_AGENT`/never re-established a session, and
the agent log showed nothing beyond `running...` again, matching the
original symptom this whole effort was meant to fix.

### Diagnosed root cause

This matches a **documented, confirmed upstream bug** in
`rmw_microxrcedds`: [GitHub issue #241 — "Reconnecting to agent after multiple disconnects"](https://github.com/micro-ROS/rmw_microxrcedds/issues/241).
The reporter traced it to `get_memory()` in
`rmw_microxrcedds_c/src/memory.c` returning `NULL` after a small number of
disconnect/reconnect cycles (their case: failed on the 3rd–5th attempt).
Cause: the `rmw` layer's small fixed-size static memory pool isn't being
fully released by `rcl_publisher_fini()`/`rcl_node_fini()` each cycle —
each `AGENT_DISCONNECTED` → `AGENT_AVAILABLE` cycle leaks a bit of that
pool, until it's exhausted and subsequent `rcl_*_init` calls fail
silently or return `RMW_RET_ERROR`.

### The fix that was identified but not yet verified working

The upstream-recommended fix is to rebuild `libmicroros.a` with dynamic
allocation enabled instead of the small fixed static pool, by adding a
flag to `colcon.meta`:

```json
{
    "names": {
        "rmw_microxrcedds": {
            "cmake-args": [
                "-DRMW_UXRCE_MAX_NODES=1",
                "-DRMW_UXRCE_MAX_PUBLISHERS=10",
                "-DRMW_UXRCE_MAX_SUBSCRIPTIONS=5",
                "-DRMW_UXRCE_MAX_SERVICES=1",
                "-DRMW_UXRCE_MAX_CLIENTS=1",
                "-DRMW_UXRCE_MAX_HISTORY=4",
                "-DRMW_UXRCE_TRANSPORT=custom",
                "-DRMW_UXRCE_ALLOW_DYNAMIC_ALLOCATIONS=ON"
            ]
        }
    }
}
```

Applying this requires **deleting and fully rebuilding** `libmicroros/`
(editing `colcon.meta` alone has no effect until the library is
recompiled):

```bash
rm -rf micro_ros_stm32cubemx_utils/microros_static_library_ide/libmicroros/
# then rebuild in CubeIDE, or re-run the Docker library-build command manually
```

**What's still outstanding:**
- This fix has **not yet been rebuilt and re-tested** against the actual
  multi-cycle reconnect failure on this project.
- Dynamic allocation trades away some determinism (heap fragmentation
  risk over very long uptimes) — acceptable for development/testing, but
  worth knowing if this ever moves toward a long-running deployed robot.
- Even if this resolves the specific `get_memory()` NULL, it's worth
  stress-testing more than the 3-5 cycles that broke the original
  reporter's case, to build real confidence before considering this
  closed.

**Next step when picking this back up:** rebuild the library with the
flag above, then deliberately cycle the agent (Ctrl+C, restart) at least
10 times in a row while watching both the agent log and the board's
behavior, to confirm the fix actually holds under repeated cycling rather
than just working once.

---

## Quick reference: all code in one place

### IWDG CubeMX settings
```
System Core → IWDG → Activated
Prescaler: /256
Reload value: 4095   (~32.7s timeout)
```

### Enum + globals needed for the state machine
```c
typedef enum states {
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
} state_t;

state_t state = WAITING_AGENT;
uint32_t last_ping_time = 0;
const uint32_t HEARTBEAT_INTERVAL_MS = 1000;
```

### Full state-machine task
See [Part 7](#part-7--final-state-machine-code) above for the complete,
current version of `StartDefaultTask`.

### Multi-subscriber / multi-task pattern (drive + arm example)
```c
QueueHandle_t driveCmdQueue;
QueueHandle_t armCmdQueue;

rcl_subscription_t drive_subscriber;
rcl_subscription_t arm_subscriber;
geometry_msgs__msg__Twist drive_msg;
std_msgs__msg__Float32MultiArray arm_msg;
float arm_data_buffer[6];

void drive_cmd_callback(const void * msgin)
{
  const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *) msgin;
  xQueueOverwrite(driveCmdQueue, msg);
}

void arm_cmd_callback(const void * msgin)
{
  const std_msgs__msg__Float32MultiArray * msg = (const std_msgs__msg__Float32MultiArray *) msgin;
  xQueueOverwrite(armCmdQueue, msg->data.data);
}

void RosCommTask(void *argument)
{
  // set_custom_transport, support_init, node_init as usual

  rclc_subscription_init_default(&drive_subscriber, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "drive_cmd");
  rclc_subscription_init_default(&arm_subscriber, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray), "arm_cmd");

  arm_msg.data.data = arm_data_buffer;
  arm_msg.data.size = 0;
  arm_msg.data.capacity = 6;

  rclc_executor_init(&executor, &support.context, 2, &allocator);
  rclc_executor_add_subscription(&executor, &drive_subscriber, &drive_msg, &drive_cmd_callback, ON_NEW_DATA);
  rclc_executor_add_subscription(&executor, &arm_subscriber, &arm_msg, &arm_cmd_callback, ON_NEW_DATA);

  for (;;)
  {
    HAL_IWDG_Refresh(&hiwdg);
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(50));
    osDelay(10);
  }
}

void DriveTask(void *argument)
{
  geometry_msgs__msg__Twist cmd;
  for (;;)
  {
    if (xQueueReceive(driveCmdQueue, &cmd, pdMS_TO_TICKS(200)) == pdTRUE)
    {
      // motor control math using cmd.linear.x, cmd.angular.z, etc.
    }
  }
}

void ArmTask(void *argument)
{
  float joint_targets[6];
  for (;;)
  {
    if (xQueueReceive(armCmdQueue, joint_targets, pdMS_TO_TICKS(200)) == pdTRUE)
    {
      // servo/joint control math
    }
  }
}
```

### UART instance by board (lesson from Part 4)
| Board | VCP UART |
|---|---|
| NUCLEO-F446RE | USART2 (PA2/PA3) |
| NUCLEO-H753ZI | USART3 (PD8/PD9) |

Always verify against the specific board's CubeMX template rather than
assuming — this is **not** standardized across the Nucleo family.
