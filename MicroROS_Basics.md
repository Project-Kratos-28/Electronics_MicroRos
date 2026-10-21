# micro-ROS LED Blink — STM32 Nucleo-F446RE + ROS 2 Jazzy

A minimal micro-ROS node running on a bare STM32 Nucleo-F446RE that blinks
the onboard LED (LD2 / PA5) on a 500 ms timer and publishes an incrementing
counter to `/led_blink_count`, bridged into ROS 2 Jazzy over UART via the
Micro-ROS Agent.

---

## Table of Contents

- [Architecture](#architecture)
- [Prerequisites](#prerequisites)
- [1. Host setup (ROS 2 Jazzy + micro-ROS Agent)](#1-host-setup-ros-2-jazzy--micro-ros-agent)
- [2. Docker setup](#2-docker-setup)
- [3. STM32CubeMX project setup](#3-stm32cubemx-project-setup)
- [4. Add the micro-ROS static library](#4-add-the-micro-ros-static-library)
- [5. Application code](#5-application-code)
- [6. Build & flash](#6-build--flash)
- [7. Run it](#7-run-it)
- [Every-restart checklist](#every-restart-checklist)
- [Troubleshooting log (real issues we hit)](#troubleshooting-log-real-issues-we-hit)
- [How it works: the protocol stack](#how-it-works-the-protocol-stack)
- [rclc API reference](#rclc-api-reference)
- [Micro XRCE-DDS Client API reference (lower level)](#micro-xrce-dds-client-api-reference-lower-level)

---

## Architecture

```
┌─────────────────────┐        UART/DMA        ┌──────────────────────┐        DDS         ┌────────────┐
│  STM32 Nucleo-F446RE │ ──────────────────────▶│  Micro-ROS Agent      │ ──────────────────▶│  ROS 2     │
│  (micro-ROS Client)  │◀────────────────────── │  (runs on your PC)    │◀────────────────── │  Jazzy     │
│  FreeRTOS + rclc     │   via ST-LINK VCP       │  translates XRCE-DDS  │   network          │  network   │
│                      │   (/dev/ttyACM0)        │  ↔ real DDS/ROS 2     │                    │            │
└─────────────────────┘                         └──────────────────────┘                    └────────────┘
```

The board never speaks full DDS directly — it's too heavy for a microcontroller.
Instead it speaks **micro-ROS / DDS-XRCE**, a lightweight client protocol, over
the same USB cable used to flash it. The **agent**, running on your laptop,
acts as the bridge/broker that makes the board's node show up as a normal
ROS 2 node (`ros2 node list`, `ros2 topic echo`, etc.).

---

## Prerequisites

- STM32 Nucleo-F446RE board + USB cable
- Ubuntu 22.04/24.04 with **ROS 2 Jazzy** installed
- **STM32CubeIDE** (bundles STM32CubeMX)
- **Docker**, usable without `sudo`

---

## 1. Host setup (ROS 2 Jazzy + micro-ROS Agent)

```bash
# Load ROS 2 environment
source /opt/ros/jazzy/setup.bash

# Create a workspace
mkdir -p ~/microros_ws/src
cd ~/microros_ws

# Get the micro-ROS build tools (branch must match your ROS distro)
git clone -b jazzy https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup

# rosdep — on Ubuntu 24.04 (Noble) the package is python3-rosdep (NOT python3-rosdep2)
sudo apt update
sudo apt install python3-rosdep
sudo rosdep init      # one-time per machine; "already exists" message is fine, skip
rosdep update

# Install this package's system dependencies
rosdep install --from-paths src --ignore-src -y

# Build the micro_ros_setup package itself
colcon build
source install/local_setup.bash

# Create + build a *second* workspace containing the actual Micro-ROS Agent
ros2 run micro_ros_setup create_agent_ws.sh
ros2 run micro_ros_setup build_agent.sh
source install/local_setup.bash
```

**Running the agent later (every session):**
```bash
source /opt/ros/jazzy/setup.bash
source ~/microros_ws/install/local_setup.bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 -b 115200
```

**Syntax breakdown:**
```
ros2 run <package_name> <executable_name> [args...]
                                  serial --dev <device_path> -b <baud_rate>
```

---

## 2. Docker setup

Docker is needed for exactly one thing: compiling `libmicroros.a` (the
static micro-ROS library) for your specific chip + config, inside a
reproducible container image, so you don't need the full ROS 2 build
toolchain installed natively.

```bash
sudo apt update
sudo apt install docker.io
sudo systemctl enable --now docker

sudo usermod -aG docker $USER
newgrp docker
docker run hello-world   # should succeed without sudo
```

> If CubeIDE was already open when you did this, close and reopen it so it
> picks up the new group membership.

---

## 3. STM32CubeMX project setup

Create the project: *File → New → STM32 Project → Board Selector →
**NUCLEO-F446RE*** (not the bare MCU selector — this auto-configures LD2
on PA5 and the ST-LINK VCP UART on USART2/PA2/PA3).

**Project Settings:**
- Toolchain / IDE: **STM32CubeIDE** (not EWARM/IAR)
- Application Structure: **Basic** (keeps everything in `main.c`, simpler for
  this size of project — “Advanced” splits files differently)

**System Core → SYS:**
- Timebase Source: **TIM6** (not SysTick — FreeRTOS needs SysTick for its own
  scheduler tick, so HAL must use a different timer)

**System Core → NVIC:**
- Priority Group: **4 bits for preemption priority, 0 bits for subpriority**
- USART2 global interrupt:
  priority **5** (any interrupt that calls FreeRTOS APIs must be ≥5)
- Time base: TIM6 global interrupt: priority **15** (CubeMX sets this
  automatically — lowest priority, correct for the periodic HAL tick)

**Middleware → FREERTOS:**
- Interface: **CMSIS_V2**
- Advanced Settings → **USE_NEWLIB_REENTRANT: Enabled** (makes libc calls
  thread-safe across tasks)
- Config parameters → **TOTAL_HEAP_SIZE: 30000** (FreeRTOS heap; micro-ROS's
  dynamic allocation needs much more than the tiny default)
- Tasks and Queues → `defaultTask` → **Stack Size: 3000 words** (~12 KB;
  micro-ROS needs >10 KB of stack for the task running the executor)

**Connectivity → USART2** (already enabled by the board template):
- NVIC Settings → enable **USART2 global interrupt**

Click **Generate Code**. Address any warnings about timebase source or
`USE_NEWLIB_REENTRANT` before generating if they appear.

---

## 4. Add the micro-ROS static library

```bash
cd /home/teerth/STM32Projects/microROS_led   # next to the .ioc file
git clone -b jazzy https://github.com/micro-ROS/micro_ros_stm32cubemx_utils.git
```

In **STM32CubeIDE → Project → Properties → C/C++ Build → Settings**:

**Tool Settings → MCU GCC Compiler → Include paths**, add:
```
../micro_ros_stm32cubemx_utils/microros_static_library_ide/libmicroros/include
```

**Tool Settings → MCU GCC Linker → Libraries:**
- Library search path (`-L`):
  ```
  ../micro_ros_stm32cubemx_utils/microros_static_library_ide/libmicroros
  ```
- Libraries (`-l`): `microros`

**Build Steps → Pre-build steps**, add (absolute path avoids
`workspace_loc` ambiguity):
```bash
docker pull microros/micro_ros_static_library_builder:jazzy && docker run --rm -v /home/teerth/STM32Projects/microROS_led:/project --env MICROROS_LIBRARY_FOLDER=micro_ros_stm32cubemx_utils/microros_static_library_ide microros/micro_ros_static_library_builder:jazzy

docker run --rm -v /home/teerth/STM32Projects/test_iwdg:/project --env MICROROS_LIBRARY_FOLDER=micro_ros_stm32cubemx_utils/microros_static_library_ide microros/micro_ros_static_library_builder:jazzy
```

Copy the transport/allocator/time source files the library needs into
`Src/` so CubeIDE actually compiles them (these implement the functions
declared as `extern` in `main.c` — without this step you get **undefined
reference** linker errors):

```bash
cd /home/teerth/STM32Projects/microROS_led
cp micro_ros_stm32cubemx_utils/extra_sources/custom_memory_manager.c Src/
cp micro_ros_stm32cubemx_utils/extra_sources/microros_allocators.c Src/
cp micro_ros_stm32cubemx_utils/extra_sources/microros_time.c Src/
cp micro_ros_stm32cubemx_utils/extra_sources/microros_transports/it_transport.c Src/
```

Refresh (F5) the `Src` folder in Project Explorer afterward so CubeIDE
picks up the new files.

First build takes a few minutes (Docker compiles ~78 packages). Subsequent
builds print `micro-ROS library found. Skipping...` and reuse the cached
library — delete `micro_ros_stm32cubemx_utils/microros_static_library_ide/libmicroros/`
to force a rebuild (needed if you edit `colcon.meta`, e.g. to add a new
message type).

---

## 5. Application code

All in `Src/main.c` (Basic application structure keeps everything there).

**Includes** (inside `/* USER CODE BEGIN Includes */`):
```c
#include "rcl/rcl.h"
#include "rclc/rclc.h"
#include "rclc/executor.h"
#include "rmw_microros/rmw_microros.h"
#include <std_msgs/msg/int32.h>
```

**Private variables** (inside `/* USER CODE BEGIN PV */`):
```c
extern UART_HandleTypeDef huart2;

// implemented in dma_transport.c
bool cubemx_transport_open(struct uxrCustomTransport * transport);
bool cubemx_transport_close(struct uxrCustomTransport * transport);
size_t cubemx_transport_write(struct uxrCustomTransport* transport, const uint8_t * buf, size_t len, uint8_t * err);
size_t cubemx_transport_read(struct uxrCustomTransport* transport, uint8_t * buf, size_t len, int timeout, uint8_t * err);

#define LED_PORT GPIOA
#define LED_PIN  GPIO_PIN_5

rcl_publisher_t publisher;
std_msgs__msg__Int32 msg;
rcl_timer_t timer;
```

**Timer callback** (before `StartDefaultTask`, e.g. in `USER CODE BEGIN 4`):
```c
void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
  (void) last_call_time;
  if (timer != NULL) {
    HAL_GPIO_TogglePin(LED_PORT, LED_PIN);
    rcl_publish(&publisher, &msg, NULL);
    msg.data++;
  }
}
```

**Task body** (inside `StartDefaultTask`, between
`/* USER CODE BEGIN 5 */` and `/* USER CODE END 5 */`):
```c
void StartDefaultTask(void *argument)
{
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
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "led_blink_count");

  const unsigned int timer_period_ms = 500;
  rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_period_ms),
    timer_callback);

  rclc_executor_t executor;
  rclc_executor_init(&executor, &support.context, 1, &allocator);
  rclc_executor_add_timer(&executor, &timer);

  msg.data = 0;

  for (;;)
  {
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
    osDelay(10);
  }
}
```

---

## 6. Build & flash

Right-click project → **Build Project**. Then flash with the green
**Run** (▶) button (use Run, not Debug, unless you plan to manually hit
Resume — Debug halts the chip until you do).

---

## 7. Run it

```bash
# Terminal 1 — leave running
source /opt/ros/jazzy/setup.bash
source ~/microros_ws/install/local_setup.bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 -b 115200
```

Press the black **RESET** button on the Nucleo once the agent is running.

```bash
# Terminal 2 — verify
source /opt/ros/jazzy/setup.bash
ros2 node list                    # should show /nucleo_led_node
ros2 topic echo /led_blink_count  # should show an incrementing int32
```

LD2 should blink at 1 Hz (toggles every 500 ms in the timer callback).

---

## Every-restart checklist

Everything above is **one-time setup**. After a reboot, this is all you
need:

1. **Flash the board** (only if it lost power or code changed): open
   CubeIDE → project → **Run** (▶).
2. **Start the agent:**
   ```bash
   source /opt/ros/jazzy/setup.bash
   source ~/microros_ws/install/local_setup.bash
   ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 -b 115200
   ```
3. **Press RESET** on the Nucleo once the agent is running.
4. Verify with `ros2 node list` / `ros2 topic echo` if needed.

---

## Troubleshooting log (real issues we hit)

| Symptom | Cause | Fix |
|---|---|---|
| `rosdep: command not found` | `python3-rosdep2` doesn't exist on Noble | Install `python3-rosdep` instead |
| `E: Package 'python3-rosdep2' has no installation candidate` | Same as above, wrong package name for Ubuntu 24.04 | `sudo apt install python3-rosdep` |
| CubeIDE pre-build `Error 35` | Transient Docker permission issue right after `newgrp docker` | Re-ran the exact Docker command manually — actually succeeded, library was already built |
| `undefined reference to cubemx_transport_read/write/close/open` | Declared the transport functions but never compiled their implementation | Copy `dma_transport.c` (and allocator/time/memory-manager files) from `extra_sources/` into `Src/` |
| `undefined reference to clock_gettime` | Same root cause — `microros_time.c` not compiled into the project | Copy `microros_time.c` into `Src/` too |
| Agent shows port open (`fd: 22`) but zero further activity, LED never blinks | `rclc_support_init()` blocks waiting for the agent; if firmware hangs before reaching the main loop, nothing is ever sent | Confirm agent is running *before* reset; increase `TOTAL_HEAP_SIZE` (FreeRTOS heap) if allocation inside micro-ROS init silently fails |
| Couldn't find `StartDefaultTask` in `freertos.c` | With **Basic** application structure, CubeMX put everything in `main.c` instead of a separate `freertos.c` | Search `main.c` with Ctrl+F instead |

---

## How it works: the protocol stack

```
Your code (StartDefaultTask)
   ↓
rclc   — convenience layer: rclc_node_init_default, rclc_publisher_init_default...
   ↓
rcl    — rcl_init, rcl_node_init, rcl_publish...
   ↓
rmw_microxrcedds  — the "ROS middleware" implementation backed by micro-ROS
   ↓
Micro XRCE-DDS Client library  — session, streams, entities, serialization
   ↓
your custom transport (dma_transport.c → cubemx_transport_open/read/write/close)
   ↓
UART/DMA hardware → wire → micro_ros_agent → real DDS/ROS 2 network
```

**Micro XRCE-DDS** (eProsima) implements the DDS-XRCE protocol so
resource-constrained microcontrollers can participate in a DDS/ROS 2
network without running full DDS. It's a client/server model: your board
is the **Client**, `micro_ros_agent` is the **Agent** — a broker that
bridges the client into the real DDS global data space.

Data isn't sent as raw C structs — it's serialized into **CDR** (Common
Data Representation) via the `ucdr` (micro-CDR) library before hitting the
transport, and deserialized back on the other side.

---

## rclc API reference

**Support:**
```c
rcl_ret_t rclc_support_init(rclc_support_t *, int argc, char const * const * argv, rcl_allocator_t *);
rcl_ret_t rclc_support_init_with_options(rclc_support_t *, int argc, char const * const * argv, rcl_init_options_t *, rcl_allocator_t *);
rcl_ret_t rclc_support_fini(rclc_support_t *);
```

**Node:**
```c
rcl_ret_t rclc_node_init_default(rcl_node_t *, const char * name, const char * namespace_, rclc_support_t *);
rcl_ret_t rclc_node_init_with_options(rcl_node_t *, const char * name, const char * namespace_, rclc_support_t *, rcl_node_options_t *);
```

**Publisher:**
```c
rcl_ret_t rclc_publisher_init_default(rcl_publisher_t *, const rcl_node_t *, const rosidl_message_type_support_t *, const char * topic_name);
rcl_ret_t rclc_publisher_init_best_effort(rcl_publisher_t *, const rcl_node_t *, const rosidl_message_type_support_t *, const char * topic_name);
rcl_ret_t rclc_publisher_init(rcl_publisher_t *, const rcl_node_t *, const rosidl_message_type_support_t *, const char * topic_name, const rcl_publisher_options_t *);
```

**Subscription:**
```c
rcl_ret_t rclc_subscription_init_default(rcl_subscription_t *, const rcl_node_t *, const rosidl_message_type_support_t *, const char * topic_name);
rcl_ret_t rclc_subscription_init_best_effort(rcl_subscription_t *, const rcl_node_t *, const rosidl_message_type_support_t *, const char * topic_name);
```

**Timer:**
```c
rcl_ret_t rclc_timer_init_default(rcl_timer_t *, rclc_support_t *, rcl_duration_value_t timeout_ns, rcl_timer_callback_t);
rcl_ret_t rclc_timer_init_default2(rcl_timer_t *, rclc_support_t *, rcl_duration_value_t timeout_ns, rcl_timer_callback_t, bool autostart); // preferred, non-deprecated
rcl_ret_t rclc_timer_cancel(rcl_timer_t *);
rcl_ret_t rclc_timer_reset(rcl_timer_t *);
```

**Executor:**
```c
rcl_ret_t rclc_executor_init(rclc_executor_t *, rcl_context_t *, size_t number_of_handles, const rcl_allocator_t *);
rcl_ret_t rclc_executor_fini(rclc_executor_t *);

rcl_ret_t rclc_executor_add_subscription(rclc_executor_t *, rcl_subscription_t *, void * msg, rclc_subscription_callback_t, rclc_executor_handle_invocation_t); // ALWAYS or ON_NEW_DATA
rcl_ret_t rclc_executor_add_timer(rclc_executor_t *, rcl_timer_t *);
rcl_ret_t rclc_executor_add_service(rclc_executor_t *, rcl_service_t *, void * request_msg, void * response_msg, rclc_service_callback_t);
rcl_ret_t rclc_executor_add_client(rclc_executor_t *, rcl_client_t *, void * response_msg, rclc_client_callback_t);

rcl_ret_t rclc_executor_spin(rclc_executor_t *);                                  // blocking, forever
rcl_ret_t rclc_executor_spin_some(rclc_executor_t *, uint64_t timeout_ns);        // one pass — used in this project
rcl_ret_t rclc_executor_spin_one_period(rclc_executor_t *, uint64_t period_ns);   // spin_some on fixed cadence

rcl_ret_t rclc_executor_set_timeout(rclc_executor_t *, uint64_t timeout_ns);
rcl_ret_t rclc_executor_set_semantics(rclc_executor_t *, rclc_executor_semantics_t);
rcl_ret_t rclc_executor_remove_subscription(rclc_executor_t *, const rcl_subscription_t *);
```

**Callback signatures:**
```c
typedef void (*rcl_timer_callback_t)(rcl_timer_t *, int64_t);
typedef void (*rclc_subscription_callback_t)(const void * msgin);
typedef void (*rclc_service_callback_t)(const void * req, void * res);
typedef void (*rclc_client_callback_t)(const void * res);
```

**Service & client:**
```c
rcl_ret_t rclc_service_init_default(rcl_service_t *, const rcl_node_t *, const rosidl_service_type_support_t *, const char * service_name);
rcl_ret_t rclc_client_init_default(rcl_client_t *, const rcl_node_t *, const rosidl_service_type_support_t *, const char * service_name);
```

**Parameters:**
```c
rcl_ret_t rclc_parameter_server_init_default(rclc_parameter_server_t *, rcl_node_t *);
rcl_ret_t rclc_executor_add_parameter_server(rclc_executor_t *, rclc_parameter_server_t *, rclc_parameter_callback_t);
bool rclc_add_parameter(rclc_parameter_server_t *, const char * name, rclc_parameter_type_t);
bool rclc_parameter_set_int(rclc_parameter_server_t *, const char * name, int);
bool rclc_parameter_set_bool(rclc_parameter_server_t *, const char * name, bool);
```

---

## Micro XRCE-DDS Client API reference (lower level)

You never call these directly — `rclc` generates them internally — but
they're useful for reading micro-ROS source/logs.

**Transport:**
```c
bool uxr_init_udp_transport(uxrUDPTransport *, uxrUDPPlatform *, const char * ip, uint16_t port);
bool uxr_init_serial_transport(uxrSerialTransport *, uxrSerialPlatform *, int fd, uint8_t remote_addr, uint8_t local_addr);

struct uxrCustomTransport {
    void* args;
    uxrCustomTransportOpen open;
    uxrCustomTransportClose close;
    uxrCustomTransportWrite write;
    uxrCustomTransportRead read;
};
```

**Session:**
```c
void uxr_init_session(uxrSession *, uxrCommunication * comm, uint32_t key); // key: any arbitrary uint32_t you choose
bool uxr_create_session(uxrSession *);
bool uxr_delete_session(uxrSession *);
bool uxr_delete_session_retries(uxrSession *, size_t retries);

void uxr_set_topic_callback(uxrSession *, uxrOnTopicFunc, void * args);
void uxr_set_status_callback(uxrSession *, uxrOnStatusFunc, void * args);
```

**Streams:**
```c
uxrStreamId uxr_create_output_best_effort_stream(uxrSession *, uint8_t * buffer, size_t size);
uxrStreamId uxr_create_output_reliable_stream(uxrSession *, uint8_t * buffer, size_t size, size_t history);
uxrStreamId uxr_create_input_best_effort_stream(uxrSession *);
uxrStreamId uxr_create_input_reliable_stream(uxrSession *, uint8_t * buffer, size_t size, size_t history);
```

**Entities:**
```c
uxrObjectId uxr_object_id(uint16_t id, uint8_t type); // UXR_PARTICIPANT_ID, UXR_TOPIC_ID, UXR_PUBLISHER_ID, UXR_SUBSCRIBER_ID, UXR_DATAWRITER_ID, UXR_DATAREADER_ID

uint16_t uxr_buffer_create_participant_ref(uxrSession *, uxrStreamId, uxrObjectId, uint16_t domain_id, const char * ref, uint8_t mode);
uint16_t uxr_buffer_create_topic_ref(uxrSession *, uxrStreamId, uxrObjectId, uxrObjectId participant_id, const char * ref, uint8_t mode);
uint16_t uxr_buffer_create_publisher_ref(uxrSession *, uxrStreamId, uxrObjectId, uxrObjectId participant_id, const char * ref, uint8_t mode);
uint16_t uxr_buffer_create_datawriter_ref(uxrSession *, uxrStreamId, uxrObjectId, uxrObjectId publisher_id, const char * ref, uint8_t mode);
uint16_t uxr_buffer_create_subscriber_ref(uxrSession *, uxrStreamId, uxrObjectId, uxrObjectId participant_id, const char * ref, uint8_t mode);
uint16_t uxr_buffer_create_datareader_ref(uxrSession *, uxrStreamId, uxrObjectId, uxrObjectId subscriber_id, const char * ref, uint8_t mode);
```

**Publishing / subscribing:**
```c
bool uxr_prepare_output_stream(uxrSession *, uxrStreamId, uxrObjectId datawriter_id, struct ucdrBuffer *, uint32_t topic_size);
// then: <MsgType>_serialize_topic(&buffer, &msg);

uint16_t uxr_buffer_request_data(uxrSession *, uxrStreamId, uxrObjectId datareader_id, uxrStreamId data_stream_id, uxrDeliveryControl *);
```

**Running the session:**
```c
void uxr_run_session_time(uxrSession *, int timeout_ms);
bool uxr_run_session_until_timeout(uxrSession *, int timeout_ms);
bool uxr_run_session_until_confirmed_delivery(uxrSession *, int timeout_ms);
bool uxr_run_session_until_all_status(uxrSession *, int timeout_ms, const uint16_t * request_id, uint8_t * status, size_t num_status);
bool uxr_run_session_until_one_status(uxrSession *, int timeout_ms, const uint16_t * request_id, uint8_t * status, size_t num_status);
```

**Topic callback:**
```c
void on_topic(uxrSession *, uxrObjectId object_id, uint16_t request_id, uxrStreamId stream_id, struct ucdrBuffer * ub, uint16_t length, void * args);
```

**rclc → uxr mapping:**

| rclc call | uxr equivalent underneath |
|---|---|
| `rclc_support_init` | `uxr_init_session` + `uxr_create_session` |
| `rclc_node_init_default` | `uxr_buffer_create_participant_ref` |
| `rclc_publisher_init_default` | `uxr_buffer_create_topic_ref` + `_publisher_ref` + `_datawriter_ref` |
| `rcl_publish` | `uxr_prepare_output_stream` + serialize + write |
| `rclc_executor_spin_some` | `uxr_run_session_time` (roughly) |

---

## Repo layout reference

```
microROS_led/
├── microROS_led.ioc
├── Inc/
├── Src/
│   ├── main.c                    ← application code lives here (Basic structure)
│   ├── custom_memory_manager.c   ← copied from extra_sources/
│   ├── microros_allocators.c     ← copied from extra_sources/
│   ├── microros_time.c           ← copied from extra_sources/
│   └── dma_transport.c           ← copied from extra_sources/microros_transports/
├── Drivers/
├── Middlewares/
├── STM32F446RETX_FLASH.ld
├── STM32F446RETX_RAM.ld
└── micro_ros_stm32cubemx_utils/
    └── microros_static_library_ide/
        ├── library_generation/
        │   └── colcon.meta        ← controls which message packages get built in
        └── libmicroros/           ← generated by Docker; libmicroros.a + include/
```
