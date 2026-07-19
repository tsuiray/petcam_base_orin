# ESP32 TCP transport patch (apply to petcam_esp32_s3)

Orin create_map now defaults to `micro_ros_agent tcp4 :8888`.
ESP32 must use matching TCP XRCE transport.

## Preferred firmware

Use / merge branch work based on **`main`** (has SIM L-home):

- Copy `wifi_tcp_transport.h` + `wifi_tcp_transport.cpp` into the sketch folder
- Update `board_config.h` and `esp32.ino` as below

Cloud agent could not push to `tsuiray/petcam_esp32_s3` (403). Apply locally or open a PR from your machine.

## Needed on `cursor/esp32-arduino-hardening-26d4`

That branch is **UDP-only** and has **no `imu_sim.*`**.

| File | Action |
|------|--------|
| `wifi_tcp_transport.h` / `.cpp` | **Add** (this folder) |
| `wifi_transport.cpp` | Keep (UDP fallback) |
| `board_config.h` | Add `MICROROS_TRANSPORT_*` (see snippet) |
| `esp32.ino` | Switch `bind_microros_wifi_transport()` (see snippet) |
| SIM L-home | Only on `main` — merge `imu_sim.*` if you need SIM |

### board_config.h snippet

```cpp
#define MICROROS_TRANSPORT_UDP 0
#define MICROROS_TRANSPORT_TCP 1
#ifndef MICROROS_TRANSPORT
#define MICROROS_TRANSPORT MICROROS_TRANSPORT_TCP
#endif
```

### esp32.ino bind snippet

```cpp
#if MICROROS_TRANSPORT == MICROROS_TRANSPORT_TCP
#include "wifi_tcp_transport.h"
#endif

void bind_microros_wifi_transport(const char *agent_ip, uint16_t agent_port) {
  static micro_ros_agent_locator locator;
  // ... fill locator.address / locator.port ...
#if MICROROS_TRANSPORT == MICROROS_TRANSPORT_TCP
  rmw_uros_set_custom_transport(
      false, &locator, arduino_wifi_tcp_transport_open,
      arduino_wifi_tcp_transport_close, arduino_wifi_tcp_transport_write,
      arduino_wifi_tcp_transport_read);
#else
  rmw_uros_set_custom_transport(
      false, &locator, arduino_wifi_transport_open,
      arduino_wifi_transport_close, arduino_wifi_transport_write,
      arduino_wifi_transport_read);
#endif
}
```

### Orin

```bash
./scripts/run_create_map.sh          # tcp4
# Serial should show: micro-ROS TCP transport -> <orin-ip>:8888
```
