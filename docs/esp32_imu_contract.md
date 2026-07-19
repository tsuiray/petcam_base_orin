# ESP32 IMU contract (Orin ↔ sensing unit)

Source firmware:
- Repo: https://github.com/tsuiray/petcam_esp32_s3
- Preferred base: **`main`** (has SIM L-home) + TCP files from this Orin repo
- Older: `cursor/esp32-arduino-hardening-26d4` — **UDP only**, no SIM
- Apply-ready TCP patch (headers + steps): [`docs/esp32_tcp_port/`](esp32_tcp_port/README.md)

## Transport (default: TCP)

| Item | Value |
|------|-------|
| Link | Wi‑Fi STA on same home AP as Orin |
| micro-ROS agent | Orin **`tcp4`** port **8888** |
| ESP32 | `MICROROS_TRANSPORT_TCP` in `board_config.h` |
| Agent IP | Orin LAN IP (`MICROROS_AGENT_IP`) |
| Rate | **50 Hz** — each sample = **20 ms** of motion |

```bash
# Orin
./scripts/run_create_map.sh          # tcp4 :8888
# ESP32 Serial should say: micro-ROS TCP transport -> <orin-ip>:8888
```

UDP fallback (both sides):
```cpp
// ESP32 board_config.h
#define MICROROS_TRANSPORT MICROROS_TRANSPORT_UDP
```
```bash
./scripts/run_create_map.sh transport:=udp4
```

## What `cursor/esp32-arduino-hardening-26d4` needs for TCP

That branch today:

| Present | Missing for TCP / SIM |
|---------|------------------------|
| `wifi_transport.cpp` (UDP only) | `wifi_tcp_transport.h` / `.cpp` |
| `set_microros` / UDP bind | `MICROROS_TRANSPORT` switch in `board_config.h` + `esp32.ino` |
| REAL MPU6050 path | `imu_sim.*` (only on `main`) |

**Minimum port** from `cursor/microros-tcp-1706` / `main`:

1. Add `wifi_tcp_transport.h` + `wifi_tcp_transport.cpp`
2. In `board_config.h`:
   ```cpp
   #define MICROROS_TRANSPORT_UDP 0
   #define MICROROS_TRANSPORT_TCP 1
   #define MICROROS_TRANSPORT MICROROS_TRANSPORT_TCP
   ```
3. In `esp32.ino` `bind_microros_wifi_transport()`: call `arduino_wifi_tcp_transport_*` when TCP
4. Flash ESP32, run Orin `transport:=tcp4`

Prefer flashing **`cursor/microros-tcp-1706`** (TCP + SIM L-home) instead of porting by hand.

## ROS interface

| Item | Value |
|------|-------|
| Node | `petcam_esp32_imu` |
| Topic | `/imu/data` |
| Type | `sensor_msgs/msg/Imu` |
| QoS | **BEST_EFFORT** (TCP still retransmits XRCE bytes) |

## SIM mode (`IMU_DATA_MODE_SIM`)

Closed L-home (~490 sq ft). World-frame `ax/ay`, `az=0`, fixed `dt=0.02` on Orin.

## Reliable delivery

TCP reduces lost samples vs UDP. ROS 2 “secure DDS” ≠ loss-free.  
See earlier table: TCP / RELIABLE vs BEST_EFFORT UDP.

## Verify

```bash
./scripts/run_create_map.sh
# create_map RX: ~50/50 Hz loss~0% ...
# ss -tlnp | grep 8888   → micro_ros_agent listening TCP
```
