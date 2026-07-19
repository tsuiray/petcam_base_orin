# ESP32 IMU contract (Orin ↔ sensing unit)

Source firmware:
- Repo: https://github.com/tsuiray/petcam_esp32_s3
- Branch: `cursor/esp32-arduino-hardening-26d4`
- Sketch: `esp32/esp32.ino`

## Transport

| Item | Value |
|------|-------|
| Link | Wi‑Fi STA on same home AP as Orin |
| micro-ROS agent | Orin UDP `MICROROS_AGENT_PORT` **8888** |
| Agent IP | Orin LAN IP (`MICROROS_AGENT_IP` in `board_config.local.h`) |

## ROS interface

| Item | Value |
|------|-------|
| Node | `petcam_esp32_imu` |
| Topic | `/imu/data` |
| Type | `sensor_msgs/msg/Imu` |
| QoS | **BEST_EFFORT** (`rclc_publisher_init_best_effort`) |
| Rate | **50 Hz** (`IMU_PUBLISH_PERIOD_MS = 20`) |
| `header.frame_id` | `imu_link` |
| Stamp | `rmw_uros_epoch_millis()` (synced when agent ping/sync OK) |

### Units (from `mpu6050.cpp`)

| Field | Unit |
|-------|------|
| `linear_acceleration.{x,y,z}` | **m/s²** (±2g scale) |
| `angular_velocity.{x,y,z}` | **rad/s** (±250 dps scale) |
| `orientation` | identity placeholder; covariance `[0] = -1` |

## Orin create_map expectations

`create_map` already matches this contract:

```bash
./scripts/run_create_map.sh
# agent udp4:8888 + subscribe /imu/data BEST_EFFORT + map window
```

Verify link:

```bash
ros2 topic hz /imu/data          # ~50 Hz
ros2 topic echo /imu/data --once
ros2 node list                   # expect petcam_esp32_imu (+ agent side)
```
