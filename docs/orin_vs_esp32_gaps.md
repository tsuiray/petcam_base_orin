# Orin gaps vs ESP32 firmware `cursor/esp32-arduino-hardening-26d4`

## ESP32 already provides (no Orin message-format gap)

| Item | Firmware |
|------|----------|
| Transport | Wi‑Fi UDP → `MICROROS_AGENT_IP:8888` |
| Node | `petcam_esp32_imu` |
| Topic | `/imu/data` (`"imu/data"`) |
| Type | `sensor_msgs/msg/Imu` |
| QoS | BEST_EFFORT |
| Rate | 20 ms / 50 Hz |
| Units | accel m/s², gyro rad/s |
| frame_id | `imu_link` |

So create_map’s topic/type/units were already correct.

## What was missing on Orin (root cause of your symptoms)

```
ESP32  --XRCE/UDP-->  micro_ros_agent  --DDS discovery?-->  imu_odometry / ros2 CLI
         works (hex)        publisher not visible in ROS graph
```

`imu_odometry` warning **No DDS publisher on /imu/data** means the agent never
appears in the same Fast DDS graph as create_map (not an ESP32 message bug).

### Fix in this repo
1. Local **Fast DDS Discovery Server** (`127.0.0.1:11811`)
2. `ROS_DISCOVERY_SERVER` + `FASTDDS_BUILTIN_TRANSPORTS=UDPv4` (no SHM)
3. Agent started via `ExecuteProcess` with the same env
4. Fallback: **Docker agent** (ESP32 README style)

```bash
git pull && ./scripts/build_petcam_ws.sh

# Preferred
./scripts/run_create_map.sh

# If still "No DDS publisher on /imu/data":
./scripts/run_create_map_docker_agent.sh
```

## Verify

```bash
ros2 topic hz /imu/data          # ~50
ros2 topic echo /imu/data --once
ros2 topic echo /create_map/debug
# status should become 1 (calib) then 2 (running); msg_count > 0
```
