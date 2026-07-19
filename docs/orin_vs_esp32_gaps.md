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
ESP32  --XRCE/UDP-->  micro_ros_agent  --DDS/SHM?-->  imu_odometry / ros2 CLI
         works (hex)                      BROKEN
```

1. **Fast DDS Shared Memory bridge**  
   Agent (from `~/microros_ws`) and system ROS 2 Humble nodes often **cannot share SHM**.  
   Symptom: agent `-v6` hex dumps IMU, but `/create_map/debug` stays all zeros (`status=0`) and `ros2 topic hz /imu/data` fails.

2. **Fix now in this repo**  
   - `FASTDDS_BUILTIN_TRANSPORTS=UDPv4` for agent + create_map (disable SHM)  
   - Fast DDS XML UDP-only profile  
   - `ros2 daemon stop` before launch  
   - Dual QoS subscriptions on `/imu/data`

3. **Operational checklist**  
   - Only **one** agent on UDP 8888 (no leftover docker agent)  
   - ESP32 `MICROROS_AGENT_IP` = Orin Wi‑Fi IP  
   - Same `ROS_DOMAIN_ID` (default `0`)  
   - After pull: `./scripts/build_petcam_ws.sh && ./scripts/run_create_map.sh`

## Verify

```bash
ros2 topic hz /imu/data          # ~50
ros2 topic echo /imu/data --once
ros2 topic echo /create_map/debug
# status should become 1 (calib) then 2 (running); msg_count > 0
```
