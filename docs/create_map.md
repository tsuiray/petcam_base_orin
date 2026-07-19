# create_map — first communication task on Orin base

## Goal

ESP32-S3 sends MPU6050 6-axis IMU (~every **20 ms** over micro-ROS UDP).
Orin `create_map` app:

1. Receives IMU packets
2. Uses **Δt between consecutive packets** to integrate accel → velocity → position
3. Draws the robot path on a live map window

## Data flow

```
ESP32 MPU6050  --UDP XRCE-->  micro_ros_agent  -->  /imu/data (sensor_msgs/Imu)
                                                      |
                                               imu_odometry
                                                      |
                         /create_map/path  /create_map/pose  /create_map/distance
                                                      |
                                                 map_viewer (OpenCV)
```

## Expected ESP32 message

Aligned with firmware branch **`cursor/esp32-arduino-hardening-26d4`**
([petcam_esp32_s3](https://github.com/tsuiray/petcam_esp32_s3)):

| Field | Value |
|-------|-------|
| Topic | `/imu/data` |
| Type | `sensor_msgs/Imu` |
| QoS | BEST_EFFORT |
| Rate | 50 Hz (20 ms) |
| Accel | m/s² |
| Gyro | rad/s |
| Agent | UDP 8888 |

Full contract: [`docs/esp32_imu_contract.md`](esp32_imu_contract.md)

Also accepted (not used by current ESP32 FW): `std_msgs/Float64MultiArray` on `/imu/raw`
with `[ax, ay, az, gx, gy, gz]` — set `prefer_raw: true` in config.

## Run with real ESP32

```bash
# terminals already sourced (ROS + microros_ws + petcam_base_orin)
ros2 launch create_map create_map.launch.py
# starts: micro-ROS agent udp4:8888 + imu_odometry + map_viewer
```

Point ESP32 agent IP at Orin, port **8888**. Move the robot; the OpenCV window shows the path.

## Verify without ESP32 (mock 20 ms IMU)

```bash
ros2 launch create_map create_map.launch.py use_mock_imu:=true start_microros_agent:=false
```

You should see a path grow and `/create_map/distance` increase:

```bash
ros2 topic echo /create_map/distance
ros2 topic hz /imu/data
```

## Tuning

Edit `src/create_map/config/create_map.yaml`:

- `imu_topic` — match your ESP32 publisher
- `accel_in_g` — true if accel units are g
- `calibrate_on_start_sec` — keep robot still at start for bias
- `enable_zupt` — zero velocity when nearly still (reduces drift)

## Note on accuracy

Accel double-integration drifts quickly. This app is the **first comms + visualization** pipeline; later we can fuse wheel odometry / vision. ZUPT + short calibration are included to keep the first demo usable.
