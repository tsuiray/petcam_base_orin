# create_map — first communication task on Orin base

## Goal

ESP32 publishes IMU at **50 Hz** (each sample = **20 ms** of motion).
Orin `create_map`:

1. Receives `/imu/data`
2. Integrates with **fixed dt=0.02 s** in SIM mode (not Wi‑Fi receive jitter)
3. Draws the path; ESP32 SIM L-home should **overlap** each lap

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

Aligned with firmware branch **`main`**
([petcam_esp32_s3](https://github.com/tsuiray/petcam_esp32_s3) — `imu_sim.cpp`):

| Field | Value |
|-------|-------|
| Topic | `/imu/data` |
| Type | `sensor_msgs/Imu` |
| QoS | BEST_EFFORT |
| Rate | **50 Hz** (20 ms/sample) |
| SIM accel | **world-frame** m/s², `az=0` |
| REAL accel | body-frame m/s² + gravity |
| Agent | UDP 8888 |

Full contract: [`docs/esp32_imu_contract.md`](esp32_imu_contract.md)

## Run with ESP32 SIM (default)

```bash
./scripts/run_create_map.sh
# imu_mode:=sim → world-frame + fixed 20 ms
# log should show: /imu/data rate: ~50 Hz
```

Point ESP32 agent IP at Orin, port **8888**. L-path (~500 sq ft) should redraw on itself each lap.

## REAL MPU6050

```bash
./scripts/run_create_map.sh imu_mode:=real
```

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
- `zupt_hold_sec` — only zero velocity after this long still (default 0.35 s)

### If distance stays 0 / no trail

1. Keep robot **still ~1 s** after launch (calibration).
2. Then **push/move** the robot — coasting is OK now (ZUPT is delayed).
3. Watch HUD: `dt` should be ~20 ms, `a_xy` should spike when you accelerate, `zupt: off` while moving.
4. Terminal log every ~1 s shows `dist=` and `path_pts=`.

```bash
ros2 topic echo /create_map/debug --once
# [dt, ax_b, ay_b, ax_w, ay_w, vx, vy, zupt, still_sec, distance, x, y]
```

## Note on accuracy

Accel double-integration drifts quickly. This app is the **first comms + visualization** pipeline; later we can fuse wheel odometry / vision.