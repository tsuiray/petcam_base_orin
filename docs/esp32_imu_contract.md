# ESP32 IMU contract (Orin ↔ sensing unit)

Source firmware:
- Repo: https://github.com/tsuiray/petcam_esp32_s3
- Branch: **`main`** (SIM L-home) — older hardening branch has no `imu_sim.*`
- Sketch: `esp32.ino` + `imu_sim.cpp` (SIM) or `mpu6050.cpp` (REAL)

## Transport

| Item | Value |
|------|-------|
| Link | Wi‑Fi STA on same home AP as Orin |
| micro-ROS agent | Orin UDP **8888** |
| Agent IP | Orin LAN IP (`MICROROS_AGENT_IP`) |
| Rate | **50 Hz** — each sample = **20 ms** of motion |

## ROS interface

| Item | Value |
|------|-------|
| Node | `petcam_esp32_imu` |
| Topic | `/imu/data` |
| Type | `sensor_msgs/msg/Imu` |
| QoS | **BEST_EFFORT** |

## SIM mode (`IMU_DATA_MODE_SIM`) — default on ESP32 `main`

Publishes a **closed L-home** (~490 sq ft / 45.5 m²):

```
(0,7)----(4.5,7)
  |            |
  |      (4.5,4)----(8,4)
  |                   |
(0,0)----------------(8,0)
```

| Field | SIM meaning |
|-------|-------------|
| `linear_acceleration.x/y` | **World-frame** m/s² (already in map axes) |
| `linear_acceleration.z` | **0** (no gravity) |
| `angular_velocity.z` | Yaw rate during corner turns |
| Intent | Naive Euler `v+=a*dt; x+=v*dt` with **fixed dt=0.02** redraws the same polygon each lap |

Orin create_map must use `imu_mode:=sim` (default):
- `accel_frame=world` (do **not** rotate by yaw)
- `use_fixed_dt=true` (ignore Wi‑Fi receive jitter)
- no 1 s bias calib during motion
- expect **~50 messages/second**

## REAL mode (`IMU_DATA_MODE_REAL`)

| Field | Unit |
|-------|------|
| accel | body-frame m/s² (±2g) |
| gyro | rad/s (±250 dps) |
| `az` | ≈ +g at rest |

```bash
./scripts/run_create_map.sh imu_mode:=real
```

## Verify

```bash
./scripts/run_create_map.sh
# look for: /imu/data rate: ~50 Hz (expect ~50 Hz / 20 ms per sample)
# L-path should overlap on lap 2+
```
