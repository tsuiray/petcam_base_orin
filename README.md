# petcam_base_orin

ROS 2 Humble workspace for the PetCam **NVIDIA Orin base station**.

This repo currently focuses on bringing up the **micro-ROS Agent** so the Orin can connect to the **ESP32-S3 robot sensing unit** (already flashing / waiting on the robot side).

## Architecture (this slice)

```
ESP32-S3 (micro-ROS client)  <--- XRCE-DDS --->  Orin micro-ROS Agent  <--- DDS --->  ROS 2 Humble graph
         sensors / cmd_vel                              (this repo)
```

Later slices on this Orin repo: Rockchip 1126 RTSP ingest, perception, teleop UI.

## Prerequisites (on Orin)

- JetPack 6.x / **Ubuntu 22.04** recommended for native ROS 2 Humble
- ESP32-S3 powered and running your micro-ROS firmware over **UDP**
- ESP32 must target the Orin LAN IP; agent default listen port is **8888**
- Serial USB remains available as a fallback (`transport:=serial`)

JetPack 5 / Ubuntu 20.04: use the Docker path under `docker/`, or upgrade to JP6.

## One-time install on Orin

```bash
git clone https://github.com/tsuiray/petcam_base_orin.git
cd petcam_base_orin

chmod +x scripts/*.sh src/petcam_bringup/scripts/*.sh docker/entrypoint_microros.sh

# 1) ROS 2 Humble
./scripts/install_ros2_humble.sh

# 2) micro-ROS Agent (~/microros_ws by default)
./scripts/install_microros_agent.sh

# 3) This repo's bringup package
./scripts/build_petcam_ws.sh

# 4) Optional: stable serial symlink /dev/petcam_sensing
./scripts/setup_udev_rules.sh
```

Open a **new terminal** (or `source ~/.bashrc`) so all overlays are loaded.

## Connect the ESP32-S3 (UDP)

ESP32 firmware uses UDP XRCE-DDS. Point the client at the Orin IP and port `8888`.

```bash
# default is already udp4 :8888
./scripts/run_microros_agent.sh

# or explicitly:
ros2 launch petcam_bringup microros_agent.launch.py transport:=udp4 port:=8888
```

Optional serial USB fallback:

```bash
TRANSPORT=serial SERIAL_DEV=/dev/ttyACM0 ./scripts/run_microros_agent.sh
```

### Verify link

In another shell:

```bash
ros2 run petcam_bringup check_microros_connection.sh
# or manually:
ros2 node list
ros2 topic list
```

On a good link you should see ESP32-published topics/nodes appear in the ROS 2 graph. Agent logs at `-v6` show session establish / create participant traffic.

## Docker alternative

Native build is preferred on Orin (arm64). Compose file is provided for convenience:

```bash
# udp (matches ESP32)
docker compose -f docker/docker-compose.microros.yml --profile udp up

# serial fallback
SERIAL_DEV=/dev/ttyACM0 docker compose -f docker/docker-compose.microros.yml --profile serial up
```

To build an arm64 agent image locally:

```bash
docker build -f docker/Dockerfile.microros -t petcam/microros-agent:humble .
```

## Optional systemd auto-start

```bash
cp systemd/petcam-microros-agent.service /tmp/petcam-microros-agent.service
# edit User= and ExecStart= paths, then:
sudo cp /tmp/petcam-microros-agent.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now petcam-microros-agent.service
```

## Repo layout

| Path | Role |
|------|------|
| `scripts/install_ros2_humble.sh` | Install ROS 2 Humble on Orin |
| `scripts/install_microros_agent.sh` | Build micro-ROS Agent via `micro_ros_setup` |
| `scripts/build_petcam_ws.sh` | `colcon build` this workspace |
| `scripts/run_microros_agent.sh` | Source overlays + launch agent |
| `src/petcam_bringup/` | Launch + health-check helpers |
| `docker/` | Optional containerized agent |
| `systemd/` | Optional service unit |

## Matching the ESP32 firmware

The agent transport **must** match the client XRCE config on the ESP32-S3:

| ESP32 client | Orin agent launch |
|--------------|-------------------|
| UDP port 8888 (default) | `transport:=udp4 port:=8888` |
| Serial USB, 115200 | `transport:=serial serial_dev:=/dev/ttyACM0 serial_baud:=115200` |
| TCP port 8888 | `transport:=tcp4 port:=8888` |

Also keep `ROS_DOMAIN_ID` consistent (default `0`).

If your ESP32 uses a non-default device, baud, or port, pass those launch args or set `SERIAL_DEV` / `SERIAL_BAUD` / `PORT` / `TRANSPORT` when calling `run_microros_agent.sh`.
