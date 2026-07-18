# Orin micro-ROS Agent ↔ ESP32-S3

## What runs on Orin

`micro_ros_agent` is an XRCE-DDS Agent. The ESP32 micro-ROS client opens a session to it; once established, ESP32 pubs/subs appear as normal ROS 2 topics on the Orin DDS graph.

## Typical ESP32-S3 serial session

```bash
# Orin
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 -b 115200 -v6
```

Success looks like agent log lines mentioning a new client / session (verbosity dependent). Then:

```bash
ros2 topic list
ros2 topic echo /<your_esp32_topic>
```

## UDP session

```bash
# Orin (listen)
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888 -v6
```

ESP32 must target the Orin LAN IP, same port, same XRCE key/domain as configured in firmware.

## Common failures

| Symptom | Likely cause |
|---------|--------------|
| Permission denied on `/dev/ttyACM0` | User not in `dialout`; run `setup_udev_rules.sh` and re-login |
| Agent starts, no topics | Transport/port/baud mismatch; ESP32 not running client; wrong USB device |
| Intermittent disconnects | USB power / cable; WiFi RSSI for UDP; baud mismatch |
| Package `micro_ros_agent` missing | `install_microros_agent.sh` not run / overlay not sourced |

## Domain ID

```bash
export ROS_DOMAIN_ID=0   # must match ESP32 client
```
