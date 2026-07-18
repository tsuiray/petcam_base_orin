#!/usr/bin/env bash
# Optional udev rules so ESP32-S3 serial devices are readable without root
# and get a stable symlink: /dev/petcam_sensing
set -euo pipefail

RULES_FILE="/etc/udev/rules.d/99-petcam-esp32.rules"

# Common ESP32-S3 USB VID/PID values:
# - Espressif USB JTAG/serial: 303a:1001
# - CP210x / CH340 bridges vary; also match ttyACM* by vendor if present
sudo tee "${RULES_FILE}" > /dev/null <<'EOF'
# PetCam ESP32-S3 sensing unit (Espressif USB Serial/JTAG)
SUBSYSTEM=="tty", ATTRS{idVendor}=="303a", ATTRS{idProduct}=="1001", MODE="0666", SYMLINK+="petcam_sensing"
# Espressif USB CDC ACM (alternate product IDs seen on some boards)
SUBSYSTEM=="tty", ATTRS{idVendor}=="303a", MODE="0666", SYMLINK+="petcam_sensing_%n"
# Silicon Labs CP210x (some ESP32 USB-UART bridges)
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE="0666", SYMLINK+="petcam_sensing"
# WCH CH340
SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", MODE="0666", SYMLINK+="petcam_sensing"
EOF

sudo udevadm control --reload-rules
sudo udevadm trigger

# Allow dialout for serial access
if ! groups "${USER}" | grep -q '\bdialout\b'; then
  sudo usermod -aG dialout "${USER}"
  echo "==> Added ${USER} to dialout. Log out/in for group membership to apply."
fi

echo "==> Installed ${RULES_FILE}"
echo "    After plugging ESP32-S3 USB, check: ls -l /dev/petcam_sensing* /dev/ttyACM* /dev/ttyUSB*"
