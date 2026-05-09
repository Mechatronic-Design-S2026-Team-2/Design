#!/usr/bin/env bash
set -euo pipefail

PATCH_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SERVICE_NAME="hexapod-ros2-full-launch.service"

sudo install -m 0755 "${PATCH_ROOT}/scripts/hexapod-ros2-full-launch.sh" /usr/local/bin/hexapod-ros2-full-launch.sh
if [ -f "${PATCH_ROOT}/scripts/hexapod-host-control.py" ]; then
  sudo install -m 0755 "${PATCH_ROOT}/scripts/hexapod-host-control.py" /usr/local/bin/hexapod-host-control.py
fi
sudo install -m 0644 "${PATCH_ROOT}/systemd/${SERVICE_NAME}" "/etc/systemd/system/${SERVICE_NAME}"
sudo install -m 0644 "${PATCH_ROOT}/systemd/hexapod-ros2-full-launch.default" /etc/default/hexapod-ros2-full-launch
if [ -f "${PATCH_ROOT}/systemd/hexapod-host-control.service" ]; then
  sudo install -m 0644 "${PATCH_ROOT}/systemd/hexapod-host-control.service" /etc/systemd/system/hexapod-host-control.service
  sudo install -m 0644 "${PATCH_ROOT}/systemd/hexapod-host-control.default" /etc/default/hexapod-host-control
  sudo systemctl enable hexapod-host-control.service
fi

sudo systemctl daemon-reload
sudo systemctl enable "${SERVICE_NAME}"
if [ -f /etc/systemd/system/hexapod-host-control.service ]; then
  sudo systemctl restart hexapod-host-control.service
fi

echo "Installed ${SERVICE_NAME}."
echo "Edit /etc/default/hexapod-ros2-full-launch if the container name is not 'jazzy'."
echo "Start now with: sudo systemctl start ${SERVICE_NAME}"
echo "View logs with: sudo journalctl -u ${SERVICE_NAME} -f"
