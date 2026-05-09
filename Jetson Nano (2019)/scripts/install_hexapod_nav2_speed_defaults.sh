#!/usr/bin/env bash
set -euo pipefail
DEFAULT_FILE="${1:-/etc/default/hexapod-ros2-full-launch}"
sudo touch "${DEFAULT_FILE}"
set_kv() {
  local key="$1" value="$2"
  if sudo grep -qE "^${key}=" "${DEFAULT_FILE}"; then
    sudo sed -i "s|^${key}=.*|${key}=${value}|" "${DEFAULT_FILE}"
  else
    echo "${key}=${value}" | sudo tee -a "${DEFAULT_FILE}" >/dev/null
  fi
}
set_kv HEXAPOD_OPERATOR_KEYBOARD_LINEAR_MPS 0.50
set_kv HEXAPOD_OPERATOR_KEYBOARD_ANGULAR_RADPS 0.50
set_kv HEXAPOD_OPERATOR_MAX_LINEAR_MPS 3.00
set_kv HEXAPOD_OPERATOR_MAX_ANGULAR_RADPS 3.00
set_kv HEXAPOD_OPERATOR_WAYPOINT_MAX_LINEAR_MPS 0.50
set_kv HEXAPOD_OPERATOR_WAYPOINT_MAX_ANGULAR_RADPS 0.50
# Keep mapping load at the known-stable decimation used with the scan gate.
set_kv HEXAPOD_ORBSLAM_SCAN_PUBLISH_DECIMATION 3

echo "Updated ${DEFAULT_FILE}. Restart with: sudo systemctl restart hexapod-ros2-full-launch.service"
