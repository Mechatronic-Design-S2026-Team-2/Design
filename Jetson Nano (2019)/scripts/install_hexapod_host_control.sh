#!/usr/bin/env bash
set -euo pipefail
PATCH_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
sudo install -m 0755 "${PATCH_ROOT}/scripts/hexapod-host-control.py" /usr/local/bin/hexapod-host-control.py
sudo install -m 0644 "${PATCH_ROOT}/systemd/hexapod-host-control.service" /etc/systemd/system/hexapod-host-control.service
sudo install -m 0644 "${PATCH_ROOT}/systemd/hexapod-host-control.default" /etc/default/hexapod-host-control
sudo systemctl daemon-reload
sudo systemctl enable hexapod-host-control.service
sudo systemctl restart hexapod-host-control.service
echo "Installed and started hexapod-host-control.service"
echo "Test: curl http://127.0.0.1:18080/status"
