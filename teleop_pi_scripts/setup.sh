#!/bin/sh
# One-time setup on a freshly flashed Pi: sh setup.sh
set -eu
cd "$(dirname "$0")"

python3 -m venv .venv
.venv/bin/pip install --disable-pip-version-check -r requirements.txt
chmod +x teleop.py telemetry.py bench.py flash.sh bench_drive.sh

missing=""
for group in dialout input; do
    id -nG "$USER" | tr ' ' '\n' | grep -qx "$group" || missing="$missing,$group"
done
if [ -n "$missing" ]; then
    sudo usermod -aG "${missing#,}" "$USER"
    echo "added $USER to ${missing#,}: log out and back in before using the serial port"
fi

.venv/bin/python -m unittest test_protocol test_controls test_telemetry test_bench
echo "setup done. Activate with: . .venv/bin/activate"
