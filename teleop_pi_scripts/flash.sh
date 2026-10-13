#!/bin/sh
# Flash the RP2040 from the Pi: sh flash.sh [firmware.uf2]
# Reboots the board into BOOTSEL over USB (1200-baud touch), copies the UF2 and
# waits for the firmware to come back. Rebooting releases both clutches.
set -eu

UF2=${1:-$HOME/andre_rp2040_controller.uf2}
PORT=${PORT:-/dev/ttyACM0}
LABEL=/dev/disk/by-label/RPI-RP2

[ -f "$UF2" ] || { echo "firmware not found: $UF2" >&2; exit 1; }

wait_for() {  # wait_for <path> <seconds>
    i=0
    while [ ! -e "$1" ] && [ "$i" -lt $(($2 * 2)) ]; do sleep 0.5; i=$((i + 1)); done
    [ -e "$1" ]
}

if [ ! -e "$LABEL" ]; then
    [ -e "$PORT" ] || { echo "no RP2040 at $PORT and none in BOOTSEL" >&2; exit 1; }
    echo "rebooting RP2040 into BOOTSEL"
    stty -F "$PORT" 1200 || true
    wait_for "$LABEL" 15 || {
        echo "RP2040 didn't enter BOOTSEL; hold BOOTSEL while replugging it" >&2; exit 1; }
fi

DEV=$(readlink -f "$LABEL")
mount_point() { findmnt -n -o TARGET "$DEV" 2>/dev/null || true; }
MNT=$(mount_point)
if [ -z "$MNT" ]; then
    i=0  # the desktop may auto-mount it a moment later
    while [ -z "$MNT" ] && [ "$i" -lt 6 ]; do sleep 0.5; MNT=$(mount_point); i=$((i + 1)); done
fi
SUDO=""
if [ -z "$MNT" ]; then
    if udisksctl mount --no-user-interaction -b "$DEV" >/dev/null 2>&1; then
        MNT=$(mount_point)
    else
        sudo mkdir -p /mnt/rp2 && sudo mount "$DEV" /mnt/rp2
        MNT=/mnt/rp2
        SUDO=sudo
    fi
fi

echo "copying $(basename "$UF2") to $MNT"
$SUDO cp "$UF2" "$MNT/"
sync
wait_for "$PORT" 15 || { echo "firmware didn't come back on $PORT" >&2; exit 1; }
echo "flashed; RP2040 running again on $PORT (clutches released)"
