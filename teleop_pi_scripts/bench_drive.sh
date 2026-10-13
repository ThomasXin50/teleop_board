#!/bin/sh
# BENCH ONLY: drive the throttle output and the steering motor from the
# controller with the safety interlocks off, then put the normal firmware back.
#
#   sh bench_drive.sh                    # controller
#   sh bench_drive.sh --input keyboard   # extra arguments go to teleop.py
#
# Flashes andre_rp2040_controller_bench.uf2, whose throttle ignores calibration,
# sensor and fault interlocks (brake > 5% and the comms timeout still cut it),
# and whose steering runs open loop: stick deflection sets the motor's speed
# (up to BENCH_STEER_MAX_STEPS_PER_S) and direction, centered stops it, no
# travel limits. Wheels off the ground and the steering motor uncoupled. Quitting restores andre_rp2040_controller.uf2.
set -eu
cd "$(dirname "$0")"

BENCH_UF2=${BENCH_UF2:-$HOME/andre_rp2040_controller_bench.uf2}
NORMAL_UF2=${NORMAL_UF2:-$HOME/andre_rp2040_controller.uf2}
for f in "$BENCH_UF2" "$NORMAL_UF2"; do
    [ -f "$f" ] || { echo "missing $f (copy it from the firmware build/ folder)" >&2; exit 1; }
done

cat <<'EOF'
BENCH MODE: throttle interlocks OFF, steering motor open loop with NO limits.
  - drive wheels off the ground, nobody near them
  - steering motor NOT coupled to the column (it would drive into the stops)
  - e-stop within reach
EOF
printf 'Type BENCH to continue: '
read -r answer
[ "$answer" = BENCH ] || { echo "cancelled"; exit 1; }

LOG=$HOME/bench_drive_restore.log
restore() {
    # Runs on quit, Ctrl+C and SSH hang-up; the terminal may be gone, so log to
    # a file and never let a failed echo stop the reflash.
    trap - EXIT INT TERM HUP
    set +e
    if sh flash.sh "$NORMAL_UF2" >"$LOG" 2>&1; then
        echo "normal firmware restored"
    else
        echo "RESTORE FAILED (see $LOG): run  sh flash.sh $NORMAL_UF2" >&2
    fi
}

sh flash.sh "$BENCH_UF2"
trap restore EXIT
trap 'exit 130' INT TERM HUP

. .venv/bin/activate
python3 teleop.py --max-speed 10 --speed-step 1 "$@" || true
