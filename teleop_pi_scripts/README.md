# Teleop Pi

Keyboard teleoperation client for the RP2040 vehicle controller
(`Andre_Code_For_Thoma`). Speaks the USB CDC protocol described in that
project's README.

## Setup (Raspberry Pi OS)

```sh
sh setup.sh            # venv, requirements, dialout/input groups, unit tests
. .venv/bin/activate
```

If it adds you to a group, log out and back in before using the serial port.

## Flashing the RP2040 from the Pi

Copy the UF2 over (`scp build/andre_rp2040_controller.uf2 cev@<pi>:~/`), then:

```sh
sh flash.sh            # or: sh flash.sh path/to/firmware.uf2
```

It reboots the RP2040 into BOOTSEL over USB, copies the file and waits for it
to restart. Rebooting releases both clutches.

## Bench checks

```sh
python3 bench.py record --seconds 60   # per-second sensor/RPM summary, listen-only
python3 bench.py link                  # commands, failsafe request, timeout failsafe
python3 bench.py hold                  # engage clutches, hold steering 6 s
python3 bench.py steer --delta 5       # steer 5 deg left of where it is, then back
```

### Bench-only drive (interlocks off)

To check the throttle output with a multimeter and watch the steering motor
turn before the brake and steering are mechanically complete, copy both UF2s
from the firmware `build/` folder to `~` and run:

```sh
sh bench_drive.sh      # asks you to type BENCH, then starts teleop.py
```

It flashes `andre_rp2040_controller_bench.uf2`: throttle ignores calibration,
sensor and fault interlocks (brake above 5% and the comms timeout still cut
it), and the steering motor runs open loop: stick deflection sets its speed and
direction, up to `BENCH_STEER_MAX_STEPS_PER_S` (500 steps/s, set in the
firmware's `config.h`) at full stick; centered stops it. No travel limits. Teleop starts with
a 10 mph ceiling and 1 mph D-pad steps: arm with Start, raise the limit with
D-pad up, and the right trigger sets PWM duty (GPIO2/3 duty = speed / 10 mph).
The screen shows `BENCH FIRMWARE`. Quitting (Q, Ctrl+C or an SSH drop)
reflashes the normal firmware. Wheels off the ground, steering motor
uncoupled, e-stop within reach; never drive on it.

Each bench check prints PASS/FAIL lines. `link`, `hold` and `steer` engage the clutches
until the RP2040 is reset, and command the angle the wheel is already at (never
0), so a calibrated steering axis never swings to center on its own. `steer`
asks for confirmation; `--yes` skips it.

## Run

```sh
python3 teleop.py --monitor          # telemetry only, sends nothing (clutches stay released)
python3 teleop.py --probe-gamepad    # print raw controller events to check mapping
python3 teleop.py                    # drive with the controller
python3 teleop.py --input keyboard   # drive with the keyboard instead
```

To watch telemetry live over SSH (listen-only, like `ros2 topic echo`; don't
run it alongside `teleop.py`):

```sh
python3 telemetry.py          # one readable line per message
python3 telemetry.py --json   # JSON lines, e.g. | tee run.jsonl
python3 telemetry.py --hz     # message rate only
```

The serial port and controller are auto-detected; override with `--port` and
`--gamepad /dev/input/eventN`. Add your user to the `input` group if the
controller isn't found (`sudo usermod -aG input "$USER"`).

## Calibration

`--monitor` sends no commands, so the RP2040 never engages its clutches and the
steering wheel and brake pedal can be moved by hand while it shows each raw
sensor voltage. Enter the readings in the CALIBRATION block of the firmware's
`src/config.h`; the firmware README walks through it. Until an axis is marked
calibrated the screen shows `UNCALIBRATED (not driven)` and that stepper never
moves. A latched overtravel or no-progress fault shows as `AXIS FAULT`; reset
the RP2040 to clear it.

## Controller mapping

Names are by position, so this works for Xbox, PlayStation, 8BitDo (X-input
mode) and Switch Pro controllers.

| Control | Action |
| --- | --- |
| Right trigger (RT / R2 / ZR) | Throttle: 0 to the current speed limit. Release to coast |
| Left stick, left/right | Steering angle, proportional; centers when released. Left = +deg |
| Left trigger (LT / L2 / ZL) | Brake, proportional 0–100%. Works even when disarmed; any brake above 5% cuts throttle |
| Start / Options / + | Arm. Only works with the right trigger released |
| Right face button (B / Circle / Nintendo A) | Disarm: speed forced to 0 until re-armed |
| D-pad up / down | Raise / lower speed limit by 0.25 mph (starts at 1 mph, max `--max-speed`) |
| Keyboard SPACE / Q | Disarm / quit |

Use `--invert-steering` if the wheels turn the wrong way. The client starts disarmed. It disarms automatically if the controller
disconnects or telemetry is lost for 1 s. Controllers whose ABS_Z/ABS_RZ axes
rest mid-range (a second stick, not triggers) are refused so a stick can't
command throttle.

If the controller disconnects, the client disarms and sends the firmware failsafe
requests instead of commands. The firmware then does exactly what it does when
comms drop: cuts throttle, holds steering, and ramps the brake to 100% over 1 s.
When the controller reconnects, normal commands resume (re-arm with Start). With
no controller connected at startup, this applies the brake right away.

Keyboard mode: Up/W and Down/S change the target speed, Left/A and Right/D
change the steering target, C centers, B/V raise/lower brake by 10%, SPACE sets
speed 0 and full brake, Q quits.

## Safety behavior

- Commands stream at 20 Hz (`--rate`). If none arrive for 500 ms the firmware
  cuts throttle, holds steering, keeps both clutches engaged and ramps the brake
  to 100% over 1 s. That includes quitting this program.
- Quitting sends zero-speed commands before closing the port.
- Speed is capped at 2 mph by default (`--max-speed`, firmware cap 10 mph).
- If telemetry stops for 1 s the speed target is forced to 0 and must be raised
  again after the link recovers.
- Keep a hardware e-stop within reach: an RP2040 reset or loss of its power
  releases both clutches, and with them the brake.

## Notes

- The port is opened at 115200 with DTR asserted. Never open it at 1200 baud:
  that reboots the RP2040 into BOOTSEL.
- The firmware ignores commands whose sequence number isn't newer than the last
  one it accepted, and remembers that across client restarts. The client detects
  this from the telemetry "command active" bit and jumps its sequence forward
  (shown as `seq resyncs`), recovering within about 3 s.

## Tests

```sh
python3 -m unittest -v test_protocol test_controls test_telemetry test_bench
```
