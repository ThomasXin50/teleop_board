# RP2040 Vehicle Controller

## Build

Install the Raspberry Pi Pico SDK and its ARM GNU toolchain, then set `PICO_SDK_PATH` to the SDK directory. From PowerShell:

```powershell
cmake -S . -B build -G Ninja
cmake --build build
```

The UF2 file is generated at `build/andre_rp2040_controller.uf2`. Hold BOOTSEL while connecting the board, then copy the UF2 to the mounted RP2040 drive.

The build also produces `build/andre_rp2040_controller_bench.uf2` (`BENCH_MODE=1`), for bench checks only: throttle follows the command with only the brake interlock and the comms timeout left (no calibration, sensor or fault interlocks, and no speed PID), and steering runs open loop: the commanded angle sets the step rate, from 0 at center to `BENCH_STEER_MAX_STEPS_PER_S` (500 steps/s; it may exceed `STEPPER_MAX_HZ`) at full travel, with no sensor feedback or travel limits. Telemetry status bit 14 is set while it runs. Use it only with the drive wheels off the ground and the steering motor uncoupled; never drive on it. On the Pi, `teleop-pi/bench_drive.sh` flashes it and restores the normal firmware on exit.

## USB CDC Protocol

All multi-byte values are little-endian. Each CDC stream frame is:

| Offset | Size | Field |
| --- | ---: | --- |
| 0 | 2 | Magic `A5 5A` |
| 2 | 1 | Version, currently `3` |
| 3 | 1 | Type: command `1`, telemetry `2`, failsafe request `3`, clutch `4` (bench firmware only) |
| 4 | 1 | Payload byte length |
| 5 | 2 | Sequence number |
| 7 | N | Payload |
| 7 + N | 4 | CRC-32/IEEE over bytes 2 through 6 + payload |

Command payload (type 1, 6 bytes): signed `int16` steering-wheel angle in centidegrees, unsigned `uint16` target speed in centi-miles-per-hour, then unsigned `uint16` brake target in hundredths of a percent (0 released, 10000 fully applied). Failsafe request payload (type 3, 1 byte): `uint8` reason, currently `1` for operator controller lost; the firmware treats every reason the same. Clutch payload (type 4, 1 byte): `uint8` `0` release, `1` engage; only the bench firmware acts on it (the vehicle firmware ignores it), it applies once a command has engaged the clutches, and it neither engages them by itself nor keeps the link alive. Commands, failsafe requests and clutch messages share one sequence counter.

Send a message periodically; the link expires 500 ms after the last one. Both clutches are released at power-up and engage on the first valid message; after that they stay engaged until reset. The firmware enters **failsafe braking** when the link expires or while the host sends failsafe requests: throttle returns to zero, steering holds its position, and the brake target ramps from its current value to 100% over 1 s (`BRAKE_FAILSAFE_PERCENT`, `BRAKE_FAILSAFE_RAMP_US`). Normal control resumes with the next command. A brake target above 5% forces throttle to zero.

Telemetry is sent at 5 Hz (type 2, 20 bytes): signed steering angle in centidegrees; measured brake in hundredths of a percent (below 0 reads as 0); front-left, front-right, back-right, and back-left RPM as unsigned integer RPM; throttle duty in hundredths of a percent; a `uint16` status bitfield; then the raw steering and brake sensor voltages in millivolts. The steering angle is reported unclamped.

| Status bit | Meaning |
| ---: | --- |
| 0 | Command active |
| 1 | Steering sensor reading valid (not near 0 V or 3.3 V, which indicates a disconnected or shorted sensor; throttle is blocked while clear) |
| 2 | Brake sensor reading valid (same rule; throttle is blocked while clear) |
| 3 | Steering at or past its calibrated travel |
| 4 | Brake at or past 0% or 100% |
| 5 | Failsafe braking (commands timed out) |
| 6 | Steering overtravel fault (latched) |
| 7 | Steering no-progress fault (latched) |
| 8 | Brake overtravel fault (latched) |
| 9 | Brake no-progress fault (latched) |
| 10 | Steering uncalibrated (`STEER_CALIBRATED` is 0; never stepped, and throttle is blocked) |
| 11 | Brake uncalibrated (`BRAKE_CALIBRATED` is 0; never stepped, and throttle is blocked) |
| 12 | Host link active (a valid command or failsafe request within 500 ms) |
| 13 | Failsafe requested by the host (latest message was type 3) |
| 14 | Bench firmware (`BENCH_MODE`): throttle interlocks off, steering open loop, clutch messages honored |
| 15 | Clutches engaged (outputs driven to engage) |

The USB CDC link handles USB packet error detection and retry. The application frame adds stream resynchronization and CRC validation. Invalid frames are discarded.

## Hardware Defaults To Verify

The values in `src/config.h` are starting assumptions, not calibrated vehicle parameters (see Calibration below). Confirm ADC wiring and sensor pin voltages, encoder pulses per revolution, steering travel and polarity, motor step angle and DM542T microstep switches, enable-input polarity, and the filtered throttle input range before enabling the mechanism. RP2040 ADC pins must never exceed 3.3 V. GPIO2 and GPIO3 output the same 25 kHz PWM duty for the RC-filtered, forward-only speed input.

The DM542T inputs are wired common-anode: PUL+, DIR+, and ENA+ connect to +5 V, and GPIO10–15 drive PUL-, DIR-, and ENA- through U7, a non-inverting SN74AHCT244 buffer powered from +5 V, so each optocoupler conducts while its GPIO is low (`STEPPER_INPUTS_COMMON_ANODE` in `src/config.h`). Set each DM542T's logic-level switch (S2 on drivers that have one) to 5 V. PUL idles high and steps on the low half of each period; ENA- is driven high to enable a driver and low to disable it. Both drivers are enabled whenever the firmware runs, so the motors hold position when not stepping. While the RP2040 is in reset or BOOTSEL its pins are pulled low, so the buffer drives every input on and both drivers are disabled (no holding torque, like the released clutches). Reversing the DIR polarity flips the steering direction; verify it before closing the steering loop.

The clutch solenoids on GPIO4 and GPIO5 are driven through low-side MOSFETs (high engages) and are the only thing holding the steering wheel and brake pedal in position. They are released at power-up and during reset (including a watchdog reset), and engaged from the first valid command onward.

Steering and brake are both closed-loop on their analog sensors, with the same protections:

- **Soft limit:** at or past the calibrated travel, the axis only steps back inside it.
- **Invalid reading** (near 0 V or 3.3 V): the axis holds still until the reading recovers.
- **Overtravel fault:** stepped past its travel by more than `*_OVERTRAVEL_*`. Latched; the axis holds until the RP2040 is reset.
- **No-progress fault:** after stepping the equivalent of `*_PROGRESS_CHECK_*` in one direction, the reading has not moved at least `*_PROGRESS_MIN_*` that way. Catches a reversed step sign, a slipped or stuck sensor, and a motor stalled against a mechanical stop. Latched like overtravel.

A faulted axis stays held by its clutch. Throttle is forced to zero while the brake or steering cannot be relied on: a latched fault on either axis (until reset), an invalid reading from either sensor (until it recovers), or either axis uncalibrated (`STEER_CALIBRATED` or `BRAKE_CALIBRATED` 0). The front RPM readings are captured and reported; steering position control uses the analog steering sensor.

## Calibration

Every value to measure is in the **CALIBRATION** block at the top of `src/config.h`. `STEER_CALIBRATED` and `BRAKE_CALIBRATED` start at `0`. While an axis is uncalibrated its stepper never moves, so placeholder values cannot drive the wheel or pedal into a stop, and throttle stays at zero until both are calibrated.

1. Flash the firmware and run `python3 teleop.py --monitor` on the Pi. Monitor mode sends no commands, so both clutches stay released and the wheel and pedal can be moved by hand. Each line shows the raw sensor voltage.
2. Steering: turn to each end of the travel you want to allow. Enter those angles as `STEER_ANGLE_MIN_DEG`/`MAX_DEG` and the voltages there as `STEER_SENSOR_V_AT_MIN`/`AT_MAX`.
3. Brake: note the voltage with the pedal released (`BRAKE_SENSOR_V_RELEASED`) and at the furthest press the stepper should ever reach, short of the mechanical stop (`BRAKE_SENSOR_V_FULL`). The failsafe drives to this point unattended. Enter the stepper output-shaft rotation between them as `BRAKE_TRAVEL_DEG`.
4. Either voltage of a pair may be the higher one. Confirm the microstep, full-step, and gear-ratio values in the same block.
5. Lower `STEPPER_MAX_HZ` (e.g. to 300), set the `*_CALIBRATED` flags to `1`, rebuild, and command small moves. If an axis heads away from its target it will stop with a no-progress or overtravel fault; flip its `*_STEP_SIGN`, reset, and try again.
6. Check that full brake travel (`BRAKE_STEPS_PER_DEG * BRAKE_TRAVEL_DEG` steps) fits within `STEPPER_MAX_HZ * 1 s`, or the failsafe ramp will take longer than 1 s. Then restore `STEPPER_MAX_HZ`.