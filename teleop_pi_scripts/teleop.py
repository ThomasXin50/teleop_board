#!/usr/bin/env python3
"""Gamepad (or keyboard) teleoperation for the RP2040 vehicle controller.

    python3 teleop.py                    # drive with a game controller
    python3 teleop.py --input keyboard   # drive with the keyboard
    python3 teleop.py --monitor          # telemetry only, never sends commands
    python3 teleop.py --probe-gamepad    # print raw controller events
"""

from __future__ import annotations

import argparse
import sys
import time

import protocol
from controls import (BRAKE_CUTS_THROTTLE_PERCENT, MAX_BRAKE_PERCENT, Command, ControlMapper,
                      RumbleCues)
from link import MIN_SEND_HZ, VehicleLink, find_port

# If telemetry stops for this long the speed target is forced to zero (and the
# gamepad is disarmed) until the operator acts again.
TELEMETRY_LOST_S = 1.0

GAMEPAD_HELP = [
    "Right trigger: throttle   Left stick: steer   Left trigger: brake (overrides throttle)",
    "Start: arm (throttle released)   B/Circle: disarm   D-pad up/down: speed limit",
    "Keyboard: SPACE disarm   Q quit",
]
KEYBOARD_HELP = [
    "Up/W faster  Down/S slower  Left/A Right/D steer  C center",
    "B brake +10%  V brake -10%  SPACE stop (speed 0, full brake)  Q quit",
]


def format_telemetry(t: protocol.Telemetry) -> list[str]:
    def sensor(volts: float, ok: bool, axis: str, at_limit: bool, calibrated: bool) -> str:
        text = f"sensor {volts:5.3f} V " + ("ok" if ok else f"FAULT ({axis} held, throttle blocked)")
        if not calibrated:
            text += "   UNCALIBRATED (not driven, throttle blocked)"
        return text + ("   AT TRAVEL LIMIT" if at_limit else "")

    lines = [
        f"steering  {t.steering_deg:+7.2f} deg   "
        f"{sensor(t.steering_volts, t.steer_sensor_ok, 'steering', t.steer_at_limit, t.steer_calibrated)}",
        f"brake     {t.brake_percent:7.2f} %     "
        f"{sensor(t.brake_volts, t.brake_sensor_ok, 'brake', t.brake_at_limit, t.brake_calibrated)}",
        f"throttle  {t.throttle_percent:7.2f} %",
        f"rpm       FL {t.rpm_front_left:5d}  FR {t.rpm_front_right:5d}  "
        f"BL {t.rpm_back_left:5d}  BR {t.rpm_back_right:5d}",
        f"command   {'ACTIVE' if t.command_active else 'inactive'}"
        + (("   FAILSAFE BRAKING (" + ("requested by client" if t.failsafe_requested
                                          else "commands timed out") + ")")
           if t.failsafe_braking else ""),
    ]
    if t.bench_mode:
        lines.insert(0, "BENCH FIRMWARE: throttle interlocks off, steering open loop "
                        "(stick = motor speed). Wheels off the ground, not for driving.")
    if t.faults:
        lines.append("AXIS FAULT (reset the RP2040 to clear): " + ", ".join(t.faults)
                     + "   THROTTLE CUT")
    return lines


def run_monitor(link: VehicleLink) -> int:
    last_sequence = None
    try:
        while link.error is None:
            telemetry, age = link.latest_telemetry()
            if telemetry is not None and telemetry.sequence != last_sequence:
                last_sequence = telemetry.sequence
                print(f"#{telemetry.sequence:5d}  " + "  |  ".join(format_telemetry(telemetry)))
            elif age > TELEMETRY_LOST_S:
                print("waiting for telemetry (is DTR asserted / firmware running?)")
                time.sleep(TELEMETRY_LOST_S)
            time.sleep(0.05)
    except KeyboardInterrupt:
        pass
    return 0 if link.error is None else 1


class KeyboardInput:
    def __init__(self, args: argparse.Namespace) -> None:
        self.args = args
        self.steering = 0.0
        self.speed = 0.0
        self.brake = 0.0
        self.steer_sign = -1.0 if args.invert_steering else 1.0

    def update(self, key: int, curses) -> Command:
        args = self.args
        if key in (curses.KEY_UP, ord("w"), ord("W")):
            self.speed += args.speed_step
        elif key in (curses.KEY_DOWN, ord("s"), ord("S")):
            self.speed -= args.speed_step
        elif key in (curses.KEY_LEFT, ord("a"), ord("A")):
            self.steering += self.steer_sign * args.steer_step
        elif key in (curses.KEY_RIGHT, ord("d"), ord("D")):
            self.steering -= self.steer_sign * args.steer_step
        elif key in (ord("c"), ord("C")):
            self.steering = 0.0
        elif key in (ord("b"), ord("B")):
            self.brake += 10.0
        elif key in (ord("v"), ord("V")):
            self.brake -= 10.0
        elif key == ord(" "):
            self.speed = 0.0
            self.brake = MAX_BRAKE_PERCENT
        self.speed = max(0.0, min(args.max_speed, self.speed))
        self.steering = max(-args.max_steer, min(args.max_steer, self.steering))
        self.brake = max(0.0, min(MAX_BRAKE_PERCENT, self.brake))
        if self.brake > BRAKE_CUTS_THROTTLE_PERCENT:
            self.speed = 0.0
        return Command(self.steering, self.speed, self.brake)

    def stop(self) -> None:
        self.speed = 0.0

    def feedback(self, vehicle_ready: bool) -> None:
        pass

    def status(self) -> list[str]:
        return []


class GamepadInput:
    def __init__(self, args: argparse.Namespace, pad) -> None:
        self.pad = pad
        self.mapper = ControlMapper(args.max_speed, args.max_steer, args.speed_step,
                                    args.invert_steering)
        self.cues = RumbleCues()

    def update(self, key: int, curses) -> Command:
        if key == ord(" "):
            self.mapper.disarm()
        return self.mapper.update(self.pad.state())

    def stop(self) -> None:
        self.mapper.disarm()

    def feedback(self, vehicle_ready: bool) -> None:
        """Vibrates the controller on ready, arm and disarm."""
        for cue in self.cues.update(self.pad.state().connected, vehicle_ready,
                                    self.mapper.armed):
            self.pad.rumble(cue)

    def status(self) -> list[str]:
        state = self.pad.state()
        if not state.connected:
            return [f"CONTROLLER NOT CONNECTED ({self.pad.error or 'searching'}) - "
                    "failsafe braking requested"]
        armed = "ARMED" if self.mapper.armed else "disarmed - press Start to arm"
        return [f"controller {state.name}   {armed}   "
                f"speed limit {self.mapper.speed_limit:.2f} mph"]


def run_teleop(link: VehicleLink, args: argparse.Namespace, source) -> int:
    import curses  # not available on stock Windows Python; the Pi has it

    help_lines = GAMEPAD_HELP if args.input == "gamepad" else KEYBOARD_HELP

    def ui(screen) -> int:
        curses.curs_set(0)
        screen.timeout(50)

        def put(row: int, text: str, attr: int = 0) -> None:
            try:
                screen.addstr(row, 0, text, attr)
            except curses.error:
                pass  # terminal too small for this row

        while link.error is None:
            key = screen.getch()
            if key in (ord("q"), ord("Q"), 27):
                break

            telemetry, age = link.latest_telemetry()
            link_lost = age > TELEMETRY_LOST_S
            if link_lost:
                source.stop()
            command = source.update(key, curses)
            steering, brake = command.steering_deg, command.brake_percent
            speed = 0.0 if link_lost else command.speed_mph
            if command.failsafe:
                link.request_failsafe(protocol.FAILSAFE_REASON_CONTROLLER_LOST)
            else:
                link.set_command(steering, speed, brake)
            source.feedback(telemetry is not None and not link_lost
                            and telemetry.command_active)

            screen.erase()
            put(0, f"RP2040 teleop  {args.port}")
            for row, line in enumerate(help_lines, start=1):
                put(row, line)
            row = len(help_lines) + 2
            for line in source.status():
                put(row, line, curses.A_BOLD)
                row += 1
            if command.failsafe:
                put(row, "target    FAILSAFE (firmware cuts throttle, holds steering, "
                         "ramps brake to 100%)")
            else:
                put(row, f"target    steering {steering:+6.2f} deg   "
                         f"speed {speed:5.2f} mph (max {args.max_speed:g})   brake {brake:5.1f} %")
            row += 2
            if link_lost:
                put(row, "LINK LOST - no telemetry, speed held at 0", curses.A_REVERSE)
            row += 1
            if telemetry is not None:
                for line in format_telemetry(telemetry):
                    put(row, line)
                    row += 1
                if not link_lost and not telemetry.command_active:
                    put(row, "vehicle is not accepting commands yet", curses.A_BOLD)
            row += 2
            stats = link.stats
            counters = (f"sent {stats.commands_sent}  failsafe {stats.failsafe_sent}"
                        f"  frames ok {stats.good_frames}"
                        f"  bad {stats.bad_frames}  seq resyncs {stats.resyncs}")
            if telemetry is not None:
                counters += f"  telemetry age {age:4.1f}s"
            put(row, counters)
            screen.refresh()
        return 0 if link.error is None else 1

    return curses.wrapper(ui)


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--port", help="serial device (default: auto-detect RP2040, "
                                       "else /dev/ttyACM0)")
    parser.add_argument("--input", choices=("gamepad", "keyboard"), default="gamepad")
    parser.add_argument("--gamepad", help="evdev device, e.g. /dev/input/event3 "
                                          "(default: first controller found)")
    parser.add_argument("--probe-gamepad", action="store_true",
                        help="print raw controller events and exit")
    parser.add_argument("--monitor", action="store_true",
                        help="print telemetry only; never send commands")
    parser.add_argument("--rate", type=float, default=20.0,
                        help=f"command rate in Hz (min {MIN_SEND_HZ:g}, default 20)")
    parser.add_argument("--max-speed", type=float, default=2.0,
                        help="operator speed ceiling in mph "
                             f"(firmware cap {protocol.SPEED_MAX_MPH:g}, default 2)")
    parser.add_argument("--max-steer", type=float, default=protocol.STEER_ANGLE_MAX_DEG,
                        help="steering limit in degrees (default 65)")
    parser.add_argument("--speed-step", type=float, default=0.25,
                        help="mph per key press / D-pad press (default 0.25)")
    parser.add_argument("--steer-step", type=float, default=2.0,
                        help="keyboard degrees per key press (default 2)")
    parser.add_argument("--invert-steering", action="store_true",
                        help="swap left/right if the vehicle steers backwards")
    args = parser.parse_args(argv)

    args.max_speed = max(0.0, min(protocol.SPEED_MAX_MPH, args.max_speed))
    args.max_steer = max(0.0, min(protocol.STEER_ANGLE_MAX_DEG, args.max_steer))
    if args.port is None:
        args.port = find_port() or "/dev/ttyACM0"
    return args


def main(argv: list[str]) -> int:
    args = parse_args(argv)
    if args.probe_gamepad:
        import gamepad
        gamepad.probe(args.gamepad)
        return 0
    try:
        with VehicleLink(args.port, send_hz=args.rate,
                         send_commands=not args.monitor) as link:
            if args.monitor:
                result = run_monitor(link)
            elif args.input == "keyboard":
                result = run_teleop(link, args, KeyboardInput(args))
            else:
                import gamepad
                with gamepad.Gamepad(args.gamepad) as pad:
                    result = run_teleop(link, args, GamepadInput(args, pad))
            if link.error is not None:
                print(f"serial link failed: {link.error}", file=sys.stderr)
            return result
    except (OSError, ValueError) as error:
        print(f"error: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
