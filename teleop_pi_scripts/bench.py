#!/usr/bin/env python3
"""Bench checks for the RP2040 link and actuators, runnable over SSH.

    python3 bench.py record [--seconds 60]   # sensor/RPM summary, listen-only
    python3 bench.py link                    # commands, both failsafes, recovery
    python3 bench.py hold [--seconds 6]      # engage the clutches, hold steering
    python3 bench.py steer --delta 5         # small steering move, then back

`record` never sends anything. `link`, `hold` and `steer` send commands, which
engage both clutches until the RP2040 is reset (power cycle, or flash.sh).
They always command the steering angle the wheel is already at (plus --delta
for `steer`), never 0, so a calibrated steering axis never swings to center.
Don't run this alongside teleop.py or telemetry.py; they share the serial port.
"""

from __future__ import annotations

import argparse
import statistics
import sys
import time
from typing import Callable, Optional

import protocol
from link import VehicleLink, find_port

FRESH_S = 0.5            # telemetry older than this is ignored
ACTIVE_TIMEOUT_S = 4.0   # includes link.py's sequence resync after a restart
FAILSAFE_TIMEOUT_S = 1.5  # firmware times out 0.5 s after the last command
STEER_TOLERANCE_DEG = 1.5
STEER_LIMIT_MARGIN_DEG = 2.0


class Checks:
    def __init__(self) -> None:
        self.failed = 0

    def check(self, ok: bool, text: str) -> bool:
        print(f"  [{'PASS' if ok else 'FAIL'}] {text}", flush=True)
        if not ok:
            self.failed += 1
        return ok

    def result(self) -> int:
        print("ALL CHECKS PASSED" if self.failed == 0 else f"{self.failed} CHECK(S) FAILED")
        return 0 if self.failed == 0 else 1


def wait_for(link: VehicleLink, predicate: Callable[[protocol.Telemetry], bool],
             timeout: float) -> Optional[protocol.Telemetry]:
    """Returns the first fresh telemetry matching predicate, or None on timeout."""
    end = time.monotonic() + timeout
    while time.monotonic() < end and link.error is None:
        telemetry, age = link.latest_telemetry()
        if telemetry is not None and age < FRESH_S and predicate(telemetry):
            return telemetry
        time.sleep(0.02)
    return None


def current(link: VehicleLink) -> protocol.Telemetry:
    telemetry = wait_for(link, lambda t: True, 2.0)
    if telemetry is None:
        raise RuntimeError("no telemetry from the RP2040 (is it running?)")
    return telemetry


def describe(t: protocol.Telemetry) -> str:
    return (f"steer {t.steering_deg:+6.1f} deg ({t.steering_volts:.3f} V)  "
            f"brake {t.brake_percent:5.1f}%  throttle {t.throttle_percent:.1f}%  "
            f"cmd={t.command_active} failsafe={t.failsafe_braking} "
            f"faults={','.join(t.faults) or 'none'}")


def steer_target(start: float, delta: float, low: float = protocol.STEER_ANGLE_MIN_DEG,
                 high: float = protocol.STEER_ANGLE_MAX_DEG,
                 margin: float = STEER_LIMIT_MARGIN_DEG) -> Optional[float]:
    """start + delta, or None if that lands within margin of a travel limit."""
    target = start + delta
    if not low + margin <= target <= high - margin:
        return None
    return target


def summarize(rows: list[tuple], bucket_s: float = 1.0) -> list[str]:
    """rows: (seconds, telemetry). One line per bucket: means and ranges."""
    buckets: dict[int, list[protocol.Telemetry]] = {}
    for seconds, telemetry in rows:
        buckets.setdefault(int(seconds // bucket_s), []).append(telemetry)
    lines = []
    for index in sorted(buckets):
        ts = buckets[index]
        steer_v = [t.steering_volts for t in ts]
        brake_v = [t.brake_volts for t in ts]
        lines.append(
            f"{index * bucket_s:5.0f}s  "
            f"steer {statistics.mean(t.steering_deg for t in ts):+6.1f} deg "
            f"{statistics.mean(steer_v):.3f} V [{min(steer_v):.3f}..{max(steer_v):.3f}]  "
            f"brake {statistics.mean(t.brake_percent for t in ts):5.1f}% "
            f"{statistics.mean(brake_v):.3f} V [{min(brake_v):.3f}..{max(brake_v):.3f}]  "
            f"rpm BL {max(t.rpm_back_left for t in ts):4d} BR {max(t.rpm_back_right for t in ts):4d} "
            f"FL {max(t.rpm_front_left for t in ts):4d} FR {max(t.rpm_front_right for t in ts):4d}")
    return lines


def steering_problem(t: protocol.Telemetry) -> Optional[str]:
    """Why commanding "hold where it is" would move a calibrated steering axis.

    The firmware clamps targets to the travel, so holding an angle outside it
    (or one from an invalid reading) would drive the motor toward the limit.
    """
    if not t.steer_calibrated:
        return None  # an uncalibrated axis never steps
    if not t.steer_sensor_ok:
        return f"steering sensor reading invalid ({t.steering_volts:.3f} V)"
    if not protocol.STEER_ANGLE_MIN_DEG <= t.steering_deg <= protocol.STEER_ANGLE_MAX_DEG:
        return (f"steering reads {t.steering_deg:+.1f} deg ({t.steering_volts:.3f} V), outside "
                f"the {protocol.STEER_ANGLE_MIN_DEG:+g}..{protocol.STEER_ANGLE_MAX_DEG:+g} deg "
                "travel; check the sensor mounting, wiring and ground, or recalibrate")
    return None


def require_idle(port: str) -> protocol.Telemetry:
    with VehicleLink(port, send_commands=False) as link:
        telemetry = current(link)
    if telemetry.host_link_active:
        raise RuntimeError("another program is already sending commands (teleop.py?)")
    if telemetry.faults:
        raise RuntimeError(f"faults already latched ({', '.join(telemetry.faults)}); "
                           "reset the RP2040 first (sh flash.sh)")
    problem = steering_problem(telemetry)
    if problem:
        raise RuntimeError(f"not sending commands: {problem}")
    return telemetry


def run_record(port: str, seconds: float) -> int:
    rows = []
    with VehicleLink(port, send_commands=False) as link:
        print(f"recording {seconds:g} s (listen-only)...", flush=True)
        start = time.monotonic()
        last = None
        while time.monotonic() - start < seconds and link.error is None:
            telemetry, _ = link.latest_telemetry()
            if telemetry is not None and telemetry.sequence != last:
                last = telemetry.sequence
                rows.append((time.monotonic() - start, telemetry))
            time.sleep(0.02)
    if not rows:
        print("no telemetry received")
        return 1
    print("\n".join(summarize(rows)))
    return 0


def run_link(port: str) -> int:
    checks = Checks()
    before = require_idle(port)
    hold = before.steering_deg
    print(f"before: {describe(before)}")
    print(f"sending commands: steering {hold:+.1f} deg (where it is), speed 0, brake 0")
    with VehicleLink(port, send_hz=20) as link:
        link.set_command(hold, 0.0, 0.0)
        checks.check(wait_for(link, lambda t: t.command_active, ACTIVE_TIMEOUT_S) is not None,
                     "firmware accepts commands")
        end = time.monotonic() + 2.0
        steady = True
        while time.monotonic() < end:
            telemetry, age = link.latest_telemetry()
            if age < FRESH_S and not telemetry.command_active:
                steady = False
            time.sleep(0.05)
        checks.check(steady, "stays active for 2 s")
        telemetry = current(link)
        checks.check(telemetry.throttle_percent == 0.0, "throttle stays at 0")
        checks.check(not telemetry.faults, f"no axis faults ({describe(telemetry)})")

        link.request_failsafe(protocol.FAILSAFE_REASON_CONTROLLER_LOST)
        checks.check(wait_for(link, lambda t: t.failsafe_requested and t.failsafe_braking,
                              FAILSAFE_TIMEOUT_S) is not None,
                     "requested failsafe starts failsafe braking")
        link.set_command(hold, 0.0, 0.0)
        checks.check(wait_for(link, lambda t: t.command_active and not t.failsafe_braking,
                              FAILSAFE_TIMEOUT_S) is not None,
                     "normal control resumes after failsafe request")
        checks.check(link.stats.bad_frames == 0, f"no corrupted frames ({link.stats})")
    with VehicleLink(port, send_commands=False) as link:
        checks.check(wait_for(link, lambda t: t.failsafe_braking and not t.host_link_active,
                              FAILSAFE_TIMEOUT_S) is not None,
                     "commands stopping starts failsafe braking")
    print("clutches stay engaged until the RP2040 is reset")
    return checks.result()


def run_hold(port: str, seconds: float) -> int:
    checks = Checks()
    before = require_idle(port)
    hold = before.steering_deg
    print(f"holding steering at {hold:+.1f} deg for {seconds:g} s; clutches engage now")
    with VehicleLink(port, send_hz=20) as link:
        link.set_command(hold, 0.0, 0.0)
        ok = checks.check(wait_for(link, lambda t: t.command_active, ACTIVE_TIMEOUT_S) is not None,
                          "firmware accepts commands (clutches engaged)")
        end = time.monotonic() + seconds
        while ok and time.monotonic() < end:
            time.sleep(1.0)
            telemetry = current(link)
            print(f"    {describe(telemetry)}", flush=True)
            if telemetry.faults:
                break
        telemetry = current(link)
        checks.check(not telemetry.faults, "no axis faults")
        checks.check(abs(telemetry.steering_deg - hold) <= STEER_TOLERANCE_DEG,
                     f"steering held within {STEER_TOLERANCE_DEG} deg")
    print("clutches stay engaged until the RP2040 is reset")
    return checks.result()


def drive_to(link: VehicleLink, target: float, seconds: float) -> protocol.Telemetry:
    link.set_command(target, 0.0, 0.0)
    end = time.monotonic() + seconds
    telemetry = current(link)
    while time.monotonic() < end:
        time.sleep(0.5)
        telemetry = current(link)
        print(f"    target {target:+6.1f}  {describe(telemetry)}", flush=True)
        if telemetry.faults:
            break
    return telemetry


def run_steer(port: str, delta: float, seconds: float, assume_yes: bool) -> int:
    checks = Checks()
    before = require_idle(port)
    print(f"before: {describe(before)}")
    if not before.steer_calibrated:
        print("steering must be calibrated")
        return 1
    start = before.steering_deg
    target = steer_target(start, delta)
    if target is None:
        print(f"{start:+.1f} {delta:+g} deg is too close to the travel limit; move the wheel")
        return 1
    print(f"plan: hold {start:+.1f} deg, steer to {target:+.1f} deg for {seconds:g} s, "
          f"back to {start:+.1f} deg for {seconds:g} s")
    if not assume_yes:
        try:
            answer = input("The steering motor will move. Hands clear, e-stop ready? [type yes] ")
        except EOFError:
            answer = ""
        if answer.strip().lower() != "yes":
            print("cancelled")
            return 1
    with VehicleLink(port, send_hz=20) as link:
        link.set_command(start, 0.0, 0.0)
        if not checks.check(wait_for(link, lambda t: t.command_active, ACTIVE_TIMEOUT_S)
                            is not None, "firmware accepts commands"):
            return checks.result()
        time.sleep(1.0)
        reached = drive_to(link, target, seconds)
        moved = reached.steering_deg - start
        checks.check(not reached.faults, "no faults while steering out")
        checks.check(moved * delta > 0,
                     f"moved the commanded direction ({moved:+.1f} deg); if it went the "
                     "other way, flip STEER_STEP_SIGN")
        checks.check(abs(reached.steering_deg - target) <= STEER_TOLERANCE_DEG,
                     f"reached {target:+.1f} deg within {STEER_TOLERANCE_DEG} deg")
        if reached.faults:
            print("  a latched fault freezes steering until reset. 'no progress' usually means "
                  "the motor isn't coupled to the column, or STEER_STEP_SIGN is reversed")
            return checks.result()
        back = drive_to(link, start, seconds)
        checks.check(not back.faults, "no faults while steering back")
        checks.check(abs(back.steering_deg - start) <= STEER_TOLERANCE_DEG,
                     f"returned to {start:+.1f} deg within {STEER_TOLERANCE_DEG} deg")
    print("commands stopped: steering holds, failsafe brakes, clutches stay engaged until reset")
    return checks.result()


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--port", help="serial device (default: auto-detect, else /dev/ttyACM0)")
    sub = parser.add_subparsers(dest="test", required=True)
    record = sub.add_parser("record", help="summarize sensors and RPM (listen-only)")
    record.add_argument("--seconds", type=float, default=60.0)
    sub.add_parser("link", help="commands, failsafe request, timeout failsafe")
    hold = sub.add_parser("hold", help="engage clutches and hold steering")
    hold.add_argument("--seconds", type=float, default=6.0)
    steer = sub.add_parser("steer", help="small steering move and back")
    steer.add_argument("--delta", type=float, default=5.0, help="degrees; + is left")
    steer.add_argument("--seconds", type=float, default=3.0, help="time at each position")
    steer.add_argument("--yes", action="store_true", help="skip the confirmation prompt")
    args = parser.parse_args(argv)
    port = args.port or find_port() or "/dev/ttyACM0"
    try:
        if args.test == "record":
            return run_record(port, args.seconds)
        if args.test == "link":
            return run_link(port)
        if args.test == "hold":
            return run_hold(port, args.seconds)
        return run_steer(port, args.delta, args.seconds, args.yes)
    except KeyboardInterrupt:
        print("\ninterrupted: commands stopped, firmware failsafe takes over")
        return 1
    except (OSError, ValueError, RuntimeError) as error:
        print(f"error: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
