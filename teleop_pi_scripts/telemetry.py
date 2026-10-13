#!/usr/bin/env python3
"""Stream RP2040 telemetry to the terminal, like `ros2 topic echo`.

    python3 telemetry.py            # one readable line per message (5 Hz)
    python3 telemetry.py --json     # one JSON object per line, for logging/piping
    python3 telemetry.py --hz       # message rate only, like `ros2 topic hz`

Listen-only: it never sends commands, so the clutches stay released. Don't run
it at the same time as teleop.py; both would read the same serial port and
each would miss messages.
"""

from __future__ import annotations

import argparse
import json
import sys
import time

import protocol
from link import VehicleLink, find_port

WAIT_WARN_S = 1.0


def telemetry_dict(t: protocol.Telemetry) -> dict:
    return {
        "sequence": t.sequence,
        "steering_deg": t.steering_deg,
        "brake_percent": t.brake_percent,
        "throttle_percent": t.throttle_percent,
        "rpm": {"front_left": t.rpm_front_left, "front_right": t.rpm_front_right,
                "back_left": t.rpm_back_left, "back_right": t.rpm_back_right},
        "steering_volts": t.steering_volts,
        "brake_volts": t.brake_volts,
        "command_active": t.command_active,
        "host_link_active": t.host_link_active,
        "failsafe_braking": t.failsafe_braking,
        "failsafe_requested": t.failsafe_requested,
        "steer_sensor_ok": t.steer_sensor_ok,
        "brake_sensor_ok": t.brake_sensor_ok,
        "steer_calibrated": t.steer_calibrated,
        "brake_calibrated": t.brake_calibrated,
        "steer_at_limit": t.steer_at_limit,
        "brake_at_limit": t.brake_at_limit,
        "faults": t.faults,
        "bench_mode": t.bench_mode,
        "status": t.status,
    }


def format_line(t: protocol.Telemetry) -> str:
    flags = ["BENCH-MODE"] if t.bench_mode else []
    if t.command_active:
        flags.append("CMD")
    if t.failsafe_braking:
        flags.append("FAILSAFE")
    if not t.steer_sensor_ok:
        flags.append("STEER-SENSOR-FAULT")
    if not t.brake_sensor_ok:
        flags.append("BRAKE-SENSOR-FAULT")
    if t.steer_at_limit:
        flags.append("STEER-LIMIT")
    if not t.steer_calibrated:
        flags.append("STEER-UNCAL")
    if not t.brake_calibrated:
        flags.append("BRAKE-UNCAL")
    flags.extend(f.upper() for f in t.faults)
    return (f"#{t.sequence:5d}  steer {t.steering_deg:+6.1f}° ({t.steering_volts:.3f} V)  "
            f"brake {t.brake_percent:5.1f}% ({t.brake_volts:.3f} V)  "
            f"throttle {t.throttle_percent:5.1f}%  "
            f"rpm FL {t.rpm_front_left:4d} FR {t.rpm_front_right:4d} "
            f"BL {t.rpm_back_left:4d} BR {t.rpm_back_right:4d}  "
            f"[{' '.join(flags) or 'ok'}]")


def stream(link: VehicleLink, mode: str) -> int:
    last_sequence = None
    last_warn = time.monotonic()  # give the first message a moment to arrive
    window_start = time.monotonic()
    window_count = 0
    while link.error is None:
        telemetry, age = link.latest_telemetry()
        now = time.monotonic()
        if telemetry is not None and telemetry.sequence != last_sequence:
            last_sequence = telemetry.sequence
            window_count += 1
            if mode == "json":
                print(json.dumps({"time": time.time(), **telemetry_dict(telemetry)}), flush=True)
            elif mode == "line":
                print(time.strftime("%H:%M:%S ") + format_line(telemetry), flush=True)
        elif age > WAIT_WARN_S and now - last_warn > WAIT_WARN_S:
            last_warn = now
            print("waiting for telemetry (is the RP2040 running?)", file=sys.stderr, flush=True)
        if mode == "hz" and now - window_start >= 1.0:
            print(f"average rate: {window_count / (now - window_start):.2f} Hz", flush=True)
            window_start, window_count = now, 0
        time.sleep(0.02)
    print(f"serial link failed: {link.error}", file=sys.stderr)
    return 1


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--port", help="serial device (default: auto-detect, else /dev/ttyACM0)")
    group = parser.add_mutually_exclusive_group()
    group.add_argument("--json", action="store_true", help="print JSON lines")
    group.add_argument("--hz", action="store_true", help="print the message rate only")
    args = parser.parse_args(argv)
    port = args.port or find_port() or "/dev/ttyACM0"
    mode = "json" if args.json else "hz" if args.hz else "line"
    try:
        with VehicleLink(port, send_commands=False) as link:
            return stream(link, mode)
    except KeyboardInterrupt:
        return 0
    except BrokenPipeError:  # output piped into head, less, etc. that exited
        sys.stdout = None
        return 0
    except (OSError, ValueError) as error:
        print(f"error: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
