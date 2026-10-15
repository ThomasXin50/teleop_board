"""USB CDC framing shared with the RP2040 vehicle controller.

Mirrors Andre_Code_For_Thoma/src/protocol.c and README.md. All multi-byte
fields are little-endian:

    offset  size  field
    0       2     magic A5 5A
    2       1     version (3)
    3       1     type (1 = command, 2 = telemetry, 3 = failsafe request,
                  4 = clutch, bench firmware only)
    4       1     payload length
    5       2     sequence
    7       N     payload
    7+N     4     CRC-32/IEEE over bytes 2..6 + payload
"""

from __future__ import annotations

import math
import struct
import zlib
from dataclasses import dataclass
from typing import Iterator, Optional

MAGIC = b"\xA5\x5A"
VERSION = 3
TYPE_COMMAND = 1
TYPE_TELEMETRY = 2
TYPE_FAILSAFE = 3
TYPE_CLUTCH = 4  # bench firmware only; the vehicle firmware ignores it

# Failsafe message reasons (informational; the firmware brakes the same way).
FAILSAFE_REASON_CONTROLLER_LOST = 1

HEADER_SIZE = 7
CRC_SIZE = 4
CMD_PAYLOAD = 6
FAILSAFE_PAYLOAD = 1
CLUTCH_PAYLOAD = 1
TELEMETRY_PAYLOAD = 20
MAX_PAYLOAD = TELEMETRY_PAYLOAD  # firmware rejects anything longer

STATUS_COMMAND_ACTIVE = 0x0001
STATUS_STEER_SENSOR_OK = 0x0002
STATUS_BRAKE_SENSOR_OK = 0x0004
STATUS_STEER_AT_LIMIT = 0x0008
STATUS_BRAKE_AT_LIMIT = 0x0010
STATUS_FAILSAFE_BRAKING = 0x0020
# Latched in the firmware until reset; the faulted axis holds still.
AXIS_FAULTS = {
    0x0040: "steering overtravel",
    0x0080: "steering no progress",
    0x0100: "brake overtravel",
    0x0200: "brake no progress",
}
STATUS_STEER_UNCALIBRATED = 0x0400
STATUS_BRAKE_UNCALIBRATED = 0x0800
STATUS_HOST_LINK_ACTIVE = 0x1000     # any valid message within the timeout
STATUS_FAILSAFE_REQUESTED = 0x2000   # latest message was a failsafe request
STATUS_BENCH_MODE = 0x4000           # bench firmware: throttle interlocks off, steering open loop
STATUS_CLUTCHES_ENGAGED = 0x8000     # clutch outputs driven to engage

# Firmware clamps to these (config.h); the client clamps too so the UI
# shows what the vehicle will actually do.
STEER_ANGLE_MIN_DEG = -65.0
STEER_ANGLE_MAX_DEG = 65.0
SPEED_MAX_MPH = 10.0
BRAKE_MAX_PERCENT = 100.0


def crc32_ieee(data: bytes) -> int:
    # Same algorithm as src/crc32.c (reflected 0xEDB88320, init/xorout 0xFFFFFFFF).
    return zlib.crc32(data) & 0xFFFFFFFF


def encode_frame(frame_type: int, sequence: int, payload: bytes) -> bytes:
    if len(payload) > MAX_PAYLOAD:
        raise ValueError(f"payload too long: {len(payload)} > {MAX_PAYLOAD}")
    body = struct.pack("<BBBH", VERSION, frame_type, len(payload),
                       sequence & 0xFFFF) + payload
    return MAGIC + body + struct.pack("<I", crc32_ieee(body))


def _clamp(value: float, low: float, high: float) -> float:
    if not math.isfinite(value):
        return 0.0
    return max(low, min(high, value))


def encode_command(sequence: int, steering_deg: float, speed_mph: float,
                   brake_percent: float,
                   steer_min: float = STEER_ANGLE_MIN_DEG,
                   steer_max: float = STEER_ANGLE_MAX_DEG,
                   speed_max: float = SPEED_MAX_MPH) -> bytes:
    """Command payload: int16 centidegrees steering, uint16 centi-mph speed,
    uint16 centi-percent brake target."""
    angle = round(_clamp(steering_deg, steer_min, steer_max) * 100.0)
    speed = round(_clamp(speed_mph, 0.0, speed_max) * 100.0)
    brake = round(_clamp(brake_percent, 0.0, BRAKE_MAX_PERCENT) * 100.0)
    angle = max(-32768, min(32767, angle))
    speed = max(0, min(65535, speed))
    return encode_frame(TYPE_COMMAND, sequence, struct.pack("<hHH", angle, speed, brake))


def encode_failsafe(sequence: int, reason: int) -> bytes:
    """Asks the firmware for failsafe braking (same as a comms timeout) while
    keeping the link alive. Shares the command sequence counter."""
    return encode_frame(TYPE_FAILSAFE, sequence, struct.pack("<B", reason & 0xFF))


def encode_clutch(sequence: int, engaged: bool) -> bytes:
    """Bench firmware only: engage or release the clutches (once a command has
    engaged them). Doesn't keep the link alive. Shares the command sequence
    counter."""
    return encode_frame(TYPE_CLUTCH, sequence, struct.pack("<B", 1 if engaged else 0))


@dataclass(frozen=True)
class Telemetry:
    sequence: int
    steering_deg: float
    brake_percent: float
    rpm_front_left: int
    rpm_front_right: int
    rpm_back_right: int
    rpm_back_left: int
    throttle_percent: float
    status: int
    steering_volts: float
    brake_volts: float

    @property
    def command_active(self) -> bool:
        return bool(self.status & STATUS_COMMAND_ACTIVE)

    @property
    def steer_sensor_ok(self) -> bool:
        return bool(self.status & STATUS_STEER_SENSOR_OK)

    @property
    def brake_sensor_ok(self) -> bool:
        return bool(self.status & STATUS_BRAKE_SENSOR_OK)

    @property
    def steer_at_limit(self) -> bool:
        return bool(self.status & STATUS_STEER_AT_LIMIT)

    @property
    def brake_at_limit(self) -> bool:
        return bool(self.status & STATUS_BRAKE_AT_LIMIT)

    @property
    def failsafe_braking(self) -> bool:
        return bool(self.status & STATUS_FAILSAFE_BRAKING)

    @property
    def host_link_active(self) -> bool:
        return bool(self.status & STATUS_HOST_LINK_ACTIVE)

    @property
    def failsafe_requested(self) -> bool:
        return bool(self.status & STATUS_FAILSAFE_REQUESTED)

    @property
    def bench_mode(self) -> bool:
        return bool(self.status & STATUS_BENCH_MODE)

    @property
    def clutches_engaged(self) -> bool:
        return bool(self.status & STATUS_CLUTCHES_ENGAGED)

    @property
    def faults(self) -> list[str]:
        return [name for bit, name in AXIS_FAULTS.items() if self.status & bit]

    @property
    def steer_calibrated(self) -> bool:
        return not self.status & STATUS_STEER_UNCALIBRATED

    @property
    def brake_calibrated(self) -> bool:
        return not self.status & STATUS_BRAKE_UNCALIBRATED

    @classmethod
    def from_payload(cls, sequence: int, payload: bytes) -> "Telemetry":
        if len(payload) != TELEMETRY_PAYLOAD:
            raise ValueError(f"telemetry payload must be {TELEMETRY_PAYLOAD} bytes")
        (angle, brake, fl, fr, br, bl, throttle, status,
         steer_mv, brake_mv) = struct.unpack("<hHHHHHHHHH", payload)
        # Brake percent is unclamped in the firmware, saturated here to 0..655.35.
        return cls(sequence, angle / 100.0, brake / 100.0, fl, fr, br, bl,
                   throttle / 100.0, status, steer_mv / 1000.0, brake_mv / 1000.0)


@dataclass(frozen=True)
class Frame:
    type: int
    sequence: int
    payload: bytes


class FrameParser:
    """Stream resynchronizing parser with the same rules as proto_rx_push()."""

    def __init__(self) -> None:
        self._buffer = bytearray()
        self.good_frames = 0
        self.bad_frames = 0

    def feed(self, data: bytes) -> Iterator[Frame]:
        self._buffer.extend(data)
        while True:
            frame = self._next_frame()
            if frame is None:
                return
            yield frame

    def _next_frame(self) -> Optional[Frame]:
        buf = self._buffer
        while len(buf) >= 2:
            start = buf.find(MAGIC)
            if start < 0:
                # Keep a trailing A5 in case its 5A has not arrived yet.
                del buf[:len(buf) - 1 if buf[-1] == MAGIC[0] else len(buf)]
                return None
            if start:
                del buf[:start]
            if len(buf) < HEADER_SIZE:
                return None

            length = buf[4]
            if buf[2] != VERSION or length > MAX_PAYLOAD:
                self.bad_frames += 1
                del buf[:1]
                continue

            frame_length = HEADER_SIZE + length + CRC_SIZE
            if len(buf) < frame_length:
                return None

            crc_offset = HEADER_SIZE + length
            (received,) = struct.unpack_from("<I", buf, crc_offset)
            if received != crc32_ieee(bytes(buf[2:crc_offset])):
                self.bad_frames += 1
                del buf[:1]
                continue

            frame = Frame(buf[3], buf[5] | (buf[6] << 8),
                          bytes(buf[HEADER_SIZE:crc_offset]))
            del buf[:frame_length]
            self.good_frames += 1
            return frame
        return None
