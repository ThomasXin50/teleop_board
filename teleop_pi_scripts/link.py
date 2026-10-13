"""Serial link to the RP2040: streams commands and collects telemetry."""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass
from typing import Optional

import serial
from serial.tools import list_ports

import protocol

RPI_USB_VID = 0x2E8A  # Raspberry Pi (RP2040 pico_stdio_usb)
# Never 1200: opening the port at 1200 baud reboots the RP2040 into BOOTSEL.
BAUD = 115200

# The firmware drops any message whose sequence is not newer (int16 compare)
# than the last one it accepted, and never forgets that value. After a client
# restart our sequence may look "old" and every message would be ignored. If
# fresh telemetry says the host link is inactive while we are sending, jump
# ahead by a quarter of the sequence space; one of four jumps always lands in
# the accepted half. Remove once the firmware clears its sequence on timeout.
RESYNC_AFTER_S = 0.75
RESYNC_JUMP = 0x4000
MIN_SEND_HZ = 5.0


def find_port() -> Optional[str]:
    for port in list_ports.comports():
        if port.vid == RPI_USB_VID:
            return port.device
    return None


@dataclass
class LinkStats:
    commands_sent: int = 0
    failsafe_sent: int = 0
    resyncs: int = 0
    good_frames: int = 0
    bad_frames: int = 0


class VehicleLink:
    def __init__(self, port: str, send_hz: float = 20.0,
                 send_commands: bool = True) -> None:
        if send_hz < MIN_SEND_HZ:
            raise ValueError(f"send rate must be at least {MIN_SEND_HZ} Hz "
                             "(firmware drops commands older than 500 ms)")
        self._serial = serial.Serial()
        self._serial.port = port
        self._serial.baudrate = BAUD
        self._serial.timeout = 0.05
        self._serial.write_timeout = 0.2
        self._serial.dtr = True  # firmware only sends telemetry while DTR is set
        self._period = 1.0 / send_hz
        self._send_commands = send_commands

        self._lock = threading.Lock()
        self._steering_deg = 0.0
        self._speed_mph = 0.0
        self._brake_percent = 0.0
        self._failsafe_reason: Optional[int] = None
        self._sequence = 0
        self._telemetry: Optional[protocol.Telemetry] = None
        self._telemetry_time = 0.0
        self._inactive_since: Optional[float] = None
        self._parser = protocol.FrameParser()
        self.stats = LinkStats()

        self._stop = threading.Event()
        self._threads: list[threading.Thread] = []
        self.error: Optional[BaseException] = None

    def __enter__(self) -> "VehicleLink":
        self.open()
        return self

    def __exit__(self, *exc) -> None:
        self.close()

    def open(self) -> None:
        self._serial.open()
        self._serial.reset_input_buffer()
        self._threads = [threading.Thread(target=self._read_loop, daemon=True)]
        if self._send_commands:
            self._threads.append(threading.Thread(target=self._send_loop, daemon=True))
        for thread in self._threads:
            thread.start()

    def close(self) -> None:
        self._stop.set()
        for thread in self._threads:
            thread.join(timeout=1.0)
        if self._send_commands and self._serial.is_open:
            # Final frames: stay in failsafe if requested, else zero speed.
            self.set_speed_zero()
            try:
                for _ in range(3):
                    self._write_next()
            except serial.SerialException:
                pass
        self._serial.close()

    def set_command(self, steering_deg: float, speed_mph: float,
                    brake_percent: float) -> None:
        """Streams this command; also ends any failsafe request."""
        with self._lock:
            self._steering_deg = steering_deg
            self._speed_mph = speed_mph
            self._brake_percent = brake_percent
            self._failsafe_reason = None

    def request_failsafe(self, reason: int) -> None:
        """Streams failsafe messages instead of commands: the firmware cuts
        throttle, holds steering and ramps the brake, as on a comms timeout."""
        with self._lock:
            self._failsafe_reason = reason

    def set_speed_zero(self) -> None:
        with self._lock:
            self._speed_mph = 0.0

    def latest_telemetry(self) -> tuple[Optional[protocol.Telemetry], float]:
        """Returns the newest telemetry and its age in seconds (inf if none)."""
        with self._lock:
            if self._telemetry is None:
                return None, float("inf")
            return self._telemetry, time.monotonic() - self._telemetry_time

    def _write_next(self) -> None:
        with self._lock:
            self._sequence = (self._sequence + 1) & 0xFFFF
            if self._failsafe_reason is not None:
                frame = protocol.encode_failsafe(self._sequence, self._failsafe_reason)
                self.stats.failsafe_sent += 1
            else:
                frame = protocol.encode_command(self._sequence, self._steering_deg,
                                                self._speed_mph, self._brake_percent)
                self.stats.commands_sent += 1
        self._serial.write(frame)

    def _send_loop(self) -> None:
        next_send = time.monotonic()
        try:
            while not self._stop.is_set():
                self._write_next()
                self._maybe_resync()
                next_send += self._period
                delay = next_send - time.monotonic()
                if delay > 0:
                    self._stop.wait(delay)
                else:
                    next_send = time.monotonic()
        except (serial.SerialException, OSError) as error:
            self.error = error
            self._stop.set()

    def _maybe_resync(self) -> None:
        now = time.monotonic()
        with self._lock:
            telemetry = self._telemetry
            fresh = telemetry is not None and now - self._telemetry_time < 0.5
            if not fresh or telemetry.host_link_active:
                self._inactive_since = None
                return
            if self._inactive_since is None:
                self._inactive_since = now
            elif now - self._inactive_since >= RESYNC_AFTER_S:
                self._sequence = (self._sequence + RESYNC_JUMP) & 0xFFFF
                self._inactive_since = now
                self.stats.resyncs += 1

    def _read_loop(self) -> None:
        try:
            while not self._stop.is_set():
                data = self._serial.read(self._serial.in_waiting or 1)
                if not data:
                    continue
                for frame in self._parser.feed(data):
                    if (frame.type != protocol.TYPE_TELEMETRY
                            or len(frame.payload) != protocol.TELEMETRY_PAYLOAD):
                        continue
                    telemetry = protocol.Telemetry.from_payload(frame.sequence, frame.payload)
                    with self._lock:
                        self._telemetry = telemetry
                        self._telemetry_time = time.monotonic()
                self.stats.good_frames = self._parser.good_frames
                self.stats.bad_frames = self._parser.bad_frames
        except (serial.SerialException, OSError) as error:
            self.error = error
            self._stop.set()
