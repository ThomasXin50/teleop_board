import os
import random
import struct
import threading
import time
import unittest

import protocol


def firmware_crc32(data: bytes) -> int:
    # Line-for-line port of Andre_Code_For_Thoma/src/crc32.c.
    crc = 0xFFFFFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            mask = -(crc & 1) & 0xFFFFFFFF
            crc = (crc >> 1) ^ (0xEDB88320 & mask)
    return ~crc & 0xFFFFFFFF


def telemetry_frame(sequence, angle=0, brake=0, rpm=(0, 0, 0, 0), throttle=0, status=0,
                    steer_mv=0, brake_mv=0):
    payload = struct.pack("<hHHHHHHHHH", angle, brake, *rpm, throttle, status,
                          steer_mv, brake_mv)
    return protocol.encode_frame(protocol.TYPE_TELEMETRY, sequence, payload)


class CrcTest(unittest.TestCase):
    def test_check_value(self):
        self.assertEqual(protocol.crc32_ieee(b"123456789"), 0xCBF43926)

    def test_matches_firmware_implementation(self):
        rng = random.Random(1)
        for length in range(0, 40):
            data = bytes(rng.randrange(256) for _ in range(length))
            self.assertEqual(protocol.crc32_ieee(data), firmware_crc32(data))


class EncodeTest(unittest.TestCase):
    def test_command_layout(self):
        frame = protocol.encode_command(0x1234, -12.34, 5.67, 42.5)
        self.assertEqual(frame[:7], bytes([0xA5, 0x5A, 3, 1, 6, 0x34, 0x12]))
        self.assertEqual(frame[7:13], struct.pack("<hHH", -1234, 567, 4250))
        self.assertEqual(len(frame), 17)
        (crc,) = struct.unpack("<I", frame[13:])
        self.assertEqual(crc, firmware_crc32(frame[2:13]))

    def test_command_clamps_to_firmware_limits(self):
        frame = protocol.encode_command(1, 90.0, 99.0, 150.0)
        self.assertEqual(struct.unpack("<hHH", frame[7:13]), (6500, 1000, 10000))
        frame = protocol.encode_command(1, -90.0, -5.0, -1.0)
        self.assertEqual(struct.unpack("<hHH", frame[7:13]), (-6500, 0, 0))
        frame = protocol.encode_command(1, float("nan"), float("inf"), float("nan"))
        self.assertEqual(struct.unpack("<hHH", frame[7:13]), (0, 0, 0))

    def test_sequence_wraps(self):
        self.assertEqual(protocol.encode_command(0x10001, 0, 0, 0)[5:7], b"\x01\x00")

    def test_failsafe_layout(self):
        frame = protocol.encode_failsafe(0x0102, protocol.FAILSAFE_REASON_CONTROLLER_LOST)
        self.assertEqual(frame[:7], bytes([0xA5, 0x5A, 3, 3, 1, 0x02, 0x01]))
        self.assertEqual(frame[7], 1)
        (crc,) = struct.unpack("<I", frame[8:])
        self.assertEqual(crc, firmware_crc32(frame[2:8]))

    def test_older_version_frames_rejected(self):
        body = struct.pack("<BBBH", 2, 2, 16, 9) + bytes(16)
        stale = protocol.MAGIC + body + struct.pack("<I", firmware_crc32(body))
        parser = protocol.FrameParser()
        self.assertEqual(list(parser.feed(stale)), [])
        self.assertGreater(parser.bad_frames, 0)

    def test_rejects_oversize_payload(self):
        with self.assertRaises(ValueError):
            protocol.encode_frame(2, 0, bytes(protocol.MAX_PAYLOAD + 1))


class ParserTest(unittest.TestCase):
    def test_telemetry_fields_follow_main_c_layout(self):
        frame = telemetry_frame(7, angle=-1550, brake=4250, rpm=(10, 20, 30, 40),
                                throttle=1234, status=0b1001_1011_1101,
                                steer_mv=1650, brake_mv=512)
        (parsed,) = protocol.FrameParser().feed(frame)
        t = protocol.Telemetry.from_payload(parsed.sequence, parsed.payload)
        self.assertEqual(t.sequence, 7)
        self.assertAlmostEqual(t.steering_deg, -15.5)
        self.assertAlmostEqual(t.brake_percent, 42.5)
        self.assertEqual((t.rpm_front_left, t.rpm_front_right,
                          t.rpm_back_right, t.rpm_back_left), (10, 20, 30, 40))
        self.assertAlmostEqual(t.throttle_percent, 12.34)
        self.assertTrue(t.command_active)
        self.assertFalse(t.steer_sensor_ok)
        self.assertTrue(t.brake_sensor_ok)
        self.assertTrue(t.steer_at_limit)
        self.assertTrue(t.brake_at_limit)
        self.assertTrue(t.failsafe_braking)
        self.assertEqual(t.faults, ["steering no progress", "brake overtravel"])
        self.assertTrue(t.steer_calibrated)
        self.assertFalse(t.brake_calibrated)
        self.assertAlmostEqual(t.steering_volts, 1.65)
        self.assertAlmostEqual(t.brake_volts, 0.512)

    def test_resyncs_through_garbage_and_split_reads(self):
        stream = (b"\x00\xA5\xA5\x5A\xFF" + telemetry_frame(1) + b"\x5A\xA5"
                  + telemetry_frame(2) + b"\xA5")
        parser = protocol.FrameParser()
        frames = [f for byte in stream for f in parser.feed(bytes([byte]))]
        self.assertEqual([f.sequence for f in frames], [1, 2])

    def test_bad_crc_is_dropped_and_next_frame_recovered(self):
        corrupt = bytearray(telemetry_frame(1))
        corrupt[9] ^= 0xFF
        parser = protocol.FrameParser()
        frames = list(parser.feed(bytes(corrupt) + telemetry_frame(2)))
        self.assertEqual([f.sequence for f in frames], [2])
        self.assertEqual(parser.bad_frames, 1)

    def test_oversize_length_is_rejected(self):
        bogus = b"\xA5\x5A\x01\x02\x11\x00\x00"  # length 17 > MAX_PAYLOAD
        frames = list(protocol.FrameParser().feed(bogus + telemetry_frame(3)))
        self.assertEqual([f.sequence for f in frames], [3])


class FakeFirmware:
    """Serial stand-in that follows vehicle_accept_packet()'s sequence rule."""

    def __init__(self, last_sequence):
        self.last_sequence = last_sequence
        self.command_seen = last_sequence is not None
        self.last_accept = None
        self.failsafe_requested = False
        self.accepted = []
        self.failsafe_count = 0
        self.parser = protocol.FrameParser()
        self.is_open = False
        self.telemetry_sequence = 0
        self.next_telemetry = 0.0
        self.lock = threading.Lock()
        self.port = self.baudrate = self.timeout = self.write_timeout = self.dtr = None

    def open(self):
        self.is_open = True

    def close(self):
        self.is_open = False

    def reset_input_buffer(self):
        pass

    @property
    def in_waiting(self):
        return 0

    def write(self, data):
        with self.lock:
            for frame in self.parser.feed(data):
                seq = frame.sequence
                if self.command_seen and not 0 < ((seq - self.last_sequence) & 0xFFFF) < 0x8000:
                    continue
                self.last_sequence, self.command_seen = seq, True
                self.last_accept = time.monotonic()
                self.failsafe_requested = frame.type == protocol.TYPE_FAILSAFE
                if self.failsafe_requested:
                    self.failsafe_count += 1
                else:
                    self.accepted.append(struct.unpack("<hHH", frame.payload))
        return len(data)

    def read(self, size):
        time.sleep(0.01)
        now = time.monotonic()
        with self.lock:
            if now < self.next_telemetry:
                return b""
            self.next_telemetry = now + 0.2  # 5 Hz like the firmware
            link = self.last_accept is not None and now - self.last_accept <= 0.5
            status = 0
            if link:
                status |= protocol.STATUS_HOST_LINK_ACTIVE
                status |= (protocol.STATUS_FAILSAFE_REQUESTED if self.failsafe_requested
                           else protocol.STATUS_COMMAND_ACTIVE)
            self.telemetry_sequence += 1
            return telemetry_frame(self.telemetry_sequence, status=status)


@unittest.skipIf(os.environ.get("SKIP_LINK_TESTS"), "link tests disabled")
class LinkTest(unittest.TestCase):
    def run_link(self, firmware, seconds, failsafe=False):
        try:
            import link
        except ImportError as error:
            self.skipTest(f"pyserial not installed: {error}")
        vehicle = link.VehicleLink("fake", send_hz=20)
        vehicle._serial = firmware
        with vehicle:
            vehicle.set_command(5.0, 1.5, 20.0)
            if failsafe:
                vehicle.request_failsafe(protocol.FAILSAFE_REASON_CONTROLLER_LOST)
            time.sleep(seconds)
            telemetry, _ = vehicle.latest_telemetry()
        return vehicle, telemetry

    def test_commands_accepted_by_fresh_firmware(self):
        firmware = FakeFirmware(last_sequence=None)
        vehicle, telemetry = self.run_link(firmware, 1.0)
        self.assertTrue(telemetry.command_active)
        self.assertEqual(vehicle.stats.resyncs, 0)
        self.assertIn((500, 150, 2000), firmware.accepted)
        self.assertEqual(firmware.accepted[-1], (500, 0, 2000),
                         "close() must send zero speed, keeping steering and brake")

    def test_failsafe_request_streams_without_resync(self):
        firmware = FakeFirmware(last_sequence=None)
        vehicle, telemetry = self.run_link(firmware, 2.0, failsafe=True)
        self.assertTrue(telemetry.failsafe_requested)
        self.assertFalse(telemetry.command_active)
        self.assertEqual(vehicle.stats.resyncs, 0, "failsafe must not look like a stale link")
        self.assertGreater(firmware.failsafe_count, 20)
        self.assertTrue(firmware.failsafe_requested, "close() must stay in failsafe")

    def test_recovers_when_firmware_remembers_a_newer_sequence(self):
        # Firmware was driven by a previous client up to sequence 30000, so
        # this client's sequence 1, 2, ... looks old and is ignored.
        firmware = FakeFirmware(last_sequence=30000)
        vehicle, telemetry = self.run_link(firmware, 5.0)
        self.assertTrue(telemetry.command_active)
        self.assertGreaterEqual(vehicle.stats.resyncs, 1)
        self.assertIn((500, 150, 2000), firmware.accepted)


if __name__ == "__main__":
    unittest.main()
