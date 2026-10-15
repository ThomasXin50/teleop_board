import json
import unittest

import protocol

try:
    import telemetry
except ImportError:  # pyserial missing; link.py imports it
    telemetry = None


@unittest.skipIf(telemetry is None, "pyserial not installed")
class TelemetryOutputTest(unittest.TestCase):
    def sample(self, status: int) -> protocol.Telemetry:
        return protocol.Telemetry(sequence=12, steering_deg=-4.5, brake_percent=25.0,
                                  rpm_front_left=0, rpm_front_right=0, rpm_back_right=31,
                                  rpm_back_left=30, throttle_percent=0.0, status=status,
                                  steering_volts=1.183, brake_volts=0.832)

    def test_json_has_every_field(self):
        status = (protocol.STATUS_COMMAND_ACTIVE | protocol.STATUS_STEER_SENSOR_OK
                  | protocol.STATUS_BRAKE_SENSOR_OK | protocol.STATUS_BRAKE_UNCALIBRATED)
        data = json.loads(json.dumps(telemetry.telemetry_dict(self.sample(status))))
        self.assertEqual(data["steering_deg"], -4.5)
        self.assertEqual(data["rpm"], {"front_left": 0, "front_right": 0,
                                       "back_left": 30, "back_right": 31})
        self.assertTrue(data["command_active"])
        self.assertTrue(data["steer_calibrated"])
        self.assertFalse(data["brake_calibrated"])
        self.assertEqual(data["faults"], [])

    def test_line_shows_values_and_flags(self):
        line = telemetry.format_line(self.sample(protocol.STATUS_BRAKE_SENSOR_OK))
        self.assertIn("steer   -4.5°", line)
        self.assertIn("brake  25.0%", line)
        self.assertIn("BL   30 BR   31", line)
        self.assertIn("STEER-SENSOR-FAULT", line)
        self.assertNotIn("BENCH-MODE", line)

    def test_bench_firmware_is_flagged(self):
        t = self.sample(protocol.STATUS_BENCH_MODE)
        self.assertTrue(t.bench_mode)
        self.assertIn("BENCH-MODE", telemetry.format_line(t))
        self.assertTrue(telemetry.telemetry_dict(t)["bench_mode"])

    def test_clutch_state_is_shown(self):
        engaged = self.sample(protocol.STATUS_CLUTCHES_ENGAGED)
        self.assertIn("CLUTCHED", telemetry.format_line(engaged))
        self.assertTrue(telemetry.telemetry_dict(engaged)["clutches_engaged"])
        self.assertNotIn("CLUTCHED", telemetry.format_line(self.sample(0)))


if __name__ == "__main__":
    unittest.main()
