import unittest

import protocol

try:
    import bench
except ImportError:  # pyserial missing; link.py imports it
    bench = None


def telemetry(steering_deg=0.0, steering_volts=1.2, brake_percent=0.0, brake_volts=0.5,
              rpm_back_left=0, rpm_back_right=0) -> protocol.Telemetry:
    return protocol.Telemetry(sequence=1, steering_deg=steering_deg,
                              brake_percent=brake_percent, rpm_front_left=0,
                              rpm_front_right=0, rpm_back_right=rpm_back_right,
                              rpm_back_left=rpm_back_left, throttle_percent=0.0, status=0,
                              steering_volts=steering_volts, brake_volts=brake_volts)


@unittest.skipIf(bench is None, "pyserial not installed")
class SteerTargetTest(unittest.TestCase):
    def test_moves_from_current_angle_not_center(self):
        self.assertEqual(bench.steer_target(20.0, 5.0), 25.0)
        self.assertEqual(bench.steer_target(-10.0, -5.0), -15.0)

    def test_refuses_targets_near_the_limits(self):
        limit = protocol.STEER_ANGLE_MAX_DEG
        self.assertIsNone(bench.steer_target(limit - 3.0, 5.0))
        self.assertIsNone(bench.steer_target(-limit + 1.0, -1.0))
        self.assertEqual(bench.steer_target(limit - 7.0, 5.0), limit - 2.0)


@unittest.skipIf(bench is None, "pyserial not installed")
class SteeringProblemTest(unittest.TestCase):
    OK = protocol.STATUS_STEER_SENSOR_OK

    def status_telemetry(self, steering_deg, status):
        t = telemetry(steering_deg=steering_deg)
        return protocol.Telemetry(**{**t.__dict__, "status": status})

    def test_angle_inside_travel_is_fine(self):
        self.assertIsNone(bench.steering_problem(self.status_telemetry(4.5, self.OK)))

    def test_angle_outside_travel_is_refused(self):
        # The case that latched a no-progress fault: sensor read -158.7 deg.
        problem = bench.steering_problem(self.status_telemetry(-158.7, self.OK))
        self.assertIn("outside", problem)

    def test_invalid_reading_is_refused(self):
        self.assertIn("invalid", bench.steering_problem(self.status_telemetry(0.0, 0)))

    def test_uncalibrated_steering_never_moves(self):
        status = self.OK | protocol.STATUS_STEER_UNCALIBRATED
        self.assertIsNone(bench.steering_problem(self.status_telemetry(-158.7, status)))


@unittest.skipIf(bench is None, "pyserial not installed")
class SummarizeTest(unittest.TestCase):
    def test_one_line_per_bucket_with_ranges(self):
        rows = [(0.1, telemetry(steering_volts=1.0, brake_volts=0.5)),
                (0.6, telemetry(steering_volts=1.2, brake_volts=0.7, rpm_back_left=30)),
                (1.2, telemetry(steering_volts=1.5, rpm_back_right=40))]
        lines = bench.summarize(rows)
        self.assertEqual(len(lines), 2)
        self.assertIn("1.100 V [1.000..1.200]", lines[0])
        self.assertIn("0.600 V [0.500..0.700]", lines[0])
        self.assertIn("BL   30", lines[0])
        self.assertIn("BR   40", lines[1])


if __name__ == "__main__":
    unittest.main()
