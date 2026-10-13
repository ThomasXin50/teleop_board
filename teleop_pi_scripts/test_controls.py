import sys
import time
import types
import unittest
from collections import namedtuple

from controls import ARM, DISARM, LIMIT_DOWN, LIMIT_UP, ControlMapper, GamepadState, RumbleCues


def pad(steer=0.0, throttle=0.0, brake=0.0, buttons=(), connected=True):
    return GamepadState(connected, steer, throttle, brake, frozenset(buttons), "test pad")


class ControlMapperTest(unittest.TestCase):
    def setUp(self):
        self.mapper = ControlMapper(max_speed=2.0, max_steer=30.0, speed_step=0.25)

    def arm(self):
        self.mapper.update(pad())
        self.mapper.update(pad(buttons={ARM}))
        self.mapper.update(pad())
        self.assertTrue(self.mapper.armed)

    def test_starts_disarmed_with_zero_speed(self):
        self.assertEqual(self.mapper.update(pad(throttle=1.0)).speed_mph, 0.0)

    def test_throttle_scales_with_speed_limit(self):
        self.arm()
        self.assertAlmostEqual(self.mapper.update(pad(throttle=1.0)).speed_mph, 1.0)
        self.mapper.update(pad(throttle=1.0, buttons={LIMIT_UP}))
        self.assertAlmostEqual(self.mapper.update(pad(throttle=1.0)).speed_mph, 1.25)

    def test_speed_limit_bounded(self):
        for _ in range(20):
            self.mapper.update(pad(buttons={LIMIT_UP}))
            self.mapper.update(pad())
        self.assertEqual(self.mapper.speed_limit, 2.0)
        for _ in range(20):
            self.mapper.update(pad(buttons={LIMIT_DOWN}))
            self.mapper.update(pad())
        self.assertEqual(self.mapper.speed_limit, 0.25)

    def test_holding_limit_button_steps_once(self):
        for _ in range(5):
            self.mapper.update(pad(buttons={LIMIT_UP}))
        self.assertEqual(self.mapper.speed_limit, 1.25)

    def test_cannot_arm_with_throttle_pressed(self):
        self.mapper.update(pad(throttle=0.6, buttons={ARM}))
        self.assertFalse(self.mapper.armed)

    def test_disarm_button(self):
        self.arm()
        self.mapper.update(pad(throttle=1.0, buttons={DISARM}))
        self.assertFalse(self.mapper.armed)
        self.assertEqual(self.mapper.update(pad(throttle=1.0)).speed_mph, 0.0)

    def test_brake_is_proportional_and_overrides_throttle(self):
        self.arm()
        command = self.mapper.update(pad(throttle=1.0, brake=1.0))
        self.assertEqual((command.speed_mph, command.brake_percent), (0.0, 100.0))
        self.assertTrue(self.mapper.armed, "braking does not disarm")
        self.assertAlmostEqual(self.mapper.update(pad(brake=0.525)).brake_percent, 50.0)
        self.assertEqual(self.mapper.update(pad(throttle=1.0, brake=0.03)).brake_percent, 0.0)
        self.assertAlmostEqual(self.mapper.update(pad(throttle=1.0, brake=0.03)).speed_mph, 1.0)

    def test_brake_works_while_disarmed(self):
        self.assertEqual(self.mapper.update(pad(brake=1.0)).brake_percent, 100.0)

    def test_disconnect_disarms_and_requests_failsafe(self):
        self.arm()
        command = self.mapper.update(pad(throttle=1.0, connected=False))
        self.assertTrue(command.failsafe)
        self.assertEqual(command.speed_mph, 0.0)
        self.assertFalse(self.mapper.armed)
        reconnected = self.mapper.update(pad(throttle=1.0))
        self.assertFalse(reconnected.failsafe)
        self.assertEqual(reconnected.speed_mph, 0.0, "must re-arm after reconnecting")

    def test_steering_direction_and_deadzone(self):
        self.assertEqual(self.mapper.update(pad(steer=0.05)).steering_deg, 0.0)
        self.assertAlmostEqual(self.mapper.update(pad(steer=-1.0)).steering_deg, 30.0)  # left = +deg
        self.assertAlmostEqual(self.mapper.update(pad(steer=1.0)).steering_deg, -30.0)
        inverted = ControlMapper(2.0, 30.0, invert_steering=True)
        self.assertAlmostEqual(inverted.update(pad(steer=-1.0)).steering_deg, -30.0)


# ---- gamepad.py against a stub evdev using real Linux input codes ----------

AbsInfo = namedtuple("AbsInfo", "value min max fuzz flat resolution")
Event = namedtuple("Event", "type code value")

EV_KEY, EV_ABS, EV_FF = 1, 3, 0x15
FF_RUMBLE = 0x50
ABS_X, ABS_Z, ABS_RZ, ABS_HAT0Y = 0x00, 0x02, 0x05, 0x11
BTN_SOUTH, BTN_EAST, BTN_TL2, BTN_TR2, BTN_START = 0x130, 0x131, 0x138, 0x139, 0x13B
BTN_DPAD_UP, BTN_DPAD_DOWN = 0x220, 0x221


def install_fake_evdev():
    ecodes = types.SimpleNamespace(
        EV_KEY=EV_KEY, EV_ABS=EV_ABS, ABS_X=ABS_X, ABS_Z=ABS_Z, ABS_RZ=ABS_RZ,
        ABS_HAT0Y=ABS_HAT0Y, BTN_SOUTH=BTN_SOUTH, BTN_EAST=BTN_EAST, BTN_TL2=BTN_TL2,
        BTN_TR2=BTN_TR2, BTN_START=BTN_START, BTN_DPAD_UP=BTN_DPAD_UP,
        BTN_DPAD_DOWN=BTN_DPAD_DOWN, EV_FF=EV_FF, FF_RUMBLE=FF_RUMBLE,
        ABS={ABS_X: "ABS_X", ABS_Z: "ABS_Z", ABS_RZ: "ABS_RZ", ABS_HAT0Y: "ABS_HAT0Y"})
    module = types.ModuleType("evdev")
    module.ecodes = ecodes
    record = lambda name: (lambda *args, **kwargs: (name, args, kwargs))
    module.ff = types.SimpleNamespace(Effect=record("Effect"), Trigger=record("Trigger"),
                                      Replay=record("Replay"), Rumble=record("Rumble"),
                                      EffectType=record("EffectType"))
    module.devices = {}
    module.list_devices = lambda: list(module.devices)
    module.InputDevice = lambda path: module.devices[path]
    sys.modules["evdev"] = module
    return module


class FakeDevice:
    def __init__(self, axes, keys=(BTN_SOUTH, BTN_EAST, BTN_START), rumble=False):
        self.name = "Fake Controller"
        self.axes = axes
        self.keys = list(keys)
        self.rumble = rumble
        self.effects = []
        self.played = []
        self.events = []
        self.fd = -1
        self.unplugged = False

    def capabilities(self):
        caps = {EV_KEY: self.keys, EV_ABS: list(self.axes.items())}
        if self.rumble:
            caps[EV_FF] = [FF_RUMBLE]
        return caps

    def upload_effect(self, effect):
        self.effects.append(effect)
        return 0

    def write(self, ev_type, code, value):
        self.played.append((ev_type, code, value))

    def read_one(self):
        if self.unplugged:
            raise OSError(19, "No such device")
        return self.events.pop(0) if self.events else None

    def close(self):
        pass


def xbox_axes(throttle_rest=0):
    return {
        ABS_X: AbsInfo(0, -32768, 32767, 16, 128, 0),
        ABS_Z: AbsInfo(0, 0, 1023, 0, 0, 0),
        ABS_RZ: AbsInfo(throttle_rest, 0, 1023, 0, 0, 0),
        ABS_HAT0Y: AbsInfo(0, -1, 1, 0, 0, 0),
    }


class GamepadReaderTest(unittest.TestCase):
    def setUp(self):
        self.evdev = install_fake_evdev()
        sys.modules.pop("gamepad", None)
        import gamepad
        self.gamepad = gamepad
        gamepad._wait_readable = lambda device, timeout: (time.sleep(0.005) or True)
        gamepad.RECONNECT_S = 0.05

    def tearDown(self):
        sys.modules.pop("evdev", None)
        sys.modules.pop("gamepad", None)

    def wait_for(self, reader, predicate, timeout=1.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            state = reader.state()
            if predicate(state):
                return state
            time.sleep(0.01)
        self.fail(f"state never matched: {reader.state()} error={reader.error}")

    def test_reads_axes_and_buttons(self):
        device = FakeDevice(xbox_axes())
        self.evdev.devices["/dev/input/event5"] = device
        with self.gamepad.Gamepad() as reader:
            self.wait_for(reader, lambda s: s.connected)
            device.events += [Event(EV_ABS, ABS_X, -32768), Event(EV_ABS, ABS_RZ, 1023),
                              Event(EV_ABS, ABS_Z, 512), Event(EV_KEY, BTN_START, 1),
                              Event(EV_ABS, ABS_HAT0Y, -1)]
            state = self.wait_for(reader, lambda s: ARM in s.buttons and s.throttle == 1.0)
            self.assertAlmostEqual(state.steer, -1.0)
            self.assertAlmostEqual(state.brake, 512 / 1023)
            self.assertIn(LIMIT_UP, state.buttons)
            device.events += [Event(EV_KEY, BTN_START, 0), Event(EV_ABS, ABS_HAT0Y, 0)]
            state = self.wait_for(reader, lambda s: not s.buttons)

    def test_digital_triggers_fallback(self):
        axes = {ABS_X: AbsInfo(128, 0, 255, 0, 0, 0)}
        device = FakeDevice(axes, keys=(BTN_SOUTH, BTN_TR2, BTN_TL2))
        self.evdev.devices["/dev/input/event1"] = device
        with self.gamepad.Gamepad() as reader:
            self.wait_for(reader, lambda s: s.connected)
            device.events += [Event(EV_KEY, BTN_TR2, 1), Event(EV_KEY, BTN_TL2, 1)]
            self.wait_for(reader, lambda s: s.throttle == 1.0 and s.brake == 1.0)

    def test_rejects_stick_on_trigger_axis(self):
        self.evdev.devices["/dev/input/event2"] = FakeDevice(xbox_axes(throttle_rest=512))
        with self.gamepad.Gamepad() as reader:
            time.sleep(0.2)
            self.assertFalse(reader.state().connected)
            self.assertIn("not like a trigger", reader.error)

    def test_unplug_reports_disconnected(self):
        device = FakeDevice(xbox_axes())
        self.evdev.devices["/dev/input/event5"] = device
        with self.gamepad.Gamepad("/dev/input/event5") as reader:
            self.wait_for(reader, lambda s: s.connected)
            device.events.append(Event(EV_ABS, ABS_RZ, 1023))
            self.wait_for(reader, lambda s: s.throttle == 1.0)
            device.unplugged = True
            self.wait_for(reader, lambda s: not s.connected and s.throttle == 0.0)
            self.assertIn("controller lost", reader.error)

    def test_rumble_plays_each_pulse_of_a_pattern(self):
        device = FakeDevice(xbox_axes(), rumble=True)
        self.evdev.devices["/dev/input/event5"] = device
        self.gamepad.RUMBLE_GAP_S = 0.0
        with self.gamepad.Gamepad() as reader:
            self.wait_for(reader, lambda s: s.connected)
            reader.rumble("ready")
            deadline = time.monotonic() + 1.0
            while len(device.played) < 2 and time.monotonic() < deadline:
                time.sleep(0.01)
            self.assertEqual(device.played, [(EV_FF, 0, 1), (EV_FF, 0, 1)])

    def test_rumble_is_ignored_without_motors(self):
        device = FakeDevice(xbox_axes(), rumble=False)
        self.evdev.devices["/dev/input/event5"] = device
        with self.gamepad.Gamepad() as reader:
            self.wait_for(reader, lambda s: s.connected)
            reader.rumble("arm")
            time.sleep(0.1)
            self.assertEqual(device.played, [])


class RumbleCuesTest(unittest.TestCase):
    def test_ready_once_when_connected_and_vehicle_accepts_commands(self):
        cues = RumbleCues()
        self.assertEqual(cues.update(True, False, False), [])
        self.assertEqual(cues.update(True, True, False), ["ready"])
        self.assertEqual(cues.update(True, True, False), [])

    def test_arm_and_disarm(self):
        cues = RumbleCues()
        cues.update(True, True, False)
        self.assertEqual(cues.update(True, True, True), ["arm"])
        self.assertEqual(cues.update(True, True, True), [])
        self.assertEqual(cues.update(True, True, False), ["disarm"])

    def test_reconnect_cues_ready_again_without_a_disarm_buzz(self):
        cues = RumbleCues()
        cues.update(True, True, True)
        self.assertEqual(cues.update(False, True, False), [])  # nothing to buzz
        self.assertEqual(cues.update(True, True, False), ["ready"])


if __name__ == "__main__":
    unittest.main()
