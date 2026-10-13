"""Linux evdev gamepad reader (runs on the Raspberry Pi)."""

from __future__ import annotations

import select
import threading
import time
from typing import Optional

import evdev
from evdev import ecodes, ff

from controls import ARM, DISARM, LIMIT_DOWN, LIMIT_UP, GamepadState

BUTTONS = {
    ecodes.BTN_START: ARM,
    ecodes.BTN_EAST: DISARM,
    ecodes.BTN_DPAD_UP: LIMIT_UP,
    ecodes.BTN_DPAD_DOWN: LIMIT_DOWN,
}
# Controllers without analog triggers report them as buttons instead.
DIGITAL_THROTTLE = ecodes.BTN_TR2
DIGITAL_BRAKE = ecodes.BTN_TL2
TRIGGER_AXES = (ecodes.ABS_Z, ecodes.ABS_RZ)
RECONNECT_S = 1.0
# Vibration cues: (milliseconds, strength 0..1) pulses, RUMBLE_GAP_S apart.
RUMBLE_PATTERNS = {
    "ready": ((150, 0.5), (150, 0.5)),  # connected and the RP2040 accepts commands
    "arm": ((400, 1.0),),
    "disarm": ((150, 0.35),),
}
RUMBLE_GAP_S = 0.15


class UnsupportedController(Exception):
    pass


def find_gamepad() -> Optional[str]:
    for path in evdev.list_devices():
        device = evdev.InputDevice(path)
        caps = device.capabilities()
        keys = caps.get(ecodes.EV_KEY, [])
        axes = [code for code, _ in caps.get(ecodes.EV_ABS, [])]
        device.close()
        if ecodes.BTN_SOUTH in keys and ecodes.ABS_X in axes:
            return path
    return None


def _check_triggers(device: evdev.InputDevice) -> None:
    # A trigger rests at its minimum. If ABS_Z/ABS_RZ rest mid-range they are a
    # second stick (common on generic pads), and using one as throttle would
    # command half speed with nothing pressed.
    axes = dict(device.capabilities().get(ecodes.EV_ABS, []))
    for code in TRIGGER_AXES:
        info = axes.get(code)
        if info is None:
            continue
        span = info.max - info.min
        if span <= 0 or info.value - info.min > 0.1 * span:
            raise UnsupportedController(
                f"{device.name}: {ecodes.ABS[code]} rests at {info.value} "
                f"(range {info.min}..{info.max}), not like a trigger")


class Gamepad:
    """Reads a controller on a background thread; state() is always safe to call."""

    def __init__(self, path: Optional[str] = None) -> None:
        self._path = path
        self._lock = threading.Lock()
        self._state = GamepadState()
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._device: Optional[evdev.InputDevice] = None
        self._rumble_lock = threading.Lock()
        self._effect_id = -1
        self.error: Optional[str] = None

    def __enter__(self) -> "Gamepad":
        self._thread.start()
        return self

    def __exit__(self, *exc) -> None:
        self._stop.set()
        self._thread.join(timeout=1.0)

    def state(self) -> GamepadState:
        with self._lock:
            return self._state

    def _set(self, state: GamepadState) -> None:
        with self._lock:
            self._state = state

    def rumble(self, pattern: str) -> None:
        """Plays a RUMBLE_PATTERNS cue without blocking. Does nothing if no
        controller is open or it has no rumble motors."""
        pulses = RUMBLE_PATTERNS[pattern]
        threading.Thread(target=self._play, args=(pulses,), daemon=True).start()

    def _play(self, pulses) -> None:
        with self._rumble_lock:
            for duration_ms, strength in pulses:
                with self._lock:
                    device = self._device
                if device is None:
                    return
                try:
                    if ecodes.FF_RUMBLE not in device.capabilities().get(ecodes.EV_FF, []):
                        return
                    level = int(0xFFFF * max(0.0, min(1.0, strength)))
                    effect = ff.Effect(
                        ecodes.FF_RUMBLE, self._effect_id, 0, ff.Trigger(0, 0),
                        ff.Replay(duration_ms, 0),
                        ff.EffectType(ff_rumble_effect=ff.Rumble(strong_magnitude=level,
                                                                 weak_magnitude=level)))
                    self._effect_id = device.upload_effect(effect)
                    device.write(ecodes.EV_FF, self._effect_id, 1)
                except OSError:  # unplugged mid-cue, or no write access
                    return
                time.sleep(duration_ms / 1000.0 + RUMBLE_GAP_S)

    def _run(self) -> None:
        while not self._stop.is_set():
            path = self._path or find_gamepad()
            if path is None:
                self.error = "no controller found"
                self._stop.wait(RECONNECT_S)
                continue
            try:
                device = evdev.InputDevice(path)
                try:
                    _check_triggers(device)
                    self.error = None
                    with self._lock:
                        self._device = device
                        self._effect_id = -1
                    self._read(device)
                finally:
                    with self._lock:
                        self._device = None
                    device.close()
            except UnsupportedController as error:
                self.error = str(error)
            except OSError as error:  # unplugged or Bluetooth dropped
                self.error = f"controller lost: {error}"
            self._set(GamepadState())
            self._stop.wait(RECONNECT_S)

    def _read(self, device: evdev.InputDevice) -> None:
        axes = dict(device.capabilities().get(ecodes.EV_ABS, []))

        def normalize(code: int, value: int, centered: bool) -> float:
            info = axes[code]
            span = info.max - info.min
            if span <= 0:
                return 0.0
            fraction = (value - info.min) / span
            return fraction * 2.0 - 1.0 if centered else fraction

        def initial(code: int, centered: bool) -> float:
            return normalize(code, axes[code].value, centered) if code in axes else 0.0

        steer = initial(ecodes.ABS_X, True)
        throttle = initial(ecodes.ABS_RZ, False)
        brake = initial(ecodes.ABS_Z, False)
        buttons: set[str] = set()
        self._set(GamepadState(True, steer, throttle, brake, frozenset(buttons), device.name))

        while not self._stop.is_set():
            # Short select timeout so the stop flag is noticed promptly.
            if not _wait_readable(device, 0.1):
                continue
            for event in _drain(device):
                if event.type == ecodes.EV_ABS:
                    if event.code == ecodes.ABS_X:
                        steer = normalize(event.code, event.value, True)
                    elif event.code == ecodes.ABS_RZ:
                        throttle = normalize(event.code, event.value, False)
                    elif event.code == ecodes.ABS_Z:
                        brake = normalize(event.code, event.value, False)
                    elif event.code == ecodes.ABS_HAT0Y:
                        buttons.discard(LIMIT_UP)
                        buttons.discard(LIMIT_DOWN)
                        if event.value < 0:
                            buttons.add(LIMIT_UP)
                        elif event.value > 0:
                            buttons.add(LIMIT_DOWN)
                elif event.type == ecodes.EV_KEY:
                    if event.code == DIGITAL_THROTTLE and ecodes.ABS_RZ not in axes:
                        throttle = 1.0 if event.value else 0.0
                    elif event.code == DIGITAL_BRAKE and ecodes.ABS_Z not in axes:
                        brake = 1.0 if event.value else 0.0
                    elif event.code in BUTTONS:
                        name = BUTTONS[event.code]
                        if event.value:
                            buttons.add(name)
                        else:
                            buttons.discard(name)
            self._set(GamepadState(True, steer, throttle, brake, frozenset(buttons), device.name))


def _wait_readable(device: evdev.InputDevice, timeout: float) -> bool:
    readable, _, _ = select.select([device.fd], [], [], timeout)
    return bool(readable)


def _drain(device: evdev.InputDevice):
    while True:
        event = device.read_one()
        if event is None:
            return
        yield event


def probe(path: Optional[str] = None) -> None:
    """Prints raw controller events so the mapping can be checked by hand."""
    path = path or find_gamepad()
    if path is None:
        print("no controller found")
        return
    device = evdev.InputDevice(path)
    print(f"{path}: {device.name}")
    for code, info in device.capabilities().get(ecodes.EV_ABS, []):
        print(f"  {ecodes.ABS[code]:12} range {info.min}..{info.max} rest {info.value}")
    try:
        _check_triggers(device)
        print("triggers look OK")
    except UnsupportedController as error:
        print(f"WARNING: {error}")
    print("move sticks / press buttons (Ctrl+C to stop)")
    try:
        for event in device.read_loop():
            if event.type in (ecodes.EV_ABS, ecodes.EV_KEY):
                names = ecodes.bytype[event.type].get(event.code, event.code)
                print(f"  {names}  {event.value}")
    except KeyboardInterrupt:
        pass
    finally:
        device.close()
