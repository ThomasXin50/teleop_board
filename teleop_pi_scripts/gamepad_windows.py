"""Windows XInput gamepad reader, a drop-in for gamepad.py off the Pi.

XInput covers Xbox controllers and anything in X-input mode (8BitDo, most
generic pads). PlayStation and Switch Pro controllers only show up through a
mapper such as Steam Input or DS4Windows. Uses ctypes, so nothing to install.
"""

from __future__ import annotations

import ctypes
import threading
import time
from ctypes import wintypes
from typing import Optional

from controls import (ARM, CLUTCH, DISARM, LIMIT_DOWN, LIMIT_UP, RUMBLE_GAP_S,
                      RUMBLE_PATTERNS, GamepadState)

XINPUT_SLOTS = 4
ERROR_SUCCESS = 0
POLL_S = 0.005
RECONNECT_S = 1.0

BUTTONS = {
    0x0010: ARM,         # XINPUT_GAMEPAD_START
    0x2000: DISARM,      # XINPUT_GAMEPAD_B (right face button)
    0x0001: LIMIT_UP,    # XINPUT_GAMEPAD_DPAD_UP
    0x0002: LIMIT_DOWN,  # XINPUT_GAMEPAD_DPAD_DOWN
    0x0020: CLUTCH,      # XINPUT_GAMEPAD_BACK
}
BUTTON_NAMES = {
    0x0001: "DPAD_UP", 0x0002: "DPAD_DOWN", 0x0004: "DPAD_LEFT", 0x0008: "DPAD_RIGHT",
    0x0010: "START", 0x0020: "BACK", 0x0040: "LEFT_THUMB", 0x0080: "RIGHT_THUMB",
    0x0100: "LEFT_SHOULDER", 0x0200: "RIGHT_SHOULDER",
    0x1000: "A", 0x2000: "B", 0x4000: "X", 0x8000: "Y",
}


class XINPUT_GAMEPAD(ctypes.Structure):
    _fields_ = [("wButtons", wintypes.WORD),
                ("bLeftTrigger", ctypes.c_ubyte),
                ("bRightTrigger", ctypes.c_ubyte),
                ("sThumbLX", ctypes.c_short),
                ("sThumbLY", ctypes.c_short),
                ("sThumbRX", ctypes.c_short),
                ("sThumbRY", ctypes.c_short)]


class XINPUT_STATE(ctypes.Structure):
    _fields_ = [("dwPacketNumber", wintypes.DWORD), ("Gamepad", XINPUT_GAMEPAD)]


class XINPUT_VIBRATION(ctypes.Structure):
    _fields_ = [("wLeftMotorSpeed", wintypes.WORD), ("wRightMotorSpeed", wintypes.WORD)]


def _load_xinput():
    for name in ("xinput1_4", "xinput1_3", "xinput9_1_0"):
        try:
            return ctypes.WinDLL(name)
        except OSError:
            continue
    raise OSError("XInput is not available on this system")


_xinput = _load_xinput()


def _read_slot(slot: int) -> Optional[XINPUT_GAMEPAD]:
    state = XINPUT_STATE()
    if _xinput.XInputGetState(slot, ctypes.byref(state)) != ERROR_SUCCESS:
        return None
    return state.Gamepad


def _vibrate(slot: int, strength: float) -> None:
    level = int(0xFFFF * max(0.0, min(1.0, strength)))
    vibration = XINPUT_VIBRATION(level, level)
    _xinput.XInputSetState(slot, ctypes.byref(vibration))


def find_gamepad() -> Optional[int]:
    for slot in range(XINPUT_SLOTS):
        if _read_slot(slot) is not None:
            return slot
    return None


def _parse_slot(path: Optional[str]) -> Optional[int]:
    if path is None:
        return None
    slot = int(path)
    if not 0 <= slot < XINPUT_SLOTS:
        raise ValueError(f"XInput slot must be 0-{XINPUT_SLOTS - 1}, got {path}")
    return slot


def _to_state(pad: XINPUT_GAMEPAD, slot: int) -> GamepadState:
    # Same conventions as gamepad.py: stick -1 full left .. +1 full right,
    # triggers 0 released .. 1 fully pressed.
    steer = max(-1.0, min(1.0, pad.sThumbLX / 32767.0))
    buttons = frozenset(name for mask, name in BUTTONS.items() if pad.wButtons & mask)
    return GamepadState(True, steer, pad.bRightTrigger / 255.0, pad.bLeftTrigger / 255.0,
                        buttons, f"XInput controller {slot}")


class Gamepad:
    """Polls an XInput controller on a background thread; state() is always
    safe to call. path is an XInput slot ("0"-"3"), default the first found."""

    def __init__(self, path: Optional[str] = None) -> None:
        self._slot_wanted = _parse_slot(path)
        self._lock = threading.Lock()
        self._state = GamepadState()
        self._slot: Optional[int] = None
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._rumble_lock = threading.Lock()
        self.error: Optional[str] = None

    def __enter__(self) -> "Gamepad":
        self._thread.start()
        return self

    def __exit__(self, *exc) -> None:
        self._stop.set()
        self._thread.join(timeout=1.0)
        with self._lock:
            slot = self._slot
        if slot is not None:
            _vibrate(slot, 0.0)

    def state(self) -> GamepadState:
        with self._lock:
            return self._state

    def rumble(self, pattern: str) -> None:
        """Plays a RUMBLE_PATTERNS cue without blocking."""
        pulses = RUMBLE_PATTERNS[pattern]
        threading.Thread(target=self._play, args=(pulses,), daemon=True).start()

    def _play(self, pulses) -> None:
        with self._rumble_lock:
            for duration_ms, strength in pulses:
                with self._lock:
                    slot = self._slot
                if slot is None:
                    return
                _vibrate(slot, strength)
                time.sleep(duration_ms / 1000.0)
                _vibrate(slot, 0.0)
                time.sleep(RUMBLE_GAP_S)

    def _run(self) -> None:
        while not self._stop.is_set():
            slot = self._slot_wanted if self._slot_wanted is not None else find_gamepad()
            if slot is None or _read_slot(slot) is None:
                self.error = "no controller found"
                self._stop.wait(RECONNECT_S)
                continue
            self.error = None
            with self._lock:
                self._slot = slot
            self._poll(slot)
            with self._lock:
                self._slot = None
                self._state = GamepadState()
            if not self._stop.is_set():
                self.error = "controller lost"
                self._stop.wait(RECONNECT_S)

    def _poll(self, slot: int) -> None:
        """Returns when the controller disconnects or on stop."""
        while not self._stop.is_set():
            pad = _read_slot(slot)
            if pad is None:
                return
            state = _to_state(pad, slot)
            with self._lock:
                self._state = state
            self._stop.wait(POLL_S)


def probe(path: Optional[str] = None) -> None:
    """Prints controller input changes so the mapping can be checked by hand."""
    slot = _parse_slot(path)
    if slot is None:
        slot = find_gamepad()
    if slot is None:
        print("no XInput controller found (PlayStation/Switch pads need Steam Input "
              "or DS4Windows)")
        return
    print(f"XInput slot {slot}: move sticks / press buttons (Ctrl+C to stop)")
    previous = None
    try:
        while True:
            pad = _read_slot(slot)
            if pad is None:
                print("controller disconnected")
                return
            names = [n for mask, n in BUTTON_NAMES.items() if pad.wButtons & mask]
            current = (f"LX {pad.sThumbLX:6d}  LY {pad.sThumbLY:6d}  "
                       f"LT {pad.bLeftTrigger:3d}  RT {pad.bRightTrigger:3d}  "
                       f"buttons {' '.join(names) or '-'}")
            if current != previous:
                print("  " + current)
                previous = current
            time.sleep(0.02)
    except KeyboardInterrupt:
        pass
