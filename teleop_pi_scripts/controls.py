"""Turns gamepad input into steering, speed and brake targets.

Button and axis names follow the Linux gamepad layout (by position), so the
same mapping works for Xbox, PlayStation, 8BitDo and Switch Pro controllers.
"""

from __future__ import annotations

from dataclasses import dataclass, field

ARM = "start"          # Start / Options / +
DISARM = "east"        # B (Xbox) / Circle (PS) / A (Nintendo) - right face button
LIMIT_UP = "dpad_up"
LIMIT_DOWN = "dpad_down"
# Bench firmware only: toggles the clutches. Back / Share / - (select button).
CLUTCH = "select"

STICK_DEADZONE = 0.08
TRIGGER_DEADZONE = 0.05
# Arming is refused unless the throttle trigger is released, so the vehicle
# never jumps forward the moment it is armed.
ARM_MAX_THROTTLE = TRIGGER_DEADZONE
# Matches the firmware's BRAKE_THROTTLE_INTERLOCK_PERCENT: any real brake
# input cuts throttle.
BRAKE_CUTS_THROTTLE_PERCENT = 5.0
MAX_BRAKE_PERCENT = 100.0

# Vibration cues shared by the gamepad readers: (milliseconds, strength 0..1)
# pulses, RUMBLE_GAP_S apart.
RUMBLE_PATTERNS = {
    "ready": ((150, 0.5), (150, 0.5)),  # connected and the RP2040 accepts commands
    "arm": ((400, 1.0),),
    "disarm": ((150, 0.35),),
}
RUMBLE_GAP_S = 0.15


@dataclass(frozen=True)
class GamepadState:
    connected: bool = False
    steer: float = 0.0      # left stick X: -1 full left .. +1 full right
    throttle: float = 0.0   # right trigger: 0 released .. 1 fully pressed
    brake: float = 0.0      # left trigger: 0 released .. 1 fully pressed
    buttons: frozenset = field(default_factory=frozenset)
    name: str = ""


@dataclass(frozen=True)
class Command:
    steering_deg: float
    speed_mph: float
    brake_percent: float
    # True: send the firmware a failsafe message instead of this command, so
    # it ramps the brake exactly as it does when comms drop.
    failsafe: bool = False


class RumbleCues:
    """Decides which vibration cue to play from state changes: "ready" once the
    controller is connected and the vehicle accepts commands, then "arm" and
    "disarm" as the arming state changes."""

    def __init__(self) -> None:
        self._ready = False
        self._armed = False

    def update(self, connected: bool, vehicle_ready: bool, armed: bool) -> list[str]:
        cues = []
        ready = connected and vehicle_ready
        if ready and not self._ready:
            cues.append("ready")
        if connected and armed != self._armed:
            cues.append("arm" if armed else "disarm")
        self._ready = ready
        self._armed = armed and connected
        return cues


def _deadzone(value: float, zone: float) -> float:
    magnitude = abs(value)
    if magnitude <= zone:
        return 0.0
    scaled = min(1.0, (magnitude - zone) / (1.0 - zone))
    return scaled if value > 0 else -scaled


class ControlMapper:
    def __init__(self, max_speed: float, max_steer: float, speed_step: float = 0.25,
                 invert_steering: bool = False) -> None:
        self.max_speed = max_speed
        self.max_steer = max_steer
        self.speed_step = speed_step
        self.steer_sign = -1.0 if invert_steering else 1.0
        self.speed_limit = min(1.0, max_speed)
        self.armed = False
        # Clutch request for the bench firmware, toggled by CLUTCH. Starts
        # engaged, which is what the firmware does on the first command.
        self.clutches_engaged = True
        self._previous_buttons: frozenset = frozenset()

    def disarm(self) -> None:
        self.armed = False

    def update(self, pad: GamepadState) -> Command:
        if not pad.connected:
            self.armed = False
            self._previous_buttons = frozenset()
            return Command(0.0, 0.0, 0.0, failsafe=True)

        pressed = pad.buttons - self._previous_buttons
        self._previous_buttons = pad.buttons
        throttle = _deadzone(max(0.0, pad.throttle), TRIGGER_DEADZONE)
        brake = _deadzone(max(0.0, pad.brake), TRIGGER_DEADZONE) * MAX_BRAKE_PERCENT

        if DISARM in pad.buttons:
            self.armed = False
        elif ARM in pressed and throttle <= ARM_MAX_THROTTLE:
            self.armed = True
        if LIMIT_UP in pressed:
            self.speed_limit = min(self.max_speed, self.speed_limit + self.speed_step)
        if LIMIT_DOWN in pressed:
            self.speed_limit = max(self.speed_step, self.speed_limit - self.speed_step)
        if CLUTCH in pressed:
            self.clutches_engaged = not self.clutches_engaged

        # Positive steering angle is a left turn (see ackermann.c), and pushing
        # the stick left gives a negative X.
        steering = -self.steer_sign * _deadzone(pad.steer, STICK_DEADZONE) * self.max_steer

        # Brake works whether or not the controller is armed.
        if not self.armed or brake > BRAKE_CUTS_THROTTLE_PERCENT:
            return Command(steering, 0.0, brake)
        return Command(steering, throttle * self.speed_limit, brake)
