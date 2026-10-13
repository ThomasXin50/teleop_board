#include "stepper.h"

#include "config.h"
#include "hal.h"

static const uint step_pins[STEPPER_COUNT] = {PIN_STEER_PULSE, PIN_BRAKE_PULSE};
static const uint dir_pins[STEPPER_COUNT] = {PIN_STEER_DIR, PIN_BRAKE_DIR};
static const uint enable_pins[STEPPER_COUNT] = {PIN_STEER_ENABLE, PIN_BRAKE_ENABLE};

void stepper_init(void) {
    for (uint axis = 0; axis < STEPPER_COUNT; ++axis) {
        hal_stepper_init(step_pins[axis], dir_pins[axis], enable_pins[axis]);
    }
}

void stepper_set_rate(stepper_axis_t axis, float steps_per_second) {
    if ((uint)axis >= STEPPER_COUNT) return;
    hal_stepper_set_rate(step_pins[axis], dir_pins[axis], steps_per_second);
}

void stepper_stop(stepper_axis_t axis) {
    if ((uint)axis >= STEPPER_COUNT) return;
    hal_stepper_stop(step_pins[axis]);
}