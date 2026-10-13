#ifndef STEPPER_H
#define STEPPER_H

typedef enum {
    STEPPER_STEERING = 0,
    STEPPER_BRAKE = 1,
} stepper_axis_t;

void stepper_init(void);
void stepper_set_rate(stepper_axis_t axis, float steps_per_second);
void stepper_stop(stepper_axis_t axis);

#endif