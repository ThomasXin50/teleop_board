#include "pid.h"

#include <math.h>

void pid_init(pid_t *p, float kp, float ki, float kd, float out_min, float out_max) {
    p->kp = kp;
    p->ki = ki;
    p->kd = kd;
    p->out_min = out_min;
    p->out_max = out_max;
    p->integrator       = 0.0f;
    p->prev_measurement = 0.0f;
    p->primed           = false;
}
void pid_reset(pid_t *p) {
    p->integrator = 0.0f;
    p->prev_measurement = 0.0f;
    p->primed = false;
}

float pid_update(pid_t *p, float setpoint, float measurement, float dt) {
    if (!isfinite(setpoint) || !isfinite(measurement) || !isfinite(dt) || dt <= 0.0f) {
        return 0.0f;
    }

    const float error = setpoint - measurement;
    const float derivative = p->primed
        ? (p->prev_measurement - measurement) / dt
        : 0.0f;
    const float candidate_integrator = p->integrator + error * dt;
    const float candidate_output = p->kp * error
                                 + p->ki * candidate_integrator
                                 + p->kd * derivative;

    if ((candidate_output >= p->out_min && candidate_output <= p->out_max)
        || (candidate_output > p->out_max && error < 0.0f)
        || (candidate_output < p->out_min && error > 0.0f)) {
        p->integrator = candidate_integrator;
    }

    p->prev_measurement = measurement;
    p->primed = true;

    float output = p->kp * error + p->ki * p->integrator + p->kd * derivative;
    if (output > p->out_max) output = p->out_max;
    if (output < p->out_min) output = p->out_min;
    return output;
}