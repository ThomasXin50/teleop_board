#ifndef PID_H
#define PID_H

#include <stdbool.h>

typedef struct {
    float kp, ki, kd;
    float integrator;
    float prev_measurement;   
    bool  primed;             
    float out_min, out_max;
} pid_t;

void  pid_init(pid_t *p, float kp, float ki, float kd, float out_min, float out_max);

// clears integrator and derivative 
void  pid_reset(pid_t *p);

float pid_update(pid_t *p, float setpoint, float measurement, float dt);

#endif 