//My notes to look at while wrting this code (I've never done ackermann before lol)

/*
All about inner and outer wheel having a slightly different angle+rpm to function properly
You may be able to tell that my brain loses functionality past 3 am in this file, ngl
Let us define some variables

L=wheelbase len
T=width between two wheels
R=Turn rad at middle of car

\tan\delta_inner = \frac{L}{R-T/2}
\tan \delta_outer = \frac{L}{R+T/2}

But as I understand...
Each wheel does not have unique angle so we use bicycle model

R = \frac{L}{\tan(\delta)}
+steer = left


One worry is the amount of float computations since RP2040 has no FPU
*/
#include "ackermann.h"
//Maybe too slow?
#include <math.h>

//Conversion constant
#define DEG_TO_RAD  (3.14159265358979f / 180.0f)

//Claude recommended I have a minimum beyond which it is considered as going straight

#define TAN_EPS     1.0e-4f

float rpm_to_mps(float rpm) {
    return (rpm / 60.0f) * WHEEL_CIRCUM_M;
}

float radius_from_angle(float steer_deg) {
    const float t = tanf(steer_deg * DEG_TO_RAD);
    if (fabsf(t) < TAN_EPS) {
        return (steer_deg < 0.0f) ? -ACKERMANN_R_STRAIGHT:ACKERMANN_R_STRAIGHT;
    }
    return WHEELBASE_M / t;
}

float curvature_from_angle(float steer_deg) {
    return tanf(steer_deg * DEG_TO_RAD) / WHEELBASE_M;
}

//NGL, Yaw is fully CLaude's here
float yaw_rate(float steer_deg, float speed_mps) {
    return speed_mps * curvature_from_angle(steer_deg);
}

float mps_to_rpm(float mps) {
    return (mps / WHEEL_CIRCUM_M) * 60.0f;
}

wheel_speeds_t ack_wheel_speeds(float steer_deg, float speed_mps) {
    const float k = (TRACK_WIDTH_M * tanf(steer_deg * DEG_TO_RAD))/ (2.0f * WHEELBASE_M);
    wheel_speeds_t w;
    //This is non-actionable so just for data I gues ssince both wheels share one steering/input and only one PWM channel
    w.rear_left_mps  = speed_mps * (1.0f - k);
    w.rear_right_mps = speed_mps * (1.0f + k);
    return w;
}

float ack_speed_from_wheels(float rear_left_mps, float rear_right_mps) {
    return 0.5f * (rear_left_mps + rear_right_mps);
}
