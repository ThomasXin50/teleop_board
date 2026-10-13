
#ifndef ACKERMANN_H
#define ACKERMANN_H

#include "config.h"

typedef struct {
    float rear_left_mps;
    float rear_right_mps;
} wheel_speeds_t;

#define ACKERMANN_R_STRAIGHT   1.0e6f

float rpm_to_mps(float rpm);
float mps_to_rpm(float mps);

float radius_from_angle(float steer_deg);

// 1/R
float curvature_from_angle(float steer_deg);


float yaw_rate(float steer_deg, float speed_mps);


wheel_speeds_t ack_wheel_speeds(float steer_deg, float speed_mps);

float ack_speed_from_wheels(float rear_left_mps, float rear_right_mps);
#endif