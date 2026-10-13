#ifndef VEHICLE_H
#define VEHICLE_H

#include <stdbool.h>
#include <stdint.h>

#include "config.h"
#include "protocol.h"

#define VEHICLE_STATUS_COMMAND_ACTIVE      0x0001u
#define VEHICLE_STATUS_STEER_SENSOR_OK     0x0002u
#define VEHICLE_STATUS_BRAKE_SENSOR_OK     0x0004u
#define VEHICLE_STATUS_STEER_AT_LIMIT      0x0008u
#define VEHICLE_STATUS_BRAKE_AT_LIMIT      0x0010u
#define VEHICLE_STATUS_FAILSAFE_BRAKING    0x0020u
// Latched until reset; the faulted axis holds still.
#define VEHICLE_STATUS_STEER_OVERTRAVEL    0x0040u
#define VEHICLE_STATUS_STEER_NO_PROGRESS   0x0080u
#define VEHICLE_STATUS_BRAKE_OVERTRAVEL    0x0100u
#define VEHICLE_STATUS_BRAKE_NO_PROGRESS   0x0200u
// *_CALIBRATED is 0 in config.h, so that axis never steps.
#define VEHICLE_STATUS_STEER_UNCALIBRATED  0x0400u
#define VEHICLE_STATUS_BRAKE_UNCALIBRATED  0x0800u
// A valid command or failsafe message arrived within USB_COMMAND_TIMEOUT_US.
#define VEHICLE_STATUS_HOST_LINK_ACTIVE    0x1000u
// The latest message was a failsafe request from the host.
#define VEHICLE_STATUS_FAILSAFE_REQUESTED  0x2000u
// Bench build: throttle interlocks off, steering open loop (see BENCH_MODE).
#define VEHICLE_STATUS_BENCH_MODE          0x4000u

typedef struct {
	float wheel_rpm[RPM_WHEEL_COUNT];
	float target_speed_mph;
	float speed_mph;
	float target_steering_deg;
	float steering_deg;
	float target_brake_percent;
	float brake_percent;
	float throttle_duty_percent;
	float steering_voltage;   // raw sensor volts, for calibration
	float brake_voltage;
	uint16_t status;
} vehicle_state_t;

void vehicle_init(void);
bool vehicle_accept_packet(const proto_packet_t *packet, uint32_t now_us);
void vehicle_update(uint32_t now_us);
const vehicle_state_t *vehicle_state(void);

#endif
