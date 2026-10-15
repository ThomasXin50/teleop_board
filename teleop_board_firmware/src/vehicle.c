#include "vehicle.h"

#include <math.h>
#include <string.h>

#include "ackermann.h"
#include "hal.h"
#include "pid.h"
#include "rpm.h"
#include "stepper.h"

#define METERS_PER_SECOND_TO_MPH 2.23693629f

// One stepper-driven position axis (steering or brake) and its protections.
typedef struct {
	pid_t pid;
	stepper_axis_t stepper;
	bool calibrated;
	float step_sign;
	float min, max;               // calibrated travel, in reading units
	float overtravel;             // hard margin beyond min/max
	float progress_check_steps;   // steps between no-progress checks
	float progress_min;           // reading change required per check
	uint16_t overtravel_fault;    // status bits latched on a fault
	uint16_t progress_fault;

	uint16_t faults;              // latched until reset
	float last_rate;              // pre-sign steps/s set on the last update
	bool was_beyond;
	int direction;                // direction being checked, 0 when idle
	float checkpoint;             // reading at the start of this check
	float steps_since_checkpoint;
} axis_t;

static vehicle_state_t state;
static pid_t speed_pid;
static uint32_t last_message_us;
static uint32_t last_speed_update_us;
static uint32_t last_steer_update_us;
static uint32_t last_brake_update_us;
static uint16_t last_command_sequence;
static bool command_seen;
static bool failsafe_requested;
static bool failsafe_active;
static uint32_t failsafe_start_us;
static float failsafe_start_brake;
// Bench firmware only: clutch messages from the host can release the clutches
// after the first command. Always true in the vehicle firmware.
static bool clutches_requested = true;
static bool clutches_engaged;

static axis_t steering = {
	.stepper = STEPPER_STEERING,
	.calibrated = STEER_CALIBRATED != 0u,
	.step_sign = STEER_STEP_SIGN,
	.min = STEER_ANGLE_MIN_DEG,
	.max = STEER_ANGLE_MAX_DEG,
	.overtravel = STEER_OVERTRAVEL_DEG,
	.progress_check_steps = STEER_PROGRESS_CHECK_DEG * STEER_STEPS_PER_DEG,
	.progress_min = STEER_PROGRESS_MIN_DEG,
	.overtravel_fault = VEHICLE_STATUS_STEER_OVERTRAVEL,
	.progress_fault = VEHICLE_STATUS_STEER_NO_PROGRESS,
};

static axis_t brake = {
	.stepper = STEPPER_BRAKE,
	.calibrated = BRAKE_CALIBRATED != 0u,
	.step_sign = BRAKE_STEP_SIGN,
	.min = 0.0f,
	.max = BRAKE_PERCENT_DENOMINATOR,
	.overtravel = BRAKE_OVERTRAVEL_PERCENT,
	.progress_check_steps = BRAKE_PROGRESS_CHECK_PERCENT * BRAKE_STEPS_PER_PERCENT,
	.progress_min = BRAKE_PROGRESS_MIN_PERCENT,
	.overtravel_fault = VEHICLE_STATUS_BRAKE_OVERTRAVEL,
	.progress_fault = VEHICLE_STATUS_BRAKE_NO_PROGRESS,
};

static float clampf(float value, float low, float high) {
	if (value < low) return low;
	if (value > high) return high;
	return value;
}

// Not clamped: past the calibrated travel the reading still tells the
// position loop which way leads back inside it.
static float sensor_value(float voltage, float voltage_at_low, float voltage_at_high,
						  float value_low, float value_high) {
	if (voltage_at_high == voltage_at_low) return value_low;
	return (voltage - voltage_at_low) * (value_high - value_low)
		 / (voltage_at_high - voltage_at_low) + value_low;
}

static bool axis_beyond(const axis_t *axis, float measured) {
	return measured > axis->max + axis->overtravel
		|| measured < axis->min - axis->overtravel;
}

// Stops stepping (the clutch keeps holding) and restarts the progress check.
static void axis_hold(axis_t *axis, float measured) {
	pid_reset(&axis->pid);
	stepper_stop(axis->stepper);
	axis->last_rate = 0.0f;
	axis->direction = 0;
	axis->steps_since_checkpoint = 0.0f;
	axis->was_beyond = axis_beyond(axis, measured);
}

static void axis_fault(axis_t *axis, uint16_t fault, float measured) {
	axis->faults |= fault;
	axis_hold(axis, measured);
}

// Steps one axis toward its target. A positive PID output is assumed to raise
// the reading. Invalid readings hold still; overtravel and no-progress latch a
// fault that holds the axis until reset.
static void axis_update(axis_t *axis, float target, float measured, bool sensor_ok,
						float dt) {
	if (axis->faults != 0u || !axis->calibrated || !sensor_ok) {
		axis_hold(axis, measured);
		return;
	}

	// Overtravel: driven past the margin while stepping. Already being out
	// there (moved by hand while the clutch was released) is not a fault.
	const bool beyond = axis_beyond(axis, measured);
	if (beyond && !axis->was_beyond && axis->last_rate != 0.0f) {
		axis_fault(axis, axis->overtravel_fault, measured);
		return;
	}
	axis->was_beyond = beyond;

	// No progress: count the steps issued since the checkpoint (at the rate set
	// last update) and require the reading to have followed them.
	if (axis->direction != 0) {
		axis->steps_since_checkpoint += fabsf(axis->last_rate) * dt;
		if (axis->steps_since_checkpoint >= axis->progress_check_steps) {
			const float moved = (measured - axis->checkpoint) * (float)axis->direction;
			if (moved < axis->progress_min) {
				axis_fault(axis, axis->progress_fault, measured);
				return;
			}
			axis->checkpoint = measured;
			axis->steps_since_checkpoint = 0.0f;
		}
	}

	float rate = pid_update(&axis->pid, target, measured, dt);
	// Soft limit: at the calibrated travel, only step back inside it.
	if ((measured >= axis->max && rate > 0.0f) || (measured <= axis->min && rate < 0.0f)) {
		rate = 0.0f;
	}
	if (fabsf(rate) < (float)STEPPER_MIN_HZ) rate = 0.0f;

	const int direction = (rate > 0.0f) - (rate < 0.0f);
	if (direction != axis->direction) {
		axis->direction = direction;
		axis->checkpoint = measured;
		axis->steps_since_checkpoint = 0.0f;
	}
	axis->last_rate = rate;
	stepper_set_rate(axis->stepper, axis->step_sign * rate);
}

static void update_steering(uint32_t now_us, bool sensor_ok) {
	if ((uint32_t)(now_us - last_steer_update_us) < STEER_PERIOD_US) return;
	last_steer_update_us = now_us;
	axis_update(&steering, state.target_steering_deg, state.steering_deg, sensor_ok,
				1.0f / (float)STEER_LOOP_HZ);
}

// Bench mode: the commanded angle sets the step rate, 0 at center up to
// BENCH_STEER_MAX_STEPS_PER_S at full travel. No sensor feedback, limits or
// progress checks.
static void update_steering_open_loop(uint32_t now_us) {
	if ((uint32_t)(now_us - last_steer_update_us) < STEER_PERIOD_US) return;
	last_steer_update_us = now_us;
	float rate = clampf(state.target_steering_deg / STEER_ANGLE_MAX_DEG, -1.0f, 1.0f)
			   * BENCH_STEER_MAX_STEPS_PER_S;
	if (fabsf(rate) < (float)STEPPER_MIN_HZ) rate = 0.0f;
	steering.last_rate = rate;
	stepper_set_rate(steering.stepper, steering.step_sign * rate);
}

static void update_brake(uint32_t now_us, bool sensor_ok) {
	if ((uint32_t)(now_us - last_brake_update_us) < BRAKE_PERIOD_US) return;
	last_brake_update_us = now_us;
	axis_update(&brake, state.target_brake_percent, state.brake_percent, sensor_ok,
				1.0f / (float)BRAKE_LOOP_HZ);
}

void vehicle_init(void) {
	memset(&state, 0, sizeof(state));
	pid_init(&speed_pid, SPEED_PID_KP, SPEED_PID_KI, SPEED_PID_KD,
			 SPEED_PID_OUT_MIN, SPEED_PID_OUT_MAX);
	pid_init(&steering.pid, STEER_PID_KP, STEER_PID_KI, STEER_PID_KD,
			 STEER_PID_OUT_MIN, STEER_PID_OUT_MAX);
	pid_init(&brake.pid, BRAKE_PID_KP, BRAKE_PID_KI, BRAKE_PID_KD,
			 BRAKE_PID_OUT_MIN, BRAKE_PID_OUT_MAX);
	stepper_init();
	hal_clutches_set(false);
	hal_throttle_pwm_set(0.0f);
}

bool vehicle_accept_packet(const proto_packet_t *packet, uint32_t now_us) {
	const bool is_command = packet->type == PROTO_TYPE_COMMAND
						 && packet->length == PROTO_CMD_PAYLOAD;
	// A failsafe message keeps the link alive but asks for the same braking as
	// a comms timeout. Its reason byte is informational.
	const bool is_failsafe = packet->type == PROTO_TYPE_FAILSAFE
						  && packet->length == PROTO_FAILSAFE_PAYLOAD;
	// Clutch messages are honored by the bench build only, and don't count as
	// commands: they neither engage the clutches nor keep the link alive.
	const bool is_clutch = BENCH_MODE
						&& packet->type == PROTO_TYPE_CLUTCH
						&& packet->length == PROTO_CLUTCH_PAYLOAD;
	if (!is_command && !is_failsafe && !is_clutch) return false;
	// Commands, failsafe and clutch messages share one sequence counter.
	if (command_seen && (int16_t)(packet->sequence - last_command_sequence) <= 0) {
		return false;
	}
	last_command_sequence = packet->sequence;
	if (is_clutch) {
		clutches_requested = packet->payload[0] != 0u;
		return true;
	}
	last_message_us = now_us;
	command_seen = true;
	failsafe_requested = is_failsafe;
	if (is_failsafe) return true;

	const uint16_t angle_raw = (uint16_t)packet->payload[0]
							 | ((uint16_t)packet->payload[1] << 8);
	const int16_t angle_centi_deg = (int16_t)angle_raw;
	const uint16_t speed_centi_mph = (uint16_t)packet->payload[2]
								   | ((uint16_t)packet->payload[3] << 8);
	const uint16_t brake_centi_percent = (uint16_t)packet->payload[4]
									   | ((uint16_t)packet->payload[5] << 8);
	state.target_steering_deg = clampf((float)angle_centi_deg / 100.0f,
									   STEER_ANGLE_MIN_DEG,
									   STEER_ANGLE_MAX_DEG);
	state.target_speed_mph = clampf((float)speed_centi_mph / 100.0f,
									0.0f, SPEED_COMMAND_MAX_MPH);
	state.target_brake_percent = clampf((float)brake_centi_percent / 100.0f,
										0.0f, BRAKE_PERCENT_DENOMINATOR);
	return true;
}

void vehicle_update(uint32_t now_us) {
	const bool link_active = command_seen
		&& (uint32_t)(now_us - last_message_us) <= USB_COMMAND_TIMEOUT_US;
	const bool command_active = link_active && !failsafe_requested;
	rpm_read_all(state.wheel_rpm, now_us);

	const float rear_average_rpm = 0.5f * (state.wheel_rpm[WHEEL_BR]
										  + state.wheel_rpm[WHEEL_BL]);
	state.speed_mph = rpm_to_mps(rear_average_rpm) * METERS_PER_SECOND_TO_MPH;

	state.steering_voltage = hal_adc_voltage(PIN_ADC_STEER, ADC_CH_STEER);
	state.brake_voltage = hal_adc_voltage(PIN_ADC_BRAKE, ADC_CH_BRAKE);
	const bool steering_sensor_ok = state.steering_voltage >= STEER_SENSOR_FAULT_V_MIN
								 && state.steering_voltage <= STEER_SENSOR_FAULT_V_MAX;
	const bool brake_sensor_ok = state.brake_voltage >= BRAKE_SENSOR_FAULT_V_MIN
							  && state.brake_voltage <= BRAKE_SENSOR_FAULT_V_MAX;

	state.steering_deg = sensor_value(state.steering_voltage,
									  STEER_SENSOR_V_AT_MIN, STEER_SENSOR_V_AT_MAX,
									  STEER_ANGLE_MIN_DEG, STEER_ANGLE_MAX_DEG);
	state.brake_percent = sensor_value(state.brake_voltage,
									   BRAKE_SENSOR_V_RELEASED, BRAKE_SENSOR_V_FULL,
									   0.0f, BRAKE_PERCENT_DENOMINATOR);
	const bool steer_at_limit = state.steering_deg >= STEER_ANGLE_MAX_DEG
							 || state.steering_deg <= STEER_ANGLE_MIN_DEG;
	const bool brake_at_limit = state.brake_percent >= BRAKE_PERCENT_DENOMINATOR
							 || state.brake_percent <= 0.0f;

	// The clutches are the only thing holding the steering wheel and brake
	// pedal. Released until the first command so both can be moved by hand;
	// from then on engaged until reset, including after comms are lost. The
	// bench build can also release them on request (clutch messages).
	clutches_engaged = command_seen && clutches_requested;
	hal_clutches_set(clutches_engaged);

	if (!command_active) {
		state.target_speed_mph = 0.0f;
		state.target_steering_deg = clampf(state.steering_deg,
										   STEER_ANGLE_MIN_DEG, STEER_ANGLE_MAX_DEG);
		state.throttle_duty_percent = 0.0f;
		pid_reset(&speed_pid);
		hal_throttle_pwm_set(0.0f);
		axis_hold(&steering, state.steering_deg);

		if (!command_seen) {
			state.target_brake_percent = clampf(state.brake_percent,
												0.0f, BRAKE_PERCENT_DENOMINATOR);
			axis_hold(&brake, state.brake_percent);
		} else {
			// Comms lost or failsafe requested: ramp the brake to the failsafe
			// level, starting from whichever is higher so braking already
			// applied is never eased off.
			if (!failsafe_active) {
				failsafe_active = true;
				failsafe_start_us = now_us;
				failsafe_start_brake = clampf(
					fmaxf(state.target_brake_percent, state.brake_percent),
					0.0f, BRAKE_FAILSAFE_PERCENT);
			}
			const float ramp = clampf((float)(uint32_t)(now_us - failsafe_start_us)
									  / (float)BRAKE_FAILSAFE_RAMP_US, 0.0f, 1.0f);
			state.target_brake_percent = failsafe_start_brake
				+ (BRAKE_FAILSAFE_PERCENT - failsafe_start_brake) * ramp;
			// Latch once complete so the 32-bit timer wrapping can't restart the ramp.
			if (ramp >= 1.0f) failsafe_start_brake = BRAKE_FAILSAFE_PERCENT;
			update_brake(now_us, brake_sensor_ok);
		}
	} else {
		failsafe_active = false;

		if ((uint32_t)(now_us - last_speed_update_us) >= SPEED_PERIOD_US) {
			last_speed_update_us = now_us;
			// Never drive against the brake, or without working brake and
			// steering: invalid readings block throttle until they recover,
			// latched faults until reset, and an uncalibrated axis always.
			// The bench build keeps only the brake interlock (and the comms
			// timeout above).
			const bool throttle_blocked =
				state.target_brake_percent > BRAKE_THROTTLE_INTERLOCK_PERCENT
				|| (!BENCH_MODE
					&& (!brake.calibrated
						|| !brake_sensor_ok
						|| brake.faults != 0u
						|| !steering.calibrated
						|| !steering_sensor_ok
						|| steering.faults != 0u));
			const float target_speed = throttle_blocked ? 0.0f : state.target_speed_mph;
			float duty = speed_mph_to_pwm_percent(target_speed) / 100.0f;
			// On the bench the wheels don't report speed, so the PID would wind
			// up; output the plain mph-to-duty map instead.
			if (target_speed > 0.0f && !BENCH_MODE) {
				duty += pid_update(&speed_pid, target_speed,
								   state.speed_mph, SPEED_DT_S);
			} else {
				pid_reset(&speed_pid);
			}
			duty = clampf(duty, 0.0f, 1.0f);
			state.throttle_duty_percent = duty * 100.0f;
			hal_throttle_pwm_set(duty);
		}

		if (BENCH_MODE) {
			update_steering_open_loop(now_us);
		} else {
			update_steering(now_us, steering_sensor_ok);
		}
		update_brake(now_us, brake_sensor_ok);
	}

	state.status = (command_active ? VEHICLE_STATUS_COMMAND_ACTIVE : 0u)
				 | (steering_sensor_ok ? VEHICLE_STATUS_STEER_SENSOR_OK : 0u)
				 | (brake_sensor_ok ? VEHICLE_STATUS_BRAKE_SENSOR_OK : 0u)
				 | (steer_at_limit ? VEHICLE_STATUS_STEER_AT_LIMIT : 0u)
				 | (brake_at_limit ? VEHICLE_STATUS_BRAKE_AT_LIMIT : 0u)
				 | (failsafe_active ? VEHICLE_STATUS_FAILSAFE_BRAKING : 0u)
				 | (link_active ? VEHICLE_STATUS_HOST_LINK_ACTIVE : 0u)
				 | ((link_active && failsafe_requested) ? VEHICLE_STATUS_FAILSAFE_REQUESTED : 0u)
				 | steering.faults
				 | brake.faults
				 | (steering.calibrated ? 0u : VEHICLE_STATUS_STEER_UNCALIBRATED)
				 | (brake.calibrated ? 0u : VEHICLE_STATUS_BRAKE_UNCALIBRATED)
				 | (BENCH_MODE ? VEHICLE_STATUS_BENCH_MODE : 0u)
				 | (clutches_engaged ? VEHICLE_STATUS_CLUTCHES_ENGAGED : 0u);
}

const vehicle_state_t *vehicle_state(void) {
	return &state;
}
