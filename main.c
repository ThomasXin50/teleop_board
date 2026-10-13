#include <math.h>
#include <stdint.h>

#include "config.h"
#include "hal.h"
#include "hardware/watchdog.h"
#include "protocol.h"
#include "rpm.h"
#include "vehicle.h"

static void write_u16_le(uint8_t *output, uint16_t value) {
	output[0] = (uint8_t)(value & 0xFFu);
	output[1] = (uint8_t)(value >> 8);
}

static uint16_t saturating_u16(float value) {
	if (!isfinite(value) || value <= 0.0f) return 0u;
	if (value >= 65535.0f) return 65535u;
	return (uint16_t)lroundf(value);
}

static uint8_t make_telemetry(uint8_t *payload) {
	const vehicle_state_t *state = vehicle_state();
	const float angle_scaled = state->steering_deg * 100.0f;
	int16_t angle = 0;
	if (isfinite(angle_scaled)) {
		if (angle_scaled > 32767.0f) angle = 32767;
		else if (angle_scaled < -32768.0f) angle = -32768;
		else angle = (int16_t)lroundf(angle_scaled);
	}

	write_u16_le(&payload[0], (uint16_t)angle);
	write_u16_le(&payload[2], saturating_u16(state->brake_percent * 100.0f));
	for (uint i = 0; i < RPM_WHEEL_COUNT; ++i) {
		write_u16_le(&payload[4u + 2u * i], saturating_u16(state->wheel_rpm[i]));
	}
	write_u16_le(&payload[12], saturating_u16(state->throttle_duty_percent * 100.0f));
	write_u16_le(&payload[14], state->status);
	write_u16_le(&payload[16], saturating_u16(state->steering_voltage * 1000.0f));
	write_u16_le(&payload[18], saturating_u16(state->brake_voltage * 1000.0f));
	return PROTO_TELEMETRY_PAYLOAD;
}

int main(void) {
	hal_init();
	rpm_init();
	vehicle_init();
	watchdog_enable(2000u, true);

	proto_rx_t receiver;
	proto_rx_init(&receiver);
	uint16_t telemetry_sequence = 0u;
	uint32_t last_telemetry_us = hal_time_us();
	uint32_t last_loop_us = hal_time_us();

	for (;;) {
		int input = hal_usb_read_byte();
		while (input >= 0) {
			proto_packet_t packet;
			if (proto_rx_push(&receiver, (uint8_t)input, &packet)) {
				vehicle_accept_packet(&packet, hal_time_us());
			}
			input = hal_usb_read_byte();
		}

		uint32_t now_us = hal_time_us();
		vehicle_update(now_us);

		if (hal_usb_connected()
			&& (uint32_t)(now_us - last_telemetry_us) >= TELEMETRY_PERIOD_US) {
			uint8_t payload[PROTO_TELEMETRY_PAYLOAD];
			uint8_t frame[PROTO_MAX_FRAME];
			const uint8_t payload_length = make_telemetry(payload);
			const size_t frame_length = proto_encode(PROTO_TYPE_TELEMETRY,
													  telemetry_sequence++,
													  payload, payload_length,
													  frame, sizeof(frame));
			if (frame_length != 0u) hal_usb_write(frame, frame_length);
			last_telemetry_us = now_us;
		}

		watchdog_update();
		now_us = hal_time_us();
		const uint32_t elapsed = now_us - last_loop_us;
		if (elapsed < CONTROL_PERIOD_US) hal_sleep_us(CONTROL_PERIOD_US - elapsed);
		last_loop_us = hal_time_us();
	}
}
