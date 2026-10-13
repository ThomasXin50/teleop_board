#ifndef HAL_H
#define HAL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "pico/types.h"

void hal_init(void);
uint32_t hal_time_us(void);
void hal_sleep_us(uint32_t duration_us);
float hal_adc_voltage(uint input_pin, uint input_channel);

void hal_throttle_pwm_set(float duty_fraction);
void hal_clutches_set(bool engaged);
void hal_stepper_init(uint step_pin, uint dir_pin, uint enable_pin);
void hal_stepper_set_rate(uint step_pin, uint dir_pin, float steps_per_second);
void hal_stepper_stop(uint step_pin);

int hal_usb_read_byte(void);
int hal_usb_connected(void);
void hal_usb_write(const uint8_t *data, size_t length);

#endif