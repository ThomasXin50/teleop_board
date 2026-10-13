#include "hal.h"

#include <math.h>

#include "config.h"
#include "hardware/adc.h"
#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "pico/stdlib.h"
#include "pico/stdio.h"
#include "pico/stdio_usb.h"

static uint32_t stepper_rates[30];
static uint16_t throttle_wrap;

static void gpio_output(uint pin, bool initial_value) {
    gpio_init(pin);
    gpio_put(pin, initial_value);
    gpio_set_dir(pin, GPIO_OUT);
}

void hal_init(void) {
    stdio_init_all();
    adc_init();
    adc_gpio_init(PIN_ADC_BRAKE);
    adc_gpio_init(PIN_ADC_STEER);

    const uint throttle_slice = pwm_gpio_to_slice_num(PIN_PWM_BACK_LEFT);
    const uint throttle_channel_left = pwm_gpio_to_channel(PIN_PWM_BACK_LEFT);
    const uint throttle_channel_right = pwm_gpio_to_channel(PIN_PWM_BACK_RIGHT);
    const uint32_t wrap = (clock_get_hz(clk_sys) / PWM_FREQ_HZ) - 1u;
    throttle_wrap = (uint16_t)wrap;
    pwm_config throttle_config = pwm_get_default_config();
    pwm_config_set_clkdiv(&throttle_config, 1.0f);
    pwm_config_set_wrap(&throttle_config, throttle_wrap);

    gpio_set_function(PIN_PWM_BACK_LEFT, GPIO_FUNC_PWM);
    gpio_set_function(PIN_PWM_BACK_RIGHT, GPIO_FUNC_PWM);
    pwm_init(throttle_slice, &throttle_config, true);
    pwm_set_chan_level(throttle_slice, throttle_channel_left, 0u);
    pwm_set_chan_level(throttle_slice, throttle_channel_right, 0u);

    gpio_output(PIN_CLUTCH_STEER, CLUTCH_ENGAGED_LEVEL == 0u);
    gpio_output(PIN_CLUTCH_BRAKE, CLUTCH_ENGAGED_LEVEL == 0u);
}

void hal_clutches_set(bool engaged) {
    const bool level = engaged == (CLUTCH_ENGAGED_LEVEL != 0u);
    gpio_put(PIN_CLUTCH_STEER, level);
    gpio_put(PIN_CLUTCH_BRAKE, level);
}

uint32_t hal_time_us(void) {
    return time_us_32();
}

void hal_sleep_us(uint32_t duration_us) {
    sleep_us(duration_us);
}

float hal_adc_voltage(uint input_pin, uint input_channel) {
    (void)input_pin;
    float total = 0.0f;
    for (uint i = 0; i < ADC_FILTER_SAMPLES; ++i) {
        adc_select_input(input_channel);
        busy_wait_us(2u);
        total += (float)adc_read() * ADC_VOLTS_PER_COUNT;
    }
    return total / (float)ADC_FILTER_SAMPLES;
}

void hal_throttle_pwm_set(float duty_fraction) {
    if (!isfinite(duty_fraction) || duty_fraction < 0.0f) duty_fraction = 0.0f;
    if (duty_fraction > 1.0f) duty_fraction = 1.0f;

    const uint slice = pwm_gpio_to_slice_num(PIN_PWM_BACK_LEFT);
    const uint16_t level = (uint16_t)(duty_fraction * (float)(throttle_wrap + 1u));
    pwm_set_chan_level(slice, pwm_gpio_to_channel(PIN_PWM_BACK_LEFT), level);
    pwm_set_chan_level(slice, pwm_gpio_to_channel(PIN_PWM_BACK_RIGHT), level);
}

static void step_output_idle(uint step_pin) {
    pwm_set_enabled(pwm_gpio_to_slice_num(step_pin), false);
    // Latch the idle level before handing the pin back to SIO so it never
    // glitches to the opto-on level.
    gpio_put(step_pin, STEPPER_STEP_IDLE_LEVEL != 0u);
    gpio_set_function(step_pin, GPIO_FUNC_SIO);
    gpio_set_dir(step_pin, GPIO_OUT);
    if (step_pin < count_of(stepper_rates)) stepper_rates[step_pin] = 0u;
}

void hal_stepper_init(uint step_pin, uint dir_pin, uint enable_pin) {
    gpio_output(step_pin, STEPPER_STEP_IDLE_LEVEL != 0u);
    gpio_output(dir_pin, false);
    // Drivers stay enabled (holding torque) for as long as the firmware runs.
    gpio_output(enable_pin, STEPPER_ENABLE_ACTIVE != 0u);
    if (step_pin < count_of(stepper_rates)) stepper_rates[step_pin] = 0u;
}

void hal_stepper_stop(uint step_pin) {
    step_output_idle(step_pin);
}

void hal_stepper_set_rate(uint step_pin, uint dir_pin, float steps_per_second) {
    if (!isfinite(steps_per_second)) steps_per_second = 0.0f;
    if (steps_per_second > STEPPER_RATE_LIMIT_HZ) steps_per_second = STEPPER_RATE_LIMIT_HZ;
    if (steps_per_second < -STEPPER_RATE_LIMIT_HZ) steps_per_second = -STEPPER_RATE_LIMIT_HZ;

    if (fabsf(steps_per_second) < (float)STEPPER_MIN_HZ) {
        hal_stepper_stop(step_pin);
        return;
    }

    const bool direction = steps_per_second < 0.0f;
    const uint32_t rate = (uint32_t)fabsf(steps_per_second);
    const bool direction_changed = gpio_get(dir_pin) != direction;
    if (direction_changed) {
        // Halt the pulse train so no step edge falls inside the DIR setup time.
        step_output_idle(step_pin);
        gpio_put(dir_pin, direction);
        busy_wait_us(STEPPER_DIR_SETUP_US);
    }

    if (step_pin < count_of(stepper_rates) && stepper_rates[step_pin] == rate) return;

    const uint slice = pwm_gpio_to_slice_num(step_pin);
    const uint channel = pwm_gpio_to_channel(step_pin);
    const uint32_t system_hz = clock_get_hz(clk_sys);
    uint32_t divider = (system_hz + rate * 65535u - 1u) / (rate * 65535u);
    if (divider < 1u) divider = 1u;
    if (divider > 255u) divider = 255u;
    uint32_t period = system_hz / (divider * rate);
    if (period < 2u) period = 2u;
    if (period > 65536u) period = 65536u;

    pwm_config step_config = pwm_get_default_config();
    pwm_config_set_clkdiv(&step_config, (float)divider);
    pwm_config_set_wrap(&step_config, (uint16_t)(period - 1u));
    // The counter restarts at 0 and the output is high while counter < level,
    // so each period starts high: the idle level when wired common-anode.
    pwm_init(slice, &step_config, false);
    pwm_set_chan_level(slice, channel, (uint16_t)(period / 2u));
    gpio_set_function(step_pin, GPIO_FUNC_PWM);
    pwm_set_enabled(slice, true);
    if (step_pin < count_of(stepper_rates)) stepper_rates[step_pin] = rate;
}

int hal_usb_read_byte(void) {
    return getchar_timeout_us(0u);
}

int hal_usb_connected(void) {
    return stdio_usb_connected();
}

void hal_usb_write(const uint8_t *data, size_t length) {
    for (size_t i = 0; i < length; ++i) putchar_raw(data[i]);
}