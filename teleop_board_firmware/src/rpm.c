#include "rpm.h"

#include <stdbool.h>

#include "hardware/gpio.h"
#include "hardware/sync.h"
#include "pico/stdlib.h"

static const uint rpm_pins[RPM_WHEEL_COUNT] = {
    PIN_RPM_FRONT_LEFT,
    PIN_RPM_FRONT_RIGHT,
    PIN_RPM_BACK_RIGHT,
    PIN_RPM_BACK_LEFT,
};

static volatile uint32_t last_edge_us[RPM_WHEEL_COUNT];
static volatile uint32_t edge_period_us[RPM_WHEEL_COUNT];
static volatile bool edge_seen[RPM_WHEEL_COUNT];

static int wheel_index_from_pin(uint pin) {
    for (uint i = 0; i < RPM_WHEEL_COUNT; ++i) {
        if (rpm_pins[i] == pin) return (int)i;
    }
    return -1;
}

static void rpm_gpio_callback(uint pin, uint32_t events) {
    if ((events & RPM_TRIGGER_EDGE) == 0u) return;
    const int index = wheel_index_from_pin(pin);
    if (index < 0) return;

    const uint32_t now = time_us_32();
    const uint32_t elapsed = now - last_edge_us[index];
    if (edge_seen[index] && elapsed < RPM_MIN_EDGE_PERIOD_US) return;
    if (edge_seen[index]) edge_period_us[index] = elapsed;
    last_edge_us[index] = now;
    edge_seen[index] = true;
}

void rpm_init(void) {
    for (uint i = 0; i < RPM_WHEEL_COUNT; ++i) {
        gpio_init(rpm_pins[i]);
        gpio_set_dir(rpm_pins[i], GPIO_IN);
        gpio_pull_down(rpm_pins[i]);
    }

    gpio_set_irq_enabled_with_callback(rpm_pins[0], RPM_TRIGGER_EDGE, true,
                                       rpm_gpio_callback);
    for (uint i = 1; i < RPM_WHEEL_COUNT; ++i) {
        gpio_set_irq_enabled(rpm_pins[i], RPM_TRIGGER_EDGE, true);
    }
}

void rpm_read_all(float wheel_rpm[RPM_WHEEL_COUNT], uint32_t now_us) {
    uint32_t periods[RPM_WHEEL_COUNT];
    uint32_t last_edges[RPM_WHEEL_COUNT];
    bool seen[RPM_WHEEL_COUNT];

    const uint32_t interrupt_state = save_and_disable_interrupts();
    for (uint i = 0; i < RPM_WHEEL_COUNT; ++i) {
        periods[i] = edge_period_us[i];
        last_edges[i] = last_edge_us[i];
        seen[i] = edge_seen[i];
    }
    restore_interrupts(interrupt_state);

    for (uint i = 0; i < RPM_WHEEL_COUNT; ++i) {
        if (!seen[i] || periods[i] == 0u
            || (uint32_t)(now_us - last_edges[i]) > RPM_STALE_TIMEOUT_US) {
            wheel_rpm[i] = 0.0f;
        } else {
            wheel_rpm[i] = 60000000.0f
                         / ((float)periods[i] * (float)RPM_PULSES_PER_REV);
        }
    }
}