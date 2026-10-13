#ifndef CONFIG_H
#define CONFIG_H

// =========================================================================
// CALIBRATION - measure these on the vehicle
// =========================================================================
// Procedure (see README "Calibration"):
//   1. Flash, then run `teleop.py --monitor` on the Pi. It sends no commands, so
//      the clutches stay released and the wheel and pedal move by hand.
//   2. Move each one to its travel ends and read the "sensor" voltages shown.
//   3. Enter the voltages and travel below, then set *_CALIBRATED to 1.
//   4. With STEPPER_MAX_HZ lowered (e.g. 300), command small moves and check
//      each axis heads toward its target; if not, flip its *_STEP_SIGN.
// Voltages are at the RP2040 ADC pin and must stay below 3.3 V. Either end may
// be the higher voltage; the mapping follows whichever way the sensor runs.

// While 0, that axis's stepper never moves (its clutch still engages and
// holds), so the placeholder values below cannot drive it into a stop. Throttle
// is held at zero until both are 1.
#define STEER_CALIBRATED        1u   // calibrated 2026-10-03
#define BRAKE_CALIBRATED        0u

// Steering: steering-wheel angle at each end of the allowed travel, and the
// sensor voltage there. Positive angle is a left turn.
#define STEER_ANGLE_MIN_DEG     (-65.0f) // right; hard stop is about -75 deg
#define STEER_ANGLE_MAX_DEG     (65.0f)  // left; hard stop is about +75 deg
#define STEER_SENSOR_V_AT_MIN   0.780f   // volts at STEER_ANGLE_MIN_DEG (stops measured 0.713 V right, 1.720 V left after re-mounting the sensor)
#define STEER_SENSOR_V_AT_MAX   1.653f   // volts at STEER_ANGLE_MAX_DEG

// Brake: sensor voltage with the pedal released (0%) and at the furthest
// position the stepper may ever push it (100%). Keep 100% short of the pedal's
// mechanical stop; the failsafe drives here with nobody at the controls.
#define BRAKE_SENSOR_V_RELEASED 0.372f   // volts at 0% (hydraulic pressure sensor on GPIO27, re-measured 2026-10-03 after rewiring)
#define BRAKE_SENSOR_V_FULL     1.400f   // volts at 100% (held full presses read 1.31-1.53 V, wheels locked)
// Stepper output-shaft rotation from 0% to 100% (after the gearbox). Only used
// to size the no-progress check below.
#define BRAKE_TRAVEL_DEG        60.0f    // [CALIBRATE]

// Each axis assumes a positive step rate raises its reading (angle, percent).
// If an axis moves away from its target, set its sign to -1.0f.
#define STEER_STEP_SIGN         1.0f     // [CALIBRATE]
#define BRAKE_STEP_SIGN         1.0f     // [CALIBRATE]

// Drivetrain between stepper and wheel/pedal.
#define STEPPER_FULL_STEPS_REV  200u     // [VERIFY] motor nameplate value
#define STEPPER_MICROSTEPS      8u       // [VERIFY] DM542T DIP-switch setting
#define STEER_GEAR_RATIO        1.0f     // [VERIFY] motor turns per steering-wheel turn
#define BRAKE_GEAR_RATIO        9.0f     // [VERIFY] motor turns per pedal-shaft turn

// Bench build only (andre_rp2040_controller_bench.uf2 sets BENCH_MODE=1): throttle
// follows the command with only the brake interlock and the comms timeout
// left, and steering runs open loop (stick deflection -> step rate, up to
// BENCH_STEER_MAX_STEPS_PER_S, no sensor or travel limits). For multimeter and
// motor checks with the wheels off the ground and the steering motor
// uncoupled. Never flash it for driving.
#ifndef BENCH_MODE
#define BENCH_MODE              0
#endif
// Bench steering: the step rate scales with the commanded angle (stick
// deflection), from 0 at center to this at full travel, in that direction.
// 1600 steps/rev at 8x microstepping, so 500 steps/s = 0.31 rev/s.
#define BENCH_STEER_MAX_STEPS_PER_S 500.0f

// =========================================================================
// PINS
// =========================================================================

#define PIN_LED                 25u

// RC-filtered analog speed command. GPIO2 and GPIO3 share a PWM slice.
#define PIN_PWM_BACK_LEFT       2u
#define PIN_PWM_BACK_RIGHT      3u

// Clutch solenoids- not mentioned in email but are on circuit sheet
// Driven through low-side MOSFETs: GPIO high engages. The clutches are what
// hold the steering wheel and brake pedal; released, the wheel turns freely
// and the brake decompresses. Released at power-up; engaged from the first
// valid command until reset.
#define PIN_CLUTCH_STEER        4u
#define PIN_CLUTCH_BRAKE        5u
#define CLUTCH_ENGAGED_LEVEL    1u

//RPM sensors
#define PIN_RPM_FRONT_LEFT      6u
#define PIN_RPM_FRONT_RIGHT     7u
#define PIN_RPM_BACK_RIGHT      8u
#define PIN_RPM_BACK_LEFT       9u

//steppers
#define PIN_STEER_PULSE         10u
#define PIN_STEER_DIR           11u
#define PIN_STEER_ENABLE        12u
#define PIN_BRAKE_PULSE         13u
#define PIN_BRAKE_DIR           14u
#define PIN_BRAKE_ENABLE        15u
//sensors
#define PIN_ADC_BRAKE           27u  // per board schematic
#define PIN_ADC_STEER           28u  // per board schematic
#define ADC_CH_BRAKE            1u   // GPIO27 = ADC1
#define ADC_CH_STEER            2u   // GPIO28 = ADC2

// =========================================================================
// STEPPERS
// =========================================================================

#define STEPPER_COUNT           2u
#define STEPPER_STEPS_PER_REV   (STEPPER_FULL_STEPS_REV * STEPPER_MICROSTEPS)
#define STEER_STEPS_PER_DEG     (((float)STEPPER_STEPS_PER_REV * STEER_GEAR_RATIO) / 360.0f)
#define BRAKE_STEPS_PER_DEG     (((float)STEPPER_STEPS_PER_REV * BRAKE_GEAR_RATIO) / 360.0f)
#define BRAKE_STEPS_PER_PERCENT (BRAKE_STEPS_PER_DEG * BRAKE_TRAVEL_DEG / 100.0f)

// Full brake travel takes BRAKE_STEPS_PER_DEG * BRAKE_TRAVEL_DEG steps; keep
// that at or below STEPPER_MAX_HZ * BRAKE_FAILSAFE_RAMP_US / 1e6 or the
// failsafe ramp takes longer than configured.
#define STEPPER_MAX_HZ          300u    // [TUNE] lowered for first steering tests; was 2000
// The pulse-rate clamp in hal.c. Only the bench build may exceed STEPPER_MAX_HZ,
// for its open-loop steering test.
#define STEPPER_RATE_LIMIT_HZ \
    ((BENCH_MODE && BENCH_STEER_MAX_STEPS_PER_S > (float)STEPPER_MAX_HZ) \
        ? BENCH_STEER_MAX_STEPS_PER_S : (float)STEPPER_MAX_HZ)
#define STEPPER_MIN_HZ          1u
#define STEPPER_PULSE_WIDTH_US  5u      // DM542T-compatible conservative pulse
#define STEPPER_DIR_SETUP_US    10u
// DM542T inputs are optocouplers. Wired common-anode: PUL+/DIR+/ENA+ tie to
// +5 V and the GPIOs drive PUL-/DIR-/ENA- directly, so an opto conducts while
// its GPIO is LOW. Set to 0 if the GPIOs drive the + side (common-cathode) or
// sink through an inverting transistor stage.
#define STEPPER_INPUTS_COMMON_ANODE 1u
// PUL idles with the opto off; each step is one opto-on half period.
#define STEPPER_STEP_IDLE_LEVEL (STEPPER_INPUTS_COMMON_ANODE ? 1u : 0u)
// The DM542T is disabled while ENA's opto conducts, enabled while it is off.
#define STEPPER_ENABLE_ACTIVE   (STEPPER_INPUTS_COMMON_ANODE ? 1u : 0u)

// ---- Axis protection ---------------------------------------------------
// Soft limit: at or past the calibrated travel an axis only steps back inside.
// The two checks below latch a fault that freezes the axis until reset.
//
// Overtravel: the axis is stepped past its travel by more than this margin
// (overshoot, or a reversed *_STEP_SIGN).
#define STEER_OVERTRAVEL_DEG        3.0f    // [TUNE]
#define BRAKE_OVERTRAVEL_PERCENT    5.0f    // [TUNE]
// No progress: after stepping the equivalent of CHECK in one direction, the
// reading must have moved at least MIN that way. Catches a reversed sign, a
// slipped or stuck sensor, and a motor stalled against a mechanical stop.
#define STEER_PROGRESS_CHECK_DEG    10.0f   // [TUNE]
#define STEER_PROGRESS_MIN_DEG      3.0f    // [TUNE]
#define BRAKE_PROGRESS_CHECK_PERCENT 10.0f  // [TUNE]
#define BRAKE_PROGRESS_MIN_PERCENT  3.0f    // [TUNE]

// =========================================================================
// RPM
// =========================================================================

#define RPM_WHEEL_COUNT         4u
#define RPM_PULSES_PER_REV      32u     // sensor pulses per wheel revolution

//Ordering the RPM sensors in comparison to each other
typedef enum {
    WHEEL_FL = 0,
    WHEEL_FR = 1,
    WHEEL_BR = 2,
    WHEEL_BL = 3,
} wheel_t;

// Anti-bouncd
#define RPM_MIN_EDGE_PERIOD_US  50u

// LOOK BACK HERE EVENTUALLY
#define RPM_TRIGGER_EDGE        GPIO_IRQ_EDGE_RISE

// =========================================================================
// ANALOG
// =========================================================================

#define ADC_VREF                3.3f
#define ADC_COUNTS              4095.0f
#define ADC_VOLTS_PER_COUNT     (ADC_VREF / ADC_COUNTS)

// Readings outside these windows mean a disconnected or shorted sensor rather
// than a real position; that axis holds still and throttle is held at zero
// until the reading recovers.
#define STEER_SENSOR_FAULT_V_MIN 0.1f   // [TUNE]
#define STEER_SENSOR_FAULT_V_MAX 3.2f   // [TUNE]
#define BRAKE_SENSOR_FAULT_V_MIN 0.1f   // [TUNE]
#define BRAKE_SENSOR_FAULT_V_MAX 3.2f   // [TUNE]

#define BRAKE_PERCENT_DENOMINATOR       100.0f
// Throttle is forced to zero while the commanded brake exceeds this.
#define BRAKE_THROTTLE_INTERLOCK_PERCENT 5.0f
// When messages stop arriving (after USB_COMMAND_TIMEOUT_US), or the host sends
// a failsafe message, the brake target ramps linearly to BRAKE_FAILSAFE_PERCENT
// over BRAKE_FAILSAFE_RAMP_US.
#define BRAKE_FAILSAFE_PERCENT  100.0f
#define BRAKE_FAILSAFE_RAMP_US  1000000u

// =========================================================================
// DRIVE TRAIN
// =========================================================================

#define PWM_FREQ_HZ             25000u

#define THROTTLE_OUT_V_MAX      3.3f//[VERIFY]

#define THROTTLE_MIN            0.0f
#define THROTTLE_MAX            1.0f

#define SPEED_COMMAND_MAX_MPH   10.0f  // [TUNE] Speed that maps to 100% PWM duty

static inline float speed_mph_to_pwm_percent(float speed_mph) {
    if (SPEED_COMMAND_MAX_MPH <= 0.0f || speed_mph <= 0.0f) {
        return 0.0f;
    }
    if (speed_mph >= SPEED_COMMAND_MAX_MPH) {
        return 100.0f;
    }
    return (speed_mph / SPEED_COMMAND_MAX_MPH) * 100.0f;
}

// =========================================================================
// CONTROL LOOPS
// =========================================================================

#define CONTROL_LOOP_HZ         1000u
#define CONTROL_PERIOD_US       (1000000u / CONTROL_LOOP_HZ)
#define CONTROL_DT_S            (1.0f / (float)CONTROL_LOOP_HZ)

#define SPEED_LOOP_HZ           200u    // capped by the RC settling time
#define SPEED_PERIOD_US         (1000000u / SPEED_LOOP_HZ)
#define SPEED_DT_S              (1.0f / (float)SPEED_LOOP_HZ)

#define STEER_LOOP_HZ           100u
#define STEER_PERIOD_US         (1000000u / STEER_LOOP_HZ)

#define BRAKE_LOOP_HZ           100u
#define BRAKE_PERIOD_US         (1000000u / BRAKE_LOOP_HZ)

#define USB_COMMAND_TIMEOUT_US  500000u
#define ADC_FILTER_SAMPLES      8u
#define RPM_STALE_TIMEOUT_US    500000u

// [TUNE] Speed PID is a duty-cycle correction around the mph feed-forward map.
#define SPEED_PID_KP            0.02f
#define SPEED_PID_KI            0.02f
#define SPEED_PID_KD            0.0f
#define SPEED_PID_OUT_MIN       (-0.25f)
#define SPEED_PID_OUT_MAX       (0.25f)

// Position gains only set how fast an axis closes on its target (capped at
// STEPPER_MAX_HZ); how far it may go is set by the calibrated travel above.
#define STEER_PID_KP            40.0f  // [TUNE] step pulses/sec per degree
#define STEER_PID_KI            0.0f
#define STEER_PID_KD            0.0f
#define STEER_PID_OUT_MIN       (-(float)STEPPER_MAX_HZ)
#define STEER_PID_OUT_MAX       ((float)STEPPER_MAX_HZ)

#define BRAKE_PID_KP            50.0f  // [TUNE] step pulses/sec per percent
#define BRAKE_PID_KI            0.0f
#define BRAKE_PID_KD            0.0f
#define BRAKE_PID_OUT_MIN       (-(float)STEPPER_MAX_HZ)
#define BRAKE_PID_OUT_MAX       ((float)STEPPER_MAX_HZ)

// =========================================================================
// ACKERMANN [VERIFY ALL]
// =========================================================================

#define WHEELBASE_M             1.20f   // [VERIFY] faxle to raxle
#define TRACK_WIDTH_M           0.70f   // [VERIFY] lwheel to rwheel
#define WHEEL_RADIUS_M          0.125f  // [VERIFY]

#define WHEEL_CIRCUM_M          (2.0f * 3.14159265f * WHEEL_RADIUS_M)

// =========================================================================
// PROTOCOL AND RATES
// =========================================================================

#define PROTO_CMD_PAYLOAD       6u  // int16 centidegrees, uint16 centi-mph, uint16 centi-percent brake
#define PROTO_FAILSAFE_PAYLOAD  1u  // uint8 reason (1 = operator controller lost)
#define PROTO_TELEMETRY_PAYLOAD 20u
#define PROTO_MAX_PAYLOAD       PROTO_TELEMETRY_PAYLOAD
#define PROTO_MAX_FRAME         (PROTO_MAX_PAYLOAD + 11u)

#define TELEMETRY_HZ            5u
#define TELEMETRY_PERIOD_US     (1000000u / TELEMETRY_HZ)

#define DIAG_HZ                 1u
#define DIAG_PERIOD_US          (1000000u / DIAG_HZ)

#endif // CONFIG_H
