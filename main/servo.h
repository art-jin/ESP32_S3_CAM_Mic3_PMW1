#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

/* JS6620 hobby servo driver via LEDC PWM.
 *
 * Hardware: see CLAUDE.md "Servo hardware".
 *   - JS6620 is a **270°** rotation servo
 *   - GPIO 38, 50 Hz PWM, pulse 500–2500 µs
 *   - 15T pinion on servo shaft, meshing EXTERNALLY with 20T spur gear
 *   - External mesh: pinion and gear rotate in OPPOSITE directions
 *   - Reduction 20/15 = 1.333 : 1 → 270° servo = ~202.5° gear travel
 *   - Mechanical limit at gear: ±101.25°
 *   - Soft clamp: ±100° (1.25° safety margin)
 *
 * Angle sign convention: looking down from above, positive angle = CW. */

#define SERVO_GPIO             38

#define SERVO_PWM_FREQ_HZ      50
#define SERVO_PERIOD_US        (1000000 / SERVO_PWM_FREQ_HZ)   /* 20000 µs */

/* JS6620 pulse-width range (standard hobby servo). */
#define SERVO_PULSE_MIN_US     500
#define SERVO_PULSE_CENTER_US  1500
#define SERVO_PULSE_MAX_US     2500

/* Mechanical limits at the gear output.
 * True mechanical limit: ±101.25° (270° servo / 1.333 reduction).
 * Clamped to ±100° for 1.25° safety margin.
 * Coverage from 6oc home: ~2:40 to ~9:20 o'clock. */
#define SERVO_ANGLE_MIN_DEG    (-100.0f)
#define SERVO_ANGLE_MAX_DEG    (+100.0f)

/* Servo shaft orientation switch.
 *
 * Current config (15T/20T external mesh, shaft-down):
 * SERVO_SHAFT_INSTALLED_DOWN = 0 (no negation needed)
 *
 * External mesh reverses direction relative to internal mesh.
 * With shaft-down + external mesh, the two direction inversions
 * (shaft flip + gear reversal) cancel out, so no negation needed.
 *
 * Verified 2026-06-30: user at 7oc → servo +30°, M3 toward 7oc ✓
 *
 * Old config (15T/50T internal mesh): was =1. */
#define SERVO_SHAFT_INSTALLED_DOWN  0

esp_err_t servo_init(void);

/* Set raw pulse width in microseconds. Clamped to [SERVO_PULSE_MIN_US,
 * SERVO_PULSE_MAX_US]. Updates the internal motion-state used by
 * servo_is_moving(). */
void servo_set_pulse_us(uint32_t us);

/* Set target angle in degrees at the ring gear. 0 = home.
 * Positive = clockwise (viewed from above). Clamped to
 * [SERVO_ANGLE_MIN_DEG, SERVO_ANGLE_MAX_DEG]. */
void servo_set_angle_deg(float angle_deg);

/* Current target angle (last commanded, after clamping). Returns the
 * ring-gear angle; if you need raw servo angle, multiply by 3.33. */
float servo_get_angle_deg(void);

/* True if the servo was commanded to a new target within the holdoff
 * window. The tracker uses this to freeze DOA updates during motion +
 * settle, so servo-motor whine doesn't corrupt GCC-PHAT.
 *
 * Phase B2 (2026-06-25): holdoff is now ADAPTIVE based on the magnitude
 * of the last commanded step:
 *   |delta| <  5°   →  200 ms
 *   |delta| < 15°   →  350 ms
 *   |delta| ≥ 15°   →  500 ms (this constant)
 * Small idle-return steps no longer trigger the full 500 ms pause, so
 * idle return rate matches the configured 2.5°/s instead of being
 * capped at ~0.5°/s by the holdoff. */
#define SERVO_MOTION_HOLDOFF_MS  500
bool servo_is_moving(void);
