#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

/* JS6620 hobby servo driver via LEDC PWM.
 *
 * Hardware: see CLAUDE.md "Servo hardware (planned)".
 *   - GPIO 38, 50 Hz PWM, pulse 500–2500 µs
 *   - Drives a 15-tooth pinion inside a 50-tooth internal-mesh ring gear
 *   - Reduction 50/15 = 3.33 : 1 → 180° servo travel = ~54° ring travel
 *   - Therefore the usable ANGLE range at the ring gear is ±27° about home
 *
 * Angle sign convention: looking down at the gimbal from above the array,
 * positive angle = clockwise rotation of the ring gear (and the mic array
 * mounted on it). 0° = home (servo centered at 1500 µs).
 *
 * This is the pure driver layer — no tracking logic, no motion-pause. The
 * tracker module (planned) sits on top. */

#define SERVO_GPIO             38

#define SERVO_PWM_FREQ_HZ      50
#define SERVO_PERIOD_US        (1000000 / SERVO_PWM_FREQ_HZ)   /* 20000 µs */

/* JS6620 pulse-width range (standard hobby servo). */
#define SERVO_PULSE_MIN_US     500
#define SERVO_PULSE_CENTER_US  1500
#define SERVO_PULSE_MAX_US     2500

/* Mechanical limits at the ring gear (clamp range). */
#define SERVO_ANGLE_MIN_DEG    (-27.0f)
#define SERVO_ANGLE_MAX_DEG    (+27.0f)

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

/* True if the servo was commanded to a new target within the last
 * SERVO_MOTION_HOLDOFF_MS milliseconds. Phase 3 (tracker) uses this to
 * freeze DOA updates during motion + settle, so servo-motor whine doesn't
 * corrupt GCC-PHAT. Phase 1 implementation just returns false if no
 * command has been issued since boot. */
#define SERVO_MOTION_HOLDOFF_MS  750
bool servo_is_moving(void);
