#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

/* JS6620 hobby servo driver via LEDC PWM.
 *
 * Hardware: see CLAUDE.md "Servo hardware".
 *   - JS6620 is a **270°** rotation servo (not the typical 180°)
 *   - GPIO 38, 50 Hz PWM, pulse 500–2500 µs
 *   - Drives a 15-tooth pinion inside a 50-tooth internal-mesh ring gear
 *   - Reduction 50/15 = 3.33 : 1 → 270° servo travel = ~81° ring travel
 *   - Therefore the mechanical ANGLE range at the ring gear is ±40.5°
 *   - We clamp to ±30° (safety margin below ±40.5° mechanical limit,
 *     also leaves room for the feed-forward tracker to maneuver without
 *     saturating the clamp)
 *
 * Angle sign convention: looking down at the gimbal from above the array,
 * positive angle = clockwise rotation of the ring gear (and the mic array
 * mounted on it). 0° = home (servo centered at 1500 µs).
 *
 * This is the pure driver layer — no tracking logic, no motion-pause. The
 * tracker module sits on top. */

#define SERVO_GPIO             38

#define SERVO_PWM_FREQ_HZ      50
#define SERVO_PERIOD_US        (1000000 / SERVO_PWM_FREQ_HZ)   /* 20000 µs */

/* JS6620 pulse-width range (standard hobby servo). */
#define SERVO_PULSE_MIN_US     500
#define SERVO_PULSE_CENTER_US  1500
#define SERVO_PULSE_MAX_US     2500

/* Mechanical limits at the ring gear (clamp range).
 * True mechanical limit is ±40.5° (270° servo travel / 3.33 reduction).
 * Clamped to ±30° — leaves a safety margin below the mechanical limit
 * and stays well within the feed-forward tracker's stable region.
 * The earlier ±20° limit was based on the wrong 180° servo assumption
 * and was unnecessarily conservative. */
#define SERVO_ANGLE_MIN_DEG    (-30.0f)
#define SERVO_ANGLE_MAX_DEG    (+30.0f)

/* Servo shaft orientation switch.
 *
 * The JS6620 is installed shaft-down in the current gimbal. Empirical
 * testing (2026-06-22): with SERVO_SHAFT_INSTALLED_DOWN=1, a positive
 * servo command produces CW rotation of the array viewed from above
 * (M3 at 6oc moves toward 7oc). This matches the tracker's intent
 * (positive target = rotate array CW from above to bring M3 toward
 * a source at α > 180°).
 *
 * Set to 0 if the servo is reinstalled shaft-up, or if the array ends
 * up rotating the wrong way (servo = +20° moves M3 to 5oc instead of 7oc). */
#define SERVO_SHAFT_INSTALLED_DOWN  1

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
#define SERVO_MOTION_HOLDOFF_MS  500
bool servo_is_moving(void);
