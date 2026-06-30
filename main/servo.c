#include "servo.h"

#include <math.h>

#include "driver/ledc.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"

static const char *TAG = "servo";

/* TIMER_0 is conventionally reserved for camera XCLK (not used in this
 * project today, but kept free for forward compatibility). CHANNEL_0 same
 * reasoning. We use TIMER_1 / CHANNEL_1. */
#define SERVO_LEDC_TIMER     LEDC_TIMER_1
#define SERVO_LEDC_CHANNEL   LEDC_CHANNEL_1
#define SERVO_LEDC_MODE      LEDC_LOW_SPEED_MODE
#define SERVO_DUTY_BITS      LEDC_TIMER_14_BIT
#define SERVO_DUTY_MAX       ((1U << SERVO_DUTY_BITS) - 1)   /* 16383 */

/* B3 PWM soft-start parameters.
 *   STEP_DEG    — per-step magnitude at the disc (post-gear-reduction).
 *                 3° at the disc = 10° at the servo shaft (3.33× ratio),
 *                 which is well under the JS6620's per-period response.
 *   PERIOD_MS   — timer fires every 20 ms = one PWM period at 50 Hz.
 *                 The LEDC peripheral latches duty at period boundaries,
 *                 so calling faster than this just wastes CPU. */
#define SERVO_SMOOTH_STEP_DEG    3.0f
#define SERVO_SMOOTH_PERIOD_MS   20

static float    s_target_angle_deg  = 0.0f;   /* goal commanded by tracker */
static float    s_current_angle_deg = 0.0f;   /* where PWM is now (smoothly catching up) */
static float    s_last_delta_deg    = 0.0f;   /* magnitude of last commanded step */
static int64_t  s_last_motion_us    = 0;      /* esp_timer timestamp of last PWM write */
static bool     s_init_ok           = false;
static TimerHandle_t s_smooth_timer = NULL;

/* Pure PWM write. Computes pulse from angle, clamps, latches LEDC duty.
 * Does not touch any state — caller manages s_current/s_last_motion_us. */
static void write_pwm_for_angle(float angle_deg)
{
    if (!s_init_ok) return;
    /* Linear map: angle at gear [-101.25°, +101.25°] → pulse [500, 2500] µs.
     * 270° servo / 1.333 reduction (20T/15T external) = 202.5° travel.
     * slope = 2000 µs / 202.5° = 9.877 µs per gear-degree.
     * Sign: external mesh + shaft-down, inversions cancel, no negate. */
    float slope_us_per_deg = 2000.0f / 202.5f;
#if SERVO_SHAFT_INSTALLED_DOWN
    slope_us_per_deg = -slope_us_per_deg;
#endif
    float pulse_f = SERVO_PULSE_CENTER_US + angle_deg * slope_us_per_deg;
    uint32_t us = (uint32_t)(pulse_f + 0.5f);
    if (us < SERVO_PULSE_MIN_US) us = SERVO_PULSE_MIN_US;
    if (us > SERVO_PULSE_MAX_US) us = SERVO_PULSE_MAX_US;
    uint32_t duty = (us * SERVO_DUTY_MAX) / SERVO_PERIOD_US;
    ledc_set_duty(SERVO_LEDC_MODE, SERVO_LEDC_CHANNEL, duty);
    ledc_update_duty(SERVO_LEDC_MODE, SERVO_LEDC_CHANNEL);
}

/* FreeRTOS software timer callback: advance s_current toward s_target by
 * one step. Stops itself when target is reached. Runs in the timer service
 * task — must be fast and non-blocking. ledc_set_duty is register write,
 * takes microseconds. */
static void smooth_timer_cb(TimerHandle_t h)
{
    float diff = s_target_angle_deg - s_current_angle_deg;
    float abs_diff = fabsf(diff);
    if (abs_diff < SERVO_SMOOTH_STEP_DEG) {
        /* Final step: snap to target exactly. */
        s_current_angle_deg = s_target_angle_deg;
        write_pwm_for_angle(s_current_angle_deg);
        s_last_motion_us = esp_timer_get_time();
        xTimerStop(s_smooth_timer, 0);
        return;
    }
    /* One step toward target. */
    s_current_angle_deg += (diff > 0.0f) ? SERVO_SMOOTH_STEP_DEG : -SERVO_SMOOTH_STEP_DEG;
    write_pwm_for_angle(s_current_angle_deg);
    s_last_motion_us = esp_timer_get_time();
}

esp_err_t servo_init(void)
{
    ledc_timer_config_t timer_cfg = {
        .speed_mode      = SERVO_LEDC_MODE,
        .duty_resolution = SERVO_DUTY_BITS,
        .timer_num       = SERVO_LEDC_TIMER,
        .freq_hz         = SERVO_PWM_FREQ_HZ,
        .clk_cfg         = LEDC_AUTO_CLK,
    };
    esp_err_t err = ledc_timer_config(&timer_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ledc_timer_config: %s", esp_err_to_name(err));
        return err;
    }

    ledc_channel_config_t ch_cfg = {
        .gpio_num   = SERVO_GPIO,
        .speed_mode = SERVO_LEDC_MODE,
        .channel    = SERVO_LEDC_CHANNEL,
        .intr_type  = LEDC_INTR_DISABLE,
        .timer_sel  = SERVO_LEDC_TIMER,
        .duty       = 0,
        .hpoint     = 0,
    };
    err = ledc_channel_config(&ch_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ledc_channel_config: %s", esp_err_to_name(err));
        return err;
    }

    s_init_ok = true;

    /* Center the servo at home directly (no smooth timer needed for 0→0). */
    s_target_angle_deg  = 0.0f;
    s_current_angle_deg = 0.0f;
    write_pwm_for_angle(0.0f);

    /* Create the smooth-motion timer (not started until first motion). */
    s_smooth_timer = xTimerCreate("servo_smooth",
                                  pdMS_TO_TICKS(SERVO_SMOOTH_PERIOD_MS),
                                  pdTRUE,       /* auto-reload */
                                  NULL,
                                  smooth_timer_cb);
    if (s_smooth_timer == NULL) {
        ESP_LOGE(TAG, "xTimerCreate failed");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "init OK: GPIO %d, %d Hz, %d-bit duty, pulse %d-%d µs (%.0f° home), smooth %d°/step@%dms",
             SERVO_GPIO, SERVO_PWM_FREQ_HZ, (int)SERVO_DUTY_BITS,
             SERVO_PULSE_MIN_US, SERVO_PULSE_MAX_US, s_target_angle_deg,
             (int)SERVO_SMOOTH_STEP_DEG, SERVO_SMOOTH_PERIOD_MS);
    return ESP_OK;
}

void servo_set_pulse_us(uint32_t us)
{
    if (!s_init_ok) return;
    if (us < SERVO_PULSE_MIN_US) us = SERVO_PULSE_MIN_US;
    if (us > SERVO_PULSE_MAX_US) us = SERVO_PULSE_MAX_US;
    uint32_t duty = (us * SERVO_DUTY_MAX) / SERVO_PERIOD_US;
    ledc_set_duty(SERVO_LEDC_MODE, SERVO_LEDC_CHANNEL, duty);
    ledc_update_duty(SERVO_LEDC_MODE, SERVO_LEDC_CHANNEL);
    /* Treat as instantaneous (no smooth profile). */
    s_last_motion_us = esp_timer_get_time();
}

void servo_set_angle_deg(float angle_deg)
{
    if (angle_deg > SERVO_ANGLE_MAX_DEG) angle_deg = SERVO_ANGLE_MAX_DEG;
    if (angle_deg < SERVO_ANGLE_MIN_DEG) angle_deg = SERVO_ANGLE_MIN_DEG;
    /* Delta is from CURRENT (not previous target) — if mid-motion, the
     * true remaining traversal is what matters for adaptive holdoff. */
    s_last_delta_deg = fabsf(angle_deg - s_current_angle_deg);
    s_target_angle_deg = angle_deg;

    /* For very small steps (< one smooth step), write immediately and
     * skip the timer — saves 20 ms of latency for idle-return micro-steps. */
    if (s_last_delta_deg < SERVO_SMOOTH_STEP_DEG) {
        s_current_angle_deg = angle_deg;
        write_pwm_for_angle(angle_deg);
        s_last_motion_us = esp_timer_get_time();
        if (s_smooth_timer) xTimerStop(s_smooth_timer, 0);
        return;
    }

    /* Larger steps go through the smooth timer. Timer auto-reloads every
     * 20 ms, advancing s_current toward s_target by SERVO_SMOOTH_STEP_DEG. */
    if (s_smooth_timer) xTimerStart(s_smooth_timer, 0);
}

float servo_get_angle_deg(void)
{
    /* Returns the ACTUAL (ramped) position, not the commanded target.
     * Feed-forward compensation (α_room = α_array + β_servo) needs the
     * array's TRUE physical orientation. With PWM soft-start, the servo
     * ramps toward target — returning the target while still ramping
     * causes α_room to be overestimated, creating positive feedback
     * that drives the servo to the clamp. */
    return s_current_angle_deg;
}

bool servo_is_moving(void)
{
    /* Moving if PWM is still catching up to target. */
    if (fabsf(s_target_angle_deg - s_current_angle_deg) > 0.1f) {
        return true;
    }
    if (s_last_motion_us == 0) return false;     /* no motion since boot */
    int64_t now = esp_timer_get_time();
    int64_t elapsed_ms = (now - s_last_motion_us) / 1000;
    /* ADAPTIVE HOLDOFF (Phase B2): scale pause duration with step size.
     * Small steps (e.g., idle-return micro-steps of ~0.25°) barely excite
     * mechanical vibration, so a short 200 ms pause is enough. Large steps
     * that slam the servo across its range need the full 500 ms to let the
     * control loop settle and the vibration couple to decay.
     *
     * Thresholds:
     *   |delta| <  5°   → 200 ms  (small corrections, idle return)
     *   |delta| < 15°   → 350 ms  (medium tracking motions)
     *   |delta| ≥ 15°   → 500 ms  (large swings, max settling time) */
    uint32_t holdoff_ms;
    if (s_last_delta_deg <  5.0f) holdoff_ms = 200;
    else if (s_last_delta_deg < 15.0f) holdoff_ms = 350;
    else                             holdoff_ms = SERVO_MOTION_HOLDOFF_MS;
    return elapsed_ms < (int64_t)holdoff_ms;
}
