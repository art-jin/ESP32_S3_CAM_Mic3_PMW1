#include "servo.h"

#include <math.h>

#include "driver/ledc.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"

static const char *TAG = "servo";

/* TIMER_0 is conventionally reserved for camera XCLK (not used in this
 * project today, but kept free for forward compatibility). CHANNEL_0 same
 * reasoning. We use TIMER_1 / CHANNEL_1. */
#define SERVO_LEDC_TIMER     LEDC_TIMER_1
#define SERVO_LEDC_CHANNEL   LEDC_CHANNEL_1
#define SERVO_LEDC_MODE      LEDC_LOW_SPEED_MODE
#define SERVO_DUTY_BITS      LEDC_TIMER_14_BIT
#define SERVO_DUTY_MAX       ((1U << SERVO_DUTY_BITS) - 1)   /* 16383 */

static float    s_target_angle_deg = 0.0f;     /* last commanded, clamped */
static float    s_last_delta_deg   = 0.0f;     /* magnitude of last commanded step */
static int64_t  s_last_motion_us   = 0;        /* esp_timer timestamp */
static bool     s_init_ok          = false;

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
    /* Center the servo at home. */
    servo_set_angle_deg(0.0f);

    ESP_LOGI(TAG, "init OK: GPIO %d, %d Hz, %d-bit duty, pulse %d-%d µs (%.0f° home)",
             SERVO_GPIO, SERVO_PWM_FREQ_HZ, (int)SERVO_DUTY_BITS,
             SERVO_PULSE_MIN_US, SERVO_PULSE_MAX_US, s_target_angle_deg);
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

    /* Mark motion time for servo_is_moving(). */
    s_last_motion_us = esp_timer_get_time();
}

void servo_set_angle_deg(float angle_deg)
{
    if (angle_deg > SERVO_ANGLE_MAX_DEG) angle_deg = SERVO_ANGLE_MAX_DEG;
    if (angle_deg < SERVO_ANGLE_MIN_DEG) angle_deg = SERVO_ANGLE_MIN_DEG;
    /* Track step magnitude for servo_is_moving()'s adaptive holdoff. */
    s_last_delta_deg = fabsf(angle_deg - s_target_angle_deg);
    s_target_angle_deg = angle_deg;

    /* Linear map: angle at disc [-40.5°, +40.5°] → pulse [500, 2500] µs.
     * Math: 270° servo / 3.33 gear reduction = 81° disc travel.
     * slope = 2000 µs / 81° = 24.69 µs per disc-degree.
     *
     * (Earlier code assumed 180° servo, slope = 37.04 µs/deg — that was
     * wrong; the servo is actually 270° per spec. The error meant our
     * "±20°" command was actually only ±13.3° at the disc, leaving most
     * of the servo range unused.)
     *
     * Sign depends on shaft orientation — see SERVO_SHAFT_INSTALLED_DOWN
     * in servo.h. With shaft-down, we negate so that positive angle still
     * means "CW viewed from above" (which is what tracker/geometry use). */
    float slope_us_per_deg = 2000.0f / 81.0f;
#if SERVO_SHAFT_INSTALLED_DOWN
    slope_us_per_deg = -slope_us_per_deg;
#endif
    float pulse_f = SERVO_PULSE_CENTER_US + angle_deg * slope_us_per_deg;
    servo_set_pulse_us((uint32_t)(pulse_f + 0.5f));
}

float servo_get_angle_deg(void)
{
    return s_target_angle_deg;
}

bool servo_is_moving(void)
{
    if (s_last_motion_us == 0) return false;     /* no motion since boot */
    int64_t now = esp_timer_get_time();
    int64_t elapsed_ms = (now - s_last_motion_us) / 1000;
    /* ADAPTIVE HOLDOFF (Phase B2): scale pause duration with step size.
     * Small steps (e.g., idle-return micro-steps of ~0.25°) barely excite
     * mechanical vibration, so a short 200 ms pause is enough. Large steps
     * that slam the servo across its range need the full 500 ms to let the
     * control loop settle and the vibration couple to decay.
     *
     * Effect on idle return: previously each micro-step triggered the full
     * 500 ms pause, capping the effective return rate at ~0.5°/s (one
     * 0.25° step every 500 ms). With 200 ms pause for small steps, the
     * rate matches the configured 2.5°/s.
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
