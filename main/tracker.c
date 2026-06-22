#include "tracker.h"

#include <math.h>
#include <string.h>

#include "esp_log.h"
#include "servo.h"

static const char *TAG = "tracker";

static tracker_config_t s_cfg = TRACKER_DEFAULT_CONFIG;
static tracker_mode_t   s_mode = TRACKER_MODE_IDLE;
static bool             s_enabled = true;
static float            s_last_target_deg = 0.0f;   /* last commanded angle */
static bool             s_have_target = false;      /* any command issued yet */
static float            s_pending_target = 0.0f;    /* last frame's raw target */
static bool             s_have_pending = false;     /* any pending target */

void tracker_init(const tracker_config_t *cfg)
{
    if (cfg) s_cfg = *cfg;
    s_mode = TRACKER_MODE_IDLE;
    s_have_target = false;
    ESP_LOGI(TAG, "init: home=%+.1f°  deadband=%.1f°  min_conf=%.2f  "
             "out_of_range=%.0f°  agreement=%.1f°  conservative=%d",
             s_cfg.home_deg, s_cfg.deadband_deg, s_cfg.min_confidence,
             s_cfg.out_of_range_deg, s_cfg.target_agreement_deg,
             s_cfg.conservative_mode ? 1 : 0);
}

void tracker_update(const doa_result_t *doa)
{
    if (!s_enabled) {
        s_mode = TRACKER_MODE_DISABLED;
        return;
    }
    if (doa == NULL) {
        s_mode = TRACKER_MODE_SUPPRESSED;
        return;
    }

    /* MOTION-PAUSE: while the servo is moving (or within its post-motion
     * holdoff window), do NOT consume DOA results. The servo's motor
     * whine couples through the PCB to the DAT0 mic pair, creating a
     * correlated noise source that makes ρ01 → 1.0 (L/R collapse) and,
     * worse, can produce phantom 3-mic DOA readings at a fixed "servo
     * direction." Feeding those back to the tracker creates a positive
     * feedback loop that slams the servo between mechanical limits. */
    if (servo_is_moving()) {
        s_mode = TRACKER_MODE_SUPPRESSED;
        return;
    }

    /* Only 3-mic DOA with sufficient confidence drives the servo.
     * 2-mic half-plane bearing is too coarse (front/back ambiguous)
     * and would cause oscillation when the source sits near the
     * broadside / extreme boundaries of the M2-M3 baseline. */
    if (doa->mode != DOA_MODE_3MIC || doa->confidence < s_cfg.min_confidence) {
        s_mode = TRACKER_MODE_SUPPRESSED;
        return;
    }

    /* Compute target: rotate the array so its 6oc (α=180°, M3) faces
     * the source. If source is at α, we rotate the array by (α - 180°).
     * Positive = clockwise viewed from above. */
    float target;
    if (s_cfg.conservative_mode && doa->stable_sextant >= 0) {
        /* Map stable_sextant (0..5, each = 60°) to the sextant center's
         * azimuth. Only re-commands when sextant changes — very stable
         * but no within-sextant resolution. */
        int sextant_center_az = doa->stable_sextant * 60;  /* 0,60,120,...,300 */
        target = (float)sextant_center_az - 180.0f + s_cfg.home_deg;
    } else {
        target = doa->azimuth_deg - 180.0f + s_cfg.home_deg;
    }

    /* OUT-OF-RANGE SUPPRESSION: if the unclamped target is far outside
     * the mechanical range, the source is in a direction the servo can
     * never point at. Rather than saturating at one limit and then
     * oscillating when DOA noise flips the reading to the other side,
     * hold the last commanded position and wait for the source to come
     * back into a trackable arc. */
    if (s_cfg.out_of_range_deg > 0.0f &&
        (target >  s_cfg.out_of_range_deg ||
         target < -s_cfg.out_of_range_deg)) {
        s_mode = TRACKER_MODE_SUPPRESSED;
        return;
    }

    /* Clamp to mechanical range. */
    if (target > SERVO_ANGLE_MAX_DEG) target = SERVO_ANGLE_MAX_DEG;
    if (target < SERVO_ANGLE_MIN_DEG) target = SERVO_ANGLE_MIN_DEG;

    /* TWO-FRAME AGREEMENT CHECK: require two consecutive frames' raw
     * targets to agree within target_agreement_deg before commanding
     * motion. This filters single-frame GCC-PHAT transients that would
     * otherwise slam the servo to ±20° limits on noise. Real position
     * changes produce consistent readings across frames; transients
     * don't. */
    if (s_cfg.target_agreement_deg > 0.0f) {
        if (!s_have_pending) {
            s_pending_target = target;
            s_have_pending = true;
            s_mode = TRACKER_MODE_SUPPRESSED;
            return;
        }
        if (fabsf(target - s_pending_target) > s_cfg.target_agreement_deg) {
            /* Disagreement — update pending and wait for confirmation. */
            s_pending_target = target;
            s_mode = TRACKER_MODE_SUPPRESSED;
            return;
        }
        /* Agreement — both frames point the same way. */
        s_pending_target = target;
    }

    /* Deadband check: skip if change is too small. */
    if (s_have_target && fabsf(target - s_last_target_deg) < s_cfg.deadband_deg) {
        s_mode = TRACKER_MODE_IDLE;
        return;
    }

    /* Command the servo. */
    servo_set_angle_deg(target);
    s_last_target_deg = target;
    s_have_target = true;
    s_mode = TRACKER_MODE_TRACKING;
}

tracker_mode_t tracker_get_mode(void)
{
    return s_mode;
}

void tracker_set_enabled(bool enabled)
{
    s_enabled = enabled;
    ESP_LOGI(TAG, "%s", enabled ? "enabled" : "disabled");
}

bool tracker_is_enabled(void)
{
    return s_enabled;
}

void tracker_set_config(const tracker_config_t *cfg)
{
    if (cfg) s_cfg = *cfg;
}

const tracker_config_t *tracker_get_config(void)
{
    return &s_cfg;
}
