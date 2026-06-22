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

    /* FEED-FORWARD COMPENSATION: convert source azimuth from array frame
     * to room frame by adding the current servo angle. Without this, any
     * servo motion changes the array's orientation, which shifts the
     * source's perceived azimuth, which flips stable_sextant, which
     * commands the servo back — closed-loop oscillation. With this,
     * α_room is invariant to servo position (mathematically), so target
     * is stable across servo motions.
     *
     *   α_room = α_array + β_servo
     *   target = α_room - 180° + home
     *
     * α_array ∈ [0, 360), β_servo ∈ [-20, +20], so α_room ∈ [-20, 380).
     * target = α_room - 180 ∈ [-200, 200]. Clamp ±20° takes care of it.
     * The math doesn't need explicit wraparound handling because the
     * clamp rejects anything far from 0 anyway. */
    float alpha_room = doa->azimuth_deg + servo_get_angle_deg();
    float target;
    if (s_cfg.conservative_mode && doa->stable_sextant >= 0) {
        int sextant_center_az = doa->stable_sextant * 60;
        /* Apply feed-forward here too — add current servo angle to the
         * sextant center, then subtract 180. */
        target = (float)sextant_center_az + servo_get_angle_deg()
               - 180.0f + s_cfg.home_deg;
    } else {
        target = alpha_room - 180.0f + s_cfg.home_deg;
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
