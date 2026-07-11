#include "tracker.h"

#include <math.h>
#include <string.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "servo.h"
#include "evlog.h"

static const char *TAG = "tracker";

static tracker_config_t s_cfg = TRACKER_DEFAULT_CONFIG;
static tracker_mode_t   s_mode = TRACKER_MODE_IDLE;
static bool             s_enabled = true;
static float            s_last_target_deg = 0.0f;   /* last commanded angle */
static bool             s_have_target = false;      /* any command issued yet */

/* EMA + plausibility state for α_room smoothing. */
static float s_sin_ema = 0.0f, s_cos_ema = 0.0f;
static bool  s_ema_init = false;
static float s_prev_alpha_room = -1.0f;

/* Idle-return state (Phase A).
 * Tracks the time of the last valid 3-mic DOA frame and the time of the
 * last tracker_update call. When no 3-mic frame has been seen for longer
 * than idle_return_threshold_s, the servo is stepped toward home at
 * idle_return_rate_deg_per_s. */
static int64_t s_last_3mic_us  = 0;
static int64_t s_last_update_us = 0;

/* 2-of-3 agreement ring buffer (Phase A).
 * Replaces the prior "two consecutive frames must agree" rule, which
 * penalized speech: at 16% 3-mic frame rate, a single noise/2-mic frame
 * sandwiched between two good frames broke the streak and reset the wait.
 * The new rule tolerates one interrupted frame in three.
 *
 * Confirmation semantics: the LATEST target must agree with at least one
 * of the (up to 2) previous targets. This means a transient on the latest
 * frame is still rejected (good — we don't want to act on noise), but a
 * transient in the middle of two good frames is tolerated. */
static float s_agree_buf[3];
static int   s_agree_n;
static int   s_agree_idx;

/* Push target into the ring buffer and check if the latest agrees with
 * any prior entry within tol. Returns 1 (confirm) with *confirmed_target
 * set to the latest target; returns 0 (no confirm) otherwise. Targets
 * are servo angles in degrees, clamped to ±33°, so no wraparound. */
static int agreement_push_check(float target, float tol, float *confirmed_target)
{
    s_agree_buf[s_agree_idx] = target;
    s_agree_idx = (s_agree_idx + 1) % 3;
    if (s_agree_n < 3) s_agree_n++;
    if (s_agree_n < 2) return 0;
    /* Slot that holds the latest target just after the push above.
     * It's at index (s_agree_idx - 1 + 3) % 3. */
    for (int i = 1; i < s_agree_n; i++) {
        int idx = (s_agree_idx - 1 - i + 3) % 3;
        if (fabsf(s_agree_buf[idx] - target) <= tol) {
            *confirmed_target = target;
            return 1;
        }
    }
    return 0;
}

void tracker_init(const tracker_config_t *cfg)
{
    if (cfg) s_cfg = *cfg;
    s_mode = TRACKER_MODE_IDLE;
    s_have_target = false;
    s_ema_init = false;
    s_prev_alpha_room = -1.0f;
    s_agree_n = 0;
    s_agree_idx = 0;
    int64_t now_us = esp_timer_get_time();
    s_last_3mic_us  = now_us;
    s_last_update_us = now_us;
    ESP_LOGI(TAG, "init: home=%+.1f°  deadband=%.1f°  min_conf=%.2f  "
             "out_of_range=%.0f°  agreement=%.1f°  conservative=%d  "
             "idle_return=%.0fs@%.1f°/s",
             s_cfg.home_deg, s_cfg.deadband_deg, s_cfg.min_confidence,
             s_cfg.out_of_range_deg, s_cfg.target_agreement_deg,
             s_cfg.conservative_mode ? 1 : 0,
             s_cfg.idle_return_threshold_s,
             s_cfg.idle_return_rate_deg_per_s);
}

void tracker_update(const doa_result_t *doa)
{
    int64_t now_us = esp_timer_get_time();
    /* dt_s for rate-limited actions (idle return). Computed from the last
     * time we actually COMMANDED the servo (not the last call) — otherwise
     * motion-pause frames would shrink dt and limit the effective rate to
     * ~1°/s instead of the configured 2.5°/s. s_last_update_us is updated
     * only at the servo-command sites below.
     *
     * Cap to 1 s so a long pause (no commands issued) doesn't produce a
     * huge first step when activity resumes. */
    float dt_s = 0.1f;
    if (s_last_update_us > 0) {
        int64_t dt_us = now_us - s_last_update_us;
        if (dt_us > 0 && dt_us < 1000000) dt_s = (float)dt_us / 1e6f;
    }

    if (!s_enabled) {
        s_mode = TRACKER_MODE_DISABLED;
        return;
    }
    if (doa == NULL) {
        s_mode = TRACKER_MODE_SUPPRESSED;
        return;
    }

    /* Refresh idle timer whenever we see a valid 3-mic frame — even if
     * later checks (motion-pause, agreement) end up suppressing this
     * frame. The point is "user is still actively speaking." */
    if (doa->mode == DOA_MODE_3MIC &&
        doa->confidence >= s_cfg.min_confidence) {
        s_last_3mic_us = now_us;
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

    /* IDLE RETURN HOME (Phase A): if no valid 3-mic frame has arrived
     * for idle_return_threshold_s, step the servo toward home at
     * idle_return_rate_deg_per_s. Prevents the servo from sitting at
     * ±33° indefinitely after the user stops speaking — both a UX issue
     * and a cause of limit-position buzz coupling into the mic PCB.
     *
     * Bypasses deadband and agreement checks (deterministic action, not
     * a tracking decision). Each step is small (< deadband) and re-enters
     * motion-pause on the next call, so the loop naturally limit-cycles
     * between "step, holdoff, step" until home is reached.
     * Stops within 0.5° of home to avoid buzzing on the centering pulse. */
    if (s_have_target &&
        s_cfg.idle_return_threshold_s > 0.0f &&
        (now_us - s_last_3mic_us) >=
            (int64_t)(s_cfg.idle_return_threshold_s * 1e6f)) {
        float current = servo_get_angle_deg();
        if (fabsf(current) > 0.5f) {
            float step = s_cfg.idle_return_rate_deg_per_s * dt_s;
            float new_angle;
            if (current > 0.0f) {
                new_angle = current - step;
                if (new_angle < 0.0f) new_angle = 0.0f;
            } else {
                new_angle = current + step;
                if (new_angle > 0.0f) new_angle = 0.0f;
            }
            servo_set_angle_deg(new_angle);
            evlog_record(EV_SERVO_CMD, SRC_IDLE, (int16_t)new_angle);
            s_last_target_deg = new_angle;
            s_last_update_us = now_us;   /* mark command time for next dt */
            s_mode = TRACKER_MODE_IDLE;
            return;
        }
        /* Already at (or within 0.5° of) home — fall through to normal
         * flow, which will likely SUPPRESS the frame (no movement). */
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
    /* Feed-forward sign: +1 if servo CW → array CW (old board),
     * -1 if servo CW → array CCW (new board, opposite gear mesh).
     * Toggle if servo tracks away from user instead of toward. */
    float alpha_room_raw = doa->azimuth_deg - servo_get_angle_deg();

    /* Plausibility check: reject physically impossible jumps (>60° in one
     * 50ms frame). Humans can't move that fast; these are GCC-PHAT noise
     * peaks or servo-motion artifacts. On rejection, partially drift
     * s_prev toward the new reading (30%) to prevent stale-lock where a
     * single wrong initial reading blocks all subsequent correct ones. */
    if (s_prev_alpha_room >= 0.0f) {
        float delta = alpha_room_raw - s_prev_alpha_room;
        if (delta > 180.0f) delta -= 360.0f;
        if (delta < -180.0f) delta += 360.0f;
        if (fabsf(delta) > 60.0f) {
            s_prev_alpha_room += delta * 0.3f;  /* drift to prevent stale-lock */
            s_mode = TRACKER_MODE_SUPPRESSED;
            return;
        }
    }
    s_prev_alpha_room = alpha_room_raw;

    /* EMA smoothing in sin/cos space (handles 0/360° wraparound correctly).
     * α=0.3 → ~100ms time constant at 20Hz, attenuates single-frame noise
     * by 70% while tracking real position changes with <200ms added lag. */
    float sin_r = sinf(alpha_room_raw * M_PI / 180.0f);
    float cos_r = cosf(alpha_room_raw * M_PI / 180.0f);
    if (!s_ema_init) {
        s_sin_ema = sin_r;
        s_cos_ema = cos_r;
        s_ema_init = true;
    } else {
        s_sin_ema = 0.7f * s_sin_ema + 0.3f * sin_r;
        s_cos_ema = 0.7f * s_cos_ema + 0.3f * cos_r;
    }
    float alpha_room = atan2f(s_sin_ema, s_cos_ema) * 180.0f / M_PI;
    if (alpha_room < 0.0f) alpha_room += 360.0f;

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

    /* 2-OF-3 AGREEMENT CHECK (Phase A): push the latest target into a
     * 3-slot ring buffer and require it to agree with at least one prior
     * entry within target_agreement_deg. Tolerates one noise/2-mic frame
     * in three without resetting the confirmation streak — important for
     * speech where 3-mic frame rate is only ~16%. */
    if (s_cfg.target_agreement_deg > 0.0f) {
        float confirmed_target = target;
        if (!agreement_push_check(target, s_cfg.target_agreement_deg,
                                  &confirmed_target)) {
            s_mode = TRACKER_MODE_SUPPRESSED;
            return;
        }
        target = confirmed_target;
    }

    /* Deadband check: skip if change is too small. */
    if (s_have_target && fabsf(target - s_last_target_deg) < s_cfg.deadband_deg) {
        s_mode = TRACKER_MODE_IDLE;
        return;
    }

    /* Velocity limit: cap single-frame target change to 15° (= 300°/s at
     * 20Hz). Matches JS6620's physical speed limit. Prevents jarring
     * large-angle jumps (e.g., user walks 3oc→7oc = 120° step); the
     * remainder carries over to subsequent frames naturally. */
    if (s_have_target) {
        float diff = target - s_last_target_deg;
        float max_delta = 15.0f;
        if (diff > max_delta) target = s_last_target_deg + max_delta;
        if (diff < -max_delta) target = s_last_target_deg - max_delta;
    }

    /* Command the servo. */
    if (!s_have_target) {
        evlog_record(EV_DOA_FIRST, (uint8_t)doa->stable_sextant,
                     (int16_t)doa->azimuth_deg);
    }
    evlog_record(EV_SERVO_CMD, SRC_TRACKER, (int16_t)target);
    servo_set_angle_deg(target);
    s_last_target_deg = target;
    s_last_update_us = now_us;   /* mark command time for idle-return dt */
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

void tracker_reset_state(void)
{
    s_have_target = false;
    s_ema_init = false;
    s_prev_alpha_room = -1.0f;
    s_agree_n = 0;
    s_agree_idx = 0;
    s_last_3mic_us = esp_timer_get_time();
    s_last_target_deg = 0.0f;
}

void tracker_set_config(const tracker_config_t *cfg)
{
    if (cfg) s_cfg = *cfg;
}

const tracker_config_t *tracker_get_config(void)
{
    return &s_cfg;
}
