/* evlog.h — persistent event ring buffer in NVS for post-mortem diagnostics.
 *
 * Records 8-byte events to a 32-slot ring buffer in NVS namespace "evlog".
 * On boot, evlog_init() prints the prior boot's events and increments a
 * boot counter. New events overwrite the oldest in cyclic fashion.
 *
 * Complements ESP-IDF's coredump-to-flash: coredump captures the panic
 * instant (registers, backtrace), this captures the event flow leading up
 * to the crash (servo commands, mode changes, DOA acceptances).
 *
 * Threading: evlog_record is thread-safe (internal mutex). Safe to call
 * from mic_task, httpd task, or timer callbacks. */
#ifndef EVLOG_H
#define EVLOG_H

#include <stdint.h>

/* 8-byte packed event record. sizeof(event_t) == 8. */
typedef struct __attribute__((packed)) {
    uint16_t seq;        /* global monotonic counter, wraps at 65536 */
    uint8_t  type;       /* EV_* enum value */
    uint8_t  flags;      /* sub-type / context (e.g., source of servo cmd) */
    int16_t  value;      /* numeric payload (angle, azimuth, reason code) */
    uint32_t uptime_ms;  /* esp_timer_get_time() / 1000 at record time */
} event_t;

/* Event types. */
enum {
    EV_BOOT       = 1,  /* flags=0,               value=esp_reset_reason() */
    EV_SWEEP_DONE = 2,  /* flags=0,               value=duration_ms */
    EV_DOA_FIRST  = 3,  /* flags=sextant,         value=azimuth_deg */
    EV_SERVO_CMD  = 4,  /* flags=SRC_*,           value=angle_deg */
    EV_WIFI_UP    = 5,  /* flags=0,               value=0 */
    EV_MODE_CHG   = 6,  /* flags=old_mode,        value=new_mode */
    EV_SHAKE_START= 7,  /* flags=0,               value=center_deg */
    EV_SHAKE_END  = 8,  /* flags=0,               value=final_deg */
};

/* Sources for EV_SERVO_CMD. */
enum {
    SRC_TRACKER = 1,
    SRC_REST    = 2,
    SRC_SHAKE   = 3,
    SRC_IDLE    = 4,
    SRC_BOOT    = 5,
};

/* Initialize: open NVS, increment boot counter, print prior events.
 * Call once from app_main after nvs_flash_init. */
void evlog_init(void);

/* Record an event. Thread-safe. No-op if evlog_init hasn't run yet. */
void evlog_record(uint8_t type, uint8_t flags, int16_t value);

#endif /* EVLOG_H */
