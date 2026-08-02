/* epd_task.h — dedicated task for e-ink rendering + refresh.
 *
 * The task runs at priority configMAX_PRIORITIES - 4 (above httpd at -5,
 * below mic_task at -2). The 14s wait_busy() inside epd_display() can run
 * without starving the audio pipeline (mic_task preempts freely).
 *
 * Notification bits (use epd_task_notify() to set):
 *   EPD_NOTIFY_REFRESH      — REST wrote med state, redraw current mode now
 *   EPD_NOTIFY_HOURLY_TICK  — internal hour boundary (for med_tick scheduling)
 *
 * Safe to call regardless of ENABLE_EPD. With EPD disabled, the task is
 * never created and notify is a no-op. */
#ifndef EPD_TASK_H
#define EPD_TASK_H

#include <stdint.h>

#define EPD_NOTIFY_REFRESH      (1u << 0)
#define EPD_NOTIFY_HOURLY_TICK  (1u << 1)

/* Create the epd task. Idempotent — second call is a no-op.
 * Safe to call regardless of ENABLE_EPD. */
void epd_task_create(void);

/* Set notification bits on the epd task. No-op if task not created yet
 * or ENABLE_EPD=0. */
void epd_task_notify(uint32_t bits);

#endif /* EPD_TASK_H */
