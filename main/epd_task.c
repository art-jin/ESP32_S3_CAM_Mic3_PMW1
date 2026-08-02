/* epd_task.c — Phase 1 skeleton.
 *
 * Phase 1 just initializes the display + detects presence + clears once.
 * Phase 2+ will add: monthly/weekly render + :00/:40 boundary switching +
 * med_tick hourly calls. */
#include "epd_task.h"
#include "board_config.h"

#if ENABLE_EPD

#include "epd.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define TAG "EPD_TASK"

static TaskHandle_t s_epd_task_handle = NULL;

void epd_task_notify(uint32_t bits)
{
    if (s_epd_task_handle) {
        xTaskNotify(s_epd_task_handle, bits, eSetBits);
    }
}

static void epd_task(void *arg)
{
    (void)arg;

    epd_init();

    if (!epd_is_present()) {
        ESP_LOGW(TAG, "idling (no display)");
        /* Even without display, future med_tick integration will run here.
         * For Phase 1, just wait for notifications and discard them. */
        while (1) {
            uint32_t flags;
            (void)xTaskNotifyWait(0, 0xFFFFFFFFu, &flags, portMAX_DELAY);
        }
    }

    ESP_LOGI(TAG, "EPD present, doing initial clear...");
    epd_clear();
    ESP_LOGI(TAG, "initial clear done");

    /* Phase 2+ will replace this loop with calendar rendering + :00/:40
     * boundary switching. For Phase 1, just sleep. */
    while (1) {
        uint32_t flags;
        (void)xTaskNotifyWait(0, 0xFFFFFFFFu, &flags, portMAX_DELAY);
    }
}

void epd_task_create(void)
{
    if (s_epd_task_handle) return;   /* idempotent */

    /* Stack 8192 covers localtime + malloc + log formatting.
     * Priority -4: above httpd (-5, REST API), below mic_task (-2, DOA). */
    BaseType_t r = xTaskCreate(epd_task, "epd", 8192, NULL,
                               configMAX_PRIORITIES - 4, &s_epd_task_handle);
    if (r != pdPASS) {
        ESP_LOGE(TAG, "failed to create epd task");
    }
}

#else  /* !ENABLE_EPD */

void epd_task_create(void) {}
void epd_task_notify(uint32_t bits) { (void)bits; }

#endif  /* ENABLE_EPD */
