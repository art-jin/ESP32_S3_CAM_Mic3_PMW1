/* epd.h — SSD1683 e-ink driver for Hink-E042A48-A1 (4.2", 400x300, tri-color).
 *
 * Ported from ArtInk4 (https://github.com/art-jin/ESP32S3Ink4) with these
 * changes for the SuperMini integration:
 *   - Pin defines come from board_config.h (SuperMini uses GPIO 8-13)
 *   - PIN_RES removed entirely (user wiring omits RES; relies on internal
 *     POR + software reset cmd 0x12)
 *   - PIN_BS is software-driven (GPIO 12) instead of hardwired to GND
 *   - Hardware detection at init: if BUSY never goes HIGH after SWRESET,
 *     marks display absent and all public functions become no-ops
 *   - BUSY internal pull-down enabled to prevent float-reading HIGH
 *   - wait_busy timeout reduced 30s → 3s for faster fail when half-broken
 *
 * Graceful degradation contract: when ENABLE_EPD=0 or hardware absent,
 * all epd_* functions are safe to call — they no-op and return immediately.
 * mic_task, servo, REST API are completely unaffected. */
#ifndef EPD_H
#define EPD_H

#include <stdint.h>
#include <stdbool.h>

#define EPD_WIDTH      400
#define EPD_HEIGHT     300
#define EPD_BUF_SIZE   (EPD_WIDTH / 8 * EPD_HEIGHT)   /* 15000 bytes */

/* Initialize SPI bus + probe hardware. Sets the internal s_epd_present flag.
 * Safe to call regardless of ENABLE_EPD — stub when disabled. */
void epd_init(void);

/* True iff EPD hardware was detected at last epd_init(). Always false when
 * ENABLE_EPD=0. Safe to call from any task. */
bool epd_is_present(void);

/* Clear the display to all-white. Blocks ~14s if hardware present.
 * No-op when absent or ENABLE_EPD=0. */
void epd_clear(void);

/* Push BW + RED framebuffers to the display and refresh. Blocks ~14s.
 * Each buffer is EPD_BUF_SIZE bytes. Bit semantics (SSD1683):
 *   BW[i] bit=1 → white, bit=0 → black (or red if RED[i] bit=1)
 *   RED[i] bit=1 → red, bit=0 → no red
 * No-op when absent or ENABLE_EPD=0. */
void epd_display(const uint8_t *bw, const uint8_t *red);

/* Put SSD1683 into deep sleep. No-op when absent or ENABLE_EPD=0. */
void epd_sleep(void);

#endif /* EPD_H */
