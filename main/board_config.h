/* board_config.h — compile-time board selection.
 *
 * Centralizes the 4 GPIO that differ between supported boards. Other
 * constants (mechanical limits, PWM timing, DOA geometry, feed-forward
 * sign) live in their owning modules — they're identical across boards
 * given the same servo + gear + 3DMIC-291 orientation.
 *
 * To switch boards: uncomment exactly one #define below, rebuild, flash.
 * No sdkconfig change needed. */
#ifndef BOARD_CONFIG_H
#define BOARD_CONFIG_H

/* === Pick exactly one board === */
#define BOARD_GOOUUU_S3_CAM    1
// #define BOARD_WAVESHARE_S3_ZERO_M   1

#if defined(BOARD_GOOUUU_S3_CAM)
    /* GOOUUU ESP32-S3-CAM + 3DMIC-291 + JS6620 (270° PWM servo, 15T/20T external gear) */
    #define MIC_CLK1_GPIO       14
    #define MIC_DAT1_GPIO       42
    #define SERVO_GPIO          38
    #define LED_GPIO            48

#elif defined(BOARD_WAVESHARE_S3_ZERO_M)
    /* Waveshare ESP32-S3-Zero + 3DMIC-291 + ZP10S (PWM mode, 15T/20T external gear) */
    #define MIC_CLK1_GPIO       3
    #define MIC_DAT1_GPIO       4
    #define SERVO_GPIO          5
    #define LED_GPIO            48   /* On-board WS2812, may not light correctly */

#else
    #error "Pick a board in board_config.h"
#endif

#endif /* BOARD_CONFIG_H */
