#pragma once

#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

/* I²S PDM RX multi-DIN capture for the 3DMIC-291 3-mic array on the
 * GOOUUU ESP32-S3-CAM board. Pinout per ArthurReadMe.md (PMW1 build):
 *
 *   3DMIC-291  | ESP32-S3 GPIO | Role
 *   -----------+---------------+-------------------------------------------
 *   CLK0       | GPIO 1        | I²S PDM RX CLK output (drives M3 + M1)
 *   CLK1       | GPIO 14       | same I²S CLK, fanned out via GPIO matrix
 *   DAT0       | GPIO 2        | DIN[0] — M3 (L slot) + M1 (R slot) via PDM clock phase
 *   DAT1       | GPIO 42       | DIN[1] — M2 (L slot)
 *
 * The S3 I²S PDM RX peripheral has only one CLK pin. CLK1 is driven by
 * copying the I²S0 RX WS signal to GPIO 14 through the GPIO matrix so
 * both 3DMIC clock inputs see the same hardware edge — mandatory for DOA.
 *
 * Note on the L/R slot collapse: empirical testing on the S3_CAM_MIC3
 * sibling project found that enabling both LINE0_SLOT_LEFT and
 * LINE0_SLOT_RIGHT on DIN[0] sometimes yields *identical* PCM in the two
 * slots (the on-chip PDM2PCM collapses them). When that happens, only
 * 2 of the 3 mics are actually independent. doa_process() detects this
 * at runtime and falls back to 2-mic mode. */
#define MIC_CLK0_GPIO     1
#define MIC_CLK1_GPIO     14
#define MIC_DAT0_GPIO     2
#define MIC_DAT1_GPIO     42

#define MIC_SAMPLE_RATE_HZ   48000
#define MIC_WINDOW_MS        100
#define MIC_NUM_CHANNELS     3   /* LINE0_L | LINE0_R | LINE1_L */
#define MIC_WINDOW_BYTES     (MIC_SAMPLE_RATE_HZ * MIC_NUM_CHANNELS * 2 * MIC_WINDOW_MS / 1000)
#define MIC_WINDOW_SAMPLES   (MIC_WINDOW_BYTES / sizeof(int16_t))
#define MIC_WINDOW_PER_CH    (MIC_WINDOW_SAMPLES / MIC_NUM_CHANNELS)

esp_err_t mic_capture_init(void);

/* Blocking read of one PCM window. Returns ESP_OK and *got_bytes ==
 * MIC_WINDOW_BYTES on success. */
esp_err_t mic_capture_read(int16_t *buf, size_t buf_bytes, size_t *got_bytes);
