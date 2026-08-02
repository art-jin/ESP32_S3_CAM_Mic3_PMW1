/* epd.c — SSD1683 driver for SuperMini + EVK011-C adapter + Hink-E042A48-A1.
 * See epd.h for the graceful-degradation contract. */
#include "epd.h"
#include "board_config.h"

#if ENABLE_EPD

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdlib.h>
#include <string.h>

#define TAG "EPD"

#define PIN_SCK   EPD_SCK_GPIO
#define PIN_MOSI  EPD_MOSI_GPIO
#define PIN_CS    EPD_CS_GPIO
#define PIN_DC    EPD_DC_GPIO
#define PIN_BS    EPD_BS_GPIO
#define PIN_BUSY  EPD_BUSY_GPIO

static spi_device_handle_t s_spi;
static bool s_epd_present = false;

bool epd_is_present(void) { return s_epd_present; }

static void send_cmd(uint8_t c)
{
    gpio_set_level(PIN_DC, 0);
    spi_transaction_t t = {0};
    t.length = 8;
    t.tx_buffer = &c;
    spi_device_polling_transmit(s_spi, &t);
}

static void send_data(uint8_t d)
{
    gpio_set_level(PIN_DC, 1);
    spi_transaction_t t = {0};
    t.length = 8;
    t.tx_buffer = &d;
    spi_device_polling_transmit(s_spi, &t);
}

static void send_buf(const uint8_t *buf, int len)
{
    gpio_set_level(PIN_DC, 1);
    int off = 0;
    while (off < len) {
        int chunk = len - off;
        if (chunk > 4096) chunk = 4096;
        spi_transaction_t t = {0};
        t.length = chunk * 8;
        t.tx_buffer = buf + off;
        spi_device_polling_transmit(s_spi, &t);
        off += chunk;
    }
}

/* SSD1683 BUSY polarity: HIGH=busy, LOW=idle. Standard convention.
 *
 * DEBUG MODE (no RES pin): we cannot reliably detect "busy done" because
 * without hardware reset, SSD1683's BUSY may be stuck HIGH. We fall back
 * to fixed delays based on typical SSD1683 timings:
 *   - SWRESET: 50ms
 *   - Set RAM window / data writes: 10ms
 *   - Display refresh (cmd 0x20): 14000ms
 * Set WAIT_BUSY_USE_FIXED=1 to use fixed delays (current debug default).
 *
 * When user adds RES, set WAIT_BUSY_USE_FIXED=0 to use proper polling. */
#define WAIT_BUSY_USE_FIXED  1

/* GPIO toggle diagnostic: when 1, epd_init does NOT init SPI — instead it
 * drives each of the 6 EPD GPIOs HIGH for 4s then LOW for 1s in sequence,
 * forever. Probe each pin with a multimeter (or LED + 1k resistor) — should
 * read ~3.3V when log says "<PIN> HIGH", 0V when log says LOW. If a pin
 * reads the same voltage in both phases → wire not actually connected.
 * Set to 0 once wiring is verified. */
#define EPD_GPIO_TOGGLE_TEST 1

#if WAIT_BUSY_USE_FIXED
static void wait_busy_or_delay(int fixed_ms)
{
    vTaskDelay(pdMS_TO_TICKS(fixed_ms));
}
#else
static void wait_busy_or_delay(int fixed_ms)
{
    (void)fixed_ms;
    int ms = 0;
    while (gpio_get_level(PIN_BUSY)) {
        vTaskDelay(pdMS_TO_TICKS(10));
        ms += 10;
        if (ms > 30000) {
            ESP_LOGE(TAG, "BUSY timeout %d ms", ms);
            return;
        }
    }
}
#endif

/* Convenience wrappers for typical wait times */
static void wait_busy_short(void) { wait_busy_or_delay(50); }    /* post-SWRESET, post-config */
static void wait_busy_long(void)  { wait_busy_or_delay(14000); } /* post-refresh */

static void set_ram_window(void)
{
    send_cmd(0x44);
    send_data(0x00);
    send_data(0x31);   /* X: 0..49 (50 bytes = 400 px) */
    send_cmd(0x45);
    send_data(0x00);
    send_data(0x00);
    send_data(0x2B);
    send_data(0x01);   /* Y: 0..299 */
    send_cmd(0x4E);
    send_data(0x00);
    send_cmd(0x4F);
    send_data(0x00);
    send_data(0x00);
}

/* Detect EPD presence by sending SWRESET and sampling BUSY.
 * Real SSD1683 pulls BUSY HIGH for 50-500ms during internal init, then LOW.
 * No hardware (BUSY floating with pull-down) stays LOW.
 * Stuck-broken hardware pulls BUSY HIGH and never releases.
 *
 * Logs raw BUSY samples so we can diagnose "BUSY stuck HIGH" cases
 * (typically: RES pin not connected + POR didn't fully complete, OR
 * BUSY wired to wrong GPIO). */
static bool detect_hardware(void)
{
    int pre = gpio_get_level(PIN_BUSY);
    ESP_LOGI(TAG, "detect: BUSY raw pre-SWRESET = %d (pull-down expected)", pre);

    send_cmd(0x12);   /* SWRESET — kicks SSD1683 into init, drives BUSY HIGH */

    int high_count = 0;
    int low_after_high = 0;
    bool seen_high = false;
    int last_samples[10] = {0};
    for (int i = 0; i < 500; i++) {     /* 500ms window for full BUSY cycle */
        int b = gpio_get_level(PIN_BUSY);
        if (i < 10) last_samples[i] = b;
        if (b) {
            high_count++;
            seen_high = true;
        } else if (seen_high) {
            low_after_high++;
            if (low_after_high >= 5) break;   /* healthy: HIGH then back to LOW */
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    ESP_LOGI(TAG, "detect: 500ms stats high=%d low_after_high=%d first10=[%d%d%d%d%d%d%d%d%d%d]",
             high_count, low_after_high,
             last_samples[0], last_samples[1], last_samples[2], last_samples[3], last_samples[4],
             last_samples[5], last_samples[6], last_samples[7], last_samples[8], last_samples[9]);

    /* Healthy: saw HIGH at least 5 times, AND saw LOW after HIGH at least 5 times.
     *
     * DEBUG OVERRIDE (no RES pin): with RES not connected, BUSY may be stuck HIGH
     * pre-SWRESET because the chip never finished internal POR. We cannot tell from
     * BUSY alone whether the panel is physically present + SPI wired correctly.
     * Force-return true so the init sequence runs anyway — the panel state after
     * init+clear will tell us if SPI is actually working (white = yes, no change = no). */
    (void)high_count; (void)low_after_high;
    return true;
}

void epd_init(void)
{
#if EPD_GPIO_TOGGLE_TEST
    /* Diagnostic mode: drive each EPD GPIO in turn, forever. See comment
     * above the #define for instructions. */
    static const int pins[] = {PIN_SCK, PIN_MOSI, PIN_CS, PIN_DC, PIN_BS, PIN_BUSY};
    static const char *names[] = {"SCK", "MOSI", "CS", "DC", "BS", "BUSY"};
    gpio_config_t toggle_out = {
        .pin_bit_mask = (1ULL << PIN_SCK) | (1ULL << PIN_MOSI) | (1ULL << PIN_CS) |
                        (1ULL << PIN_DC) | (1ULL << PIN_BS) | (1ULL << PIN_BUSY),
        .mode = GPIO_MODE_INPUT_OUTPUT,
    };
    gpio_config(&toggle_out);
    ESP_LOGI(TAG, "=== GPIO TOGGLE TEST ===");
    ESP_LOGI(TAG, "Probe each pad with multimeter (or LED+1k).");
    ESP_LOGI(TAG, "Pin should read ~3.3V when 'HIGH', 0V when 'LOW'.");
    while (1) {
        for (int i = 0; i < 6; i++) {
            /* All LOW first so only one pin is HIGH at a time */
            for (int j = 0; j < 6; j++) gpio_set_level(pins[j], 0);
            vTaskDelay(pdMS_TO_TICKS(500));
            ESP_LOGI(TAG, "[GPIO %d] %s = HIGH for 4s", pins[i], names[i]);
            gpio_set_level(pins[i], 1);
            vTaskDelay(pdMS_TO_TICKS(4000));
            ESP_LOGI(TAG, "[GPIO %d] %s = LOW", pins[i], names[i]);
            gpio_set_level(pins[i], 0);
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
#endif

    /* Outputs: BS + D/C. No RES on this wiring. */
    gpio_config_t out = {
        .pin_bit_mask = (1ULL << PIN_BS) | (1ULL << PIN_DC),
        .mode = GPIO_MODE_INPUT_OUTPUT,
    };
    gpio_config(&out);
    gpio_set_level(PIN_BS, 0);   /* 4-wire SPI mode */

    /* BUSY input with pull-down so an unconnected panel reads LOW (idle)
     * instead of floating random — wait_busy would otherwise spin. */
    gpio_config_t in = {
        .pin_bit_mask = 1ULL << PIN_BUSY,
        .mode = GPIO_MODE_INPUT,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
    };
    gpio_config(&in);

    spi_bus_config_t bus = {
        .sclk_io_num  = PIN_SCK,
        .mosi_io_num  = PIN_MOSI,
        .miso_io_num  = -1,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4096,
    };
    spi_device_interface_config_t dev = {
        .clock_speed_hz = 500000,     /* 500 kHz — safer for long jumper wires */
        .mode = 0,
        .spics_io_num = PIN_CS,
        .queue_size = 1,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &bus, SPI_DMA_CH_AUTO));
    ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &dev, &s_spi));

    /* Hardware detection: probe BUSY after SWRESET. */
    s_epd_present = detect_hardware();
    if (!s_epd_present) {
        ESP_LOGW(TAG, "EPD ABSENT — skipping init (mic/servo unaffected)");
        return;
    }

    /* Real hardware: complete the init sequence. */
    wait_busy_short();        /* wait for SWRESET to finish */

    send_cmd(0x12);           /* SWRESET (formal, after detect) */
    wait_busy_short();

    send_cmd(0x01);           /* Driver output control: 300 lines */
    send_data(0x2B);
    send_data(0x01);
    send_data(0x00);
    send_cmd(0x11);           /* Data entry: X increment, Y decrement */
    send_data(0x03);
    send_cmd(0x44);
    send_data(0x00);
    send_data(0x31);
    send_cmd(0x45);
    send_data(0x00);
    send_data(0x00);
    send_data(0x2B);
    send_data(0x01);
    send_cmd(0x3C);           /* Border waveform: red LUT1 */
    send_data(0x05);
    send_cmd(0x18);           /* Temperature sensor: internal */
    send_data(0x80);
    send_cmd(0x4E);
    send_data(0x00);
    send_cmd(0x4F);
    send_data(0x00);
    send_data(0x00);

    wait_busy_short();
    ESP_LOGI(TAG, "SSD1683 init done");
}

void epd_clear(void)
{
    if (!s_epd_present) return;

    set_ram_window();
    uint8_t *buf = malloc(EPD_BUF_SIZE);
    if (!buf) {
        ESP_LOGE(TAG, "epd_clear: malloc failed");
        return;
    }
    memset(buf, 0xFF, EPD_BUF_SIZE);
    send_cmd(0x24);          /* BW: all white */
    send_buf(buf, EPD_BUF_SIZE);
    memset(buf, 0x00, EPD_BUF_SIZE);
    send_cmd(0x26);          /* RED: none */
    send_buf(buf, EPD_BUF_SIZE);
    free(buf);
    send_cmd(0x22);          /* Display update sequence */
    send_data(0xF7);
    send_cmd(0x20);          /* Refresh */
    wait_busy_long();
}

void epd_display(const uint8_t *bw, const uint8_t *red)
{
    if (!s_epd_present) return;

    set_ram_window();
    send_cmd(0x24);
    send_buf(bw, EPD_BUF_SIZE);
    send_cmd(0x26);
    send_buf(red, EPD_BUF_SIZE);
    send_cmd(0x22);
    send_data(0xF7);
    send_cmd(0x20);
    wait_busy_long();
}

void epd_sleep(void)
{
    if (!s_epd_present) return;
    send_cmd(0x10);          /* Deep sleep mode */
    send_data(0x01);
}

#else  /* !ENABLE_EPD */

/* Stubs for boards without EPD wiring. All zero-cost no-ops. */
void epd_init(void) {}
bool epd_is_present(void) { return false; }
void epd_clear(void) {}
void epd_display(const uint8_t *bw, const uint8_t *red) { (void)bw; (void)red; }
void epd_sleep(void) {}

#endif  /* ENABLE_EPD */
