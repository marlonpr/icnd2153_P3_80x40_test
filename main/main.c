#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>

#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_attr.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "soc/gpio_struct.h"

/*
 * ESP32-S3 first-light diagnostic for an 80x40 panel using:
 *   - ICND2153/ICN2053-family memory/S-PWM column drivers
 *   - HX6158H serial row drivers
 *
 * This is intentionally bit-banged. It does not initialize W5500 Ethernet.
 
 
 */
 
 
 
 
 /*
 //============================ HUB75 ETH Development Board ================================

 // Upper RGB
 config.pins.r1 = 33;
 config.pins.g1 = 34;
 config.pins.b1 = 35;

 // Lower RGB
 config.pins.r2 = 36;
 config.pins.g2 = 37;
 config.pins.b2 = 38;

 // Address
 config.pins.a = 1;
 config.pins.b = 2;
 config.pins.c = 15;
 config.pins.d = 45;//45 
 config.pins.e = 46; //46

 // Control
 config.pins.lat = 16;
 config.pins.oe  = 21;
 config.pins.clk = 47;

 //==========================================================================================
 */
/* ----------------------------- GPIO mapping ----------------------------- */
/* Matches the ESP32-S3 ETH HUB75 map discussed previously. */
#define PIN_R1       33
#define PIN_G1       34
#define PIN_B1       35
#define PIN_R2       36
#define PIN_G2       37
#define PIN_B2       38

#define PIN_ROW_A    1//39
#define PIN_ROW_B    2//40
#define PIN_ROW_C    15//41
#define PIN_ROW_D    45//42
#define PIN_ROW_E    46//45

#define PIN_LE        16   /* 1 HUB75 LAT -> ICND LE */
#define PIN_GCLK      21   /* 2 HUB75 OE  -> ICND GCLK/PWCLK */
#define PIN_DCLK      47   /* 3 HUB75 CLK -> ICND DCLK */

/* -------------------------- Panel configuration ------------------------- */
#define PANEL_WIDTH                 80U
#define PANEL_HEIGHT                40U
#define PANEL_SCAN_ROWS             20U
#define ICND_CHANNELS               16U
#define ICND_DRIVERS_PER_CHAIN      (PANEL_WIDTH / ICND_CHANNELS) /* 5 */

/* Probable 1/20 setting. Alternates: 0x1F70 and 0x0F70. */
#ifndef ICND_REG1_VALUE
#define ICND_REG1_VALUE             0x1370U
#endif

#define ICND_REG2_VALUE             0xFFFFU
#define ICND_REG3_VALUE             0x40F3U
#define ICND_REG4_VALUE             0x0000U
#define ICND_DEBUG_VALUE            0x0000U

/* Preserve original demo total: 58 + (5 x 16) = 138 GCLK pulses. */
#ifndef GCLK_TOTAL_PER_SLOT
#define GCLK_TOTAL_PER_SLOT         138U  //138U
#endif
#define GCLK_UPLOAD_PULSES          (ICND_DRIVERS_PER_CHAIN * 16U)
#define GCLK_LEADING_PULSES         \
    ((GCLK_TOTAL_PER_SLOT > GCLK_UPLOAD_PULSES) ? \
     (GCLK_TOTAL_PER_SLOT - GCLK_UPLOAD_PULSES) : 0U)

/* Default hypothesis: A=row clock, C=row serial data. Set 0 to swap. */
#ifndef HX_A_IS_CLOCK
#define HX_A_IS_CLOCK               1
#endif
#if HX_A_IS_CLOCK
#define PIN_ROW_CLOCK               PIN_ROW_A
#define PIN_ROW_DATA                PIN_ROW_C
#else
#define PIN_ROW_CLOCK               PIN_ROW_C
#define PIN_ROW_DATA                PIN_ROW_A
#endif

/* B is not proven to be blank/OE. Keep LOW initially. */
#ifndef HX_USE_B_AS_BLANK
#define HX_USE_B_AS_BLANK           0
#endif

#define HX_TOKEN_PERIOD             PANEL_SCAN_ROWS
#define HX_CLEAR_CLOCKS             48U
#define FRAME_BYTES                 (PANEL_WIDTH * PANEL_HEIGHT * 3U)
#define TEST_LEVEL                  255U //12U 64U 128U

#if CONFIG_FREERTOS_UNICORE
#define REFRESH_CORE                0
#else
#define REFRESH_CORE                1
#endif

static const char *TAG = "ICND2153_TEST";
static uint8_t s_fb[2][FRAME_BYTES];
static volatile uint8_t s_front = 0;
static uint8_t s_hx_state = 0;

/* ---------------------------- Fast GPIO access -------------------------- */
static inline void IRAM_ATTR pin_write(uint8_t pin, bool high)
{
    if (pin < 32U) {
        const uint32_t mask = 1UL << pin;

        if (high) {
            GPIO.out_w1ts = mask;
        } else {
            GPIO.out_w1tc = mask;
        }
    } else {
        const uint32_t mask = 1UL << (pin - 32U);

        if (high) {
            GPIO.out1_w1ts.val = mask;
        } else {
            GPIO.out1_w1tc.val = mask;
        }
    }
}

static void configure_output(gpio_num_t pin)
{
    gpio_config_t cfg = {
        .pin_bit_mask = 1ULL << pin,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&cfg));
}

static inline void IRAM_ATTR pulse_dclk(void)
{
    pin_write(PIN_DCLK, true);
    pin_write(PIN_DCLK, false);
}

static inline void IRAM_ATTR pulse_gclk(void)
{
    pin_write(PIN_GCLK, true);
    pin_write(PIN_GCLK, false);
}

static void IRAM_ATTR send_gclk(uint16_t clocks)
{
    while (clocks-- > 0U) pulse_gclk();
}

/* Hold LE high while pulsing DCLK N times: ICND command encoding. */
static void IRAM_ATTR send_latch_command(uint8_t clocks)
{
    pin_write(PIN_LE, true);
    while (clocks-- > 0U) pulse_dclk();
    pin_write(PIN_LE, false);
}

/* Send six parallel 16-bit grayscale words, MSB first. */
static void IRAM_ATTR send_six_words(
    uint16_t r1, uint16_t g1, uint16_t b1,
    uint16_t r2, uint16_t g2, uint16_t b2,
    uint8_t latch_bits)
{
    uint16_t mask = 0x8000U;
    uint8_t bit = 0U;

    pin_write(PIN_LE, false);

    while (mask != 0U) {
        pin_write(PIN_DCLK, false);

        if (latch_bits != 0U && bit == (uint8_t)(16U - latch_bits)) {
            pin_write(PIN_LE, true);
        }

        pin_write(PIN_R1, (r1 & mask) != 0U);
        pin_write(PIN_G1, (g1 & mask) != 0U);
        pin_write(PIN_B1, (b1 & mask) != 0U);
        pin_write(PIN_R2, (r2 & mask) != 0U);
        pin_write(PIN_G2, (g2 & mask) != 0U);
        pin_write(PIN_B2, (b2 & mask) != 0U);

        pin_write(PIN_DCLK, true);
        mask >>= 1U;
        bit++;
    }

    pin_write(PIN_DCLK, false);
    pin_write(PIN_LE, false);
}

static void send_value_all_drivers(uint16_t value, uint8_t latch_bits)
{
    for (uint8_t driver = 0; driver < ICND_DRIVERS_PER_CHAIN; driver++) {
        bool last = driver == (ICND_DRIVERS_PER_CHAIN - 1U);
        send_six_words(value, value, value, value, value, value,
                       last ? latch_bits : 0U);
    }
}

static inline void IRAM_ATTR icnd_vsync(void)
{
    send_latch_command(3U);
}

static void icnd_start(void)
{
    ESP_LOGI(TAG, "REG1=0x%04X, GCLK/slot=%u, leading=%u",
             ICND_REG1_VALUE, GCLK_TOTAL_PER_SLOT, GCLK_LEADING_PULSES);

    send_latch_command(14U);                 /* pre-active */
    send_latch_command(12U);                 /* enable outputs */
    icnd_vsync();

    send_latch_command(14U);
    send_value_all_drivers(ICND_REG1_VALUE, 4U);

    send_latch_command(14U);
    send_value_all_drivers(ICND_REG2_VALUE, 6U);

    send_latch_command(14U);
    send_value_all_drivers(ICND_REG3_VALUE, 8U);

    send_latch_command(14U);
    send_value_all_drivers(ICND_REG4_VALUE, 10U);

    send_latch_command(14U);
    send_value_all_drivers(ICND_DEBUG_VALUE, 2U);

    pin_write(PIN_LE, false);
    pin_write(PIN_DCLK, false);
}

/* ----------------------------- HX6158H rows ------------------------------ */
static inline void IRAM_ATTR hx_shift_bit(bool data)
{
    pin_write(PIN_ROW_DATA, data);
    pin_write(PIN_ROW_CLOCK, false);
    pin_write(PIN_ROW_CLOCK, true);
    pin_write(PIN_ROW_CLOCK, false);
    pin_write(PIN_ROW_DATA, false);
}

static void hx_clear(void)
{
#if HX_USE_B_AS_BLANK
    pin_write(PIN_ROW_B, true);
#else
    pin_write(PIN_ROW_B, false);
#endif

    for (uint8_t i = 0; i < HX_CLEAR_CLOCKS; i++) hx_shift_bit(false);
    s_hx_state = 0;

#if HX_USE_B_AS_BLANK
    pin_write(PIN_ROW_B, false);
#endif
}

/* Inject one token every 20 row clocks. */
static inline void IRAM_ATTR hx_scan_next(void)
{
#if HX_USE_B_AS_BLANK
    pin_write(PIN_ROW_B, true);
#endif

    hx_shift_bit(s_hx_state == 0U);
    s_hx_state++;
    if (s_hx_state >= HX_TOKEN_PERIOD) s_hx_state = 0U;

#if HX_USE_B_AS_BLANK
    pin_write(PIN_ROW_B, false);
#endif
}

/* -------------------------- Frame upload engine ------------------------- */
static inline uint16_t to_word(uint8_t value)
{
    return (uint16_t)value << 8U;
}

static inline void IRAM_ATTR send_pixel_packet(
    uint8_t section, uint8_t channel, uint8_t memory_row,
    const uint8_t *frame)
{
    uint32_t x = (uint32_t)section * ICND_CHANNELS + channel;
    uint32_t y2 = (uint32_t)memory_row + PANEL_SCAN_ROWS;
    uint32_t i1 = (((uint32_t)memory_row * PANEL_WIDTH) + x) * 3U;
    uint32_t i2 = ((y2 * PANEL_WIDTH) + x) * 3U;
    bool last = section == (ICND_DRIVERS_PER_CHAIN - 1U);

    send_six_words(
        to_word(frame[i1 + 0U]), to_word(frame[i1 + 1U]), to_word(frame[i1 + 2U]),
        to_word(frame[i2 + 0U]), to_word(frame[i2 + 1U]), to_word(frame[i2 + 2U]),
        last ? 1U : 0U);
}

static void refresh_once(const uint8_t *frame)
{
    icnd_vsync();

    for (uint8_t row = 0; row < PANEL_SCAN_ROWS; row++) {
        for (uint8_t channel = 0; channel < ICND_CHANNELS; channel++) {
            hx_scan_next();
            send_gclk(GCLK_LEADING_PULSES);

            for (uint8_t section = 0;
                 section < ICND_DRIVERS_PER_CHAIN;
                 section++) {
                send_gclk(16U);
                send_pixel_packet(section, channel, row, frame);
            }
        }
    }
}

/* ------------------------------ Test images ------------------------------ */
static inline void set_pixel(uint8_t *fb, uint16_t x, uint16_t y,
                             uint8_t r, uint8_t g, uint8_t b)
{
    if (x >= PANEL_WIDTH || y >= PANEL_HEIGHT) return;
    uint32_t i = (((uint32_t)y * PANEL_WIDTH) + x) * 3U;
    fb[i + 0U] = r;
    fb[i + 1U] = g;
    fb[i + 2U] = b;
}

static void fill(uint8_t *fb, uint8_t r, uint8_t g, uint8_t b)
{
    for (uint16_t y = 0; y < PANEL_HEIGHT; y++)
        for (uint16_t x = 0; x < PANEL_WIDTH; x++)
            set_pixel(fb, x, y, r, g, b);
}

static void make_bands(uint8_t *fb)
{
    memset(fb, 0, FRAME_BYTES);
    for (uint16_t y = 0; y < PANEL_HEIGHT; y++) {
        for (uint16_t x = 0; x < PANEL_WIDTH; x++) {
            switch (x / 16U) {
                case 0: set_pixel(fb, x, y, TEST_LEVEL, 0, 0); break;
                case 1: set_pixel(fb, x, y, 0, TEST_LEVEL, 0); break;
                case 2: set_pixel(fb, x, y, 0, 0, TEST_LEVEL); break;
                case 3: set_pixel(fb, x, y, TEST_LEVEL, TEST_LEVEL, 0); break;
                default:set_pixel(fb, x, y, TEST_LEVEL, 0, TEST_LEVEL); break;
            }
        }
    }
}

static void make_boundaries(uint8_t *fb)
{
    static const uint16_t xs[] = {0,15,16,31,32,47,48,63,64,79};
    static const uint16_t ys[] = {0,19,20,39};
    memset(fb, 0, FRAME_BYTES);

    for (size_t n = 0; n < sizeof(xs)/sizeof(xs[0]); n++)
        for (uint16_t y = 0; y < PANEL_HEIGHT; y++)
            set_pixel(fb, xs[n], y, TEST_LEVEL, TEST_LEVEL, TEST_LEVEL);

    for (size_t n = 0; n < sizeof(ys)/sizeof(ys[0]); n++)
        for (uint16_t x = 0; x < PANEL_WIDTH; x++)
            set_pixel(fb, x, ys[n], TEST_LEVEL, TEST_LEVEL, TEST_LEVEL);
}

static void make_row(uint8_t *fb, uint16_t row)
{
    memset(fb, 0, FRAME_BYTES);
    for (uint16_t x = 0; x < PANEL_WIDTH; x++)
        set_pixel(fb, x, row, TEST_LEVEL, TEST_LEVEL, TEST_LEVEL);
    set_pixel(fb, 0, row, TEST_LEVEL, 0, 0);
    set_pixel(fb, PANEL_WIDTH - 1U, row, 0, 0, TEST_LEVEL);
}

static uint8_t *back_buffer(void)
{
    uint8_t current = __atomic_load_n(&s_front, __ATOMIC_ACQUIRE);
    return s_fb[current ^ 1U];
}

static void publish_back(void)
{
    uint8_t current = __atomic_load_n(&s_front, __ATOMIC_ACQUIRE);
    __atomic_store_n(&s_front, current ^ 1U, __ATOMIC_RELEASE);
}

/* -------------------------------- Tasks ---------------------------------- */
#define REFRESH_IDLE_INTERVAL 16U

static void refresh_task(void *arg)
{
    (void)arg;

    int64_t start = esp_timer_get_time();
    uint32_t count = 0U;
    uint32_t idle_counter = 0U;

    while (true) {
        const uint8_t front =
            __atomic_load_n(
                &s_front,
                __ATOMIC_ACQUIRE);

        refresh_once(s_fb[front]);

        count++;
        idle_counter++;

        /*
         * Do not pause after every complete framebuffer upload.
         * Allow the Idle task and watchdog to run periodically.
         */
        if (idle_counter >= REFRESH_IDLE_INTERVAL) {
            idle_counter = 0U;
            vTaskDelay(1);
        }

        const int64_t now = esp_timer_get_time();

        if ((now - start) >= 5000000LL) {
            const float seconds =
                (float)(now - start) / 1000000.0f;

            ESP_LOGI(
                TAG,
                "Refresh uploads %.2f/s, HX state=%u",
                (float)count / seconds,
                s_hx_state);

            start = now;
            count = 0U;
        }
    }
}

static void pattern_task(void *arg)
{
    (void)arg;
    while (true) {
        ESP_LOGI(TAG, "Pattern: RED");
        fill(back_buffer(), TEST_LEVEL, 0, 0); publish_back();
        vTaskDelay(pdMS_TO_TICKS(4000));

        ESP_LOGI(TAG, "Pattern: GREEN");
        fill(back_buffer(), 0, TEST_LEVEL, 0); publish_back();
        vTaskDelay(pdMS_TO_TICKS(4000));

        ESP_LOGI(TAG, "Pattern: BLUE");
        fill(back_buffer(), 0, 0, TEST_LEVEL); publish_back();
        vTaskDelay(pdMS_TO_TICKS(4000));

        ESP_LOGI(TAG, "Pattern: WHITE");
        fill(back_buffer(), TEST_LEVEL, TEST_LEVEL, TEST_LEVEL); publish_back();
        vTaskDelay(pdMS_TO_TICKS(4000));

        ESP_LOGI(TAG, "Pattern: five 16-column sections");
        make_bands(back_buffer()); publish_back();
        vTaskDelay(pdMS_TO_TICKS(6000));

        ESP_LOGI(TAG, "Pattern: boundaries");
        make_boundaries(back_buffer()); publish_back();
        vTaskDelay(pdMS_TO_TICKS(6000));

        ESP_LOGI(TAG, "Pattern: row walk");
        for (uint16_t y = 0; y < PANEL_HEIGHT; y++) {
            ESP_LOGI(TAG, "Logical row y=%u", y);
            make_row(back_buffer(), y); publish_back();
            vTaskDelay(pdMS_TO_TICKS(700));
        }
    }
}

static void initialize_gpio(void)
{
    static const gpio_num_t pins[] = {
        PIN_R1, PIN_G1, PIN_B1, PIN_R2, PIN_G2, PIN_B2,
        PIN_ROW_A, PIN_ROW_B, PIN_ROW_C, PIN_ROW_D, PIN_ROW_E,
        PIN_LE, PIN_GCLK, PIN_DCLK
    };

    for (size_t i = 0; i < sizeof(pins)/sizeof(pins[0]); i++)
        configure_output(pins[i]);

    for (size_t i = 0; i < sizeof(pins)/sizeof(pins[0]); i++)
        pin_write((uint8_t)pins[i], false);
}

void app_main(void)
{
    ESP_LOGI(TAG, "ESP32-S3 ICND2153/HX6158H first-light test");
    ESP_LOGI(TAG, "Panel %ux%u, scan=%u, chain=%u",
             PANEL_WIDTH, PANEL_HEIGHT, PANEL_SCAN_ROWS,
             ICND_DRIVERS_PER_CHAIN);
    ESP_LOGI(TAG, "HX clock=%s/GPIO%d, data=%s/GPIO%d, B=GPIO%d",
             HX_A_IS_CLOCK ? "A" : "C", PIN_ROW_CLOCK,
             HX_A_IS_CLOCK ? "C" : "A", PIN_ROW_DATA,
             PIN_ROW_B);

    memset(s_fb, 0, sizeof(s_fb));
    initialize_gpio();
    hx_clear();
    icnd_start();

    /* Initialize both internal display-memory banks with black. */
    refresh_once(s_fb[0]);
    refresh_once(s_fb[0]);

    BaseType_t ok = xTaskCreatePinnedToCore(
        refresh_task, "icnd_refresh", 4096, NULL, 18, NULL, REFRESH_CORE);
    ESP_ERROR_CHECK(ok == pdPASS ? ESP_OK : ESP_ERR_NO_MEM);

    ok = xTaskCreate(pattern_task, "panel_patterns", 4096, NULL, 5, NULL);
    ESP_ERROR_CHECK(ok == pdPASS ? ESP_OK : ESP_ERR_NO_MEM);
}
