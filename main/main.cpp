#include <assert.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "driver/gpio.h"
#include "esp_attr.h"
#include "esp_err.h"
#include "esp_heap_caps.h"
#include "esp_lcd_io_i80.h"
#include "esp_lcd_panel_io.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

/*
 * ESP32-S3 LCD_CAM + GDMA first test for:
 *
 *   Panel:       P3 80x40
 *   Column IC:   ICND2153
 *   Row IC:      HX6158H
 *   Scan:        1/20
 *
 * Transport:
 *   esp_lcd I80 public driver -> ESP32-S3 LCD_CAM + GDMA
 *
 * Important:
 *   - LCD WR/PCLK is NOT connected to the panel.
 *   - DCLK, GCLK, LE, RGB, A, B and C are encoded as data-bus bits.
 *   - The stream keeps the empirically proven 138 GCLK pulses per slot.
 *
 * Before building, copy every panel GPIO below from the working bit-banged
 * main.c. A/B/C are already set to the values shown in your successful log.
 */

/* -------------------------------------------------------------------------- */
/* GPIO MAP: COPY FROM THE WORKING BIT-BANGED TEST                            */
/* -------------------------------------------------------------------------- */

#define PIN_R1          GPIO_NUM_33
#define PIN_G1          GPIO_NUM_34
#define PIN_B1          GPIO_NUM_35
#define PIN_R2          GPIO_NUM_36
#define PIN_G2          GPIO_NUM_37
#define PIN_B2          GPIO_NUM_38

#define PIN_ROW_A       GPIO_NUM_1   // confirmed: HX6158H row clock
#define PIN_ROW_B       GPIO_NUM_2   // held low
#define PIN_ROW_C       GPIO_NUM_15  // confirmed: HX6158H serial data

/*
 * Verify these three against your working main.c.
 * These defaults are only a likely ESP32-S3 ETH mapping.
 */
#define PIN_LE          GPIO_NUM_16  // ICND2153 LE / HUB75 LAT
#define PIN_GCLK        GPIO_NUM_21  // ICND2153 GCLK / HUB75 OE
#define PIN_DCLK        GPIO_NUM_47  // ICND2153 DCLK / HUB75 CLK

/*
 * LCD_CAM requires a WR/PCLK output even though the panel must not receive it.
 * Leave this GPIO physically unconnected.
 */
#define PIN_LCD_WR      GPIO_NUM_8

#define PIN_LCD_DC GPIO_NUM_48

/*
 * The I80 peripheral is configured as a 16-bit bus. The panel uses only
 * lanes 0..11. These four lanes must be valid output GPIOs but must remain
 * physically unconnected. Change them if your carrier uses these pins.
 */
#define PIN_UNUSED_D12  GPIO_NUM_4
#define PIN_UNUSED_D13  GPIO_NUM_5
#define PIN_UNUSED_D14  GPIO_NUM_6
#define PIN_UNUSED_D15  GPIO_NUM_7

/* -------------------------------------------------------------------------- */
/* PANEL AND ICND2153 PARAMETERS                                              */
/* -------------------------------------------------------------------------- */

static constexpr uint16_t PANEL_WIDTH = 80;
static constexpr uint16_t PANEL_HEIGHT = 40;
static constexpr uint8_t PANEL_SCAN_ROWS = 20;

static constexpr uint8_t ICND_CHANNELS = 16;
static constexpr uint8_t ICND_DRIVERS_PER_CHAIN = 5;

static constexpr uint16_t ICND_REG1 = 0x1370;
static constexpr uint16_t ICND_REG2 = 0xFFFF;
static constexpr uint16_t ICND_REG3 = 0x40F3;
static constexpr uint16_t ICND_REG4 = 0x0000;
static constexpr uint16_t ICND_DEBUG = 0x0000;

/*
 * Proven on this exact panel:
 * 90 does not work correctly; 138 works.
 * Do not derive this value from panel width.
 */
static constexpr uint16_t ICND_GCLK_PER_SCAN_SLOT = 138;
static constexpr uint16_t ICND_GCLK_DURING_UPLOAD =
    ICND_DRIVERS_PER_CHAIN * ICND_CHANNELS;
static constexpr uint16_t ICND_GCLK_PREFIX =
    ICND_GCLK_PER_SCAN_SLOT - ICND_GCLK_DURING_UPLOAD;

static_assert(ICND_GCLK_PER_SCAN_SLOT >= ICND_GCLK_DURING_UPLOAD);

/*
 * One LCD_CAM sample is one 16-bit parallel output state.
 * Two samples encode one complete high/low panel-clock pulse.
 *
 * Start at 10 MHz LCD sample clock:
 * effective maximum DCLK/GCLK pulse rate is approximately 5 MHz.
 */
static constexpr uint32_t LCD_SAMPLE_CLOCK_HZ = 10'000'000;

/*
 * Optional byte-lane diagnostic.
 * Keep 0 first. Set to 1 only if logic-analyzer results show D0..D7 and
 * D8..D15 exchanged.
 */
static constexpr bool LCD_WORD_SWAP_BYTES = false;

static constexpr uint8_t DMA_BUFFER_COUNT = 3;
static constexpr size_t DMA_ROW_WORD_CAPACITY = 7168;
static constexpr size_t DMA_ROW_BYTES =
    DMA_ROW_WORD_CAPACITY * sizeof(uint16_t);

static constexpr size_t FRAME_BYTES =
    PANEL_WIDTH * PANEL_HEIGHT * 3U;

static constexpr uint8_t TEST_LEVEL = 64;

#if CONFIG_FREERTOS_UNICORE
static constexpr BaseType_t STREAM_CORE = 0;
#else
static constexpr BaseType_t STREAM_CORE = 1;
#endif

static const char *TAG = "ICND_LCD_CAM";

/* -------------------------------------------------------------------------- */
/* LCD DATA-LANE ASSIGNMENT                                                   */
/* -------------------------------------------------------------------------- */

enum SignalLane : uint8_t {
    LANE_R1 = 0,
    LANE_G1 = 1,
    LANE_B1 = 2,
    LANE_R2 = 3,
    LANE_G2 = 4,
    LANE_B2 = 5,

    LANE_DCLK = 6,
    LANE_LE = 7,
    LANE_GCLK = 8,

    LANE_ROW_A = 9,
    LANE_ROW_B = 10,
    LANE_ROW_C = 11,
};

static constexpr uint16_t SIG_R1 = 1U << LANE_R1;
static constexpr uint16_t SIG_G1 = 1U << LANE_G1;
static constexpr uint16_t SIG_B1 = 1U << LANE_B1;
static constexpr uint16_t SIG_R2 = 1U << LANE_R2;
static constexpr uint16_t SIG_G2 = 1U << LANE_G2;
static constexpr uint16_t SIG_B2 = 1U << LANE_B2;

static constexpr uint16_t SIG_DCLK = 1U << LANE_DCLK;
static constexpr uint16_t SIG_LE = 1U << LANE_LE;
static constexpr uint16_t SIG_GCLK = 1U << LANE_GCLK;

static constexpr uint16_t SIG_ROW_A = 1U << LANE_ROW_A;
static constexpr uint16_t SIG_ROW_B = 1U << LANE_ROW_B;
static constexpr uint16_t SIG_ROW_C = 1U << LANE_ROW_C;

/* -------------------------------------------------------------------------- */
/* GLOBAL STATE                                                               */
/* -------------------------------------------------------------------------- */

static uint8_t s_fb[2][FRAME_BYTES];
static volatile uint8_t s_front = 0;

static esp_lcd_i80_bus_handle_t s_i80_bus = nullptr;
static esp_lcd_panel_io_handle_t s_lcd_io = nullptr;
static SemaphoreHandle_t s_dma_done = nullptr;

static uint16_t *s_dma_rows[DMA_BUFFER_COUNT] = {};
static uint8_t s_encode_hx_state = 0;







static void validate_lcd_gpio_map()
{
    struct PinEntry {
        const char *name;
        gpio_num_t pin;
    };

    const PinEntry pins[] = {
        {"D0/R1",       PIN_R1},
        {"D1/G1",       PIN_G1},
        {"D2/B1",       PIN_B1},
        {"D3/R2",       PIN_R2},
        {"D4/G2",       PIN_G2},
        {"D5/B2",       PIN_B2},
        {"D6/DCLK",     PIN_DCLK},
        {"D7/LE",       PIN_LE},
        {"D8/GCLK",     PIN_GCLK},
        {"D9/ROW_A",    PIN_ROW_A},
        {"D10/ROW_B",   PIN_ROW_B},
        {"D11/ROW_C",   PIN_ROW_C},
        {"D12/unused",  PIN_UNUSED_D12},
        {"D13/unused",  PIN_UNUSED_D13},
        {"D14/unused",  PIN_UNUSED_D14},
        {"D15/unused",  PIN_UNUSED_D15},
        {"LCD_WR",      PIN_LCD_WR},
        {"LCD_DC",      PIN_LCD_DC},
    };

    constexpr size_t pin_count =
        sizeof(pins) / sizeof(pins[0]);

    for (size_t i = 0; i < pin_count; i++) {
        if (!GPIO_IS_VALID_OUTPUT_GPIO(pins[i].pin)) {
            ESP_LOGE(
                TAG,
                "Invalid output GPIO: %s = GPIO%d",
                pins[i].name,
                static_cast<int>(pins[i].pin));

            ESP_ERROR_CHECK(ESP_ERR_INVALID_ARG);
        }

        for (size_t j = i + 1; j < pin_count; j++) {
            if (pins[i].pin == pins[j].pin) {
                ESP_LOGE(
                    TAG,
                    "Duplicate GPIO%d: %s and %s",
                    static_cast<int>(pins[i].pin),
                    pins[i].name,
                    pins[j].name);

                ESP_ERROR_CHECK(ESP_ERR_INVALID_ARG);
            }
        }

        ESP_LOGI(
            TAG,
            "%-12s -> GPIO%d",
            pins[i].name,
            static_cast<int>(pins[i].pin));
    }
}












/* -------------------------------------------------------------------------- */
/* BIT-BANGED STARTUP: KNOWN-GOOD ICND2153 COMMAND PROTOCOL                   */
/* -------------------------------------------------------------------------- */

static inline void pin_write(gpio_num_t pin, bool high)
{
    gpio_set_level(pin, high ? 1 : 0);
}

static void configure_output(gpio_num_t pin)
{
    gpio_config_t cfg = {};
    cfg.pin_bit_mask = 1ULL << static_cast<unsigned>(pin);
    cfg.mode = GPIO_MODE_OUTPUT;
    cfg.pull_up_en = GPIO_PULLUP_DISABLE;
    cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
    cfg.intr_type = GPIO_INTR_DISABLE;
    ESP_ERROR_CHECK(gpio_config(&cfg));
}

static inline void pulse_dclk_bitbang()
{
    pin_write(PIN_DCLK, true);
    pin_write(PIN_DCLK, false);
}

static void icnd_latch_command_bitbang(uint8_t clocks)
{
    pin_write(PIN_LE, true);

    while (clocks-- > 0) {
        pulse_dclk_bitbang();
    }

    pin_write(PIN_LE, false);
}

static void icnd_send_six_words_bitbang(
    uint16_t r1,
    uint16_t g1,
    uint16_t b1,
    uint16_t r2,
    uint16_t g2,
    uint16_t b2,
    uint8_t latch_bits)
{
    uint16_t mask = 0x8000;
    uint8_t bit_index = 0;

    pin_write(PIN_LE, false);

    while (mask != 0) {
        pin_write(PIN_DCLK, false);

        if ((latch_bits != 0) &&
            (bit_index == static_cast<uint8_t>(16U - latch_bits))) {
            pin_write(PIN_LE, true);
        }

        pin_write(PIN_R1, (r1 & mask) != 0);
        pin_write(PIN_G1, (g1 & mask) != 0);
        pin_write(PIN_B1, (b1 & mask) != 0);

        pin_write(PIN_R2, (r2 & mask) != 0);
        pin_write(PIN_G2, (g2 & mask) != 0);
        pin_write(PIN_B2, (b2 & mask) != 0);

        pin_write(PIN_DCLK, true);

        mask >>= 1U;
        bit_index++;
    }

    pin_write(PIN_DCLK, false);
    pin_write(PIN_LE, false);
}

static void icnd_send_value_to_all_drivers_bitbang(
    uint16_t value,
    uint8_t latch_bits)
{
    for (uint8_t driver = 0;
         driver < ICND_DRIVERS_PER_CHAIN;
         driver++) {

        const bool final_driver =
            driver == (ICND_DRIVERS_PER_CHAIN - 1);

        icnd_send_six_words_bitbang(
            value, value, value,
            value, value, value,
            final_driver ? latch_bits : 0);
    }
}

static void hx_shift_zero_bitbang()
{
    pin_write(PIN_ROW_C, false);
    pin_write(PIN_ROW_A, false);
    pin_write(PIN_ROW_A, true);
    pin_write(PIN_ROW_A, false);
}

static void hx_clear_chain_bitbang()
{
    pin_write(PIN_ROW_B, false);

    for (uint8_t i = 0; i < 48; i++) {
        hx_shift_zero_bitbang();
    }

    s_encode_hx_state = 0;
}

static void initialize_panel_gpio_bitbang()
{
    const gpio_num_t pins[] = {
        PIN_R1, PIN_G1, PIN_B1,
        PIN_R2, PIN_G2, PIN_B2,
        PIN_ROW_A, PIN_ROW_B, PIN_ROW_C,
        PIN_LE, PIN_GCLK, PIN_DCLK,
    };

    for (gpio_num_t pin : pins) {
        configure_output(pin);
        pin_write(pin, false);
    }
}

static void icnd_start_memory_protocol_bitbang()
{
    ESP_LOGI(
        TAG,
        "ICND init REG1=0x%04X, GCLK/slot=%u, prefix=%u",
        ICND_REG1,
        ICND_GCLK_PER_SCAN_SLOT,
        ICND_GCLK_PREFIX);

    icnd_latch_command_bitbang(14); // PRE-ACTIVE
    icnd_latch_command_bitbang(12); // ENABLE OUTPUTS
    icnd_latch_command_bitbang(3);  // VSYNC

    icnd_latch_command_bitbang(14);
    icnd_send_value_to_all_drivers_bitbang(ICND_REG1, 4);

    icnd_latch_command_bitbang(14);
    icnd_send_value_to_all_drivers_bitbang(ICND_REG2, 6);

    icnd_latch_command_bitbang(14);
    icnd_send_value_to_all_drivers_bitbang(ICND_REG3, 8);

    icnd_latch_command_bitbang(14);
    icnd_send_value_to_all_drivers_bitbang(ICND_REG4, 10);

    icnd_latch_command_bitbang(14);
    icnd_send_value_to_all_drivers_bitbang(ICND_DEBUG, 2);

    pin_write(PIN_LE, false);
    pin_write(PIN_DCLK, false);
    pin_write(PIN_GCLK, false);
}

/* -------------------------------------------------------------------------- */
/* WAVEFORM ENCODER                                                           */
/* -------------------------------------------------------------------------- */

struct WordWriter {
    uint16_t *data;
    size_t capacity;
    size_t size;

    void push(uint16_t word)
    {
        assert(size < capacity);

        if constexpr (LCD_WORD_SWAP_BYTES) {
            word = __builtin_bswap16(word);
        }

        data[size++] = word;
    }
};

static inline uint16_t expand_u8(uint8_t value)
{
    return static_cast<uint16_t>(value) * 257U;
}

static void emit_vsync(WordWriter &out)
{
    out.push(SIG_LE);

    for (uint8_t i = 0; i < 3; i++) {
        out.push(SIG_LE | SIG_DCLK);
        out.push(SIG_LE);
    }

    out.push(0);
}

static void emit_gclk_pulses(WordWriter &out, uint16_t clocks)
{
    while (clocks-- > 0) {
        out.push(SIG_GCLK);
        out.push(0);
    }
}

static void emit_hx_advance(WordWriter &out)
{
    const bool inject_token = s_encode_hx_state == 0;

    if (inject_token) {
        /*
         * Exact proven sequence:
         * C=1/A=0, C=1/A=1, C=1/A=0, C=0/A=0
         */
        out.push(SIG_ROW_C);
        out.push(SIG_ROW_C | SIG_ROW_A);
        out.push(SIG_ROW_C);
        out.push(0);
    } else {
        /*
         * Subsequent advance:
         * C=0/A=1, C=0/A=0
         */
        out.push(SIG_ROW_A);
        out.push(0);
    }

    s_encode_hx_state++;
    if (s_encode_hx_state >= PANEL_SCAN_ROWS) {
        s_encode_hx_state = 0;
    }
}

static void emit_dclk_bit(
    WordWriter &out,
    uint16_t signal_state)
{
    signal_state &= static_cast<uint16_t>(~SIG_DCLK);

    out.push(signal_state);
    out.push(signal_state | SIG_DCLK);
}

static void emit_pixel_packet(
    WordWriter &out,
    uint8_t section,
    uint8_t channel,
    uint8_t memory_row,
    const uint8_t *frame)
{
    const uint16_t x =
        static_cast<uint16_t>(section) * ICND_CHANNELS + channel;

    const uint16_t lower_row =
        static_cast<uint16_t>(memory_row) + PANEL_SCAN_ROWS;

    const size_t upper_index =
        ((static_cast<size_t>(memory_row) * PANEL_WIDTH) + x) * 3U;

    const size_t lower_index =
        ((static_cast<size_t>(lower_row) * PANEL_WIDTH) + x) * 3U;

    const uint16_t r1 = expand_u8(frame[upper_index + 0]);
    const uint16_t g1 = expand_u8(frame[upper_index + 1]);
    const uint16_t b1 = expand_u8(frame[upper_index + 2]);

    const uint16_t r2 = expand_u8(frame[lower_index + 0]);
    const uint16_t g2 = expand_u8(frame[lower_index + 1]);
    const uint16_t b2 = expand_u8(frame[lower_index + 2]);

    const bool final_driver =
        section == (ICND_DRIVERS_PER_CHAIN - 1);

    for (uint8_t bit = 0; bit < 16; bit++) {
        const uint16_t mask =
            static_cast<uint16_t>(0x8000U >> bit);

        uint16_t state = 0;

        if ((r1 & mask) != 0) state |= SIG_R1;
        if ((g1 & mask) != 0) state |= SIG_G1;
        if ((b1 & mask) != 0) state |= SIG_B1;

        if ((r2 & mask) != 0) state |= SIG_R2;
        if ((g2 & mask) != 0) state |= SIG_G2;
        if ((b2 & mask) != 0) state |= SIG_B2;

        /*
         * Same behavior as the working bit-banged test:
         * LE is high only for the final data bit of the final driver.
         */
        if (final_driver && (bit == 15)) {
            state |= SIG_LE;
        }

        emit_dclk_bit(out, state);
    }

    /* Return DCLK and LE low before the next GCLK burst. */
    out.push(0);
}

static size_t encode_memory_row(
    uint16_t *destination,
    size_t capacity,
    uint8_t memory_row,
    const uint8_t *frame)
{
    WordWriter out = {
        .data = destination,
        .capacity = capacity,
        .size = 0,
    };

    if (memory_row == 0) {
        /*
         * 20 rows x 16 slots = 320 row advances per upload.
         * 320 is exactly divisible by 20, so every upload must start
         * with HX state zero.
         */
        assert(s_encode_hx_state == 0);
        emit_vsync(out);
    }

    for (uint8_t channel = 0;
         channel < ICND_CHANNELS;
         channel++) {

        emit_hx_advance(out);

        emit_gclk_pulses(out, ICND_GCLK_PREFIX);

        for (uint8_t section = 0;
             section < ICND_DRIVERS_PER_CHAIN;
             section++) {

            emit_gclk_pulses(out, 16);

            emit_pixel_packet(
                out,
                section,
                channel,
                memory_row,
                frame);
        }
    }

    /*
     * Safe idle state at the transaction boundary.
     * The I80 driver can pause between queued buffers without leaving
     * DCLK, GCLK, LE, A or C high.
     */
    out.push(0);

    return out.size;
}

/* -------------------------------------------------------------------------- */
/* ESP LCD I80 TRANSPORT: LCD_CAM + GDMA                                      */
/* -------------------------------------------------------------------------- */

static bool IRAM_ATTR on_dma_transfer_done(
    esp_lcd_panel_io_handle_t,
    esp_lcd_panel_io_event_data_t *,
    void *user_ctx)
{
    auto semaphore =
        static_cast<SemaphoreHandle_t>(user_ctx);

    BaseType_t high_priority_task_woken = pdFALSE;

    xSemaphoreGiveFromISR(
        semaphore,
        &high_priority_task_woken);

    return high_priority_task_woken == pdTRUE;
}

static void initialize_lcd_cam_i80()
{
    s_dma_done =
        xSemaphoreCreateCounting(DMA_BUFFER_COUNT, 0);

    assert(s_dma_done != nullptr);

    esp_lcd_i80_bus_config_t bus_cfg = {};

    bus_cfg.dc_gpio_num = PIN_LCD_DC;
    bus_cfg.wr_gpio_num = PIN_LCD_WR;
    bus_cfg.clk_src = LCD_CLK_SRC_DEFAULT;

    /*
     * D0..D15 correspond directly to bit 0..15 in each waveform word.
     */
    bus_cfg.data_gpio_nums[0] = PIN_R1;
    bus_cfg.data_gpio_nums[1] = PIN_G1;
    bus_cfg.data_gpio_nums[2] = PIN_B1;
    bus_cfg.data_gpio_nums[3] = PIN_R2;
    bus_cfg.data_gpio_nums[4] = PIN_G2;
    bus_cfg.data_gpio_nums[5] = PIN_B2;

    bus_cfg.data_gpio_nums[6] = PIN_DCLK;
    bus_cfg.data_gpio_nums[7] = PIN_LE;
    bus_cfg.data_gpio_nums[8] = PIN_GCLK;

    bus_cfg.data_gpio_nums[9] = PIN_ROW_A;
    bus_cfg.data_gpio_nums[10] = PIN_ROW_B;
    bus_cfg.data_gpio_nums[11] = PIN_ROW_C;

    bus_cfg.data_gpio_nums[12] = PIN_UNUSED_D12;
    bus_cfg.data_gpio_nums[13] = PIN_UNUSED_D13;
    bus_cfg.data_gpio_nums[14] = PIN_UNUSED_D14;
    bus_cfg.data_gpio_nums[15] = PIN_UNUSED_D15;

    bus_cfg.bus_width = 16;
    bus_cfg.max_transfer_bytes = DMA_ROW_BYTES;
    bus_cfg.dma_burst_size = 64;

    ESP_ERROR_CHECK(
        esp_lcd_new_i80_bus(
            &bus_cfg,
            &s_i80_bus));

    esp_lcd_panel_io_i80_config_t io_cfg = {};

    /*
     * CS=-1 gives this IO exclusive ownership of the I80 bus.
     * lcd_cmd=-1 is used for every transaction, so no command phase
     * is emitted.
     */
    io_cfg.cs_gpio_num = GPIO_NUM_NC;
    io_cfg.pclk_hz = LCD_SAMPLE_CLOCK_HZ;
    io_cfg.trans_queue_depth = DMA_BUFFER_COUNT;
    io_cfg.on_color_trans_done = on_dma_transfer_done;
    io_cfg.user_ctx = s_dma_done;
    io_cfg.lcd_cmd_bits = 8;
    io_cfg.lcd_param_bits = 8;

    io_cfg.dc_levels.dc_idle_level = 0;
    io_cfg.dc_levels.dc_cmd_level = 0;
    io_cfg.dc_levels.dc_dummy_level = 0;
    io_cfg.dc_levels.dc_data_level = 0;

    io_cfg.flags.cs_active_high = 0;
    io_cfg.flags.reverse_color_bits = 0;
    io_cfg.flags.swap_color_bytes = 0;
    io_cfg.flags.pclk_active_neg = 0;
    io_cfg.flags.pclk_idle_low = 1;

    ESP_ERROR_CHECK(
        esp_lcd_new_panel_io_i80(
            s_i80_bus,
            &io_cfg,
            &s_lcd_io));

    for (uint8_t i = 0; i < DMA_BUFFER_COUNT; i++) {
        s_dma_rows[i] =
            static_cast<uint16_t *>(
                esp_lcd_i80_alloc_draw_buffer(
                    s_lcd_io,
                    DMA_ROW_BYTES,
                    MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL));

        assert(s_dma_rows[i] != nullptr);
        memset(s_dma_rows[i], 0, DMA_ROW_BYTES);
    }

    ESP_LOGI(
        TAG,
        "LCD_CAM I80 ready: sample clock=%lu Hz, buffers=%u x %u bytes",
        static_cast<unsigned long>(LCD_SAMPLE_CLOCK_HZ),
        DMA_BUFFER_COUNT,
        static_cast<unsigned>(DMA_ROW_BYTES));
}

static void submit_encoded_row(
    uint8_t buffer_index,
    uint8_t memory_row,
    const uint8_t *frame)
{
    const size_t word_count =
        encode_memory_row(
            s_dma_rows[buffer_index],
            DMA_ROW_WORD_CAPACITY,
            memory_row,
            frame);

    const size_t byte_count =
        word_count * sizeof(uint16_t);

    ESP_ERROR_CHECK(
        esp_lcd_panel_io_tx_color(
            s_lcd_io,
            -1,
            s_dma_rows[buffer_index],
            byte_count));
}

/* -------------------------------------------------------------------------- */
/* TEST FRAMEBUFFER                                                           */
/* -------------------------------------------------------------------------- */

static void set_pixel(
    uint8_t *frame,
    uint16_t x,
    uint16_t y,
    uint8_t r,
    uint8_t g,
    uint8_t b)
{
    if ((x >= PANEL_WIDTH) || (y >= PANEL_HEIGHT)) {
        return;
    }

    const size_t index =
        ((static_cast<size_t>(y) * PANEL_WIDTH) + x) * 3U;

    frame[index + 0] = r;
    frame[index + 1] = g;
    frame[index + 2] = b;
}

static void fill_frame(
    uint8_t *frame,
    uint8_t r,
    uint8_t g,
    uint8_t b)
{
    for (uint16_t y = 0; y < PANEL_HEIGHT; y++) {
        for (uint16_t x = 0; x < PANEL_WIDTH; x++) {
            set_pixel(frame, x, y, r, g, b);
        }
    }
}

static void build_vertical_bands(uint8_t *frame)
{
    memset(frame, 0, FRAME_BYTES);

    for (uint16_t y = 0; y < PANEL_HEIGHT; y++) {
        for (uint16_t x = 0; x < PANEL_WIDTH; x++) {
            switch (x / 16U) {
                case 0:
                    set_pixel(frame, x, y, TEST_LEVEL, 0, 0);
                    break;

                case 1:
                    set_pixel(frame, x, y, 0, TEST_LEVEL, 0);
                    break;

                case 2:
                    set_pixel(frame, x, y, 0, 0, TEST_LEVEL);
                    break;

                case 3:
                    set_pixel(
                        frame, x, y,
                        TEST_LEVEL, TEST_LEVEL, 0);
                    break;

                default:
                    set_pixel(
                        frame, x, y,
                        TEST_LEVEL, 0, TEST_LEVEL);
                    break;
            }
        }
    }
}

static void build_boundaries(uint8_t *frame)
{
    memset(frame, 0, FRAME_BYTES);

    const uint16_t x_lines[] = {
        0, 15, 16, 31, 32,
        47, 48, 63, 64, 79
    };

    const uint16_t y_lines[] = {
        0, 19, 20, 39
    };

    for (uint16_t x : x_lines) {
        for (uint16_t y = 0; y < PANEL_HEIGHT; y++) {
            set_pixel(
                frame, x, y,
                TEST_LEVEL, TEST_LEVEL, TEST_LEVEL);
        }
    }

    for (uint16_t y : y_lines) {
        for (uint16_t x = 0; x < PANEL_WIDTH; x++) {
            set_pixel(
                frame, x, y,
                TEST_LEVEL, TEST_LEVEL, TEST_LEVEL);
        }
    }
}

static void publish_solid(
    uint8_t r,
    uint8_t g,
    uint8_t b)
{
    const uint8_t current =
        __atomic_load_n(
            &s_front,
            __ATOMIC_ACQUIRE);

    const uint8_t back = current ^ 1U;

    fill_frame(s_fb[back], r, g, b);

    __atomic_store_n(
        &s_front,
        back,
        __ATOMIC_RELEASE);
}

static void publish_builder(
    void (*builder)(uint8_t *))
{
    const uint8_t current =
        __atomic_load_n(
            &s_front,
            __ATOMIC_ACQUIRE);

    const uint8_t back = current ^ 1U;

    builder(s_fb[back]);

    __atomic_store_n(
        &s_front,
        back,
        __ATOMIC_RELEASE);
}

/* -------------------------------------------------------------------------- */
/* CONTINUOUS DMA PIPELINE                                                    */
/* -------------------------------------------------------------------------- */

static void lcd_stream_task(void *)
{
    uint8_t next_memory_row = 0;
    uint8_t reuse_buffer = 0;

    const uint8_t *frame_snapshot =
        s_fb[__atomic_load_n(
            &s_front,
            __ATOMIC_ACQUIRE)];

    /*
     * Preload three DMA transactions.
     */
    for (uint8_t buffer = 0;
         buffer < DMA_BUFFER_COUNT;
         buffer++) {

        if (next_memory_row == 0) {
            frame_snapshot =
                s_fb[__atomic_load_n(
                    &s_front,
                    __ATOMIC_ACQUIRE)];
        }

        submit_encoded_row(
            buffer,
            next_memory_row,
            frame_snapshot);

        next_memory_row++;

        if (next_memory_row >= PANEL_SCAN_ROWS) {
            next_memory_row = 0;
        }
    }

    int64_t report_start_us = esp_timer_get_time();
    uint32_t uploaded_frames = 0;

    while (true) {
        /*
         * Wait until the oldest queued DMA buffer has completed.
         */
        xSemaphoreTake(s_dma_done, portMAX_DELAY);

        if (next_memory_row == 0) {
            frame_snapshot =
                s_fb[__atomic_load_n(
                    &s_front,
                    __ATOMIC_ACQUIRE)];
        }

        /*
         * Refill and requeue the completed buffer.
         */
        submit_encoded_row(
            reuse_buffer,
            next_memory_row,
            frame_snapshot);

        reuse_buffer++;

        if (reuse_buffer >= DMA_BUFFER_COUNT) {
            reuse_buffer = 0;
        }

        next_memory_row++;

        bool completed_upload = false;

        if (next_memory_row >= PANEL_SCAN_ROWS) {
            next_memory_row = 0;
            uploaded_frames++;
            completed_upload = true;
        }

        /*
         * The DMA queue is full again at this point.
         *
         * Block this high-priority task for one RTOS tick after every
         * complete 20-row upload. This lets IDLE1 run while LCD_CAM/GDMA
         * continues transmitting the queued row buffers.
         */
        if (completed_upload) {
            vTaskDelay(1);
        }

        const int64_t now_us = esp_timer_get_time();

        if ((now_us - report_start_us) >= 5000000LL) {
            const float seconds =
                static_cast<float>(
                    now_us - report_start_us) /
                1000000.0f;

            ESP_LOGI(
                TAG,
                "DMA uploads %.2f/s, encoder HX state=%u",
                static_cast<float>(uploaded_frames) / seconds,
                s_encode_hx_state);

            uploaded_frames = 0;
            report_start_us = now_us;
        }
    }
}

static void pattern_task(void *)
{
    /*
     * Allow at least two black uploads so both ICND display-memory banks
     * begin from a deterministic state.
     */
    vTaskDelay(pdMS_TO_TICKS(1000));

    while (true) {
        ESP_LOGI(TAG, "Pattern: RED");
        publish_solid(TEST_LEVEL, 0, 0);
        vTaskDelay(pdMS_TO_TICKS(4000));

        ESP_LOGI(TAG, "Pattern: GREEN");
        publish_solid(0, TEST_LEVEL, 0);
        vTaskDelay(pdMS_TO_TICKS(4000));

        ESP_LOGI(TAG, "Pattern: BLUE");
        publish_solid(0, 0, TEST_LEVEL);
        vTaskDelay(pdMS_TO_TICKS(4000));

        ESP_LOGI(TAG, "Pattern: WHITE");
        publish_solid(
            TEST_LEVEL,
            TEST_LEVEL,
            TEST_LEVEL);
        vTaskDelay(pdMS_TO_TICKS(4000));

        ESP_LOGI(TAG, "Pattern: five 16-column sections");
        publish_builder(build_vertical_bands);
        vTaskDelay(pdMS_TO_TICKS(6000));

        ESP_LOGI(TAG, "Pattern: boundaries");
        publish_builder(build_boundaries);
        vTaskDelay(pdMS_TO_TICKS(6000));
    }
}

/* -------------------------------------------------------------------------- */
/* APPLICATION ENTRY                                                         */
/* -------------------------------------------------------------------------- */

extern "C" void app_main(void)
{
    ESP_LOGI(
        TAG,
        "ESP32-S3 LCD_CAM/GDMA test: ICND2153 + HX6158H");

    ESP_LOGI(
        TAG,
        "Panel %ux%u, scan=%u, chain=%u",
        PANEL_WIDTH,
        PANEL_HEIGHT,
        PANEL_SCAN_ROWS,
        ICND_DRIVERS_PER_CHAIN);

    ESP_LOGI(
        TAG,
        "HX: A=clock GPIO%d, C=data GPIO%d, B=GPIO%d LOW",
        static_cast<int>(PIN_ROW_A),
        static_cast<int>(PIN_ROW_C),
        static_cast<int>(PIN_ROW_B));

    memset(s_fb, 0, sizeof(s_fb));

    /*
     * First configure the same GPIO protocol that already worked and use it
     * only for IC initialization and row-chain clearing.
     */
    initialize_panel_gpio_bitbang();
    hx_clear_chain_bitbang();
    icnd_start_memory_protocol_bitbang();

    /*
     * LCD_CAM takes ownership of the panel GPIO matrix and uses GDMA for
     * all repeated waveform output.
     */
	 
	 
	 
	 validate_lcd_gpio_map();
	 initialize_lcd_cam_i80();
	 
	 
	 

    BaseType_t task_result =
        xTaskCreatePinnedToCore(
            lcd_stream_task,
            "icnd_lcd_stream",
            6144,
            nullptr,
            12,
            nullptr,
            STREAM_CORE);

    ESP_ERROR_CHECK(
        task_result == pdPASS ?
            ESP_OK :
            ESP_ERR_NO_MEM);

    task_result =
        xTaskCreate(
            pattern_task,
            "panel_patterns",
            4096,
            nullptr,
            5,
            nullptr);

    ESP_ERROR_CHECK(
        task_result == pdPASS ?
            ESP_OK :
            ESP_ERR_NO_MEM);
}
