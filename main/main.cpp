/* ==========================================================================
 * ESP32-S3 ICND2153 + HX6158H  —  Step 2: raw LCD_CAM + GDMA, flat PSRAM ring
 * ==========================================================================
 *
 * Panel:      P3 80x40, ICND2153 S-PWM column drivers, HX6158H serial rows,
 *             1/20 scan, rows y and y+20 paired, walking token injected
 *             every 20 row clocks.
 *
 * What changed vs. the esp_lcd I80 step:
 *   - The per-row streaming encoder and its task are GONE. Two complete
 *     upload waveforms (283,640 B each) live in octal PSRAM and loop
 *     forever through a circular GDMA descriptor chain. Steady-state CPU
 *     cost of refresh is zero.
 *   - Frame presentation = patch pixels into the back ring, cache-writeback
 *     the dirty span, retarget one descriptor 'next' pointer. The switch
 *     lands exactly at the upload boundary (the ring starts with VSYNC),
 *     confirmed by the GDMA EOF interrupt.
 *   - esp_lcd is not used. Only LCD data lanes D0..D11 are routed, so
 *     GPIO4-7 (TF card), GPIO8 and GPIO48 are returned to the board.
 *
 * Waveform is byte-for-byte the proven one:
 *   VSYNC(8w) + 20 rows x [16 slots x (HX adv + 58 GCLK + 5 x (16 GCLK +
 *   16-bit packet w/ LE on last word)) ] + 1 idle word per row
 *   = 141,820 words per upload (static_assert'd below).
 *
 * Expected boot behavior (10 MHz sample clock):
 *   "uploads/s ~= 70.5" in the telemetry log. If you see ~105.8 instead,
 *   the LCD clock source mux resolved to PLL240M, not PLL160M: change
 *   LCD_CLK_SEL_VALUE from 3 to 2 or raise the divider to 24. The EOF
 *   rate is a built-in frequency counter, and PCLK_DEBUG_GPIO (GPIO8,
 *   physically unconnected) can be scoped directly.
 *
 * Build (CMakeLists): idf_component_register(SRCS "main.cpp"
 *     PRIV_REQUIRES esp_hw_support esp_driver_dma esp_mm esp_driver_gpio
 *                   esp_timer)
 *
 * Wiring: identical to the working Step 1 log. No jumper changes.
 * ========================================================================== */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "sdkconfig.h"

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "esp_cache.h"
#include "esp_err.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_rom_gpio.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"

#include "esp_private/gdma.h"        /* AHB-DMA channel driver              */
#include "esp_private/periph_ctrl.h" /* periph_module_enable (verified v6)  */
#include "soc/gpio_sig_map.h"        /* LCD_DATA_OUT0_IDX..15, LCD_PCLK_IDX */
#include "soc/lcd_cam_struct.h"      /* LCD_CAM register file (verified v6) */
#include "soc/periph_defs.h"         /* PERIPH_LCD_CAM_MODULE               */

#if !defined(CONFIG_SPIRAM) || !CONFIG_SPIRAM
#error "This build requires octal PSRAM (the waveform rings live there)."
#endif
#if !defined(CONFIG_SPIRAM_MODE_OCT) || !CONFIG_SPIRAM_MODE_OCT
#error "ESP32-S3R8 requires octal PSRAM mode."
#endif

static const char *TAG = "ICND_RING";

/* -------------------------------------------------------------------------- */
/* GPIO MAP — identical to the proven Step 1 map (PSRAM-safe lanes)           */
/* -------------------------------------------------------------------------- */

#define PIN_R1      GPIO_NUM_17
#define PIN_G1      GPIO_NUM_18
#define PIN_B1      GPIO_NUM_39
#define PIN_R2      GPIO_NUM_40
#define PIN_G2      GPIO_NUM_41
#define PIN_B2      GPIO_NUM_38

#define PIN_ROW_A   GPIO_NUM_1     /* HX6158H row clock                     */
#define PIN_ROW_B   GPIO_NUM_2     /* held low                              */
#define PIN_ROW_C   GPIO_NUM_15    /* HX6158H serial row data               */

#define PIN_LE      GPIO_NUM_16
#define PIN_GCLK    GPIO_NUM_42
#define PIN_DCLK    GPIO_NUM_47

/*
 * Optional: mirror the internal LCD sample clock on a free, unconnected
 * GPIO for scope verification during first bring-up. Expect 10.0 MHz.
 * Plain integer on purpose (enumerators are invisible to #if). Set to -1
 * after verifying. GPIO8 is free on this board once esp_lcd is gone (it
 * was only the dummy WR pin).
 */
#define PCLK_DEBUG_GPIO 8

/* -------------------------------------------------------------------------- */
/* PANEL / ICND2153 PARAMETERS (all proven on this panel)                     */
/* -------------------------------------------------------------------------- */

static constexpr uint16_t PANEL_WIDTH = 80;
static constexpr uint16_t PANEL_HEIGHT = 40;
static constexpr uint8_t PANEL_SCAN_ROWS = 20;

static constexpr uint8_t ICND_CHANNELS = 16;
static constexpr uint8_t ICND_DRIVERS_PER_CHAIN = 5;

static constexpr uint16_t ICND_REG1 = 0x1370; /* 20-scan variant, proven */
static constexpr uint16_t ICND_REG2 = 0xFFFF;
static constexpr uint16_t ICND_REG3 = 0x40F3;
static constexpr uint16_t ICND_REG4 = 0x0000;
static constexpr uint16_t ICND_DEBUG = 0x0000;

/* Silicon requirement, NOT width-derived: 90 fails, 138 works. */
static constexpr uint16_t ICND_GCLK_PER_SCAN_SLOT = 138;
static constexpr uint16_t ICND_GCLK_DURING_UPLOAD =
    ICND_DRIVERS_PER_CHAIN * ICND_CHANNELS; /* 80 */
static constexpr uint16_t ICND_GCLK_PREFIX =
    ICND_GCLK_PER_SCAN_SLOT - ICND_GCLK_DURING_UPLOAD; /* 58 */
static_assert(ICND_GCLK_PER_SCAN_SLOT >= ICND_GCLK_DURING_UPLOAD);

/*
 * Gamma exponent for the u8 -> u16 grayscale expansion.
 * 1.0f reproduces the proven linear (*257) behavior exactly — keep it for
 * the transport-verification flash so brightness is comparable, then
 * switch to 2.2f once the ring is confirmed pixel-identical.
 */
static constexpr float GAMMA_EXPONENT = 1.0f;

/* -------------------------------------------------------------------------- */
/* LCD SAMPLE CLOCK                                                           */
/* -------------------------------------------------------------------------- */

/*
 * Target: 10 MHz sample clock = 5 MHz max DCLK/GCLK pulse rate, the rate
 * proven with esp_lcd. Source mux value 3 selects PLL_F160M on the S3
 * (2 = PLL240M, 1 = XTAL 40M). 160 / 16 = 10 MHz.
 * The telemetry log verifies this: 70.5 uploads/s <=> exactly 10 MHz.
 */
static constexpr uint32_t LCD_CLK_SEL_VALUE = 3;  /* PLL_F160M            */
static constexpr uint32_t LCD_CLK_DIV_NUM = 16;   /* integer divider       */

/* -------------------------------------------------------------------------- */
/* LCD DATA-LANE ASSIGNMENT (unchanged word format)                           */
/* -------------------------------------------------------------------------- */

enum SignalLane : uint8_t {
    LANE_R1 = 0, LANE_G1 = 1, LANE_B1 = 2,
    LANE_R2 = 3, LANE_G2 = 4, LANE_B2 = 5,
    LANE_DCLK = 6, LANE_LE = 7, LANE_GCLK = 8,
    LANE_ROW_A = 9, LANE_ROW_B = 10, LANE_ROW_C = 11,
    LANE_COUNT = 12,
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

static constexpr uint16_t RGB1_MASK = SIG_R1 | SIG_G1 | SIG_B1;
static constexpr uint16_t RGB2_MASK = SIG_R2 | SIG_G2 | SIG_B2;

static const gpio_num_t k_lane_gpio[LANE_COUNT] = {
    PIN_R1, PIN_G1, PIN_B1, PIN_R2, PIN_G2, PIN_B2,
    PIN_DCLK, PIN_LE, PIN_GCLK, PIN_ROW_A, PIN_ROW_B, PIN_ROW_C,
};

/* -------------------------------------------------------------------------- */
/* RING GEOMETRY — computed by simulating the proven encoder                  */
/* -------------------------------------------------------------------------- */

static constexpr size_t WORDS_PER_SECTION = 16 * 2 /*GCLK*/ + 16 * 2 /*data*/ + 1;
static constexpr size_t WORDS_SLOT_BASE =
    ICND_GCLK_PREFIX * 2 + ICND_DRIVERS_PER_CHAIN * WORDS_PER_SECTION; /* 441 */

static constexpr size_t compute_ring_words()
{
    size_t words = 8; /* vsync */
    unsigned hx = 0;
    for (unsigned row = 0; row < PANEL_SCAN_ROWS; row++) {
        for (unsigned ch = 0; ch < ICND_CHANNELS; ch++) {
            words += (hx == 0) ? 4 : 2; /* HX advance */
            hx = (hx + 1) % PANEL_SCAN_ROWS;
            words += WORDS_SLOT_BASE;
        }
        words += 1; /* per-row idle word, kept for byte-exactness */
    }
    return words;
}

static constexpr size_t RING_WORDS = compute_ring_words();
static_assert(RING_WORDS == 141820, "ring layout drifted from proven waveform");

static constexpr size_t RING_BYTES = RING_WORDS * sizeof(uint16_t); /* 283,640 */
static constexpr size_t CACHE_LINE = 64;
static constexpr size_t RING_BYTES_PADDED =
    (RING_BYTES + CACHE_LINE - 1) & ~(CACHE_LINE - 1); /* 283,648 */

/* Descriptor slicing: 4032 = 63 * 64, so every chunk start is 64-aligned
 * (required for GDMA external-memory access) and < 4095 (12-bit size field). */
static constexpr size_t DESC_CHUNK_BYTES = 4032;
static constexpr size_t DESC_PER_RING =
    (RING_BYTES + DESC_CHUNK_BYTES - 1) / DESC_CHUNK_BYTES; /* 71 */
static constexpr size_t LAST_CHUNK_BYTES =
    RING_BYTES - (DESC_PER_RING - 1) * DESC_CHUNK_BYTES; /* 1,400 */

/* Unique RGB-carrying packets, for the pixel patch table. */
static constexpr size_t TOTAL_PACKETS =
    (size_t)PANEL_SCAN_ROWS * ICND_CHANNELS * ICND_DRIVERS_PER_CHAIN; /* 1600 */

/* -------------------------------------------------------------------------- */
/* GDMA descriptor — hardware layout (matches hal/dma_types.h, defined        */
/* locally so this file has no dependency on that header's disk location)     */
/* -------------------------------------------------------------------------- */

typedef struct lcd_dma_desc_s {
    struct {
        uint32_t size : 12;      /* buffer capacity                  */
        uint32_t length : 12;    /* valid bytes to transmit          */
        uint32_t reserved24 : 4;
        uint32_t err_eof : 1;
        uint32_t reserved29 : 1;
        uint32_t suc_eof : 1;    /* fire OUT_EOF when this completes */
        uint32_t owner : 1;      /* 1 = DMA                          */
    } dw0;
    uint8_t *buffer;
    struct lcd_dma_desc_s *next;
} lcd_dma_desc_t;

static_assert(sizeof(lcd_dma_desc_t) == 12, "unexpected GDMA descriptor size");

/* Descriptors MUST live in internal RAM (uncached, DMA-visible). Static
 * arrays in DRAM satisfy both requirements with zero heap involvement.   */
static lcd_dma_desc_t s_desc[2][DESC_PER_RING] __attribute__((aligned(4)));

/* -------------------------------------------------------------------------- */
/* GLOBAL STATE                                                               */
/* -------------------------------------------------------------------------- */

static uint16_t *s_ring[2] = {nullptr, nullptr}; /* PSRAM waveforms         */
static uint32_t s_packet_off[TOTAL_PACKETS];     /* word offset per packet  */
static uint16_t s_gamma16[256];

static uint8_t s_front = 0; /* ring currently looping in hardware           */

/* dirty span (word offsets) per ring, for ranged cache writeback           */
static size_t s_dirty_lo[2] = {SIZE_MAX, SIZE_MAX};
static size_t s_dirty_hi[2] = {0, 0};

static gdma_channel_handle_t s_dma_chan = nullptr;
static SemaphoreHandle_t s_flip_done = nullptr;

static volatile bool s_flip_pending = false;
static volatile uint32_t s_flip_watch_addr = 0;  /* &s_desc[back][last]     */
static volatile uint32_t s_eof_count = 0;
static volatile uint32_t s_flip_count = 0;

/* -------------------------------------------------------------------------- */
/* BIT-BANGED STARTUP — proven ICND2153/HX6158H init, unchanged               */
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

static void icnd_send_six_words_bitbang(uint16_t v, uint8_t latch_bits)
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
        const bool b = (v & mask) != 0;
        pin_write(PIN_R1, b);
        pin_write(PIN_G1, b);
        pin_write(PIN_B1, b);
        pin_write(PIN_R2, b);
        pin_write(PIN_G2, b);
        pin_write(PIN_B2, b);
        pin_write(PIN_DCLK, true);
        mask >>= 1U;
        bit_index++;
    }
    pin_write(PIN_DCLK, false);
    pin_write(PIN_LE, false);
}

static void icnd_send_value_to_all_drivers_bitbang(uint16_t value,
                                                   uint8_t latch_bits)
{
    for (uint8_t d = 0; d < ICND_DRIVERS_PER_CHAIN; d++) {
        const bool last = d == (ICND_DRIVERS_PER_CHAIN - 1);
        icnd_send_six_words_bitbang(value, last ? latch_bits : 0);
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
}

static void initialize_panel_gpio_bitbang()
{
    for (size_t i = 0; i < LANE_COUNT; i++) {
        configure_output(k_lane_gpio[i]);
        pin_write(k_lane_gpio[i], false);
    }
}

static void icnd_start_memory_protocol_bitbang()
{
    ESP_LOGI(TAG, "ICND init REG1=0x%04X, GCLK/slot=%u, prefix=%u",
             ICND_REG1, ICND_GCLK_PER_SCAN_SLOT, ICND_GCLK_PREFIX);

    icnd_latch_command_bitbang(14); /* PRE-ACTIVE     */
    icnd_latch_command_bitbang(12); /* ENABLE OUTPUTS */
    icnd_latch_command_bitbang(3);  /* VSYNC          */

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
/* RING BUILDER — same emit sequence as the proven encoder, plus a packet     */
/* offset table recorded on the fly (layout is identical for both rings)     */
/* -------------------------------------------------------------------------- */

struct RingWriter {
    uint16_t *data;
    size_t pos;

    inline void push(uint16_t word)
    {
        if (pos >= RING_WORDS) {
            ESP_LOGE(TAG, "ring overflow at word %u",
                     static_cast<unsigned>(pos));
            abort();
        }
        data[pos++] = word;
    }
};

static void emit_vsync(RingWriter &w)
{
    w.push(SIG_LE);
    for (uint8_t i = 0; i < 3; i++) {
        w.push(SIG_LE | SIG_DCLK);
        w.push(SIG_LE);
    }
    w.push(0);
}

static void emit_gclk_pulses(RingWriter &w, uint16_t clocks)
{
    while (clocks-- > 0) {
        w.push(SIG_GCLK);
        w.push(0);
    }
}

static void emit_hx_advance(RingWriter &w, bool inject_token)
{
    if (inject_token) {
        w.push(SIG_ROW_C);
        w.push(SIG_ROW_C | SIG_ROW_A);
        w.push(SIG_ROW_C);
        w.push(0);
    } else {
        w.push(SIG_ROW_A);
        w.push(0);
    }
}

static inline uint16_t expand_gray(uint8_t v)
{
    return s_gamma16[v];
}

static void emit_pixel_packet(RingWriter &w, uint8_t section, uint8_t channel,
                              uint8_t memory_row, const uint8_t *frame)
{
    const uint16_t x =
        static_cast<uint16_t>(section) * ICND_CHANNELS + channel;
    const uint16_t lower_row =
        static_cast<uint16_t>(memory_row) + PANEL_SCAN_ROWS;

    const size_t ui = ((static_cast<size_t>(memory_row) * PANEL_WIDTH) + x) * 3U;
    const size_t li = ((static_cast<size_t>(lower_row) * PANEL_WIDTH) + x) * 3U;

    const uint16_t r1 = expand_gray(frame[ui + 0]);
    const uint16_t g1 = expand_gray(frame[ui + 1]);
    const uint16_t b1 = expand_gray(frame[ui + 2]);
    const uint16_t r2 = expand_gray(frame[li + 0]);
    const uint16_t g2 = expand_gray(frame[li + 1]);
    const uint16_t b2 = expand_gray(frame[li + 2]);

    /* Record where this packet's 32 data words start (same for both rings). */
    const size_t packet =
        ((static_cast<size_t>(memory_row) * ICND_CHANNELS + channel) *
         ICND_DRIVERS_PER_CHAIN) + section;
    s_packet_off[packet] = static_cast<uint32_t>(w.pos);

    const bool final_driver = section == (ICND_DRIVERS_PER_CHAIN - 1);

    for (uint8_t bit = 0; bit < 16; bit++) {
        const uint16_t mask = static_cast<uint16_t>(0x8000U >> bit);
        uint16_t state = 0;
        if (r1 & mask) state |= SIG_R1;
        if (g1 & mask) state |= SIG_G1;
        if (b1 & mask) state |= SIG_B1;
        if (r2 & mask) state |= SIG_R2;
        if (g2 & mask) state |= SIG_G2;
        if (b2 & mask) state |= SIG_B2;
        if (final_driver && (bit == 15)) state |= SIG_LE;

        w.push(state);            /* DCLK low  */
        w.push(state | SIG_DCLK); /* DCLK high */
    }

    w.push(0); /* return DCLK/LE low before the next GCLK burst */
}

static void build_ring(uint16_t *dst, const uint8_t *frame)
{
    RingWriter w = {dst, 0};
    uint8_t hx = 0;

    emit_vsync(w);

    for (uint8_t row = 0; row < PANEL_SCAN_ROWS; row++) {
        for (uint8_t ch = 0; ch < ICND_CHANNELS; ch++) {
            emit_hx_advance(w, hx == 0);
            hx = static_cast<uint8_t>((hx + 1) % PANEL_SCAN_ROWS);
            emit_gclk_pulses(w, ICND_GCLK_PREFIX);
            for (uint8_t s = 0; s < ICND_DRIVERS_PER_CHAIN; s++) {
                emit_gclk_pulses(w, 16);
                emit_pixel_packet(w, s, ch, row, frame);
            }
        }
        w.push(0); /* per-row idle word (byte-exact with proven stream) */
    }

    if (w.pos != RING_WORDS || hx != 0) {
        ESP_LOGE(TAG, "ring build invariant broken: pos=%u hx=%u",
                 static_cast<unsigned>(w.pos), hx);
        abort();
    }
}

/* -------------------------------------------------------------------------- */
/* PIXEL PATCHING — direct read-modify-write into a ring, dirty tracking      */
/* -------------------------------------------------------------------------- */

static void ring_set_pixel(uint8_t ring, uint16_t x, uint16_t y,
                           uint8_t r8, uint8_t g8, uint8_t b8)
{
    if (x >= PANEL_WIDTH || y >= PANEL_HEIGHT) {
        return;
    }

    const uint8_t memory_row = static_cast<uint8_t>(y % PANEL_SCAN_ROWS);
    const uint8_t half = static_cast<uint8_t>(y / PANEL_SCAN_ROWS);
    const uint8_t section = static_cast<uint8_t>(x / ICND_CHANNELS);
    const uint8_t channel = static_cast<uint8_t>(x % ICND_CHANNELS);

    const size_t packet =
        ((static_cast<size_t>(memory_row) * ICND_CHANNELS + channel) *
         ICND_DRIVERS_PER_CHAIN) + section;

    uint16_t *w = s_ring[ring] + s_packet_off[packet];

    const uint16_t r = s_gamma16[r8];
    const uint16_t g = s_gamma16[g8];
    const uint16_t b = s_gamma16[b8];
    const uint16_t lane_mask = (half == 0) ? RGB1_MASK : RGB2_MASK;

    for (uint8_t bit = 0; bit < 16; bit++) {
        const uint16_t gm = static_cast<uint16_t>(0x8000U >> bit);
        uint16_t rgb = 0;
        if (half == 0) {
            if (r & gm) rgb |= SIG_R1;
            if (g & gm) rgb |= SIG_G1;
            if (b & gm) rgb |= SIG_B1;
        } else {
            if (r & gm) rgb |= SIG_R2;
            if (g & gm) rgb |= SIG_G2;
            if (b & gm) rgb |= SIG_B2;
        }
        const size_t lo = static_cast<size_t>(bit) * 2U;
        w[lo] = static_cast<uint16_t>((w[lo] & ~lane_mask) | rgb);
        w[lo + 1] = static_cast<uint16_t>((w[lo + 1] & ~lane_mask) | rgb);
    }

    const size_t off = s_packet_off[packet];
    if (off < s_dirty_lo[ring]) s_dirty_lo[ring] = off;
    if (off + 33 > s_dirty_hi[ring]) s_dirty_hi[ring] = off + 33;
}

static void ring_mark_all_dirty(uint8_t ring)
{
    s_dirty_lo[ring] = 0;
    s_dirty_hi[ring] = RING_WORDS;
}

static void ring_writeback_dirty(uint8_t ring)
{
    if (s_dirty_lo[ring] >= s_dirty_hi[ring]) {
        return; /* nothing to sync */
    }

    /* Align the byte range to cache lines; the ring allocation is
     * 64-aligned and 64-padded, so this never leaves the buffer. */
    uintptr_t base = reinterpret_cast<uintptr_t>(s_ring[ring]);
    uintptr_t lo = base + s_dirty_lo[ring] * sizeof(uint16_t);
    uintptr_t hi = base + s_dirty_hi[ring] * sizeof(uint16_t);
    lo &= ~(uintptr_t)(CACHE_LINE - 1);
    hi = (hi + CACHE_LINE - 1) & ~(uintptr_t)(CACHE_LINE - 1);

    ESP_ERROR_CHECK(esp_cache_msync(reinterpret_cast<void *>(lo),
                                    static_cast<size_t>(hi - lo),
                                    ESP_CACHE_MSYNC_FLAG_DIR_C2M));

    s_dirty_lo[ring] = SIZE_MAX;
    s_dirty_hi[ring] = 0;
}

/* -------------------------------------------------------------------------- */
/* DESCRIPTOR CHAINS                                                          */
/* -------------------------------------------------------------------------- */

static void init_ring_descriptors(uint8_t ring)
{
    uint8_t *base = reinterpret_cast<uint8_t *>(s_ring[ring]);

    for (size_t i = 0; i < DESC_PER_RING; i++) {
        const bool last = i == (DESC_PER_RING - 1);
        const size_t len = last ? LAST_CHUNK_BYTES : DESC_CHUNK_BYTES;

        s_desc[ring][i].dw0.size = DESC_CHUNK_BYTES;
        s_desc[ring][i].dw0.length = len;
        s_desc[ring][i].dw0.reserved24 = 0;
        s_desc[ring][i].dw0.err_eof = 0;
        s_desc[ring][i].dw0.reserved29 = 0;
        s_desc[ring][i].dw0.suc_eof = last ? 1 : 0; /* one EOF per upload */
        s_desc[ring][i].dw0.owner = 1;              /* DMA               */
        s_desc[ring][i].buffer = base + i * DESC_CHUNK_BYTES;
        s_desc[ring][i].next = last ? &s_desc[ring][0] : &s_desc[ring][i + 1];
    }
}

/* -------------------------------------------------------------------------- */
/* GDMA — AHB channel bound to the LCD peripheral, circular chain             */
/* -------------------------------------------------------------------------- */

static bool IRAM_ATTR on_gdma_eof(gdma_channel_handle_t,
                                  gdma_event_data_t *event_data, void *)
{
    s_eof_count = s_eof_count + 1;

    BaseType_t woken = pdFALSE;
    if (s_flip_pending &&
        event_data->tx_eof_desc_addr == s_flip_watch_addr) {
        s_flip_pending = false;
        s_flip_count = s_flip_count + 1;
        xSemaphoreGiveFromISR(s_flip_done, &woken);
    }
    return woken == pdTRUE;
}

static void initialize_gdma()
{
    s_flip_done = xSemaphoreCreateBinary();
    ESP_ERROR_CHECK(s_flip_done != nullptr ? ESP_OK : ESP_ERR_NO_MEM);

    /* v6 API (esp_driver_dma): no .direction field — the direction is
     * expressed by which handle pointers you pass. TX only, so RX = NULL. */
    gdma_channel_alloc_config_t alloc_cfg = {};
    ESP_ERROR_CHECK(gdma_new_ahb_channel(&alloc_cfg, &s_dma_chan, nullptr));

    ESP_ERROR_CHECK(gdma_connect(
        s_dma_chan, GDMA_MAKE_TRIGGER(GDMA_TRIG_PERIPH_LCD, 0)));

    /* access_ext_mem is mandatory: the ring lives in octal PSRAM. */
    gdma_transfer_config_t trans_cfg = {};
    trans_cfg.max_data_burst_size = 64;
    trans_cfg.access_ext_mem = true;
    ESP_ERROR_CHECK(gdma_config_transfer(s_dma_chan, &trans_cfg));

    gdma_tx_event_callbacks_t cbs = {};
    cbs.on_trans_eof = on_gdma_eof;
    ESP_ERROR_CHECK(gdma_register_tx_event_callbacks(s_dma_chan, &cbs, nullptr));
}

/* -------------------------------------------------------------------------- */
/* LCD_CAM — raw register bring-up, i8080 mode, 16-bit, continuous output     */
/* Field names verified against IDF v6.0.1 soc/lcd_cam_struct.h.              */
/* -------------------------------------------------------------------------- */

static void route_lcd_gpio()
{
    for (size_t lane = 0; lane < LANE_COUNT; lane++) {
        const gpio_num_t pin = k_lane_gpio[lane];
        gpio_set_direction(pin, GPIO_MODE_OUTPUT);
        esp_rom_gpio_connect_out_signal(
            pin, LCD_DATA_OUT0_IDX + lane, false, false);
    }
    /* D12-D15, WR/PCLK and DC are intentionally NOT routed:
     * GPIO4-7 (TF card), GPIO8 and GPIO48 stay free for the application. */

#if PCLK_DEBUG_GPIO >= 0
    gpio_set_direction(static_cast<gpio_num_t>(PCLK_DEBUG_GPIO),
                       GPIO_MODE_OUTPUT);
    esp_rom_gpio_connect_out_signal(PCLK_DEBUG_GPIO, LCD_PCLK_IDX,
                                    false, false);
    ESP_LOGI(TAG, "PCLK mirrored on GPIO%d for scope check (expect 10 MHz)",
             PCLK_DEBUG_GPIO);
#endif
}

static void initialize_lcd_cam()
{
    periph_module_enable(PERIPH_LCD_CAM_MODULE);
    periph_module_reset(PERIPH_LCD_CAM_MODULE);

    /* Clock: PLL_F160M / 16 = 10 MHz, PCLK = module clock. */
    LCD_CAM.lcd_clock.clk_en = 1;
    LCD_CAM.lcd_clock.lcd_clk_sel = LCD_CLK_SEL_VALUE;
    LCD_CAM.lcd_clock.lcd_clkm_div_num = LCD_CLK_DIV_NUM;
    LCD_CAM.lcd_clock.lcd_clkm_div_a = 0;
    LCD_CAM.lcd_clock.lcd_clkm_div_b = 0;
    LCD_CAM.lcd_clock.lcd_clk_equ_sysclk = 1; /* PCLK == divided clock */
    LCD_CAM.lcd_clock.lcd_clkcnt_n = 0;
    LCD_CAM.lcd_clock.lcd_ck_idle_edge = 0;
    LCD_CAM.lcd_clock.lcd_ck_out_edge = 0;

    /* i8080 mode, 16-bit bus, no cmd/dummy phases, no color conversion. */
    LCD_CAM.lcd_ctrl.lcd_rgb_mode_en = 0;
    LCD_CAM.lcd_rgb_yuv.lcd_conv_bypass = 0;

    LCD_CAM.lcd_user.lcd_2byte_en = 1;
    LCD_CAM.lcd_user.lcd_byte_order = 0;
    LCD_CAM.lcd_user.lcd_bit_order = 0;
    LCD_CAM.lcd_user.lcd_8bits_order = 0;
    LCD_CAM.lcd_user.lcd_cmd = 0;
    LCD_CAM.lcd_user.lcd_dummy = 0;
    LCD_CAM.lcd_user.lcd_dummy_cyclelen = 0;
    LCD_CAM.lcd_user.lcd_cmd_2_cycle_en = 0;
    LCD_CAM.lcd_user.lcd_dout = 1;
    LCD_CAM.lcd_user.lcd_dout_cyclelen = 8191;
    /* The key bit: never stop at a frame boundary; DMA's circular list is
     * the only frame structure. A momentary DMA stall just holds the last
     * word (all-zero fill would equal our idle word) — both are benign for
     * this encoding, since every control line is a data bit. */
    LCD_CAM.lcd_user.lcd_always_out_en = 1;

    LCD_CAM.lcd_misc.lcd_next_frame_en = 0;
    LCD_CAM.lcd_misc.lcd_bk_en = 0;
    LCD_CAM.lcd_misc.lcd_vfk_cyclelen = 0;
    LCD_CAM.lcd_misc.lcd_vbk_cyclelen = 0;
    LCD_CAM.lcd_misc.lcd_cd_data_set = 0;
    LCD_CAM.lcd_misc.lcd_cd_dummy_set = 0;
    LCD_CAM.lcd_misc.lcd_cd_cmd_set = 0;
    LCD_CAM.lcd_misc.lcd_cd_idle_edge = 0;

    LCD_CAM.lcd_data_dout_mode.val = 0; /* no per-lane output delay */

    /* Reset the peripheral state and its async FIFO before starting. */
    LCD_CAM.lcd_user.lcd_reset = 1;
    LCD_CAM.lcd_misc.lcd_afifo_reset = 1;
}

static void start_lcd_output()
{
    /* DMA first, so the FIFO has data the instant LCD starts clocking. */
    ESP_ERROR_CHECK(gdma_start(
        s_dma_chan, reinterpret_cast<intptr_t>(&s_desc[0][0])));

    esp_rom_delay_us(5);

    LCD_CAM.lcd_user.lcd_update = 1;
    LCD_CAM.lcd_user.lcd_start = 1;
}

/* -------------------------------------------------------------------------- */
/* PRESENT — patch back ring, write back cache, retarget one pointer          */
/* -------------------------------------------------------------------------- */

static uint8_t back_ring()
{
    return s_front ^ 1U;
}

static void commit_and_flip()
{
    const uint8_t back = back_ring();
    const uint8_t front = s_front;

    ring_writeback_dirty(back);

    /* Back ring must self-loop before we route traffic into it. */
    s_desc[back][DESC_PER_RING - 1].next = &s_desc[back][0];

    s_flip_watch_addr =
        reinterpret_cast<uint32_t>(&s_desc[back][DESC_PER_RING - 1]);
    s_flip_pending = true;

    /* Single aligned pointer store; DMA picks it up at the next upload
     * boundary. Descriptors are in internal (uncached) DRAM, so no msync. */
    __atomic_store_n(&s_desc[front][DESC_PER_RING - 1].next,
                     &s_desc[back][0], __ATOMIC_RELEASE);

    /* Worst case: retarget lands just after the boundary -> ~2 uploads.
     * A timeout here means the EOF never arrived (or arrived >100 ms
     * late) AFTER the retarget was already published: hardware may or
     * may not have switched rings, so s_front can no longer be trusted.
     * In this test build, fail loud. The library port should instead
     * resolve ground truth via gdma_get_group_channel_id() + the GDMA
     * current-descriptor register (is it inside ring A's or ring B's
     * descriptor range?), and drain a possibly-late semaphore give
     * (xSemaphoreTake(s_flip_done, 0)) before arming the next flip. */
    if (xSemaphoreTake(s_flip_done, pdMS_TO_TICKS(100)) != pdTRUE) {
        ESP_LOGE(TAG, "FATAL flip timeout: ring ownership unknown "
                      "(EOF lost or DMA halted)");
        abort();
    }

    s_front = back;
}

/* -------------------------------------------------------------------------- */
/* TEST PATTERNS (drawn straight into the back ring)                          */
/* -------------------------------------------------------------------------- */

static constexpr uint8_t TEST_LEVEL = 64;

static void draw_solid(uint8_t ring, uint8_t r, uint8_t g, uint8_t b)
{
    for (uint16_t y = 0; y < PANEL_HEIGHT; y++)
        for (uint16_t x = 0; x < PANEL_WIDTH; x++)
            ring_set_pixel(ring, x, y, r, g, b);
}

static void draw_bands(uint8_t ring)
{
    for (uint16_t y = 0; y < PANEL_HEIGHT; y++) {
        for (uint16_t x = 0; x < PANEL_WIDTH; x++) {
            switch (x / 16U) {
                case 0:  ring_set_pixel(ring, x, y, TEST_LEVEL, 0, 0); break;
                case 1:  ring_set_pixel(ring, x, y, 0, TEST_LEVEL, 0); break;
                case 2:  ring_set_pixel(ring, x, y, 0, 0, TEST_LEVEL); break;
                case 3:  ring_set_pixel(ring, x, y, TEST_LEVEL, TEST_LEVEL, 0); break;
                default: ring_set_pixel(ring, x, y, TEST_LEVEL, 0, TEST_LEVEL); break;
            }
        }
    }
}

static void draw_boundaries(uint8_t ring)
{
    draw_solid(ring, 0, 0, 0);

    static const uint16_t xs[] = {0, 15, 16, 31, 32, 47, 48, 63, 64, 79};
    static const uint16_t ys[] = {0, 19, 20, 39};

    for (uint16_t x : xs)
        for (uint16_t y = 0; y < PANEL_HEIGHT; y++)
            ring_set_pixel(ring, x, y, TEST_LEVEL, TEST_LEVEL, TEST_LEVEL);
    for (uint16_t y : ys)
        for (uint16_t x = 0; x < PANEL_WIDTH; x++)
            ring_set_pixel(ring, x, y, TEST_LEVEL, TEST_LEVEL, TEST_LEVEL);
}

static void pattern_task(void *)
{
    /* Let the black rings loop a few uploads so both ICND memory banks are
     * deterministic before the first visible frame. */
    vTaskDelay(pdMS_TO_TICKS(200));

    while (true) {
        ESP_LOGI(TAG, "Pattern: RED");
        draw_solid(back_ring(), TEST_LEVEL, 0, 0);
        commit_and_flip();
        vTaskDelay(pdMS_TO_TICKS(4000));

        ESP_LOGI(TAG, "Pattern: GREEN");
        draw_solid(back_ring(), 0, TEST_LEVEL, 0);
        commit_and_flip();
        vTaskDelay(pdMS_TO_TICKS(4000));

        ESP_LOGI(TAG, "Pattern: BLUE");
        draw_solid(back_ring(), 0, 0, TEST_LEVEL);
        commit_and_flip();
        vTaskDelay(pdMS_TO_TICKS(4000));

        ESP_LOGI(TAG, "Pattern: WHITE");
        draw_solid(back_ring(), TEST_LEVEL, TEST_LEVEL, TEST_LEVEL);
        commit_and_flip();
        vTaskDelay(pdMS_TO_TICKS(4000));

        ESP_LOGI(TAG, "Pattern: five 16-column sections");
        draw_bands(back_ring());
        commit_and_flip();
        vTaskDelay(pdMS_TO_TICKS(6000));

        ESP_LOGI(TAG, "Pattern: boundaries");
        draw_boundaries(back_ring());
        commit_and_flip();
        vTaskDelay(pdMS_TO_TICKS(6000));
    }
}

static void telemetry_task(void *)
{
    /* Snapshot, don't start at zero: EOFs accumulate from lcd_start()
     * onward, before this task exists. Counting them in window 1 produced
     * the 71.81 reading (359 EOFs / 5 s instead of ~352.5). */
    uint32_t last_eof = s_eof_count;
    int64_t last_us = esp_timer_get_time();

    while (true) {
        vTaskDelay(pdMS_TO_TICKS(5000));

        const uint32_t eof = s_eof_count;
        const int64_t now = esp_timer_get_time();
        const float secs = static_cast<float>(now - last_us) / 1e6f;

        ESP_LOGI(TAG,
                 "uploads/s=%.2f (expect ~70.5 @10MHz), flips=%u, "
                 "front=ring%u, internal free=%u",
                 static_cast<float>(eof - last_eof) / secs,
                 static_cast<unsigned>(s_flip_count),
                 static_cast<unsigned>(s_front),
                 static_cast<unsigned>(heap_caps_get_free_size(
                     MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT)));

        last_eof = eof;
        last_us = now;
    }
}

/* -------------------------------------------------------------------------- */
/* SETUP HELPERS                                                              */
/* -------------------------------------------------------------------------- */

static void initialize_gamma()
{
    for (uint32_t i = 0; i < 256; i++) {
        if (GAMMA_EXPONENT == 1.0f) {
            s_gamma16[i] = static_cast<uint16_t>(i * 257U);
        } else {
            const float n = static_cast<float>(i) / 255.0f;
            float c = 1.0f;
            /* powf via exp/log is fine here; startup-only */
            c = __builtin_powf(n, GAMMA_EXPONENT);
            s_gamma16[i] = static_cast<uint16_t>(c * 65535.0f + 0.5f);
        }
    }
}

static void allocate_and_build_rings()
{
    for (uint8_t r = 0; r < 2; r++) {
        s_ring[r] = static_cast<uint16_t *>(heap_caps_aligned_alloc(
            CACHE_LINE, RING_BYTES_PADDED,
            MALLOC_CAP_SPIRAM | MALLOC_CAP_DMA | MALLOC_CAP_8BIT));
        if (s_ring[r] == nullptr) {
            ESP_LOGE(TAG, "PSRAM ring %u allocation failed", r);
            abort();
        }
    }

    /* Build both rings black; padding stays zero. */
    static uint8_t black[PANEL_WIDTH * PANEL_HEIGHT * 3]; /* zero-init BSS */
    memset(s_ring[0], 0, RING_BYTES_PADDED);
    memset(s_ring[1], 0, RING_BYTES_PADDED);
    build_ring(s_ring[0], black);
    build_ring(s_ring[1], black);

    /* Full writeback of both rings before DMA ever reads them. */
    ring_mark_all_dirty(0);
    ring_writeback_dirty(0);
    ring_mark_all_dirty(1);
    ring_writeback_dirty(1);

    init_ring_descriptors(0);
    init_ring_descriptors(1);

    ESP_LOGI(TAG,
             "rings: 2 x %u B PSRAM @ %p / %p, %u desc x %u B chunks, "
             "packets=%u",
             static_cast<unsigned>(RING_BYTES), s_ring[0], s_ring[1],
             static_cast<unsigned>(DESC_PER_RING),
             static_cast<unsigned>(DESC_CHUNK_BYTES),
             static_cast<unsigned>(TOTAL_PACKETS));
}

/* -------------------------------------------------------------------------- */
/* APPLICATION ENTRY                                                          */
/* -------------------------------------------------------------------------- */

extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "ESP32-S3 raw LCD_CAM/GDMA PSRAM ring: ICND2153 + HX6158H");
    ESP_LOGI(TAG, "Panel %ux%u scan=%u chain=%u, ring=%u words (%u B)",
             PANEL_WIDTH, PANEL_HEIGHT, PANEL_SCAN_ROWS,
             ICND_DRIVERS_PER_CHAIN,
             static_cast<unsigned>(RING_WORDS),
             static_cast<unsigned>(RING_BYTES));

    const size_t psram_total = heap_caps_get_total_size(MALLOC_CAP_SPIRAM);
    ESP_LOGI(TAG, "PSRAM total=%u free=%u",
             static_cast<unsigned>(psram_total),
             static_cast<unsigned>(
                 heap_caps_get_free_size(MALLOC_CAP_SPIRAM)));
    if (psram_total < 7U * 1024U * 1024U) {
        ESP_LOGE(TAG, "expected ~8 MB PSRAM");
        abort();
    }

    initialize_gamma();

    /* 1. Proven bit-banged bring-up: ICND registers + HX chain clear.     */
    initialize_panel_gpio_bitbang();
    hx_clear_chain_bitbang();
    icnd_start_memory_protocol_bitbang();

    /* 2. Waveform rings + descriptor chains.                              */
    allocate_and_build_rings();

    /* 3. Peripherals: GDMA channel, then LCD_CAM, then GPIO matrix.       */
    initialize_gdma();
    initialize_lcd_cam();
    route_lcd_gpio();

    /* 4. Go. From here the panel refreshes with zero CPU involvement.     */
    start_lcd_output();

    vTaskDelay(pdMS_TO_TICKS(100));
    ESP_LOGI(TAG, "streaming: eof_count=%u (should be increasing)",
             static_cast<unsigned>(s_eof_count));

    BaseType_t ok = xTaskCreate(pattern_task, "panel_patterns", 4096,
                                nullptr, 5, nullptr);
    ESP_ERROR_CHECK(ok == pdPASS ? ESP_OK : ESP_ERR_NO_MEM);

    ok = xTaskCreate(telemetry_task, "panel_telemetry", 4096,
                     nullptr, 3, nullptr);
    ESP_ERROR_CHECK(ok == pdPASS ? ESP_OK : ESP_ERR_NO_MEM);
}