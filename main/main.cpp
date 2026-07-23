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
 *   - Frame presentation = draw on a canonical 9.6 KB RGB canvas, then
 *     present(): diff against the back ring's shadow, patch only changed
 *     pixels, cache-writeback only touched lines (coalesced), retarget
 *     one descriptor 'next' pointer. The switch
 *     lands exactly at the upload boundary (the ring starts with VSYNC),
 *     confirmed by the GDMA EOF interrupt.
 *   - esp_lcd is not used. Only LCD data lanes D0..D11 are routed, so
 *     GPIO4-7 (TF card), GPIO8 and GPIO48 are returned to the board.
 *   - VERIFIED PANEL CONFIG: REG1 0x1370 (= (scan_rows-1)<<8 | 0x70),
 *     REG2 0xFFFF, REG3 0x40FC, REG4 0x0000, DEBUG 0x0000, GCLK/slot
 *     138 (prefix 58), max drive 0.85, gamma 2.2, OE enabled on GCLK
 *     only. Every one of these was established on hardware; none is
 *     inherited unverified from the demo source.
 *   - ROW OE DISCIPLINE (proven): rows are enabled only during the 138
 *     GCLK pulses per slot; blanked through vsync, HX advances, pixel
 *     packets, LE and idle. Without this the ICND2153's residual output
 *     during GCLK-paused intervals paints a full-half vertical ghost
 *     bar at every lit column. Costs zero ring bytes.
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
#define PIN_ROW_OE_N GPIO_NUM_2       /* HX6158H active-low row output enable  */
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

/*
 * REG1 high byte = scan_rows - 1. PROVEN: 0x1370 (19) works on this
 * 20-scan panel; forcing 0x1F70 (31, the 32-scan value from the demo
 * source) produces full-white noise because the chip then expects a
 * 32-row frame while we upload 20 — its display-SRAM addressing and PWM
 * framing desync. The transport is unaffected (still 70.52 uploads/s),
 * so this is purely a chip-side interpretation failure. Never hardcode.
 */
static constexpr uint16_t ICND_REG1 =
    static_cast<uint16_t>(((PANEL_SCAN_ROWS - 1) << 8) | 0x70);
static_assert(ICND_REG1 == 0x1370, "REG1 drifted from the proven value");
static constexpr uint16_t ICND_REG2 = 0xFFFF;
/*
 * REG3 low nibble: bits 2-3 set (0b1100), NOT bits 0-1 (0b0011) as in
 * every demo-derived value. PROVEN by A/B at identical drive: 0x40F3 at
 * drive 0.45 still shows a green lower ghost; 0x40FC at 0.45 is clean in
 * both colours, and 0x40FC remains clean all the way to 0.85 with a
 * moving full-scale dot. The lower-ghost threshold moved from 0.60
 * (0x40F3) to somewhere above 0.85 (0x40FC).
 */
static constexpr uint16_t ICND_REG3 = 0x40FC;
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
 * Live ICND configuration. Defaults are the proven demo-derived values;
 * ghost-tune mode rewrites reg3 between sweep steps.
 */
struct IcndConfig {
    uint16_t reg1;
    uint16_t reg2;
    uint16_t reg3;
    uint16_t reg4;
    uint16_t dbg;
};

static IcndConfig s_icnd_cfg = {
    ICND_REG1, ICND_REG2, ICND_REG3, ICND_REG4, ICND_DEBUG,
};

/*
 * HX advance placement within the scan slot: number of prefix GCLK
 * pulses emitted BEFORE the row-advance. 0 = advance first (the original
 * construction). The ICND2153 frames its own 138-GCLK line internally
 * and blanks outputs around ITS line boundary; sliding the physical row
 * switch through the prefix probes for that blank window. The slot's
 * word budget is invariant, so ring size and packet offsets never move.
 */
static uint16_t s_hx_phase = 0;

/*
 * ROW OUTPUT-ENABLE POLICY (HX6158H pin B, active low).
 *
 * PROVEN ON HARDWARE by a static-image A/B/A (B high => panel black;
 * enable-on-GCLK-only => ghost gone; enable-always => ghost returns):
 * the ICND2153 column outputs conduct residual state whenever GCLK is
 * paused (packet shifting, LE, HX advance, vsync, idle). With rows
 * enabled continuously, the selected row conducted that residual and
 * painted a full-half vertical bar at the lit column's x.
 *
 * PRODUCTION INVARIANT (mode 3): rows are enabled ONLY during the 138
 * GCLK pulses of each slot and blanked through every other word. The
 * blanked intervals were never supposed to emit light, so legitimate
 * brightness is essentially unaffected. Do NOT enable rows during
 * pixel packets to "reclaim duty" — that is exactly the ghost.
 *
 * Modes 0/1/2 are retained only for the ghost-tuning harness:
 *   0 = enabled always (the original bug — reproduces the ghost)
 *   1 = blanked always (panel black; the OE-polarity proof)
 *   2 = blanked only as a guard around the HX advance
 *   3 = PRODUCTION: enabled only during GCLK
 */
static uint8_t s_row_oe_mode = 3;

/*
 * Settling blank: keep rows disabled for the first N GCLK pulses AFTER
 * the HX advance, giving the previous row's column drive longer to decay
 * before the new row is enabled. 0 = the proven policy. This is one of
 * two levers that may buy drive headroom above the measured threshold
 * (the other is REG2 current gain); it costs N/138 of brightness.
 *
 * NOTE: these pulses must come from the POST-advance budget. At the
 * default hx_phase = 0 the entire 58-pulse prefix is emitted after the
 * advance, so a pre-advance settle would be silently ignored.
 */
static uint16_t s_oe_settle = 0;

/*
 * GHOST TUNING MODE
 *
 * 1 = the pattern task becomes a sweep harness: the bouncing dot runs
 * forever while sweep steps are applied live every ~12 s (transport
 * stopped, rings rebuilt from the canvas, bit-banged init re-run,
 * verified restart). The dot cycles red/green/blue/cyan per step.
 * 0 = normal pattern cycle.
 */
#define GHOST_TUNE_MODE 0

/*
 * PROBE MODE (within tune mode): instead of sweeping, apply the baseline
 * config once, draw a STATIC dot, and hold forever — a stable target for
 * oscilloscope work. The vsync words also drive LCD lane D12, routed to
 * FRAME_SYNC_GPIO: an ~800 ns pulse at every upload start to trigger on.
 * With the dot at (PROBE_DOT_X, PROBE_DOT_Y): its physical row is
 * selected during slots ≡ y (mod 20); slot ≈ 44.3 µs, so visits recur
 * every ~886 µs, 16 per 14.18 ms frame, the first ~y*44.3 µs after the
 * sync pulse. Set to 0 to run the sweep harness instead.
 */
#define GHOST_PROBE_MODE 1
#define PROBE_DOT_X 8
#define PROBE_DOT_Y 5
#define FRAME_SYNC_GPIO 48   /* -1 to disable; rides unused LCD lane D12 */

static constexpr uint16_t SIG_FRAME_SYNC = 1U << 12;

#if GHOST_TUNE_MODE
struct SweepStep {
    const char *label;
    uint16_t reg2;
    uint16_t reg3;
    uint16_t reg4;
    uint16_t hx_phase;
    uint8_t b_mode;
    float drive;     /* max drive as a fraction of full scale */
    uint16_t settle; /* post-advance blanked GCLK pulses      */
};

/*
 * One lap answers four questions (~3.5 min):
 *   1-9   HX-advance phase     (persistent-node model predicts null)
 *   10-12 REG2 current gain    (datasheet: 8-bit gain 12.5%..200%;
 *         0xFFFF is the 200% end. If the bar dims MUCH faster than the
 *         dot as gain drops -> output-stage charge/tail; if bar/dot
 *         ratio is constant -> leakage tracking programmed current)
 *   13-14 REG3 with bit14 CLEARED (never tried; lap one only set bits)
 *   15-16 ROW_B: OE role check, then transition guard
 */
/*
 * DRIVE CEILING WITH REG3 0x40FC.
 *
 * The production register set is now fixed; the only open question is
 * how much brightness it bought. 0.85 is verified clean at full-scale
 * content. This walks up to find the first level where
 * the two-row lower ghost returns, with a clean reference every few
 * steps so the eye stays calibrated. Whatever level fails, ship one
 * step below it.
 *
 * Watch the two rows immediately BELOW the dot, red and green
 * separately. The dot itself gets brighter each step; that is the point.
 */
static const SweepStep k_sweep[] = {
    /*  label                   REG2    REG3    REG4   hx oe drive settle */
    {"REF clean      0.45",    0xFFFF, 0x40FC, 0x0000, 0, 3, 0.45f, 0},
    {"drive 0.50",             0xFFFF, 0x40FC, 0x0000, 0, 3, 0.50f, 0},
    {"drive 0.55",             0xFFFF, 0x40FC, 0x0000, 0, 3, 0.55f, 0},
    {"drive 0.60",             0xFFFF, 0x40FC, 0x0000, 0, 3, 0.60f, 0},
    {"REF clean      0.45",    0xFFFF, 0x40FC, 0x0000, 0, 3, 0.45f, 0},
    {"drive 0.65",             0xFFFF, 0x40FC, 0x0000, 0, 3, 0.65f, 0},
    {"drive 0.70",             0xFFFF, 0x40FC, 0x0000, 0, 3, 0.70f, 0},
    {"drive 0.75",             0xFFFF, 0x40FC, 0x0000, 0, 3, 0.75f, 0},
    {"REF clean      0.45",    0xFFFF, 0x40FC, 0x0000, 0, 3, 0.45f, 0},
    {"drive 0.85",             0xFFFF, 0x40FC, 0x0000, 0, 3, 0.85f, 0},
    {"drive 1.00",             0xFFFF, 0x40FC, 0x0000, 0, 3, 1.00f, 0},
    {"drive 1.00 + settle 16", 0xFFFF, 0x40FC, 0x0000, 0, 3, 1.00f, 16},
};




static constexpr size_t SWEEP_COUNT = sizeof(k_sweep) / sizeof(k_sweep[0]);
#endif

/*
 * Gamma exponent for the u8 -> u16 grayscale expansion.
 * 1.0f reproduces the proven linear (*257) behavior exactly — keep it for
 * the transport-verification flash so brightness is comparable, then
 * switch to 2.2f once the ring is confirmed pixel-identical.
 */
static constexpr float GAMMA_EXPONENT = 2.2f;

/*
 * Maximum drive as a fraction of full scale. THIS is the ghost control,
 * not the gamma exponent: a measured brightness sweep put the ghost
 * threshold between 50% and 87% of full-scale grayscale (visible at
 * ~87%, clean at ~50%), because at high current the ICND column's
 * turn-off tail outlasts the ~3.5 us blanked window between one row's
 * last GCLK and the next row's first.
 *
 * Gamma alone does NOT protect: value 255 maps to 65535 at ANY exponent
 * (2.2 only darkens midtones), so one full-white pixel would ghost.
 * Capping max drive is what keeps every pixel inside the safe envelope.
 *
 * MEASURED on this panel: with REG3 0x40F3 the "lower ghost" (red and
 * green on the two rows scanned immediately after a lit row) appeared
 * at drive 0.60. With REG3 0x40FC it is clean at 0.85 with a moving
 * full-scale dot - the register bought roughly double the headroom, so
 * these are NOT a matched pair as an earlier revision of this comment
 * claimed. The lower ghost is a different artifact from the full-half
 * green bar the OE policy fixed: it is confined to the next ~2 slots
 * (~88 us of decay) rather than spanning the whole half.
 */
static float s_max_drive = 0.85f;
static_assert(GAMMA_EXPONENT > 0.0f, "gamma 0 maps every level to full scale");

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
    LANE_ROW_A = 9, LANE_ROW_OE_N = 10, LANE_ROW_C = 11,
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
static constexpr uint16_t SIG_ROW_OE_N = 1U << LANE_ROW_OE_N;
static constexpr uint16_t SIG_ROW_C = 1U << LANE_ROW_C;

static constexpr uint16_t RGB1_MASK = SIG_R1 | SIG_G1 | SIG_B1;
static constexpr uint16_t RGB2_MASK = SIG_R2 | SIG_G2 | SIG_B2;

static const gpio_num_t k_lane_gpio[LANE_COUNT] = {
    PIN_R1, PIN_G1, PIN_B1, PIN_R2, PIN_G2, PIN_B2,
    PIN_DCLK, PIN_LE, PIN_GCLK, PIN_ROW_A, PIN_ROW_OE_N, PIN_ROW_C,
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

/*
 * Presentation layer state.
 *
 * Drawing NEVER touches the rings: all primitives write the canonical
 * 8-bit RGB canvas. present() discovers what changed by diffing the
 * canvas against the back ring's shadow (a copy of the canvas state that
 * ring currently encodes), patches only those pixels, and syncs only the
 * cache lines it touched. The line bitmap is transient scratch inside
 * present() — it never outlives a call, so one bitmap serves both rings
 * and drawing code carries zero dirty-tracking obligations.
 */
static constexpr size_t CANVAS_BYTES =
    static_cast<size_t>(PANEL_WIDTH) * PANEL_HEIGHT * 3U; /* 9,600 */

static uint8_t s_canvas[CANVAS_BYTES];       /* what the app draws        */
static uint8_t s_shadow[2][CANVAS_BYTES];    /* what each ring encodes    */

static constexpr size_t RING_LINES = RING_BYTES_PADDED / CACHE_LINE; /* 4432 */
static uint32_t s_line_dirty[(RING_LINES + 31) / 32]; /* scratch, 556 B    */

static SemaphoreHandle_t s_present_mutex = nullptr;

/* last-present stats, read by telemetry */
static uint32_t s_present_px = 0;
static uint32_t s_present_lines = 0;
static uint32_t s_present_sync_bytes = 0;
static uint32_t s_present_work_us = 0;

static gdma_channel_handle_t s_dma_chan = nullptr;
static SemaphoreHandle_t s_flip_done = nullptr;

/* ISR<->task shared state: accessed only via __atomic builtins. The
 * watch/pending pair uses release/acquire so the ISR can never observe
 * pending=true before the watch address it must compare against. */
static bool s_flip_pending = false;
static uint32_t s_flip_watch_addr = 0;  /* &s_desc[back][last] */
static uint32_t s_eof_count = 0;
static uint32_t s_flip_count = 0;

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
    pin_write(PIN_ROW_OE_N, false);
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
    ESP_LOGI(TAG,
             "ICND init REG1=0x%04X REG2=0x%04X REG3=0x%04X REG4=0x%04X "
             "DBG=0x%04X, GCLK/slot=%u, prefix=%u",
             s_icnd_cfg.reg1, s_icnd_cfg.reg2, s_icnd_cfg.reg3,
             s_icnd_cfg.reg4, s_icnd_cfg.dbg,
             ICND_GCLK_PER_SCAN_SLOT, ICND_GCLK_PREFIX);

    icnd_latch_command_bitbang(14); /* PRE-ACTIVE     */
    icnd_latch_command_bitbang(12); /* ENABLE OUTPUTS */
    icnd_latch_command_bitbang(3);  /* VSYNC          */

    icnd_latch_command_bitbang(14);
    icnd_send_value_to_all_drivers_bitbang(s_icnd_cfg.reg1, 4);
    icnd_latch_command_bitbang(14);
    icnd_send_value_to_all_drivers_bitbang(s_icnd_cfg.reg2, 6);
    icnd_latch_command_bitbang(14);
    icnd_send_value_to_all_drivers_bitbang(s_icnd_cfg.reg3, 8);
    icnd_latch_command_bitbang(14);
    icnd_send_value_to_all_drivers_bitbang(s_icnd_cfg.reg4, 10);
    icnd_latch_command_bitbang(14);
    icnd_send_value_to_all_drivers_bitbang(s_icnd_cfg.dbg, 2);

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

static void emit_vsync(RingWriter &w, uint16_t or_mask = 0)
{
    /* SIG_FRAME_SYNC rides otherwise-unused lane D12 through the whole
     * vsync burst (~800 ns) — a scope trigger at every upload start.
     * Unrouted unless FRAME_SYNC_GPIO >= 0. */
    w.push(static_cast<uint16_t>(SIG_LE | SIG_FRAME_SYNC | or_mask));
    for (uint8_t i = 0; i < 3; i++) {
        w.push(static_cast<uint16_t>(SIG_LE | SIG_DCLK | SIG_FRAME_SYNC |
                                     or_mask));
        w.push(static_cast<uint16_t>(SIG_LE | SIG_FRAME_SYNC | or_mask));
    }
    w.push(static_cast<uint16_t>(SIG_FRAME_SYNC | or_mask));
}

static void emit_gclk_pulses(RingWriter &w, uint16_t clocks,
                             uint16_t or_mask = 0)
{
    while (clocks-- > 0) {
        w.push(static_cast<uint16_t>(SIG_GCLK | or_mask));
        w.push(or_mask);
    }
}

static void emit_hx_advance(RingWriter &w, bool inject_token,
                            uint16_t or_mask = 0)
{
    if (inject_token) {
        w.push(static_cast<uint16_t>(SIG_ROW_C | or_mask));
        w.push(static_cast<uint16_t>(SIG_ROW_C | SIG_ROW_A | or_mask));
        w.push(static_cast<uint16_t>(SIG_ROW_C | or_mask));
        w.push(or_mask);
    } else {
        w.push(static_cast<uint16_t>(SIG_ROW_A | or_mask));
        w.push(or_mask);
    }
}

static inline uint16_t expand_gray(uint8_t v)
{
    return s_gamma16[v];
}

static void emit_pixel_packet(RingWriter &w, uint8_t section, uint8_t channel,
                              uint8_t memory_row, const uint8_t *frame,
                              uint16_t or_mask = 0)
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
        uint16_t state = or_mask;
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

    w.push(or_mask); /* return DCLK/LE low before the next GCLK burst */
}

static void build_ring(uint16_t *dst, const uint8_t *frame)
{
    if (s_hx_phase > ICND_GCLK_PREFIX) {
        ESP_LOGE(TAG, "hx_phase %u exceeds prefix %u", s_hx_phase,
                 ICND_GCLK_PREFIX);
        abort();
    }

    RingWriter w = {dst, 0};
    uint8_t hx = 0;

    /* Mode 3: rows enabled only during GCLK — blank everything else. */
    const uint16_t blank = (s_row_oe_mode == 3) ? SIG_ROW_OE_N : 0;

    emit_vsync(w, blank);

    for (uint8_t row = 0; row < PANEL_SCAN_ROWS; row++) {
        for (uint8_t ch = 0; ch < ICND_CHANNELS; ch++) {
            const uint16_t b_guard =
                (s_row_oe_mode == 2) ? SIG_ROW_OE_N : 0;

            const uint16_t pre = s_hx_phase;
            const uint16_t guard_pre = (pre < 4) ? pre : 4;
            const uint16_t tail_budget =
                static_cast<uint16_t>(ICND_GCLK_PREFIX - pre);
            const uint16_t guard_post =
                (tail_budget < 4) ? tail_budget : 4;

            emit_gclk_pulses(w,
                             static_cast<uint16_t>(pre - guard_pre));
            emit_gclk_pulses(w, guard_pre, b_guard);
            emit_hx_advance(w, hx == 0,
                            static_cast<uint16_t>(b_guard | blank));
            hx = static_cast<uint8_t>((hx + 1) % PANEL_SCAN_ROWS);
            emit_gclk_pulses(w, guard_post, b_guard);

            /* Settling blank: first N post-advance GCLKs with rows off. */
            const uint16_t tail_free =
                static_cast<uint16_t>(tail_budget - guard_post);
            const uint16_t settle =
                (blank != 0 && s_oe_settle < tail_free) ? s_oe_settle : 0;
            emit_gclk_pulses(w, settle, blank);
            emit_gclk_pulses(w,
                             static_cast<uint16_t>(tail_free - settle));
            for (uint8_t s = 0; s < ICND_DRIVERS_PER_CHAIN; s++) {
                emit_gclk_pulses(w, 16);
                emit_pixel_packet(w, s, ch, row, frame, blank);
            }
        }
        w.push(blank); /* per-row idle word */
    }

    if (w.pos != RING_WORDS || hx != 0) {
        ESP_LOGE(TAG, "ring build invariant broken: pos=%u hx=%u",
                 static_cast<unsigned>(w.pos), hx);
        abort();
    }

    /* Mode 1: true ROW_B-high test — OR B into EVERY word, including
     * vsync, packets, and idle. Safe under later pixel patching:
     * ring_set_pixel masks only the RGB lanes and preserves B. */
    if (s_row_oe_mode == 1) {
        for (size_t i = 0; i < RING_WORDS; i++) {
            dst[i] |= SIG_ROW_OE_N;
        }
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

    /* Mark the cache lines this packet's byte range touches (66 bytes:
     * 33 words). Bitmap is present()-transient scratch, see above. */
    const size_t off = s_packet_off[packet];
    const size_t first_line = (off * sizeof(uint16_t)) / CACHE_LINE;
    const size_t last_line =
        ((off + 33) * sizeof(uint16_t) - 1) / CACHE_LINE;
    for (size_t l = first_line; l <= last_line; l++) {
        s_line_dirty[l >> 5] |= 1UL << (l & 31);
    }
}

/*
 * Write back every dirty cache line of the ring, coalescing runs into as
 * few esp_cache_msync calls as possible. A single CLEAN line between two
 * dirty ones is bridged: full repaints touch every packet but skip the
 * 64-byte GCLK gaps between them, and without bridging that alternation
 * would fragment into ~1,600 tiny msync calls. Bridging costs one extra
 * clean line per merge and collapses full repaints into a handful of
 * large syncs, while sparse updates stay tight.
 */
static void msync_dirty_lines(uint8_t ring, uint32_t *out_lines,
                              uint32_t *out_bytes)
{
    uint8_t *base = reinterpret_cast<uint8_t *>(s_ring[ring]);
    uint32_t lines = 0;
    uint32_t bytes = 0;

    auto dirty = [](size_t l) -> bool {
        return (s_line_dirty[l >> 5] & (1UL << (l & 31))) != 0;
    };

    size_t line = 0;
    while (line < RING_LINES) {
        /* fast-skip fully clean bitmap words */
        if ((line & 31) == 0 && s_line_dirty[line >> 5] == 0) {
            line += 32;
            continue;
        }
        if (!dirty(line)) {
            line++;
            continue;
        }

        size_t run = line;
        while (run < RING_LINES) {
            if (dirty(run)) {
                run++;
            } else if (run + 1 < RING_LINES && dirty(run + 1)) {
                run++; /* bridge a single clean line */
            } else {
                break;
            }
        }

        const size_t len = (run - line) * CACHE_LINE;
        ESP_ERROR_CHECK(esp_cache_msync(base + line * CACHE_LINE, len,
                                        ESP_CACHE_MSYNC_FLAG_DIR_C2M));
        lines += static_cast<uint32_t>(run - line);
        bytes += static_cast<uint32_t>(len);
        line = run;
    }

    *out_lines = lines;
    *out_bytes = bytes;
}

/* Full writeback: used once per ring after the initial black build. */
static void ring_writeback_full(uint8_t ring)
{
    ESP_ERROR_CHECK(esp_cache_msync(s_ring[ring], RING_BYTES_PADDED,
                                    ESP_CACHE_MSYNC_FLAG_DIR_C2M));
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
    __atomic_fetch_add(&s_eof_count, 1U, __ATOMIC_RELAXED);

    BaseType_t woken = pdFALSE;
    if (__atomic_load_n(&s_flip_pending, __ATOMIC_ACQUIRE) &&
        event_data->tx_eof_desc_addr ==
            __atomic_load_n(&s_flip_watch_addr, __ATOMIC_RELAXED)) {
        __atomic_store_n(&s_flip_pending, false, __ATOMIC_RELAXED);
        __atomic_fetch_add(&s_flip_count, 1U, __ATOMIC_RELAXED);
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
    /* D13-D15, WR/PCLK and DC are intentionally NOT routed:
     * GPIO4-7 (TF card) and GPIO8 stay free for the application. */

#if FRAME_SYNC_GPIO >= 0
    gpio_set_direction(static_cast<gpio_num_t>(FRAME_SYNC_GPIO),
                       GPIO_MODE_OUTPUT);
    esp_rom_gpio_connect_out_signal(FRAME_SYNC_GPIO,
                                    LCD_DATA_OUT0_IDX + 12, false, false);
#endif

#if PCLK_DEBUG_GPIO >= 0
    gpio_set_direction(static_cast<gpio_num_t>(PCLK_DEBUG_GPIO),
                       GPIO_MODE_OUTPUT);
    esp_rom_gpio_connect_out_signal(PCLK_DEBUG_GPIO, LCD_PCLK_IDX,
                                    false, false);
    static bool s_pclk_logged = false;
    if (!s_pclk_logged) {
        s_pclk_logged = true;
        ESP_LOGI(TAG,
                 "PCLK mirrored on GPIO%d for scope check (expect 10 MHz)",
                 PCLK_DEBUG_GPIO);
    }
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

/* -------------------------------------------------------------------------- */
/* TRANSPORT START/STOP — verified start with retry                           */
/*                                                                            */
/* The LCD_CAM start sequence is marginal: on a small fraction of attempts    */
/* (observed at cold boot AND after live reconfiguration) the peripheral      */
/* never begins consuming DMA data. The EOF counter is a perfect started-     */
/* ness oracle — the first EOF arrives ~14.2 ms after any good start — so     */
/* start is verified and retried instead of trusted.                          */
/* -------------------------------------------------------------------------- */

static void transport_stop()
{
    /* Consumer first, then producer: stopping the LCD before GDMA means
     * we never force an underrun mid-word. */
    LCD_CAM.lcd_user.lcd_start = 0;
    LCD_CAM.lcd_user.lcd_update = 1;

    ESP_ERROR_CHECK(gdma_stop(s_dma_chan));
    ESP_ERROR_CHECK(gdma_reset(s_dma_chan));

    LCD_CAM.lcd_user.lcd_reset = 1;
    LCD_CAM.lcd_misc.lcd_afifo_reset = 1;

    __atomic_store_n(&s_flip_pending, false, __ATOMIC_RELAXED);
    xSemaphoreTake(s_flip_done, 0); /* drain any in-flight give */

    /* Hand every panel pin back to plain GPIO-matrix output, driven low,
     * so the proven bit-banged init owns the bus again. */
    for (size_t lane = 0; lane < LANE_COUNT; lane++) {
        esp_rom_gpio_connect_out_signal(k_lane_gpio[lane],
                                        SIG_GPIO_OUT_IDX, false, false);
        gpio_set_level(k_lane_gpio[lane], 0);
    }
}

static bool transport_try_start()
{
    route_lcd_gpio();

    LCD_CAM.lcd_user.lcd_reset = 1;
    LCD_CAM.lcd_misc.lcd_afifo_reset = 1;
    esp_rom_delay_us(10);

    /* Start at the FRONT ring's head: its first words are VSYNC, and the
     * HX chain state matches (freshly cleared on reconfig, cold on boot).
     * Ring contents (the current frame) are untouched. */
    ESP_ERROR_CHECK(gdma_start(
        s_dma_chan, reinterpret_cast<intptr_t>(&s_desc[s_front][0])));

    /* Let GDMA prefill the FIFO from PSRAM before the LCD starts. */
    esp_rom_delay_us(200);

    LCD_CAM.lcd_user.lcd_update = 1;
    LCD_CAM.lcd_user.lcd_start = 1;

    /* Verify: allow four upload periods for the first EOF. */
    const uint32_t baseline =
        __atomic_load_n(&s_eof_count, __ATOMIC_RELAXED);
    const int64_t deadline = esp_timer_get_time() + 60000;
    while (esp_timer_get_time() < deadline) {
        if (__atomic_load_n(&s_eof_count, __ATOMIC_RELAXED) != baseline) {
            return true;
        }
        vTaskDelay(1);
    }
    return false;
}

static void transport_start()
{
    for (int attempt = 1; attempt <= 5; attempt++) {
        if (transport_try_start()) {
            if (attempt > 1) {
                ESP_LOGW(TAG, "transport started on attempt %d", attempt);
            }
            return;
        }
        ESP_LOGW(TAG, "start attempt %d: no EOF within 60 ms; full "
                      "protocol reset", attempt);

        /* A failed attempt may have emitted a partial ring: HX could sit
         * at a non-multiple-of-20 position and the ICND data port may
         * hold a partial packet. Restore the exact known state a sweep
         * transition uses, not just peripheral resets. */
        transport_stop();
        hx_clear_chain_bitbang();
        icnd_start_memory_protocol_bitbang();
        vTaskDelay(pdMS_TO_TICKS(2));
    }

    ESP_LOGE(TAG, "transport failed to start after 5 attempts");
    abort();
}

static void initialize_gamma(); /* defined below; drive lives in the table */

#if GHOST_TUNE_MODE


static void apply_sweep_step(const SweepStep &step)
{
    xSemaphoreTake(s_present_mutex, portMAX_DELAY);

    transport_stop();

    s_icnd_cfg.reg2 = step.reg2;
    s_icnd_cfg.reg3 = step.reg3;
    s_icnd_cfg.reg4 = step.reg4;
    s_hx_phase = step.hx_phase;
    s_row_oe_mode = step.b_mode;
    s_max_drive = step.drive;
    s_oe_settle = step.settle;

    /* Drive lives in the gamma table, which the ring builder reads. */
    initialize_gamma();

    /* Timing/B state lives in the waveform: rebuild both rings from the
     * canvas and make both shadows the canvas. Packet offsets are
     * invariant (slot word budget fixed), so the patch table stays
     * valid. */
    build_ring(s_ring[0], s_canvas);
    build_ring(s_ring[1], s_canvas);
    ring_writeback_full(0);
    ring_writeback_full(1);
    memcpy(s_shadow[0], s_canvas, CANVAS_BYTES);
    memcpy(s_shadow[1], s_canvas, CANVAS_BYTES);

    hx_clear_chain_bitbang();
    icnd_start_memory_protocol_bitbang();
    transport_start();

    xSemaphoreGive(s_present_mutex);
}
#endif /* GHOST_TUNE_MODE */

/* -------------------------------------------------------------------------- */
/* PRESENT — patch back ring, write back cache, retarget one pointer          */
/* -------------------------------------------------------------------------- */

static uint8_t back_ring()
{
    return s_front ^ 1U;
}

static void flip_locked(uint8_t back)
{
    const uint8_t front = s_front;

    /* Drain a stale give before arming. With abort-on-timeout below a
     * stale give should be impossible; the library's softer recovery
     * path (resolve ring from hardware instead of aborting) makes this
     * drain mandatory, so establish the pattern here. */
    xSemaphoreTake(s_flip_done, 0);

    /* Back ring must self-loop before we route traffic into it. */
    s_desc[back][DESC_PER_RING - 1].next = &s_desc[back][0];

    __atomic_store_n(&s_flip_watch_addr,
                     reinterpret_cast<uint32_t>(
                         &s_desc[back][DESC_PER_RING - 1]),
                     __ATOMIC_RELAXED);
    __atomic_store_n(&s_flip_pending, true, __ATOMIC_RELEASE);

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
     * descriptor range?). */
    if (xSemaphoreTake(s_flip_done, pdMS_TO_TICKS(100)) != pdTRUE) {
        ESP_LOGE(TAG, "FATAL flip timeout: ring ownership unknown "
                      "(EOF lost or DMA halted)");
        abort();
    }

    s_front = back;
}

/*
 * Present the canvas.
 *
 * Diffs the canvas against the back ring's shadow, patches only changed
 * pixels, writes back only the touched cache lines, flips at the upload
 * boundary, then records the canvas as that ring's new shadow. If
 * nothing changed, no flip occurs.
 */
static void present()
{
    xSemaphoreTake(s_present_mutex, portMAX_DELAY);

    const int64_t t0 = esp_timer_get_time();
    const uint8_t back = back_ring();
    const uint8_t *shadow = s_shadow[back];

    memset(s_line_dirty, 0, sizeof(s_line_dirty));

    uint32_t changed = 0;
    for (uint16_t y = 0; y < PANEL_HEIGHT; y++) {
        for (uint16_t x = 0; x < PANEL_WIDTH; x++) {
            const size_t i =
                ((static_cast<size_t>(y) * PANEL_WIDTH) + x) * 3U;
            if (s_canvas[i] != shadow[i] ||
                s_canvas[i + 1] != shadow[i + 1] ||
                s_canvas[i + 2] != shadow[i + 2]) {
                ring_set_pixel(back, x, y, s_canvas[i],
                               s_canvas[i + 1], s_canvas[i + 2]);
                changed++;
            }
        }
    }

    if (changed == 0) {
        xSemaphoreGive(s_present_mutex);
        return;
    }

    uint32_t lines = 0;
    uint32_t bytes = 0;
    msync_dirty_lines(back, &lines, &bytes);

    const int64_t t1 = esp_timer_get_time();

    flip_locked(back);
    memcpy(s_shadow[back], s_canvas, CANVAS_BYTES);

    s_present_px = changed;
    s_present_lines = lines;
    s_present_sync_bytes = bytes;
    s_present_work_us = static_cast<uint32_t>(t1 - t0);

    xSemaphoreGive(s_present_mutex);
}

/*
 * Change max drive live. Drive lives in the gamma table, which the ring
 * builder reads, so both rings must be rebuilt - but the transport keeps
 * running: rebuild the BACK ring, flip to it, then rebuild the other.
 * Never rewrite the ring DMA is currently reading.
 */
static void apply_drive(float drive)
{
    xSemaphoreTake(s_present_mutex, portMAX_DELAY);

    s_max_drive = drive;
    initialize_gamma();

    uint8_t back = back_ring();
    build_ring(s_ring[back], s_canvas);
    ring_writeback_full(back);
    memcpy(s_shadow[back], s_canvas, CANVAS_BYTES);
    flip_locked(back);

    back = back_ring();
    build_ring(s_ring[back], s_canvas);
    ring_writeback_full(back);
    memcpy(s_shadow[back], s_canvas, CANVAS_BYTES);

    xSemaphoreGive(s_present_mutex);
}

/* -------------------------------------------------------------------------- */
/* TEST PATTERNS (drawn on the canonical canvas; present() does the rest)     */
/* -------------------------------------------------------------------------- */

/* Full scale: the production config must be clean at the brightest
 * value the application can ask for, not just at a dim test level. */
static constexpr uint8_t TEST_LEVEL = 255;

static inline void canvas_set_pixel(uint16_t x, uint16_t y,
                                    uint8_t r, uint8_t g, uint8_t b)
{
    if (x >= PANEL_WIDTH || y >= PANEL_HEIGHT) {
        return;
    }
    uint8_t *p =
        &s_canvas[((static_cast<size_t>(y) * PANEL_WIDTH) + x) * 3U];
    p[0] = r;
    p[1] = g;
    p[2] = b;
}

static void draw_solid(uint8_t r, uint8_t g, uint8_t b)
{
    for (uint16_t y = 0; y < PANEL_HEIGHT; y++)
        for (uint16_t x = 0; x < PANEL_WIDTH; x++)
            canvas_set_pixel(x, y, r, g, b);
}

#if !GHOST_TUNE_MODE
static void draw_bands()
{
    for (uint16_t y = 0; y < PANEL_HEIGHT; y++) {
        for (uint16_t x = 0; x < PANEL_WIDTH; x++) {
            switch (x / 16U) {
                case 0:  canvas_set_pixel(x, y, TEST_LEVEL, 0, 0); break;
                case 1:  canvas_set_pixel(x, y, 0, TEST_LEVEL, 0); break;
                case 2:  canvas_set_pixel(x, y, 0, 0, TEST_LEVEL); break;
                case 3:  canvas_set_pixel(x, y, TEST_LEVEL, TEST_LEVEL, 0); break;
                default: canvas_set_pixel(x, y, TEST_LEVEL, 0, TEST_LEVEL); break;
            }
        }
    }
}

/*
 * Horizontal 0..255 ramp, one colour band per 10 rows (R/G/B/white).
 * This is the ONLY test content where GAMMA_EXPONENT is visible: gamma
 * is an identity at both endpoints (0 -> 0, 255 -> 65535*drive), so any
 * pattern built from 0 and 255 looks the same at every exponent. Low
 * exponents brighten the midtones and crush the top; 2.2 spreads the
 * dark end out, which is also where this chip's low-gray consistency
 * can be judged.
 */
static void draw_gray_ramp()
{
    for (uint16_t y = 0; y < PANEL_HEIGHT; y++) {
        for (uint16_t x = 0; x < PANEL_WIDTH; x++) {
            const uint8_t v = static_cast<uint8_t>(
                (static_cast<uint32_t>(x) * 255U) / (PANEL_WIDTH - 1));
            switch (y / 10U) {
                case 0:  canvas_set_pixel(x, y, v, 0, 0); break;
                case 1:  canvas_set_pixel(x, y, 0, v, 0); break;
                case 2:  canvas_set_pixel(x, y, 0, 0, v); break;
                default: canvas_set_pixel(x, y, v, v, v); break;
            }
        }
    }
}

static void draw_boundaries()
{
    draw_solid(0, 0, 0);

    static const uint16_t xs[] = {0, 15, 16, 31, 32, 47, 48, 63, 64, 79};
    static const uint16_t ys[] = {0, 19, 20, 39};

    for (uint16_t x : xs)
        for (uint16_t y = 0; y < PANEL_HEIGHT; y++)
            canvas_set_pixel(x, y, TEST_LEVEL, TEST_LEVEL, TEST_LEVEL);
    for (uint16_t y : ys)
        for (uint16_t x = 0; x < PANEL_WIDTH; x++)
            canvas_set_pixel(x, y, TEST_LEVEL, TEST_LEVEL, TEST_LEVEL);
}

/*
 * Sparse-update stress: a 2x2 dot bouncing at 40 Hz for ~8 seconds.
 * Each present changes at most 8 pixels, so telemetry should report
 * px=8, a handful of synced lines, and double-digit work_us. This is
 * the path a clock's seconds digit will exercise — and the phase that
 * would have exposed the stale-back-ring bug had the shadows not fixed
 * it: without them, the dot would leave a trail of two-frame-old state.
 */
static void run_sparse_dot_phase()
{
    struct DotColour {
        uint8_t r, g, b;
        const char *name;
    };
    /* All seven saturated combinations: cyan alone never isolates red,
     * and red behaved differently from green/blue in every earlier lap. */
    static const DotColour k_colours[] = {
        {255,   0,   0, "RED"},
        {  0, 255,   0, "GREEN"},
        {  0,   0, 255, "BLUE"},
        {  0, 255, 255, "CYAN"},
        {255,   0, 255, "MAGENTA"},
        {255, 255,   0, "YELLOW"},
        {255, 255, 255, "WHITE"},
    };
    static const float k_drives[] = {0.85f, 1.00f};

    int16_t px = 3, py = 5, vx = 1, vy = 1;

    for (float drive : k_drives) {
        draw_solid(0, 0, 0);
        present();
        apply_drive(drive);
        ESP_LOGW(TAG, "  == dot phase at max_drive %.2f (255 -> %u) ==",
                 static_cast<double>(drive),
                 static_cast<unsigned>(s_gamma16[255]));

        for (const DotColour &c : k_colours) {
            ESP_LOGI(TAG, "     dot: %s", c.name);

            for (uint16_t step = 0; step < 160; step++) {
                canvas_set_pixel(px, py, 0, 0, 0);
                canvas_set_pixel(px + 1, py, 0, 0, 0);
                canvas_set_pixel(px, py + 1, 0, 0, 0);
                canvas_set_pixel(px + 1, py + 1, 0, 0, 0);

                px += vx;
                py += vy;
                if (px <= 0 || px >= PANEL_WIDTH - 2) vx = -vx;
                if (py <= 0 || py >= PANEL_HEIGHT - 2) vy = -vy;

                canvas_set_pixel(px, py, c.r, c.g, c.b);
                canvas_set_pixel(px + 1, py, c.r, c.g, c.b);
                canvas_set_pixel(px, py + 1, c.r, c.g, c.b);
                canvas_set_pixel(px + 1, py + 1, c.r, c.g, c.b);

                present();
                vTaskDelay(pdMS_TO_TICKS(25));
            }
        }
    }

    /* Leave the panel at the configured production drive. */
    apply_drive(0.85f);
}

#endif /* !GHOST_TUNE_MODE */

#if GHOST_TUNE_MODE
static void pattern_task(void *)
{
    vTaskDelay(pdMS_TO_TICKS(200));

    static const struct {
        uint8_t r, g, b;
        const char *name;
    } k_dot_colors[] = {
        {TEST_LEVEL, 0, 0, "RED"},
        {0, TEST_LEVEL, 0, "GREEN"},
        {0, 0, TEST_LEVEL, "BLUE"},
        {0, TEST_LEVEL, TEST_LEVEL, "CYAN"},
    };

    draw_solid(0, 0, 0);
    present();

#if GHOST_PROBE_MODE
    /*
     * DRIVE-THRESHOLD HARNESS.
     *
     * A static FULL-SCALE (255) 2x2 dot, held while the sweep walks
     * drive, settling blank, and current gain. Watch only one thing:
     * does the full-half vertical bar appear at the dot's column?
     *
     *   A: the first drive level that ghosts is the panel's ceiling.
     *   B: if a settle value clears the ghost at drive 1.00, the tail-
     *      vs-blank-window model is right and settle buys headroom.
     *   C: if lower gain clears it instead, the ghost tracks current
     *      magnitude and REG2 is the better global brightness axis.
     *   REF: the original always-enabled policy, for calibration.
     *
     * Only 4 pixels are lit, so full drive here is thermally harmless;
     * do NOT run a full-screen white at drive 1.00 without checking
     * supply current first.
     */
    canvas_set_pixel(PROBE_DOT_X, PROBE_DOT_Y, 255, 255, 255);
    canvas_set_pixel(PROBE_DOT_X + 1, PROBE_DOT_Y, 255, 255, 255);
    canvas_set_pixel(PROBE_DOT_X, PROBE_DOT_Y + 1, 255, 255, 255);
    canvas_set_pixel(PROBE_DOT_X + 1, PROBE_DOT_Y + 1, 255, 255, 255);

    while (true) {
        for (size_t i = 0; i < SWEEP_COUNT; i++) {
            apply_sweep_step(k_sweep[i]);
            present();
            ESP_LOGW(TAG,
                     "==== %u/%u: %s (drive=%.2f REG2=0x%04X "
                     "REG3=0x%04X REG4=0x%04X) -> 255 maps to %u ====",
                     static_cast<unsigned>(i + 1),
                     static_cast<unsigned>(SWEEP_COUNT), k_sweep[i].label,
                     static_cast<double>(k_sweep[i].drive),
                     k_sweep[i].reg2, k_sweep[i].reg3, k_sweep[i].reg4,
                     static_cast<unsigned>(s_gamma16[255]));
            vTaskDelay(pdMS_TO_TICKS(12000));
        }
    }
#else
    int16_t px = 3, py = 5, vx = 1, vy = 1;

    while (true) {
        for (size_t cfg = 0; cfg < SWEEP_COUNT; cfg++) {
            apply_sweep_step(k_sweep[cfg]);
            ESP_LOGW(TAG,
                     "==== SWEEP %u/%u: %s (REG2=0x%04X REG3=0x%04X "
                     "hx=%u B=%u) ====",
                     static_cast<unsigned>(cfg + 1),
                     static_cast<unsigned>(SWEEP_COUNT),
                     k_sweep[cfg].label, k_sweep[cfg].reg2,
                     k_sweep[cfg].reg3, k_sweep[cfg].hx_phase,
                     k_sweep[cfg].b_mode);

            for (size_t color = 0; color < 4; color++) {
                ESP_LOGI(TAG, "  dot: %s", k_dot_colors[color].name);

                for (uint16_t step = 0; step < 120; step++) {
                    canvas_set_pixel(px, py, 0, 0, 0);
                    canvas_set_pixel(px + 1, py, 0, 0, 0);
                    canvas_set_pixel(px, py + 1, 0, 0, 0);
                    canvas_set_pixel(px + 1, py + 1, 0, 0, 0);

                    px += vx;
                    py += vy;
                    if (px <= 0 || px >= PANEL_WIDTH - 2) vx = -vx;
                    if (py <= 0 || py >= PANEL_HEIGHT - 2) vy = -vy;

                    const uint8_t r = k_dot_colors[color].r;
                    const uint8_t g = k_dot_colors[color].g;
                    const uint8_t b = k_dot_colors[color].b;
                    canvas_set_pixel(px, py, r, g, b);
                    canvas_set_pixel(px + 1, py, r, g, b);
                    canvas_set_pixel(px, py + 1, r, g, b);
                    canvas_set_pixel(px + 1, py + 1, r, g, b);

                    present();
                    vTaskDelay(pdMS_TO_TICKS(25));
                }
            }
        }
    }
#endif /* GHOST_PROBE_MODE */
}
#else /* !GHOST_TUNE_MODE */
static void pattern_task(void *)
{
    /* Let the black rings loop a few uploads so both ICND memory banks are
     * deterministic before the first visible frame. */
    vTaskDelay(pdMS_TO_TICKS(200));

    while (true) {
        ESP_LOGI(TAG, "Pattern: RED");
        draw_solid(TEST_LEVEL, 0, 0);
        present();
        vTaskDelay(pdMS_TO_TICKS(4000));

        ESP_LOGI(TAG, "Pattern: GREEN");
        draw_solid(0, TEST_LEVEL, 0);
        present();
        vTaskDelay(pdMS_TO_TICKS(4000));

        ESP_LOGI(TAG, "Pattern: BLUE");
        draw_solid(0, 0, TEST_LEVEL);
        present();
        vTaskDelay(pdMS_TO_TICKS(4000));

        ESP_LOGI(TAG, "Pattern: WHITE");
        draw_solid(TEST_LEVEL, TEST_LEVEL, TEST_LEVEL);
        present();
        vTaskDelay(pdMS_TO_TICKS(4000));

        ESP_LOGI(TAG, "Pattern: five 16-column sections");
        draw_bands();
        present();
        vTaskDelay(pdMS_TO_TICKS(6000));

        ESP_LOGI(TAG, "Pattern: boundaries");
        draw_boundaries();
        present();
        vTaskDelay(pdMS_TO_TICKS(6000));

        ESP_LOGI(TAG, "Pattern: grayscale ramp (gamma is visible here)");
        draw_gray_ramp();
        present();
        vTaskDelay(pdMS_TO_TICKS(8000));

        ESP_LOGI(TAG, "Pattern: sparse bouncing dot (partial updates)");
        run_sparse_dot_phase();
    }
}
#endif /* GHOST_TUNE_MODE */

static void telemetry_task(void *)
{
    /* Snapshot, don't start at zero: EOFs accumulate from lcd_start()
     * onward, before this task exists. Counting them in window 1 produced
     * the 71.81 reading (359 EOFs / 5 s instead of ~352.5). */
    uint32_t last_eof = __atomic_load_n(&s_eof_count, __ATOMIC_RELAXED);
    int64_t last_us = esp_timer_get_time();

    while (true) {
        vTaskDelay(pdMS_TO_TICKS(5000));

        const uint32_t eof =
            __atomic_load_n(&s_eof_count, __ATOMIC_RELAXED);
        const uint32_t flips =
            __atomic_load_n(&s_flip_count, __ATOMIC_RELAXED);
        const int64_t now = esp_timer_get_time();
        const float secs = static_cast<float>(now - last_us) / 1e6f;

        ESP_LOGI(TAG,
                 "uploads/s=%.2f (expect ~70.5 @10MHz), flips=%u, "
                 "front=ring%u, REG2=0x%04X, REG3=0x%04X, drive=%.2f, OE=%u, "
                 "internal free=%u | last present: px=%u lines=%u "
                 "sync=%uB work=%uus",
                 static_cast<float>(eof - last_eof) / secs,
                 static_cast<unsigned>(flips),
                 static_cast<unsigned>(s_front),
                 s_icnd_cfg.reg2,
                 s_icnd_cfg.reg3,
                 static_cast<double>(s_max_drive),
                 static_cast<unsigned>(s_row_oe_mode),
                 static_cast<unsigned>(heap_caps_get_free_size(
                     MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT)),
                 static_cast<unsigned>(s_present_px),
                 static_cast<unsigned>(s_present_lines),
                 static_cast<unsigned>(s_present_sync_bytes),
                 static_cast<unsigned>(s_present_work_us));

        last_eof = eof;
        last_us = now;
    }
}

/* -------------------------------------------------------------------------- */
/* SETUP HELPERS                                                              */
/* -------------------------------------------------------------------------- */

static void initialize_gamma()
{
    const float ceiling = 65535.0f * s_max_drive;

    for (uint32_t i = 0; i < 256; i++) {
        const float n = static_cast<float>(i) / 255.0f;
        const float c = (GAMMA_EXPONENT == 1.0f)
                            ? n
                            : __builtin_powf(n, GAMMA_EXPONENT);
        s_gamma16[i] = static_cast<uint16_t>(c * ceiling + 0.5f);
    }

    ESP_LOGI(TAG, "gamma=%.2f max_drive=%.2f -> level 255 = %u/65535",
             static_cast<double>(GAMMA_EXPONENT),
             static_cast<double>(s_max_drive),
             static_cast<unsigned>(s_gamma16[255]));
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

    /* Full writeback of both rings before DMA ever reads them. Shadows
     * are zero-initialized BSS = black, matching the black rings. */
    ring_writeback_full(0);
    ring_writeback_full(1);

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
    transport_start();

    vTaskDelay(pdMS_TO_TICKS(100));
    ESP_LOGI(TAG, "streaming: eof_count=%u (should be increasing)",
             static_cast<unsigned>(
                 __atomic_load_n(&s_eof_count, __ATOMIC_RELAXED)));

    s_present_mutex = xSemaphoreCreateMutex();
    ESP_ERROR_CHECK(s_present_mutex != nullptr ? ESP_OK : ESP_ERR_NO_MEM);

    BaseType_t ok = xTaskCreate(pattern_task, "panel_patterns", 4096,
                                nullptr, 5, nullptr);
    ESP_ERROR_CHECK(ok == pdPASS ? ESP_OK : ESP_ERR_NO_MEM);

    ok = xTaskCreate(telemetry_task, "panel_telemetry", 4096,
                     nullptr, 3, nullptr);
    ESP_ERROR_CHECK(ok == pdPASS ? ESP_OK : ESP_ERR_NO_MEM);
}