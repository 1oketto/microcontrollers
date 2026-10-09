/**
 * Hunter Adams (vha3@cornell.edu)
 *
 * This demonstration drops multiple boids through a 16-row Galton board.
 * The rotary encoder controls the number of animated boids.
 *
 * HARDWARE CONNECTIONS
  - GPIO 16 ---> VGA Hsync
  - GPIO 17 ---> VGA Vsync
  - GPIO 18 ---> VGA Green lo-bit --> 470 ohm resistor --> VGA_Green
  - GPIO 19 ---> VGA Green hi_bit --> 330 ohm resistor --> VGA_Green
  - GPIO 20 ---> 330 ohm resistor ---> VGA-Blue
  - GPIO 21 ---> 330 ohm resistor ---> VGA-Red
  - RP2040 GND ---> VGA-GND
 *
 * RESOURCES USED
 *  - PIO state machines 0, 1, and 2 on PIO instance 0
 *  - DMA channels (2, by claim mechanism)
 *  - 153.6 kBytes of RAM (for pixel color data)
 *
 */

// Include the VGA grahics library
#include "VGA/vga16_graphics_v3.h"
// Include standard libraries
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <limits.h>
// Include Pico libraries
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "pico/sync.h"
// Include hardware libraries
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/spi.h"
#include "hardware/clocks.h"
#include "hardware/pll.h"
#include "hardware/gpio.h"
#include "hardware/sync.h"
#include "hardware/vreg.h"
// Include protothreads
#include "pt_cornell_rp2040_v1_4.h"

// Number of samples per period in sine table
#define sine_table_size 256

// Table of values to be sent to DAC
unsigned short DAC_data[sine_table_size];

// Pointer to the address of the DAC data table
unsigned short *address_pointer = &DAC_data[0];

// A-channel, 1x, active
#define DAC_config_chan_A 0b0011000000000000

// SPI configurations (don't gaf about MISO)
#define PIN_CS 5
#define PIN_SCK 6
#define PIN_MOSI 3
#define SPI_PORT spi0

#define PEG_ROWS 16
// Histogram parameters and variables
#define HISTOGRAM_BINS (PEG_ROWS - 1) // one bin for each gap between adjacent pegs in the bottom row

// shared variables
typedef struct {
    uint32_t peg_sound_events;
    uint32_t total_fallen;
    uint32_t histogram[HISTOGRAM_BINS];
    uint32_t rng;                      // per-core generator state
} core_stats;
static core_stats stats0 = {.rng = 1};
static core_stats stats1 = {.rng = 2};
static uint32_t spawn_rng = 3;

static uint32_t nextRandom(uint32_t *state)
{
    uint32_t v = *state;
    v ^= v << 13; v ^= v >> 17; v ^= v << 5;
    return *state = v;
}

// Number of DMA transfers per event
const uint32_t transfer_count = sine_table_size;

static int data_chan;
static int ctrl_chan;
// boid collisions produce events; the sound thread consumes them.

// 60 fps gives each animation frame 16,666 microseconds.
#define FRAME_BUDGET_US (1000000u / 60u)
#define DEADLINE_LED_PIN PICO_DEFAULT_LED_PIN

// played_peg_events = how many collisions have happened
// ^^ together they keep track of whether sound has been played for a collision
// and collisions aren't lost while DMA is busy

// rotary encoder GPIOs (C_PIN is connected to GND pin 18)
#define A_PIN 13
#define B_PIN 14
#define BOUNCE_PIN_OUT 12
#define BOUNCE_PIN_IN 15

static volatile int rotary_count = 0; // Requested active boids; never negative.

//// NEW : USE BIT SHIFTS INSTEAD OF FLOATING PT DIVISION OR MULTIPLICATION
// Bounciness levels: 0 (Off), 1 (1/16th), 2 (1/8th), 3 (1/4th), 4 (1/2), 5 (Full/1)
#define BOUNCE_LEVEL_MAX 5
#define BOUNCE_OFF_SHIFT 31u // Special shift amount to completely absorb velocity

static volatile int bounciness_level = 3; // Default to 1/4th bounce

// Convert a level (0-5) into a bit-shift count (or off state)
static inline unsigned getBouncinessShift(int level) {
    if (level <= 0) return BOUNCE_OFF_SHIFT;
    return (unsigned)(BOUNCE_LEVEL_MAX - level); // e.g., level 4 -> shift by 1 (divide by 2)
}
////

#define NUM_BALL_MODE 0
#define BOUNCY_MODE 1
volatile int rotary_mode = NUM_BALL_MODE;

static uint32_t histogram[HISTOGRAM_BINS] = {0};
static uint64_t total_fallen = 0;
static uint32_t peg_sound_events = 0; // NOTE: reset each frame

static int frame_boid_count = 0;
static int count_core0 = 0;

#define FLAG_VALUE 5

// GPIO ISR: CW is A falls while B is high

static volatile bool stats_reset_pending = false;
void gpio_core0_callback(uint gpio, uint32_t events)
{
    if (gpio != A_PIN || !(events & GPIO_IRQ_EDGE_FALL)) return;
    stats_reset_pending = true;

    if (rotary_mode == NUM_BALL_MODE) {
        if (gpio_get(B_PIN)) {
            if (rotary_count < MAX_BALLS)
                ++rotary_count;
        } else if (rotary_count > 0) {
            --rotary_count;
        }
    }
    else {  // BOUNCY_MODE
        if (gpio_get(B_PIN)) {
            if (bounciness_level < BOUNCE_LEVEL_MAX)
                ++bounciness_level;
        } else if (bounciness_level > 0) {
            --bounciness_level;
        }
    }
}

// GPIO ISR: If input pin gets pulled low (normally to VDD w PU resistor), means button pressed (shorted to low output pin)
static volatile uint64_t last_button_time = 0;

void gpio_core1_callback(uint gpio, uint32_t events)
{
    if (gpio != BOUNCE_PIN_IN || !(events & GPIO_IRQ_EDGE_FALL)) return;
    stats_reset_pending = true;

    uint64_t now = time_us_64();

    if (now - last_button_time > 20000) {  // 20 ms debounce
        rotary_mode = (rotary_mode + 1) % 2;
        last_button_time = now;
    }
}

// play a DMA sound for each boid-peg collision
static PT_THREAD(protothread_sound(struct pt *pt))
{
    static uint32_t played_peg_events = 0; // how many collision sounds we've already played
    PT_BEGIN(pt);
    while (1)
    {
        if (played_peg_events != peg_sound_events &&
            !dma_channel_is_busy(ctrl_chan) &&
            !dma_channel_is_busy(data_chan) && !spi_is_busy(SPI_PORT))
        {
            // the control channel restores the table address, then starts data DMA
            dma_channel_set_trans_count(data_chan, transfer_count, false); // rearm data DMA
            dma_channel_set_trans_count(ctrl_chan, 1, false); // rearm control DMA
            dma_start_channel_mask(1u << ctrl_chan); // start control DMA
            ++played_peg_events;
        }
        PT_YIELD_usec(1000);
    }
    PT_END(pt);
}

// ========================================
// === boidS AND PEGS !!!!
// ========================================
static const char color = WHITE;

typedef signed int fix15;
#define multfix15(a, b) ((fix15)(((int64_t)(a) * (int64_t)(b)) / 32768)) // symmetric rounding toward zero
#define float2fix15(a) ((fix15)((a) * 32768.0)) // 2^15
#define fix2float15(a) ((float)(a) / 32768.0)
#define absfix15(a) abs(a)
#define int2fix15(a) ((fix15)((a) * 32768))
#define fix2int15(a) ((int)(a >> 15))
#define char2fix15(a) (fix15)(((fix15)(a)) << 15)
#define divfix(a, b) ((fix15)(((int64_t)(a) * 32768) / (int64_t)(b)))

// global variables for boids and pegs. positions are pixels; velocities are pixels/frame.
#define boid_RADIUS 1
#define PEG_RADIUS 6
// PEG_ROWS defined earlier
// #define PEG_ROWS 16
#define PEG_COUNT (PEG_ROWS * (PEG_ROWS + 1) / 2)
// Center-to-center spacing between pegs and rows.
#define PEG_HORIZONTAL_SPACING 38
#define PEG_VERTICAL_SPACING 19
#define PEG_X int2fix15(320)
#define PEG_Y int2fix15(60)
#define GRAVITY ((fix15)12124) // 0.37 in Q15
#define BOUNCINESS ((fix15)12124) // 0.37 in Q15

static fix15 peg_x[PEG_COUNT], peg_y[PEG_COUNT];

// NEW: PEG OPTIMIZATION
static const uint8_t peg_left_child[120] = {
     1,  3,  4,  6,  7,  8, 10, 11,
    12, 13, 15, 16, 17, 18, 20, 21,
    22, 23, 24, 26, 27, 28, 29, 30,
    31, 33, 34, 35, 36, 37, 38, 39,
    40, 42, 43, 44, 45, 46, 47, 48,
    49, 50, 52, 53, 54, 55, 56, 57,
    58, 59, 60, 62, 63, 64, 65, 66,
    67, 68, 69, 70, 71, 72, 74, 75,
    76, 77, 78, 79, 80, 81, 82, 83,
    84, 85, 86, 88, 89, 90, 91, 92,
    93, 94, 95, 96, 97, 98, 99, 100,
   101, 102, 103, 104, 105, 106, 107, 108,
   110, 111, 112, 113, 114, 115, 116, 117,
   118, 119, 120, 121, 122, 123, 124, 125,
    126, 127, 128, 129, 130, 131, 132, 133,
    134
};

static const uint8_t peg_right_child[120] = {
     2,  4,  5,  7,  8,  9, 11, 12,
    13, 14, 16, 17, 18, 19, 21, 22,
    23, 24, 25, 27, 28, 29, 30, 31,
    32, 34, 35, 36, 37, 38, 39, 40,
    41, 43, 44, 45, 46, 47, 48, 49,
    50, 51, 53, 54, 55, 56, 57, 58,
    59, 60, 61, 63, 64, 65, 66, 67,
    68, 69, 70, 71, 72, 73, 75, 76,
    77, 78, 79, 80, 81, 82, 83, 84,
    85, 86, 87, 89, 90, 91, 92, 93,
    94, 95, 96, 97, 98, 99, 100, 101,
   102, 103, 104, 105, 106, 107, 108, 109,
   111, 112, 113, 114, 115, 116, 117, 118,
   119, 120, 121, 122, 123, 124, 125, 126,
   127, 128, 129, 130, 131, 132, 133, 134,
   135
};

static void initPegs(void)
{
    int peg = 0;
    for (int row = 0; row < PEG_ROWS; ++row) {
        for (int col = 0; col <= row; ++col) {
            peg_x[peg] = PEG_X + (2 * col - row) * int2fix15(PEG_HORIZONTAL_SPACING) / 2;
            peg_y[peg] = PEG_Y + int2fix15(row * PEG_VERTICAL_SPACING);
            ++peg;
        }
    }
}

static void drawPegs(void)
{
    for (int peg = 0; peg < PEG_COUNT; ++peg)
        drawCircle((short)fix2int15(peg_x[peg]), (short)fix2int15(peg_y[peg]), PEG_RADIUS, WHITE);
}

// boid
// CHANGED: float --> int16_t; drops the struct size from 24 bytes down to 10 bytes
typedef struct {
    int16_t x, y;     // Packed pixel coordinates
    int16_t vx, vy;   // Packed velocity vectors
    uint8_t last_peg;
    bool histogram_recorded;
} boid;

static inline int16_t packFixed(fix15 value, unsigned shift) // HELPS WITH CONVERTING TO int16_t FORMAT
{
    int32_t packed = (value + (1 << (shift - 1))) >> shift;
    if (packed > INT16_MAX) packed = INT16_MAX;
    if (packed < INT16_MIN) packed = INT16_MIN;
    return (int16_t)packed;
}

//_Static_assert(sizeof(boid) * MAX_BALLS < 100 * 1024, "ball pool too big");
#define MAX_BALLS 20000
static boid boids[MAX_BALLS];
static int boid_count = 0;
// static int boid_capacity = 0;

static void dropboid(boid *b, uint32_t *rng)
{
    b->x = packFixed(PEG_X, 10);
    b->y = boid_RADIUS * 32;
    fix15 vx = (fix15)(328 + nextRandom(rng) % 3605);
    if (nextRandom(rng) & 1u) vx = -vx;
    b->vx = packFixed(vx, 5);
    b->vy = 0;
    b->last_peg = -1;
    b->histogram_recorded = false;
}

// The fixed pool removes reallocations, heap fragmentation and worker hazards.
static void syncboidCount(void)
{
    int requested = rotary_count;
    if (requested > MAX_BALLS) requested = MAX_BALLS;
    if (requested < 0) requested = 0;
    for (int i = boid_count; i < requested; ++i)
        dropboid(&boids[i], &spawn_rng);
    boid_count = requested;
}

// // Integer square root of a Q30 squared distance returns a Q15 distance.
// static fix15 distanceFix15(fix15 dx, fix15 dy)
// {
//     uint64_t remainder = (uint64_t)((int64_t)dx * dx) +
//                          (uint64_t)((int64_t)dy * dy);
//     uint64_t root = 0;
//     uint64_t bit = (uint64_t)1 << 62;
//     while (bit > remainder) bit >>= 2;
//     while (bit != 0) {
//         if (remainder >= root + bit) {
//             remainder -= root + bit;
//             root = (root >> 1) + bit;
//         } else {
//             root >>= 1;
//         }
//         bit >>= 2;
//     }
//     return (fix15)root;
// }

// NEW: fast inverse square root (trying in place of 1/sqrtf)
static inline float fastInvSqrt(float x)
{
    float y = x;
    uint32_t i;
    memcpy(&i, &y, sizeof i);
    i = 0x5f3759df - (i >> 1);
    memcpy(&y, &i, sizeof y);
    return y * (1.5f - 0.5f * x * y * y);   // one Newton step
}

static inline fix15 dampVelocity(fix15 velocity, unsigned shift) {
    if (shift == BOUNCE_OFF_SHIFT) return 0; // Completely stop velocity if bounciness is 0
    return velocity >> shift;                // Fast 1-cycle right bit-shift
}

//PUT COLLISION BODY MATH INTO HELPER FUNCTION
#define FAST_PEG_MAX_BOUNCE 3   // level 3 = 1/4 (25%) bounciness

static inline void collidePeg(boid *b, int peg, core_stats *stats)
{
    fix15 dx = b->x - peg_x[peg];
    fix15 dy = b->y - peg_y[peg];
    fix15 cd = int2fix15(boid_RADIUS + PEG_RADIUS);
    if (absfix15(dx) >= cd || absfix15(dy) >= cd) return;

    // CHANGED: distance check w/o using distanceFix15 and sqrt
    int64_t d2 = (int64_t)dx * dx + (int64_t)dy * dy; // distance^2
    if (d2 >= (int64_t)cd * cd) return;       // exact reject, no sqrt

    fix15 normal_x, normal_y; // NEW: find without divfix
    if (d2 == 0) {
        normal_x = 0;
        normal_y = -int2fix15(1); // coincident centers: push up
    } else {
        float fx = (float)dx, fy = (float)dy;
        float inv = fastInvSqrt(fx * fx + fy * fy); // or just do 1.0f / sqrtf, might be faster?
        normal_x = (fix15)(fx * inv * 32768.0f);
        normal_y = (fix15)(fy * inv * 32768.0f);
    }

    fix15 dot = multfix15(normal_x, b->vx) + multfix15(normal_y, b->vy);
    b->x = peg_x[peg] + multfix15(normal_x, cd + int2fix15(1));
    b->y = peg_y[peg] + multfix15(normal_y, cd + int2fix15(1));
    if (dot < 0) {
        fix15 k = -2 * dot;
        b->vx += multfix15(normal_x, k);
        b->vy += multfix15(normal_y, k);
        if (b->last_peg != peg) {
            ++stats->peg_sound_events;
            // NEW: (1-cycle bit shifts instead of fixed-pt division):
            unsigned bounce_shift = getBouncinessShift(bounciness_level);
            b->vx = dampVelocity(b->vx, bounce_shift);
            b->vy = dampVelocity(b->vy, bounce_shift);

            b->last_peg = peg;
        }
    }
}

static void updateboid(boid *boid, core_stats *stats)
{
    boid->x += boid->vx;
    boid->y += boid->vy;

    if (bounciness_level <= FAST_PEG_MAX_BOUNCE) {
        int lp = boid->last_peg;
        if (lp == -1) {
            collidePeg(boid, 0, stats);
        } else if (lp < 120) {
            int l = peg_left_child[lp], r = peg_right_child[lp];
            collidePeg(boid, lp, stats);
            collidePeg(boid, l, stats);
            collidePeg(boid, r, stats);
        } else {
            collidePeg(boid, lp, stats);
        }

        // ADD ETIHER THIS OR THE ELSE BLOCK
    } else {
        const int R = boid_RADIUS + PEG_RADIUS;
        int dy = fix2int15(boid->y) - fix2int15(PEG_Y);
        int r0 = (dy - R > 0) ? (dy - R) / PEG_VERTICAL_SPACING : 0;
        int r1 = (dy + R < 0) ? -1 : (dy + R) / PEG_VERTICAL_SPACING;
        if (r1 > PEG_ROWS - 1) r1 = PEG_ROWS - 1;
        for (int row = r0; row <= r1; ++row) {
            int start = row * (row + 1) / 2;
            for (int col = 0; col <= row; ++col)
                collidePeg(boid, start + col, stats);
        }
    }

    // Record the gap as soon as the boid clears the bottom row.
    // Waiting until the screen edge lets horizontal drift change the bin.
    const fix15 bottom_row_y = PEG_Y + int2fix15((PEG_ROWS - 1) * PEG_VERTICAL_SPACING);
    if (!boid->histogram_recorded &&
        boid->y > bottom_row_y + int2fix15(PEG_RADIUS + boid_RADIUS))
    {
        const fix15 first_peg_x =
            PEG_X - int2fix15((PEG_ROWS - 1) * PEG_HORIZONTAL_SPACING) / 2;
        // Reject positions left of the board before integer division.
        fix15 offset = boid->x - first_peg_x;
        int bin = offset < 0 ? -1 : offset / int2fix15(PEG_HORIZONTAL_SPACING);
        if (bin >= 0 && bin < HISTOGRAM_BINS)
            ++stats->histogram[bin];
        boid->histogram_recorded = true;
    }

    // Respawn this boid after it falls completely below the screen.
    if (boid->y > int2fix15(480 + boid_RADIUS))
    {
        ++stats->total_fallen;

        dropboid(boid, &spawn_rng);
        return; // reset the new drop's initial y-velocity to zero
    }
    boid->vy += GRAVITY; // add
}

// draw the stats: # boids being animated, total # of boids fallen since reset, and time since boot
static void drawStats(void)
{
    // Static text buffers
    static char count_text[32], fallen_text[48], time_text[32], bounce_text[32];
    static int last_count = -1, last_bounce = -1;
    static uint64_t last_fallen = UINT64_MAX, last_seconds = UINT64_MAX;

    uint64_t seconds = time_us_64() / 1000000u;

    // Only update string buffers when values actually change
    if (boid_count != last_count) {
        snprintf(count_text, sizeof(count_text), "Boids: %d", boid_count);
        last_count = boid_count;
    }
    if (total_fallen != last_fallen) {
        snprintf(fallen_text, sizeof(fallen_text), "Fallen: %llu", (unsigned long long)total_fallen);
        last_fallen = total_fallen;
    }
    if (seconds != last_seconds) {
        snprintf(time_text, sizeof(time_text), "Time: %llu:%02u:%02u",
                 (unsigned long long)(seconds / 3600),
                 (unsigned)((seconds / 60) % 60), (unsigned)(seconds % 60));
        last_seconds = seconds;
    }
    if (bounciness_level != last_bounce) {
        snprintf(bounce_text, sizeof(bounce_text), "Bounciness: %d/%d", bounciness_level, BOUNCE_LEVEL_MAX);
        last_bounce = bounciness_level;
    }

    setTextSize(2);
    setTextColor(WHITE);

    setCursor(10, 5);  writeString(count_text);
    setCursor(10, 22); writeString(fallen_text);
    setCursor(10, 39); writeString(time_text);
    setCursor(10, 56); writeString(bounce_text);
}

// Bars align with the fifteen gaps in the bottom peg row.
static void drawHistogram(void)
{
    const int height = 110;
    int max_count = 0;
    for (int i = 0; i < HISTOGRAM_BINS; ++i)
        if (histogram[i] > max_count) max_count = histogram[i];
    if (max_count == 0) return;
    for (int i = 0; i < HISTOGRAM_BINS; ++i) {
        int bar_height = (int)((int64_t)histogram[i] * height / max_count);
        if (bar_height > 0)
            fillRect(35 + i * PEG_HORIZONTAL_SPACING, 475 - bar_height,
                     PEG_HORIZONTAL_SPACING - 1, bar_height, WHITE);
    }
}

void renderHollowBoidsParallel(unsigned parity) {
    // Separate 256-entry direct-mapped cache for each core/parity
    static uint32_t drawn_centers[2][256];
    uint32_t *cache = drawn_centers[parity & 1u];

    // Reset the cache for this core's frame pass
    memset(cache, 0xFF, sizeof(drawn_centers[0]));

    for (int i = 0; i < frame_boid_count; ++i) {
        short px = (short)fix2int15(boids[i].x);
        short py = (short)fix2int15(boids[i].y);

        if (px >= -boid_RADIUS && px <= 640 + boid_RADIUS &&
            py >= -boid_RADIUS && py <= 480 + boid_RADIUS) {

            // Pack coordinates into a 32-bit key
            uint32_t key = ((uint32_t)(py + boid_RADIUS) << 16) | (uint32_t)(px + boid_RADIUS);
            
            // Hash key into a 0-255 cache slot
            unsigned slot = (key ^ (key >> 8)) & 0xFFu;

            // If a boid was already drawn at this exact coordinate, skip it!
            if (cache[slot] == key) continue;
            
            cache[slot] = key; // Update cache entry

            drawCircleParity(px, py, boid_RADIUS, WHITE, parity);
        }
    }
}

// Animation on core 0
static PT_THREAD(protothread_anim(struct pt *pt))
{
    // LED missed timing variables init
    static uint32_t frame_start_us = 0, elapsed_us = 0;
    static bool missed_deadline = false;

    // Mark beginning of thread
    PT_BEGIN(pt);

    // Start with an empty board; the encoder adds and removes boids.
    initPegs();

    while (1)
    {
        // Wait for the signal that the buffer's changed
        PT_YIELD_UNTIL(pt, draw_start_signal());
        frame_start_us = draw_frame_start_us(); // start counting frame draw time
        syncboidCount();
        frame_boid_count = boid_count;
        count_core0 = frame_boid_count / 2 + frame_boid_count % 2;

        // if Core 1 finishes its physics loop early and pushes its completion flag, Core 0 might miss it if timing gets desynced
        while (multicore_fifo_rvalid()) multicore_fifo_pop_blocking(); // Drain stale tokens
        multicore_fifo_push_blocking(FLAG_VALUE); // Signal Core 1 to begin physics

        for (int i = 0; i < count_core0; ++i) // update boid physics
            updateboid(&boids[i], &stats0);

        PT_YIELD_UNTIL(pt, multicore_fifo_rvalid()); // wait for core 1 to finish
        multicore_fifo_pop_blocking();
        // synchronized now with core 1, safe to draw

        // IF ROTARY ENCODER ISR CALLED: RESET total balls fallen & histogram
        uint32_t irq = save_and_disable_interrupts();
        bool reset = stats_reset_pending;
        stats_reset_pending = false;
        restore_interrupts(irq);
        if (reset) {
            stats0.total_fallen = stats1.total_fallen = 0;
            memset(stats0.histogram, 0, sizeof(stats0.histogram));
            memset(stats1.histogram, 0, sizeof(stats1.histogram));
        }
        
        // MERGE: combine core 0 and core 1 histogram
        for (int bin = 0; bin < HISTOGRAM_BINS; ++bin) {
            histogram[bin] = stats0.histogram[bin] + stats1.histogram[bin];
        }
        peg_sound_events = stats0.peg_sound_events + stats1.peg_sound_events;
        total_fallen = stats0.total_fallen + stats1.total_fallen;

        // Clear the buffer
        clearLowFrame(0, BLACK);
        drawPegs();

        multicore_fifo_push_blocking(FLAG_VALUE); // Signal Core 1 to start drawing scanlines
        // draw even scanlines
        renderHollowBoidsParallel(0); // 0 = even rows
        PT_YIELD_UNTIL(pt, multicore_fifo_rvalid()); // Wait for Core 1 drawing to finish
        multicore_fifo_pop_blocking();

        drawStats();
        drawHistogram();
        // Keep the LED on until the next frame completes within its deadline.
        elapsed_us = (uint32_t)(time_us_32() - frame_start_us);
        missed_deadline = elapsed_us > FRAME_BUDGET_US || draw_frame_expired();
        vga_frame_complete(); // No framebuffer writes until the next acquisition.
        gpio_put(DEADLINE_LED_PIN, missed_deadline);

        // NEVER exit while
    } // END WHILE(1)
    PT_END(pt);

} // animation thread


// Button interrupt on core 1
static PT_THREAD (protothread_anim1(struct pt *pt))
{
    PT_BEGIN(pt);

    while (1)
    {
        multicore_fifo_pop_blocking(); // wait for start

        // update boid physics
        for (int i = count_core0; i < frame_boid_count; ++i) 
            updateboid(&boids[i], &stats1);

        multicore_fifo_push_blocking(FLAG_VALUE); // Signal physics done

        multicore_fifo_pop_blocking(); // Wait for scanlinedraw signal
        // draw odd scanlines
        renderHollowBoidsParallel(1); // 1 = odd rows
        multicore_fifo_push_blocking(FLAG_VALUE); // Signal draw done

        multicore_fifo_push_blocking(FLAG_VALUE); // done
    }
    PT_END(pt);
}

// ========================================
// === core 1 main -- started in main below
// ========================================
void core1_main(){
  // configure GPIOs and enable pullups
    gpio_init(BOUNCE_PIN_OUT);
    gpio_set_dir(BOUNCE_PIN_OUT, GPIO_OUT);
    gpio_put(BOUNCE_PIN_OUT, 0);

    gpio_init(BOUNCE_PIN_IN);
    gpio_set_dir(BOUNCE_PIN_IN, GPIO_IN);
    gpio_pull_up(BOUNCE_PIN_IN);

    // // trigger an interrupt ONLY when input pin is pulled low, indicating short with output pin
    // gpio_set_irq_enabled (BOUNCE_PIN_IN, GPIO_IRQ_EDGE_FALL, true);
    // gpio_set_irq_callback (&gpio_core1_callback);
    gpio_set_irq_enabled_with_callback(
    BOUNCE_PIN_IN,
    GPIO_IRQ_EDGE_FALL,
    true,
    &gpio_core1_callback
);

  pt_add_thread(protothread_anim1);
  // Start the scheduler
  pt_schedule_start ;

}

// ========================================
// === main
// ========================================
// USE ONLY C-sdk library
int main()
{
    // Core DVDD, NOT the 3.3 V I/O rail. Keep the SDK voltage limit enabled.
    // Raise voltage before the existing 300 MHz overclock (board-test required).
    vreg_set_voltage(VREG_VOLTAGE_1_30);
    sleep_ms(10);
    set_sys_clock_khz(400000, true);
    // initialize stio
    stdio_init_all();

    // initialize VGA
    initVGA();

    // LED PIN FOR INDICATING MISSED DEADLINES (ON IF MISSED)
    gpio_init(DEADLINE_LED_PIN);
    gpio_put(DEADLINE_LED_PIN, false);
    gpio_set_dir(DEADLINE_LED_PIN, GPIO_OUT);


    // ========================================
    // === ROTARY PINS !!!
    // ========================================
    // configure GPIOs and enable pullups
    gpio_init(A_PIN);
    gpio_set_dir(A_PIN, GPIO_IN);
    gpio_pull_up(A_PIN);

    gpio_init(B_PIN);
    gpio_set_dir(B_PIN, GPIO_IN);
    gpio_pull_up(B_PIN);

    // trigger an inetrrupt ONLY when channel A changes
    // gpio_set_irq_enabled (A_PIN, GPIO_IRQ_EDGE_FALL, true);
    // gpio_set_irq_callback (&gpio_core0_callback);

    gpio_set_irq_enabled_with_callback(
    A_PIN,
    GPIO_IRQ_EDGE_FALL,
    true,
    &gpio_core0_callback
);

    // ========================================
    // === DMA STUFF BELOW !!!
    // ========================================

    // precommpute juicy thump ;)
    for (int i = 0; i < sine_table_size; i++) {
        float t = (float)i / 44000.0f; // 44,000 samples/s
        float thump =
            2047 +                            // midpoint or "zero"
            1500 * expf(-45.0f * t)         // amplitude * decay factor
                 * sinf(2.0f * 3.14159f * 120.0f * t) + 500 * expf(-180.0f * t) * sinf(2.0f * 3.14159f * 350.0f * t);  // oscillation, 150Hz sine wave
        DAC_data[i] = DAC_config_chan_A | (((int)thump) & 0x0fff); // takes calculated 12b sample -> 16b for DAC
    }

    // Guarantee the final sample leaves the DAC at midpoint.
    DAC_data[sine_table_size - 1] = DAC_config_chan_A | 2047;

    // Initialize SPI channel (channel, baud rate set to 20MHz)
    spi_init(SPI_PORT, 20000000);

    // Format SPI channel (channel, data bits per transfer, polarity, phase, order)
    spi_set_format(SPI_PORT, 16, 0, 0, 0);

    // Map SPI signals to GPIO ports
    // LDAC must be tied low btw
    gpio_set_function(PIN_CS, GPIO_FUNC_SPI);
    gpio_set_function(PIN_SCK, GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);

    // Start with the output at midpoint.
    spi_write16_blocking(SPI_PORT, &DAC_data[sine_table_size - 1], 1);

    // Claim unused channels so the VGA DMA channels are preserved.
    data_chan = dma_claim_unused_channel(true);
    ctrl_chan = dma_claim_unused_channel(true);

    // Setup the control channel
    dma_channel_config c = dma_channel_get_default_config(ctrl_chan); // default configs
    channel_config_set_transfer_data_size(&c, DMA_SIZE_32);           // 32-bit txfers
    channel_config_set_read_increment(&c, false);                     // no read incrementing
    channel_config_set_write_increment(&c, false);                    // no write incrementing
    channel_config_set_chain_to(&c, data_chan);                       // chain to data channel

    dma_channel_configure(
        ctrl_chan,                        // Channel to be configured
        &c,                               // The configuration we just created
        &dma_hw->ch[data_chan].read_addr, // Write address (data channel read address)
        &address_pointer,                 // Read address (POINTER TO AN ADDRESS)
        1,                                // Number of transfers
        false                             // Don't start immediately
    );

    // Setup the data channel
    dma_channel_config c2 = dma_channel_get_default_config(data_chan); // Default configs
    channel_config_set_transfer_data_size(&c2, DMA_SIZE_16);           // 16-bit txfers
    channel_config_set_read_increment(&c2, true);                      // yes read incrementing
    channel_config_set_write_increment(&c2, false);                    // no write incrementing
    // Timer rate = sys_clk * X/Y. At this program's 150 MHz: 44,000 samples/s.
    int audio_timer = dma_claim_unused_timer(true);
    dma_timer_set_fraction(audio_timer, 1, 9091);
    channel_config_set_dreq(&c2, dma_get_timer_dreq(audio_timer));
    // chain back to data channel because unlike the demo we don't want it to keep looping
    channel_config_set_chain_to(&c2, data_chan);

    dma_channel_configure(
        data_chan,                 // Channel to be configured
        &c2,                       // The configuration we just created
        &spi_get_hw(SPI_PORT)->dr, // write address (SPI data register)
        DAC_data,                  // The initial read address
        transfer_count,            // Number of transfers
        false                      // Don't start immediately.
    );
    // The sound thread starts DMA only when a peg impact is queued.

    // Randomize the initial horizontal velocity of each boid.
    spawn_rng  = time_us_32() | 1u;
    stats0.rng = spawn_rng ^ 0x9e3779b9u;
    stats1.rng = spawn_rng ^ 0x85ebca6bu;

    // start core 1 
    multicore_reset_core1();
    multicore_launch_core1(&core1_main);

    // add threads
    pt_add_thread(protothread_sound);
    pt_add_thread(protothread_anim);

    // start scheduler
    pt_schedule_start;
}
