/**
 * Hunter Adams (vha3@cornell.edu)
 *
 * Ball storage grows on demand; the initial count is configurable at build time.
 * Both cores update physics and render disjoint scanlines.
 * The rotary encoder controls ball count or bounciness; click to switch.
 * Encoder button: between GPIO 15 (input) and GPIO 12 (output low).
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
 *  - PIO0 state machines 0/1 for sync, PIO1 state machine 0 for pixels
 *  - 6 claimed DMA channels: VGA including background (4), sound (2)
 *  - 76.8 KB of RAM for two monochrome buffers; 38.4 KB background in flash
 *
 */

// Include the VGA grahics library
#include "VGA/vga16_graphics_v3.h"
#include "board_config.h"
// Include standard libraries
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <limits.h>
#include <stdatomic.h>
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

// Number of DMA transfers per event
const uint32_t transfer_count = sine_table_size;

static int data_chan;
static int ctrl_chan;
// boid collisions produce events; the sound thread consumes them.
static uint32_t peg_sound_events = 0; // how many collisions have happened
// played_peg_events = how many collisions have happened
// ^^ together they keep track of whether sound has been played for a collision
// and collisions aren't lost while DMA is busy

// 60 fps gives each animation frame 16,666 microseconds.
#define FRAME_BUDGET_US (1000000u / 60u)
#ifndef LAB_BENCH_VARIANT
#define LAB_BENCH_VARIANT "full"
#endif
#ifndef LAB_CALIBRATION_WARMUP_FRAMES
#define LAB_CALIBRATION_WARMUP_FRAMES 600u
#endif
#ifndef LAB_CALIBRATION_FRAMES
#define LAB_CALIBRATION_FRAMES 600u
#endif
#ifndef LAB_INITIAL_BALL_COUNT
#define LAB_INITIAL_BALL_COUNT 20000
#endif
#ifndef LAB_CAPACITY_CALIBRATION
#define LAB_CAPACITY_CALIBRATION 0
#endif
#ifndef LAB_SINGLE_CORE
#define LAB_SINGLE_CORE 0
#endif
#ifndef LAB_GENERIC_CIRCLE
#define LAB_GENERIC_CIRCLE 0
#endif
#ifndef LAB_NO_DRAW_CACHE
#define LAB_NO_DRAW_CACHE 0
#endif
#ifndef LAB_NO_TEXT_CACHE
#define LAB_NO_TEXT_CACHE 0
#endif
_Static_assert(LAB_CALIBRATION_FRAMES > 0, "A trial must time at least one frame");
#define ROTARY_BALL_STEP 500
_Static_assert(LAB_INITIAL_BALL_COUNT >= 0, "Initial ball count cannot be negative");
_Static_assert(LAB_INITIAL_BALL_COUNT <= INT_MAX, "Initial ball count exceeds the supported count type");
#if LAB_CAPACITY_CALIBRATION
#define CALIBRATION_MAX_BALLS 20000
static volatile bool calibrating = true;
static int calibration_good = -1; // No untested count is assumed to pass.
static int calibration_bad = CALIBRATION_MAX_BALLS + 1;
static unsigned calibration_frames = 0;
static unsigned calibration_warmup = 0;
static uint64_t calibration_total_us = 0;
static uint32_t calibration_max_us = 0;
static int calibration_target = CALIBRATION_MAX_BALLS;
#else
static volatile bool calibrating = false;
#endif

// rotary encoder GPIOs (C_PIN is connected to GND pin 18)
#define A_PIN 13
#define B_PIN 14
#define BUTTON_PIN 15
#define BUTTON_GROUND_PIN 12 // Driven low; switch wired between GPIO 15 and 12.

static volatile int rotary_count = LAB_INITIAL_BALL_COUNT; // Requested active boids.
static atomic_bool edit_bounciness = false; // Shared between the two cores.
// Bounciness ranges from 0% to 100% in 10% steps.
#define BOUNCE_LEVEL_MAX 10
static volatile int bounciness_level = 5;
static volatile int bounciness_q15 = (5 * 32768 + 5) / 10;
static volatile bool statistics_reset_pending = false;
static bool ball_allocation_failed = false;

// GPIO edges restart the debounce interval; a timer IRQ checks the result.
// A held button toggles only once, and must be released before another click.
#define BUTTON_DEBOUNCE_US 20000u
static volatile uint32_t button_last_edge_us;
static volatile bool button_alarm_pending = false;
static bool button_stable_high = true;
static alarm_pool_t *button_alarm_pool; // Created on core 1 so debounce IRQs run there.

static int64_t debounceButton(alarm_id_t id, void *user_data)
{
    (void)id;
    (void)user_data;
    uint32_t elapsed = (uint32_t)(time_us_32() - button_last_edge_us);
    if (elapsed < BUTTON_DEBOUNCE_US)
        return BUTTON_DEBOUNCE_US - elapsed;

    bool high = gpio_get(BUTTON_PIN);
    if (high != button_stable_high) {
        button_stable_high = high;
        if (!high) edit_bounciness = !edit_bounciness;
    }
    button_alarm_pending = false;
    return 0;
}

// Button GPIO ISR runs only on core 1.
static void button_gpio_callback(uint gpio, uint32_t events)
{
    if (gpio != BUTTON_PIN ||
        !(events & (GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE))) return;
    button_last_edge_us = time_us_32();
    if (!button_alarm_pending) {
        button_alarm_pending = true;
        if (alarm_pool_add_alarm_in_us(button_alarm_pool, BUTTON_DEBOUNCE_US,
                                      debounceButton, NULL, true) < 0)
            button_alarm_pending = false;
    }
}

static void core1_entry(void);

// Rotation GPIO ISR runs only on core 0: CW is A falls while B is high.
void gpio_callback(uint gpio, uint32_t events)
{
    if (gpio != A_PIN || !(events & GPIO_IRQ_EDGE_FALL))
        return;
    // A manual adjustment takes over immediately instead of waiting for the
    // multi-trial startup calibration to finish.
    calibrating = false;
    if (edit_bounciness) {
        int level = bounciness_level + (gpio_get(B_PIN) ? 1 : -1);
        if (level < 0) level = 0;
        if (level > BOUNCE_LEVEL_MAX) level = BOUNCE_LEVEL_MAX;
        if (level != bounciness_level) {
            bounciness_level = level;
            bounciness_q15 = (level * 32768 + BOUNCE_LEVEL_MAX / 2) /
                             BOUNCE_LEVEL_MAX;
            statistics_reset_pending = true;
        }
        return;
    }
    if (gpio_get(B_PIN))
    {
        rotary_count = rotary_count > INT_MAX - ROTARY_BALL_STEP
            ? INT_MAX : rotary_count + ROTARY_BALL_STEP;
    }
    else
    {
        rotary_count = rotary_count > ROTARY_BALL_STEP
            ? rotary_count - ROTARY_BALL_STEP : 0;
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
            dma_channel_set_trans_count(ctrl_chan, 1, false);              // rearm control DMA
            dma_start_channel_mask(1u << ctrl_chan);                       // start control DMA
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
static inline fix15 multiplyFix15(fix15 a, fix15 b)
{
    int64_t product = (int64_t)a * b;
    uint64_t magnitude = product < 0 ? 0u - (uint64_t)product : (uint64_t)product;
    fix15 result = (fix15)(magnitude >> 15);
    return product < 0 ? -result : result;
}
#define multfix15(a, b) multiplyFix15((a), (b))
#define float2fix15(a) ((fix15)((a) * 32768.0))                          // 2^15
#define fix2float15(a) ((float)(a) / 32768.0)
#define absfix15(a) abs(a)
#define int2fix15(a) ((fix15)((a) * 32768))
#define fix2int15(a) ((int)(a >> 15))
#define char2fix15(a) int2fix15(a)
#define divfix(a, b) ((fix15)(((int64_t)(a) * 32768) / (int64_t)(b)))

// global variables for boids and pegs. positions are pixels; velocities are pixels/frame.
#define boid_RADIUS BOARD_BALL_RADIUS
#define PEG_RADIUS BOARD_PEG_RADIUS
#define PEG_ROWS BOARD_PEG_ROWS
#define PEG_COUNT (PEG_ROWS * (PEG_ROWS + 1) / 2)
// Center-to-center spacing between pegs and rows.
#define PEG_HORIZONTAL_SPACING BOARD_PEG_HORIZONTAL_SPACING
#define PEG_VERTICAL_SPACING BOARD_PEG_VERTICAL_SPACING
#define CONTACT_RADIUS (boid_RADIUS + PEG_RADIUS)
#define PEG_X int2fix15(BOARD_CENTER_X)
#define PEG_Y int2fix15(BOARD_TOP_Y)
#define GRAVITY ((fix15)12124)
#define BOUNCINESS ((fix15)bounciness_q15)
#define RANDOM_LATERAL_KICK_Q15 16384 // 0.5 pixels/frame, with a fair random sign.

// Independent ablations for on-board measurements; default to optimized paths.
#ifndef LAB_FULL_PEG_SCAN
#define LAB_FULL_PEG_SCAN 0
#endif
#ifndef LAB_REFERENCE_NORMAL
#define LAB_REFERENCE_NORMAL 0
#endif
_Static_assert((PEG_HORIZONTAL_SPACING & 1) == 0, "peg spacing must be even");

static fix15 peg_x[PEG_COUNT], peg_y[PEG_COUNT];

// Histogram parameters and variables
#define HISTOGRAM_BINS (PEG_ROWS - 1)       // one bin for each gap between adjacent pegs in the bottom row
static int histogram[HISTOGRAM_BINS] = {0}; // tracks num of boids fallen in each bin

static void initPegs(void)
{
    int peg = 0;
    for (int row = 0; row < PEG_ROWS; ++row)
    {
        for (int col = 0; col <= row; ++col)
        {
            peg_x[peg] = PEG_X + ((col << 1) - row) * int2fix15(PEG_HORIZONTAL_SPACING / 2);
            peg_y[peg] = PEG_Y + int2fix15(row * PEG_VERTICAL_SPACING);
            ++peg;
        }
    }
}

// Compact storage: positions have 1/32-pixel precision, velocities 1/1024.
// Collision calculations still use Q15 intermediates. Signed positions allow
// balls to leave the screen; 32 pixels/frame covers normal Galton-board speeds.
typedef struct
{
    int16_t x, y;   // Q5, range -1024 to just below 1024 pixels.
    int16_t vx, vy; // Q10, range -32 to just below 32 pixels/frame.
    uint8_t last_peg; // 0..135, or NO_PEG before the first impact.
    bool histogram_recorded;
} boid;
#define NO_PEG UINT8_MAX
_Static_assert(sizeof(boid) == 10, "Ball storage must remain compact");
static boid *boids;
static int boid_capacity;
static int boid_count = 0;

// Round to the compact format. Saturation keeps off-screen trajectories from
// wrapping back into the board if a ball travels far beyond the display.
static inline int16_t packFixed(fix15 value, unsigned shift)
{
    int32_t packed = (value + (1 << (shift - 1))) >> shift;
    if (packed > INT16_MAX) packed = INT16_MAX;
    if (packed < INT16_MIN) packed = INT16_MIN;
    return (int16_t)packed;
}

static uint64_t total_fallen = 0;

// Each core owns its deltas and RNG; core 0 merges deltas after completion.
typedef struct {
    uint32_t rng;
    uint32_t collisions;
    uint32_t fallen;
    int histogram[HISTOGRAM_BINS];
} physics_state;
static physics_state physics[2] = {{.rng = 1}, {.rng = 2}};
static uint32_t spawn_rng = 3;
static semaphore_t physics_start, physics_done;
static int worker_begin, worker_end;
static fix15 frame_bounciness;
static bool worker_draw;

static uint32_t nextRandom(uint32_t *state)
{
    uint32_t value = *state;
    value ^= value << 13;
    value ^= value >> 17;
    value ^= value << 5;
    *state = value;
    return value;
}

static void dropboid(boid *ball, uint32_t *rng)
{
    ball->x = packFixed(PEG_X, 10);
    ball->y = boid_RADIUS * 32;
    // More than 100 initial speeds avoid a small set of repeating paths.
    fix15 vx = (fix15)(328 + nextRandom(rng) % 3605);
    if (nextRandom(rng) & 1u) vx = -vx;
    ball->vx = packFixed(vx, 5);
    ball->vy = 0;
    ball->last_peg = NO_PEG;
    ball->histogram_recorded = false;
}

// Resize only between worker jobs so neither core can retain a stale ball pointer.
static void syncboidCount(void)
{
    int requested = rotary_count;
    if (requested < 0) requested = 0;
    if (requested > boid_capacity) {
        boid *resized = NULL;
        if ((size_t)requested <= SIZE_MAX / sizeof(*boids))
            resized = realloc(boids, (size_t)requested * sizeof(*boids));
        if (resized == NULL) {
            ball_allocation_failed = true;
            uint32_t irq_state = save_and_disable_interrupts();
            if (rotary_count == requested)
                rotary_count = boid_capacity;
            restore_interrupts(irq_state);
            requested = boid_capacity;
        } else {
            boids = resized;
            boid_capacity = requested;
            ball_allocation_failed = false;
        }
    }
    for (int i = boid_count; i < requested; ++i)
        dropboid(&boids[i], &spawn_rng);
    boid_count = requested;
}

// Consume the encoder request outside the ISR. Protect the read-and-clear
// so an interrupt cannot overwrite a newly requested reset.
static void resetStatisticsIfRequested(void)
{
    uint32_t irq_state = save_and_disable_interrupts();
    bool reset = statistics_reset_pending;
    statistics_reset_pending = false;
    restore_interrupts(irq_state);
    if (reset) {
        memset(histogram, 0, sizeof(histogram));
        total_fallen = 0;
    }
}

// Start each trial with the same drop sequence so counts are comparable.
#if LAB_CAPACITY_CALIBRATION
static void startCalibrationTrial(int count)
{
    calibration_target = count;
    rotary_count = count;
    syncboidCount();
    spawn_rng = 3;
    physics[0].rng = 1;
    physics[1].rng = 2;
    for (int i = 0; i < boid_count; ++i)
        dropboid(&boids[i], &spawn_rng);
    memset(histogram, 0, sizeof(histogram));
    total_fallen = 0;
    calibration_frames = 0;
    calibration_warmup = 0;
    calibration_total_us = 0;
    calibration_max_us = 0;
}

// Time only steady-state frames; log after submitting the framebuffer so serial
// output is outside the timed region. Warmup on the next trial absorbs logging.
static void finishCalibrationFrame(bool missed_deadline, uint32_t elapsed_us)
{
    if (!calibrating) return;
    if (calibration_warmup < LAB_CALIBRATION_WARMUP_FRAMES) {
        ++calibration_warmup;
        return;
    }
    ++calibration_frames;
    calibration_total_us += elapsed_us;
    if (elapsed_us > calibration_max_us) calibration_max_us = elapsed_us;
    if (!missed_deadline && calibration_frames < LAB_CALIBRATION_FRAMES) return;

    printf("BENCH,%s,%lu,1300,%d,%u,%u,%lu,%lu,%u,%u\n",
           LAB_BENCH_VARIANT, (unsigned long)clock_get_hz(clk_sys),
           calibration_target, calibration_warmup, calibration_frames,
           (unsigned long)calibration_max_us,
           (unsigned long)(calibration_total_us / calibration_frames),
           missed_deadline ? 1u : 0u, missed_deadline ? 0u : 1u);
    if (missed_deadline) calibration_bad = calibration_target;
    else calibration_good = calibration_target;

    if (calibration_bad - calibration_good <= 1) {
        rotary_count = calibration_good < 0 ? 0 : calibration_good;
        syncboidCount();
        spawn_rng = time_us_32() | 1u;
        physics[0].rng = spawn_rng;
        physics[1].rng = spawn_rng ^ 0x9e3779b9u;
        if (physics[1].rng == 0) physics[1].rng = 2;
        for (int i = 0; i < boid_count; ++i) dropboid(&boids[i], &spawn_rng);
        memset(histogram, 0, sizeof(histogram));
        total_fallen = 0;
        calibrating = false;
        if (calibration_good >= 0)
            printf("CAPACITY,%s,%d,%d,%u\n", LAB_BENCH_VARIANT,
                   calibration_good, CALIBRATION_MAX_BALLS,
                   calibration_good == CALIBRATION_MAX_BALLS);
        else
            printf("BENCH_ERROR,%s,even_zero_balls_missed_deadline\n", LAB_BENCH_VARIANT);
        return;
    }
    int next = calibration_good + (calibration_bad - calibration_good) / 2;
    startCalibrationTrial(next);
}
#endif

// Pico 2 has hardware floating-point square root. Use it for an initial root,
// then correct against the exact integer square to retain Q15 collision results.
static fix15 distanceFix15(fix15 dx, fix15 dy)
{
    uint64_t squared = (uint64_t)((int64_t)dx * dx) +
                       (uint64_t)((int64_t)dy * dy);
#if LAB_REFERENCE_NORMAL
    uint64_t remainder = squared, root = 0, bit = UINT64_C(1) << 62;
    while (bit > remainder) bit >>= 2;
    while (bit != 0) {
        if (remainder >= root + bit) {
            remainder -= root + bit;
            root = (root >> 1) + bit;
        } else {
            root >>= 1;
        }
        bit >>= 2;
    }
#else
    float fx = (float)dx, fy = (float)dy;
    uint32_t root = (uint32_t)sqrtf(fx * fx + fy * fy);
    while ((uint64_t)root * root > squared) --root;
    while ((uint64_t)(root + 1) * (root + 1) <= squared) ++root;
#endif
    return (fix15)root;
}

// Estimate a Q15 normal with the FPU, then correct integer products to match
// signed division exactly. This avoids software 64-bit division per component.
static inline fix15 normalComponent(fix15 component, fix15 distance, float scale)
{
#if LAB_REFERENCE_NORMAL
    (void)scale;
    return divfix(component, distance);
#else
    uint32_t magnitude = component < 0 ? (uint32_t)-component : (uint32_t)component;
    uint64_t numerator = (uint64_t)magnitude * 32768u;
    uint32_t quotient = (uint32_t)((float)magnitude * scale);
    uint64_t product = (uint64_t)quotient * (uint32_t)distance;
    while (product > numerator) { --quotient; product -= distance; }
    while (product + distance <= numerator) { ++quotient; product += distance; }
    return component < 0 ? -(fix15)quotient : (fix15)quotient;
#endif
}

static inline fix15 dampVelocity(fix15 velocity, fix15 bounciness)
{
    return multfix15(velocity, bounciness);
}

static inline fix15 collisionClearance(fix15 normal)
{
    // Place the ball one pixel beyond the combined ball/peg contact radius.
    uint32_t magnitude = normal < 0 ? 0u - (uint32_t)normal : (uint32_t)normal;
    fix15 offset = (fix15)(magnitude * (CONTACT_RADIUS + 1u));
    return normal < 0 ? -offset : offset;
}

static void updateboid(boid *ball, physics_state *state, fix15 bounciness)
{
    const int radius = boid_RADIUS + PEG_RADIUS;
    const fix15 collision_distance = int2fix15(radius);
    if (ball->last_peg != NO_PEG && ball->last_peg >= PEG_COUNT) {
        ball->last_peg = NO_PEG;
    } else if (ball->last_peg != NO_PEG) {
        fix15 previous_dx = (fix15)ball->x * 1024 - peg_x[ball->last_peg];
        fix15 previous_dy = (fix15)ball->y * 1024 - peg_y[ball->last_peg];
        int64_t previous_distance_squared = (int64_t)previous_dx * previous_dx +
                                            (int64_t)previous_dy * previous_dy;
        if (previous_distance_squared >= (int64_t)collision_distance * collision_distance)
            ball->last_peg = NO_PEG;
    }

    fix15 vx = (fix15)ball->vx * 32;
    fix15 vy = (fix15)ball->vy * 32;
    fix15 x = (fix15)ball->x * 1024 + vx;
    fix15 y = (fix15)ball->y * 1024 + vy;

    // Conservative lower bounds preserve the full scan's row/column order.
    // Later rows/columns test the current position after collision correction.
#if LAB_FULL_PEG_SCAN
    int first_row = 0;
#else
    int first_row = (fix2int15(y) - fix2int15(PEG_Y) - radius) / PEG_VERTICAL_SPACING;
    if (first_row < 0) first_row = 0;
#endif
    for (int row = first_row; row < PEG_ROWS; ++row)
    {
        int row_start = (row * (row + 1)) >> 1;
#if LAB_FULL_PEG_SCAN
        int first_col = 0;
#else
        fix15 row_dy = y - peg_y[row_start];
        if (row_dy <= -int2fix15(radius)) break;
        if (row_dy >= int2fix15(radius)) continue;
        int first_col = (fix2int15(x) - fix2int15(peg_x[row_start]) - radius)
                        / PEG_HORIZONTAL_SPACING;
        if (first_col < 0) first_col = 0;
#endif
        for (int col = first_col; col <= row; ++col)
        {
            int peg = row_start + col;
#if !LAB_FULL_PEG_SCAN
            if (peg_x[peg] - x >= int2fix15(radius)) break;
#endif
            fix15 dx = x - peg_x[peg];
            fix15 dy = y - peg_y[peg];
            if (absfix15(dx) < collision_distance && absfix15(dy) < collision_distance)
            {
                int64_t distance_squared = (int64_t)dx * dx + (int64_t)dy * dy;
                if (distance_squared < (int64_t)collision_distance * collision_distance)
                {
                    fix15 distance = distanceFix15(dx, dy);
                    // Normalize by the ACTUAL distance, which varies during penetration.
                    // The combined radius is not a power of two, so dx/distance
                    // cannot be replaced with a bit shift.
                    // Coincident centers use an upward normal to avoid division by zero.
                    float normal_scale = distance > 0 ? 32768.0f / (float)distance : 0.0f;
                    fix15 normal_x = distance > 0 ? normalComponent(dx, distance, normal_scale) : 0;
                    fix15 normal_y = distance > 0 ? normalComponent(dy, distance, normal_scale) : -int2fix15(1);
                    fix15 dot = multfix15(normal_x, vx) + multfix15(normal_y, vy);
                    x = peg_x[peg] + collisionClearance(normal_x);
                    y = peg_y[peg] + collisionClearance(normal_y);
                    // Only reflect when moving toward the peg, not away from it.
                    if (dot < 0)
                    {
                        fix15 intermediate_term = (fix15)((0u - (uint32_t)dot) << 1);
                        vx += multfix15(normal_x, intermediate_term);
                        vy += multfix15(normal_y, intermediate_term);
                        // Sound and damping only when this boid hits a different peg.
                        if (ball->last_peg != peg)
                        {
                            ++state->collisions;
                            vx = dampVelocity(vx, bounciness);
                            vy = dampVelocity(vy, bounciness);
                            vx += (nextRandom(&state->rng) & 0x80000000u)
                                ? RANDOM_LATERAL_KICK_Q15
                                : -RANDOM_LATERAL_KICK_Q15;
                            ball->last_peg = peg;
                        }
                    }
                }
            }
        }
    }

    // Record the gap as soon as the boid clears the bottom row.
    // Waiting until the screen edge lets horizontal drift change the bin.
    const fix15 bottom_row_y = PEG_Y + int2fix15((PEG_ROWS - 1) * PEG_VERTICAL_SPACING);
    if (!ball->histogram_recorded &&
        y > bottom_row_y + int2fix15(PEG_RADIUS + boid_RADIUS))
    {
        const fix15 first_peg_x =
            PEG_X - int2fix15((PEG_ROWS - 1) * PEG_HORIZONTAL_SPACING) / 2;
        // Reject positions left of the board before integer division.
        fix15 offset = x - first_peg_x;
        int bin = offset < 0 ? -1 : offset / int2fix15(PEG_HORIZONTAL_SPACING);
        if (bin >= 0 && bin < HISTOGRAM_BINS)
            state->histogram[bin]++;
        ball->histogram_recorded = true;
    }

    // Respawn this boid after it falls completely below the screen.
    if (y > int2fix15(480 + boid_RADIUS))
    {
        ++state->fallen;

        dropboid(ball, &state->rng);
        return; // reset the new drop's initial y-velocity to zero
    }
    vy += GRAVITY;
    ball->x = packFixed(x, 10);
    ball->y = packFixed(y, 10);
    ball->vx = packFixed(vx, 5);
    ball->vy = packFixed(vy, 5);
}

static void updateRange(int begin, int end, physics_state *state)
{
    state->collisions = 0;
    state->fallen = 0;
    memset(state->histogram, 0, sizeof(state->histogram));
    fix15 bounciness = frame_bounciness;
    for (int i = begin; i < end; ++i)
        updateboid(&boids[i], state, bounciness);
}

static void drawBallRows(unsigned parity);

static void core1_entry(void)
{
    button_alarm_pool = alarm_pool_create_with_unused_hardware_alarm(1);
    gpio_set_irq_enabled_with_callback(BUTTON_PIN,
        GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE, true, button_gpio_callback);
    while (true) {
        sem_acquire_blocking(&physics_start);
        if (worker_draw)
            drawBallRows(1);
        else
            updateRange(worker_begin, worker_end, &physics[1]);
        sem_release(&physics_done);
    }
}

static void mergePhysics(void)
{
    for (int core = 0; core < 2; ++core) {
        peg_sound_events += physics[core].collisions;
        total_fallen += physics[core].fallen;
        for (int bin = 0; bin < HISTOGRAM_BINS; ++bin)
            histogram[bin] += physics[core].histogram[bin];
    }
}

// Alternate scanlines give both cores disjoint framebuffer bytes, even for
// overlapping balls. All physics must finish before either renderer reads them.
static void drawBallRows(unsigned parity)
{
    for (int row = 0; row < PEG_ROWS; ++row) {
        for (int col = 0; col <= row; ++col) {
            short x = (short)(BOARD_CENTER_X +
                              (2 * col - row) * PEG_HORIZONTAL_SPACING / 2);
            short y = (short)(BOARD_TOP_Y + row * PEG_VERTICAL_SPACING);
            newCircleRows(x, y, parity);
        }
    }

    // An exact-key cache skips redundant stamps when many balls overlap.
    // Hash collisions merely cause another draw; they never hide another ball.
#if !LAB_NO_DRAW_CACHE
    static uint32_t drawn_centers[2][256];
    uint32_t *cache = drawn_centers[parity];
    memset(cache, 0xff, sizeof(drawn_centers[0]));
#endif
    for (int i = 0; i < boid_count; ++i) {
        if (boids[i].histogram_recorded) continue;
        int x = boids[i].x >> 5;
        int y = boids[i].y >> 5;
        if (x >= -BOARD_BALL_DRAW_RADIUS && x <= 639 + BOARD_BALL_DRAW_RADIUS &&
            y >= -BOARD_BALL_DRAW_RADIUS && y <= 479 + BOARD_BALL_DRAW_RADIUS) {
#if !LAB_NO_DRAW_CACHE
            uint32_t key = ((uint32_t)(y + BOARD_BALL_DRAW_RADIUS) << 10) |
                           (uint32_t)(x + BOARD_BALL_DRAW_RADIUS);
            unsigned slot = (key ^ (key >> 8)) & 255u;
            if (cache[slot] == key) continue;
            cache[slot] = key;
#endif
#if LAB_GENERIC_CIRCLE
            referenceWhiteBallRows(x, y, parity);
#else
            drawWhiteBallRows(x, y, parity);
#endif
        }
    }
}

// draw the stats: # boids being animated, total # of boids fallen since reset, and time since boot
static void drawStats(void)
{
    static char ball_text[48], fallen_text[64], bounce_text[32], time_text[64];
    static int last_count = -1, last_bounciness = -1;
    static uint64_t last_fallen = UINT64_MAX, last_seconds = UINT64_MAX;
    uint64_t seconds = time_us_64() / 1000000u;
    int bounciness = bounciness_q15;
    if (LAB_NO_TEXT_CACHE || boid_count != last_count) {
        snprintf(ball_text, sizeof(ball_text), "%d", boid_count);
        last_count = boid_count;
    }
    if (LAB_NO_TEXT_CACHE || total_fallen != last_fallen) {
        snprintf(fallen_text, sizeof(fallen_text), "%llu",
                 (unsigned long long)total_fallen);
        last_fallen = total_fallen;
    }
    if (LAB_NO_TEXT_CACHE || bounciness != last_bounciness) {
        snprintf(bounce_text, sizeof(bounce_text), "%d%%",
                 bounciness_level * 10);
        last_bounciness = bounciness;
    }
    if (LAB_NO_TEXT_CACHE || seconds != last_seconds) {
        snprintf(time_text, sizeof(time_text), "%llu:%02u:%02u",
                 (unsigned long long)(seconds / 3600),
                 (unsigned)((seconds / 60) % 60), (unsigned)(seconds % 60));
        last_seconds = seconds;
    }
    setTextSize(1);
    setTextColor2(WHITE, BLACK);
    // Separate selection marker from the fixed left edge of every label.
    setCursor(10, edit_bounciness ? 30 : 10);
    writeString(">");
    setCursor(22, 10);
    writeString("boids being animated: ");
    setCursor(22 + 6 * (sizeof("boids being animated: ") - 1), 10);
    writeString(ball_text);
    setCursor(22, 20);
    writeString("Total fallen since reset: ");
    setCursor(22 + 6 * (sizeof("Total fallen since reset: ") - 1), 20);
    writeString(fallen_text);
    setCursor(22, 30);
    writeString("Bounciness: ");
    setCursor(22 + 6 * (sizeof("Bounciness: ") - 1), 30);
    writeString(bounce_text);
    setCursor(22, 40);
    writeString("Time since boot: ");
    setCursor(22 + 6 * (sizeof("Time since boot: ") - 1), 40);
    writeString(time_text);
#if LAB_CAPACITY_CALIBRATION
    if (calibrating) {
        setCursor(22, 50);
        writeString("Calibrating 60 fps capacity...");
    }
#endif
    if (ball_allocation_failed) {
        setCursor(22, 50);
        writeString("Ball memory limit reached");
    }
}

// Bars align with the fifteen gaps in the bottom peg row.
static void drawHistogram(void)
{
    static char count_text[HISTOGRAM_BINS][12];
    static int last_count[HISTOGRAM_BINS];
    const int height = 98;
    const int baseline = 463; // Leave room below the bars for bin totals.
    int max_count = 0;
    for (int i = 0; i < HISTOGRAM_BINS; ++i)
        if (histogram[i] > max_count)
            max_count = histogram[i];

    setTextSize(1);
    setTextColor(WHITE);
    for (int i = 0; i < HISTOGRAM_BINS; ++i)
    {
        int bin_x = 35 + i * PEG_HORIZONTAL_SPACING;
        int bar_height = max_count > 0
            ? (int)((int64_t)histogram[i] * height / max_count) : 0;
        if (bar_height > 0)
            drawRect(bin_x, baseline - bar_height,
                     PEG_HORIZONTAL_SPACING - 1, bar_height, WHITE);

        if (LAB_NO_TEXT_CACHE || count_text[i][0] == '\0' || histogram[i] != last_count[i]) {
            snprintf(count_text[i], sizeof(count_text[i]), "%d", histogram[i]);
            last_count[i] = histogram[i];
        }
        int text_width = (int)strlen(count_text[i]) * 6;
        setCursor(bin_x + (PEG_HORIZONTAL_SPACING - 1 - text_width) / 2, 468);
        writeString(count_text[i]);
    }
}

// Animation on core 0
static PT_THREAD(protothread_anim(struct pt *pt))
{
    static uint32_t frame_start_us, elapsed_us;
    static bool missed_deadline;
    static int split;
    // Mark beginning of thread
    PT_BEGIN(pt);

    initPegs();
#if LAB_CAPACITY_CALIBRATION
    startCalibrationTrial(calibration_target);
#endif

    while (1)
    {
        // Wait for the signal that the buffer's changed
        PT_YIELD_UNTIL(pt, draw_start_signal());
        frame_start_us = draw_frame_start_us();
        clearRegion(0, 480, BLACK);
        // The previous worker has completed before any resize or new drop.
        syncboidCount();
        frame_bounciness = BOUNCINESS;
        split = boid_count / 2;
#if LAB_SINGLE_CORE
        // Preserve both RNG streams and ranges for an identical workload.
        updateRange(0, split, &physics[0]);
        updateRange(split, boid_count, &physics[1]);
        mergePhysics();
        drawBallRows(0);
        drawBallRows(1);
#else
        worker_begin = split;
        worker_end = boid_count;
        worker_draw = false;
        sem_release(&physics_start);
        updateRange(0, split, &physics[0]);
        PT_YIELD_UNTIL(pt, sem_try_acquire(&physics_done));
        mergePhysics();
        worker_draw = true;
        sem_release(&physics_start);
        drawBallRows(0);
        PT_YIELD_UNTIL(pt, sem_try_acquire(&physics_done));
#endif
        resetStatisticsIfRequested();
        drawStats();
        drawHistogram();
        elapsed_us = (uint32_t)(time_us_32() - frame_start_us);
        missed_deadline = elapsed_us > FRAME_BUDGET_US || draw_frame_expired();
        vga_frame_complete(); // No framebuffer writes until the next acquisition.
#if LAB_CAPACITY_CALIBRATION
        finishCalibrationFrame(missed_deadline, elapsed_us);
#endif
        // NEVER exit while
    } // END WHILE(1)
    PT_END(pt);

} // animation thread

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
    set_sys_clock_khz(300000, true);
    // initialize stio
    stdio_init_all();

    // initialize VGA
    initVGA();

    // configure GPIOs and enable pullups
    gpio_init(A_PIN);
    gpio_set_dir(A_PIN, GPIO_IN);
    gpio_pull_up(A_PIN);

    gpio_init(B_PIN);
    gpio_set_dir(B_PIN, GPIO_IN);
    gpio_pull_up(B_PIN);

    gpio_init(BUTTON_GROUND_PIN);
    gpio_put(BUTTON_GROUND_PIN, false);
    gpio_set_dir(BUTTON_GROUND_PIN, GPIO_OUT);

    gpio_init(BUTTON_PIN);
    gpio_set_dir(BUTTON_PIN, GPIO_IN);
    gpio_pull_up(BUTTON_PIN);

    // Each core installs its own GPIO callback and enables its own input IRQ.
    gpio_set_irq_enabled_with_callback(A_PIN, GPIO_IRQ_EDGE_FALL, true, &gpio_callback);
    sem_init(&physics_start, 0, 1);
    sem_init(&physics_done, 0, 1);
    multicore_launch_core1(core1_entry);

    // ========================================
    // === DMA STUFF BELOW !!!
    // ========================================

    // precommpute juicy thump ;)
    for (int i = 0; i < sine_table_size; i++)
    {
        float t = (float)i / 44000.0f; // 44,000 samples/s
        float thump =
            2047 + 1500 * expf(-45.0f * t) * sinf(2.0f * 3.14159f * 120.0f * t) + 500 * expf(-180.0f * t) * sinf(2.0f * 3.14159f * 350.0f * t);
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
    // Timer rate = sys_clk * X/Y: approximately 44,000 samples/s at 300 MHz.
    int audio_timer = dma_claim_unused_timer(true);
    dma_timer_set_fraction(audio_timer, 5, 34091);
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
    spawn_rng = time_us_32() | 1u;

    // add threads
    pt_add_thread(protothread_sound);
    pt_add_thread(protothread_anim);

    // start scheduler
    pt_schedule_start;
}
