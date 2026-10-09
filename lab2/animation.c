/**
 * Hunter Adams (vha3@cornell.edu)
 *
 * This demonstration drops multiple balls through a 16-row Galton board.
 * Starts with 16,000 balls. If rendering misses refreshes, the count is reduced
 * until the VGA output can keep up; the rotary encoder adjusts it manually.
 *
 * HARDWARE CONNECTIONS
  - GPIO 16 ---> VGA Hsync
  - GPIO 17 ---> VGA Vsync
  - GPIO 18 ---> VGA Green --> 470 ohm resistor --> VGA_Green
  - RP2040 GND ---> VGA-GND
 *
 * RESOURCES USED
 *  - PIO state machines 0, 1, and 2 on PIO instance 0
 *  - DMA channels claimed for VGA, background restore, and sound
 *  - Two packed 1-bit frame buffers (38,400 bytes each)
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

// The waveform table is a precomputed audio envelope used by the DAC-driven
// "thump" sound. 256 entries gives a compact table that can be streamed
// continuously without generating audio in the main simulation loop.
#define sine_table_size 256

// DAC_data[] holds the 16-bit values sent to the MCP4921 DAC over SPI. Each
// entry is precomputed so the sound thread can fire DMA without heavy CPU work.
unsigned short DAC_data[sine_table_size];

// address_pointer points at the table base so the DMA control channel can
// re-arm the audio stream by writing the next address back to the data channel.
unsigned short *address_pointer = &DAC_data[0];

// MCP4921 config word for channel A, gain=1x, active output. The lower 12 bits
// hold the waveform sample value created by the exponential-decay sine pulse.
#define DAC_config_chan_A 0b0011000000000000

// SPI wiring for the DAC: CS, SCK, and MOSI are fixed to specific GPIO pins.
#define PIN_CS 5
#define PIN_SCK 6
#define PIN_MOSI 3
#define SPI_PORT spi0

// Each audio DMA transfer sends one 16-bit sample. This constant matches the
// size of the waveform table and is reused by the DMA setup code.
const uint32_t transfer_count = sine_table_size;

static int data_chan;
static int ctrl_chan;
// peg_sound_events is a queue-like counter: the physics thread increments it for
// each collision, and the sound thread consumes it by playing a matching DAC
// burst. The extra played_peg_events counter prevents missed or duplicated
// sound events while DMA is busy and the main loop is still running.
static uint32_t peg_sound_events = 0; // how many collisions have happened
// played_peg_events = how many collisions have happened
// ^^ together they keep track of whether sound has been played for a collision
// and collisions aren't lost while DMA is busy

// The rotary encoder is read using GPIOs A and B. The sketch treats channel A's
// falling edge as the event that resolves direction using the state of B.
#define A_PIN 13
#define B_PIN 14

// Start at the observed high-load count without probing above it on startup.
#define START_BALL_COUNT 16000
static volatile int rotary_count = START_BALL_COUNT;
static volatile bool auto_ramp = false;
static int auto_safe_count = 0;
static int auto_failed_count = INT_MAX;
static int auto_probe_step = 1;
static uint32_t auto_clean_frames = 0;
#define AUTO_RAMP_STABLE_FRAMES 30u

// The GPIO ISR is intentionally tiny: it only looks at channel A falling edges
// and adjusts the requested ball count while keeping the value bounded.

// GPIO ISR: CW is A falls while B is high
void gpio_callback(uint gpio, uint32_t events)
{
    if (gpio != A_PIN || !(events & GPIO_IRQ_EDGE_FALL)) return;
    auto_ramp = false;
    if (gpio_get(B_PIN)) {
        if (rotary_count < INT_MAX) ++rotary_count;
    } else if (rotary_count > 0) {
        --rotary_count;
    }
}

// The sound protothread acts like a very small scheduler: it watches the
// collision counter, waits until the DAC DMA pipeline is idle, then re-arms the
// DMA chain to emit a short thump for each impact. This keeps sound generation
// decoupled from the physics update, which reduces jitter in the simulation.
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
            // Re-arm both DMA stages so the same table can be replayed. The
            // control channel writes the table address back to the data channel,
            // then the data channel pushes the waveform to the DAC.
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
// === BALLS AND PEGS !!!!
// ========================================
// The simulation keeps most positions and velocities in fixed-point form so the
// physics can stay fast and deterministic on a microcontroller without needing a
// floating-point math library. The chosen Q-format gives enough precision for a
// compact Galton board while still fitting in 16-bit and 32-bit arithmetic.
#define POSITION_BITS 5
#define POSITION_ONE (1 << POSITION_BITS)
#define VELOCITY_BITS 10
#define VELOCITY_ONE (1 << VELOCITY_BITS)
#define NORMAL_BITS 15
#define NORMAL_ONE (1 << NORMAL_BITS)
#define BALL_RADIUS BOARD_BALL_RADIUS
#define PEG_RADIUS BOARD_PEG_RADIUS
#define PEG_ROWS BOARD_PEG_ROWS
#define PEG_COUNT (PEG_ROWS * (PEG_ROWS + 1) / 2)
#define PEG_HORIZONTAL_SPACING BOARD_PEG_HORIZONTAL_SPACING
#define PEG_VERTICAL_SPACING BOARD_PEG_VERTICAL_SPACING
#define PEG_X BOARD_CENTER_X
#define PEG_Y BOARD_TOP_Y
#define CONTACT_RADIUS (1 << BOARD_COLLISION_RADIUS_SHIFT)
#define CONTACT_FIXED (CONTACT_RADIUS * POSITION_ONE)
#define GRAVITY_FIXED 379 // round(0.37 * 1024)
#define NO_PEG UINT8_MAX
#define HISTOGRAM_BINS (PEG_ROWS - 1)
#define HISTOGRAM_TOP 390
#define HISTOGRAM_BOTTOM 470
#define LAST_PEG_Y (PEG_Y + (PEG_ROWS - 1) * PEG_VERTICAL_SPACING)
#define LAST_ROW_LEFT (PEG_X - (PEG_ROWS - 1) * PEG_HORIZONTAL_SPACING / 2)
#define STAMP_CACHE_SIZE 1024

_Static_assert(PEG_COUNT < NO_PEG, "Last peg must fit in one byte");
_Static_assert(BALL_RADIUS + PEG_RADIUS == CONTACT_RADIUS,
               "Collision radius must match the power-of-two scale");

// Each ball stores the compact state needed for the simulation: position,
// velocity, most recently hit peg, and whether it has contributed to the
// histogram yet. Packing this into 10 bytes reduces memory traffic and keeps the
// physics loop cache-friendly on the RP2350.
typedef struct {
    int16_t x, y;
    int16_t vx, vy;
    uint8_t last_peg;
    uint8_t histogram_recorded;
} Ball;
_Static_assert(sizeof(Ball) == 10, "Each ball occupies exactly 10 bytes");

// PhysicsResult holds the per-frame deltas for one core: collisions, fallen
// balls, and the per-slot histogram counts for the bins at the bottom of the board.
typedef struct {
    uint32_t collisions;
    uint32_t fallen;
    uint32_t histogram[HISTOGRAM_BINS];
} PhysicsResult;

static Ball *balls = NULL;
static int ball_count = 0;
static int ball_capacity = 0;
static uint64_t total_fallen = 0;
static uint64_t histogram[HISTOGRAM_BINS];
static PhysicsResult physics_result[2];
static uint32_t random_state[2];
static uint32_t stamp_cache[2][STAMP_CACHE_SIZE];
static semaphore_t physics_start, physics_done, drawing_start, drawing_done;

// Separate generators let each core respawn its own balls without sharing rand().
// This Xorshift-style generator is extremely small and deterministic enough for
// animation randomness without the overhead of a heavier RNG implementation.
static uint32_t nextRandom(uint32_t *state)
{
    uint32_t value = *state;
    value ^= value << 13;
    value ^= value >> 17;
    value ^= value << 5;
    *state = value;
    return value;
}

// A ball is reset to the top of the board with a random horizontal kick and zero
// vertical velocity. The speed is kept nonzero so the animation does not stall
// in a single static frame when the random value happens to be tiny.
static void dropBall(Ball *ball, uint32_t *rng)
{
    ball->x = PEG_X * POSITION_ONE;
    ball->y = BALL_RADIUS * POSITION_ONE;
    int32_t speed = ((1 + nextRandom(rng) % 12) * VELOCITY_ONE + 50) / 100;
    // Preserve a nonzero horizontal step after converting velocity to Q5 position.
    if (speed < (VELOCITY_ONE / POSITION_ONE))
        speed = VELOCITY_ONE / POSITION_ONE;
    ball->vx = (nextRandom(rng) & 1u) ? -speed : speed;
    ball->vy = 0;
    ball->last_peg = NO_PEG;
    ball->histogram_recorded = 0;
}

// This hook synchronizes the live ball array with the rotary encoder count. It
// grows the array only when needed and gracefully degrades if memory allocation
// fails, keeping the visible count stable instead of crashing the system.
static void syncBallCount(void)
{
    int requested = rotary_count;
    if (requested > ball_capacity) {
        Ball *resized = NULL;
        if ((size_t)requested <= SIZE_MAX / sizeof(*balls))
            resized = realloc(balls, (size_t)requested * sizeof(*balls));
        if (resized == NULL) {
            uint32_t irq_state = save_and_disable_interrupts();
            if (auto_ramp && requested < auto_failed_count)
                auto_failed_count = requested;
            if (rotary_count == requested)
                rotary_count = auto_ramp ? auto_safe_count : ball_count;
            restore_interrupts(irq_state);
            return;
        }
        balls = resized;
        ball_capacity = requested;
    }
    for (int i = ball_count; i < requested; ++i)
        dropBall(&balls[i], &random_state[0]);
    ball_count = requested;
}

// Probe progressively larger workloads, then binary-search between the highest
// render-safe count and the first count that misses a VGA frame deadline.
static void tuneBallCount(void)
{
    static uint32_t observed_missed_frames;
    static bool initialized;
    uint32_t missed_frames = vga_missed_frame_count();
    if (!initialized) {
        observed_missed_frames = missed_frames;
        initialized = true;
        return;
    }

    uint32_t new_misses = missed_frames - observed_missed_frames;
    observed_missed_frames = missed_frames;
    if (new_misses != 0) {
        auto_clean_frames = 0;
        uint32_t irq_state = save_and_disable_interrupts();
        if (auto_ramp) {
            int failed_count = rotary_count;
            if (failed_count < auto_failed_count)
                auto_failed_count = failed_count;
            rotary_count = auto_safe_count;
            auto_probe_step = 0;
        } else if (rotary_count > 0) {
            int reduction = new_misses > (uint32_t)rotary_count
                                ? rotary_count
                                : (int)new_misses;
            rotary_count -= reduction;
        }
        restore_interrupts(irq_state);
        return;
    }

    if (++auto_clean_frames < AUTO_RAMP_STABLE_FRAMES) return;
    auto_clean_frames = 0;
    uint32_t irq_state = save_and_disable_interrupts();
    if (!auto_ramp) {
        restore_interrupts(irq_state);
        return;
    }
    auto_safe_count = rotary_count;

    int next_count;
    if (auto_failed_count != INT_MAX) {
        int gap = auto_failed_count - auto_safe_count;
        if (gap <= 1) {
            auto_ramp = false;
            restore_interrupts(irq_state);
            return;
        }
        auto_probe_step = gap / 2;
    } else if (auto_probe_step > INT_MAX / 2) {
        auto_probe_step = INT_MAX;
    } else {
        auto_probe_step *= 2;
    }

    if (auto_probe_step > INT_MAX - auto_safe_count)
        next_count = INT_MAX;
    else
        next_count = auto_safe_count + auto_probe_step;
    if (next_count > auto_failed_count)
        next_count = auto_failed_count - 1;
    if (next_count <= auto_safe_count) {
        auto_ramp = false;
        restore_interrupts(irq_state);
        return;
    }
    rotary_count = next_count;
    restore_interrupts(irq_state);
}

// Clamp to a 16-bit signed range so game state remains valid even when a ball
// is knocked or falls beyond the viewport. The physics uses these clamped values
// as the shared representation of position and velocity.
static int16_t clamp16(int32_t value)
{
    if (value < INT16_MIN) return INT16_MIN;
    if (value > INT16_MAX) return INT16_MAX;
    return (int16_t)value;
}

// integerRoot computes a reciprocal-like square root for the peg collision code
// without doing expensive floating-point math on every collision. It starts from a
// floating estimate and then corrects the integer result to be exact.
static uint32_t integerRoot(uint32_t squared)
{
    uint32_t root = (uint32_t)sqrtf((float)squared);
    // The float estimate can round either way; compare in integers to correct it.
    while ((uint64_t)root * root > squared) --root;
    while ((uint64_t)(root + 1u) * (root + 1u) <= squared) ++root;
    return root;
}

// normalComponent resolves the signed unit-normal for one Cartesian component by
// using a precomputed reciprocal and a one-step correction. This reproduces the
// exact division result while avoiding a full 64-bit division in the hot path.
static int32_t normalComponent(int32_t component, uint32_t distance,
                               uint32_t reciprocal)
{
    uint32_t magnitude = component < 0 ? (uint32_t)-component : (uint32_t)component;
    uint32_t normal = (magnitude * reciprocal) >> 15;
    // Components and distance are Q10, with |component| < 8192.
    // Their Q30 reciprocal estimate leaves the Q15 normal at most one unit low.
    // Correct it to the same result as (component * NORMAL_ONE) / distance.
    if ((normal + 1u) * distance <= magnitude * NORMAL_ONE) ++normal;
    return component < 0 ? -(int32_t)normal : (int32_t)normal;
}

// collideWithPeg is the core peg bounce routine. It computes the offset from a
// peg, converts that offset into a collision normal, pushes the ball out of the
// peg if it overlaps, and reflects the velocity when the ball is moving toward
// the peg. The result is a physically plausible collision without expensive
// floating-point operations.
static void collideWithPeg(Ball *ball, int row, int col, PhysicsResult *result)
{
    int32_t peg_x = (PEG_X + (2 * col - row) * PEG_HORIZONTAL_SPACING / 2) * POSITION_ONE;
    int32_t peg_y = (PEG_Y + row * PEG_VERTICAL_SPACING) * POSITION_ONE;
    int32_t dx = ball->x - peg_x;
    int32_t dy = ball->y - peg_y;
    if (dx <= -CONTACT_FIXED || dx >= CONTACT_FIXED ||
        dy <= -CONTACT_FIXED || dy >= CONTACT_FIXED) return;
    uint32_t squared = (uint32_t)(dx * dx + dy * dy);
    if (squared >= CONTACT_FIXED * CONTACT_FIXED) return;

    int32_t normal_x = 0, normal_y = -NORMAL_ONE;
    if (squared != 0) {
        // Add five fractional bits to distance before finding the normal.
        // The bounding circle guarantees (squared << 10) < 2^26.
        uint32_t distance = integerRoot(squared << 10);
        // Both components share one division; the small corrections are exact.
        uint32_t reciprocal = (1u << 30) / distance;
        normal_x = normalComponent(dx * 32, distance, reciprocal);
        normal_y = normalComponent(dy * 32, distance, reciprocal);
    }
    int32_t dot = (int32_t)(((int64_t)normal_x * ball->vx +
                             (int64_t)normal_y * ball->vy) / NORMAL_ONE);
    // Resolve overlap at the 8-pixel contact radius plus the original one-pixel
    // clearance. Both the multiplication by 9 and Q15-to-Q5 scaling are constant.
    ball->x = clamp16(peg_x + (normal_x * (CONTACT_RADIUS + 1)) / (NORMAL_ONE / POSITION_ONE));
    ball->y = clamp16(peg_y + (normal_y * (CONTACT_RADIUS + 1)) / (NORMAL_ONE / POSITION_ONE));
    if (dot < 0) {
        int32_t reflected_x = ball->vx + (int32_t)(((int64_t)normal_x * (-2 * dot)) / NORMAL_ONE);
        int32_t reflected_y = ball->vy + (int32_t)(((int64_t)normal_y * (-2 * dot)) / NORMAL_ONE);
        uint8_t peg = (uint8_t)(row * (row + 1) / 2 + col);
        if (ball->last_peg != peg) {
            ++result->collisions;
            // Bounciness is 0.5: shift by one, rounding negative values to zero.
            reflected_x = (reflected_x + (reflected_x < 0)) >> 1;
            reflected_y = (reflected_y + (reflected_y < 0)) >> 1;
            ball->last_peg = peg;
        }
        ball->vx = clamp16(reflected_x);
        ball->vy = clamp16(reflected_y);
    }
}

// Only the nearest peg can affect a ball because the peg spacing is much larger
// than the collision diameter. This prunes the expensive full-board collision
// search to a single candidate row/column.
static void collideWithNearbyPeg(Ball *ball, PhysicsResult *result)
{
    // Rows are 19 pixels apart and columns are 38 pixels apart, both greater
    // than the 16-pixel collision diameter. Only the nearest row/column can hit.
    if (ball->y <= (PEG_Y - CONTACT_RADIUS) * POSITION_ONE ||
        ball->y >= (LAST_PEG_Y + CONTACT_RADIUS) * POSITION_ONE) return;
    int row = (ball->y - PEG_Y * POSITION_ONE +
               PEG_VERTICAL_SPACING * POSITION_ONE / 2) /
              (PEG_VERTICAL_SPACING * POSITION_ONE);
    int32_t row_left = (PEG_X - row * PEG_HORIZONTAL_SPACING / 2) * POSITION_ONE;
    if (ball->x <= row_left - CONTACT_FIXED ||
        ball->x >= row_left + row * PEG_HORIZONTAL_SPACING * POSITION_ONE + CONTACT_FIXED)
        return;
    int col = (ball->x - row_left + PEG_HORIZONTAL_SPACING * POSITION_ONE / 2) /
              (PEG_HORIZONTAL_SPACING * POSITION_ONE);
    collideWithPeg(ball, row, col, result);
}

// Each physics step moves the ball by one time step, checks for a peg collision,
// records the distribution bin when the ball exits the board, and respawns the
// ball if it falls past the bottom edge.
static void updateBall(Ball *ball, PhysicsResult *result, uint32_t *rng)
{
    ball->x = clamp16(ball->x + ball->vx / (VELOCITY_ONE / POSITION_ONE));
    ball->y = clamp16(ball->y + ball->vy / (VELOCITY_ONE / POSITION_ONE));
    collideWithNearbyPeg(ball, result);

    if (!ball->histogram_recorded &&
        ball->y > (LAST_PEG_Y + CONTACT_RADIUS) * POSITION_ONE) {
        int bin = (ball->x - LAST_ROW_LEFT * POSITION_ONE) /
                  (PEG_HORIZONTAL_SPACING * POSITION_ONE);
        if (bin < 0) bin = 0;
        if (bin >= HISTOGRAM_BINS) bin = HISTOGRAM_BINS - 1;
        ++result->histogram[bin];
        ball->histogram_recorded = 1;
    }
    if (ball->y > (480 + BALL_RADIUS) * POSITION_ONE) {
        ++result->fallen;
        dropBall(ball, rng);
        return;
    }
    ball->vy = clamp16(ball->vy + GRAVITY_FIXED);
}

// The physics work is split evenly across both cores so the board can process a
// large number of balls without stalling the VGA draw loop. Each core owns its
// own RNG and writes only into its dedicated result block.
static void updateBalls(unsigned core)
{
    PhysicsResult *result = &physics_result[core];
    memset(result, 0, sizeof(*result));
    int midpoint = ball_count / 2;
    int first = core ? midpoint : 0;
    int limit = core ? ball_count : midpoint;
    for (int i = first; i < limit; ++i)
        updateBall(&balls[i], result, &random_state[core]);
}

// combinePhysics merges the two parallel physics result buffers into the global
// counters. It is intentionally done after the two cores join so the totals and
// histogram remain consistent for the next frame.
static void combinePhysics(void)
{
    for (int core = 0; core < 2; ++core) {
        peg_sound_events += physics_result[core].collisions;
        total_fallen += physics_result[core].fallen;
        for (int bin = 0; bin < HISTOGRAM_BINS; ++bin)
            histogram[bin] += physics_result[core].histogram[bin];
    }
}

// The drawing pass uses a tiny hash cache to avoid re-stamping the same ball in
// the same row parity. This is an optimization: a circular ball stamp is only
// drawn once per cell, and duplicate work is skipped before the expensive
// pixel-fill routine is called.
static void drawBalls(unsigned row_parity)
{
    uint32_t *cache = stamp_cache[row_parity];
    memset(cache, 0, sizeof(stamp_cache[0]));
    for (int i = 0; i < ball_count; ++i) {
        int x = balls[i].x / POSITION_ONE;
        int y = balls[i].y / POSITION_ONE;
        if (x < -BALL_RADIUS || x > 640 + BALL_RADIUS ||
            y < -BALL_RADIUS || y > 480 + BALL_RADIUS) continue;
        uint32_t key = (uint32_t)((y + BALL_RADIUS) * (640 + 2 * BALL_RADIUS + 1) +
                                  x + BALL_RADIUS + 1);
        uint32_t slot = (key * 2654435761u) >> 22;
        // Hash collisions only lose a skip opportunity; compare the full key.
        if (cache[slot] == key) continue;
        cache[slot] = key;
        newCircleRows(x, y, row_parity);
    }
}

// drawHistogram visualizes the distribution of balls that have fallen beyond the
// peg field, giving a quick read on the statistical bias of the board.
static void drawHistogram(void)
{
    uint64_t largest = 0;
    for (int bin = 0; bin < HISTOGRAM_BINS; ++bin)
        if (histogram[bin] > largest) largest = histogram[bin];
    if (largest == 0) return;
    for (int bin = 0; bin < HISTOGRAM_BINS; ++bin) {
        int height = (int)(histogram[bin] * (HISTOGRAM_BOTTOM - HISTOGRAM_TOP) / largest);
        if (height == 0) continue;
        drawRect(LAST_ROW_LEFT + bin * PEG_HORIZONTAL_SPACING + 2,
                 HISTOGRAM_BOTTOM - height, PEG_HORIZONTAL_SPACING - 4,
                 height, WHITE);
    }
}

// drawStats updates the text values only when they change. This avoids wasting
// time re-formatting strings every frame while still keeping the UI responsive.
static void drawStats(void)
{
    static int cached_balls = -1;
    static uint64_t cached_fallen = UINT64_MAX, cached_seconds = UINT64_MAX;
    static char ball_text[16], fallen_text[24], time_text[32];
    uint64_t seconds = time_us_64() / 1000000u;
    if (cached_balls != ball_count) {
        snprintf(ball_text, sizeof(ball_text), "%d", ball_count);
        cached_balls = ball_count;
    }
    if (cached_fallen != total_fallen) {
        snprintf(fallen_text, sizeof(fallen_text), "%llu", (unsigned long long)total_fallen);
        cached_fallen = total_fallen;
    }
    if (cached_seconds != seconds) {
        snprintf(time_text, sizeof(time_text), "%llu:%02u:%02u",
                 (unsigned long long)(seconds / 3600),
                 (unsigned)((seconds / 60) % 60), (unsigned)(seconds % 60));
        cached_seconds = seconds;
    }
    // Labels are already in the flash background; values are redrawn each frame.
    setTextSize(1);
    setTextColor(WHITE);
    setCursor(154, 10);
    writeString(ball_text);
    setCursor(178, 20);
    writeString(fallen_text);
    setCursor(94, 30);
    writeString("0.5");
    setCursor(124, 40);
    writeString(time_text);
}

// Core 1 is dedicated to physics. It waits for a start signal, updates half the
// balls, and then hands control back to the main draw loop after the frame passes.
static void core1_entry(void)
{
    while (1) {
        sem_acquire_blocking(&physics_start);
        updateBalls(1);
        sem_release(&physics_done);
        sem_acquire_blocking(&drawing_start);
        drawBalls(1);
        sem_release(&drawing_done);
    }
}

// Animation on core 0. Each semaphore boundary also publishes shared memory.
// This protothread is the frame coordinator: it waits for background restore,
// syncs the ball count, runs the physics, joins the second core, draws the balls,
// and finally completes the VGA frame. The semaphore handshake keeps the double
// buffer and the multi-core state coherent.
static PT_THREAD(protothread_anim(struct pt *pt))
{
    PT_BEGIN(pt);
    while (1) {
        // This becomes true only after DMA finishes restoring the background.
        PT_YIELD_UNTIL(pt, draw_start_signal());
        tuneBallCount();
        syncBallCount();
        sem_release(&physics_start);
        updateBalls(0);
        sem_acquire_blocking(&physics_done);
        combinePhysics();

        sem_release(&drawing_start);
        drawBalls(0);
        sem_acquire_blocking(&drawing_done);
        // Text and bars can touch both row parities, so draw them after the join.
        drawHistogram();
        drawStats();
        vga_frame_complete();
    }
    PT_END(pt);
}

// ========================================
// === main
// ========================================
// USE ONLY C-sdk library
// This is the program entry point. It initializes the board clock, VGA output,
// encoder inputs, and audio DMA plumbing, then starts the two MCU cores and the
// protothread scheduler so the simulation can run continuously.
int main()
{
    // The RP2350 runs at a lower default voltage; this sets it to the minimum
    // supported level for the display and audio pipeline and then waits briefly
    // for regulation to settle.
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

    // trigger an interrupt ONLY when channel A changes. The callback updates the
    // ball count based on encoder direction, and this keeps the user control path
    // isolated from the simulation update.
    gpio_set_irq_enabled_with_callback(A_PIN, GPIO_IRQ_EDGE_FALL, true, &gpio_callback);

    // ========================================
    // === DMA STUFF BELOW !!!
    // ========================================

    // precompute juicy thump ;)
    // The waveform is a decaying 150 Hz sine burst. This produces a short impact
    // tone without wasting CPU time in the main simulation loop; it is simple,
    // cheap, and repeatable.
    for (int i = 0; i < sine_table_size; i++) {
        float t = (float)i / 44000.0f; // 44,000 samples/s
        float thump =
            2047 +                                      // midpoint or "zero"
            1800 * expf(-1200.0f * t)          // amplitude * decay factor
                 * sinf(2.0f * 3.14159f * 150.0f * t);  // oscillation, 150Hz sine wave
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
    // The control DMA channel reprograms the data-channel read address to the
    // waveform table pointer. This lets the audio burst restart on demand without
    // CPU involvement.
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
    // The SPI TX channel streams one 16-bit sample per timer tick. Because the
    // timer DREQ is linked to the DMA transfer, the sound remains smooth and the
    // CPU is free to handle the board physics.
    dma_channel_config c2 = dma_channel_get_default_config(data_chan); // Default configs
    channel_config_set_transfer_data_size(&c2, DMA_SIZE_16);           // 16-bit txfers
    channel_config_set_read_increment(&c2, true);                      // yes read incrementing
    channel_config_set_write_increment(&c2, false);                    // no write incrementing
    // Timer rate = sys_clk * X/Y. At 300 MHz this is 43,999.74 samples/s.
    // The exact 11/75000 fraction would exceed the timer's 16-bit denominator.
    int audio_timer = dma_claim_unused_timer(true);
    dma_timer_set_fraction(audio_timer, 9, 61364);
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

    // Each core owns its RNG and result counters; ball allocation begins on the first frame.
    random_state[0] = time_us_32() | 1u;
    random_state[1] = (random_state[0] ^ 0x9e3779b9u) | 1u;
    sem_init(&physics_start, 0, 1);
    sem_init(&physics_done, 0, 1);
    sem_init(&drawing_start, 0, 1);
    sem_init(&drawing_done, 0, 1);
    multicore_launch_core1(core1_entry);

    // add threads
    // The motion logic is split into the animation protothread and the audio
    // protothread so the system can handle asynchronous display refresh, physics,
    // and sound without large blocking delays.
    pt_add_thread(protothread_sound);
    pt_add_thread(protothread_anim);

    // start scheduler
    pt_schedule_start;
}
