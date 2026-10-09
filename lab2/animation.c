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
} core_stats;

static core_stats stats0 = {0};
static core_stats stats1 = {0};


// Number of DMA transfers per event
const uint32_t transfer_count = sine_table_size;

static int data_chan;
static int ctrl_chan;
// boid collisions produce events; the sound thread consumes them.

// played_peg_events = how many collisions have happened
// ^^ together they keep track of whether sound has been played for a collision
// and collisions aren't lost while DMA is busy

// rotary encoder GPIOs (C_PIN is connected to GND pin 18)
#define A_PIN 13
#define B_PIN 14
#define BOUNCE_PIN_OUT 15
#define BOUNCE_PIN_IN 12

static volatile int rotary_count = 0; // Requested active boids; never negative.
static volatile int current_bounciness = 37;

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

void gpio_core0_callback(uint gpio, uint32_t events)
{
    total_fallen = 0;
    if (gpio != A_PIN || !(events & GPIO_IRQ_EDGE_FALL)) return;
    
    if (rotary_mode == NUM_BALL_MODE) {
        if (gpio_get(B_PIN)) {
            if (rotary_count < INT_MAX)
                ++rotary_count;
        } else if (rotary_count > 0) {
            --rotary_count;
        }
    }
    else {  // BOUNCY_MODE
        if (gpio_get(B_PIN)) {
            if (current_bounciness < 100)
                ++current_bounciness;
        } else if (current_bounciness > 0) {
            --current_bounciness;
        }
    }
}

// GPIO ISR: If input pin gets pulled low (normally to VDD w PU resistor), means button pressed (shorted to low output pin)
static volatile uint64_t last_button_time = 0;

void gpio_core1_callback(uint gpio, uint32_t events)
{
    total_fallen = 0;
    if (gpio != BOUNCE_PIN_IN || !(events & GPIO_IRQ_EDGE_FALL))
        return;

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
#define boid_RADIUS 4
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
typedef struct {
    fix15 x, y;
    fix15 vx, vy;
    int last_peg;
    bool histogram_recorded;
} boid;

static boid *boids = NULL;
static int boid_count = 0;
static int boid_capacity = 0;

static void dropboid(boid *boid)
{
    boid->x = PEG_X;
    boid->y = int2fix15(boid_RADIUS);
    // Use fine Q15 increments (~0.01 to 0.12 pixels/frame). Twelve
    // discrete speeds repeat the same paths and leave holes in the histogram.
    boid->vx = (fix15)(328 + rand() % 3605);
    if (rand() % 2)
        boid->vx = -boid->vx;
    boid->vy = 0;
    boid->last_peg = -1;
    boid->histogram_recorded = false;
}

// Allocate outside the ISR and preserve existing boids when the count changes.
static void syncboidCount(void)
{
    int requested = rotary_count;
    if (requested > boid_capacity) {
        boid *resized = NULL;
        if ((size_t)requested <= SIZE_MAX / sizeof(*boids))
            resized = realloc(boids, (size_t)requested * sizeof(*boids));
        if (resized == NULL) {
            // Keep the displayed and requested counts consistent if memory is full.
            uint32_t irq_state = save_and_disable_interrupts();
            if (rotary_count == requested) rotary_count = boid_count;
            restore_interrupts(irq_state);
            return;
        }
        boids = resized;
        boid_capacity = requested;
    }
    for (int i = boid_count; i < requested; ++i)
        dropboid(&boids[i]);
    boid_count = requested;
}

// Integer square root of a Q30 squared distance returns a Q15 distance.
static fix15 distanceFix15(fix15 dx, fix15 dy)
{
    uint64_t remainder = (uint64_t)((int64_t)dx * dx) +
                         (uint64_t)((int64_t)dy * dy);
    uint64_t root = 0;
    uint64_t bit = (uint64_t)1 << 62;
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
    return (fix15)root;
}

static void updateboid(boid *boid, core_stats *stats)
{
    boid->x += boid->vx;
    boid->y += boid->vy;

    int next_peg_arr[2] = {boid->last_peg, 0};
    if (boid->last_peg < 120){ // assume in last row; no more collisions

        if (boid->last_peg == -1) {
            next_peg_arr[1] = 0;
        }
        else if (boid->vx > 0)
            next_peg_arr[1] = peg_right_child[boid->last_peg];
        else
            next_peg_arr[1] = peg_left_child[boid->last_peg];

        for (int i = 0; i < sizeof(next_peg_arr) / sizeof(next_peg_arr[0]); ++i) {
            fix15 dx = boid->x - peg_x[next_peg_arr[i]];
            fix15 dy = boid->y - peg_y[next_peg_arr[i]];

            fix15 collision_distance = int2fix15(boid_RADIUS + PEG_RADIUS);
            if (absfix15(dx) < collision_distance && absfix15(dy) < collision_distance)
            {
                fix15 distance = distanceFix15(dx, dy);
                if (distance < collision_distance)
                {
                    // If centers coincide, choose an upward normal to avoid dividing by zero.
                    fix15 normal_x = distance > 0 ? divfix(dx, distance) : 0;
                    fix15 normal_y = distance > 0 ? divfix(dy, distance) : -int2fix15(1);
                    fix15 dot = multfix15(normal_x, boid->vx) + multfix15(normal_y, boid->vy);
                    boid->x = peg_x[next_peg_arr[i]] + multfix15(normal_x, collision_distance + int2fix15(1));
                    boid->y = peg_y[next_peg_arr[i]] + multfix15(normal_y, collision_distance + int2fix15(1));
                    // Only reflect when moving toward the peg, not away from it.
                    if (dot < 0)
                    {
                        fix15 intermediate_term = -2 * dot;
                        boid->vx += multfix15(normal_x, intermediate_term);
                        boid->vy += multfix15(normal_y, intermediate_term);
                        // Sound and damping only when this boid hits a different peg.
                        if (boid->last_peg != next_peg_arr[i])
                        {
                            ++stats->peg_sound_events;
                            fix15 bounce = int2fix15(current_bounciness) / 100;

                            boid->vx = multfix15(boid->vx, bounce);
                            boid->vy = multfix15(boid->vy, bounce);
                            boid->last_peg = next_peg_arr[i];
                        }
                    }
                }
            }
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

        dropboid(boid);
        return; // reset the new drop's initial y-velocity to zero
    }
    boid->vy += GRAVITY; // add
}

// draw the stats: # boids being animated, total # of boids fallen since reset, and time since boot
static void drawStats(void)
{
    char text[80];
    uint64_t seconds = time_us_64() / 1000000u;
    setTextSize(2);
    setTextColor(WHITE);
    setCursor(10, 5);
    snprintf(text, sizeof(text), "Boids being animated: %d", boid_count);
    writeString(text);
    setCursor(10, 22);
    snprintf(text, sizeof(text), "Total fallen since reset: %llu",
             (unsigned long long)total_fallen);
    writeString(text);
    setCursor(10, 39);
    snprintf(text, sizeof(text), "Time since boot: %llu:%02u:%02u",
             (unsigned long long)(seconds / 3600),
             (unsigned)((seconds / 60) % 60), (unsigned)(seconds % 60));
    writeString(text);
    setCursor(10, 56);
    snprintf(text, sizeof(text), "Bounciness: %d", current_bounciness);
    writeString(text);
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

// Animation on core 0
static PT_THREAD(protothread_anim(struct pt *pt))
{
    // Mark beginning of thread
    PT_BEGIN(pt);

    // Start with an empty board; the encoder adds and removes boids.
    initPegs();

    while (1)
    {
        // Wait for the signal that the buffer's changed
        PT_YIELD_UNTIL(pt, draw_start_signal());
        syncboidCount();
        frame_boid_count = boid_count;
        count_core0 = frame_boid_count / 2 + frame_boid_count % 2;
        
        multicore_fifo_push_blocking(FLAG_VALUE);

        for (int i = 0; i < count_core0; ++i) {
            updateboid(&boids[i], &stats0);
        }
        multicore_fifo_push_blocking(FLAG_VALUE);
        // synchronized now with core 1, safe to draw

        // NEW: combine core 0 and core 1 histogram
        for (int bin = 0; bin < HISTOGRAM_BINS; ++bin) {
            histogram[bin] = stats0.histogram[bin] + stats1.histogram[bin];
        }
        peg_sound_events = stats0.peg_sound_events + stats1.peg_sound_events;
        total_fallen = stats0.total_fallen + stats1.total_fallen;

        // Clear the buffer
        clearLowFrame(0, BLACK);
        drawPegs();
        for (int i = 0; i < frame_boid_count; ++i) {
            // Avoid drawing off-screen coordinates while a boid falls past the sides.
            if (boids[i].x >= -int2fix15(boid_RADIUS) && boids[i].x <= int2fix15(640 + boid_RADIUS) &&
                boids[i].y >= -int2fix15(boid_RADIUS) && boids[i].y <= int2fix15(480 + boid_RADIUS))
                drawCircle((short)fix2int15(boids[i].x), (short)fix2int15(boids[i].y), boid_RADIUS, color);
        }
        
        drawStats();
        drawHistogram();

        multicore_fifo_push_blocking(FLAG_VALUE); // synchronize w Core 1 to proceed to next frame
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
        multicore_fifo_pop_blocking();
        for (int i = count_core0; i < frame_boid_count; ++i) {
                updateboid(&boids[i], &stats1);
        }
        multicore_fifo_pop_blocking(); // synchronize w Core 0 to proceed with drawing
        multicore_fifo_pop_blocking(); // synchronize w Core 0 to proceed to next frame
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
    set_sys_clock_khz(300000, true);
    vreg_set_voltage(VREG_VOLTAGE_1_30);
    // initialize stio
    stdio_init_all();

    // initialize VGA
    initVGA();

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
            1800 * expf(-800.0f * t)          // amplitude * decay factor
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
    dma_timer_set_fraction(audio_timer, 11, 37500);
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
    srand(time_us_32());

    // start core 1 
    multicore_reset_core1();
    multicore_launch_core1(&core1_main);

    // add threads
    pt_add_thread(protothread_sound);
    pt_add_thread(protothread_anim);

    // start scheduler
    pt_schedule_start;
}
