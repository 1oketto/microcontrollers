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

// rotary encoder GPIOs (C_PIN is connected to GND pin 18)
#define A_PIN 13
#define B_PIN 14

static volatile int rotary_count = 0; // Requested active boids; never negative.

// GPIO ISR: CW is A falls while B is high
void gpio_callback(uint gpio, uint32_t events)
{
    if (gpio != A_PIN || !(events & GPIO_IRQ_EDGE_FALL)) return;
    if (gpio_get(B_PIN)) {
        if (rotary_count < INT_MAX) ++rotary_count;
    } else if (rotary_count > 0) {
        --rotary_count;
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
#define PEG_ROWS 16
#define PEG_COUNT (PEG_ROWS * (PEG_ROWS + 1) / 2)
// Center-to-center spacing between pegs and rows.
#define PEG_HORIZONTAL_SPACING 38
#define PEG_VERTICAL_SPACING 19
#define PEG_X int2fix15(320)
#define PEG_Y int2fix15(60)
#define GRAVITY ((fix15)12124) // 0.37 in Q15
#define BOUNCINESS ((fix15)16384) // 0.5 in Q15

static fix15 peg_x[PEG_COUNT], peg_y[PEG_COUNT];

// Histogram parameters and variables
#define HISTOGRAM_BINS (PEG_ROWS - 1) // one bin for each gap between adjacent pegs in the bottom row
static int histogram[HISTOGRAM_BINS] = {0}; // tracks num of boids fallen in each bin

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
        fillCircle((short)fix2int15(peg_x[peg]), (short)fix2int15(peg_y[peg]), PEG_RADIUS, WHITE);
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
static uint64_t total_fallen = 0;

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

static void updateboid(boid *boid)
{
    boid->x += boid->vx;
    boid->y += boid->vy;

    for (int peg = 0; peg < PEG_COUNT; ++peg) {
        fix15 dx = boid->x - peg_x[peg];
        fix15 dy = boid->y - peg_y[peg];
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
                boid->x = peg_x[peg] + multfix15(normal_x, collision_distance + int2fix15(1));
                boid->y = peg_y[peg] + multfix15(normal_y, collision_distance + int2fix15(1));
                // Only reflect when moving toward the peg, not away from it.
                if (dot < 0)
                {
                    fix15 intermediate_term = -2 * dot;
                    boid->vx += multfix15(normal_x, intermediate_term);
                    boid->vy += multfix15(normal_y, intermediate_term);
                    // Sound and damping only when this boid hits a different peg.
                    if (boid->last_peg != peg)
                    {
                        ++peg_sound_events;
                        boid->vx = multfix15(boid->vx, BOUNCINESS);
                        boid->vy = multfix15(boid->vy, BOUNCINESS);
                        boid->last_peg = peg;
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
            histogram[bin]++;
        boid->histogram_recorded = true;
    }

    // Respawn this boid after it falls completely below the screen.
    if (boid->y > int2fix15(480 + boid_RADIUS))
    {
        ++total_fallen;

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
    snprintf(text, sizeof(text), "boids being animated: %d", boid_count);
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
        // Clear the buffer
        clearLowFrame(0, BLACK);
        drawPegs();
        for (int i = 0; i < boid_count; ++i) {
            updateboid(&boids[i]);
            // Avoid drawing off-screen coordinates while a boid falls past the sides.
            if (boids[i].x >= -int2fix15(boid_RADIUS) && boids[i].x <= int2fix15(640 + boid_RADIUS) &&
                boids[i].y >= -int2fix15(boid_RADIUS) && boids[i].y <= int2fix15(480 + boid_RADIUS))
                fillCircle((short)fix2int15(boids[i].x), (short)fix2int15(boids[i].y), boid_RADIUS, color);
        }
        drawStats();
        drawHistogram();
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

    // trigger an inetrrupt ONLY when channel A changes
    gpio_set_irq_enabled_with_callback(A_PIN, GPIO_IRQ_EDGE_FALL, true, &gpio_callback);

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

    // add threads
    pt_add_thread(protothread_sound);
    pt_add_thread(protothread_anim);

    // start scheduler
    pt_schedule_start;
}
