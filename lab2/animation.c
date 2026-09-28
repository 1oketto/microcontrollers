/**
 * Hunter Adams (vha3@cornell.edu)
 *
 * This demonstration drops multiple balls through a 16-row Galton board.
 * Through a serial interface, the user can change the ball color.
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
#include "pico/divider.h"
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

// Sine table
int raw_sin[sine_table_size];

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
// Ball collisions produce events; the sound thread consumes them.
static uint32_t peg_sound_events = 0; // how many collisions have happened
// played_peg_events = how many collisions have happened
// ^^ together they keep track of whether sound has been played for a collision
// and collisions aren't lost while DMA is busy

// rotary encoder GPIOs (C_PIN is connected to GND pin 18)
#define A_PIN 13
#define B_PIN 14

// define states for the rotary encoder
typedef enum
{
    CW,
    CCW,
    STOPPED
} rotary_dir_t;
static volatile rotary_dir_t rotary_dir_state = STOPPED; // and initialize state variable

static volatile int rotary_count = 0; // Requested active balls; never negative.

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

// play a DMA sound for each ball-peg collision
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
// === BALLS AND PEGS !!!!
// ========================================
// the fixed point macros
typedef signed int fix15;
#define multfix15(a, b) ((fix15)((((signed long long)(a)) * ((signed long long)(b))) >> 15))
#define float2fix15(a) ((fix15)((a) * 32768.0)) // 2^15
#define fix2float15(a) ((float)(a) / 32768.0)
#define absfix15(a) abs(a)
#define int2fix15(a) ((fix15)(a << 15))
#define fix2int15(a) ((int)(a >> 15))
#define char2fix15(a) (fix15)(((fix15)(a)) << 15)
#define divfix(a, b) (fix15)(div_s64s64((((signed long long)(a)) << 15), ((signed long long)(b))))

// the color of the boid
char color = WHITE;

// Boid on core 0
fix15 boid0_x;
fix15 boid0_y;
fix15 boid0_vx;
fix15 boid0_vy;

// Boid on core 1
fix15 boid1_x;
fix15 boid1_y;
fix15 boid1_vx;
fix15 boid1_vy;

// Create a semaphore
semaphore_t draw_semaphore;

// Create a boid
void spawnBoid(fix15 *x, fix15 *y, fix15 *vx, fix15 *vy, int direction)
{
    // Start in the top center of screen
    *x = int2fix15(320);
    *y = int2fix15(0);
    // Choose left or right ...
    if (direction)
        *vx = int2fix15(3);
    else
        *vx = int2fix15(-3);
    // Moving down
    *vy = int2fix15(1);
}

// global variables for balls and pegs. positions are pixels; velocities are pixels/frame.
#define BALL_RADIUS 4
#define PEG_RADIUS 6
#define PEG_ROWS 16
#define PEG_COUNT (PEG_ROWS * (PEG_ROWS + 1) / 2)
// Center-to-center spacing between pegs and rows.
#define PEG_HORIZONTAL_SPACING 38
#define PEG_VERTICAL_SPACING 19
#define PEG_X 320.0f
#define PEG_Y 60.0f
#define GRAVITY 0.37f
#define BOUNCINESS 0.5f

static float peg_x[PEG_COUNT], peg_y[PEG_COUNT];

static void initPegs(void)
{
    int peg = 0;
    for (int row = 0; row < PEG_ROWS; ++row) {
        for (int col = 0; col <= row; ++col) {
            peg_x[peg] = PEG_X + (col - row * 0.5f) * PEG_HORIZONTAL_SPACING;
            peg_y[peg] = PEG_Y + row * PEG_VERTICAL_SPACING;
            ++peg;
        }
    }
}

static void drawPegs(void)
{
    for (int peg = 0; peg < PEG_COUNT; ++peg)
        fillCircle((short)peg_x[peg], (short)peg_y[peg], PEG_RADIUS, WHITE);
}

// ball
typedef struct {
    float x, y;
    float vx, vy;
    int last_peg;
} Ball;

static Ball *balls = NULL;
static int ball_count = 0;
static int ball_capacity = 0;
static uint64_t total_fallen = 0;

static void dropBall(Ball *ball)
{
    ball->x = PEG_X;
    ball->y = BALL_RADIUS;
    // Small nonzero horizontal speed, randomized to either side.
    ball->vx = (float)(1 + rand() % 12) * 0.01f;
    if (rand() % 2)
        ball->vx = -ball->vx;
    ball->vy = 0.0f;
    ball->last_peg = -1;
}

// Allocate outside the ISR and preserve existing balls when the count changes.
static void syncBallCount(void)
{
    int requested = rotary_count;
    if (requested > ball_capacity) {
        Ball *resized = NULL;
        if ((size_t)requested <= SIZE_MAX / sizeof(*balls))
            resized = realloc(balls, (size_t)requested * sizeof(*balls));
        if (resized == NULL) {
            // Keep the displayed and requested counts consistent if memory is full.
            uint32_t irq_state = save_and_disable_interrupts();
            if (rotary_count == requested) rotary_count = ball_count;
            restore_interrupts(irq_state);
            return;
        }
        balls = resized;
        ball_capacity = requested;
    }
    for (int i = ball_count; i < requested; ++i)
        dropBall(&balls[i]);
    ball_count = requested;
}

static void updateBall(Ball *ball)
{
    ball->x += ball->vx;
    ball->y += ball->vy;

    for (int peg = 0; peg < PEG_COUNT; ++peg) {
        float dx = ball->x - peg_x[peg];
        float dy = ball->y - peg_y[peg];
        float collision_distance = BALL_RADIUS + PEG_RADIUS;
        if (fabsf(dx) < collision_distance && fabsf(dy) < collision_distance)
        {
            float distance = sqrtf(dx * dx + dy * dy);
            if (distance < collision_distance)
            {
                // If centers coincide, choose an upward normal to avoid dividing by zero.
                float normal_x = distance > 0.0001f ? dx / distance : 0.0f;
                float normal_y = distance > 0.0001f ? dy / distance : -1.0f;
                float dot = normal_x * ball->vx + normal_y * ball->vy;
                ball->x = peg_x[peg] + normal_x * (collision_distance + 1.0f);
                ball->y = peg_y[peg] + normal_y * (collision_distance + 1.0f);
                // Only reflect when moving toward the peg, not away from it.
                if (dot < 0.0f)
                {
                    float intermediate_term = -2.0f * dot;
                    ball->vx += normal_x * intermediate_term;
                    ball->vy += normal_y * intermediate_term;
                    // Sound and damping only when this ball hits a different peg.
                    if (ball->last_peg != peg)
                    {
                        ++peg_sound_events;
                        ball->vx *= BOUNCINESS;
                        ball->vy *= BOUNCINESS;
                        ball->last_peg = peg;
                    }
                }
            }
        }
    }

    // Respawn this ball after it falls completely below the screen.
    if (ball->y > 480 + BALL_RADIUS)
    {
        ++total_fallen;
        dropBall(ball);
        return; // Keep the new drop's initial y-velocity at zero.
    }
    ball->vy += GRAVITY;
}

// draw the stats: # balls being animated, total # of balls fallen since reset, and time since boot
static void drawStats(void)
{
    char text[80];
    uint64_t seconds = time_us_64() / 1000000u;
    setTextSize(2);
    setTextColor(WHITE);
    setCursor(10, 400);
    snprintf(text, sizeof(text), "Balls being animated: %d", ball_count);
    writeString(text);
    setCursor(10, 420);
    snprintf(text, sizeof(text), "Total fallen since reset: %llu",
             (unsigned long long)total_fallen);
    writeString(text);
    setCursor(10, 440);
    snprintf(text, sizeof(text), "Time since boot: %llu:%02u:%02u",
             (unsigned long long)(seconds / 3600),
             (unsigned)((seconds / 60) % 60), (unsigned)(seconds % 60));
    writeString(text);
}

// Animation on core 0
static PT_THREAD(protothread_anim(struct pt *pt))
{
    // Mark beginning of thread
    PT_BEGIN(pt);

    // Start with an empty board; the encoder adds and removes balls.
    initPegs();

    while (1)
    {
        // Wait for the signal that the buffer's changed
        PT_YIELD_UNTIL(pt, draw_start_signal());
        syncBallCount();
        // Clear the buffer
        clearLowFrame(0, BLACK);
        drawPegs();
        for (int i = 0; i < ball_count; ++i) {
            updateBall(&balls[i]);
            // Avoid drawing off-screen coordinates while a ball falls past the sides.
            if (balls[i].x >= -BALL_RADIUS && balls[i].x <= 640 + BALL_RADIUS &&
                balls[i].y >= -BALL_RADIUS && balls[i].y <= 480 + BALL_RADIUS)
                fillCircle((short)balls[i].x, (short)balls[i].y, BALL_RADIUS, color);
        }
        drawStats();
        // NEVER exit while
    } // END WHILE(1)
    PT_END(pt);
} // animation thread

// Animation on core 1
static PT_THREAD(protothread_anim1(struct pt *pt))
{
    // Mark beginning of thread
    PT_BEGIN(pt);

    // Spawn a boid
    spawnBoid(&boid1_x, &boid1_y, &boid1_vx, &boid1_vy, 1);

    while (1)
    {
        // Wait for the signal from core 0
        PT_SEM_SDK_WAIT(pt, &draw_semaphore);
        // Update position without wall collisions (inactive original demo).
        boid1_x += boid1_vx;
        boid1_y += boid1_vy;
        // draw the boid at its new position
        fillCircle(fix2int15(boid1_x), fix2int15(boid1_y), 15, color);
        // NEVER exit while
    } // END WHILE(1)
    PT_END(pt);
} // animation thread

// ========================================
// === core 1 main -- started in main below
// ========================================
void core1_main()
{
    // Add animation thread
    pt_add_thread(protothread_anim1);
    // Start the scheduler
    pt_schedule_start;
}

// ========================================
// === main
// ========================================
// USE ONLY C-sdk library
int main()
{
    set_sys_clock_khz(150000, true);
    // initialize stio
    stdio_init_all();

    // initialize VGA
    initVGA();

    // Initialize the semaphore
    // Arguments: pointer to sem, initial count, max count
    sem_init(&draw_semaphore, 0, 1);

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

    // One ball is drawn on core 0; leave the original core 1 demo inactive.
    srand(time_us_32());

    // add threads
    pt_add_thread(protothread_sound);
    pt_add_thread(protothread_anim);

    // start scheduler
    pt_schedule_start;
}
