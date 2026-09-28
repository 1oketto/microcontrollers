
/*
cool stuff happening here
*/

#include "hardware/gpio.h"
#include "hardware/timer.h"
#include "hardware/adc.h"
#include "pico/stdlib.h"
#include <stdio.h>
#include <string.h>
#include "stdlib.h"

// ==========================================
// === protothreads globals
// ==========================================
// protothreads header
#include "pt_cornell_rp2040_v1_4.h"

#define LED_PIN 25
#define ADC_PIN 26
#define ADC_MUX 0

// ==========================================
// === timer interrupt DDS demo setup + globals
// ==========================================

#include <math.h>
#include "hardware/irq.h"
#include "hardware/spi.h"

// Low-level alarm infrastructure we'll be using
#define ALARM_NUM 0
#define ALARM_IRQ timer_hardware_alarm_get_irq_num(timer_hw, ALARM_NUM)

// DDS parameters
#define two32 4294967296.0 // 2^32
#define Fs 50000
#define DELAY 20 // 1/Fs (in microseconds)
// the DDS units:
volatile unsigned int phase_accum_main;
volatile unsigned int phase_incr_main = (800.0 * two32) / Fs;

// SPI data
uint16_t DAC_data; // output value

// DAC parameters (we are using channel B of DAC)
//  A-channel, 1x, active
#define DAC_config_chan_A 0b0011000000000000
// B-channel, 1x, active
#define DAC_config_chan_B 0b1011000000000000

// SPI configurations
#define PIN_MISO 4
#define PIN_CS 5
#define PIN_SCK 6
#define PIN_MOSI 7
#define SPI_PORT spi0

// GPIO for timing the ISR
#define ISR_GPIO 2

// DDS sine table
#define sine_table_size 256
volatile int sin_table[sine_table_size];

// ==========================================
// === keypad demo setup + globals
// ==========================================
// Keypad pin configurations
#define BASE_KEYPAD_PIN 9
#define KEYROWS 4
#define NUMKEYS 12

unsigned int keycodes[NUMKEYS] = {0x57, 0x6E, 0x5E, 0x3E, 0x6D,
                                  0x5D, 0x3D, 0x6B, 0x5B, 0x3B,
                                  0x67, 0x37};
unsigned int scancodes[KEYROWS] = {0xE, 0xD, 0xB, 0x7};
unsigned int button = 0x70;

char keytext[40];
int prev_key = -1;

// ==========================================
// === additional globals added
// ==========================================
volatile float amplitude = 1.0f;
volatile int mute = 0;

// BEEP globals for record mode
#define BEEP_RAMP_SAMPLES 250u
static volatile bool beep_enabled = false;
static unsigned int beep_position = 0;
static unsigned int beep_phase = 0;

typedef enum
{
    STATE_NOT_PRESSED,
    STATE_MAYBE_PRESSED,
    STATE_PRESSED,
    STATE_MAYBE_NOT_PRESSED
} debounce_state_t;

typedef enum {
    MODE_NORMAL,
    MODE_READY,
    MODE_RECORDING,
    MODE_PLAYBACK,
    MODE_COMPOSING,
    MODE_CBACK
} synth_mode_t;

// globals shared by the 10 ms recording/keypad thread and the 1 ms playback thread.
static synth_mode_t mode = MODE_NORMAL; // default mode
static uint32_t recordings[10][10000]; // first index is keypad # 1-9 and second is up to 10 seconds of recording
static unsigned int recording_length[10] = {0}; // keeps track of current length of recording
static int active_key = -1; // active key being recorded/playbacked
static unsigned int playback_index = 0;

// Composition stores up to 50 keypad digits whose recordings will play in order.
static unsigned int composition[50]; // composition buffer
static unsigned int composing_length = 0;
static unsigned int cback_index = 0;

// ==================================================
// === Alarm ISR
//  this is where we update the DAC + make the beeps
// ==================================================

static void alarm_irq(void)
{

    // Assert a GPIO when we enter the interrupt
    gpio_put(ISR_GPIO, 1);

    // Clear the alarm irq
    hw_clear_bits(&timer_hw->intr, 1u << ALARM_NUM);

    // Reset the alarm register
    timer_hw->alarm[ALARM_NUM] = timer_hw->timerawl + DELAY;

    // DDS phase and sine table lookup
    phase_accum_main += phase_incr_main;
    int sample = sin_table[phase_accum_main >> 24]; // changed so that we don't immediately send to DAC

    // BEEP BEEP BEEP
    if (beep_enabled)
    {
        // Ramp up for 5 ms, then sustain until a digit starts recording.
        if (beep_position < BEEP_RAMP_SAMPLES) beep_position++;
        beep_phase += (unsigned int)((400.0 * two32) / Fs);
        sample = sin_table[beep_phase >> 24] * (int)beep_position / BEEP_RAMP_SAMPLES;
    } else {
        beep_position = 0;
        beep_phase = 0;
    }
    int dac_value = 2048 + (int)(amplitude * sample);
    DAC_data = DAC_config_chan_B | (dac_value & 0x0FFF); // now send to DAC channel

    // Perform an SPI transaction
    spi_write16_blocking(SPI_PORT, &DAC_data, 1);

    // De-assert the GPIO when we leave the interrupt
    gpio_put(ISR_GPIO, 0);

}


// ==================================================
// === toggle25 thread
//  here we read from ADC + handle different states
// ==================================================

static PT_THREAD(protothread_toggle25(struct pt *pt))
{
    PT_BEGIN(pt);

    static unsigned int adc_val;

    // Additional variables !!!
    static int i;
    static uint32_t keypad;

    static debounce_state_t debounce_state = STATE_NOT_PRESSED;
    static int possible = -1;
    static int pressed_key, released_key;
    static uint32_t live_frequency;

    while (1)
    {
        // toggle gpio 25
        gpio_put(LED_PIN, !gpio_get(LED_PIN));

        // Read the ADC
        adc_val = adc_read();

        // additional variables
        live_frequency = (uint32_t)((2.43 * adc_val * two32) / Fs);
        pressed_key = released_key = -1;

        // Scan the keypad!
        for (i = 0; i < KEYROWS; i++)
        {
            // Set a row low
            gpio_put_masked((0xF << BASE_KEYPAD_PIN),
                            (scancodes[i] << BASE_KEYPAD_PIN));
            // Small delay required
            sleep_us(1);
            // Read the keycode
            keypad = ((gpio_get_all() >> BASE_KEYPAD_PIN) & 0x7F);
            // Break if button(s) are pressed
            if ((~keypad) & button)
                break;
        }
        // If we found a button . . .
        if ((~keypad) & button)
        {
            // Look for a valid keycode.
            for (i = 0; i < NUMKEYS; i++) {
                // if we find a key then remember it in prev_key
                if (keypad == keycodes[i]) { prev_key = i; break; }
            }
            // If we don't find one, report invalid keycode + set prev_key to none (-1)
            if (i == NUMKEYS) { i = -1; prev_key = -1; }
        }
        // Otherwise, indicate invalid/non-pressed buttons
        else { i = -1; prev_key = -1; }

        // ==================================================
        // === DEBOUNCING STATE MACHINE
        // ==================================================
        switch (debounce_state)
        {
        case STATE_NOT_PRESSED:
            if (prev_key != -1) {
                possible = prev_key;
                debounce_state = STATE_MAYBE_PRESSED;
            }
            break;
        case STATE_MAYBE_PRESSED:
            if (prev_key == possible) {
                pressed_key = possible;
                debounce_state = STATE_PRESSED;
            } else {
                debounce_state = STATE_NOT_PRESSED;
            }
            break;
        case STATE_PRESSED:
            if (prev_key != possible)
                debounce_state = STATE_MAYBE_NOT_PRESSED;
            break;
        case STATE_MAYBE_NOT_PRESSED:
            if (prev_key == possible) {
                debounce_state = STATE_PRESSED;
            } else {
                released_key = possible;
                debounce_state = STATE_NOT_PRESSED;
            }
            break;
        }

        // ==================================================
        // === SYNTH MODE LOGIC HANDLING IDK
        // ==================================================

        // mute, record, compose key presses
        if (pressed_key == 0) {
            mute = !mute;
            amplitude = mute ? 0.0f : 1.0f;
        } else if (pressed_key == 10) {
            mode = MODE_READY;
            active_key = -1;
            beep_enabled = true;
        } else if (pressed_key == 11) {
            if (mode == MODE_COMPOSING) {
                cback_index = 0;
                playback_index = 0;
                active_key = -1;
                mode = composing_length > 0 ? MODE_CBACK : MODE_NORMAL; // if we have composition ready, then mode is cback
            } else {
                composing_length = 0;
                cback_index = 0;
                active_key = -1;
                beep_enabled = false;
                mode = MODE_COMPOSING;
            }
        }

        // ready to record -> recording
        if (mode == MODE_READY && pressed_key >= 1 && pressed_key <= 9) {
            beep_enabled = false;
            active_key = pressed_key;
            recording_length[active_key] = 0;
            mode = MODE_RECORDING;
        // initiate playbacking when key is pressed (not in ready mode)
        } else if ((mode == MODE_NORMAL || mode == MODE_PLAYBACK || mode == MODE_COMPOSING) &&
                   pressed_key >= 1 && pressed_key <= 9 &&
                   recording_length[pressed_key] > 0) {
            active_key = pressed_key;
            playback_index = 0;
            if (mode != MODE_COMPOSING) mode = MODE_PLAYBACK; // differentiate?
        }

        // when recording, update array with frequencies
        if (mode == MODE_RECORDING && prev_key == active_key) {
            recordings[active_key][recording_length[active_key]++] = live_frequency;
            // full buffer
            if (recording_length[active_key] == 10000) {
                mode = MODE_NORMAL;
                active_key = -1;
            }
        }
        // end recording once key is released
        if (mode == MODE_RECORDING && released_key == active_key) {
            mode = MODE_NORMAL;
            active_key = -1;
        }

        // keep the playback value until the next recorded sample is due
        if (mode != MODE_PLAYBACK && mode != MODE_CBACK && !(mode == MODE_COMPOSING && active_key != -1)) {
            phase_incr_main = live_frequency;
        }

        // only add key to composition if composition buffer has room
        if (mode == MODE_COMPOSING && pressed_key >= 1 && pressed_key <= 9 &&
            recording_length[pressed_key] > 0 && composing_length < 50) {
            composition[composing_length++] = (unsigned int)pressed_key; // idk why we need cast
        }

        PT_YIELD_usec(10000);

    } // END WHILE(1)
    // every thread ends with PT_END(pt);
    PT_END(pt);
} // end blink thread

// ==================================================
// === PLAYBACK THREAD
//  plays back 10x faster :)
// ==================================================
static PT_THREAD(protothread_playback(struct pt *pt))
{
    PT_BEGIN(pt);

    while (1) {
        // if composition is ready, go through composition array and set active key 
        if (mode == MODE_CBACK && active_key == -1) {
            if (cback_index < composing_length) {
                active_key = (int)composition[cback_index++]; // idk why we need cast
                playback_index = 0;
            } else {
                mode = MODE_NORMAL;
            }
        }
        // playback
        if (mode == MODE_PLAYBACK || mode == MODE_CBACK || (mode == MODE_COMPOSING && active_key != -1)) {
            if (active_key < 1 || active_key > 9) {
                if (mode != MODE_COMPOSING) mode = MODE_NORMAL;
                active_key = -1;
            } else if (playback_index < recording_length[active_key]) {
                phase_incr_main = recordings[active_key][playback_index++];
            // if we are playing back composition, set active key to none (so it can be set again)
            } else if (mode == MODE_CBACK) {
                active_key = -1;
            // if we are composing still, set active key to none
            } else if (mode == MODE_COMPOSING) {
                active_key = -1;
            } else {
                mode = MODE_NORMAL;
                active_key = -1;
            }
        }
        PT_YIELD_usec(1250);
    }

    PT_END(pt);
}

// ========================================
// === core 0 main
// ========================================

int main()
{
    // Initialize stdio
    stdio_init_all();

    // Initialize SPI channel (channel, baud rate set to 20MHz)
    spi_init(SPI_PORT, 20000000);
    // Format (channel, data bits per transfer, polarity, phase, order)
    spi_set_format(SPI_PORT, 16, 0, 0, 0);

    // Setup the ISR-timing GPIO
    gpio_init(ISR_GPIO);
    gpio_set_dir(ISR_GPIO, GPIO_OUT);
    gpio_put(ISR_GPIO, 0);

    // Map SPI signals to GPIO ports
    gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
    gpio_set_function(PIN_SCK, GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);
    gpio_set_function(PIN_CS, GPIO_FUNC_SPI);

    // === build the sine lookup table =======
    // scaled to produce values between 0 and 4096
    int ii;
    for (ii = 0; ii < sine_table_size; ii++)
    {
        sin_table[ii] = (int)(2047 * sin((float)ii * 6.283 / (float)sine_table_size));
    }

    // Enable the interrupt for the alarm (we're using Alarm 0)
    hw_set_bits(&timer_hw->inte, 1u << ALARM_NUM);
    // Associate an interrupt handler with the ALARM_IRQ
    irq_set_exclusive_handler(ALARM_IRQ, alarm_irq);
    // Enable the alarm interrupt
    irq_set_enabled(ALARM_IRQ, true);
    // Write the lower 32 bits of the target time to the alarm register, arming it.
    timer_hw->alarm[ALARM_NUM] = timer_hw->timerawl + DELAY;

    // set up LED gpio 25
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    gpio_put(LED_PIN, true);

    // Setup the ADC
    adc_init();
    adc_gpio_init(ADC_PIN);
    adc_select_input(ADC_MUX);

    ////////////////// KEYPAD INITS ///////////////////////
    // Initialize the keypad GPIO's
    gpio_init_mask((0x7F << BASE_KEYPAD_PIN));
    gpio_set_dir((BASE_KEYPAD_PIN + 4), GPIO_IN);
    gpio_set_dir((BASE_KEYPAD_PIN + 5), GPIO_IN);
    gpio_set_dir((BASE_KEYPAD_PIN + 6), GPIO_IN);
    // Set row-pins to output
    gpio_set_dir_out_masked((0xF << BASE_KEYPAD_PIN));
    // Set all output pins to high
    gpio_put_masked((0xF << BASE_KEYPAD_PIN), (0xF << BASE_KEYPAD_PIN));
    // Turn on pullup resistors for column pins
    gpio_pull_up((BASE_KEYPAD_PIN + 4));
    gpio_pull_up((BASE_KEYPAD_PIN + 5));
    gpio_pull_up((BASE_KEYPAD_PIN + 6));

    // === config threads ========================
    pt_add_thread(protothread_toggle25);
    pt_add_thread(protothread_playback); // config playback thread

    // === initalize the scheduler ===============
    pt_schedule_start;
}
