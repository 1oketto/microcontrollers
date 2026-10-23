/**
 * V. Hunter Adams (vha3@cornell.edu)
 * PWM demo code with serial input
 * 
 * This demonstration sets a PWM duty cycle to a
 * user-specified value.
 * 
 * HARDWARE CONNECTIONS
 *   - GPIO 4 ---> PWM output
 * 
 * (from imu_demo.c) This demonstration utilizes the MPU6050.
 * It gathers raw accelerometer/gyro measurements, scales
 * them, and plots them to the VGA display. The top plot
 * shows gyro measurements, bottom plot shows accelerometer
 * measurements.
 * 
 * on the right: top = beam angle, bottom = slow-passed motor command signal
 * 
 * HARDWARE CONNECTIONS
 *  - GPIO 16 ---> VGA Hsync
 *  - GPIO 17 ---> VGA Vsync
 *  - GPIO 18 ---> 470 ohm resistor ---> VGA Green
 *  - GPIO 19 ---> 330 ohm resistor ---> VGA Green
 *  - GPIO 20 ---> 330 ohm resistor ---> VGA Blue
 *  - GPIO 21 ---> 330 ohm resistor ---> VGA Red
 *  - RP2040 GND ---> VGA GND
 *  - GPIO 8 ---> MPU6050 SDA
 *  - GPIO 9 ---> MPU6050 SCL
 *  - 3.3v ---> MPU6050 VCC
 *  - RP2040 GND ---> MPU6050 GND
 */


#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

#include "pico/stdlib.h"
#include "pico/multicore.h"

// Include hardware libraries
#include "hardware/pwm.h"
#include "hardware/irq.h"

// include custom libraries and additional imu/vga hardware libraries
#include "hardware/dma.h"
#include "hardware/adc.h"
#include "hardware/pio.h"
#include "hardware/i2c.h"
#include "hardware/clocks.h"
#include "hardware/sync.h"
#include "VGA/vga16_graphics_v3.h"
#include "mpu6050.h"
#include "pt_cornell_rp2040_v1_4.h"

// TUNING PARAMETERS: edit these values, then rebuild and flash.
volatile float Kp = 1.0f;            // PWM counts per degree of angle error
volatile float pwm_bias = 0.0f;      // Baseline motor command (0..6000)
volatile float desired_angle = 180.0f; // Startup target; serial input updates it
// Reserved gains: unused while control is proportional-only.
volatile float Ki = 0.0f, Kd = 0.0f;
#define ACCEL_FILTER_SHIFT 4        // Accel smoothing: 1/16 per sample
#define MOTOR_FILTER_SHIFT 6        // VGA motor-command smoothing: 1/64 per sample
// 150 MHz / (25 * 6000) = 1 kHz. PWM counter runs from 0 to WRAPVAL-1.
#define WRAPVAL 6000
#define CLKDIV 25.0f

// We will start drawing at column 81
#define LEFT_EDGE 81 
// place the checkpoint graphs in the right half of the display
#define RIGHT_EDGE 401

// Rescale the measurements for display
#define OLD_RANGE 500.  // (+/- 250)
#define NEW_RANGE 150.  // (looks nice on VGA)
#define OLD_MIN -250. 
#define OLD_MAX 250. 
#define ACCEL_SCALE 120.0

// x/y accelerometer components and rotation about sensor z
#define PI 3.14159265358979323846

// Place the plots
#define TOP_PLOT_BOTTOM 230
#define BOTTOM_PLOT_BOTTOM 430
#define PLOT_WIDTH 230  // each column shows about 2.3 seconds at 100 saved samples/sec

// Arrays in which raw measurements will be stored
fix15 acceleration[3], gyro[3];

// Arrays for circular display buffer
fix15 ax[PLOT_WIDTH] ;
fix15 ay[PLOT_WIDTH] ;
fix15 az[PLOT_WIDTH] ;
fix15 gx[PLOT_WIDTH] ;
fix15 gy[PLOT_WIDTH] ;
fix15 gz[PLOT_WIDTH] ;

fix15 angle_history[PLOT_WIDTH] ;
int motor_history[PLOT_WIDTH] ;
// 15.16 fixed-point angle representation
fix15 filtered_ax, filtered_ay ;
fix15 accel_angle, gyro_angle_delta, complementary_angle ;
// signed state persists between interrupts
static int motor_disp = 0 ;

// Pointer to head of buffer
volatile int head = 0 ;

// character array
char screentext[40];

// draw speed
volatile int threshold = 10 ;
volatile int throttle = 10 ;

// GPIO we're using for PWM
#define PWM_OUT 4

// Variable to hold PWM slice number
uint slice_num ;

volatile int old_control ;

static float calculate_control(float angle) {
    float error = desired_angle - angle;
    float output = pwm_bias + Kp * error;
    return fmaxf(0.0f, fminf(WRAPVAL, output));
}

void drawAxes() {
    // Draw the static aspects of the display
    setTextSize(1) ;
    setTextColor(WHITE);

    // titles for the original sensor plots and the checkpoint plots
    setCursor(82, 60) ; writeString("Gyro (deg/s)") ;
    setCursor(82, 260) ; writeString("Accel (g)") ;
    setCursor(402, 60) ; writeString("Beam angle (deg)") ;
    setCursor(402, 260) ; writeString("Filtered command (PWM)") ;

    // Draw bottom plot
    drawHLine(75, 430, 5, CYAN) ;
    drawHLine(75, 355, 5, CYAN) ;
    drawHLine(75, 280, 5, CYAN) ;
    drawVLine(80, 280, 150, CYAN) ;
    sprintf(screentext, "0") ;
    setCursor(50, 350) ;
    writeString(screentext) ;
    sprintf(screentext, "+2") ;
    setCursor(50, 280) ;
    writeString(screentext) ;
    sprintf(screentext, "-2") ;
    setCursor(50, 425) ;
    writeString(screentext) ;

    // Draw top plot
    drawHLine(75, 230, 5, CYAN) ;
    drawHLine(75, 155, 5, CYAN) ;
    drawHLine(75, 80, 5, CYAN) ;
    drawVLine(80, 80, 150, CYAN) ;
    sprintf(screentext, "0") ;
    setCursor(50, 150) ;
    writeString(screentext) ;
    sprintf(screentext, "+250") ;
    setCursor(45, 75) ;
    writeString(screentext) ;
    sprintf(screentext, "-250") ;
    setCursor(45, 225) ;
    writeString(screentext) ;

    // top-right plot: complementary angle using 0..360 convention
    drawHLine(395, 230, 5, CYAN) ;
    drawHLine(395, 155, 5, CYAN) ;
    drawHLine(395, 80, 5, CYAN) ;
    drawVLine(400, 80, 150, CYAN) ;
    setCursor(365, 225) ; writeString("0") ;
    setCursor(365, 150) ; writeString("180") ;
    setCursor(365, 75) ; writeString("360") ;

    // bottom-right plot: software-filtered motor command, 0..6000 counts
    drawHLine(395, 430, 5, CYAN) ;
    drawHLine(395, 355, 5, CYAN) ;
    drawHLine(395, 280, 5, CYAN) ;
    drawVLine(400, 280, 150, CYAN) ;
    setCursor(365, 425) ; writeString("0") ;
    setCursor(365, 350) ; writeString("3000") ;
    setCursor(365, 275) ; writeString("6000") ;
}

//
// Motor control ISR: uses a PWM interrupt
// 
void on_pwm_wrap() {
    // 1. Clear the interrupt flag that brought us here
    pwm_clear_irq(pwm_gpio_to_slice_num(PWM_OUT));
    // 2. Read the IMU
    // NOTE! This is in 15.16 fixed point. Accel in g's, gyro in deg/s
    // If you want these values in floating point, call fix2float15() on
    // the raw measurements.
    mpu6050_read_raw(acceleration, gyro);

    // 3. low-pass the accelerometer measurements before calculating angle
    filtered_ax = filtered_ax + ((acceleration[0] - filtered_ax)>>ACCEL_FILTER_SHIFT);
    filtered_ay = filtered_ay + ((acceleration[1] - filtered_ay)>>ACCEL_FILTER_SHIFT);

    // 4. estimates angle using complementary filter of accelerometer and gyro measurements
    accel_angle = multfix15(float2fix15(atan2(-filtered_ax, filtered_ay) + PI), oneeightyoverpi);
    gyro_angle_delta = multfix15(gyro[2], zeropt001) ;
    complementary_angle = multfix15(complementary_angle - gyro_angle_delta, zeropt999) + multfix15(accel_angle, zeropt001);

    // 5. proportional control at 1000 Hz: bias + Kp * angle error.
    int pwm_on_time = (int)calculate_control(fix2float15(complementary_angle));

    // 6. Sets a hardware PWM signal using output-compare unit to control the motor using the command
    // Update duty cycle
    if (pwm_on_time!=old_control) {
        old_control = pwm_on_time ;
        pwm_set_chan_level(slice_num, PWM_CHAN_A, pwm_on_time);
    }

    // low-pass the displayed motor command, every 1 khz pwm interrupt
    // Integer rounding can leave a small offset below a rising target.
    motor_disp = motor_disp + ((pwm_on_time - motor_disp)>>MOTOR_FILTER_SHIFT);

    // Increment drawspeed controller
    throttle += 1 ;
    // Maintain a circular buffer for display
    if ((throttle >= threshold)) { 
        // Zero drawspeed controller
        throttle = 0 ;

        // Replace oldest data in accel arrays with new sample
        ax[head] = acceleration[0] ;
        ay[head] = acceleration[1] ;
        az[head] = acceleration[2] ;

        // Replace oldest data in gyro arrays with new sample
        gx[head] = gyro[0] ;
        gy[head] = gyro[1] ;
        gz[head] = gyro[2] ;

        // replace oldest data with the measured angle and filtered motor command
        angle_history[head] = complementary_angle ;
        motor_history[head] = motor_disp ;

        // Move the pointer to the oldest data
        head = ((head+1) == PLOT_WIDTH) ? 0 : (head + 1) ;

    }
}

//
// THREAD 2: VGA DRAWING
//
static PT_THREAD (protothread_draw(struct pt *pt))
{
    PT_BEGIN(pt) ;

    static int start ;
    static int i ;

    // Draw axes
    drawAxes() ;
    // Copy to other buffer
    copy_buffer_to_other() ;

    while(1) {
        // Wait for a buffer swap
        PT_YIELD_UNTIL(pt, draw_start_signal()) ;
        gpio_put(15, !gpio_get(15)) ;
        
        // Clear the buffer, draw the axes
        clearLowFrame(0, BLACK) ;
        drawAxes() ;

        start = head ;
        for (i = 0; i < PLOT_WIDTH; i++) {
            drawPixel(LEFT_EDGE + i, (BOTTOM_PLOT_BOTTOM -
                (int)(NEW_RANGE*((float)((fix2float15(ax[start])*ACCEL_SCALE)-OLD_MIN)/OLD_RANGE))), WHITE) ;
            drawPixel(LEFT_EDGE + i, (BOTTOM_PLOT_BOTTOM -
                (int)(NEW_RANGE*((float)((fix2float15(ay[start])*ACCEL_SCALE)-OLD_MIN)/OLD_RANGE))), RED) ;
            drawPixel(LEFT_EDGE + i, (BOTTOM_PLOT_BOTTOM -
                (int)(NEW_RANGE*((float)((fix2float15(az[start])*ACCEL_SCALE)-OLD_MIN)/OLD_RANGE))), GREEN) ;
            drawPixel(LEFT_EDGE + i, (TOP_PLOT_BOTTOM -
                (int)(NEW_RANGE*((float)((fix2float15(gx[start]))-OLD_MIN)/OLD_RANGE))), WHITE) ;
            drawPixel(LEFT_EDGE + i, (TOP_PLOT_BOTTOM -
                (int)(NEW_RANGE*((float)((fix2float15(gy[start]))-OLD_MIN)/OLD_RANGE))), RED) ;
            drawPixel(LEFT_EDGE + i, (TOP_PLOT_BOTTOM -
                (int)(NEW_RANGE*((float)((fix2float15(gz[start]))-OLD_MIN)/OLD_RANGE))), GREEN) ;
            // draw the checkpoint signals beside the original sensor traces
            drawPixel(RIGHT_EDGE + i, (TOP_PLOT_BOTTOM -
                (int)(NEW_RANGE * fix2float15(angle_history[start]) / 360.0f)), WHITE) ;
            drawPixel(RIGHT_EDGE + i, (BOTTOM_PLOT_BOTTOM -
                (int)(NEW_RANGE * (float)motor_history[start] / WRAPVAL)), GREEN) ;
            start = ((start+1)==PLOT_WIDTH) ? 0 : (start+1) ;
        }
    }

    PT_END(pt) ;
}

//
// THREAD 1: USER SERIAL INPUT
//
static void handle_serial_angle(void) {
    float angle;
    char extra;
    snprintf(pt_serial_out_buffer, sizeof(pt_serial_out_buffer),
             "Enter an angle from 0 to 360 degrees.\r\n");
    if (memchr(pt_serial_in_buffer, '\0', sizeof(pt_serial_in_buffer)) == NULL) return;
    if (sscanf(pt_serial_in_buffer, "%f %c", &angle, &extra) != 1 ||
        !isfinite(angle) || angle < 0.0f || angle > 360.0f) return;

    uint32_t irq_state = save_and_disable_interrupts();
    desired_angle = angle;
    restore_interrupts(irq_state);
    snprintf(pt_serial_out_buffer, sizeof(pt_serial_out_buffer),
             "Target angle = %.2f degrees\r\n", angle);
}

static PT_THREAD (protothread_serial(struct pt *pt))
{
    PT_BEGIN(pt) ;
    while(1) {
        snprintf(pt_serial_out_buffer, sizeof(pt_serial_out_buffer),
                 "Target angle (0-360 degrees): ");
        serial_write ;
        serial_read ;
        handle_serial_angle();
        serial_write ;
    }
    PT_END(pt) ;
}

// Entry point for core 1
void core1_entry() {
    gpio_init(15) ;
    gpio_set_dir(15, GPIO_OUT) ;
    gpio_put(15, 0) ;
    pt_add_thread(protothread_draw) ;
    pt_schedule_start ;
}

int main() {

    // overclock cpu to 150 mhz for the vga driver
    set_sys_clock_khz(150000, true) ;

    // Initialize stdio
    stdio_init_all();

    // Initialize VGA
    initVGA() ;

    ////////////////////////////////////////////////////////////////////////
    ///////////////////////// I2C CONFIGURATION ////////////////////////////
    i2c_init(I2C_CHAN, I2C_BAUD_RATE) ;
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C) ;
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C) ;

    // MPU6050 initialization
    mpu6050_reset();
    mpu6050_read_raw(acceleration, gyro);

    // initialize the complementary angle from the accelerometer
    filtered_ax = acceleration[0];
    filtered_ay = acceleration[1];
    accel_angle = multfix15(float2fix15(atan2(-filtered_ax, filtered_ay) + PI), oneeightyoverpi);
    complementary_angle = accel_angle;

    ////////////////////////////////////////////////////////////////////////
    ///////////////////////// PWM CONFIGURATION ////////////////////////////
    ////////////////////////////////////////////////////////////////////////
    // Tell GPIO PWM_OUT that it is allocated to the PWM
    gpio_set_function(PWM_OUT, GPIO_FUNC_PWM);

    // Find out which PWM slice is connected to GPIO PWM_OUT (it's slice 2)
    slice_num = pwm_gpio_to_slice_num(PWM_OUT);

    // Mask our slice's IRQ output into the PWM block's single interrupt line,
    // and register our interrupt handler
    pwm_clear_irq(slice_num);
    pwm_set_irq_enabled(slice_num, true);
    irq_set_exclusive_handler(PWM_IRQ_WRAP, on_pwm_wrap);
    irq_set_enabled(PWM_IRQ_WRAP, true);

    // This section configures the period of the PWM signals
    // 150 mhz / (25 * 6000) = exactly 1000 pwm wraps per second.
    pwm_set_wrap(slice_num, WRAPVAL - 1) ;
    pwm_set_clkdiv(slice_num, CLKDIV) ;

    // This sets duty cycle
    // match the initially zero PWM bias and motor_disp values.
    pwm_set_chan_level(slice_num, PWM_CHAN_A, 0);

    // Start the channel
    pwm_set_mask_enabled((1u << slice_num));

    ////////////////////////////////////////////////////////////////////////
    ///////////////////////////// ROCK AND ROLL ////////////////////////////
    ////////////////////////////////////////////////////////////////////////
    // start core 1 
    multicore_reset_core1();
    multicore_launch_core1(core1_entry);

    pt_add_thread(protothread_serial) ;
    pt_schedule_start ;

}
