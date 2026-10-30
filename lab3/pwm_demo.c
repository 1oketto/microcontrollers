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
 * the vga display shows two full-width plots:
 * top = beam angle, bottom = low-passed motor command signal
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

// TUNING DEFAULTS: edit and rebuild, or set gains/target/bias over serial.
volatile float Kp = 50.0f;            // PWM counts per degree of angle error
volatile float pwm_bias = 2000.0f;      // Baseline motor command (0..MOTOR_PWM_MAX)
volatile float desired_angle = 90.0f; // Target: 0 stops motor; otherwise 0..180 degrees
// Stored serial gains: unused while control is proportional-only.
volatile float Ki = 0.0f, Kd = 0.0f;
#define ACCEL_FILTER_SHIFT 4        // Accel smoothing: 1/16 per sample
#define MOTOR_FILTER_SHIFT 6        // VGA motor-command smoo   thing: 1/64 per sample
// 150 MHz / (25 * 6000) = 1 kHz. PWM counter runs from 0 to WRAPVAL-1.
#define MOTOR_PWM_MAX 4000          // Motor command limit; PWM period stays 6000
#define WRAPVAL 6000
#define CLKDIV 25.0f

// both checkpoint plots span the screen from column 81 to 630
#define LEFT_EDGE 81
#define PLOT_HEIGHT 150

// x/y accelerometer components and rotation about sensor z
#define PI 3.14159265358979323846

// Place the plots
#define TOP_PLOT_BOTTOM 230
#define BOTTOM_PLOT_BOTTOM 430
#define PLOT_WIDTH 550  // about 5.5 seconds at 100 saved samples/sec

// Arrays in which raw measurements will be stored
fix15 acceleration[3], gyro[3];

// circular display buffers for the two checkpoint signals
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
    float target = desired_angle;
    if (target <= 0.0f) return 0.0f; // Explicit motor-off command, regardless of bias.
    float error = target - angle;
    float output = pwm_bias + Kp * error;
    return fmaxf(0.0f, fminf(MOTOR_PWM_MAX, output));
}

void drawAxes() {
    // Draw the static aspects of the display
    setTextSize(1) ;
    setTextColor(WHITE);

    // Live settings header; refreshed along with the plots each frame.
    snprintf(screentext, sizeof(screentext), "Kp: %.4f", Kp);
    setCursor(10, 10); writeString(screentext);
    snprintf(screentext, sizeof(screentext), "Ki: %.4f", Ki);
    setCursor(210, 10); writeString(screentext);
    snprintf(screentext, sizeof(screentext), "Kd: %.4f", Kd);
    setCursor(410, 10); writeString(screentext);
    snprintf(screentext, sizeof(screentext), "Target: %.2f deg", desired_angle);
    setCursor(10, 28); writeString(screentext);
    snprintf(screentext, sizeof(screentext), "Bias: %.0f PWM", pwm_bias);
    setCursor(210, 28); writeString(screentext);
    setCursor(410, 28); writeString("P-only: Ki/Kd inactive");

    // full-width checkpoint plots
    setCursor(82, 60) ; writeString("Beam angle (deg)") ;
    setCursor(82, 260) ; writeString("Filtered command (PWM)") ;

    // top plot: beam angle, 0..180 degrees
    drawHLine(75, 230, 5, CYAN) ;
    drawHLine(75, 155, 5, CYAN) ;
    drawHLine(75, 80, 5, CYAN) ;
    drawVLine(80, 80, PLOT_HEIGHT, CYAN) ;
    setCursor(45, 225) ; writeString("0") ;
    setCursor(45, 150) ; writeString("90") ;
    setCursor(45, 75) ; writeString("180") ;

    // bottom plot: low-passed motor command, 0..4000 counts
    drawHLine(75, 430, 5, CYAN) ;
    drawHLine(75, 355, 5, CYAN) ;
    drawHLine(75, 280, 5, CYAN) ;
    drawVLine(80, 280, PLOT_HEIGHT, CYAN) ;
    setCursor(45, 425) ; writeString("0") ;
    setCursor(45, 350) ; writeString("2000") ;
    setCursor(45, 275) ; writeString("4000") ;
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
            // keep out-of-range angle samples from drawing over the header
            float angle = fix2float15(angle_history[start]);
            if (angle >= 0.0f && angle <= 180.0f) {
                drawPixel(LEFT_EDGE + i, (TOP_PLOT_BOTTOM -
                    (int)(PLOT_HEIGHT * angle / 180.0f)), WHITE) ;
            }
            drawPixel(LEFT_EDGE + i, (BOTTOM_PLOT_BOTTOM -
                (int)(PLOT_HEIGHT * (float)motor_history[start] / MOTOR_PWM_MAX)), GREEN) ;
            start = ((start+1)==PLOT_WIDTH) ? 0 : (start+1) ;
        }
    }

    PT_END(pt) ;
}

//
// THREAD 1: USER SERIAL INPUT
//
static void handle_serial_command(void) {
    char extra;
    float kp, ki, kd, target, bias;
    snprintf(pt_serial_out_buffer, sizeof(pt_serial_out_buffer),
             "Invalid input. Use: kp ki kd target bias. Gains: 0..1000000; target: 0..180; bias: integer 0..%d.\r\n",
             MOTOR_PWM_MAX);
    if (memchr(pt_serial_in_buffer, '\0', sizeof(pt_serial_in_buffer)) == NULL) return;
    if (sscanf(pt_serial_in_buffer, "%f %f %f %f %f %c",
               &kp, &ki, &kd, &target, &bias, &extra) != 5) {
        if (sscanf(pt_serial_in_buffer, "%f %c", &target, &extra) != 1 || target != 0.0f) return;
        uint32_t irq_state = save_and_disable_interrupts();
        desired_angle = 0.0f;
        restore_interrupts(irq_state);
        snprintf(pt_serial_out_buffer, sizeof(pt_serial_out_buffer), "Motor off\r\n");
        return;
    }
    if (!isfinite(kp) || kp < 0.0f || kp > 1000000.0f ||
        !isfinite(ki) || ki < 0.0f || ki > 1000000.0f ||
        !isfinite(kd) || kd < 0.0f || kd > 1000000.0f ||
        !isfinite(target) || target < 0.0f || target > 180.0f ||
        !isfinite(bias) || bias < 0.0f || bias > MOTOR_PWM_MAX || bias != truncf(bias)) return;

    uint32_t irq_state = save_and_disable_interrupts();
    Kp = kp;
    Ki = ki;
    Kd = kd;
    desired_angle = target;
    pwm_bias = bias;
    restore_interrupts(irq_state);
    snprintf(pt_serial_out_buffer, sizeof(pt_serial_out_buffer),
             "Kp=%.4f Ki=%.4f Kd=%.4f target=%.4f bias=%.0f (Ki/Kd inactive)%s\r\n",
             kp, ki, kd, target, bias, target == 0.0f ? " (motor off)" : "");
}

static PT_THREAD (protothread_serial(struct pt *pt))
{
    PT_BEGIN(pt) ;
    while(1) {
        snprintf(pt_serial_out_buffer, sizeof(pt_serial_out_buffer),
                 "kp ki kd target bias (space-separated), or 0 = motor off: ");
        serial_write ;
        serial_read ;
        handle_serial_command();
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
    // start at the configured PWM bias.
    old_control = (int)pwm_bias;
    pwm_set_chan_level(slice_num, PWM_CHAN_A, old_control);

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
