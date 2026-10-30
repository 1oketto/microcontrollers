/**
 * V. Hunter Adams (vha3@cornell.edu)
 * PWM demo code with serial input
 * 
 * This demonstration sets a PWM duty cycle to a
 * user-specified value.
 * 
 * HARDWARE CONNECTIONS
 *   - GPIO 4 ---> PWM output
 *   - gpio 15 ---> pushbutton ---> gnd (internal pull-up)
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

// parameters for the PID controller
volatile float Kp = 100.0f;        
volatile float desired_angle = 0.0f;    // down = -90, horizontal = 0, up = 90
volatile float Ki = 0.0f;              
volatile float Kd = 1000.0f;             
#define CONTROL_DT 0.001f
#define GYRO_FILTER_SHIFT 3             // derivative rate smoothing: 1/8 per sample
#define DEMO_BUTTON 15
#define BUTTON_DEBOUNCE_US 20000u
#define ACCEL_FILTER_SHIFT 4            // Accel smoothing: 1/16 per sample
#define MOTOR_FILTER_SHIFT 6            // VGA motor-command smoothing: 1/64 per sample
// 150 MHz / (25 * 6000) = 1 kHz. PWM counter runs from 0 to WRAPVAL-1.
#define MOTOR_PWM_MAX 4000              // Motor command limit; PWM period stays 6000
#define WRAPVAL 6000
#define CLKDIV 25.0f

// x/z accelerometer components and rotation about sensor y
// extra variables

#define PI 3.14159265358979323846

// Place the plot
#define LEFT_EDGE 81
#define PLOT_HEIGHT 150
#define TOP_PLOT_BOTTOM 230
#define BOTTOM_PLOT_BOTTOM 430
#define PLOT_WIDTH 550

// arrays where raw measurements will be stored
fix15 acceleration[3], gyro[3];

// circular display buffers for the two checkpoint signals
fix15 angle_history[PLOT_WIDTH] ;
int motor_history[PLOT_WIDTH] ;
// 15.16 fixed-point angle representation
fix15 filtered_ax, filtered_az, filtered_gy ;
fix15 accel_angle, gyro_angle_delta ;
volatile fix15 complementary_angle ;
// signed state persists between interrupts
static int motor_disp = 0 ;

// pointer to head of buffer
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

volatile bool motor_enabled = false;
volatile bool demo_active = false;
static float integral_error = 0.0f;
static bool button_candidate = false;
static bool button_stable = false;
static bool demo_armed = false;
static uint64_t button_changed_us = 0;
static uint64_t demo_start_us = 0;

static void stop_control(void) {
    motor_enabled = false;
    demo_active = false;
    demo_armed = false;
    integral_error = 0.0f;
}

static void set_manual_target(float target) {
    desired_angle = target;
    demo_active = false;
    demo_armed = false;
    motor_enabled = !button_candidate && !button_stable;
}

static void update_demo(bool pressed, uint64_t now) {
    if (pressed != button_candidate) {
        button_candidate = pressed;
        button_changed_us = now;
    }
    // stop on the first press sample; debounce before arming the sequence.
    if (pressed) {
        motor_enabled = false;
        demo_active = false;
        integral_error = 0.0f;
        desired_angle = -90.0f;
    }
    if (button_candidate != button_stable &&
        now - button_changed_us >= BUTTON_DEBOUNCE_US) {
        button_stable = button_candidate;
        if (button_stable) {
            demo_armed = true;
        } else if (demo_armed) {
            demo_armed = false;
            demo_start_us = now;
            demo_active = true;
            motor_enabled = true;
            desired_angle = 0.0f;
        }
    }
    if (demo_active) {
        uint64_t elapsed = now - demo_start_us;
        if (elapsed >= 15000000u) {
            desired_angle = 0.0f;
            demo_active = false;
        } else if (elapsed >= 10000000u) {
            desired_angle = -30.0f;
        } else if (elapsed >= 5000000u) {
            desired_angle = 30.0f;
        }
    }
}

static float calculate_control(float angle, float angular_rate) {
    if (!motor_enabled) {
        integral_error = 0.0f;
        return 0.0f;
    }
    float error = desired_angle - angle;
    float proportional = Kp * error;
    float derivative = -Kd * angular_rate;
    if (Ki > 0.0f) {
        float limit = MOTOR_PWM_MAX / Ki;
        float candidate = fmaxf(-limit, fminf(limit,
            integral_error + error * CONTROL_DT));
        float output = proportional + Ki * candidate + derivative;
        // integrate within the limits, or when the error helps leave saturation.
        if ((output >= 0.0f && output <= MOTOR_PWM_MAX) ||
            (output > MOTOR_PWM_MAX && error < 0.0f) ||
            (output < 0.0f && error > 0.0f)) {
            integral_error = candidate;
        }
    } else {
        integral_error = 0.0f;
    }
    float output = proportional + Ki * integral_error + derivative;
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
    snprintf(screentext, sizeof(screentext), "Angle: %.2f deg", fix2float15(complementary_angle));
    setCursor(210, 28); writeString(screentext);
    snprintf(screentext, sizeof(screentext), "Mode: %s",
             !motor_enabled ? "OFF" : (demo_active ? "DEMO" : "HOLD"));
    setCursor(410, 28); writeString(screentext);

    // full-width checkpoint plots
    setCursor(82, 60) ; writeString("Beam angle (deg)") ;
    setCursor(82, 260) ; writeString("Filtered command (PWM)") ;

    // top plot: beam angle, -90..90 degrees
    drawHLine(75, 230, 5, CYAN) ;
    drawHLine(75, 155, 5, CYAN) ;
    drawHLine(75, 80, 5, CYAN) ;
    drawVLine(80, 80, PLOT_HEIGHT, CYAN) ;
    setCursor(45, 225) ; writeString("-90") ;
    setCursor(45, 150) ; writeString("0") ;
    setCursor(45, 75) ; writeString("90") ;

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
    update_demo(!gpio_get(DEMO_BUTTON), time_us_64());
    // 2. Read the IMU
    // NOTE! This is in 15.16 fixed point. Accel in g's, gyro in deg/s
    // If you want these values in floating point, call fix2float15() on
    // the raw measurements.
    if (!mpu6050_read_raw(acceleration, gyro)) {
        stop_control();
        old_control = 0;
        pwm_set_chan_level(slice_num, PWM_CHAN_A, 0);
        return;
    }

    // 3. low-pass the accelerometer measurements before calculating angle
    filtered_ax = filtered_ax + ((acceleration[0] - filtered_ax)>>ACCEL_FILTER_SHIFT);
    filtered_az = filtered_az + ((acceleration[2] - filtered_az)>>ACCEL_FILTER_SHIFT);
    filtered_gy = filtered_gy + ((gyro[1] - filtered_gy)>>GYRO_FILTER_SHIFT);

    // 4. estimates angle using complementary filter of accelerometer and gyro measurements
    accel_angle = multfix15(float2fix15(atan2(-filtered_ax, filtered_az)), oneeightyoverpi);
    gyro_angle_delta = gyro[1] / 1000;
    complementary_angle += gyro_angle_delta;
    complementary_angle += multfix15(
        accel_angle - complementary_angle, float2fix15(0.01f));

    // 5. pid control at 1000 hz; derivative uses the measured angular rate.
    int pwm_on_time = (int)calculate_control(fix2float15(complementary_angle),
                                            fix2float15(filtered_gy));

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
        
        // Clear the buffer, draw the axes
        clearLowFrame(0, BLACK) ;
        drawAxes() ;

        start = head ;
        for (i = 0; i < PLOT_WIDTH; i++) {
            // keep out-of-range angle samples from drawing over the header
            float angle = fix2float15(angle_history[start]);
            if (angle >= -90.0f && angle <= 90.0f) {
                drawPixel(LEFT_EDGE + i, (TOP_PLOT_BOTTOM -
                    (int)(PLOT_HEIGHT * (angle + 90.0f) / 180.0f)), WHITE) ;
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
    char command[16], extra;
    float kp = 0.0f, ki = 0.0f, kd = 0.0f, target = 0.0f, value;
    unsigned updates = 0;
    snprintf(pt_serial_out_buffer, sizeof(pt_serial_out_buffer),
             "Invalid input. Use: kp N, ki N, kd N, target N, kp ki kd target, or stop. "
             "Gains: 0..1000000; target: -90..90 deg.\r\n");
    if (memchr(pt_serial_in_buffer, '\0', sizeof(pt_serial_in_buffer)) == NULL) return;
    if ((sscanf(pt_serial_in_buffer, "%15s %c", command, &extra) == 1 &&
         strcmp(command, "stop") == 0) ||
        (sscanf(pt_serial_in_buffer, "%f %c", &value, &extra) == 1 && value == 0.0f)) {
        uint32_t irq_state = save_and_disable_interrupts();
        stop_control();
        restore_interrupts(irq_state);
        snprintf(pt_serial_out_buffer, sizeof(pt_serial_out_buffer), "Motor off\r\n");
        return;
    }
    if (sscanf(pt_serial_in_buffer, "%f %f %f %f %c",
               &kp, &ki, &kd, &target, &extra) == 4) {
        updates = 15;
    } else if (sscanf(pt_serial_in_buffer, "%15s %f %c", command, &value, &extra) == 2) {
        if (strcmp(command, "kp") == 0) { kp = value; updates = 1; }
        else if (strcmp(command, "ki") == 0) { ki = value; updates = 2; }
        else if (strcmp(command, "kd") == 0) { kd = value; updates = 4; }
        else if (strcmp(command, "target") == 0) { target = value; updates = 8; }
    }
    if (updates == 0 ||
        ((updates & 1) && (!isfinite(kp) || kp < 0.0f || kp > 1000000.0f)) ||
        ((updates & 2) && (!isfinite(ki) || ki < 0.0f || ki > 1000000.0f)) ||
        ((updates & 4) && (!isfinite(kd) || kd < 0.0f || kd > 1000000.0f)) ||
        ((updates & 8) && (!isfinite(target) || target < -90.0f || target > 90.0f))) return;

    uint32_t irq_state = save_and_disable_interrupts();
    if (updates & 1) Kp = kp;
    if (updates & 2) {
        Ki = ki;
        if (Ki == 0.0f) integral_error = 0.0f;
        else {
            float limit = MOTOR_PWM_MAX / Ki;
            integral_error = fmaxf(-limit, fminf(limit, integral_error));
        }
    }
    if (updates & 4) Kd = kd;
    if (updates & 8) set_manual_target(target);
    restore_interrupts(irq_state);
    snprintf(pt_serial_out_buffer, sizeof(pt_serial_out_buffer),
             "Kp=%.4f Ki=%.4f Kd=%.4f target=%.2f deg motor=%s\r\n",
             Kp, Ki, Kd, desired_angle, motor_enabled ? "on" : "off");
}

static PT_THREAD (protothread_serial(struct pt *pt))
{
    PT_BEGIN(pt) ;
    while(1) {
        snprintf(pt_serial_out_buffer, sizeof(pt_serial_out_buffer),
                 "kp | ki | kd | target angle");
        serial_write ;
        serial_read ;
        handle_serial_command();
        serial_write ;
    }
    PT_END(pt) ;
}

// Entry point for core 1
void core1_entry() {
    pt_add_thread(protothread_draw) ;
    pt_schedule_start ;
}

//
// MAIN
//
int main() {

    gpio_init(12);
    gpio_set_dir(12, GPIO_OUT);
    sleep_ms(10);
    gpio_put(12, 1);
    sleep_ms(100);

    // overclock cpu to 150 mhz for the vga driver
    set_sys_clock_khz(150000, true) ;

    // Initialize stdio
    stdio_init_all();

    // button connects gpio 15 to ground; holding it keeps the motor off.
    gpio_init(DEMO_BUTTON);
    gpio_set_dir(DEMO_BUTTON, GPIO_IN);
    gpio_pull_up(DEMO_BUTTON);

    // Initialize VGA
    initVGA() ;

    ////////////////////////////////////////////////////////////////////////
    ///////////////////////// I2C CONFIGURATION ////////////////////////////
    i2c_init(I2C_CHAN, I2C_BAUD_RATE) ;
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C) ;
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C) ;

    // MPU6050 initialization
    if (!mpu6050_reset() || !mpu6050_read_raw(acceleration, gyro)) {
        gpio_init(PWM_OUT);
        gpio_set_dir(PWM_OUT, GPIO_OUT);
        printf("IMU initialization failed; motor disabled.\r\n");
        while (1) tight_loop_contents();
    }

    // initialize the measured angle: down=-90, horizontal=0, up=90 degrees.
    filtered_ax = acceleration[0];
    filtered_az = acceleration[2];
    accel_angle = multfix15(float2fix15(atan2(-filtered_ax, filtered_az)), oneeightyoverpi);
    complementary_angle = accel_angle;
    filtered_gy = gyro[1];
    for (int i = 0; i < PLOT_WIDTH; i++) angle_history[i] = complementary_angle;

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
    // start with zero PWM output.
    old_control = 0;
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
