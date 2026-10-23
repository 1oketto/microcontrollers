#ifndef PID_CONTROL_H
#define PID_CONTROL_H

#include <stdbool.h>
#include <stddef.h>

#define PID_PWM_MAX 6000
#define PID_GAIN_MAX 1000000.0f
#define PID_DT_SECONDS 0.001f
#define PID_DEFAULT_SETTINGS {1.0f, 0.0f, 0.0f, 180.0f, 0}

typedef struct {
    // Gains use PWM counts, degrees, and seconds:
    // kp: counts/degree, ki: counts/(degree*second), kd: counts*second/degree.
    float kp, ki, kd;
    // Same 0..360 degree convention as the complementary angle display.
    float target;
    int bias;
} pid_settings_t;

typedef struct {
    float integral_output;
    float previous_angle;
    bool have_previous_angle;
} pid_state_t;

typedef enum {
    PID_COMMAND_NONE,
    PID_COMMAND_UPDATE,
    PID_COMMAND_RESET
} pid_command_result_t;

// Call once per PWM interrupt (1 ms). Output is clamped to 0..PID_PWM_MAX.
float pid_step(const pid_settings_t *settings, pid_state_t *state, float angle);
// Lowercase commands: kp/ki/kd/target/bias VALUE, show, help, reset.
// UPDATE changes settings; RESET requests clearing state without changing gains.
// The caller applies updates and clears state with the PWM interrupt disabled.
pid_command_result_t pid_command(const char *line, pid_settings_t *settings,
                                char *response, size_t response_size);

#endif
