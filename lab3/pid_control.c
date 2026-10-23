#include "pid_control.h"

#include <errno.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static float clamp_output(float value, float low, float high) {
    return value < low ? low : (value > high ? high : value);
}

float pid_step(const pid_settings_t *settings, pid_state_t *state, float angle) {
    float error = settings->target - angle;
    // Differentiate the measurement so changing the target does not kick D.
    float derivative = state->have_previous_angle
        ? -(angle - state->previous_angle) / PID_DT_SECONDS : 0.0f;
    state->previous_angle = angle;
    state->have_previous_angle = true;

    if (settings->ki == 0.0f) state->integral_output = 0.0f;
    float increment = settings->ki * error * PID_DT_SECONDS;
    float candidate = clamp_output(state->integral_output + increment,
                                   -PID_PWM_MAX, PID_PWM_MAX);
    float base = settings->bias + settings->kp * error + settings->kd * derivative;
    float output = base + candidate;
    // Integrate only when unsaturated or when the error unwinds saturation.
    if ((output >= 0.0f && output <= PID_PWM_MAX) ||
        (output > PID_PWM_MAX && increment < 0.0f) ||
        (output < 0.0f && increment > 0.0f)) {
        state->integral_output = candidate;
    }
    return clamp_output(base + state->integral_output, 0.0f, PID_PWM_MAX);
}

static bool parse_number(const char *text, float *value) {
    char *end;
    errno = 0;
    *value = strtof(text, &end);
    return end != text && *end == '\0' && errno != ERANGE && isfinite(*value);
}

static void show_settings(const pid_settings_t *settings, char *response, size_t size) {
    snprintf(response, size, "kp=%.6f ki=%.6f kd=%.6f target=%.3f bias=%d\r\n",
             settings->kp, settings->ki, settings->kd, settings->target, settings->bias);
}

pid_command_result_t pid_command(const char *line, pid_settings_t *settings,
                                char *response, size_t response_size) {
    if (response_size == 0) return PID_COMMAND_NONE;
    char command[32], argument[64], extra;
    float value;
    int fields = sscanf(line, "%31s %63s %c", command, argument, &extra);
    response[0] = '\0';
    if (fields <= 0) return PID_COMMAND_NONE;

    if (fields == 1 && strcmp(command, "help") == 0) {
        snprintf(response, response_size,
                 "kp N | ki N | kd N (0..1000000)\r\n"
                 "target N (0..360 deg) | bias N (integer 0..6000)\r\n"
                 "show | reset (clear PID history) | help\r\n"
                 "A bare integer sets bias. Settings are not saved across resets.\r\n");
        return PID_COMMAND_NONE;
    }
    if (fields == 1 && strcmp(command, "show") == 0) {
        show_settings(settings, response, response_size);
        return PID_COMMAND_NONE;
    }
    if (fields == 1 && strcmp(command, "reset") == 0) {
        snprintf(response, response_size, "PID history cleared; settings unchanged.\r\n");
        return PID_COMMAND_RESET;
    }

    // Retain the original numeric PWM bias interface.
    if (fields == 1 && parse_number(command, &value)) {
        strcpy(command, "bias");
    } else if (fields != 2 || !parse_number(argument, &value)) {
        snprintf(response, response_size, "Error: expected a command and finite number. Type help.\r\n");
        return PID_COMMAND_NONE;
    }

    float *gain = NULL;
    if (strcmp(command, "kp") == 0) gain = &settings->kp;
    else if (strcmp(command, "ki") == 0) gain = &settings->ki;
    else if (strcmp(command, "kd") == 0) gain = &settings->kd;

    if (gain != NULL) {
        if (value < 0.0f || value > PID_GAIN_MAX) {
            snprintf(response, response_size, "Error: gain must be 0..1000000.\r\n");
            return PID_COMMAND_NONE;
        }
        *gain = value;
    } else if (strcmp(command, "target") == 0) {
        if (value < 0.0f || value > 360.0f) {
            snprintf(response, response_size, "Error: target must be 0..360 degrees.\r\n");
            return PID_COMMAND_NONE;
        }
        settings->target = value;
    } else if (strcmp(command, "bias") == 0) {
        if (value < 0.0f || value > PID_PWM_MAX || value != truncf(value)) {
            snprintf(response, response_size, "Error: bias must be an integer 0..6000.\r\n");
            return PID_COMMAND_NONE;
        }
        settings->bias = (int)value;
    } else {
        snprintf(response, response_size, "Error: unknown command. Type help.\r\n");
        return PID_COMMAND_NONE;
    }
    show_settings(settings, response, response_size);
    return PID_COMMAND_UPDATE;
}
