// Run from the project directory:
// cc -std=c11 -Wall -Wextra -Werror -I. pid_control.c tests/test_pid_control.c -lm -o /tmp/test_pid_control
#include "pid_control.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static void close_to(float actual, float expected) {
    assert(fabsf(actual - expected) < 0.01f);
}

static void test_controller(void) {
    pid_settings_t settings = PID_DEFAULT_SETTINGS;
    pid_state_t state = {0};
    settings.bias = 3000;
    // Defaults retain the previous proportional-only control law.
    close_to(pid_step(&settings, &state, 170), 3010);
    close_to(pid_step(&settings, &state, 190), 2990);

    settings.kp = 0;
    settings.ki = 2;
    state = (pid_state_t){0};
    for (int i = 0; i < 1000; ++i) pid_step(&settings, &state, 170);
    close_to(state.integral_output, 20); // 2 * 10 degrees * 1 second

    // Saturation must not wind up the integrator in either direction.
    settings.bias = 6000;
    state = (pid_state_t){0};
    for (int i = 0; i < 1000; ++i)
        close_to(pid_step(&settings, &state, 170), 6000);
    close_to(state.integral_output, 0);
    settings.bias = 0;
    for (int i = 0; i < 1000; ++i)
        close_to(pid_step(&settings, &state, 190), 0);
    close_to(state.integral_output, 0);

    // Existing integral can unwind while the output is saturated.
    settings.bias = 6000;
    state.integral_output = 100;
    close_to(pid_step(&settings, &state, 190), 6000);
    assert(state.integral_output < 100);
    settings.bias = 0;
    state.integral_output = -100;
    close_to(pid_step(&settings, &state, 170), 0);
    assert(state.integral_output > -100);

    settings.bias = 3000;
    settings.ki = 0;
    settings.kd = 2;
    state = (pid_state_t){0};
    close_to(pid_step(&settings, &state, 170), 3000); // no startup D kick
    close_to(pid_step(&settings, &state, 171), 1000);
    settings.target = 200;
    close_to(pid_step(&settings, &state, 171), 3000); // no target D kick
    close_to(pid_step(&settings, &state, 181), 0);
    close_to(pid_step(&settings, &state, 171), 6000);
    state.integral_output = 100;
    pid_step(&settings, &state, 171);
    close_to(state.integral_output, 0); // disabling I clears its contribution
}

static void test_commands(void) {
    pid_settings_t settings = PID_DEFAULT_SETTINGS;
    char response[255];
    assert(pid_command("kp 2.5", &settings, response, sizeof response) == PID_COMMAND_UPDATE);
    close_to(settings.kp, 2.5f);
    assert(pid_command("ki 0.1", &settings, response, sizeof response) == PID_COMMAND_UPDATE);
    close_to(settings.ki, 0.1f);
    assert(pid_command("kd 1e-2", &settings, response, sizeof response) == PID_COMMAND_UPDATE);
    close_to(settings.kd, 0.01f);
    assert(pid_command("target 190", &settings, response, sizeof response) == PID_COMMAND_UPDATE);
    close_to(settings.target, 190);
    assert(pid_command("bias 6000", &settings, response, sizeof response) == PID_COMMAND_UPDATE);
    assert(settings.bias == 6000);
    assert(pid_command(" 3000 \r\n", &settings, response, sizeof response) == PID_COMMAND_UPDATE);
    assert(settings.bias == 3000);

    const char *invalid[] = {"kp -1", "ki nan", "kd inf", "kp 1e40", "kp 1000001",
        "target 361", "target -1", "bias 6001", "bias -1", "bias 1.5",
        "kp", "kp nope", "kp 2 extra", "kp 2junk", "100junk", "show 2", "unknown 2"};
    pid_settings_t before = settings;
    for (size_t i = 0; i < sizeof invalid / sizeof invalid[0]; ++i) {
        assert(pid_command(invalid[i], &settings, response, sizeof response) == PID_COMMAND_NONE);
        assert(strncmp(response, "Error:", 6) == 0);
        assert(settings.kp == before.kp && settings.ki == before.ki && settings.kd == before.kd);
        assert(settings.target == before.target && settings.bias == before.bias);
    }
    assert(pid_command("show", &settings, response, sizeof response) == PID_COMMAND_NONE);
    assert(strstr(response, "kp=2.500000") && strstr(response, "bias=3000"));
    assert(pid_command("help", &settings, response, sizeof response) == PID_COMMAND_NONE);
    assert(strstr(response, "kd N") && strstr(response, "across resets."));
    assert(pid_command("reset", &settings, response, sizeof response) == PID_COMMAND_RESET);
    assert(settings.kp == before.kp && settings.ki == before.ki && settings.kd == before.kd);
    assert(settings.target == before.target && settings.bias == before.bias);
    assert(pid_command(" \r\n", &settings, response, sizeof response) == PID_COMMAND_NONE);
    assert(response[0] == '\0');
    char small[4];
    pid_command("help", &settings, small, sizeof small);
    assert(small[3] == '\0');
}

int main(void) {
    test_controller();
    test_commands();
    puts("PID controller and serial command tests passed.");
    return 0;
}
