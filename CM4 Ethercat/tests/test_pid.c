/*
 * test_pid.c — Unit tests for PID controller
 */

#include <stdio.h>
#include <assert.h>
#include <math.h>
#include "../src/pid_controller.h"

static void test_proportional(void)
{
    printf("  test_proportional... ");

    struct pid_state state;
    struct pid_params params = {
        .kp = 10.0, .ki = 0.0, .kd = 0.0,
        .ff_velocity = 0.0, .ff_acceleration = 0.0,
        .max_following_error = 5.0, .max_velocity = 100.0,
        .max_accel = 1000.0, .position_scale = 1.0,
        .integral_windup_limit = 10.0,
    };

    pid_init(&state);

    double output = pid_compute(&state, &params, 1.0, 0.001);
    assert(fabs(output - 10.0) < 0.001);

    output = pid_compute(&state, &params, -2.0, 0.001);
    assert(fabs(output - (-20.0)) < 0.1);

    printf("PASS\n");
}

static void test_integral(void)
{
    printf("  test_integral... ");

    struct pid_state state;
    struct pid_params params = {
        .kp = 0.0, .ki = 100.0, .kd = 0.0,
        .ff_velocity = 0.0, .ff_acceleration = 0.0,
        .max_following_error = 5.0, .max_velocity = 1000.0,
        .max_accel = 1000.0, .position_scale = 1.0,
        .integral_windup_limit = 10.0,
    };

    pid_init(&state);

    /* Apply constant error for 10 cycles */
    double output = 0;
    for (int i = 0; i < 10; i++) {
        output = pid_compute(&state, &params, 1.0, 0.001);
    }
    /* integral = 10 * 1.0 * 0.001 = 0.01, ki * integral = 100 * 0.01 = 1.0 */
    assert(fabs(output - 1.0) < 0.1);

    printf("PASS\n");
}

static void test_feedforward(void)
{
    printf("  test_feedforward... ");

    struct pid_state state;
    struct pid_params params = {
        .kp = 0.0, .ki = 0.0, .kd = 0.0,
        .ff_velocity = 1.0, .ff_acceleration = 0.0,
        .max_following_error = 5.0, .max_velocity = 500.0,
        .max_accel = 1000.0, .position_scale = 1.0,
        .integral_windup_limit = 10.0,
    };

    pid_init(&state);

    /* Zero error, but target velocity = 100 mm/s */
    double output = pid_compute_with_ff(&state, &params,
                                         0.0, 100.0, 0.0, 0.001);
    assert(fabs(output - 100.0) < 0.1);

    printf("PASS\n");
}

static void test_output_clamping(void)
{
    printf("  test_output_clamping... ");

    struct pid_state state;
    struct pid_params params = {
        .kp = 1000.0, .ki = 0.0, .kd = 0.0,
        .ff_velocity = 0.0, .ff_acceleration = 0.0,
        .max_following_error = 5.0, .max_velocity = 100.0,
        .max_accel = 1000.0, .position_scale = 1.0,
        .integral_windup_limit = 10.0,
    };

    pid_init(&state);

    /* Large error should be clamped to max_velocity */
    double output = pid_compute_with_ff(&state, &params,
                                         10.0, 0.0, 0.0, 0.001);
    assert(fabs(output) <= 100.0 + 0.001);

    printf("PASS\n");
}

static void test_reset(void)
{
    printf("  test_reset... ");

    struct pid_state state;
    struct pid_params params = {
        .kp = 10.0, .ki = 100.0, .kd = 1.0,
        .ff_velocity = 0.0, .ff_acceleration = 0.0,
        .max_following_error = 5.0, .max_velocity = 500.0,
        .max_accel = 1000.0, .position_scale = 1.0,
        .integral_windup_limit = 10.0,
    };

    pid_init(&state);

    /* Accumulate some state */
    for (int i = 0; i < 100; i++) {
        pid_compute(&state, &params, 1.0, 0.001);
    }
    assert(state.integral > 0.0);

    /* Reset */
    pid_reset(&state);
    assert(state.integral == 0.0);
    assert(state.prev_error == 0.0);
    assert(state.initialized == 0);

    printf("PASS\n");
}

int main(void)
{
    printf("Running PID controller tests:\n");
    test_proportional();
    test_integral();
    test_feedforward();
    test_output_clamping();
    test_reset();
    printf("All PID tests PASSED\n");
    return 0;
}
