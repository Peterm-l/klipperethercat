/*
 * pid_controller.c — PID Controller with Feedforward Implementation
 */

#include "pid_controller.h"
#include <string.h>
#include <math.h>

/* Derivative low-pass filter coefficient.
 * Lower values = more filtering (smoother but slower response).
 * 0.1 is a good default for 1kHz servo loops. */
#define DERIVATIVE_FILTER_ALPHA     0.1

static inline double clamp(double value, double min, double max)
{
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

void pid_init(struct pid_state *state)
{
    memset(state, 0, sizeof(*state));
}

void pid_reset(struct pid_state *state)
{
    state->integral = 0.0;
    state->prev_error = 0.0;
    state->prev_derivative = 0.0;
    state->output = 0.0;
    state->initialized = 0;
}

double pid_compute(struct pid_state *state, const struct pid_params *params,
                   double error, double dt)
{
    if (dt <= 0.0)
        return state->output;

    /* On first call, initialize previous error to avoid derivative spike */
    if (!state->initialized) {
        state->prev_error = error;
        state->initialized = 1;
    }

    /* Proportional term */
    double p_term = params->kp * error;

    /* Integral term with anti-windup clamping */
    state->integral += error * dt;

    double windup_limit = params->integral_windup_limit;
    if (windup_limit <= 0.0) {
        /* Default windup limit based on max velocity and Ki */
        if (params->ki > 0.0)
            windup_limit = params->max_velocity / params->ki;
        else
            windup_limit = 1000.0;
    }
    state->integral = clamp(state->integral, -windup_limit, windup_limit);

    double i_term = params->ki * state->integral;

    /* Derivative term with low-pass filter to suppress noise */
    double raw_derivative = (error - state->prev_error) / dt;

    /* First-order IIR low-pass filter */
    state->prev_derivative = DERIVATIVE_FILTER_ALPHA * raw_derivative
                           + (1.0 - DERIVATIVE_FILTER_ALPHA) * state->prev_derivative;

    double d_term = params->kd * state->prev_derivative;

    /* Store previous error */
    state->prev_error = error;

    /* Sum PID terms */
    state->output = p_term + i_term + d_term;

    return state->output;
}

double pid_compute_with_ff(struct pid_state *state,
                           const struct pid_params *params,
                           double error, double target_vel,
                           double target_accel, double dt)
{
    /* Compute PID correction */
    double pid_output = pid_compute(state, params, error, dt);

    /* Add velocity feedforward */
    double ff_vel = params->ff_velocity * target_vel;

    /* Add acceleration feedforward */
    double ff_accel = params->ff_acceleration * target_accel;

    /* Total velocity command */
    double total = pid_output + ff_vel + ff_accel;

    /* Clamp to maximum velocity */
    total = clamp(total, -params->max_velocity, params->max_velocity);

    return total;
}
