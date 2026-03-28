/*
 * pid_controller.h — PID Controller with Feedforward
 *
 * Standard PID position controller with:
 *   - Anti-windup on integral term
 *   - Low-pass filter on derivative term
 *   - Velocity and acceleration feedforward
 *   - Output clamping
 */

#ifndef KLIPPER_ETHERCAT_PID_CONTROLLER_H
#define KLIPPER_ETHERCAT_PID_CONTROLLER_H

#include "shared_mem.h"

/* PID controller runtime state (per axis) */
struct pid_state {
    double integral;            /* Integral accumulator */
    double prev_error;          /* Previous error (for derivative) */
    double prev_derivative;     /* Previous filtered derivative */
    double output;              /* Last computed output */
    int    initialized;         /* 0 until first call */
};

/*
 * Initialize PID state. Call once per axis.
 */
void pid_init(struct pid_state *state);

/*
 * Reset PID state (e.g., after disable/enable cycle or mode change).
 */
void pid_reset(struct pid_state *state);

/*
 * Compute PID output for one cycle.
 *
 * Parameters:
 *   state      - per-axis runtime state
 *   params     - PID parameters from shared memory (klippy-configured)
 *   error      - position error: commanded - actual (mm)
 *   dt         - cycle time (seconds), typically 0.001
 *
 * Returns:
 *   Velocity command in mm/s (before feedforward — add ff externally)
 */
double pid_compute(struct pid_state *state, const struct pid_params *params,
                   double error, double dt);

/*
 * Compute full velocity command including feedforward.
 *
 * Parameters:
 *   state          - per-axis runtime state
 *   params         - PID parameters
 *   error          - position error (mm)
 *   target_vel     - target velocity from trajectory (mm/s)
 *   target_accel   - target acceleration from trajectory (mm/s^2)
 *   dt             - cycle time (seconds)
 *
 * Returns:
 *   Total velocity command in mm/s (PID + feedforward), clamped to max_velocity
 */
double pid_compute_with_ff(struct pid_state *state,
                           const struct pid_params *params,
                           double error, double target_vel,
                           double target_accel, double dt);

#endif /* KLIPPER_ETHERCAT_PID_CONTROLLER_H */
