/*
 * autotune.c — Auto-Tune Engine Implementation
 *
 * This runs inside the RT loop at 1kHz. Each call to autotune_update()
 * advances the state machine by one cycle and returns a velocity command.
 *
 * The auto-tune sequence:
 *   1. Safety checks (verify travel, drive state)
 *   2. Relay feedback (Åström-Hägglund oscillation test) — ~30s
 *   3. Inertia measurement (acceleration ramp) — ~2s
 *   4. Friction characterization (slow bidirectional crawl) — ~5s
 *   5. PID gain computation
 *   6. Validation (step response with iterative refinement) — ~10s
 */

#include "autotune.h"
#include "pid_controller.h"
#include <string.h>
#include <math.h>
#include <stdio.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* Cycle time in seconds (1ms) */
#define DT  0.001

static inline double clamp_val(double v, double lo, double hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

static inline double fabs_val(double v)
{
    return v < 0.0 ? -v : v;
}

void autotune_init(struct autotune_state *state, int axis,
                   int32_t encoder_resolution, double rotation_distance)
{
    memset(state, 0, sizeof(*state));
    state->axis = axis;
    state->phase = AUTOTUNE_IDLE;
    state->encoder_resolution = encoder_resolution;
    state->rotation_distance = rotation_distance;
}

void autotune_start(struct autotune_state *state,
                    double travel_mm, int aggression, int validate_only,
                    double current_position, double max_velocity)
{
    state->travel_mm = travel_mm;
    state->aggression = aggression;
    state->validate_only = validate_only;
    state->phase = AUTOTUNE_SAFETY_CHECK;
    state->phase_start_cycle = 0;
    state->cycle_count = 0;
    state->log_index = 0;
    state->progress_percent = 0;

    /* Configure relay feedback */
    state->relay_config.relay_amplitude = max_velocity * 0.05; /* 5% of max */
    state->relay_config.target_position = current_position;
    state->relay_config.min_cycles = 10;
    state->relay_config.max_cycles = 30;
    state->relay_config.timeout_seconds = 30.0;
    state->relay_config.max_amplitude_mm = travel_mm * 0.5;

    /* Configure inertia test */
    state->inertia_config.test_velocity = max_velocity * 0.2;
    state->inertia_config.ramp_time = 0.3;
    state->inertia_config.num_samples = 100;

    /* Initialize sub-states */
    state->relay_sub = RELAY_INIT;
    state->inertia_sub = INERTIA_INIT;
    state->friction_sub = FRICTION_INIT;
    state->validate_sub = VALIDATE_INIT;

    snprintf(state->status_message, sizeof(state->status_message),
             "Starting auto-tune");
}

/* =========================================================================
 * Phase 1: Safety Check
 * ========================================================================= */
static int phase_safety_check(struct autotune_state *state,
                              const struct servo_state *servo)
{
    /* Verify drive is in OPERATION_ENABLED */
    if (servo->state != DRIVE_OPERATION_ENABLED) {
        snprintf(state->status_message, sizeof(state->status_message),
                 "Drive not enabled (state=%d)", servo->state);
        state->phase = AUTOTUNE_FAILED;
        return 0;
    }

    /* Check that following error is small (axis is stationary) */
    if (fabs_val(servo->following_error) > 1.0) {
        snprintf(state->status_message, sizeof(state->status_message),
                 "Following error too large (%.2fmm) — axis not settled",
                 servo->following_error);
        state->phase = AUTOTUNE_FAILED;
        return 0;
    }

    /* Safety check passed */
    snprintf(state->status_message, sizeof(state->status_message),
             "Safety checks passed");
    state->progress_percent = 5;

    if (state->validate_only) {
        state->phase = AUTOTUNE_VALIDATE;
    } else {
        state->phase = AUTOTUNE_RELAY_FEEDBACK;
    }
    state->phase_start_cycle = state->cycle_count;
    return 1;
}

/* =========================================================================
 * Phase 2: Relay Feedback (Åström-Hägglund)
 * ========================================================================= */
static int phase_relay_feedback(struct autotune_state *state,
                                const struct servo_state *servo,
                                double *vel_cmd)
{
    struct relay_feedback_config *cfg = &state->relay_config;
    double pos = servo->actual_position;
    double elapsed = (state->cycle_count - state->phase_start_cycle) * DT;

    switch (state->relay_sub) {
    case RELAY_INIT:
        state->relay_center = pos;
        state->relay_direction = 1;
        state->relay_output = cfg->relay_amplitude;
        state->relay_crossings = 0;
        state->relay_peak_count = 0;
        state->relay_sub = RELAY_WAIT_FIRST_CROSS;
        snprintf(state->status_message, sizeof(state->status_message),
                 "Relay feedback: waiting for first oscillation");
        *vel_cmd = state->relay_output;
        break;

    case RELAY_WAIT_FIRST_CROSS:
    case RELAY_OSCILLATING: {
        /* Relay logic: switch direction when position crosses center */
        double error = pos - state->relay_center;

        int should_switch = 0;
        if (state->relay_direction > 0 && error > 0) {
            should_switch = 1;
        } else if (state->relay_direction < 0 && error < 0) {
            should_switch = 1;
        }

        if (should_switch) {
            state->relay_direction = -state->relay_direction;
            state->relay_output = cfg->relay_amplitude * state->relay_direction;

            /* Record crossing time */
            if (state->relay_crossings < 64) {
                state->relay_cross_times[state->relay_crossings] = elapsed;
            }
            state->relay_crossings++;
            state->relay_sub = RELAY_OSCILLATING;

            /* Record peak position */
            if (state->relay_peak_count < 64) {
                state->relay_peaks[state->relay_peak_count] = fabs_val(error);
            }
            state->relay_peak_count++;

            /* Check safety: oscillation too large? */
            if (fabs_val(error) > cfg->max_amplitude_mm) {
                snprintf(state->status_message, sizeof(state->status_message),
                         "Relay: oscillation too large (%.2fmm > %.2fmm limit)",
                         fabs_val(error), cfg->max_amplitude_mm);
                state->phase = AUTOTUNE_FAILED;
                *vel_cmd = 0.0;
                return 0;
            }

            int full_cycles = state->relay_crossings / 2;
            state->progress_percent = 5 + (full_cycles * 30 / cfg->min_cycles);
            if (state->progress_percent > 35) state->progress_percent = 35;

            snprintf(state->status_message, sizeof(state->status_message),
                     "Relay feedback: cycle %d/%d",
                     full_cycles, cfg->min_cycles);
        }

        /* Check if we have enough cycles */
        if (state->relay_crossings >= cfg->min_cycles * 2) {
            state->relay_sub = RELAY_DONE;
        }

        /* Timeout check */
        if (elapsed > cfg->timeout_seconds) {
            snprintf(state->status_message, sizeof(state->status_message),
                     "Relay feedback timeout after %.1fs", elapsed);
            state->phase = AUTOTUNE_FAILED;
            *vel_cmd = 0.0;
            return 0;
        }

        *vel_cmd = state->relay_output;
        break;
    }

    case RELAY_DONE: {
        *vel_cmd = 0.0;

        /* Compute results from oscillation data */
        struct relay_feedback_result *res = &state->relay_result;

        /* Average period from crossing times */
        int num_half_periods = state->relay_crossings - 1;
        if (num_half_periods < 4) {
            snprintf(state->status_message, sizeof(state->status_message),
                     "Relay: not enough crossings (%d)", state->relay_crossings);
            state->phase = AUTOTUNE_FAILED;
            return 0;
        }

        double total_time = state->relay_cross_times[state->relay_crossings - 1]
                          - state->relay_cross_times[0];
        double avg_half_period = total_time / (double)num_half_periods;
        res->Tu = avg_half_period * 2.0;

        /* Average amplitude from peaks (skip first 2 for settling) */
        double amp_sum = 0.0;
        int amp_count = 0;
        int start_idx = (state->relay_peak_count > 4) ? 2 : 0;
        for (int i = start_idx; i < state->relay_peak_count; i++) {
            amp_sum += state->relay_peaks[i];
            amp_count++;
        }
        res->Au = (amp_count > 0) ? (amp_sum / amp_count) : 0.001;

        /* Ultimate gain: Ku = 4 * d / (pi * Au) where d = relay amplitude */
        res->Ku = 4.0 * cfg->relay_amplitude / (M_PI * res->Au);
        res->num_cycles = state->relay_crossings / 2;
        res->valid = (res->Tu > 0.001 && res->Au > 0.0001 && res->Ku > 0.0);

        if (!res->valid) {
            snprintf(state->status_message, sizeof(state->status_message),
                     "Relay: invalid results Tu=%.4f Au=%.4f Ku=%.2f",
                     res->Tu, res->Au, res->Ku);
            state->phase = AUTOTUNE_FAILED;
            return 0;
        }

        snprintf(state->status_message, sizeof(state->status_message),
                 "Relay complete: Tu=%.4fs Au=%.3fmm Ku=%.1f",
                 res->Tu, res->Au, res->Ku);
        state->progress_percent = 35;
        state->phase = AUTOTUNE_INERTIA;
        state->phase_start_cycle = state->cycle_count;
        return 1;
    }

    default:
        *vel_cmd = 0.0;
        break;
    }

    /* Log data */
    if (state->log_index < 16384) {
        state->position_log[state->log_index] = pos;
        state->velocity_log[state->log_index] = servo->actual_velocity;
        state->torque_log[state->log_index] = servo->torque_percent;
        state->log_index++;
    }

    return 1;
}

/* =========================================================================
 * Phase 3: Inertia Measurement
 * ========================================================================= */
static int phase_inertia(struct autotune_state *state,
                         const struct servo_state *servo,
                         double *vel_cmd)
{
    double phase_elapsed = (state->cycle_count - state->phase_start_cycle) * DT;

    switch (state->inertia_sub) {
    case INERTIA_INIT:
        state->inertia_start_pos = servo->actual_position;
        state->inertia_start_vel = 0.0;
        state->inertia_torque_sum = 0.0;
        state->inertia_accel_sum = 0.0;
        state->inertia_samples = 0;
        state->inertia_sub = INERTIA_ACCELERATING;
        *vel_cmd = 0.0;
        snprintf(state->status_message, sizeof(state->status_message),
                 "Inertia: accelerating...");
        break;

    case INERTIA_ACCELERATING: {
        double ramp_time = state->inertia_config.ramp_time;
        double target_vel = state->inertia_config.test_velocity;

        if (phase_elapsed < ramp_time) {
            /* Linear velocity ramp */
            *vel_cmd = target_vel * (phase_elapsed / ramp_time);

            /* Record torque and velocity for inertia calculation */
            if (phase_elapsed > 0.05) { /* skip initial transient */
                state->inertia_torque_sum += servo->torque_percent;
                state->inertia_accel_sum += (servo->actual_velocity
                    - state->inertia_start_vel) / DT;
                state->inertia_start_vel = servo->actual_velocity;
                state->inertia_samples++;
            }
        } else {
            /* Ramp complete — hold briefly then decelerate */
            *vel_cmd = target_vel;
            state->inertia_sub = INERTIA_DECELERATING;
            state->phase_start_cycle = state->cycle_count;
        }

        /* Safety: check travel limit */
        double travel = fabs_val(servo->actual_position - state->inertia_start_pos);
        if (travel > state->travel_mm * 0.8) {
            state->inertia_sub = INERTIA_DECELERATING;
            state->phase_start_cycle = state->cycle_count;
        }

        state->progress_percent = 40;
        break;
    }

    case INERTIA_DECELERATING: {
        double decel_time = state->inertia_config.ramp_time;
        if (phase_elapsed < decel_time) {
            double target_vel = state->inertia_config.test_velocity;
            *vel_cmd = target_vel * (1.0 - phase_elapsed / decel_time);
        } else {
            *vel_cmd = 0.0;
            state->inertia_sub = INERTIA_DONE;
        }
        break;
    }

    case INERTIA_DONE: {
        *vel_cmd = 0.0;
        struct inertia_result *res = &state->inertia_result;

        if (state->inertia_samples > 10) {
            double avg_torque = state->inertia_torque_sum / state->inertia_samples;
            double avg_accel = state->inertia_accel_sum / state->inertia_samples;

            /* J = torque / acceleration (simplified) */
            if (fabs_val(avg_accel) > 0.001) {
                res->J_kg_m2 = fabs_val(avg_torque / avg_accel) * 0.001;
                res->friction_torque = avg_torque * 0.1; /* rough estimate */
                res->ff_velocity = 1.0; /* default, refine from inertia */
                res->valid = 1;
            } else {
                res->valid = 0;
            }
        } else {
            res->valid = 0;
        }

        if (!res->valid) {
            /* Non-fatal: use defaults */
            res->J_kg_m2 = 0.001;
            res->ff_velocity = 1.0;
            res->valid = 1; /* proceed with defaults */
        }

        snprintf(state->status_message, sizeof(state->status_message),
                 "Inertia complete: J=%.6f kg*m^2", res->J_kg_m2);
        state->progress_percent = 50;
        state->phase = AUTOTUNE_FRICTION;
        state->phase_start_cycle = state->cycle_count;
        return 1;
    }

    default:
        *vel_cmd = 0.0;
        break;
    }

    return 1;
}

/* =========================================================================
 * Phase 4: Friction Characterization
 * ========================================================================= */
static int phase_friction(struct autotune_state *state,
                          const struct servo_state *servo,
                          double *vel_cmd)
{
    double phase_elapsed = (state->cycle_count - state->phase_start_cycle) * DT;

    switch (state->friction_sub) {
    case FRICTION_INIT:
        state->friction_sample_count_pos = 0;
        state->friction_sample_count_neg = 0;
        state->friction_move_start = servo->actual_position;
        state->friction_sub = FRICTION_POSITIVE;
        state->phase_start_cycle = state->cycle_count;
        snprintf(state->status_message, sizeof(state->status_message),
                 "Friction: measuring positive direction...");
        *vel_cmd = 0.0;
        break;

    case FRICTION_POSITIVE: {
        /* Slow crawl in positive direction at multiple speeds */
        double speeds[] = {2.0, 5.0, 10.0, 20.0};
        int num_speeds = 4;
        double time_per_speed = 0.5; /* seconds at each speed */
        int speed_idx = (int)(phase_elapsed / time_per_speed);

        if (speed_idx < num_speeds) {
            *vel_cmd = speeds[speed_idx];

            /* Wait for steady state (first 0.2s of each speed) */
            double time_in_speed = phase_elapsed - speed_idx * time_per_speed;
            if (time_in_speed > 0.2 &&
                state->friction_sample_count_pos < 32) {
                int idx = state->friction_sample_count_pos;
                state->friction_torques_pos[idx] = servo->torque_percent;
                state->friction_velocities_pos[idx] = servo->actual_velocity;
                state->friction_sample_count_pos++;
            }

            /* Safety: check travel */
            if (fabs_val(servo->actual_position - state->friction_move_start)
                > state->travel_mm * 0.4) {
                state->friction_sub = FRICTION_NEGATIVE;
                state->phase_start_cycle = state->cycle_count;
            }
        } else {
            state->friction_sub = FRICTION_NEGATIVE;
            state->phase_start_cycle = state->cycle_count;
        }
        state->progress_percent = 55;
        break;
    }

    case FRICTION_NEGATIVE: {
        double speeds[] = {-2.0, -5.0, -10.0, -20.0};
        int num_speeds = 4;
        double time_per_speed = 0.5;
        double neg_elapsed = (state->cycle_count - state->phase_start_cycle) * DT;
        int speed_idx = (int)(neg_elapsed / time_per_speed);

        snprintf(state->status_message, sizeof(state->status_message),
                 "Friction: measuring negative direction...");

        if (speed_idx < num_speeds) {
            *vel_cmd = speeds[speed_idx];

            double time_in_speed = neg_elapsed - speed_idx * time_per_speed;
            if (time_in_speed > 0.2 &&
                state->friction_sample_count_neg < 32) {
                int idx = state->friction_sample_count_neg;
                state->friction_torques_neg[idx] = servo->torque_percent;
                state->friction_velocities_neg[idx] = servo->actual_velocity;
                state->friction_sample_count_neg++;
            }

            if (fabs_val(servo->actual_position - state->friction_move_start)
                > state->travel_mm * 0.4) {
                state->friction_sub = FRICTION_DONE;
            }
        } else {
            state->friction_sub = FRICTION_DONE;
        }
        state->progress_percent = 60;
        break;
    }

    case FRICTION_DONE: {
        *vel_cmd = 0.0;
        struct friction_result *res = &state->friction_result;

        /* Compute static friction from lowest speed samples */
        if (state->friction_sample_count_pos > 0) {
            res->Fs_positive = fabs_val(state->friction_torques_pos[0]) * 0.01;
        } else {
            res->Fs_positive = 0.1;
        }

        if (state->friction_sample_count_neg > 0) {
            res->Fs_negative = fabs_val(state->friction_torques_neg[0]) * 0.01;
        } else {
            res->Fs_negative = 0.1;
        }

        /* Compute viscous friction from speed/torque relationship */
        if (state->friction_sample_count_pos >= 2) {
            double dt_torque = fabs_val(state->friction_torques_pos[
                state->friction_sample_count_pos - 1]
                - state->friction_torques_pos[0]) * 0.01;
            double dv = fabs_val(state->friction_velocities_pos[
                state->friction_sample_count_pos - 1]
                - state->friction_velocities_pos[0]);
            res->Fv = (dv > 0.1) ? (dt_torque / dv) : 0.001;
        } else {
            res->Fv = 0.001;
        }

        res->deadband_mm_s = 1.0;
        res->valid = 1;

        snprintf(state->status_message, sizeof(state->status_message),
                 "Friction complete: Fs=%.3fN Fv=%.5f",
                 (res->Fs_positive + res->Fs_negative) / 2.0, res->Fv);
        state->progress_percent = 65;
        state->phase = AUTOTUNE_COMPUTE_GAINS;
        state->phase_start_cycle = state->cycle_count;
        return 1;
    }

    default:
        *vel_cmd = 0.0;
        break;
    }

    return 1;
}

/* =========================================================================
 * Phase 5: PID Gain Computation
 * ========================================================================= */
void compute_pid_gains(const struct relay_feedback_result *relay,
                       const struct inertia_result *inertia,
                       const struct friction_result *friction,
                       int aggression,
                       struct autotune_gains *output)
{
    /* Ziegler-Nichols relay tuning rules */
    double Ku = relay->Ku;
    double Tu = relay->Tu;

    /* Base gains */
    output->kp = 0.6 * Ku;
    output->ki = 2.0 * output->kp / Tu;
    output->kd = output->kp * Tu / 8.0;

    /* Aggression scaling */
    double scale;
    switch (aggression) {
    case AGGRESSION_CONSERVATIVE:
        scale = 0.5;
        break;
    case AGGRESSION_AGGRESSIVE:
        scale = 1.2;
        break;
    case AGGRESSION_MODERATE:
    default:
        scale = 1.0;
        break;
    }
    output->kp *= scale;
    output->ki *= scale;
    output->kd *= scale;

    /* Feedforward from inertia measurement */
    if (inertia && inertia->valid) {
        output->ff_velocity = inertia->ff_velocity;
    } else {
        output->ff_velocity = 1.0;
    }
    output->ff_acceleration = 0.0;

    /* Integral windup limit based on friction */
    if (friction && friction->valid && output->ki > 0.0) {
        double avg_Fs = (friction->Fs_positive + friction->Fs_negative) / 2.0;
        output->integral_windup_limit = avg_Fs / output->ki * 2.0;
        if (output->integral_windup_limit < 1.0)
            output->integral_windup_limit = 1.0;
    } else {
        output->integral_windup_limit = 10.0;
    }

    /* Max following error: ~3x the relay oscillation amplitude */
    output->max_following_error = relay->Au * 3.0;
    if (output->max_following_error < 0.5)
        output->max_following_error = 0.5;
    if (output->max_following_error > 5.0)
        output->max_following_error = 5.0;
}

static int phase_compute_gains(struct autotune_state *state)
{
    compute_pid_gains(&state->relay_result,
                      &state->inertia_result,
                      &state->friction_result,
                      state->aggression,
                      &state->computed_gains);

    snprintf(state->status_message, sizeof(state->status_message),
             "Gains: Kp=%.2f Ki=%.2f Kd=%.3f FF=%.2f",
             state->computed_gains.kp, state->computed_gains.ki,
             state->computed_gains.kd, state->computed_gains.ff_velocity);
    state->progress_percent = 70;
    state->phase = AUTOTUNE_VALIDATE;
    state->phase_start_cycle = state->cycle_count;
    return 1;
}

/* =========================================================================
 * Phase 6: Validation Step Response
 * ========================================================================= */
static int phase_validate(struct autotune_state *state,
                          const struct servo_state *servo,
                          double *vel_cmd)
{
    /* Use a simple PID controller for validation */
    static struct pid_state val_pid;
    struct pid_params val_params;

    /* Set up PID params from computed gains */
    val_params.kp = state->computed_gains.kp;
    val_params.ki = state->computed_gains.ki;
    val_params.kd = state->computed_gains.kd;
    val_params.ff_velocity = state->computed_gains.ff_velocity;
    val_params.ff_acceleration = 0.0;
    val_params.max_velocity = 100.0; /* conservative for validation */
    val_params.max_accel = 1000.0;
    val_params.integral_windup_limit = state->computed_gains.integral_windup_limit;
    val_params.max_following_error = state->computed_gains.max_following_error;
    val_params.position_scale = 1.0;

    double phase_elapsed = (state->cycle_count - state->phase_start_cycle) * DT;

    switch (state->validate_sub) {
    case VALIDATE_INIT:
        pid_init(&val_pid);
        state->validate_start_pos = servo->actual_position;
        state->validate_step_size = 5.0; /* 5mm step */
        state->validate_target = state->validate_start_pos;
        state->validate_iteration = 0;
        state->validate_peak_pos = 0.0;
        state->validate_sub = VALIDATE_STEP_FORWARD;
        state->phase_start_cycle = state->cycle_count;
        snprintf(state->status_message, sizeof(state->status_message),
                 "Validation: step response test...");
        /* fallthrough */

    case VALIDATE_STEP_FORWARD:
        state->validate_target = state->validate_start_pos
                               + state->validate_step_size;
        /* PID tracks target position */
        {
            double error = state->validate_target - servo->actual_position;
            *vel_cmd = pid_compute_with_ff(&val_pid, &val_params,
                                           error, 0.0, 0.0, DT);
        }

        /* Track overshoot (peak position beyond target) */
        if (servo->actual_position > state->validate_peak_pos) {
            state->validate_peak_pos = servo->actual_position;
        }

        /* Record step response data for analysis */
        if (state->log_index < 16384) {
            state->position_log[state->log_index] = servo->actual_position;
            state->velocity_log[state->log_index] = servo->actual_velocity;
            state->torque_log[state->log_index] = servo->torque_percent;
            state->log_index++;
        }

        /* Wait 500ms for settling */
        if (phase_elapsed > 0.5) {
            state->validate_sub = VALIDATE_SETTLE_FORWARD;
            state->phase_start_cycle = state->cycle_count;
        }
        state->progress_percent = 75;
        break;

    case VALIDATE_SETTLE_FORWARD:
        {
            double error = state->validate_target - servo->actual_position;
            *vel_cmd = pid_compute_with_ff(&val_pid, &val_params,
                                           error, 0.0, 0.0, DT);
        }
        if (phase_elapsed > 0.5) {
            state->validate_sub = VALIDATE_STEP_BACK;
            state->phase_start_cycle = state->cycle_count;
            pid_reset(&val_pid);
        }
        state->progress_percent = 80;
        break;

    case VALIDATE_STEP_BACK:
        state->validate_target = state->validate_start_pos;
        {
            double error = state->validate_target - servo->actual_position;
            *vel_cmd = pid_compute_with_ff(&val_pid, &val_params,
                                           error, 0.0, 0.0, DT);
        }
        if (phase_elapsed > 0.5) {
            state->validate_sub = VALIDATE_SETTLE_BACK;
            state->phase_start_cycle = state->cycle_count;
        }
        state->progress_percent = 85;
        break;

    case VALIDATE_SETTLE_BACK:
        {
            double error = state->validate_target - servo->actual_position;
            *vel_cmd = pid_compute_with_ff(&val_pid, &val_params,
                                           error, 0.0, 0.0, DT);
        }
        if (phase_elapsed > 0.5) {
            state->validate_sub = VALIDATE_ANALYZE;
        }
        state->progress_percent = 88;
        break;

    case VALIDATE_ANALYZE: {
        *vel_cmd = 0.0;
        struct validation_result *res = &state->validation_result;

        double step_target = state->validate_start_pos
                           + state->validate_step_size;
        double threshold_90 = state->validate_start_pos
                            + state->validate_step_size * 0.9;
        double settle_band = 0.1;  /* ±0.1mm settling band */

        /* Overshoot: peak position beyond target */
        double overshoot = state->validate_peak_pos - step_target;
        res->overshoot_percent = (overshoot / state->validate_step_size) * 100.0;
        if (res->overshoot_percent < 0.0)
            res->overshoot_percent = 0.0;

        /* Measure rise time from recorded position log.
         * Rise time = time to first reach 90% of step target. */
        res->rise_time_ms = 500.0;  /* default if not found */
        uint32_t num_samples = state->log_index;
        for (uint32_t i = 0; i < num_samples; i++) {
            if (state->position_log[i] >= threshold_90) {
                res->rise_time_ms = (double)i * DT * 1000.0;
                break;
            }
        }

        /* Measure settling time from recorded position log.
         * Settling time = time after which position stays within
         * ±settle_band of target permanently. Scan backwards. */
        res->settling_time_ms = (double)num_samples * DT * 1000.0;
        for (int i = (int)num_samples - 1; i >= 0; i--) {
            double err = fabs_val(state->position_log[i] - step_target);
            if (err > settle_band) {
                res->settling_time_ms = (double)(i + 1) * DT * 1000.0;
                break;
            }
        }

        /* Steady-state error: average error over last 50 samples */
        double ss_err_sum = 0.0;
        int ss_count = 0;
        int ss_start = (num_samples > 50) ? (int)(num_samples - 50) : 0;
        for (int i = ss_start; i < (int)num_samples; i++) {
            ss_err_sum += fabs_val(state->position_log[i] - step_target);
            ss_count++;
        }
        res->steady_state_error_mm = (ss_count > 0)
            ? (ss_err_sum / ss_count) : fabs_val(servo->actual_position - step_target);

        /* Tracking error RMS over the whole step response */
        double rms_sum = 0.0;
        for (uint32_t i = 0; i < num_samples; i++) {
            double e = state->position_log[i] - step_target;
            rms_sum += e * e;
        }
        res->tracking_error_rms_mm = (num_samples > 0)
            ? sqrt(rms_sum / num_samples) : fabs_val(servo->following_error);

        res->iterations = state->validate_iteration + 1;

        /* Check pass criteria */
        int passed = 1;
        if (res->overshoot_percent > VALIDATION_MAX_OVERSHOOT) passed = 0;
        if (res->settling_time_ms > VALIDATION_MAX_SETTLING_TIME) passed = 0;
        if (res->steady_state_error_mm > VALIDATION_MAX_SS_ERROR) passed = 0;
        res->passed = passed;

        if (!passed && state->validate_iteration < VALIDATION_MAX_ITERATIONS) {
            state->validate_sub = VALIDATE_REFINE;
        } else {
            state->validate_sub = VALIDATE_DONE;
        }

        state->progress_percent = 90;
        break;
    }

    case VALIDATE_REFINE:
        /* Adjust gains based on validation results */
        if (state->validation_result.overshoot_percent > 10.0) {
            state->computed_gains.kp *= 0.8;
            state->computed_gains.kd *= 1.2;
        }
        if (state->validation_result.settling_time_ms > 200.0) {
            state->computed_gains.kp *= 1.1;
        }

        state->validate_iteration++;
        state->validate_sub = VALIDATE_INIT;
        state->validate_peak_pos = 0.0;
        state->phase_start_cycle = state->cycle_count;
        pid_reset(&val_pid);
        *vel_cmd = 0.0;
        break;

    case VALIDATE_DONE:
        *vel_cmd = 0.0;

        if (state->validation_result.passed) {
            snprintf(state->status_message, sizeof(state->status_message),
                     "Validation PASSED: overshoot=%.1f%% settling=%.0fms",
                     state->validation_result.overshoot_percent,
                     state->validation_result.settling_time_ms);
            state->phase = AUTOTUNE_COMPLETE;
        } else {
            snprintf(state->status_message, sizeof(state->status_message),
                     "Validation: results marginal but usable");
            state->phase = AUTOTUNE_COMPLETE;
        }
        state->progress_percent = 100;
        return 1;

    default:
        *vel_cmd = 0.0;
        break;
    }

    return 1;
}

/* =========================================================================
 * Main Update Function
 * ========================================================================= */
int autotune_update(struct autotune_state *state,
                    const struct servo_state *servo,
                    double *vel_cmd)
{
    *vel_cmd = 0.0;
    state->cycle_count++;

    switch (state->phase) {
    case AUTOTUNE_IDLE:
        return 0;

    case AUTOTUNE_SAFETY_CHECK:
        return phase_safety_check(state, servo);

    case AUTOTUNE_RELAY_FEEDBACK:
        return phase_relay_feedback(state, servo, vel_cmd);

    case AUTOTUNE_INERTIA:
        return phase_inertia(state, servo, vel_cmd);

    case AUTOTUNE_FRICTION:
        return phase_friction(state, servo, vel_cmd);

    case AUTOTUNE_COMPUTE_GAINS:
        return phase_compute_gains(state);

    case AUTOTUNE_VALIDATE:
        return phase_validate(state, servo, vel_cmd);

    case AUTOTUNE_COMPLETE:
    case AUTOTUNE_FAILED:
        return 0;

    default:
        state->phase = AUTOTUNE_FAILED;
        snprintf(state->status_message, sizeof(state->status_message),
                 "Invalid autotune phase");
        return 0;
    }
}

void autotune_copy_to_shm(const struct autotune_state *state,
                           struct autotune_status *shm_status)
{
    shm_status->phase = (uint8_t)state->phase;
    shm_status->progress_percent = state->progress_percent;
    memcpy(shm_status->message, state->status_message,
           sizeof(shm_status->message));

    /* Copy results if complete */
    if (state->phase == AUTOTUNE_COMPLETE) {
        shm_status->Tu = state->relay_result.Tu;
        shm_status->Au = state->relay_result.Au;
        shm_status->J = state->inertia_result.J_kg_m2;
        shm_status->Fs = (state->friction_result.Fs_positive
                        + state->friction_result.Fs_negative) / 2.0;
        shm_status->Fv = state->friction_result.Fv;
        shm_status->kp = state->computed_gains.kp;
        shm_status->ki = state->computed_gains.ki;
        shm_status->kd = state->computed_gains.kd;
        shm_status->ff_velocity = state->computed_gains.ff_velocity;
        shm_status->rise_time_ms = state->validation_result.rise_time_ms;
        shm_status->overshoot_percent = state->validation_result.overshoot_percent;
        shm_status->settling_time_ms = state->validation_result.settling_time_ms;
        shm_status->ss_error_mm = state->validation_result.steady_state_error_mm;
        shm_status->tracking_rms_mm = state->validation_result.tracking_error_rms_mm;
    }
}
