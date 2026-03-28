/*
 * autotune.h — Auto-Tune Engine
 *
 * Automated PID tuning that physically moves the axis to characterize
 * the mechanical system. Implements:
 *   1. Relay feedback (Åström-Hägglund) for critical frequency/gain
 *   2. Inertia measurement via acceleration test
 *   3. Friction characterization via slow bidirectional motion
 *   4. PID gain computation (Ziegler-Nichols relay method)
 *   5. Step response validation with iterative refinement
 */

#ifndef KLIPPER_ETHERCAT_AUTOTUNE_H
#define KLIPPER_ETHERCAT_AUTOTUNE_H

#include "shared_mem.h"
#include <stdint.h>

/* =========================================================================
 * Relay Feedback Test
 * ========================================================================= */

struct relay_feedback_config {
    double relay_amplitude;         /* Velocity command magnitude (mm/s) */
    double target_position;         /* Oscillation center (mm) */
    int    min_cycles;              /* Minimum oscillation cycles (10) */
    int    max_cycles;              /* Maximum before giving up (30) */
    double timeout_seconds;         /* Safety timeout (30.0) */
    double max_amplitude_mm;        /* Abort if oscillation exceeds this (5.0) */
};

struct relay_feedback_result {
    double Tu;                      /* Ultimate period (seconds) */
    double Au;                      /* Oscillation amplitude (mm) */
    double Ku;                      /* Ultimate gain */
    int    num_cycles;              /* Number of complete cycles measured */
    int    valid;                   /* 1 if measurement is reliable */
};

/* =========================================================================
 * Inertia Measurement
 * ========================================================================= */

struct inertia_config {
    double test_velocity;           /* Target velocity for ramp (mm/s) */
    double ramp_time;               /* Acceleration time (seconds) */
    int    num_samples;             /* Encoder samples to average */
};

struct inertia_result {
    double J_kg_m2;                 /* Load inertia in kg*m^2 */
    double friction_torque;         /* Average friction torque during move */
    double ff_velocity;             /* Computed velocity feedforward gain */
    int    valid;
};

/* =========================================================================
 * Friction Characterization
 * ========================================================================= */

struct friction_result {
    double Fs_positive;             /* Static friction, positive dir (N) */
    double Fs_negative;             /* Static friction, negative dir (N) */
    double Fv;                      /* Viscous friction coefficient (N*s/mm) */
    double deadband_mm_s;           /* Velocity below which friction dominates */
    int    valid;
};

/* =========================================================================
 * Validation
 * ========================================================================= */

#define VALIDATION_MAX_OVERSHOOT        5.0     /* percent */
#define VALIDATION_MAX_SETTLING_TIME    100.0   /* ms */
#define VALIDATION_MAX_SS_ERROR         0.01    /* mm */
#define VALIDATION_MAX_ITERATIONS       3

struct validation_result {
    double rise_time_ms;
    double overshoot_percent;
    double settling_time_ms;
    double steady_state_error_mm;
    double tracking_error_rms_mm;
    int    iterations;
    int    passed;
};

/* =========================================================================
 * Computed Gains
 * ========================================================================= */

struct autotune_gains {
    double kp;
    double ki;
    double kd;
    double ff_velocity;
    double ff_acceleration;
    double integral_windup_limit;
    double max_following_error;
};

/* =========================================================================
 * Auto-Tune State Machine
 * ========================================================================= */

/* Internal sub-states for each phase */
enum relay_substate {
    RELAY_INIT = 0,
    RELAY_WAIT_FIRST_CROSS,
    RELAY_OSCILLATING,
    RELAY_MEASURING,
    RELAY_DONE,
};

enum inertia_substate {
    INERTIA_INIT = 0,
    INERTIA_ACCELERATING,
    INERTIA_DECELERATING,
    INERTIA_DONE,
};

enum friction_substate {
    FRICTION_INIT = 0,
    FRICTION_POSITIVE,
    FRICTION_NEGATIVE,
    FRICTION_DONE,
};

enum validate_substate {
    VALIDATE_INIT = 0,
    VALIDATE_STEP_FORWARD,
    VALIDATE_SETTLE_FORWARD,
    VALIDATE_STEP_BACK,
    VALIDATE_SETTLE_BACK,
    VALIDATE_TRACKING_TEST,
    VALIDATE_ANALYZE,
    VALIDATE_REFINE,
    VALIDATE_DONE,
};

/* Complete auto-tune state for one axis */
struct autotune_state {
    enum autotune_phase phase;
    int    axis;

    /* Configuration */
    double travel_mm;
    int    aggression;              /* enum autotune_aggression */
    int    validate_only;

    /* Phase contexts */
    struct relay_feedback_config relay_config;
    struct relay_feedback_result relay_result;
    enum relay_substate          relay_sub;

    struct inertia_config        inertia_config;
    struct inertia_result        inertia_result;
    enum inertia_substate        inertia_sub;

    struct friction_result       friction_result;
    enum friction_substate       friction_sub;

    struct autotune_gains        computed_gains;
    struct validation_result     validation_result;
    enum validate_substate       validate_sub;

    /* Relay feedback working data */
    double relay_output;            /* Current relay output (+/- amplitude) */
    double relay_center;            /* Center position for oscillation */
    int    relay_direction;         /* +1 or -1 */
    int    relay_crossings;         /* Number of zero crossings */
    double relay_cross_times[64];   /* Timestamps of zero crossings */
    double relay_peaks[64];         /* Peak positions */
    int    relay_peak_count;

    /* Inertia working data */
    double inertia_start_pos;
    double inertia_start_vel;
    double inertia_torque_sum;
    double inertia_accel_sum;
    int    inertia_samples;

    /* Friction working data */
    double friction_torques_pos[32];
    double friction_velocities_pos[32];
    double friction_torques_neg[32];
    double friction_velocities_neg[32];
    int    friction_sample_count_pos;
    int    friction_sample_count_neg;
    double friction_move_start;

    /* Validation working data */
    double validate_start_pos;
    double validate_step_size;
    double validate_target;
    int    validate_iteration;
    double validate_peak_pos;
    double validate_settle_start_cycle;
    double validate_tracking_error_sum;
    int    validate_tracking_samples;

    /* Data recording for analysis */
    double position_log[16384];
    double velocity_log[16384];
    double torque_log[16384];
    uint32_t log_index;

    /* Timing */
    uint32_t phase_start_cycle;     /* Cycle when current phase started */
    uint32_t cycle_count;           /* Cycles elapsed in current phase */

    /* Progress reporting */
    uint8_t progress_percent;
    char    status_message[128];

    /* Encoder config needed for unit conversion */
    int32_t encoder_resolution;
    double  rotation_distance;
};

/*
 * Initialize auto-tune state for an axis.
 */
void autotune_init(struct autotune_state *state, int axis,
                   int32_t encoder_resolution, double rotation_distance);

/*
 * Start auto-tune with the given configuration.
 */
void autotune_start(struct autotune_state *state,
                    double travel_mm, int aggression, int validate_only,
                    double current_position, double max_velocity);

/*
 * Update auto-tune for one cycle.
 * Called every EtherCAT cycle (1ms) when auto-tune is active.
 *
 * Parameters:
 *   state      - auto-tune state
 *   servo      - current servo state (position, velocity, torque)
 *   vel_cmd    - output: velocity command to send to drive (mm/s)
 *
 * Returns:
 *   1 if auto-tune is still running, 0 if complete or failed.
 */
int autotune_update(struct autotune_state *state,
                    const struct servo_state *servo,
                    double *vel_cmd);

/*
 * Compute PID gains from relay feedback, inertia, and friction results.
 */
void compute_pid_gains(const struct relay_feedback_result *relay,
                       const struct inertia_result *inertia,
                       const struct friction_result *friction,
                       int aggression,
                       struct autotune_gains *output);

/*
 * Copy auto-tune results to shared memory status structure.
 */
void autotune_copy_to_shm(const struct autotune_state *state,
                           struct autotune_status *shm_status);

#endif /* KLIPPER_ETHERCAT_AUTOTUNE_H */
