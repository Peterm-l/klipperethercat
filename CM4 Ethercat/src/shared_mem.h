/*
 * shared_mem.h — Shared Memory Interface for Klipper EtherCAT
 *
 * Defines the lock-free shared memory structures used for communication
 * between klippy (Python, non-RT) and ethercat_rt (C, RT).
 *
 * The shared memory is created by klippy and attached to by ethercat_rt.
 * All data exchange uses atomic operations — no mutexes, no syscalls in
 * the RT path.
 */

#ifndef KLIPPER_ETHERCAT_SHARED_MEM_H
#define KLIPPER_ETHERCAT_SHARED_MEM_H

#include <stdint.h>
#include <stdatomic.h>

/* Shared memory identification */
#define SHM_NAME            "/klipper_ethercat"
#define SHM_MAGIC           0x4B455443  /* "KETC" */
#define SHM_VERSION         1

/* Limits */
#define MAX_TRAJECTORY_SEGMENTS 256
#define MAX_AXES                6
#define MAX_POSITION_LOG        16384   /* ~16 seconds at 1kHz */

/* =========================================================================
 * Trajectory Segment
 * =========================================================================
 * Each segment describes a constant-acceleration move from klippy's motion
 * planner. The RT process interpolates position at 1kHz within each segment.
 *
 * position(t) = start_position + start_velocity * t + 0.5 * accel * t^2
 * velocity(t) = start_velocity + accel * t
 * where t = time elapsed since segment start (seconds)
 */

/* Segment flags */
#define SEGMENT_ACTIVE      (1 << 0)    /* Segment contains valid data */
#define SEGMENT_LAST        (1 << 1)    /* Last segment in current move sequence */
#define SEGMENT_ESTOP       (1 << 2)    /* Emergency stop — decelerate immediately */

struct trajectory_segment {
    uint64_t timestamp_ns;              /* Absolute time when segment starts */
    double   duration;                  /* Segment duration in seconds */
    double   start_position[MAX_AXES];  /* Starting position (mm) */
    double   start_velocity[MAX_AXES];  /* Starting velocity (mm/s) */
    double   accel[MAX_AXES];           /* Acceleration (mm/s^2) */
    uint32_t flags;                     /* SEGMENT_ACTIVE, SEGMENT_LAST, etc. */
    uint32_t _pad;                      /* Alignment padding */
};

/* =========================================================================
 * Trajectory Ring Buffer
 * =========================================================================
 * Lock-free SPSC (single-producer single-consumer) ring buffer.
 * klippy writes segments and increments write_idx.
 * ethercat_rt reads segments and increments read_idx.
 *
 * Buffer is empty when read_idx == write_idx.
 * Buffer is full when (write_idx + 1) % SIZE == read_idx.
 */
struct trajectory_ring_buffer {
    _Atomic uint32_t write_idx;         /* klippy increments after writing */
    _Atomic uint32_t read_idx;          /* ethercat_rt increments after reading */
    struct trajectory_segment segments[MAX_TRAJECTORY_SEGMENTS];
};

/* =========================================================================
 * Servo State (per axis)
 * =========================================================================
 * Written by ethercat_rt every cycle, read by klippy for monitoring.
 */

/* Drive states (maps to CiA 402 state machine) */
enum servo_drive_state {
    DRIVE_NOT_READY         = 0,
    DRIVE_SWITCH_ON_DISABLED = 1,
    DRIVE_READY_TO_SWITCH_ON = 2,
    DRIVE_SWITCHED_ON       = 3,
    DRIVE_OPERATION_ENABLED = 4,
    DRIVE_QUICK_STOP_ACTIVE = 5,
    DRIVE_FAULT_REACTION    = 6,
    DRIVE_FAULT             = 7,
};

struct servo_state {
    double   actual_position;           /* Encoder position (mm) */
    double   actual_velocity;           /* Encoder velocity (mm/s) */
    double   commanded_position;        /* Last commanded position (mm) */
    double   following_error;           /* commanded - actual (mm) */
    double   torque_percent;            /* Current torque output (% of rated) */
    uint16_t cia402_status;             /* Raw CiA 402 status word */
    uint16_t error_code;                /* Drive error code (0x603F) */
    uint8_t  state;                     /* enum servo_drive_state */
    uint8_t  homing_complete;           /* 1 if absolute encoder home is set */
    uint8_t  _pad[2];
};

/* =========================================================================
 * EtherCAT Bus Status
 * =========================================================================
 * Written by ethercat_rt, read by klippy for diagnostics.
 */

enum ethercat_bus_state {
    BUS_INIT    = 0,
    BUS_PREOP   = 1,
    BUS_SAFEOP  = 2,
    BUS_OP      = 3,
};

struct ethercat_status {
    uint32_t cycle_count;               /* Total EtherCAT cycles executed */
    uint32_t cycle_overruns;            /* Cycles that exceeded deadline */
    double   max_cycle_time_us;         /* Worst case cycle time */
    double   avg_cycle_time_us;         /* Average cycle time */
    uint8_t  bus_state;                 /* enum ethercat_bus_state */
    uint8_t  num_slaves;                /* Number of slaves detected */
    uint8_t  _pad[6];
};

/* =========================================================================
 * PID Tuning Parameters (per axis)
 * =========================================================================
 * Written by klippy (from printer.cfg or auto-tune results).
 * Read by ethercat_rt for the position control loop.
 */
struct pid_params {
    double kp;                          /* Proportional gain */
    double ki;                          /* Integral gain */
    double kd;                          /* Derivative gain */
    double ff_velocity;                 /* Velocity feedforward gain (~1.0) */
    double ff_acceleration;             /* Acceleration feedforward gain */
    double max_following_error;         /* Fault threshold (mm) */
    double max_velocity;                /* Velocity limit (mm/s) */
    double max_accel;                   /* Acceleration limit (mm/s^2) */
    double position_scale;              /* Encoder counts per mm */
    double integral_windup_limit;       /* Anti-windup clamp for integral term */
};

/* =========================================================================
 * Commands (klippy → ethercat_rt)
 * =========================================================================
 * Simple command/acknowledge protocol using atomics.
 * klippy writes command + command_axis, ethercat_rt reads and sets command_ack.
 */

enum shm_command {
    CMD_NONE        = 0,
    CMD_ENABLE      = 1,    /* Enable drive (CiA 402 → OPERATION_ENABLED) */
    CMD_DISABLE     = 2,    /* Disable drive (→ SWITCH_ON_DISABLED) */
    CMD_HOME        = 3,    /* Home axis (absolute encoder or torque limit) */
    CMD_ESTOP       = 4,    /* Emergency stop all axes */
    CMD_AUTOTUNE    = 5,    /* Start auto-tune on specified axis */
    CMD_RECORD      = 6,    /* Start position recording (for SERVO_PLOT) */
    CMD_STOP_RECORD = 7,    /* Stop position recording */
};

/* =========================================================================
 * Auto-Tune Structures
 * =========================================================================
 */

/* Auto-tune phases */
enum autotune_phase {
    AUTOTUNE_IDLE           = 0,
    AUTOTUNE_SAFETY_CHECK   = 1,
    AUTOTUNE_RELAY_FEEDBACK = 2,
    AUTOTUNE_INERTIA        = 3,
    AUTOTUNE_FRICTION       = 4,
    AUTOTUNE_COMPUTE_GAINS  = 5,
    AUTOTUNE_VALIDATE       = 6,
    AUTOTUNE_COMPLETE       = 7,
    AUTOTUNE_FAILED         = 8,
};

/* Auto-tune aggression levels */
enum autotune_aggression {
    AGGRESSION_CONSERVATIVE = 0,
    AGGRESSION_MODERATE     = 1,
    AGGRESSION_AGGRESSIVE   = 2,
};

/* Configuration (klippy writes before CMD_AUTOTUNE) */
struct autotune_config {
    double  travel_mm;              /* Safe travel distance from current pos */
    uint8_t aggression;             /* enum autotune_aggression */
    uint8_t validate_only;          /* 1 = skip characterization, just validate */
    uint8_t _pad[6];
};

/* Status/results (ethercat_rt writes, klippy polls) */
struct autotune_status {
    uint8_t  phase;                 /* enum autotune_phase */
    uint8_t  progress_percent;
    uint8_t  _pad[6];
    char     message[128];          /* Human-readable status message */

    /* Results (valid when phase == AUTOTUNE_COMPLETE) */
    double   Tu;                    /* Ultimate period (seconds) */
    double   Au;                    /* Oscillation amplitude (mm) */
    double   J;                     /* Load inertia (kg*m^2) */
    double   Fs;                    /* Static friction (N) */
    double   Fv;                    /* Viscous friction (N*s/mm) */
    double   kp, ki, kd;           /* Computed PID gains */
    double   ff_velocity;           /* Computed feedforward gain */

    /* Validation metrics */
    double   rise_time_ms;
    double   overshoot_percent;
    double   settling_time_ms;
    double   ss_error_mm;           /* Steady-state error */
    double   tracking_rms_mm;       /* Tracking error RMS */
};

/* =========================================================================
 * Following Error Monitoring
 * =========================================================================
 */
struct following_error_stats {
    double   current_rms;           /* Current rolling RMS following error */
    double   baseline_rms;          /* RMS recorded after last auto-tune */
    double   peak_error;            /* Maximum error since last reset */
    uint8_t  degradation_warning;   /* 1 if current_rms > 1.5x baseline >10s */
    uint8_t  _pad[7];
};

/* =========================================================================
 * Position Logging (for SERVO_PLOT)
 * =========================================================================
 */
struct position_log_entry {
    double time_ms;
    double commanded;
    double actual;
    double error;
    double torque;
};

struct position_log {
    _Atomic uint32_t recording;     /* 1 = actively recording */
    uint32_t log_count;             /* Number of entries written */
    struct position_log_entry log[MAX_POSITION_LOG];
};

/* =========================================================================
 * Top-Level Shared Memory Structure
 * =========================================================================
 * This is the single contiguous block of shared memory accessed by both
 * klippy and ethercat_rt.
 */
struct klipper_ethercat_shm {
    /* Header */
    uint32_t magic;                         /* SHM_MAGIC */
    uint32_t version;                       /* SHM_VERSION */

    /* Trajectory data (klippy writes, ethercat_rt reads) */
    struct trajectory_ring_buffer trajectory;

    /* Per-axis servo state (ethercat_rt writes, klippy reads) */
    struct servo_state axes[MAX_AXES];

    /* Bus status (ethercat_rt writes, klippy reads) */
    struct ethercat_status status;

    /* PID parameters (klippy writes, ethercat_rt reads) */
    struct pid_params pid[MAX_AXES];

    /* Command interface (klippy writes, ethercat_rt reads/acks) */
    _Atomic uint8_t command;                /* enum shm_command */
    _Atomic uint8_t command_axis;           /* Which axis for command */
    _Atomic uint8_t command_ack;            /* ethercat_rt sets when done */
    uint8_t _cmd_pad[5];

    /* Auto-tune (per axis) */
    struct autotune_config  autotune_config[MAX_AXES];
    struct autotune_status  autotune_status[MAX_AXES];

    /* Following error monitoring (per axis) */
    struct following_error_stats following_error_stats[MAX_AXES];

    /* Position logging (per axis) */
    struct position_log position_log[MAX_AXES];
};

/* =========================================================================
 * API Functions
 * =========================================================================
 */

/*
 * Create shared memory segment (called by klippy/the creator).
 * Returns pointer to mapped memory, or NULL on failure.
 */
struct klipper_ethercat_shm *shm_create(void);

/*
 * Open existing shared memory segment (called by ethercat_rt).
 * Returns pointer to mapped memory, or NULL on failure.
 */
struct klipper_ethercat_shm *shm_open_existing(void);

/*
 * Close and unmap shared memory.
 * If is_creator is true, also unlinks (destroys) the segment.
 */
void shm_close(struct klipper_ethercat_shm *shm, int is_creator);

/*
 * Ring buffer helper: number of segments available to read.
 */
static inline uint32_t ring_buffer_available(
    const struct trajectory_ring_buffer *rb)
{
    uint32_t w = atomic_load_explicit(&rb->write_idx, memory_order_acquire);
    uint32_t r = atomic_load_explicit(&rb->read_idx, memory_order_relaxed);
    return (w - r) & (MAX_TRAJECTORY_SEGMENTS - 1);
}

/*
 * Ring buffer helper: number of free slots for writing.
 */
static inline uint32_t ring_buffer_free(
    const struct trajectory_ring_buffer *rb)
{
    return MAX_TRAJECTORY_SEGMENTS - 1 - ring_buffer_available(rb);
}

#endif /* KLIPPER_ETHERCAT_SHARED_MEM_H */
