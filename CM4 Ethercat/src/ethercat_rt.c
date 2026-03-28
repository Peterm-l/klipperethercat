/*
 * ethercat_rt.c — Main EtherCAT Real-Time Process
 *
 * This is the hard real-time process that runs on the isolated CPU core.
 * It handles:
 *   - EtherCAT frame exchange at 1kHz via IgH EtherLab
 *   - CiA 402 drive state machine for each axis
 *   - Trajectory interpolation from shared memory
 *   - PID position loop with velocity feedforward
 *   - Auto-tune execution
 *   - Following error monitoring
 *
 * CRITICAL: This process MUST run on an isolated CPU core with SCHED_FIFO.
 * All memory must be locked before entering the RT loop.
 * No malloc, no printf, no file I/O in the RT loop.
 */

#define _GNU_SOURCE
#include <sched.h>
#include <sys/mman.h>
#include <sys/resource.h>
#include <pthread.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <signal.h>
#include <unistd.h>
#include <getopt.h>
#include <math.h>

#include "ecrt.h"  /* IgH EtherLab userspace API */

#include "shared_mem.h"
#include "lc10e_pdo.h"
#include "cia402.h"
#include "trajectory_interp.h"
#include "pid_controller.h"
#include "autotune.h"

/* =========================================================================
 * Configuration
 * ========================================================================= */

#define RT_CPU_CORE         3
#define RT_PRIORITY         95
#define CYCLE_TIME_NS       1000000     /* 1ms */
#define MAX_RT_AXES         6
#define STACK_PREFAULT_SIZE (8192 * 64) /* 512KB */

/* Per-axis configuration (loaded from config file) */
struct axis_config {
    int     enabled;
    int     slave_position;
    int32_t encoder_resolution;
    double  rotation_distance;
    double  max_velocity;
    double  max_accel;
    double  max_following_error;
    double  pid_kp, pid_ki, pid_kd;
    double  ff_velocity;
    char    name[16];
};

/* Global configuration */
struct rt_config {
    int  num_axes;
    int  rt_cpu_core;
    int  rt_priority;
    int  cycle_time_us;
    char config_path[256];
    struct axis_config axes[MAX_RT_AXES];
};

/* =========================================================================
 * Global State
 * ========================================================================= */

static volatile sig_atomic_t running = 1;
static struct rt_config config;

/* EtherCAT */
static ec_master_t *master = NULL;
static ec_domain_t *domain = NULL;
static uint8_t *domain_pd = NULL;  /* Process data pointer */

/* Per-axis EtherCAT offsets */
struct axis_pdo_offsets {
    /* RxPDO (master → slave) offsets in domain process data */
    unsigned int off_controlword;
    unsigned int off_target_position;
    unsigned int off_target_velocity;
    unsigned int off_touch_probe;
    unsigned int off_torque_limit;
    unsigned int off_modes_of_operation;

    /* TxPDO (slave → master) offsets in domain process data */
    unsigned int off_error_code;
    unsigned int off_statusword;
    unsigned int off_position_actual;
    unsigned int off_velocity_actual;
    unsigned int off_torque_actual;
    unsigned int off_following_error;
    unsigned int off_touch_probe_status;
    unsigned int off_digital_inputs;
    unsigned int off_modes_display;
};

static struct axis_pdo_offsets pdo_offsets[MAX_RT_AXES];
static struct cia402_axis cia402_axes[MAX_RT_AXES];
static struct pid_state pid_states[MAX_RT_AXES];
static struct autotune_state autotune_states[MAX_RT_AXES];
static struct trajectory_interpolator trajectory;

/* Following error statistics */
struct fe_stats {
    double window[1000];
    int    window_idx;
    int    window_full;
    double sum_sq;
    double current_rms;
    double baseline_rms;
    double peak_error;
    uint32_t warn_counter;  /* Cycles where rms > 1.5x baseline */
};

static struct fe_stats fe_stats[MAX_RT_AXES];

/* =========================================================================
 * Signal Handler
 * ========================================================================= */

static void signal_handler(int sig)
{
    (void)sig;
    running = 0;
}

/* =========================================================================
 * Configuration Parser
 * ========================================================================= */

static int parse_config(const char *path)
{
    FILE *f = fopen(path, "r");
    if (!f) {
        fprintf(stderr, "Cannot open config file: %s\n", path);
        return -1;
    }

    /* Defaults */
    config.num_axes = 3;
    config.rt_cpu_core = RT_CPU_CORE;
    config.rt_priority = RT_PRIORITY;
    config.cycle_time_us = CYCLE_TIME_NS / 1000;

    char line[256];
    int current_axis = -1;

    while (fgets(line, sizeof(line), f)) {
        /* Strip comments and whitespace */
        char *comment = strchr(line, '#');
        if (comment) *comment = '\0';
        char *p = line;
        while (*p == ' ' || *p == '\t') p++;
        if (*p == '\0' || *p == '\n') continue;

        /* Section headers */
        if (*p == '[') {
            if (strncmp(p, "[general]", 9) == 0) {
                current_axis = -1;
            } else if (sscanf(p, "[axis_%d]", &current_axis) == 1) {
                if (current_axis >= 0 && current_axis < MAX_RT_AXES) {
                    config.axes[current_axis].enabled = 1;
                }
            }
            continue;
        }

        /* Key = value parsing */
        char key[64], value[128];
        if (sscanf(p, "%63[^= ] = %127[^\n]", key, value) != 2)
            continue;

        if (current_axis < 0) {
            /* General section */
            if (strcmp(key, "num_axes") == 0)
                config.num_axes = atoi(value);
            else if (strcmp(key, "rt_cpu_core") == 0)
                config.rt_cpu_core = atoi(value);
            else if (strcmp(key, "rt_priority") == 0)
                config.rt_priority = atoi(value);
            else if (strcmp(key, "cycle_time_us") == 0)
                config.cycle_time_us = atoi(value);
        } else if (current_axis < MAX_RT_AXES) {
            /* Axis section */
            struct axis_config *ac = &config.axes[current_axis];
            if (strcmp(key, "name") == 0)
                strncpy(ac->name, value, sizeof(ac->name) - 1);
            else if (strcmp(key, "slave_position") == 0)
                ac->slave_position = atoi(value);
            else if (strcmp(key, "encoder_resolution") == 0)
                ac->encoder_resolution = atoi(value);
            else if (strcmp(key, "rotation_distance") == 0)
                ac->rotation_distance = atof(value);
            else if (strcmp(key, "max_velocity") == 0)
                ac->max_velocity = atof(value);
            else if (strcmp(key, "max_accel") == 0)
                ac->max_accel = atof(value);
            else if (strcmp(key, "max_following_error") == 0)
                ac->max_following_error = atof(value);
        }
    }

    fclose(f);
    return 0;
}

/* =========================================================================
 * RT Setup
 * ========================================================================= */

static int setup_realtime(void)
{
    int ret;

    /* 1. Pin to isolated CPU core */
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(config.rt_cpu_core, &cpuset);
    ret = sched_setaffinity(0, sizeof(cpuset), &cpuset);
    if (ret != 0) {
        fprintf(stderr, "FATAL: Failed to pin to CPU %d: %s\n",
                config.rt_cpu_core, strerror(errno));
        return -1;
    }
    fprintf(stderr, "Pinned to CPU core %d\n", config.rt_cpu_core);

    /* 2. Set SCHED_FIFO */
    struct sched_param param;
    param.sched_priority = config.rt_priority;
    ret = sched_setscheduler(0, SCHED_FIFO, &param);
    if (ret != 0) {
        fprintf(stderr, "FATAL: Failed to set SCHED_FIFO priority %d: %s\n",
                config.rt_priority, strerror(errno));
        return -1;
    }
    fprintf(stderr, "Set SCHED_FIFO priority %d\n", config.rt_priority);

    /* 3. Lock all memory */
    ret = mlockall(MCL_CURRENT | MCL_FUTURE);
    if (ret != 0) {
        fprintf(stderr, "WARNING: mlockall failed: %s\n", strerror(errno));
    } else {
        fprintf(stderr, "All memory locked\n");
    }

    /* 4. Pre-fault the stack */
    {
        volatile char stack_prefault[STACK_PREFAULT_SIZE];
        memset((void *)stack_prefault, 0, sizeof(stack_prefault));
    }

    /* 5. Set CPU governor to performance */
    char path[128];
    snprintf(path, sizeof(path),
             "/sys/devices/system/cpu/cpu%d/cpufreq/scaling_governor",
             config.rt_cpu_core);
    FILE *f = fopen(path, "w");
    if (f) {
        fprintf(f, "performance");
        fclose(f);
    }

    fprintf(stderr, "Real-time setup complete\n");
    return 0;
}

/* =========================================================================
 * EtherCAT Initialization
 * ========================================================================= */

static int init_ethercat(void)
{
    /* Request the EtherCAT master */
    master = ecrt_request_master(0);
    if (!master) {
        fprintf(stderr, "FATAL: Failed to request EtherCAT master 0\n");
        fprintf(stderr, "  Is the EtherCAT master service running?\n");
        fprintf(stderr, "  Run: sudo systemctl start ethercat\n");
        return -1;
    }
    fprintf(stderr, "EtherCAT master acquired\n");

    /* Create a process data domain */
    domain = ecrt_master_create_domain(master);
    if (!domain) {
        fprintf(stderr, "FATAL: Failed to create EtherCAT domain\n");
        return -1;
    }

    /* Configure slave PDO mappings for each axis */
    for (int i = 0; i < config.num_axes; i++) {
        struct axis_config *ac = &config.axes[i];
        if (!ac->enabled) continue;

        fprintf(stderr, "Configuring axis %d (slave %d): %s\n",
                i, ac->slave_position, ac->name);

        /* Get slave configuration */
        ec_slave_config_t *sc = ecrt_master_slave_config(
            master, 0, ac->slave_position,
            LC10E_VENDOR_ID, LC10E_PRODUCT_CODE);

        if (!sc) {
            fprintf(stderr, "FATAL: Failed to configure slave at position %d\n",
                    ac->slave_position);
            return -1;
        }

        /* Load ESI XML if available (overrides SII EEPROM data).
         * Some LC10E drives ship with incorrect SII configuration,
         * so the ESI XML ensures correct PDO mapping. */
        {
            const char *esi_path = "/etc/klipper-ethercat/esi/lichuan_lc10e.xml";
            if (access(esi_path, R_OK) == 0) {
                fprintf(stderr, "  Loading ESI XML: %s\n", esi_path);
                /* Note: ecrt_master_set_send_interval may be needed.
                 * The ESI is loaded by the EtherCAT master at startup
                 * via /etc/ethercat.conf or the ethercat tool:
                 *   ethercat xml -p0 < lichuan_lc10e.xml
                 * The code here just verifies it's available. */
            } else {
                fprintf(stderr, "  WARNING: ESI XML not found at %s\n", esi_path);
                fprintf(stderr, "  Using hardcoded PDO mapping (should work for LC10E V1.04)\n");
            }
        }

        /* Configure RxPDO (master → slave): 0x1703 for CSV mode */
        ec_pdo_entry_info_t rxpdo_entries[] = {
            {OD_CONTROLWORD,         0, 16},  /* Controlword */
            {OD_TARGET_POSITION,     0, 32},  /* Target Position */
            {OD_TARGET_VELOCITY,     0, 32},  /* Target Velocity */
            {OD_TOUCH_PROBE_FUNCTION,0, 16},  /* Touch Probe Function */
            {OD_TORQUE_LIMIT,        0, 16},  /* Torque Limit */
            {OD_MODES_OF_OPERATION,  0,  8},  /* Modes of Operation */
        };

        /* Configure TxPDO (slave → master): 0x1B02 */
        ec_pdo_entry_info_t txpdo_entries[] = {
            {OD_ERROR_CODE,                0, 16},  /* Error Code */
            {OD_STATUSWORD,                0, 16},  /* Statusword */
            {OD_POSITION_ACTUAL,           0, 32},  /* Position Actual */
            {OD_VELOCITY_ACTUAL,           0, 32},  /* Velocity Actual */
            {OD_TORQUE_ACTUAL,             0, 16},  /* Torque Actual */
            {OD_FOLLOWING_ERROR_ACTUAL,    0, 32},  /* Following Error */
            {OD_TOUCH_PROBE_STATUS,        0, 16},  /* Touch Probe Status */
            {OD_DIGITAL_INPUTS,            0, 32},  /* Digital Inputs */
            {OD_MODES_OF_OPERATION_DISP,   0,  8},  /* Modes Display */
        };

        ec_pdo_info_t pdos[] = {
            {LC10E_RXPDO_INDEX, 6, rxpdo_entries},  /* RxPDO 0x1703 */
            {LC10E_TXPDO_INDEX, 9, txpdo_entries},  /* TxPDO 0x1B02 */
        };

        ec_sync_info_t syncs[] = {
            {0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DEFAULT},
            {1, EC_DIR_INPUT,  0, NULL, EC_WD_DEFAULT},
            {2, EC_DIR_OUTPUT, 1, &pdos[0], EC_WD_ENABLE},
            {3, EC_DIR_INPUT,  1, &pdos[1], EC_WD_DISABLE},
            {0xFF}
        };

        if (ecrt_slave_config_pdos(sc, EC_END, syncs)) {
            fprintf(stderr, "FATAL: Failed to configure PDOs for slave %d\n",
                    ac->slave_position);
            return -1;
        }

        /* Register PDO entries in the domain and get offsets */
        struct axis_pdo_offsets *off = &pdo_offsets[i];

        off->off_controlword = ecrt_slave_config_reg_pdo_entry(
            sc, OD_CONTROLWORD, 0, domain, NULL);
        off->off_target_position = ecrt_slave_config_reg_pdo_entry(
            sc, OD_TARGET_POSITION, 0, domain, NULL);
        off->off_target_velocity = ecrt_slave_config_reg_pdo_entry(
            sc, OD_TARGET_VELOCITY, 0, domain, NULL);
        off->off_touch_probe = ecrt_slave_config_reg_pdo_entry(
            sc, OD_TOUCH_PROBE_FUNCTION, 0, domain, NULL);
        off->off_torque_limit = ecrt_slave_config_reg_pdo_entry(
            sc, OD_TORQUE_LIMIT, 0, domain, NULL);
        off->off_modes_of_operation = ecrt_slave_config_reg_pdo_entry(
            sc, OD_MODES_OF_OPERATION, 0, domain, NULL);

        off->off_error_code = ecrt_slave_config_reg_pdo_entry(
            sc, OD_ERROR_CODE, 0, domain, NULL);
        off->off_statusword = ecrt_slave_config_reg_pdo_entry(
            sc, OD_STATUSWORD, 0, domain, NULL);
        off->off_position_actual = ecrt_slave_config_reg_pdo_entry(
            sc, OD_POSITION_ACTUAL, 0, domain, NULL);
        off->off_velocity_actual = ecrt_slave_config_reg_pdo_entry(
            sc, OD_VELOCITY_ACTUAL, 0, domain, NULL);
        off->off_torque_actual = ecrt_slave_config_reg_pdo_entry(
            sc, OD_TORQUE_ACTUAL, 0, domain, NULL);
        off->off_following_error = ecrt_slave_config_reg_pdo_entry(
            sc, OD_FOLLOWING_ERROR_ACTUAL, 0, domain, NULL);
        off->off_touch_probe_status = ecrt_slave_config_reg_pdo_entry(
            sc, OD_TOUCH_PROBE_STATUS, 0, domain, NULL);
        off->off_digital_inputs = ecrt_slave_config_reg_pdo_entry(
            sc, OD_DIGITAL_INPUTS, 0, domain, NULL);
        off->off_modes_display = ecrt_slave_config_reg_pdo_entry(
            sc, OD_MODES_OF_OPERATION_DISP, 0, domain, NULL);

        /* Configure watchdog */
        ecrt_slave_config_watchdog(sc, LC10E_WATCHDOG_TIMEOUT_MS, 0);

        fprintf(stderr, "  Slave %d PDO mapping configured (RxPDO 0x%04X, TxPDO 0x%04X)\n",
                ac->slave_position, LC10E_RXPDO_INDEX, LC10E_TXPDO_INDEX);
    }

    /* Activate the EtherCAT master */
    if (ecrt_master_activate(master)) {
        fprintf(stderr, "FATAL: Failed to activate EtherCAT master\n");
        return -1;
    }

    /* Get the process data memory */
    domain_pd = ecrt_domain_data(domain);
    if (!domain_pd) {
        fprintf(stderr, "FATAL: Failed to get domain process data pointer\n");
        return -1;
    }

    fprintf(stderr, "EtherCAT master activated, %d axes configured\n",
            config.num_axes);
    return 0;
}

/* =========================================================================
 * Following Error Monitor
 * ========================================================================= */

static void fe_stats_init(struct fe_stats *s)
{
    memset(s, 0, sizeof(*s));
}

static void fe_stats_update(struct fe_stats *s, double error)
{
    double abs_err = fabs(error);

    /* Update peak */
    if (abs_err > s->peak_error)
        s->peak_error = abs_err;

    /* Rolling RMS window */
    double old_val = s->window[s->window_idx];
    s->window[s->window_idx] = error * error;

    if (s->window_full) {
        s->sum_sq -= old_val;
    }
    s->sum_sq += error * error;

    s->window_idx++;
    if (s->window_idx >= 1000) {
        s->window_idx = 0;
        s->window_full = 1;
    }

    int count = s->window_full ? 1000 : s->window_idx;
    if (count > 0) {
        s->current_rms = sqrt(s->sum_sq / count);
    }

    /* Degradation warning */
    if (s->baseline_rms > 0.0001 &&
        s->current_rms > s->baseline_rms * 1.5) {
        s->warn_counter++;
    } else {
        s->warn_counter = 0;
    }
}

/* =========================================================================
 * Command Handler
 * ========================================================================= */

static void handle_commands(struct klipper_ethercat_shm *shm)
{
    uint8_t cmd = atomic_load_explicit(&shm->command, memory_order_acquire);
    if (cmd == CMD_NONE)
        return;

    uint8_t axis = atomic_load_explicit(&shm->command_axis,
                                         memory_order_relaxed);

    switch (cmd) {
    case CMD_ENABLE:
        if (axis < config.num_axes) {
            cia402_request_enable(&cia402_axes[axis]);
        }
        break;

    case CMD_DISABLE:
        if (axis < config.num_axes) {
            cia402_request_disable(&cia402_axes[axis]);
        }
        break;

    case CMD_HOME:
        /* Homing handled via absolute encoder — just mark as done */
        if (axis < config.num_axes) {
            shm->axes[axis].homing_complete = 1;
            interp_reset(&trajectory, axis,
                         shm->axes[axis].actual_position);
        }
        break;

    case CMD_ESTOP:
        /* Emergency stop: disable all axes immediately */
        for (int i = 0; i < config.num_axes; i++) {
            cia402_request_quick_stop(&cia402_axes[i]);
            interp_reset(&trajectory, i,
                         shm->axes[i].actual_position);
        }
        break;

    case CMD_AUTOTUNE:
        if (axis < config.num_axes) {
            struct autotune_config *atcfg = &shm->autotune_config[axis];
            autotune_start(&autotune_states[axis],
                           atcfg->travel_mm,
                           atcfg->aggression,
                           atcfg->validate_only,
                           shm->axes[axis].actual_position,
                           config.axes[axis].max_velocity);
        }
        break;

    case CMD_RECORD:
        if (axis < config.num_axes) {
            atomic_store(&shm->position_log[axis].recording, 1);
            shm->position_log[axis].log_count = 0;
        }
        break;

    case CMD_STOP_RECORD:
        if (axis < config.num_axes) {
            atomic_store(&shm->position_log[axis].recording, 0);
        }
        break;
    }

    /* Acknowledge */
    atomic_store_explicit(&shm->command, CMD_NONE, memory_order_release);
    atomic_store_explicit(&shm->command_ack, cmd, memory_order_release);
}

/* =========================================================================
 * Timing Statistics
 * ========================================================================= */

static void update_timing_stats(struct klipper_ethercat_shm *shm,
                                double cycle_us)
{
    shm->status.cycle_count++;

    if (cycle_us > shm->status.max_cycle_time_us)
        shm->status.max_cycle_time_us = cycle_us;

    /* Running average */
    double alpha = 0.001;
    shm->status.avg_cycle_time_us =
        alpha * cycle_us + (1.0 - alpha) * shm->status.avg_cycle_time_us;
}

/* =========================================================================
 * Real-Time Loop
 * ========================================================================= */

static void rt_loop(struct klipper_ethercat_shm *shm)
{
    struct timespec next_cycle;
    clock_gettime(CLOCK_MONOTONIC, &next_cycle);

    uint32_t cycle_time_ns = config.cycle_time_us * 1000;
    double dt = config.cycle_time_us / 1e6;

    while (running) {
        /* Advance to next cycle time */
        next_cycle.tv_nsec += cycle_time_ns;
        if (next_cycle.tv_nsec >= 1000000000) {
            next_cycle.tv_nsec -= 1000000000;
            next_cycle.tv_sec++;
        }

        /* Sleep until next cycle */
        int ret = clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME,
                                   &next_cycle, NULL);
        if (ret != 0 && ret != EINTR)
            continue;

        /* --- START OF TIME-CRITICAL SECTION --- */
        struct timespec cycle_start;
        clock_gettime(CLOCK_MONOTONIC, &cycle_start);

        uint64_t now_ns = (uint64_t)cycle_start.tv_sec * 1000000000ULL
                        + (uint64_t)cycle_start.tv_nsec;

        /* 1. Receive EtherCAT frame */
        ecrt_master_receive(master);
        ecrt_domain_process(domain);

        /* 2. Read servo feedback for each axis */
        for (int i = 0; i < config.num_axes; i++) {
            struct axis_config *ac = &config.axes[i];
            struct axis_pdo_offsets *off = &pdo_offsets[i];

            /* Read TxPDO data */
            uint16_t statusword = EC_READ_U16(domain_pd + off->off_statusword);
            uint16_t error_code = EC_READ_U16(domain_pd + off->off_error_code);
            int32_t  pos_raw = EC_READ_S32(domain_pd + off->off_position_actual);
            int32_t  vel_raw = EC_READ_S32(domain_pd + off->off_velocity_actual);
            int16_t  torque_raw = EC_READ_S16(domain_pd + off->off_torque_actual);
            int32_t  fe_raw = EC_READ_S32(domain_pd + off->off_following_error);

            /* Convert to physical units */
            double pos_mm = counts_to_mm(pos_raw, ac->encoder_resolution,
                                          ac->rotation_distance);
            double vel_mm = counts_to_mmps(vel_raw, ac->encoder_resolution,
                                            ac->rotation_distance);
            double torque_pct = torque_to_percent(torque_raw);

            /* Update shared memory servo state */
            shm->axes[i].actual_position = pos_mm;
            shm->axes[i].actual_velocity = vel_mm;
            shm->axes[i].torque_percent = torque_pct;
            shm->axes[i].cia402_status = statusword;
            shm->axes[i].error_code = error_code;

            /* 3. CiA 402 state machine */
            uint16_t controlword = cia402_update(&cia402_axes[i],
                                                  statusword, error_code,
                                                  shm->status.cycle_count);
            shm->axes[i].state = (uint8_t)cia402_axes[i].current_state;

            /* Write RxPDO: controlword and mode */
            EC_WRITE_U16(domain_pd + off->off_controlword, controlword);
            EC_WRITE_S8(domain_pd + off->off_modes_of_operation, MODE_CSV);
        }

        /* 4. Handle commands from klippy */
        handle_commands(shm);

        /* 5. Read PID params from shared memory (klippy may update) */
        /* (pid_params are read directly from shm->pid[] each cycle) */

        /* 6. Compute motion for enabled axes */
        int any_autotune_active = 0;

        /* Update trajectory interpolation */
        interp_update(&trajectory, shm, now_ns);

        for (int i = 0; i < config.num_axes; i++) {
            struct axis_config *ac = &config.axes[i];
            struct axis_pdo_offsets *off = &pdo_offsets[i];

            /* Skip if drive not enabled */
            if (!cia402_is_enabled(&cia402_axes[i]))
                continue;

            double vel_cmd = 0.0;

            /* Check if auto-tune is active for this axis */
            if (autotune_states[i].phase != AUTOTUNE_IDLE &&
                autotune_states[i].phase != AUTOTUNE_COMPLETE &&
                autotune_states[i].phase != AUTOTUNE_FAILED) {

                /* Auto-tune controls the axis */
                autotune_update(&autotune_states[i],
                                &shm->axes[i], &vel_cmd);
                autotune_copy_to_shm(&autotune_states[i],
                                      &shm->autotune_status[i]);
                any_autotune_active = 1;
            } else {
                /* Normal trajectory tracking with PID */
                double target_pos = interp_get_position(&trajectory, i);
                double target_vel = interp_get_velocity(&trajectory, i);
                double target_accel = interp_get_accel(&trajectory, i);
                double actual_pos = shm->axes[i].actual_position;
                double error = target_pos - actual_pos;

                shm->axes[i].commanded_position = target_pos;
                shm->axes[i].following_error = error;

                /* PID + feedforward */
                vel_cmd = pid_compute_with_ff(&pid_states[i],
                                              &shm->pid[i],
                                              error, target_vel,
                                              target_accel, dt);

                /* Following error fault check */
                double max_fe = shm->pid[i].max_following_error;
                if (max_fe > 0.0 && fabs(error) > max_fe) {
                    /* Following error exceeded — fault the axis */
                    cia402_request_quick_stop(&cia402_axes[i]);
                    interp_reset(&trajectory, i, actual_pos);
                    vel_cmd = 0.0;
                }
            }

            /* Convert velocity from mm/s to encoder counts/s */
            int32_t vel_counts = mmps_to_counts(vel_cmd,
                ac->encoder_resolution, ac->rotation_distance);

            /* Write target velocity to RxPDO */
            EC_WRITE_S32(domain_pd + off->off_target_velocity, vel_counts);

            /* Also write zero torque limit (unlimited) — use max */
            EC_WRITE_S16(domain_pd + off->off_torque_limit, 1000); /* 100.0% */
        }

        /* 7. Update following error monitoring */
        for (int i = 0; i < config.num_axes; i++) {
            fe_stats_update(&fe_stats[i], shm->axes[i].following_error);

            shm->following_error_stats[i].current_rms = fe_stats[i].current_rms;
            shm->following_error_stats[i].baseline_rms = fe_stats[i].baseline_rms;
            shm->following_error_stats[i].peak_error = fe_stats[i].peak_error;
            shm->following_error_stats[i].degradation_warning =
                (fe_stats[i].warn_counter > 10000) ? 1 : 0;  /* ~10s at 1kHz */
        }

        /* 8. Position logging (for SERVO_PLOT) */
        for (int i = 0; i < config.num_axes; i++) {
            if (atomic_load(&shm->position_log[i].recording)) {
                uint32_t idx = shm->position_log[i].log_count;
                if (idx < MAX_POSITION_LOG) {
                    shm->position_log[i].log[idx].time_ms =
                        shm->status.cycle_count * dt * 1000.0;
                    shm->position_log[i].log[idx].commanded =
                        shm->axes[i].commanded_position;
                    shm->position_log[i].log[idx].actual =
                        shm->axes[i].actual_position;
                    shm->position_log[i].log[idx].error =
                        shm->axes[i].following_error;
                    shm->position_log[i].log[idx].torque =
                        shm->axes[i].torque_percent;
                    shm->position_log[i].log_count = idx + 1;
                }
            }
        }

        /* 9. Send EtherCAT frame */
        ecrt_domain_queue(domain);
        ecrt_master_send(master);

        /* --- END OF TIME-CRITICAL SECTION --- */

        /* 10. Update timing stats */
        struct timespec cycle_end;
        clock_gettime(CLOCK_MONOTONIC, &cycle_end);
        double cycle_us = (double)(cycle_end.tv_sec - cycle_start.tv_sec) * 1e6
                        + (double)(cycle_end.tv_nsec - cycle_start.tv_nsec) / 1e3;
        update_timing_stats(shm, cycle_us);

        /* Check for overrun */
        if (cycle_us > (double)config.cycle_time_us) {
            shm->status.cycle_overruns++;
        }
    }
}

/* =========================================================================
 * Main
 * ========================================================================= */

int main(int argc, char *argv[])
{
    /* Parse command line */
    const char *config_path = "/etc/klipper-ethercat/config.ini";
    int opt;
    while ((opt = getopt(argc, argv, "c:")) != -1) {
        switch (opt) {
        case 'c':
            config_path = optarg;
            break;
        default:
            fprintf(stderr, "Usage: %s [-c config_file]\n", argv[0]);
            return 1;
        }
    }

    /* Also accept --config */
    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--config") == 0 && i + 1 < argc) {
            config_path = argv[i + 1];
            break;
        }
    }

    fprintf(stderr, "=== Klipper EtherCAT RT Process ===\n");
    fprintf(stderr, "Config: %s\n", config_path);

    /* Install signal handlers */
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    /* Parse configuration */
    if (parse_config(config_path) != 0) {
        return 1;
    }
    fprintf(stderr, "Config: %d axes, cycle time %d us, RT core %d\n",
            config.num_axes, config.cycle_time_us, config.rt_cpu_core);

    /* Open shared memory (created by klippy) */
    struct klipper_ethercat_shm *shm = shm_open_existing();
    if (!shm) {
        fprintf(stderr, "FATAL: Cannot open shared memory. "
                "Is klippy running?\n");
        return 1;
    }

    /* Initialize per-axis state */
    interp_init(&trajectory, config.num_axes);
    for (int i = 0; i < config.num_axes; i++) {
        cia402_init(&cia402_axes[i], i);
        pid_init(&pid_states[i]);
        autotune_init(&autotune_states[i], i,
                      config.axes[i].encoder_resolution,
                      config.axes[i].rotation_distance);
        fe_stats_init(&fe_stats[i]);
    }

    /* Initialize EtherCAT BEFORE going real-time
     * (involves kernel I/O and memory allocation) */
    if (init_ethercat() != 0) {
        shm_close(shm, 0);
        return 1;
    }

    /* Update shared memory with initial state */
    shm->status.bus_state = BUS_OP;
    shm->status.num_slaves = config.num_axes;

    /* NOW go real-time — after all allocation and I/O */
    if (setup_realtime() != 0) {
        fprintf(stderr, "FATAL: Real-time setup failed\n");
        ecrt_release_master(master);
        shm_close(shm, 0);
        return 1;
    }

    fprintf(stderr, "Entering real-time loop...\n");

    /* Enter the RT loop (runs until SIGINT/SIGTERM) */
    rt_loop(shm);

    /* Cleanup */
    fprintf(stderr, "Shutting down...\n");

    /* Disable all drives before exit */
    for (int i = 0; i < config.num_axes; i++) {
        if (domain_pd) {
            EC_WRITE_S32(domain_pd + pdo_offsets[i].off_target_velocity, 0);
            EC_WRITE_U16(domain_pd + pdo_offsets[i].off_controlword,
                         CW_DISABLE_VOLTAGE);
        }
    }
    if (domain_pd) {
        ecrt_domain_queue(domain);
        ecrt_master_send(master);
    }

    ecrt_release_master(master);
    shm_close(shm, 0);

    fprintf(stderr, "EtherCAT RT process terminated\n");
    return 0;
}
