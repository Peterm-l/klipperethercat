# Klipper EtherCAT Servo Integration Project

## Project Instructions for Claude Code

### Overview

Build a Klipper module that enables EtherCAT servo motor control via CSV (Cyclic Synchronous Velocity) mode with closed-loop position feedback and auto-tuning. This replaces stepper motor step/dir output with real-time EtherCAT communication to industrial servo drives, while preserving all existing Klipper features (pressure advance, input shaping, bed mesh, CAN toolhead support).

### Target Hardware

- **Host**: Raspberry Pi CM4 on a BigTreeTech Manta M8P V2.0 board
- **OS**: Raspberry Pi OS with PREEMPT_RT patched kernel
- **Servo Drives**: Lichuan LC10E EtherCAT drives with 23-bit absolute encoder motors
- **Toolhead**: Existing Klipper CAN toolhead board (EBB36/42) for extruder, hotend, fans
- **Additional stepper**: One onboard TMC driver on Manta M8P for NEMA 17 harmonic drive (rotary axis)
- **Networking**: CM4 onboard Ethernet dedicated to EtherCAT, CM4 WiFi for web interface

### Architecture

```
CM4 on Manta M8P V2:
├── Core 0-2 (normal Linux):
│   ├── Klipper (klippy) — motion planning, gcode, pressure advance, input shaping
│   ├── Moonraker — web API
│   ├── Mainsail/Fluidd — web UI
│   └── Linux housekeeping
│
├── Core 3 (isolated, PREEMPT_RT, SCHED_FIFO):
│   └── ethercat_rt — real-time C process
│       ├── IgH EtherLab master (kernel module)
│       ├── CiA 402 state machine
│       ├── CSV mode position loop (PID + velocity feedforward)
│       ├── Trajectory interpolator
│       ├── Auto-tune engine
│       └── Shared memory ring buffer ↔ klippy
│
├── CM4 Ethernet (RJ45 on Manta) → EtherCAT daisy chain
│   └── LC10E servo X → LC10E servo Y → LC10E servo Z
│
├── CAN bus (Manta onboard) → EBB36/42 toolhead
│   └── Extruder stepper, hotend heater, thermistor, fans
│
├── TMC driver slot (Manta onboard) → NEMA 17 + harmonic drive
│
└── CM4 WiFi → network, web interface
```

### Data Flow

```
Gcode → klippy motion planner → trajectory segments → shared memory ring buffer
    → ethercat_rt reads segments → interpolates position at 1kHz
    → computes PID + feedforward → sends velocity command via EtherCAT
    → reads encoder position back → writes to shared memory → klippy reads for monitoring
```

---

## Component Breakdown

### Component 1: System Setup Scripts

Create setup and configuration scripts for the CM4/Manta M8P environment.

#### 1.1 PREEMPT_RT Kernel Setup

```
File: scripts/setup_rt_kernel.sh
```

- Script to build and install PREEMPT_RT patched kernel for CM4
- Use the Raspberry Pi kernel source (6.6.y branch recommended for stability)
- Enable PREEMPT_RT full preemption
- Disable kernel debugging/profiling/tracing for lower latency
- Set 1000Hz periodic ticks (no dynticks)
- Configure kernel command line: `isolcpus=3 nohz_full=3 rcu_nocbs=3`

#### 1.2 IgH EtherLab Master Installation

```
File: scripts/setup_etherlab.sh
```

- Clone and build the IgH EtherLab master (version 1.5.2 or latest stable)
- Build as kernel module targeting the CM4's Ethernet interface
- The CM4 uses the bcmgenet Ethernet driver — configure IgH to use the generic driver or write a minimal native driver
- Install udev rules for /dev/EtherCAT0
- Create systemd service for the EtherCAT master
- Test script to verify slave scanning: `ethercat slaves` should list connected LC10E drives

#### 1.3 CPU Isolation and RT Configuration

```
File: scripts/configure_rt.sh
```

**This script is CRITICAL. Without proper core isolation, the ethercat_rt process will experience multi-millisecond latency spikes from Linux kernel tasks, making servo control unstable.**

The script must do the following (in order):

**a) Kernel command line — isolate core 3 from the Linux scheduler:**

Add to `/boot/firmware/cmdline.txt` (append to existing line, do not create a new line):
```
isolcpus=3 nohz_full=3 rcu_nocbs=3 rcu_nocb_poll
```

- `isolcpus=3` — removes core 3 from the general scheduler. No normal Linux process will be placed on this core. Only processes that explicitly pin themselves to core 3 (via sched_setaffinity) will run there.
- `nohz_full=3` — disables the scheduler tick on core 3 when only one task is running. This eliminates periodic timer interrupts that would wake and disturb the RT process.
- `rcu_nocbs=3` — moves RCU (Read-Copy-Update) callback processing off core 3. RCU callbacks are a major source of latency spikes in PREEMPT_RT kernels.
- `rcu_nocb_poll` — uses polling instead of interrupts for RCU on isolated cores, further reducing interrupt load.

**b) Move all IRQ handlers off core 3:**

```bash
# Move all IRQs to cores 0-2
for irq in /proc/irq/*/smp_affinity_list; do
    echo "0-2" > "$irq" 2>/dev/null
done

# Set default IRQ affinity for new IRQs
echo 7 > /proc/irq/default_smp_affinity   # bitmask: cores 0,1,2
```

Install this as a systemd service that runs at boot, before ethercat_rt starts.

**c) Disable unnecessary services:**

```bash
sudo systemctl disable bluetooth
sudo systemctl disable avahi-daemon
sudo systemctl disable multipathd      # Known to cause RT latency spikes
sudo systemctl disable ModemManager
sudo systemctl disable wpa_supplicant   # Only if using Ethernet, not WiFi
sudo systemctl mask apt-daily.timer
sudo systemctl mask apt-daily-upgrade.timer
```

**d) Set CPU governor to performance (prevent frequency scaling):**

```bash
# Set all cores to performance governor
for cpu in /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor; do
    echo "performance" > "$cpu"
done

# Make persistent via /etc/rc.local or systemd service
```

**e) Configure /etc/security/limits.conf for RT privileges:**

```
# Allow the ethercat_rt process to use real-time scheduling
@realtime   -   rtprio      99
@realtime   -   memlock     unlimited
@realtime   -   nice        -20
```

Add the user running ethercat_rt to the `realtime` group:
```bash
sudo groupadd -f realtime
sudo usermod -aG realtime $USER
```

**f) Use X11, not Wayland:**

```bash
sudo raspi-config nonint do_wayland W1   # Switch to X11
```

Wayland compositing causes significant jitter. If running headless (recommended for production), disable the display manager entirely:
```bash
sudo systemctl set-default multi-user.target
```

**g) Create systemd service for ethercat_rt:**

```ini
# /etc/systemd/system/ethercat-rt.service
[Unit]
Description=Klipper EtherCAT Real-Time Servo Controller
After=ethercat.service
Requires=ethercat.service

[Service]
Type=simple
ExecStart=/usr/local/bin/ethercat_rt --config /etc/klipper-ethercat/config.ini
# Run as root for RT privileges (or use capabilities)
User=root
# Do NOT set CPUAffinity here — the process pins itself in code
# This avoids a race condition where systemd pins before mlockall
LimitMEMLOCK=infinity
LimitRTPRIO=99
LimitNICE=-20
Restart=on-failure
RestartSec=2

[Install]
WantedBy=multi-user.target
```

**h) Verification script — run this after setup to confirm isolation is correct:**

```bash
#!/bin/bash
# scripts/verify_rt_setup.sh

echo "=== RT Setup Verification ==="

# Check kernel is PREEMPT_RT
if uname -v | grep -q "PREEMPT_RT"; then
    echo "[OK] PREEMPT_RT kernel active"
else
    echo "[FAIL] Kernel is NOT PREEMPT_RT"
fi

# Check isolcpus
if cat /proc/cmdline | grep -q "isolcpus=3"; then
    echo "[OK] Core 3 isolated (isolcpus=3)"
else
    echo "[FAIL] Core 3 NOT isolated — add isolcpus=3 to cmdline.txt"
fi

# Check nohz_full
if cat /proc/cmdline | grep -q "nohz_full=3"; then
    echo "[OK] nohz_full=3 set"
else
    echo "[WARN] nohz_full=3 not set — scheduler tick still active on core 3"
fi

# Check rcu_nocbs
if cat /proc/cmdline | grep -q "rcu_nocbs=3"; then
    echo "[OK] rcu_nocbs=3 set"
else
    echo "[WARN] rcu_nocbs=3 not set — RCU callbacks may cause latency on core 3"
fi

# Check no processes on core 3
procs_on_3=$(ps -eo pid,psr,comm | awk '$2 == 3 {print}' | wc -l)
if [ "$procs_on_3" -le 2 ]; then
    echo "[OK] Core 3 has $procs_on_3 processes (expected: 0-2 kernel threads)"
else
    echo "[WARN] Core 3 has $procs_on_3 processes — should be nearly empty"
fi

# Check CPU governor
gov=$(cat /sys/devices/system/cpu/cpu3/cpufreq/scaling_governor 2>/dev/null)
if [ "$gov" = "performance" ]; then
    echo "[OK] CPU3 governor: performance"
else
    echo "[WARN] CPU3 governor: $gov (should be 'performance')"
fi

# Check EtherCAT master
if lsmod | grep -q ec_master; then
    echo "[OK] EtherCAT master module loaded"
else
    echo "[FAIL] EtherCAT master module NOT loaded"
fi

# Check EtherCAT slaves
slave_count=$(ethercat slaves 2>/dev/null | wc -l)
echo "[INFO] EtherCAT slaves detected: $slave_count"

# Quick latency test (10 second run)
echo ""
echo "Running 10-second latency test on core 3..."
sudo cyclictest --smp --priority=95 --affinity=3 --interval=1000 \
    --duration=10 --mlockall --quiet 2>/dev/null
echo "  (Run longer test with: cyclictest -a3 -p95 -i1000 -D 300 -m -q)"

echo ""
echo "=== Verification Complete ==="
```

---

### Component 2: Shared Memory Interface

The bridge between klippy (Python, non-RT) and ethercat_rt (C, RT). Must be lock-free.

```
Files:
  src/shared_mem.h        — shared memory structure definitions
  src/shared_mem.c        — shared memory creation and access
  klippy/extras/ethercat_shm.py  — Python wrapper using mmap + ctypes
```

#### Shared Memory Layout

```c
#define MAX_TRAJECTORY_SEGMENTS 256
#define MAX_AXES 6

// Single trajectory segment from klippy
struct trajectory_segment {
    uint64_t timestamp_ns;       // When this segment starts
    double duration;             // Segment duration in seconds
    double start_position[MAX_AXES];  // Starting position (mm)
    double start_velocity[MAX_AXES];  // Starting velocity (mm/s)
    double accel[MAX_AXES];      // Acceleration (mm/s^2)
    uint32_t flags;              // SEGMENT_ACTIVE, SEGMENT_LAST, etc.
};

// Ring buffer: klippy writes, ethercat_rt reads
struct trajectory_ring_buffer {
    _Atomic uint32_t write_idx;  // klippy increments after writing
    _Atomic uint32_t read_idx;   // ethercat_rt increments after reading
    struct trajectory_segment segments[MAX_TRAJECTORY_SEGMENTS];
};

// Per-axis servo state: ethercat_rt writes, klippy reads
struct servo_state {
    double actual_position;       // Encoder position (mm)
    double actual_velocity;       // Encoder velocity (mm/s)
    double commanded_position;    // Last commanded position (mm)
    double following_error;       // commanded - actual (mm)
    double torque_percent;        // Current torque output (%)
    uint16_t cia402_status;       // Drive status word
    uint16_t error_code;          // Drive error code
    uint8_t  state;               // NOT_READY, SWITCH_ON_DISABLED, READY, OPERATION_ENABLED, FAULT
    uint8_t  homing_complete;     // 1 if absolute encoder home is set
};

// Master status: ethercat_rt writes, klippy reads
struct ethercat_status {
    uint32_t cycle_count;         // Total EtherCAT cycles executed
    uint32_t cycle_overruns;      // Cycles that exceeded deadline
    double   max_cycle_time_us;   // Worst case cycle time
    double   avg_cycle_time_us;   // Average cycle time
    uint8_t  bus_state;           // INIT, PREOP, SAFEOP, OP
    uint8_t  num_slaves;          // Number of slaves detected
};

// PID tuning parameters: klippy writes, ethercat_rt reads
struct pid_params {
    double kp;                    // Proportional gain
    double ki;                    // Integral gain
    double kd;                    // Derivative gain
    double ff_velocity;           // Velocity feedforward gain (typically ~1.0)
    double ff_acceleration;       // Acceleration feedforward gain
    double max_following_error;   // Fault threshold (mm)
    double max_velocity;          // Velocity limit (mm/s)
    double max_accel;             // Acceleration limit (mm/s^2)
    double position_scale;        // Encoder counts per mm
};

// Top-level shared memory structure
struct klipper_ethercat_shm {
    uint32_t magic;               // 0x4B455443 "KETC"
    uint32_t version;             // Protocol version
    struct trajectory_ring_buffer trajectory;
    struct servo_state axes[MAX_AXES];
    struct ethercat_status status;
    struct pid_params pid[MAX_AXES];
    _Atomic uint8_t command;      // NONE, ENABLE, DISABLE, HOME, ESTOP, AUTOTUNE, RECORD, STOP_RECORD
    _Atomic uint8_t command_axis; // Which axis for command
    _Atomic uint8_t command_ack;  // ethercat_rt acknowledges command

    // Auto-tune configuration (klippy writes before AUTOTUNE command)
    struct {
        double travel_mm;            // Safe travel distance from current position
        uint8_t aggression;          // 0=conservative, 1=moderate, 2=aggressive
        uint8_t validate_only;       // 1=skip characterization, just validate current gains
    } autotune_config[MAX_AXES];

    // Auto-tune status (ethercat_rt writes, klippy polls)
    struct {
        uint8_t  phase;              // IDLE, SAFETY_CHECK, RELAY_FEEDBACK, INERTIA, FRICTION, COMPUTE, VALIDATE, COMPLETE, FAILED
        uint8_t  progress_percent;
        char     message[128];
        // Results (valid when phase == COMPLETE)
        double   Tu;                 // Ultimate period
        double   Au;                 // Oscillation amplitude
        double   J;                  // Load inertia kg·m²
        double   Fs;                 // Static friction N
        double   Fv;                 // Viscous friction N·s/mm
        double   kp, ki, kd;        // Computed PID gains
        double   ff_velocity;        // Computed feedforward gain
        double   rise_time_ms;       // Validation: rise time
        double   overshoot_percent;  // Validation: overshoot
        double   settling_time_ms;   // Validation: settling time
        double   ss_error_mm;        // Validation: steady-state error
        double   tracking_rms_mm;    // Validation: tracking error RMS
    } autotune_status[MAX_AXES];

    // Following error monitoring (ethercat_rt writes continuously)
    struct {
        double current_rms;          // Current rolling RMS following error
        double baseline_rms;         // RMS recorded after last auto-tune
        double peak_error;           // Maximum error since last reset
        uint8_t degradation_warning; // 1 if current_rms > 1.5x baseline for >10s
    } following_error_stats[MAX_AXES];

    // Position logging for SERVO_PLOT (ethercat_rt writes when RECORD active)
    struct {
        _Atomic uint32_t recording;  // 1 = actively recording
        uint32_t log_count;
        struct {
            double time_ms;
            double commanded;
            double actual;
            double error;
            double torque;
        } log[16384];                // ~16 seconds at 1kHz
    } position_log[MAX_AXES];
};
```

Key requirements:
- Use POSIX shared memory (shm_open) so both processes access same physical memory
- Lock-free ring buffer using atomic read/write indices
- klippy side uses Python ctypes/mmap to read/write the structures
- No mutexes, no syscalls in the RT path

---

### Component 3: EtherCAT Real-Time Process

The hard real-time C process that runs on the isolated CPU core.

```
Files:
  src/ethercat_rt.c       — main RT loop
  src/cia402.h / .c       — CiA 402 drive state machine
  src/trajectory_interp.h / .c — trajectory interpolation
  src/pid_controller.h / .c    — PID with feedforward
  src/autotune.h / .c     — auto-tuning engine
  src/lc10e_pdo.h         — LC10E specific PDO mapping
  CMakeLists.txt          — build system
```

#### 3.1 Main RT Loop (ethercat_rt.c)

**CRITICAL: The ethercat_rt process MUST run on an isolated CPU core with real-time scheduling. This is not optional. Without proper isolation, Linux kernel housekeeping tasks will cause multi-millisecond latency spikes that make servo control unstable.**

##### RT Core Isolation — Actual Implementation

The following C code is not pseudocode — implement this exactly in the main() function before entering the RT loop:

```c
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

#define RT_CPU_CORE      3      // Isolated core (must match isolcpus= kernel param)
#define RT_PRIORITY      95     // SCHED_FIFO priority (higher than all other RT tasks)
#define CYCLE_TIME_NS    1000000  // 1ms = 1,000,000 ns

// Call this BEFORE entering the real-time loop
static int setup_realtime(void) {
    int ret;

    // 1. Pin this process to the isolated CPU core
    //    This core must be isolated via kernel parameter: isolcpus=3
    //    Without isolcpus, the kernel scheduler will still place tasks on this core
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(RT_CPU_CORE, &cpuset);
    ret = sched_setaffinity(0, sizeof(cpuset), &cpuset);
    if (ret != 0) {
        fprintf(stderr, "FATAL: Failed to pin to CPU %d: %s\n",
                RT_CPU_CORE, strerror(errno));
        fprintf(stderr, "Verify that isolcpus=%d is set in /boot/firmware/cmdline.txt\n",
                RT_CPU_CORE);
        return -1;
    }
    printf("Pinned to CPU core %d\n", RT_CPU_CORE);

    // 2. Set real-time scheduling policy SCHED_FIFO
    //    This ensures our process always preempts non-RT tasks
    //    Priority 95 is above default RT tasks (typically 50) but below
    //    the kernel watchdog (priority 99)
    struct sched_param param;
    param.sched_priority = RT_PRIORITY;
    ret = sched_setscheduler(0, SCHED_FIFO, &param);
    if (ret != 0) {
        fprintf(stderr, "FATAL: Failed to set SCHED_FIFO priority %d: %s\n",
                RT_PRIORITY, strerror(errno));
        fprintf(stderr, "Run as root or set rtprio in /etc/security/limits.conf\n");
        return -1;
    }
    printf("Set SCHED_FIFO priority %d\n", RT_PRIORITY);

    // 3. Lock all memory — prevent page faults in the RT loop
    //    Page faults cause unbounded latency (disk I/O to load pages)
    //    MCL_CURRENT locks all currently mapped pages
    //    MCL_FUTURE locks all pages mapped in the future (malloc, mmap)
    ret = mlockall(MCL_CURRENT | MCL_FUTURE);
    if (ret != 0) {
        fprintf(stderr, "WARNING: mlockall failed: %s (continuing anyway)\n",
                strerror(errno));
        // Non-fatal but will cause occasional latency spikes
    } else {
        printf("All memory locked (mlockall)\n");
    }

    // 4. Pre-fault the stack to avoid page faults during RT execution
    //    Touch every page of a large stack allocation so they're in RAM
    {
        volatile char stack_prefault[8192 * 64]; // 512KB
        memset((void *)stack_prefault, 0, sizeof(stack_prefault));
    }

    // 5. Disable core frequency scaling on our core for consistent timing
    //    This should also be done at system level via configure_rt.sh
    //    but we verify it here
    char path[128];
    snprintf(path, sizeof(path),
             "/sys/devices/system/cpu/cpu%d/cpufreq/scaling_governor",
             RT_CPU_CORE);
    FILE *f = fopen(path, "w");
    if (f) {
        fprintf(f, "performance");
        fclose(f);
        printf("CPU%d governor set to performance\n", RT_CPU_CORE);
    }

    printf("Real-time setup complete on CPU %d\n", RT_CPU_CORE);
    return 0;
}

// Precise nanosecond timer for the RT loop
// Uses CLOCK_MONOTONIC for drift-free timing
static void rt_loop(struct klipper_ethercat_shm *shm,
                     ec_master_t *master,
                     ec_domain_t *domain) {
    struct timespec next_cycle;
    clock_gettime(CLOCK_MONOTONIC, &next_cycle);

    while (running) {
        // Advance to next cycle time
        next_cycle.tv_nsec += CYCLE_TIME_NS;
        if (next_cycle.tv_nsec >= 1000000000) {
            next_cycle.tv_nsec -= 1000000000;
            next_cycle.tv_sec++;
        }

        // Sleep until next cycle — this is where the core is idle
        // clock_nanosleep is the most precise sleep on PREEMPT_RT
        int ret = clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME,
                                   &next_cycle, NULL);
        if (ret != 0 && ret != EINTR) {
            fprintf(stderr, "clock_nanosleep error: %d\n", ret);
            continue;
        }

        // --- START OF TIME-CRITICAL SECTION ---
        struct timespec cycle_start;
        clock_gettime(CLOCK_MONOTONIC, &cycle_start);

        // 1. Receive EtherCAT frame
        ecrt_master_receive(master);
        ecrt_domain_process(domain);

        // 2. Read servo feedback for each axis
        for (int axis = 0; axis < num_axes; axis++) {
            read_actual_position(axis);
            read_actual_velocity(axis);
            read_status_word(axis);
            update_servo_state(shm, axis);
        }

        // 3. Handle CiA 402 state machine
        for (int axis = 0; axis < num_axes; axis++) {
            cia402_update(axis);
        }

        // 4. Check for commands from klippy
        handle_commands(shm);

        // 5. Compute motion (if drives are enabled)
        if (all_axes_enabled()) {
            update_trajectory_interpolation(shm);

            for (int axis = 0; axis < num_axes; axis++) {
                double target_pos = interpolate_position(axis, cycle_start);
                double target_vel = interpolate_velocity(axis, cycle_start);
                double actual_pos = shm->axes[axis].actual_position;
                double error = target_pos - actual_pos;

                double vel_cmd = pid_compute(&pid_state[axis],
                                              &shm->pid[axis], error, 0.001);
                vel_cmd += shm->pid[axis].ff_velocity * target_vel;

                vel_cmd = clamp(vel_cmd,
                               -shm->pid[axis].max_velocity,
                                shm->pid[axis].max_velocity);
                write_target_velocity(axis, vel_cmd);
            }
        }

        // 6. Handle autotune if active
        for (int axis = 0; axis < num_axes; axis++) {
            if (autotune_state[axis].phase != AUTOTUNE_IDLE) {
                double vel_cmd;
                autotune_update(&autotune_state[axis],
                               &shm->axes[axis], &vel_cmd);
                write_target_velocity(axis, vel_cmd);
            }
        }

        // 7. Update following error monitoring
        for (int axis = 0; axis < num_axes; axis++) {
            update_following_error_stats(&fe_stats[axis],
                                         shm->axes[axis].following_error);
            shm->following_error_stats[axis] = fe_stats[axis];
        }

        // 8. Send EtherCAT frame
        ecrt_domain_queue(domain);
        ecrt_master_send(master);

        // --- END OF TIME-CRITICAL SECTION ---

        // 9. Update timing stats (not time-critical)
        struct timespec cycle_end;
        clock_gettime(CLOCK_MONOTONIC, &cycle_end);
        double cycle_us = (cycle_end.tv_sec - cycle_start.tv_sec) * 1e6
                        + (cycle_end.tv_nsec - cycle_start.tv_nsec) / 1e3;
        update_timing_stats(shm, cycle_us);

        // 10. Check for overrun
        if (cycle_us > CYCLE_TIME_NS / 1000.0) {
            shm->status.cycle_overruns++;
            // Log but don't stop — the next cycle will correct
        }
    }
}

int main(int argc, char *argv[]) {
    // Parse config
    parse_config(argc, argv);

    // Open shared memory (created by klippy, we attach to it)
    struct klipper_ethercat_shm *shm = open_shared_memory();
    if (!shm) return 1;

    // Initialize EtherLab master BEFORE going real-time
    // (these calls involve kernel I/O and memory allocation)
    ec_master_t *master = init_etherlab_master();
    if (!master) return 1;

    scan_slaves(master);
    ec_domain_t *domain = configure_pdo_mapping(master);
    if (!domain) return 1;

    // NOW go real-time — after all allocation and I/O is done
    if (setup_realtime() != 0) {
        fprintf(stderr, "FATAL: Real-time setup failed. Cannot run servo control.\n");
        return 1;
    }

    // Enter the real-time loop (never returns until shutdown)
    rt_loop(shm, master, domain);

    // Cleanup (after shutdown signal)
    cleanup_etherlab(master);
    close_shared_memory(shm);
    return 0;
}
```

**Key ordering requirements:**
1. All memory allocation, file I/O, and EtherCAT initialization happens BEFORE `setup_realtime()`
2. `setup_realtime()` pins to the isolated core, sets SCHED_FIFO, and locks memory
3. The RT loop uses ONLY pre-allocated memory — no malloc, no printf, no file I/O
4. The RT loop uses `clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, ...)` for precise cycle timing
5. Shared memory access uses atomic operations only — no mutexes, no syscalls

#### 3.2 CiA 402 State Machine (cia402.c)

Implement the DS402 state machine for servo drive control:

```
States:
  NOT_READY_TO_SWITCH_ON → (automatic)
  SWITCH_ON_DISABLED → ready_to_switch_on (controlword 0x0006)
  READY_TO_SWITCH_ON → switched_on (controlword 0x0007)
  SWITCHED_ON → operation_enabled (controlword 0x000F)
  OPERATION_ENABLED → (running, accepts velocity commands)
  FAULT → fault_reset (controlword 0x0080) → SWITCH_ON_DISABLED

PDO objects used:
  Controlword:    0x6040 (RxPDO, uint16)
  Statusword:     0x6041 (TxPDO, uint16)
  Modes of operation:      0x6060 (RxPDO, int8) — set to 9 for CSV mode
  Modes of operation display: 0x6061 (TxPDO, int8)
  Target velocity:         0x60FF (RxPDO, int32) — velocity command in encoder counts/s
  Position actual value:   0x6064 (TxPDO, int32) — encoder position in counts
  Velocity actual value:   0x606C (TxPDO, int32) — encoder velocity in counts/s
  Torque actual value:     0x6077 (TxPDO, int16) — torque in 0.1% rated
```

Important: The LC10E has quirks with its PDO mapping. The default PDO indices (0x1601, 0x1B01) may not work. Based on LinuxCNC community experience, use:
- RxPDO: index 0x1702
- TxPDO: index 0x1B02

Include the ESI/XML file for the LC10E (available from the LinuxCNC forum thread). Load it during slave configuration.

#### 3.3 Trajectory Interpolator (trajectory_interp.c)

Reads segments from the shared memory ring buffer and evaluates position/velocity at the current time.

```c
// Each segment describes a constant-acceleration move:
// position(t) = start_position + start_velocity * t + 0.5 * accel * t^2
// velocity(t) = start_velocity + accel * t
// where t is time elapsed since segment start

double interpolate_position(int axis, uint64_t now_ns) {
    struct trajectory_segment *seg = current_segment[axis];
    double t = (now_ns - seg->timestamp_ns) / 1e9;

    if (t > seg->duration) {
        // Advance to next segment
        advance_segment(axis);
        seg = current_segment[axis];
        t = (now_ns - seg->timestamp_ns) / 1e9;
    }

    return seg->start_position[axis]
         + seg->start_velocity[axis] * t
         + 0.5 * seg->accel[axis] * t * t;
}
```

Must handle:
- Ring buffer empty (no new segments from klippy) — hold last position
- Segment transitions — smooth, no discontinuities
- klippy paused/stopped — decelerate to stop safely

#### 3.4 PID Controller (pid_controller.c)

Standard PID with anti-windup, derivative filtering, and feedforward:

```c
struct pid_state {
    double integral;
    double prev_error;
    double prev_derivative;  // For low-pass filter on D term
    double output;
};

double pid_compute(struct pid_state *state, struct pid_params *params,
                   double error, double dt) {
    // Proportional
    double p_term = params->kp * error;

    // Integral with anti-windup (clamp)
    state->integral += error * dt;
    state->integral = clamp(state->integral, -windup_limit, windup_limit);
    double i_term = params->ki * state->integral;

    // Derivative with low-pass filter (prevent noise amplification)
    double raw_derivative = (error - state->prev_error) / dt;
    double alpha = 0.1;  // Filter coefficient
    state->prev_derivative = alpha * raw_derivative
                           + (1.0 - alpha) * state->prev_derivative;
    double d_term = params->kd * state->prev_derivative;

    state->prev_error = error;
    state->output = p_term + i_term + d_term;
    return state->output;
}
```

#### 3.5 Auto-Tune Engine (autotune.c)

Automated PID tuning that physically moves the axis to characterize the system. The motors MUST move during auto-tune — there is no way to characterize a mechanical system without observing its physical response. This is how every industrial servo auto-tune works.

The full auto-tune sequence takes approximately 60-90 seconds per axis and requires about 50-100mm of clear travel in each direction from the starting position.

```
Files:
  src/autotune.h
  src/autotune.c
  src/autotune_relay.c        — relay feedback oscillation test
  src/autotune_inertia.c      — inertia measurement
  src/autotune_friction.c     — friction characterization
  src/autotune_validate.c     — step response validation
```

##### Phase 1: Safety Checks (before any motion)

Before any axis moves, the auto-tune engine must verify:
- Axis has clear travel range (read from config or use absolute encoder to check distance to limits)
- All other axes are stationary
- No active print job
- E-stop is not triggered
- Drive is in OPERATION_ENABLED state

The safe travel range is configured in printer.cfg:
```ini
[ethercat_stepper stepper_x]
autotune_travel: 40    # mm of safe travel in each direction from current position
autotune_velocity: 50  # mm/s max velocity during auto-tune (conservative)
```

##### Phase 2: Relay Feedback Test (Åström-Hägglund Method) — ~30 seconds

This is the primary characterization test. It determines the system's critical frequency and gain.

```c
// The relay feedback method:
//
// 1. Set a target position (current position)
// 2. Apply a fixed velocity command: +relay_amplitude
// 3. When actual position crosses above target: switch to -relay_amplitude
// 4. When actual position crosses below target: switch to +relay_amplitude
// 5. The axis oscillates back and forth around the target
// 6. Repeat for 10-20 full oscillation cycles
//
// The oscillation is self-sustaining. The system finds its own natural
// oscillation frequency — the frequency at which the phase lag equals 180°.
//
// Measurements taken:
//   Tu = average period of oscillation (seconds)
//   Au = average amplitude of oscillation (mm, peak-to-peak / 2)
//
// These give us:
//   Ku (ultimate gain) = 4 * relay_amplitude / (pi * Au)
//
// PID gains computed via Ziegler-Nichols relay tuning rules:
//   Kp = 0.6 * Ku
//   Ki = 2.0 * Kp / Tu
//   Kd = Kp * Tu / 8.0

struct relay_feedback_result {
    double Tu;              // Ultimate period (seconds)
    double Au;              // Oscillation amplitude (mm)
    double Ku;              // Ultimate gain
    int    num_cycles;      // Number of complete cycles measured
    int    valid;           // 1 if measurement is reliable
};

struct relay_feedback_config {
    double relay_amplitude;     // Velocity command magnitude (mm/s), start with 20-50
    double target_position;     // Oscillation center (mm), use current position
    int    min_cycles;          // Minimum oscillation cycles to measure (10)
    int    max_cycles;          // Maximum before giving up (30)
    double timeout_seconds;     // Safety timeout (30.0)
    double max_amplitude_mm;    // Abort if oscillation exceeds this (5.0)
};

// The relay amplitude should be chosen to produce small but measurable
// oscillation. Too small: noise dominates, bad measurement. Too large:
// unnecessary wear and risk. Start with ~5% of max_velocity.
//
// Typical motion during this phase: ±1-5mm oscillation for 10-20 cycles.
// Total duration: ~30 seconds.
```

##### Phase 3: Inertia Measurement — ~2 seconds

Apply a known torque and measure the resulting acceleration to compute load inertia.

```c
// Inertia measurement procedure:
//
// 1. Command a velocity ramp: 0 to test_velocity over ramp_time
// 2. During the ramp, read:
//    - Actual velocity from encoder (PDO 0x606C)
//    - Actual torque from drive (PDO 0x6077, in 0.1% of rated torque)
// 3. Compute acceleration: a = dv/dt from encoder data
// 4. Compute torque in physical units from drive rated torque
// 5. Inertia: J = torque / angular_acceleration
// 6. Decelerate back to zero
//
// The velocity feedforward gain is derived from inertia:
//   ff_velocity ≈ J * (encoder_resolution / rotation_distance)
//   In practice, start ff_velocity at 1.0 and scale from inertia ratio
//
// Typical motion: single acceleration/deceleration, 20-50mm of travel.
// Duration: ~2 seconds.

struct inertia_result {
    double J_kg_m2;         // Load inertia in kg·m²
    double friction_torque;  // Average friction torque during move
    double ff_velocity;      // Computed velocity feedforward gain
    int    valid;
};

struct inertia_config {
    double test_velocity;   // Target velocity for ramp (mm/s), ~20% of max
    double ramp_time;       // Acceleration time (seconds), 0.2-0.5
    int    num_samples;     // Encoder samples to average over
};
```

##### Phase 4: Friction Characterization — ~5 seconds

Measure static and viscous friction by moving very slowly in both directions.

```c
// Friction measurement procedure:
//
// 1. Command very slow velocity in positive direction (+2-5 mm/s)
// 2. Record steady-state torque at multiple low speeds
// 3. Repeat in negative direction
// 4. Fit a friction model: T_friction = Fs * sign(v) + Fv * v
//    where Fs = static/Coulomb friction, Fv = viscous friction coefficient
//
// This data improves:
// - PID integral term sizing (must overcome static friction)
// - Low-speed tracking accuracy
// - Dead-band compensation at velocity zero crossings
//
// Typical motion: slow crawl back and forth, ~10mm each direction
// Duration: ~5 seconds

struct friction_result {
    double Fs_positive;     // Static friction, positive direction (N or N·m)
    double Fs_negative;     // Static friction, negative direction
    double Fv;              // Viscous friction coefficient (N·s/mm)
    double deadband_mm_s;   // Velocity below which friction dominates
    int    valid;
};
```

##### Phase 5: PID Gain Computation

Compute gains from relay feedback, inertia, and friction measurements.

```c
// Primary gains from relay feedback (Ziegler-Nichols relay method):
//   Kp = 0.6 * Ku
//   Ki = 2.0 * Kp / Tu
//   Kd = Kp * Tu / 8.0
//
// Adjustments from inertia measurement:
//   ff_velocity = computed from inertia (typically ~1.0)
//   ff_acceleration = computed from inertia (typically small, ~0.01)
//
// Adjustments from friction measurement:
//   Ki minimum = sufficient to overcome static friction
//   Integral windup limit = based on Fs / Ki
//
// Additional gain scaling based on desired response:
//   "conservative" preset: multiply Kp by 0.5 (less aggressive, more stable)
//   "moderate" preset: use computed values directly
//   "aggressive" preset: multiply Kp by 1.2 (tighter tracking, risk of oscillation)

struct autotune_gains {
    double kp;
    double ki;
    double kd;
    double ff_velocity;
    double ff_acceleration;
    double integral_windup_limit;
    double max_following_error;   // Set to ~3x the relay oscillation amplitude
};

void compute_pid_gains(struct relay_feedback_result *relay,
                       struct inertia_result *inertia,
                       struct friction_result *friction,
                       const char *aggressiveness,   // "conservative", "moderate", "aggressive"
                       struct autotune_gains *output);
```

##### Phase 6: Validation Step Response — ~10 seconds

Apply the computed gains and test with actual moves to verify performance.

```c
// Validation procedure:
//
// 1. Apply computed PID gains to the position loop
// 2. Command a step move: current_position + step_size (e.g. 5mm)
// 3. Record the position response at 1kHz (every EtherCAT cycle)
// 4. Analyze the response:
//    - Rise time: time to reach 90% of target (want < 50ms)
//    - Overshoot: max position beyond target (want < 5%)
//    - Settling time: time to stay within ±0.1mm of target (want < 100ms)
//    - Steady-state error: final position error (want < 0.01mm)
// 5. If overshoot > 10%: reduce Kp by 20%, increase Kd by 20%, retest
// 6. If settling time > 200ms: increase Kp by 10%, retest
// 7. Repeat up to 3 refinement iterations
// 8. Command a return move to starting position
// 9. Run a short trajectory (small triangle or sine wave) to verify
//    tracking during continuous motion, not just step response
//
// Total motion: a few 5mm moves back and forth
// Duration: ~10 seconds including analysis

struct validation_result {
    double rise_time_ms;
    double overshoot_percent;
    double settling_time_ms;
    double steady_state_error_mm;
    double tracking_error_rms_mm;    // During continuous motion test
    int    iterations;               // How many refinement loops
    int    passed;                   // 1 if all criteria met
};

#define VALIDATION_MAX_OVERSHOOT       5.0   // percent
#define VALIDATION_MAX_SETTLING_TIME   100.0  // ms
#define VALIDATION_MAX_SS_ERROR        0.01   // mm
#define VALIDATION_MAX_ITERATIONS      3
```

##### Complete Auto-Tune Sequence (orchestrator)

```c
struct autotune_state {
    enum {
        AUTOTUNE_IDLE,
        AUTOTUNE_SAFETY_CHECK,
        AUTOTUNE_RELAY_FEEDBACK,
        AUTOTUNE_INERTIA,
        AUTOTUNE_FRICTION,
        AUTOTUNE_COMPUTE_GAINS,
        AUTOTUNE_VALIDATE,
        AUTOTUNE_COMPLETE,
        AUTOTUNE_FAILED
    } phase;

    int axis;
    struct relay_feedback_config relay_config;
    struct relay_feedback_result relay_result;
    struct inertia_config inertia_config;
    struct inertia_result inertia_result;
    struct friction_result friction_result;
    struct autotune_gains computed_gains;
    struct validation_result validation_result;

    // Data recording buffer for analysis
    double position_log[16384];   // ~16 seconds at 1kHz
    double velocity_log[16384];
    double torque_log[16384];
    uint32_t log_index;

    // Progress reporting (klippy reads from shared memory)
    uint8_t progress_percent;
    char    status_message[128];
};

// Called every EtherCAT cycle (1ms) when autotune is active
void autotune_update(struct autotune_state *state,
                     struct servo_state *servo,
                     double *velocity_command_out);
```

##### User-Facing Output (klippy side)

When auto-tune completes, klippy reads the results from shared memory and presents:

```
> SERVO_AUTOTUNE AXIS=X
Auto-tune starting for axis X
  Safe travel: ±20mm from current position
  Phase 1/5: Relay feedback test...
    [axis oscillates gently ±2mm for ~30 seconds]
  Phase 1 complete: Tu=0.045s, Au=0.82mm, Ku=31.2
  Phase 2/5: Inertia measurement...
    [axis accelerates briefly, ~30mm travel]
  Phase 2 complete: J=0.00034 kg·m²
  Phase 3/5: Friction measurement...
    [axis creeps slowly ±10mm]
  Phase 3 complete: Fs=0.12N, Fv=0.003 N·s/mm
  Phase 4/5: Computing gains...
  Phase 4 complete:
    Kp = 42.6
    Ki = 4.8
    Kd = 0.38
    FF_velocity = 0.97
  Phase 5/5: Validation step response...
    [axis makes small 5mm test moves]
    Iteration 1: overshoot=2.1%, settling=34ms — PASS
  Validation complete:
    Rise time: 18ms
    Overshoot: 2.1%
    Settling time: 34ms
    Steady-state error: 0.003mm
    Tracking error RMS: 0.008mm

Auto-tune PASSED for axis X. Use SAVE_CONFIG to save gains.
```

##### Adaptive Monitoring (runtime, after tuning)

During normal operation, continuously monitor following error to detect when re-tuning is needed.

```c
// In the main RT loop, track following error statistics:
struct following_error_stats {
    double rms_window[1000];    // Rolling 1-second RMS window
    double current_rms;
    double baseline_rms;        // Set after auto-tune completes
    double peak_error;          // Maximum error since last reset
    uint32_t sample_count;
    uint8_t  degradation_warning;  // Set to 1 if RMS > 1.5x baseline
};

// Every cycle:
void update_following_error_stats(struct following_error_stats *stats,
                                  double following_error) {
    // Update rolling RMS
    // Compare to baseline
    // If current_rms > baseline_rms * 1.5 for more than 10 seconds:
    //   Set degradation_warning = 1
    //   klippy reads this and shows warning to user:
    //   "WARNING: Axis X following error increased 50% since last tune.
    //    Consider running SERVO_AUTOTUNE AXIS=X"
}

// klippy can also trigger gentle background characterization during
// non-printing moves (homing, travel moves, park moves) to detect
// drift in system dynamics without requiring a dedicated calibration run.
// This is optional and can be implemented in a later phase.
```

Store all tuning results so klippy can write them to printer.cfg via SAVE_CONFIG.

---

### Component 4: Klipper Module (Python)

The Klipper extras module that integrates EtherCAT into klippy's motion system.

```
Files:
  klippy/extras/ethercat_servo.py    — main module, printer.cfg interface
  klippy/extras/ethercat_stepper.py  — replacement for stepper.py for EtherCAT axes
  klippy/extras/ethercat_homing.py   — homing via CiA 402 / absolute encoder
  klippy/extras/ethercat_autotune.py — auto-tune UI and results handler
  klippy/extras/ethercat_shm.py      — shared memory Python interface
```

#### 4.1 Main Module (ethercat_servo.py)

```python
# printer.cfg interface:
#
# [ethercat]
# cycle_time: 0.001           # 1ms EtherCAT cycle (default)
# rt_process_path: /usr/local/bin/ethercat_rt
#
# [ethercat_stepper stepper_x]
# slave_position: 0           # Position on EtherCAT bus
# drive_type: lichuan_lc10e   # Drive profile name
# encoder_resolution: 8388608 # 23-bit = 2^23 counts/rev
# rotation_distance: 40       # mm per motor revolution
# max_velocity: 500           # mm/s
# max_accel: 30000            # mm/s^2
# pid_kp: 0                   # 0 = use auto-tune
# pid_ki: 0
# pid_kd: 0
# ff_velocity: 1.0            # Velocity feedforward gain
# max_following_error: 2.0    # mm, fault threshold
#
# [ethercat_stepper stepper_y]
# slave_position: 1
# ... (same structure)
#
# [ethercat_stepper stepper_z]
# slave_position: 2
# ...

class EtherCATServo:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.cycle_time = config.getfloat('cycle_time', 0.001)

        # Launch the RT process
        self.rt_process = None
        self.shm = EtherCATSharedMemory()

        # Register event handlers
        self.printer.register_event_handler("klippy:connect", self._handle_connect)
        self.printer.register_event_handler("klippy:shutdown", self._handle_shutdown)

        # Register GCode commands
        gcode = self.printer.lookup_object('gcode')
        gcode.register_command('ETHERCAT_STATUS', self.cmd_ETHERCAT_STATUS)
        gcode.register_command('SERVO_ENABLE', self.cmd_SERVO_ENABLE)
        gcode.register_command('SERVO_DISABLE', self.cmd_SERVO_DISABLE)
        gcode.register_command('SERVO_AUTOTUNE', self.cmd_SERVO_AUTOTUNE)
        gcode.register_command('SERVO_POSITION', self.cmd_SERVO_POSITION)

    def _handle_connect(self):
        """Launch the RT process and verify it's running on the isolated core."""
        import subprocess
        import os
        import time

        # 1. Create shared memory (klippy creates, ethercat_rt attaches)
        self.shm = EtherCATSharedMemory()
        self.shm.create()

        # 2. Verify RT prerequisites before launching
        #    Check that isolcpus=3 is in the kernel command line
        with open('/proc/cmdline', 'r') as f:
            cmdline = f.read()
        if 'isolcpus=3' not in cmdline:
            raise self.printer.config_error(
                "FATAL: isolcpus=3 not found in kernel command line. "
                "EtherCAT servo control requires an isolated CPU core. "
                "Add 'isolcpus=3 nohz_full=3 rcu_nocbs=3' to "
                "/boot/firmware/cmdline.txt and reboot."
            )

        #    Check that PREEMPT_RT kernel is running
        uname = subprocess.check_output(['uname', '-v']).decode()
        if 'PREEMPT_RT' not in uname:
            raise self.printer.config_error(
                "FATAL: Kernel is not PREEMPT_RT. "
                "EtherCAT servo control requires a PREEMPT_RT patched kernel. "
                "Run scripts/setup_rt_kernel.sh to build and install one."
            )

        #    Check that IgH EtherCAT master module is loaded
        lsmod = subprocess.check_output(['lsmod']).decode()
        if 'ec_master' not in lsmod:
            raise self.printer.config_error(
                "EtherCAT master kernel module not loaded. "
                "Run: sudo systemctl start ethercat"
            )

        # 3. Launch the RT process
        rt_path = self.config.get('rt_process_path',
                                   '/usr/local/bin/ethercat_rt')
        config_path = self.config.get('rt_config_path',
                                       '/etc/klipper-ethercat/config.ini')
        self.rt_process = subprocess.Popen(
            ['sudo', rt_path, '--config', config_path],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )

        # 4. Wait for RT process to initialize and reach OP state
        #    Poll shared memory for bus_state == OP
        timeout = 10.0  # seconds
        start = time.monotonic()
        while time.monotonic() - start < timeout:
            try:
                status = self.shm.read_ethercat_status()
                if status['bus_state'] == 'OP':
                    break
            except Exception:
                pass
            time.sleep(0.1)
        else:
            raise self.printer.config_error(
                "EtherCAT bus did not reach OP state within 10 seconds. "
                "Check servo drive power and Ethernet connection."
            )

        # 5. Verify the RT process is actually running on the isolated core
        pid = self.rt_process.pid
        try:
            with open(f'/proc/{pid}/status', 'r') as f:
                for line in f:
                    if line.startswith('Cpus_allowed_list:'):
                        allowed_cpus = line.split(':')[1].strip()
                        if allowed_cpus != '3':
                            logging.warning(
                                f"ethercat_rt (PID {pid}) running on CPUs "
                                f"{allowed_cpus}, expected only core 3. "
                                f"RT performance may be degraded."
                            )
                        else:
                            logging.info(
                                f"ethercat_rt (PID {pid}) confirmed on "
                                f"isolated core 3"
                            )
                        break
        except Exception:
            pass

        # 6. Write PID parameters from printer.cfg to shared memory
        for stepper in self.ethercat_steppers:
            stepper.write_pid_to_shm(self.shm)

        logging.info(
            f"EtherCAT connected: {status['num_slaves']} slaves, "
            f"bus state OP, RT process on core 3"
        )

    def _handle_shutdown(self):
        """Emergency shutdown — stop all servos immediately."""
        if self.shm:
            # Send ESTOP command via shared memory
            # The RT process will:
            # 1. Immediately send zero velocity to all drives
            # 2. Transition drives to SWITCH_ON_DISABLED state
            # 3. Continue running to keep EtherCAT bus alive
            self.shm.send_command('ESTOP', 0)

        if self.rt_process:
            # Give RT process 2 seconds to stop servos gracefully
            try:
                self.rt_process.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                self.rt_process.terminate()
                try:
                    self.rt_process.wait(timeout=1.0)
                except subprocess.TimeoutExpired:
                    self.rt_process.kill()
```

#### 4.2 Motion Integration (ethercat_stepper.py)

This is the critical piece — intercepting Klipper's motion planner output.

Klipper's motion planner generates move segments with acceleration profiles. Normally these get converted to step times by itersolve. For EtherCAT axes, we bypass step generation and instead write trajectory segments to the shared memory ring buffer.

```python
class EtherCATStepper:
    """Replaces standard stepper for EtherCAT servo axes.

    Instead of generating step pulses, writes trajectory segments
    to shared memory for the RT process to interpolate.
    """

    def __init__(self, config):
        self.name = config.get_name().split()[-1]
        self.slave_position = config.getint('slave_position')
        self.encoder_resolution = config.getint('encoder_resolution')
        self.rotation_distance = config.getfloat('rotation_distance')

        # Scale factor: encoder counts per mm
        self.position_scale = self.encoder_resolution / self.rotation_distance

        # PID parameters
        self.pid_kp = config.getfloat('pid_kp', 0.0)
        self.pid_ki = config.getfloat('pid_ki', 0.0)
        self.pid_kd = config.getfloat('pid_kd', 0.0)
        self.ff_velocity = config.getfloat('ff_velocity', 1.0)

        # Register as a stepper with the motion planner
        # This is where we hook into Klipper's move system
        # Need to register a custom kinematic solver that outputs
        # trajectory segments instead of step times

    def set_position(self, newpos):
        """Called by kinematic solver to update commanded position."""
        pass

    def generate_steps(self, flush_time):
        """Normally generates step times. We generate trajectory segments instead.

        Read from Klipper's move queue, convert each move to a trajectory
        segment (start pos, start vel, accel, duration), and write to
        shared memory ring buffer.
        """
        pass

    def get_actual_position(self):
        """Read encoder position from shared memory."""
        pass
```

The exact integration point into Klipper's motion system requires careful study of:
- `klippy/stepper.py` — MCUStepper class
- `klippy/toolhead.py` — ToolHead._process_moves()
- `klippy/kinematics/` — cartesian.py, corexy.py etc.
- `klippy/chelper/itersolve.c` — the C extension that converts moves to steps

The cleanest approach is to create a new stepper type that klippy's kinematics can use. The kinematics code calls `stepper.set_position()` and the toolhead calls the flush mechanism. We intercept at the flush stage and write trajectory segments instead of step schedules.

#### 4.3 Homing (ethercat_homing.py)

For LC10E with 23-bit absolute encoders:

```python
class EtherCATHoming:
    """Homing for EtherCAT servo axes.

    With absolute encoders, the drive knows position at power-on.
    Homing just sets the coordinate system offset.

    Methods supported:
    1. absolute_encoder: Read absolute position, apply offset. No motion needed.
    2. torque_limit: Move slowly until hitting hard stop, set as home.
       Uses CiA 402 homing mode (sent via SDO to the drive).
    """

    def home(self, axis, method='absolute_encoder'):
        if method == 'absolute_encoder':
            # Read current absolute position from drive
            # Set klippy's position offset so current = home position
            pass
        elif method == 'torque_limit':
            # Send CiA 402 homing command to drive
            # Drive handles the motion internally
            # Wait for homing complete bit in statusword
            # Read final position, set offset
            pass
```

#### 4.4 Auto-Tune Interface (ethercat_autotune.py)

```python
class EtherCATAutoTune:
    """GCode interface for servo auto-tuning.

    The auto-tune physically moves each axis to characterize the system.
    Total sequence takes ~60-90 seconds per axis, requiring ~50-100mm of
    clear travel in each direction from current position.

    Usage:
        SERVO_AUTOTUNE AXIS=X                          — full auto-tune
        SERVO_AUTOTUNE AXIS=X AGGRESSION=conservative  — gentler gains
        SERVO_AUTOTUNE AXIS=X AGGRESSION=aggressive    — tighter tracking
        SERVO_AUTOTUNE AXIS=X TRAVEL=30                — limit travel to ±30mm
        SERVO_AUTOTUNE AXIS=X VALIDATE_ONLY=1          — retest current gains
    """

    def __init__(self, config):
        self.printer = config.get_printer()
        self.shm = None  # Set during connect

        gcode = self.printer.lookup_object('gcode')
        gcode.register_command('SERVO_AUTOTUNE', self.cmd_SERVO_AUTOTUNE)
        gcode.register_command('SERVO_STATUS', self.cmd_SERVO_STATUS)
        gcode.register_command('SERVO_PLOT', self.cmd_SERVO_PLOT)

    def cmd_SERVO_AUTOTUNE(self, gcmd):
        axis = gcmd.get('AXIS').upper()
        aggression = gcmd.get('AGGRESSION', 'moderate')
        travel = gcmd.getfloat('TRAVEL', 20.0)
        validate_only = gcmd.getint('VALIDATE_ONLY', 0)

        # Safety checks before sending command to RT process
        toolhead = self.printer.lookup_object('toolhead')
        if toolhead.get_status(None).get('printing', False):
            raise gcmd.error("Cannot auto-tune during a print")

        # Verify axis exists and is an EtherCAT servo
        axis_idx = self._get_axis_index(axis)
        if axis_idx is None:
            raise gcmd.error(f"Unknown EtherCAT axis: {axis}")

        # Check drive is enabled
        state = self.shm.read_servo_state(axis_idx)
        if state['state'] != 'OPERATION_ENABLED':
            raise gcmd.error(f"Axis {axis} drive not enabled. Run SERVO_ENABLE first.")

        # Send autotune command to RT process
        self.shm.write_autotune_config(axis_idx, {
            'travel_mm': travel,
            'aggression': aggression,
            'validate_only': validate_only,
        })
        self.shm.send_command('AUTOTUNE', axis_idx)

        # Poll progress from shared memory and report to user
        gcmd.respond_info(f"Auto-tune starting for axis {axis}")
        gcmd.respond_info(f"  Safe travel: ±{travel}mm from current position")

        phase_names = {
            'SAFETY_CHECK': 'Safety checks',
            'RELAY_FEEDBACK': 'Relay feedback test (axis will oscillate gently)',
            'INERTIA': 'Inertia measurement (axis will accelerate briefly)',
            'FRICTION': 'Friction measurement (axis will creep slowly)',
            'COMPUTE_GAINS': 'Computing gains',
            'VALIDATE': 'Validation step response (axis will make small test moves)',
        }

        last_phase = None
        while True:
            status = self.shm.read_autotune_status(axis_idx)

            if status['phase'] != last_phase:
                last_phase = status['phase']
                name = phase_names.get(status['phase'], status['phase'])
                gcmd.respond_info(f"  Phase: {name}...")

            if status['phase'] == 'COMPLETE':
                break
            elif status['phase'] == 'FAILED':
                raise gcmd.error(f"Auto-tune failed: {status['message']}")

            # Yield to reactor for ~100ms between polls
            self.printer.get_reactor().pause(
                self.printer.get_reactor().monotonic() + 0.1)

        # Read results
        results = self.shm.read_autotune_results(axis_idx)

        # Report to user
        gcmd.respond_info(
            f"Auto-tune complete for axis {axis}:\n"
            f"  System characterization:\n"
            f"    Oscillation period: {results['Tu']:.4f}s\n"
            f"    Oscillation amplitude: {results['Au']:.3f}mm\n"
            f"    Load inertia: {results['J']:.6f} kg·m²\n"
            f"    Static friction: {results['Fs']:.3f}N\n"
            f"    Viscous friction: {results['Fv']:.5f} N·s/mm\n"
            f"  Computed gains ({aggression}):\n"
            f"    Kp = {results['kp']:.4f}\n"
            f"    Ki = {results['ki']:.4f}\n"
            f"    Kd = {results['kd']:.4f}\n"
            f"    FF_velocity = {results['ff_velocity']:.4f}\n"
            f"  Validation results:\n"
            f"    Rise time: {results['rise_time_ms']:.1f}ms\n"
            f"    Overshoot: {results['overshoot_percent']:.1f}%\n"
            f"    Settling time: {results['settling_time_ms']:.1f}ms\n"
            f"    Steady-state error: {results['ss_error_mm']:.4f}mm\n"
            f"    Tracking error RMS: {results['tracking_rms_mm']:.4f}mm\n"
            f"  Use SAVE_CONFIG to save these values."
        )

        # Store gains in config for SAVE_CONFIG
        configfile = self.printer.lookup_object('configfile')
        section = f"ethercat_stepper stepper_{axis.lower()}"
        configfile.set(section, 'pid_kp', f"{results['kp']:.4f}")
        configfile.set(section, 'pid_ki', f"{results['ki']:.4f}")
        configfile.set(section, 'pid_kd', f"{results['kd']:.4f}")
        configfile.set(section, 'ff_velocity', f"{results['ff_velocity']:.4f}")

    def cmd_SERVO_STATUS(self, gcmd):
        """Report real-time servo status including following error monitoring."""
        axis = gcmd.get('AXIS', None)

        if axis:
            axis_idx = self._get_axis_index(axis.upper())
            state = self.shm.read_servo_state(axis_idx)
            stats = self.shm.read_following_error_stats(axis_idx)
            gcmd.respond_info(
                f"Axis {axis}:\n"
                f"  State: {state['state']}\n"
                f"  Position: {state['actual_position']:.4f}mm\n"
                f"  Following error: {state['following_error']:.4f}mm\n"
                f"  Following error RMS: {stats['current_rms']:.4f}mm\n"
                f"  Baseline RMS: {stats['baseline_rms']:.4f}mm\n"
                f"  Peak error: {stats['peak_error']:.4f}mm\n"
                f"  Torque: {state['torque_percent']:.1f}%"
            )

            # Warn if following error has degraded
            if stats['current_rms'] > stats['baseline_rms'] * 1.5:
                gcmd.respond_info(
                    f"  WARNING: Following error increased "
                    f"{stats['current_rms']/stats['baseline_rms']*100:.0f}% "
                    f"since last tune. Consider running SERVO_AUTOTUNE AXIS={axis}"
                )
        else:
            # Report all axes
            status = self.shm.read_ethercat_status()
            gcmd.respond_info(
                f"EtherCAT bus: {status['bus_state']}, "
                f"{status['num_slaves']} slaves, "
                f"cycle avg {status['avg_cycle_time_us']:.1f}μs, "
                f"max {status['max_cycle_time_us']:.1f}μs, "
                f"overruns {status['cycle_overruns']}"
            )

    def cmd_SERVO_PLOT(self, gcmd):
        """Dump position log data for external plotting.

        Writes CSV of commanded vs actual position to a file that can be
        downloaded via Moonraker and plotted in a browser or with matplotlib.
        This helps users visualize tuning quality.
        """
        axis = gcmd.get('AXIS').upper()
        axis_idx = self._get_axis_index(axis)
        duration = gcmd.getfloat('DURATION', 2.0)

        # Trigger data recording in RT process
        self.shm.send_command('RECORD', axis_idx)
        self.printer.get_reactor().pause(
            self.printer.get_reactor().monotonic() + duration)
        self.shm.send_command('STOP_RECORD', axis_idx)

        # Read recorded data and write CSV
        log = self.shm.read_position_log(axis_idx)
        path = f"/tmp/servo_plot_{axis.lower()}.csv"
        with open(path, 'w') as f:
            f.write("time_ms,commanded_mm,actual_mm,error_mm,torque_pct\n")
            for entry in log:
                f.write(f"{entry['time']:.1f},{entry['cmd']:.4f},"
                        f"{entry['actual']:.4f},{entry['error']:.4f},"
                        f"{entry['torque']:.1f}\n")
        gcmd.respond_info(f"Position log saved to {path} ({len(log)} samples)")
```

---

### Component 5: LC10E Drive Profile

Drive-specific configuration for the Lichuan LC10E.

```
Files:
  profiles/lichuan_lc10e.py    — drive profile
  profiles/lichuan_lc10e.xml   — ESI file (obtain from LinuxCNC forum)
  profiles/README.md           — how to add new drive profiles
```

#### LC10E PDO Mapping

Based on working LinuxCNC configurations from the community:

```python
LC10E_PROFILE = {
    'name': 'Lichuan LC10E',
    'vendor_id': 0x000001DD,       # Verify from ESI file
    'product_code': 0x10400200,    # Verify from ESI file

    # PDO assignment — LC10E uses non-standard indices
    'rxpdo_index': 0x1702,         # NOT the default 0x1600/0x1601
    'txpdo_index': 0x1B02,         # NOT the default 0x1A00/0x1A01

    # RxPDO mapping (master → drive)
    'rxpdo_entries': [
        (0x6040, 0x00, 16),  # Controlword
        (0x6060, 0x00, 8),   # Modes of operation
        (0x60FF, 0x00, 32),  # Target velocity (CSV mode)
    ],

    # TxPDO mapping (drive → master)
    'txpdo_entries': [
        (0x6041, 0x00, 16),  # Statusword
        (0x6061, 0x00, 8),   # Modes of operation display
        (0x6064, 0x00, 32),  # Position actual value
        (0x606C, 0x00, 32),  # Velocity actual value
        (0x6077, 0x00, 16),  # Torque actual value
    ],

    # CSV mode = 9
    'default_mode': 9,

    # Encoder: 23-bit absolute = 8388608 counts/rev
    'encoder_resolution_23bit': 8388608,
    'encoder_resolution_17bit': 131072,

    # Default PID starting point (conservative)
    'default_pid': {
        'kp': 50.0,
        'ki': 5.0,
        'kd': 0.5,
        'ff_velocity': 1.0,
        'ff_acceleration': 0.0,
    },

    # Homing
    'supports_absolute_encoder': True,
    'cia402_homing_methods': [1, 2, 17, 18, 33, 34, 35, 37],

    # Safety
    'max_following_error_default': 5.0,  # mm
    'watchdog_timeout_ms': 100,
}
```

#### Known LC10E Issues and Workarounds

1. **PDO index mismatch**: Default PDO indices don't work. Must use 0x1702/0x1B02. If you try reading non-existent SDO addresses, the drive goes silent.

2. **CSP mode following error**: The internal position loop in CSP mode has poor tuning from factory. This is why we use CSV mode — we bypass the drive's position loop entirely.

3. **ESI/SII EEPROM**: Some drives ship with incorrect SII configuration. The ESI XML file must be loaded explicitly during configuration.

4. **Initial state after power-on**: The drive may not transition cleanly from INIT to PREOP on first scan. Retry logic needed in the state machine.

5. **Velocity feed forward (0x60B1)**: If you ever need to use CSP mode as a fallback, writing to this SDO significantly reduces following error.

---

### Component 6: Configuration Examples

#### Example printer.cfg section

```ini
# === EtherCAT Configuration ===
[ethercat]
cycle_time: 0.001

[ethercat_stepper stepper_x]
slave_position: 0
drive_type: lichuan_lc10e
encoder_resolution: 8388608
rotation_distance: 40
max_velocity: 500
max_accel: 30000
max_following_error: 2.0
autotune_travel: 30       # mm safe travel for auto-tune in each direction

[ethercat_stepper stepper_y]
slave_position: 1
drive_type: lichuan_lc10e
encoder_resolution: 8388608
rotation_distance: 40
max_velocity: 500
max_accel: 30000
max_following_error: 2.0

[ethercat_stepper stepper_z]
slave_position: 2
drive_type: lichuan_lc10e
encoder_resolution: 8388608
rotation_distance: 8
max_velocity: 20
max_accel: 500
max_following_error: 1.0

# === Standard Klipper stepper for rotary axis ===
# Uses onboard TMC driver on Manta M8P
[stepper_a]
step_pin: PE6
dir_pin: PE5
enable_pin: !PC14
microsteps: 16
rotation_distance: 360      # degrees per motor rev (adjust for harmonic ratio)
gear_ratio: 50:1            # harmonic drive ratio
endstop_pin: tmc2209_stepper_a:virtual_endstop

[tmc2209 stepper_a]
uart_pin: PC13
run_current: 0.8
stealthchop_threshold: 999999

# === CAN Toolhead (unchanged from standard Klipper) ===
[mcu EBBCan]
canbus_uuid: <your_uuid_here>

[extruder]
step_pin: EBBCan:PD0
dir_pin: EBBCan:PD1
enable_pin: !EBBCan:PD2
microsteps: 16
rotation_distance: 22.6789511
nozzle_diameter: 0.400
filament_diameter: 1.750
heater_pin: EBBCan:PB13
sensor_type: EPCOS 100K B57560G104F
sensor_pin: EBBCan:PA3
control: pid
pid_kp: 26.213
pid_ki: 1.304
pid_kd: 131.721
min_temp: 0
max_temp: 300
pressure_advance: 0.035

[fan]
pin: EBBCan:PA1

[heater_fan hotend_fan]
pin: EBBCan:PA0
```

---

### Component 7: Build System

```
Files:
  CMakeLists.txt              — C build for ethercat_rt
  setup.py                    — Python module installation
  Makefile                    — top-level build orchestration
  scripts/install.sh          — full installation script
```

#### Build Dependencies

```bash
# System packages
sudo apt install build-essential cmake
sudo apt install libethercat-dev         # IgH EtherLab (after building/installing)
sudo apt install python3-dev python3-pip

# Python packages (for klippy module)
pip3 install mmap ctypes struct
```

#### Installation Script (scripts/install.sh)

```bash
#!/bin/bash
# 1. Build PREEMPT_RT kernel (if not already done)
# 2. Build and install IgH EtherLab master
# 3. Build ethercat_rt C process
# 4. Install Klipper modules (copy to klippy/extras/)
# 5. Install drive profiles
# 6. Configure systemd services
# 7. Configure CPU isolation
# 8. Print verification checklist
```

---

## Development Phases

### Phase 1: Foundation (weeks 1-2)
- [ ] PREEMPT_RT kernel on CM4
- [ ] IgH EtherLab master compiled and running
- [ ] `ethercat slaves` lists LC10E drives
- [ ] Shared memory structure defined and tested
- [ ] Basic RT process skeleton: timer, EtherCAT frame exchange

### Phase 2: Drive Communication (weeks 2-3)
- [ ] LC10E PDO mapping working
- [ ] CiA 402 state machine: INIT → PREOP → SAFEOP → OP
- [ ] Read encoder position in real-time
- [ ] Send velocity commands in CSV mode
- [ ] Manual jogging via direct velocity command

### Phase 3: Motion Integration (weeks 3-5)
- [ ] Klipper module loads and connects to RT process
- [ ] Trajectory segments flow from klippy to shared memory
- [ ] RT process interpolates and tracks commanded trajectory
- [ ] PID position loop working with manual gains
- [ ] Basic homing with absolute encoder
- [ ] First coordinated multi-axis moves

### Phase 4: Auto-Tune (weeks 5-7)
- [ ] Relay feedback auto-tune implemented
- [ ] Step response analysis
- [ ] Frequency sweep for resonance detection
- [ ] SERVO_AUTOTUNE gcode command working
- [ ] Gains save to printer.cfg

### Phase 5: Integration and Testing (weeks 7-9)
- [ ] Pressure advance working with servo axes
- [ ] Input shaping working with servo axes
- [ ] Bed mesh compensation working
- [ ] CAN toolhead fully functional alongside EtherCAT
- [ ] Following error monitoring and fault handling
- [ ] E-stop handling
- [ ] Extended print testing

### Phase 6: Polish (weeks 9-10)
- [ ] Real-time position monitoring in web UI (via moonraker)
- [ ] Servo status dashboard
- [ ] Documentation
- [ ] Automated setup script

---

## Key Technical Decisions

1. **CSV mode over CSP**: We close the position loop on the Pi, not in the drive. This gives us drive-agnostic tuning and auto-tune capability. The LC10E's internal position loop is poorly tuned from factory.

2. **Shared memory over sockets**: Zero-copy, zero-syscall data path between klippy and the RT process. Lock-free ring buffer with atomic indices.

3. **IgH EtherLab over SOEM**: IgH is a kernel module with better real-time properties. SOEM is simpler but runs in userspace with more jitter. For a dedicated RT core with PREEMPT_RT, IgH is the right choice.

4. **1ms cycle time**: Standard for servo drives, well within Pi CM4's RT capability. The LC10E supports down to 125μs but 1ms is sufficient for 3D printing and light CNC.

5. **Velocity feedforward**: The trajectory planner already knows the target velocity at every point. Feed this forward to the servo drive directly, with the PID only correcting for small errors. This gives excellent tracking with low PID gains, which means stable operation even on machines with unknown dynamics.

---

## Repository Structure

```
klipper-ethercat/
├── README.md
├── CLAUDE_CODE_INSTRUCTIONS.md   # This file
├── CMakeLists.txt
├── Makefile
├── scripts/
│   ├── setup_rt_kernel.sh
│   ├── setup_etherlab.sh
│   ├── configure_rt.sh
│   └── install.sh
├── src/
│   ├── ethercat_rt.c
│   ├── shared_mem.h
│   ├── shared_mem.c
│   ├── cia402.h
│   ├── cia402.c
│   ├── trajectory_interp.h
│   ├── trajectory_interp.c
│   ├── pid_controller.h
│   ├── pid_controller.c
│   ├── autotune.h
│   ├── autotune.c
│   └── lc10e_pdo.h
├── klippy/
│   └── extras/
│       ├── ethercat_servo.py
│       ├── ethercat_stepper.py
│       ├── ethercat_homing.py
│       ├── ethercat_autotune.py
│       └── ethercat_shm.py
├── profiles/
│   ├── lichuan_lc10e.py
│   ├── lichuan_lc10e.xml
│   └── README.md
├── config/
│   └── example_printer.cfg
├── tests/
│   ├── test_shared_mem.py
│   ├── test_cia402.c
│   ├── test_pid.c
│   └── test_trajectory.c
└── docs/
    ├── architecture.md
    ├── setup_guide.md
    ├── tuning_guide.md
    └── adding_drive_profiles.md
```

---

## Notes for Claude Code

- Start with Phase 1 foundation work. Get the RT process skeleton compiling and the shared memory tested before touching Klipper internals.
- The Klipper motion integration (Component 4.2) is the hardest part. Study klippy/toolhead.py and klippy/stepper.py carefully before implementing. The key function is `ToolHead._process_moves()` which flushes the move queue.
- Test each component independently before integration. The CiA 402 state machine can be tested with just `ethercat_rt` and one LC10E drive.
- The LC10E ESI XML file is critical. If it's not available, use `ethercat xml` to dump the slave's SII and generate one.
- Keep the PID controller simple initially. Proportional + velocity feedforward alone will get the first moves working. Add I and D terms after basic motion is confirmed.
- For auto-tune, start with the relay feedback method — it's well documented and robust. Frequency sweep can come later.
- All encoder position values from the LC10E are in raw counts (int32). Convert to mm using: `position_mm = raw_counts / encoder_resolution * rotation_distance`
