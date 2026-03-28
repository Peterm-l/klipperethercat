# ethercat_shm.py — Python Shared Memory Interface for Klipper EtherCAT
#
# Provides a Python wrapper around the POSIX shared memory structures
# defined in shared_mem.h. Uses mmap + ctypes for zero-copy access.
#
# This module is used by the Klipper extras to communicate with the
# ethercat_rt C process.

import ctypes
import mmap
import os
import struct
import time
import logging

logger = logging.getLogger(__name__)

# Constants (must match shared_mem.h)
SHM_NAME = "/klipper_ethercat"
SHM_MAGIC = 0x4B455443  # "KETC"
SHM_VERSION = 1
MAX_TRAJECTORY_SEGMENTS = 256
MAX_AXES = 6
MAX_POSITION_LOG = 16384

# Commands (must match enum shm_command in shared_mem.h)
CMD_NONE = 0
CMD_ENABLE = 1
CMD_DISABLE = 2
CMD_HOME = 3
CMD_ESTOP = 4
CMD_AUTOTUNE = 5
CMD_RECORD = 6
CMD_STOP_RECORD = 7

# Drive states (must match enum servo_drive_state)
DRIVE_STATES = {
    0: 'NOT_READY',
    1: 'SWITCH_ON_DISABLED',
    2: 'READY_TO_SWITCH_ON',
    3: 'SWITCHED_ON',
    4: 'OPERATION_ENABLED',
    5: 'QUICK_STOP_ACTIVE',
    6: 'FAULT_REACTION',
    7: 'FAULT',
}

# Bus states
BUS_STATES = {
    0: 'INIT',
    1: 'PREOP',
    2: 'SAFEOP',
    3: 'OP',
}

# Auto-tune phases
AUTOTUNE_PHASES = {
    0: 'IDLE',
    1: 'SAFETY_CHECK',
    2: 'RELAY_FEEDBACK',
    3: 'INERTIA',
    4: 'FRICTION',
    5: 'COMPUTE_GAINS',
    6: 'VALIDATE',
    7: 'COMPLETE',
    8: 'FAILED',
}


# =============================================================================
# ctypes Structure Definitions (must match shared_mem.h exactly)
# =============================================================================

class TrajectorySegment(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ("timestamp_ns", ctypes.c_uint64),
        ("duration", ctypes.c_double),
        ("start_position", ctypes.c_double * MAX_AXES),
        ("start_velocity", ctypes.c_double * MAX_AXES),
        ("accel", ctypes.c_double * MAX_AXES),
        ("flags", ctypes.c_uint32),
        ("_pad", ctypes.c_uint32),
    ]


class TrajectoryRingBuffer(ctypes.Structure):
    _fields_ = [
        ("write_idx", ctypes.c_uint32),
        ("read_idx", ctypes.c_uint32),
        ("segments", TrajectorySegment * MAX_TRAJECTORY_SEGMENTS),
    ]


class ServoState(ctypes.Structure):
    _fields_ = [
        ("actual_position", ctypes.c_double),
        ("actual_velocity", ctypes.c_double),
        ("commanded_position", ctypes.c_double),
        ("following_error", ctypes.c_double),
        ("torque_percent", ctypes.c_double),
        ("cia402_status", ctypes.c_uint16),
        ("error_code", ctypes.c_uint16),
        ("state", ctypes.c_uint8),
        ("homing_complete", ctypes.c_uint8),
        ("_pad", ctypes.c_uint8 * 2),
    ]


class EtherCATStatus(ctypes.Structure):
    _fields_ = [
        ("cycle_count", ctypes.c_uint32),
        ("cycle_overruns", ctypes.c_uint32),
        ("max_cycle_time_us", ctypes.c_double),
        ("avg_cycle_time_us", ctypes.c_double),
        ("bus_state", ctypes.c_uint8),
        ("num_slaves", ctypes.c_uint8),
        ("_pad", ctypes.c_uint8 * 6),
    ]


class PIDParams(ctypes.Structure):
    _fields_ = [
        ("kp", ctypes.c_double),
        ("ki", ctypes.c_double),
        ("kd", ctypes.c_double),
        ("ff_velocity", ctypes.c_double),
        ("ff_acceleration", ctypes.c_double),
        ("max_following_error", ctypes.c_double),
        ("max_velocity", ctypes.c_double),
        ("max_accel", ctypes.c_double),
        ("position_scale", ctypes.c_double),
        ("integral_windup_limit", ctypes.c_double),
    ]


class AutotuneConfig(ctypes.Structure):
    _fields_ = [
        ("travel_mm", ctypes.c_double),
        ("aggression", ctypes.c_uint8),
        ("validate_only", ctypes.c_uint8),
        ("_pad", ctypes.c_uint8 * 6),
    ]


class AutotuneStatus(ctypes.Structure):
    _fields_ = [
        ("phase", ctypes.c_uint8),
        ("progress_percent", ctypes.c_uint8),
        ("_pad", ctypes.c_uint8 * 6),
        ("message", ctypes.c_char * 128),
        ("Tu", ctypes.c_double),
        ("Au", ctypes.c_double),
        ("J", ctypes.c_double),
        ("Fs", ctypes.c_double),
        ("Fv", ctypes.c_double),
        ("kp", ctypes.c_double),
        ("ki", ctypes.c_double),
        ("kd", ctypes.c_double),
        ("ff_velocity", ctypes.c_double),
        ("rise_time_ms", ctypes.c_double),
        ("overshoot_percent", ctypes.c_double),
        ("settling_time_ms", ctypes.c_double),
        ("ss_error_mm", ctypes.c_double),
        ("tracking_rms_mm", ctypes.c_double),
    ]


class FollowingErrorStats(ctypes.Structure):
    _fields_ = [
        ("current_rms", ctypes.c_double),
        ("baseline_rms", ctypes.c_double),
        ("peak_error", ctypes.c_double),
        ("degradation_warning", ctypes.c_uint8),
        ("_pad", ctypes.c_uint8 * 7),
    ]


class PositionLogEntry(ctypes.Structure):
    _fields_ = [
        ("time_ms", ctypes.c_double),
        ("commanded", ctypes.c_double),
        ("actual", ctypes.c_double),
        ("error", ctypes.c_double),
        ("torque", ctypes.c_double),
    ]


class PositionLog(ctypes.Structure):
    _fields_ = [
        ("recording", ctypes.c_uint32),
        ("log_count", ctypes.c_uint32),
        ("log", PositionLogEntry * MAX_POSITION_LOG),
    ]


class KlipperEtherCATShm(ctypes.Structure):
    _fields_ = [
        ("magic", ctypes.c_uint32),
        ("version", ctypes.c_uint32),
        ("trajectory", TrajectoryRingBuffer),
        ("axes", ServoState * MAX_AXES),
        ("status", EtherCATStatus),
        ("pid", PIDParams * MAX_AXES),
        ("command", ctypes.c_uint8),
        ("command_axis", ctypes.c_uint8),
        ("command_ack", ctypes.c_uint8),
        ("_cmd_pad", ctypes.c_uint8 * 5),
        ("autotune_config", AutotuneConfig * MAX_AXES),
        ("autotune_status", AutotuneStatus * MAX_AXES),
        ("following_error_stats", FollowingErrorStats * MAX_AXES),
        ("position_log", PositionLog * MAX_AXES),
    ]


# Segment flags
SEGMENT_ACTIVE = (1 << 0)
SEGMENT_LAST = (1 << 1)
SEGMENT_ESTOP = (1 << 2)


# =============================================================================
# Shared Memory Manager
# =============================================================================

class EtherCATSharedMemory:
    """Python interface to the shared memory used by ethercat_rt."""

    def __init__(self):
        self._shm_fd = None
        self._mmap = None
        self._shm = None
        self._created = False

    def create(self):
        """Create the shared memory segment (called by klippy at startup)."""
        shm_size = ctypes.sizeof(KlipperEtherCATShm)

        # Remove stale segment
        try:
            os.unlink("/dev/shm" + SHM_NAME)
        except OSError:
            pass

        # Create new segment
        shm_path = "/dev/shm" + SHM_NAME
        self._shm_fd = os.open(shm_path,
                                os.O_CREAT | os.O_RDWR | os.O_EXCL,
                                0o660)
        os.ftruncate(self._shm_fd, shm_size)

        # Memory-map it
        self._mmap = mmap.mmap(self._shm_fd, shm_size,
                                mmap.MAP_SHARED,
                                mmap.PROT_READ | mmap.PROT_WRITE)

        # Create ctypes structure overlay
        self._shm = KlipperEtherCATShm.from_buffer(self._mmap)

        # Initialize
        self._shm.magic = SHM_MAGIC
        self._shm.version = SHM_VERSION
        self._shm.trajectory.write_idx = 0
        self._shm.trajectory.read_idx = 0
        self._shm.command = CMD_NONE

        self._created = True
        logger.info("Created shared memory %s (%d bytes)", SHM_NAME, shm_size)

    def open(self):
        """Open existing shared memory (for testing/debugging)."""
        shm_size = ctypes.sizeof(KlipperEtherCATShm)
        shm_path = "/dev/shm" + SHM_NAME

        self._shm_fd = os.open(shm_path, os.O_RDWR)
        self._mmap = mmap.mmap(self._shm_fd, shm_size,
                                mmap.MAP_SHARED,
                                mmap.PROT_READ | mmap.PROT_WRITE)
        self._shm = KlipperEtherCATShm.from_buffer(self._mmap)

        if self._shm.magic != SHM_MAGIC:
            raise RuntimeError(
                f"Bad shared memory magic: 0x{self._shm.magic:08X} "
                f"(expected 0x{SHM_MAGIC:08X})")
        if self._shm.version != SHM_VERSION:
            raise RuntimeError(
                f"Shared memory version mismatch: {self._shm.version} "
                f"(expected {SHM_VERSION})")

        self._created = False
        logger.info("Opened shared memory %s", SHM_NAME)

    def close(self):
        """Close and optionally unlink shared memory."""
        if self._mmap:
            self._mmap.close()
            self._mmap = None
        if self._shm_fd is not None:
            os.close(self._shm_fd)
            self._shm_fd = None
        if self._created:
            try:
                os.unlink("/dev/shm" + SHM_NAME)
            except OSError:
                pass
        self._shm = None

    # =========================================================================
    # Trajectory Ring Buffer
    # =========================================================================

    def write_trajectory_segment(self, timestamp_ns, duration,
                                  start_position, start_velocity, accel,
                                  flags=SEGMENT_ACTIVE, num_axes=3):
        """Write a trajectory segment to the ring buffer.

        Args:
            timestamp_ns: Absolute start time (nanoseconds)
            duration: Segment duration (seconds)
            start_position: List of start positions per axis (mm)
            start_velocity: List of start velocities per axis (mm/s)
            accel: List of accelerations per axis (mm/s^2)
            flags: Segment flags (SEGMENT_ACTIVE, SEGMENT_LAST, etc.)
            num_axes: Number of axes to write
        """
        rb = self._shm.trajectory
        w = rb.write_idx
        r = rb.read_idx

        # Check for space
        next_w = (w + 1) & (MAX_TRAJECTORY_SEGMENTS - 1)
        if next_w == (r & (MAX_TRAJECTORY_SEGMENTS - 1)):
            raise RuntimeError("Trajectory ring buffer full")

        idx = w & (MAX_TRAJECTORY_SEGMENTS - 1)
        seg = rb.segments[idx]

        seg.timestamp_ns = timestamp_ns
        seg.duration = duration
        seg.flags = flags

        for i in range(min(num_axes, MAX_AXES)):
            seg.start_position[i] = start_position[i] if i < len(start_position) else 0.0
            seg.start_velocity[i] = start_velocity[i] if i < len(start_velocity) else 0.0
            seg.accel[i] = accel[i] if i < len(accel) else 0.0

        # Memory barrier + increment write index
        rb.write_idx = w + 1

    def get_trajectory_buffer_usage(self):
        """Return (used, total) slots in the trajectory ring buffer."""
        w = self._shm.trajectory.write_idx
        r = self._shm.trajectory.read_idx
        used = (w - r) & (MAX_TRAJECTORY_SEGMENTS - 1)
        return used, MAX_TRAJECTORY_SEGMENTS - 1

    # =========================================================================
    # Servo State
    # =========================================================================

    def read_servo_state(self, axis):
        """Read the current servo state for an axis."""
        if axis >= MAX_AXES:
            raise ValueError(f"Invalid axis: {axis}")
        s = self._shm.axes[axis]
        return {
            'actual_position': s.actual_position,
            'actual_velocity': s.actual_velocity,
            'commanded_position': s.commanded_position,
            'following_error': s.following_error,
            'torque_percent': s.torque_percent,
            'cia402_status': s.cia402_status,
            'error_code': s.error_code,
            'state': DRIVE_STATES.get(s.state, f'UNKNOWN({s.state})'),
            'homing_complete': bool(s.homing_complete),
        }

    # =========================================================================
    # EtherCAT Bus Status
    # =========================================================================

    def read_ethercat_status(self):
        """Read the EtherCAT bus status."""
        s = self._shm.status
        return {
            'cycle_count': s.cycle_count,
            'cycle_overruns': s.cycle_overruns,
            'max_cycle_time_us': s.max_cycle_time_us,
            'avg_cycle_time_us': s.avg_cycle_time_us,
            'bus_state': BUS_STATES.get(s.bus_state, f'UNKNOWN({s.bus_state})'),
            'num_slaves': s.num_slaves,
        }

    # =========================================================================
    # PID Parameters
    # =========================================================================

    def write_pid_params(self, axis, kp, ki, kd, ff_velocity=1.0,
                          ff_acceleration=0.0, max_following_error=2.0,
                          max_velocity=500.0, max_accel=30000.0,
                          position_scale=1.0, integral_windup_limit=10.0):
        """Write PID parameters for an axis."""
        if axis >= MAX_AXES:
            raise ValueError(f"Invalid axis: {axis}")
        p = self._shm.pid[axis]
        p.kp = kp
        p.ki = ki
        p.kd = kd
        p.ff_velocity = ff_velocity
        p.ff_acceleration = ff_acceleration
        p.max_following_error = max_following_error
        p.max_velocity = max_velocity
        p.max_accel = max_accel
        p.position_scale = position_scale
        p.integral_windup_limit = integral_windup_limit

    # =========================================================================
    # Commands
    # =========================================================================

    def send_command(self, command, axis=0, timeout=2.0):
        """Send a command to the RT process and wait for acknowledgement.

        Args:
            command: Command string ('ENABLE', 'DISABLE', 'HOME', 'ESTOP',
                     'AUTOTUNE', 'RECORD', 'STOP_RECORD')
            axis: Axis index
            timeout: Seconds to wait for ack
        """
        cmd_map = {
            'ENABLE': CMD_ENABLE,
            'DISABLE': CMD_DISABLE,
            'HOME': CMD_HOME,
            'ESTOP': CMD_ESTOP,
            'AUTOTUNE': CMD_AUTOTUNE,
            'RECORD': CMD_RECORD,
            'STOP_RECORD': CMD_STOP_RECORD,
        }
        cmd = cmd_map.get(command)
        if cmd is None:
            raise ValueError(f"Unknown command: {command}")

        self._shm.command_ack = 0
        self._shm.command_axis = axis
        self._shm.command = cmd

        # Wait for ack
        start = time.monotonic()
        while time.monotonic() - start < timeout:
            if self._shm.command_ack == cmd:
                return True
            time.sleep(0.01)

        logger.warning("Command %s timeout (no ack from RT process)", command)
        return False

    # =========================================================================
    # Auto-Tune
    # =========================================================================

    def write_autotune_config(self, axis, config):
        """Write auto-tune configuration for an axis."""
        if axis >= MAX_AXES:
            raise ValueError(f"Invalid axis: {axis}")
        c = self._shm.autotune_config[axis]
        c.travel_mm = config.get('travel_mm', 20.0)

        aggression_map = {
            'conservative': 0,
            'moderate': 1,
            'aggressive': 2,
        }
        c.aggression = aggression_map.get(
            config.get('aggression', 'moderate'), 1)
        c.validate_only = config.get('validate_only', 0)

    def read_autotune_status(self, axis):
        """Read auto-tune status for an axis."""
        if axis >= MAX_AXES:
            raise ValueError(f"Invalid axis: {axis}")
        s = self._shm.autotune_status[axis]
        return {
            'phase': AUTOTUNE_PHASES.get(s.phase, f'UNKNOWN({s.phase})'),
            'progress_percent': s.progress_percent,
            'message': s.message.decode('utf-8', errors='replace').rstrip('\x00'),
        }

    def read_autotune_results(self, axis):
        """Read auto-tune results (valid when phase == COMPLETE)."""
        if axis >= MAX_AXES:
            raise ValueError(f"Invalid axis: {axis}")
        s = self._shm.autotune_status[axis]
        return {
            'Tu': s.Tu,
            'Au': s.Au,
            'J': s.J,
            'Fs': s.Fs,
            'Fv': s.Fv,
            'kp': s.kp,
            'ki': s.ki,
            'kd': s.kd,
            'ff_velocity': s.ff_velocity,
            'rise_time_ms': s.rise_time_ms,
            'overshoot_percent': s.overshoot_percent,
            'settling_time_ms': s.settling_time_ms,
            'ss_error_mm': s.ss_error_mm,
            'tracking_rms_mm': s.tracking_rms_mm,
        }

    # =========================================================================
    # Following Error Stats
    # =========================================================================

    def read_following_error_stats(self, axis):
        """Read following error statistics for an axis."""
        if axis >= MAX_AXES:
            raise ValueError(f"Invalid axis: {axis}")
        s = self._shm.following_error_stats[axis]
        return {
            'current_rms': s.current_rms,
            'baseline_rms': s.baseline_rms,
            'peak_error': s.peak_error,
            'degradation_warning': bool(s.degradation_warning),
        }

    # =========================================================================
    # Position Logging
    # =========================================================================

    def read_position_log(self, axis):
        """Read the position log for an axis (for SERVO_PLOT)."""
        if axis >= MAX_AXES:
            raise ValueError(f"Invalid axis: {axis}")
        log = self._shm.position_log[axis]
        count = min(log.log_count, MAX_POSITION_LOG)
        entries = []
        for i in range(count):
            e = log.log[i]
            entries.append({
                'time': e.time_ms,
                'cmd': e.commanded,
                'actual': e.actual,
                'error': e.error,
                'torque': e.torque,
            })
        return entries
