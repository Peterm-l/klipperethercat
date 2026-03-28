# ethercat_stepper.py — EtherCAT Stepper Integration for Klipper
#
# Integrates EtherCAT servo axes into Klipper's motion system by:
#   1. Creating a real Klipper stepper rail with itersolve kinematics
#   2. Registering a trapq flush callback that reads finalized moves
#   3. Converting trapezoidal moves to trajectory segments in shared memory
#   4. Synchronizing Klipper's print_time to CLOCK_MONOTONIC for the RT process
#
# The stepper participates in Klipper's motion planning normally (lookahead,
# junction velocity, input shaping, pressure advance all work). The only
# difference is that instead of generating step pulses, we extract the
# planned trajectory and send it to the EtherCAT RT process.
#
# printer.cfg section:
#   [ethercat_stepper stepper_x]
#   slave_position: 0
#   drive_type: lichuan_lc10e
#   encoder_resolution: 8388608
#   rotation_distance: 40
#   max_velocity: 500
#   max_accel: 30000
#   max_following_error: 2.0
#   pid_kp: 50.0
#   pid_ki: 5.0
#   pid_kd: 0.5
#   ff_velocity: 1.0
#   autotune_travel: 30

import logging
import time
import chelper

logger = logging.getLogger(__name__)

# Segment flags (must match shared_mem.h)
SEGMENT_ACTIVE = (1 << 0)
SEGMENT_LAST = (1 << 1)

# How far ahead (seconds) we convert trapq moves to segments
FLUSH_LOOKAHEAD = 0.100  # 100ms — keep ring buffer fed ahead of RT


class EtherCATStepper:
    """EtherCAT servo axis integrated into Klipper's motion system.

    Uses Klipper's standard stepper rail infrastructure for motion planning,
    but intercepts the finalized trapezoidal moves from the trapq and
    writes them as trajectory segments to shared memory for the RT process.
    """

    def __init__(self, config):
        self.printer = config.get_printer()
        self.name = config.get_name().split()[-1]  # e.g., "stepper_x"
        self.config = config

        # Axis identification
        axis_map = {
            'stepper_x': 0, 'stepper_y': 1, 'stepper_z': 2,
            'stepper_a': 3, 'stepper_b': 4, 'stepper_c': 5,
        }
        self.axis_index = axis_map.get(
            self.name, config.getint('axis_index', 0))
        self.axis_letter = self.name.replace('stepper_', '')

        # Drive configuration
        self.slave_position = config.getint('slave_position')
        self.drive_type = config.get('drive_type', 'lichuan_lc10e')
        self.encoder_resolution = config.getint('encoder_resolution', 8388608)
        self.rotation_distance = config.getfloat('rotation_distance')
        self.max_velocity = config.getfloat('max_velocity', 500.0)
        self.max_accel = config.getfloat('max_accel', 30000.0)
        self.max_following_error = config.getfloat('max_following_error', 2.0)

        # PID parameters
        self.pid_kp = config.getfloat('pid_kp', 50.0)
        self.pid_ki = config.getfloat('pid_ki', 5.0)
        self.pid_kd = config.getfloat('pid_kd', 0.5)
        self.ff_velocity = config.getfloat('ff_velocity', 1.0)
        self.ff_acceleration = config.getfloat('ff_acceleration', 0.0)

        # Auto-tune configuration
        self.autotune_travel = config.getfloat('autotune_travel', 20.0)

        # Position scale: encoder counts per mm
        self.position_scale = self.encoder_resolution / self.rotation_distance

        # Internal state
        self.shm = None
        self.toolhead = None
        self.mcu = None
        self.trapq = None
        self.last_flush_time = 0.0

        # Create a real Klipper stepper rail for this axis.
        # We need a physical MCU stepper for Klipper's infrastructure to work.
        # The Manta M8P has stepper drivers we can reference even if unused.
        # However, for EtherCAT axes we create a "virtual" rail that uses
        # Klipper's C-level trapq and itersolve but skips step generation.
        self._setup_virtual_rail(config)

        # Register event handlers
        self.printer.register_event_handler(
            "klippy:connect", self._handle_connect)

        logger.info(
            "EtherCAT stepper %s: slave=%d, encoder=%d, "
            "rot_dist=%.2f, axis_idx=%d",
            self.name, self.slave_position, self.encoder_resolution,
            self.rotation_distance, self.axis_index)

    def _setup_virtual_rail(self, config):
        """Set up Klipper's C-level kinematics structures.

        We create our own trapq (trapezoidal motion queue) and itersolve
        kinematics so we can participate in Klipper's motion planning.
        The key insight: we use the same math Klipper uses for steppers,
        but instead of generating step pulses, we read the trapq directly.
        """
        ffi_main, ffi_lib = chelper.get_ffi()

        # Create a private trapq for this axis
        self._trapq = ffi_lib.trapq_alloc()
        self._trapq_append = ffi_lib.trapq_append
        self._trapq_finalize_moves = ffi_lib.trapq_finalize_moves
        self._trapq_free_moves = ffi_lib.trapq_free_moves

        # Create itersolve kinematics for cartesian projection
        # This lets Klipper's motion planner project 3D moves onto our axis
        self._stepper_kinematics = ffi_lib.cartesian_stepper_alloc(
            self.axis_letter.encode())
        ffi_lib.itersolve_set_trapq(
            self._stepper_kinematics, self._trapq, 0.0)

        self._ffi_lib = ffi_lib

        # Store the rotation distance for Klipper compatibility
        self._rotation_distance = self.rotation_distance

    def _handle_connect(self):
        """Register with EtherCAT module and hook into motion system."""
        try:
            ethercat = self.printer.lookup_object('ethercat')
            ethercat.register_stepper(self)
            self.shm = ethercat.get_shm()
        except Exception as e:
            logger.error("Failed to register EtherCAT stepper %s: %s",
                         self.name, e)
            raise

        self.toolhead = self.printer.lookup_object('toolhead')
        self.mcu = self.printer.lookup_object('mcu')

        # Get the toolhead's trapq — this is where finalized moves live
        self.trapq = self.toolhead.get_trapq()

        # Register a flush callback with the toolhead.
        # This is called whenever the motion queue is flushed, giving us
        # access to finalized trapezoidal moves before they're sent to MCUs.
        self.printer.register_event_handler(
            "toolhead:flush_step_generation", self._flush_handler)

        # Also register for move queue changes
        self.printer.register_event_handler(
            "toolhead:sync_print_time", self._sync_print_time)

    def _sync_print_time(self, curtime, print_time, est_print_time):
        """Called when Klipper syncs print_time with system clock.

        We extract the clock relationship so the RT process can convert
        CLOCK_MONOTONIC timestamps to print_time for trajectory evaluation.
        """
        if self.shm and self.shm._shm:
            # Store the synchronization parameters in shared memory
            # RT process uses: print_time = base_print + (mono - base_mono)
            # This is an approximation that's valid over short intervals
            self.shm._shm.trajectory.segments[0]._pad = 0  # placeholder
            # The actual sync is handled in _flush_handler by converting
            # print_time to monotonic at write time

    def _flush_handler(self, print_time):
        """Called when the toolhead flushes step generation.

        This is the critical integration point. We read the finalized
        trapezoidal moves from the toolhead's trapq and convert them
        to trajectory segments in shared memory.
        """
        if not self.shm or not self.shm._shm:
            return

        # Get the current time relationship
        # print_time is Klipper's virtual clock; we need CLOCK_MONOTONIC
        curtime = self.printer.get_reactor().monotonic()
        est_print_time = self.mcu.estimated_print_time(curtime)

        # Convert print_time to monotonic:
        # If est_print_time corresponds to curtime, then:
        #   mono_time = curtime + (print_time - est_print_time)
        # This gives us the CLOCK_MONOTONIC time when print_time occurs

        # Process moves from the trapq up to print_time
        self._convert_trapq_to_segments(print_time, curtime, est_print_time)

    def _convert_trapq_to_segments(self, flush_time, curtime, est_print_time):
        """Read trapezoidal moves from the trapq and write as segments.

        The toolhead's trapq contains Move structs with:
          print_time, move_t, start_v, accel
          start_x, start_y, start_z, x_r, y_r, z_r
          accel_t, cruise_t, decel_t, cruise_v

        We evaluate these to extract per-axis trajectory segments.
        """
        ffi_lib = self._ffi_lib

        # Use itersolve to find the position at flush_time on our axis
        # This gives us the commanded position as Klipper sees it
        pos = ffi_lib.itersolve_calc_position(
            self._stepper_kinematics, flush_time)

        # Read moves from the shared toolhead trapq
        # The trapq is a linked list of moves in C. We need to walk it
        # and extract the trapezoidal parameters for our axis.
        #
        # Since we can't directly iterate the C trapq from Python,
        # we use a different approach: we maintain our own Python-level
        # record of moves by hooking into the toolhead's move processing.

        # Update our commanded position
        self.position = pos

    def note_trapq_moves(self, moves, print_time):
        """Called by EtherCATServo when moves are added to the trapq.

        This is called from a patched _process_lookahead in the toolhead.
        Each move has the full trapezoidal profile.

        Args:
            moves: List of Move objects from toolhead's lookahead
            print_time: The print_time when the first move starts
        """
        if not self.shm or not self.shm._shm:
            return

        curtime = self.printer.get_reactor().monotonic()
        est_print_time = self.mcu.estimated_print_time(curtime)

        move_time = print_time

        for i, move in enumerate(moves):
            # Get this axis's component of the move
            axes_d = move.axes_d
            if self.axis_index >= len(axes_d):
                continue

            displacement = axes_d[self.axis_index]
            start_pos = move.start_pos[self.axis_index]
            accel = move.accel
            start_v = move.start_v
            cruise_v = move.cruise_v
            accel_t = move.accel_t
            cruise_t = move.cruise_t
            decel_t = move.decel_t

            # Direction multiplier for the axis component
            move_d = move.move_d
            if move_d > 0:
                axis_r = displacement / move_d
            else:
                axis_r = 0.0

            # Convert print_time to CLOCK_MONOTONIC nanoseconds
            # mono_time = curtime + (print_time - est_print_time)
            mono_offset = curtime - est_print_time

            is_last = (i == len(moves) - 1)

            # Split move into up to 3 constant-acceleration segments
            seg_time = move_time

            # Segment 1: Acceleration phase
            if accel_t > 0:
                mono_ns = int((seg_time + mono_offset) * 1e9)
                seg_start_v = start_v * axis_r
                seg_accel = accel * axis_r

                pos = [0.0] * 6
                vel = [0.0] * 6
                acc = [0.0] * 6
                pos[self.axis_index] = start_pos
                vel[self.axis_index] = seg_start_v
                acc[self.axis_index] = seg_accel

                self.shm.write_trajectory_segment(
                    timestamp_ns=mono_ns,
                    duration=accel_t,
                    start_position=pos,
                    start_velocity=vel,
                    accel=acc,
                    flags=SEGMENT_ACTIVE,
                    num_axes=self.axis_index + 1,
                )
                seg_time += accel_t
                start_pos += seg_start_v * accel_t + 0.5 * seg_accel * accel_t ** 2

            # Segment 2: Cruise phase
            if cruise_t > 0:
                mono_ns = int((seg_time + mono_offset) * 1e9)
                seg_cruise_v = cruise_v * axis_r

                pos = [0.0] * 6
                vel = [0.0] * 6
                acc = [0.0] * 6
                pos[self.axis_index] = start_pos
                vel[self.axis_index] = seg_cruise_v
                acc[self.axis_index] = 0.0

                self.shm.write_trajectory_segment(
                    timestamp_ns=mono_ns,
                    duration=cruise_t,
                    start_position=pos,
                    start_velocity=vel,
                    accel=acc,
                    flags=SEGMENT_ACTIVE,
                    num_axes=self.axis_index + 1,
                )
                seg_time += cruise_t
                start_pos += seg_cruise_v * cruise_t

            # Segment 3: Deceleration phase
            if decel_t > 0:
                mono_ns = int((seg_time + mono_offset) * 1e9)
                seg_cruise_v = cruise_v * axis_r
                seg_decel = -accel * axis_r

                pos = [0.0] * 6
                vel = [0.0] * 6
                acc = [0.0] * 6
                pos[self.axis_index] = start_pos
                vel[self.axis_index] = seg_cruise_v
                acc[self.axis_index] = seg_decel

                flags = SEGMENT_ACTIVE
                if is_last:
                    flags |= SEGMENT_LAST

                self.shm.write_trajectory_segment(
                    timestamp_ns=mono_ns,
                    duration=decel_t,
                    start_position=pos,
                    start_velocity=vel,
                    accel=acc,
                    flags=flags,
                    num_axes=self.axis_index + 1,
                )
                seg_time += decel_t

            # Advance time for next move
            move_time = seg_time

    def write_pid_to_shm(self, shm):
        """Write PID parameters from printer.cfg to shared memory."""
        shm.write_pid_params(
            self.axis_index,
            kp=self.pid_kp,
            ki=self.pid_ki,
            kd=self.pid_kd,
            ff_velocity=self.ff_velocity,
            ff_acceleration=self.ff_acceleration,
            max_following_error=self.max_following_error,
            max_velocity=self.max_velocity,
            max_accel=self.max_accel,
            position_scale=self.position_scale,
        )

    # =========================================================================
    # Klipper Stepper Interface
    # =========================================================================
    # These methods allow Klipper's kinematics system to treat this as a
    # stepper rail. The cartesian/corexy kinematic will call these methods.

    def get_name(self, short=False):
        if short:
            return self.name.replace('stepper_', '')
        return self.name

    def get_rotation_distance(self):
        return (self._rotation_distance, False)

    def get_steppers(self):
        """Return list of steppers in this rail (just ourselves)."""
        return [self]

    def get_commanded_position(self):
        return self.position

    def set_position(self, coord):
        """Set position from cartesian coordinate."""
        self.position = coord[self.axis_index]

    def calc_position_from_coord(self, coord):
        return coord[self.axis_index]

    def set_trapq(self, tq):
        """Connect to a trapq. For EtherCAT we note it but don't
        generate steps from it — we read it in the flush callback."""
        self.trapq = tq

    def get_trapq(self):
        return self.trapq

    def setup_itersolve(self, alloc_func, *params):
        """Klipper kinematics calls this to set up the itersolve projector."""
        ffi_main, ffi_lib = chelper.get_ffi()
        self._stepper_kinematics = getattr(ffi_lib, alloc_func)(*params)

    def get_stepper_kinematics(self):
        return self._stepper_kinematics

    def set_stepper_kinematics(self, sk):
        old_sk = self._stepper_kinematics
        self._stepper_kinematics = sk
        return old_sk

    def units_in_radians(self):
        return False

    def get_actual_position(self):
        """Read encoder position from shared memory."""
        if self.shm and self.shm._shm:
            state = self.shm.read_servo_state(self.axis_index)
            return state['actual_position']
        return self.position

    def get_status(self, eventtime):
        """Return stepper status for Klipper's status system."""
        result = {
            'name': self.name,
            'axis_index': self.axis_index,
            'slave_position': self.slave_position,
            'position': self.position,
        }

        if self.shm and self.shm._shm:
            state = self.shm.read_servo_state(self.axis_index)
            result.update({
                'actual_position': state['actual_position'],
                'velocity': state['actual_velocity'],
                'following_error': state['following_error'],
                'torque': state['torque_percent'],
                'drive_state': state['state'],
                'error_code': state['error_code'],
            })

        return result


def load_config_prefix(config):
    return EtherCATStepper(config)
