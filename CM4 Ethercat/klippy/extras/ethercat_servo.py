# ethercat_servo.py — Main Klipper EtherCAT Module
#
# This is the top-level Klipper extras module for EtherCAT servo integration.
# It manages the RT process lifecycle, shared memory, and provides the
# primary GCode interface.
#
# printer.cfg section:
#   [ethercat]
#   cycle_time: 0.001           # 1ms EtherCAT cycle
#   rt_process_path: /usr/local/bin/ethercat_rt
#   rt_config_path: /etc/klipper-ethercat/config.ini

import os
import subprocess
import time
import logging

from . import ethercat_shm

logger = logging.getLogger(__name__)


class EtherCATServo:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.config = config
        self.cycle_time = config.getfloat('cycle_time', 0.001)
        self.rt_process_path = config.get(
            'rt_process_path', '/usr/local/bin/ethercat_rt')
        self.rt_config_path = config.get(
            'rt_config_path', '/etc/klipper-ethercat/config.ini')

        self.rt_process = None
        self.shm = ethercat_shm.EtherCATSharedMemory()
        self.ethercat_steppers = []

        # Register event handlers
        self.printer.register_event_handler(
            "klippy:connect", self._handle_connect)
        self.printer.register_event_handler(
            "klippy:shutdown", self._handle_shutdown)
        self.printer.register_event_handler(
            "klippy:disconnect", self._handle_disconnect)

        # Register GCode commands
        gcode = self.printer.lookup_object('gcode')
        gcode.register_command(
            'ETHERCAT_STATUS', self.cmd_ETHERCAT_STATUS,
            desc="Report EtherCAT bus and servo status")
        gcode.register_command(
            'SERVO_ENABLE', self.cmd_SERVO_ENABLE,
            desc="Enable an EtherCAT servo axis")
        gcode.register_command(
            'SERVO_DISABLE', self.cmd_SERVO_DISABLE,
            desc="Disable an EtherCAT servo axis")
        gcode.register_command(
            'SERVO_POSITION', self.cmd_SERVO_POSITION,
            desc="Report servo position for an axis")

    def register_stepper(self, stepper):
        """Called by EtherCATStepper instances during init."""
        self.ethercat_steppers.append(stepper)

    def get_shm(self):
        """Get the shared memory interface."""
        return self.shm

    def _hook_motion_system(self):
        """Hook into Klipper's motion system to intercept finalized moves.

        We monkey-patch the toolhead's _process_lookahead to also forward
        finalized moves to our EtherCAT steppers. This runs after the
        lookahead has computed junction velocities and trapezoidal profiles.
        """
        toolhead = self.printer.lookup_object('toolhead')

        # Save original method
        original_process = toolhead._process_lookahead

        ethercat_steppers = self.ethercat_steppers

        def patched_process_lookahead(flush_count=None):
            # Call original — this runs the lookahead, calls trapq_append,
            # and schedules step generation for normal steppers
            result = original_process(flush_count)

            # Now forward the finalized moves to EtherCAT steppers.
            # The moves have been through lookahead and have final
            # accel_t, cruise_t, decel_t, start_v, cruise_v.
            #
            # We access the move queue that was just processed.
            # toolhead.move_queue contains Move objects.
            if hasattr(toolhead, '_last_flush_moves'):
                moves = toolhead._last_flush_moves
                if moves and ethercat_steppers:
                    print_time = moves[0].timing[0]  # move start time
                    for stepper in ethercat_steppers:
                        stepper.note_trapq_moves(moves, print_time)

            return result

        toolhead._process_lookahead = patched_process_lookahead

        # Also hook into the move queue flush to capture moves.
        # We patch LookAheadQueue.flush() to save the moves for us.
        lookahead = toolhead.lookahead
        original_flush = lookahead.flush

        def patched_flush(lazy=False):
            result = original_flush(lazy)
            # Save the flushed moves for our patched_process_lookahead
            if hasattr(lookahead, '_queue'):
                toolhead._last_flush_moves = list(lookahead._queue)
            return result

        lookahead.flush = patched_flush

        logger.info("EtherCAT motion system hook installed")

    def _verify_prerequisites(self):
        """Verify that the system is properly configured for EtherCAT."""
        errors = []

        # Check PREEMPT_RT kernel
        try:
            uname = subprocess.check_output(['uname', '-v']).decode()
            if 'PREEMPT_RT' not in uname:
                errors.append(
                    "Kernel is not PREEMPT_RT. "
                    "Run scripts/setup_rt_kernel.sh to build and install one.")
        except Exception:
            errors.append("Could not check kernel version")

        # Check isolcpus
        try:
            with open('/proc/cmdline', 'r') as f:
                cmdline = f.read()
            if 'isolcpus=3' not in cmdline:
                errors.append(
                    "isolcpus=3 not found in kernel command line. "
                    "Add 'isolcpus=3 nohz_full=3 rcu_nocbs=3' to "
                    "/boot/firmware/cmdline.txt and reboot.")
        except Exception:
            errors.append("Could not read /proc/cmdline")

        # Check EtherCAT master module
        try:
            lsmod = subprocess.check_output(['lsmod']).decode()
            if 'ec_master' not in lsmod:
                errors.append(
                    "EtherCAT master kernel module not loaded. "
                    "Run: sudo systemctl start ethercat")
        except Exception:
            errors.append("Could not check loaded modules")

        # Check RT binary exists
        if not os.path.isfile(self.rt_process_path):
            errors.append(
                f"RT process binary not found at {self.rt_process_path}. "
                f"Build it first: cd src && mkdir build && cd build && "
                f"cmake .. && make && sudo make install")

        return errors

    def _handle_connect(self):
        """Called when klippy connects. Launch the RT process."""
        # Verify prerequisites
        errors = self._verify_prerequisites()
        if errors:
            for err in errors:
                logger.error("EtherCAT prerequisite error: %s", err)
            raise self.printer.config_error(
                "EtherCAT prerequisites not met:\n  " +
                "\n  ".join(errors))

        # Create shared memory
        self.shm.create()

        # Write PID parameters from printer.cfg
        for stepper in self.ethercat_steppers:
            stepper.write_pid_to_shm(self.shm)

        # Launch the RT process
        logger.info("Launching EtherCAT RT process: %s",
                     self.rt_process_path)
        try:
            self.rt_process = subprocess.Popen(
                ['sudo', self.rt_process_path,
                 '--config', self.rt_config_path],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
            )
        except Exception as e:
            self.shm.close()
            raise self.printer.config_error(
                f"Failed to launch RT process: {e}")

        # Wait for RT process to initialize and reach OP state
        timeout = 10.0
        start = time.monotonic()
        bus_state = None
        while time.monotonic() - start < timeout:
            try:
                status = self.shm.read_ethercat_status()
                bus_state = status['bus_state']
                if bus_state == 'OP':
                    break
            except Exception:
                pass
            time.sleep(0.1)
        else:
            stderr = ""
            try:
                stderr = self.rt_process.stderr.read().decode()
            except Exception:
                pass
            self.rt_process.terminate()
            self.shm.close()
            raise self.printer.config_error(
                f"EtherCAT bus did not reach OP state within {timeout}s. "
                f"Last state: {bus_state}. "
                f"Check servo drive power and Ethernet connection.\n"
                f"RT process stderr: {stderr}")

        # Verify RT process is on isolated core
        pid = self.rt_process.pid
        try:
            with open(f'/proc/{pid}/status', 'r') as f:
                for line in f:
                    if line.startswith('Cpus_allowed_list:'):
                        allowed = line.split(':')[1].strip()
                        if allowed != '3':
                            logger.warning(
                                "ethercat_rt (PID %d) on CPUs %s, "
                                "expected only core 3", pid, allowed)
                        else:
                            logger.info(
                                "ethercat_rt (PID %d) confirmed on "
                                "isolated core 3", pid)
                        break
        except Exception:
            pass

        status = self.shm.read_ethercat_status()
        logger.info(
            "EtherCAT connected: %d slaves, bus state %s, "
            "RT process on core 3",
            status['num_slaves'], status['bus_state'])

        # Hook into Klipper's motion system to forward moves to EtherCAT
        self._hook_motion_system()

    def _handle_shutdown(self):
        """Emergency shutdown — stop all servos immediately."""
        if self.shm and self.shm._shm:
            self.shm.send_command('ESTOP', 0, timeout=1.0)

        if self.rt_process:
            try:
                self.rt_process.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                self.rt_process.terminate()
                try:
                    self.rt_process.wait(timeout=1.0)
                except subprocess.TimeoutExpired:
                    self.rt_process.kill()

    def _handle_disconnect(self):
        """Clean disconnect."""
        self._handle_shutdown()
        if self.shm:
            self.shm.close()

    # =========================================================================
    # GCode Commands
    # =========================================================================

    def cmd_ETHERCAT_STATUS(self, gcmd):
        """Report EtherCAT bus and servo status."""
        if not self.shm or not self.shm._shm:
            gcmd.respond_info("EtherCAT: not connected")
            return

        status = self.shm.read_ethercat_status()
        gcmd.respond_info(
            f"EtherCAT bus: {status['bus_state']}, "
            f"{status['num_slaves']} slaves, "
            f"cycle avg {status['avg_cycle_time_us']:.1f}us, "
            f"max {status['max_cycle_time_us']:.1f}us, "
            f"overruns {status['cycle_overruns']}, "
            f"cycles {status['cycle_count']}")

        for stepper in self.ethercat_steppers:
            idx = stepper.axis_index
            state = self.shm.read_servo_state(idx)
            fe_stats = self.shm.read_following_error_stats(idx)
            gcmd.respond_info(
                f"  Axis {stepper.name}: {state['state']}, "
                f"pos={state['actual_position']:.4f}mm, "
                f"vel={state['actual_velocity']:.1f}mm/s, "
                f"fe={state['following_error']:.4f}mm, "
                f"torque={state['torque_percent']:.1f}%, "
                f"fe_rms={fe_stats['current_rms']:.4f}mm")

    def cmd_SERVO_ENABLE(self, gcmd):
        """Enable an EtherCAT servo axis."""
        axis = gcmd.get('AXIS', 'ALL').upper()
        if axis == 'ALL':
            for stepper in self.ethercat_steppers:
                self.shm.send_command('ENABLE', stepper.axis_index)
            gcmd.respond_info("All EtherCAT axes enabled")
        else:
            idx = self._axis_name_to_index(axis)
            if idx is None:
                raise gcmd.error(f"Unknown axis: {axis}")
            self.shm.send_command('ENABLE', idx)
            gcmd.respond_info(f"Axis {axis} enabled")

    def cmd_SERVO_DISABLE(self, gcmd):
        """Disable an EtherCAT servo axis."""
        axis = gcmd.get('AXIS', 'ALL').upper()
        if axis == 'ALL':
            for stepper in self.ethercat_steppers:
                self.shm.send_command('DISABLE', stepper.axis_index)
            gcmd.respond_info("All EtherCAT axes disabled")
        else:
            idx = self._axis_name_to_index(axis)
            if idx is None:
                raise gcmd.error(f"Unknown axis: {axis}")
            self.shm.send_command('DISABLE', idx)
            gcmd.respond_info(f"Axis {axis} disabled")

    def cmd_SERVO_POSITION(self, gcmd):
        """Report servo position for an axis."""
        axis = gcmd.get('AXIS').upper()
        idx = self._axis_name_to_index(axis)
        if idx is None:
            raise gcmd.error(f"Unknown axis: {axis}")
        state = self.shm.read_servo_state(idx)
        gcmd.respond_info(
            f"Axis {axis}: pos={state['actual_position']:.4f}mm, "
            f"cmd={state['commanded_position']:.4f}mm, "
            f"fe={state['following_error']:.4f}mm")

    def _axis_name_to_index(self, name):
        """Convert axis name (X, Y, Z) to stepper index."""
        name = name.upper()
        for stepper in self.ethercat_steppers:
            if stepper.name.upper() == name:
                return stepper.axis_index
            # Also match stepper_x → X
            if stepper.name.upper() == f"STEPPER_{name}":
                return stepper.axis_index
        return None


def load_config(config):
    return EtherCATServo(config)
