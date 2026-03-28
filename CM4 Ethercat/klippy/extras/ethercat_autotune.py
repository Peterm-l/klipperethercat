# ethercat_autotune.py — Auto-Tune GCode Interface
#
# Provides GCode commands for servo auto-tuning and diagnostics:
#   SERVO_AUTOTUNE — Run the auto-tune sequence on an axis
#   SERVO_STATUS   — Show detailed servo status
#   SERVO_PLOT     — Record and dump position log data

import logging
import time

logger = logging.getLogger(__name__)


class EtherCATAutoTune:
    """GCode interface for servo auto-tuning and diagnostics."""

    def __init__(self, config):
        self.printer = config.get_printer()
        self.shm = None
        self.ethercat = None

        self.printer.register_event_handler(
            "klippy:connect", self._handle_connect)

        gcode = self.printer.lookup_object('gcode')
        gcode.register_command(
            'SERVO_AUTOTUNE', self.cmd_SERVO_AUTOTUNE,
            desc="Run servo auto-tune on an axis")
        gcode.register_command(
            'SERVO_STATUS', self.cmd_SERVO_STATUS,
            desc="Show detailed servo status with following error stats")
        gcode.register_command(
            'SERVO_PLOT', self.cmd_SERVO_PLOT,
            desc="Record position data for plotting")

    def _handle_connect(self):
        self.ethercat = self.printer.lookup_object('ethercat')
        self.shm = self.ethercat.get_shm()

    def _get_axis_index(self, axis_name):
        """Convert axis name to index."""
        return self.ethercat._axis_name_to_index(axis_name)

    def cmd_SERVO_AUTOTUNE(self, gcmd):
        """Run the auto-tune sequence on an axis.

        Usage:
            SERVO_AUTOTUNE AXIS=X
            SERVO_AUTOTUNE AXIS=X AGGRESSION=conservative
            SERVO_AUTOTUNE AXIS=X AGGRESSION=aggressive
            SERVO_AUTOTUNE AXIS=X TRAVEL=30
            SERVO_AUTOTUNE AXIS=X VALIDATE_ONLY=1
        """
        if not self.shm or not self.shm._shm:
            raise gcmd.error("EtherCAT not connected")

        axis = gcmd.get('AXIS').upper()
        aggression = gcmd.get('AGGRESSION', 'moderate').lower()
        travel = gcmd.getfloat('TRAVEL', 20.0)
        validate_only = gcmd.getint('VALIDATE_ONLY', 0)

        axis_idx = self._get_axis_index(axis)
        if axis_idx is None:
            raise gcmd.error(f"Unknown EtherCAT axis: {axis}")

        # Safety checks
        toolhead = self.printer.lookup_object('toolhead')
        # Check drive is enabled
        state = self.shm.read_servo_state(axis_idx)
        if state['state'] != 'OPERATION_ENABLED':
            raise gcmd.error(
                f"Axis {axis} drive not enabled. "
                f"Run SERVO_ENABLE AXIS={axis} first.")

        # Configure and start auto-tune
        self.shm.write_autotune_config(axis_idx, {
            'travel_mm': travel,
            'aggression': aggression,
            'validate_only': validate_only,
        })
        self.shm.send_command('AUTOTUNE', axis_idx)

        # Report
        gcmd.respond_info(f"Auto-tune starting for axis {axis}")
        gcmd.respond_info(f"  Safe travel: +/-{travel}mm from current position")
        gcmd.respond_info(f"  Aggression: {aggression}")
        if validate_only:
            gcmd.respond_info("  Mode: validate current gains only")

        phase_names = {
            'SAFETY_CHECK': 'Safety checks',
            'RELAY_FEEDBACK': 'Relay feedback test (axis will oscillate gently)',
            'INERTIA': 'Inertia measurement (axis will accelerate briefly)',
            'FRICTION': 'Friction measurement (axis will creep slowly)',
            'COMPUTE_GAINS': 'Computing gains',
            'VALIDATE': 'Validation step response (small test moves)',
        }

        # Poll progress
        last_phase = None
        last_progress = -1
        reactor = self.printer.get_reactor()

        while True:
            status = self.shm.read_autotune_status(axis_idx)

            # Report phase changes
            if status['phase'] != last_phase:
                last_phase = status['phase']
                name = phase_names.get(status['phase'], status['phase'])
                gcmd.respond_info(f"  Phase: {name}...")

            # Report progress changes (every 10%)
            progress = status['progress_percent']
            if progress // 10 != last_progress // 10:
                last_progress = progress
                if status['message']:
                    gcmd.respond_info(
                        f"    [{progress}%] {status['message']}")

            # Check completion
            if status['phase'] == 'COMPLETE':
                break
            elif status['phase'] == 'FAILED':
                raise gcmd.error(
                    f"Auto-tune failed: {status['message']}")
            elif status['phase'] == 'IDLE':
                raise gcmd.error(
                    "Auto-tune did not start. Check drive state.")

            # Yield to reactor
            reactor.pause(reactor.monotonic() + 0.1)

        # Read and display results
        results = self.shm.read_autotune_results(axis_idx)

        gcmd.respond_info(
            f"Auto-tune complete for axis {axis}:\n"
            f"  System characterization:\n"
            f"    Oscillation period: {results['Tu']:.4f}s\n"
            f"    Oscillation amplitude: {results['Au']:.3f}mm\n"
            f"    Load inertia: {results['J']:.6f} kg*m^2\n"
            f"    Static friction: {results['Fs']:.3f}N\n"
            f"    Viscous friction: {results['Fv']:.5f} N*s/mm\n"
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

        # Also update the live PID params in shared memory
        self.shm.write_pid_params(
            axis_idx,
            kp=results['kp'],
            ki=results['ki'],
            kd=results['kd'],
            ff_velocity=results['ff_velocity'],
        )

    def cmd_SERVO_STATUS(self, gcmd):
        """Show detailed servo status including following error monitoring."""
        if not self.shm or not self.shm._shm:
            raise gcmd.error("EtherCAT not connected")

        axis = gcmd.get('AXIS', None)

        if axis:
            axis = axis.upper()
            axis_idx = self._get_axis_index(axis)
            if axis_idx is None:
                raise gcmd.error(f"Unknown axis: {axis}")

            state = self.shm.read_servo_state(axis_idx)
            stats = self.shm.read_following_error_stats(axis_idx)

            gcmd.respond_info(
                f"Axis {axis}:\n"
                f"  State: {state['state']}\n"
                f"  Position: {state['actual_position']:.4f}mm\n"
                f"  Velocity: {state['actual_velocity']:.1f}mm/s\n"
                f"  Commanded: {state['commanded_position']:.4f}mm\n"
                f"  Following error: {state['following_error']:.4f}mm\n"
                f"  Following error RMS: {stats['current_rms']:.4f}mm\n"
                f"  Baseline RMS: {stats['baseline_rms']:.4f}mm\n"
                f"  Peak error: {stats['peak_error']:.4f}mm\n"
                f"  Torque: {state['torque_percent']:.1f}%\n"
                f"  CiA 402 status: 0x{state['cia402_status']:04X}\n"
                f"  Error code: 0x{state['error_code']:04X}")

            if stats['degradation_warning']:
                ratio = (stats['current_rms'] / stats['baseline_rms'] * 100
                         if stats['baseline_rms'] > 0 else 0)
                gcmd.respond_info(
                    f"  WARNING: Following error increased "
                    f"{ratio:.0f}% since last tune. "
                    f"Consider running SERVO_AUTOTUNE AXIS={axis}")
        else:
            # Report all axes
            status = self.shm.read_ethercat_status()
            gcmd.respond_info(
                f"EtherCAT bus: {status['bus_state']}, "
                f"{status['num_slaves']} slaves, "
                f"cycle avg {status['avg_cycle_time_us']:.1f}us, "
                f"max {status['max_cycle_time_us']:.1f}us, "
                f"overruns {status['cycle_overruns']}")

            for stepper in self.ethercat.ethercat_steppers:
                idx = stepper.axis_index
                state = self.shm.read_servo_state(idx)
                gcmd.respond_info(
                    f"  {stepper.name}: {state['state']}, "
                    f"pos={state['actual_position']:.4f}mm, "
                    f"fe={state['following_error']:.4f}mm, "
                    f"torque={state['torque_percent']:.1f}%")

    def cmd_SERVO_PLOT(self, gcmd):
        """Record and dump position log data for plotting.

        Usage:
            SERVO_PLOT AXIS=X              # Record 2 seconds
            SERVO_PLOT AXIS=X DURATION=5   # Record 5 seconds
        """
        if not self.shm or not self.shm._shm:
            raise gcmd.error("EtherCAT not connected")

        axis = gcmd.get('AXIS').upper()
        duration = gcmd.getfloat('DURATION', 2.0)

        axis_idx = self._get_axis_index(axis)
        if axis_idx is None:
            raise gcmd.error(f"Unknown axis: {axis}")

        gcmd.respond_info(
            f"Recording {duration}s of position data for axis {axis}...")

        # Start recording
        self.shm.send_command('RECORD', axis_idx)

        # Wait for duration
        reactor = self.printer.get_reactor()
        reactor.pause(reactor.monotonic() + duration)

        # Stop recording
        self.shm.send_command('STOP_RECORD', axis_idx)

        # Read and write CSV
        log = self.shm.read_position_log(axis_idx)
        path = f"/tmp/servo_plot_{axis.lower()}.csv"

        with open(path, 'w') as f:
            f.write("time_ms,commanded_mm,actual_mm,error_mm,torque_pct\n")
            for entry in log:
                f.write(
                    f"{entry['time']:.1f},"
                    f"{entry['cmd']:.4f},"
                    f"{entry['actual']:.4f},"
                    f"{entry['error']:.4f},"
                    f"{entry['torque']:.1f}\n")

        gcmd.respond_info(
            f"Position log saved to {path} ({len(log)} samples)\n"
            f"Download via Moonraker: "
            f"http://<printer-ip>/server/files/gcodes/../{path}")


def load_config(config):
    return EtherCATAutoTune(config)
