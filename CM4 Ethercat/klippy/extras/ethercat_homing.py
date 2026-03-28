# ethercat_homing.py — Homing for EtherCAT Servo Axes
#
# Supports two homing methods:
#   1. absolute_encoder: No motion needed. Read absolute position from
#      the drive's encoder and set the coordinate offset.
#   2. torque_limit: Slowly move until hitting a hard stop (torque spike),
#      then set that as the home position.
#
# With the LC10E's 23-bit absolute encoder, method 1 is preferred.

import logging

logger = logging.getLogger(__name__)


class EtherCATHoming:
    """Homing support for EtherCAT servo axes."""

    def __init__(self, config, ethercat_servo):
        self.printer = config.get_printer()
        self.ethercat = ethercat_servo
        self.shm = None

        self.printer.register_event_handler(
            "klippy:connect", self._handle_connect)

        gcode = self.printer.lookup_object('gcode')
        gcode.register_command(
            'SERVO_HOME', self.cmd_SERVO_HOME,
            desc="Home an EtherCAT servo axis")

    def _handle_connect(self):
        self.shm = self.ethercat.get_shm()

    def cmd_SERVO_HOME(self, gcmd):
        """Home an EtherCAT servo axis.

        Usage:
            SERVO_HOME AXIS=X                     # absolute encoder (default)
            SERVO_HOME AXIS=X METHOD=absolute      # explicit absolute encoder
            SERVO_HOME AXIS=X METHOD=torque_limit  # hit hard stop
        """
        axis = gcmd.get('AXIS').upper()
        method = gcmd.get('METHOD', 'absolute').lower()

        idx = self.ethercat._axis_name_to_index(axis)
        if idx is None:
            raise gcmd.error(f"Unknown axis: {axis}")

        if not self.shm or not self.shm._shm:
            raise gcmd.error("EtherCAT not connected")

        state = self.shm.read_servo_state(idx)

        if method == 'absolute' or method == 'absolute_encoder':
            self._home_absolute_encoder(gcmd, axis, idx, state)
        elif method == 'torque_limit':
            self._home_torque_limit(gcmd, axis, idx, state)
        else:
            raise gcmd.error(
                f"Unknown homing method: {method}. "
                f"Use 'absolute' or 'torque_limit'.")

    def _home_absolute_encoder(self, gcmd, axis, idx, state):
        """Home using absolute encoder — no motion needed.

        The absolute encoder knows its position at power-on.
        We just read the current position and set it as the
        coordinate system reference.
        """
        current_pos = state['actual_position']

        # Send HOME command to RT process
        # This tells it to reset the trajectory interpolator
        # to the current position
        self.shm.send_command('HOME', idx)

        gcmd.respond_info(
            f"Axis {axis} homed via absolute encoder at "
            f"position {current_pos:.4f}mm")

        # Update the stepper's internal position
        for stepper in self.ethercat.ethercat_steppers:
            if stepper.axis_index == idx:
                stepper.set_position(current_pos)
                break

    def _home_torque_limit(self, gcmd, axis, idx, state):
        """Home by moving slowly until hitting a hard stop.

        The axis moves in the negative direction at low speed.
        When the torque exceeds a threshold (indicating a mechanical
        hard stop), the position is recorded as home.
        """
        # Check drive is enabled
        if state['state'] != 'OPERATION_ENABLED':
            raise gcmd.error(
                f"Axis {axis} drive not enabled. "
                f"Run SERVO_ENABLE AXIS={axis} first.")

        gcmd.respond_info(
            f"Axis {axis} homing via torque limit... "
            f"(axis will move slowly)")

        # Use the CiA 402 homing mode via SDO
        # This tells the drive to handle homing internally
        self.shm.send_command('HOME', idx)

        # Poll until homing complete
        timeout = 30.0
        start_time = self.printer.get_reactor().monotonic()
        while True:
            current_time = self.printer.get_reactor().monotonic()
            if current_time - start_time > timeout:
                raise gcmd.error(
                    f"Axis {axis} homing timed out after {timeout}s")

            state = self.shm.read_servo_state(idx)
            if state['homing_complete']:
                break

            self.printer.get_reactor().pause(
                self.printer.get_reactor().monotonic() + 0.1)

        final_pos = state['actual_position']
        gcmd.respond_info(
            f"Axis {axis} homed at position {final_pos:.4f}mm")

        for stepper in self.ethercat.ethercat_steppers:
            if stepper.axis_index == idx:
                stepper.set_position(final_pos)
                break


def load_config(config):
    ethercat = config.get_printer().lookup_object('ethercat')
    return EtherCATHoming(config, ethercat)
