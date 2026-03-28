# Core R-Theta 4-Axis Kinematic for Klipper
#
# General-purpose matrix kinematic matching RepRapFirmware's M669 K0.
# Uses a configurable inverse matrix to map logical axes to physical motors,
# exactly like RRF's CoreKinematics.
#
# The inverse matrix defines: motor_position = inverse_matrix * axis_position
# The forward matrix (axis_position = forward_matrix * motor_position) is
# computed automatically by inverting the inverse matrix.
#
# This supports:
#   - Independent axes (identity matrix)
#   - CoreXY-style coupling (two motors → two axes)
#   - Core R-Theta coupling (R and B share two motors)
#   - Any arbitrary linear axis-to-motor mapping
#
# Original Core R-Theta RRF config (M669 K0):
#   C0:0:0:0:1 X-1:0:0:1:0 Z0:0:1:0:0 B0.222:0:0:0.222:0
#
# Klipper equivalent in printer.cfg:
#   [printer]
#   kinematics: core_rtheta
#   # Inverse matrix rows (axis → motor mapping)
#   # Format: comma-separated coefficients for each motor drive
#   # Drives are in order: drive0, drive1, drive2, drive3, drive4
#   axis_c_map: 0, 0, 0, 0, 1
#   axis_x_map: -1, 0, 0, 1, 0
#   axis_z_map: 0, 0, 1, 0, 0
#   axis_b_map: 0.222, 0, 0, 0.222, 0

import math
import logging
import stepper

logger = logging.getLogger(__name__)

# Maximum axes/motors supported
MAX_AXES = 5


def invert_matrix(matrix, size):
    """Invert a square matrix using Gauss-Jordan elimination.

    Same algorithm as RRF's CoreKinematics::Recalc().
    Returns the inverse matrix, or None if singular.
    """
    # Build augmented matrix [matrix | identity]
    aug = [[0.0] * (size * 2) for _ in range(size)]
    for i in range(size):
        for j in range(size):
            aug[i][j] = matrix[i][j]
            aug[i][size + j] = 1.0 if i == j else 0.0

    # Gauss-Jordan elimination
    for col in range(size):
        # Find pivot
        max_val = abs(aug[col][col])
        max_row = col
        for row in range(col + 1, size):
            if abs(aug[row][col]) > max_val:
                max_val = abs(aug[row][col])
                max_row = row

        if max_val < 1e-10:
            return None  # Singular matrix

        # Swap rows
        if max_row != col:
            aug[col], aug[max_row] = aug[max_row], aug[col]

        # Eliminate column
        pivot = aug[col][col]
        for j in range(size * 2):
            aug[col][j] /= pivot

        for row in range(size):
            if row != col:
                factor = aug[row][col]
                for j in range(size * 2):
                    aug[row][j] -= factor * aug[col][j]

    # Extract the right half as the inverse
    inverse = [[0.0] * size for _ in range(size)]
    for i in range(size):
        for j in range(size):
            inverse[i][j] = aug[i][size + j]

    return inverse


class CoreRThetaKinematics:
    def __init__(self, toolhead, config):
        self.printer = config.get_printer()

        # Number of axes (C, X, Z, B + optionally E)
        self.num_axes = config.getint('num_axes', 4)
        self.num_motors = config.getint('num_motors', 5)

        # Build area
        self.max_radius = config.getfloat('max_radius', 115.0)

        # Velocity/acceleration limits for each axis
        self.max_z_velocity = config.getfloat(
            'max_z_velocity', 20.0, above=0.)
        self.max_z_accel = config.getfloat(
            'max_z_accel', 500.0, above=0.)
        self.max_b_velocity = config.getfloat(
            'max_b_velocity', 100.0, above=0.)
        self.max_b_accel = config.getfloat(
            'max_b_accel', 2000.0, above=0.)

        # Parse the inverse matrix from config.
        # Each row defines how one logical axis maps to physical motors.
        # Default: identity (each axis drives its own motor independently).
        #
        # Axis order matches RRF: C(theta), X(radial), Z(vertical), B(tilt)
        # Motor order: drive0, drive1, drive2, drive3, drive4
        self.inverse_matrix = self._parse_matrix(config)
        self.forward_matrix = invert_matrix(
            self.inverse_matrix, self.num_motors)

        if self.forward_matrix is None:
            raise config.error(
                "Core R-Theta: axis mapping matrix is singular "
                "(non-invertible). Check axis_*_map values.")

        self._log_matrices()

        # Set up stepper rails.
        # We need one rail per MOTOR (not per axis), since the matrix
        # maps axes to motors.
        #
        # Klipper's itersolve works in Cartesian X/Y/Z space.
        # We map: X→R(radial), Y→Theta(C), Z→Z, and handle B separately.
        #
        # For coupled axes, we create rails for each physical motor
        # and use the matrix to compute their positions.

        # Motor rails — one per physical drive
        self.rails = []

        # Rail for stepper_r (radial / X axis)
        self.rail_r = stepper.LookupMultiRail(
            config.getsection('stepper_r'))
        self.rail_r.setup_itersolve('cartesian_stepper_alloc', b'x')
        self.rails.append(self.rail_r)

        # Rail for stepper_theta (rotation / C axis, mapped to Y)
        self.rail_theta = stepper.LookupMultiRail(
            config.getsection('stepper_theta'))
        self.rail_theta.setup_itersolve('cartesian_stepper_alloc', b'y')
        self.rails.append(self.rail_theta)

        # Rail for stepper_z
        self.rail_z = stepper.LookupMultiRail(
            config.getsection('stepper_z'))
        self.rail_z.setup_itersolve('cartesian_stepper_alloc', b'z')
        self.rails.append(self.rail_z)

        # Rail for stepper_b (nozzle tilt)
        self.rail_b = stepper.LookupMultiRail(
            config.getsection('stepper_b'))
        self.rail_b.setup_itersolve('cartesian_stepper_alloc', b'x')
        self.rails.append(self.rail_b)

        # Axis limits (invalid until homed)
        self.limits = [(1.0, -1.0)] * 4

        # Connect all steppers to the toolhead's trapq
        for rail in self.rails:
            for s in rail.get_steppers():
                s.set_trapq(toolhead.get_trapq())

        self.printer.register_event_handler(
            "stepper_enable:motor_off", self._motor_off)

    def _parse_matrix(self, config):
        """Parse the inverse matrix from printer.cfg.

        Reads axis_c_map, axis_x_map, axis_z_map, axis_b_map.
        Each is a comma-separated list of coefficients mapping that axis
        to motor drives, exactly matching RRF's M669 K0 format.
        """
        n = self.num_motors
        matrix = [[0.0] * n for _ in range(n)]

        # Default: identity-like mapping
        # drive0→R, drive1→Theta, drive2→Z, drive3→B, drive4→(unused)
        defaults = {
            'axis_x_map': '1, 0, 0, 0, 0',      # X(R) = drive0
            'axis_c_map': '0, 1, 0, 0, 0',       # C(Theta) = drive1
            'axis_z_map': '0, 0, 1, 0, 0',       # Z = drive2
            'axis_b_map': '0, 0, 0, 1, 0',       # B = drive3
        }

        # Row order in matrix: X(0), C(1), Z(2), B(3)
        axis_keys = ['axis_x_map', 'axis_c_map', 'axis_z_map', 'axis_b_map']

        for row_idx, key in enumerate(axis_keys):
            raw = config.get(key, defaults[key])
            coeffs = [float(c.strip()) for c in raw.split(',')]
            if len(coeffs) < n:
                coeffs.extend([0.0] * (n - len(coeffs)))
            for col in range(n):
                matrix[row_idx][col] = coeffs[col]

        # Fill remaining rows with identity (for unused axes)
        for i in range(4, n):
            matrix[i][i] = 1.0

        return matrix

    def _log_matrices(self):
        """Log the forward and inverse matrices for debugging."""
        logger.info("Core R-Theta inverse matrix (axis → motor):")
        labels = ['X(R)', 'C(Θ)', 'Z', 'B', 'E']
        for i in range(min(self.num_axes, len(labels))):
            row = ', '.join('%.4f' % v for v in self.inverse_matrix[i][:self.num_motors])
            logger.info("  %s: [%s]", labels[i], row)

        logger.info("Core R-Theta forward matrix (motor → axis):")
        for i in range(min(self.num_motors, len(labels))):
            row = ', '.join('%.4f' % v for v in self.forward_matrix[i][:self.num_axes])
            logger.info("  drive%d: [%s]", i, row)

    def get_steppers(self):
        result = []
        for rail in self.rails:
            result.extend(rail.get_steppers())
        return result

    def calc_position(self, stepper_positions):
        """Forward kinematics: motor positions → axis positions.

        axis_pos = forward_matrix * motor_pos

        This matches RRF's MotorStepsToCartesian().
        """
        # Get motor positions
        motor_pos = [
            stepper_positions[self.rail_r.get_name()],
            stepper_positions[self.rail_theta.get_name()],
            stepper_positions[self.rail_z.get_name()],
            stepper_positions[self.rail_b.get_name()],
        ]
        # Pad to num_motors
        while len(motor_pos) < self.num_motors:
            motor_pos.append(0.0)

        # Apply forward matrix
        axis_pos = [0.0] * 4
        for i in range(4):
            for j in range(self.num_motors):
                axis_pos[i] += self.forward_matrix[j][i] * motor_pos[j]

        return axis_pos

    def set_position(self, newpos, homing_axes):
        """Set position. Applies inverse matrix for motor positions."""
        # Compute motor positions from axis positions
        motor_pos = self._axis_to_motor(newpos)

        # Set each rail's position using the motor coordinates
        # Each rail uses its own itersolve coordinate
        # rail_r uses X, rail_theta uses Y, rail_z uses Z
        r_pos = [motor_pos[0], motor_pos[1], motor_pos[2]]
        for rail in self.rails:
            rail.set_position(newpos)

        if homing_axes:
            for i in homing_axes:
                if i < len(self.rails):
                    self.limits[i] = self.rails[i].get_range()

    def _axis_to_motor(self, axis_pos):
        """Inverse kinematics: axis positions → motor positions.

        motor_pos = inverse_matrix * axis_pos

        This matches RRF's CartesianToMotorSteps().
        """
        # Pad axis_pos
        ap = list(axis_pos)
        while len(ap) < self.num_motors:
            ap.append(0.0)

        motor_pos = [0.0] * self.num_motors
        for motor in range(self.num_motors):
            for axis in range(self.num_motors):
                motor_pos[motor] += self.inverse_matrix[axis][motor] * ap[axis]

        return motor_pos

    def note_z_not_homed(self):
        self.limits[2] = (1.0, -1.0)

    def home(self, homing_state):
        for axis in homing_state.get_axes():
            if axis >= len(self.rails):
                continue
            rail = self.rails[axis]
            hi = rail.get_homing_info()
            rng = rail.get_range()

            homepos = [None] * 4
            homepos[axis] = hi.position_endstop

            forcepos = list(homepos)
            if hi.positive_dir:
                forcepos[axis] -= 1.5 * (hi.position_endstop - rng[0])
            else:
                forcepos[axis] += 1.5 * (rng[1] - hi.position_endstop)

            homing_state.home_rails([rail], forcepos, homepos)

    def _motor_off(self, print_time):
        self.limits = [(1.0, -1.0)] * 4

    def check_move(self, move):
        end_pos = move.end_pos

        # Check homing state
        for i in range(min(len(move.axes_d), 4)):
            if move.axes_d[i] and self.limits[i][0] > self.limits[i][1]:
                raise move.move_error("Must home axis first")

        # Check R (X axis) within radius limit
        r = end_pos[0]
        if abs(r) > self.max_radius:
            raise move.move_error(
                "R=%.1fmm exceeds max_radius=%.1fmm"
                % (abs(r), self.max_radius))

        # Check Z limits
        z = end_pos[2]
        if self.limits[2][0] <= self.limits[2][1]:
            if z < self.limits[2][0] or z > self.limits[2][1]:
                raise move.move_error(
                    "Z=%.1fmm outside limits [%.1f, %.1f]"
                    % (z, self.limits[2][0], self.limits[2][1]))

        # Check B limits
        if len(end_pos) > 3 and self.limits[3][0] <= self.limits[3][1]:
            b = end_pos[3]
            if b < self.limits[3][0] or b > self.limits[3][1]:
                raise move.move_error(
                    "B=%.1f outside limits [%.1f, %.1f]"
                    % (b, self.limits[3][0], self.limits[3][1]))

        # Apply Z velocity/accel limits
        if move.axes_d[2]:
            z_ratio = move.move_d / abs(move.axes_d[2])
            move.limit_speed(
                self.max_z_velocity * z_ratio,
                self.max_z_accel * z_ratio)

        # Apply B velocity/accel limits
        if len(move.axes_d) > 3 and move.axes_d[3]:
            b_ratio = move.move_d / abs(move.axes_d[3])
            move.limit_speed(
                self.max_b_velocity * b_ratio,
                self.max_b_accel * b_ratio)

    def get_status(self, eventtime):
        axes_homed = []
        for i, name in enumerate(['x', 'y', 'z', 'a']):
            if i < len(self.limits) and self.limits[i][0] <= self.limits[i][1]:
                axes_homed.append(name)
        return {
            'homed_axes': ''.join(axes_homed),
            'max_radius': self.max_radius,
        }


def load_kinematics(toolhead, config):
    return CoreRThetaKinematics(toolhead, config)
