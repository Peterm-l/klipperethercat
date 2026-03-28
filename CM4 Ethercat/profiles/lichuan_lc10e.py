# lichuan_lc10e.py — Lichuan LC10E EtherCAT Drive Profile
#
# Drive-specific configuration for the Lichuan LC10E servo drive.
# Used by the Klipper module for drive identification, PDO mapping,
# and default parameters.

LC10E_PROFILE = {
    'name': 'Lichuan LC10E',
    'vendor_id': 0x00000766,        # Lichuan
    'product_code': 0x00000402,
    'revision': 0x00000204,

    # PDO assignment — LC10E uses non-standard indices
    # 0x1703 includes Target Velocity (0x60FF) needed for CSV mode
    'rxpdo_index': 0x1703,
    'txpdo_index': 0x1B02,

    # RxPDO mapping (master -> drive) — 0x1703
    'rxpdo_entries': [
        (0x6040, 0x00, 16),  # Controlword
        (0x607A, 0x00, 32),  # Target Position
        (0x60FF, 0x00, 32),  # Target Velocity (CSV mode)
        (0x60B8, 0x00, 16),  # Touch Probe Function
        (0x6072, 0x00, 16),  # Torque Limit
        (0x6060, 0x00,  8),  # Modes of Operation
    ],

    # TxPDO mapping (drive -> master) — 0x1B02
    'txpdo_entries': [
        (0x603F, 0x00, 16),  # Error Code
        (0x6041, 0x00, 16),  # Statusword
        (0x6064, 0x00, 32),  # Position Actual Value
        (0x606C, 0x00, 32),  # Velocity Actual Value
        (0x6077, 0x00, 16),  # Torque Actual Value
        (0x60F4, 0x00, 32),  # Following Error Actual
        (0x60B9, 0x00, 16),  # Touch Probe Status
        (0x60FD, 0x00, 32),  # Digital Inputs
        (0x6061, 0x00,  8),  # Modes of Operation Display
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

    # Known issues and workarounds
    'notes': [
        "PDO index mismatch: Must use 0x1703/0x1B02, not default 0x1600/0x1A00",
        "CSP mode has poor factory tuning — use CSV mode instead",
        "ESI/SII EEPROM may have incorrect config — load ESI XML explicitly",
        "Drive may not transition cleanly INIT->PREOP on first scan — retry needed",
        "If using CSP fallback, write to velocity feedforward SDO 0x60B1",
    ],
}

# Alternative PDO configurations available
LC10E_ALTERNATE_PDOS = {
    # For CSP mode (if ever needed as fallback)
    'csp': {
        'rxpdo_index': 0x1702,
        'txpdo_index': 0x1B02,
        'rxpdo_entries': [
            (0x6040, 0x00, 16),  # Controlword
            (0x607A, 0x00, 32),  # Target Position
            (0x60B8, 0x00, 16),  # Touch Probe Function
            (0x6071, 0x00, 16),  # Target Torque
            (0x607F, 0x00, 32),  # Max Profile Velocity
            (0x6060, 0x00,  8),  # Modes of Operation
        ],
    },
    # Full mapping with all objects
    'full': {
        'rxpdo_index': 0x1705,
        'txpdo_index': 0x1B04,
    },
}


def get_profile():
    """Return the LC10E drive profile dictionary."""
    return LC10E_PROFILE
