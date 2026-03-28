/*
 * lc10e_pdo.h — Lichuan LC10E EtherCAT Drive PDO Mapping
 *
 * PDO (Process Data Object) mapping for the LC10E servo drive.
 * Based on the official Lichuan ESI XML (V1.04) and LinuxCNC community
 * experience.
 *
 * IMPORTANT: The LC10E uses non-standard PDO indices. The default
 * 0x1600/0x1A00 indices work but have limited object sets. For CSV
 * (Cyclic Synchronous Velocity) mode, use 0x1703 for RxPDO which
 * includes Target Velocity (0x60FF).
 *
 * Vendor ID:    0x00000766 (decimal 1894 = Lichuan)
 * Product Code: 0x00000402
 * Revision:     0x00000204
 */

#ifndef KLIPPER_ETHERCAT_LC10E_PDO_H
#define KLIPPER_ETHERCAT_LC10E_PDO_H

#include <stdint.h>

/* =========================================================================
 * Drive Identification
 * ========================================================================= */
#define LC10E_VENDOR_ID         0x00000766
#define LC10E_PRODUCT_CODE      0x00000402
#define LC10E_REVISION          0x00000204

/* =========================================================================
 * SyncManager Configuration
 * ========================================================================= */
/*
 * SM0: Mailbox Receive (Master → Slave), Start 0x1000, Size 128
 * SM1: Mailbox Send    (Slave → Master), Start 0x1100, Size 128
 * SM2: Process Data Output (RxPDO, Master → Slave), Start 0x1200
 * SM3: Process Data Input  (TxPDO, Slave → Master), Start 0x1300
 */
#define LC10E_SM2_START         0x1200  /* RxPDO (commands to drive) */
#define LC10E_SM3_START         0x1300  /* TxPDO (feedback from drive) */

/* =========================================================================
 * Available RxPDO Mappings (Master → Slave, Commands)
 * =========================================================================
 *
 * 0x1600 — Basic: Controlword, Target Position, Touch Probe, Modes, Target Velocity
 * 0x1701 — CSP focused: Controlword, Target Position, Touch Probe, Target Torque, Modes
 * 0x1702 — CSP + limits: Controlword, Target Position, Touch Probe, Target Torque, Max Profile Vel, Modes
 * 0x1703 — CSV mode: Controlword, Target Position, Target Velocity, Touch Probe, Torque Limits, Modes  ← USE THIS
 * 0x1704 — CSV + offsets: 0x1703 + Torque Offset
 * 0x1705 — Full: 0x1703 + Target Torque + Max Profile Velocity
 */

/* We use RxPDO 0x1703 for CSV mode — it includes Target Velocity (0x60FF) */
#define LC10E_RXPDO_INDEX       0x1703

/* RxPDO 0x1703 object layout and byte offsets within PDO data */
/* Total RxPDO size: 2 + 4 + 4 + 2 + 2 + 1 = 15 bytes (+ padding) */
typedef struct {
    uint16_t controlword;           /* 0x6040:00, 16 bits */
    int32_t  target_position;       /* 0x607A:00, 32 bits (encoder counts) */
    int32_t  target_velocity;       /* 0x60FF:00, 32 bits (encoder counts/s) */
    uint16_t touch_probe_function;  /* 0x60B8:00, 16 bits */
    int16_t  torque_limit;          /* 0x6072:00, 16 bits (0.1% rated) */
    int8_t   modes_of_operation;    /* 0x6060:00, 8 bits */
} __attribute__((packed)) lc10e_rxpdo_t;

/* =========================================================================
 * Available TxPDO Mappings (Slave → Master, Feedback)
 * =========================================================================
 *
 * 0x1A00 — CSP/CSV: Error Code, Statusword, Position, Velocity, Touch Probe Status/Pos,
 *                    Digital Inputs, Modes Display
 * 0x1B01 — Extended: Error Code, Statusword, Position, Torque, Following Error,
 *                     Touch Probe Status/Pos1/Pos2, Digital Inputs
 * 0x1B02 — Standard: Error Code, Statusword, Position, Velocity, Torque, Following Error,
 *                     Touch Probe Status, Digital Inputs, Modes Display
 * 0x1B03 — Minimal+: Error Code, Statusword, Position, Torque, Touch Probe Status/Pos1, Modes Display
 * 0x1B04 — Full: Error Code, Statusword, Position, Velocity, Torque, Following Error,
 *                Touch Probe Status/Pos1/Pos2, Digital Inputs, Modes Display
 */

/* We use TxPDO 0x1B02 — has everything we need including velocity and torque */
#define LC10E_TXPDO_INDEX       0x1B02

/* TxPDO 0x1B02 object layout */
typedef struct {
    uint16_t error_code;            /* 0x603F:00, 16 bits */
    uint16_t statusword;            /* 0x6041:00, 16 bits */
    int32_t  position_actual;       /* 0x6064:00, 32 bits (encoder counts) */
    int32_t  velocity_actual;       /* 0x606C:00, 32 bits (encoder counts/s) */
    int16_t  torque_actual;         /* 0x6077:00, 16 bits (0.1% rated) */
    int32_t  following_error;       /* 0x60F4:00, 32 bits (encoder counts) */
    uint16_t touch_probe_status;    /* 0x60B9:00, 16 bits */
    uint32_t digital_inputs;        /* 0x60FD:00, 32 bits */
    int8_t   modes_display;         /* 0x6061:00, 8 bits */
} __attribute__((packed)) lc10e_txpdo_t;

/* =========================================================================
 * CiA 402 Object Dictionary Indices
 * ========================================================================= */

/* Control objects (SDO/PDO) */
#define OD_CONTROLWORD              0x6040
#define OD_STATUSWORD               0x6041
#define OD_MODES_OF_OPERATION       0x6060
#define OD_MODES_OF_OPERATION_DISP  0x6061

/* Target objects (RxPDO) */
#define OD_TARGET_POSITION          0x607A
#define OD_TARGET_VELOCITY          0x60FF
#define OD_TARGET_TORQUE            0x6071

/* Actual value objects (TxPDO) */
#define OD_POSITION_ACTUAL          0x6064
#define OD_VELOCITY_ACTUAL          0x606C
#define OD_TORQUE_ACTUAL            0x6077
#define OD_FOLLOWING_ERROR_ACTUAL   0x60F4
#define OD_ERROR_CODE               0x603F

/* Motion profile objects (SDO) */
#define OD_MAX_PROFILE_VELOCITY     0x607F
#define OD_PROFILE_VELOCITY         0x6081
#define OD_PROFILE_ACCELERATION     0x6083
#define OD_PROFILE_DECELERATION     0x6084
#define OD_QUICK_STOP_DECELERATION  0x6085
#define OD_TORQUE_LIMIT             0x6072
#define OD_TORQUE_OFFSET            0x60B2

/* Velocity feedforward (useful if CSP mode fallback needed) */
#define OD_VELOCITY_FEEDFORWARD     0x60B1

/* Homing objects */
#define OD_HOMING_METHOD            0x6098
#define OD_HOMING_SPEED_SEARCH      0x6099
#define OD_HOMING_SPEED_ZERO        0x609A
#define OD_HOMING_ACCELERATION      0x609B

/* Touch probe */
#define OD_TOUCH_PROBE_FUNCTION     0x60B8
#define OD_TOUCH_PROBE_STATUS       0x60B9
#define OD_TOUCH_PROBE_POS1         0x60BA
#define OD_TOUCH_PROBE_POS2         0x60BC

/* Digital I/O */
#define OD_DIGITAL_INPUTS           0x60FD
#define OD_DIGITAL_OUTPUTS          0x60FE

/* =========================================================================
 * CiA 402 Modes of Operation
 * ========================================================================= */
#define MODE_PROFILE_POSITION       1
#define MODE_VELOCITY               2
#define MODE_PROFILE_VELOCITY       3
#define MODE_PROFILE_TORQUE         4
#define MODE_HOMING                 6
#define MODE_INTERPOLATED_POSITION  7
#define MODE_CSP                    8   /* Cyclic Synchronous Position */
#define MODE_CSV                    9   /* Cyclic Synchronous Velocity — WE USE THIS */
#define MODE_CST                    10  /* Cyclic Synchronous Torque */

/* =========================================================================
 * Encoder Configuration
 * ========================================================================= */
#define LC10E_ENCODER_23BIT         8388608     /* 2^23 counts/revolution */
#define LC10E_ENCODER_17BIT         131072      /* 2^17 counts/revolution */

/* =========================================================================
 * CiA 402 Controlword Bit Definitions
 * ========================================================================= */
#define CW_SWITCH_ON                (1 << 0)    /* Bit 0 */
#define CW_ENABLE_VOLTAGE           (1 << 1)    /* Bit 1 */
#define CW_QUICK_STOP               (1 << 2)    /* Bit 2 (active low) */
#define CW_ENABLE_OPERATION         (1 << 3)    /* Bit 3 */
#define CW_FAULT_RESET              (1 << 7)    /* Bit 7 */
#define CW_HALT                     (1 << 8)    /* Bit 8 */

/* Controlword commands (standard CiA 402 state transitions) */
#define CW_SHUTDOWN                 0x0006      /* → READY_TO_SWITCH_ON */
#define CW_SWITCH_ON_CMD            0x0007      /* → SWITCHED_ON */
#define CW_ENABLE_OPERATION_CMD     0x000F      /* → OPERATION_ENABLED */
#define CW_DISABLE_VOLTAGE          0x0000      /* → SWITCH_ON_DISABLED */
#define CW_QUICK_STOP_CMD           0x0002      /* → QUICK_STOP_ACTIVE */
#define CW_DISABLE_OPERATION        0x0007      /* → SWITCHED_ON */
#define CW_FAULT_RESET_CMD          0x0080      /* FAULT → SWITCH_ON_DISABLED */

/* =========================================================================
 * CiA 402 Statusword Bit Definitions
 * ========================================================================= */
#define SW_READY_TO_SWITCH_ON       (1 << 0)    /* Bit 0 */
#define SW_SWITCHED_ON              (1 << 1)    /* Bit 1 */
#define SW_OPERATION_ENABLED        (1 << 2)    /* Bit 2 */
#define SW_FAULT                    (1 << 3)    /* Bit 3 */
#define SW_VOLTAGE_ENABLED          (1 << 4)    /* Bit 4 */
#define SW_QUICK_STOP               (1 << 5)    /* Bit 5 (active low) */
#define SW_SWITCH_ON_DISABLED       (1 << 6)    /* Bit 6 */
#define SW_WARNING                  (1 << 7)    /* Bit 7 */
#define SW_REMOTE                   (1 << 9)    /* Bit 9 */
#define SW_TARGET_REACHED           (1 << 10)   /* Bit 10 */
#define SW_INTERNAL_LIMIT           (1 << 11)   /* Bit 11 */

/* Statusword state mask (bits 0-3, 5-6) */
#define SW_STATE_MASK               0x006F

/* Statusword state values */
#define SW_NOT_READY_TO_SWITCH_ON   0x0000
#define SW_SWITCH_ON_DISABLED_VAL   0x0040
#define SW_READY_TO_SWITCH_ON_VAL   0x0021
#define SW_SWITCHED_ON_VAL          0x0023
#define SW_OPERATION_ENABLED_VAL    0x0027
#define SW_QUICK_STOP_ACTIVE_VAL    0x0007
#define SW_FAULT_REACTION_VAL       0x000F
#define SW_FAULT_VAL                0x0008

/* =========================================================================
 * Safety Defaults
 * ========================================================================= */
#define LC10E_MAX_FOLLOWING_ERROR_DEFAULT    5.0     /* mm */
#define LC10E_WATCHDOG_TIMEOUT_MS           100
#define LC10E_DEFAULT_MAX_VELOCITY          500.0   /* mm/s */
#define LC10E_DEFAULT_MAX_ACCEL             30000.0 /* mm/s^2 */

/* Default PID (conservative starting point) */
#define LC10E_DEFAULT_KP                    50.0
#define LC10E_DEFAULT_KI                    5.0
#define LC10E_DEFAULT_KD                    0.5
#define LC10E_DEFAULT_FF_VELOCITY           1.0
#define LC10E_DEFAULT_FF_ACCELERATION       0.0

/* =========================================================================
 * Conversion Helpers
 * ========================================================================= */

/*
 * Convert encoder counts to millimeters.
 *   position_mm = raw_counts / encoder_resolution * rotation_distance
 */
static inline double counts_to_mm(int32_t counts, int32_t encoder_res,
                                   double rotation_distance)
{
    return (double)counts / (double)encoder_res * rotation_distance;
}

/*
 * Convert millimeters to encoder counts.
 */
static inline int32_t mm_to_counts(double mm, int32_t encoder_res,
                                    double rotation_distance)
{
    return (int32_t)(mm / rotation_distance * (double)encoder_res);
}

/*
 * Convert velocity in mm/s to encoder counts/s.
 */
static inline int32_t mmps_to_counts(double mm_per_s, int32_t encoder_res,
                                      double rotation_distance)
{
    return (int32_t)(mm_per_s / rotation_distance * (double)encoder_res);
}

/*
 * Convert encoder counts/s to mm/s.
 */
static inline double counts_to_mmps(int32_t counts_per_s, int32_t encoder_res,
                                     double rotation_distance)
{
    return (double)counts_per_s / (double)encoder_res * rotation_distance;
}

/*
 * Convert torque from drive units (0.1% of rated) to percentage.
 */
static inline double torque_to_percent(int16_t torque_raw)
{
    return (double)torque_raw * 0.1;
}

#endif /* KLIPPER_ETHERCAT_LC10E_PDO_H */
