/*
 * cia402.h — CiA 402 (DS402) Drive State Machine
 *
 * Implements the standard CiA 402 state machine for servo drive control.
 * Handles state transitions, fault recovery, and mode switching.
 */

#ifndef KLIPPER_ETHERCAT_CIA402_H
#define KLIPPER_ETHERCAT_CIA402_H

#include <stdint.h>
#include "lc10e_pdo.h"

/* State machine states (derived from statusword) */
enum cia402_state {
    CIA402_NOT_READY_TO_SWITCH_ON = 0,
    CIA402_SWITCH_ON_DISABLED,
    CIA402_READY_TO_SWITCH_ON,
    CIA402_SWITCHED_ON,
    CIA402_OPERATION_ENABLED,
    CIA402_QUICK_STOP_ACTIVE,
    CIA402_FAULT_REACTION_ACTIVE,
    CIA402_FAULT,
    CIA402_UNKNOWN,
};

/* Desired target states */
enum cia402_target {
    CIA402_TARGET_DISABLED = 0,     /* Go to SWITCH_ON_DISABLED */
    CIA402_TARGET_ENABLED,          /* Go to OPERATION_ENABLED */
    CIA402_TARGET_QUICK_STOP,       /* Go to QUICK_STOP_ACTIVE */
};

/* Per-axis state machine context */
struct cia402_axis {
    int                  axis_id;
    enum cia402_state    current_state;
    enum cia402_target   target;
    uint16_t             statusword;
    uint16_t             controlword;
    uint16_t             error_code;
    int8_t               modes_of_operation;
    int8_t               modes_display;

    /* Fault handling */
    int                  fault_reset_pending;
    int                  fault_reset_cycles;    /* Counter for fault reset pulse */

    /* State transition timing */
    uint32_t             state_entry_cycle;     /* Cycle when current state was entered */
    uint32_t             transition_timeout;    /* Max cycles to wait for transition */

    /* INIT→PREOP retry handling (LC10E known quirk) */
    int                  init_retry_count;      /* Number of retries attempted */
    int                  init_max_retries;      /* Max retries (default 5) */
    uint32_t             init_stuck_cycles;     /* Cycles stuck in NOT_READY/UNKNOWN */
};

/*
 * Initialize a CiA 402 axis state machine.
 * Call once per axis during setup.
 */
void cia402_init(struct cia402_axis *axis, int axis_id);

/*
 * Decode the current CiA 402 state from the statusword.
 */
enum cia402_state cia402_decode_state(uint16_t statusword);

/*
 * Get human-readable state name.
 */
const char *cia402_state_name(enum cia402_state state);

/*
 * Update the state machine for one cycle.
 *
 * Reads the current statusword, determines the appropriate controlword
 * to send based on the target state, and handles fault recovery.
 *
 * Parameters:
 *   axis       - per-axis state machine context
 *   statusword - current statusword read from TxPDO
 *   error_code - current error code read from TxPDO (0x603F)
 *   cycle      - current cycle counter (for timeouts)
 *
 * Returns:
 *   The controlword to write to RxPDO.
 */
uint16_t cia402_update(struct cia402_axis *axis, uint16_t statusword,
                       uint16_t error_code, uint32_t cycle);

/*
 * Request the drive to transition to OPERATION_ENABLED.
 * The state machine will walk through the required intermediate states.
 */
void cia402_request_enable(struct cia402_axis *axis);

/*
 * Request the drive to transition to SWITCH_ON_DISABLED (disabled).
 */
void cia402_request_disable(struct cia402_axis *axis);

/*
 * Request a quick stop.
 */
void cia402_request_quick_stop(struct cia402_axis *axis);

/*
 * Request fault reset. The state machine will pulse the fault reset bit.
 */
void cia402_request_fault_reset(struct cia402_axis *axis);

/*
 * Check if the drive is in OPERATION_ENABLED state.
 */
static inline int cia402_is_enabled(const struct cia402_axis *axis)
{
    return axis->current_state == CIA402_OPERATION_ENABLED;
}

/*
 * Check if the drive is in a fault state.
 */
static inline int cia402_is_faulted(const struct cia402_axis *axis)
{
    return axis->current_state == CIA402_FAULT ||
           axis->current_state == CIA402_FAULT_REACTION_ACTIVE;
}

#endif /* KLIPPER_ETHERCAT_CIA402_H */
