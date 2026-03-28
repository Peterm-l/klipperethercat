/*
 * cia402.c — CiA 402 (DS402) Drive State Machine Implementation
 */

#include "cia402.h"
#include <string.h>

/* Fault reset pulse width in cycles (at 1kHz = milliseconds) */
#define FAULT_RESET_PULSE_CYCLES    50
/* Timeout for state transitions (cycles) */
#define TRANSITION_TIMEOUT_CYCLES   5000    /* 5 seconds */

/* Cycles to wait in NOT_READY before attempting retry */
#define INIT_STUCK_THRESHOLD    2000    /* 2 seconds */
/* Max retries for INIT→PREOP transition */
#define INIT_MAX_RETRIES        5

void cia402_init(struct cia402_axis *axis, int axis_id)
{
    memset(axis, 0, sizeof(*axis));
    axis->axis_id = axis_id;
    axis->current_state = CIA402_UNKNOWN;
    axis->target = CIA402_TARGET_DISABLED;
    axis->transition_timeout = TRANSITION_TIMEOUT_CYCLES;
    axis->modes_of_operation = MODE_CSV;
    axis->init_max_retries = INIT_MAX_RETRIES;
}

enum cia402_state cia402_decode_state(uint16_t statusword)
{
    /*
     * CiA 402 state decoding from statusword bits:
     *   Bit 6  Bit 5  Bit 3  Bit 2  Bit 1  Bit 0
     *   xxxx   0      0      x      x      0      = Not ready to switch on
     *   x1xx   0      0      x      x      0      = Switch on disabled
     *   x0xx   1      0      0      0      1      = Ready to switch on
     *   x0xx   1      0      0      1      1      = Switched on
     *   x0xx   1      0      1      1      1      = Operation enabled
     *   x0xx   0      0      1      1      1      = Quick stop active
     *   x0xx   x      1      1      1      1      = Fault reaction active
     *   x0xx   x      1      0      0      0      = Fault
     */

    uint16_t masked = statusword & SW_STATE_MASK;

    if ((masked & 0x004F) == 0x0000)
        return CIA402_NOT_READY_TO_SWITCH_ON;
    if ((masked & 0x004F) == 0x0040)
        return CIA402_SWITCH_ON_DISABLED;
    if ((masked & 0x006F) == 0x0021)
        return CIA402_READY_TO_SWITCH_ON;
    if ((masked & 0x006F) == 0x0023)
        return CIA402_SWITCHED_ON;
    if ((masked & 0x006F) == 0x0027)
        return CIA402_OPERATION_ENABLED;
    if ((masked & 0x006F) == 0x0007)
        return CIA402_QUICK_STOP_ACTIVE;
    if ((masked & 0x004F) == 0x000F)
        return CIA402_FAULT_REACTION_ACTIVE;
    if ((masked & 0x004F) == 0x0008)
        return CIA402_FAULT;

    return CIA402_UNKNOWN;
}

const char *cia402_state_name(enum cia402_state state)
{
    switch (state) {
    case CIA402_NOT_READY_TO_SWITCH_ON: return "NOT_READY_TO_SWITCH_ON";
    case CIA402_SWITCH_ON_DISABLED:     return "SWITCH_ON_DISABLED";
    case CIA402_READY_TO_SWITCH_ON:     return "READY_TO_SWITCH_ON";
    case CIA402_SWITCHED_ON:            return "SWITCHED_ON";
    case CIA402_OPERATION_ENABLED:      return "OPERATION_ENABLED";
    case CIA402_QUICK_STOP_ACTIVE:      return "QUICK_STOP_ACTIVE";
    case CIA402_FAULT_REACTION_ACTIVE:  return "FAULT_REACTION_ACTIVE";
    case CIA402_FAULT:                  return "FAULT";
    case CIA402_UNKNOWN:                return "UNKNOWN";
    default:                            return "INVALID";
    }
}

uint16_t cia402_update(struct cia402_axis *axis, uint16_t statusword,
                       uint16_t error_code, uint32_t cycle)
{
    enum cia402_state prev_state = axis->current_state;
    axis->statusword = statusword;
    axis->error_code = error_code;
    axis->current_state = cia402_decode_state(statusword);

    /* Track state entry time */
    if (axis->current_state != prev_state) {
        axis->state_entry_cycle = cycle;
    }

    /* Handle fault reset pulse */
    if (axis->fault_reset_pending) {
        axis->fault_reset_cycles++;
        if (axis->fault_reset_cycles > FAULT_RESET_PULSE_CYCLES) {
            axis->fault_reset_pending = 0;
            axis->fault_reset_cycles = 0;
        }
        /* Send fault reset controlword */
        axis->controlword = CW_FAULT_RESET_CMD;
        return axis->controlword;
    }

    /* State machine: generate controlword based on current state and target */
    switch (axis->current_state) {

    case CIA402_NOT_READY_TO_SWITCH_ON:
        /* Drive is initializing.
         * LC10E known quirk: may get stuck here on first power-on.
         * If stuck for too long, try sending a disable voltage command
         * to nudge it into SWITCH_ON_DISABLED. */
        axis->init_stuck_cycles++;
        if (axis->init_stuck_cycles > INIT_STUCK_THRESHOLD
            && axis->init_retry_count < axis->init_max_retries) {
            /* Attempt to nudge drive forward by toggling controlword */
            axis->controlword = CW_DISABLE_VOLTAGE;
            axis->init_retry_count++;
            axis->init_stuck_cycles = 0;
        } else {
            axis->controlword = 0x0000;
        }
        break;

    case CIA402_SWITCH_ON_DISABLED:
        /* Successfully reached SWITCH_ON_DISABLED — reset retry counters */
        axis->init_stuck_cycles = 0;
        axis->init_retry_count = 0;

        if (axis->target == CIA402_TARGET_ENABLED) {
            /* Transition: SWITCH_ON_DISABLED → READY_TO_SWITCH_ON */
            axis->controlword = CW_SHUTDOWN;    /* 0x0006 */
        } else {
            axis->controlword = CW_DISABLE_VOLTAGE; /* 0x0000 */
        }
        break;

    case CIA402_READY_TO_SWITCH_ON:
        if (axis->target == CIA402_TARGET_ENABLED) {
            /* Transition: READY_TO_SWITCH_ON → SWITCHED_ON */
            axis->controlword = CW_SWITCH_ON_CMD;  /* 0x0007 */
        } else if (axis->target == CIA402_TARGET_DISABLED) {
            axis->controlword = CW_DISABLE_VOLTAGE; /* 0x0000 */
        } else {
            axis->controlword = CW_SHUTDOWN;    /* 0x0006 — stay here */
        }
        break;

    case CIA402_SWITCHED_ON:
        if (axis->target == CIA402_TARGET_ENABLED) {
            /* Transition: SWITCHED_ON → OPERATION_ENABLED */
            axis->controlword = CW_ENABLE_OPERATION_CMD; /* 0x000F */
        } else if (axis->target == CIA402_TARGET_DISABLED) {
            axis->controlword = CW_DISABLE_VOLTAGE; /* 0x0000 */
        } else {
            axis->controlword = CW_SWITCH_ON_CMD; /* 0x0007 — stay here */
        }
        break;

    case CIA402_OPERATION_ENABLED:
        if (axis->target == CIA402_TARGET_ENABLED) {
            /* Stay in OPERATION_ENABLED — normal operation */
            axis->controlword = CW_ENABLE_OPERATION_CMD; /* 0x000F */
        } else if (axis->target == CIA402_TARGET_QUICK_STOP) {
            axis->controlword = CW_QUICK_STOP_CMD; /* 0x0002 */
        } else {
            /* Disable: OPERATION_ENABLED → SWITCHED_ON */
            axis->controlword = CW_DISABLE_OPERATION; /* 0x0007 */
        }
        break;

    case CIA402_QUICK_STOP_ACTIVE:
        if (axis->target == CIA402_TARGET_ENABLED) {
            /* Re-enable after quick stop */
            axis->controlword = CW_ENABLE_OPERATION_CMD; /* 0x000F */
        } else {
            /* Let quick stop complete, then go to SWITCH_ON_DISABLED */
            axis->controlword = CW_DISABLE_VOLTAGE; /* 0x0000 */
        }
        break;

    case CIA402_FAULT_REACTION_ACTIVE:
        /* Drive is handling fault — wait for FAULT state */
        axis->controlword = 0x0000;
        break;

    case CIA402_FAULT:
        /* Drive is in fault. To recover, user must request fault reset. */
        if (axis->target == CIA402_TARGET_ENABLED) {
            /* Auto-attempt fault reset if target is enabled */
            cia402_request_fault_reset(axis);
        }
        axis->controlword = 0x0000;
        break;

    case CIA402_UNKNOWN:
    default:
        /* Unknown state — drive may be initializing or communication lost.
         * LC10E may report unknown statusword during INIT→PREOP transition.
         * Apply same retry logic as NOT_READY. */
        axis->init_stuck_cycles++;
        if (axis->init_stuck_cycles > INIT_STUCK_THRESHOLD
            && axis->init_retry_count < axis->init_max_retries) {
            axis->controlword = CW_DISABLE_VOLTAGE;
            axis->init_retry_count++;
            axis->init_stuck_cycles = 0;
        } else {
            axis->controlword = CW_DISABLE_VOLTAGE;
        }
        break;
    }

    return axis->controlword;
}

void cia402_request_enable(struct cia402_axis *axis)
{
    axis->target = CIA402_TARGET_ENABLED;
}

void cia402_request_disable(struct cia402_axis *axis)
{
    axis->target = CIA402_TARGET_DISABLED;
}

void cia402_request_quick_stop(struct cia402_axis *axis)
{
    axis->target = CIA402_TARGET_QUICK_STOP;
}

void cia402_request_fault_reset(struct cia402_axis *axis)
{
    if (!axis->fault_reset_pending) {
        axis->fault_reset_pending = 1;
        axis->fault_reset_cycles = 0;
    }
}
