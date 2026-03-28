/*
 * test_cia402.c — Unit tests for CiA 402 state machine
 */

#include <stdio.h>
#include <assert.h>
#include "../src/cia402.h"

static void test_state_decoding(void)
{
    printf("  test_state_decoding... ");

    assert(cia402_decode_state(0x0000) == CIA402_NOT_READY_TO_SWITCH_ON);
    assert(cia402_decode_state(0x0040) == CIA402_SWITCH_ON_DISABLED);
    assert(cia402_decode_state(0x0021) == CIA402_READY_TO_SWITCH_ON);
    assert(cia402_decode_state(0x0023) == CIA402_SWITCHED_ON);
    assert(cia402_decode_state(0x0027) == CIA402_OPERATION_ENABLED);
    assert(cia402_decode_state(0x0007) == CIA402_QUICK_STOP_ACTIVE);
    assert(cia402_decode_state(0x000F) == CIA402_FAULT_REACTION_ACTIVE);
    assert(cia402_decode_state(0x0008) == CIA402_FAULT);

    printf("PASS\n");
}

static void test_enable_sequence(void)
{
    printf("  test_enable_sequence... ");

    struct cia402_axis axis;
    cia402_init(&axis, 0);
    cia402_request_enable(&axis);

    uint16_t cw;

    /* SWITCH_ON_DISABLED → READY_TO_SWITCH_ON */
    cw = cia402_update(&axis, SW_SWITCH_ON_DISABLED_VAL, 0, 1);
    assert(cw == CW_SHUTDOWN);  /* 0x0006 */

    /* READY_TO_SWITCH_ON → SWITCHED_ON */
    cw = cia402_update(&axis, SW_READY_TO_SWITCH_ON_VAL, 0, 2);
    assert(cw == CW_SWITCH_ON_CMD);  /* 0x0007 */

    /* SWITCHED_ON → OPERATION_ENABLED */
    cw = cia402_update(&axis, SW_SWITCHED_ON_VAL, 0, 3);
    assert(cw == CW_ENABLE_OPERATION_CMD);  /* 0x000F */

    /* Stay in OPERATION_ENABLED */
    cw = cia402_update(&axis, SW_OPERATION_ENABLED_VAL, 0, 4);
    assert(cw == CW_ENABLE_OPERATION_CMD);
    assert(cia402_is_enabled(&axis));

    printf("PASS\n");
}

static void test_disable(void)
{
    printf("  test_disable... ");

    struct cia402_axis axis;
    cia402_init(&axis, 0);

    /* Start enabled */
    cia402_request_enable(&axis);
    cia402_update(&axis, SW_OPERATION_ENABLED_VAL, 0, 1);
    assert(cia402_is_enabled(&axis));

    /* Request disable */
    cia402_request_disable(&axis);
    uint16_t cw = cia402_update(&axis, SW_OPERATION_ENABLED_VAL, 0, 2);
    assert(cw == CW_DISABLE_OPERATION);  /* 0x0007 */

    printf("PASS\n");
}

static void test_fault_recovery(void)
{
    printf("  test_fault_recovery... ");

    struct cia402_axis axis;
    cia402_init(&axis, 0);

    /* Put into fault state */
    cia402_update(&axis, SW_FAULT_VAL, 0x1234, 1);
    assert(cia402_is_faulted(&axis));
    assert(axis.error_code == 0x1234);

    /* Request fault reset */
    cia402_request_fault_reset(&axis);
    uint16_t cw = cia402_update(&axis, SW_FAULT_VAL, 0x1234, 2);
    assert(cw == CW_FAULT_RESET_CMD);  /* 0x0080 */

    printf("PASS\n");
}

static void test_state_names(void)
{
    printf("  test_state_names... ");

    assert(cia402_state_name(CIA402_OPERATION_ENABLED) != NULL);
    assert(cia402_state_name(CIA402_FAULT) != NULL);
    assert(cia402_state_name(CIA402_UNKNOWN) != NULL);

    printf("PASS\n");
}

int main(void)
{
    printf("Running CiA 402 state machine tests:\n");
    test_state_decoding();
    test_enable_sequence();
    test_disable();
    test_fault_recovery();
    test_state_names();
    printf("All CiA 402 tests PASSED\n");
    return 0;
}
