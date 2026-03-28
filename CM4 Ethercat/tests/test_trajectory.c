/*
 * test_trajectory.c — Unit tests for trajectory interpolator
 */

#include <stdio.h>
#include <assert.h>
#include <math.h>
#include <string.h>
#include "../src/trajectory_interp.h"
#include "../src/shared_mem.h"

static void test_hold_position(void)
{
    printf("  test_hold_position... ");

    struct trajectory_interpolator interp;
    interp_init(&interp, 3);
    interp_reset(&interp, 0, 50.0);

    /* Create a minimal shm with empty ring buffer */
    struct klipper_ethercat_shm shm;
    memset(&shm, 0, sizeof(shm));

    interp_update(&interp, &shm, 1000000000ULL);

    assert(fabs(interp_get_position(&interp, 0) - 50.0) < 0.001);
    assert(fabs(interp_get_velocity(&interp, 0)) < 0.001);
    assert(!interp_is_moving(&interp));

    printf("PASS\n");
}

static void test_constant_velocity(void)
{
    printf("  test_constant_velocity... ");

    struct trajectory_interpolator interp;
    interp_init(&interp, 1);
    interp_reset(&interp, 0, 0.0);

    struct klipper_ethercat_shm shm;
    memset(&shm, 0, sizeof(shm));

    /* Write a constant-velocity segment: v=100mm/s, duration=1s */
    uint32_t idx = 0;
    shm.trajectory.segments[idx].timestamp_ns = 1000000000ULL; /* t=1s */
    shm.trajectory.segments[idx].duration = 1.0;
    shm.trajectory.segments[idx].start_position[0] = 0.0;
    shm.trajectory.segments[idx].start_velocity[0] = 100.0;
    shm.trajectory.segments[idx].accel[0] = 0.0;
    shm.trajectory.segments[idx].flags = SEGMENT_ACTIVE | SEGMENT_LAST;
    atomic_store(&shm.trajectory.write_idx, 1);

    /* Evaluate at t=1.5s (0.5s into segment) */
    interp_update(&interp, &shm, 1500000000ULL);

    double pos = interp_get_position(&interp, 0);
    double vel = interp_get_velocity(&interp, 0);

    /* pos = 0 + 100*0.5 + 0 = 50mm */
    assert(fabs(pos - 50.0) < 0.1);
    assert(fabs(vel - 100.0) < 0.1);

    printf("PASS\n");
}

static void test_acceleration(void)
{
    printf("  test_acceleration... ");

    struct trajectory_interpolator interp;
    interp_init(&interp, 1);
    interp_reset(&interp, 0, 0.0);

    struct klipper_ethercat_shm shm;
    memset(&shm, 0, sizeof(shm));

    /* Write an accelerating segment: v0=0, a=200mm/s^2, duration=1s */
    uint32_t idx = 0;
    shm.trajectory.segments[idx].timestamp_ns = 0;
    shm.trajectory.segments[idx].duration = 1.0;
    shm.trajectory.segments[idx].start_position[0] = 10.0;
    shm.trajectory.segments[idx].start_velocity[0] = 0.0;
    shm.trajectory.segments[idx].accel[0] = 200.0;
    shm.trajectory.segments[idx].flags = SEGMENT_ACTIVE | SEGMENT_LAST;
    atomic_store(&shm.trajectory.write_idx, 1);

    /* Evaluate at t=0.5s */
    interp_update(&interp, &shm, 500000000ULL);

    double pos = interp_get_position(&interp, 0);
    double vel = interp_get_velocity(&interp, 0);

    /* pos = 10 + 0*0.5 + 0.5*200*0.25 = 10 + 25 = 35mm */
    assert(fabs(pos - 35.0) < 0.1);
    /* vel = 0 + 200*0.5 = 100mm/s */
    assert(fabs(vel - 100.0) < 0.1);

    printf("PASS\n");
}

int main(void)
{
    printf("Running trajectory interpolator tests:\n");
    test_hold_position();
    test_constant_velocity();
    test_acceleration();
    printf("All trajectory tests PASSED\n");
    return 0;
}
