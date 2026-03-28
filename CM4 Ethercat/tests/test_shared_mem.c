/*
 * test_shared_mem.c — Unit tests for shared memory interface
 */

#include <stdio.h>
#include <assert.h>
#include <string.h>
#include "../src/shared_mem.h"

static void test_create_and_open(void)
{
    printf("  test_create_and_open... ");

    /* Create */
    struct klipper_ethercat_shm *shm = shm_create();
    assert(shm != NULL);
    assert(shm->magic == SHM_MAGIC);
    assert(shm->version == SHM_VERSION);

    /* Open from "another process" */
    struct klipper_ethercat_shm *shm2 = shm_open_existing();
    assert(shm2 != NULL);
    assert(shm2->magic == SHM_MAGIC);

    /* Verify they point to the same data */
    shm->status.cycle_count = 12345;
    assert(shm2->status.cycle_count == 12345);

    shm_close(shm2, 0);
    shm_close(shm, 1);
    printf("PASS\n");
}

static void test_ring_buffer(void)
{
    printf("  test_ring_buffer... ");

    struct klipper_ethercat_shm *shm = shm_create();
    assert(shm != NULL);

    struct trajectory_ring_buffer *rb = &shm->trajectory;

    /* Initially empty */
    assert(ring_buffer_available(rb) == 0);
    assert(ring_buffer_free(rb) == MAX_TRAJECTORY_SEGMENTS - 1);

    /* Write a segment */
    uint32_t w = atomic_load(&rb->write_idx);
    uint32_t idx = w & (MAX_TRAJECTORY_SEGMENTS - 1);
    rb->segments[idx].timestamp_ns = 1000000;
    rb->segments[idx].duration = 0.001;
    rb->segments[idx].start_position[0] = 10.0;
    rb->segments[idx].start_velocity[0] = 5.0;
    rb->segments[idx].accel[0] = 100.0;
    rb->segments[idx].flags = SEGMENT_ACTIVE;
    atomic_store(&rb->write_idx, w + 1);

    assert(ring_buffer_available(rb) == 1);

    /* Read it back */
    uint32_t r = atomic_load(&rb->read_idx);
    idx = r & (MAX_TRAJECTORY_SEGMENTS - 1);
    assert(rb->segments[idx].start_position[0] == 10.0);
    assert(rb->segments[idx].start_velocity[0] == 5.0);
    assert(rb->segments[idx].accel[0] == 100.0);
    atomic_store(&rb->read_idx, r + 1);

    assert(ring_buffer_available(rb) == 0);

    shm_close(shm, 1);
    printf("PASS\n");
}

static void test_command_interface(void)
{
    printf("  test_command_interface... ");

    struct klipper_ethercat_shm *shm = shm_create();
    assert(shm != NULL);

    /* Send a command */
    atomic_store(&shm->command_axis, 1);
    atomic_store(&shm->command, CMD_ENABLE);

    /* Read command */
    uint8_t cmd = atomic_load(&shm->command);
    uint8_t axis = atomic_load(&shm->command_axis);
    assert(cmd == CMD_ENABLE);
    assert(axis == 1);

    /* Acknowledge */
    atomic_store(&shm->command, CMD_NONE);
    atomic_store(&shm->command_ack, CMD_ENABLE);

    assert(atomic_load(&shm->command) == CMD_NONE);
    assert(atomic_load(&shm->command_ack) == CMD_ENABLE);

    shm_close(shm, 1);
    printf("PASS\n");
}

int main(void)
{
    printf("Running shared memory tests:\n");
    test_create_and_open();
    test_ring_buffer();
    test_command_interface();
    printf("All shared memory tests PASSED\n");
    return 0;
}
