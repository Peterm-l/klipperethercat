/*
 * shared_mem.c — POSIX Shared Memory Implementation
 *
 * Creates and manages the shared memory segment used for communication
 * between klippy (Python) and ethercat_rt (C RT process).
 */

#include "shared_mem.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

struct klipper_ethercat_shm *shm_create(void)
{
    /* Remove any stale shared memory segment */
    shm_unlink(SHM_NAME);

    /* Create new shared memory segment */
    int fd = shm_open(SHM_NAME, O_CREAT | O_RDWR | O_EXCL, 0660);
    if (fd < 0) {
        fprintf(stderr, "shm_create: shm_open(%s) failed: %s\n",
                SHM_NAME, strerror(errno));
        return NULL;
    }

    /* Set size */
    size_t shm_size = sizeof(struct klipper_ethercat_shm);
    if (ftruncate(fd, shm_size) < 0) {
        fprintf(stderr, "shm_create: ftruncate(%zu) failed: %s\n",
                shm_size, strerror(errno));
        close(fd);
        shm_unlink(SHM_NAME);
        return NULL;
    }

    /* Map into process address space */
    struct klipper_ethercat_shm *shm = mmap(
        NULL, shm_size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    close(fd);  /* fd no longer needed after mmap */

    if (shm == MAP_FAILED) {
        fprintf(stderr, "shm_create: mmap failed: %s\n", strerror(errno));
        shm_unlink(SHM_NAME);
        return NULL;
    }

    /* Zero-initialize everything */
    memset(shm, 0, shm_size);

    /* Set header */
    shm->magic = SHM_MAGIC;
    shm->version = SHM_VERSION;

    /* Initialize atomic fields */
    atomic_store(&shm->trajectory.write_idx, 0);
    atomic_store(&shm->trajectory.read_idx, 0);
    atomic_store(&shm->command, CMD_NONE);
    atomic_store(&shm->command_axis, 0);
    atomic_store(&shm->command_ack, 0);

    for (int i = 0; i < MAX_AXES; i++) {
        atomic_store(&shm->position_log[i].recording, 0);
    }

    fprintf(stderr, "shm_create: created %s (%zu bytes)\n",
            SHM_NAME, shm_size);

    return shm;
}

struct klipper_ethercat_shm *shm_open_existing(void)
{
    /* Open existing shared memory segment */
    int fd = shm_open(SHM_NAME, O_RDWR, 0);
    if (fd < 0) {
        fprintf(stderr, "shm_open_existing: shm_open(%s) failed: %s\n",
                SHM_NAME, strerror(errno));
        fprintf(stderr, "  Make sure klippy has started and created the "
                "shared memory segment.\n");
        return NULL;
    }

    /* Map into process address space */
    size_t shm_size = sizeof(struct klipper_ethercat_shm);
    struct klipper_ethercat_shm *shm = mmap(
        NULL, shm_size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    close(fd);

    if (shm == MAP_FAILED) {
        fprintf(stderr, "shm_open_existing: mmap failed: %s\n",
                strerror(errno));
        return NULL;
    }

    /* Validate magic number and version */
    if (shm->magic != SHM_MAGIC) {
        fprintf(stderr, "shm_open_existing: bad magic 0x%08X "
                "(expected 0x%08X)\n", shm->magic, SHM_MAGIC);
        munmap(shm, shm_size);
        return NULL;
    }

    if (shm->version != SHM_VERSION) {
        fprintf(stderr, "shm_open_existing: version mismatch %u "
                "(expected %u)\n", shm->version, SHM_VERSION);
        munmap(shm, shm_size);
        return NULL;
    }

    fprintf(stderr, "shm_open_existing: attached to %s (%zu bytes)\n",
            SHM_NAME, shm_size);

    return shm;
}

void shm_close(struct klipper_ethercat_shm *shm, int is_creator)
{
    if (!shm)
        return;

    size_t shm_size = sizeof(struct klipper_ethercat_shm);
    munmap(shm, shm_size);

    if (is_creator) {
        shm_unlink(SHM_NAME);
        fprintf(stderr, "shm_close: unlinked %s\n", SHM_NAME);
    }
}
