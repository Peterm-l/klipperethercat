/*
 * trajectory_interp.h — Trajectory Interpolator
 *
 * Reads trajectory segments from the shared memory ring buffer and
 * evaluates position/velocity at the current time for each axis.
 *
 * Each segment describes a constant-acceleration move:
 *   position(t) = start_position + start_velocity * t + 0.5 * accel * t^2
 *   velocity(t) = start_velocity + accel * t
 *   where t = time elapsed since segment start
 */

#ifndef KLIPPER_ETHERCAT_TRAJECTORY_INTERP_H
#define KLIPPER_ETHERCAT_TRAJECTORY_INTERP_H

#include "shared_mem.h"
#include <stdint.h>

/* Per-axis interpolation state */
struct interp_state {
    int      active;                /* 1 if interpolation is active */
    uint32_t current_seg_idx;       /* Ring buffer index of current segment */
    int      has_segment;           /* 1 if we have a valid current segment */
    double   hold_position;         /* Position to hold when no segments */
    double   last_position;         /* Last interpolated position (mm) */
    double   last_velocity;         /* Last interpolated velocity (mm/s) */
    double   last_accel;            /* Last interpolated acceleration (mm/s^2) */
};

/* Interpolator context for all axes */
struct trajectory_interpolator {
    struct interp_state axis[MAX_AXES];
    int                 num_axes;
    int                 motion_active;  /* 1 if any axis has active segments */
};

/*
 * Initialize the trajectory interpolator.
 */
void interp_init(struct trajectory_interpolator *interp, int num_axes);

/*
 * Reset interpolation for an axis (e.g., after homing or emergency stop).
 * Sets the hold position to the current actual position.
 */
void interp_reset(struct trajectory_interpolator *interp, int axis,
                  double current_position);

/*
 * Update trajectory interpolation for all axes.
 *
 * Call this once per RT cycle. It reads new segments from the shared
 * memory ring buffer if available, and evaluates the current position
 * and velocity for each axis at the given timestamp.
 *
 * Parameters:
 *   interp  - interpolator context
 *   shm     - shared memory (for ring buffer access)
 *   now_ns  - current time in nanoseconds (CLOCK_MONOTONIC)
 */
void interp_update(struct trajectory_interpolator *interp,
                   struct klipper_ethercat_shm *shm,
                   uint64_t now_ns);

/*
 * Get the interpolated position for an axis at the current time.
 * Call after interp_update().
 */
double interp_get_position(const struct trajectory_interpolator *interp,
                           int axis);

/*
 * Get the interpolated velocity for an axis at the current time.
 * Call after interp_update().
 */
double interp_get_velocity(const struct trajectory_interpolator *interp,
                           int axis);

/*
 * Get the interpolated acceleration for an axis at the current time.
 * Call after interp_update().
 */
double interp_get_accel(const struct trajectory_interpolator *interp,
                        int axis);

/*
 * Check if any axis has active motion (segments being processed).
 */
static inline int interp_is_moving(const struct trajectory_interpolator *interp)
{
    return interp->motion_active;
}

#endif /* KLIPPER_ETHERCAT_TRAJECTORY_INTERP_H */
