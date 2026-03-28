/*
 * trajectory_interp.c — Trajectory Interpolator Implementation
 *
 * Reads constant-acceleration segments from the shared memory ring buffer
 * and evaluates position/velocity at the current cycle time.
 */

#include "trajectory_interp.h"
#include <string.h>
#include <stdatomic.h>

void interp_init(struct trajectory_interpolator *interp, int num_axes)
{
    memset(interp, 0, sizeof(*interp));
    interp->num_axes = num_axes;
}

void interp_reset(struct trajectory_interpolator *interp, int axis,
                  double current_position)
{
    if (axis < 0 || axis >= interp->num_axes)
        return;

    struct interp_state *is = &interp->axis[axis];
    is->active = 0;
    is->has_segment = 0;
    is->hold_position = current_position;
    is->last_position = current_position;
    is->last_velocity = 0.0;
    is->last_accel = 0.0;
}

/*
 * Try to consume the next segment from the ring buffer.
 * Returns 1 if a new segment was consumed, 0 if buffer is empty.
 */
static int consume_next_segment(struct trajectory_ring_buffer *rb,
                                struct interp_state *is)
{
    uint32_t w = atomic_load_explicit(&rb->write_idx, memory_order_acquire);
    uint32_t r = atomic_load_explicit(&rb->read_idx, memory_order_relaxed);

    if (r == w) {
        /* Ring buffer empty */
        return 0;
    }

    /* Read the segment at the current read index */
    uint32_t idx = r & (MAX_TRAJECTORY_SEGMENTS - 1);
    struct trajectory_segment *seg = &rb->segments[idx];

    /* Check if segment is active */
    if (!(seg->flags & SEGMENT_ACTIVE)) {
        return 0;
    }

    is->current_seg_idx = idx;
    is->has_segment = 1;
    is->active = 1;

    /* Advance read index */
    atomic_store_explicit(&rb->read_idx, r + 1, memory_order_release);

    return 1;
}

void interp_update(struct trajectory_interpolator *interp,
                   struct klipper_ethercat_shm *shm,
                   uint64_t now_ns)
{
    struct trajectory_ring_buffer *rb = &shm->trajectory;
    int any_moving = 0;

    /*
     * Try to consume new segments from the ring buffer.
     * We consume segments that are ready (their start time has arrived).
     */

    for (int a = 0; a < interp->num_axes; a++) {
        struct interp_state *is = &interp->axis[a];

        if (!is->has_segment) {
            /* Try to get the first segment */
            if (!consume_next_segment(rb, is)) {
                /* No segments available — hold current position */
                is->last_velocity = 0.0;
                is->last_accel = 0.0;
                continue;
            }
        }

        /* We have a current segment — interpolate */
        struct trajectory_segment *seg = &rb->segments[is->current_seg_idx];

        /* Calculate time within this segment */
        double t;
        if (now_ns >= seg->timestamp_ns) {
            t = (double)(now_ns - seg->timestamp_ns) / 1e9;
        } else {
            /* Segment hasn't started yet — hold at start position */
            t = 0.0;
        }

        /* Check if we've passed the end of this segment */
        while (t > seg->duration && is->has_segment) {
            /* Check for emergency stop */
            if (seg->flags & SEGMENT_ESTOP) {
                /* Emergency stop — hold at current position */
                is->hold_position = is->last_position;
                is->last_velocity = 0.0;
                is->last_accel = 0.0;
                is->has_segment = 0;
                is->active = 0;
                goto next_axis;
            }

            /* End of this segment — try to advance to next */
            if (seg->flags & SEGMENT_LAST) {
                /* Last segment in sequence — evaluate at end and hold */
                double dur = seg->duration;
                is->hold_position = seg->start_position[a]
                                  + seg->start_velocity[a] * dur
                                  + 0.5 * seg->accel[a] * dur * dur;
                is->last_position = is->hold_position;
                is->last_velocity = 0.0;
                is->last_accel = 0.0;
                is->has_segment = 0;
                is->active = 0;
                goto next_axis;
            }

            /* Try to consume next segment */
            if (!consume_next_segment(rb, is)) {
                /* No more segments — hold at end of last segment */
                double dur = seg->duration;
                is->hold_position = seg->start_position[a]
                                  + seg->start_velocity[a] * dur
                                  + 0.5 * seg->accel[a] * dur * dur;
                is->last_position = is->hold_position;
                is->last_velocity = seg->start_velocity[a]
                                  + seg->accel[a] * dur;
                is->last_accel = 0.0;
                is->has_segment = 0;
                goto next_axis;
            }

            /* Re-evaluate with new segment */
            seg = &rb->segments[is->current_seg_idx];
            if (now_ns >= seg->timestamp_ns) {
                t = (double)(now_ns - seg->timestamp_ns) / 1e9;
            } else {
                t = 0.0;
            }
        }

        /* Evaluate position and velocity within current segment */
        if (is->has_segment) {
            /* Constant-acceleration kinematic equations */
            is->last_position = seg->start_position[a]
                              + seg->start_velocity[a] * t
                              + 0.5 * seg->accel[a] * t * t;
            is->last_velocity = seg->start_velocity[a]
                              + seg->accel[a] * t;
            is->last_accel = seg->accel[a];
            any_moving = 1;
        } else {
            /* Holding position */
            is->last_position = is->hold_position;
            is->last_velocity = 0.0;
            is->last_accel = 0.0;
        }

next_axis:
        (void)0; /* label needs a statement */
    }

    interp->motion_active = any_moving;
}

double interp_get_position(const struct trajectory_interpolator *interp,
                           int axis)
{
    if (axis < 0 || axis >= interp->num_axes)
        return 0.0;
    return interp->axis[axis].last_position;
}

double interp_get_velocity(const struct trajectory_interpolator *interp,
                           int axis)
{
    if (axis < 0 || axis >= interp->num_axes)
        return 0.0;
    return interp->axis[axis].last_velocity;
}

double interp_get_accel(const struct trajectory_interpolator *interp,
                        int axis)
{
    if (axis < 0 || axis >= interp->num_axes)
        return 0.0;
    return interp->axis[axis].last_accel;
}
