/*
 * Moonpool Subsea Robotics Core
 * C17 High-Performance Fastpath Execution Engine Implementation
 * ISO C17 Standard (-std=c17)
 */

#include "fastpath.h"
#include <math.h>
#include <string.h>

#define EPSILON_FP 1.0e-9

int fastpath_init(void) {
    return 1;
}

void fastpath_mat6x6_mul(const FastpathMat6x6 *a, const FastpathMat6x6 *b, FastpathMat6x6 *out) {
    for (int r = 0; r < 6; ++r) {
        for (int c = 0; c < 6; ++c) {
            double sum = 0.0;
            for (int k = 0; k < 6; ++k) {
                sum += a->data[r * 6 + k] * b->data[k * 6 + c];
            }
            out->data[r * 6 + c] = sum;
        }
    }
}

void fastpath_mat6x6_vec_mul(const FastpathMat6x6 *a, const FastpathVec6 *x, FastpathVec6 *out) {
    for (int r = 0; r < 6; ++r) {
        double sum = 0.0;
        for (int c = 0; c < 6; ++c) {
            sum += a->data[r * 6 + c] * x->data[c];
        }
        out->data[r] = sum;
    }
}

int fastpath_trace_layer_step(
    double p,
    double z_in,
    double z_out,
    double c_in,
    double c_out,
    double *out_dx,
    double *out_dt,
    double *out_ds
) {
    double dz = z_out - z_in;
    if (fabs(dz) < EPSILON_FP) {
        *out_dx = 0.0;
        *out_dt = 0.0;
        *out_ds = 0.0;
        return 1;
    }

    double g = (c_out - c_in) / dz;

    /* Iso-velocity layer handling */
    if (fabs(g) < 1.0e-6) {
        double cos_theta = p * c_in;
        if (cos_theta > 1.0) cos_theta = 1.0;
        if (cos_theta < -1.0) cos_theta = -1.0;
        double sin_theta = sqrt(fabs(1.0 - cos_theta * cos_theta));
        if (sin_theta < EPSILON_FP) sin_theta = EPSILON_FP;

        double dx = fabs(dz) * cos_theta / sin_theta;
        double ds = sqrt(dx * dx + dz * dz);
        double dt = ds / c_in;

        *out_dx = dx;
        *out_dt = dt;
        *out_ds = ds;
        return 1;
    }

    double cos_in = p * c_in;
    if (cos_in > 1.0) cos_in = 1.0;
    if (cos_in < -1.0) cos_in = -1.0;
    double sin_in = sqrt(fabs(1.0 - cos_in * cos_in));

    double c_turn = 1.0 / p;
    int reached_boundary = 1;
    double actual_c_out = c_out;

    if ((g > 0.0 && c_out > c_turn && c_turn > c_in) ||
        (g < 0.0 && c_out < c_turn && c_turn < c_in)) {
        reached_boundary = 0;
        actual_c_out = c_turn;
    }

    double cos_out = p * actual_c_out;
    if (cos_out > 1.0) cos_out = 1.0;
    if (cos_out < -1.0) cos_out = -1.0;
    double sin_out = sqrt(fabs(1.0 - cos_out * cos_out));

    double dx = fabs((sin_in - sin_out) / (g * p));
    double ratio_c = actual_c_out / c_in;
    double num_t = 1.0 + sin_in;
    double den_t = 1.0 + sin_out;
    if (den_t < EPSILON_FP) den_t = EPSILON_FP;

    double dt = (1.0 / fabs(g)) * fabs(log(ratio_c * (num_t / den_t)));
    double ds = sqrt(dx * dx + (actual_c_out - c_in) * (actual_c_out - c_in) / (g * g));

    *out_dx = dx;
    *out_dt = dt;
    *out_ds = ds;

    return reached_boundary;
}

int fastpath_ring_push(FastpathRingBuffer *rb, uint64_t ts, const double *values, size_t count) {
    size_t next_head = (rb->head + 1) % FASTPATH_RING_BUFFER_CAPACITY;
    if (next_head == rb->tail) {
        return 0; /* Ring buffer full */
    }

    FastpathTelemetrySample *sample = &rb->buffer[rb->head];
    sample->timestamp_ns = ts;
    size_t copy_n = count > 8 ? 8 : count;
    for (size_t i = 0; i < copy_n; ++i) {
        sample->values[i] = values[i];
    }
    for (size_t i = copy_n; i < 8; ++i) {
        sample->values[i] = 0.0;
    }

    rb->head = next_head;
    return 1;
}

int fastpath_ring_pop(FastpathRingBuffer *rb, FastpathTelemetrySample *out_sample) {
    if (rb->head == rb->tail) {
        return 0; /* Buffer empty */
    }

    *out_sample = rb->buffer[rb->tail];
    rb->tail = (rb->tail + 1) % FASTPATH_RING_BUFFER_CAPACITY;
    return 1;
}

size_t fastpath_ring_count(const FastpathRingBuffer *rb) {
    if (rb->head >= rb->tail) {
        return rb->head - rb->tail;
    }
    return FASTPATH_RING_BUFFER_CAPACITY - (rb->tail - rb->head);
}
