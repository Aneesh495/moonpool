/*
 * Moonpool Subsea Robotics Core
 * C17 High-Performance Fastpath Execution Engine Header
 * ISO C17 Standard (-std=c17)
 */

#ifndef MOONPOOL_FASTPATH_H
#define MOONPOOL_FASTPATH_H

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define FASTPATH_RING_BUFFER_CAPACITY 1024

/* 6x6 Matrix Structure for Hard Real-Time Dynamic Kinematics */
typedef struct {
    double data[36]; /* Stored row-major: data[r * 6 + c] */
} FastpathMat6x6;

/* 6-Element Vector Structure for 6-DOF Wrench and Velocities */
typedef struct {
    double data[6];
} FastpathVec6;

/* Circular Lockless Ring Buffer Sample for 1 kHz Telemetry Streaming */
typedef struct {
    uint64_t timestamp_ns;
    double values[8];
} FastpathTelemetrySample;

/* Circular Ring Buffer */
typedef struct {
    FastpathTelemetrySample buffer[FASTPATH_RING_BUFFER_CAPACITY];
    volatile size_t head;
    volatile size_t tail;
} FastpathRingBuffer;

/* Fastpath Engine Initialization */
int fastpath_init(void);

/* Fast 6x6 Matrix Multiplication: C = A * B */
void fastpath_mat6x6_mul(const FastpathMat6x6 *a, const FastpathMat6x6 *b, FastpathMat6x6 *out);

/* Fast 6x6 Matrix-Vector Multiplication: y = A * x */
void fastpath_mat6x6_vec_mul(const FastpathMat6x6 *a, const FastpathVec6 *x, FastpathVec6 *out);

/* Vectorized Snell Law Acoustic Ray Layer Step */
/* Returns 1 if ray reached boundary, 0 if turning point encountered */
int fastpath_trace_layer_step(
    double p,
    double z_in,
    double z_out,
    double c_in,
    double c_out,
    double *out_dx,
    double *out_dt,
    double *out_ds
);

/* High-Throughput Ring Buffer Operations */
int fastpath_ring_push(FastpathRingBuffer *rb, uint64_t ts, const double *values, size_t count);
int fastpath_ring_pop(FastpathRingBuffer *rb, FastpathTelemetrySample *out_sample);
size_t fastpath_ring_count(const FastpathRingBuffer *rb);

#ifdef __cplusplus
}
#endif

#endif /* MOONPOOL_FASTPATH_H */
