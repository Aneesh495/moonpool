/*
 * Moonpool Subsea Robotics Core
 * C17 Fastpath Test Suite
 * ISO C17 Standard (-std=c17)
 */

#include "src/fastpath/fastpath.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <assert.h>

int main(void) {
    printf("Executing Moonpool C17 fastpath test suite...\n");

    /* 1. Initialization test */
    int init_ok = fastpath_init();
    assert(init_ok == 1);
    printf("  [PASS] fastpath_init\n");

    /* 2. 6x6 Matrix Multiplication test */
    FastpathMat6x6 a, b, c;
    for (int i = 0; i < 36; ++i) {
        a.data[i] = 0.0;
        b.data[i] = 0.0;
        c.data[i] = 0.0;
    }
    /* Set identity matrices */
    for (int i = 0; i < 6; ++i) {
        a.data[i * 6 + i] = 1.0;
        b.data[i * 6 + i] = 2.5;
    }
    fastpath_mat6x6_mul(&a, &b, &c);
    for (int i = 0; i < 6; ++i) {
        assert(fabs(c.data[i * 6 + i] - 2.5) < 1.0e-7);
    }
    printf("  [PASS] fastpath_mat6x6_mul\n");

    /* 3. 6x6 Matrix-Vector Multiplication test */
    FastpathVec6 x, y;
    for (int i = 0; i < 6; ++i) {
        x.data[i] = (double)(i + 1);
        y.data[i] = 0.0;
    }
    fastpath_mat6x6_vec_mul(&b, &x, &y);
    for (int i = 0; i < 6; ++i) {
        assert(fabs(y.data[i] - 2.5 * (i + 1)) < 1.0e-7);
    }
    printf("  [PASS] fastpath_mat6x6_vec_mul\n");

    /* 4. Acoustic Ray Layer Step test */
    double dx = 0.0, dt = 0.0, ds = 0.0;
    double p = cos(0.5) / 1500.0;
    int step_ok = fastpath_trace_layer_step(p, 0.0, 50.0, 1500.0, 1500.0, &dx, &dt, &ds);
    assert(step_ok == 1);
    assert(dx > 0.0);
    assert(dt > 0.0);
    assert(ds > 0.0);
    printf("  [PASS] fastpath_trace_layer_step\n");

    /* 5. Circular Ring Buffer test */
    FastpathRingBuffer rb;
    rb.head = 0;
    rb.tail = 0;

    double vals[4] = {10.5, 20.2, 30.1, 40.8};
    int push_ok = fastpath_ring_push(&rb, 1000000000ULL, vals, 4);
    assert(push_ok == 1);
    assert(fastpath_ring_count(&rb) == 1);

    FastpathTelemetrySample sample;
    int pop_ok = fastpath_ring_pop(&rb, &sample);
    assert(pop_ok == 1);
    assert(sample.timestamp_ns == 1000000000ULL);
    assert(fabs(sample.values[0] - 10.5) < 1.0e-7);
    assert(fabs(sample.values[1] - 20.2) < 1.0e-7);
    assert(fastpath_ring_count(&rb) == 0);
    printf("  [PASS] fastpath_ring_buffer\n");

    printf("All C17 fastpath tests passed successfully.\n");
    return 0;
}
