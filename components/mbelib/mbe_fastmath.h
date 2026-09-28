/*
 * Fast cosf() for mbelib, force-included (-include) into the mbelib sources
 * only (see CMakeLists.txt) - mbelib itself is not modified.
 *
 * Why: with uvquality 3 mbelib's speech synthesis calls cosf() ~21000 times
 * per 20 ms frame (~1 M calls/s). On the ESP32-P4 newlib's cosf is a software
 * routine of roughly 100-200 cycles, i.e. 30-55 % of a core for cosines
 * alone - enough to starve the DMR task and make the voice choppy.
 *
 * This version: Cody-Waite reduction to [-pi, pi], fold to [0, pi/2] by symmetry,
 * then a Taylor polynomial to x^12 (remainder < 1e-8 on that interval).
 * About 15 FPU operations. Host test: decoded speech vs. libm cosf -> see
 * test/host_dmr (identical AMBE frames, audio difference far below audibility).
 */
#ifndef MBE_FASTMATH_H
#define MBE_FASTMATH_H

#include <math.h>

static inline float mbe_fast_cosf(float x)
{
    float k = x * 0.15915494309f; /* 1 / (2 pi) */
    float s = 1.0f, x2;
    k = (float)(int)(k + (k >= 0.0f ? 0.5f : -0.5f));
    /* Cody-Waite reduction: 2*pi = C1 + C2 with C1 = 6.28125 exact in 8 bits,
     * so k*C1 is exact for |k| < 65536 and only the tiny C2 term rounds. */
    x -= k * 6.28125f;
    x -= k * 1.9353071795864769e-3f; /* [-pi, pi] */
    if (x < 0.0f)
    {
        x = -x; /* cos is even */
    }
    if (x > 1.57079632679f)
    {
        x = 3.14159265359f - x; /* cos(x) = -cos(pi - x) */
        s = -1.0f;
    }
    x2 = x * x;
    return s * (1.0f + x2 * (-0.5f + x2 * (4.16666667e-2f + x2 * (-1.38888889e-3f +
                x2 * (2.48015873e-5f + x2 * (-2.75573192e-7f + x2 * 2.08767570e-9f))))));
}

#define cosf mbe_fast_cosf

#endif /* MBE_FASTMATH_H */
