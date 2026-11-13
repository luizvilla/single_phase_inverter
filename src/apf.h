/*
 * Copyright (c) 2026-present LAAS-CNRS
 * SPDX-License-Identifier: LGPL-2.1
 */

#ifndef APF_H
#define APF_H

#ifndef __arm__
#include "my_types.h"
#else
#include "arm_math.h"
#endif

/**
 * @brief First-order all-pass filter, used as the quadrature signal generator.
 *
 * H(s) = (w0 - s) / (w0 + s), discretized with Tustin prewarped at w0:
 * y[n] = a x[n] + x[n-1] - a y[n-1], a = (k - 1) / (k + 1), k = tan(w0 Ts / 2).
 * Gain 1 at every frequency (DC and harmonics included), phase -90 degrees exactly
 * at w0. Fed with the fundamental at w0, the output is the beta (quadrature) signal
 * of the input taken as alpha.
 *
 * @note w0 is given at every call (the inverter passes its clamped PLL estimate), so
 * the -90 degrees follow the grid frequency. The cost is one division per call.
 */
class Apf {
public:
    /**
     * @brief Set the sampling period and clear the state.
     * @param Ts Sampling period in seconds; w0 * Ts must stay well below pi.
     */
    void init(float32_t Ts);

    /** @brief Clear the state (previous input and output) without changing Ts. */
    void reset();

    /**
     * @brief Advance the filter by one sample.
     * @param input Input sample.
     * @param w0 Frequency at which the output lags the input by 90 degrees, in rad/s.
     * @return Output sample: the input shifted by -90 degrees at w0, gain 1.
     */
    float32_t calc(float32_t input, float32_t w0);

private:
    float32_t _Ts = 0.0F;    ///< Sampling period (s).
    float32_t _x_prev = 0.0F; ///< Previous input.
    float32_t _y_prev = 0.0F; ///< Previous output.
};

#endif // APF_H
