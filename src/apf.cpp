/*
 * Copyright (c) 2026-present LAAS-CNRS
 * SPDX-License-Identifier: LGPL-2.1
 */

/**
 * @brief First-order all-pass filter (quadrature signal generator).
 */

#include "apf.h"
#include "inverterFastCode.h"

void Apf::init(float32_t Ts) {
    _Ts = Ts;
    reset();
}

void Apf::reset() {
    _x_prev = 0.0F;
    _y_prev = 0.0F;
}

INV_FAST_CODE float32_t Apf::calc(float32_t input, float32_t w0) {
    // k = tan(w0 Ts / 2) by its series to x^5: at 50 Hz and 50 us, x = 0.0079 and the next term
    // (17/315 x^7) is below 1e-16; at w0 Ts / 2 = 0.3 (2 kHz at 50 us) the error is still 1e-5.
    const float32_t x = 0.5F * w0 * _Ts;
    const float32_t x2 = x * x;
    const float32_t k = x * (1.0F + x2 * (1.0F / 3.0F + x2 * (2.0F / 15.0F)));
    const float32_t a = (k - 1.0F) / (k + 1.0F);
    const float32_t output = a * input + _x_prev - a * _y_prev;
    _x_prev = input;
    _y_prev = output;
    return output;
}
