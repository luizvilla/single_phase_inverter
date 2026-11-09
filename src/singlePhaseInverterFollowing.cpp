/*
 *
 * Copyright (c) 2021-2024 LAAS-CNRS
 *
 *   This program is free software: you can redistribute it and/or modify
 *   it under the terms of the GNU Lesser General Public License as published by
 *   the Free Software Foundation, either version 2.1 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU Lesser General Public License for more details.
 *
 *   You should have received a copy of the GNU Lesser General Public License
 *   along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 * SPDX-License-Identifier: LGPL-2.1
 */

#include "singlePhaseInverterFollowing.h"
#include <math.h>
#include <errno.h>
#include "inverterValidation.h"

// Constructor for singlePhaseInverterFollowing
singlePhaseInverterFollowing::singlePhaseInverterFollowing() : _w(0.0F) {}

// Initialization function for singlePhaseInverterFollowing
int8_t singlePhaseInverterFollowing::init(float32_t V_bus, float32_t grid_Vpk, float32_t grid_w0, float32_t Ts) {
    _initialized = false;
    _fault_reason = VSI_FAULT_NONE;
    _mode = VSI_OFF;
    if (!inverter_validation::positive(V_bus) ||
        !inverter_validation::timing(grid_w0, Ts) ||
        !inverter_validation::positive(grid_Vpk)) {
        tripFault(VSI_FAULT_INVALID_CONFIGURATION);
        return -EINVAL;
    }

    // parameters of the SOGI filter
    float32_t rise_time = 1.0F * 2.0F * PI / grid_w0;
    float32_t wn = 3.0F / rise_time;
    float32_t xsi = 0.7F;
    float32_t Kp = 2 * wn * xsi / grid_Vpk;
    float32_t Ki = (wn * wn) / grid_Vpk;
    float32_t Kr = 500.0;

    _grid_Vpk = grid_Vpk;
    _w = grid_w0;
    _w_ref = grid_w0;
    _Ts = Ts;
    _sync_voltage_tolerance = 0.01F * grid_Vpk;
    resetSyncWindow();
    _theta = 0;
    _V_bus = V_bus;

    _sogi_v.init(Kr, _Ts);
    _sogi_i.init(Kr, _Ts);

    _pll_pi_params.Ts = Ts;
    _pll_pi_params.Td = 0.0;
    _pll_pi_params.N = 1;
    _pll_pi_params.Ti = Kp / Ki;
    _pll_pi_params.Kp = Kp;
    _pll_pi_params.lower_bound = -10.0F * _w_ref;
    _pll_pi_params.upper_bound = 10.0F * _w_ref;

    if (!inverter_validation::positive(Kp) || !inverter_validation::positive(Ki) ||
        !inverter_validation::positive(_pll_pi_params.Ti) ||
        !isfinite(_pll_pi_params.upper_bound)) {
        tripFault(VSI_FAULT_INVALID_CONFIGURATION);
        return -EINVAL;
    }
    const int8_t pll_result = _pll_pi.init(_pll_pi_params);
    if (pll_result != 0) {
        tripFault(VSI_FAULT_INVALID_CONFIGURATION);
        return pll_result;
    }
    _pll_pi.reset(); // The PLL supplies a correction added to _w_ref.

    _Idq_ref = {};
    _current_pi_params.Ts = _Ts;
    _current_pi_params.Kp = 0.001;
    _current_pi_params.Ti = 0.0003;
    _current_pi_params.Td = 0.0;
    _current_pi_params.N = 1.0;
    _current_pi_params.upper_bound = 30;
    _current_pi_params.lower_bound = -30;
    const int8_t current_d_result = _current_d_pi.init(_current_pi_params);
    if (current_d_result != 0) {
        tripFault(VSI_FAULT_INVALID_CONFIGURATION);
        return current_d_result;
    }
    const int8_t current_q_result = _current_q_pi.init(_current_pi_params);
    if (current_q_result != 0) {
        tripFault(VSI_FAULT_INVALID_CONFIGURATION);
        return current_q_result;
    }
    _current_d_pi.reset();
    _current_q_pi.reset();

    _Vdq = {};
    _Idq = {};
    _Vdq_output = {};
    _Vab = {};
    _Iab = {};
    _Vab_output = {};
    _power = {};
    _Vond = 0.0F;
    _duty_cycle = 0.5F;
    _enabled = false;
    _connect_requested = false;
    _run_requested = false;
    _grid_connected = false;
    _pwm_enabled = false;
    _grid_quality_good = true;
    _fault_reason = VSI_FAULT_NONE;
    _resync_requested = false;
    enterMode(VSI_OFF);

    _initialized = true;
    return 0;  // Return 0 to indicate success
}

// Calculate function for singlePhaseInverterFollowing
float32_t singlePhaseInverterFollowing::calculateDuty(float32_t vgrid_meas, float32_t igrid_meas) {
    if (!_initialized) {
        tripFault(VSI_FAULT_INVALID_CONFIGURATION);
        return _duty_cycle;
    }

    if (!isfinite(vgrid_meas) || !isfinite(igrid_meas)) {
        tripFault(VSI_FAULT_INVALID_MEASUREMENT);
        return _duty_cycle;
    }
    if (!isfinite(_V_bus) || _V_bus <= 0.0F) {
        tripFault(VSI_FAULT_INVALID_BUS_VOLTAGE);
        return _duty_cycle;
    }

    _w = _w_ref + _pll_pi.calculateWithReturn(0, -1.0F * _Vdq.q);

    const float32_t next_theta = _theta + _w * _Ts;
    if (!isfinite(_w) || !isfinite(next_theta)) {
        tripFault(VSI_FAULT_INVALID_CONTROL_STATE);
        return _duty_cycle;
    }
    // Floating-point wrapping avoids integer conversion overflow for large angles.
    _theta = fmodf(next_theta, 2.0F * PI);
    if (_theta < 0.0F) _theta += 2.0F * PI;

    // Tune the SOGI to the PLL estimate so its quadrature output stays balanced
    // off-nominal; at a fixed w_ref, Vq carries a 2w ripple far above the sync
    // tolerance. The estimate is clamped to the qualification window (±5 %) so
    // the SOGI-PLL loop cannot run away during lock transients.
    const float32_t w_sogi = fminf(fmaxf(_w, 0.95F * _w_ref), 1.05F * _w_ref);
    _Vab = _sogi_v.calc(vgrid_meas, w_sogi);
    _Iab = _sogi_i.calc(igrid_meas, w_sogi);

    _Vdq = Transform::rotation_to_dqo(_Vab, _theta);
    _Idq = Transform::rotation_to_dqo(_Iab, _theta);
    if (!isfinite(_w) || !isfinite(_Vdq.d) || !isfinite(_Vdq.q) ||
        !isfinite(_Idq.d) || !isfinite(_Idq.q)) {
        tripFault(VSI_FAULT_INVALID_CONTROL_STATE);
        return _duty_cycle;
    }
    updateSyncWindow();

    updateMode();
    if (_mode == VSI_RUN || _mode == VSI_GRID_CONNECTED ||
        _mode == VSI_STOPPING) {
        const float32_t id_ref = _mode == VSI_RUN ? _Idq_ref.d : 0.0F;
        const float32_t iq_ref = _mode == VSI_RUN ? _Idq_ref.q : 0.0F;
        _Vdq_output.d = _Vdq.d + _current_d_pi.calculateWithReturn(id_ref, _Idq.d);
        _Vdq_output.q = _Vdq.q + _current_q_pi.calculateWithReturn(iq_ref, _Idq.q);
        _Vdq_output.o = 0.0F;
    } else {
        _current_d_pi.reset();
        _current_q_pi.reset();
        // Compute grid-matching duty while waiting for a near-neutral PWM start.
        _Vdq_output = (_mode == VSI_PWM_WAIT || _mode == VSI_PWM_SETTLE ||
                       _mode == VSI_GRID_CONNECT) ? _Vdq : dqo_t{};
    }

    if (_mode == VSI_FAULT) {
        _Vdq_output = {};
    }

    _Vab_output = Transform::rotation_to_clarke(_Vdq_output, _theta);

    _Vond = _Vab_output.alpha;
    _duty_cycle = _Vond / (2.0F * _V_bus) + 0.5F;

    // Calculate active and reactive power
    _power.d = 0.5F * (_Vdq.d * _Idq.d + _Vdq.q * _Idq.q);
    _power.q = 0.5F * (_Idq.d * _Vdq.q - _Idq.q * _Vdq.d);

    if (!isfinite(_Vdq_output.d) || !isfinite(_Vdq_output.q) ||
        !isfinite(_Vab_output.alpha) || !isfinite(_Vab_output.beta) ||
        !isfinite(_duty_cycle) || !isfinite(_power.d) || !isfinite(_power.q)) {
        tripFault(VSI_FAULT_INVALID_CONTROL_STATE);
    }
    if (_mode == VSI_PWM_WAIT) {
        if (_pwm_enabled && fabsf(_duty_cycle - 0.5F) <= _pwm_start_duty_tolerance) {
            // Link the computed waveform only near neutral, after PWM start acknowledgement.
            enterMode(VSI_PWM_SETTLE);
        } else {
            // Start both outputs at neutral and hold until the waveform can be linked.
            _Vdq_output = {};
            _Vab_output = {};
            _Vond = 0.0F;
            _duty_cycle = 0.5F;
        }
    }
    return _duty_cycle;

}

dqo_t singlePhaseInverterFollowing::getVdq() {
    return _Vdq;
}

dqo_t singlePhaseInverterFollowing::getVdqOut() {
    return _Vdq_output;
}

dqo_t singlePhaseInverterFollowing::getIdq() {
    return _Idq;
}

dqo_t singlePhaseInverterFollowing::getPower() {
    return _power;
}

clarke_t singlePhaseInverterFollowing::getIab() {
    return _Iab;
}

clarke_t singlePhaseInverterFollowing::getVab() {
    return _Vab;
}

clarke_t singlePhaseInverterFollowing::getVabOutput() {
    return _Vab_output;
}

float32_t singlePhaseInverterFollowing::getTheta() {
    return _theta;
}

float32_t singlePhaseInverterFollowing::getw() {
    return _w;
}

bool singlePhaseInverterFollowing::getSync() {
    return synchronizationAligned() &&
           ((_mode == VSI_VOLTAGE_SYNC && _sync_delay_counter >= _sync_min_delay) ||
            _mode == VSI_PWM_WAIT || _mode == VSI_PWM_SETTLE || _mode == VSI_GRID_CONNECT ||
            _mode == VSI_GRID_CONNECTED || _mode == VSI_RUN);
}

void singlePhaseInverterFollowing::setVBus(float32_t V_bus) {
    _V_bus = V_bus;
}

void singlePhaseInverterFollowing::setIdqRef(dqo_t Idq_ref) {
    if (!isfinite(Idq_ref.d) || !isfinite(Idq_ref.q)) {
        tripFault(VSI_FAULT_INVALID_REFERENCE);
        return;
    }
    _Idq_ref = {Idq_ref.d, Idq_ref.q, 0.0F};
}

void singlePhaseInverterFollowing::requestRun() {
    if (_mode != VSI_FAULT) _run_requested = true;
}

void singlePhaseInverterFollowing::requestStop() {
    if (_mode != VSI_FAULT) _run_requested = false;
}

void singlePhaseInverterFollowing::setSyncOff() {
    if (_mode == VSI_FAULT) return;
    _resync_requested = true;
    _sync_delay_counter = 0;
}

void singlePhaseInverterFollowing::setWRef(float32_t w_ref) {
    if (!_initialized || !inverter_validation::timing(w_ref, _Ts)) {
        tripFault(VSI_FAULT_INVALID_REFERENCE);
        return;
    }
    _w_ref = w_ref;
    resetSyncWindow();  // the window spans one period of the new reference
    if (_mode == VSI_VOLTAGE_SYNC) _sync_delay_counter = 0;
    if (_mode == VSI_PWM_SETTLE) _pwm_settle_cycles = 0.0;
}

vsi_mode_t singlePhaseInverterFollowing::getMode() const {
    return _mode;
}

void singlePhaseInverterFollowing::requestSync() {
    if (_mode == VSI_FAULT) return;
    _enabled = true;
    _connect_requested = false;
    _run_requested = false;
    if (_mode == VSI_PWM_WAIT || _mode == VSI_PWM_SETTLE || _mode == VSI_GRID_CONNECT ||
        _mode == VSI_GRID_CONNECTED || _mode == VSI_RUN)
        setSyncOff();
}

void singlePhaseInverterFollowing::requestConnect() {
    if (_mode != VSI_FAULT) {
        _enabled = true;
        _connect_requested = true;
    }
}

void singlePhaseInverterFollowing::requestDisconnect() {
    if (_mode != VSI_FAULT) {
        _enabled = false;
        _connect_requested = false;
    }
}

void singlePhaseInverterFollowing::setGridConnected(bool connected) {
    _grid_connected = connected;
}

void singlePhaseInverterFollowing::setPwmEnabled(bool enabled) {
    _pwm_enabled = enabled;
    if (!enabled) {
        _pwm_settle_cycles = 0.0;
    }
}

bool singlePhaseInverterFollowing::setPwmStartDutyTolerance(float32_t tolerance) {
    if (!isfinite(tolerance) || tolerance <= 0.0F || tolerance >= 0.5F) return false;
    _pwm_start_duty_tolerance = tolerance;
    return true;
}

float32_t singlePhaseInverterFollowing::getPwmStartDutyTolerance() const {
    return _pwm_start_duty_tolerance;
}

void singlePhaseInverterFollowing::setPwmSettleCycles(uint16_t cycles) {
    if (_pwm_settle_min_cycles != cycles) {
        _pwm_settle_min_cycles = cycles;
        _pwm_settle_cycles = 0.0;
    }
}

uint16_t singlePhaseInverterFollowing::getPwmSettleCycles() const {
    return _pwm_settle_min_cycles;
}

bool singlePhaseInverterFollowing::isGridConnectionRequested() const {
    return (_mode == VSI_GRID_CONNECT ||
           _mode == VSI_GRID_CONNECTED || _mode == VSI_RUN ||
           _mode == VSI_STOPPING);
}

bool singlePhaseInverterFollowing::isPwmRequested() const {
    if (_mode == VSI_FAULT) return false;
    return _mode == VSI_PWM_WAIT || _mode == VSI_PWM_SETTLE || isGridConnectionRequested();
}

vsi_fault_reason_t singlePhaseInverterFollowing::getFaultReason() const {
    return _fault_reason;
}

void singlePhaseInverterFollowing::setGridQuality(bool good) {
    _grid_quality_good = good;
    if (!good && _mode == VSI_VOLTAGE_SYNC) _sync_delay_counter = 0;
    if (!good &&
        (_mode == VSI_PWM_WAIT || _mode == VSI_PWM_SETTLE || _mode == VSI_GRID_CONNECT || _mode == VSI_GRID_CONNECTED ||
         _mode == VSI_RUN || _mode == VSI_STOPPING || _mode == VSI_DISCONNECT)) {
        tripFault(VSI_FAULT_BAD_GRID_QUALITY);
    }
}

void singlePhaseInverterFollowing::tripFault(vsi_fault_reason_t reason) {
    if (_mode == VSI_FAULT) return;
    _fault_reason = reason == VSI_FAULT_NONE ? VSI_FAULT_EXTERNAL : reason;
    _enabled = false;
    _run_requested = false;
    enterMode(VSI_FAULT);
    if (_initialized) {
        _current_d_pi.reset();
        _current_q_pi.reset();
        if (reason == VSI_FAULT_INVALID_CONTROL_STATE) {
            // Discard poisoned filter/PLL history so recovery can observe a healthy grid.
            _sogi_v.init(500.0F, _Ts);
            _sogi_i.init(500.0F, _Ts);
            _theta = 0.0F;
            _Vdq = {};
            _pll_pi.reset();
        }
    }
    _Vdq_output = {};
    _Vab_output = {};
    _duty_cycle = 0.5F;
}

bool singlePhaseInverterFollowing::clearFault() {
    if (!_initialized || !inverter_validation::positive(_V_bus)) return false;

    if (_mode != VSI_FAULT) return true;
    if (_grid_connected || !_grid_quality_good) return false;
    _resync_requested = false;
    _fault_reason = VSI_FAULT_NONE;
    enterMode(VSI_OFF);
    return true;
}

void singlePhaseInverterFollowing::enterMode(vsi_mode_t mode) {
    _mode = mode;
    _state_elapsed = 0.0F;
    _sync_delay_counter = 0;
    _pwm_settle_cycles = 0.0;
    if (mode == VSI_PWM_WAIT) _pwm_enabled = false;
}

void singlePhaseInverterFollowing::updateMode() {
    if (_mode == VSI_FAULT) return;

    if ((!_enabled || _resync_requested) && _mode != VSI_OFF &&
        _mode != VSI_STOPPING && _mode != VSI_DISCONNECT) {
        if (_grid_connected) {
            enterMode(VSI_STOPPING);
        } else if (_mode == VSI_GRID_CONNECT) {
            enterMode(VSI_DISCONNECT);
        } else {
            enterMode(VSI_OFF);
        }
    }

    switch (_mode) {
    case VSI_OFF:
        _current_d_pi.reset();
        _current_q_pi.reset();
        if (_enabled) {
            _resync_requested = false;
            enterMode(VSI_VOLTAGE_SYNC);
        }
        break;
    case VSI_VOLTAGE_SYNC:
        if (synchronizationAligned()) {
            if (_sync_delay_counter < _sync_min_delay) ++_sync_delay_counter;
            if (_sync_delay_counter >= _sync_min_delay && _connect_requested)
                enterMode(VSI_PWM_WAIT);
        } else {
            _sync_delay_counter = 0;
        }
        break;
    case VSI_PWM_WAIT:
        if (!synchronizationAligned()) {
            tripFault(VSI_FAULT_SYNC_LOST);
        } else if (_grid_connected) {
            tripFault(VSI_FAULT_EXTERNAL);
        }
        // The duty check follows voltage synthesis in calculateDuty().
        break;
    case VSI_PWM_SETTLE:
        if (!synchronizationAligned()) {
            tripFault(VSI_FAULT_SYNC_LOST);
        } else if (_grid_connected) {
            // Contact feedback must remain open throughout the preconnection dwell.
            tripFault(VSI_FAULT_EXTERNAL);
        } else if (_pwm_enabled && _pwm_settle_min_cycles > 0) {
            // Integrate unwrapped PLL phase over the configured number of periods.
            _pwm_settle_cycles += static_cast<double>(_w) * _Ts / (2.0 * PI);
            if (_pwm_settle_cycles >= _pwm_settle_min_cycles)
                enterMode(VSI_GRID_CONNECT);
        } else {
            _pwm_settle_cycles = 0.0;
        }
        break;
    case VSI_GRID_CONNECT:
        // Revalidate before accepting contact feedback, even if it arrives this tick.
        if (!synchronizationAligned()) {
            tripFault(VSI_FAULT_SYNC_LOST);
        } else if (!_pwm_enabled) {
            tripFault(VSI_FAULT_PWM_UPDATE_FAILED);
        } else if (_grid_connected) {
            enterMode(VSI_GRID_CONNECTED);
        } else if ((_state_elapsed += _Ts) >= _connection_timeout) {
            tripFault(VSI_FAULT_CONNECTION_TIMEOUT);
        }
        break;
    case VSI_GRID_CONNECTED:
    case VSI_RUN:
        if (!_grid_connected) {
            tripFault(VSI_FAULT_GRID_CONNECTION_LOST);
        } else if (_run_requested && _mode == VSI_GRID_CONNECTED) {
            enterMode(VSI_RUN);
        } else if (!_run_requested && _mode == VSI_RUN) {
            enterMode(VSI_GRID_CONNECTED);
        }
        break;
    case VSI_STOPPING:
        if (!_grid_connected) {
            enterMode(VSI_OFF);
        } else if ((_state_elapsed += _Ts) >= _connection_timeout) {
            tripFault(VSI_FAULT_DISCONNECTION_TIMEOUT);
        } else if (fabsf(_Idq.d) < _disconnect_current_tolerance &&
                   fabsf(_Idq.q) < _disconnect_current_tolerance) {
            // Preserve the total timeout budget across both disconnect phases.
            const float32_t elapsed = _state_elapsed;
            enterMode(VSI_DISCONNECT);
            _state_elapsed = elapsed;
        }
        break;
    case VSI_DISCONNECT:
        if (!_grid_connected) {
            enterMode(VSI_OFF);
        } else if ((_state_elapsed += _Ts) >= _connection_timeout) {
            tripFault(VSI_FAULT_DISCONNECTION_TIMEOUT);
        }
        break;
    case VSI_FAULT:
        break;
    }
}

bool singlePhaseInverterFollowing::synchronizationAligned() const {
    // Judged on one-period averages (luizvilla/Core#7): instantaneous Vdq and w carry the
    // ripple of the grid harmonics and of any measurement offset.
    return _initialized && _grid_quality_good && isSyncWindowFull() &&
           _Vdq_avg.d > 0.9F * _grid_Vpk && _Vdq_avg.d < 1.1F * _grid_Vpk &&
           fabsf(_Vdq_avg.q) < _sync_voltage_tolerance &&
           fabsf(_w_avg - _w_ref) < 0.05F * _w_ref;
}

uint16_t singlePhaseInverterFollowing::syncWindowLength(float32_t w) const {
    const float32_t samples = (w > 0.0F && _Ts > 0.0F) ? 2.0F * PI / (w * _Ts) : 1.0F;
    long len = lroundf(samples);
    if (len < 1) len = 1;
    if (len > SYNC_WINDOW_MAX) len = SYNC_WINDOW_MAX;
    return static_cast<uint16_t>(len);
}

void singlePhaseInverterFollowing::resetSyncWindow() {
    _sync_win_len = syncWindowLength(_w_ref);
    _sync_win_pos = 0;
    _sync_win_count = 0;
    _sync_win_age = 0;
    _sync_win_full = false;
    _sync_sum_vd = _sync_sum_vq = _sync_sum_w = 0.0F;
    _sync_shadow_vd = _sync_shadow_vq = _sync_shadow_w = 0.0F;
    _Vdq_avg = {};
    _w_avg = 0.0F;
}

void singlePhaseInverterFollowing::updateSyncWindow() {
    // Drop the sample leaving the window (the one _sync_win_len steps back), then add the new one.
    if (_sync_win_count >= _sync_win_len) {
        const uint16_t old = static_cast<uint16_t>((_sync_win_pos + SYNC_WINDOW_MAX - _sync_win_len) % SYNC_WINDOW_MAX);
        _sync_sum_vd -= _sync_win_vd[old];
        _sync_sum_vq -= _sync_win_vq[old];
        _sync_sum_w -= _sync_win_w[old];
    }
    _sync_win_vd[_sync_win_pos] = _Vdq.d;
    _sync_win_vq[_sync_win_pos] = _Vdq.q;
    _sync_win_w[_sync_win_pos] = _w;
    _sync_sum_vd += _Vdq.d;
    _sync_sum_vq += _Vdq.q;
    _sync_sum_w += _w;
    _sync_shadow_vd += _Vdq.d;
    _sync_shadow_vq += _Vdq.q;
    _sync_shadow_w += _w;
    _sync_win_pos = static_cast<uint16_t>((_sync_win_pos + 1) % SYNC_WINDOW_MAX);
    if (_sync_win_count < SYNC_WINDOW_MAX) ++_sync_win_count;
    if (_sync_win_count >= _sync_win_len) _sync_win_full = true;
    const uint16_t n = _sync_win_count < _sync_win_len ? _sync_win_count : _sync_win_len;
    const float32_t inv = 1.0F / static_cast<float32_t>(n);
    _Vdq_avg = {_sync_sum_vd * inv, _sync_sum_vq * inv, 0.0F};
    _w_avg = _sync_sum_w * inv;
    // Once per period: the shadow sums hold exactly the last _sync_win_len samples, so
    // they replace the running sums (float rounding cannot accumulate); then the window
    // follows the averaged frequency. A length change is exact from the next period on.
    if (++_sync_win_age >= _sync_win_len) {
        _sync_win_age = 0;
        if (_sync_win_count >= _sync_win_len) {
            _sync_sum_vd = _sync_shadow_vd;
            _sync_sum_vq = _sync_shadow_vq;
            _sync_sum_w = _sync_shadow_w;
        }
        _sync_shadow_vd = _sync_shadow_vq = _sync_shadow_w = 0.0F;
        if (_sync_win_full) {
            const uint16_t len = syncWindowLength(_w_avg);
            if (len != _sync_win_len) {
                // Resize: adjust the running sums by the samples entering or leaving.
                while (_sync_win_len < len && _sync_win_len < _sync_win_count) {
                    const uint16_t i = static_cast<uint16_t>((_sync_win_pos + SYNC_WINDOW_MAX - _sync_win_len - 1) % SYNC_WINDOW_MAX);
                    _sync_sum_vd += _sync_win_vd[i]; _sync_sum_vq += _sync_win_vq[i]; _sync_sum_w += _sync_win_w[i];
                    ++_sync_win_len;
                }
                while (_sync_win_len > len) {
                    const uint16_t i = static_cast<uint16_t>((_sync_win_pos + SYNC_WINDOW_MAX - _sync_win_len) % SYNC_WINDOW_MAX);
                    _sync_sum_vd -= _sync_win_vd[i]; _sync_sum_vq -= _sync_win_vq[i]; _sync_sum_w -= _sync_win_w[i];
                    --_sync_win_len;
                }
            }
        }
        const uint16_t m = _sync_win_count < _sync_win_len ? _sync_win_count : _sync_win_len;
        const float32_t invm = 1.0F / static_cast<float32_t>(m);
        _Vdq_avg = {_sync_sum_vd * invm, _sync_sum_vq * invm, 0.0F};
        _w_avg = _sync_sum_w * invm;
    }
}

dqo_t singlePhaseInverterFollowing::getVdqAverage() const {
    return isSyncWindowFull() ? _Vdq_avg : dqo_t{};
}

float32_t singlePhaseInverterFollowing::getwAverage() const {
    return isSyncWindowFull() ? _w_avg : 0.0F;
}

bool singlePhaseInverterFollowing::isSyncWindowFull() const {
    return _sync_win_full && _sync_win_count >= _sync_win_len;
}
