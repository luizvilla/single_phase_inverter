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

#pragma once

#include "sogi.h"
#include "transform.h"
#include "filters.h"
#include "inverterTypes.h"

/**
 * @class singlePhaseInverterFollowing
 * @brief Control a single-phase VSI using SOGI quadrature signals and dq PI loops.
 *
 * Uses a PLL to track grid phase, then applies current control with
 * measured-voltage feedforward through an explicit operating state machine.
 *
 * @pre Call init() before using the controller. Call calculateDuty() once per
 * sampling period, then read its cached measurements and commands as needed.
 * @note This class calculates references only; the caller configures and enables
 * hardware PWM, selects modulation, applies duty limits, and supplies valid measurements.
 * Only d and q components of dqo_t quantities are used or computed here; the
 * zero-sequence component is not maintained.
 */
class singlePhaseInverterFollowing {
public:
    /**
     * @brief Construct a controller with angular frequency initialized to zero.
     * @note Call init() before using any other operation.
     */
    singlePhaseInverterFollowing();

    /**
     * @brief Configure filters and PI loops and initialize control references.
     *
     * Resets phase, synchronization, PI states, and current references.
     * Power injection is initially disabled.
     *
     * @param V_bus Positive DC-bus voltage in volts, used for duty scaling.
     * @param grid_Vpk Positive nominal grid peak voltage in volts, used for PLL tuning.
     * @param grid_w0 Positive nominal grid angular frequency in radians per second.
     * @param Ts Positive control sampling period in seconds.
     * @return 0 on success, -EINVAL for invalid parameters, or a PI initialization error.
     * Invalid setup latches INVALID_CONFIGURATION and requires a successful init().
     * Frequency must be below the sampling Nyquist limit (grid_w0 * Ts < pi).
     * @note Starts in VSI_OFF with connection feedback and all requests cleared.
     */
    int8_t init(float32_t V_bus, float32_t grid_Vpk, float32_t grid_w0, float32_t Ts);

    /**
     * @brief Advance phase estimation and control by one sampling period.
     *
     * Updates SOGI signals, dq measurements, current control, and power.
     * FOLLOWING qualifies, on values averaged over one nominal grid period, positive
     * d voltage within 10% of the nominal peak, frequency within 5% of its reference,
     * and |Vq| below 1% of the nominal peak, for 1000 consecutive calls before
     * allowing a requested connection. requestSync() holds this
     * stage without requesting PWM or relay closure. A connection request first enables
     * PWM at 50 percent duty. After PWM enable acknowledgement, it holds neutral
     * until the computed grid-matching duty is near 50 percent, then links that
     * waveform and waits the configured settling periods (default ten),
     * then requests relay closure. Connection/opening feedback has a
     * one-second timeout. These are control defaults, not grid protection limits.
     *
     * @param vgrid_meas Instantaneous measured grid voltage in volts.
     * @param igrid_meas Instantaneous measured grid current in amperes.
     * @return Leg-1 duty ratio: output alpha voltage / (2 * V_bus) + 0.5.
     * The result is not clamped to [0, 1]. Pass it to the hardware API that
     * updates both bridge legs according to the selected modulation scheme.
     * @pre init() has been called and the stored bus voltage is positive.
     * @note VSI_RUN uses the supplied current reference without a ramp or clamp.
     * VSI_GRID_CONNECTED and VSI_STOPPING regulate both axes to zero.
     * PLL estimation continues in OFF; hardware actions remain with the caller.
     */
    float32_t calculateDuty(float32_t vgrid_meas, float32_t igrid_meas);

    /** @brief Read the latest measured dq grid voltage. @return d/q voltage in volts. */
    dqo_t getVdq();

    /** @brief Read the latest dq voltage command. @return d/q voltage in volts. */
    dqo_t getVdqOut();

    /** @brief Read the SOGI current signals. @return Alpha/beta current in amperes. */
    clarke_t getIab();

    /** @brief Read the SOGI voltage signals. @return Alpha/beta voltage in volts. */
    clarke_t getVab();

    /** @brief Read the bridge output voltage command. @return Alpha/beta voltage in volts. */
    clarke_t getVabOutput();

    /** @brief Read the latest measured dq grid current. @return d/q current in amperes. */
    dqo_t getIdq();

    /**
     * @brief Read power computed during the latest control step.
     * @return d = active power P in watts; q = reactive power Q in vars.
     * P = 0.5 * (Vd * Id + Vq * Iq); Q = 0.5 * (Id * Vq - Iq * Vd).
     */
    dqo_t getPower();

    /**
     * @brief Read the angular frequency used for the latest phase update.
     * @return PLL frequency, in rad/s.
     */
    float32_t getw();

    /** @brief Read the latest electrical phase. @return Wrapped phase in radians. */
    float32_t getTheta();

    /**
     * @brief Check whether FOLLOWING has qualified synchronization.
     * @return True after consecutive qualification in VOLTAGE_SYNC, including
     * sync-only operation, or while aligned in PWM_WAIT, PWM_SETTLE, GRID_CONNECT,
     * GRID_CONNECTED, or RUN.
     * False in OFF, STOPPING, DISCONNECT, FAULT, or when current alignment is lost.
     */
    bool getSync();

    /** @brief Read the operating mode. @return Current mode. */
    vsi_mode_t getMode() const;

    /** @brief Read the first latched fault cause. @return NONE when no fault is latched. */
    vsi_fault_reason_t getFaultReason() const;

    /**
     * @brief Supply the quality verdict of an external grid observer.
     * @param good True for acceptable grid quality; false for unacceptable quality.
     * @note In FOLLOWING, false blocks progression from synchronization to connection.
     * In PWM_WAIT, PWM_SETTLE, CONNECT, CONNECTED, RUN, STOPPING, or DISCONNECT it latches BAD_GRID_QUALITY
     * and drops PWM/relay requests, without waiting for current to decrease.
     * True does not clear a latched fault.
     * Defaults to true after init() for operation without an observer; the application
     * must supply an initial verdict and refresh it when using external monitoring.
     * Call from the control task; this method does not synchronize concurrent access.
     */
    void setGridQuality(bool good);

    /**
     * @brief Start or hold voltage synchronization without requesting PWM or connection.
     * @note Cancels RUN. If already connecting or connected, disconnects before
     * synchronizing again. Call requestConnect() to allow connection afterwards.
     * Consumed by calculateDuty(); ignored while faulted.
     */
    void requestSync();

    /**
     * @brief Request synchronization and grid connection in FOLLOWING mode.
     * @note Once connected, target zero current unless requestRun() was also issued.
     * Consumed by calculateDuty(); ignored while faulted.
     */
    void requestConnect();

    /**
     * @brief Request tracking of the supplied dq current reference in FOLLOWING.
     * @note Call requestConnect() separately to initiate connection. RUN begins
     * only after synchronization and confirmed connection. The reference is
     * applied directly, without an internal ramp or reference clamp.
     * Ignored while faulted.
     */
    void requestRun();

    /**
     * @brief Request zero current on both axes while keeping the grid connection.
     * @note Cancels any pending RUN request; does not disconnect or disable PWM.
     * Consumed by calculateDuty(); ignored while faulted.
     */
    void requestStop();

    /**
     * @brief Request orderly grid disconnection and return to OFF in FOLLOWING.
     * @note Enters STOPPING to target zero current with PWM and relay enabled, then
     * DISCONNECT to stop PWM, request relay opening, and wait for open feedback.
     * The separate RUN request is retained; use requestStop() to cancel it.
     * Consumed by calculateDuty(); ignored while faulted.
     */
    void requestDisconnect();

    /**
     * @brief Supply application-confirmed connection state.
     * @param connected True when the grid connection is established, false when open.
     * @note Does not command hardware. Keep feedback false throughout PWM_WAIT and PWM_SETTLE;
     * supply closed-contact feedback only when the connection is established.
     */
    void setGridConnected(bool connected);

    /**
     * @brief Acknowledge successfully applied PWM enable from the control task.
     * @param enabled True only after duty has been applied and hardware PWM started.
     * @note Defaults to false and is cleared on entry to PWM_WAIT. Without this
     * acknowledgement, the configured dwell cannot advance or request relay closure.
     * Report false whenever PWM stops; this restarts the full dwell in PWM_SETTLE.
     * Loss of acknowledgement in GRID_CONNECT faults before accepting relay feedback.
     * This confirms application action, not measured inverter terminal voltage.
     */
    void setPwmEnabled(bool enabled);

    /**
     * @brief Configure the near-neutral window for linking the computed waveform.
     * @param tolerance Maximum absolute difference from duty 0.5, in duty units.
     * Default 0.01 permits 0.49 through 0.51. Must be finite and in (0, 0.5).
     * @return False for invalid values (configuration unchanged), true otherwise.
     * @note Call from the control task. Preserved across init() and mode changes.
     * PWM_WAIT first requests PWM at exactly 0.5; apply this duty, enable both
     * outputs and acknowledge setPwmEnabled(true). Hold neutral until calculated
     * duty is within this window, then follow the returned waveform each step.
     * The relay request stays off throughout PWM_WAIT and PWM_SETTLE.
     */
    bool setPwmStartDutyTolerance(float32_t tolerance);

    /** @brief Read the near-neutral PWM start tolerance in duty units. */
    float32_t getPwmStartDutyTolerance() const;

    /**
     * @brief Configure acknowledged PWM settling before requesting relay closure.
     * @param cycles Number of complete grid periods; default 10. Zero holds
     * PWM_SETTLE indefinitely with synchronized PWM and the relay request off.
     * @note Changing this value restarts the dwell. To release a zero-cycle hold,
     * set a positive value; the full new dwell starts on the next control step.
     * Repeating the same value does not restart timing. Configuration persists
     * across init() and mode changes. Call from the control task. Changes after
     * PWM_SETTLE apply to the next connection attempt; they do not reopen a relay.
     */
    void setPwmSettleCycles(uint16_t cycles);

    /** @brief Read the configured settling periods; zero means indefinite hold. */
    uint16_t getPwmSettleCycles() const;

    /**
     * @brief Set the gains of the d and q current PIs.
     * @param kp Proportional gain in volts per ampere; finite and positive. Default 0.001.
     * @param ti Integral time in seconds; finite and positive. Default 0.0003.
     * @return False for invalid values (configuration unchanged), true otherwise.
     * @note Applied at once, also while the current loop runs (the integral state is kept).
     * Preserved across init(). Call from the control task.
     */
    bool setCurrentPiGains(float32_t kp, float32_t ti);

    /** @brief Read the current-PI proportional gain (V/A). */
    float32_t getCurrentPiKp() const;

    /** @brief Read the current-PI integral time (s). */
    float32_t getCurrentPiTi() const;

    /**
     * @brief Set the symmetric output limit of each current PI.
     * @param limit Largest correction added to the voltage feedforward, in volts; finite
     * and positive. Default 30.
     * @return False for invalid values or while the current loop runs (GRID_CONNECTED, RUN,
     * STOPPING); configuration unchanged. True otherwise.
     * @note Reinitializes both current PIs. Preserved across init(). Call from the control task.
     */
    bool setCurrentPiLimit(float32_t limit);

    /** @brief Read the current-PI output limit (V). */
    float32_t getCurrentPiLimit() const;

    /**
     * @brief Set the resonance gain of the current SOGI, separately from the voltage SOGI.
     * @param kr Gain in rad/s; finite and positive, and kr * Ts < 1 once init() has run.
     * Default 500 (the voltage SOGI's gain). In the dq frame the SOGI acts as a first-order
     * low-pass at kr / 2 on the measured current, which bounds the current-loop bandwidth.
     * @return False for invalid values or while the current loop runs (GRID_CONNECTED, RUN,
     * STOPPING); configuration unchanged. True otherwise.
     * @note Resets the current SOGI state. Preserved across init(). Call from the control task.
     */
    bool setCurrentSogiGain(float32_t kr);

    /** @brief Read the current-SOGI resonance gain (rad/s). */
    float32_t getCurrentSogiGain() const;

    /**
     * @brief Advance the output rotation to compensate the actuation delay.
     * @param periods Advance in control periods, in [0, 2]: the output voltage is rotated by
     * theta + periods * w * Ts. Default 0.
     * @return False for invalid values (configuration unchanged), true otherwise.
     * @note Applied at once. Preserved across init(). Call from the control task.
     */
    bool setOutputPhaseAdvance(float32_t periods);

    /** @brief Read the output-rotation advance in control periods. */
    float32_t getOutputPhaseAdvance() const;

    /**
     * @brief Scale the grid-voltage feedforward (the measured Vdq added to the current PI output).
     * @param gain Factor in [0.5, 1.5]; default 1. Above 1 the inverter-side voltage leads the grid in
     * amplitude, so current flows out of the inverter when the relay closes (bench step 1c/2).
     * @return False for invalid values (configuration unchanged), true otherwise.
     * @note Applied at once, in PWM_WAIT/PWM_SETTLE/GRID_CONNECT and with the current loop. In the
     * connected states the current PI integrates the resulting steady error away. Preserved across
     * init(). Call from the control task.
     */
    bool setFeedforwardGain(float32_t gain);

    /** @brief Read the feedforward gain. */
    float32_t getFeedforwardGain() const;

    /**
     * @brief Run the current PI (zero reference) from GRID_CONNECT, i.e. from the relay request, instead of
     * from GRID_CONNECTED. Default false. With the relay open the measured current is about zero, so the PI
     * stays at zero until the contact, then holds the current from the first sample.
     */
    void setCurrentLoopFromConnect(bool enabled);

    /** @brief Read whether the current PI runs from GRID_CONNECT. */
    bool getCurrentLoopFromConnect() const;

    /**
     * @brief Freeze the PLL for a connection: the frequency is latched at its period average (or the current
     * estimate before the window is full), the PLL integrator is held and the angle free-runs.
     * @param duration Seconds, in [0, 1]; 0 releases at once. @return False for invalid values.
     * @note Counts down every calculateDuty() call, in every mode.
     */
    bool freezePll(float32_t duration);

    /** @brief True while the PLL is frozen. */
    bool isPllFrozen() const;

    /**
     * @brief Hold the feedforward for a connection: the period-averaged Vd at the call (Vq 0), rotated by the
     * PLL angle, replaces the measured Vdq (synthesized feedforward); it also overrides an external feedforward.
     * @param duration Seconds, in [0, 1]; 0 releases at once. @return False for invalid values.
     * @note Counts down every calculateDuty() call. The feedforward gain applies.
     */
    bool holdFeedforward(float32_t duration);

    /** @brief True while the feedforward is held. */
    bool isFeedforwardHeld() const;

    /**
     * @brief Supply an instantaneous feedforward voltage (stationary frame, V) that replaces the measured
     * feedforward in PWM_WAIT, PWM_SETTLE, GRID_CONNECT and with the current loop, while enabled.
     * @param v_alpha Voltage, e.g. a copy of VAC compensated for the actuation delay; non-finite disables.
     * @param enabled Use it (true) or the measured feedforward (false).
     * @note Call every control period before calculateDuty() while enabled. The current PI output is
     * rotated by the PLL angle and added; the feedforward gain applies; a feedforward hold overrides it.
     */
    void setExternalFeedforward(float32_t v_alpha, bool enabled);

    /**
     * @brief Blanking after the predicted relay contact: the PLL stays frozen and the feedforward held this long
     * after it (see notifyRelayCommand()). 0 (default) disables the blanking.
     * @param after_contact Seconds, in [0, 0.5]. @return False for invalid values. Kept across init().
     */
    bool setConnectionBlanking(float32_t after_contact);

    /** @brief Read the blanking after the predicted contact (s). */
    float32_t getConnectionBlanking() const;

    /**
     * @brief The application has just commanded the relay closed; the contact is expected close_delay later.
     * With a blanking set: the PLL is frozen from now until close_delay + blanking; at the predicted contact the
     * feedforward is held (synthesized) for the blanking. Whatever the blanking, an external feedforward stops
     * at the predicted contact (after it, VAC is the inverter's own voltage through a series impedance).
     * @param close_delay Relay closing time (s), in [0, 0.5]. @return False for invalid values.
     * @note The schedule is cleared in OFF, VOLTAGE_SYNC, PWM_WAIT and FAULT.
     */
    bool notifyRelayCommand(float32_t close_delay);

    /** @brief True once the predicted contact has passed (until the connection ends). */
    bool isContactPredicted() const;

    /**
     * @brief Latch FAULT immediately and drop PWM/relay requests without waiting for zero current.
     * @param reason Fault cause; defaults to EXTERNAL. NONE is treated as EXTERNAL.
     * @note Preserves the first fault cause until cleared. Hardware actions remain
     * with the application; no relay opening or PWM stopping is performed here.
     */
    void tripFault(vsi_fault_reason_t reason = VSI_FAULT_EXTERNAL);

    /**
     * @brief Clear FAULT only after the application confirms disconnection.
     * @return True if no fault remains; false if connection feedback is still true
     * or the external grid-quality verdict is bad in FOLLOWING.
     * @note Invalid initialization cannot be cleared: call init() successfully.
     * A cleared controller stays OFF until explicitly enabled again.
     */
    bool clearFault();

    /**
     * @brief Query the grid connection request for the application.
     * @return True in CONNECT, CONNECTED, RUN, or STOPPING; false in DISCONNECT.
     */
    bool isGridConnectionRequested() const;

    /**
     * @brief Query the PWM enable request for the application.
     * @return True in PWM_WAIT, PWM_SETTLE and all states requesting grid connection.
     */
    bool isPwmRequested() const;

    /**
     * @brief Update the voltage used to normalize the bridge duty command.
     * @param V_bus Positive DC-bus voltage in volts; no validation is performed.
     */
    void setVBus(float32_t V_bus);

    /**
     * @brief Set the current reference without clamping.
     * @param Idq_ref Finite d/q current reference in amperes, used directly in RUN.
     * Nonfinite d/q values latch INVALID_REFERENCE; valid references are not clamped.
     */
    void setIdqRef(dqo_t Idq_ref);

    /**
     * @brief Restart synchronization, disconnecting first if connection is active.
     * @note Retains enable/run requests. Does not clear a fault.
     */
    void setSyncOff();

    /**
     * @brief Update the frequency reference used by phase control and both SOGIs.
     * @param w_ref Positive finite reference frequency in radians per second.
     * Must satisfy w_ref * Ts < pi; invalid values latch INVALID_REFERENCE
     * and leave the previous reference unchanged.
     * @note Does not retune the PLL gains or limits established by init().
     */
    void setWRef(float32_t w_ref);

private:
    bool synchronizationAligned() const;
public:
    /** @brief Vdq averaged over one nominal grid period (V); zero until the window is full. */
    dqo_t getVdqAverage() const;
    /** @brief PLL frequency averaged over one nominal grid period (rad/s); zero until the window is full. */
    float32_t getwAverage() const;
    /** @brief True once the averaging window holds one full period. */
    bool isSyncWindowFull() const;
private:

    // Internal state variables

    bool _initialized = false; ///< True only after all controller initialization succeeds.
    dqo_t _power;

    dqo_t _Vdq;
    dqo_t _Vdq_output;

    dqo_t _Idq;
    dqo_t _Idq_ref;

    clarke_t _Vab;
    clarke_t _Vab_output;
    clarke_t _Iab;
    float32_t _Vond;

    float32_t _grid_Vpk;
    float32_t _Ts;

    Sogi _sogi_i;
    Sogi _sogi_v;

    PidParams _pll_pi_params;    ///< PI controller parameters.
    Pid _pll_pi;                 ///< PI controller.

    PidParams _current_pi_params;    ///< PI controller parameters for the current loop.
    Pid _current_d_pi;                 ///< PI controller for the current loop.
    Pid _current_q_pi;                 ///< PI controller for the current loop.

    float32_t _theta;        ///< Current phase angle.
    float32_t _w;            ///< Angular frequency.
    float32_t _w_ref;        ///< Reference angular frequency.

    float32_t _duty_cycle;            ///< Internal duty cycle.

    float32_t _V_bus;   ///< Internal bus voltage.

    void updateMode();
    void enterMode(vsi_mode_t mode);

    vsi_mode_t _mode = VSI_OFF;
    vsi_fault_reason_t _fault_reason = VSI_FAULT_NONE;
    bool _grid_quality_good = true;
    bool _enabled = false;
    bool _connect_requested = false;
    bool _run_requested = false;
    bool _grid_connected = false;
    bool _pwm_enabled = false;
    float32_t _pwm_start_duty_tolerance = 0.01F;
    float32_t _pwm_settle_cycles = 0.0F; ///< Grid periods elapsed in PWM_SETTLE (single precision: the M4 FPU has no double).
    uint16_t _pwm_settle_min_cycles = 10; ///< Zero holds the relay open indefinitely.
    float32_t _current_kp = 0.001F;      ///< Current-PI proportional gain (V/A).
    float32_t _current_ti = 0.0003F;     ///< Current-PI integral time (s).
    float32_t _current_limit = 30.0F;    ///< Current-PI output limit (V).
    float32_t _current_sogi_kr = 500.0F; ///< Current-SOGI resonance gain (rad/s).
    float32_t _output_phase_advance = 0.0F; ///< Output-rotation advance (control periods).
    float32_t _feedforward_gain = 1.0F; ///< Scale of the grid-voltage feedforward.
    bool _current_loop_from_connect = false; ///< Current PI from GRID_CONNECT.
    float32_t _pll_freeze_remaining = 0.0F;  ///< PLL freeze left (s).
    float32_t _pll_freeze_w = 0.0F;          ///< Latched frequency during the freeze (rad/s).
    float32_t _ff_hold_remaining = 0.0F;     ///< Feedforward hold left (s).
    float32_t _ff_hold_vd = 0.0F;            ///< Held Vd (V).
    float32_t _external_ff_v = 0.0F;         ///< External feedforward voltage (V).
    bool _external_ff_enabled = false;       ///< External feedforward in use.
    float32_t _connection_blanking = 0.0F;   ///< Blanking after the predicted contact (s).
    float32_t _contact_countdown = 0.0F;     ///< Time to the predicted contact (s).
    bool _contact_pending = false;           ///< A relay command was notified, contact not reached yet.
    bool _contact_passed = false;            ///< The predicted contact has passed.
    /** @brief True while the current PIs drive the output. */
    bool currentLoopActive() const;
    /** @brief (Re)initialize both current PIs from the stored gains and limit. */
    int8_t initCurrentPis();
    bool _resync_requested = false;
    float32_t _state_elapsed = 0.0F;
    const float32_t _connection_timeout = 1.0F; ///< Connection/disconnection timeout (s).
    float32_t _sync_voltage_tolerance = 0.0F; ///< Tolerance on the period-averaged |Vq| (V): 1 % of the nominal peak, set by init().

    /* Synchronization is judged on Vd, Vq and w averaged over one grid period: a real
     * grid's harmonics and the VAC offset ripple these quantities at multiples of the grid
     * frequency, and the ripple averages out over a whole period (luizvilla/Core#7). The
     * window spans one period of the averaged frequency (one period of w_ref until the
     * first window is full), resized once per period, so it stays a whole period off-nominal. */
    static constexpr uint16_t SYNC_WINDOW_MAX = 256; ///< Ring buffer size: most samples in a window.
    float32_t _sync_win_vd[SYNC_WINDOW_MAX] = {};
    float32_t _sync_win_vq[SYNC_WINDOW_MAX] = {};
    float32_t _sync_win_w[SYNC_WINDOW_MAX] = {};
    uint16_t _sync_win_len = 1;   ///< Window length in samples (capped at SYNC_WINDOW_MAX).
    uint16_t _sync_win_pos = 0;   ///< Next ring slot to write.
    uint16_t _sync_win_count = 0; ///< Samples stored (up to SYNC_WINDOW_MAX).
    uint16_t _sync_win_age = 0;   ///< Samples since the window length was last set.
    bool _sync_win_full = false;  ///< The window has held one full period at least once.
    /** @brief Window length for a frequency: one period, within [1, SYNC_WINDOW_MAX]. */
    uint16_t syncWindowLength(float32_t w) const;
    /* Shadow sums: the same samples accumulated from the start of the current period;
     * at the end of each period they replace the running sums, which removes float drift
     * at constant cost per call (no O(window) loop in the control task). */
    float32_t _sync_shadow_vd = 0.0F;
    float32_t _sync_shadow_vq = 0.0F;
    float32_t _sync_shadow_w = 0.0F;
    float32_t _sync_sum_vd = 0.0F;
    float32_t _sync_sum_vq = 0.0F;
    float32_t _sync_sum_w = 0.0F;
    dqo_t _Vdq_avg = {};          ///< Period-averaged Vdq (V).
    float32_t _w_avg = 0.0F;      ///< Period-averaged PLL frequency (rad/s).
    /** @brief Empty the averaging window and size it to one period of w_ref. */
    void resetSyncWindow();
    /** @brief Add this step's Vdq and w to the averaging window. */
    void updateSyncWindow();
    const float32_t _disconnect_current_tolerance = 0.1F; ///< Both dq axes (A).
    uint16_t _sync_delay_counter = 0;
    const uint16_t _sync_min_delay = 1000;
};
