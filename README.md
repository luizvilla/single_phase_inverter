# single_phase_inverter

Two independent controllers use SOGI quadrature signals and dq PI loops:

- `singlePhaseInverterFollowing` tracks grid phase with a PLL and controls current through synchronization and connection modes.
- `singlePhaseInverterForming` generates phase from a frequency reference and uses cascaded voltage/current loops.

Each class lives in its own header and source file. Include the specific header,
or `singlePhaseInverter.h` for both. The controller type is chosen when declaring
the object; there is no FORMING/FOLLOWING selector or runtime `setMode()`.

## Initialization and control

```cpp
#include "singlePhaseInverterFollowing.h"
#include "singlePhaseInverterForming.h"

singlePhaseInverterFollowing following;
if (following.init(100.0F, 20.0F, 2.0F * PI * 50.0F, 0.0001F) == 0) {
    following.requestSync();
}

singlePhaseInverterForming forming;
if (forming.init(100.0F, 2.0F * PI * 50.0F, 0.0001F) == 0) {
    forming.setVdqRef({20.0F, 0.0F, 0.0F});
}
```

Following initialization takes bus voltage (V), nominal grid peak voltage (V),
reference angular frequency (rad/s), and sampling period (s). Forming takes bus
voltage, reference angular frequency, and sampling period. Both return `0` on
success, `-EINVAL` for invalid parameters, or the failing PI initialization error.
Parameters must be finite and positive, with representable reciprocals and a
frequency below Nyquist (`w_ref * Ts < pi`). Invalid initialization latches
`VSI_FAULT_INVALID_CONFIGURATION`; control stays disabled until `init()` succeeds. Call `init()` with hardware stopped to reinitialize
filters, PI loops, phase, references, measurements, and faults.

Call `calculateDuty(voltage, current)` once per sampling period with instantaneous
measurements in volts and amperes. It returns the unclamped leg duty
`0.5 + alpha_output / (2 * V_bus)`. The application selects modulation, limits duty,
applies paired bridge PWM updates, and handles PWM enable and relay hardware.

Both controllers provide `getVdq()`, `getIdq()`, `getVdqOut()`, `getVab()`,
`getIab()`, `getVabOutput()`, `getTheta()`, and `getw()`. Frequency is the PLL
estimate for following and the oscillator reference for forming. `getPower()`
returns active power (d, W) and reactive power (q, var). `setVBus()`, `setIdqRef()`,
and `setWRef()` update bus voltage, dq current reference, and angular frequency
reference without clamping. Nonfinite d/q references latch
`VSI_FAULT_INVALID_REFERENCE` immediately and leave the previous reference intact.
Frequency updates must satisfy the same positive/finite/timing rules as initialization;
invalid updates fault without replacing the frequency reference. Following frequency
updates do not retune PLL gains. Both controllers check computed commands, duty,
and power for nonfinite results and latch `VSI_FAULT_INVALID_CONTROL_STATE` on
arithmetic overflow, returning neutral duty (0.5) with PWM requests off.

Forming additionally provides `setVdqRef()` and `getIdqRefDelta()`. The voltage
loop correction is added to its current reference. A nonpositive d-axis voltage
reference resets both loops and commands zero voltage on the next step.
`isPwmRequested()` is true when that reference is positive and no fault is latched.
Both controllers expose `tripFault()`, `getFaultReason()`, and `clearFault()`.
Forming retains its voltage reference when a fault is cleared; the caller must
ensure hardware is ready before clearing it.

## Following operating modes

`vsi_mode_t` names the following controller's operating modes. Read the current
mode with `getMode()` and request transitions using the lifecycle methods.
The previous `vsi_status_t` and `getStatus()` names have been replaced.

| Mode | Behavior |
| --- | --- |
| `VSI_OFF` | No PWM or relay request; current PI states reset. PLL still runs. |
| `VSI_VOLTAGE_SYNC` | Qualify grid voltage, PLL alignment, and frequency continuously. |
| `VSI_PWM_WAIT` | Request PWM at exactly 50%; after enable acknowledgement, hold neutral until computed grid-matching duty is near 50%, then link the waveform. |
| `VSI_PWM_SETTLE` | Match measured grid voltage with PWM requested and relay open for the configured acknowledged grid periods, or hold indefinitely. |
| `VSI_GRID_CONNECT` | Keep grid-matching PWM active, request relay connection, wait for feedback. |
| `VSI_GRID_CONNECTED` | Regulate both current axes to zero, regardless of stored current reference. |
| `VSI_RUN` | Apply the supplied dq current reference directly, without ramping or reference clamping. |
| `VSI_STOPPING` | Keep PWM and relay enabled; regulate both current axes to zero. |
| `VSI_DISCONNECT` | PWM off; request relay opening and wait for open feedback. |
| `VSI_FAULT` | Drop PWM/relay requests immediately; latch until explicitly cleared. |

Call `requestSync()` to start and hold `VSI_VOLTAGE_SYNC` with PWM and relay
requests off, even after synchronization qualifies. It cancels a pending RUN
request and disconnects first if already connecting or connected.
Call `requestConnect()` to allow neutral PWM startup and the settling stage after synchronization. After successful connection,
`requestRun()` requests RUN and `requestStop()` selects connected
zero-current control. A RUN request can be supplied before connecting, but
connection always passes through one zero-current GRID_CONNECTED control step.
`requestDisconnect()` requests disconnection. Any reference ramp, including a
planned power reduction before disconnecting, belongs to the caller.

Synchronization requires, on values averaged over one grid period (a moving window
of 2π / (w · Ts) samples, at most 512, where w is the averaged frequency estimate,
resized once per period; w_ref until the first window is full), positive d
voltage within 10% of the configured grid peak, frequency within 5% of the frequency
reference, and `abs(Vq)` below 1% of the configured peak (about 3.3 V at 230 V, a
phase error of about 0.6°), for 100 ms of consecutive control calls (1000 at 10 kHz, 2000 at 20 kHz).
A real grid's harmonics and any offset on the voltage measurement ripple Vd, Vq and
the frequency estimate at multiples of the grid frequency; the ripple averages out
over a period, while a phase jump moves the averages at once. `getVdqAverage()` and
`getwAverage()` return the averages. The window restarts on `init()` and
`setWRef()`, so qualification needs one period more after either. A failed sample resets the
counter. `getSync()` reports true once this dwell completes, even while
`requestSync()` holds VOLTAGE_SYNC with PWM and relay requests off. A failed
sample or bad external verdict clears qualification; a full dwell is required again.
Changing the frequency reference in VOLTAGE_SYNC also restarts the dwell.
In PWM_WAIT, PWM_SETTLE and GRID_CONNECT, alignment is checked on every step, before accepting relay
feedback. Loss of alignment latches `VSI_FAULT_SYNC_LOST` and drops both hardware
requests, including when contact feedback arrives on that same step.
These are control defaults, not a grid protection implementation.
Loss of established connection feedback latches FAULT. Grid quality assessment
belongs to an external observer. Feed its verdict to `setGridQuality(bool)`
from the control task; no grid-quality observer is implemented in this class.
A bad verdict blocks connection while synchronizing. In PWM_WAIT, PWM_SETTLE, GRID_CONNECT,
GRID_CONNECTED, RUN, STOPPING, or DISCONNECT, it immediately latches
`VSI_FAULT_BAD_GRID_QUALITY` and drops PWM/relay requests without waiting for
current to decrease. A good verdict alone does not clear FAULT or restart RUN.
The verdict defaults to true after `init()` for operation without an observer;
when monitoring is used, supply the observer's initial verdict before connecting
and refresh it as needed. Unknown/unready observer status can be supplied as false.

`getFaultReason()` reports the first fault cause until a successful `clearFault()`
or `init()`. Causes distinguish bad grid quality, connection/disconnection timeout,
lost connection feedback, invalid measurements, invalid bus voltage, invalid
control state, invalid configuration/references, lost connection-time alignment,
failed PWM updates, and externally reported faults. Applications
can call `tripFault(reason)`; the no-argument call records `VSI_FAULT_EXTERNAL`.
A later fault does not overwrite the first cause. `VSI_FAULT_NONE` means no
latched fault.

After qualification, PWM_WAIT requests PWM at exactly 50% duty with the relay
request off. Apply that returned duty through the paired bridge API, enable both
outputs, then acknowledge with `setPwmEnabled(true)`. While waiting for that
acknowledgement, the returned command remains exactly 50%.

The PLL and full-amplitude grid-matching waveform continue to be computed.
After acknowledgement, hold the applied duty at 50% until the computed duty is
within the near-neutral window. On that control step, the returned duty switches
to the computed waveform and the mode becomes PWM_SETTLE. Subsequent calls track
the waveform directly; there is no voltage-amplitude ramp. The current PIs stay
reset during PWM_WAIT, PWM_SETTLE and GRID_CONNECT.

`setPwmStartDutyTolerance(tolerance)` configures this handover window in duty
units; the default 0.01 means 49–51%. Values must be finite and strictly between
0 and 0.5; the setter returns false without changing the configuration otherwise.
The setting survives init() and mode changes. The application's
`inverter_pwm_start_duty_tolerance` constant in `src/main.cpp` supplies its initial
value. The first enable request always returns exactly 50%, independently of the
window. Apply each returned duty in the same control step as its hardware request.

The full settling dwell starts after waveform handover, with PWM acknowledged.
The controller integrates PLL frequency for the configured number of grid periods
(default ten: about 200 ms at 50 Hz, 167 ms at 60 Hz). A false acknowledgement
resets the dwell; entry into PWM_WAIT clears previous acknowledgement. A frequency
reference update during settling also restarts the dwell. Loss of alignment or
bad grid quality faults with both requests off. Unexpected closed-contact feedback
in PWM_WAIT or PWM_SETTLE faults with EXTERNAL. `requestSync()` or
`requestDisconnect()` cancels either stage before relay actuation. PWM
acknowledgement confirms software action; it does not measure inverter voltage
across the relay. Terminal voltage/amplitude matching still needs hardware validation.

Configure this from the control task using `setPwmSettleCycles(uint16_t)` and read
it with `getPwmSettleCycles()`. Zero holds PWM_SETTLE indefinitely: synchronized
PWM stays requested and relay closure stays inhibited. The normal PWM
acknowledgement, synchronization, quality and contact-feedback checks still apply.
To release the hold, set a positive count; the controller waits the full new dwell
before requesting closure. Changing the count also restarts an in-progress dwell;
repeating the same setting preserves progress. The configuration survives `init()`
and mode changes. Changing it after PWM_SETTLE affects the next connection attempt.

```cpp
inverter.setPwmSettleCycles(0);  // Hold synchronized PWM with the relay open.
inverter.requestConnect();     // Qualify sync, enable neutral PWM, then link waveform.
// The hardware loop applies duty, starts PWM, and acknowledges setPwmEnabled(true).
// Later, from the control task:
inverter.setPwmSettleCycles(20); // Allow closure after 20 more complete periods.
```

The application's `inverter_pwm_settle_cycles` constant in `src/main.cpp` supplies
the startup configuration; it defaults to 10. The current application's sync-only
operation does not enter PWM_SETTLE.

Connection feedback must arrive within one second of GRID_CONNECT. Loss of PWM
acknowledgement in GRID_CONNECT faults with PWM_UPDATE_FAILED before accepting
contact feedback. During
STOPPING, PWM and relay remain enabled while both current axes target zero.
Once both measured dq axes are below 0.1 A, the mode becomes DISCONNECT:
PWM and relay requests are false while awaiting open contact feedback. Current
rising again does not re-enable PWM or request relay closure in DISCONNECT.
The combined STOPPING/DISCONNECT sequence has a one-second timeout. Timeout latches FAULT.
This describes the existing orderly `requestDisconnect()` path. Emergency
`tripFault(reason)` immediately drops requests and does not follow that current
settling sequence. Actual PWM stopping and relay actuation remain application
hardware actions.

### Application hardware hooks

- `isGridConnectionRequested()` supplies the relay command. In FOLLOWING, this
  is true exactly in GRID_CONNECT, GRID_CONNECTED, RUN, and STOPPING.
- `isPwmRequested()` supplies the PWM enable command. The application applies
  the duty through its paired hardware API and stops PWM when this is false.
  In FOLLOWING, it is true in PWM_WAIT and PWM_SETTLE plus the four relay-request states;
  DISCONNECT always requests both outputs off. After applying the duty and starting
  PWM successfully, acknowledge with `setPwmEnabled(true)` from the control task.
  Report `setPwmEnabled(false)` whenever PWM stops; report failed duty/start actions
  with `tripFault(VSI_FAULT_PWM_UPDATE_FAILED)`.
- `setGridConnected(bool)` supplies confirmed relay contact feedback, independently
  of the requested state. Do not echo the request as physical feedback.
- `clearFault()` succeeds only with open connection feedback and, in FOLLOWING,
  an acceptable external grid-quality verdict. It clears the fault reason and
  leaves the controller OFF; connection and RUN requests must be issued again.
  Invalid initialization requires a successful `init()`; invalid stored bus
  voltage must be corrected before clearing a fault.

The example in `src/main.cpp` supports synchronization only. Serial command `c`
starts and holds `VSI_VOLTAGE_SYNC`, independently of the boost mode. `i` cancels
it and selects idle. Selecting `p`, `b`, or `m` also cancels the synchronization
hold and selects the corresponding boost mode. No command requests grid
connection or RUN. The inverter bridge stays disabled, and the application
has no relay command or contact-feedback integration.

## Validation

Run `bash tests/inverter/run.sh` from the project root. Host tests use the real
SOGI, PLL, transforms, and PI implementation with CMSIS math/logging shims.
They cover both controller types, following sequencing, direct references, and
fault handling, invalid setup/references, arithmetic overflow, sync-only
qualification, neutral PWM start and near-neutral waveform handover, ten-period acknowledged PWM settling at 50/60 Hz and two sample
rates, configurable dwell counts, indefinite hold/release, interrupted/cancelled settling, and alignment loss before relay closure. Tests enable undefined
behavior and float-to-integer overflow sanitizers. Power-stage behavior and relay timing require hardware validation.

## Dependencies

Requires `transform`, `pid`, `trigo`, and math/type headers from the OwnTech
control library configured in `platformio.ini`.

## Assumptions to confirm for the 450 W design

These are settings and gaps found in the source, not approved hardware ratings.
The runtime example uses FOLLOWING; FORMING tuning is listed for completeness.

| Item | Current assumption | Evidence or decision needed |
| --- | --- | --- |
| Power rating | 450 W is the intended rating supplied by the user; controllers have no explicit power limit. | Define whether this is PV input, DC output, or continuous AC output, and the voltage/temperature range. Obtain the schematic, component ratings, efficiency and thermal limits. |
| Boost total input current | Voltage PID output is 0–2 A in main.cpp, shared by BUSREG and MPPT. | No rating-derived justification is documented. Use input voltage and efficiency: for 450 W downstream, input current is `450 / (efficiency * VLow)`. At 40 V this is 11.25 A ideally or 12.5 A at assumed 90% efficiency. Do not replace 2 A using bus current. |
| Leg allocation | Leg 1 gets up to 1 A; leg 2 receives all excess. STARTUP only uses leg 1. | Confirm sequential allocation versus balanced/interleaved operation. Leg 2 has no independent current-reference ceiling; raising total current alone would send almost all extra current to leg 2. |
| Startup current | 1 A input-current reference maximum, not a measured overcurrent trip. | Confirm capacitor charging, input-source capability, inductor saturation and switch peak current including ripple. |
| Startup ramp | 10 V/s, 0.01 V per 1 ms cold call, seeded from measured bus voltage. | Confirm capacitance and allowed inrush: capacitor power is `Cbus * Vbus * dVbus/dt`; add loads and losses to derive input current. A 40→400 V reference takes 36 s without pauses. Actual bus rise can lag the reference. |
| Uncontrolled precharge | A boost normally has a passive input-to-bus path depending on its topology. | Check actual diode/body-diode paths and precharge hardware. A software voltage ramp cannot limit current through an uncontrolled path. |
| BUSREG transitions | BUSREG uses the bus target immediately; selecting a mode resets PI states and demand. No input-current slew limiter. | Decide whether direct BUSREG entry is allowed and specify current/power slew and bumpless transitions. Lowering the bus target cannot actively discharge a unidirectional boost. |
| Bus target/headroom | 400 V target, 395 V burst resume, 405 V pause; target can be changed from 1 to 404 V by serial commands. | Confirm bus capacitor/switch ratings, grid peak plus filter voltage and modulation margin, and undervoltage/overvoltage trips. The command lower bound does not guarantee a usable inverter bus. |
| Burst operation | STARTUP/BUSREG pause both legs at 405 V and reset all PIs; resume at 395 V. MPPT bypasses burst protection. | Confirm acceptable ripple, restart transients, sensing delay and 100 Hz single-phase bus ripple. Burst cutoff is not an independent overvoltage protection mechanism. |
| Boost voltage PI | Ts=1 ms, Kp=0.05, Ti=0.1 s; no derivative. | Validate loop bandwidth and stability with actual capacitance, source impedance and load. Check interaction with 100 Hz ripple and inverter power regulation. |
| Boost current PI | Ts=100 us, Kp=0.000215, Ti=75.175 us; same gains for both legs. | Validate inductance, parasitics, sensor scaling, PWM frequency and sample/update delay across the operating range. |
| Boost duty | Current PID output is 5–95%; hardware minimum duty is also 5%. Zero reference disables the leg through enable flags. | Confirm off behavior and required conversion ratio. Ideal boost duty is `1 - VLow/Vbus`; 40→400 V requires about 90%, before losses. Verify available off-time and saturation handling. |
| MPPT speed | Fixed 0.1 A perturbation each 1 ms cold-loop call: up to 100 A/s nominal reference slew. Uses instantaneous VLow × summed current. | Confirm PV dynamics and settling before each comparison; decide averaging/filtering, independent MPPT cadence, perturbation size and response to irradiance changes. |
| MPPT power balance | Boost MPPT does not regulate bus voltage. The example has no inverter bus-voltage controller and keeps inverter PWM off. | Define DC-bus regulation and power handoff before MPPT operation with injection. Prevent uncontrolled bus charging when no output stage consumes power. |
| Grid nominal values | 230 V RMS (325.27 V peak), 50 Hz; inverter measurements and dq references use instantaneous/peak conventions. | Confirm sensor units, polarity, phase order and P/Q sign. At unity power factor, 450 W / 230 V = 1.96 A RMS, 2.77 A peak; this is separate from boost input current. |
| Inverter current PI | FOLLOWING Kp=0.001, Ti=0.0003 s; each output correction limited to ±30 V, added to grid-voltage feedforward. | Tune for the actual L/LCL filter, resistance, delay and damping. ±30 V is a voltage correction limit, not a 30 A current limit. Verify cross-axis coupling and saturation behavior. |
| Inverter RUN reference | d/q current references are applied directly, without clamp, ramp, or power limit. | Define peak/RMS convention, active/reactive priority, total current-vector limit and current/power slew. Verify measured overcurrent protection separately. |
| Modulation | Controller duty is `0.5 + Valpha/(2*Vbus)` without clamping; example initializes paired unipolar PWM. | Validate factor of two, bridge polarity, actual output amplitude, paired updates, carrier alignment, duty limits and anti-windup when modulation saturates. |
| PLL/SOGI | SOGI gain 500; PLL tuned for nominal one-period rise time and damping 0.7; correction bounds ±10 nominal angular frequency. | Check gain units, phase/amplitude errors, acquisition, harmonics, DC offset, delay and frequency excursions. PLL bounds can permit unrealistic estimates before alignment is checked. |
| Initial synchronization | Positive Vd within ±10% nominal peak, abs(Vq)<0.1 V, frequency within ±5%; 100 ms of consecutive calls (counted from Ts). | Confirm tolerances against sensor noise and distortion. At 325 V peak, 0.1 V q corresponds to about 0.018° under the small-angle assumption. |
| PWM before closure | PWM starts at 50%, then links the computed waveform near neutral (default ±0.01 duty). Configurable measured periods in PWM_SETTLE (default ten, zero holds indefinitely), with application PWM acknowledgement and relay request off. | Validate actual terminal voltage across open contacts and filter settling. Matching measured grid feedforward does not itself prove inverter-side voltage matching. Confirm required duration on hardware. |
| Relay integration | GRID_CONNECT waits up to 1 s for independent contact feedback. main.cpp has no relay output/feedback path and never requests connection. | Confirm pin mapping, contact delay/bounce, welded-contact detection, zero-crossing strategy and startup feedback plausibility. Never substitute the relay request for physical feedback. |
| Stop/disconnect | Both dq currents below 0.1 A allow PWM off and relay opening; combined timeout 1 s. Faults drop requests immediately. | Confirm thresholds, decay, filter energy/freewheel paths and relay opening behavior under normal and emergency shutdown. |
| Grid observer | 207–253 V RMS, 47.5–52.5 Hz, 100 ms violation delays, 1 s healthy qualification; VAC stale after 20 ms. | Confirm installation protection requirements and total filter/PLL latency. This observer is not complete islanding detection. Connected/RUN states rely on this verdict rather than continuously faulting on the strict connection-alignment test. |
| Measurements | Cached samples; freshness tracked only for VAC. Boost setters ignore nonfinite values. Invalid/nonpositive live bus samples do not replace the inverter's previous positive bus value in main.cpp. | Validate calibration, offset, bandwidth and synchronized ADC/PWM sampling. Define stale/invalid handling for all currents and voltages, bus plausibility and independent trip paths. |
| Timing and concurrency | Control task 100 us, boost cold loop every ten ticks. Background task changes shared mode, voltage target and deadtime. | Measure worst-case execution time, jitter and actual sample cadence; check safe transfer of requests and coherent telemetry. Grid-period dwell assumes calculateDuty is called once per configured Ts. |
| Hardware configuration | platformio.ini lists twist 1.4.2; src/app.ini selects uSolarVerter 0.9.0; current CMake selects uSolarVerter. | Verify resolved build configuration and actual board/shield/sensor/PWM mappings, switching frequencies, interleaving and inverter deadtime. |
| Deadtime controls | Boost setup uses the leg-2 variable for both legs (currently both 100 ns). Serial decrement can go below the declared 40 ns minimum; no explicit maximum. | Confirm gate-driver requirements and bounds. Correct these controls before using them to characterize hardware. |
| Hardware protections | Neither controller's current-reference limit is a hardware trip. No system-level thermal, PV undervoltage, bus-energy or independent peak-current protection is established by these classes. | Identify implemented hardware shutdown paths and define software limits, response times and fault-reset interlocks using the actual design. |
| FORMING, if used | Current Kp=0.001, Ti=0.001/3000 s (~0.333 us), ±30 V; voltage Kp=0.01, Ti=3 ms, ±30 A correction added to external reference. Positive Vd reference enables PWM. | Confirm the unusually short current integral time, loop separation, current limit after summation, voltage ramp and restart behavior. FORMING has no grid-following relay sequence. |
