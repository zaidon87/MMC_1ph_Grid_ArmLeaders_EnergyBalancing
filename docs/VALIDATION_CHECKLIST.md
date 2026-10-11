# Validation checklist (not yet executed)

This is a proposed simulation verification procedure. **No PASS/FAIL data or waveforms were generated in preparing this repository.**

## A. Load and structural checks

- [ ] MATLAB R2024a + Simulink + Specialized Power Systems available.
- [ ] Original SLX opens without missing libraries or compile errors.
- [ ] Both leaders, all 16 local controllers and `powergui` are present.
- [ ] Model callbacks initialize all expected parameters.
- [ ] Fixed step `Ts=10 µs`, arm-leader `Ts_ctrl=100 µs` and stop time 1 s remain unchanged.

## B. Baseline: `Pstar=1500 W`, `Qstar=0 var`

- [ ] Confirm first 150 ms reference hold, then ramp behavior.
- [ ] Plot PCC voltage/current, measured P and Q, and both AC-current references.
- [ ] Confirm expected steady-state P and Q trend after startup; quantify error, settling and overshoot from simulated data.
- [ ] Plot circulating current `0.5*(i_u+i_l)`, its 100 Hz component and arm-reference tracking.
- [ ] Plot upper/lower stored energy and individual capacitor voltages; compare against nominal reference and quantify spread.
- [ ] Verify all modulation indices and half-bridge gate values respect 0–1 and complementary gate constraints.

## C. Perturbation tests

- [ ] Change active power reference in a separate test (document values and timestamps).
- [ ] Change reactive power reference in a separate test.
- [ ] Introduce a **documented small initial capacitor voltage imbalance** and quantify convergence and energy balance.
- [ ] Compare case with/without local balancing correction, using controlled and otherwise identical scenarios.
- [ ] Compare case with/without 100 Hz current-compensation term, if exposing the flag in the embedded block.
- [ ] Test modest grid-voltage disturbance and confirm the PLL and limits operate as intended.

## D. Communication and implementation boundaries

- [ ] Confirm inter-arm W exchange uses an explicit 100 µs Unit Delay with nominal initial state.
- [ ] Document that neighbor communication is vector-based ring emulation, not actual peer-to-peer hardware.
- [ ] Before hardware deployment, separately specify sampling jitter, link latency, packet loss, saturation, dead-time, startup/shutdown and fault protections.
- [ ] Do **not** claim real-time/OPAL-RT validation without an actual deployment and traceable measurements.

## Report template

| Quantity | Method | Observed | Target/criterion | Result |
| --- | --- | --- | --- | --- |
| P tracking | Average P after settling | Not tested | Set before test | Pending |
| Q tracking | Average Q after settling | Not tested | Set before test | Pending |
| Capacitor imbalance | Max/mean/range of Vc | Not tested | Set before test | Pending |
| Energy difference | `W_upper-W_lower` | Not tested | Set before test | Pending |
| Circulating current | RMS / 100 Hz component | Not tested | Set before test | Pending |
| Control saturation | Duty and current clips | Not tested | No invalid values | Pending |

Record MATLAB release, model SHA-256, any changed parameters, scope exports, scripts and numerical results with each study.
