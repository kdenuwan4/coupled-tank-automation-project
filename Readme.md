# Two-Tank Control, PLC & AI Project

Nonlinear two-tank liquid level control system, taken end to end: Simulink modeling, PID design with root locus and frequency-domain checks, Siemens TIA Portal PLC implementation, industrial communication/SCADA, and ML-based anomaly detection.

**Goal:** control the water level in Tank 2 by manipulating the pump feeding Tank 1.

**Status:** Phase 1 (Simulink modeling and control design) in progress.

---

## Project roadmap

| Phase | Folder | Status |
|---|---|---|
| 1. Simulink modeling and control design | `01-simulink-modeling/` | In progress |
| 2. PLC implementation (TIA Portal + PLCSIM) | `02-plc-tia-portal/` | Not started |
| 3. SCADA / HMI and communication (Modbus, OPC-UA, MQTT) | `03-scada-communication/` | Not started |
| 4. AI layer (anomaly detection, system identification) | `04-ai-anomaly-detection/` | Not started |
| 5. Final integrated project | `05-final-project/` | Not started |

## Repository structure

```
two-tank-control-plc-ai/
├── 01-simulink-modeling/
│   ├── TankModel.slx
│   ├── params.m
│   ├── linearize_tanks.m
│   ├── design_controller.m
│   ├── stability.m
│   ├── sys_lin.mat
│   └── figures/
├── 02-plc-tia-portal/
├── 03-scada-communication/
├── 04-ai-anomaly-detection/
├── 05-final-project/
└── README.md
```

---

## Phase 1: Simulink modeling and control design

### System description

```
Pump -> Qin -> [Tank 1, h1] -> Q1 (valve 1) -> [Tank 2, h2] -> Q2 (valve 2) -> drain
                                                  ^
                                            level sensor (h2)
```

Cascade configuration: Tank 1 drains one-way into Tank 2, which drains to atmosphere. Tank 2's level does not affect Tank 1.

### Plant model (first principles)

Mass balance:

```
A1 * dh1/dt = Qin - Q1
A2 * dh2/dt = Q1 - Q2
```

Bernoulli outflow (the source of the nonlinearity):

```
Q1 = k1 * sqrt(h1),   k1 = Cd1 * a1 * sqrt(2g)
Q2 = k2 * sqrt(h2),   k2 = Cd2 * a2 * sqrt(2g)
```

Simulink implementation: each tank is a subsystem (Sum -> 1/A gain -> Integrator -> Sqrt -> Gain, with the outflow fed back to the Sum). Tank 1's outflow `Q1` is the inflow of Tank 2.

### Parameters (`params.m`)

| Parameter | Symbol | Value |
|---|---|---|
| Tank cross-section | A1, A2 | 0.02 m^2 |
| Valve orifice area | a1, a2 | 0.5e-4 m^2 |
| Discharge coefficient | Cd1, Cd2 | 0.6 |
| Lumped outflow coefficient | k1, k2 | 1.3288e-4 m^2.5/s |
| Pump max flow | Qmax | 9e-5 m^3/s |
| Pump gain | k_pump | Qmax/10 (m^3/s per V) |

### Operating point

| Quantity | Value |
|---|---|
| Tank 2 level (target) | h2* = 0.20 m |
| Tank 1 level | h1* = 0.20 m (equal because k1 = k2) |
| Inflow / Q1 | Qin* = 5.94e-5 m^3/s |
| Equivalent pump voltage | u* = 6.60 V |

### Validation so far

- Single tank: settles at the analytically predicted level (checked with toy values k = 1, Qin = 1, h_ss = 1).
- Two-tank cascade: both tanks settle at the hand-calculated steady state; Tank 2 lags Tank 1 as expected for a cascade.
- Realistic parameters: both levels settle at 0.2 m in roughly 1200 s, matching `Qin = k1*sqrt(h)` at the operating point.
- Model started at the operating point shows flat lines (dx/dt = 0), confirming the trim values are consistent.

### Linearization (`linearize_tanks.m`)

Linearized at the operating point above using `linio` + `linearize`. States are deviations `[dh1; dh2]`, input is the deviation in pump flow, output is `dh2`.

```
A = [ -0.007428        0
       0.007428  -0.007428 ]
B = [ 50; 0 ]
C = [ 0  1 ]
D = 0
```

Notes:
- Each nonzero A entry is `k/(2*sqrt(h*)*A)`, the slope of the sqrt curve at the operating point divided by the tank area.
- A(1,2) = 0 reflects the one-way cascade.
- B(1) = 1/A1.
- Poles: -0.007428 (double), so the open loop is stable and non-oscillatory, with a time constant of about 135 s per tank.
- Validity: small deviations around h = 0.2 m only. The linear model degrades far from this point (for example near empty, where the sqrt curve is much steeper).


### Plant transfer function

`G(s) = 0.3714 / (s + 0.007428)^2`

Two identical first-order lags in series (one per tank, about 135 s each), DC gain 6731 m per (m^3/s). Poles (-0.007428, double) and DC gain were checked against hand calculation. Notably, the plant has no integrator.

### Controller design spec (Tank 2 level)

| Spec | Target |
|---|---|
| Steady-state error | 0 |
| Overshoot | <= 10% |
| Settling time (2%) | <= 400 s (open loop is about 790 s) |
| Pump command | within 0 to 10 V (to be checked once pump block is added) |

### Controller design results

| Controller | Overshoot | Settling time | Final value (setpoint = 1) |
|---|---|---|---|
| P (K = 2.8e-4) | 10.0% | not meaningful (settles at wrong value) | 0.651 (35% steady-state error) |
| PI (Kp = <your value>, Ki = Kp*0.005) | 6.0% | 636 s | 1.000 |
| PID (K = 0.03, z = 0.007428, N = 0.05) | 0% | 275 s | 1.000 |

Findings:
- On the root locus the double plant pole splits vertically, so the real part stays at -0.0074 whatever the gain. P control therefore cannot be faster than the plant (about 540 s) and, with no integrator in the plant, always leaves steady-state error.
- PI removes the steady-state error but is still limited to roughly open-loop speed. The 400 s spec needs derivative action, which is the next step.
- PID places two zeros on the plant's double pole, leaving closed-loop poles at -0.0168 and -0.0333 (both real, no overshoot). It meets all three specs, and settling is more than 2x faster than open loop.
- Equivalent standard-form gains: Kp = 4.13e-4, Ki = 1.66e-6, Kd = 2.17e-2 (Ti about 249 s, Td about 53 s, derivative filter pole at 0.05 rad/s). These are in flow-per-metre units and need rescaling for the PLC.
- Caveat: a 5 cm setpoint step demands an initial pump flow change of about 7.5e-5 m^3/s, above the roughly 3.1e-5 m^3/s of headroom before saturation. To be tested in the nonlinear model with the pump block.
- The design relies on pole-zero cancellation at the linearization point, so it may degrade when the real operating point drifts. To be tested in the disturbance test.

### Stability margins (PID, K = 0.03)

Gain margin: infinite. Phase margin: 77.7 deg at 0.0109 rad/s (`stability.m`).
The large phase margin explains the 0% overshoot. The infinite gain margin is a result of the ideal pump model and will become finite once actuator lag is added.

Gain sweep (raising K shrinks the phase margin and overshoot appears):

| K | Phase margin | Overshoot |
|---|---|---|
| 0.03 | <your value> | <your value> |
| 0.1 | <your value> | <your value> |
| 1 | <your value> | <your value> |

### Model assumptions

- The plant input is **flow** (m^3/s), not pump voltage. Pump gain and saturation are not yet in the Simulink model; they will be added before the PLC phase.
- Ideal actuator (no pump lag), ideal level sensor (no noise or delay).
- Identical valves and tanks (symmetric). Asymmetric valves are a planned extension.
- No physical level limits (overflow / empty) enforced in the integrators yet.

### Next steps
- [x] Verify poles and DC gain of `sys_lin` against hand calculation
- [x] Define the closed-loop spec (settling time, overshoot, steady-state error)
- [x] P and PI design using root locus
- [x] PID design using root locus
- [x] Stability margins with `margin()` and link to time-domain behavior
- [ ] Disturbance test (extra outflow valve opening)
- [ ] Add pump gain, saturation, and optionally pump lag
- [ ] Optional: asymmetric valves, sensor noise

---

## Progress log

| Date | Entry |
|---|---|
| 2026-10-09 | Built and validated single-tank and two-tank nonlinear Simulink models. Parameterized with `params.m`. Linearized at h2* = 0.2 m. |
| 2026-10-14 | Verified linearization (poles, DC gain). Set design spec. Designed P (35% steady-state error) and PI (0% error, 6% overshoot, 636 s settling). Speed limited by plant poles; PID next. |
| 2026-10-14 | PID designed by pole-zero cancellation: 0% overshoot, 275 s settling, 0 steady-state error. Flagged pump saturation risk for realistic steps. |
| 2026-10-14 | PID margins: PM 77.7 deg, GM infinite. Gain sweep shows overshoot growing as phase margin shrinks (`stability.m`). |
## How to run

1. Open MATLAB in `01-simulink-modeling/`.
2. Run `params.m` to load parameters.
3. Open `TankModel.slx` and simulate, or run `linearize_tanks.m` to regenerate `sys_lin.mat`.

Requires MATLAB, Simulink, and Simulink Control Design.

## Tools

MATLAB / Simulink, Siemens TIA Portal + PLCSIM (planned), Python (planned for the AI phase).V