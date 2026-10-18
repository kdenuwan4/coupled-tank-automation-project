# Two-Tank Level Control: Simulink, PLC and AI

An end-to-end control and automation project built around one nonlinear plant, a two-tank liquid level system. The same system is modeled and controlled in Simulink, implemented on a Siemens PLC, connected through industrial communication to SCADA/HMI, and extended with machine-learning-based anomaly detection.

**Control objective:** regulate the water level in Tank 2 by manipulating the pump that feeds Tank 1.

## Highlights

- Nonlinear plant built from first principles (mass balance and Bernoulli outflow), not a fitted transfer function.
- Linearized at an operating point; the resulting model is checked against hand calculation.
- PID designed against explicit specs using root locus, then verified with stability margins and on the nonlinear model with a saturating pump.
- Open-loop settling is about 790 s. The PID meets 0% overshoot and 275 s settling on the linear model.

| Phase | Folder | Status |
|---|---|---|
| 1. Modeling and control design (MATLAB/Simulink) | `01-simulink-modeling/` | In progress |
| 2. PLC implementation (TIA Portal + PLCSIM) | `02-plc-tia-portal/` | Planned |
| 3. SCADA/HMI and communication (Modbus, OPC-UA, MQTT) | `03-scada-communication/` | Planned |
| 4. AI layer (anomaly detection, system identification) | `04-ai-anomaly-detection/` | Planned |
| 5. Integrated final project | `05-final-project/` | Planned |

Dated history lives in [`docs/progress-log.md`](docs/progress-log.md). Open work is tracked in the repository's Issues.

## Repository structure

```
two-tank-control-plc-ai/
├── 01-simulink-modeling/
│   ├── TankModel.slx          nonlinear plant + closed-loop PID model
│   ├── params.m               physical parameters, operating point, PID values
│   ├── linearize_tanks.m      linearization at the operating point
│   ├── design_controller.m    P, PI and PID design on the linear plant
│   ├── stability.m            margins and gain sweep
│   ├── sys_lin.mat            saved linearized model
│   └── figures/
├── 02-plc-tia-portal/
├── 03-scada-communication/
├── 04-ai-anomaly-detection/
├── 05-final-project/
├── docs/
│   └── progress-log.md
└── README.md
```

## Getting started

Requires MATLAB, Simulink and Simulink Control Design.

1. Open MATLAB in `01-simulink-modeling/`.
2. Run `params.m` to load parameters.
3. Open `TankModel.slx` to simulate the nonlinear plant and closed loop.
4. Run `linearize_tanks.m` to regenerate `sys_lin.mat`, then `design_controller.m` and `stability.m` for the control analysis.

---

## The system

```
Pump -> Qin -> [Tank 1, h1] -> Q1 (valve 1) -> [Tank 2, h2] -> Q2 (valve 2) -> drain
                                                  ^
                                            level sensor (h2)
```

Cascade configuration: Tank 1 drains one-way into Tank 2, which drains to atmosphere. Tank 2's level does not affect Tank 1.

## Phase 1: Modeling and control design

### Plant model

Mass balance:

```
A1 * dh1/dt = Qin - Q1
A2 * dh2/dt = Q1 - Q2
```

Bernoulli outflow, the source of the nonlinearity:

```
Q1 = k1 * sqrt(h1),   k1 = Cd1 * a1 * sqrt(2g)
Q2 = k2 * sqrt(h2),   k2 = Cd2 * a2 * sqrt(2g)
```

Each tank is a Simulink subsystem (Sum, 1/A gain, Integrator, Sqrt, Gain, with the outflow fed back to the Sum). Tank 1's outflow is Tank 2's inflow. All constants come from `params.m`.

| Parameter | Symbol | Value |
|---|---|---|
| Tank cross-section | A1, A2 | 0.02 m^2 |
| Valve orifice area | a1, a2 | 0.5e-4 m^2 |
| Discharge coefficient | Cd1, Cd2 | 0.6 |
| Lumped outflow coefficient | k1, k2 | 1.3288e-4 |
| Pump max flow | Qmax | 9e-5 m^3/s |

The model was validated by checking steady states against hand-calculated values: with unit constants, and with the realistic parameters, where both tanks settle at 0.2 m in about 1200 s.

### Operating point

| Quantity | Value |
|---|---|
| Tank 2 level (target) | 0.20 m |
| Tank 1 level | 0.20 m (equal because k1 = k2) |
| Pump flow Qin* | 5.94e-5 m^3/s |
| Equivalent pump voltage | 6.60 V (0 to 10 V range) |

### Linearization

Linearized at the operating point with `linio` and `linearize`. States are deviations `[dh1; dh2]`, the input is the deviation in pump flow, the output is `dh2`.

```
A = [ -0.007428        0
       0.007428  -0.007428 ]
B = [ 50; 0 ]    C = [ 0  1 ]    D = 0
```

- Each nonzero A entry is `k / (2*sqrt(h*) * A)`, the slope of the sqrt curve at the operating point divided by the tank area.
- A(1,2) = 0 reflects the one-way cascade. B(1) = 1/A1.
- Resulting transfer function: `G(s) = 0.3714 / (s + 0.007428)^2`. It is two identical first-order lags (about 135 s each) with DC gain 6731, and no integrator.
- Poles and DC gain match hand calculation.
- The model is valid for small deviations around 0.2 m and degrades far from it, for example near empty where the sqrt curve is much steeper.

### Controller design

**Specification (Tank 2 level)**

| Spec | Target |
|---|---|
| Steady-state error | 0 |
| Overshoot | <= 10% |
| Settling time (2%) | <= 400 s |
| Pump command | within 0 to Qmax |

**Results on the linear plant (unit step)**

| Controller | Overshoot | Settling time | Final value |
|---|---|---|---|
| P (K = 2.8e-4) | 10.0% | n/a | 0.651 (35% steady-state error) |
| PI (Kp = 0.000186, Ki = 0.005 * Kp) | 6.0% | 636 s | 1.000 |
| PID (K = 0.03) | 0% | 275 s | 1.000 |

**Why each step was needed**

- On the root locus the double plant pole splits vertically, so its real part stays at -0.0074 for any gain. A P controller can therefore never be faster than the plant, and since the plant has no integrator it always leaves steady-state error.
- PI removes the steady-state error but stays limited to roughly open-loop speed.
- PID places its two zeros on the plant's double pole (pole-zero cancellation) and adds a derivative filter at 0.05 rad/s. Closed-loop poles land at -0.0168 and -0.0333, both real, which meets all three specs.
- Equivalent standard-form gains: Kp = 4.13e-4, Ki = 1.66e-6, Kd = 2.17e-2, so Ti is about 249 s and Td about 53 s. These are in flow-per-metre units and must be rescaled for the PLC.

### Stability margins

For the PID at K = 0.03: gain margin infinite, phase margin 77.7 deg at 0.0109 rad/s (`stability.m`). The large phase margin explains the 0% overshoot. The infinite gain margin follows from the ideal pump model and would become finite with actuator lag.

Raising the gain shrinks the phase margin and overshoot appears:

| K | Phase margin | Overshoot |
|---|---|---|
| 0.03 | 77.7 deg | 0% |
| 0.1 | 57.8 deg | 10.7% |
| 1 | 20.8 deg | 55.5% |

### Validation on the nonlinear model

The PID is closed around `TankModel.slx` with the operating-flow offset `Qin_ss` added to the controller output and a pump saturation block (0 to Qmax).

| Setpoint step | Behavior |
|---|---|
| 0.20 to 0.22 m (2 cm) | Settles about 275 s after the step, matching the linear prediction. The pump stays inside its limit. |
| 0.20 to 0.25 m (5 cm) | The pump saturates and the integrator winds up. Overshoot 6%, settling time about 650 s after the step (2% band), against 275 s predicted by the linear design. Meets the overshoot spec but not the settling-time spec. |

The linear design therefore holds only for steps small enough to stay inside the pump limit. For larger steps an anti-windup scheme is required.

| Setpoint step | Behavior |
|---|---|
| 0.20 to 0.22 m (2 cm) | Settles in about 246 s after the step (2% band), close to the 275 s predicted by the linear design. The pump stays inside its limit. |
| 0.20 to 0.25 m (5 cm) | The pump saturates. With clamping anti-windup, the response settles about 375 s after the step with no overshoot. Without anti-windup (plain Transfer Fcn controller) the same step took about 650 s and overshot by 6%. |

Controller: Simulink PID block (parallel form, filtered derivative, output limits matching the pump range, clamping anti-windup). All specs are met for both step sizes.

**Disturbance rejection.** An extra outflow valve on Tank 2 (25% of valve 2's coefficient, `kd_extra` in `params.m`) opens at t = 600 s while the setpoint is held at 0.2 m. Tank 2 dips by 31 mm (to 0.1687 m) and returns to the 2% band after about 496 s. The integrator raises the pump flow to 7.43e-5 m^3/s, matching the hand-calculated sum of both outflows, and Tank 1 settles near 0.31 m (calculated 0.3125 m; a linearized model would predict 0.300 m). The pump does not saturate. Recovery is slower than setpoint tracking (about 246 s) because the disturbance enters downstream of the pole-zero cancellation, so the plant's slow pole (135 s time constant) appears in the response. The plant poles moved to about -0.0059 and -0.0093 at the new operating point, yet the controller still recovered, consistent with the 77.7 deg phase margin.

## Assumptions and known limitations

- The plant input is pump **flow** (m^3/s). A voltage-to-flow pump gain is not in the model yet and is needed for the PLC phase.
- Ideal actuator (no pump lag) and ideal level sensor (no noise or delay).
- Identical tanks and valves. Asymmetric valves would split the two poles.
- Physical level limits (empty and overflow) are not enforced in the integrators.
- Disturbance recovery (about 496 s) is slower than setpoint tracking because the PID cancels the plant's slow poles, which a disturbance at Tank 2 bypasses. A design that does not rely on cancellation would recover faster.


## Tools

MATLAB / Simulink and Simulink Control Design. Siemens TIA Portal with PLCSIM and Python are planned for later phases.
