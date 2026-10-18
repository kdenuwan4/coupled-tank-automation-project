# Progress log

Dated history of the project, newest entries at the bottom. The README describes the project as it stands; this file records how it got there.

| Date | Entry |
|---|---|
| 2026-10-09 | Built and validated single-tank and two-tank nonlinear Simulink models. Parameterized with `params.m`. Linearized at h2* = 0.2 m. |
| 2026-10-14 | Verified linearization (poles, DC gain). Set design spec. Designed P (35% steady-state error) and PI (0% error, 6% overshoot, 636 s settling). Speed limited by plant poles. |
| 2026-10-14 | PID designed by pole-zero cancellation: 0% overshoot, 275 s settling, 0 steady-state error. Flagged pump saturation risk for realistic steps. |
| 2026-10-14 | PID margins: PM 77.7 deg, GM infinite. Gain sweep shows overshoot growing as phase margin shrinks (`stability.m`). |
| 2026-10-14 | PID closed on nonlinear model: 2 cm setpoint step settles in about 275 s, matching linear prediction. |
| 2026-10-14 | 5 cm step on nonlinear model: pump saturation and integrator windup cause overshoot and slower settling. Anti-windup needed. |
| 2026-10-18 | Measured 5 cm step on nonlinear model: 6% overshoot, about 650 s settling (vs 275 s linear prediction). Created GitHub issues for anti-windup, disturbance test, pump gain block and extensions. |
| 2026-10-18 | Replaced Transfer Fcn with PID block (clamping anti-windup, output limits). 5 cm step: 375 s settling, no overshoot (was 650 s, 6%). Re-measured 2 cm step with 2% band: 246 s. |
| 2026-10-18 | Disturbance test on nonlinear model: extra valve on Tank 2 at t = 600 s. h2 dips 31 mm, recovers in about 496 s, pump settles at 7.43e-5 (matches hand calculation). h1 rises to about 0.31 m. |