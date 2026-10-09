# Physical Constants & Vehicle Configuration
Per WP-2 of the Phase 2 Completion Plan, every physical constant must have a documented source. Numbers with no source are marked **ASSUMED** and will block the exit gate from opening.

| Parameter | Value | Unit | Source | Date | Verified By |
| :--- | :--- | :--- | :--- | :--- | :--- |
| Mass | 1.5 | kg | ASSUMED (Placeholder for actual scale weight) | 2026-10 | Lead Architect |
| Inertia (Ixx, Iyy, Izz) | [0.015, 0.015, 0.025] | kg*m^2 | ASSUMED (CAD estimation) | 2026-10 | Lead Architect |
| Motor Positions | [+/-0.2, +/-0.2, 0.0] | m | ASSUMED (CAD estimation) | 2026-10 | Lead Architect |
| Thrust Coefficient (C_T) | 0.11 | - | ASSUMED (Pending UIUC WP-6) | 2026-10 | Dev B |
| Yaw torque coefficient (kappa, `torque_coeff`) | 0.005 | N*m per N of thrust | ASSUMED (Pending UIUC WP-6). Unit corrected 2026-10-09: the mixer multiplies it by thrust in N. If 0.005 was meant as C_Q, kappa = C_Q * D / C_T = 0.005 * 0.25 / 0.11 = 0.011364 m (B-3, B-5) | 2026-10 | Dev B |
| Max RPM | 10000.0 | rpm | ASSUMED (Manufacturer Spec pending bench test) | 2026-10 | Dev C |
| Hover throttle | 0.5015112082212391 (1.5 kg) | - | DERIVED at load: (60/max_rpm)*sqrt(m*g/(4*C_T*rho0*D^4)), rho0 = ISA sea level; inherits ASSUMED from mass, C_T, D, max RPM | 2026-10 | Not human-checked |
