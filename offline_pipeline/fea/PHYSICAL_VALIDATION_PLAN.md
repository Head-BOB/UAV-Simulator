# Physical Frame Prototype Bench Validation Plan
**Author:** Dev 2 (Dev A) — FEA & Structural Surrogate Owner
**Phase:** 2 — Advanced Multiphysics

## 1. Purpose & Objective
To complete the project-wide Three-Step Validation Ladder:
`1. Literature / Baseline Theory -> 2. FEA Offline Simulation / GPR Surrogate -> 3. Physical Hardware Testing`

This document details the experimental protocols required once a physical prototype frame is manufactured to validate the structural surrogate model predictions against physical measurements.

---

## 2. Experimental Protocols

### Test 1: Quasi-Static 3-Point Bending Test (Flexural Rigidity)
- **Objective:** Measure experimental deflection and bending stiffness of the carbon composite arm.
- **Setup:**
  - Secure the central frame chassis to a rigid steel test fixture.
  - Apply vertical point displacement at the motor mount center using a calibrated universal tensile/compression testing machine (e.g., Instron) equipped with a 500 N load cell.
  - Ramp load from 0 N to 45 N (corresponding to an ~8G maneuver on a single arm).
- **Pass/Fail Acceptance Criteria:**
  - Arm vertical deflection must match the FEA beam model within **$\pm 5.0\%$**.

### Test 2: Arm Root Fillet Strain Gauge Instrumentation
- **Objective:** Directly measure peak mechanical stress in the region of highest predicted stress concentration.
- **Setup:**
  - Adhere two uniaxial electrical resistance strain gauges (120 Ohm, gauge factor ~2.0) at the arm root radius adjacent to the inner mounting holes.
  - Connect gauges in a quarter-bridge Wheatstone configuration to a high-speed data acquisition unit (DAQ).
  - Apply static loading and record microstrain ($\mu\varepsilon$).
  - Calculate physical normal stress:
    $$\sigma_{\text{measured}} = E_1 \cdot \varepsilon_{\text{measured}}$$
- **Pass/Fail Acceptance Criteria:**
  - Measured stress vs. Surrogate predicted stress must agree within **$\le 5.0\%$** across the validated load range.

### Test 3: Electrodynamic Shaker Dynamic Resonance
- **Objective:** Validate the fundamental bending natural frequency ($f_n$) and structural damping ratio ($\zeta$).
- **Setup:**
  - Mount frame to an electrodynamic shaker table with a triaxial accelerometer at the motor mount.
  - Perform a sine-sweep from 50 Hz to 650 Hz at 0.5G excitation.
  - Identify peak acceleration response frequency ($f_n$).
- **Pass/Fail Acceptance Criteria:**
  - Measured natural frequency must match theoretical $f_n \approx 119\text{ Hz}$ within **$\pm 7.0\%$**.
  - Calibrate damping ratio $\zeta$ in `run_baseline_case.py` to match the physical half-power bandwidth measurement.
