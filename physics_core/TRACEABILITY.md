# Tier F Software Assurance - Traceability Matrix

Per the Phase 2 Core Engineering Standards & Safety Classification Framework (Section 3.5), this document maps flight-candidate logic requirements to their verifying tests. Any PR modifying Tier F modules MUST state which row(s) of this table it affects.

| Requirement | Verified By | Status |
| :--- | :--- | :--- |
| Free-fall position matches analytical `0.5 * g * t^2` within 1e-5m | `integrator::tests::test_c1_free_fall_match` | UNVERIFIED |
| Hover equilibrium maintains position and velocity exactly at zero | `integrator::tests::test_c2_hover_equilibrium` | PASS |
| Integration maintains quaternion normalization to 1.0 (1e-6) | `integrator::tests::test_c3_quaternion_norm` | UNVERIFIED |
| Zero-velocity inputs produce exactly zero aerodynamic drag | `aero::tests::test_c1_zero_velocity` | PASS |
| Aerodynamic drag strictly opposes the velocity vector | `aero::tests::test_c2_drag_opposes_motion` | PASS |
| Aerodynamic drag scales with the square of the velocity | `aero::tests::test_c3_drag_scales_with_velocity_squared` | PASS |
| Standard ISA density calculation matches sea-level reference (1.225) | `aero::tests::test_b1_sea_level_density` | UNVERIFIED |
| Air density decreases monotonically with altitude | `aero::tests::test_b3_density_decreases_with_altitude` | PASS |
| Zero throttle input produces exactly zero thrust | `aero::tests::test_d1_zero_throttle` | PASS |
| Out-of-bounds throttle inputs (< 0.0 or > 1.0) are strictly clamped | `aero::tests::test_d5_throttle_clamping` | PASS |