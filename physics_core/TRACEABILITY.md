| Requirement | Verified By | Status |
| :--- | :--- | :--- |
| Constant velocity 10 minutes at 0.002 s | `integrator::tests::test_c4_constant_velocity` | PASS |
| Free-fall position matches analytical `0.5 * g * t^2` within 1e-9m | `integrator::tests::test_c1_free_fall_match` | PASS |
| Spin at constant rate about one axis | `integrator::tests::test_c5_spin_constant_rate` | PASS |
| Same run with dt and dt/2 error ratio near 16 | `integrator::tests::test_c6_rk4_order` | PASS |
| ISA at 0, 500, 1000, 5000, 11000 m | `aero::tests::test_b1_isa_values` | PASS |
| Hover equilibrium maintains position and velocity exactly at zero | `integrator::tests::test_c2_hover_equilibrium` | PASS |
| Zero-velocity inputs produce exactly zero aerodynamic drag | `aero::tests::test_c1_zero_velocity` | PASS |
| Aerodynamic drag strictly opposes the velocity vector | `aero::tests::test_c2_drag_opposes_motion` | PASS |
| Aerodynamic drag scales with the square of the velocity | `aero::tests::test_c3_drag_scales_with_velocity_squared` | PASS |
| Zero throttle input produces exactly zero thrust | `aero::tests::test_d1_zero_throttle` | PASS |
| Out-of-bounds throttle inputs (< 0.0 or > 1.0) are strictly clamped | `aero::tests::test_d5_throttle_clamping` | PASS |