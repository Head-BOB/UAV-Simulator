# Traceability matrix

Checked by `tools/check_traceability.py` (CI). Rules enforced by the script: every row names exactly one
existing test, every test in `cargo test -- --list` is named in exactly the table below, every row has a
pass rule (tolerance), and Status is PASS, UNVERIFIED or FAIL.

Status meaning: PASS = the test asserts the stated pass rule and the pass rule meets the plan.
UNVERIFIED = the test exists but is weaker than the plan, has an underived expected value, or relies on
an open question. Source `unmapped` = no requirement ID exists yet. Hazard IDs (HZ-nn) are defined in `docs/hazard-matrix.md`.

| Requirement | Source | Verified By | Pass rule (tolerance) | Status | Notes |
| :--- | :--- | :--- | :--- | :--- | :--- |
| Constant velocity for 10 minutes at dt 0.002 s | WP-1 | `integrator::tests::test_c4_constant_velocity` | x = 1 m/s * 600 s = 600 m, abs 1e-9 m | PASS | meets WP-1 |
| Free fall for 10 s matches 0.5 g t^2 | WP-1 | `integrator::tests::test_c1_free_fall_match` | 0.5 * 9.80665 * 10^2 = 490.3325 m, abs 1e-9 m | PASS | meets WP-1 |
| Constant-rate spin: angle = rate * time, quaternion norm 1 | WP-1 | `integrator::tests::test_c5_spin_constant_rate` | norm abs 1e-12; angle 1.0 rad abs 1e-12 | PASS | meets WP-1 |
| RK4 is 4th order: dt vs dt/2 error ratio near 16 | WP-1 | `integrator::tests::test_c6_rk4_order` | per-run abs 1e-13 rad vs closed form; ratio 16 +/- 0.05 | PASS | meets WP-1 |
| Hover equilibrium keeps position and velocity | unmapped | `integrator::tests::test_c2_hover_equilibrium` | exact (assert_eq) | UNVERIFIED | zero force and torque, one step at dt 0.1 s through the legacy wrapper; no thrust and gravity balance |
| ISA density at 0, 500, 1000, 5000, 11000 m | WP-1 | `aero::tests::test_b1_isa_values` | relative 1e-4 | UNVERIFIED | 5000 m expects 0.73611, plan says 0.7364 (human item B-2); no derivation comment |
| ISA density at 500 m | unmapped | `aero::tests::test_b2_altitude_500m` | abs 0.005 kg/m3 (about 0.4 %) | UNVERIFIED | looser than the WP-1 1e-4 relative rule; no derivation |
| Density decreases with altitude | unmapped | `aero::tests::test_b3_density_decreases_with_altitude` | strict inequality at 0, 500, 1000 m | PASS | |
| ISA valid only 0 to 11000 m, Err outside | WP-1 | `aero::tests::test_b4_troposphere_limit` | Err at 12000 m; 0 < rho < 1.225 at 11000 m | UNVERIFIED | altitude below 0 m not tested |
| Zero velocity gives exactly zero drag | unmapped | `aero::tests::test_c1_zero_velocity` | exact [0, 0, 0] | PASS | |
| Drag opposes velocity | unmapped | `aero::tests::test_c2_drag_opposes_motion` | sign of X component only | UNVERIFIED | one axis, no collinearity check |
| Drag scales with velocity squared | unmapped | `aero::tests::test_c3_drag_scales_with_velocity_squared` | abs 0.1 N | UNVERIFIED | tolerance not justified |
| Drag differs per body axis | unmapped | `aero::tests::test_c4_different_axes_different_drag` | strict inequality Z > X | PASS | drag constants are ASSUMED (docs/constants.md) |
| Zero throttle gives exactly zero thrust | unmapped | `aero::tests::test_d1_zero_throttle` | exact 0.0 | PASS | |
| Thrust scales with throttle squared | unmapped | `aero::tests::test_d2_thrust_scales_with_throttle_squared` | abs 0.1 N | UNVERIFIED | tolerance not justified |
| Throttle outside 0 to 1 is clamped | unmapped | `aero::tests::test_d5_throttle_clamping` | exact equality at -0.5 and 1.5 | PASS | |
| Mixer: positive roll gives positive roll torque | WP-5.3, HZ-04 | `mixer::tests::test_mixer_roll_positive` | sign of torque X (shipped config) | PASS | sign only; hand calc +0.4 N*m at T = 10 tau; convention vs real airframe unconfirmed (B-3) |
| Mixer: positive pitch gives positive pitch torque | WP-5.3, HZ-04 | `mixer::tests::test_mixer_pitch_positive` | sign of torque Y (shipped config) | PASS | sign only; convention vs real airframe unconfirmed (B-3) |
| Mixer: positive yaw gives positive yaw torque | WP-5.3, HZ-04 | `mixer::tests::test_mixer_yaw_positive` | sign of torque Z (shipped config) | PASS | yaw mix sign fixed in item 7; magnitude in the hand-calc rows below; convention unconfirmed (B-3) |
| Mixer: equal thrust gives zero torque | WP-5.3, HZ-04 | `mixer::tests::test_mixer_equal_thrust_zero_torque` | exact [0, 0, 0] (shipped config) | PASS | now uses the shipped config |
| Mixer: yaw +0.5 at throttle 0.5 gives +0.02 N*m, roll and pitch 0 | WP-5.3, HZ-04 | `mixer::tests::test_mixer_yaw_positive_hand_calc` | tau abs 1e-15; torque abs 1e-12; 0.005 * (6-4-4+6) = 0.02 | PASS | hand calc in test comment; kappa 0.005 ASSUMED, spin per motor TODO(HUMAN-VERIFY) (B-3) |
| Mixer: yaw -0.5 gives -0.02 N*m | WP-5.3, HZ-04 | `mixer::tests::test_mixer_yaw_negative_hand_calc` | tau abs 1e-15; torque Z abs 1e-12; 0.005 * (4-6-6+4) = -0.02 | PASS | hand calc in test comment |
| Mixer: yaw sign survives saturation (throttle 1.0, yaw 0.5 gives +0.01 N*m) | WP-5.3, HZ-04 | `mixer::tests::test_mixer_yaw_saturated_keeps_sign` | tau abs 1e-15; torque Z abs 1e-12; 0.005 * (10-9-9+10) = 0.01 | PASS | clamp keeps the sign for this case only |
| Mixer clamps every motor to [0, 1] | WP-5.3 | `mixer::tests::test_mixer_clamps_every_motor_to_unit_range` | abs 1e-6 (f32 inputs); 5 hand-calculated cases in the test comment | PASS | |
| Impact inputs for v = (3, 0, 4): speed 5, sink 4, angle atan(0.75) | WP-8.6 | `impact::tests::test_impact_inputs_3_4_5_triangle` | abs 1e-15 on speed, sink and angle 0.6435011087932844 rad | PASS | hand calc in test; input format TODO(HUMAN-VERIFY: WP-8, B-11) |
| Impact angle is 0 for a vertical drop | WP-8.6 | `impact::tests::test_impact_inputs_vertical_drop_angle_zero` | speed 2 abs 1e-15; angle 0 abs 1e-15 | PASS | |
| Impact angle 45 deg uses the east component; speed 5 sqrt(2) | WP-8.6 | `impact::tests::test_impact_inputs_45_degrees_uses_east_component` | angle pi/4 abs 1e-15; speed 7.0710678118654752 abs 1e-15 | PASS | |
| Impact orientation passes through exactly | WP-8.6 | `impact::tests::test_impact_inputs_orientation_passes_through_exactly` | exact (values exact in binary) | PASS | |
| Level or ascending velocity gives NotDescending | WP-8.6 | `impact::tests::test_impact_inputs_not_descending` | 3 velocity cases give Err(NotDescending) | PASS | |
| NaN/Inf velocity or orientation gives NonFinite | WP-8.6, HZ-02 | `impact::tests::test_impact_inputs_non_finite` | 3 cases give Err(NonFinite) | PASS | |
| Ground crossing truth table (prev < ground <= next) | WP-8.6 | `impact::tests::test_ground_contact_truth_table` | 6 exact boolean cases | PASS | |
| Ground crossing honours the ground height | WP-8.6 | `impact::tests::test_ground_contact_uses_ground_height` | 2 exact boolean cases at down = 10 m | PASS | |
| Ground crossing rejects NaN/Inf | WP-8.6 | `impact::tests::test_ground_contact_non_finite` | 3 cases give Err(NonFinite) | PASS | |
| Hook queries the model once, with the post-step velocity | WP-8.6 | `impact::tests::test_hook_calls_model_once_with_post_step_velocity` | Ok(Some(1)); 1 call; sink 4 abs 1e-15 | PASS | |
| Hook does not touch the model without a crossing | WP-8.6 | `impact::tests::test_hook_does_not_call_model_without_crossing` | Ok(None) twice; 0 calls | PASS | |
| Hook fires once over a sequence of steps | WP-8.6 | `impact::tests::test_hook_once_per_crossing_over_a_sequence` | 4 steps, 1 call | PASS | |
| Hook errors leave the model untouched | WP-8.6 | `impact::tests::test_hook_error_leaves_model_untouched` | Err(NotDescending), Err(NonFinite); 0 calls | PASS | |
| Mixer yaw sign table equals shipped motor_spin; shipped positions are FR, FL, RR, RL | WP-5.3, WP-5.4, HZ-04 | `mixer::tests::test_mixer_yaw_table_matches_shipped_config` | exact sign arrays | PASS | guards the index order assumption |
| FFI step with positive yaw gives positive yaw rate and ~0 roll/pitch rate | WP-5.3, HZ-04 | `tests::test_ffi_step_positive_yaw_gives_positive_yaw_rate` | omega_z > 0; omega_x, omega_y abs 1e-9 rad/s | PASS | shipped config, one step at dt 0.002 s, no models |
| Zero throttle cools to ambient | WP-3, HZ-07 | `thermal::tests::test_thermal_zero_throttle_long_time` | abs 1e-6 C vs ambient after 10000 s | PASS | independent physical end state |
| Fixed power, no airflow, short time | WP-3, HZ-07 | `thermal::tests::test_thermal_fixed_power_no_airflow_short_time` | abs 1e-6 C | UNVERIFIED | expected value reuses the model formula; derivation of 3850.3125 W added as a comment (A12). The plan rule `rise = P t / C within 1e-6` is checked by the no-cooling test below (h overridden to 0); this test keeps the real h(0) = 10 and still reuses the model formula |
| Fixed power, no cooling (h = 0), 1 s: rise = P t / C | WP-3, HZ-07 | `thermal::tests::test_thermal_no_cooling_rise_equals_p_t_over_c` | abs 1e-6 C; 3850.3125 W / 50 J/K = 77.00625 C (T = 97.00625 C); 2 x 0.5 s equals 1 x 1 s; 50 C start gives 127.00625 C | PASS | hand calc in test comment; goes through the inner `step_motor_temps` with h = 0 (production always has h >= 10); C = 50 J/K and the electrical constants are ASSUMED (B-4) |
| Steady-state temperature at fixed power | WP-3, HZ-07 | `thermal::tests::test_thermal_steady_state_fixed_power` | abs 0.1 C | UNVERIFIED | expected value reuses model constants and formula (item A12) |
| Thermal update independent of step size | WP-3, HZ-07 | `thermal::tests::test_thermal_dt_invariance` | abs 0.01 C | UNVERIFIED | compares 0.002 with 2 x 0.001; the plan case is `test_thermal_dt_0p002_vs_0p0002_one_second` (A12) |
| Power ratio at throttle 0.5 vs 1.0 (rpm proportional) | WP-3, HZ-07 | `thermal::tests::test_thermal_power_ratio_half_vs_full_throttle` | ratio 0.25 abs 1e-12; steady T 11648.125 and 2927.03125 C abs 1e-9 | PASS | hand calc in test comment; constants ASSUMED (B-4); 5 mutations caught |
| Thermal 1 s at dt 0.002 vs 0.0002 matches closed form | WP-3, HZ-07 | `thermal::tests::test_thermal_dt_0p002_vs_0p0002_one_second` | runs agree < 0.01 C (plan) and < 1e-8 C; each vs closed form 43.19820615503639 C abs 1e-8 | PASS | closed-form derivation in test comment; Euler mutant caught |
| FFI step rejects null pointers | unmapped | `tests::test_ffi_null_pointers` | exact error code 1 for each of 5 pointers | PASS | step only; load/query/unload/reset have their own rows (item A11) |
| FFI step rejects NaN, zero, negative and over-large dt | WP-4.3, HZ-03 | `tests::test_ffi_invalid_dt` | exact error code 7 for NaN, 0, -0.002, 0.1 | PASS | |
| FFI step catches NaN in state | WP-4.4 | `tests::test_ffi_nan_propagation_catch` | exact error code 8 and physics_fault = 1 | PASS | input state is already NaN, so rollback is not observable here; rollback and latching are covered by `test_ffi_fault_latch_sticky_until_caller_clears` |
| Fault latch: rollback on NaN/Inf, sticky for later steps, only the caller clears it | WP-4.4, WP-11.4 | `tests::test_ffi_fault_latch_sticky_until_caller_clears` | code 8; thermal == [20; 4] exactly after rollback; latch and telemetry fault == 1 for 5 later steps (code 0, finite state); 0 after caller clears | PASS | hand calc in test comment (a = F/0 = -inf); state restore itself is redundant by construction and unobservable; data after a latch is invalid |
| FFI step rejects a null fault latch | WP-4.4 | `tests::test_ffi_null_fault_latch` | exact error code 1, state unchanged | PASS | |
| No models: fea_source 0, aero_source 1, margin NaN, stub_loaded 0 | WP-11.1, WP-11.2, WP-7 D.5, HZ-02 | `tests::test_ffi_step_no_models_reports_sources` | exact codes; margin is NaN | PASS | handle present but query failed, and a stub handle reaching `stub_loaded`, need a real ONNX session (not tested) |
| stub_loaded is 1 if either handle is a stub | WP-0.4, WP-11.1, HZ-01 | `tests::test_stub_loaded_flag_truth_table` | exact for all 4 input pairs | PASS | pure helper only |
| FFI interface version is 12 | WP-4.5 | `tests::test_ffi_interface_version` | exact 12 | PASS | C++ constant is checked against Rust by `test_cpp_expected_version_matches_rust`; the UE5 plugin build itself is NOT done (BLOCKED-HUMAN) |
| Wind layout: `WindInput` is 3 x f64, 24 bytes, align 8, offset 0 | unmapped (Unreal demo task, wind) | `types::tests::test_wind_input_layout` | size 24, align 8, offset 0, still() is all zeros | PASS | also a compile-time assert in `types.rs` and static_asserts in `DroneFfiLayout.h` (g++ syntax check) |
| Zero wind leaves the air-relative velocity bit-identical (incl. -0.0, subnormal) | unmapped (Unreal demo task, wind) | `aero::tests::test_wind_zero_wind_velocity_is_bit_identical` | `to_bits()` equal for 4 vectors: v - 0.0 is exact in f64 and narrowing back is exact | PASS | hand reasoning in the test comment |
| Zero wind leaves the drag bit-identical | unmapped (Unreal demo task, wind) | `aero::tests::test_wind_zero_wind_drag_is_bit_identical` | `to_bits()` equal for the 3 drag components, 30 deg yaw, v = (3, -2, 1) | PASS | - |
| Head wind at hover: drag opposes the apparent air motion, magnitude from the existing drag formula | unmapped (Unreal demo task, wind) | `aero::tests::test_wind_head_wind_at_hover_drag_opposes_apparent_air_motion` | wind (-5,0,0), v = 0: v_air = (5,0,0), F_x = -0.5 * 1.2250 * 25 * 1.0 * 0.02 = -0.30625 N, rel 1e-4 (the ISA tolerance of test_b1) | PASS | rho(0) = 1.2250 is the table value used by `test_b1_isa_values` |
| Vertical wind uses the Z drag coefficient | unmapped (Unreal demo task, wind) | `aero::tests::test_wind_vertical_wind_uses_z_coefficient` | wind (0,0,-4): F_z = -0.5 * 1.2250 * 16 * 1.5 * 0.08 = -1.176 N, rel 1e-4 | PASS | - |
| Wind equal to the vehicle velocity gives exactly zero drag | unmapped (Unreal demo task, wind) | `aero::tests::test_wind_equal_to_vehicle_velocity_gives_zero_drag` | v_air = (3,-2,1) - (3,-2,1) = 0 exactly, drag == [0,0,0] at 30 deg yaw | PASS | - |
| Wind validation: NaN, +-Inf, 31, -31, 30.000001 rejected; +-30 accepted | unmapped (Unreal demo task, wind) | `aero::tests::test_wind_validation_rejects_non_finite_and_over_30` | each rejected value on each component gives the exact error variant; exactly 30 accepted | PASS | bound is per component, not on the norm |
| Zero-wind step is bit-identical to the previous (v10) build | unmapped (Unreal demo task, wind) | `tests::test_wind_zero_wind_step_is_bit_identical_to_v10_build` | 27 bit patterns after 500 steps equal the values captured from git HEAD 4a4659b | PASS | reference is the pre-change build, not a hand calculation (a bit-identity claim has no hand value); capture script was run in a scratch copy of HEAD |
| Head wind through `ffi_step_physics`: telemetry drag | unmapped (Unreal demo task, wind) | `tests::test_wind_head_wind_at_hover_step_drag_is_minus_0_30625_n` | still air gives exactly [0,0,0]; wind (-5,0,0) gives -0.30625 N (rel 1e-4), y and z exactly 0 | PASS | - |
| Standard environment (zeros) density equals the v11 ISA density bit for bit | unmapped (demo M4, environment) | `aero::tests::test_env_standard_density_is_bit_identical_to_isa` | to_bits equal at 0, 1, 123.456, 500, 5000, 11000 m | PASS | the whole-step version is `test_wind_zero_wind_step_is_bit_identical_to_v10_build` (golden, 500 steps, now with the environment pointer) |
| Sea level standard-day density | unmapped (demo M4) | `aero::tests::test_env_sea_level_standard_day_density` | 1.2250122659906946 kg/m3, abs 1e-12 | PASS | 101325/(287.05*288.15), python3 f64 |
| Density at site 3000 m, air 5 C | unmapped (demo M4) | `aero::tests::test_env_3000_m_5_c_density_hand_calc` | 0.8780770469291939 kg/m3, abs 1e-12 | PASS | T_isa 268.65 K, dT +9.5 K, p 70108.268 Pa, derivation in the test comment |
| Temperature offset sign: hot air is thinner | unmapped (demo M4) | `aero::tests::test_env_temperature_offset_sign` | +20 K 1.1455047361519344, -20 K 1.3163799531800062, abs 1e-12, hot < cold | PASS |  |
| Site altitude and vehicle altitude add | unmapped (demo M4) | `aero::tests::test_env_site_and_vehicle_altitude_add` | site 3000 + 100 m equals site 3100 + 0 m bit for bit; 0.8689988332494722 abs 1e-12 | PASS |  |
| Total height range 0..=11000 m (boundaries, below origin, NaN) | unmapped (demo M4) | `aero::tests::test_env_total_height_range` | 11000 accepted, 11000.5 / -1 / NaN rejected; -1 m with a 1000 m site accepted | PASS | same ISA troposphere limit as before, applied to site + vehicle |
| Thrust scales with density | unmapped (demo M4) | `aero::tests::test_env_thrust_scales_with_density` | T(+20 K) / T(std) = 0.9350965438909621 abs 1e-12; std equals `get_thrust` bit for bit | PASS |  |
| Analytic drag scales with density | unmapped (demo M4) | `aero::tests::test_env_drag_scales_with_density` | ratio 0.9350965438909621 rel 1e-6 (f32 output); std equals `get_drag` | PASS | a surrogate answer ignores the environment (documented) |
| Environment validation bounds and order | unmapped (demo M4) | `aero::tests::test_validate_environment_truth_table` | site 0..=5000 and |offset| <= 40 inclusive; NaN/Inf NonFinite first | PASS | 5000 m and 40 K are TODO(HUMAN-VERIFY: demo value) bounds |
| Thermal ambient override settles to the ambient | unmapped (demo M4) | `thermal::tests::test_thermal_ambient_override_settles_to_ambient` | throttle 0, long step: 35 C, abs 1e-6 | PASS | thermal model unvalidated (`thermal_valid` 0) |
| Thermal ambient shifts the equilibrium one for one | unmapped (demo M4) | `thermal::tests::test_thermal_ambient_shifts_equilibrium_one_for_one` | +15 K ambient gives +15 K, abs 1e-9 | PASS |  |
| Default thermal ambient unchanged (20 C baseline) | unmapped (demo M4) | `thermal::tests::test_thermal_default_ambient_is_unchanged` | bit-identical via `update_temperatures` and `update_temperatures_ambient(AMBIENT_TEMP)` | PASS |  |
| Step thrust at a 3000 m site, +9.5 K | unmapped (demo M4) | `tests::test_env_ffi_step_thrust_at_3000_m_site` | 2.6201300770999163 N per motor, abs 1e-6 (f32 telemetry) | PASS | 0.11 * 0.8780770469291939 * (0.5*10000/60)^2 * 0.25^4 |
| Null environment pointer is code 1, nothing written | unmapped (demo M4) | `tests::test_env_ffi_null_environment_is_code_1_and_writes_nothing` | code 1; state and telemetry unchanged | PASS |  |
| Invalid environment is code 13, nothing written | unmapped (demo M4) | `tests::test_env_ffi_invalid_environment_is_code_13_and_writes_nothing` | NaN, Inf, -1 m, 5000.5 m, +-40.5 K each give 13; state, telemetry, latch unchanged | PASS |  |
| Analytic drag through the step uses the environment density | unmapped (demo M4) | `tests::test_env_ffi_step_drag_uses_density` | v = (4,0,0): standard -0.19600196255851114 N abs 1e-6; +20 K ratio 0.9350965438909621 abs 1e-6 | PASS | -0.5*1.2250122659906946*16*1.0*0.02 |
| Demo environment header limits equal the core limits | unmapped (demo M4) | `types::tests::test_cpp_environment_limits_match_rust` | kMaxSiteAltitudeM == 5000, kMaxTempOffsetC == 40, exact, parsed from `DroneEnvironment.h` | PASS | the wind and gust ceilings (15, 5 m/s) are demo choices and not core limits |
| Wind error is reported before an environment error | unmapped (demo M4) | `tests::test_env_ffi_wind_error_is_reported_before_environment_error` | bad wind and bad environment give code 12 | PASS | keeps every earlier code stable |
| Total height above the ISA range is code 6, latch untouched | unmapped (demo M4) | `tests::test_env_ffi_total_height_above_isa_range_is_code_6` | site 5000 + vehicle 6001 = 11001 m gives 6; latch 0 | PASS |  |
| Thermal ambient follows the temperature offset through the step | unmapped (demo M4) | `tests::test_env_ffi_thermal_ambient_follows_temp_offset` | after 5 s at +15 K all motors are in (20, 35) C | UNVERIFIED | bounds only, no closed-form value for the 2500-step path |
| Wind equal to velocity through the step: zero drag, ground velocity still integrated | unmapped (Unreal demo task, wind) | `tests::test_wind_equal_to_vehicle_velocity_step_has_zero_drag` | control run has drag_x < 0; with wind drag == 0 exactly and the ground velocity stays (3,-2) exactly | PASS | catches integrating the air-relative velocity |
| Invalid wind returns code 12 and writes nothing | unmapped (Unreal demo task, wind) | `tests::test_wind_invalid_is_code_12_and_writes_nothing` | NaN, +Inf, -Inf, 31, -31 give 12; state, telemetry, latch unchanged | PASS | new code 12 is in the `ffi_step_physics` code table |
| Wind of exactly 30 m/s per component is accepted | unmapped (Unreal demo task, wind) | `tests::test_wind_30_mps_component_is_accepted` | code 0 for (30,-30,30) | PASS | - |
| Null wind pointer is code 1; an earlier rejection (bad dt) keeps code 7 | unmapped (Unreal demo task, wind) | `tests::test_wind_null_pointer_is_code_1_and_bad_dt_keeps_code_7` | exact codes 1 and 7 | PASS | wind is validated last |
| Only kind "trained" is not a stub | WP-0.2, HZ-01 | `surrogate::tests::test_is_stub_kind` | exact for "trained", "stub", "", "Trained", "trained " | PASS | loader refusal itself still needs an ONNX file |
| Default telemetry has no safe-looking margin | HZ-02, WP-7 D.5 | `types::tests::test_debug_telemetry_default_no_margin` | margin NaN; source and fault fields exactly 0 | PASS | |
| FaultLatch layout: size 4, field offset 0, zero by default | WP-4.6, WP-4.4 | `types::tests::test_fault_latch_layout` | exact | PASS | |
| FFI struct sizes and field offsets | WP-4.6 | `types::tests::test_ffi_offsets_and_sizes` | exact (size 64 and 32, listed offsets) | PASS | DroneState and ThermalState only; the other structs are in the rows below (item A13) |
| ControlInputs layout: size 16, offsets 0, 4, 8, 12 | WP-4.6 | `types::tests::test_control_inputs_layout` | exact | PASS | hand calc in test comment; `const _` block in `types.rs` also guards size and align at build |
| VehicleConfig layout: size 200, 9 field offsets | WP-4.6, ADR-001 D7 | `types::tests::test_vehicle_config_layout` | exact | PASS | hand calc in test comment |
| DebugTelemetry layout: size 128, 15 field offsets | WP-4.6, ADR-001 D7 | `types::tests::test_debug_telemetry_layout` | exact | PASS | hand calc in test comment |
| SurrogateQueryResult layout: size 20, offsets 0, 12, 16 | WP-4.6 | `types::tests::test_surrogate_query_result_layout` | exact | PASS | hand calc in test comment |
| Alignment of all 7 repr(C) structs (8, 4, 8, 4, 4, 8, 4) | WP-4.6 | `types::tests::test_ffi_struct_alignments` | exact | PASS | |
| C++ ExpectedVersion constant equals the Rust interface version | WP-4.5, ADR-001 D7 | `types::tests::test_cpp_expected_version_matches_rust` | exact; parses `kDroneFfiExpectedInterfaceVersion` from `DroneFfiLayout.h`, must be declared once | PASS | ties the C++ header to Rust without Unreal; the C++ static_asserts are checked by the CI job `cpp-ffi-layout` (g++ syntax-only), not by cargo |
| Shipped vehicle_config.json passes validation | WP-2.2, HZ-05 | `config::tests::test_config_shipped_file_is_valid` | Ok; mass_kg == 1.5 exact (JSON literal) | PASS |  |
| Base config literal (equal to shipped file) is valid | WP-2.2, HZ-05 | `config::tests::test_config_base_is_valid` | Ok(()) | PASS | fixture sanity for the rejection tests |
| Negative mass rejected | WP-2.2, HZ-05 | `config::tests::test_config_rejects_negative_mass` | InvalidMass for -1.0 | PASS | WP-2 done-when |
| Zero, +-Inf, NaN mass rejected | WP-2.2, HZ-05 | `config::tests::test_config_rejects_bad_mass` | InvalidMass for 0, +Inf, -Inf, NaN | PASS | +Inf passed the old check |
| Bad inertia rejected, every axis | WP-2.2, HZ-05 | `config::tests::test_config_rejects_bad_inertia` | InvalidInertia for -1, 0, +Inf, NaN on each of 3 axes | PASS |  |
| Non-finite motor position rejected | WP-2.2 | `config::tests::test_config_rejects_bad_motor_position` | InvalidMotorPosition for NaN, +-Inf at each of 12 components; 0 and negative accepted | PASS | no plausibility bound (TODO(HUMAN-VERIFY) B-5) |
| Bad motor spin rejected, every motor | WP-2.2, HZ-04 | `config::tests::test_config_rejects_bad_spin` | InvalidSpin for 0, 0.5, 2, NaN on each of 4 motors | PASS | does not decide the spin convention (B-3) |
| Bad propeller diameter rejected | WP-2.2 | `config::tests::test_config_rejects_bad_prop_diameter` | InvalidPropDiameter for 0, -1, +Inf, NaN | PASS |  |
| Bad thrust coefficient rejected | WP-2.2 | `config::tests::test_config_rejects_bad_thrust_coeff` | InvalidThrustCoeff for 0, -1, +Inf, NaN | PASS |  |
| Bad torque coefficient rejected | WP-2.2 | `config::tests::test_config_rejects_bad_torque_coeff` | InvalidTorqueCoeff for 0, -1, +Inf, NaN | PASS |  |
| Bad max RPM rejected | WP-2.2 | `config::tests::test_config_rejects_bad_max_rpm` | InvalidMaxRpm for 0, -1, +Inf, NaN | PASS |  |
| Hover throttle must be in (0, 1] | WP-2.2 | `config::tests::test_config_hover_throttle_range` | reject 0, -0.1, 1+EPSILON, +Inf, NaN; accept 1.0 | PASS | range only; derived value is checked by the rows below |
| Hover throttle derived: 1.5 kg shipped config | WP-2.7 | `aero::tests::test_hover_throttle_shipped_config` | 0.5015112082212391, abs 1e-12 | PASS | hand calc in test comment; inputs C_T, D, max RPM still ASSUMED |
| Hover throttle, demo reference build (APC 12x6E, MN3508 catalog config) | unmapped (demo M2) | `aero::tests::test_hover_throttle_reference_build_config` | 0.4386224371767256, abs 1e-12 (3552.84 RPM) | UNVERIFIED | config loads and validates; Ct from UIUC static fit, masses and max RPM ASSUMED (catalog.json) |
| Hover throttle derived: 1.0 kg | WP-2.7 | `aero::tests::test_hover_throttle_one_kg` | 0.4094821868095746, abs 1e-12 | PASS | the old 0.4095 literal was the 1 kg value |
| Four motors at hover throttle give back the weight | WP-2.7 | `aero::tests::test_hover_throttle_balances_weight` | 4 * get_thrust = 14.709975 N, relative 1e-12 | PASS | tightened from 1e-6 when thrust became f64 (item A10); hand calc rel error 2.4e-16 |
| Hover throttle altitude outside ISA range is an error | WP-2.7 | `aero::tests::test_hover_throttle_altitude_out_of_range` | AltitudeOutOfRange at -1 and 11001 m | PASS |  |
| Shipped config loads with derived hover throttle | WP-2.7 | `config::tests::test_config_shipped_hover_throttle_derived` | 0.5015112082212391, abs 1e-12 | PASS | JSON no longer has the field |
| JSON hover_throttle key is rejected | WP-2.7, HZ-05 | `config::tests::test_config_hover_throttle_key_rejected` | ParseError | PASS | deny_unknown_fields; also rejects any misspelled key |
| Vehicle that cannot hover is rejected | WP-2.7, HZ-05 | `config::tests::test_config_cannot_hover_rejected` | 6.0 kg InvalidHoverThrottle; 5.9 kg Ok (limit 5.9639 kg) | PASS | FFI code 12 |
| Missing JSON field is a parse error | WP-2.2 | `config::tests::test_config_missing_field_is_parse_error` | ParseError | PASS |  |
| Wrong motor count is a parse error | WP-2.2 | `config::tests::test_config_wrong_motor_count_is_parse_error` | ParseError for 3 spin entries | PASS | four motors enforced by [_; 4] |
| Out-of-range JSON number rejected | WP-2.2 | `config::tests::test_config_overflow_number_rejected` | ParseError for mass 1e400 | PASS |  |
| Missing config file is an IO error | WP-2.2 | `config::tests::test_config_missing_file_is_io_error` | IoError | PASS |  |
| ffi_load_vehicle_config null, missing file, success codes | WP-2.2, WP-11.5 | `tests::test_ffi_load_vehicle_config_codes` | null path 1, null out 1, missing file 2 (out untouched), shipped 0 with mass 1.5 | PASS | load only; query/unload are item A11 |
| Config error to FFI code map is exact | WP-2.2 | `tests::test_ffi_config_error_codes_distinct` | IoError 2 ... InvalidHoverThrottle 12, exact | PASS | interface version 6 |
| Wrapper passes f64 loads to the RK4 core, no f32 narrowing | WP-1.1, ADR-001 D2 | `integrator::tests::test_step_rk4_wrapper_passes_f64_loads` | z = 0.5 * 9.80665 * 0.002^2 = 1.96133e-5 m, abs 1e-18 m; bit-identical to `step_rk4_f64` | PASS | |
| ffi_step_physics passes f64 net force to the integrator | WP-1.1, ADR-001 D2 | `tests::test_ffi_step_gravity_not_truncated` | z = 1.96133e-5 m, abs 1e-18 m; x, y exactly 0 | PASS | torque truncation not observable through the f32 output state; covered by the `[f64; 3]` parameter type |
| Airspeed magnitude computed in f64 | WP-1.1, ADR-001 D2 | `tests::test_airspeed_mps_is_f64` | sqrt(2) abs 2.3e-16; (3,4,0) gives exactly 5 | PASS | |
| Full-throttle thrust at sea level is computed in f64 | WP-1.1, M7 | `aero::tests::test_thrust_full_throttle_sea_level_f64` | 14.621457167857683 N, abs 1e-12 | PASS | f32 rounding would be off by 6.8e-8; inputs ASSUMED |
| Thrust uses the density at the given altitude | M7 | `aero::tests::test_thrust_uses_altitude_density` | 3.3171047909823668 N at 1000 m, abs 1e-12; ratio to 0 m = 0.9074621641061471, abs 1e-12 | PASS | no airspeed term yet (audit M7 open part) |
| Thrust altitude outside ISA range is an error | M7, WP-11.2 | `aero::tests::test_thrust_altitude_out_of_range` | AltitudeOutOfRange at -1, 11001 m and NaN | PASS | |
| Motor thrust helper returns the error, never 0 N | M7, WP-11.2 | `tests::test_motor_thrusts_n_propagates_error` | Err at 12000 m; 3.3171047909823668 N per motor at 1000 m, abs 1e-12 | PASS | |
| ffi_step_physics reports thrust at altitude | M7 | `tests::test_ffi_step_thrust_at_altitude` | motor_thrusts = 3.3171047909823668 N at 1000 m, abs 1e-6 N | PASS | tolerance from the f32 telemetry field |
| ffi_step_physics rejects altitude outside ISA with code 6 and writes nothing | M7, WP-11.2 | `tests::test_ffi_step_altitude_out_of_range_code_6` | code 6 at 12000 m; state, thermal, latch unchanged | PASS | exits via the drag check (no aero handle); the thrust-error branch in the FFI needs a loaded surrogate |
| Model I/O shapes accepted: shipped aero [1,4]->[1,3], fea [1,1]->[1,1], boundary input 64 | WP-11.5, HZ-01 | `surrogate::tests::test_io_widths_accepts_shipped_shapes` | exact (4,3), (1,1), (64,3) | PASS | shapes read from the shipped stub models with the onnx package |
| Unsupported model shapes rejected (dynamic dim, zero, 65, rank, batch, predicted 4, unc/env not [1,1]) | WP-11.5, HZ-01 | `surrogate::tests::test_io_widths_rejects_bad_shapes` | exact BadModelShape for 11 shapes | PASS | MAX_INPUT_WIDTH 64 is an engineering bound, not physics |
| input_count must equal the model width before any slice is built | WP-11.5, HZ-03 | `surrogate::tests::test_check_input_count` | exact: 4 ok; 5, 3, 0, -1, i32::MAX rejected | PASS | FFI wiring needs a loaded handle (known gap) |
| Query inputs: count first, then all finite | WP-11.5, HZ-03 | `surrogate::tests::test_check_inputs` | exact: mismatch found 3/5/1; NaN, +Inf, -Inf rejected; 3 NaNs reports mismatch | PASS | a swapped aero/fea handle fails the width check |
| Model outputs packed exactly | WP-11.5 | `surrogate::tests::test_pack_outputs_exact` | exact f32 values; width 1 leaves slots 1-2 at 0 | PASS | values chosen exact in binary |
| Malformed model outputs rejected (wrong count, NaN/Inf, envelope not 0/1) | WP-11.5, HZ-01 | `surrogate::tests::test_pack_outputs_rejects_wrong_count_and_nonfinite` | exact BadOutput for 11 cases incl. envelope 0.5 | PASS | a real wrong-output ONNX model is not tested (known gap) |
| Loading a missing model file returns IoError without ORT | WP-11.5 | `surrogate::tests::test_load_surrogate_missing_file_is_io_error` | exact IoError | PASS | mutation is caught only where ORT is absent |
| Missing ONNX Runtime library is an error, not an abort | WP-11.5 | `surrogate::tests::test_init_ort_from_missing_dylib_is_engine_error` | EngineError returned, no panic | PASS | assumes no test in the process loaded ORT |
| ffi_load_surrogate_model null, bad UTF-8, missing file | WP-11.5 | `tests::test_ffi_load_surrogate_null_and_bad_path` | exact code 1 x4; out_handle stays null | PASS |  |
| ffi_query_surrogate null, negative count, unknown handle, huge count | WP-11.5, HZ-03 | `tests::test_ffi_query_surrogate_null_and_unknown_handle` | exact: 1 for null/negative; 2 for unknown handle even with i32::MAX; out unchanged | PASS | live handle with wrong count (3) needs ORT |
| ffi_unload_surrogate_model null and never-loaded handle | WP-11.5 | `tests::test_ffi_unload_surrogate_codes` | exact 1 and 2 | PASS | double unload covered at registry level |
| ffi_step_physics rejects an unknown handle with code 9, writes nothing | WP-11.5 | `tests::test_ffi_step_unknown_handle_code_9` | exact 9 for aero and for fea slot; state, thermal, latch, telemetry unchanged | PASS |  |
| Step handle check truth table, aero == fea rejected | WP-11.5 | `tests::test_step_handles_ok_truth_table` | exact for 7 pairs | PASS | aliasing of two &mut |
| Handle registry register/unregister, double unload reported | WP-11.5 | `tests::test_handle_registry_register_unregister` | exact true/false sequence | PASS | fake addresses, local set |
| ffi_reset_drone_state null and reset of a NaN/Inf state | WP-11.5 | `tests::test_ffi_reset_drone_state_null_and_ok` | null 1; reset gives the default state exactly | PASS |  |
| NaN throttle through the FFI step latches (code 8, state restored) | WP-11.5, WP-4.4, HZ-03 | `tests::test_ffi_step_nan_controls_latches` | exact code 8; state and thermal [20;4] unchanged; latch 1 | PASS | characterizes existing behaviour |
| NaN mass through the FFI step latches (code 8, state restored) | WP-11.5, WP-4.4, HZ-03 | `tests::test_ffi_step_nan_config_latches` | exact code 8; state and thermal [20;4] unchanged; latch 1 | PASS | config built directly, bypassing load validation |
| Model provenance accepted only with matching kind, geometry hash and config hash | WP-2.5, HZ-10 | `surrogate::tests::test_check_provenance_accepts_matching_hashes` | exact `Ok(false)` (trained) and `Ok(true)` (stub, feature on) | PASS | pure function; the load with a real ONNX session needs ORT (B-13) |
| A config hash that differs in any way is refused | WP-2.5, HZ-10 | `surrogate::tests::test_check_provenance_config_mismatch` | exact `ConfigMismatch{expected,found}`; also uppercase, empty, trailing newline, geometry hash as config | PASS | exact case-sensitive compare |
| A missing kind, geometry_hash or config_hash is MissingProvenance | WP-2.5, HZ-10 | `surrogate::tests::test_check_provenance_missing_keys` | exact `MissingProvenance` for each of 3 keys | PASS | models made before this change have no config_hash and are refused |
| Provenance check order: stub, then geometry, then config | WP-2.5, HZ-01, HZ-10 | `surrogate::tests::test_check_provenance_check_order` | exact `StubModel`, then `GeometryMismatch` | PASS | keeps the previous order for kind and geometry |
| Compiled CONFIG_HASH is 64 lowercase hex chars and not the geometry hash | WP-2.5, HZ-10 | `surrogate::tests::test_compiled_config_hash_format` | length 64; chars 0-9a-f; differs from GEOMETRY_HASH | PASS | value compared with `sha256sum` by hand, not in a test |
| build.rs SHA-256 of `tools/hash_sample.json` equals the coreutils reference | WP-10.6, HZ-10 | `surrogate::tests::test_build_rs_sha256_matches_reference_sample` | exact string equal to `sha256sum` (139 bytes, LF, non-ASCII) | PASS | Python twin: `tools/test_hash_equality.py` asserts the same string, so Rust == Python |
| Load error to FFI code map is exact (config mismatch is 8) | WP-2.5, WP-11.5 | `tests::test_ffi_load_error_codes` | exact: 1, 2, 3, 4 (x4 variants), 5, 6, 8 | PASS | interface version 9 |
| aero input alpha for a 3-4-5 triangle | WP-6.8 | `surrogate::aero_surrogate::tests::test_aero_inputs_alpha_3_4_5_triangle` | V 5, alpha = atan2(4,3) = 0.9272952180016122, beta 0; abs 1e-15 | UNVERIFIED | alpha sign convention is TODO(HUMAN-VERIFY) |
| aero input beta for a 3-4-5 triangle | WP-6.8 | `surrogate::aero_surrogate::tests::test_aero_inputs_beta_3_4_5_triangle` | V 5, alpha 0, beta = asin(0.6) = 0.6435011087932844; abs 1e-15 | UNVERIFIED | beta convention is TODO(HUMAN-VERIFY) |
| aero input pure sideslip | WP-6.8 | `surrogate::aero_surrogate::tests::test_aero_inputs_pure_sideslip_is_half_pi` | V 5, alpha 0, beta = pi/2; abs 1e-15 | PASS | geometric identity |
| aero input uses q^-1 (yaw 90 deg) | WP-6.8, ADR-001 | `surrogate::aero_surrogate::tests::test_aero_inputs_yaw_90_aligns_body_x_with_east` | v_ned (0,10,0) with yaw +90 gives V 10, alpha 0, beta 0; abs 1e-6 (f32 q) | PASS | body X = NED east after yaw +90 |
| aero input alpha for pitch +30 deg | WP-6.8, ADR-001 | `surrogate::aero_surrogate::tests::test_aero_inputs_pitch_30_gives_alpha_30` | v_body (8.660254, 0, 5) gives alpha 0.5235987755982988; abs 1e-6 (f32 q) | UNVERIFIED | alpha sign convention is TODO(HUMAN-VERIFY) |
| aero input zero velocity is exactly zero | WP-6.8 | `surrogate::aero_surrogate::tests::test_aero_inputs_zero_velocity_is_exactly_zero` | exact [0,0,0] for +0.0 and -0.0 velocity | PASS | undefined-angle convention is TODO(HUMAN-VERIFY) |
| aero input negative-zero body velocity | WP-6.8 | `surrogate::aero_surrogate::tests::test_inputs_from_body_velocity_negative_zero_is_exactly_zero` | exact +0.0 x3 (atan2(-0,-0) would be -pi) | PASS | pins the V == 0 branch |
| aero input beta uses total airspeed | WP-6.8 | `surrogate::aero_surrogate::tests::test_inputs_from_body_velocity_beta_uses_total_airspeed` | v_body (3,4,12): V 13, alpha 1.3258176636680326, beta asin(4/13) = 0.312766721941545; abs 1e-15 / 1e-14 | PASS | atan2(v,u) alone would give 0.9273 |
| aero input NaN velocity is not finite | WP-6.8, HZ-03 | `surrogate::aero_surrogate::tests::test_aero_inputs_nan_velocity_is_not_finite` | at least one non-finite output | PASS | rejected later by check_inputs |
| body to NED rotation (yaw 90 deg) | WP-6.8, ADR-001 | `surrogate::aero_surrogate::tests::test_body_to_ned_yaw_90` | (1,0,0) maps to (0,1,0); abs 1e-6 (f32 q) | UNVERIFIED | output-in-body-frame decision is TODO(HUMAN-VERIFY) |
| aero input width constant | WP-6.8 | `surrogate::aero_surrogate::tests::test_aero_input_width_matches_inputs` | 3 == inputs length | PASS | stub model input width is 3 (checked with onnx, not queried) |
| aero surrogate result rotated to NED, flags kept | WP-6.8 | `aero::tests::test_result_to_ned_rotates_force_and_keeps_flags` | (2,0,5) with yaw +90 gives (0,2,5); abs 1e-6; uncertainty and envelope unchanged | UNVERIFIED | body-frame decision is TODO(HUMAN-VERIFY); the surrogate query path itself needs ORT (B-13) |
| FEA inputs: hover-like load, hottest motor in K | WP-7 D.3 | `surrogate::fea_surrogate::tests::test_fea_inputs_hover_like_load_and_hottest_motor` | loads exact, direction (0,0,-1) exact, 90.5 C = 363.65 K; abs 1e-12 | UNVERIFIED | input layout is TODO(HUMAN-VERIFY) |
| FEA inputs: drag tilts the load direction | WP-7 D.3 | `surrogate::fea_surrogate::tests::test_fea_inputs_drag_tilts_direction` | (-3,0,-10)/sqrt(109) = (-0.2873478855663454, 0, -0.9578262852211513); abs 1e-14 | UNVERIFIED | input layout is TODO(HUMAN-VERIFY) |
| FEA inputs: drag rotated into body frame | WP-7 D.3, ADR-001 | `surrogate::fea_surrogate::tests::test_fea_inputs_drag_is_rotated_into_body_frame` | yaw +90, drag NED (0,-3,0) gives direction (-1,0,0); abs 1e-6 | UNVERIFIED | input layout is TODO(HUMAN-VERIFY) |
| FEA inputs: zero load has zero direction | WP-7 D.3 | `surrogate::fea_surrogate::tests::test_fea_inputs_zero_load_has_zero_direction` | exact (0,0,0); 20 C = 293.15 K abs 1e-12 | PASS | zero-direction convention is TODO(HUMAN-VERIFY) |
| FEA inputs: NaN temperature rejected at query | WP-7 D.3, HZ-03 | `surrogate::fea_surrogate::tests::test_fea_inputs_nan_temperature_is_rejected_at_query` | motor_temp_k NaN; check_inputs gives NonFiniteInput | PASS | f64::max would hide a NaN |
| FEA inputs: NaN in first temperature position | WP-7 D.3, HZ-03 | `surrogate::fea_surrogate::tests::test_fea_inputs_nan_in_first_position_also_rejected` | motor_temp_k NaN | PASS |  |
| FEA model input order | WP-7 D.3 | `surrogate::fea_surrogate::tests::test_fea_to_model_inputs_order` | exact [1..8] order, length 8 | PASS | stub model input width is 8 (checked with onnx, not queried) |
| Envelope flag truth table | WP-6, WP-11.3 | `tests::test_envelope_flag_truth_table` | exact 0/1 for all 9 combinations of none/in/out | PASS | pure move of the previous inline logic |
| Stub model never reports validated (envelope forced to 0) | Review A | `surrogate::tests::test_force_unvalidated_if_stub_truth_table` | 4 exact cases; other fields unchanged | PASS | wiring inside query_model needs ONNX Runtime and is NOT run (B-13); step level covered by test_step_envelope_flag_truth_table |
| Step envelope flag: stub or failed model gives 0 | Review A, F | `tests::test_step_envelope_flag_truth_table` | 6 exact cases | PASS |  |
| Guard converts a panic to code 99 | Review B | `tests::test_guarded_converts_panic_to_code_99` | exact codes 99, 5, -7; constants 99 and 98 | PASS |  |
| Cargo profile does not set panic = abort (catch_unwind needs unwind) | Review B | `tests::test_cargo_profile_does_not_abort_on_panic` | manifest text has no panic abort | PASS | guards the manifest only; a RUSTFLAGS panic=abort build is not detected |
| Deliberate panic is caught in version, size, default state, reset, config load | Review B | `tests::test_ffi_panic_is_caught_in_pure_exports` | exact -1, -1, NaN state, 99, 99; state untouched | PASS |  |
| Deliberate panic is caught in step, load, query, unload; nothing written | Review B | `tests::test_ffi_panic_is_caught_in_step_and_handle_exports` | exact 99 x4; state, thermal, telemetry, latch, out pointers unchanged | PASS |  |
| Registry register, lookup, remove, double remove | Review B | `registry::tests::test_registry_register_lookup_remove` | exact Some/None sequence | PASS |  |
| Registry step slot pairs truth table (null, live, alias, unknown) | Review B | `registry::tests::test_registry_step_slots_truth_table` | 7 exact cases | PASS |  |
| Registry lock is not held while a handle is in use | Review B | `registry::tests::test_registry_not_held_while_slot_is_locked` | no hang (single thread); lookups, register, remove succeed | PASS |  |
| Unload waits for an in-flight query, then the slot is dead | Review B | `registry::tests::test_unload_waits_for_in_flight_query` | blocked at 100 ms, done after release | PASS | timing check can only false-pass if the OS stalls the worker 100 ms |
| Stale slot looked up before unload reads Dead | Review B | `registry::tests::test_stale_slot_after_unload_is_dead` | exact status Dead | PASS |  |
| Poisoned registry lock recovers: Recovered once, then works, entries kept | Review B | `registry::tests::test_registry_poison_is_recovered_once` | exact error then Ok | PASS |  |
| Poisoned slot marks the handle Faulted permanently, still unloadable | Review B | `registry::tests::test_slot_poison_marks_handle_faulted` | Faulted twice, then Dead | PASS |  |
| Slot status truth table | Review B | `registry::tests::test_slot_status_truth_table` | 4 exact cases | PASS |  |
| Locking two slots in opposite orders does not deadlock | Review B | `registry::tests::test_lock_pair_opposite_orders_do_not_deadlock` | 200000 iterations x 2 threads finish | PASS | failure mode is a hang until the CI timeout |
| Over-temperature comparison is strict, NaN is not over | Review C | `thermal::tests::test_any_over_temp_truth_table` | 5 exact cases at limit 120 | PASS |  |
| Temperature finiteness check | Review C | `thermal::tests::test_all_finite_truth_table` | 4 exact cases | PASS |  |
| Thermal constants stay flagged unvalidated until B-4 is closed by a human | Review C | `thermal::tests::test_thermal_stays_unvalidated_until_b4` | const assert false | PASS | fails on purpose if the flag is flipped |
| Step telemetry: thermal_valid 0; over-temp 1 at 119.99 C start, 0 at 20 C | Review C | `tests::test_ffi_step_reports_thermal_unvalidated_and_over_temp` | exact flags; hand calc 0.0465 K per step | PASS | arithmetic on ASSUMED constants (B-4) |
| NaN or Inf in the caller's motor_temp_c gives code 8 and latch | Review C | `tests::test_ffi_step_nan_thermal_state_latches` | code 8; latch 1; thermal unchanged | PASS | the output-side finiteness check is redundant with the input check, so removing only the input check is not caught |
| Runtime config re-check truth table | Review D | `config::tests::test_validate_runtime_config_truth_table` | exact variant per case; max_rpm 0 allowed | PASS |  |
| Step re-validates a hand-filled config (mass 0, inertia 0, max_rpm -1, Inf mass give code 8; max_rpm 0 ok) | Review D | `tests::test_ffi_step_revalidates_config` | code 8 + latch x4; code 0 and thrust [0;4] for max_rpm 0 | PASS |  |
| Code 6 resets telemetry to no data and leaves the latch | Review D | `tests::test_ffi_step_code_6_resets_telemetry_and_leaves_latch` | telemetry fields exact; latch 0, stays 1 when preset | PASS |  |
| Unit quaternion check: tolerance 1e-3, normalization, NaN, zero | Review F | `types::tests::test_unit_quaternion_xyzw_truth_table` | exact results; abs 1e-7 for (0.6, 0.8) | PASS |  |
| Non-unit state quaternion gives code 11 and writes nothing | Review F | `tests::test_ffi_step_code_11_non_unit_quaternion_writes_nothing` | code 11 x3; state, telemetry, latch unchanged | PASS |  |
| Slightly non-unit quaternion is normalized and the step runs | Review F | `tests::test_ffi_step_slightly_non_unit_quaternion_is_normalized` | vz = 0.0196133 abs 1e-6; norm 1 abs 1e-6 | PASS |  |
| NaN state quaternion gives code 8 and latch | Review F | `tests::test_ffi_step_nan_quaternion_latches` | code 8; latch 1 | PASS |  |
| Telemetry source codes: model failed (3) differs from no model (aero 1, fea 0) | Review F | `tests::test_source_codes_distinguish_failed_from_absent` | 6 exact cases | PASS | a real failing ONNX model is NOT tested (needs ONNX Runtime, B-13) |
| Rejected-step telemetry is all no-data | Review D, F | `tests::test_rejected_telemetry_is_no_data` | exact fields; margin NaN | PASS |  |

## Python tests (not parsed by `tools/check_traceability.py`; run by `python -m unittest discover -s offline_pipeline/fea -p 'test_*.py'`)

Requirement: WP-7 Step A.2 (one unit module) and Step C.2 (reserve factor). No hazard ID is mapped yet.
Status of all rows below is PASS locally; real CI not run. File `offline_pipeline/fea/test_units.py` (14 tests total with the next file).

| Test | Pass rule |
| :--- | :--- |
| `test_units.UnitsTest.test_density_carbon_fibre_1600_kg_m3` | exact: 1600 kg/m3 = 1.6e-9 t/mm3 |
| `test_units.UnitsTest.test_length_area_second_moment` | exact: 250 mm, 100 mm2, 1e4 mm4 |
| `test_units.UnitsTest.test_stress_and_modulus` | exact: 200 MPa, 69000 MPa |
| `test_units.UnitsTest.test_mass_force_time_acceleration` | exact: 0.0015 t, 9 N, 2 s, 9000 mm/s2 |
| `test_units.UnitsTest.test_dimensional_consistency` | exact: 0.002 t x 3000 mm/s2 = 6 N |
| `test_units.UnitsTest.test_round_trip` | relative 1e-15 (bound 2.2e-16) |
| `test_units.UnitsTest.test_negative_exponent_is_correctly_rounded_division` | exact vs `fractions.Fraction` reference |
| `test_units.UnitsTest.test_rejects_bad_input` | ValueError: NaN, Inf, bool, str, None, unknown quantity, overflow |
| `test_reserve_factor.ReserveFactorTest.test_rf_hand_calc` | exact 2.0 |
| `test_reserve_factor.ReserveFactorTest.test_rf_exactly_one_at_boundary` | exact 1.0 |
| `test_reserve_factor.ReserveFactorTest.test_rf_below_one_is_failure_case` | exact 0.5 |
| `test_reserve_factor.ReserveFactorTest.test_rf_same_case_mm_and_m` | exact, SI == MPa inputs (exact only for these values) |
| `test_reserve_factor.ReserveFactorTest.test_rf_fos_one_accepted` | exact 2.0 |
| `test_reserve_factor.ReserveFactorTest.test_rf_rejects_bad_input` | ValueError: applied <= 0, allowable <= 0, FoS < 1, NaN, Inf, bool, str, overflow |

## Python case-runner tests (item 18a; not parsed by the script; run by `python -m unittest discover -s offline_pipeline -p 'test_run_case.py'`)

Requirement: WP-6.1 (case.json, one entry point), WP-6.3 (VRAM check refuses a case that does not fit). No hazard ID mapped. PASS locally, real CI not run; 24 tests, 10 of 10 mutations caught (scratch copy).

| Test | Pass rule |
| :--- | :--- |
| `test_run_case.VramTest.test_fits_example` | exact: 1e6 cells x 55 B = 55,000,000 B <= 4.8e9 |
| `test_run_case.VramTest.test_boundary_exact_and_one_over` | exact: 500 B vs limit 500.0 fits; 501 B refused |
| `test_run_case.VramTest.test_headroom_is_applied` | 600 B refused at limit 500.0 |
| `test_run_case.LoadCaseTest.*` (14 tests) | CaseError: missing/unknown key, bool/zero grid, bad rpm, headroom 0 and 1, case_id mismatch, kind, formula, geometry hash, path traversal, no fit |
| `test_run_case.RunSetTest.test_happy_path_results_and_summary` | exact results.json dict; summary line exact |
| `test_run_case.RunSetTest.test_refusal_runs_nothing` | exit 1; solver calls 0; no results dir |
| `test_run_case.RunSetTest.test_nan_output_fails_without_results` | exit 1; no results.json |
| `test_run_case.RunSetTest.test_unknown_solver_fails` / `test_missing_or_empty_set` / `test_real_solver_not_configured` | exit 1 |
| `test_run_case.MainTest.test_bad_set_names` | exit 1 and message `bad set name` |

## Python CalculiX `.dat` parser tests (item 18d; not parsed by the script; run by the `offline_pipeline/fea` unittest step)

Requirement: WP-7 Step C.4 (parser tested on a small known file, every number checked). FORMAT ASSUMED from the documentation, no real sample (TODO(HUMAN-VERIFY), B-9). PASS locally, real CI not run; 12 tests in `test_calculix_dat.py`, 10 of 10 mutations caught.

| Test | Pass rule |
| :--- | :--- |
| `test_calculix_dat.ParseDatTest.test_sample_every_number` | exact: 2 displacement rows, 2 stress rows (12 + 3 numbers), set names, times, columns |
| `test_calculix_dat.ParseDatTest.test_two_time_steps_keep_file_order` | exact values and order |
| `test_calculix_dat.ParseDatTest.test_unknown_block` / `test_data_before_header` / `test_empty_and_blank` / `test_wrong_column_count` / `test_stress_needs_two_keys` / `test_bad_keys` / `test_duplicate_key` / `test_nan_inf_and_fortran_exponent` / `test_overflow_value_rejected` / `test_bad_time` | ValueError with the expected message (line number where relevant) |

## Python Tier-header checker tests (item 17e; not parsed by the script; run by `python -m unittest discover -s tools -p 'test_*.py'`)

Requirement: WP-10.11 (`Tier:` first line in every source file, CI check). No hazard ID mapped. Tier assignments are an AI draft; a human must review them.

| Test | Pass rule |
| :--- | :--- |
| `test_check_tier_headers.CheckText.test_valid_rust` | `// Tier: F` on line 1 gives no error |
| `test_check_tier_headers.CheckText.test_missing_header` | no tag gives an error |
| `test_check_tier_headers.CheckText.test_tag_on_line_two_rejected_for_rust` | tag on line 2 of a C++ file is an error |
| `test_check_tier_headers.CheckText.test_tag_on_line_three_rejected_for_python_shebang` | shebang file: tag on line 3 is an error |
| `test_check_tier_headers.CheckText.test_python_shebang_then_tag_ok` | shebang on line 1, tag on line 2 is valid |
| `test_check_tier_headers.CheckText.test_unknown_tier_x_rejected` | `Tier: X` is an error |
| `test_check_tier_headers.CheckText.test_lowercase_tier_rejected` | `Tier: f` is an error |
| `test_check_tier_headers.CheckText.test_physics_core_must_be_f` | a `physics_core/` file tagged D is an error |
| `test_check_tier_headers.CheckText.test_offline_pipeline_must_be_d` | an `offline_pipeline/` file tagged F is an error |
| `test_check_tier_headers.CheckText.test_crlf_header_accepted` | CRLF line ending after the tag is accepted |
| `test_check_tier_headers.CheckText.test_empty_file_rejected` | empty file is an error |
| `test_check_tier_headers.CollectFiles.test_skips_generated_and_finds_scopes` | exact file list; `Generated/` and `__pycache__` skipped |

## Python mass/inertia literal checker tests (item 17f; not parsed by the script; run by `python -m unittest discover -s tools -p 'test_*.py'`)

Requirement: WP-2 "Done when" (no mass or inertia literal in the source, searched in CI). No hazard ID mapped (HZ-05 context). A tripwire, not a proof: arithmetic and function-argument literals are not flagged.

| Test | Pass rule |
| :--- | :--- |
| `test_check_mass_literals.Flagged.*` (8 tests) | struct field, let, array, negative, typed const, typed array, comparison and two-line cases each give the stated error count (1 or 2) |
| `test_check_mass_literals.NotFlagged.*` (8 tests) | type-only, arithmetic with config mass, line/doc/block comments, string literal, unrelated names, `mod tests` body give 0 errors |
| `test_check_mass_literals.StructureChecks.*` (4 tests) | code before `#[cfg(test)]` is scanned, line number reported, two `#[cfg(test)]` and `#[cfg(test)]` not before `mod tests` are rejected |

## Python model-promotion tests (item 18e; not parsed by the script; run by `python -m unittest discover -s offline_pipeline -p 'test_promote_model.py'`, in the `model-promotion.yml` workflow and locally; NOT in the `ci.yml` build job, which has no onnx)

Requirement: WP-10.4 (promotion by reviewed PR, one manifest, hash/kind/holdout error recorded), exit gate 12, HZ-01, HZ-10. 35 tests, 14 of 14 mutations caught. Holdout metric definition is TODO(HUMAN-VERIFY: WP-6/WP-7).

| Test | Pass rule |
| :--- | :--- |
| `test_promote_model.HashAndJson.*` (3) | SHA-256("abc") = ba7816bf...15ad (FIPS 180-4 vector); canonical JSON text exact; NaN rejected |
| `test_promote_model.PathRules.*` (2) | valid path accepted; absolute, `..`, backslash, wrong directory, wrong suffix, empty, non-string rejected |
| `test_promote_model.EntryFields.*` (7) | valid trained/stub; stub with holdout rejected; trained needs finite holdout >= 0 (None, negative, NaN, Inf, bool, string rejected); unknown kind; bad hashes; extra/missing keys; non-object |
| `test_promote_model.BuildEntry.*` (7) | stub needs `--allow-stub`; entry values exact (path, sha256, hashes, null holdout); trained needs holdout; model outside the models dir or the repo rejected; missing metadata rejected |
| `test_promote_model.CheckManifest.*` (11) | valid passes; missing file, invalid JSON, changed bytes, corrupt model, deleted model, kind mismatch, stale geometry hash, stale config hash, non-canonical text, wrong roles/schema/keys each give the stated error count |
| `test_promote_model.Main.*` (5) | promote then check exits 0; refusals exit 1 and write nothing; one role only fails the check; manifest has no CR |

## Known gaps (no test exists yet; not parsed by the script)

- Reserve factor: no hazard ID mapped; laminate criterion (Tsai-Wu or max strain) not implemented; the plan's "same case in mm and in m" rule is only checked on exact values, not on a real CalculiX run (B-8, B-9). The `drop-finite-input` check in `units.py` is redundant with the result check (equivalent mutation, kept as defence in depth).

- run_case.py (18a): the FluidX3D launcher, bytes_per_cell and the real output format are unknown (TODO(HUMAN-VERIFY), B-7); only fake solvers are tested; no example case set is committed; no step-summary test on real CI; `kind` fea/impact refused.

- calculix_dat.py (18d): layout unverified against a real `.dat`; no `.frd`, no eigenvalue/frequency table, no max-stress-and-location extraction, no results.json writer (proposed 18d2/18d3).

- Surrogate inputs (item 19): no test queries a real ONNX model with the new widths (3 aero, 8 FEA; B-13, needs ORT); the `fea_inputs` temperature source (pre-update thermal state) is not observable without a live handle (mutation survived); an old model with width 4 or 1 makes every query fail and the step falls back silently to the analytic drag (aero_source 1) or the NaN margin; a load-time width check would need a new FFI load code. Conventions (alpha/beta, body-frame force, still air, FEA layout) are TODO(HUMAN-VERIFY).

- Hover throttle derived from mass, thrust coefficient and density (item A6).
- FFI with a real ONNX session (B-13): load/query/unload round trip, `input_count` against a live handle (code 3), a model with the wrong output count (code 6), load code 6, the step thrust `Err => 6` branch. The logic is covered by the pure tests above. A freed address reused by a later load is not detected by the handle registry.
- C++ layout: the `static_assert`s in `DroneFfiLayout.h` are only syntax-checked with g++ and clang++ on x86-64 Linux (CI job `cpp-ffi-layout`, not yet run on a real CI). MSVC and the full UE5 plugin build have NOT been done (BLOCKED-HUMAN, see STATUS). Header freshness: `build.rs` reruns cbindgen on any `src/` change, and the `ci.yml` step "Generated FFI header matches the committed copy" diffs it (item A15; not yet run on real CI; its mutations were NOT run).
- NED to Unreal conversion (WP-5.2, HZ-04): `tools/cpp_ffi_check/frame_conversion_test.cpp` (g++, CI step in `cpp-ffi-layout`, not run on real CI) checks the algebra only; engine photos (B-6) and the compile of `DroneTelemetryComponent.cpp`/`DronePhysicsBridge.cpp` are BLOCKED-HUMAN.
- Surrogate stub refusal (StubModel, code 5) needs an ONNX file and `ORT_DYLIB_PATH`.
- Config hash (item A16): no test loads a real ONNX file with a wrong `config_hash` (needs ORT, B-13); no Rust vs Python hash-equality test (item A17); `tag_and_place.py` records only `geometry_hash`.
- Hazards with no cargo test: HZ-01, HZ-02, HZ-05, HZ-06, HZ-08, HZ-09 (see `docs/hazard-matrix.md`). HZ-10 now has pure-function tests only.
- Model promotion (item 18e): the manifest is not read by Unreal or by `ffi_load_surrogate_model` yet; CODEOWNERS / branch protection that forces a human review of `active_model.json` is not configured (HUMAN-TODO). `train-surrogate.yml` still uploads an artifact and does not run `promote_model.py`. Re-running `train_*.py` changes the stub bytes and the manifest check then fails until `promote_model.py` is run again.
~- miri (item 17d): `cargo miri test --no-default-features --lib` runs 76 of the 128 tests (the pure modules). The 52 tests that need the `onnx` feature (FFI step, load, query, unload, aero fallback) are not under miri. miri was NOT run here (no nightly installed); the CI job has never run.

- Demo M4/M5 additions (C++ tests are NOT parsed by the script; all run in CI jobs `cpp-ffi-layout` / `build-and-test`, none has run on a real CI): `tools/cpp_ffi_check/environment_test.cpp` (ISA temperature, density hand values, hover scale, typed-weather validation without silent clamping), `electrical_test.cpp` (prop power, motor current and voltage, battery current and sag, arm bending, margin and verdict rules), `mission_test.cpp` (scripted mission with a kinematic stand-in plant), `report_test.cpp` (ranked margins with hand values, causes, suggestions, JSON/CSV/HTML writers), `mission_runner_test.cpp` (the mission against the real release library: hover electrical hand values, density effect, determinism, forced failures) and `mission_runner.cpp` (writes the report files; a CI smoke step checks the JSON parses). Python: `tools/test_gen_demo_headers.py`, `tools/test_environments_json.py`, `tools/props/`, `tools/geometry/`, `offline_pipeline/components/`, `offline_pipeline/cfd/`.
- Unreal demo (wind, prop spin, demo controller): the C++ tests are NOT parsed by the script. `tools/cpp_ffi_check/wind_test.cpp` (seeded OU gust generator: SplitMix64 reference vectors, determinism, different seeds, zero sigma, clamp, long-run mean / std / lag-1 autocorrelation with theory-derived bounds, parameter validation), `visual_math_test.cpp` (spin angle, uniform scale, motion-disc opacity) and `demo_control_test.cpp` (PID, Euler angles, heading frame, modes, throttle) run in the CI job `cpp-ffi-layout`; `demo_flight_sim.cpp` flies the DEMO controller and the wind against the REAL release library (CI job `build-and-test`). None of them ran on a real CI. The closed-loop sim checks stability and limits (pass bands are TODO(HUMAN-VERIFY: demo value)), not accuracy. `DroneDemoPawn.cpp`, `DroneDemoGameMode.cpp` and the Build.cs change have NEVER been compiled (no Unreal here): a Windows build is BLOCKED-HUMAN.
