// Tier: F
//! Ground-impact event hook (Completion Plan WP-8.6, Remaining work (A) item 20).
//!
//! An impact is an EVENT check, not a per-tick one. The host calls [`on_ground_contact`] once per
//! physics step with the state before and after the step. When the vehicle crosses the ground
//! plane, the impact inputs are built once and the impact model is queried once. Nothing here is
//! called from `ffi_step_physics`, nothing here runs inside the 0.002 s loop, and no FFI function
//! or struct exposes it (Rust-only API, interface version unchanged).
//!
//! Frames follow ADR-001: NED world, position down is +Z, velocity in the local NED frame,
//! quaternions are (x, y, z, w).
//!
//! TODO(HUMAN-VERIFY: WP-8, B-11): the impact input set (speed, sink speed, angle from vertical,
//! orientation), which state defines it (post-step state is used here), the contact point and the
//! ground model are not decided. The `OpenRadioss` case plan has not been written.

use crate::types::DroneState;
use glam::DVec3;

/// Inputs handed to an impact model. All fields are f64 and finite.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ImpactInputs {
    /// Total speed `|v|` in m/s.
    pub speed_mps: f64,
    /// Downward (NED +Z) speed in m/s. Always > 0.
    pub sink_speed_mps: f64,
    /// Angle between the velocity vector and straight down, in rad: 0 = vertical drop,
    /// pi/2 = horizontal. Range [0, pi/2) because the sink speed is > 0.
    pub impact_angle_rad: f64,
    /// Body to NED attitude as a quaternion in order (x, y, z, w).
    pub orientation_xyzw: [f64; 4],
}

/// Why impact inputs could not be built.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ImpactError {
    /// A position, velocity, orientation or ground value is NaN or infinite.
    NonFinite,
    /// The vertical velocity is not downward (NED +Z must be > 0), so there is no impact.
    NotDescending,
}

/// An impact model, for example a surrogate trained on `OpenRadioss` runs (WP-8). Called at most
/// once per ground crossing. TODO(HUMAN-VERIFY: WP-8): the output type (damage flag, peak g, RF).
pub trait ImpactModel {
    /// What the model reports for one impact.
    type Output;
    /// Evaluates one impact. Must not be slow enough to matter: it runs outside the step loop.
    fn evaluate(&mut self, inputs: &ImpactInputs) -> Self::Output;
}

/// True when the vehicle moved from above the ground plane onto or through it in one step.
///
/// Positions are NED down in metres, so "above ground" means a smaller value:
/// `prev_down_m < ground_down_m <= next_down_m`. A vehicle that starts a step already at or
/// below the ground does not trigger a second event.
///
/// # Errors
/// [`ImpactError::NonFinite`] if any argument is NaN or infinite.
pub fn ground_contact_event(
    prev_down_m: f64,
    next_down_m: f64,
    ground_down_m: f64,
) -> Result<bool, ImpactError> {
    if ![prev_down_m, next_down_m, ground_down_m]
        .iter()
        .all(|x| x.is_finite())
    {
        return Err(ImpactError::NonFinite);
    }
    Ok(prev_down_m < ground_down_m && ground_down_m <= next_down_m)
}

/// Builds the impact inputs from a state (velocity in NED m/s, orientation (x, y, z, w)).
///
/// # Errors
/// [`ImpactError::NonFinite`] for any NaN/Inf velocity or orientation component;
/// [`ImpactError::NotDescending`] if the downward velocity is not > 0.
pub fn build_impact_inputs(state: &DroneState) -> Result<ImpactInputs, ImpactError> {
    let v = DVec3::new(
        f64::from(state.velocity[0]),
        f64::from(state.velocity[1]),
        f64::from(state.velocity[2]),
    );
    let q = state.orientation.map(f64::from);
    if !v.is_finite() || !q.iter().all(|c| c.is_finite()) {
        return Err(ImpactError::NonFinite);
    }
    let sink_speed_mps = v.z;
    if sink_speed_mps <= 0.0 {
        return Err(ImpactError::NotDescending);
    }
    let horizontal_mps = v.x.hypot(v.y);
    Ok(ImpactInputs {
        speed_mps: v.length(),
        sink_speed_mps,
        impact_angle_rad: horizontal_mps.atan2(sink_speed_mps),
        orientation_xyzw: q,
    })
}

/// Event hook: if the step from `prev` to `next` crosses the ground plane, builds the impact
/// inputs from `next` and queries `model` exactly once. Otherwise returns `Ok(None)` and does
/// not touch the model.
///
/// # Errors
/// Any [`ImpactError`] from [`ground_contact_event`] or [`build_impact_inputs`]; the model is
/// not called in that case.
pub fn on_ground_contact<M: ImpactModel>(
    prev: &DroneState,
    next: &DroneState,
    ground_down_m: f64,
    model: &mut M,
) -> Result<Option<M::Output>, ImpactError> {
    if !ground_contact_event(prev.position[2], next.position[2], ground_down_m)? {
        return Ok(None);
    }
    let inputs = build_impact_inputs(next)?;
    Ok(Some(model.evaluate(&inputs)))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn state(down_m: f64, velocity: [f32; 3]) -> DroneState {
        DroneState {
            position: [0.0, 0.0, down_m],
            velocity,
            orientation: [0.0, 0.0, 0.0, 1.0],
            angular_velocity: [0.0, 0.0, 0.0],
        }
    }

    /// Counts calls and echoes the sink speed so tests can see what the model received.
    struct CountingModel {
        calls: u32,
        last_sink_mps: f64,
    }

    impl ImpactModel for CountingModel {
        type Output = u32;
        fn evaluate(&mut self, inputs: &ImpactInputs) -> u32 {
            self.calls += 1;
            self.last_sink_mps = inputs.sink_speed_mps;
            self.calls
        }
    }

    fn model() -> CountingModel {
        CountingModel {
            calls: 0,
            last_sink_mps: f64::NAN,
        }
    }

    // Hand calc: v = (3, 0, 4) m/s. speed = sqrt(9 + 16) = 5; sink = 4; horizontal = 3;
    // angle = atan2(3, 4) = atan(0.75) = 0.6435011087932844 rad (36.87 deg, the 3-4-5 triangle).
    #[test]
    fn test_impact_inputs_3_4_5_triangle() {
        let i = build_impact_inputs(&state(0.0, [3.0, 0.0, 4.0])).expect("descending");
        assert!((i.speed_mps - 5.0).abs() < 1e-15);
        assert!((i.sink_speed_mps - 4.0).abs() < 1e-15);
        assert!((i.impact_angle_rad - 0.643_501_108_793_284_4).abs() < 1e-15);
    }

    // Hand calc: pure vertical drop v = (0, 0, 2): speed 2, sink 2, angle atan2(0, 2) = 0.
    #[test]
    fn test_impact_inputs_vertical_drop_angle_zero() {
        let i = build_impact_inputs(&state(0.0, [0.0, 0.0, 2.0])).expect("descending");
        assert!((i.speed_mps - 2.0).abs() < 1e-15);
        assert!(i.impact_angle_rad.abs() < 1e-15);
    }

    // Hand calc: v = (0, 5, 5): horizontal 5 = sink 5, angle = atan2(5, 5) = pi/4 = 0.7853981633974483,
    // speed = 5 sqrt(2) = 7.0710678118654752.
    #[test]
    fn test_impact_inputs_45_degrees_uses_east_component() {
        let i = build_impact_inputs(&state(0.0, [0.0, 5.0, 5.0])).expect("descending");
        assert!((i.impact_angle_rad - std::f64::consts::FRAC_PI_4).abs() < 1e-15);
        assert!((i.speed_mps - 7.071_067_811_865_475_2).abs() < 1e-15);
    }

    #[test]
    fn test_impact_inputs_orientation_passes_through_exactly() {
        let mut s = state(0.0, [0.0, 0.0, 1.0]);
        s.orientation = [0.25, -0.5, 0.5, 0.75]; // all exact in binary
        let i = build_impact_inputs(&s).expect("descending");
        assert_eq!(i.orientation_xyzw, [0.25, -0.5, 0.5, 0.75]);
    }

    #[test]
    fn test_impact_inputs_not_descending() {
        for v in [[2.0, 0.0, 0.0], [0.0, 0.0, -1.0], [3.0, 4.0, 0.0]] {
            assert_eq!(
                build_impact_inputs(&state(0.0, v)),
                Err(ImpactError::NotDescending),
                "{v:?}"
            );
        }
    }

    #[test]
    fn test_impact_inputs_non_finite() {
        let mut s = state(0.0, [0.0, 0.0, f32::NAN]);
        assert_eq!(build_impact_inputs(&s), Err(ImpactError::NonFinite));
        s = state(0.0, [f32::INFINITY, 0.0, 1.0]);
        assert_eq!(build_impact_inputs(&s), Err(ImpactError::NonFinite));
        s = state(0.0, [0.0, 0.0, 1.0]);
        s.orientation[3] = f32::NAN;
        assert_eq!(build_impact_inputs(&s), Err(ImpactError::NonFinite));
    }

    // Truth table, ground at down = 0 m (prev < ground <= next):
    // -0.5 -> 0.01 crosses; -0.5 -> -0.1 stays above; 0.01 -> 0.02 already below;
    // -0.1 -> 0.0 lands exactly on the plane (counts); 0.0 -> 0.1 starts on the plane (no event).
    #[test]
    fn test_ground_contact_truth_table() {
        let cases = [
            (-0.5, 0.01, true),
            (-0.5, -0.1, false),
            (0.01, 0.02, false),
            (-0.1, 0.0, true),
            (0.0, 0.1, false),
            (-0.5, -0.6, false),
        ];
        for (prev, next, want) in cases {
            assert_eq!(
                ground_contact_event(prev, next, 0.0),
                Ok(want),
                "{prev} -> {next}"
            );
        }
    }

    #[test]
    fn test_ground_contact_uses_ground_height() {
        // Ground 10 m below the origin is down = +10: crossing 9.9 -> 10.1 counts, 0 -> 9.9 does not.
        assert_eq!(ground_contact_event(9.9, 10.1, 10.0), Ok(true));
        assert_eq!(ground_contact_event(0.0, 9.9, 10.0), Ok(false));
    }

    #[test]
    fn test_ground_contact_non_finite() {
        assert_eq!(
            ground_contact_event(f64::NAN, 0.0, 0.0),
            Err(ImpactError::NonFinite)
        );
        assert_eq!(
            ground_contact_event(-1.0, f64::INFINITY, 0.0),
            Err(ImpactError::NonFinite)
        );
        assert_eq!(
            ground_contact_event(-1.0, 0.0, f64::NAN),
            Err(ImpactError::NonFinite)
        );
    }

    #[test]
    fn test_hook_calls_model_once_with_post_step_velocity() {
        let mut m = model();
        let out = on_ground_contact(
            &state(-0.01, [0.0, 0.0, 3.0]),
            &state(0.01, [0.0, 0.0, 4.0]),
            0.0,
            &mut m,
        );
        assert_eq!(out, Ok(Some(1)));
        assert_eq!(m.calls, 1);
        assert!((m.last_sink_mps - 4.0).abs() < 1e-15);
    }

    #[test]
    fn test_hook_does_not_call_model_without_crossing() {
        let mut m = model();
        let above = on_ground_contact(
            &state(-2.0, [0.0, 0.0, 3.0]),
            &state(-1.9, [0.0, 0.0, 3.0]),
            0.0,
            &mut m,
        );
        let below = on_ground_contact(
            &state(0.5, [0.0, 0.0, 3.0]),
            &state(0.6, [0.0, 0.0, 3.0]),
            0.0,
            &mut m,
        );
        assert_eq!((above, below), (Ok(None), Ok(None)));
        assert_eq!(m.calls, 0);
    }

    #[test]
    fn test_hook_once_per_crossing_over_a_sequence() {
        let mut m = model();
        let downs = [-0.02, -0.01, 0.01, 0.02, 0.03];
        for pair in downs.windows(2) {
            let _ = on_ground_contact(
                &state(pair[0], [0.0, 0.0, 3.0]),
                &state(pair[1], [0.0, 0.0, 3.0]),
                0.0,
                &mut m,
            );
        }
        assert_eq!(m.calls, 1);
    }

    #[test]
    fn test_hook_error_leaves_model_untouched() {
        let mut m = model();
        // Crossing, but moving up (inconsistent state): error, no model call.
        let up = on_ground_contact(
            &state(-0.01, [0.0, 0.0, -1.0]),
            &state(0.01, [0.0, 0.0, -1.0]),
            0.0,
            &mut m,
        );
        assert_eq!(up, Err(ImpactError::NotDescending));
        let nan = on_ground_contact(
            &state(-0.01, [0.0, 0.0, 1.0]),
            &state(f64::NAN, [0.0, 0.0, 1.0]),
            0.0,
            &mut m,
        );
        assert_eq!(nan, Err(ImpactError::NonFinite));
        assert_eq!(m.calls, 0);
    }
}
