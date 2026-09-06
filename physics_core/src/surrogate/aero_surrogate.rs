use crate::surrogate::{SurrogateHandle, SurrogateLoadError, query_model};
use crate::types::{DroneState, SurrogateQueryResult};
use glam::{Quat, Vec3};

/// Queries the aerodynamic surrogate model to predict drag force.
///
/// # Units
/// * `state.orientation` - Quaternion converted to Euler XYZ (radians)
/// * `state.velocity` - Converted to scalar airspeed (m/s)
/// * Returns - 3D Drag vector (Newtons)
///
/// # Errors
/// * `EngineError` - Forwarded from the underlying generic ONNX query.
pub fn query_aero(
    handle: &mut SurrogateHandle,
    state: &DroneState,
) -> Result<SurrogateQueryResult, SurrogateLoadError> {
    let q = Quat::from_array(state.orientation);
    let (roll, pitch, yaw) = q.to_euler(glam::EulerRot::XYZ);

    let velocity = Vec3::from_array(state.velocity);
    let airspeed = velocity.length();

    let inputs = [roll, pitch, yaw, airspeed];

    query_model(handle, &inputs)
}
