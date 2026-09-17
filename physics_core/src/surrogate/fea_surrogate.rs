use crate::surrogate::{SurrogateHandle, SurrogateLoadError, query_model};
use crate::types::SurrogateQueryResult;

/// Queries the structural FEA surrogate model to predict safety margin.
///
/// # Units
/// * `net_force_magnitude` - Newtons (proxy for G-loading)
/// * Returns - Structural safety margin (unitless, < 1.0 is failure)
///
/// # Errors
/// * `EngineError` - Forwarded from the underlying generic ONNX query.
pub fn query_structural(
    handle: &mut SurrogateHandle,
    net_force_magnitude: f32,
) -> Result<SurrogateQueryResult, SurrogateLoadError> {
    let inputs = [net_force_magnitude];
    query_model(handle, &inputs)
}
