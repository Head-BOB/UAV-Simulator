// Tier: F
use crate::surrogate::SurrogateHandle;
use crate::surrogate::fea_surrogate::{FeaInputs, query_structural};
use crate::types::SurrogateQueryResult;

/// Calculates structural safety margin, utilizing the surrogate model if available,
/// or falling back to a default safe value.
///
/// # Units
/// * `inputs` - per-arm loads (N), body-frame load direction, motor temperature (K); see `FeaInputs`
/// * Returns - `(Safety Margin, Optional Surrogate Metadata)`
pub fn get_safety_margin_with_fallback(
    inputs: &FeaInputs,
    fea_handle: Option<&mut SurrogateHandle>,
) -> (f32, Option<SurrogateQueryResult>) {
    if let Some(handle) = fea_handle
        && let Ok(result) = query_structural(handle, inputs)
    {
        return (result.predicted_values[0], Some(result));
    }

    // WP-7 Step D.5: No model means no number. Return NaN, not a fake 10.0 safe margin.
    (f32::NAN, None)
}
