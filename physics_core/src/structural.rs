use crate::surrogate::SurrogateHandle;
use crate::surrogate::fea_surrogate::query_structural;
use crate::types::SurrogateQueryResult;

/// Calculates structural safety margin, utilizing the surrogate model if available,
/// or falling back to a default safe value.
///
/// # Units
/// * `net_force_magnitude` - Newtons
/// * Returns - `(Safety Margin, Optional Surrogate Metadata)`
pub fn get_safety_margin_with_fallback(
    net_force_magnitude: f32,
    fea_handle: Option<&mut SurrogateHandle>,
) -> (f32, Option<SurrogateQueryResult>) {
    if let Some(handle) = fea_handle {
        if let Ok(result) = query_structural(handle, net_force_magnitude) {
            // The surrogate returns the safety margin as its first predicted value
            return (result.predicted_values[0], Some(result));
        }
    }

    // Fallback: If no model is loaded, assume the structure is perfectly safe (margin 10.0)
    (10.0, None)
}
