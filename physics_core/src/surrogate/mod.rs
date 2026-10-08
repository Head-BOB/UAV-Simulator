pub mod aero_surrogate;
pub mod fea_surrogate;

use crate::types::SurrogateQueryResult;
use ort::session::Session;
use std::sync::Once;

/// Specific failure modes for surrogate model loading.
pub enum SurrogateLoadError {
    IoError,
    MissingProvenance,
    GeometryMismatch { expected: String, found: String },
    StubModel,
    EngineError(String),
}

/// Handle to an active ONNX surrogate model session.
pub struct SurrogateHandle {
    pub session: Session,
}

const CURRENT_GEOMETRY_HASH: &str = env!("GEOMETRY_HASH");

static INIT_ORT: Once = Once::new();

/// Validates ONNX metadata against the compiled canonical geometry hash and loads the ML session.
///
/// # Errors
/// Returns EngineError if the ONNX runtime failed to initialize or parse the model.
/// Returns IoError if the file cannot be read from the filesystem.
/// Returns MissingProvenance if the ONNX file lacks a geometry_hash or kind property.
/// Returns GeometryMismatch if the embedded hash does not match CURRENT_GEOMETRY_HASH.
/// Returns StubModel if the model is a stub and the allow_stub_models feature is not enabled.
pub fn load_surrogate(path: &str) -> Result<SurrogateHandle, SurrogateLoadError> {
    INIT_ORT.call_once(|| {
        let _ = ort::init().with_name("UAV_Simulator_Physics").commit();
    });

    let session = Session::builder()
        .map_err(|e| SurrogateLoadError::EngineError(e.to_string()))?
        .commit_from_file(path)
        .map_err(|_| SurrogateLoadError::IoError)?;

    {
        let metadata = session
            .metadata()
            .map_err(|e| SurrogateLoadError::EngineError(e.to_string()))?;

        let kind = metadata
            .custom("kind")
            .ok_or(SurrogateLoadError::MissingProvenance)?;

        if kind != "trained" && !cfg!(feature = "allow_stub_models") {
            return Err(SurrogateLoadError::StubModel);
        }

        let embedded_hash = metadata
            .custom("geometry_hash")
            .ok_or(SurrogateLoadError::MissingProvenance)?;

        if embedded_hash != CURRENT_GEOMETRY_HASH {
            return Err(SurrogateLoadError::GeometryMismatch {
                expected: CURRENT_GEOMETRY_HASH.to_string(),
                found: embedded_hash,
            });
        }
    }

    Ok(SurrogateHandle { session })
}

/// Queries the loaded ONNX surrogate model generically.
///
/// # Units
/// * `inputs` - Context-dependent flat array of floats.
///
/// # Errors
/// Returns EngineError if tensor creation, execution, or extraction fails.
pub fn query_model(
    handle: &mut SurrogateHandle,
    inputs: &[f32],
) -> Result<SurrogateQueryResult, SurrogateLoadError> {
    let input_tensor = ort::value::Tensor::from_array(([1, inputs.len()], inputs.to_vec()))
        .map_err(|e| SurrogateLoadError::EngineError(e.to_string()))?;

    let outputs = handle
        .session
        .run(ort::inputs!["inputs" => input_tensor])
        .map_err(|e| SurrogateLoadError::EngineError(e.to_string()))?;

    let pred_tuple = outputs["predicted_values"]
        .try_extract_tensor::<f32>()
        .map_err(|e| SurrogateLoadError::EngineError(e.to_string()))?;
    let pred_slice = pred_tuple.1;

    let mut predicted_values = [0.0; 3];
    for (i, &val) in pred_slice.iter().take(3).enumerate() {
        predicted_values[i] = val;
    }

    let unc_tuple = outputs["uncertainty"]
        .try_extract_tensor::<f32>()
        .map_err(|e| SurrogateLoadError::EngineError(e.to_string()))?;
    let uncertainty = *unc_tuple.1.first().unwrap_or(&0.0);

    let env_tuple = outputs["in_validated_envelope"]
        .try_extract_tensor::<f32>()
        .map_err(|e| SurrogateLoadError::EngineError(e.to_string()))?;
    let in_validated_envelope = *env_tuple.1.first().unwrap_or(&0.0) as i32;

    Ok(SurrogateQueryResult {
        predicted_values,
        uncertainty,
        in_validated_envelope,
    })
}
