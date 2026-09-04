use crate::types::SurrogateQueryResult;
use ort::session::Session;
use std::sync::Once;

/// Specific failure modes for surrogate model loading.
pub enum SurrogateLoadError {
    IoError,
    MissingProvenance,
    GeometryMismatch { expected: String, found: String },
    EngineError(String),
}

/// Handle to an active ONNX surrogate model session.
pub struct SurrogateHandle {
    pub session: Session,
}

#[allow(dead_code)]
const CURRENT_GEOMETRY_HASH: &str = env!("GEOMETRY_HASH");

static INIT_ORT: Once = Once::new();

/// Validates ONNX metadata against the compiled canonical geometry hash and loads the ML session.
///
/// # Errors
/// * `EngineError` - The ONNX runtime failed to initialize or parse the model.
/// * `IoError` - The file cannot be read from the filesystem.
/// * `MissingProvenance` - The ONNX file lacks a `geometry_hash` property.
/// * `GeometryMismatch` - The embedded hash does not match `CURRENT_GEOMETRY_HASH`.
pub fn load_surrogate(path: &str) -> Result<SurrogateHandle, SurrogateLoadError> {
    INIT_ORT.call_once(|| {
        let _ = ort::init()
            .with_name("UAV_Simulator_Physics")
            .commit();
    });

    let session = Session::builder()
        .map_err(|e| SurrogateLoadError::EngineError(e.to_string()))?
        .commit_from_file(path)
        .map_err(|_| SurrogateLoadError::IoError)?;

    {
        let metadata = session
            .metadata()
            .map_err(|e| SurrogateLoadError::EngineError(e.to_string()))?;

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
/// * `EngineError` - If tensor creation, execution, or extraction fails.
/// Queries the loaded ONNX surrogate model generically.
///
/// # Units
/// * `inputs` - Context-dependent flat array of floats.
///
/// # Errors
/// * `EngineError` - If tensor creation, execution, or extraction fails.
/// Queries the loaded ONNX surrogate model generically.
///
/// # Errors
/// * `EngineError` - If tensor creation, execution, or extraction fails.
/// Queries the loaded ONNX surrogate model generically.
///
/// # Errors
/// * `EngineError` - If tensor creation, execution, or extraction fails.
pub fn query_model(
    handle: &mut SurrogateHandle,
    inputs: &[f32],
) -> Result<SurrogateQueryResult, SurrogateLoadError> {

    // We pass a tuple of ([shape], data) directly to ort, bypassing ndarray entirely
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