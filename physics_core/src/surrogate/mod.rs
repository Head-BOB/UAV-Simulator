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
