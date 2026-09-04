/// Specific failure modes for surrogate model loading.
pub enum SurrogateLoadError {
    IoError,
    MissingProvenance,
    GeometryMismatch { expected: String, found: String },
}

/// Handle to an active ONNX surrogate model session.
pub struct SurrogateHandle {
    // TODO(Dev B): Add ort::Session handle here once ONNX runtime is linked
}

impl SurrogateHandle {
    pub const fn new() -> Self {
        Self {}
    }
}

impl Default for SurrogateHandle {
    fn default() -> Self {
        Self::new()
    }
}

#[allow(dead_code)]
const CURRENT_GEOMETRY_HASH: &str = env!("GEOMETRY_HASH");

/// Validates ONNX metadata against the compiled canonical geometry hash.
///
/// # Errors
/// * `MissingProvenance` - The ONNX file lacks a `geometry_hash` property.
/// * `GeometryMismatch` - The embedded hash does not match `CURRENT_GEOMETRY_HASH`.
/// * `IoError` - The file cannot be read from the filesystem.
pub fn load_surrogate(_path: &str) -> Result<SurrogateHandle, SurrogateLoadError> {
    // TODO(Dev B): Implement ONNX loading and extract metadata_props["geometry_hash"].
    Err(SurrogateLoadError::MissingProvenance)
}
