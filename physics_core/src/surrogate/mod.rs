// Tier: F
pub mod aero_surrogate;
pub mod fea_surrogate;

use crate::types::SurrogateQueryResult;
use ort::session::Session;
use std::path::{Path, PathBuf};

/// Specific failure modes for surrogate model loading and querying.
#[derive(Debug, PartialEq, Eq)]
pub enum SurrogateLoadError {
    IoError,
    MissingProvenance,
    GeometryMismatch {
        expected: String,
        found: String,
    },
    StubModel,
    /// The model's `config_hash` differs from the hash of the compiled-in `vehicle_config.json`.
    ConfigMismatch {
        expected: String,
        found: String,
    },
    EngineError(String),
    /// The model's input/output tensors are not the supported shapes (see `io_widths`).
    BadModelShape,
    /// The caller passed a different number of inputs than the model declares.
    InputCountMismatch {
        expected: usize,
        found: usize,
    },
    /// An input value is NaN or infinite.
    NonFiniteInput,
    /// The model returned the wrong number of values, a non-finite value, or an
    /// envelope flag that is not exactly 0.0 or 1.0.
    BadOutput,
}

/// Handle to an active ONNX surrogate model session.
pub struct SurrogateHandle {
    pub session: Session,
    /// True when the model's `kind` metadata is not `trained` (only possible
    /// with the `allow_stub_models` feature). Drives `stub_loaded` telemetry.
    pub is_stub: bool,
    /// Number of input values the model declares (tensor shape `[1, input_width]`).
    pub input_width: usize,
    /// Number of values in the model's `predicted_values` output (1..=`PREDICTED_SLOTS`).
    pub predicted_width: usize,
}

/// Engineering bound on the model input width, not a physics limit. The widest planned
/// input today is 4 (aero); raise only by a reviewed change.
pub const MAX_INPUT_WIDTH: usize = 64;

/// Length of `SurrogateQueryResult::predicted_values`.
pub const PREDICTED_SLOTS: usize = 3;

/// True when a model `kind` value must be treated as a stub.
/// Only the exact string `trained` is trusted; anything else is a stub.
pub fn is_stub_kind(kind: &str) -> bool {
    kind != "trained"
}

const CURRENT_GEOMETRY_HASH: &str = env!("GEOMETRY_HASH");

/// Lowercase hex SHA-256 of the raw bytes of `vehicle_config.json`, computed by `build.rs`.
const CURRENT_CONFIG_HASH: &str = env!("CONFIG_HASH");

/// Provenance keys read from the ONNX custom metadata; `None` means the key is absent.
struct Provenance<'a> {
    kind: Option<&'a str>,
    geometry_hash: Option<&'a str>,
    config_hash: Option<&'a str>,
}

/// Decides whether a model's provenance is acceptable. Returns `Ok(is_stub)`.
///
/// Check order: kind missing, stub refused, geometry missing, geometry mismatch,
/// config missing, config mismatch. Hash compares are exact and case-sensitive
/// (both producers emit lowercase hex).
///
/// # Errors
/// `MissingProvenance`, `StubModel`, `GeometryMismatch` or `ConfigMismatch`.
fn check_provenance(
    p: &Provenance,
    allow_stub: bool,
    expected_geometry: &str,
    expected_config: &str,
) -> Result<bool, SurrogateLoadError> {
    let kind = p.kind.ok_or(SurrogateLoadError::MissingProvenance)?;
    let is_stub = is_stub_kind(kind);
    if is_stub && !allow_stub {
        return Err(SurrogateLoadError::StubModel);
    }
    let geometry = p
        .geometry_hash
        .ok_or(SurrogateLoadError::MissingProvenance)?;
    if geometry != expected_geometry {
        return Err(SurrogateLoadError::GeometryMismatch {
            expected: expected_geometry.to_string(),
            found: geometry.to_string(),
        });
    }
    let config = p.config_hash.ok_or(SurrogateLoadError::MissingProvenance)?;
    if config != expected_config {
        return Err(SurrogateLoadError::ConfigMismatch {
            expected: expected_config.to_string(),
            found: config.to_string(),
        });
    }
    Ok(is_stub)
}

/// Width of a `[1, n]` tensor shape, or `None` for any other rank, batch size or
/// dynamic (negative) dimension.
fn width_of(shape: &[i64]) -> Option<usize> {
    match shape {
        [1, n] => usize::try_from(*n).ok(),
        _ => None,
    }
}

/// Validates the declared tensor shapes of a surrogate model and returns
/// `(input_width, predicted_width)`.
///
/// Every tensor must be `[1, n]`. Input `n` is 1..=`MAX_INPUT_WIDTH`, `predicted_values`
/// `n` is 1..=`PREDICTED_SLOTS`, `uncertainty` and `in_validated_envelope` are `[1, 1]`.
/// Dynamic dimensions (-1) are rejected.
///
/// # Errors
/// `BadModelShape` for any other shape.
pub fn io_widths(
    input: &[i64],
    predicted: &[i64],
    uncertainty: &[i64],
    envelope: &[i64],
) -> Result<(usize, usize), SurrogateLoadError> {
    let input_width = width_of(input).ok_or(SurrogateLoadError::BadModelShape)?;
    let predicted_width = width_of(predicted).ok_or(SurrogateLoadError::BadModelShape)?;
    let unc_width = width_of(uncertainty).ok_or(SurrogateLoadError::BadModelShape)?;
    let env_width = width_of(envelope).ok_or(SurrogateLoadError::BadModelShape)?;

    let ok = (1..=MAX_INPUT_WIDTH).contains(&input_width)
        && (1..=PREDICTED_SLOTS).contains(&predicted_width)
        && unc_width == 1
        && env_width == 1;
    if ok {
        Ok((input_width, predicted_width))
    } else {
        Err(SurrogateLoadError::BadModelShape)
    }
}

/// Checks a caller-supplied `input_count` against the model width BEFORE any slice is built
/// from it. This is the upper bound for the FFI: only the model's own width is accepted.
///
/// # Errors
/// `InputCountMismatch` when `input_count` is negative or differs from `expected_width`.
pub fn check_input_count(
    input_count: i32,
    expected_width: usize,
) -> Result<usize, SurrogateLoadError> {
    match usize::try_from(input_count) {
        Ok(n) if n == expected_width => Ok(n),
        Ok(n) => Err(SurrogateLoadError::InputCountMismatch {
            expected: expected_width,
            found: n,
        }),
        Err(_) => Err(SurrogateLoadError::InputCountMismatch {
            expected: expected_width,
            found: 0,
        }),
    }
}

/// Checks the input slice length first, then that every value is finite.
///
/// # Errors
/// `InputCountMismatch`, then `NonFiniteInput`.
pub fn check_inputs(inputs: &[f32], expected_width: usize) -> Result<(), SurrogateLoadError> {
    if inputs.len() != expected_width {
        return Err(SurrogateLoadError::InputCountMismatch {
            expected: expected_width,
            found: inputs.len(),
        });
    }
    if inputs.iter().any(|v| !v.is_finite()) {
        return Err(SurrogateLoadError::NonFiniteInput);
    }
    Ok(())
}

/// Builds a `SurrogateQueryResult` from raw model outputs with exact length and value checks.
///
/// `pred` must hold exactly `predicted_width` values, `unc` and `env` exactly one. All values
/// must be finite and `env` exactly 0.0 or 1.0 (exact sentinel compare). Unused
/// `predicted_values` slots (width < `PREDICTED_SLOTS`) stay 0.0.
///
/// # Errors
/// `BadOutput` on any violation.
pub fn pack_outputs(
    pred: &[f32],
    unc: &[f32],
    env: &[f32],
    predicted_width: usize,
) -> Result<SurrogateQueryResult, SurrogateLoadError> {
    if predicted_width > PREDICTED_SLOTS || pred.len() != predicted_width {
        return Err(SurrogateLoadError::BadOutput);
    }
    let (Some(&uncertainty), Some(&env_value)) = (unc.first(), env.first()) else {
        return Err(SurrogateLoadError::BadOutput);
    };
    if unc.len() != 1 || env.len() != 1 {
        return Err(SurrogateLoadError::BadOutput);
    }
    if pred.iter().any(|v| !v.is_finite()) || !uncertainty.is_finite() {
        return Err(SurrogateLoadError::BadOutput);
    }
    // Exact compare on purpose: the envelope flag is a 0/1 sentinel, anything else is a bad model.
    let in_validated_envelope = if env_value == 1.0 {
        1
    } else if env_value == 0.0 {
        0
    } else {
        return Err(SurrogateLoadError::BadOutput);
    };

    let mut predicted_values = [0.0f32; PREDICTED_SLOTS];
    for (slot, &value) in predicted_values.iter_mut().zip(pred.iter()) {
        *slot = value;
    }
    Ok(SurrogateQueryResult {
        predicted_values,
        uncertainty,
        in_validated_envelope,
    })
}

/// Platform default file name of the ONNX Runtime library.
const DEFAULT_ORT_FILE: &str = if cfg!(target_os = "windows") {
    "onnxruntime.dll"
} else if cfg!(target_os = "macos") {
    "libonnxruntime.dylib"
} else {
    "libonnxruntime.so"
};

/// Where to load ONNX Runtime from, in order: `env` (`ORT_DYLIB_PATH`) if set and non-empty; else the default file name
/// NEXT TO THE EXECUTABLE (`exe_dir`) if `exists` says it is there; else the bare default name (the system search).
///
/// The packaged helpers (`run_campaign.exe`, `validate_models.exe`) ship `onnxruntime.dll` in their own folder, so they need no
/// environment variable. An absolute path is returned on purpose: Windows would otherwise find an older `onnxruntime.dll` in
/// `System32` first. Pure (no file or environment access) so each branch is tested.
fn resolve_ort_path(
    env: Option<&str>,
    exe_dir: Option<&Path>,
    exists: &dyn Fn(&Path) -> bool,
) -> PathBuf {
    if let Some(s) = env
        && !s.is_empty()
    {
        return PathBuf::from(s);
    }
    if let Some(dir) = exe_dir {
        let beside = dir.join(DEFAULT_ORT_FILE);
        if exists(&beside) {
            return beside;
        }
    }
    PathBuf::from(DEFAULT_ORT_FILE)
}

/// ONNX Runtime library path (see `resolve_ort_path`).
fn ort_dylib_path() -> PathBuf {
    let env = std::env::var("ORT_DYLIB_PATH").ok();
    let exe_dir = std::env::current_exe()
        .ok()
        .and_then(|p| p.parent().map(Path::to_path_buf));
    resolve_ort_path(env.as_deref(), exe_dir.as_deref(), &|p| p.is_file())
}

/// Loads the ONNX Runtime library from `dylib`. Unlike `ort::init()`, a missing or
/// incompatible library is returned as an error instead of aborting the host process.
/// Repeated calls with a loadable library are harmless.
fn init_ort_from(dylib: &Path) -> Result<(), SurrogateLoadError> {
    let builder =
        ort::init_from(dylib).map_err(|e| SurrogateLoadError::EngineError(e.to_string()))?;
    let _ = builder.with_name("UAV_Simulator_Physics").commit();
    Ok(())
}

/// Reads the declared shape of the named session outlet.
fn outlet_shape(outlets: &[ort::value::Outlet], name: &str) -> Option<Vec<i64>> {
    let outlet = outlets.iter().find(|o| o.name() == name)?;
    outlet.dtype().tensor_shape().map(|s| s.to_vec())
}

/// Validates ONNX metadata against the compiled canonical geometry hash and loads the ML session.
///
/// # Errors
/// Returns EngineError if the ONNX runtime library cannot be loaded, or the model cannot be parsed.
/// Returns IoError if the path is not a readable file.
/// Returns MissingProvenance if the ONNX file lacks a geometry_hash, config_hash or kind property.
/// Returns GeometryMismatch if the embedded hash does not match CURRENT_GEOMETRY_HASH.
/// Returns ConfigMismatch if the embedded config_hash does not match CURRENT_CONFIG_HASH.
/// Returns StubModel if the model is a stub and the allow_stub_models feature is not enabled.
/// Returns BadModelShape if the model inputs/outputs are not the supported shapes.
pub fn load_surrogate(path: &str) -> Result<SurrogateHandle, SurrogateLoadError> {
    // Checked before touching ORT so a bad path never needs the runtime library.
    if !Path::new(path).is_file() {
        return Err(SurrogateLoadError::IoError);
    }
    init_ort_from(&ort_dylib_path())?;

    let session = Session::builder()
        .map_err(|e| SurrogateLoadError::EngineError(e.to_string()))?
        .commit_from_file(path)
        .map_err(|_| SurrogateLoadError::IoError)?;

    let is_stub;
    {
        let metadata = session
            .metadata()
            .map_err(|e| SurrogateLoadError::EngineError(e.to_string()))?;

        let kind = metadata.custom("kind");
        let geometry_hash = metadata.custom("geometry_hash");
        let config_hash = metadata.custom("config_hash");
        is_stub = check_provenance(
            &Provenance {
                kind: kind.as_deref(),
                geometry_hash: geometry_hash.as_deref(),
                config_hash: config_hash.as_deref(),
            },
            cfg!(feature = "allow_stub_models"),
            CURRENT_GEOMETRY_HASH,
            CURRENT_CONFIG_HASH,
        )?;
    }

    let (input_width, predicted_width) = declared_widths(&session)?;
    Ok(SurrogateHandle {
        session,
        is_stub,
        input_width,
        predicted_width,
    })
}

// ---------------------------------------------------------------------------------------------- per-build models

/// Lowercase hex SHA-256 of `bytes`.
#[must_use]
pub fn sha256_hex(bytes: &[u8]) -> String {
    use sha2::{Digest, Sha256};
    hex::encode(Sha256::digest(bytes))
}

/// Binding hash of a per-build model: SHA-256 of `config_hash|geometry_hash|training_data_sha256|trainer_version`
/// (UTF-8). Produced by `offline_pipeline/cfd/train_vehicle_aero.py`, recomputed here.
#[must_use]
pub fn binding_hash_of(config: &str, geometry: &str, data: &str, trainer_version: &str) -> String {
    sha256_hex(format!("{config}|{geometry}|{data}|{trainer_version}").as_bytes())
}

/// What the caller knows about the build a model must belong to (all lowercase hex).
#[derive(Debug, Clone, Copy)]
pub struct BuildBinding<'a> {
    /// SHA-256 of the build's CFD geometry STL.
    pub geometry_hash: &'a str,
    /// SHA-256 of the build's `vehicle_config.json`.
    pub config_hash: &'a str,
    /// The binding hash the model must carry.
    pub binding_hash: &'a str,
    /// SHA-256 the model FILE must have, if the caller knows it (from the sidecar).
    pub model_sha256: Option<&'a str>,
}

/// Facts about a loaded per-build model, for the run manifest.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ModelInfo {
    /// SHA-256 of the model file as read.
    pub model_sha256: String,
    /// `kind` metadata (`trained` or `stub`).
    pub kind: String,
    /// The binding hash the model carries (checked).
    pub binding_hash: String,
    /// `cfd_sanity_status` metadata, if present.
    pub cfd_sanity_status: Option<String>,
    /// `propellers` metadata, if present.
    pub propellers: Option<String>,
    /// `provenance` metadata, if present (for example "SYNTHETIC formula, not CFD | NOT VALIDATED ...").
    pub provenance: Option<String>,
    /// `holdout_rms_n` metadata, if present: leave-one-speed-out RMS force error in N, as text (`none` when it could not be computed).
    pub holdout_rms_n: Option<String>,
}

/// SHA-256 of the committed SYNTHETIC fixture model `offline_pipeline/cfd/sample/synthetic_quad_standard_aero.onnx` (formula, not CFD).
/// The only stub the per-build loader accepts. Equals `model_sha256` of its `.model.json` (a test checks this against the file).
pub const SYNTHETIC_FIXTURE_SHA256: &str =
    "f41c465cd0e5372da7ee0119327e60ba82b6b52a689adabaa3e8bae185fccb45";

/// Refuses every stub model except the committed SYNTHETIC fixture (see `SYNTHETIC_FIXTURE_SHA256`). A trained model always passes.
///
/// # Errors
/// `BuildLoadError::StubNotFixture` for a stub whose file hash is not the fixture's.
pub fn stub_gate(is_stub: bool, model_sha256: &str) -> Result<(), BuildLoadError> {
    if is_stub && model_sha256 != SYNTHETIC_FIXTURE_SHA256 {
        return Err(BuildLoadError::StubNotFixture {
            found: model_sha256.to_string(),
        });
    }
    Ok(())
}

/// How a loaded per-build model must be labelled everywhere (manifest, report, app): `(text, level)` with level `danger` or `warn`.
/// `trained` is the only kind that is a CFD result, and it is still PRELIMINARY and UNVALIDATED. A stub whose provenance starts with
/// `SYNTHETIC` is the formula fixture; any other stub is labelled as a stub.
#[must_use]
pub fn surrogate_status_text(kind: &str, provenance: Option<&str>) -> (&'static str, &'static str) {
    if kind == "trained" {
        ("CFD surrogate: PRELIMINARY, UNVALIDATED", "warn")
    } else if provenance.is_some_and(|p| p.starts_with("SYNTHETIC")) {
        ("CFD surrogate: SYNTHETIC (formula, not CFD)", "danger")
    } else {
        ("CFD surrogate: STUB (not validated)", "danger")
    }
}

/// Why a per-build model was refused.
#[derive(Debug, PartialEq, Eq)]
pub enum BuildLoadError {
    /// The generic checks failed (file, runtime, kind, geometry hash, config hash, shapes).
    Load(SurrogateLoadError),
    /// `binding_hash`, `training_data_sha256` or `trainer_version` is missing from the model.
    MissingBindingMetadata,
    /// The model's `binding_hash` is not the hash of its own parts.
    BindingInconsistent,
    /// The model's binding hash differs from the one the caller expects.
    BindingMismatch {
        /// Hash the caller expects.
        expected: String,
        /// Hash the model carries.
        found: String,
    },
    /// A stub model other than the committed SYNTHETIC fixture (`SYNTHETIC_FIXTURE_SHA256`) was offered: refused even when
    /// `allow_stub_models` is on, so a stub of unknown origin can never be taken for a CFD result.
    StubNotFixture {
        /// SHA-256 of the stub model file read.
        found: String,
    },
    /// The model file's SHA-256 differs from the one the caller expects.
    ModelFileMismatch {
        /// Hash the caller expects.
        expected: String,
        /// Hash of the file read.
        found: String,
    },
}

impl From<SurrogateLoadError> for BuildLoadError {
    fn from(e: SurrogateLoadError) -> Self {
        Self::Load(e)
    }
}

/// Binding-related metadata of a per-build model; `None` = key absent.
struct BuildMeta<'a> {
    config_hash: &'a str,
    geometry_hash: &'a str,
    binding_hash: Option<&'a str>,
    training_data_sha256: Option<&'a str>,
    trainer_version: Option<&'a str>,
}

/// Checks the binding: all three parts present, `binding_hash` equals the hash of its parts, and equals what the caller expects.
fn check_build_binding(
    m: &BuildMeta<'_>,
    expected: &BuildBinding<'_>,
) -> Result<String, BuildLoadError> {
    let (Some(binding), Some(data), Some(version)) =
        (m.binding_hash, m.training_data_sha256, m.trainer_version)
    else {
        return Err(BuildLoadError::MissingBindingMetadata);
    };
    if binding_hash_of(m.config_hash, m.geometry_hash, data, version) != binding {
        return Err(BuildLoadError::BindingInconsistent);
    }
    if binding != expected.binding_hash {
        return Err(BuildLoadError::BindingMismatch {
            expected: expected.binding_hash.to_string(),
            found: binding.to_string(),
        });
    }
    Ok(binding.to_string())
}

/// Loads a per-build surrogate model and checks it belongs to THIS build.
///
/// Same checks as `load_surrogate`, but against the hashes in `expected` instead of the compiled-in ones, plus the binding
/// (`binding_hash` is recomputed from the model's metadata) and, if given, the model file's own SHA-256. The compiled-in hashes
/// and the FFI loader are untouched. Stub models need the `allow_stub_models` feature, as with `load_surrogate`.
///
/// # Errors
/// `BuildLoadError::Load` wraps every error of `load_surrogate` (`IoError`, `EngineError`, `MissingProvenance`, `StubModel`,
/// `GeometryMismatch`, `ConfigMismatch`, `BadModelShape`); the other variants are the binding and file-hash failures.
pub fn load_surrogate_for_build(
    path: &str,
    expected: &BuildBinding<'_>,
) -> Result<(SurrogateHandle, ModelInfo), BuildLoadError> {
    let bytes = std::fs::read(path).map_err(|_| SurrogateLoadError::IoError)?;
    let model_sha256 = sha256_hex(&bytes);
    if let Some(want) = expected.model_sha256
        && want != model_sha256
    {
        return Err(BuildLoadError::ModelFileMismatch {
            expected: want.to_string(),
            found: model_sha256,
        });
    }
    init_ort_from(&ort_dylib_path())?;
    let session = Session::builder()
        .map_err(|e| SurrogateLoadError::EngineError(e.to_string()))?
        .commit_from_memory(&bytes)
        .map_err(|e| SurrogateLoadError::EngineError(e.to_string()))?;
    let (is_stub, info) = {
        let metadata = session
            .metadata()
            .map_err(|e| SurrogateLoadError::EngineError(e.to_string()))?;
        let get = |k: &str| metadata.custom(k);
        let (kind, geometry, config) = (get("kind"), get("geometry_hash"), get("config_hash"));
        let is_stub = check_provenance(
            &Provenance {
                kind: kind.as_deref(),
                geometry_hash: geometry.as_deref(),
                config_hash: config.as_deref(),
            },
            cfg!(feature = "allow_stub_models"),
            expected.geometry_hash,
            expected.config_hash,
        )?;
        stub_gate(is_stub, &model_sha256)?;
        let (binding, data, version) = (
            get("binding_hash"),
            get("training_data_sha256"),
            get("trainer_version"),
        );
        let binding = check_build_binding(
            &BuildMeta {
                config_hash: config.as_deref().unwrap_or_default(),
                geometry_hash: geometry.as_deref().unwrap_or_default(),
                binding_hash: binding.as_deref(),
                training_data_sha256: data.as_deref(),
                trainer_version: version.as_deref(),
            },
            expected,
        )?;
        (
            is_stub,
            ModelInfo {
                model_sha256,
                kind: kind.unwrap_or_default(),
                binding_hash: binding,
                cfd_sanity_status: get("cfd_sanity_status"),
                propellers: get("propellers"),
                provenance: get("provenance"),
                holdout_rms_n: get("holdout_rms_n"),
            },
        )
    };
    let (input_width, predicted_width) = declared_widths(&session)?;
    Ok((
        SurrogateHandle {
            session,
            is_stub,
            input_width,
            predicted_width,
        },
        info,
    ))
}

/// Reads and validates the session's declared tensor shapes.
fn declared_widths(session: &Session) -> Result<(usize, usize), SurrogateLoadError> {
    if session.inputs().len() != 1 {
        return Err(SurrogateLoadError::BadModelShape);
    }
    let input =
        outlet_shape(session.inputs(), "inputs").ok_or(SurrogateLoadError::BadModelShape)?;
    let outs = session.outputs();
    let predicted =
        outlet_shape(outs, "predicted_values").ok_or(SurrogateLoadError::BadModelShape)?;
    let uncertainty = outlet_shape(outs, "uncertainty").ok_or(SurrogateLoadError::BadModelShape)?;
    let envelope =
        outlet_shape(outs, "in_validated_envelope").ok_or(SurrogateLoadError::BadModelShape)?;
    io_widths(&input, &predicted, &uncertainty, &envelope)
}

/// Queries the loaded ONNX surrogate model generically.
///
/// # Units
/// * `inputs` - Context-dependent flat array of floats. Length must equal the model's
///   declared input width and every value must be finite.
///
/// # Errors
/// Returns InputCountMismatch / NonFiniteInput for bad inputs (nothing is run).
/// Returns EngineError if tensor creation, execution, or extraction fails.
/// Returns BadOutput if the model output is malformed (see `pack_outputs`).
pub fn query_model(
    handle: &mut SurrogateHandle,
    inputs: &[f32],
) -> Result<SurrogateQueryResult, SurrogateLoadError> {
    check_inputs(inputs, handle.input_width)?;

    let input_tensor = ort::value::Tensor::from_array(([1, inputs.len()], inputs.to_vec()))
        .map_err(|e| SurrogateLoadError::EngineError(e.to_string()))?;

    let outputs = handle
        .session
        .run(ort::inputs!["inputs" => input_tensor])
        .map_err(|e| SurrogateLoadError::EngineError(e.to_string()))?;

    let pred = outputs["predicted_values"]
        .try_extract_tensor::<f32>()
        .map_err(|e| SurrogateLoadError::EngineError(e.to_string()))?;
    let unc = outputs["uncertainty"]
        .try_extract_tensor::<f32>()
        .map_err(|e| SurrogateLoadError::EngineError(e.to_string()))?;
    let env = outputs["in_validated_envelope"]
        .try_extract_tensor::<f32>()
        .map_err(|e| SurrogateLoadError::EngineError(e.to_string()))?;

    let result = pack_outputs(pred.1, unc.1, env.1, handle.predicted_width)?;
    Ok(force_unvalidated_if_stub(result, handle.is_stub))
}

/// A stub model is never inside a validated envelope, whatever the model file says
/// (review finding A; HZ: a stub must not read as validated). Forces
/// `in_validated_envelope` to 0 when `is_stub`; every other field is unchanged.
pub fn force_unvalidated_if_stub(
    mut result: SurrogateQueryResult,
    is_stub: bool,
) -> SurrogateQueryResult {
    if is_stub {
        result.in_validated_envelope = 0;
    }
    result
}

#[cfg(test)]
mod tests {
    use super::*;

    // Hand calc: a stub that reports envelope 1 (the shipped stubs hard-code 1.0) must read 0;
    // a trained model keeps its own flag (1 stays 1, 0 stays 0); other fields are untouched.
    #[test]
    fn test_force_unvalidated_if_stub_truth_table() {
        let make = |env: i32| SurrogateQueryResult {
            predicted_values: [1.5, -2.5, 3.5],
            uncertainty: 0.25,
            in_validated_envelope: env,
        };
        for (env, is_stub, want) in [(1, true, 0), (0, true, 0), (1, false, 1), (0, false, 0)] {
            let r = force_unvalidated_if_stub(make(env), is_stub);
            assert_eq!(r.in_validated_envelope, want, "env {env} stub {is_stub}");
            assert_eq!(r.predicted_values, [1.5, -2.5, 3.5]);
            assert_eq!(r.uncertainty, 0.25);
        }
    }

    #[test]
    fn test_is_stub_kind() {
        // Only the exact string "trained" is trusted (HZ-01). Case and whitespace variants,
        // the empty string and "stub" are all stubs.
        assert!(!is_stub_kind("trained"));
        assert!(is_stub_kind("stub"));
        assert!(is_stub_kind(""));
        assert!(is_stub_kind("Trained"));
        assert!(is_stub_kind("trained "));
    }

    const GEO: &str = "geo";
    const CFG: &str = "cfg";
    const X: &str = "0000000000000000000000000000000000000000000000000000000000000000";

    fn prov<'a>(
        kind: Option<&'a str>,
        geometry_hash: Option<&'a str>,
        config_hash: Option<&'a str>,
    ) -> Provenance<'a> {
        Provenance {
            kind,
            geometry_hash,
            config_hash,
        }
    }

    #[test]
    fn test_check_provenance_accepts_matching_hashes() {
        // Truth table: both hashes equal the expected ones; the result is only the stub flag.
        let p = prov(Some("trained"), Some(GEO), Some(CFG));
        assert_eq!(check_provenance(&p, false, GEO, CFG), Ok(false));
        let p = prov(Some("stub"), Some(GEO), Some(CFG));
        assert_eq!(check_provenance(&p, true, GEO, CFG), Ok(true));
    }

    #[test]
    fn test_check_provenance_config_mismatch() {
        let p = prov(Some("trained"), Some(GEO), Some(X));
        assert_eq!(
            check_provenance(&p, false, GEO, CFG),
            Err(SurrogateLoadError::ConfigMismatch {
                expected: CFG.to_string(),
                found: X.to_string()
            })
        );
        // Case, empty, trailing newline and the geometry hash given as config all differ.
        let cfg_with_newline = format!("{CFG}\n");
        for bad in ["CFG", "", cfg_with_newline.as_str(), GEO] {
            let p = prov(Some("trained"), Some(GEO), Some(bad));
            assert!(
                matches!(
                    check_provenance(&p, false, GEO, CFG),
                    Err(SurrogateLoadError::ConfigMismatch { .. })
                ),
                "{bad:?}"
            );
        }
    }

    #[test]
    fn test_check_provenance_missing_keys() {
        let missing = Err(SurrogateLoadError::MissingProvenance);
        let cases = [
            prov(None, Some(GEO), Some(CFG)),
            prov(Some("trained"), None, Some(CFG)),
            prov(Some("trained"), Some(GEO), None),
        ];
        for p in &cases {
            assert_eq!(check_provenance(p, false, GEO, CFG), missing);
        }
    }

    #[test]
    fn test_check_provenance_check_order() {
        // Stub is refused before any hash is looked at.
        let p = prov(Some("stub"), Some(X), Some(X));
        assert_eq!(
            check_provenance(&p, false, GEO, CFG),
            Err(SurrogateLoadError::StubModel)
        );
        // Geometry is checked before config.
        let p = prov(Some("trained"), Some(X), Some(X));
        assert!(matches!(
            check_provenance(&p, false, GEO, CFG),
            Err(SurrogateLoadError::GeometryMismatch { .. })
        ));
    }

    #[test]
    fn test_compiled_config_hash_format() {
        // Lowercase hex SHA-256 is 64 characters; it must not be the geometry hash.
        assert_eq!(CURRENT_CONFIG_HASH.len(), 64);
        assert!(
            CURRENT_CONFIG_HASH
                .bytes()
                .all(|b| b.is_ascii_digit() || (b'a'..=b'f').contains(&b))
        );
        assert_ne!(CURRENT_CONFIG_HASH, CURRENT_GEOMETRY_HASH);
    }

    #[test]
    fn test_build_rs_sha256_matches_reference_sample() {
        // Reference: GNU coreutils `sha256sum tools/hash_sample.json` (139 bytes, LF).
        // Python twin: tools/test_hash_equality.py asserts the same string.
        assert_eq!(
            env!("HASH_SAMPLE_SHA256"),
            "9a9e1c38f305cf448b252551f0ad5ad19f25cf4023680f1f343083d1d93b5f5b"
        );
    }

    #[test]
    fn test_io_widths_accepts_shipped_shapes() {
        // Shapes read from the shipped stub models with the `onnx` package:
        // aero in [1,4], predicted [1,3]; fea in [1,1], predicted [1,1]; unc/env [1,1].
        assert_eq!(io_widths(&[1, 4], &[1, 3], &[1, 1], &[1, 1]), Ok((4, 3)));
        assert_eq!(io_widths(&[1, 1], &[1, 1], &[1, 1], &[1, 1]), Ok((1, 1)));
        // Boundary: widest accepted input.
        assert_eq!(io_widths(&[1, 64], &[1, 3], &[1, 1], &[1, 1]), Ok((64, 3)));
    }

    #[test]
    fn test_io_widths_rejects_bad_shapes() {
        let bad = Err(SurrogateLoadError::BadModelShape);
        // Input: dynamic dims, zero, over the bound, wrong rank, wrong batch.
        for input in [
            &[-1i64, 4][..],
            &[1, -1],
            &[1, 0],
            &[1, 65],
            &[4],
            &[1, 1, 4],
            &[2, 4],
        ] {
            assert_eq!(
                io_widths(input, &[1, 3], &[1, 1], &[1, 1]),
                bad,
                "{input:?}"
            );
        }
        // Predicted wider than the 3 result slots, or empty.
        assert_eq!(io_widths(&[1, 4], &[1, 4], &[1, 1], &[1, 1]), bad);
        assert_eq!(io_widths(&[1, 4], &[1, 0], &[1, 1], &[1, 1]), bad);
        // Uncertainty and envelope must be exactly [1,1].
        assert_eq!(io_widths(&[1, 4], &[1, 3], &[1, 2], &[1, 1]), bad);
        assert_eq!(io_widths(&[1, 4], &[1, 3], &[1, 1], &[1, 2]), bad);
    }

    #[test]
    fn test_check_input_count() {
        assert_eq!(check_input_count(4, 4), Ok(4));
        for n in [5, 3, 0, -1, i32::MAX] {
            assert!(check_input_count(n, 4).is_err(), "{n}");
        }
        assert_eq!(
            check_input_count(5, 4),
            Err(SurrogateLoadError::InputCountMismatch {
                expected: 4,
                found: 5
            })
        );
    }

    #[test]
    fn test_check_inputs() {
        assert_eq!(check_inputs(&[1.0, 2.0, 3.0, 4.0], 4), Ok(()));
        for (n, vals) in [(3usize, &[1.0f32, 2.0, 3.0][..]), (5, &[0.0; 5][..])] {
            assert_eq!(
                check_inputs(vals, 4),
                Err(SurrogateLoadError::InputCountMismatch {
                    expected: 4,
                    found: n
                })
            );
        }
        // A one-value (fea) input sent to a four-wide (aero) model.
        assert!(matches!(
            check_inputs(&[1.0], 4),
            Err(SurrogateLoadError::InputCountMismatch { .. })
        ));
        for bad in [f32::NAN, f32::INFINITY, f32::NEG_INFINITY] {
            assert_eq!(
                check_inputs(&[1.0, 2.0, bad, 4.0], 4),
                Err(SurrogateLoadError::NonFiniteInput)
            );
        }
        // Count is checked before finiteness.
        assert!(matches!(
            check_inputs(&[f32::NAN; 3], 4),
            Err(SurrogateLoadError::InputCountMismatch { .. })
        ));
    }

    #[test]
    fn test_pack_outputs_exact() {
        // All values are exact in binary f32.
        let r = pack_outputs(&[1.5, -2.0, 0.25], &[0.125], &[1.0], 3).unwrap();
        assert_eq!(r.predicted_values, [1.5, -2.0, 0.25]);
        assert_eq!(r.uncertainty, 0.125);
        assert_eq!(r.in_validated_envelope, 1);

        let r = pack_outputs(&[0.75], &[0.5], &[0.0], 1).unwrap();
        assert_eq!(r.predicted_values, [0.75, 0.0, 0.0]);
        assert_eq!(r.in_validated_envelope, 0);
    }

    #[test]
    fn test_pack_outputs_rejects_wrong_count_and_nonfinite() {
        let bad = |p: &[f32], u: &[f32], e: &[f32]| {
            assert_eq!(
                pack_outputs(p, u, e, 3).map(|_| ()),
                Err(SurrogateLoadError::BadOutput),
                "{p:?} {u:?} {e:?}"
            );
        };
        let p = [1.0f32, 2.0, 3.0];
        bad(&[1.0, 2.0], &[0.1], &[1.0]);
        bad(&[1.0, 2.0, 3.0, 4.0], &[0.1], &[1.0]);
        bad(&p, &[], &[1.0]);
        bad(&p, &[0.1, 0.2], &[1.0]);
        bad(&p, &[0.1], &[]);
        bad(&[1.0, f32::NAN, 3.0], &[0.1], &[1.0]);
        bad(&p, &[f32::INFINITY], &[1.0]);
        bad(&p, &[0.1], &[f32::NAN]);
        bad(&p, &[0.1], &[0.5]); // previously truncated silently to 0
        bad(&p, &[0.1], &[2.0]);
        bad(&p, &[0.1], &[-1.0]);
    }

    #[test]
    fn test_load_surrogate_missing_file_is_io_error() {
        let path = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/no_such_model.onnx");
        assert_eq!(
            load_surrogate(path).map(|_| ()),
            Err(SurrogateLoadError::IoError)
        );
    }

    #[test]
    fn test_init_ort_from_missing_dylib_is_engine_error() {
        // Valid only while no test in this process has loaded ORT (none does).
        let r = init_ort_from(Path::new("/nonexistent/libonnxruntime.so"));
        assert!(
            matches!(r, Err(SurrogateLoadError::EngineError(_))),
            "{r:?}"
        );
    }

    // ------------------------------------------------------------------ runtime path, stub gate, labels (no ONNX Runtime needed)

    #[test]
    fn test_ort_path_env_wins_even_when_a_library_sits_beside_the_exe() {
        // Branch 1: a non-empty ORT_DYLIB_PATH is returned verbatim.
        let always = |_: &Path| true;
        let r = resolve_ort_path(
            Some("/opt/ort/lib.so"),
            Some(Path::new("/app/bin")),
            &always,
        );
        assert_eq!(r, PathBuf::from("/opt/ort/lib.so"));
    }

    #[test]
    fn test_ort_path_beside_the_exe_when_env_is_unset_or_empty() {
        // Branch 2: no usable env value, the default file exists next to the exe -> that absolute path.
        let always = |_: &Path| true;
        let want = Path::new("/app/bin").join(DEFAULT_ORT_FILE);
        assert_eq!(
            resolve_ort_path(None, Some(Path::new("/app/bin")), &always),
            want
        );
        assert_eq!(
            resolve_ort_path(Some(""), Some(Path::new("/app/bin")), &always),
            want
        );
    }

    #[test]
    fn test_ort_path_falls_back_to_the_bare_name() {
        // Branch 3: nothing beside the exe, or no exe folder known -> the bare default name (system search).
        let never = |_: &Path| false;
        let bare = PathBuf::from(DEFAULT_ORT_FILE);
        assert_eq!(
            resolve_ort_path(None, Some(Path::new("/app/bin")), &never),
            bare
        );
        assert_eq!(resolve_ort_path(None, None, &|_: &Path| true), bare);
    }

    #[test]
    fn test_stub_gate_accepts_only_the_committed_fixture_stub() {
        let other = "0".repeat(64);
        // a trained model is never gated, whatever its hash
        assert_eq!(stub_gate(false, &other), Ok(()));
        // the committed fixture is the one accepted stub
        assert_eq!(stub_gate(true, SYNTHETIC_FIXTURE_SHA256), Ok(()));
        // any other stub is refused and its hash is reported
        assert_eq!(
            stub_gate(true, &other),
            Err(BuildLoadError::StubNotFixture { found: other })
        );
    }

    #[test]
    fn test_fixture_constant_is_the_hash_of_the_committed_fixture_file() {
        let dir = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../offline_pipeline/cfd/sample");
        let bytes = std::fs::read(dir.join("synthetic_quad_standard_aero.onnx")).unwrap();
        assert_eq!(sha256_hex(&bytes), SYNTHETIC_FIXTURE_SHA256);
        let side: serde_json::Value = serde_json::from_str(
            &std::fs::read_to_string(dir.join("synthetic_quad_standard_aero.onnx.model.json"))
                .unwrap(),
        )
        .unwrap();
        assert_eq!(side["model_sha256"], SYNTHETIC_FIXTURE_SHA256);
    }

    #[test]
    fn test_status_text_names_each_kind_and_never_calls_a_stub_validated() {
        let syn = Some("SYNTHETIC formula, not CFD | NOT VALIDATED against any measurement");
        assert_eq!(
            surrogate_status_text("stub", syn),
            ("CFD surrogate: SYNTHETIC (formula, not CFD)", "danger")
        );
        // a stub that does not say SYNTHETIC, or says nothing, is still a stub
        assert_eq!(
            surrogate_status_text("stub", Some("preliminary CFD, unvalidated")),
            ("CFD surrogate: STUB (not validated)", "danger")
        );
        assert_eq!(
            surrogate_status_text("stub", None),
            ("CFD surrogate: STUB (not validated)", "danger")
        );
        // only the exact kind `trained` is a (preliminary) CFD result; the provenance text cannot upgrade a stub
        assert_eq!(
            surrogate_status_text(
                "trained",
                Some("preliminary CFD, unvalidated | NOT VALIDATED")
            ),
            ("CFD surrogate: PRELIMINARY, UNVALIDATED", "warn")
        );
        assert_eq!(
            surrogate_status_text("Trained", None),
            ("CFD surrogate: STUB (not validated)", "danger")
        );
    }

    // ------------------------------------------------------------------ per-build binding (no ONNX Runtime needed)

    fn expect<'a>(binding: &'a str) -> BuildBinding<'a> {
        BuildBinding {
            geometry_hash: "g",
            config_hash: "c",
            binding_hash: binding,
            model_sha256: None,
        }
    }

    fn meta<'a>(
        binding: Option<&'a str>,
        data: Option<&'a str>,
        version: Option<&'a str>,
    ) -> BuildMeta<'a> {
        BuildMeta {
            config_hash: "c",
            geometry_hash: "g",
            binding_hash: binding,
            training_data_sha256: data,
            trainer_version: version,
        }
    }

    #[test]
    fn test_binding_hash_is_the_hash_of_its_four_parts() {
        // independent (Python hashlib) value: sha256("c|g|d|1")
        let want = binding_hash_of("c", "g", "d", "1");
        assert_eq!(want.len(), 64);
        assert_eq!(binding_hash_of("c", "g", "d", "1"), want);
        assert_ne!(binding_hash_of("c", "g", "d", "2"), want);
        assert_ne!(binding_hash_of("g", "c", "d", "1"), want);
        assert_ne!(binding_hash_of("c", "g", "e", "1"), want);
        assert_eq!(
            binding_hash_of("a", "b", "c", "1"),
            "4128761d4504d08c6bf2197fd092e345ffeabd124bfea6e96c0b823b992c5206"
        );
    }

    #[test]
    fn test_check_build_binding_accepts_a_consistent_model() {
        let good = binding_hash_of("c", "g", "d", "1");
        let got = check_build_binding(&meta(Some(&good), Some("d"), Some("1")), &expect(&good));
        assert_eq!(got, Ok(good));
    }

    #[test]
    fn test_check_build_binding_refusals() {
        let good = binding_hash_of("c", "g", "d", "1");
        // any of the three binding keys absent
        for m in [
            meta(None, Some("d"), Some("1")),
            meta(Some(&good), None, Some("1")),
            meta(Some(&good), Some("d"), None),
        ] {
            assert_eq!(
                check_build_binding(&m, &expect(&good)),
                Err(BuildLoadError::MissingBindingMetadata)
            );
        }
        // the stored binding is not the hash of the stored parts (a part was edited)
        assert_eq!(
            check_build_binding(&meta(Some(&good), Some("other"), Some("1")), &expect(&good)),
            Err(BuildLoadError::BindingInconsistent)
        );
        // consistent, but for another build than the caller expects
        let other = "0".repeat(64);
        assert_eq!(
            check_build_binding(&meta(Some(&good), Some("d"), Some("1")), &expect(&other)),
            Err(BuildLoadError::BindingMismatch {
                expected: other,
                found: good
            })
        );
    }

    #[test]
    fn test_per_build_loader_missing_file_is_an_io_error_before_the_runtime() {
        let r = load_surrogate_for_build(
            concat!(env!("CARGO_MANIFEST_DIR"), "/tests/no_such_model.onnx"),
            &expect("x"),
        );
        assert_eq!(
            r.err(),
            Some(BuildLoadError::Load(SurrogateLoadError::IoError))
        );
    }
}
