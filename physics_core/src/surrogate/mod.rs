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

/// ONNX Runtime library path, resolved the way `ort` does: `ORT_DYLIB_PATH` if set and
/// non-empty, otherwise the platform default file name.
fn ort_dylib_path() -> PathBuf {
    match std::env::var("ORT_DYLIB_PATH") {
        Ok(s) if !s.is_empty() => PathBuf::from(s),
        _ => PathBuf::from(if cfg!(target_os = "windows") {
            "onnxruntime.dll"
        } else if cfg!(target_os = "macos") {
            "libonnxruntime.dylib"
        } else {
            "libonnxruntime.so"
        }),
    }
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
}
