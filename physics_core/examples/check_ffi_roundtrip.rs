// Tier: F
//! Manual end-to-end check (HUMAN-TODO B-13) of the FFI model functions against the REAL stub models in
//! `offline_pipeline/trained_models/` and a real ONNX Runtime: load, query, error codes, unload.
//! Not a unit test: it needs an ONNX Runtime that `ort 2.0.0-rc.13` accepts (1.27.x).
//! Usage: ORT_DYLIB_PATH=/path/to/libonnxruntime.so.1.27.1 cargo run --features allow_stub_models --example check_ffi_roundtrip
//! Prints PASS / FAIL per check and exits 1 on any failure. The stub models are `kind: stub` (not trained, not validated).
use physics_core::surrogate::SurrogateHandle;
use physics_core::types::SurrogateQueryResult;
use physics_core::{ffi_load_surrogate_model, ffi_query_surrogate, ffi_unload_surrogate_model};
use std::ffi::CString;
use std::path::PathBuf;

fn cpath(p: &std::path::Path) -> CString {
    CString::new(p.to_string_lossy().as_bytes()).expect("path has no NUL")
}

/// Loads `path`; returns (code, handle).
fn load(path: &std::path::Path) -> (i32, *mut SurrogateHandle) {
    let c = cpath(path);
    let mut h: *mut SurrogateHandle = std::ptr::null_mut();
    // SAFETY: `c` is a valid NUL-terminated string and `h` is a valid, aligned out-pointer, both alive for the call.
    let code = unsafe { ffi_load_surrogate_model(c.as_ptr(), &raw mut h) };
    (code, h)
}

fn query(
    h: *mut SurrogateHandle,
    inputs: &[f32],
    count: i32,
) -> (i32, Option<SurrogateQueryResult>) {
    let mut out = SurrogateQueryResult {
        predicted_values: [0.0; 3],
        uncertainty: 0.0,
        in_validated_envelope: 0,
    };
    // SAFETY: `inputs` is a live slice (the caller passes a count no larger than its length except in the deliberate
    // wrong-count check, which the FFI rejects BEFORE reading), `out` is a valid out-pointer.
    let code = unsafe { ffi_query_surrogate(h, inputs.as_ptr(), count, &raw mut out) };
    (code, (code == 0).then_some(out))
}

fn main() {
    let root = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("..");
    let models = root.join("offline_pipeline/trained_models");
    let mut failed = false;
    let mut check = |name: &str, ok: bool| {
        println!("{} {name}", if ok { "PASS" } else { "FAIL" });
        failed |= !ok;
    };

    for (label, file) in [("aero", "aero_stub.onnx"), ("fea", "fea_stub.onnx")] {
        let (code, h) = load(&models.join(file));
        check(
            &format!("{label} stub loads (code 0, got {code})"),
            code == 0,
        );
        if code != 0 {
            continue;
        }
        // The input width is not known to this example: the FFI answers code 3 for every wrong width, so find the one it accepts.
        let probe = [0.1_f32; 64];
        let width = (1..=64).find(|&n| query(h, &probe, n).0 != 3);
        check(
            &format!("{label} input width found by probing (got {width:?})"),
            width.is_some(),
        );
        let Some(n) = width else { continue };
        let (qc, res) = query(h, &probe, n);
        check(&format!("{label} query returns code 0 (got {qc})"), qc == 0);
        if let Some(r) = res {
            check(
                &format!("{label} outputs are finite"),
                r.predicted_values.iter().all(|v| v.is_finite()) && r.uncertainty.is_finite(),
            );
            check(
                &format!("{label} a stub never reports in_validated_envelope"),
                r.in_validated_envelope == 0,
            );
        }
        check(
            &format!("{label} wrong input count is code 3"),
            query(h, &probe, n + 1).0 == 3 || n == 64,
        );
        let mut nan = probe;
        nan[0] = f32::NAN;
        check(
            &format!("{label} NaN input is code 5"),
            query(h, &nan, n).0 == 5,
        );
        // SAFETY: `h` came from a successful ffi_load_surrogate_model and is not used after the first unload below.
        let u1 = unsafe { ffi_unload_surrogate_model(h) };
        check(&format!("{label} unload is code 0 (got {u1})"), u1 == 0);
        check(
            &format!("{label} query after unload is code 2"),
            query(h, &probe, n).0 == 2,
        );
        // SAFETY: the registry checks the address; a stale handle is refused with code 2 and nothing is freed.
        let u2 = unsafe { ffi_unload_surrogate_model(h) };
        check(
            &format!("{label} second unload is code 2 (got {u2})"),
            u2 == 2,
        );
    }

    // Error codes on real files.
    check(
        "missing file is code 1",
        load(&models.join("no_such_model.onnx")).0 == 1,
    );
    let garbage = std::env::temp_dir().join("uav_sim_not_an_onnx_model.onnx");
    std::fs::write(&garbage, b"this is not an onnx model").expect("write temp file");
    let garbage_code = load(&garbage).0;
    let _ = std::fs::remove_file(&garbage);
    // CURRENT behaviour: `load_surrogate` maps an ONNX Runtime parse failure to IoError, so the FFI returns 1 (not 4 as the doc table
    // for "engine failure" suggests). Recorded as an open question in HUMAN-TODO section 12; changing a code needs approval.
    check(
        &format!(
            "a file that is not an ONNX model is refused with code 1 (current behaviour, got {garbage_code})"
        ),
        garbage_code == 1,
    );
    // The synthetic per-build fixture is bound to the REFERENCE BUILD's config, not to the compiled-in demo config: it must be refused
    // by the hash checks (geometry mismatch 3 or config mismatch 8), never loaded.
    let fixture = root.join("offline_pipeline/cfd/sample/synthetic_quad_standard_aero.onnx");
    let (fc, fh) = load(&fixture);
    check(
        &format!(
            "synthetic per-build fixture is refused by the compiled-in hashes (code 3 or 8, got {fc})"
        ),
        (fc == 3 || fc == 8) && fh.is_null(),
    );
    std::process::exit(i32::from(failed));
}
