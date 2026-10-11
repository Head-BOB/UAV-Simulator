// Tier: F
//! Manual end-to-end check of the per-build surrogate loader against the SYNTHETIC fixture model (not a test: it needs an ONNX Runtime
//! that `ort 2.0.0-rc.13` accepts, i.e. version >= 1.27.x, which this repository does not pin yet).
//! Usage: ORT_DYLIB_PATH=/path/to/libonnxruntime.so cargo run --features allow_stub_models --example check_per_build_model
//! Prints PASS / FAIL per check and exits 1 on any failure.
use physics_core::ffi_create_default_drone_state;
use physics_core::surrogate::aero_surrogate::query_aero;
use physics_core::surrogate::{
    BuildBinding, BuildLoadError, SurrogateLoadError, load_surrogate_for_build,
};
use std::path::PathBuf;

fn main() {
    let root = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("..");
    let sample = |n: &str| root.join("offline_pipeline/cfd/sample").join(n);
    let side: serde_json::Value = serde_json::from_str(
        &std::fs::read_to_string(sample("synthetic_quad_standard_aero.onnx.model.json"))
            .expect("sidecar"),
    )
    .expect("json");
    let get = |k: &str| side[k].as_str().expect("string").to_string();
    let (geometry, config, binding, model_sha) = (
        get("geometry_hash"),
        get("config_hash"),
        get("binding_hash"),
        get("model_sha256"),
    );
    let csv = std::fs::read_to_string(sample("synthetic_quad_standard_cfd.csv")).expect("csv");
    let cols: Vec<&str> = csv.lines().nth(1).expect("row").split(',').collect();
    let ref_area: f64 = cols[cols.len() - 2].parse().expect("ref area");
    let path = sample("synthetic_quad_standard_aero.onnx")
        .to_string_lossy()
        .into_owned();
    let mut failed = false;
    let mut check = |name: &str, ok: bool| {
        println!("{} {name}", if ok { "PASS" } else { "FAIL" });
        failed |= !ok;
    };
    let b = BuildBinding {
        geometry_hash: &geometry,
        config_hash: &config,
        binding_hash: &binding,
        model_sha256: Some(&model_sha),
    };
    match load_surrogate_for_build(&path, &b) {
        Ok((mut handle, info)) => {
            check("loads with the right hashes", true);
            check(
                "model sha256 and binding reported",
                info.model_sha256 == model_sha
                    && info.binding_hash == binding
                    && info.kind == "stub",
            );
            let mut state = ffi_create_default_drone_state();
            state.velocity = [10.0, 0.0, 0.0];
            let q = query_aero(&mut handle, &state).expect("query");
            check(
                "F_x at V = 10, alpha = beta = 0 is A x 100 x (-0.05)",
                (f64::from(q.predicted_values[0]) + 5.0 * ref_area).abs()
                    < 1e-5 * ref_area.max(1e-3),
            );
            state.velocity = [
                10.0 * 0.1_f64.cos() as f32,
                0.0,
                10.0 * 0.1_f64.sin() as f32,
            ];
            let q2 = query_aero(&mut handle, &state).expect("query");
            check(
                "F_x, F_z at alpha = 0.1 rad",
                (f64::from(q2.predicted_values[0]) + 5.1 * ref_area).abs()
                    < 1e-4 * ref_area.max(1e-3)
                    && (f64::from(q2.predicted_values[2]) + 15.2 * ref_area).abs()
                        < 1e-4 * ref_area.max(1e-3),
            );
            check(
                "a stub model is never inside a validated envelope",
                q.in_validated_envelope == 0 && q2.in_validated_envelope == 0,
            );
        }
        Err(e) => check(&format!("loads with the right hashes ({e:?})"), false),
    }
    let bad = "f".repeat(64);
    let wrong_geometry = load_surrogate_for_build(
        &path,
        &BuildBinding {
            geometry_hash: &bad,
            ..b
        },
    )
    .err();
    check(
        "wrong geometry hash is refused",
        matches!(
            wrong_geometry,
            Some(BuildLoadError::Load(
                SurrogateLoadError::GeometryMismatch { .. }
            ))
        ),
    );
    let wrong_config = load_surrogate_for_build(
        &path,
        &BuildBinding {
            config_hash: &bad,
            ..b
        },
    )
    .err();
    check(
        "wrong config hash is refused",
        matches!(
            wrong_config,
            Some(BuildLoadError::Load(
                SurrogateLoadError::ConfigMismatch { .. }
            ))
        ),
    );
    let wrong_binding = load_surrogate_for_build(
        &path,
        &BuildBinding {
            binding_hash: &bad,
            ..b
        },
    )
    .err();
    check(
        "wrong binding hash is refused",
        matches!(wrong_binding, Some(BuildLoadError::BindingMismatch { .. })),
    );
    std::process::exit(i32::from(failed));
}
