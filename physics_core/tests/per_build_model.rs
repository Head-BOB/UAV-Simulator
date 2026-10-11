// Tier: F
//! Per-build surrogate loading (P5): the hash and binding parts that need no ONNX Runtime.
//! NOT covered here: creating a session from the model. `ort 2.0.0-rc.13` rejects the pinned ONNX Runtime 1.19.0 (it asks for
//! version 1.27 or newer), so no test in this repository can load a model today. `examples/check_per_build_model.rs` does that check by
//! hand once a compatible runtime is available (HUMAN-TODO). The fixture model is SYNTHETIC (formula, not CFD) and has kind `stub`.
#![cfg(feature = "onnx")]

mod tests {
    use physics_core::surrogate::{
        BuildBinding, BuildLoadError, SurrogateLoadError, binding_hash_of,
        load_surrogate_for_build, sha256_hex,
    };
    use std::path::PathBuf;

    fn root() -> PathBuf {
        PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("..")
    }

    fn sample(name: &str) -> String {
        root()
            .join("offline_pipeline/cfd/sample")
            .join(name)
            .to_string_lossy()
            .into_owned()
    }

    struct Fixture {
        model_path: String,
        geometry: String,
        config: String,
        binding: String,
        model_sha: String,
    }

    fn fixture() -> Fixture {
        let side: serde_json::Value = serde_json::from_str(
            &std::fs::read_to_string(sample("synthetic_quad_standard_aero.onnx.model.json"))
                .unwrap(),
        )
        .unwrap();
        let get = |k: &str| side[k].as_str().unwrap().to_string();
        Fixture {
            model_path: sample("synthetic_quad_standard_aero.onnx"),
            geometry: get("geometry_hash"),
            config: get("config_hash"),
            binding: get("binding_hash"),
            model_sha: get("model_sha256"),
        }
    }

    fn binding<'a>(f: &'a Fixture, model_sha: Option<&'a str>) -> BuildBinding<'a> {
        BuildBinding {
            geometry_hash: &f.geometry,
            config_hash: &f.config,
            binding_hash: &f.binding,
            model_sha256: model_sha,
        }
    }

    #[test]
    fn test_per_build_sha256_and_binding_hash_vectors() {
        // NIST vectors: SHA-256("abc") and SHA-256("")
        assert_eq!(
            sha256_hex(b"abc"),
            "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad"
        );
        assert_eq!(
            sha256_hex(b""),
            "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855"
        );
        // independent (Python hashlib) value of sha256("a|b|c|1")
        assert_eq!(
            binding_hash_of("a", "b", "c", "1"),
            "4128761d4504d08c6bf2197fd092e345ffeabd124bfea6e96c0b823b992c5206"
        );
        assert_ne!(
            binding_hash_of("a", "b", "c", "2"),
            binding_hash_of("a", "b", "c", "1")
        );
        assert_ne!(
            binding_hash_of("b", "a", "c", "1"),
            binding_hash_of("a", "b", "c", "1")
        );
    }

    #[test]
    fn test_per_build_fixture_hashes_agree_with_independent_rust_hashing() {
        let f = fixture();
        // the model file hash, the build's vehicle_config.json hash and the binding recomputed here equal what the Python trainer wrote
        assert_eq!(
            sha256_hex(&std::fs::read(&f.model_path).unwrap()),
            f.model_sha
        );
        let cfg =
            std::fs::read(root().join("tools/parts/builds/reference_quad/vehicle_config.json"))
                .unwrap();
        assert_eq!(sha256_hex(&cfg), f.config);
        let csv = std::fs::read(sample("synthetic_quad_standard_cfd.csv")).unwrap();
        assert_eq!(
            binding_hash_of(&f.config, &f.geometry, &sha256_hex(&csv), "1"),
            f.binding
        );
    }

    #[test]
    fn test_per_build_missing_file_is_an_io_error_without_the_runtime() {
        let f = fixture();
        let r = load_surrogate_for_build("/no/such/model.onnx", &binding(&f, None));
        assert_eq!(
            r.err(),
            Some(BuildLoadError::Load(SurrogateLoadError::IoError))
        );
    }

    #[test]
    fn test_per_build_model_file_hash_mismatch_is_refused_before_the_runtime() {
        let f = fixture();
        let wrong = "0".repeat(64);
        let r = load_surrogate_for_build(&f.model_path, &binding(&f, Some(&wrong)));
        assert_eq!(
            r.err(),
            Some(BuildLoadError::ModelFileMismatch {
                expected: wrong,
                found: f.model_sha.clone()
            })
        );
    }
}
