use sha2::{Digest, Sha256};
use std::env;
use std::fs;
use std::path::Path;

fn main() {
    let crate_dir = env::var("CARGO_MANIFEST_DIR")
        .expect("CARGO_MANIFEST_DIR environment variable is required by build script");

    // Generates the C/C++ header file for FFI boundary compliance.
    cbindgen::Builder::new()
        .with_crate(&crate_dir)
        .with_config(
            cbindgen::Config::from_file("cbindgen.toml").expect("cbindgen.toml must be present"),
        )
        .generate()
        .expect("Failed to generate FFI bindings")
        .write_to_file("drone_ffi.h");

    // Computes GEOMETRY_HASH from the canonical geometry export to enforce surrogate model provenance.
    // See Core Engineering Standards & Safety Classification Framework, Part II, Section 2.2.
    let geometry_path =
        Path::new(&crate_dir).join("../offline_pipeline/geometry/current_geometry.json");

    let hash_hex = if geometry_path.exists() {
        let geometry_data = fs::read(&geometry_path)
            .expect("Failed to read canonical geometry file for hash computation");
        let mut hasher = Sha256::new();
        hasher.update(&geometry_data);
        format!("{:x}", hasher.finalize())
    } else {
        // PLACEHOLDER: fallback zero-hash — offline geometry pipeline not yet fully integrated locally.
        // See Core Engineering Standards & Safety Classification Framework, Part II.
        "0000000000000000000000000000000000000000000000000000000000000000".to_string()
    };

    // Exposes the computed hash as a build-time environment variable.
    println!("cargo:rustc-env=GEOMETRY_HASH={}", hash_hex);

    if geometry_path.exists() {
        println!("cargo:rerun-if-changed={}", geometry_path.display());
    }
}