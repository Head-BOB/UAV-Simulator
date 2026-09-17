extern crate cbindgen;
use sha2::{Digest, Sha256};
use std::env;
use std::fs;
use std::path::Path;

fn main() {
    let crate_dir = env::var("CARGO_MANIFEST_DIR").unwrap();

    cbindgen::Builder::new()
        .with_crate(&crate_dir)
        .with_config(cbindgen::Config::from_file("cbindgen.toml").unwrap())
        .generate()
        .expect("Unable to generate bindings")
        .write_to_file("drone_ffi.h");

    let geometry_path = Path::new(&crate_dir)
        .parent()
        .unwrap()
        .join("offline_pipeline/geometry/current_geometry.json");

    let geometry_data = fs::read(&geometry_path).unwrap_or_else(|_| {
        panic!(
            "CRITICAL: Canonical geometry file missing at {:?}",
            geometry_path
        );
    });

    let mut hasher = Sha256::new();
    hasher.update(&geometry_data);
    let hash_result = hasher.finalize();
    let hash_hex = hex::encode(hash_result);

    println!("cargo:rustc-env=GEOMETRY_HASH={}", hash_hex);
    println!("cargo:rerun-if-changed=current_geometry.json");
    println!("cargo:rerun-if-changed=build.rs");
}
