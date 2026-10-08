extern crate cbindgen;

use sha2::{Digest, Sha256};
use std::env;
use std::fs;
use std::path::Path;

fn main() {
    let crate_dir = env::var("CARGO_MANIFEST_DIR").unwrap();

    let out_path = Path::new(&crate_dir)
        .parent()
        .unwrap()
        .join("UE5Project/Plugins/DronePhysicsBridge/Source/DronePhysicsBridge/Public/Generated/drone_ffi.h");

    cbindgen::Builder::new()
        .with_crate(&crate_dir)
        .with_config(cbindgen::Config::from_file("cbindgen.toml").unwrap())
        .generate()
        .expect("Unable to generate bindings")
        .write_to_file(out_path);

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

    let mut geo_hasher = Sha256::new();
    geo_hasher.update(&geometry_data);
    let geo_hash_result = geo_hasher.finalize();
    let geo_hash_hex = hex::encode(geo_hash_result);

    let config_path = Path::new(&crate_dir)
        .parent()
        .unwrap()
        .join("offline_pipeline/geometry/vehicle_config.json");

    let config_data = fs::read(&config_path).unwrap_or_else(|_| {
        panic!("CRITICAL: Vehicle config file missing at {:?}", config_path);
    });

    let mut config_hasher = Sha256::new();
    config_hasher.update(&config_data);
    let config_hash_result = config_hasher.finalize();
    let config_hash_hex = hex::encode(config_hash_result);

    println!("cargo:rustc-env=GEOMETRY_HASH={}", geo_hash_hex);
    println!("cargo:rustc-env=CONFIG_HASH={}", config_hash_hex);
    println!("cargo:rerun-if-changed=../offline_pipeline/geometry/current_geometry.json");
    println!("cargo:rerun-if-changed=../offline_pipeline/geometry/vehicle_config.json");
    println!("cargo:rerun-if-changed=build.rs");
}
