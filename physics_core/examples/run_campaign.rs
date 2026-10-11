// Tier: F
//! Runs the test campaign on a golden build and writes the result files.
//! Usage: run_campaign <build dir> <conditions.json> <definitions.json> <out dir> [--tests a,b,c] [--threads n] [--git-hash H]
//!        [--aero-model <model.onnx> --aero-sidecar <model.onnx.model.json> --aero-geometry <geometry.json>]
//! With an aero model the drag comes from the per-build CFD surrogate (preliminary CFD, unvalidated; needs an ONNX Runtime that `ort` accepts).
//! The expected hashes are NOT taken from the model's own sidecar alone: the config hash is computed from the build's vehicle_config.json,
//! the geometry hash comes from the CFD sweep's geometry.json, and the binding hash is recomputed from those and the sidecar's data hash.
use physics_core::campaign::{
    CampaignError, CampaignInputs, DragFactory, DragSource, SurrogateDrag, TestId,
    parse_conditions, parse_definitions, render_outputs, run_campaign,
};
use physics_core::controller_sim::load_build;
use physics_core::surrogate::{
    BuildBinding, binding_hash_of, load_surrogate_for_build, sha256_hex,
};
use std::fs;
use std::path::{Path, PathBuf};

fn read(p: &Path) -> String {
    fs::read_to_string(p).unwrap_or_else(|e| panic!("cannot read {}: {e}", p.display()))
}

type AeroFactory = Box<DragFactory>;

/// The drag factory for `--aero-model`, or `None` for the analytic drag. Panics with a message on a bad option (this is a command line tool).
fn aero_factory(args: &[String], build_dir: &Path) -> Option<AeroFactory> {
    let opt = |name: &str| {
        args.iter()
            .position(|a| a == name)
            .and_then(|i| args.get(i + 1))
            .cloned()
    };
    let model = opt("--aero-model")?;
    let sidecar_path = opt("--aero-sidecar").expect("--aero-model needs --aero-sidecar");
    let geometry_path = opt("--aero-geometry")
        .expect("--aero-model needs --aero-geometry (the geometry.json of the CFD sweep)");
    let sidecar: serde_json::Value =
        serde_json::from_str(&read(Path::new(&sidecar_path))).expect("sidecar json");
    let geometry: serde_json::Value =
        serde_json::from_str(&read(Path::new(&geometry_path))).expect("geometry json");
    let text = |v: &serde_json::Value, k: &str| {
        v[k].as_str()
            .unwrap_or_else(|| panic!("missing {k}"))
            .to_string()
    };
    let config_hash = sha256_hex(
        &std::fs::read(build_dir.join("vehicle_config.json")).expect("vehicle_config.json"),
    );
    let geometry_hash = text(&geometry, "stl_sha256");
    let binding_hash = binding_hash_of(
        &config_hash,
        &geometry_hash,
        &text(&sidecar, "training_data_sha256"),
        &text(&sidecar, "trainer_version"),
    );
    let model_sha = text(&sidecar, "model_sha256");
    Some(Box::new(move || {
        let binding = BuildBinding {
            geometry_hash: &geometry_hash,
            config_hash: &config_hash,
            binding_hash: &binding_hash,
            model_sha256: Some(&model_sha),
        };
        let (handle, info) = load_surrogate_for_build(&model, &binding)
            .map_err(|_| CampaignError::Model("aero model load"))?;
        Ok(Box::new(SurrogateDrag::new(handle, info)) as Box<dyn DragSource>)
    }))
}

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let [dir, cond_path, def_path, out] =
        [0, 1, 2, 3].map(|i| args.get(i).cloned().unwrap_or_default());
    if out.is_empty() {
        eprintln!(
            "usage: run_campaign <build dir> <conditions.json> <definitions.json> <out dir> [--tests a,b] [--threads n] [--git-hash H]"
        );
        std::process::exit(2);
    }
    let opt = |name: &str| {
        args.iter()
            .position(|a| a == name)
            .and_then(|i| args.get(i + 1))
            .cloned()
    };
    let threads: usize = opt("--threads").and_then(|t| t.parse().ok()).unwrap_or(1);
    let git_hash = opt("--git-hash").unwrap_or_else(|| "unknown".to_string());
    let tests: Vec<TestId> = opt("--tests").map_or_else(
        || TestId::ALL.to_vec(),
        |t| {
            t.split(',')
                .map(|n| TestId::from_name(n).unwrap_or_else(|| panic!("unknown test {n}")))
                .collect()
        },
    );

    let dir = PathBuf::from(dir);
    let build = load_build(&dir).expect("load build");
    let conditions = parse_conditions(&read(Path::new(&cond_path))).expect("conditions");
    let definitions = parse_definitions(&read(Path::new(&def_path))).expect("definitions");
    let sidecar: serde_json::Value =
        serde_json::from_str(&read(&dir.join("vehicle_build.json"))).expect("sidecar");
    let parts: Vec<(String, String)> = sidecar["parts"]
        .as_array()
        .map(|a| {
            a.iter()
                .filter_map(|p| {
                    Some((
                        p["role"].as_str()?.to_string(),
                        p["part_id"].as_str()?.to_string(),
                    ))
                })
                .collect()
        })
        .unwrap_or_default();
    let db_version = sidecar["database"]["db_version"]
        .as_str()
        .unwrap_or("unknown")
        .to_string();
    let build_name = dir
        .file_name()
        .map_or_else(|| "build".to_string(), |n| n.to_string_lossy().into_owned());
    let aero = aero_factory(&args, &dir);
    let inputs = CampaignInputs {
        build: &build,
        build_name: &build_name,
        parts,
        db_version: &db_version,
        git_hash: &git_hash,
        conditions: &conditions,
        definitions: &definitions,
        tests,
        drag: aero.as_deref(),
    };
    let results = run_campaign(&inputs, threads).expect("campaign");
    let files = render_outputs(&inputs, &results).expect("render");
    for (rel, text) in &files {
        let path = Path::new(&out).join(rel);
        if let Some(parent) = path.parent() {
            fs::create_dir_all(parent).expect("create dir");
        }
        fs::write(&path, text).expect("write file");
    }
    print!(
        "{}",
        files
            .iter()
            .find(|(p, _)| p == "summary.csv")
            .map_or("", |(_, t)| t.as_str())
    );
}
