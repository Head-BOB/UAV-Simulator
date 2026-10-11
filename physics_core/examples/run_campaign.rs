// Tier: F
//! Runs the test campaign on a golden build and writes the result files.
//! Usage: run_campaign <build dir> <conditions.json> <definitions.json> <out dir> [--tests a,b,c] [--threads n] [--git-hash H]
//!        [--aero-dir <cfd dir> | --aero-model <model.onnx> --aero-sidecar <model.onnx.model.json> --aero-geometry <geometry.json>]
//!        [--analytic-fallback]
//! With an aero model the drag comes from the per-build CFD surrogate (preliminary CFD, unvalidated; needs ONNX Runtime 1.27.x for
//! `ort 2.0.0-rc.13`: `ORT_DYLIB_PATH`, or `onnxruntime.dll` / `libonnxruntime.so` next to this program).
//! `--aero-dir` picks, from the `*.onnx.model.json` files of a CFD folder, the ONE model whose `config_hash` is the hash of this build's
//! `vehicle_config.json` (geometry.json must sit in the same folder). No match: the run uses analytic drag and says so in the result files.
//! The expected hashes are NOT taken from the model's own sidecar alone: the config hash is computed from the build's vehicle_config.json,
//! the geometry hash comes from the CFD sweep's geometry.json, and the binding hash is recomputed from those and the sidecar's data hash.
//! A stub model other than the committed SYNTHETIC fixture is refused. A model that matches the build but cannot be used STOPS the run:
//! reason in `<out>/aero_model_status.json`, exit code 3, no results. Only `--analytic-fallback` (never set automatically) continues on
//! analytic drag, and then every result file says "CFD model failed to load: <reason>; results use analytic drag".
use physics_core::campaign::{
    CampaignError, CampaignInputs, CfdNote, DragFactory, DragSource, SurrogateDrag, TestId,
    parse_conditions, parse_definitions, render_outputs, run_campaign,
};
use physics_core::controller_sim::load_build;
use physics_core::surrogate::{
    BuildBinding, BuildLoadError, ModelInfo, SurrogateLoadError, binding_hash_of,
    load_surrogate_for_build, sha256_hex, surrogate_status_text,
};
use std::fs;
use std::path::{Path, PathBuf};

/// Exit code of a CFD model that matched the build but could not be used (and `--analytic-fallback` was not given).
const EXIT_AERO_MODEL: i32 = 3;

fn read(p: &Path) -> String {
    fs::read_to_string(p).unwrap_or_else(|e| panic!("cannot read {}: {e}", p.display()))
}

type AeroFactory = Box<DragFactory>;

/// What the aero options produced: the drag factory (`None` = analytic) and the note for the result files.
struct AeroSetup {
    factory: Option<AeroFactory>,
    note: Option<CfdNote>,
}

/// The three files that name one per-build model.
struct ModelFiles {
    model: PathBuf,
    sidecar: PathBuf,
    geometry: PathBuf,
}

fn text(v: &serde_json::Value, k: &str) -> Result<String, String> {
    v[k].as_str()
        .map(str::to_string)
        .ok_or_else(|| format!("missing \"{k}\""))
}

fn read_json(p: &Path) -> Result<serde_json::Value, String> {
    let t = fs::read_to_string(p).map_err(|e| format!("cannot read {}: {e}", p.display()))?;
    serde_json::from_str(&t).map_err(|e| format!("{} is not valid JSON: {e}", p.display()))
}

/// Plain-language reason for a refused model (shown to the user as is).
fn describe(e: &BuildLoadError) -> String {
    match e {
        BuildLoadError::Load(l) => match l {
            SurrogateLoadError::IoError => {
                "the model file cannot be read or is not a valid ONNX model".to_string()
            }
            SurrogateLoadError::MissingProvenance => {
                "the model carries no provenance metadata (kind, geometry_hash, config_hash)"
                    .to_string()
            }
            SurrogateLoadError::GeometryMismatch { .. } => {
                "the model was made for a different geometry than this geometry.json".to_string()
            }
            SurrogateLoadError::ConfigMismatch { .. } => {
                "the model was made for a different vehicle_config.json than this build's"
                    .to_string()
            }
            SurrogateLoadError::StubModel => "the model is a stub and this helper was built without allow_stub_models (the demo helper is built with it)".to_string(),
            SurrogateLoadError::EngineError(m) => format!(
                "ONNX Runtime could not be used (ort 2.0.0-rc.13 needs ONNX Runtime 1.27.x; set ORT_DYLIB_PATH or put the library next to this program): {m}"
            ),
            SurrogateLoadError::BadModelShape => {
                "the model's inputs or outputs are not the supported shapes".to_string()
            }
            other => format!("{other:?}"),
        },
        BuildLoadError::StubNotFixture { found } => format!(
            "this is a stub model that is not the committed synthetic fixture (sha256 {found}); stubs are refused"
        ),
        BuildLoadError::ModelFileMismatch { expected, found } => format!(
            "the model file changed after training (sha256 {found}, sidecar says {expected})"
        ),
        BuildLoadError::BindingMismatch { expected, found } => format!(
            "the model is bound to another build, geometry or training data (binding {found}, expected {expected})"
        ),
        other => format!("{other:?}"),
    }
}

/// Finds the one model of `dir` that is bound to `config_hash`.
/// `Ok(None)` = the folder holds no model for this build (with the number of models that belong to other builds).
fn pick_model(dir: &Path, config_hash: &str) -> Result<Result<ModelFiles, usize>, String> {
    let mut sidecars: Vec<PathBuf> = fs::read_dir(dir)
        .map_err(|e| format!("cannot read the CFD folder {}: {e}", dir.display()))?
        .filter_map(Result::ok)
        .map(|d| d.path())
        .filter(|p| {
            p.file_name()
                .is_some_and(|n| n.to_string_lossy().ends_with(".onnx.model.json"))
        })
        .collect();
    sidecars.sort();
    let mut found = Vec::new();
    let mut others = 0;
    for sc in sidecars {
        let doc = read_json(&sc)?;
        if text(&doc, "config_hash").is_ok_and(|h| h == config_hash) {
            let file = text(&doc, "model_file")?;
            found.push(ModelFiles {
                model: dir.join(file),
                sidecar: sc,
                geometry: dir.join("geometry.json"),
            });
        } else {
            others += 1;
        }
    }
    match found.len() {
        0 => Ok(Err(others)),
        1 => Ok(Ok(found.remove(0))),
        n => Err(format!(
            "{n} models in {} are bound to this build; keep one",
            dir.display()
        )),
    }
}

/// Loads the model once (fail fast) and builds the factory that reloads it for every simulation.
fn load_model(files: &ModelFiles, build_dir: &Path) -> Result<(AeroFactory, ModelInfo), String> {
    let sidecar = read_json(&files.sidecar)?;
    let geometry = read_json(&files.geometry)?;
    let config_hash = sha256_hex(
        &fs::read(build_dir.join("vehicle_config.json"))
            .map_err(|e| format!("cannot read the build's vehicle_config.json: {e}"))?,
    );
    let geometry_hash = text(&geometry, "stl_sha256")?;
    let binding_hash = binding_hash_of(
        &config_hash,
        &geometry_hash,
        &text(&sidecar, "training_data_sha256")?,
        &text(&sidecar, "trainer_version")?,
    );
    let model_sha = text(&sidecar, "model_sha256")?;
    let model = files.model.to_string_lossy().into_owned();
    let expected = BuildBinding {
        geometry_hash: &geometry_hash,
        config_hash: &config_hash,
        binding_hash: &binding_hash,
        model_sha256: Some(&model_sha),
    };
    // loaded once here so a bad model stops the run before any test starts; the handle is dropped at once
    let (_handle, info) = load_surrogate_for_build(&model, &expected).map_err(|e| describe(&e))?;
    let factory: AeroFactory = Box::new(move || {
        let binding = BuildBinding {
            geometry_hash: &geometry_hash,
            config_hash: &config_hash,
            binding_hash: &binding_hash,
            model_sha256: Some(&model_sha),
        };
        let (handle, info) = load_surrogate_for_build(&model, &binding)
            .map_err(|_| CampaignError::Model("aero model load"))?;
        Ok(Box::new(SurrogateDrag::new(handle, info)) as Box<dyn DragSource>)
    });
    Ok((factory, info))
}

fn note_for(info: &ModelInfo) -> CfdNote {
    let (status, level) = surrogate_status_text(&info.kind, info.provenance.as_deref());
    CfdNote {
        status: status.to_string(),
        level,
        kind: Some(info.kind.clone()),
        model_sha256: Some(info.model_sha256.clone()),
        provenance: info.provenance.clone(),
        holdout_rms_n: info.holdout_rms_n.clone(),
        cfd_sanity_status: info.cfd_sanity_status.clone(),
        load_error: None,
    }
}

/// Writes `<out>/aero_model_status.json`.
fn write_status(out: &Path, status: &str, reason: &str, model: &str) {
    use physics_core::textout::json_str;
    let body = format!(
        "{{\n  \"schema_version\": 1,\n  \"status\": {},\n  \"reason\": {},\n  \"model\": {}\n}}\n",
        json_str(status),
        json_str(reason),
        json_str(model)
    );
    if let Err(e) =
        fs::create_dir_all(out).and_then(|()| fs::write(out.join("aero_model_status.json"), body))
    {
        eprintln!("cannot write aero_model_status.json: {e}");
    }
}

/// A model that matched the build could not be used: stop (exit 3), or with `fallback` carry on with analytic drag and say so.
fn refuse(reason: &str, model: &str, out: &Path, fallback: bool) -> AeroSetup {
    eprintln!("CFD MODEL NOT USED: {reason}");
    if !fallback {
        write_status(out, "load_failed", reason, model);
        eprintln!(
            "No results were produced (nothing falls back silently). Re-run with --analytic-fallback to accept analytic drag."
        );
        std::process::exit(EXIT_AERO_MODEL);
    }
    write_status(out, "load_failed_fallback", reason, model);
    eprintln!("*** --analytic-fallback: this run uses the legacy analytic drag ***");
    AeroSetup {
        factory: None,
        note: Some(CfdNote {
            status: format!("CFD model failed to load: {reason}; results use analytic drag"),
            level: "warn",
            kind: None,
            model_sha256: None,
            provenance: None,
            holdout_rms_n: None,
            cfd_sanity_status: None,
            load_error: Some(reason.to_string()),
        }),
    }
}

fn use_files(files: &ModelFiles, build_dir: &Path, out: &Path, fallback: bool) -> AeroSetup {
    let model = files.model.to_string_lossy().into_owned();
    match load_model(files, build_dir) {
        Ok((factory, info)) => {
            let note = note_for(&info);
            eprintln!("{} (model sha256 {})", note.status, info.model_sha256);
            if info.kind != "trained" {
                eprintln!(
                    "*** SYNTHETIC CFD SURROGATE: a formula, NOT a CFD result and NOT validated ***"
                );
            }
            AeroSetup {
                factory: Some(factory),
                note: Some(note),
            }
        }
        Err(reason) => refuse(&reason, &model, out, fallback),
    }
}

/// Interprets the aero options. Exits the process (code 3) for a model that matched the build but cannot be used.
fn aero_setup(args: &[String], build_dir: &Path, out: &Path) -> AeroSetup {
    let opt = |name: &str| {
        args.iter()
            .position(|a| a == name)
            .and_then(|i| args.get(i + 1))
            .cloned()
    };
    let fallback = args.iter().any(|a| a == "--analytic-fallback");
    if let Some(dir) = opt("--aero-dir") {
        let config_hash = match fs::read(build_dir.join("vehicle_config.json")) {
            Ok(b) => sha256_hex(&b),
            Err(e) => panic!("cannot read the build's vehicle_config.json: {e}"),
        };
        return match pick_model(Path::new(&dir), &config_hash) {
            Err(reason) => refuse(&reason, &dir, out, fallback),
            Ok(Err(others)) => AeroSetup {
                factory: None,
                note: Some(CfdNote {
                    status: format!(
                        "No CFD model for this build ({others} model(s) in the CFD folder are bound to other builds); analytic drag"
                    ),
                    level: "warn",
                    kind: None,
                    model_sha256: None,
                    provenance: None,
                    holdout_rms_n: None,
                    cfd_sanity_status: None,
                    load_error: None,
                }),
            },
            Ok(Ok(files)) => use_files(&files, build_dir, out, fallback),
        };
    }
    let Some(model) = opt("--aero-model") else {
        return AeroSetup {
            factory: None,
            note: None,
        };
    };
    let sidecar = opt("--aero-sidecar").expect("--aero-model needs --aero-sidecar");
    let geometry = opt("--aero-geometry")
        .expect("--aero-model needs --aero-geometry (the geometry.json of the CFD sweep)");
    let files = ModelFiles {
        model: PathBuf::from(model),
        sidecar: PathBuf::from(sidecar),
        geometry: PathBuf::from(geometry),
    };
    use_files(&files, build_dir, out, fallback)
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
    let aero = aero_setup(&args, &dir, Path::new(&out));
    let inputs = CampaignInputs {
        build: &build,
        build_name: &build_name,
        parts,
        db_version: &db_version,
        git_hash: &git_hash,
        conditions: &conditions,
        definitions: &definitions,
        tests,
        drag: aero.factory.as_deref(),
        cfd_surrogate: aero.note.as_ref(),
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
