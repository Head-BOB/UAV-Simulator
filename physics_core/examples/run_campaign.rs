// Tier: F
//! Runs the test campaign on a golden build and writes the result files.
//! Usage: run_campaign <build dir> <conditions.json> <definitions.json> <out dir> [--tests a,b,c] [--threads n] [--git-hash H]
//!        [--aero-dir <cfd dir> | --aero-model <model.onnx> --aero-sidecar <model.onnx.model.json> --aero-geometry <geometry.json>]
//!        [--analytic-fallback] [--progress]
//! `--progress` prints `UAVPROGRESS {json}` lines (flushed; at least one per second and on every state change; percent from the
//! simulated time done over an upper bound of the planned simulated time, never from a timer), writes each finished test's files at
//! once (the final files are rewritten identically), and writes `timing.json`. Without it the outputs are byte-identical to before.
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
    parse_conditions, parse_definitions, planned_sim_s, render_outputs, render_partial,
    run_campaign, run_campaign_with,
};
use physics_core::controller_sim::load_build;
use physics_core::progress;
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

/// Writes a file through a temporary name so a reader never sees half of it.
fn write_atomic(path: &Path, text: &str) -> std::io::Result<()> {
    if let Some(parent) = path.parent() {
        fs::create_dir_all(parent)?;
    }
    let tmp = path.with_extension("tmp");
    fs::write(&tmp, text)?;
    fs::rename(&tmp, path)
}

fn json_f(v: f64) -> String {
    if v.is_finite() {
        format!("{v:.3}")
    } else {
        "null".to_string()
    }
}

/// One `UAVPROGRESS` line: the lowest running test is the current one (else the last finished, else the first).
fn progress_line(
    stage: &str,
    event: &str,
    names: &[&str],
    totals: &[f64],
    step_s: f64,
    elapsed_s: f64,
    overall_span: f64,
) -> String {
    let n = names.len();
    let mut cur = None;
    let mut last_done = None;
    let (mut done_sum, mut total_sum, mut sim_done, mut finished) = (0.0, 0.0, 0.0, 0usize);
    let mut tests = Vec::new();
    for i in 0..n {
        let total = totals.get(i).copied().unwrap_or(0.0);
        let st = progress::state(i);
        let sim = progress::steps(i) as f64 * step_s;
        let (done, pct, label) = match st {
            d if d == progress::state_done() || d == progress::state_failed() => {
                finished += 1;
                last_done = Some(i);
                (
                    total,
                    100.0,
                    if st == progress::state_done() {
                        "done"
                    } else {
                        "failed"
                    },
                )
            }
            r if r == progress::state_running() => {
                if cur.is_none() {
                    cur = Some(i);
                }
                let d = sim.min(total * 0.99);
                (
                    d,
                    if total > 0.0 { 100.0 * d / total } else { 0.0 },
                    "running",
                )
            }
            _ => (0.0, 0.0, "waiting"),
        };
        done_sum += done;
        total_sum += total;
        sim_done += sim;
        tests.push(format!(
            "{{\"id\": \"{}\", \"state\": \"{label}\", \"percent\": {}, \"sim_s\": {}}}",
            names.get(i).copied().unwrap_or(""),
            json_f(pct),
            json_f(sim)
        ));
    }
    let idx = cur.or(last_done).unwrap_or(0);
    let total_i = totals.get(idx).copied().unwrap_or(0.0);
    let pct_test = match progress::state(idx) {
        d if d == progress::state_done() || d == progress::state_failed() => 100.0,
        r if r == progress::state_running() && total_i > 0.0 => {
            (100.0 * progress::steps(idx) as f64 * step_s / total_i).min(99.0)
        }
        _ => 0.0,
    };
    let frac = if total_sum > 0.0 {
        done_sum / total_sum
    } else {
        0.0
    };
    let overall = overall_span * 100.0 * frac;
    // an estimate, and only once a test has finished (before that the budget of the longest test says nothing)
    let eta = if finished > 0 && frac > 0.0 && frac < 1.0 {
        json_f(elapsed_s * (1.0 - frac) / frac)
    } else {
        "null".to_string()
    };
    format!(
        "UAVPROGRESS {{\"stage\": \"{stage}\", \"event\": \"{event}\", \"test_id\": \"{}\", \"test_index\": {idx}, \"test_count\": {n}, \"sim_s_done\": {}, \"sim_s_total\": {}, \"percent_overall\": {}, \"percent_test\": {}, \"elapsed_s\": {}, \"eta_s\": {eta}, \"tests\": [{}]}}",
        names.get(idx).copied().unwrap_or(""),
        json_f(sim_done),
        json_f(total_sum),
        json_f(overall),
        json_f(pct_test),
        json_f(elapsed_s),
        tests.join(", ")
    )
}

fn emit(line: &str) {
    use std::io::Write;
    let mut out = std::io::stdout().lock();
    let _ = writeln!(out, "{line}");
    let _ = out.flush();
}

fn timing_json(names: &[&str], totals: &[f64], step_s: f64, threads: usize, wall_s: f64) -> String {
    let rows: Vec<String> = names
        .iter()
        .enumerate()
        .map(|(i, n)| {
            let (a, b) = progress::wall_us(i);
            format!(
                "    {{\"test\": \"{n}\", \"state\": {}, \"wall_start_s\": {}, \"wall_s\": {}, \"sim_s\": {}, \"sim_s_budget\": {}}}",
                progress::state(i),
                json_f(a as f64 / 1e6),
                json_f(b.saturating_sub(a) as f64 / 1e6),
                json_f(progress::steps(i) as f64 * step_s),
                json_f(totals.get(i).copied().unwrap_or(0.0))
            )
        })
        .collect();
    format!(
        "{{\n  \"threads\": {threads},\n  \"wall_s\": {},\n  \"note\": \"wall clock of this machine; not part of any result\",\n  \"tests\": [\n{}\n  ]\n}}\n",
        json_f(wall_s),
        rows.join(",\n")
    )
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
    let show_progress = args.iter().any(|a| a == "--progress");
    let results = if show_progress {
        run_with_progress(&inputs, threads, &out)
    } else {
        run_campaign(&inputs, threads).expect("campaign")
    };
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

/// The `--progress` run: a reporter thread prints a line at least once a second; each finished test's files are written at once.
/// Campaign share of the overall percentage is 90 %: the report that follows takes the last 10 %.
fn run_with_progress(
    inputs: &CampaignInputs<'_>,
    threads: usize,
    out: &str,
) -> Vec<physics_core::campaign::TestResult> {
    use std::sync::atomic::{AtomicBool, Ordering};
    const SHARE: f64 = 0.9;
    const STEP_S: f64 = 0.002; // ADR-001 fixed step
    let names: Vec<&str> = inputs.tests.iter().map(|t| t.name()).collect();
    let totals: Vec<f64> = inputs
        .tests
        .iter()
        .map(|t| planned_sim_s(inputs.definitions, *t))
        .collect();
    progress::reset();
    let t0 = std::time::Instant::now();
    emit(&progress_line(
        "preparing",
        "start",
        &names,
        &totals,
        STEP_S,
        0.0,
        SHARE,
    ));
    let stop = AtomicBool::new(false);
    let results = std::thread::scope(|scope| {
        scope.spawn(|| {
            let mut last_states: Vec<u8> = vec![255; names.len()];
            let mut last_emit = std::time::Instant::now();
            while !stop.load(Ordering::Relaxed) {
                std::thread::sleep(std::time::Duration::from_millis(100));
                let states: Vec<u8> = (0..names.len()).map(progress::state).collect();
                if states != last_states || last_emit.elapsed().as_millis() >= 1000 {
                    emit(&progress_line(
                        "test",
                        "tick",
                        &names,
                        &totals,
                        STEP_S,
                        t0.elapsed().as_secs_f64(),
                        SHARE,
                    ));
                    last_states = states;
                    last_emit = std::time::Instant::now();
                }
            }
        });
        let r = run_campaign_with(inputs, threads, &|i, so_far| {
            // the recording of test `i` is on disk before the line that announces it
            if let Ok(files) = render_partial(inputs, so_far) {
                for (rel, text) in &files {
                    if let Err(e) = write_atomic(&Path::new(out).join(rel), text) {
                        eprintln!("cannot write {rel}: {e}");
                    }
                }
            }
            let ev = format!("test_done:{}", names.get(i).copied().unwrap_or(""));
            emit(&progress_line(
                "test",
                &ev,
                &names,
                &totals,
                STEP_S,
                t0.elapsed().as_secs_f64(),
                SHARE,
            ));
        });
        stop.store(true, Ordering::Relaxed);
        r
    });
    let ok = results.is_ok();
    emit(&progress_line(
        if ok { "done" } else { "failed" },
        if ok {
            "campaign_done"
        } else {
            "campaign_failed"
        },
        &names,
        &totals,
        STEP_S,
        t0.elapsed().as_secs_f64(),
        SHARE,
    ));
    if let Err(e) = write_atomic(
        &Path::new(out).join("timing.json"),
        &timing_json(&names, &totals, STEP_S, threads, t0.elapsed().as_secs_f64()),
    ) {
        eprintln!("cannot write timing.json: {e}");
    }
    results.expect("campaign")
}
