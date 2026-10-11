// Tier: F
//! Writes `validation_report.json` and `validation_report.csv` for a golden build directory.
//! Usage: cargo run --release --example validate_models -- <build dir> <out dir>
use physics_core::powertrain::load_powertrain_config;
use physics_core::validation::{
    Bands, RefFile, ReportInputs, build_report, parse_battery_record, parse_curve_csv,
    parse_motor_table, parse_prop_data, to_csv, to_json,
};
use std::fs;
use std::path::{Path, PathBuf};

fn read(p: &Path) -> String {
    fs::read_to_string(p).unwrap_or_else(|e| panic!("cannot read {}: {e}", p.display()))
}

fn part_id(sidecar: &serde_json::Value, role: &str) -> String {
    sidecar["parts"]
        .as_array()
        .and_then(|a| a.iter().find(|p| p["role"] == role))
        .and_then(|p| p["part_id"].as_str())
        .unwrap_or_else(|| panic!("no {role} in the sidecar"))
        .to_string()
}

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let (Some(dir), Some(out)) = (args.get(1), args.get(2)) else {
        eprintln!("usage: validate_models <build dir> <out dir>");
        std::process::exit(2);
    };
    let (dir, out) = (PathBuf::from(dir), PathBuf::from(out));
    let root = Path::new(env!("CARGO_MANIFEST_DIR")).join("..");
    let sidecar: serde_json::Value =
        serde_json::from_str(&read(&dir.join("vehicle_build.json"))).expect("sidecar json");
    let cfg = load_powertrain_config(&dir.join("powertrain_config.json").to_string_lossy())
        .expect("powertrain config");
    let motor_id = part_id(&sidecar, "motor");
    let battery_id = part_id(&sidecar, "battery");
    let prop_id = part_id(&sidecar, "propeller");

    let table_path = root.join("parts_db/validation/tmotor_mn3508_kv700_table.json");
    let table = parse_motor_table(&read(&table_path)).expect("motor table");
    let prop_rec: serde_json::Value = serde_json::from_str(&read(
        &root.join(format!("parts_db/parts/propeller/{prop_id}.json")),
    ))
    .expect("prop record");
    let data_file = sidecar["powertrain"]["propeller_table"]["source_file"]
        .as_str()
        .expect("propeller source file");
    let _ = prop_rec;
    let prop = parse_prop_data(&read(&root.join(data_file))).expect("prop data");
    let battery = parse_battery_record(&read(
        &root.join(format!("parts_db/parts/battery/{battery_id}.json")),
    ))
    .expect("battery record");
    let bands = Bands::parse(&read(
        &root.join("parts_db/validation/acceptance_bands.json"),
    ))
    .expect("bands");
    let curve_path = root.join(format!(
        "parts_db/validation/battery_curve_{battery_id}.csv"
    ));
    let curve = curve_path
        .exists()
        .then(|| parse_curve_csv(&read(&curve_path)).expect("battery curve csv"));

    let build = dir
        .file_name()
        .map_or_else(|| "build".to_string(), |n| n.to_string_lossy().into_owned());
    let mut files = vec![
        RefFile {
            label: "tmotor_mn3508_kv700_table.json".into(),
            origin: format!("{} sha256 {}", table.source.url, table.source.sha256),
        },
        RefFile {
            label: data_file.into(),
            origin: "UIUC propeller database (Selig), see the file's citation".into(),
        },
        RefFile {
            label: format!("parts/battery/{battery_id}.json"),
            origin: "battery part record (cited fields)".into(),
        },
        RefFile {
            label: "acceptance_bands.json".into(),
            origin: "ASSUMED bands".into(),
        },
    ];
    if curve.is_some() {
        files.push(RefFile {
            label: format!("battery_curve_{battery_id}.csv"),
            origin: "supplied discharge curve".into(),
        });
    }
    let inputs = ReportInputs {
        build: &build,
        motor_id: &motor_id,
        cfg: &cfg,
        motor_table: &table,
        prop: &prop,
        battery: &battery,
        curve: curve.as_deref(),
        bands: &bands,
        reference_files: files,
    };
    let report = build_report(&inputs).expect("report");
    fs::create_dir_all(&out).expect("create out dir");
    fs::write(
        out.join("validation_report.json"),
        to_json(&report).expect("json"),
    )
    .expect("write json");
    fs::write(
        out.join("validation_report.csv"),
        to_csv(&report).expect("csv"),
    )
    .expect("write csv");
    for s in &report.sections {
        let m = &s.summary;
        println!(
            "{:<22} {:?} rows {:>3} mean|%| {:>8.3} rms {:>8.3} max|%| {:>9.3} ({}) outside band {}",
            s.name,
            s.status,
            m.n,
            m.mean_abs_percent,
            m.rms_percent,
            m.max_abs_percent,
            m.worst_row_id.as_deref().unwrap_or("-"),
            m.n_outside_band
        );
    }
}
