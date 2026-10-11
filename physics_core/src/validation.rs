// Tier: F
//! Validation module (P3d): compares the shipped core models with published data and reports the error per point.
//!
//! Sections (see `parts_db/validation/README.md` for the reference files):
//! * `motor_electrical`: the 20 T-Motor MN3508 KV700 bench rows. Prop-independent: model motor voltage `rpm / Kv + I Rm` against the
//!   voltage the throttle setting implies (`throttle x supply`), and the published power against `V x I`. Torque, `Ct` and `Cp` derived
//!   from thrust, rpm and current are INFORMATIONAL only (the table's carbon propellers have no Ct/Cp data, so thrust is not predicted).
//! * `uiuc_propeller`: `powertrain::coefficients` against every raw UIUC wind-tunnel point and static row of the propeller file.
//! * `battery_cited_points`: the model against the numbers the battery record cites (max / nominal / cutoff volts per cell, internal
//!   resistance, capacity).
//! * `battery_curve`: `NoPublishedData` unless a real discharge curve CSV is supplied.
//!
//! Report only: a row outside its (ASSUMED) band is flagged, never failed. Percent error is `(model - reference) / |reference| x 100`
//! and is left out (`None`) for rows whose reference is below 5 % of the largest |reference| of the same quantity (a coefficient that
//! crosses zero would give meaningless percentages); the absolute error is always there.
//!
//! Offline module: it allocates and reads files, and is never part of a step loop. It validates the real functions
//! (`powertrain::coefficients`, `ocv_per_cell`, `loaded_voltage_v`, `step`), not a copy.

use crate::aero;
use crate::powertrain::{self, StepInput};
use crate::types::{EnvironmentInput, PowertrainConfig};
use serde::Deserialize;
use std::collections::BTreeMap;
use std::fmt::Write;

/// Version of the report layout.
pub const SCHEMA_VERSION: u32 = 1;
/// Rows whose |reference| is below this fraction of the largest |reference| of their quantity get no percent error.
pub const PERCENT_FLOOR_FRACTION: f64 = 0.05;
/// Metres per inch (exact by definition, 1959 international yard and pound agreement).
const INCH_M: f64 = 0.0254;
/// Standard gravity, m/s^2 (CGPM 1901), to turn gram-force into newtons.
const GRAVITY_MPS2: f64 = 9.806_65;
/// Current used for the battery sag row, A.
const SAG_CURRENT_A: f64 = 10.0;
/// Step used to draw charge from the battery model, s (at most the powertrain limit of 10 ms).
const COULOMB_DT_S: f64 = 0.01;
/// Upper bound on the steps of the coulomb-counting run.
const COULOMB_MAX_STEPS: u32 = 200_000;
/// Charge drawn in the coulomb-counting run, Ah.
const COULOMB_TARGET_AH: f64 = 1.0;

/// Why a validation input or report was refused.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum ValidationError {
    /// A file could not be read.
    Io,
    /// Text is not valid JSON / CSV of the expected shape.
    Parse,
    /// A value is out of range, non-finite or inconsistent.
    Invalid,
    /// A model call failed (config the model refuses).
    Model,
    /// A report value is not finite and cannot be written.
    NonFinite,
}

/// Outcome of one section.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum Status {
    /// Rows were computed.
    Ok,
    /// The reference data does not exist (e.g. no discharge curve).
    NoPublishedData,
    /// The build does not use the part the data is about.
    NotApplicable,
}

impl Status {
    fn label(self) -> &'static str {
        match self {
            Self::Ok => "OK",
            Self::NoPublishedData => "NO_PUBLISHED_DATA",
            Self::NotApplicable => "NOT_APPLICABLE",
        }
    }
}

/// One compared point.
#[derive(Clone, Debug, PartialEq)]
pub struct ErrorRow {
    /// Row id (stable, from the reference data).
    pub id: String,
    /// Quantity name; `<quantity>_percent` is its band key.
    pub quantity: String,
    /// Unit of `reference` and `model`.
    pub unit: &'static str,
    /// Published (or cited) value.
    pub reference: f64,
    /// Model value.
    pub model: f64,
    /// `model - reference`.
    pub abs_error: f64,
    /// `(model - reference) / |reference| x 100`, or `None` below the 5 % floor.
    pub percent_error: Option<f64>,
    /// The model had to hold an edge value (outside its own data).
    pub outside_data: bool,
    /// Band for this quantity, percent, if configured.
    pub band_percent: Option<f64>,
    /// `|percent| > band`; `None` without percent or band.
    pub outside_band: Option<bool>,
    /// Where the reference value comes from.
    pub source: String,
}

/// A derived value shown for information only (no reference to compare with).
#[derive(Clone, Debug, PartialEq)]
pub struct InfoRow {
    /// Row id.
    pub id: String,
    /// Quantity name.
    pub quantity: &'static str,
    /// Value.
    pub value: f64,
    /// Unit.
    pub unit: &'static str,
}

/// Statistics over the rows of a section that have a percent error.
#[derive(Clone, Debug, PartialEq)]
pub struct Summary {
    /// Rows in the section.
    pub n: usize,
    /// Rows with a percent error.
    pub n_with_percent: usize,
    /// Mean of |percent|.
    pub mean_abs_percent: f64,
    /// Root mean square of percent.
    pub rms_percent: f64,
    /// Largest |percent|.
    pub max_abs_percent: f64,
    /// Id of the row with the largest |percent| (first if tied).
    pub worst_row_id: Option<String>,
    /// Rows outside their band.
    pub n_outside_band: usize,
}

/// One section of the report.
#[derive(Clone, Debug, PartialEq)]
pub struct Section {
    /// Section name.
    pub name: &'static str,
    /// Outcome.
    pub status: Status,
    /// Why the status is not `Ok`, or what the section does not cover.
    pub note: String,
    /// Compared rows.
    pub rows: Vec<ErrorRow>,
    /// Informational rows.
    pub info: Vec<InfoRow>,
    /// Statistics of `rows`.
    pub summary: Summary,
}

/// A reference file and where its data comes from.
#[derive(Clone, Debug, PartialEq)]
pub struct RefFile {
    /// Label (file name).
    pub label: String,
    /// Origin pin (sha256 of the saved source page) or citation text.
    pub origin: String,
}

/// The whole report.
#[derive(Clone, Debug, PartialEq)]
pub struct Report {
    /// Layout version.
    pub schema_version: u32,
    /// Build name.
    pub build: String,
    /// `ffi_get_interface_version()` of this core.
    pub core_interface_version: i32,
    /// Reference files used.
    pub reference_files: Vec<RefFile>,
    /// Sections in a fixed order.
    pub sections: Vec<Section>,
}

// ----------------------------------------------------------------------------------------------- reference data

/// One row of the T-Motor bench table.
#[derive(Clone, Debug, PartialEq, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct MotorRow {
    /// Row id.
    pub id: String,
    /// Supply voltage, V.
    pub supply_voltage_v: f64,
    /// Propeller name as printed.
    pub prop: String,
    /// Propeller diameter, inch.
    pub prop_diameter_in: f64,
    /// Propeller pitch, inch.
    pub prop_pitch_in: f64,
    /// Throttle, percent.
    pub throttle_percent: f64,
    /// Current, A.
    pub current_a: f64,
    /// Power, W.
    pub power_w: f64,
    /// Thrust, gram-force.
    pub thrust_g: f64,
    /// Speed, rev/min.
    pub rpm: f64,
    /// Efficiency, gf/W.
    pub efficiency_g_per_w: f64,
    /// Motor surface temperature, deg C (first row of each propeller only).
    pub temperature_c: Option<f64>,
}

/// Origin of a reference file.
#[derive(Clone, Debug, PartialEq, Deserialize)]
pub struct SourceRef {
    /// Page URL.
    pub url: String,
    /// sha256 of the saved page.
    pub sha256: String,
}

/// `tmotor_mn3508_kv700_table.json`.
#[derive(Clone, Debug, PartialEq, Deserialize)]
pub struct MotorTable {
    /// Layout version (1).
    pub schema_version: u32,
    /// Part id the data is about.
    pub motor_id: String,
    /// Origin.
    pub source: SourceRef,
    /// The rows.
    pub rows: Vec<MotorRow>,
}

#[derive(Deserialize)]
struct RawPoint {
    j: f64,
    ct: f64,
    cp: f64,
}

#[derive(Deserialize)]
struct StaticRow {
    rpm: f64,
    ct: f64,
    cp: f64,
}

/// `offline_pipeline/props/*.json` (only the measured data is read).
#[derive(Deserialize)]
pub struct PropData {
    source: String,
    wind_tunnel: BTreeMap<String, Vec<RawPoint>>,
    #[serde(rename = "static")]
    static_rows: Vec<StaticRow>,
}

/// Numbers a battery record cites, volts per cell unless named.
#[derive(Clone, Debug, PartialEq)]
pub struct BatteryRef {
    /// Part id.
    pub id: String,
    /// Series cells.
    pub cells: f64,
    /// Capacity, Ah.
    pub capacity_ah: f64,
    /// Maximum (full) voltage per cell, V.
    pub v_max_cell: f64,
    /// Nominal voltage per cell, V.
    pub v_nominal_cell: f64,
    /// Cutoff voltage per cell, V.
    pub v_cutoff_cell: f64,
    /// Internal resistance of the pack, ohm.
    pub internal_resistance_ohm: f64,
}

/// Acceptance bands, percent per quantity (absent = no band).
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Bands {
    map: BTreeMap<String, f64>,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct BandsJson {
    schema_version: u32,
    #[allow(dead_code)]
    description: Option<String>,
    #[allow(dead_code)]
    provenance: serde_json::Value,
    bands_percent: BTreeMap<String, f64>,
}

const BAND_QUANTITIES: [&str; 8] = [
    "motor_voltage_percent",
    "motor_power_self_consistency_percent",
    "uiuc_ct_percent",
    "uiuc_cp_percent",
    "battery_ocv_percent",
    "battery_sag_percent",
    "battery_soc_percent",
    "battery_curve_percent",
];

fn positive(v: f64) -> bool {
    v.is_finite() && v > 0.0
}

/// Parses the motor bench table.
///
/// # Errors
/// `Parse` for bad JSON or an unknown / missing row key; `Invalid` for a wrong version, no rows, duplicate ids or a non-positive value.
pub fn parse_motor_table(json: &str) -> Result<MotorTable, ValidationError> {
    let t: MotorTable = serde_json::from_str(json).map_err(|_| ValidationError::Parse)?;
    let mut ids: Vec<&str> = t.rows.iter().map(|r| r.id.as_str()).collect();
    ids.sort_unstable();
    let unique = ids.windows(2).all(|w| matches!(w, [a, b] if a != b));
    let values_ok = t.rows.iter().all(|r| {
        [
            r.supply_voltage_v,
            r.prop_diameter_in,
            r.prop_pitch_in,
            r.throttle_percent,
            r.current_a,
            r.power_w,
            r.thrust_g,
            r.rpm,
            r.efficiency_g_per_w,
        ]
        .iter()
        .all(|&v| positive(v))
            && r.temperature_c.is_none_or(f64::is_finite)
    });
    if t.schema_version == SCHEMA_VERSION && !t.rows.is_empty() && unique && values_ok {
        Ok(t)
    } else {
        Err(ValidationError::Invalid)
    }
}

/// Parses the UIUC propeller file.
///
/// # Errors
/// `Parse` for bad JSON or missing sections; `Invalid` for no data or a non-finite value.
pub fn parse_prop_data(json: &str) -> Result<PropData, ValidationError> {
    let p: PropData = serde_json::from_str(json).map_err(|_| ValidationError::Parse)?;
    let finite = p
        .wind_tunnel
        .values()
        .flatten()
        .all(|q| q.j.is_finite() && q.ct.is_finite() && q.cp.is_finite())
        && p.static_rows
            .iter()
            .all(|s| positive(s.rpm) && s.ct.is_finite() && s.cp.is_finite());
    if finite && !p.wind_tunnel.is_empty() && !p.static_rows.is_empty() {
        Ok(p)
    } else {
        Err(ValidationError::Invalid)
    }
}

fn field(v: &serde_json::Value, key: &str) -> Option<f64> {
    v.get(key)?.get("value")?.as_f64()
}

/// Reads the cited numbers of a battery part record (`parts_db/parts/battery/<id>.json`).
///
/// # Errors
/// `Parse` for bad JSON or a missing field; `Invalid` for non-positive values.
pub fn parse_battery_record(json: &str) -> Result<BatteryRef, ValidationError> {
    let v: serde_json::Value = serde_json::from_str(json).map_err(|_| ValidationError::Parse)?;
    let id = v
        .get("id")
        .and_then(serde_json::Value::as_str)
        .ok_or(ValidationError::Parse)?
        .to_string();
    let get = |k: &str| field(&v, k).ok_or(ValidationError::Parse);
    let b = BatteryRef {
        id,
        cells: get("cells")?,
        capacity_ah: get("capacity_ah")?,
        v_max_cell: get("voltage_per_cell_max_v")?,
        v_nominal_cell: get("voltage_per_cell_nominal_v")?,
        v_cutoff_cell: get("voltage_per_cell_cutoff_v")?,
        internal_resistance_ohm: get("internal_resistance_ohm")?,
    };
    let ok = [
        b.cells,
        b.capacity_ah,
        b.v_max_cell,
        b.v_nominal_cell,
        b.v_cutoff_cell,
        b.internal_resistance_ohm,
    ]
    .iter()
    .all(|&x| positive(x))
        && b.v_cutoff_cell < b.v_nominal_cell
        && b.v_nominal_cell < b.v_max_cell;
    if ok {
        Ok(b)
    } else {
        Err(ValidationError::Invalid)
    }
}

impl Bands {
    /// No bands at all.
    #[must_use]
    pub fn none() -> Self {
        Self::default()
    }

    /// Band for a quantity (`<quantity>_percent` key), percent.
    #[must_use]
    pub fn get(&self, quantity: &str) -> Option<f64> {
        self.map.get(&format!("{quantity}_percent")).copied()
    }

    /// Parses `acceptance_bands.json`.
    ///
    /// # Errors
    /// `Parse` for bad JSON or an unknown key; `Invalid` for a wrong version, an unknown quantity or a non-positive band.
    pub fn parse(json: &str) -> Result<Self, ValidationError> {
        let b: BandsJson = serde_json::from_str(json).map_err(|_| ValidationError::Parse)?;
        let ok = b.schema_version == SCHEMA_VERSION
            && b.bands_percent
                .iter()
                .all(|(k, &v)| BAND_QUANTITIES.contains(&k.as_str()) && positive(v));
        if ok {
            Ok(Self {
                map: b.bands_percent,
            })
        } else {
            Err(ValidationError::Invalid)
        }
    }
}

// ----------------------------------------------------------------------------------------------- rows and statistics

fn new_row(
    id: String,
    quantity: &str,
    unit: &'static str,
    reference: f64,
    model: f64,
    outside_data: bool,
    source: &str,
) -> ErrorRow {
    ErrorRow {
        id,
        quantity: quantity.to_string(),
        unit,
        reference,
        model,
        abs_error: model - reference,
        percent_error: None,
        outside_data,
        band_percent: None,
        outside_band: None,
        source: source.to_string(),
    }
}

/// Fills `percent_error`, `band_percent` and `outside_band` of every row: the floor is 5 % of the largest |reference| of the
/// row's quantity.
fn finish_rows(rows: &mut [ErrorRow], bands: &Bands) {
    let mut largest: BTreeMap<String, f64> = BTreeMap::new();
    for r in rows.iter() {
        let e = largest.entry(r.quantity.clone()).or_insert(0.0);
        *e = e.max(r.reference.abs());
    }
    for r in rows.iter_mut() {
        let floor = PERCENT_FLOOR_FRACTION * largest.get(&r.quantity).copied().unwrap_or(0.0);
        r.percent_error = (r.reference.abs() >= floor && r.reference != 0.0)
            .then(|| (r.model - r.reference) / r.reference.abs() * 100.0);
        r.band_percent = bands.get(&r.quantity);
        r.outside_band = r
            .percent_error
            .zip(r.band_percent)
            .map(|(p, band)| p.abs() > band);
    }
}

/// Mean / rms / max of the percent errors of `rows`.
#[must_use]
pub fn summarize(rows: &[ErrorRow]) -> Summary {
    let with: Vec<(&str, f64)> = rows
        .iter()
        .filter_map(|r| r.percent_error.map(|p| (r.id.as_str(), p)))
        .collect();
    let count = u32::try_from(with.len()).unwrap_or(u32::MAX);
    let nf = f64::from(count.max(1));
    let mut worst: Option<(&str, f64)> = None;
    for &(id, p) in &with {
        if worst.is_none_or(|(_, w)| p.abs() > w) {
            worst = Some((id, p.abs()));
        }
    }
    Summary {
        n: rows.len(),
        n_with_percent: with.len(),
        // + 0.0 turns the -0.0 that an empty float sum gives into 0.0
        mean_abs_percent: (with.iter().map(|(_, p)| p.abs()).sum::<f64>() + 0.0) / nf,
        rms_percent: ((with.iter().map(|(_, p)| p * p).sum::<f64>() + 0.0) / nf).sqrt(),
        max_abs_percent: worst.map_or(0.0, |(_, w)| w),
        worst_row_id: worst.map(|(id, _)| id.to_string()),
        n_outside_band: rows.iter().filter(|r| r.outside_band == Some(true)).count(),
    }
}

fn make_section(
    name: &'static str,
    status: Status,
    note: &str,
    mut rows: Vec<ErrorRow>,
    info: Vec<InfoRow>,
    bands: &Bands,
) -> Section {
    finish_rows(&mut rows, bands);
    let summary = summarize(&rows);
    Section {
        name,
        status,
        note: note.to_string(),
        rows,
        info,
        summary,
    }
}

fn empty_section(name: &'static str, status: Status, note: &str) -> Section {
    make_section(name, status, note, Vec::new(), Vec::new(), &Bands::none())
}

// ----------------------------------------------------------------------------------------------- sections

fn info(id: &str, quantity: &'static str, value: f64, unit: &'static str) -> InfoRow {
    InfoRow {
        id: id.to_string(),
        quantity,
        value,
        unit,
    }
}

/// Section A: motor electrical check against the bench table. Applicable only if `motor_id` is the table's motor.
///
/// # Errors
/// `Model` if the standard atmosphere cannot be evaluated.
pub fn motor_section(
    table: &MotorTable,
    cfg: &PowertrainConfig,
    motor_id: &str,
    bands: &Bands,
) -> Result<Section, ValidationError> {
    const NAME: &str = "motor_electrical";
    if motor_id != table.motor_id {
        return Ok(empty_section(
            NAME,
            Status::NotApplicable,
            &format!(
                "the build's motor is {motor_id}; the table is for {}",
                table.motor_id
            ),
        ));
    }
    let rho = aero::air_density(0.0, &EnvironmentInput::standard())
        .map_err(|_| ValidationError::Model)?;
    let kt = powertrain::kt_nm_per_a(cfg.motor_kv_rpm_per_v);
    let src = format!("T-Motor store page, sha256 {}", table.source.sha256);
    let mut rows = Vec::new();
    let mut infos = Vec::new();
    for r in &table.rows {
        let v_model = r.rpm / cfg.motor_kv_rpm_per_v + r.current_a * cfg.motor_resistance_ohm;
        let v_ref = r.throttle_percent / 100.0 * r.supply_voltage_v;
        rows.push(new_row(
            format!("{}:voltage", r.id),
            "motor_voltage",
            "V",
            v_ref,
            v_model,
            false,
            &src,
        ));
        rows.push(new_row(
            format!("{}:power", r.id),
            "motor_power_self_consistency",
            "W",
            r.power_w,
            r.supply_voltage_v * r.current_a,
            false,
            &src,
        ));
        let torque = kt * (r.current_a - cfg.motor_no_load_current_a);
        let n = r.rpm / 60.0;
        let d = r.prop_diameter_in * INCH_M;
        let thrust_n = r.thrust_g * 1e-3 * GRAVITY_MPS2;
        infos.push(info(&r.id, "derived_torque", torque, "N m"));
        infos.push(info(
            &r.id,
            "derived_ct",
            thrust_n / (rho * n * n * d.powi(4)),
            "1",
        ));
        infos.push(info(
            &r.id,
            "derived_cp",
            2.0 * std::f64::consts::PI * torque / (rho * n * n * d.powi(5)),
            "1",
        ));
    }
    Ok(make_section(
        NAME,
        Status::Ok,
        "Prop-independent. The voltage residual is the ESC / wiring / supply drop and the meaning of 'throttle' on the page, which the model does not know. Thrust and efficiency are NOT validated (the carbon propellers have no Ct/Cp data); derived Ct / Cp are informational.",
        rows,
        infos,
        bands,
    ))
}

/// Nominal rpm of a UIUC run from its file name (`..._3040.txt` -> 3040).
fn run_rpm(name: &str) -> Option<f64> {
    let stem = name.strip_suffix(".txt")?;
    let digits = stem.rsplit('_').next()?;
    digits.parse::<f64>().ok().filter(|v| positive(*v))
}

/// Section B: `powertrain::coefficients` against every raw point and static row.
///
/// # Errors
/// `Parse` if a run name has no rpm; `Model` if the model refuses a point.
pub fn uiuc_section(
    prop: &PropData,
    cfg: &PowertrainConfig,
    bands: &Bands,
) -> Result<Section, ValidationError> {
    const NAME: &str = "uiuc_propeller";
    if cfg.prop_model != 1 {
        return Ok(empty_section(
            NAME,
            Status::NotApplicable,
            "the build uses the static Ct/Cp (prop_model 0): no table to validate",
        ));
    }
    let mut rows = Vec::new();
    for (name, points) in &prop.wind_tunnel {
        let rpm = run_rpm(name).ok_or(ValidationError::Parse)?;
        let mut pts: Vec<(f64, f64, f64)> = points.iter().map(|p| (p.j, p.ct, p.cp)).collect();
        pts.sort_by(|a, b| {
            a.0.total_cmp(&b.0)
                .then(a.1.total_cmp(&b.1))
                .then(a.2.total_cmp(&b.2))
        });
        pts.dedup(); // exact duplicate rows (the files repeat their last row) carry no information
        for (k, &(j, ct, cp)) in pts.iter().enumerate() {
            let m = powertrain::coefficients(cfg, j, rpm).map_err(|_| ValidationError::Model)?;
            let id = format!("{name}#{k}");
            rows.push(new_row(
                format!("{id}:ct"),
                "uiuc_ct",
                "1",
                ct,
                m.ct,
                m.extrapolated,
                &prop.source,
            ));
            rows.push(new_row(
                format!("{id}:cp"),
                "uiuc_cp",
                "1",
                cp,
                m.cp,
                m.extrapolated,
                &prop.source,
            ));
        }
    }
    for s in &prop.static_rows {
        let m = powertrain::coefficients(cfg, 0.0, s.rpm).map_err(|_| ValidationError::Model)?;
        let id = format!("static@{}rpm", s.rpm);
        rows.push(new_row(
            format!("{id}:ct"),
            "uiuc_ct",
            "1",
            s.ct,
            m.ct,
            m.extrapolated,
            &prop.source,
        ));
        rows.push(new_row(
            format!("{id}:cp"),
            "uiuc_cp",
            "1",
            s.cp,
            m.cp,
            m.extrapolated,
            &prop.source,
        ));
    }
    Ok(make_section(
        NAME,
        Status::Ok,
        "Run points are evaluated at the run's NOMINAL rpm (from the file name); the table curves sit at the mean rpm of merged runs, so small differences are expected. Rows with outside_data = true are held edge values.",
        rows,
        Vec::new(),
        bands,
    ))
}

fn ocv_pack(cfg: &PowertrainConfig, soc: f64) -> Result<f64, ValidationError> {
    let (per_cell, _) = powertrain::ocv_per_cell(cfg, soc).map_err(|_| ValidationError::Model)?;
    Ok(per_cell * f64::from(cfg.battery_cells))
}

/// Draws `COULOMB_TARGET_AH` from a full pack at full duty and returns `(charge_used_ah, soc)`.
fn draw_one_amp_hour(cfg: &PowertrainConfig) -> Result<(f64, f64), ValidationError> {
    let rho = aero::air_density(0.0, &EnvironmentInput::standard())
        .map_err(|_| ValidationError::Model)?;
    let mut state = powertrain::initial_state(1.0, 20.0).map_err(|_| ValidationError::Model)?;
    let input = StepInput {
        duties: [1.0; 4],
        rho_kgpm3: rho,
        airspeed_mps: 0.0,
        axial_inflow_mps: 0.0,
        ambient_c: 20.0,
        dt_s: COULOMB_DT_S,
    };
    for _ in 0..COULOMB_MAX_STEPS {
        powertrain::step(cfg, &mut state, &input).map_err(|_| ValidationError::Model)?;
        if state.charge_used_ah >= COULOMB_TARGET_AH {
            return Ok((state.charge_used_ah, state.soc));
        }
    }
    Err(ValidationError::Model)
}

/// Section C: the model against the numbers the battery record cites.
///
/// # Errors
/// `Model` if a model call fails or the coulomb-counting run does not reach 1 Ah.
pub fn battery_section(
    batt: &BatteryRef,
    cfg: &PowertrainConfig,
    bands: &Bands,
) -> Result<Section, ValidationError> {
    let src = format!("battery record {} (CITED fields)", batt.id);
    let mut rows = vec![
        new_row(
            "ocv_full".into(),
            "battery_ocv",
            "V",
            batt.cells * batt.v_max_cell,
            ocv_pack(cfg, 1.0)?,
            false,
            &src,
        ),
        new_row(
            "ocv_mid_vs_nominal".into(),
            "battery_ocv",
            "V",
            batt.cells * batt.v_nominal_cell,
            ocv_pack(cfg, 0.5)?,
            false,
            &src,
        ),
        new_row(
            "ocv_empty_vs_cutoff".into(),
            "battery_ocv",
            "V",
            batt.cells * batt.v_cutoff_cell,
            ocv_pack(cfg, 0.0)?,
            false,
            &src,
        ),
    ];
    let v_oc = ocv_pack(cfg, 0.5)?;
    let sag_model = v_oc
        - powertrain::loaded_voltage_v(v_oc, SAG_CURRENT_A, cfg.battery_internal_resistance_ohm);
    rows.push(new_row(
        "sag_at_10a".into(),
        "battery_sag",
        "V",
        SAG_CURRENT_A * batt.internal_resistance_ohm,
        sag_model,
        false,
        &src,
    ));
    let (used, soc) = draw_one_amp_hour(cfg)?;
    rows.push(new_row(
        "soc_after_charge".into(),
        "battery_soc",
        "1",
        1.0 - used / batt.capacity_ah,
        soc,
        false,
        &src,
    ));
    Ok(make_section(
        "battery_cited_points",
        Status::Ok,
        "Cited points only. The nominal voltage is a mid-discharge average, so a difference at 50 % SOC is expected. The discharge curve itself is in battery_curve.",
        rows,
        Vec::new(),
        bands,
    ))
}

/// One line of a discharge-curve CSV.
#[derive(Clone, Debug, PartialEq)]
pub struct CurvePoint {
    /// State of charge, 0..1.
    pub soc: f64,
    /// Measured pack voltage under the stated current, V.
    pub voltage_v: f64,
    /// Current during the measurement, A (0 = open circuit).
    pub current_a: f64,
    /// Source of the line.
    pub source: String,
}

/// Parses a discharge-curve CSV with the header `soc_fraction,voltage_v,current_a,source`.
///
/// # Errors
/// `Parse` for a wrong header or a malformed line; `Invalid` for a non-finite value, SOC outside 0..1, non-positive voltage or negative current.
pub fn parse_curve_csv(text: &str) -> Result<Vec<CurvePoint>, ValidationError> {
    let mut lines = text.lines().filter(|l| !l.trim().is_empty());
    if lines.next().map(str::trim) != Some("soc_fraction,voltage_v,current_a,source") {
        return Err(ValidationError::Parse);
    }
    let mut out = Vec::new();
    for line in lines {
        let cols: Vec<&str> = line.splitn(4, ',').collect();
        let [soc, volts, amps, source] = cols.as_slice() else {
            return Err(ValidationError::Parse);
        };
        let num = |s: &str| s.trim().parse::<f64>().map_err(|_| ValidationError::Parse);
        let p = CurvePoint {
            soc: num(soc)?,
            voltage_v: num(volts)?,
            current_a: num(amps)?,
            source: source.trim().to_string(),
        };
        let ok = p.soc.is_finite()
            && (0.0..=1.0).contains(&p.soc)
            && positive(p.voltage_v)
            && p.current_a.is_finite()
            && p.current_a >= 0.0
            && !p.source.is_empty();
        if !ok {
            return Err(ValidationError::Invalid);
        }
        out.push(p);
    }
    if out.is_empty() {
        Err(ValidationError::Invalid)
    } else {
        Ok(out)
    }
}

/// Section D: the discharge curve. `NoPublishedData` without a CSV.
///
/// # Errors
/// `Model` if the OCV table cannot be evaluated.
pub fn curve_section(
    points: Option<&[CurvePoint]>,
    cfg: &PowertrainConfig,
    bands: &Bands,
) -> Result<Section, ValidationError> {
    const NAME: &str = "battery_curve";
    let Some(points) = points else {
        return Ok(empty_section(
            NAME,
            Status::NoPublishedData,
            "no published discharge curve is stored; the OCV curve in the database is an ASSUMED generic shape. Supply parts_db/validation/battery_curve_<battery id>.csv",
        ));
    };
    let mut rows = Vec::new();
    for (k, p) in points.iter().enumerate() {
        let v_oc = ocv_pack(cfg, p.soc)?;
        let model =
            powertrain::loaded_voltage_v(v_oc, p.current_a, cfg.battery_internal_resistance_ohm);
        rows.push(new_row(
            format!("curve#{k}"),
            "battery_curve",
            "V",
            p.voltage_v,
            model,
            false,
            &p.source,
        ));
    }
    Ok(make_section(
        NAME,
        Status::Ok,
        "model OCV(SOC) minus I x R_int against the supplied curve",
        rows,
        Vec::new(),
        bands,
    ))
}

/// Everything `build_report` needs, already loaded.
pub struct ReportInputs<'a> {
    /// Build name.
    pub build: &'a str,
    /// Motor part id of the build.
    pub motor_id: &'a str,
    /// The build's powertrain config.
    pub cfg: &'a PowertrainConfig,
    /// T-Motor bench table.
    pub motor_table: &'a MotorTable,
    /// UIUC data of the build's propeller.
    pub prop: &'a PropData,
    /// Cited numbers of the build's battery.
    pub battery: &'a BatteryRef,
    /// Supplied discharge curve, if any.
    pub curve: Option<&'a [CurvePoint]>,
    /// Acceptance bands.
    pub bands: &'a Bands,
    /// Reference files for the manifest.
    pub reference_files: Vec<RefFile>,
}

/// Runs all four sections.
///
/// # Errors
/// The first error of any section.
pub fn build_report(inp: &ReportInputs<'_>) -> Result<Report, ValidationError> {
    Ok(Report {
        schema_version: SCHEMA_VERSION,
        build: inp.build.to_string(),
        core_interface_version: crate::ffi_get_interface_version(),
        reference_files: inp.reference_files.clone(),
        sections: vec![
            motor_section(inp.motor_table, inp.cfg, inp.motor_id, inp.bands)?,
            uiuc_section(inp.prop, inp.cfg, inp.bands)?,
            battery_section(inp.battery, inp.cfg, inp.bands)?,
            curve_section(inp.curve, inp.cfg, inp.bands)?,
        ],
    })
}

// ----------------------------------------------------------------------------------------------- writers

fn json_str(s: &str) -> String {
    let mut out = String::with_capacity(s.len() + 2);
    out.push('"');
    for c in s.chars() {
        match c {
            '"' => out.push_str("\\\""),
            '\\' => out.push_str("\\\\"),
            '\n' => out.push_str("\\n"),
            c if (c as u32) < 0x20 => {
                // writing to a String cannot fail
                write!(out, "\\u{:04x}", c as u32).ok();
            }
            c => out.push(c),
        }
    }
    out.push('"');
    out
}

/// 12 significant digits: the last bits of a value (libm differences, 25.529999999999998 vs 25.53) are not information.
fn round12(v: f64) -> f64 {
    format!("{v:.11e}").parse().unwrap_or(v)
}

fn json_num(v: f64) -> Result<String, ValidationError> {
    if v.is_finite() {
        Ok(format!("{}", round12(v)))
    } else {
        Err(ValidationError::NonFinite)
    }
}

fn json_opt(v: Option<f64>) -> Result<String, ValidationError> {
    v.map_or(Ok("null".to_string()), json_num)
}

fn json_row(r: &ErrorRow) -> Result<String, ValidationError> {
    let band_flag = r
        .outside_band
        .map_or("null", |b| if b { "true" } else { "false" });
    Ok(format!(
        "{{\"id\": {}, \"quantity\": {}, \"unit\": {}, \"reference\": {}, \"model\": {}, \"abs_error\": {}, \"percent_error\": {}, \"outside_data\": {}, \"band_percent\": {}, \"outside_band\": {band_flag}, \"source\": {}}}",
        json_str(&r.id),
        json_str(&r.quantity),
        json_str(r.unit),
        json_num(r.reference)?,
        json_num(r.model)?,
        json_num(r.abs_error)?,
        json_opt(r.percent_error)?,
        r.outside_data,
        json_opt(r.band_percent)?,
        json_str(&r.source),
    ))
}

fn json_section(s: &Section) -> Result<String, ValidationError> {
    let rows = s.rows.iter().map(json_row).collect::<Result<Vec<_>, _>>()?;
    let infos = s
        .info
        .iter()
        .map(|i| {
            Ok(format!(
                "{{\"id\": {}, \"quantity\": {}, \"value\": {}, \"unit\": {}}}",
                json_str(&i.id),
                json_str(i.quantity),
                json_num(i.value)?,
                json_str(i.unit)
            ))
        })
        .collect::<Result<Vec<_>, ValidationError>>()?;
    let m = &s.summary;
    Ok(format!(
        "    {{\"name\": {}, \"status\": {}, \"note\": {}, \"summary\": {{\"n\": {}, \"n_with_percent\": {}, \"mean_abs_percent\": {}, \"rms_percent\": {}, \"max_abs_percent\": {}, \"worst_row_id\": {}, \"n_outside_band\": {}}},\n      \"rows\": [\n        {}\n      ],\n      \"info\": [\n        {}\n      ]}}",
        json_str(s.name),
        json_str(s.status.label()),
        json_str(&s.note),
        m.n,
        m.n_with_percent,
        json_num(m.mean_abs_percent)?,
        json_num(m.rms_percent)?,
        json_num(m.max_abs_percent)?,
        m.worst_row_id
            .as_deref()
            .map_or("null".to_string(), json_str),
        m.n_outside_band,
        rows.join(",\n        "),
        infos.join(",\n        "),
    ))
}

/// The report as JSON text (stable key order; refuses a non-finite value).
///
/// # Errors
/// `NonFinite` if any value is NaN or infinite.
pub fn to_json(report: &Report) -> Result<String, ValidationError> {
    let files = report
        .reference_files
        .iter()
        .map(|f| {
            format!(
                "{{\"label\": {}, \"origin\": {}}}",
                json_str(&f.label),
                json_str(&f.origin)
            )
        })
        .collect::<Vec<_>>()
        .join(", ");
    let sections = report
        .sections
        .iter()
        .map(json_section)
        .collect::<Result<Vec<_>, _>>()?;
    let mut out = String::new();
    write!(
        out,
        "{{\n  \"schema_version\": {},\n  \"build\": {},\n  \"core_interface_version\": {},\n  \"reference_files\": [{files}],\n  \"sections\": [\n{}\n  ]\n}}\n",
        report.schema_version,
        json_str(&report.build),
        report.core_interface_version,
        sections.join(",\n"),
    )
    .map_err(|_| ValidationError::Invalid)?;
    Ok(out)
}

fn csv_field(s: &str) -> String {
    if s.contains([',', '"', '\n']) {
        format!("\"{}\"", s.replace('"', "\"\""))
    } else {
        s.to_string()
    }
}

/// The report as CSV, one line per compared or informational row.
///
/// # Errors
/// `NonFinite` if any value is NaN or infinite.
pub fn to_csv(report: &Report) -> Result<String, ValidationError> {
    let mut out = String::from(
        "section,status,id,quantity,unit,reference,model,abs_error,percent_error,outside_data,band_percent,outside_band,source\n",
    );
    let opt = |v: Option<f64>| v.map_or(Ok(String::new()), json_num);
    for s in &report.sections {
        for r in &s.rows {
            let flag = r
                .outside_band
                .map_or("", |b| if b { "true" } else { "false" });
            writeln!(
                out,
                "{},{},{},{},{},{},{},{},{},{},{},{},{}",
                s.name,
                s.status.label(),
                csv_field(&r.id),
                r.quantity,
                r.unit,
                json_num(r.reference)?,
                json_num(r.model)?,
                json_num(r.abs_error)?,
                opt(r.percent_error)?,
                r.outside_data,
                opt(r.band_percent)?,
                flag,
                csv_field(&r.source),
            )
            .map_err(|_| ValidationError::Invalid)?;
        }
        for i in &s.info {
            writeln!(
                out,
                "{},{},{},{},{},,{},,,,,,informational (no reference)",
                s.name,
                s.status.label(),
                csv_field(&i.id),
                i.quantity,
                i.unit,
                json_num(i.value)?,
            )
            .map_err(|_| ValidationError::Invalid)?;
        }
        if s.rows.is_empty() && s.info.is_empty() {
            writeln!(
                out,
                "{},{},,,,,,,,,,,{}",
                s.name,
                s.status.label(),
                csv_field(&s.note)
            )
            .map_err(|_| ValidationError::Invalid)?;
        }
    }
    Ok(out)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::powertrain::tests::{hand_cfg, ref_cfg};

    // Every expected value below is written out by hand; the reference quad's motor is Kv 700, Rm 0.072, I0 0.5, 4S, 5 Ah, 0.02 ohm.

    fn near(a: f64, b: f64, tol: f64) -> bool {
        (a - b).abs() <= tol
    }

    fn row(id: &str, q: &str, reference: f64, model: f64) -> ErrorRow {
        new_row(id.to_string(), q, "1", reference, model, false, "test")
    }

    fn two_row_table() -> MotorTable {
        parse_motor_table(
            r#"{"schema_version":1,"motor_id":"tmotor_mn3508_kv700","source":{"url":"u","sha256":"s"},"rows":[
{"id":"a","supply_voltage_v":14.8,"prop":"T-MOTOR 12*4CF","prop_diameter_in":12.0,"prop_pitch_in":4.0,"throttle_percent":100.0,"current_a":16.1,"power_w":238.28,"thrust_g":1360.0,"rpm":8100.0,"efficiency_g_per_w":5.71,"temperature_c":null},
{"id":"b","supply_voltage_v":11.1,"prop":"T-MOTOR 12*4CF","prop_diameter_in":12.0,"prop_pitch_in":4.0,"throttle_percent":50.0,"current_a":2.3,"power_w":25.53,"thrust_g":280.0,"rpm":3800.0,"efficiency_g_per_w":10.97,"temperature_c":42.0}]}"#,
        )
        .unwrap()
    }

    fn batt() -> BatteryRef {
        BatteryRef {
            id: "ref_battery_generic_4s_5000".into(),
            cells: 4.0,
            capacity_ah: 5.0,
            v_max_cell: 4.2,
            v_nominal_cell: 3.7,
            v_cutoff_cell: 3.3,
            internal_resistance_ohm: 0.02,
        }
    }

    fn find<'a>(s: &'a Section, id: &str) -> &'a ErrorRow {
        s.rows.iter().find(|r| r.id == id).unwrap()
    }

    // ------------------------------------------------------------------ rows and summary

    #[test]
    fn test_val_percent_sign_floor_and_summary() {
        // reference 100 for every row; models 110, 90, 120, 100 -> +10, -10, +20, 0 percent
        let mut rows = vec![
            row("p", "q", 100.0, 110.0),
            row("n", "q", 100.0, 90.0),
            row("w", "q", 100.0, 120.0),
            row("z", "q", 100.0, 100.0),
        ];
        finish_rows(&mut rows, &Bands::none());
        let pct: Vec<f64> = rows.iter().map(|r| r.percent_error.unwrap()).collect();
        assert!(
            pct.iter()
                .zip([10.0, -10.0, 20.0, 0.0])
                .all(|(a, b)| near(*a, b, 1e-12))
        );
        let s = summarize(&rows);
        // mean |%| = (10 + 10 + 20 + 0) / 4 = 10; rms = sqrt((100 + 100 + 400) / 4) = sqrt(150) = 12.247449; max 20 at "w"
        assert_eq!((s.n, s.n_with_percent), (4, 4));
        assert!(
            near(s.mean_abs_percent, 10.0, 1e-12)
                && near(s.rms_percent, 150.0_f64.sqrt(), 1e-12)
                && near(s.max_abs_percent, 20.0, 1e-12)
        );
        assert_eq!(s.worst_row_id.as_deref(), Some("w"));
        // floor: 5 % of the largest |reference| (100) = 5; a reference of 4 or 0 gets no percent, 5 does (boundary inclusive)
        let mut rows = vec![
            row("big", "q", 100.0, 100.0),
            row("small", "q", 4.0, 5.0),
            row("zero", "q", 0.0, 1.0),
            row("edge", "q", 5.0, 6.0),
        ];
        finish_rows(&mut rows, &Bands::none());
        assert!(rows[1].percent_error.is_none() && rows[2].percent_error.is_none());
        assert!(near(rows[3].percent_error.unwrap(), 20.0, 1e-12));
        assert!(near(rows[1].abs_error, 1.0, 1e-15)); // the absolute error is always there
        // the floor is per quantity: another quantity with only small numbers keeps its own scale
        let mut rows = vec![row("a", "q", 100.0, 100.0), row("b", "r", 0.04, 0.05)];
        finish_rows(&mut rows, &Bands::none());
        assert!(near(rows[1].percent_error.unwrap(), 25.0, 1e-9));
    }

    #[test]
    fn test_val_summary_of_nothing_is_plain_zero() {
        let s = summarize(&[]);
        assert_eq!((s.n, s.n_with_percent, s.n_outside_band), (0, 0, 0));
        assert!(
            s.mean_abs_percent.to_bits() == 0.0_f64.to_bits()
                && s.rms_percent.to_bits() == 0.0_f64.to_bits()
                && s.worst_row_id.is_none()
        );
    }

    #[test]
    fn test_val_bands_flag_but_never_fail() {
        let bands = Bands::parse(r#"{"schema_version":1,"description":"d","provenance":{},"bands_percent":{"uiuc_ct_percent":15.0}}"#).unwrap();
        let mut rows = vec![
            row("in", "uiuc_ct", 100.0, 110.0),
            row("out", "uiuc_ct", 100.0, 120.0),
            row("other", "uiuc_cp", 100.0, 500.0),
        ];
        finish_rows(&mut rows, &bands);
        assert_eq!(
            (
                rows[0].outside_band,
                rows[1].outside_band,
                rows[2].outside_band
            ),
            (Some(false), Some(true), None)
        );
        assert_eq!(
            (rows[0].band_percent, rows[2].band_percent),
            (Some(15.0), None)
        );
        assert_eq!(summarize(&rows).n_outside_band, 1);
        // exactly at the band is inside
        let mut edge = vec![row("e", "uiuc_ct", 100.0, 115.0)];
        finish_rows(&mut edge, &bands);
        assert_eq!(edge[0].outside_band, Some(false));
    }

    #[test]
    fn test_val_negative_reference_tie_and_band_count() {
        // reference -100, model -90: the model is 10 above, so +10 % (the denominator is |reference|)
        let mut rows = vec![row("neg", "uiuc_ct", -100.0, -90.0)];
        finish_rows(&mut rows, &Bands::none());
        assert!(
            near(rows[0].percent_error.unwrap(), 10.0, 1e-12)
                && near(rows[0].abs_error, 10.0, 1e-12)
        );
        // a tie in |percent| (+10 and -10) names the first row as the worst
        let mut tie = vec![
            row("first", "uiuc_ct", 100.0, 110.0),
            row("second", "uiuc_ct", 100.0, 90.0),
        ];
        finish_rows(&mut tie, &Bands::none());
        assert_eq!(summarize(&tie).worst_row_id.as_deref(), Some("first"));
        // the outside-band count counts only rows that are outside: two inside, one outside -> 1
        let bands = Bands::parse(
            r#"{"schema_version":1,"provenance":{},"bands_percent":{"uiuc_ct_percent":15.0}}"#,
        )
        .unwrap();
        let mut b = vec![
            row("a", "uiuc_ct", 100.0, 101.0),
            row("b", "uiuc_ct", 100.0, 102.0),
            row("c", "uiuc_ct", 100.0, 130.0),
        ];
        finish_rows(&mut b, &bands);
        assert_eq!(summarize(&b).n_outside_band, 1);
    }

    #[test]
    fn test_val_numbers_are_written_with_12_significant_digits() {
        // 0.1 + 0.2 = 0.30000000000000004 and 1/3 are written without the noise digits
        assert_eq!(json_num(0.1 + 0.2).unwrap(), "0.3");
        assert_eq!(json_num(1.0 / 3.0).unwrap(), "0.333333333333");
        assert_eq!(json_num(25.529_999_999_999_998).unwrap(), "25.53");
        assert_eq!(json_num(-2.5).unwrap(), "-2.5");
        assert_eq!(json_num(0.0).unwrap(), "0");
    }

    // ------------------------------------------------------------------ motor section

    #[test]
    fn test_val_motor_rows_hand_values() {
        let s = motor_section(
            &two_row_table(),
            &ref_cfg(),
            "tmotor_mn3508_kv700",
            &Bands::none(),
        )
        .unwrap();
        assert_eq!((s.status, s.rows.len(), s.info.len()), (Status::Ok, 4, 6));
        // 14.8 V, 100 %: 8100 / 700 + 16.1 x 0.072 = 11.571429 + 1.1592 = 12.730629 V against 14.8 V: -2.069371 V = -13.98224 %
        let v = find(&s, "a:voltage");
        assert!(
            near(v.reference, 14.8, 1e-12)
                && near(v.model, 12.730_628_571_428_571, 1e-9)
                && near(v.abs_error, -2.069_371_428_571_429, 1e-9)
        );
        assert!(near(
            v.percent_error.unwrap(),
            -13.982_239_382_239_387,
            1e-9
        ));
        // 11.1 V, 50 %: 3800 / 700 + 2.3 x 0.072 = 5.428571 + 0.1656 = 5.594171 V against 5.55 V: +0.044171 V = +0.79588 %
        let w = find(&s, "b:voltage");
        assert!(
            near(w.reference, 5.55, 1e-12)
                && near(w.model, 5.594_171_428_571_428, 1e-9)
                && near(w.percent_error.unwrap(), 0.795_881_595_881_595, 1e-9)
        );
        // power self-consistency: 14.8 x 16.1 = 238.28 and 11.1 x 2.3 = 25.53 (the page prints exactly those)
        assert!(
            near(find(&s, "a:power").abs_error, 0.0, 1e-9)
                && near(find(&s, "b:power").abs_error, 0.0, 1e-9)
        );
    }

    #[test]
    fn test_val_motor_derived_values_hand_values() {
        // row a: Kt = 60 / (2 pi 700) = 0.0136419; Q = Kt (16.1 - 0.5) = 0.2128129 N m; n = 135 rev/s; D = 12 x 0.0254 = 0.3048 m;
        // T = 1.36 kg x 9.80665 = 13.337044 N; rho = 1.2250123; Ct = T / (rho n^2 D^4) = 0.0692137; Cp = 2 pi Q / (rho n^2 D^5) = 0.0227664
        let s = motor_section(
            &two_row_table(),
            &ref_cfg(),
            "tmotor_mn3508_kv700",
            &Bands::none(),
        )
        .unwrap();
        let get = |q: &str| {
            s.info
                .iter()
                .find(|i| i.id == "a" && i.quantity == q)
                .unwrap()
                .value
        };
        assert!(near(get("derived_torque"), 0.212_812_895_334_305_8, 1e-9));
        assert!(near(get("derived_ct"), 0.069_213_651_038_511_64, 1e-9));
        assert!(near(get("derived_cp"), 0.022_766_434_549_818_96, 1e-9));
        // informational values have no percent and no band
        assert!(
            s.rows
                .iter()
                .all(|r| r.quantity == "motor_voltage"
                    || r.quantity == "motor_power_self_consistency")
        );
    }

    #[test]
    fn test_val_motor_not_applicable_for_another_motor() {
        let s = motor_section(
            &two_row_table(),
            &ref_cfg(),
            "tmotor_mn3508_kv580",
            &Bands::none(),
        )
        .unwrap();
        assert_eq!(
            (s.status, s.rows.len(), s.info.len()),
            (Status::NotApplicable, 0, 0)
        );
        assert!(s.note.contains("kv580"));
    }

    // ------------------------------------------------------------------ UIUC section

    fn small_prop() -> PropData {
        // run 2000 rpm: the node (0.2, 0.08, 0.035) twice (exact duplicate), and a point beyond curve A's J end (0.7, 0.02, 0.021);
        // static rows at 3000 rpm and at 1000 rpm (below the first curve)
        parse_prop_data(
            r#"{"source":"UIUC test","wind_tunnel":{"x_2000.txt":[{"j":0.2,"ct":0.08,"cp":0.035},{"j":0.2,"ct":0.08,"cp":0.035},{"j":0.7,"ct":0.02,"cp":0.021}]},
"static":[{"rpm":3000.0,"ct":0.10,"cp":0.04},{"rpm":1000.0,"ct":0.11,"cp":0.05}]}"#,
        )
        .unwrap()
    }

    #[test]
    fn test_val_uiuc_rows_hand_values() {
        // hand_cfg: curve A (2000 rpm) (0, .10, .040) (.2, .08, .035) (.6, 0, .020); curve B (4000 rpm) (0, .12, .044) (.3, .09, .038) (.5, .04, .026)
        let s = uiuc_section(&small_prop(), &hand_cfg(), &Bands::none()).unwrap();
        // the duplicate row is dropped: 2 distinct run points + 2 static rows, 2 quantities each
        assert_eq!((s.status, s.rows.len()), (Status::Ok, 8));
        // the node: model equals the raw value exactly
        let node = find(&s, "x_2000.txt#0:ct");
        assert!(near(node.abs_error, 0.0, 1e-15) && !node.outside_data);
        // J 0.7 at 2000 rpm: beyond curve A's end (0.6) -> held (0.00, 0.020), flagged; raw Ct 0.02 -> error -0.02
        let far = find(&s, "x_2000.txt#1:ct");
        assert!(
            far.outside_data && near(far.model, 0.0, 1e-12) && near(far.abs_error, -0.02, 1e-12)
        );
        // static 3000 rpm, J 0: halfway between A (0.10) and B (0.12) = 0.11 against 0.10 -> +10 %; Cp (0.040 + 0.044) / 2 = 0.042 against 0.04
        let st = find(&s, "static@3000rpm:ct");
        assert!(
            near(st.model, 0.11, 1e-12)
                && near(st.percent_error.unwrap(), 10.0, 1e-9)
                && !st.outside_data
        );
        assert!(near(find(&s, "static@3000rpm:cp").model, 0.042, 1e-12));
        // static 1000 rpm is below the first curve: held at curve A's J = 0 value 0.10, flagged
        let low = find(&s, "static@1000rpm:ct");
        assert!(low.outside_data && near(low.model, 0.10, 1e-12));
    }

    #[test]
    fn test_val_uiuc_static_model_is_not_applicable_and_bad_run_name_is_refused() {
        let s = uiuc_section(&small_prop(), &ref_cfg(), &Bands::none()).unwrap();
        assert_eq!((s.status, s.rows.len()), (Status::NotApplicable, 0));
        let bad = parse_prop_data(r#"{"source":"s","wind_tunnel":{"noRpm.txt":[{"j":0.1,"ct":0.1,"cp":0.04}]},"static":[{"rpm":1000.0,"ct":0.1,"cp":0.04}]}"#).unwrap();
        assert_eq!(
            uiuc_section(&bad, &hand_cfg(), &Bands::none()).err(),
            Some(ValidationError::Parse)
        );
    }

    #[test]
    fn test_val_run_rpm_from_file_names() {
        assert_eq!(run_rpm("apce_12x6_0630od_3040.txt"), Some(3040.0));
        assert_eq!(run_rpm("a_b_5007.txt"), Some(5007.0));
        assert_eq!(run_rpm("a_b_5007.dat"), None);
        assert_eq!(run_rpm("nounderscore.txt"), None);
        assert_eq!(run_rpm("x_0.txt"), None);
        assert_eq!(run_rpm("x_abc.txt"), None);
    }

    // ------------------------------------------------------------------ battery sections

    #[test]
    fn test_val_battery_rows_hand_values() {
        let s = battery_section(&batt(), &ref_cfg(), &Bands::none()).unwrap();
        assert_eq!((s.status, s.rows.len()), (Status::Ok, 5));
        // full: 4 x 4.20 = 16.8 V for both; empty: 4 x 3.30 = 13.2 V for both
        assert!(
            near(find(&s, "ocv_full").abs_error, 0.0, 1e-12)
                && near(find(&s, "ocv_empty_vs_cutoff").abs_error, 0.0, 1e-12)
        );
        // mid: model table gives 3.82 V per cell at 50 % -> 15.28 V against the nominal 4 x 3.7 = 14.8 V: +3.2432 %
        let mid = find(&s, "ocv_mid_vs_nominal");
        assert!(
            near(mid.model, 15.28, 1e-12)
                && near(mid.percent_error.unwrap(), 3.243_243_243_243_234, 1e-9)
        );
        // sag: 10 A x 0.02 ohm = 0.2 V on both sides
        let sag = find(&s, "sag_at_10a");
        assert!(near(sag.reference, 0.2, 1e-15) && near(sag.model, 0.2, 1e-12));
        // coulomb counting: soc = 1 - charge_used / 5 Ah, the reference built from the drawn charge (>= 1 Ah)
        let soc = find(&s, "soc_after_charge");
        assert!(soc.reference <= 0.8 && soc.reference > 0.79 && near(soc.abs_error, 0.0, 1e-12));
    }

    #[test]
    fn test_val_battery_cited_resistance_difference_shows_up() {
        // a record that cites 0.03 ohm against the config's 0.02: sag reference 0.3 V, model 0.2 V -> -33.33 %
        let mut b = batt();
        b.internal_resistance_ohm = 0.03;
        let s = battery_section(&b, &ref_cfg(), &Bands::none()).unwrap();
        assert!(near(
            find(&s, "sag_at_10a").percent_error.unwrap(),
            -100.0 / 3.0,
            1e-9
        ));
    }

    const CSV: &str = "soc_fraction,voltage_v,current_a,source\n0.5,15.28,0,SYNTHETIC test line\n0.5,15.0,10,SYNTHETIC with, comma\n";

    #[test]
    fn test_val_curve_import_hand_values_and_refusals() {
        let pts = parse_curve_csv(CSV).unwrap();
        assert_eq!(pts.len(), 2);
        assert_eq!(pts[1].source, "SYNTHETIC with, comma"); // the source column keeps its commas
        let s = curve_section(Some(&pts), &ref_cfg(), &Bands::none()).unwrap();
        // soc 0.5, open circuit: model 15.28 V = measured; at 10 A: 15.28 - 0.2 = 15.08 V against 15.0 V: +0.5333 %
        assert!(near(s.rows[0].abs_error, 0.0, 1e-12));
        assert!(
            near(s.rows[1].model, 15.08, 1e-12)
                && near(s.rows[1].percent_error.unwrap(), 0.08 / 15.0 * 100.0, 1e-9)
        );
        let none = curve_section(None, &ref_cfg(), &Bands::none()).unwrap();
        assert_eq!((none.status, none.rows.len()), (Status::NoPublishedData, 0));
        let bad_header = "soc,voltage_v,current_a,source\n0.5,15,0,x\n";
        assert_eq!(parse_curve_csv(bad_header), Err(ValidationError::Parse));
        let cases = [
            ("0.5,15", ValidationError::Parse),
            ("abc,15,0,x", ValidationError::Parse),
            ("1.5,15,0,x", ValidationError::Invalid),
            ("-0.1,15,0,x", ValidationError::Invalid),
            ("0.5,0,0,x", ValidationError::Invalid),
            ("0.5,15,-1,x", ValidationError::Invalid),
            ("0.5,NaN,0,x", ValidationError::Invalid),
            ("0.5,15,inf,x", ValidationError::Invalid),
            ("0.5,15,0,", ValidationError::Invalid),
        ];
        for (line, err) in cases {
            let text = format!("soc_fraction,voltage_v,current_a,source\n{line}\n");
            assert_eq!(parse_curve_csv(&text), Err(err), "{line}");
        }
        assert_eq!(
            parse_curve_csv("soc_fraction,voltage_v,current_a,source\n"),
            Err(ValidationError::Invalid)
        );
    }

    // ------------------------------------------------------------------ loaders

    #[test]
    fn test_val_loader_rules() {
        let ok =
            r#"{"schema_version":1,"motor_id":"m","source":{"url":"u","sha256":"s"},"rows":[ROW]}"#;
        let r = r#"{"id":"a","supply_voltage_v":14.8,"prop":"p","prop_diameter_in":12.0,"prop_pitch_in":4.0,"throttle_percent":100.0,"current_a":16.1,"power_w":238.28,"thrust_g":1360.0,"rpm":8100.0,"efficiency_g_per_w":5.71,"temperature_c":null}"#;
        assert!(parse_motor_table(&ok.replace("ROW", r)).is_ok());
        let expect =
            |text: String, e: ValidationError| assert_eq!(parse_motor_table(&text).err(), Some(e));
        expect(
            ok.replace(
                "ROW",
                &r.replace("\"id\":\"a\",", "\"id\":\"a\",\"extra\":1,"),
            ),
            ValidationError::Parse,
        );
        expect(
            ok.replace("ROW", &r.replace("\"rpm\":8100.0,", "")),
            ValidationError::Parse,
        );
        expect(
            ok.replace("ROW", &r.replace("8100.0", "-8100.0")),
            ValidationError::Invalid,
        );
        expect(
            ok.replace("ROW", &r.replace("16.1", "0.0")),
            ValidationError::Invalid,
        );
        expect(
            ok.replace("ROW", &format!("{r},{r}")),
            ValidationError::Invalid,
        ); // duplicate id
        expect(ok.replace("ROW", ""), ValidationError::Invalid); // no rows
        expect(
            ok.replace("\"schema_version\":1", "\"schema_version\":2")
                .replace("ROW", r),
            ValidationError::Invalid,
        );
        expect("{".into(), ValidationError::Parse);
        let bands = |inner: &str| {
            Bands::parse(&format!(
                r#"{{"schema_version":1,"provenance":{{}},"bands_percent":{{{inner}}}}}"#
            ))
        };
        assert!(bands(r#""uiuc_ct_percent":10.0"#).is_ok());
        assert_eq!(
            bands(r#""unknown_percent":10.0"#),
            Err(ValidationError::Invalid)
        );
        assert_eq!(
            bands(r#""uiuc_ct_percent":0.0"#),
            Err(ValidationError::Invalid)
        );
        assert_eq!(
            Bands::parse(r#"{"schema_version":2,"provenance":{},"bands_percent":{}}"#),
            Err(ValidationError::Invalid)
        );
        assert_eq!(
            Bands::parse(r#"{"schema_version":1,"provenance":{},"bands_percent":{},"x":1}"#),
            Err(ValidationError::Parse)
        );
        assert_eq!(
            parse_prop_data(r#"{"source":"s","wind_tunnel":{},"static":[]}"#).err(),
            Some(ValidationError::Invalid)
        );
        assert_eq!(parse_prop_data("[]").err(), Some(ValidationError::Parse));
    }

    #[test]
    fn test_val_battery_record_loader() {
        let rec = |cells: &str, nom: &str| {
            format!(
                r#"{{"id":"b","cells":{{"value":{cells}}},"capacity_ah":{{"value":5.0}},"voltage_per_cell_max_v":{{"value":4.2}},"voltage_per_cell_nominal_v":{{"value":{nom}}},"voltage_per_cell_cutoff_v":{{"value":3.3}},"internal_resistance_ohm":{{"value":0.02}}}}"#
            )
        };
        assert_eq!(
            parse_battery_record(&rec("4", "3.7")).unwrap(),
            batt_with_id("b")
        );
        assert_eq!(
            parse_battery_record(&rec("0", "3.7")),
            Err(ValidationError::Invalid)
        );
        assert_eq!(
            parse_battery_record(&rec("4", "4.3")),
            Err(ValidationError::Invalid)
        ); // nominal above the maximum
        assert_eq!(
            parse_battery_record(&rec("4", "3.2")),
            Err(ValidationError::Invalid)
        ); // nominal below the cutoff
        assert_eq!(
            parse_battery_record(&rec("4", "3.7").replace("\"capacity_ah\":{\"value\":5.0},", "")),
            Err(ValidationError::Parse)
        );
        assert_eq!(parse_battery_record("nope"), Err(ValidationError::Parse));
    }

    fn batt_with_id(id: &str) -> BatteryRef {
        BatteryRef {
            id: id.into(),
            ..batt()
        }
    }

    // ------------------------------------------------------------------ writers

    fn small_report() -> Report {
        let rows = vec![row("a\"b", "q", 100.0, 110.0), row("c", "q", 100.0, 90.0)];
        Report {
            schema_version: 1,
            build: "t".into(),
            core_interface_version: 14,
            reference_files: vec![RefFile {
                label: "f".into(),
                origin: "o".into(),
            }],
            sections: vec![
                make_section(
                    "sec",
                    Status::Ok,
                    "note, with comma",
                    rows,
                    vec![info("a\"b", "derived_x", 2.5, "N")],
                    &Bands::none(),
                ),
                empty_section("none", Status::NoPublishedData, "nothing"),
            ],
        }
    }

    #[test]
    fn test_val_json_is_valid_and_has_the_hand_numbers() {
        let text = to_json(&small_report()).unwrap();
        let v: serde_json::Value = serde_json::from_str(&text).unwrap();
        assert_eq!(v["schema_version"], 1);
        assert_eq!(v["core_interface_version"], 14);
        let sec = &v["sections"][0];
        assert_eq!(sec["status"], "OK");
        assert_eq!(sec["rows"][0]["id"], "a\"b"); // escaping survives a round trip
        assert_eq!(sec["rows"][0]["percent_error"], 10.0);
        assert_eq!(sec["rows"][1]["percent_error"], -10.0);
        assert_eq!(sec["rows"][0]["band_percent"], serde_json::Value::Null);
        assert_eq!(sec["summary"]["mean_abs_percent"], 10.0);
        assert_eq!(sec["info"][0]["value"], 2.5);
        assert_eq!(v["sections"][1]["status"], "NO_PUBLISHED_DATA");
        assert_eq!(v["sections"][1]["summary"]["rms_percent"], 0.0);
    }

    #[test]
    fn test_val_csv_exact_lines() {
        let text = to_csv(&small_report()).unwrap();
        let lines: Vec<&str> = text.lines().collect();
        assert_eq!(
            lines[0],
            "section,status,id,quantity,unit,reference,model,abs_error,percent_error,outside_data,band_percent,outside_band,source"
        );
        assert_eq!(lines[1], "sec,OK,\"a\"\"b\",q,1,100,110,10,10,false,,,test");
        assert_eq!(lines[2], "sec,OK,c,q,1,100,90,-10,-10,false,,,test");
        assert_eq!(
            lines[3],
            "sec,OK,\"a\"\"b\",derived_x,N,,2.5,,,,,,informational (no reference)"
        );
        assert_eq!(lines[4], "none,NO_PUBLISHED_DATA,,,,,,,,,,,nothing");
        assert_eq!(lines.len(), 5);
    }

    #[test]
    fn test_val_writers_refuse_non_finite_and_are_deterministic() {
        let mut r = small_report();
        assert_eq!(to_json(&r).unwrap(), to_json(&r).unwrap());
        assert_eq!(to_csv(&r).unwrap(), to_csv(&r).unwrap());
        for bad in [f64::NAN, f64::INFINITY] {
            if let Some(sec) = r.sections.get_mut(0) {
                sec.rows[0].model = bad;
            }
            assert_eq!(to_json(&r).err(), Some(ValidationError::NonFinite));
            assert_eq!(to_csv(&r).err(), Some(ValidationError::NonFinite));
        }
    }

    // ------------------------------------------------------------------ real files

    fn root() -> std::path::PathBuf {
        std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("..")
    }

    fn read(rel: &str) -> String {
        std::fs::read_to_string(root().join(rel)).unwrap()
    }

    #[test]
    fn test_val_committed_reference_files_load_and_the_reference_quad_report_has_the_hand_row() {
        let table =
            parse_motor_table(&read("parts_db/validation/tmotor_mn3508_kv700_table.json")).unwrap();
        assert_eq!(table.rows.len(), 20);
        let bands = Bands::parse(&read("parts_db/validation/acceptance_bands.json")).unwrap();
        let prop = parse_prop_data(&read("offline_pipeline/props/apce_12x6.json")).unwrap();
        let battery = parse_battery_record(&read(
            "parts_db/parts/battery/ref_battery_generic_4s_5000.json",
        ))
        .unwrap();
        assert!(near(battery.capacity_ah, 5.0, 0.0) && near(battery.cells, 4.0, 0.0));
        let cfg = crate::powertrain::load_powertrain_config(
            root()
                .join("tools/parts/builds/reference_quad/powertrain_config.json")
                .to_str()
                .unwrap(),
        )
        .unwrap();
        let inputs = ReportInputs {
            build: "reference_quad",
            motor_id: "tmotor_mn3508_kv700",
            cfg: &cfg,
            motor_table: &table,
            prop: &prop,
            battery: &battery,
            curve: None,
            bands: &bands,
            reference_files: Vec::new(),
        };
        let report = build_report(&inputs).unwrap();
        assert_eq!(report.sections.len(), 4);
        let motor = &report.sections[0];
        assert_eq!(
            (motor.status, motor.rows.len(), motor.info.len()),
            (Status::Ok, 40, 60)
        );
        // the row worked out by hand above (14.8 V, 12*4CF, 100 %): -13.98224 %, the largest voltage residual of the table
        let hand = find(motor, "kv700_14.8v_12x4cf_100pct:voltage");
        assert!(near(
            hand.percent_error.unwrap(),
            -13.982_239_382_239_387,
            1e-9
        ));
        assert_eq!(
            motor.summary.worst_row_id.as_deref(),
            Some("kv700_14.8v_12x4cf_100pct:voltage")
        );
        let uiuc = &report.sections[1];
        assert_eq!(uiuc.status, Status::Ok);
        assert!(
            uiuc.rows
                .iter()
                .all(|r| r.model.is_finite() && r.reference.is_finite())
        );
        // static rows below the first curve (3040 rpm) are held values and say so
        assert!(find(uiuc, "static@1046.667rpm:ct").outside_data);
        assert_eq!(report.sections[3].status, Status::NoPublishedData);
        assert!(to_json(&report).is_ok() && to_csv(&report).is_ok());
    }
}
