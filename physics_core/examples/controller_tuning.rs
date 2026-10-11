// Tier: F
//! Flies the controller scenarios on a golden build and prints / writes the measured numbers.
//! Usage: cargo run --release --example controller_tuning -- <build name> [out.json]
use physics_core::controller_sim::{Report, golden_dir, load_build, run_scenarios};

fn json(r: &Report) -> String {
    let (rise, over, settle, fin) = r.metrics.map_or((None, 0.0, 0.0, 0.0), |m| {
        (
            m.rise_time_s,
            m.overshoot_percent,
            m.settling_time_s,
            m.final_error_fraction,
        )
    });
    format!(
        "    {{\"name\": \"{}\", \"rise_time_s\": {}, \"overshoot_percent\": {over:.3}, \"settling_time_s\": {settle:.3}, \"final_error_fraction\": {fin:.6}, \
\"has_step_metrics\": {}, \"max_tilt_deg\": {:.2}, \"max_speed_mps\": {:.3}, \"altitude_change_m\": {:.3}, \"min_altitude_change_m\": {:.3}, \"max_drift_m\": {:.3}, \
\"steps\": {}, \"collective_limited_steps\": {}, \"axis_saturated_steps\": {}, \"tilt_limited_steps\": {}, \"prop_outside_table_fraction\": {:.4}, \"all_finite\": {}}}",
        r.name,
        rise.map_or("null".to_string(), |v| format!("{v:.3}")),
        r.metrics.is_some(),
        r.max_tilt_deg,
        r.max_speed_mps,
        r.altitude_change_m,
        r.min_altitude_change_m,
        r.max_drift_m,
        r.steps,
        r.collective_limited_steps,
        r.axis_saturated_steps,
        r.tilt_limited_steps,
        r.prop_outside_fraction,
        r.all_finite
    )
}

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let name = args.get(1).map_or("reference_quad", String::as_str);
    let build = load_build(&golden_dir(name)).expect("load build");
    let reports = run_scenarios(&build).expect("scenarios");
    for r in &reports {
        println!("{r:?}");
    }
    if let Some(path) = args.get(2) {
        let body: Vec<String> = reports.iter().map(json).collect();
        let text = format!(
            "{{\n  \"build\": \"{name}\",\n  \"generated_by\": \"physics_core/examples/controller_tuning.rs\",\n  \"dt_s\": 0.002,\n  \"preroll_s\": 8.0,\n  \"note\": \"measured in closed loop on the real core; all controller limits are ASSUMED (TODO(HUMAN-VERIFY))\",\n  \"scenarios\": [\n{}\n  ]\n}}\n",
            body.join(",\n")
        );
        std::fs::write(path, text).expect("write json");
    }
}
