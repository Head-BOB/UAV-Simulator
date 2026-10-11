// Tier: F
use serde::Deserialize;

/// Represents the drone's current physical condition.
///
/// #[repr(C)] guarantees this struct has the exact same memory layout on
/// both sides of the FFI boundary.
///
/// COORDINATE FRAME (see ADR-001, Section 1.3 of the Standards Framework):
/// position is expressed in a local North-East-Down (NED) frame, in meters,
/// relative to a fixed WGS84 geodetic origin defined once per simulation
/// scenario. Do not reinterpret this as UE5 world-space — conversion
/// happens ONLY inside the wrapper class described in Section 1.4.
#[derive(Copy, Clone)]
#[repr(C)]
pub struct DroneState {
    /// North-East-Down position relative to the scenario's WGS84 origin.
    ///
    /// # Units
    /// Meters. f64: see ADR-001 — f32 loses sub-meter precision at realistic mission ranges.
    pub position: [f64; 3],

    /// Local-frame linear velocity.
    ///
    /// # Units
    /// Meters per second. f32 is sufficient: magnitude never grows large enough to lose useful precision.
    pub velocity: [f32; 3],

    /// Rotation as a unit quaternion, stored in order (x, y, z, w).
    ///
    /// # Units
    /// Unitless quaternion. f32 is sufficient: components are always within [-1.0, 1.0].
    pub orientation: [f32; 4],

    /// Angular velocity in the body frame.
    ///
    /// # Units
    /// Radians per second. f32 is sufficient for the same reason as velocity.
    pub angular_velocity: [f32; 3],
}

/// Represents what the pilot/controller is commanding.
#[derive(Copy, Clone)]
#[repr(C)]
pub struct ControlInputs {
    /// Commanded throttle.
    ///
    /// # Units
    /// Normalized range 0.0 (none) to 1.0 (full).
    pub throttle: f32,

    /// Commanded roll input.
    ///
    /// # Units
    /// Normalized range -1.0 to 1.0.
    pub roll: f32,

    /// Commanded pitch input.
    ///
    /// # Units
    /// Normalized range -1.0 to 1.0.
    pub pitch: f32,

    /// Commanded yaw input.
    ///
    /// # Units
    /// Normalized range -1.0 to 1.0.
    pub yaw: f32,
}

/// Ambient wind for one `ffi_step_physics` call (interface v11).
///
/// COORDINATE FRAME: NED world frame (ADR-001), the direction the air moves TOWARD.
/// All zeros is still air. The step computes the air-relative velocity
/// `v_air = v_ground - wind_ned_mps` for the aerodynamic drag only.
#[derive(Copy, Clone)]
#[repr(C)]
pub struct WindInput {
    /// Wind velocity, north / east / down components.
    ///
    /// # Units
    /// Metres per second, f64. Each component must be finite with |value| <= `MAX_WIND_COMPONENT_MPS`.
    pub wind_ned_mps: [f64; 3],
}

impl WindInput {
    /// Still air.
    #[must_use]
    pub const fn still() -> Self {
        Self {
            wind_ned_mps: [0.0; 3],
        }
    }
}

impl Default for WindInput {
    fn default() -> Self {
        Self::still()
    }
}

/// Largest accepted |component| of `WindInput::wind_ned_mps`, m/s.
/// TODO(HUMAN-VERIFY: demo value) Chosen by the project owner as a sanity bound for the demo
/// (30 m/s is about gale force); it is not a validated aerodynamic limit of the drag model.
pub const MAX_WIND_COMPONENT_MPS: f64 = 30.0;

/// Atmosphere for one `ffi_step_physics` call (interface v12): where the NED origin sits above sea
/// level and how much warmer or colder than the ISA standard day the air is.
///
/// All zeros is the ISA standard day at sea level, which reproduces the v11 behaviour bit for bit.
/// Air density at the vehicle comes from the ISA pressure at the total height (`site_altitude_m`
/// plus the vehicle altitude above the origin) and the ISA temperature at that height PLUS
/// `temp_offset_c` (`aero::air_density`). Humidity is not modelled.
#[derive(Copy, Clone)]
#[repr(C)]
pub struct EnvironmentInput {
    /// Height of the NED origin above sea level.
    ///
    /// # Units
    /// Metres, f64, finite, 0..=`MAX_SITE_ALTITUDE_M`.
    pub site_altitude_m: f64,

    /// Air temperature minus the ISA standard-day temperature at the same height.
    ///
    /// # Units
    /// Kelvin (equal to degrees C for a difference), f64, finite, |value| <= `MAX_TEMP_OFFSET_C`.
    pub temp_offset_c: f64,
}

impl EnvironmentInput {
    /// ISA standard day at sea level (both fields zero).
    #[must_use]
    pub const fn standard() -> Self {
        Self {
            site_altitude_m: 0.0,
            temp_offset_c: 0.0,
        }
    }
}

impl Default for EnvironmentInput {
    fn default() -> Self {
        Self::standard()
    }
}

/// Largest accepted `EnvironmentInput::site_altitude_m`, m.
/// TODO(HUMAN-VERIFY: demo value) The tallest demo site is a mountain preset; 5000 m keeps the
/// total height (site plus flight) inside the ISA troposphere model (11000 m) with room to fly.
pub const MAX_SITE_ALTITUDE_M: f64 = 5000.0;

/// Largest accepted |`EnvironmentInput::temp_offset_c`|, K.
/// TODO(HUMAN-VERIFY: demo value) A sanity bound for the demo (about -40..+40 K around the standard
/// day covers polar to desert air); it is not a validated limit of any model here.
pub const MAX_TEMP_OFFSET_C: f64 = 40.0;

/// State container for the thermal and electrical simulation.
#[derive(Copy, Clone)]
#[repr(C)]
pub struct ThermalState {
    /// Internal winding temperatures for each motor.
    ///
    /// # Units
    /// Degrees Celsius.
    pub motor_temp_c: [f64; 4],
}

impl ThermalState {
    pub const fn new() -> Self {
        Self {
            motor_temp_c: [20.0, 20.0, 20.0, 20.0],
        }
    }
}

impl Default for ThermalState {
    fn default() -> Self {
        Self::new()
    }
}

/// Caller-owned latched fault flag for `ffi_step_physics` (WP-4.4).
///
/// The caller allocates it, zero-initializes it, and passes it to every step.
/// Rust sets `physics_fault` to 1 when a step produces a NaN or Inf state and
/// NEVER clears it. Only the caller clears it, on purpose (for example at the
/// start of a new run). Rust holds no copy, so two vehicles or two tests do
/// not share fault state.
///
/// Data produced while the latch is set is invalid. It must not be used as
/// validation or pass/fail evidence.
#[derive(Copy, Clone)]
#[repr(C)]
pub struct FaultLatch {
    /// 0 = no fault latched, 1 = a NaN/Inf divergence occurred since the caller last cleared it.
    pub physics_fault: i32,
}

impl FaultLatch {
    pub const fn new() -> Self {
        Self { physics_fault: 0 }
    }
}

impl Default for FaultLatch {
    fn default() -> Self {
        Self::new()
    }
}

/// Standardized return payload for any trained surrogate model query.
#[derive(Copy, Clone)]
#[repr(C)]
pub struct SurrogateQueryResult {
    /// The primary output of the surrogate model (e.g., drag vector or scalar safety margin).
    ///
    /// # Units
    /// Context-dependent based on the specific model queried.
    pub predicted_values: [f32; 3],

    /// Statistical confidence metric from the Gaussian Process Regression.
    ///
    /// # Units
    /// Context-dependent variance.
    pub uncertainty: f32,

    /// Evaluates if the queried condition fell within the model's training data envelope.
    ///
    /// # Units
    /// Unitless boolean flag (1 for true, 0 for false).
    pub in_validated_envelope: i32,
}

/// Physical properties and layout of the specific airframe being simulated.
#[derive(Copy, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
#[repr(C)]
pub struct VehicleConfig {
    /// Total mass of the vehicle.
    ///
    /// # Units
    /// Kilograms.
    pub mass_kg: f64,

    /// Moment of inertia about the center of mass (Ixx, Iyy, Izz).
    ///
    /// # Units
    /// Kilogram-square meters.
    pub inertia_kgm2: [f64; 3],

    /// Positions of the 4 motors in the body frame.
    ///
    /// # Units
    /// Meters.
    pub motor_pos_m: [[f64; 3]; 4],

    /// Spin direction of each motor (+1.0 or -1.0), in the same index order as `motor_pos_m`.
    /// +1.0 means the reaction torque on the airframe is about body +Z (nose right), i.e. the
    /// propeller turns CCW seen from above.
    ///
    /// # Units
    /// Unitless multiplier.
    pub motor_spin: [f64; 4],

    /// Diameter of the propellers.
    ///
    /// # Units
    /// Meters.
    pub prop_diameter_m: f64,

    /// Propeller thrust coefficient.
    ///
    /// # Units
    /// Unitless.
    pub thrust_coeff: f64,

    /// Propeller yaw-torque coefficient: reaction torque per unit of thrust (`kappa`).
    /// Yaw torque on the airframe is `sum(motor_spin * torque_coeff * thrust_n)`.
    ///
    /// # Units
    /// Newton-metres per Newton of thrust (N*m/N, equivalent to metres).
    pub torque_coeff: f64,

    /// Maximum rotational speed of the motors.
    ///
    /// # Units
    /// Revolutions per minute.
    pub max_rpm: f64,

    /// Throttle level required to maintain steady hover.
    ///
    /// Derived at load from mass, C_T, propeller diameter, max RPM and ISA
    /// sea-level density (`aero::hover_throttle`); never read from JSON.
    ///
    /// # Units
    /// Normalized range 0.0 to 1.0.
    #[serde(skip_deserializing)]
    pub hover_throttle: f64,
}

/// Telemetry data exposed exclusively for UE5 On-Screen Display (OSD) and debug visualization.
///
/// Must be synced every physics tick.
#[derive(Copy, Clone)]
#[repr(C)]
pub struct DebugTelemetry {
    /// Net thrust vector across all motors in world space.
    ///
    /// # Units
    /// Newtons.
    pub net_thrust: [f32; 3],

    /// Aerodynamic drag force vector in world space.
    ///
    /// # Units
    /// Newtons.
    pub aero_drag: [f32; 3],

    /// Gravity vector applied to the drone.
    ///
    /// # Units
    /// Newtons.
    pub gravity: [f32; 3],

    /// Resulting net force vector.
    ///
    /// # Units
    /// Newtons.
    pub net_force: [f32; 3],

    /// Individual motor thrust outputs.
    ///
    /// # Units
    /// Newtons.
    pub motor_thrusts: [f32; 4],

    /// Individual motor RPMs.
    ///
    /// # Units
    /// Revolutions per minute.
    pub motor_rpms: [f32; 4],

    /// Temperature of each motor winding.
    ///
    /// # Units
    /// Degrees Celsius.
    pub motor_temperatures_c: [f32; 4],

    /// Structural failure margin from the FEA surrogate.
    ///
    /// # Units
    /// Unitless multiplier. Values strictly less than 1.0 indicate structural failure.
    pub structural_safety_margin: f32,

    /// Aggregated envelope validation flag for all surrogate queries in the current tick.
    ///
    /// # Units
    /// Unitless boolean flag (1 if all queries are valid, 0 if any query extrapolated).
    pub is_validated_envelope: i32,

    /// Flag indicating if an un-trained stub model is currently driving the physics.
    ///
    /// # Units
    /// Unitless boolean flag (1 if a surrogate handle passed to this step came
    /// from a model whose `kind` is not `trained`, 0 otherwise).
    pub stub_loaded: i32,

    /// Source of aerodynamic data. 0 = none (step rejected), 1 = analytic model, no surrogate
    /// handle was passed, 2 = surrogate, 3 = a surrogate handle was passed but its query FAILED
    /// and the analytic model was used instead (the two are never reported as the same value).
    pub aero_source: i32,

    /// Source of structural data. 0 = none (no handle passed, or step rejected), 2 = surrogate,
    /// 3 = a surrogate handle was passed but its query FAILED (margin is NaN). There is no
    /// analytic structural fallback, so 1 is not used.
    pub fea_source: i32,

    /// Copy of the caller's `FaultLatch`: 1 from the faulting step until the caller clears it.
    /// Data produced while this is 1 is invalid.
    pub physics_fault: i32,

    /// Count of fixed-timestep execution cycles dropped to prevent a "spiral of death".
    pub dropped_time_events: i32,

    /// Indicates physics constants lacking validated sources are in use.
    pub uncalibrated_flags: i32,

    /// 1 only when every thermal constant has a sourced, verified value. 0 means the thermal
    /// model is UNVALIDATED: `motor_temperatures_c` and `thermal_over_temp` are arithmetic on
    /// ASSUMED constants (item B-4) and must not be used as evidence. Always 0 until B-4 closes.
    pub thermal_valid: i32,

    /// 1 if any motor winding temperature exceeds `thermal::MOTOR_TEMP_LIMIT_C` after this step
    /// (limit is a placeholder, TODO(HUMAN-VERIFY)); 0 otherwise. Only meaningful when
    /// `thermal_valid` is 1.
    pub thermal_over_temp: i32,
}

impl DebugTelemetry {
    pub const fn new() -> Self {
        Self {
            net_thrust: [0.0; 3],
            aero_drag: [0.0; 3],
            gravity: [0.0; 3],
            net_force: [0.0; 3],
            motor_thrusts: [0.0; 4],
            motor_rpms: [0.0; 4],
            motor_temperatures_c: [20.0; 4],
            // WP-7 D.5: no model means no number. NaN, never a fake safe margin (HZ-02).
            structural_safety_margin: f32::NAN,
            is_validated_envelope: 1,
            stub_loaded: 0,
            aero_source: 0,
            fea_source: 0,
            physics_fault: 0,
            dropped_time_events: 0,
            uncalibrated_flags: 0,
            thermal_valid: 0,
            thermal_over_temp: 0,
        }
    }
}

impl Default for DebugTelemetry {
    fn default() -> Self {
        Self::new()
    }
}

/// Number of points the battery open-circuit-voltage table can hold (fixed size: no heap in the FFI).
pub const MAX_OCV_POINTS: usize = 16;

/// Number of propeller speed curves the Ct/Cp table can hold (fixed size: no heap in the FFI).
pub const MAX_PROP_CURVES: usize = 8;

/// Number of (J, Ct, Cp) points per propeller speed curve.
pub const MAX_PROP_POINTS: usize = 48;

/// Motor, ESC, battery and thermal parameters of the powertrain (interface v13).
///
/// Loaded from `powertrain_config.json` (written by `tools/parts/build_vehicle.py`) with
/// `ffi_load_powertrain_config`. Separate from `VehicleConfig` on purpose: `vehicle_config.json` is
/// hashed into every surrogate model, so it must not change when the powertrain model does.
/// Every value is SI; the unit is in the field name. See `powertrain.rs` for the model.
#[derive(Copy, Clone)]
#[repr(C)]
pub struct PowertrainConfig {
    /// Motor velocity constant, rpm per volt (no-load).
    pub motor_kv_rpm_per_v: f64,
    /// Motor phase-to-phase winding resistance, ohm.
    pub motor_resistance_ohm: f64,
    /// Motor no-load current, A (iron + friction loss = this times the back-EMF).
    pub motor_no_load_current_a: f64,
    /// Motor current rating, A (flag only; the model does not limit current).
    pub motor_max_current_a: f64,
    /// Motor winding temperature limit, deg C (flag only).
    pub motor_winding_temp_limit_c: f64,
    /// Propeller diameter, m. Must equal `VehicleConfig::prop_diameter_m`.
    pub prop_diameter_m: f64,
    /// Static thrust coefficient (unitless). Must equal `VehicleConfig::thrust_coeff`.
    pub prop_ct: f64,
    /// Static power coefficient (unitless).
    pub prop_cp: f64,
    /// ESC efficiency, unitless in (0, 1].
    pub esc_efficiency: f64,
    /// ESC continuous current rating, A (flag only).
    pub esc_max_current_a: f64,
    /// ESC temperature limit, deg C (flag only).
    pub esc_temp_limit_c: f64,
    /// Battery capacity, Ah.
    pub battery_capacity_ah: f64,
    /// Battery pack internal resistance, ohm.
    pub battery_internal_resistance_ohm: f64,
    /// Battery cutoff voltage per cell, V.
    pub battery_cutoff_v_per_cell: f64,
    /// Battery continuous current rating (C rating x capacity), A (flag only).
    pub battery_max_continuous_a: f64,
    /// Battery temperature limit, deg C (flag only).
    pub battery_temp_limit_c: f64,
    /// Thermal resistance motor winding to case, K/W.
    pub motor_r_winding_case_k_per_w: f64,
    /// Thermal resistance motor case to ambient in still air, K/W.
    pub motor_r_case_ambient_k_per_w: f64,
    /// Heat capacity of the motor winding, J/K.
    pub motor_c_winding_j_per_k: f64,
    /// Heat capacity of the motor case (stator, bell, magnets), J/K.
    pub motor_c_case_j_per_k: f64,
    /// Thermal resistance ESC to ambient in still air, K/W.
    pub esc_r_ambient_k_per_w: f64,
    /// Heat capacity of the ESC, J/K.
    pub esc_c_j_per_k: f64,
    /// Thermal resistance battery to ambient in still air, K/W.
    pub battery_r_ambient_k_per_w: f64,
    /// Heat capacity of the battery, J/K.
    pub battery_c_j_per_k: f64,
    /// Air speed at which forced convection doubles the cooling (`R_eff = R / (1 + (v/v_ref)^0.8)`), m/s.
    pub cooling_ref_airspeed_mps: f64,
    /// Open-circuit-voltage table: state of charge, strictly increasing from 0 to 1 (first `ocv_points` entries).
    pub ocv_soc: [f64; MAX_OCV_POINTS],
    /// Open-circuit voltage per cell at `ocv_soc`, V, non-decreasing.
    pub ocv_v_per_cell: [f64; MAX_OCV_POINTS],
    /// Number of cells in series (1..=14).
    pub battery_cells: i32,
    /// Number of used entries in the OCV table (2..=16).
    pub ocv_points: i32,
    /// 1 only if every thermal and electrical input is MEASURED or CITED. The builder writes 0 while any input is ASSUMED.
    pub parameters_verified: i32,
    /// Reserved, must be 0.
    pub reserved: i32,
    /// Speed of each propeller curve, rpm, strictly increasing (first `prop_curve_count` entries). Used when `prop_model` is 1.
    pub prop_curve_rpm: [f64; MAX_PROP_CURVES],
    /// Advance ratio `J = V_axial / (n D)` of each curve point, strictly increasing from exactly 0.
    pub prop_j: [[f64; MAX_PROP_POINTS]; MAX_PROP_CURVES],
    /// Thrust coefficient `Ct` at each curve point (may be negative at high J).
    pub prop_ct_table: [[f64; MAX_PROP_POINTS]; MAX_PROP_CURVES],
    /// Power coefficient `Cp` at each curve point, > 0.
    pub prop_cp_table: [[f64; MAX_PROP_POINTS]; MAX_PROP_CURVES],
    /// Number of used points of each curve (2..=48).
    pub prop_curve_points: [i32; MAX_PROP_CURVES],
    /// Number of used curves (1..=8 when `prop_model` is 1, else 0).
    pub prop_curve_count: i32,
    /// 0: constant `prop_ct`/`prop_cp` (the static fit). 1: the Ct/Cp(J, rpm) table.
    pub prop_model: i32,
}

/// Evolving powertrain state, owned by the caller and updated by `ffi_step_powertrain` (interface v13).
#[derive(Copy, Clone)]
#[repr(C)]
pub struct PowertrainState {
    /// Battery state of charge, unitless in [0, 1].
    pub soc: f64,
    /// Charge drawn from the battery since the start, Ah.
    pub charge_used_ah: f64,
    /// Battery current of the previous step, A (feeds the voltage sag of the next step).
    pub battery_current_a: f64,
    /// Battery temperature, deg C.
    pub battery_temp_c: f64,
    /// Time the loaded pack voltage has been continuously below the cutoff, s.
    pub time_below_cutoff_s: f64,
    /// Motor winding temperatures, deg C.
    pub motor_winding_c: [f64; 4],
    /// Motor case temperatures, deg C.
    pub motor_case_c: [f64; 4],
    /// ESC temperatures, deg C.
    pub esc_temp_c: [f64; 4],
    /// Total time during which at least one propeller was outside the measured Ct/Cp data, s.
    pub prop_extrapolated_time_s: f64,
}

/// Per-step powertrain results (interface v13). All values are for the step just taken.
/// A rejected step overwrites it with NaN floats and all flags 0 (`step_valid` 0).
#[derive(Copy, Clone)]
#[repr(C)]
pub struct PowertrainTelemetry {
    /// Motor current, A.
    pub motor_current_a: [f64; 4],
    /// Voltage at the motor terminals (duty x pack voltage), V.
    pub motor_voltage_v: [f64; 4],
    /// Motor speed, rpm.
    pub motor_rpm: [f64; 4],
    /// Propeller torque, N*m.
    pub motor_torque_nm: [f64; 4],
    /// Propeller thrust, N.
    pub motor_thrust_n: [f64; 4],
    /// Copper loss, W.
    pub motor_copper_loss_w: [f64; 4],
    /// No-load (iron + friction) loss, W.
    pub motor_noload_loss_w: [f64; 4],
    /// ESC loss, W.
    pub esc_loss_w: [f64; 4],
    /// Motor winding temperature after the step, deg C.
    pub motor_winding_c: [f64; 4],
    /// Motor case temperature after the step, deg C.
    pub motor_case_c: [f64; 4],
    /// ESC temperature after the step, deg C.
    pub esc_temp_c: [f64; 4],
    /// Loaded pack voltage, V.
    pub battery_voltage_v: f64,
    /// Open-circuit pack voltage, V.
    pub battery_ocv_v: f64,
    /// Pack current, A.
    pub battery_current_a: f64,
    /// Power drawn from the pack, W.
    pub battery_power_w: f64,
    /// State of charge after the step, unitless.
    pub battery_soc: f64,
    /// Battery temperature after the step, deg C.
    pub battery_temp_c: f64,
    /// Battery heating, W.
    pub battery_heat_w: f64,
    /// Charge drawn since the start, Ah.
    pub charge_used_ah: f64,
    /// Time continuously below the cutoff voltage, s.
    pub time_below_cutoff_s: f64,
    /// 1 if this struct holds the results of a successful step.
    pub step_valid: i32,
    /// 1 if the loaded pack voltage is below cells x cutoff voltage (instantaneous; the step keeps running).
    pub battery_cutoff: i32,
    /// 1 if the pack current exceeds its continuous rating.
    pub battery_over_current: i32,
    /// 1 if the battery temperature exceeds its limit.
    pub battery_over_temp: i32,
    /// 1 if the state of charge reached 0 and was held there.
    pub battery_soc_clamped: i32,
    /// Bit i set: motor i current above its rating.
    pub motor_over_current_mask: i32,
    /// Bit i set: motor i winding above its temperature limit.
    pub motor_over_temp_mask: i32,
    /// Bit i set: ESC i input current above its rating.
    pub esc_over_current_mask: i32,
    /// Bit i set: ESC i above its temperature limit.
    pub esc_over_temp_mask: i32,
    /// Copy of `PowertrainConfig::parameters_verified` (0 while any input is ASSUMED).
    pub thermal_valid: i32,
    /// Bit i set: propeller i is outside the measured Ct/Cp data (held at the nearest edge value). Always 0 with `prop_model` 0.
    pub prop_extrapolated: i32,
    /// Reserved, 0.
    pub reserved0: i32,
    /// Reserved, 0.
    pub reserved1: i32,
    /// Reserved, 0.
    pub reserved2: i32,
    /// Advance ratio of each propeller (0 for a rotor at rest).
    pub motor_advance_ratio: [f64; 4],
    /// Thrust coefficient used for each propeller (0 for a rotor at rest).
    pub motor_ct: [f64; 4],
    /// Power coefficient used for each propeller (0 for a rotor at rest).
    pub motor_cp: [f64; 4],
}

/// Result of a stateless motor + propeller bench point (`ffi_powertrain_bench_point`), interface v13.
#[derive(Copy, Clone)]
#[repr(C)]
pub struct PowertrainBenchOut {
    /// Motor speed, rpm.
    pub rpm: f64,
    /// Motor current, A.
    pub motor_current_a: f64,
    /// Propeller torque, N*m.
    pub torque_nm: f64,
    /// Propeller thrust, N.
    pub thrust_n: f64,
    /// Shaft power, W.
    pub shaft_w: f64,
    /// Motor electrical input power, W.
    pub motor_electrical_w: f64,
    /// Copper loss, W.
    pub copper_loss_w: f64,
    /// No-load loss, W.
    pub noload_loss_w: f64,
    /// ESC loss, W.
    pub esc_loss_w: f64,
    /// Current drawn from the pack, A.
    pub battery_current_a: f64,
    /// Power drawn from the pack, W.
    pub battery_power_w: f64,
    /// Shaft power over pack power, unitless.
    pub system_efficiency: f64,
    /// Advance ratio used.
    pub advance_ratio: f64,
    /// Thrust coefficient used.
    pub ct: f64,
    /// Power coefficient used.
    pub cp: f64,
    /// 1 if the propeller was outside the measured Ct/Cp data.
    pub extrapolated: i32,
    /// Reserved, 0.
    pub reserved: i32,
}

/// Propeller coefficients at one operating point (`ffi_propeller_coefficients`), interface v14.
#[derive(Copy, Clone)]
#[repr(C)]
pub struct PropCoefficients {
    /// Thrust coefficient.
    pub ct: f64,
    /// Power coefficient.
    pub cp: f64,
    /// 1 if (J, rpm) is outside the measured data and the nearest edge value was used.
    pub extrapolated: i32,
    /// Reserved, 0.
    pub reserved: i32,
}

impl PowertrainTelemetry {
    /// "No data" telemetry: NaN floats, all flags 0. Written for a rejected step.
    #[must_use]
    pub const fn no_data() -> Self {
        Self {
            motor_current_a: [f64::NAN; 4],
            motor_voltage_v: [f64::NAN; 4],
            motor_rpm: [f64::NAN; 4],
            motor_torque_nm: [f64::NAN; 4],
            motor_thrust_n: [f64::NAN; 4],
            motor_copper_loss_w: [f64::NAN; 4],
            motor_noload_loss_w: [f64::NAN; 4],
            esc_loss_w: [f64::NAN; 4],
            motor_winding_c: [f64::NAN; 4],
            motor_case_c: [f64::NAN; 4],
            esc_temp_c: [f64::NAN; 4],
            battery_voltage_v: f64::NAN,
            battery_ocv_v: f64::NAN,
            battery_current_a: f64::NAN,
            battery_power_w: f64::NAN,
            battery_soc: f64::NAN,
            battery_temp_c: f64::NAN,
            battery_heat_w: f64::NAN,
            charge_used_ah: f64::NAN,
            time_below_cutoff_s: f64::NAN,
            step_valid: 0,
            battery_cutoff: 0,
            battery_over_current: 0,
            battery_over_temp: 0,
            battery_soc_clamped: 0,
            motor_over_current_mask: 0,
            motor_over_temp_mask: 0,
            esc_over_current_mask: 0,
            esc_over_temp_mask: 0,
            thermal_valid: 0,
            prop_extrapolated: 0,
            reserved0: 0,
            reserved1: 0,
            reserved2: 0,
            motor_advance_ratio: [f64::NAN; 4],
            motor_ct: [f64::NAN; 4],
            motor_cp: [f64::NAN; 4],
        }
    }
}

/// Largest accepted deviation of a state quaternion's norm from 1 at the FFI boundary.
/// f32 storage keeps a unit quaternion within about 2.4e-7 of norm 1 and the integrator
/// re-normalizes every step, so 1e-3 only trips on a caller bug (zero, scaled or garbage
/// quaternion), never on rounding. TODO(HUMAN-VERIFY): the tolerance is an engineering choice.
pub const QUAT_NORM_TOLERANCE: f64 = 1.0e-3;

/// Why `unit_quaternion_xyzw` rejected a quaternion.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum QuatError {
    /// A component is NaN or infinite.
    NonFinite,
    /// `| |q| - 1 | > QUAT_NORM_TOLERANCE` (includes the zero quaternion).
    NotUnit,
}

/// Validates a body -> NED quaternion in (x, y, z, w) order and returns it re-normalized
/// (norm computed in f64, result rounded to f32). Within tolerance the change is at most
/// the tolerance; outside it the quaternion is REJECTED, never silently rescaled.
///
/// # Errors
/// `NonFinite` for NaN/Inf, `NotUnit` when the norm is farther than `QUAT_NORM_TOLERANCE` from 1.
pub fn unit_quaternion_xyzw(q: [f32; 4]) -> Result<[f32; 4], QuatError> {
    let c = q.map(f64::from);
    if !c.iter().all(|v| v.is_finite()) {
        return Err(QuatError::NonFinite);
    }
    let norm = c.iter().map(|v| v * v).sum::<f64>().sqrt();
    if (norm - 1.0).abs() > QUAT_NORM_TOLERANCE {
        return Err(QuatError::NotUnit);
    }
    Ok(c.map(|v| (v / norm) as f32))
}

// WP-4.6 / ADR-001 Decision 7: compile-time layout guard for every repr(C) FFI struct.
// A layout change fails `cargo build`, not only `cargo test`. Per-field offsets are
// checked in `tests` below and in `UE5Project/.../Public/DroneFfiLayout.h` (C++).
// Rules: field at next multiple of its alignment; size rounded up to struct alignment.
// f64 = 8 B (align 8); f32 and i32 = 4 B (align 4); arrays: n * element size.
const _: () = {
    use std::mem::{align_of, size_of};
    // 3*8 + 3*4 + 4*4 + 3*4 = 64
    assert!(size_of::<DroneState>() == 64 && align_of::<DroneState>() == 8);
    // 4*4 = 16
    assert!(size_of::<ControlInputs>() == 16 && align_of::<ControlInputs>() == 4);
    // 4*8 = 32
    assert!(size_of::<ThermalState>() == 32 && align_of::<ThermalState>() == 8);
    // 3*8 = 24
    assert!(size_of::<WindInput>() == 24 && align_of::<WindInput>() == 8);
    assert!(size_of::<EnvironmentInput>() == 16 && align_of::<EnvironmentInput>() == 8);
    // 1*4 = 4
    assert!(size_of::<FaultLatch>() == 4 && align_of::<FaultLatch>() == 4);
    // 3*4 + 4 + 4 = 20
    assert!(size_of::<SurrogateQueryResult>() == 20 && align_of::<SurrogateQueryResult>() == 4);
    // 8 + 24 + 96 + 32 + 5*8 = 200
    assert!(size_of::<VehicleConfig>() == 200 && align_of::<VehicleConfig>() == 8);
    // 4 vec3 f32 (48) + 3 vec4 f32 (48) + margin f32 (4) + 9 i32 (36) = 136
    assert!(size_of::<DebugTelemetry>() == 136 && align_of::<DebugTelemetry>() == 4);
    // PowertrainConfig: 472 + 8*8 (curve rpm) + 3*8*48*8 (tables) + 8*4 (point counts) + 2*4 = 472 + 64 + 9216 + 32 + 8 = 9792
    assert!(size_of::<PowertrainConfig>() == 9792 && align_of::<PowertrainConfig>() == 8);
    // PowertrainState: 5*8 + 3*4*8 + 8 = 144
    assert!(size_of::<PowertrainState>() == 144 && align_of::<PowertrainState>() == 8);
    // PowertrainTelemetry: 11*4*8 + 9*8 + 14*4 + 3*4*8 = 352 + 72 + 56 + 96 = 576
    assert!(size_of::<PowertrainTelemetry>() == 576 && align_of::<PowertrainTelemetry>() == 8);
    // PowertrainBenchOut: 12*8 + 3*8 + 2*4 = 128
    assert!(size_of::<PowertrainBenchOut>() == 128 && align_of::<PowertrainBenchOut>() == 8);
    // PropCoefficients: 2*8 + 2*4 = 24
    assert!(size_of::<PropCoefficients>() == 24 && align_of::<PropCoefficients>() == 8);
};

#[cfg(test)]
mod tests {
    use super::*;
    use std::mem::offset_of;

    #[test]
    fn test_ffi_offsets_and_sizes() {
        // DroneState layout verification
        assert_eq!(std::mem::size_of::<DroneState>(), 64);
        assert_eq!(offset_of!(DroneState, position), 0);
        assert_eq!(offset_of!(DroneState, velocity), 24);
        assert_eq!(offset_of!(DroneState, orientation), 36);
        assert_eq!(offset_of!(DroneState, angular_velocity), 52);

        // ThermalState layout verification
        assert_eq!(std::mem::size_of::<ThermalState>(), 32);
        assert_eq!(offset_of!(ThermalState, motor_temp_c), 0);
    }

    #[test]
    fn test_fault_latch_layout() {
        // i32 field: 4 bytes, offset 0. A zeroed latch means "no fault".
        assert_eq!(std::mem::size_of::<FaultLatch>(), 4);
        assert_eq!(offset_of!(FaultLatch, physics_fault), 0);
        assert_eq!(FaultLatch::new().physics_fault, 0);
    }

    #[test]
    fn test_wind_input_layout() {
        // 3 x f64, no padding: offset 0, size 24, align 8. Zero is still air.
        assert_eq!(std::mem::size_of::<WindInput>(), 24);
        assert_eq!(std::mem::align_of::<WindInput>(), 8);
        assert_eq!(offset_of!(WindInput, wind_ned_mps), 0);
        // EnvironmentInput (v12): 2 * 8 = 16, align 8.
        assert_eq!(std::mem::size_of::<EnvironmentInput>(), 16);
        assert_eq!(std::mem::align_of::<EnvironmentInput>(), 8);
        assert_eq!(offset_of!(EnvironmentInput, site_altitude_m), 0);
        assert_eq!(offset_of!(EnvironmentInput, temp_offset_c), 8);
        assert_eq!(WindInput::still().wind_ned_mps, [0.0; 3]);
    }

    #[test]
    fn test_control_inputs_layout() {
        // 4 x f32, no padding: offsets 0, 4, 8, 12; size 16.
        assert_eq!(std::mem::size_of::<ControlInputs>(), 16);
        assert_eq!(offset_of!(ControlInputs, throttle), 0);
        assert_eq!(offset_of!(ControlInputs, roll), 4);
        assert_eq!(offset_of!(ControlInputs, pitch), 8);
        assert_eq!(offset_of!(ControlInputs, yaw), 12);
    }

    #[test]
    fn test_vehicle_config_layout() {
        // mass 8 B @0; inertia 3x8=24 @8; motor_pos 4x3x8=96 @32; motor_spin 4x8=32 @128;
        // then five f64 at 160, 168, 176, 184, 192; total 200 (multiple of align 8).
        assert_eq!(std::mem::size_of::<VehicleConfig>(), 200);
        assert_eq!(offset_of!(VehicleConfig, mass_kg), 0);
        assert_eq!(offset_of!(VehicleConfig, inertia_kgm2), 8);
        assert_eq!(offset_of!(VehicleConfig, motor_pos_m), 32);
        assert_eq!(offset_of!(VehicleConfig, motor_spin), 128);
        assert_eq!(offset_of!(VehicleConfig, prop_diameter_m), 160);
        assert_eq!(offset_of!(VehicleConfig, thrust_coeff), 168);
        assert_eq!(offset_of!(VehicleConfig, torque_coeff), 176);
        assert_eq!(offset_of!(VehicleConfig, max_rpm), 184);
        assert_eq!(offset_of!(VehicleConfig, hover_throttle), 192);
    }

    // Hand calc: (0,0,0,1) is exact. (0,0,0,1.0005) has norm 1.0005, inside 1e-3, and
    // normalizes to w = 1. (0,0,0,1.002) and (0,0,0,0) are rejected; (0,0,0,0.998) too.
    // (3,0,0,4)/5 style: (0.6, 0, 0, 0.8) has norm 1 exactly in real numbers; f32 rounding of
    // 0.6 and 0.8 keeps it within 1e-7, so it is accepted and stays within 1e-7 of (0.6, 0, 0, 0.8).
    #[test]
    fn test_unit_quaternion_xyzw_truth_table() {
        assert_eq!(
            unit_quaternion_xyzw([0.0, 0.0, 0.0, 1.0]),
            Ok([0.0, 0.0, 0.0, 1.0])
        );
        assert_eq!(
            unit_quaternion_xyzw([0.0, 0.0, 0.0, 1.0005]),
            Ok([0.0, 0.0, 0.0, 1.0])
        );
        assert_eq!(
            unit_quaternion_xyzw([0.0, 0.0, 0.0, 1.002]),
            Err(QuatError::NotUnit)
        );
        assert_eq!(
            unit_quaternion_xyzw([0.0, 0.0, 0.0, 0.998]),
            Err(QuatError::NotUnit)
        );
        assert_eq!(unit_quaternion_xyzw([0.0; 4]), Err(QuatError::NotUnit));
        assert_eq!(
            unit_quaternion_xyzw([f32::NAN, 0.0, 0.0, 1.0]),
            Err(QuatError::NonFinite)
        );
        assert_eq!(
            unit_quaternion_xyzw([0.0, f32::INFINITY, 0.0, 1.0]),
            Err(QuatError::NonFinite)
        );
        let q = unit_quaternion_xyzw([0.6, 0.0, 0.0, 0.8]).expect("unit within tolerance");
        assert!((f64::from(q[0]) - 0.6).abs() < 1e-7 && (f64::from(q[3]) - 0.8).abs() < 1e-7);
    }

    #[test]
    fn test_debug_telemetry_layout() {
        // vec3 f32 = 12 B at 0, 12, 24, 36; vec4 f32 = 16 B at 48, 64, 80; margin f32 @96;
        // nine i32 at 100, 104, ..., 132; total 136.
        assert_eq!(std::mem::size_of::<DebugTelemetry>(), 136);
        assert_eq!(offset_of!(DebugTelemetry, net_thrust), 0);
        assert_eq!(offset_of!(DebugTelemetry, aero_drag), 12);
        assert_eq!(offset_of!(DebugTelemetry, gravity), 24);
        assert_eq!(offset_of!(DebugTelemetry, net_force), 36);
        assert_eq!(offset_of!(DebugTelemetry, motor_thrusts), 48);
        assert_eq!(offset_of!(DebugTelemetry, motor_rpms), 64);
        assert_eq!(offset_of!(DebugTelemetry, motor_temperatures_c), 80);
        assert_eq!(offset_of!(DebugTelemetry, structural_safety_margin), 96);
        assert_eq!(offset_of!(DebugTelemetry, is_validated_envelope), 100);
        assert_eq!(offset_of!(DebugTelemetry, stub_loaded), 104);
        assert_eq!(offset_of!(DebugTelemetry, aero_source), 108);
        assert_eq!(offset_of!(DebugTelemetry, fea_source), 112);
        assert_eq!(offset_of!(DebugTelemetry, physics_fault), 116);
        assert_eq!(offset_of!(DebugTelemetry, dropped_time_events), 120);
        assert_eq!(offset_of!(DebugTelemetry, uncalibrated_flags), 124);
        assert_eq!(offset_of!(DebugTelemetry, thermal_valid), 128);
        assert_eq!(offset_of!(DebugTelemetry, thermal_over_temp), 132);
    }

    #[test]
    fn test_powertrain_struct_layouts() {
        // PowertrainConfig: 25 f64 scalars (0..200), two 16-entry tables (200, 328), four i32 (456..472).
        assert_eq!(std::mem::size_of::<PowertrainConfig>(), 9792);
        assert_eq!(offset_of!(PowertrainConfig, motor_kv_rpm_per_v), 0);
        assert_eq!(offset_of!(PowertrainConfig, prop_diameter_m), 40);
        assert_eq!(offset_of!(PowertrainConfig, esc_efficiency), 64);
        assert_eq!(offset_of!(PowertrainConfig, battery_capacity_ah), 88);
        assert_eq!(
            offset_of!(PowertrainConfig, motor_r_winding_case_k_per_w),
            128
        );
        assert_eq!(offset_of!(PowertrainConfig, cooling_ref_airspeed_mps), 192);
        assert_eq!(offset_of!(PowertrainConfig, ocv_soc), 200);
        assert_eq!(offset_of!(PowertrainConfig, ocv_v_per_cell), 328);
        assert_eq!(offset_of!(PowertrainConfig, battery_cells), 456);
        assert_eq!(offset_of!(PowertrainConfig, ocv_points), 460);
        assert_eq!(offset_of!(PowertrainConfig, parameters_verified), 464);
        assert_eq!(offset_of!(PowertrainConfig, reserved), 468);
        // P3b table: curve rpm 472, J table 536, Ct table 3608, Cp table 6680, point counts 9752, count 9784, model 9788.
        assert_eq!(offset_of!(PowertrainConfig, prop_curve_rpm), 472);
        assert_eq!(offset_of!(PowertrainConfig, prop_j), 536);
        assert_eq!(offset_of!(PowertrainConfig, prop_ct_table), 3608);
        assert_eq!(offset_of!(PowertrainConfig, prop_cp_table), 6680);
        assert_eq!(offset_of!(PowertrainConfig, prop_curve_points), 9752);
        assert_eq!(offset_of!(PowertrainConfig, prop_curve_count), 9784);
        assert_eq!(offset_of!(PowertrainConfig, prop_model), 9788);
        // PowertrainState: five f64 (0..40) then three arrays of four f64 (40, 72, 104); total 136.
        assert_eq!(std::mem::size_of::<PowertrainState>(), 144);
        assert_eq!(offset_of!(PowertrainState, soc), 0);
        assert_eq!(offset_of!(PowertrainState, battery_current_a), 16);
        assert_eq!(offset_of!(PowertrainState, time_below_cutoff_s), 32);
        assert_eq!(offset_of!(PowertrainState, motor_winding_c), 40);
        assert_eq!(offset_of!(PowertrainState, motor_case_c), 72);
        assert_eq!(offset_of!(PowertrainState, esc_temp_c), 104);
        assert_eq!(offset_of!(PowertrainState, prop_extrapolated_time_s), 136);
        // PowertrainTelemetry: eleven arrays of four f64 (0..352), nine f64 (352..424), fourteen i32 (424..480).
        assert_eq!(std::mem::size_of::<PowertrainTelemetry>(), 576);
        assert_eq!(offset_of!(PowertrainTelemetry, motor_current_a), 0);
        assert_eq!(offset_of!(PowertrainTelemetry, motor_rpm), 64);
        assert_eq!(offset_of!(PowertrainTelemetry, motor_thrust_n), 128);
        assert_eq!(offset_of!(PowertrainTelemetry, esc_temp_c), 320);
        assert_eq!(offset_of!(PowertrainTelemetry, battery_voltage_v), 352);
        assert_eq!(offset_of!(PowertrainTelemetry, battery_soc), 384);
        assert_eq!(offset_of!(PowertrainTelemetry, time_below_cutoff_s), 416);
        assert_eq!(offset_of!(PowertrainTelemetry, step_valid), 424);
        assert_eq!(
            offset_of!(PowertrainTelemetry, motor_over_current_mask),
            444
        );
        assert_eq!(offset_of!(PowertrainTelemetry, thermal_valid), 460);
        assert_eq!(offset_of!(PowertrainTelemetry, prop_extrapolated), 464);
        assert_eq!(offset_of!(PowertrainTelemetry, reserved2), 476);
        assert_eq!(offset_of!(PowertrainTelemetry, motor_advance_ratio), 480);
        assert_eq!(offset_of!(PowertrainTelemetry, motor_ct), 512);
        assert_eq!(offset_of!(PowertrainTelemetry, motor_cp), 544);
        // PowertrainBenchOut: twelve f64.
        assert_eq!(std::mem::size_of::<PowertrainBenchOut>(), 128);
        assert_eq!(offset_of!(PowertrainBenchOut, rpm), 0);
        assert_eq!(offset_of!(PowertrainBenchOut, battery_current_a), 72);
        assert_eq!(offset_of!(PowertrainBenchOut, system_efficiency), 88);
        assert_eq!(offset_of!(PowertrainBenchOut, advance_ratio), 96);
        assert_eq!(offset_of!(PowertrainBenchOut, ct), 104);
        assert_eq!(offset_of!(PowertrainBenchOut, cp), 112);
        assert_eq!(offset_of!(PowertrainBenchOut, extrapolated), 120);
        assert_eq!(std::mem::size_of::<PropCoefficients>(), 24);
        assert_eq!(offset_of!(PropCoefficients, cp), 8);
        assert_eq!(offset_of!(PropCoefficients, extrapolated), 16);
        assert_eq!(align_of::<PropCoefficients>(), 8);
        use std::mem::align_of;
        assert_eq!(align_of::<PowertrainConfig>(), 8);
        assert_eq!(align_of::<PowertrainState>(), 8);
        assert_eq!(align_of::<PowertrainTelemetry>(), 8);
        assert_eq!(align_of::<PowertrainBenchOut>(), 8);
    }

    #[test]
    fn test_surrogate_query_result_layout() {
        // predicted_values 3x4=12 @0; uncertainty @12; in_validated_envelope @16; total 20.
        assert_eq!(std::mem::size_of::<SurrogateQueryResult>(), 20);
        assert_eq!(offset_of!(SurrogateQueryResult, predicted_values), 0);
        assert_eq!(offset_of!(SurrogateQueryResult, uncertainty), 12);
        assert_eq!(offset_of!(SurrogateQueryResult, in_validated_envelope), 16);
    }

    #[test]
    fn test_ffi_struct_alignments() {
        // Alignment is that of the widest member: f64 -> 8, f32/i32 -> 4.
        use std::mem::align_of;
        assert_eq!(align_of::<DroneState>(), 8);
        assert_eq!(align_of::<ControlInputs>(), 4);
        assert_eq!(align_of::<ThermalState>(), 8);
        assert_eq!(align_of::<FaultLatch>(), 4);
        assert_eq!(align_of::<SurrogateQueryResult>(), 4);
        assert_eq!(align_of::<VehicleConfig>(), 8);
        assert_eq!(align_of::<DebugTelemetry>(), 4);
    }

    #[test]
    fn test_cpp_expected_version_matches_rust() {
        // WP-4.5 / ADR-001 D7: the C++ constant must equal the Rust interface version.
        // No Unreal needed: parse the one declaration line out of the UE-free C++ header.
        const HEADER: &str = include_str!(
            "../../UE5Project/Plugins/DronePhysicsBridge/Source/DronePhysicsBridge/Public/DroneFfiLayout.h"
        );
        const PREFIX: &str = "constexpr std::int32_t kDroneFfiExpectedInterfaceVersion = ";
        let mut found = HEADER
            .lines()
            .filter_map(|l| l.trim().strip_prefix(PREFIX))
            .map(|rest| rest.trim_end_matches(';').trim().parse::<i32>());
        let cpp_version = found.next();
        assert!(found.next().is_none(), "constant declared more than once");
        assert_eq!(cpp_version, Some(Ok(crate::ffi_get_interface_version())));
    }

    // The demo's UE-free environment header must use the same limits as the core, or a preset it
    // accepts could be rejected with code 13 (or the reverse). Parses `constexpr double NAME = VALUE;`.
    #[test]
    fn test_cpp_environment_limits_match_rust() {
        const HEADER: &str = include_str!(
            "../../UE5Project/Plugins/DronePhysicsBridge/Source/DronePhysicsBridge/Public/DroneEnvironment.h"
        );
        let value = |name: &str| -> f64 {
            let prefix = format!("constexpr double {name} = ");
            let mut found = HEADER
                .lines()
                .filter_map(|l| l.trim().strip_prefix(prefix.as_str()))
                .map(|rest| rest.split(';').next().unwrap_or("").trim().parse::<f64>());
            let v = found.next();
            assert!(found.next().is_none(), "{name} declared more than once");
            v.expect("constant not found").expect("not a number")
        };
        assert_eq!(value("kMaxSiteAltitudeM"), MAX_SITE_ALTITUDE_M);
        assert_eq!(value("kMaxTempOffsetC"), MAX_TEMP_OFFSET_C);
    }

    #[test]
    fn test_debug_telemetry_default_no_margin() {
        // HZ-02: a default telemetry must never show a safe-looking margin.
        let t = DebugTelemetry::new();
        assert!(t.structural_safety_margin.is_nan());
        assert_eq!(t.aero_source, 0);
        assert_eq!(t.fea_source, 0);
        assert_eq!(t.stub_loaded, 0);
        assert_eq!(t.physics_fault, 0);
    }
}
