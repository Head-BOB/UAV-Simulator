#ifndef DRONE_FFI_H
#define DRONE_FFI_H

#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

/**
 * Return code: a panic was caught inside the FFI function, or the handle was faulted by an
 * earlier panic. The call's effect is undefined except that no memory was corrupted; the
 * caller should treat the run as invalid. Telemetry is not written.
 */
#define FFI_CODE_PANIC 99

/**
 * Return code: an internal lock was poisoned and has been recovered. The call did nothing;
 * calling again works.
 */
#define FFI_CODE_LOCK_RECOVERED 98

/**
 * Largest mean wind accepted, m/s (the core accepts +-30 m/s per component including turbulence).
 */
#define MAX_MEAN_WIND_MPS 20.0

/**
 * Version of the conditions / definitions / output layouts.
 */
#define SCHEMA_VERSION 1

/**
 * Version of the controller config file layout. A file with another version is refused.
 */
#define CONTROLLER_CONFIG_VERSION 1

/**
 * Fixed step of every closed-loop run, s.
 */
#define DT_S 0.002

/**
 * Altitude of the start of every scenario, m.
 */
#define START_ALTITUDE_M 20.0

/**
 * Pre-roll hovering time, s: the battery is full, so the duty that hovers is lower than the build's estimate (nominal voltage);
 * the velocity integrator takes this time to absorb it. Every scenario starts after it.
 */
#define PREROLL_S 8.0



/**
 * Largest payload accepted, kg.
 */
#define MAX_PAYLOAD_KG 50.0

/**
 * Largest |coordinate| of the payload position, m.
 */
#define MAX_PAYLOAD_POSITION_M 2.0

/**
 * Engineering bound on the model input width, not a physics limit. The widest planned
 * input today is 4 (aero); raise only by a reviewed change.
 */
#define MAX_INPUT_WIDTH 64

/**
 * Length of `SurrogateQueryResult::predicted_values`.
 */
#define PREDICTED_SLOTS 3

/**
 * Number of inputs the aero surrogate takes: airspeed, angle of attack, sideslip.
 */
#define AERO_INPUT_WIDTH 3

/**
 * Number of inputs the FEA surrogate takes: 4 arm loads, 3 direction components, 1 temperature.
 */
#define FEA_INPUT_WIDTH 8

#define AMBIENT_TEMP 20.0

/**
 * True only when every thermal constant above has a sourced, verified value (item B-4).
 * Stays `false` until a human closes B-4; reported as `DebugTelemetry::thermal_valid`.
 */
#define THERMAL_CONSTANTS_VALIDATED false

/**
 * Motor winding over-temperature limit, deg C. PLACEHOLDER, no source.
 * TODO(HUMAN-VERIFY): replace with the motor datasheet limit (B-4). Not a `VehicleConfig`
 * field on purpose: that would change the config hash that every model is bound to.
 */
#define MOTOR_TEMP_LIMIT_C 120.0

/**
 * Floor of the air speed used in the filters, m/s. ASSUMED, `TODO(HUMAN-VERIFY)`.
 */
#define V_MIN_MPS 2.0

/**
 * Metres per foot (exact by definition).
 */
#define FOOT_M 0.3048

/**
 * Time constant of the simple gust, s. ASSUMED, `TODO(HUMAN-VERIFY)`.
 */
#define SIMPLE_TAU_S 2.0

/**
 * Vertical share of the simple gust intensity. ASSUMED, `TODO(HUMAN-VERIFY)`.
 */
#define SIMPLE_VERTICAL_FRACTION 0.5

/**
 * Largest accepted |component| of `WindInput::wind_ned_mps`, m/s.
 * TODO(HUMAN-VERIFY: demo value) Chosen by the project owner as a sanity bound for the demo
 * (30 m/s is about gale force); it is not a validated aerodynamic limit of the drag model.
 */
#define MAX_WIND_COMPONENT_MPS 30.0

/**
 * Largest accepted `EnvironmentInput::site_altitude_m`, m.
 * TODO(HUMAN-VERIFY: demo value) The tallest demo site is a mountain preset; 5000 m keeps the
 * total height (site plus flight) inside the ISA troposphere model (11000 m) with room to fly.
 */
#define MAX_SITE_ALTITUDE_M 5000.0

/**
 * Largest accepted |`EnvironmentInput::temp_offset_c`|, K.
 * TODO(HUMAN-VERIFY: demo value) A sanity bound for the demo (about -40..+40 K around the standard
 * day covers polar to desert air); it is not a validated limit of any model here.
 */
#define MAX_TEMP_OFFSET_C 40.0

/**
 * Number of points the battery open-circuit-voltage table can hold (fixed size: no heap in the FFI).
 */
#define MAX_OCV_POINTS 16

/**
 * Number of propeller speed curves the Ct/Cp table can hold (fixed size: no heap in the FFI).
 */
#define MAX_PROP_CURVES 8

/**
 * Number of (J, Ct, Cp) points per propeller speed curve.
 */
#define MAX_PROP_POINTS 48

/**
 * Largest accepted deviation of a state quaternion's norm from 1 at the FFI boundary.
 * f32 storage keeps a unit quaternion within about 2.4e-7 of norm 1 and the integrator
 * re-normalizes every step, so 1e-3 only trips on a caller bug (zero, scaled or garbage
 * quaternion), never on rounding. TODO(HUMAN-VERIFY): the tolerance is an engineering choice.
 */
#define QUAT_NORM_TOLERANCE 1.0e-3

/**
 * Rows whose |reference| is below this fraction of the largest |reference| of their quantity get no percent error.
 */
#define PERCENT_FLOOR_FRACTION 0.05

/**
 * Handle to an active ONNX surrogate model session.
 */
typedef struct SurrogateHandle SurrogateHandle;

/**
 * The tests of this phase, in report order.
 */
typedef struct TestId TestId;

/**
 * Represents the drone's current physical condition.
 *
 * #[repr(C)] guarantees this struct has the exact same memory layout on
 * both sides of the FFI boundary.
 *
 * COORDINATE FRAME (see ADR-001, Section 1.3 of the Standards Framework):
 * position is expressed in a local North-East-Down (NED) frame, in meters,
 * relative to a fixed WGS84 geodetic origin defined once per simulation
 * scenario. Do not reinterpret this as UE5 world-space — conversion
 * happens ONLY inside the wrapper class described in Section 1.4.
 */
typedef struct DroneState {
  /**
   * North-East-Down position relative to the scenario's WGS84 origin.
   *
   * # Units
   * Meters. f64: see ADR-001 — f32 loses sub-meter precision at realistic mission ranges.
   */
  double position[3];
  /**
   * Local-frame linear velocity.
   *
   * # Units
   * Meters per second. f32 is sufficient: magnitude never grows large enough to lose useful precision.
   */
  float velocity[3];
  /**
   * Rotation as a unit quaternion, stored in order (x, y, z, w).
   *
   * # Units
   * Unitless quaternion. f32 is sufficient: components are always within [-1.0, 1.0].
   */
  float orientation[4];
  /**
   * Angular velocity in the body frame.
   *
   * # Units
   * Radians per second. f32 is sufficient for the same reason as velocity.
   */
  float angular_velocity[3];
} DroneState;

/**
 * State container for the thermal and electrical simulation.
 */
typedef struct ThermalState {
  /**
   * Internal winding temperatures for each motor.
   *
   * # Units
   * Degrees Celsius.
   */
  double motor_temp_c[4];
} ThermalState;

/**
 * Represents what the pilot/controller is commanding.
 */
typedef struct ControlInputs {
  /**
   * Commanded throttle.
   *
   * # Units
   * Normalized range 0.0 (none) to 1.0 (full).
   */
  float throttle;
  /**
   * Commanded roll input.
   *
   * # Units
   * Normalized range -1.0 to 1.0.
   */
  float roll;
  /**
   * Commanded pitch input.
   *
   * # Units
   * Normalized range -1.0 to 1.0.
   */
  float pitch;
  /**
   * Commanded yaw input.
   *
   * # Units
   * Normalized range -1.0 to 1.0.
   */
  float yaw;
} ControlInputs;

/**
 * Ambient wind for one `ffi_step_physics` call (interface v11).
 *
 * COORDINATE FRAME: NED world frame (ADR-001), the direction the air moves TOWARD.
 * All zeros is still air. The step computes the air-relative velocity
 * `v_air = v_ground - wind_ned_mps` for the aerodynamic drag only.
 */
typedef struct WindInput {
  /**
   * Wind velocity, north / east / down components.
   *
   * # Units
   * Metres per second, f64. Each component must be finite with |value| <= `MAX_WIND_COMPONENT_MPS`.
   */
  double wind_ned_mps[3];
} WindInput;

/**
 * Atmosphere for one `ffi_step_physics` call (interface v12): where the NED origin sits above sea
 * level and how much warmer or colder than the ISA standard day the air is.
 *
 * All zeros is the ISA standard day at sea level, which reproduces the v11 behaviour bit for bit.
 * Air density at the vehicle comes from the ISA pressure at the total height (`site_altitude_m`
 * plus the vehicle altitude above the origin) and the ISA temperature at that height PLUS
 * `temp_offset_c` (`aero::air_density`). Humidity is not modelled.
 */
typedef struct EnvironmentInput {
  /**
   * Height of the NED origin above sea level.
   *
   * # Units
   * Metres, f64, finite, 0..=`MAX_SITE_ALTITUDE_M`.
   */
  double site_altitude_m;
  /**
   * Air temperature minus the ISA standard-day temperature at the same height.
   *
   * # Units
   * Kelvin (equal to degrees C for a difference), f64, finite, |value| <= `MAX_TEMP_OFFSET_C`.
   */
  double temp_offset_c;
} EnvironmentInput;

/**
 * Physical properties and layout of the specific airframe being simulated.
 */
typedef struct VehicleConfig {
  /**
   * Total mass of the vehicle.
   *
   * # Units
   * Kilograms.
   */
  double mass_kg;
  /**
   * Moment of inertia about the center of mass (Ixx, Iyy, Izz).
   *
   * # Units
   * Kilogram-square meters.
   */
  double inertia_kgm2[3];
  /**
   * Positions of the 4 motors in the body frame.
   *
   * # Units
   * Meters.
   */
  double motor_pos_m[4][3];
  /**
   * Spin direction of each motor (+1.0 or -1.0), in the same index order as `motor_pos_m`.
   * +1.0 means the reaction torque on the airframe is about body +Z (nose right), i.e. the
   * propeller turns CCW seen from above.
   *
   * # Units
   * Unitless multiplier.
   */
  double motor_spin[4];
  /**
   * Diameter of the propellers.
   *
   * # Units
   * Meters.
   */
  double prop_diameter_m;
  /**
   * Propeller thrust coefficient.
   *
   * # Units
   * Unitless.
   */
  double thrust_coeff;
  /**
   * Propeller yaw-torque coefficient: reaction torque per unit of thrust (`kappa`).
   * Yaw torque on the airframe is `sum(motor_spin * torque_coeff * thrust_n)`.
   *
   * # Units
   * Newton-metres per Newton of thrust (N*m/N, equivalent to metres).
   */
  double torque_coeff;
  /**
   * Maximum rotational speed of the motors.
   *
   * # Units
   * Revolutions per minute.
   */
  double max_rpm;
  /**
   * Throttle level required to maintain steady hover.
   *
   * Derived at load from mass, C_T, propeller diameter, max RPM and ISA
   * sea-level density (`aero::hover_throttle`); never read from JSON.
   *
   * # Units
   * Normalized range 0.0 to 1.0.
   */
  double hover_throttle;
} VehicleConfig;

/**
 * Telemetry data exposed exclusively for UE5 On-Screen Display (OSD) and debug visualization.
 *
 * Must be synced every physics tick.
 */
typedef struct DebugTelemetry {
  /**
   * Net thrust vector across all motors in world space.
   *
   * # Units
   * Newtons.
   */
  float net_thrust[3];
  /**
   * Aerodynamic drag force vector in world space.
   *
   * # Units
   * Newtons.
   */
  float aero_drag[3];
  /**
   * Gravity vector applied to the drone.
   *
   * # Units
   * Newtons.
   */
  float gravity[3];
  /**
   * Resulting net force vector.
   *
   * # Units
   * Newtons.
   */
  float net_force[3];
  /**
   * Individual motor thrust outputs.
   *
   * # Units
   * Newtons.
   */
  float motor_thrusts[4];
  /**
   * Individual motor RPMs.
   *
   * # Units
   * Revolutions per minute.
   */
  float motor_rpms[4];
  /**
   * Temperature of each motor winding.
   *
   * # Units
   * Degrees Celsius.
   */
  float motor_temperatures_c[4];
  /**
   * Structural failure margin from the FEA surrogate.
   *
   * # Units
   * Unitless multiplier. Values strictly less than 1.0 indicate structural failure.
   */
  float structural_safety_margin;
  /**
   * Aggregated envelope validation flag for all surrogate queries in the current tick.
   *
   * # Units
   * Unitless boolean flag (1 if all queries are valid, 0 if any query extrapolated).
   */
  int32_t is_validated_envelope;
  /**
   * Flag indicating if an un-trained stub model is currently driving the physics.
   *
   * # Units
   * Unitless boolean flag (1 if a surrogate handle passed to this step came
   * from a model whose `kind` is not `trained`, 0 otherwise).
   */
  int32_t stub_loaded;
  /**
   * Source of aerodynamic data. 0 = none (step rejected), 1 = analytic model, no surrogate
   * handle was passed, 2 = surrogate, 3 = a surrogate handle was passed but its query FAILED
   * and the analytic model was used instead (the two are never reported as the same value).
   */
  int32_t aero_source;
  /**
   * Source of structural data. 0 = none (no handle passed, or step rejected), 2 = surrogate,
   * 3 = a surrogate handle was passed but its query FAILED (margin is NaN). There is no
   * analytic structural fallback, so 1 is not used.
   */
  int32_t fea_source;
  /**
   * Copy of the caller's `FaultLatch`: 1 from the faulting step until the caller clears it.
   * Data produced while this is 1 is invalid.
   */
  int32_t physics_fault;
  /**
   * Count of fixed-timestep execution cycles dropped to prevent a "spiral of death".
   */
  int32_t dropped_time_events;
  /**
   * Indicates physics constants lacking validated sources are in use.
   */
  int32_t uncalibrated_flags;
  /**
   * 1 only when every thermal constant has a sourced, verified value. 0 means the thermal
   * model is UNVALIDATED: `motor_temperatures_c` and `thermal_over_temp` are arithmetic on
   * ASSUMED constants (item B-4) and must not be used as evidence. Always 0 until B-4 closes.
   */
  int32_t thermal_valid;
  /**
   * 1 if any motor winding temperature exceeds `thermal::MOTOR_TEMP_LIMIT_C` after this step
   * (limit is a placeholder, TODO(HUMAN-VERIFY)); 0 otherwise. Only meaningful when
   * `thermal_valid` is 1.
   */
  int32_t thermal_over_temp;
} DebugTelemetry;

/**
 * Caller-owned latched fault flag for `ffi_step_physics` (WP-4.4).
 *
 * The caller allocates it, zero-initializes it, and passes it to every step.
 * Rust sets `physics_fault` to 1 when a step produces a NaN or Inf state and
 * NEVER clears it. Only the caller clears it, on purpose (for example at the
 * start of a new run). Rust holds no copy, so two vehicles or two tests do
 * not share fault state.
 *
 * Data produced while the latch is set is invalid. It must not be used as
 * validation or pass/fail evidence.
 */
typedef struct FaultLatch {
  /**
   * 0 = no fault latched, 1 = a NaN/Inf divergence occurred since the caller last cleared it.
   */
  int32_t physics_fault;
} FaultLatch;

/**
 * Standardized return payload for any trained surrogate model query.
 */
typedef struct SurrogateQueryResult {
  /**
   * The primary output of the surrogate model (e.g., drag vector or scalar safety margin).
   *
   * # Units
   * Context-dependent based on the specific model queried.
   */
  float predicted_values[3];
  /**
   * Statistical confidence metric from the Gaussian Process Regression.
   *
   * # Units
   * Context-dependent variance.
   */
  float uncertainty;
  /**
   * Evaluates if the queried condition fell within the model's training data envelope.
   *
   * # Units
   * Unitless boolean flag (1 for true, 0 for false).
   */
  int32_t in_validated_envelope;
} SurrogateQueryResult;

/**
 * Motor, ESC, battery and thermal parameters of the powertrain (interface v13).
 *
 * Loaded from `powertrain_config.json` (written by `tools/parts/build_vehicle.py`) with
 * `ffi_load_powertrain_config`. Separate from `VehicleConfig` on purpose: `vehicle_config.json` is
 * hashed into every surrogate model, so it must not change when the powertrain model does.
 * Every value is SI; the unit is in the field name. See `powertrain.rs` for the model.
 */
typedef struct PowertrainConfig {
  /**
   * Motor velocity constant, rpm per volt (no-load).
   */
  double motor_kv_rpm_per_v;
  /**
   * Motor phase-to-phase winding resistance, ohm.
   */
  double motor_resistance_ohm;
  /**
   * Motor no-load current, A (iron + friction loss = this times the back-EMF).
   */
  double motor_no_load_current_a;
  /**
   * Motor current rating, A (flag only; the model does not limit current).
   */
  double motor_max_current_a;
  /**
   * Motor winding temperature limit, deg C (flag only).
   */
  double motor_winding_temp_limit_c;
  /**
   * Propeller diameter, m. Must equal `VehicleConfig::prop_diameter_m`.
   */
  double prop_diameter_m;
  /**
   * Static thrust coefficient (unitless). Must equal `VehicleConfig::thrust_coeff`.
   */
  double prop_ct;
  /**
   * Static power coefficient (unitless).
   */
  double prop_cp;
  /**
   * ESC efficiency, unitless in (0, 1].
   */
  double esc_efficiency;
  /**
   * ESC continuous current rating, A (flag only).
   */
  double esc_max_current_a;
  /**
   * ESC temperature limit, deg C (flag only).
   */
  double esc_temp_limit_c;
  /**
   * Battery capacity, Ah.
   */
  double battery_capacity_ah;
  /**
   * Battery pack internal resistance, ohm.
   */
  double battery_internal_resistance_ohm;
  /**
   * Battery cutoff voltage per cell, V.
   */
  double battery_cutoff_v_per_cell;
  /**
   * Battery continuous current rating (C rating x capacity), A (flag only).
   */
  double battery_max_continuous_a;
  /**
   * Battery temperature limit, deg C (flag only).
   */
  double battery_temp_limit_c;
  /**
   * Thermal resistance motor winding to case, K/W.
   */
  double motor_r_winding_case_k_per_w;
  /**
   * Thermal resistance motor case to ambient in still air, K/W.
   */
  double motor_r_case_ambient_k_per_w;
  /**
   * Heat capacity of the motor winding, J/K.
   */
  double motor_c_winding_j_per_k;
  /**
   * Heat capacity of the motor case (stator, bell, magnets), J/K.
   */
  double motor_c_case_j_per_k;
  /**
   * Thermal resistance ESC to ambient in still air, K/W.
   */
  double esc_r_ambient_k_per_w;
  /**
   * Heat capacity of the ESC, J/K.
   */
  double esc_c_j_per_k;
  /**
   * Thermal resistance battery to ambient in still air, K/W.
   */
  double battery_r_ambient_k_per_w;
  /**
   * Heat capacity of the battery, J/K.
   */
  double battery_c_j_per_k;
  /**
   * Air speed at which forced convection doubles the cooling (`R_eff = R / (1 + (v/v_ref)^0.8)`), m/s.
   */
  double cooling_ref_airspeed_mps;
  /**
   * Open-circuit-voltage table: state of charge, strictly increasing from 0 to 1 (first `ocv_points` entries).
   */
  double ocv_soc[MAX_OCV_POINTS];
  /**
   * Open-circuit voltage per cell at `ocv_soc`, V, non-decreasing.
   */
  double ocv_v_per_cell[MAX_OCV_POINTS];
  /**
   * Number of cells in series (1..=14).
   */
  int32_t battery_cells;
  /**
   * Number of used entries in the OCV table (2..=16).
   */
  int32_t ocv_points;
  /**
   * 1 only if every thermal and electrical input is MEASURED or CITED. The builder writes 0 while any input is ASSUMED.
   */
  int32_t parameters_verified;
  /**
   * Reserved, must be 0.
   */
  int32_t reserved;
  /**
   * Speed of each propeller curve, rpm, strictly increasing (first `prop_curve_count` entries). Used when `prop_model` is 1.
   */
  double prop_curve_rpm[MAX_PROP_CURVES];
  /**
   * Advance ratio `J = V_axial / (n D)` of each curve point, strictly increasing from exactly 0.
   */
  double prop_j[MAX_PROP_CURVES][MAX_PROP_POINTS];
  /**
   * Thrust coefficient `Ct` at each curve point (may be negative at high J).
   */
  double prop_ct_table[MAX_PROP_CURVES][MAX_PROP_POINTS];
  /**
   * Power coefficient `Cp` at each curve point, > 0.
   */
  double prop_cp_table[MAX_PROP_CURVES][MAX_PROP_POINTS];
  /**
   * Number of used points of each curve (2..=48).
   */
  int32_t prop_curve_points[MAX_PROP_CURVES];
  /**
   * Number of used curves (1..=8 when `prop_model` is 1, else 0).
   */
  int32_t prop_curve_count;
  /**
   * 0: constant `prop_ct`/`prop_cp` (the static fit). 1: the Ct/Cp(J, rpm) table.
   */
  int32_t prop_model;
} PowertrainConfig;

/**
 * Evolving powertrain state, owned by the caller and updated by `ffi_step_powertrain` (interface v13).
 */
typedef struct PowertrainState {
  /**
   * Battery state of charge, unitless in [0, 1].
   */
  double soc;
  /**
   * Charge drawn from the battery since the start, Ah.
   */
  double charge_used_ah;
  /**
   * Battery current of the previous step, A (feeds the voltage sag of the next step).
   */
  double battery_current_a;
  /**
   * Battery temperature, deg C.
   */
  double battery_temp_c;
  /**
   * Time the loaded pack voltage has been continuously below the cutoff, s.
   */
  double time_below_cutoff_s;
  /**
   * Motor winding temperatures, deg C.
   */
  double motor_winding_c[4];
  /**
   * Motor case temperatures, deg C.
   */
  double motor_case_c[4];
  /**
   * ESC temperatures, deg C.
   */
  double esc_temp_c[4];
  /**
   * Total time during which at least one propeller was outside the measured Ct/Cp data, s.
   */
  double prop_extrapolated_time_s;
} PowertrainState;

/**
 * Result of a stateless motor + propeller bench point (`ffi_powertrain_bench_point`), interface v13.
 */
typedef struct PowertrainBenchOut {
  /**
   * Motor speed, rpm.
   */
  double rpm;
  /**
   * Motor current, A.
   */
  double motor_current_a;
  /**
   * Propeller torque, N*m.
   */
  double torque_nm;
  /**
   * Propeller thrust, N.
   */
  double thrust_n;
  /**
   * Shaft power, W.
   */
  double shaft_w;
  /**
   * Motor electrical input power, W.
   */
  double motor_electrical_w;
  /**
   * Copper loss, W.
   */
  double copper_loss_w;
  /**
   * No-load loss, W.
   */
  double noload_loss_w;
  /**
   * ESC loss, W.
   */
  double esc_loss_w;
  /**
   * Current drawn from the pack, A.
   */
  double battery_current_a;
  /**
   * Power drawn from the pack, W.
   */
  double battery_power_w;
  /**
   * Shaft power over pack power, unitless.
   */
  double system_efficiency;
  /**
   * Advance ratio used.
   */
  double advance_ratio;
  /**
   * Thrust coefficient used.
   */
  double ct;
  /**
   * Power coefficient used.
   */
  double cp;
  /**
   * 1 if the propeller was outside the measured Ct/Cp data.
   */
  int32_t extrapolated;
  /**
   * Reserved, 0.
   */
  int32_t reserved;
} PowertrainBenchOut;

/**
 * Propeller coefficients at one operating point (`ffi_propeller_coefficients`), interface v14.
 */
typedef struct PropCoefficients {
  /**
   * Thrust coefficient.
   */
  double ct;
  /**
   * Power coefficient.
   */
  double cp;
  /**
   * 1 if (J, rpm) is outside the measured data and the nearest edge value was used.
   */
  int32_t extrapolated;
  /**
   * Reserved, 0.
   */
  int32_t reserved;
} PropCoefficients;

/**
 * Per-step powertrain results (interface v13). All values are for the step just taken.
 * A rejected step overwrites it with NaN floats and all flags 0 (`step_valid` 0).
 */
typedef struct PowertrainTelemetry {
  /**
   * Motor current, A.
   */
  double motor_current_a[4];
  /**
   * Voltage at the motor terminals (duty x pack voltage), V.
   */
  double motor_voltage_v[4];
  /**
   * Motor speed, rpm.
   */
  double motor_rpm[4];
  /**
   * Propeller torque, N*m.
   */
  double motor_torque_nm[4];
  /**
   * Propeller thrust, N.
   */
  double motor_thrust_n[4];
  /**
   * Copper loss, W.
   */
  double motor_copper_loss_w[4];
  /**
   * No-load (iron + friction) loss, W.
   */
  double motor_noload_loss_w[4];
  /**
   * ESC loss, W.
   */
  double esc_loss_w[4];
  /**
   * Motor winding temperature after the step, deg C.
   */
  double motor_winding_c[4];
  /**
   * Motor case temperature after the step, deg C.
   */
  double motor_case_c[4];
  /**
   * ESC temperature after the step, deg C.
   */
  double esc_temp_c[4];
  /**
   * Loaded pack voltage, V.
   */
  double battery_voltage_v;
  /**
   * Open-circuit pack voltage, V.
   */
  double battery_ocv_v;
  /**
   * Pack current, A.
   */
  double battery_current_a;
  /**
   * Power drawn from the pack, W.
   */
  double battery_power_w;
  /**
   * State of charge after the step, unitless.
   */
  double battery_soc;
  /**
   * Battery temperature after the step, deg C.
   */
  double battery_temp_c;
  /**
   * Battery heating, W.
   */
  double battery_heat_w;
  /**
   * Charge drawn since the start, Ah.
   */
  double charge_used_ah;
  /**
   * Time continuously below the cutoff voltage, s.
   */
  double time_below_cutoff_s;
  /**
   * 1 if this struct holds the results of a successful step.
   */
  int32_t step_valid;
  /**
   * 1 if the loaded pack voltage is below cells x cutoff voltage (instantaneous; the step keeps running).
   */
  int32_t battery_cutoff;
  /**
   * 1 if the pack current exceeds its continuous rating.
   */
  int32_t battery_over_current;
  /**
   * 1 if the battery temperature exceeds its limit.
   */
  int32_t battery_over_temp;
  /**
   * 1 if the state of charge reached 0 and was held there.
   */
  int32_t battery_soc_clamped;
  /**
   * Bit i set: motor i current above its rating.
   */
  int32_t motor_over_current_mask;
  /**
   * Bit i set: motor i winding above its temperature limit.
   */
  int32_t motor_over_temp_mask;
  /**
   * Bit i set: ESC i input current above its rating.
   */
  int32_t esc_over_current_mask;
  /**
   * Bit i set: ESC i above its temperature limit.
   */
  int32_t esc_over_temp_mask;
  /**
   * Copy of `PowertrainConfig::parameters_verified` (0 while any input is ASSUMED).
   */
  int32_t thermal_valid;
  /**
   * Bit i set: propeller i is outside the measured Ct/Cp data (held at the nearest edge value). Always 0 with `prop_model` 0.
   */
  int32_t prop_extrapolated;
  /**
   * Reserved, 0.
   */
  int32_t reserved0;
  /**
   * Reserved, 0.
   */
  int32_t reserved1;
  /**
   * Reserved, 0.
   */
  int32_t reserved2;
  /**
   * Advance ratio of each propeller (0 for a rotor at rest).
   */
  double motor_advance_ratio[4];
  /**
   * Thrust coefficient used for each propeller (0 for a rotor at rest).
   */
  double motor_ct[4];
  /**
   * Power coefficient used for each propeller (0 for a rotor at rest).
   */
  double motor_cp[4];
} PowertrainTelemetry;



#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

/**
 * Returns the current layout version to UE5 to prevent memory corruption on mismatch.
 *
 * # Safety
 * Safe to call at any time.
 */
int32_t ffi_get_interface_version(void);

/**
 * Allows UE5 to verify the byte size of DroneState during module initialization.
 *
 * # Safety
 * Safe to call at any time.
 */
int32_t ffi_get_drone_state_size(void);

/**
 * Instantiates a default, zeroed drone state with an identity quaternion.
 * If a panic is caught the state is all NaN, which the next step rejects loudly (code 8).
 *
 * # Safety
 * Safe to call at any time.
 */
struct DroneState ffi_create_default_drone_state(void);

/**
 * Resets the provided state to default values.
 *
 * # Returns
 * `0` ok, `1` null pointer, `99` panic caught.
 *
 * # Safety
 * * `state` must be a valid, aligned, and mutable pointer to a DroneState instance.
 * * The memory must not be concurrently accessed by another thread.
 */
int32_t ffi_reset_drone_state(struct DroneState *state);

/**
 * Main execution block for the fixed-timestep RK4 physics pipeline.
 *
 * Executes modules in strict order: Aero -> Structural -> Thermal -> Mixer -> Integrator.
 *
 * # Wind (interface v11)
 * `wind` is the ambient air velocity in the NED world frame, m/s, all zeros = still air. Only the
 * aerodynamic drag uses it, through the air-relative velocity `v_air = v_ground - wind`. Thrust,
 * thermal convection (ground speed) and everything else are unchanged.
 *
 * # Environment (interface v12)
 * `environment` sets the atmosphere: `site_altitude_m` (height of the NED origin above sea level) and
 * `temp_offset_c` (air temperature minus the ISA standard day). Air density for thrust and for the
 * analytic drag fallback comes from `aero::air_density`; the motor thermal ambient becomes
 * `thermal::AMBIENT_TEMP + temp_offset_c`. All zeros is the v11 behaviour bit for bit. A surrogate
 * aero answer ignores the environment (it was trained at one density). Humidity is not modelled.
 *
 * # Units
 * * `dt` - Timestep in seconds.
 *
 * # Returns
 * * `0` - Success
 * * `1` - Null pointer (nothing is written)
 * * `6` - Aero model error (drag or thrust), including altitude outside ISA 0..=11000 m
 * * `7` - Invalid dt (NaN, <= 0 or > 0.01)
 * * `8` - Divergence or invalid input: a NaN/Inf in `config`, `thermal_state.motor_temp_c` or
 *   `state.orientation`; `config.mass_kg <= 0`, an inertia <= 0 or `max_rpm < 0` (re-checked
 *   every step because C++ may fill the struct by hand); or a NaN/Inf in the new state or new
 *   thermal state. `state` and `thermal_state` are left/restored to their previous values and
 *   `fault_latch` is set to 1.
 * * `9` - A non-null surrogate handle is not live (never loaded, already unloaded, foreign),
 *   or `aero_handle` and `fea_handle` are the same non-null handle
 * * `11` - `state.orientation` is finite but its norm differs from 1 by more than
 *   `types::QUAT_NORM_TOLERANCE` (rejected, never silently rescaled)
 * * `12` - `wind.wind_ned_mps` has a NaN/Inf component or a component with magnitude above
 *   `types::MAX_WIND_COMPONENT_MPS` (30 m/s). Rejected before any computation: NOTHING is written.
 * * `13` - `environment` has a NaN/Inf field, `site_altitude_m` outside 0..=5000, or
 *   `|temp_offset_c| > 40`. Rejected before any computation, after the wind check: NOTHING is written.
 *   (A total height `site_altitude_m` + vehicle altitude outside 0..=11000 m is code 6.)
 * * `98` - an internal lock was poisoned and recovered; nothing was done, call again
 * * `99` - a panic was caught, or a handle was faulted by an earlier panic; the run is invalid
 *
 * # Behavior on a failed step
 * * `state` and `thermal_state` are always unchanged.
 * * Codes 1, 7, 9, 11, 12, 13, 98 and 99 are rejected before any computation: NOTHING is written
 *   (telemetry and latch keep their previous values).
 * * Codes 6 and 8 fail during the tick: `out_telemetry` is overwritten with "no data" (margin
 *   NaN, `is_validated_envelope` 0, sources 0, `thermal_valid` 0, `physics_fault` = latch value,
 *   so it is never left holding the previous tick's numbers).
 * * `fault_latch` is set to 1 ONLY by code 8. Code 6 (altitude outside the ISA range) is a
 *   scenario error, not a divergence, and leaves the latch as it was.
 *
 * # Fault latch
 * `fault_latch` is caller-owned and must be zero-initialized by the caller. Rust sets it to 1
 * on a code-8 step and never clears it; later steps keep running from the restored state and
 * report `physics_fault = 1` in telemetry. The caller clears it on purpose (for example at
 * the start of a new run). Data produced while it is set is invalid and must not be used as
 * validation or pass/fail evidence.
 *
 * # Safety
 * * `state` must be a valid, aligned, mutable pointer to a DroneState.
 * * `thermal_state` must be a valid, aligned, mutable pointer to a ThermalState.
 * * `controls` must be a valid, aligned, immutable pointer to ControlInputs.
 * * `wind` must be a valid, aligned, immutable pointer to a `WindInput` (null gives code 1).
 * * `environment` must be a valid, aligned, immutable pointer to an `EnvironmentInput` (null gives code 1).
 * * `aero_handle` and `fea_handle` may be null. If non-null, must be valid SurrogateHandles.
 * * `config` must be a valid, aligned, immutable pointer to a VehicleConfig.
 * * `out_telemetry` must be a valid, aligned, mutable pointer to a DebugTelemetry struct.
 * * `fault_latch` must be a valid, aligned, mutable pointer to a FaultLatch.
 * * Pointers must not alias or be subject to concurrent mutation.
 */
int32_t ffi_step_physics(struct DroneState *state,
                         struct ThermalState *thermal_state,
                         const struct ControlInputs *controls,
                         const struct WindInput *wind,
                         const struct EnvironmentInput *environment,
                         struct SurrogateHandle *aero_handle,
                         struct SurrogateHandle *fea_handle,
                         const struct VehicleConfig *config,
                         struct DebugTelemetry *out_telemetry,
                         struct FaultLatch *fault_latch,
                         double dt);

/**
 * Loads a trained ONNX surrogate model and returns an opaque handle to C++.
 *
 * # Returns
 * * `0` - Success
 * * `1` - Null pointer, invalid UTF-8 path, or the path is not a readable file
 * * `2` - Missing provenance metadata
 * * `3` - Geometry hash mismatch
 * * `4` - Engine/ONNX initialization failure (including a missing ONNX Runtime library)
 * * `5` - Stub model rejected (allow_stub_models feature not enabled)
 * * `6` - Model inputs/outputs are not the supported shapes
 * * `8` - Vehicle config hash mismatch (model `config_hash` differs from the compiled `vehicle_config.json`)
 * * `98` - Handle registry lock was poisoned and recovered; the model is dropped, `out_handle`
 *   is untouched, call again (replaces the old code 7)
 * * `99` - A panic was caught; `out_handle` is untouched
 *
 * A model whose `kind` is not `trained` (a stub) loads only with the `allow_stub_models`
 * feature, and its queries always report `in_validated_envelope = 0`.
 *
 * # Safety
 * * `path` must be a valid, null-terminated C string.
 * * `out_handle` must be a valid, aligned, mutable pointer to a pointer.
 */
int32_t ffi_load_surrogate_model(const char *path,
                                 struct SurrogateHandle **out_handle);

/**
 * Queries a loaded surrogate model.
 *
 * `input_count` must equal the model's declared input width; no slice is built from it
 * before that check, so a huge value cannot read out of bounds. The registry lock is NOT held
 * during inference, only this handle's own lock.
 *
 * # Returns
 * * `0` - Success
 * * `1` - Null pointer or negative `input_count`
 * * `2` - `handle` is not a live handle (never loaded, already unloaded, foreign)
 * * `3` - `input_count` differs from the model's input width
 * * `4` - Engine/ONNX inference error
 * * `5` - An input value is NaN or infinite
 * * `6` - Malformed model output (wrong count, non-finite, envelope flag not 0/1)
 * * `98` - Registry lock was poisoned and recovered; nothing was done, call again
 * * `99` - A panic was caught, or this handle was faulted by an earlier panic (unload it)
 *
 * A stub model always reports `in_validated_envelope = 0`.
 *
 * # Safety
 * * `handle` must be a pointer created by `ffi_load_surrogate_model` (checked against the
 *   live registry before use).
 * * `inputs` must point to an array of exactly `input_count` floats.
 * * `out_result` must be a valid, aligned pointer.
 */
int32_t ffi_query_surrogate(struct SurrogateHandle *handle,
                            const float *inputs,
                            int32_t input_count,
                            struct SurrogateQueryResult *out_result);

/**
 * Unloads a surrogate model and frees its memory.
 *
 * Waits for any query or step still using the handle, then frees it.
 *
 * # Returns
 * * `0` - Freed
 * * `1` - Null pointer
 * * `2` - `handle` is not live (never loaded or already unloaded); nothing is freed
 * * `98` - Registry lock was poisoned and recovered; nothing was freed, call again
 * * `99` - A panic was caught while freeing
 *
 * # Safety
 * * `handle` must be a pointer created by `ffi_load_surrogate_model` (checked against the
 *   live registry before it is freed).
 * * The caller must not use `handle` after this function returns 0.
 */
int32_t ffi_unload_surrogate_model(struct SurrogateHandle *handle);

/**
 * Loads a vehicle configuration file.
 *
 * # Returns
 * * `0` - Success
 * * `1` - Null pointer or Invalid UTF-8 path
 * * `2` - File IO Error
 * * `3` - JSON Parse Error
 * * `4` - Invalid Mass
 * * `5` - Invalid Inertia
 * * `6` - Invalid Motor Spin
 * * `7` - Invalid Motor Position (non-finite)
 * * `8` - Invalid Propeller Diameter
 * * `9` - Invalid Thrust Coefficient
 * * `10` - Invalid Torque Coefficient
 * * `11` - Invalid Max RPM
 * * `12` - Invalid Hover Throttle (must be in (0, 1])
 * * `99` - A panic was caught
 *
 * # Safety
 * * `path` must be a valid, null-terminated C string.
 * * `out_config` must be a valid, aligned, mutable pointer to a VehicleConfig.
 */
int32_t ffi_load_vehicle_config(const char *path, struct VehicleConfig *out_config);

/**
 * Loads and validates a powertrain configuration file (`powertrain_config.json`).
 *
 * # Returns
 * * `0` - Success
 * * `1` - Null pointer or invalid UTF-8 path
 * * `2` - File IO error
 * * `3` - JSON parse error (malformed, missing or unknown key, OCV table longer than 16 points)
 * * `4` - A value is out of range or the OCV table is malformed (see `powertrain::validate_powertrain_config`)
 * * `99` - A panic was caught
 *
 * # Safety
 * * `path` must be a valid, null-terminated C string.
 * * `out_config` must be a valid, aligned, mutable pointer to a `PowertrainConfig`.
 */
int32_t ffi_load_powertrain_config(const char *path,
                                   struct PowertrainConfig *out_config);

/**
 * Creates a fresh `PowertrainState`: state of charge `soc`, every thermal node at `ambient_c`, no current history.
 *
 * # Returns
 * * `0` - Success
 * * `1` - Null pointer
 * * `4` - `config` fails `validate_powertrain_config`
 * * `5` - `soc` outside [0, 1], or `ambient_c` not finite / outside -100..=2000 deg C. `out_state` is untouched.
 * * `99` - A panic was caught
 *
 * # Safety
 * * `config` must be a valid, aligned, immutable pointer to a `PowertrainConfig`.
 * * `out_state` must be a valid, aligned, mutable pointer to a `PowertrainState`.
 */
int32_t ffi_init_powertrain_state(const struct PowertrainConfig *config,
                                  double ambient_c,
                                  double soc,
                                  struct PowertrainState *out_state);

/**
 * Stateless bench point: one motor + propeller + ESC at a fixed pack voltage and duty, in air of density `rho_kgpm3`, with the
 * vehicle moving `axial_inflow_mps` along the rotor axis (0 for a static bench; the scenario of a manufacturer's thrust table:
 * voltage, propeller, throttle -> current, thrust, rpm).
 *
 * # Returns
 * * `0` - Success
 * * `1` - Null pointer
 * * `4` - `config` fails `validate_powertrain_config`
 * * `5` - `duty` outside [0, 1], `v_pack_v` negative, `rho_kgpm3` not positive, or `axial_inflow_mps` not finite
 * * `6` - The solve produced a non-finite value. `out` is untouched on every nonzero code.
 * * `99` - A panic was caught
 *
 * # Safety
 * * `config` must be a valid, aligned, immutable pointer to a `PowertrainConfig`.
 * * `out` must be a valid, aligned, mutable pointer to a `PowertrainBenchOut`.
 */
int32_t ffi_powertrain_bench_point(const struct PowertrainConfig *config,
                                   double duty,
                                   double v_pack_v,
                                   double rho_kgpm3,
                                   double axial_inflow_mps,
                                   struct PowertrainBenchOut *out);

/**
 * Propeller coefficients `Ct` and `Cp` at advance ratio `j` and shaft speed `rpm` (the same lookup the step uses).
 * With `prop_model` 0 they are the constants; with 1 they come from the UIUC table, the nearest edge value is held outside the
 * measured data and `extrapolated` is set to 1.
 *
 * # Returns
 * * `0` - Success
 * * `1` - Null pointer
 * * `4` - `config` fails `validate_powertrain_config`
 * * `5` - `j` or `rpm` not finite. `out` is untouched on every nonzero code.
 * * `99` - A panic was caught
 *
 * # Safety
 * * `config` must be a valid, aligned, immutable pointer to a `PowertrainConfig`.
 * * `out` must be a valid, aligned, mutable pointer to a `PropCoefficients`.
 */
int32_t ffi_propeller_coefficients(const struct PowertrainConfig *config,
                                   double j,
                                   double rpm,
                                   struct PropCoefficients *out);

/**
 * Fixed-timestep RK4 step with the powertrain model (interface v13): the motors' speed, thrust, current and temperatures
 * come from the motor/propeller torque balance at the voltage the battery can supply, instead of `throttle x max_rpm`.
 *
 * Order: aero -> mixer (throttle = ESC duty) -> powertrain (battery, ESC, motors, thermal) -> structural -> forces -> integrator.
 * `VehicleConfig::max_rpm` and `hover_throttle` are NOT used for the motors here (hover duty differs from `hover_throttle`;
 * read it from the build sidecar). Cooling uses the AIR-relative speed (ground velocity minus wind).
 *
 * # Returns
 * * `0` - Success
 * * `1` - Null pointer (`aero_handle` and `fea_handle` may be null)
 * * `6` - Aero model error, altitude outside the ISA range, or a non-finite powertrain result (16 is used for the latter)
 * * `7` - Invalid dt (NaN, <= 0 or > 0.01)
 * * `8` - Divergence or invalid input (as in `ffi_step_physics`); latch set
 * * `9` / `11` / `12` / `13` / `98` / `99` - as in `ffi_step_physics`
 * * `14` - `powertrain` is invalid, contradicts `config` (propeller diameter, Ct, kappa), or the thermal network is too
 *   fast for `dt` (`dt > 0.2 x tau_min`). Nothing is written.
 * * `15` - `powertrain_state` is invalid (SOC outside [0, 1], non-finite or implausible temperature, negative charge). Nothing is written.
 * * `16` - A powertrain result was not finite. `out_telemetry` and `out_powertrain` are reset to "no data"; the latch is untouched.
 *
 * # Behavior on a failed step
 * `state` and `powertrain_state` are always unchanged. Codes 1, 7, 9, 11, 12, 13, 14, 15, 98, 99 write nothing. Codes 6, 16
 * and 8 overwrite both telemetry structs with "no data" (NaN powertrain floats, `step_valid` 0); only 8 sets the latch.
 *
 * # Safety
 * * `state`, `powertrain_state`, `out_telemetry`, `out_powertrain`, `fault_latch` must be valid, aligned, mutable pointers.
 * * `controls`, `wind`, `environment`, `config`, `powertrain` must be valid, aligned, immutable pointers.
 * * `aero_handle` and `fea_handle` may be null; if non-null they must be valid `SurrogateHandle`s.
 * * Pointers must not alias or be subject to concurrent mutation.
 */
int32_t ffi_step_powertrain(struct DroneState *state,
                            struct PowertrainState *powertrain_state,
                            const struct ControlInputs *controls,
                            const struct WindInput *wind,
                            const struct EnvironmentInput *environment,
                            struct SurrogateHandle *aero_handle,
                            struct SurrogateHandle *fea_handle,
                            const struct VehicleConfig *config,
                            const struct PowertrainConfig *powertrain,
                            struct DebugTelemetry *out_telemetry,
                            struct PowertrainTelemetry *out_powertrain,
                            struct FaultLatch *fault_latch,
                            double dt);

#ifdef __cplusplus
}  // extern "C"
#endif  // __cplusplus

#endif  /* DRONE_FFI_H */
