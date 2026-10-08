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

    /// Spin direction of each motor (+1.0 or -1.0).
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

    /// Propeller torque coefficient.
    ///
    /// # Units
    /// Unitless.
    pub torque_coeff: f64,

    /// Maximum rotational speed of the motors.
    ///
    /// # Units
    /// Revolutions per minute.
    pub max_rpm: f64,

    /// Throttle level required to maintain steady hover.
    ///
    /// # Units
    /// Normalized range 0.0 to 1.0.
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
    /// Unitless boolean flag (1 if stub loaded, 0 otherwise).
    pub stub_loaded: i32,

    /// Source of aerodynamic data. 0 = none, 1 = analytic fallback, 2 = surrogate.
    pub aero_source: i32,

    /// Source of structural data. 0 = none, 1 = analytic fallback, 2 = surrogate.
    pub fea_source: i32,

    /// Latched fault flag indicating a physics NaN or Inf divergence occurred.
    pub physics_fault: i32,

    /// Count of fixed-timestep execution cycles dropped to prevent a "spiral of death".
    pub dropped_time_events: i32,

    /// Indicates physics constants lacking validated sources are in use.
    pub uncalibrated_flags: i32,
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
            structural_safety_margin: 10.0,
            is_validated_envelope: 1,
            stub_loaded: 0,
            aero_source: 0,
            fea_source: 0,
            physics_fault: 0,
            dropped_time_events: 0,
            uncalibrated_flags: 0,
        }
    }
}

impl Default for DebugTelemetry {
    fn default() -> Self {
        Self::new()
    }
}

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
}
