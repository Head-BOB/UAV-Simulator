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

    /// Rotation as a unit quaternion, stored in order (w, x, y, z).
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
}

impl DebugTelemetry {
    /// Instantiates a default, zeroed telemetry block.
    pub const fn new() -> Self {
        Self {
            net_thrust: [0.0, 0.0, 0.0],
            aero_drag: [0.0, 0.0, 0.0],
            gravity: [0.0, 0.0, 0.0],
            net_force: [0.0, 0.0, 0.0],
            motor_thrusts: [0.0, 0.0, 0.0, 0.0],
            motor_rpms: [0.0, 0.0, 0.0, 0.0],
        }
    }
}
