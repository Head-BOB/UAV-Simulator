// types.rs

/// Represents the drone's current physical condition.
/// #[repr(C)] guarantees this struct has the exact same memory layout as a C/C++ struct.
#[derive(Copy, Clone)]
#[repr(C)]
pub struct DroneState {
    /// World-space X, Y, Z position of the center of gravity (meters)
    pub position: [f32; 3],

    /// World-space linear velocity (meters per second)
    pub velocity: [f32; 3],

    /// Rotation as a unit quaternion, stored in order (w, x, y, z)
    pub orientation: [f32; 4],

    /// Angular velocity in the body frame (radians per second)
    pub angular_velocity: [f32; 3],
}

/// Represents what the pilot/controller is commanding.
#[derive(Copy, Clone)]
#[repr(C)]
pub struct ControlInputs {
    /// Commanded throttle, range 0.0 (none) to 1.0 (full)
    pub throttle: f32,

    /// Commanded roll input, range -1.0 to 1.0
    pub roll: f32,

    /// Commanded pitch input, range -1.0 to 1.0
    pub pitch: f32,

    /// Commanded yaw input, range -1.0 to 1.0
    pub yaw: f32,
}

/// Telemetry data exposed exclusively for UE5 On-Screen Display (OSD) and debug visualization.
/// Must be synced every physics tick.
#[derive(Copy, Clone)]
#[repr(C)]
pub struct DebugTelemetry {
    /// Net thrust vector across all motors in world space (Newtons)
    pub net_thrust: [f32; 3],

    /// Aerodynamic drag force vector in world space (Newtons)
    pub aero_drag: [f32; 3],

    /// Gravity vector applied to the drone (Newtons)
    pub gravity: [f32; 3],

    /// Resulting net force vector (Newtons)
    pub net_force: [f32; 3],

    /// Individual motor thrust outputs (Newtons)
    pub motor_thrusts: [f32; 4],

    /// Individual motor RPMs
    pub motor_rpms: [f32; 4],
}

impl DebugTelemetry {
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