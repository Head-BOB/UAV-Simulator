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