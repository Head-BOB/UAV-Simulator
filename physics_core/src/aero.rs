use crate::types::DroneState;

pub fn get_drag(_state: &DroneState, _altitude: f32) -> [f32; 3] {
    // TODO(Dev B): Replace with ISA model and 6-axis drag tensor
    [0.0, 0.0, 0.0]
}

pub fn get_thrust(_throttle: f32) -> f32 {
    // TODO(Dev B): Replace with propeller thrust model
    0.0
}