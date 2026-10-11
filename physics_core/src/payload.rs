// Tier: F
//! Payload mass properties for the test campaign (P4): a point mass added to the vehicle at a body-frame position.
//!
//! Body frame X forward / Y right / Z down, origin at the vehicle's centre of mass (ADR-001 body axes). The result is expressed about
//! the NEW centre of mass:
//! * mass `M = m + m_p`;
//! * centre-of-mass shift `c = m_p r / M` (body frame, from the old CG);
//! * inertia (diagonal only, parallel-axis theorem): `I' = I + m (|c|^2 E - c c^T) + m_p (|d|^2 E - d d^T)` with `d = r - c`;
//!   the off-diagonal terms are dropped (`VehicleConfig` holds a diagonal inertia): APPROXIMATION, stated in the campaign manifest;
//! * motor positions `p_i - c`.
//!
//! The controller's hover duty scales with `sqrt(M / m)` (thrust ~ duty^2 around hover), see `PayloadResult::hover_duty_scale`.

use crate::types::VehicleConfig;

/// Smallest vehicle mass the payload code accepts, kg: the smallest positive normal `f64`. It only separates a usable mass from zero,
/// a negative or a non-finite value; it is a validity guard, not a mass of any vehicle.
pub const MIN_VEHICLE_MASS_KG: f64 = f64::MIN_POSITIVE;

/// Largest payload accepted, kg.
pub const MAX_PAYLOAD_KG: f64 = 50.0;
/// Largest |coordinate| of the payload position, m.
pub const MAX_PAYLOAD_POSITION_M: f64 = 2.0;

/// Why a payload was refused.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum PayloadError {
    /// Mass negative / non-finite / above `MAX_PAYLOAD_KG`, or a position coordinate non-finite or beyond `MAX_PAYLOAD_POSITION_M`.
    InvalidInput,
    /// The vehicle has a non-positive mass.
    InvalidVehicle,
}

/// The vehicle with the payload added.
#[derive(Copy, Clone)]
pub struct PayloadResult {
    /// New mass properties and motor positions (other fields unchanged; `hover_throttle` scaled by `hover_duty_scale`).
    pub vehicle: VehicleConfig,
    /// Centre-of-mass shift from the old CG, body frame, m.
    pub cg_shift_m: [f64; 3],
    /// `sqrt(M / m)`.
    pub hover_duty_scale: f64,
}

fn add_point_mass(inertia: [f64; 3], mass_kg: f64, v: [f64; 3]) -> [f64; 3] {
    let [x, y, z] = v;
    let [ixx, iyy, izz] = inertia;
    [
        ixx + mass_kg * (y * y + z * z),
        iyy + mass_kg * (x * x + z * z),
        izz + mass_kg * (x * x + y * y),
    ]
}

/// Range check of a payload on its own (no vehicle needed): mass finite and in `0..=MAX_PAYLOAD_KG`, every position coordinate finite
/// and within `MAX_PAYLOAD_POSITION_M`. The campaign conditions use this; `apply_payload` runs it first.
///
/// # Errors
/// `InvalidInput` when a value is out of range or not finite.
pub fn validate_payload(payload_kg: f64, position_m: [f64; 3]) -> Result<(), PayloadError> {
    let pos_ok = position_m
        .iter()
        .all(|c| c.is_finite() && c.abs() <= MAX_PAYLOAD_POSITION_M);
    let mass_ok = payload_kg.is_finite() && (0.0..=MAX_PAYLOAD_KG).contains(&payload_kg);
    if mass_ok && pos_ok {
        Ok(())
    } else {
        Err(PayloadError::InvalidInput)
    }
}

/// Adds a point-mass payload.
///
/// # Units
/// `payload_kg` in kg (0 ..= 50), `position_m` body frame metres from the vehicle CG (each |coordinate| <= 2 m).
///
/// # Errors
/// `InvalidInput` / `InvalidVehicle` as described on `PayloadError`.
pub fn apply_payload(
    vehicle: &VehicleConfig,
    payload_kg: f64,
    position_m: [f64; 3],
) -> Result<PayloadResult, PayloadError> {
    validate_payload(payload_kg, position_m)?;
    if !(vehicle.mass_kg.is_finite() && vehicle.mass_kg >= MIN_VEHICLE_MASS_KG) {
        return Err(PayloadError::InvalidVehicle);
    }
    let total = vehicle.mass_kg + payload_kg;
    let c = position_m.map(|r| payload_kg * r / total);
    let d = [
        position_m[0] - c[0],
        position_m[1] - c[1],
        position_m[2] - c[2],
    ];
    let with_vehicle = add_point_mass(vehicle.inertia_kgm2, vehicle.mass_kg, c);
    let inertia = add_point_mass(with_vehicle, payload_kg, d);
    let scale = (total / vehicle.mass_kg).sqrt();
    let mut v = *vehicle;
    v.mass_kg = total;
    v.inertia_kgm2 = inertia;
    v.motor_pos_m = vehicle
        .motor_pos_m
        .map(|p| [p[0] - c[0], p[1] - c[1], p[2] - c[2]]);
    v.hover_throttle = vehicle.hover_throttle * scale;
    Ok(PayloadResult {
        vehicle: v,
        cg_shift_m: c,
        hover_duty_scale: scale,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::controller::tests::ref_vehicle;

    fn near(a: f64, b: f64, tol: f64) -> bool {
        (a - b).abs() <= tol
    }

    #[test]
    fn test_payload_hand_values_for_half_a_kilo() {
        // m = 1.536 kg, payload 0.5 kg at r = (0.1, 0, 0.05): M = 2.036; c = 0.5 r / 2.036 = (0.0245580, 0, 0.0122790);
        // d = r - c = (0.0754420, 0, 0.0377210);
        // Ixx = 0.030135 + 1.536 (0 + 0.0122790^2) + 0.5 (0 + 0.0377210^2) = 0.0310780
        // Iyy = 0.03081 + 1.536 (0.0245580^2 + 0.0122790^2) + 0.5 (0.0754420^2 + 0.0377210^2) = 0.0355251
        // Izz = 0.055933 + 1.536 (0.0245580^2 + 0) + 0.5 (0.0754420^2 + 0) = 0.0597051
        let r = apply_payload(&ref_vehicle(), 0.5, [0.1, 0.0, 0.05]).unwrap();
        assert!(near(r.vehicle.mass_kg, 2.036, 1e-15));
        assert!(
            near(r.cg_shift_m[0], 0.024_557_956_777_996_07, 1e-12)
                && r.cg_shift_m[1] == 0.0
                && near(r.cg_shift_m[2], 0.012_278_978_388_998_04, 1e-12)
        );
        let i = r.vehicle.inertia_kgm2;
        assert!(
            near(i[0], 0.031_078_025_540_275_05, 1e-12)
                && near(i[1], 0.035_525_127_701_375_25, 1e-12)
                && near(i[2], 0.059_705_102_161_100_19, 1e-12)
        );
        // motor 0 at (0.2, 0.2, -0.052) moves to (0.2 - 0.0245580, 0.2, -0.052 - 0.0122790)
        let p = r.vehicle.motor_pos_m[0];
        assert!(
            near(p[0], 0.175_442_043_222_003_93, 1e-12)
                && near(p[1], 0.2, 1e-15)
                && near(p[2], -0.064_278_978_388_998_03, 1e-12)
        );
        // hover duty scales with sqrt(2.036 / 1.536) = 1.1513127
        assert!(near(r.hover_duty_scale, 1.151_312_656_637_341_5, 1e-12));
        assert!(near(
            r.vehicle.hover_throttle,
            0.398 * 1.151_312_656_637_341_5,
            1e-12
        ));
    }

    #[test]
    fn test_payload_zero_returns_the_vehicle_unchanged() {
        let v = ref_vehicle();
        let r = apply_payload(&v, 0.0, [0.3, -0.2, 0.1]).unwrap();
        assert_eq!(r.vehicle.mass_kg.to_bits(), v.mass_kg.to_bits());
        assert_eq!(
            r.vehicle.inertia_kgm2.map(f64::to_bits),
            v.inertia_kgm2.map(f64::to_bits)
        );
        assert_eq!(
            r.vehicle.motor_pos_m.map(|p| p.map(f64::to_bits)),
            v.motor_pos_m.map(|p| p.map(f64::to_bits))
        );
        assert_eq!(
            (r.hover_duty_scale, r.vehicle.hover_throttle.to_bits()),
            (1.0, v.hover_throttle.to_bits())
        );
        assert_eq!(r.cg_shift_m, [0.0; 3]);
    }

    #[test]
    fn test_payload_at_the_cg_adds_mass_only() {
        // a payload exactly at the CG: no shift, inertia unchanged (a point mass at the CG has no inertia about it)
        let v = ref_vehicle();
        let r = apply_payload(&v, 0.4, [0.0; 3]).unwrap();
        assert_eq!(r.cg_shift_m, [0.0; 3]);
        assert_eq!(
            r.vehicle.inertia_kgm2.map(f64::to_bits),
            v.inertia_kgm2.map(f64::to_bits)
        );
        assert!(near(r.vehicle.mass_kg, 1.936, 1e-15));
    }

    #[test]
    fn test_payload_inertia_about_the_new_cg_is_symmetric_in_sign() {
        // moving the payload to the mirror position (-x) mirrors the CG shift and leaves the diagonal inertia unchanged
        let a = apply_payload(&ref_vehicle(), 0.7, [0.15, 0.1, -0.05]).unwrap();
        let b = apply_payload(&ref_vehicle(), 0.7, [-0.15, -0.1, 0.05]).unwrap();
        assert!(
            near(a.vehicle.inertia_kgm2[1], b.vehicle.inertia_kgm2[1], 1e-15)
                && near(a.cg_shift_m[0], -b.cg_shift_m[0], 1e-15)
        );
    }

    #[test]
    fn test_payload_total_inertia_is_not_below_the_vehicle_alone() {
        // adding a point mass can only add inertia about the new CG (parallel-axis terms are non-negative)
        let v = ref_vehicle();
        for r in [[0.2, 0.0, 0.0], [0.0, -0.3, 0.1], [0.05, 0.05, 0.05]] {
            let p = apply_payload(&v, 1.0, r).unwrap();
            assert!(
                p.vehicle
                    .inertia_kgm2
                    .iter()
                    .zip(v.inertia_kgm2)
                    .all(|(n, o)| *n >= o)
            );
        }
    }

    #[test]
    fn test_payload_refusals() {
        let v = ref_vehicle();
        let bad = [
            (-0.1, [0.0; 3]),
            (f64::NAN, [0.0; 3]),
            (50.1, [0.0; 3]),
            (1.0, [2.01, 0.0, 0.0]),
            (1.0, [0.0, f64::INFINITY, 0.0]),
            (1.0, [0.0, 0.0, f64::NAN]),
        ];
        for (m, p) in bad {
            assert_eq!(
                apply_payload(&v, m, p).err(),
                Some(PayloadError::InvalidInput),
                "{m} {p:?}"
            );
        }
        assert!(apply_payload(&v, 50.0, [2.0, -2.0, 2.0]).is_ok()); // limits accepted
        let mut no_mass = v;
        no_mass.mass_kg = 0.0;
        assert_eq!(
            apply_payload(&no_mass, 1.0, [0.0; 3]).err(),
            Some(PayloadError::InvalidVehicle)
        );
    }

    #[test]
    fn test_payload_validate_payload_matches_the_apply_checks_without_a_vehicle() {
        // limits are inclusive, anything beyond or non-finite is refused
        assert!(validate_payload(0.0, [0.0; 3]).is_ok());
        assert!(validate_payload(MAX_PAYLOAD_KG, [2.0, -2.0, 2.0]).is_ok());
        for (m, p) in [
            (-0.001, [0.0; 3]),
            (50.001, [0.0; 3]),
            (f64::NAN, [0.0; 3]),
            (1.0, [2.001, 0.0, 0.0]),
            (1.0, [0.0, f64::INFINITY, 0.0]),
        ] {
            assert_eq!(
                validate_payload(m, p),
                Err(PayloadError::InvalidInput),
                "{m} {p:?}"
            );
        }
    }

    #[test]
    fn test_payload_vehicle_mass_guard_boundaries() {
        // the guard accepts any normal positive mass and refuses zero, negative and NaN
        let mut v = ref_vehicle();
        for m in [f64::MIN_POSITIVE, 1e-9, 0.5, 2.0] {
            v.mass_kg = m;
            assert!(apply_payload(&v, 0.0, [0.0; 3]).is_ok(), "{m}");
        }
        for m in [0.0, -1.0, f64::NAN, f64::INFINITY] {
            v.mass_kg = m;
            assert_eq!(
                apply_payload(&v, 0.0, [0.0; 3]).err(),
                Some(PayloadError::InvalidVehicle),
                "{m}"
            );
        }
    }
}
