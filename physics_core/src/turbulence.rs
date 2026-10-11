// Tier: F
//! Seeded, deterministic turbulence for the test campaign (P4).
//!
//! **Every Dryden equation here is `[UNVERIFIED]`**: they were written from memory of MIL-HDBK-1797 (low-altitude form, successor of
//! MIL-F-8785C) and have not been checked against the handbook (`dev-docs/HUMAN-TODO.md` section 11 lists them). Reports carry
//! `TURBULENCE_LABEL`.
//!
//! Dryden low-altitude form, `h` in feet, `W20` the mean wind at 20 ft:
//! * scale lengths `L_w = h`, `L_u = L_v = h / (0.177 + 0.000823 h)^1.2`;
//! * intensities `sigma_w = 0.1 W20`, `sigma_u = sigma_v = sigma_w / (0.177 + 0.000823 h)^0.4`;
//! * shaping filters (`V` = air speed): `H_u = sigma_u sqrt(2 L_u / (pi V)) / (1 + (L_u / V) s)` and
//!   `H_v,w = sigma sqrt(L / (pi V)) (1 + sqrt(3) (L / V) s) / (1 + (L / V) s)^2`.
//!
//! Implementation: the filters are discretised EXACTLY for the step (first order: `a = exp(-V dt / L)`; second order: the transition
//! matrix of the repeated pole and the noise covariance `P - Phi P Phi^T` of the stationary state), so the stationary variance does not
//! depend on `dt`. States are kept normalised (unit stationary variance) so a changing `V` or `h` only changes the transition.
//!
//! Deviations from the handbook, all ASSUMED (`TODO(HUMAN-VERIFY)`): the air speed is floored at `V_MIN_MPS` (a hovering vehicle would
//! otherwise get an infinite filter time constant); the mean wind is constant with height; the height is clamped into the form's
//! validity range 10 ft ..= 1000 ft and the number of clamped steps is counted. A mean wind of zero gives exactly zero turbulence.
//!
//! `Simple` is the selectable fallback: first-order filtered noise with `sigma = level x mean wind` (vertical half of it) and an
//! ASSUMED time constant.

use std::f64::consts::PI;

/// Label that reports and the UI must show with Dryden turbulence.
pub const TURBULENCE_LABEL: &str = "Dryden (MIL-HDBK-1797 low-altitude form), UNVERIFIED";
/// Floor of the air speed used in the filters, m/s. ASSUMED, `TODO(HUMAN-VERIFY)`.
pub const V_MIN_MPS: f64 = 2.0;
/// Metres per foot (exact by definition).
pub const FOOT_M: f64 = 0.3048;
/// Lowest height of the low-altitude form, ft.
const H_MIN_FT: f64 = 10.0;
/// Highest height of the low-altitude form, ft.
const H_MAX_FT: f64 = 1000.0;
/// Time constant of the simple gust, s. ASSUMED, `TODO(HUMAN-VERIFY)`.
pub const SIMPLE_TAU_S: f64 = 2.0;
/// Vertical share of the simple gust intensity. ASSUMED, `TODO(HUMAN-VERIFY)`.
pub const SIMPLE_VERTICAL_FRACTION: f64 = 0.5;
/// Largest `dt` accepted, s.
const MAX_DT_S: f64 = 0.1;

/// Which model is used.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum TurbulenceKind {
    /// No turbulence.
    None,
    /// Dryden shaping filters (UNVERIFIED).
    Dryden,
    /// First-order filtered noise (fallback).
    Simple,
}

/// Why a turbulence call was refused.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum TurbulenceError {
    /// `dt`, height, speed or wind is non-finite or out of range, or the level is negative / non-finite.
    InvalidInput,
}

/// `SplitMix64` pseudo-random generator (public-domain algorithm by S. Vigna). Own implementation, no crate.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct SplitMix64 {
    state: u64,
}

impl SplitMix64 {
    /// Generator with the given seed.
    #[must_use]
    pub const fn new(seed: u64) -> Self {
        Self { state: seed }
    }

    /// Next 64 random bits.
    pub fn next_u64(&mut self) -> u64 {
        self.state = self.state.wrapping_add(0x9E37_79B9_7F4A_7C15);
        let mut z = self.state;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
        z ^ (z >> 31)
    }

    /// Uniform number in (0, 1] (53 bits, never 0 so `ln` is safe).
    pub fn next_uniform(&mut self) -> f64 {
        // top 53 bits as an integer < 2^53, exactly representable
        #[allow(clippy::cast_precision_loss)]
        let k = (self.next_u64() >> 11) as f64;
        (k + 1.0) * (1.0 / 9_007_199_254_740_992.0)
    }

    /// Standard normal sample (Box-Muller, one of the two values).
    pub fn next_normal(&mut self) -> f64 {
        let u1 = self.next_uniform();
        let u2 = self.next_uniform();
        (-2.0 * u1.ln()).sqrt() * (2.0 * PI * u2).cos()
    }
}

/// Scale lengths and intensities of the Dryden low-altitude form at one height.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct DrydenParams {
    /// Longitudinal scale length, m.
    pub l_u_m: f64,
    /// Lateral scale length, m.
    pub l_v_m: f64,
    /// Vertical scale length, m.
    pub l_w_m: f64,
    /// Longitudinal intensity, m/s.
    pub sigma_u_mps: f64,
    /// Lateral intensity, m/s.
    pub sigma_v_mps: f64,
    /// Vertical intensity, m/s.
    pub sigma_w_mps: f64,
    /// The height had to be clamped into 10 ft ..= 1000 ft.
    pub clamped: bool,
}

/// Dryden low-altitude scale lengths and intensities `[UNVERIFIED]`.
///
/// # Units
/// `height_m` above ground in metres (clamped to 3.048 ..= 304.8 m); `w20_mps` the mean wind at 20 ft, m/s, >= 0.
#[must_use]
pub fn dryden_params(height_m: f64, w20_mps: f64) -> DrydenParams {
    let h_ft_raw = height_m / FOOT_M;
    let h_ft = h_ft_raw.clamp(H_MIN_FT, H_MAX_FT);
    let base = 0.177 + 0.000_823 * h_ft;
    let l_u_ft = h_ft / base.powf(1.2);
    let sigma_w = 0.1 * w20_mps;
    let sigma_h = sigma_w / base.powf(0.4);
    DrydenParams {
        l_u_m: l_u_ft * FOOT_M,
        l_v_m: l_u_ft * FOOT_M,
        l_w_m: h_ft * FOOT_M,
        sigma_u_mps: sigma_h,
        sigma_v_mps: sigma_h,
        sigma_w_mps: sigma_w,
        clamped: !(H_MIN_FT..=H_MAX_FT).contains(&h_ft_raw),
    }
}

/// Stationary covariance (normalised states) of the 2nd-order shaping state: `[[1/2, 1/4], [1/4, 1/4]]` (derivation in the tests).
const P11: f64 = 0.5;
const P12: f64 = 0.25;
const P22: f64 = 0.25;

/// Two-state normalised filter state of one transverse component.
#[derive(Copy, Clone, Debug, PartialEq)]
struct Second {
    x1: f64,
    x2: f64,
}

impl Second {
    /// One exact step of the repeated-pole filter with `rdt = V dt / L`, driven by two standard normals.
    /// Returns the unit-variance output `sqrt(3) x1 + (1 - sqrt(3)) x2`.
    fn step(&mut self, rdt: f64, n1: f64, n2: f64) -> f64 {
        let e = (-rdt).exp();
        // Phi = e [[1, 0], [rdt, 1]]
        let (f11, f21, f22) = (e, e * rdt, e);
        // Q = P - Phi P Phi^T
        let pp11 = f11 * (P11 * f11);
        let pp12 = f11 * (P11 * f21 + P12 * f22);
        let pp22 = f21 * (P11 * f21 + P12 * f22) + f22 * (P12 * f21 + P22 * f22);
        let (q11, q12, q22) = ((P11 - pp11).max(0.0), P12 - pp12, (P22 - pp22).max(0.0));
        // Cholesky of the 2x2 covariance
        let l11 = q11.sqrt();
        let l21 = if l11 > 0.0 { q12 / l11 } else { 0.0 };
        let l22 = (q22 - l21 * l21).max(0.0).sqrt();
        let x1 = f11 * self.x1 + l11 * n1;
        let x2 = f21 * self.x1 + f22 * self.x2 + l21 * n1 + l22 * n2;
        self.x1 = x1;
        self.x2 = x2;
        3.0_f64.sqrt() * x1 + (1.0 - 3.0_f64.sqrt()) * x2
    }
}

/// A turbulence generator for one run.
#[derive(Clone, Debug, PartialEq)]
pub struct Turbulence {
    kind: TurbulenceKind,
    level: f64,
    rng: SplitMix64,
    u: f64,
    v: Second,
    w: Second,
    /// Steps on which the height had to be clamped into the low-altitude validity range (Dryden only).
    pub clamped_steps: u64,
}

impl Turbulence {
    /// New generator. `level` is the simple gust's RMS as a fraction of the mean wind (ignored by Dryden).
    ///
    /// # Errors
    /// `InvalidInput` if `level` is negative or not finite.
    pub fn new(kind: TurbulenceKind, seed: u64, level: f64) -> Result<Self, TurbulenceError> {
        if !(level.is_finite() && level >= 0.0) {
            return Err(TurbulenceError::InvalidInput);
        }
        Ok(Self {
            kind,
            level,
            rng: SplitMix64::new(seed),
            u: 0.0,
            v: Second { x1: 0.0, x2: 0.0 },
            w: Second { x1: 0.0, x2: 0.0 },
            clamped_steps: 0,
        })
    }

    /// The model in use.
    #[must_use]
    pub const fn kind(&self) -> TurbulenceKind {
        self.kind
    }

    /// One step: the gust velocity (NED, m/s) to add to the mean wind.
    ///
    /// The wind axes: `u` along the horizontal mean wind, `v` horizontal across it (to the left of the wind direction seen from above:
    /// `(-w_e, w_n) / |w|`), `w` vertical (NED down positive).
    ///
    /// # Errors
    /// `InvalidInput` for a non-finite or out-of-range argument (`0 < dt <= 0.1`, `height_m >= 0`, `air_speed_mps >= 0`).
    pub fn step(
        &mut self,
        dt_s: f64,
        height_m: f64,
        air_speed_mps: f64,
        mean_wind_ned_mps: [f64; 3],
    ) -> Result<[f64; 3], TurbulenceError> {
        let inputs_ok = dt_s > 0.0
            && dt_s <= MAX_DT_S
            && height_m.is_finite()
            && height_m >= 0.0
            && air_speed_mps.is_finite()
            && air_speed_mps >= 0.0
            && mean_wind_ned_mps.iter().all(|c| c.is_finite());
        if !inputs_ok {
            return Err(TurbulenceError::InvalidInput);
        }
        let [north, east, _] = mean_wind_ned_mps;
        let w_mag = north.hypot(east);
        // five standard normals per step whatever the model, so the random stream never depends on the mean wind
        let noise: [f64; 5] = std::array::from_fn(|_| self.rng.next_normal());
        let (u, v, w) = match self.kind {
            TurbulenceKind::None => return Ok([0.0; 3]),
            TurbulenceKind::Dryden => self.dryden(dt_s, height_m, air_speed_mps, w_mag, noise),
            TurbulenceKind::Simple => {
                let [long, lat, _, vert, _] = noise;
                self.simple(dt_s, w_mag, [long, lat, vert])
            }
        };
        if w_mag <= 0.0 {
            return Ok([0.0; 3]);
        }
        let (ex, ey) = (north / w_mag, east / w_mag);
        Ok([u * ex - v * ey, u * ey + v * ex, w])
    }

    fn dryden(
        &mut self,
        dt: f64,
        height_m: f64,
        air_speed: f64,
        w20: f64,
        n: [f64; 5],
    ) -> (f64, f64, f64) {
        let p = dryden_params(height_m, w20);
        if p.clamped {
            self.clamped_steps = self.clamped_steps.saturating_add(1);
        }
        let speed = air_speed.max(V_MIN_MPS);
        let a = (-speed * dt / p.l_u_m).exp();
        self.u = a * self.u + (1.0 - a * a).max(0.0).sqrt() * n[0];
        let yv = self.v.step(speed * dt / p.l_v_m, n[1], n[2]);
        let yw = self.w.step(speed * dt / p.l_w_m, n[3], n[4]);
        (
            p.sigma_u_mps * self.u,
            p.sigma_v_mps * yv,
            p.sigma_w_mps * yw,
        )
    }

    fn simple(&mut self, dt: f64, w_mag: f64, n: [f64; 3]) -> (f64, f64, f64) {
        let a = (-dt / SIMPLE_TAU_S).exp();
        let k = (1.0 - a * a).max(0.0).sqrt();
        self.u = a * self.u + k * n[0];
        self.v.x1 = a * self.v.x1 + k * n[1];
        self.w.x1 = a * self.w.x1 + k * n[2];
        let sigma = self.level * w_mag;
        (
            sigma * self.u,
            sigma * self.v.x1,
            SIMPLE_VERTICAL_FRACTION * sigma * self.w.x1,
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // Expected values come from the formulas in the module doc worked by hand / by an independent script, never from this module.

    fn near(a: f64, b: f64, tol: f64) -> bool {
        (a - b).abs() <= tol
    }

    #[allow(clippy::too_many_arguments)]
    fn run(
        kind: TurbulenceKind,
        seed: u64,
        level: f64,
        dt: f64,
        n: usize,
        speed: f64,
        wind: [f64; 3],
        h: f64,
    ) -> Vec<[f64; 3]> {
        let mut t = Turbulence::new(kind, seed, level).unwrap();
        (0..n)
            .map(|_| t.step(dt, h, speed, wind).unwrap())
            .collect()
    }

    fn variance(x: &[f64]) -> f64 {
        let m = x.iter().sum::<f64>() / x.len() as f64;
        x.iter().map(|v| (v - m) * (v - m)).sum::<f64>() / x.len() as f64
    }

    fn autocorr(x: &[f64], lag: usize) -> f64 {
        let m = x.iter().sum::<f64>() / x.len() as f64;
        let c0: f64 = x.iter().map(|v| (v - m) * (v - m)).sum();
        let ck: f64 = x
            .iter()
            .zip(&x[lag..])
            .map(|(a, b)| (a - m) * (b - m))
            .sum();
        ck / c0
    }

    // ------------------------------------------------------------------ PRNG

    #[test]
    fn test_turb_splitmix64_reference_vectors() {
        // independent implementation in Python: seed 0 -> e220a8397b1dcdaf, 6e789e6aa1b965f4, 06c45d188009454f; seed 12345 -> 22118258a9d111a0, 346edce5f713f8ed
        let mut a = SplitMix64::new(0);
        assert_eq!(
            [a.next_u64(), a.next_u64(), a.next_u64()],
            [
                0xe220_a839_7b1d_cdaf,
                0x6e78_9e6a_a1b9_65f4,
                0x06c4_5d18_8009_454f
            ]
        );
        let mut b = SplitMix64::new(12345);
        assert_eq!(
            [b.next_u64(), b.next_u64()],
            [0x2211_8258_a9d1_11a0, 0x346e_dce5_f713_f8ed]
        );
    }

    #[test]
    fn test_turb_uniform_and_normal_samples() {
        // first seed-0 output 0xe220a8397b1dcdaf: ((x >> 11) + 1) / 2^53 = 0.8833108082136427
        assert!(near(
            SplitMix64::new(0).next_uniform(),
            0.883_310_808_213_642_7,
            1e-15
        ));
        let mut g = SplitMix64::new(7);
        assert!((0..100_000).all(|_| {
            let u = g.next_uniform();
            u > 0.0 && u <= 1.0
        }));
        let xs: Vec<f64> = (0..400_000).map(|_| g.next_normal()).collect();
        let m = xs.iter().sum::<f64>() / xs.len() as f64;
        assert!(
            m.abs() < 0.01 && near(variance(&xs), 1.0, 0.015),
            "{m} {}",
            variance(&xs)
        );
    }

    // ------------------------------------------------------------------ parameters

    #[test]
    fn test_turb_dryden_scale_lengths_and_intensities_hand_values() {
        // h = 100 ft = 30.48 m, W20 = 10 m/s: 0.177 + 0.000823 x 100 = 0.2593
        // L_u = L_v = 100 / 0.2593^1.2 = 505.169 ft = 153.9756 m; L_w = 100 ft = 30.48 m
        // sigma_w = 0.1 x 10 = 1 m/s; sigma_u = sigma_v = 1 / 0.2593^0.4 = 1.715849 m/s
        let p = dryden_params(30.48, 10.0);
        assert!(
            near(p.l_u_m, 153.975_613_279_079, 1e-9)
                && near(p.l_v_m, p.l_u_m, 0.0)
                && near(p.l_w_m, 30.48, 1e-12)
        );
        assert!(
            near(p.sigma_w_mps, 1.0, 1e-15)
                && near(p.sigma_u_mps, 1.715_848_712_971_171_7, 1e-12)
                && near(p.sigma_v_mps, p.sigma_u_mps, 0.0)
        );
        assert!(!p.clamped);
    }

    #[test]
    fn test_turb_dryden_height_clamp() {
        // 1000 ft: 0.177 + 0.823 = 1.0, so L_u = L_w = 1000 ft = 304.8 m and sigma_u = sigma_w
        let hi = dryden_params(400.0, 10.0);
        assert!(
            hi.clamped
                && near(hi.l_u_m, 304.8, 1e-9)
                && near(hi.l_w_m, 304.8, 1e-9)
                && near(hi.sigma_u_mps, 1.0, 1e-12)
        );
        // 10 ft = 3.048 m: L_w = 3.048 m, L_u = 10 / (0.18523)^1.2 ft = 75.6366 ft = 23.0548 m
        let lo = dryden_params(1.0, 10.0);
        assert!(
            lo.clamped
                && near(lo.l_w_m, 3.048, 1e-12)
                && near(lo.l_u_m, 23.054_800_611_602_21, 1e-6)
        );
        // exactly on the edges is not clamped
        assert!(!dryden_params(3.048, 10.0).clamped && !dryden_params(304.8, 10.0).clamped);
        assert_eq!(dryden_params(30.48, 0.0).sigma_u_mps, 0.0);
    }

    #[test]
    fn test_turb_second_order_state_has_unit_output_variance() {
        // unit output y = sqrt(3) x1 + (1 - sqrt(3)) x2 with P = [[1/2, 1/4], [1/4, 1/4]]:
        // var = 3 (1/2) + 2 sqrt(3) (1 - sqrt(3)) (1/4) + (1 - sqrt(3))^2 (1/4) = 1.5 - 0.633975 + 0.133975 = 1
        let s3 = 3.0_f64.sqrt();
        let var = 3.0 * P11 + 2.0 * s3 * (1.0 - s3) * P12 + (1.0 - s3) * (1.0 - s3) * P22;
        assert!(near(var, 1.0, 1e-14));
        // P solves the Lyapunov equation A P + P A^T + B B^T = 0 for A = [[-1, 0], [1, -1]], B = [1, 0] (r = 1)
        assert!(
            near(-2.0 * P11 + 1.0, 0.0, 1e-15)
                && near(-2.0 * P12 + P11, 0.0, 1e-15)
                && near(2.0 * (P12 - P22), 0.0, 1e-15)
        );
    }

    // ------------------------------------------------------------------ statistics

    const WIND_N: [f64; 3] = [10.0, 0.0, 0.0];

    #[test]
    fn test_turb_dryden_component_variances_match_sigma_squared() {
        // h = 30.48 m, W20 = 10 m/s, V = 10 m/s: sigma_u = sigma_v = 1.715849, sigma_w = 1. 3e6 steps of 10 ms = 30000 s
        let s = run(
            TurbulenceKind::Dryden,
            11,
            0.0,
            0.01,
            3_000_000,
            10.0,
            WIND_N,
            30.48,
        );
        let u: Vec<f64> = s.iter().map(|g| g[0]).collect();
        let v: Vec<f64> = s.iter().map(|g| g[1]).collect();
        let w: Vec<f64> = s.iter().map(|g| g[2]).collect();
        let su2 = 1.715_848_712_971_171_7_f64.powi(2);
        // statistical tolerance 15 % (about 3 standard deviations of a variance estimate over this record)
        assert!(
            near(variance(&u) / su2, 1.0, 0.15),
            "u {}",
            variance(&u) / su2
        );
        assert!(
            near(variance(&v) / su2, 1.0, 0.15),
            "v {}",
            variance(&v) / su2
        );
        assert!(near(variance(&w), 1.0, 0.15), "w {}", variance(&w));
    }

    #[test]
    fn test_turb_variance_does_not_depend_on_dt() {
        let s = run(
            TurbulenceKind::Dryden,
            5,
            0.0,
            0.05,
            600_000,
            10.0,
            WIND_N,
            30.48,
        );
        let w: Vec<f64> = s.iter().map(|g| g[2]).collect();
        let v: Vec<f64> = s.iter().map(|g| g[1]).collect();
        assert!(near(variance(&w), 1.0, 0.2), "{}", variance(&w));
        assert!(near(
            variance(&v) / 1.715_848_712_971_171_7_f64.powi(2),
            1.0,
            0.2
        ));
    }

    #[test]
    fn test_turb_autocorrelation_matches_the_dryden_form() {
        // dt 0.01 s, V = 10: xi = lag V / L. u: rho = exp(-xi) with L_u = 153.9756 m; w: rho = (1 - xi/2) exp(-xi) with L_w = 30.48 m
        let s = run(
            TurbulenceKind::Dryden,
            21,
            0.0,
            0.01,
            3_000_000,
            10.0,
            WIND_N,
            30.48,
        );
        let u: Vec<f64> = s.iter().map(|g| g[0]).collect();
        let w: Vec<f64> = s.iter().map(|g| g[2]).collect();
        // u: lag 5 s -> xi = 0.324729, rho = 0.72270; lag 15 s -> xi = 0.974187, rho = 0.37762
        assert!(near(
            autocorr(&u, 500),
            (-50.0_f64 / 153.975_613_279_079).exp(),
            0.05
        ));
        assert!(near(
            autocorr(&u, 1500),
            (-150.0_f64 / 153.975_613_279_079).exp(),
            0.05
        ));
        // w: lag 1 s -> xi = 0.328084, rho = 0.83596 x 0.72030 = 0.60216; lag 3 s -> xi = 0.984252, rho = 0.50787 x 0.37369 = 0.18978;
        // lag 6 s -> xi = 1.968504, rho = 0.015748 x 0.13965 = 0.0022
        let rho = |lag: f64| {
            let xi = lag * 10.0 / 30.48;
            (1.0 - xi / 2.0) * (-xi).exp()
        };
        assert!(
            near(autocorr(&w, 100), rho(1.0), 0.05)
                && near(autocorr(&w, 300), rho(3.0), 0.05)
                && near(autocorr(&w, 600), rho(6.0), 0.05)
        );
        assert!(near(rho(1.0), 0.602_16, 1e-4) && near(rho(3.0), 0.189_78, 1e-4));
    }

    /// Averaged Hann-window periodogram of `x` (sampled every `dt`), one-sided, per Hz, at DFT bin `k` of segments of `n` samples.
    fn psd_at(x: &[f64], dt: f64, n: usize, k: usize) -> f64 {
        let w: Vec<f64> = (0..n)
            .map(|i| 0.5 - 0.5 * (2.0 * PI * i as f64 / n as f64).cos())
            .collect();
        let w2: f64 = w.iter().map(|v| v * v).sum();
        let segs = x.len() / n;
        let mut acc = 0.0;
        for s in 0..segs {
            let seg = &x[s * n..(s + 1) * n];
            let m = seg.iter().sum::<f64>() / n as f64;
            let (mut re, mut im) = (0.0, 0.0);
            for (i, v) in seg.iter().enumerate() {
                let ang = -2.0 * PI * (k * i) as f64 / n as f64;
                re += (v - m) * w[i] * ang.cos();
                im += (v - m) * w[i] * ang.sin();
            }
            acc += 2.0 * dt * (re * re + im * im) / w2;
        }
        acc / segs as f64
    }

    #[test]
    fn test_turb_spectrum_shape_at_three_frequencies() {
        // dt = 0.05 s, V = 10 m/s, 1024-sample Hann segments (bin spacing 1/51.2 Hz), 400 segments. One-sided temporal PSD per Hz
        // S(f) = 2 pi Phi(Omega) / V with Omega = 2 pi f / V and the (one-sided) Dryden spatial spectra
        // Phi_u = sigma^2 (2L/pi) / (1 + (L Omega)^2), Phi_w = sigma^2 (L/pi) (1 + 3 (L Omega)^2) / (1 + (L Omega)^2)^2
        let (dt, n, segs) = (0.05, 1024, 400);
        let s = run(
            TurbulenceKind::Dryden,
            31,
            0.0,
            dt,
            n * segs,
            10.0,
            WIND_N,
            30.48,
        );
        let u: Vec<f64> = s.iter().map(|g| g[0]).collect();
        let w: Vec<f64> = s.iter().map(|g| g[2]).collect();
        let (lu, lw, su, sw) = (153.975_613_279_079, 30.48, 1.715_848_712_971_171_7, 1.0);
        for k in [4_usize, 16, 64] {
            let f = k as f64 / (n as f64 * dt);
            let om = 2.0 * PI * f / 10.0;
            let phi_u = su * su * (2.0 * lu / PI) / (1.0 + (lu * om).powi(2));
            let phi_w = sw * sw * (lw / PI) * (1.0 + 3.0 * (lw * om).powi(2))
                / (1.0 + (lw * om).powi(2)).powi(2);
            let (eu, ew) = (psd_at(&u, dt, n, k), psd_at(&w, dt, n, k));
            assert!(
                near(eu / (2.0 * PI * phi_u / 10.0), 1.0, 0.2),
                "u bin {k}: {}",
                eu / (2.0 * PI * phi_u / 10.0)
            );
            assert!(
                near(ew / (2.0 * PI * phi_w / 10.0), 1.0, 0.2),
                "w bin {k}: {}",
                ew / (2.0 * PI * phi_w / 10.0)
            );
        }
    }

    // ------------------------------------------------------------------ determinism, axes, inputs

    #[test]
    fn test_turb_same_seed_is_bit_identical_and_different_seed_differs() {
        let a = run(
            TurbulenceKind::Dryden,
            9,
            0.0,
            0.002,
            5000,
            6.0,
            [3.0, -4.0, 0.0],
            20.0,
        );
        let b = run(
            TurbulenceKind::Dryden,
            9,
            0.0,
            0.002,
            5000,
            6.0,
            [3.0, -4.0, 0.0],
            20.0,
        );
        let c = run(
            TurbulenceKind::Dryden,
            10,
            0.0,
            0.002,
            5000,
            6.0,
            [3.0, -4.0, 0.0],
            20.0,
        );
        assert!(
            a.iter()
                .zip(&b)
                .all(|(x, y)| x.map(f64::to_bits) == y.map(f64::to_bits))
        );
        assert!(a.iter().zip(&c).any(|(x, y)| x != y));
    }

    #[test]
    fn test_turb_zero_wind_gives_exactly_zero_and_none_kind_too() {
        for kind in [
            TurbulenceKind::Dryden,
            TurbulenceKind::Simple,
            TurbulenceKind::None,
        ] {
            let s = run(kind, 3, 0.5, 0.002, 2000, 5.0, [0.0; 3], 20.0);
            assert!(s.iter().all(|g| g.iter().all(|v| *v == 0.0)), "{kind:?}");
        }
        // vertical wind alone is not a horizontal mean wind: W20 uses the horizontal part only
        let s = run(
            TurbulenceKind::Dryden,
            3,
            0.0,
            0.002,
            500,
            5.0,
            [0.0, 0.0, 4.0],
            20.0,
        );
        assert!(s.iter().all(|g| g.iter().all(|v| *v == 0.0)));
    }

    #[test]
    fn test_turb_axes_follow_the_mean_wind_direction() {
        // same seed and same |wind|: wind toward north gives (u, v, w); wind toward east gives (-v, u, w) exactly
        let n = run(
            TurbulenceKind::Dryden,
            4,
            0.0,
            0.01,
            3000,
            10.0,
            [10.0, 0.0, 0.0],
            30.0,
        );
        let e = run(
            TurbulenceKind::Dryden,
            4,
            0.0,
            0.01,
            3000,
            10.0,
            [0.0, 10.0, 0.0],
            30.0,
        );
        assert!(n.iter().zip(&e).all(|(a, b)| near(b[0], -a[1], 1e-12)
            && near(b[1], a[0], 1e-12)
            && near(b[2], a[2], 1e-12)));
        // and a diagonal wind keeps the magnitude of the horizontal gust
        let d = run(
            TurbulenceKind::Dryden,
            4,
            0.0,
            0.01,
            3000,
            10.0,
            [6.0, 8.0, 0.0],
            30.0,
        );
        assert!(
            n.iter()
                .zip(&d)
                .all(|(a, b)| near(a[0].hypot(a[1]), b[0].hypot(b[1]), 1e-12))
        );
    }

    #[test]
    fn test_turb_height_clamp_is_counted() {
        let mut t = Turbulence::new(TurbulenceKind::Dryden, 1, 0.0).unwrap();
        for _ in 0..10 {
            t.step(0.01, 1.0, 5.0, WIND_N).unwrap(); // 1 m = 3.3 ft: clamped
        }
        for _ in 0..5 {
            t.step(0.01, 50.0, 5.0, WIND_N).unwrap();
        }
        assert_eq!(t.clamped_steps, 10);
    }

    #[test]
    fn test_turb_air_speed_floor() {
        // below V_MIN the filters use 2 m/s: a hovering run (speed 0) equals a run at 2 m/s bit for bit
        let a = run(TurbulenceKind::Dryden, 8, 0.0, 0.01, 500, 0.0, WIND_N, 30.0);
        let b = run(
            TurbulenceKind::Dryden,
            8,
            0.0,
            0.01,
            500,
            V_MIN_MPS,
            WIND_N,
            30.0,
        );
        assert!(
            a.iter()
                .zip(&b)
                .all(|(x, y)| x.map(f64::to_bits) == y.map(f64::to_bits))
        );
    }

    #[test]
    fn test_turb_simple_gust_statistics_and_level_zero() {
        // level 0.3 x 10 m/s: sigma 3 m/s horizontally, 1.5 m/s vertically (half), time constant 2 s
        let s = run(
            TurbulenceKind::Simple,
            6,
            0.3,
            0.01,
            1_500_000,
            10.0,
            WIND_N,
            30.0,
        );
        let u: Vec<f64> = s.iter().map(|g| g[0]).collect();
        let w: Vec<f64> = s.iter().map(|g| g[2]).collect();
        assert!(near(variance(&u) / 9.0, 1.0, 0.1) && near(variance(&w) / 2.25, 1.0, 0.1));
        assert!(near(autocorr(&u, 200), (-1.0_f64).exp(), 0.05)); // lag 2 s = one time constant
        let z = run(
            TurbulenceKind::Simple,
            6,
            0.0,
            0.01,
            100,
            10.0,
            WIND_N,
            30.0,
        );
        assert!(z.iter().all(|g| g.iter().all(|v| *v == 0.0)));
    }

    #[test]
    fn test_turb_invalid_inputs_are_refused() {
        let mut t = Turbulence::new(TurbulenceKind::Dryden, 1, 0.0).unwrap();
        let e = Err(TurbulenceError::InvalidInput);
        assert_eq!(t.step(0.0, 20.0, 5.0, WIND_N), e);
        assert_eq!(t.step(0.101, 20.0, 5.0, WIND_N), e);
        assert_eq!(t.step(f64::NAN, 20.0, 5.0, WIND_N), e);
        assert_eq!(t.step(0.01, -1.0, 5.0, WIND_N), e);
        assert_eq!(t.step(0.01, f64::INFINITY, 5.0, WIND_N), e);
        assert_eq!(t.step(0.01, 20.0, -1.0, WIND_N), e);
        assert_eq!(t.step(0.01, 20.0, f64::NAN, WIND_N), e);
        assert_eq!(t.step(0.01, 20.0, 5.0, [f64::NAN, 0.0, 0.0]), e);
        assert!(t.step(0.1, 0.0, 0.0, WIND_N).is_ok()); // edges accepted
        assert_eq!(
            Turbulence::new(TurbulenceKind::Simple, 1, -0.1).err(),
            Some(TurbulenceError::InvalidInput)
        );
        assert_eq!(
            Turbulence::new(TurbulenceKind::Simple, 1, f64::NAN).err(),
            Some(TurbulenceError::InvalidInput)
        );
        assert!(Turbulence::new(TurbulenceKind::Simple, 1, 0.0).is_ok());
    }

    #[test]
    fn test_turb_label_says_unverified() {
        assert!(TURBULENCE_LABEL.contains("Dryden") && TURBULENCE_LABEL.contains("UNVERIFIED"));
    }

    #[test]
    fn test_turb_lateral_component_is_to_the_right_of_the_wind_direction() {
        // Simple model, first step, dt = 0.01 s, level 0.3, mean wind 10 m/s (sigma = 3): with n = the first five normals of the seed,
        // a = exp(-dt / tau), k = sqrt(1 - a^2): u = sigma k n0, v = sigma k n1, w = 0.5 sigma k n3.
        // Wind blowing toward the NORTH: gust NED = (u, v, w) (v points east, to the right of the wind direction).
        // Wind blowing toward the EAST: gust NED = (-v, u, w).
        let mut g = SplitMix64::new(4);
        let n: Vec<f64> = (0..5).map(|_| g.next_normal()).collect();
        let a = (-0.01_f64 / SIMPLE_TAU_S).exp();
        let k = (1.0 - a * a).sqrt();
        let (u, v, w) = (3.0 * k * n[0], 3.0 * k * n[1], 0.5 * 3.0 * k * n[3]);
        let north = Turbulence::new(TurbulenceKind::Simple, 4, 0.3)
            .unwrap()
            .step(0.01, 30.0, 10.0, [10.0, 0.0, 0.0])
            .unwrap();
        assert!(near(north[0], u, 1e-12) && near(north[1], v, 1e-12) && near(north[2], w, 1e-12));
        let east = Turbulence::new(TurbulenceKind::Simple, 4, 0.3)
            .unwrap()
            .step(0.01, 30.0, 10.0, [0.0, 10.0, 0.0])
            .unwrap();
        assert!(near(east[0], -v, 1e-12) && near(east[1], u, 1e-12) && near(east[2], w, 1e-12));
    }
}
