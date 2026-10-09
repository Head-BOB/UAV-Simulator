// Tier: V
// DEMO, not flight code. A simple cascaded PID flight controller so the Unreal demo can be flown
// from the keyboard or a gamepad. It is NOT a validated controller, has no failsafes, and must
// never be reused on a vehicle. Authority limits, gains and speeds below are
// TODO(HUMAN-VERIFY: demo value): chosen for the shipped quad (1.5 kg) and checked only in the
// closed-loop simulation tools/cpp_ffi_check/demo_flight_sim.cpp (stability and limits, not
// accuracy).
//
// UE-free (only <cmath>): plain g++ can test it.
//
// Structure (cascade, outer to inner):
//   altitude P -> climb-rate command -> vertical-velocity PI -> thrust (throttle)
//   horizontal velocity PI (yaw-aligned frame) -> acceleration -> tilt angle command
//   attitude P (roll, pitch) -> body-rate command -> rate PI -> roll / pitch command
//   yaw-rate command -> rate PI -> yaw command
// Output is ControlInputs-style: throttle 0..1, roll / pitch / yaw -1..1, with the Rust mixer's
// sign convention (physics_core/src/mixer.rs): roll+ = right wing down, pitch+ = nose up,
// yaw+ = nose right.
//
// Frames: NED world, body X forward / Y right / Z down, quaternion order (x, y, z, w).
#pragma once

#include <cmath>

namespace DroneDemo
{
constexpr double kPi = 3.14159265358979323846;
// Standard gravity, m/s^2 (CGPM 1901, same value as physics_core).
constexpr double kGravityMps2 = 9.80665;

inline double Clamp(double V, double Lo, double Hi)
{
    return V < Lo ? Lo : (V > Hi ? Hi : V);
}

// PID with a clamped integral and a clamped output. The integral only accumulates while the
// output is not saturated in the direction of the error (simple anti-windup).
struct Pid
{
    double Kp = 0.0;
    double Ki = 0.0;
    double Kd = 0.0;
    double IntegralLimit = 1.0;  // |integral term| <= this, in output units
    double OutputLimit = 1.0;    // |output| <= this
    double Integral = 0.0;       // integral state, in error * seconds
    double PrevError = 0.0;
    bool bHavePrev = false;

    void Reset()
    {
        Integral = 0.0;
        PrevError = 0.0;
        bHavePrev = false;
    }

    // dt must be > 0 and finite, Error finite; otherwise returns 0 and changes nothing.
    double Step(double Error, double Dt)
    {
        if (!(Dt > 0.0) || !std::isfinite(Dt) || !std::isfinite(Error)) return 0.0;
        const double Derivative = bHavePrev ? (Error - PrevError) / Dt : 0.0;
        PrevError = Error;
        bHavePrev = true;

        const double Unsat = Kp * Error + Ki * Integral + Kd * Derivative;
        const bool bSaturatedSameWay = (Unsat > OutputLimit && Error > 0.0) || (Unsat < -OutputLimit && Error < 0.0);
        if (!bSaturatedSameWay && Ki != 0.0)
        {
            const double Limit = IntegralLimit / Ki;
            Integral = Clamp(Integral + Error * Dt, -std::fabs(Limit), std::fabs(Limit));
        }
        return Clamp(Kp * Error + Ki * Integral + Kd * Derivative, -OutputLimit, OutputLimit);
    }
};

struct Euler
{
    double RollRad;   // + = right wing down
    double PitchRad;  // + = nose up
    double YawRad;    // + = nose right, from north
};

// Aerospace ZYX Euler angles from a unit quaternion (x, y, z, w) body -> NED:
//   roll  = atan2(2(wx + yz), 1 - 2(x^2 + y^2))
//   pitch = asin(2(wy - zx))
//   yaw   = atan2(2(wz + xy), 1 - 2(y^2 + z^2))
inline Euler QuatToEuler(const double Q[4])
{
    const double X = Q[0], Y = Q[1], Z = Q[2], W = Q[3];
    const double SinPitch = Clamp(2.0 * (W * Y - Z * X), -1.0, 1.0);
    return Euler{std::atan2(2.0 * (W * X + Y * Z), 1.0 - 2.0 * (X * X + Y * Y)), std::asin(SinPitch),
                 std::atan2(2.0 * (W * Z + X * Y), 1.0 - 2.0 * (Y * Y + Z * Z))};
}

// Wraps an angle error into [-pi, pi].
inline double WrapPi(double A)
{
    double W = std::fmod(A + kPi, 2.0 * kPi);
    if (W < 0.0) W += 2.0 * kPi;
    return W - kPi;
}

// North/east velocity to the yaw-aligned frame: forward along the nose heading, right of it.
inline void NedToHeadingFrame(double VNorth, double VEast, double YawRad, double* Forward, double* Right)
{
    *Forward = std::cos(YawRad) * VNorth + std::sin(YawRad) * VEast;
    *Right = -std::sin(YawRad) * VNorth + std::cos(YawRad) * VEast;
}

// Pilot command: speeds in m/s and rad/s (already scaled by the pawn from key or stick input).
struct Command
{
    double ForwardMps = 0.0;
    double RightMps = 0.0;
    double ClimbMps = 0.0;  // + = up
    double YawRateRadS = 0.0;
};

// What the controller reads (from DroneState, converted by the caller; no Unreal types here).
struct Estimate
{
    double AltitudeM;       // -position_z
    double VelNedMps[3];
    double QuatXyzw[4];
    double BodyRatesRadS[3];  // p, q, r
};

struct Output
{
    double Throttle;  // 0..1
    double Roll;      // -1..1
    double Pitch;     // -1..1
    double Yaw;       // -1..1
};

enum class Mode
{
    OnGround,  // motors off, nothing flies
    Takeoff,   // climbing to TakeoffAltitudeM
    Flying,    // altitude hold with pilot commands
    Landing    // descending at LandingSinkMps
};

struct Tuning
{
    double HoverThrottle = 0.5;           // from vehicle_config.json (hover_throttle), set by the caller
    double TakeoffAltitudeM = 2.0;
    double MinHoldAltitudeM = 0.5;        // altitude target floor while flying
    double MaxHoldAltitudeM = 100.0;
    double MaxClimbMps = 2.0;
    double LandingSinkMps = 1.0;
    double SoftLandingAltitudeM = 0.5;    // below this the sink rate halves
    double MaxTiltRad = 0.35;             // about 20 deg
    double MaxHorizAccelMps2 = 3.0;
    double AltitudeKp = 1.0;              // 1/s
    double AttitudeKp = 6.0;              // 1/s, angle error -> rate command
    double MaxRateCmdRadS = 3.0;
};

// DEMO ground handling. The Rust core has no ground contact (the ISA density model rejects
// altitude < 0 with step code 6), so the demo pins the vehicle to the ground itself: when
// landing reaches TouchdownAltitudeM, or when any flying mode would go below the ground.
// TODO(HUMAN-VERIFY: demo value) 0.05 m.
constexpr double kTouchdownAltitudeM = 0.05;

inline bool ShouldTouchDown(Mode M, double AltitudeM)
{
    if (M == Mode::OnGround) return false;
    if (AltitudeM <= 0.0) return true;
    return M == Mode::Landing && AltitudeM <= kTouchdownAltitudeM;
}

class Controller
{
public:
    explicit Controller(const Tuning& T = Tuning()) : Tune(T)
    {
        VertVel = Pid{4.0, 1.5, 0.0, 3.0, 6.0};          // m/s error -> m/s^2 up
        HorizF = Pid{1.5, 0.3, 0.0, 1.0, T.MaxHorizAccelMps2};
        HorizR = Pid{1.5, 0.3, 0.0, 1.0, T.MaxHorizAccelMps2};
        // Rate gains from the airframe: roll authority 2.34 N*m per unit command at hover
        // (4 arms x 0.2 m x 2.92 N), inertia 0.015 kg m^2 -> 156 rad/s^2 per unit; Kp 0.16 gives
        // about 25 rad/s of rate-loop bandwidth. Yaw: 0.058 N*m per unit / 0.025 -> 2.3 rad/s^2.
        RateRoll = Pid{0.16, 0.5, 0.0, 0.3, 1.0};
        RatePitch = Pid{0.16, 0.5, 0.0, 0.3, 1.0};
        RateYaw = Pid{2.0, 1.0, 0.0, 0.3, 1.0};
    }

    Mode GetMode() const { return CurrentMode; }
    double GetAltitudeTargetM() const { return AltTargetM; }

    // Takeoff is accepted only from OnGround; Land only while flying or taking off.
    bool RequestTakeoff(double CurrentAltitudeM)
    {
        if (CurrentMode != Mode::OnGround) return false;
        ResetLoops();
        AltTargetM = CurrentAltitudeM + 0.0;
        CurrentMode = Mode::Takeoff;
        return true;
    }

    bool RequestLand()
    {
        if (CurrentMode != Mode::Takeoff && CurrentMode != Mode::Flying) return false;
        CurrentMode = Mode::Landing;
        return true;
    }

    // The caller (pawn) tells the controller the vehicle is on the ground again.
    void NotifyOnGround()
    {
        CurrentMode = Mode::OnGround;
        ResetLoops();
    }

    // Returns false (Out untouched) if Dt or the estimate is not finite.
    bool Update(const Estimate& E, const Command& C, double Dt, Output* Out)
    {
        if (!Out || !(Dt > 0.0) || !std::isfinite(Dt) || !EstimateFinite(E)) return false;
        if (CurrentMode == Mode::OnGround)
        {
            *Out = Output{0.0, 0.0, 0.0, 0.0};
            return true;
        }

        const double Climb = ClimbCommand(E, C, Dt);
        AttitudeAndThrottle(E, C, Dt, Climb, Out);
        return true;
    }

private:
    static bool EstimateFinite(const Estimate& E)
    {
        bool Ok = std::isfinite(E.AltitudeM);
        for (int I = 0; I < 3; ++I) Ok = Ok && std::isfinite(E.VelNedMps[I]) && std::isfinite(E.BodyRatesRadS[I]);
        for (int I = 0; I < 4; ++I) Ok = Ok && std::isfinite(E.QuatXyzw[I]);
        return Ok;
    }

    void ResetLoops()
    {
        VertVel.Reset();
        HorizF.Reset();
        HorizR.Reset();
        RateRoll.Reset();
        RatePitch.Reset();
        RateYaw.Reset();
    }

    // Climb-rate command (m/s, + up) for the current mode, and the altitude-target bookkeeping.
    double ClimbCommand(const Estimate& E, const Command& C, double Dt)
    {
        switch (CurrentMode)
        {
        case Mode::Takeoff:
            AltTargetM = Tune.TakeoffAltitudeM;
            if (E.AltitudeM >= Tune.TakeoffAltitudeM - 0.2) CurrentMode = Mode::Flying;
            break;
        case Mode::Flying:
            AltTargetM = Clamp(AltTargetM + C.ClimbMps * Dt, Tune.MinHoldAltitudeM, Tune.MaxHoldAltitudeM);
            break;
        case Mode::Landing:
            return -(E.AltitudeM < Tune.SoftLandingAltitudeM ? 0.5 * Tune.LandingSinkMps : Tune.LandingSinkMps);
        default:
            break;
        }
        return Clamp(Tune.AltitudeKp * (AltTargetM - E.AltitudeM), -Tune.MaxClimbMps, Tune.MaxClimbMps);
    }

    void AttitudeAndThrottle(const Estimate& E, const Command& C, double Dt, double ClimbCmdMps, Output* Out)
    {
        const Euler Att = QuatToEuler(E.QuatXyzw);

        // Horizontal velocity (yaw-aligned) -> acceleration -> tilt.
        double VForward = 0.0, VRight = 0.0;
        NedToHeadingFrame(E.VelNedMps[0], E.VelNedMps[1], Att.YawRad, &VForward, &VRight);
        // While landing the pilot's horizontal command is honoured; the pawn clamps it if needed.
        const double AccF = HorizF.Step(C.ForwardMps - VForward, Dt);
        const double AccR = HorizR.Step(C.RightMps - VRight, Dt);
        // Small-angle: forward accel needs nose down (pitch < 0), right accel needs right wing down.
        const double PitchDes = Clamp(-AccF / kGravityMps2, -Tune.MaxTiltRad, Tune.MaxTiltRad);
        const double RollDes = Clamp(AccR / kGravityMps2, -Tune.MaxTiltRad, Tune.MaxTiltRad);

        // Attitude P -> rate command -> rate PI -> axis command.
        const double RollRateDes = Clamp(Tune.AttitudeKp * (RollDes - Att.RollRad), -Tune.MaxRateCmdRadS, Tune.MaxRateCmdRadS);
        const double PitchRateDes = Clamp(Tune.AttitudeKp * (PitchDes - Att.PitchRad), -Tune.MaxRateCmdRadS, Tune.MaxRateCmdRadS);
        Out->Roll = RateRoll.Step(RollRateDes - E.BodyRatesRadS[0], Dt);
        Out->Pitch = RatePitch.Step(PitchRateDes - E.BodyRatesRadS[1], Dt);
        Out->Yaw = RateYaw.Step(C.YawRateRadS - E.BodyRatesRadS[2], Dt);

        // Vertical velocity PI -> acceleration -> throttle. Thrust scales with throttle^2, and the
        // vertical part of thrust is T cos(roll) cos(pitch):
        //   throttle = hover * sqrt((g + a_up) / (g cos(roll) cos(pitch))).
        const double VUp = -E.VelNedMps[2];
        const double AccUp = VertVel.Step(ClimbCmdMps - VUp, Dt);
        const double TiltCos = Clamp(std::cos(Att.RollRad) * std::cos(Att.PitchRad), 0.5, 1.0);
        const double Ratio = Clamp((kGravityMps2 + AccUp) / (kGravityMps2 * TiltCos), 0.0, 4.0);
        Out->Throttle = Clamp(Tune.HoverThrottle * std::sqrt(Ratio), 0.0, 1.0);
    }

    Tuning Tune;
    Mode CurrentMode = Mode::OnGround;
    double AltTargetM = 0.0;
    Pid VertVel, HorizF, HorizR, RateRoll, RatePitch, RateYaw;
};
}  // namespace DroneDemo
