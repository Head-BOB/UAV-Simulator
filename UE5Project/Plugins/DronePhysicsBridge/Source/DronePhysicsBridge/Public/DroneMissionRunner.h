// Tier: V
// DEMO ONLY headless mission runner: flies the scripted mission (DroneMission.h) with the DEMO controller
// against the real Rust physics through a step callback, logs a 50 Hz flight log with the electrical
// quantities of DroneElectrical.h, and builds the report bundle (DroneReportOutput.h) including the
// re-flown suggestions and the cross-environment table. UE-free: the caller supplies the step function
// (tools: ffi_step_physics from the shared library; Unreal: FDronePhysicsBridgeModule::StepPhysics), and
// the starting state. No wall clock, no hidden randomness: the wind seed is an input, so the same inputs
// give byte-identical reports on the same build.
//
// Step function contract: int Step(DroneState*, ThermalState*, const ControlInputs*, const WindInput*,
//   const EnvironmentInput*, const VehicleConfig*, DebugTelemetry*, FaultLatch*, double dt); 0 = success.
#pragma once

#include "DroneDemoControl.h"
#include "DroneEnvironment.h"
#include "DroneReportOutput.h"
#include "DroneWind.h"
#include "Generated/DroneEnvironmentsGenerated.h"
#include "Generated/drone_ffi.h"

#include <cmath>
#include <cstring>
#include <string>
#include <vector>

namespace DroneRun
{
constexpr double kDt = 0.002;                 // ADR-001 fixed step
constexpr int kStepsPerSample = 10;           // 50 Hz log
constexpr double kMaxTiltAbortRad = 1.4;      // about 80 degrees: the vehicle is not under control any more
constexpr double kHoverRefDensityKgM3 = DroneEnv::kHoverReferenceDensityKgM3;

// One motor's electrical operating point from its rpm and the air density (DroneElectrical.h formulas).
struct MotorPoint
{
    double CurrentA = 0.0;        // motor current
    double ElectricalPowerW = 0.0;  // V I
    double BatteryA = 0.0;        // share of the pack current
};

inline MotorPoint ComputeMotorPoint(const DroneReport::Build& B, double Rpm, double RhoKgM3)
{
    MotorPoint M;
    double P = 0.0, Q = 0.0, I = 0.0, V = 0.0, Ib = 0.0;
    DroneElec::ShaftPowerW(B.PropCp, RhoKgM3, Rpm, B.PropDiameterM, &P);
    DroneElec::ShaftTorqueNm(P, Rpm, &Q);
    DroneElec::MotorCurrentA(Q, Rpm, B.MotorKv, B.MotorNoLoadA, &I);
    DroneElec::MotorVoltageV(Rpm, B.MotorKv, I, B.MotorResistanceOhm, &V);
    DroneElec::BatteryCurrentA(V, I, B.BatteryVoc, B.EscEfficiency, &Ib);
    M.CurrentA = I;
    M.ElectricalPowerW = V * I;
    M.BatteryA = Ib;
    return M;
}

struct Inputs
{
    VehicleConfig Config;
    DroneReport::Build B;
    DroneState StartState;
    ThermalState StartThermal;
    EnvironmentInput Env = {0.0, 0.0};
    DroneWind::Params Wind;
    DroneMission::Params Mission;
    unsigned long long Seed = 12345;
};

struct Result
{
    DroneReport::Log Log;
    DroneReport::MissionOutcome Outcome;
    double SiteDensityKgM3 = 0.0;
};

// Hover throttle at sea-level ISA density for this config: the same formula as the core's aero::hover_throttle
// (sqrt(m g / (4 Ct rho0 (rpm/60)^2 D^4))). Used after a variant changes the mass.
inline double HoverThrottleAtReference(const VehicleConfig& C)
{
    const double N = C.max_rpm / 60.0;
    const double MaxThrustN = C.thrust_coeff * kHoverRefDensityKgM3 * N * N * std::pow(C.prop_diameter_m, 4.0);
    return std::sqrt(C.mass_kg * 9.80665 / (4.0 * MaxThrustN));
}

// Speed of the air through the props along the thrust axis, + = climbing: -(R^T v_air).z with body Z down and R the body-to-NED
// rotation of the (x, y, z, w) quaternion. Negative (descent) is returned as 0 because the UIUC data has no descent.
inline double AxialClimbMps(const float Q[4], const double VAirNed[3])
{
    // R^T v = rotate v by the conjugate quaternion: v' = v + 2 w (u x v) + 2 u x (u x v) with u = -(x, y, z).
    const double Ux = -static_cast<double>(Q[0]), Uy = -static_cast<double>(Q[1]), Uz = -static_cast<double>(Q[2]), W = Q[3];
    const double Cx = Uy * VAirNed[2] - Uz * VAirNed[1];
    const double Cy = Uz * VAirNed[0] - Ux * VAirNed[2];
    const double Cz = Ux * VAirNed[1] - Uy * VAirNed[0];
    const double Vz = VAirNed[2] + 2.0 * (W * Cz + Ux * Cy - Uy * Cx);
    const double Up = -Vz;
    return Up > 0.0 ? Up : 0.0;
}

inline double Hypot3(const double V[3])
{
    return std::sqrt(V[0] * V[0] + V[1] * V[1] + V[2] * V[2]);
}

// Flies one mission. StepFn is any callable with the signature in the file comment.
template <class StepFn>
Result Fly(StepFn&& Step, const Inputs& In)
{
    using namespace DroneReport;
    Result R;
    DroneState State = In.StartState;
    ThermalState Thermal = In.StartThermal;
    VehicleConfig Config = In.Config;
    FaultLatch Latch;
    std::memset(&Latch, 0, sizeof(Latch));
    DebugTelemetry Tele;
    std::memset(&Tele, 0, sizeof(Tele));

    // Air density at the origin and the hover throttle for it (thin or hot air needs more throttle).
    double SiteRho = kHoverRefDensityKgM3;
    DroneEnv::DensityAtHeightKgM3(In.Env.site_altitude_m, In.Env.temp_offset_c, &SiteRho);
    R.SiteDensityKgM3 = SiteRho;
    double Scale = 1.0;
    DroneEnv::HoverThrottleScale(SiteRho, &Scale);
    DroneDemo::Tuning Tune;
    Tune.HoverThrottle = HoverThrottleAtReference(Config) * Scale;
    if (Tune.HoverThrottle > 1.0) Tune.HoverThrottle = 1.0;  // the saturation finding reports it
    DroneDemo::Controller Ctl(Tune);

    DroneWind::Generator Gen(In.Seed);
    DroneWind::Sample Wind;
    std::memset(&Wind, 0, sizeof(Wind));
    DroneMission::Script Script(In.Mission);

    const long MaxSteps = static_cast<long>(In.Mission.MaxDurationS / kDt) + 10;
    R.Log.Samples.reserve(static_cast<std::size_t>(MaxSteps / kStepsPerSample) + 16);  // reserved once, never per step
    double UsedAh = 0.0;
    double Time = 0.0;
    int StepErrors = 0;
    double Rho = SiteRho;

    for (long K = 0; K < MaxSteps && !Script.Finished(); ++K)
    {
        DroneWind::Sample S;
        if (Gen.Step(In.Wind, kDt, &S)) Wind = S;
        const WindInput W = {{Wind.Total[0], Wind.Total[1], Wind.Total[2]}};

        double Quat[4] = {State.orientation[0], State.orientation[1], State.orientation[2], State.orientation[3]};
        const DroneDemo::Euler Att = DroneDemo::QuatToEuler(Quat);
        const double Altitude = -State.position[2];
        DroneMission::Pose Pose;
        Pose.TimeS = Time;
        Pose.North = State.position[0];
        Pose.East = State.position[1];
        Pose.AltitudeM = Altitude;
        Pose.YawRad = Att.YawRad;
        const DroneDemo::Command Cmd = Script.Next(Pose, Ctl);

        DroneDemo::Estimate E;
        E.AltitudeM = Altitude;
        for (int I = 0; I < 3; ++I)
        {
            E.VelNedMps[I] = State.velocity[I];
            E.BodyRatesRadS[I] = State.angular_velocity[I];
        }
        for (int I = 0; I < 4; ++I) E.QuatXyzw[I] = State.orientation[I];
        DroneDemo::Output O;
        if (!Ctl.Update(E, Cmd, kDt, &O)) ++StepErrors;

        if (Ctl.GetMode() != DroneDemo::Mode::OnGround)
        {
            ControlInputs C;
            C.throttle = static_cast<float>(O.Throttle);
            C.roll = static_cast<float>(O.Roll);
            C.pitch = static_cast<float>(O.Pitch);
            C.yaw = static_cast<float>(O.Yaw);
            const int Code = Step(&State, &Thermal, &C, &W, &In.Env, &Config, &Tele, &Latch, kDt);
            if (Code != 0)
            {
                ++StepErrors;
                Script.Abort("the physics step returned an error");
                R.Outcome.Reason = "physics step error code " + std::to_string(Code);
            }
            else if (Latch.physics_fault != 0)
            {
                Script.Abort("the physics fault latch was set");
            }
            if (DroneDemo::ShouldTouchDown(Ctl.GetMode(), -State.position[2]))
            {
                // Same ground handling as the pawn: level, at rest, on the ground, yaw kept.
                const double H = 0.5 * Att.YawRad;
                State.position[2] = 0.0;
                for (int I = 0; I < 3; ++I) State.velocity[I] = 0.0f, State.angular_velocity[I] = 0.0f;
                State.orientation[0] = 0.0f;
                State.orientation[1] = 0.0f;
                State.orientation[2] = static_cast<float>(std::sin(H));
                State.orientation[3] = static_cast<float>(std::cos(H));
                Ctl.NotifyOnGround();
            }
        }
        const double Tilt = std::fmax(std::fabs(Att.RollRad), std::fabs(Att.PitchRad));
        if (Tilt > kMaxTiltAbortRad && !Script.Finished()) Script.Abort("lost control (tilt above 80 degrees)");
        if (!std::isfinite(State.position[0]) || !std::isfinite(State.position[2])) Script.Abort("the state became non-finite");

        if (K % kStepsPerSample == 0)
        {
            Sample Sm;
            Sm.TimeS = Time;
            Sm.Phase = static_cast<int>(Script.GetPhase());
            Sm.AltitudeM = Altitude;
            Sm.TiltRad = Tilt;
            DroneEnv::DensityAtHeightKgM3(In.Env.site_altitude_m + (Altitude > 0.0 ? Altitude : 0.0), In.Env.temp_offset_c, &Rho);
            Sm.RhoKgM3 = Rho;
            double TotalBattery = 0.0;
            for (int M = 0; M < kMotors; ++M)
            {
                const double Rpm = Tele.motor_rpms[M];
                Sm.ThrustN[M] = Tele.motor_thrusts[M];
                Sm.Rpm[M] = Rpm;
                Sm.MotorTempC[M] = Tele.motor_temperatures_c[M];
                const MotorPoint Mp = ComputeMotorPoint(In.B, Rpm, Rho);
                Sm.CurrentA[M] = Mp.CurrentA;
                Sm.PowerW[M] = Mp.ElectricalPowerW;
                TotalBattery += Mp.BatteryA;
                if (Rpm >= In.B.SaturationThrottle * Config.max_rpm) Sm.Saturated = true;
            }
            Sm.BatteryA = TotalBattery;
            double Vt = In.B.BatteryVoc;
            DroneElec::PackTerminalV(In.B.BatteryVoc, TotalBattery, In.B.BatteryResistanceOhm, &Vt);
            Sm.PackV = Vt;
            UsedAh += TotalBattery * (kStepsPerSample * kDt) / 3600.0;
            Sm.UsedAh = UsedAh;
            Sm.AeroDragN = std::sqrt(double(Tele.aero_drag[0]) * Tele.aero_drag[0] + double(Tele.aero_drag[1]) * Tele.aero_drag[1] + double(Tele.aero_drag[2]) * Tele.aero_drag[2]);
            const double VAir[3] = {double(State.velocity[0]) - Wind.Total[0], double(State.velocity[1]) - Wind.Total[1], double(State.velocity[2]) - Wind.Total[2]};
            Sm.AxialClimbMps = AxialClimbMps(State.orientation, VAir);
            Sm.WindMps = Hypot3(Wind.Total);
            Sm.GustMps = Hypot3(Wind.Gust);
            Sm.CrossTrackM = Script.CrossTrackM(Pose);
            R.Log.Samples.push_back(Sm);
        }
        Time += kDt;
    }
    R.Outcome.Completed = Script.GetPhase() == DroneMission::Phase::Done;
    if (!R.Outcome.Completed && R.Outcome.Reason.empty())
        R.Outcome.Reason = Script.Finished() ? std::string(Script.FailureReason()) : std::string("the step budget ran out before the mission finished");
    R.Outcome.DurationS = Time;
    R.Outcome.StepErrors = StepErrors;
    return R;
}

// A copy of the base inputs with one improvement variant applied. Mass changes scale the inertia with the mass
// (ASSUMED proportional) and re-derive the hover throttle the core would give.
inline Inputs ApplyVariant(const Inputs& Base, const DroneReport::Variant& V)
{
    Inputs In = Base;
    if (V.CruiseMps > 0.0) In.Mission.CruiseMps = V.CruiseMps;
    if (V.MassDeltaKg != 0.0)
    {
        const double Old = In.Config.mass_kg;
        const double New = Old + V.MassDeltaKg;
        for (double& I : In.Config.inertia_kgm2) I *= New / Old;
        In.Config.mass_kg = New;
        In.Config.hover_throttle = HoverThrottleAtReference(In.Config);
        In.B.MassKg = New;
    }
    if (V.BatteryAh > 0.0) In.B.BatteryCapacityAh = V.BatteryAh;
    if (V.ArmWallM > 0.0) In.B.ArmWallM = V.ArmWallM;
    return In;
}

// Inputs for one environment: atmosphere for the core and the wind for the generator.
inline Inputs MakeInputs(const VehicleConfig& Config, const DroneState& Start, const ThermalState& StartThermal, const DroneReport::EnvInfo& E)
{
    Inputs In;
    In.Config = Config;
    In.B = DroneReport::Build::Reference();
    In.B.MassKg = Config.mass_kg;
    In.B.MaxRpm = Config.max_rpm;
    In.B.PropCt = Config.thrust_coeff;
    In.B.PropDiameterM = Config.prop_diameter_m;
    In.StartState = Start;
    In.StartThermal = StartThermal;
    In.Env.site_altitude_m = E.SiteAltitudeM;
    In.Env.temp_offset_c = E.TempOffsetC;
    In.Wind.SteadySpeedMps = E.WindSpeedMps;
    In.Wind.SteadyDirectionDeg = E.WindTowardDeg;
    In.Wind.GustSigmaMps = E.GustSigmaMps;
    In.Wind.bGustsEnabled = E.GustSigmaMps > 0.0;
    return In;
}

// An EnvInfo from the generated environment table row (typed ambient temperature -> offset from the ISA day).
inline DroneReport::EnvInfo EnvFromRow(const DroneEnvTable::Row& Row)
{
    DroneReport::EnvInfo E;
    E.Id = Row.Id;
    E.Name = Row.Name;
    E.SiteAltitudeM = Row.SiteAltitudeM;
    E.AmbientTempC = Row.AmbientTempC;
    E.HumidityPct = Row.HumidityPct;
    E.WindSpeedMps = Row.WindSpeedMps;
    E.WindTowardDeg = Row.WindTowardDeg;
    E.GustSigmaMps = Row.GustSigmaMps;
    E.TempOffsetC = DroneEnv::TempOffsetC(Row.SiteAltitudeM, Row.AmbientTempC);
    return E;
}

// Flies the flown environment, attaches tested suggestions, and flies every environment in Envs for the
// comparison table (the flown one is reused, not flown twice). GeneratedUtc is text from the caller.
template <class StepFn>
DroneReport::Bundle BuildBundle(StepFn&& Step, const VehicleConfig& Config, const DroneState& Start, const ThermalState& StartThermal,
                                const DroneReport::EnvInfo& Flown, const std::vector<DroneReport::EnvInfo>& Envs, unsigned long long Seed,
                                const std::string& GeneratedUtc)
{
    using namespace DroneReport;
    Bundle B;
    B.GeneratedUtc = GeneratedUtc;
    B.Seed = Seed;
    B.Flown = Flown;
    Inputs Base = MakeInputs(Config, Start, StartThermal, Flown);
    Base.Seed = Seed;
    const Result Main = Fly(Step, Base);
    B.DensityKgM3 = Main.SiteDensityKgM3;
    B.Outcome = Main.Outcome;
    B.A = Analyze(Main.Log, Base.B, Main.Outcome);

    ReflightFn Reflight = [&](const Variant& V, Analysis* Out) {
        const Inputs In = ApplyVariant(Base, V);
        if (V.Reflight)
        {
            const Result R = Fly(Step, In);
            *Out = Analyze(R.Log, In.B, R.Outcome);
        }
        else
        {
            *Out = Analyze(Main.Log, In.B, Main.Outcome);  // same flight, changed part
        }
        return true;
    };
    AttachSuggestions(B.A, Reflight);

    for (const EnvInfo& E : Envs)
    {
        CrossRow Row;
        Row.Env = E;
        if (E.Id == Flown.Id)
        {
            Row.Completed = Main.Outcome.Completed;
            Row.MinMargin = B.A.MinMargin;
            Row.Limiting = B.A.Limiting;
            Row.V = B.A.V;
            Row.Reason = B.A.VerdictReason;
            Row.DensityKgM3 = Main.SiteDensityKgM3;
        }
        else
        {
            Inputs In = MakeInputs(Config, Start, StartThermal, E);
            In.Seed = Seed;
            const Result R = Fly(Step, In);
            const Analysis A = Analyze(R.Log, In.B, R.Outcome);
            Row.Completed = R.Outcome.Completed;
            Row.MinMargin = A.MinMargin;
            Row.Limiting = A.Limiting;
            Row.V = A.V;
            Row.Reason = A.VerdictReason;
            Row.DensityKgM3 = R.SiteDensityKgM3;
        }
        B.Cross.push_back(Row);
    }
    return B;
}
}  // namespace DroneRun
