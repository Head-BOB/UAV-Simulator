// Tier: V
// Tests for DroneMissionRunner.h against the REAL Rust physics (libphysics_core.so): the headless mission,
// the electrical numbers, determinism, environment effect, forced failures. DEMO checks: pass bands are
// engineering choices marked TODO(HUMAN-VERIFY: demo value); the hand values are derived in the comments.
//   g++ -std=c++17 -Wall -Wextra -Werror -I UE5Project/Plugins/DronePhysicsBridge/Source/DronePhysicsBridge/Public tools/cpp_ffi_check/mission_runner_test.cpp -o mission_runner_test -Lphysics_core/target/release -lphysics_core -Wl,-rpath,$PWD/physics_core/target/release
// Run from the repo root (it loads offline_pipeline/geometry/quad_ref_build_config.json).

#include "DroneMissionRunner.h"

#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

static int g_failures = 0;

static void Check(bool ok, const char* name)
{
    if (!ok)
    {
        std::printf("FAIL: %s\n", name);
        ++g_failures;
    }
}

static bool Near(double a, double b, double tol)
{
    return std::fabs(a - b) <= tol;
}

static bool Has(const std::string& S, const std::string& Sub)
{
    return S.find(Sub) != std::string::npos;
}

static auto RealStep = [](DroneState* S, ThermalState* T, const ControlInputs* C, const WindInput* W, const EnvironmentInput* E, const VehicleConfig* Cfg,
                          DebugTelemetry* Tele, FaultLatch* L, double Dt) { return ffi_step_physics(S, T, C, W, E, nullptr, nullptr, Cfg, Tele, L, Dt); };

struct World
{
    VehicleConfig Config;
    DroneState Start;
    ThermalState Thermal;
    std::vector<DroneReport::EnvInfo> Envs;
    World()
    {
        if (ffi_load_vehicle_config("offline_pipeline/geometry/quad_ref_build_config.json", &Config) != 0)
        {
            std::printf("FAIL: cannot load the reference config (run from the repo root)\n");
            std::exit(1);
        }
        Start = ffi_create_default_drone_state();
        std::memset(&Thermal, 0, sizeof(Thermal));
        for (double& T : Thermal.motor_temp_c) T = 20.0;
        for (std::size_t I = 0; I < DroneEnvTable::kRowCount; ++I) Envs.push_back(DroneRun::EnvFromRow(DroneEnvTable::kRows[I]));
    }
};

static DroneReport::EnvInfo Calm(const DroneReport::EnvInfo& E)
{
    DroneReport::EnvInfo C = E;
    C.WindSpeedMps = 0.0;
    C.GustSigmaMps = 0.0;
    return C;
}

static void TestHoverElectricsMatchHandCalc(const World& W)
{
    // Beach, calm: site 0 m, 26 C -> dT = 26 - 15 = +11 K. Density at the 20 m flight height (python3 f64): 1.1776840077949462 kg/m3.
    // Hover: 4 Ct rho n^2 D^4 = m g with Ct 0.100256, D 0.3048, m 1.516 -> n = 60.392 rev/s = 3623.5 rpm.
    // Per motor: P = Cp rho n^3 D^5 = 21.752 W, I = P/(2 pi n)/Kt + I0 = 4.7021 A, V = rpm/Kv + I Rm = 5.5150 V,
    // battery current = V I / (0.95 * 14.8) = 1.8444 A, four motors 7.3775 A.
    const DroneReport::EnvInfo E = Calm(W.Envs[0]);
    DroneRun::Inputs In = DroneRun::MakeInputs(W.Config, W.Start, W.Thermal, E);
    const DroneRun::Result R = DroneRun::Fly(RealStep, In);
    Check(R.Outcome.Completed, "calm Beach mission completes");
    double Rpm = 0.0, Cur = 0.0, Bat = 0.0, Pow = 0.0;
    int N = 0;
    double HoverStart = -1.0;
    for (const DroneReport::Sample& S : R.Log.Samples)
        if (S.Phase == static_cast<int>(DroneMission::Phase::Hover) && HoverStart < 0.0) HoverStart = S.TimeS;
    for (const DroneReport::Sample& S : R.Log.Samples)
    {
        if (S.Phase != static_cast<int>(DroneMission::Phase::Hover) || S.TimeS < HoverStart + 4.0) continue;  // let the hold settle
        Rpm += (S.Rpm[0] + S.Rpm[1] + S.Rpm[2] + S.Rpm[3]) / 4.0;
        Cur += (S.CurrentA[0] + S.CurrentA[1] + S.CurrentA[2] + S.CurrentA[3]) / 4.0;
        Bat += S.BatteryA;
        Pow += (S.PowerW[0] + S.PowerW[1] + S.PowerW[2] + S.PowerW[3]) / 4.0;
        ++N;
    }
    Check(N > 100, "enough hover samples");
    if (N == 0) return;
    Rpm /= N;
    Cur /= N;
    Bat /= N;
    Pow /= N;
    Check(Near(Rpm, 3623.5, 0.02 * 3623.5), "hover rpm within 2 % of 3623.5");
    // 2 % bands: using the sea-level density instead of the actual 1.1777 would put the current 3.9 % too high.
    Check(Near(Cur, 4.7021, 0.02 * 4.7021), "hover motor current within 2 % of 4.702 A");
    Check(Near(Bat, 7.3775, 0.02 * 7.3775), "hover battery current within 2 % of 7.378 A");
    // Electrical power per motor V I = 5.5150 V * 4.7021 A = 25.932 W.
    Check(Near(Pow, 25.932, 0.03 * 25.932), "hover electrical power within 3 % of 25.93 W");
    // The vehicle holds 20 m: altitude within 0.5 m during the hover.
    bool AltOk = true;
    for (const DroneReport::Sample& S : R.Log.Samples)
        if (S.Phase == static_cast<int>(DroneMission::Phase::Hover) && S.TimeS >= HoverStart + 4.0 && std::fabs(S.AltitudeM - 20.0) > 0.5) AltOk = false;
    Check(AltOk, "hover holds 20 m within 0.5 m");
    // Energy: at least hover current * duration (7.3775 A * duration / 3600), at most 3x that. Duration is ~130 s.
    const double Used = R.Log.Samples.back().UsedAh;
    const double Floor = 0.9 * 7.3775 * R.Outcome.DurationS / 3600.0;
    Check(Used >= Floor && Used <= 3.0 * Floor, "battery energy between hover-only and 3x hover-only");
    // Axial climb speed: the climb phase flies at up to 2 m/s, so the logged axial inflow peaks near 2 m/s and the report shows a few percent thrust loss.
    double MaxAxial = 0.0;
    for (const DroneReport::Sample& S : R.Log.Samples) MaxAxial = std::fmax(MaxAxial, S.AxialClimbMps);
    Check(MaxAxial > 1.5 && MaxAxial < 2.5, "peak axial climb speed near 2 m/s");
    const DroneReport::Analysis An = DroneReport::Analyze(R.Log, In.B, R.Outcome);
    const DroneReport::Finding* Ax = nullptr;
    for (const DroneReport::Finding& F : An.Findings) if (F.Id == "prop_axial") Ax = &F;
    Check(Ax && !Ax->Assessed && Ax->PeakValue > 2.0 && Ax->PeakValue < 9.0, "axial thrust loss between 2 % and 9 % in the climb");
    // Phases are visited in order and the log is at 50 Hz.
    Check(Near(R.Log.Samples[1].TimeS - R.Log.Samples[0].TimeS, 0.02, 1e-9), "log at 50 Hz");
    Check(R.Log.Samples.front().Phase == static_cast<int>(DroneMission::Phase::Takeoff), "starts in takeoff");
    Check(R.Log.Samples.front().TimeS == 0.0, "the first sample is at t = 0");
}

static void TestAxialClimbGeometry()
{
    // Level (identity quaternion): body Z is down, so a vertical velocity of -2 m/s in NED (up) is a 2 m/s climb through the props.
    const float Level[4] = {0.0f, 0.0f, 0.0f, 1.0f};
    const double Up2[3] = {0.0, 0.0, -2.0}, Down2[3] = {0.0, 0.0, 2.0}, Fwd5[3] = {5.0, 0.0, 0.0};
    Check(Near(DroneRun::AxialClimbMps(Level, Up2), 2.0, 1e-12), "level climb 2 m/s");
    Check(DroneRun::AxialClimbMps(Level, Down2) == 0.0, "descent is not reported");
    Check(Near(DroneRun::AxialClimbMps(Level, Fwd5), 0.0, 1e-12), "level forward flight has no axial component");
    // q = (0, sin45, 0, cos45) is +90 deg about body Y: R_y(90) maps body X -> NED -Z (up) and body Z -> NED +X (north). The thrust axis
    // is -Z body, so it points SOUTH. Moving south at 5 m/s is moving along the thrust direction: a 5 m/s axial climb. Moving north at
    // 5 m/s is against it: a descent, reported as 0.
    const float S = 0.70710678f;
    const float Pitched[4] = {0.0f, S, 0.0f, S};
    const double South5[3] = {-5.0, 0.0, 0.0};
    Check(Near(DroneRun::AxialClimbMps(Pitched, South5), 5.0, 1e-5), "thrust axis pointing south: 5 m/s south is a 5 m/s axial climb");
    Check(DroneRun::AxialClimbMps(Pitched, Fwd5) == 0.0, "moving against the thrust axis is a descent: 0");
}

static void TestEnvironmentEffect(const World& W)
{
    // Thrust headroom scales with air density (Tmax ~ rho): the Mountains margin / Beach margin ~ rho ratio.
    // Beach vehicle-height density 1.17996752 (report), Mountains 0.94556319: ratio 0.80137 (hover thrust = weight in both).
    const DroneReport::EnvInfo Beach = Calm(W.Envs[0]), Mountains = Calm(W.Envs[2]);
    DroneRun::Inputs InB = DroneRun::MakeInputs(W.Config, W.Start, W.Thermal, Beach);
    DroneRun::Inputs InM = DroneRun::MakeInputs(W.Config, W.Start, W.Thermal, Mountains);
    const DroneRun::Result RB = DroneRun::Fly(RealStep, InB), RM = DroneRun::Fly(RealStep, InM);
    const DroneReport::Analysis AB = DroneReport::Analyze(RB.Log, InB.B, RB.Outcome), AM = DroneReport::Analyze(RM.Log, InM.B, RM.Outcome);
    const DroneReport::Finding* TB = nullptr;
    const DroneReport::Finding* TM = nullptr;
    for (const DroneReport::Finding& F : AB.Findings) if (F.Id == "prop_thrust") TB = &F;
    for (const DroneReport::Finding& F : AM.Findings) if (F.Id == "prop_thrust") TM = &F;
    Check(TB && TM && TM->Margin < TB->Margin, "thinner air: smaller thrust margin");
    if (TB && TM)
    {
        const double Ratio = TM->Margin / TB->Margin;
        Check(Near(Ratio, RM.SiteDensityKgM3 / RB.SiteDensityKgM3, 0.05), "thrust margin ratio follows the density ratio (+-0.05)");
    }
    // Site densities (python3): Beach 26 C, 0 m: 101325/(287.05*299.15) = 1.17996...; Mountains 2500 m, 2 C: 0.8..: checked against the formula.
    double Rho = 0.0;
    Check(DroneEnv::DensityKgM3(0.0, 26.0, &Rho) && Near(RB.SiteDensityKgM3, Rho, 1e-12), "site density equals the formula (beach)");
    Check(Near(RB.SiteDensityKgM3, 101325.0 / (287.05 * 299.15), 1e-9), "beach density hand calc 101325/(287.05*299.15)");
}

static void TestDeterminismAndSeed(const World& W)
{
    const DroneReport::EnvInfo E = W.Envs[3];  // Desert: wind 6 m/s, gust sigma 2
    const DroneReport::Bundle A = DroneRun::BuildBundle(RealStep, W.Config, W.Start, W.Thermal, E, {E}, 12345, "T");
    const DroneReport::Bundle B = DroneRun::BuildBundle(RealStep, W.Config, W.Start, W.Thermal, E, {E}, 12345, "T");
    Check(DroneReport::ToJson(A) == DroneReport::ToJson(B), "same inputs and seed: byte-identical JSON");
    Check(DroneReport::ToHtml(A) == DroneReport::ToHtml(B), "same inputs and seed: byte-identical HTML");
    const DroneReport::Bundle C = DroneRun::BuildBundle(RealStep, W.Config, W.Start, W.Thermal, E, {E}, 999, "T");
    Check(DroneReport::ToJson(A) != DroneReport::ToJson(C), "a different gust seed changes the report");
    Check(A.Outcome.Completed && A.Cross.size() == 1 && A.Cross[0].Env.Id == "Desert", "bundle has the flown environment row");
}

static void TestHeavyVehicleFails(const World& W)
{
    // 8 kg on the same motors: hover needs sqrt(8/1.516) * 0.4386 = 1.008 > 1 of full throttle at sea level, so it cannot hold
    // 20 m. The mission must not complete, and the verdict must be "not recommended" with the failure named.
    DroneReport::EnvInfo E = Calm(W.Envs[0]);
    DroneRun::Inputs In = DroneRun::MakeInputs(W.Config, W.Start, W.Thermal, E);
    In.Config.mass_kg = 8.0;
    In.Config.hover_throttle = DroneRun::HoverThrottleAtReference(In.Config);
    for (double& I : In.Config.inertia_kgm2) I *= 8.0 / 1.516;
    In.B.MassKg = 8.0;
    In.Mission.MaxDurationS = 120.0;
    const DroneRun::Result R = DroneRun::Fly(RealStep, In);
    const DroneReport::Analysis A = DroneReport::Analyze(R.Log, In.B, R.Outcome);
    Check(!R.Outcome.Completed, "an 8 kg vehicle does not complete the mission");
    Check(A.V == DroneElec::Verdict::NotRecommended && A.Findings[0].Id == "mission" && !R.Outcome.Reason.empty(), "failure is the first finding, not recommended");
    const DroneReport::Finding* Sat = nullptr;
    for (const DroneReport::Finding& F : A.Findings) if (F.Id == "saturation") Sat = &F;
    Check(Sat && Sat->Margin < 1.0, "motors were saturated: saturation margin below 1");
}

static void TestHeavyButFlyable(const World& W)
{
    // 6 kg: hover throttle sqrt(6/1.516) * 0.4386 = 0.8726 at sea level; the climb and the corners need more, so a motor reaches
    // 98 % throttle for a while: the saturation finding must be weaker than the 1.5 "within limits" line.
    DroneReport::EnvInfo E = Calm(W.Envs[0]);
    DroneRun::Inputs In = DroneRun::MakeInputs(W.Config, W.Start, W.Thermal, E);
    In.Config.mass_kg = 6.0;
    In.Config.hover_throttle = DroneRun::HoverThrottleAtReference(In.Config);
    for (double& I : In.Config.inertia_kgm2) I *= 6.0 / 1.516;
    In.B.MassKg = 6.0;
    const DroneRun::Result R = DroneRun::Fly(RealStep, In);
    const DroneReport::Analysis A = DroneReport::Analyze(R.Log, In.B, R.Outcome);
    const DroneReport::Finding* Thrust = nullptr;
    for (const DroneReport::Finding& F : A.Findings) if (F.Id == "prop_thrust") Thrust = &F;
    // Hover thrust per motor = 6 * 9.80665 / 4 = 14.71 N against Tmax = 0.100256 * 1.18 * 135^2 * 0.3048^4 = 18.6 N: margin about 1.26 at best.
    Check(Thrust && Thrust->Margin < 1.3 && Thrust->Margin > 0.9, "6 kg: thrust margin between 0.9 and 1.3 (hand: 18.6 / 14.7 = 1.26 at hover)");
    Check(A.MinMargin < DroneElec::kWithinLimitsMargin, "6 kg is not 'within limits'");
}

static void TestStepErrorAndFailureReasons(const World& W)
{
    // A step function that fails after 1000 steps: the mission must stop with the reason, never complete.
    int Calls = 0;
    auto Failing = [&](DroneState* S, ThermalState* T, const ControlInputs* C, const WindInput* Wd, const EnvironmentInput* E, const VehicleConfig* Cfg,
                       DebugTelemetry* Tele, FaultLatch* L, double Dt) {
        if (++Calls > 1000) return 6;
        return RealStep(S, T, C, Wd, E, Cfg, Tele, L, Dt);
    };
    DroneRun::Inputs In = DroneRun::MakeInputs(W.Config, W.Start, W.Thermal, Calm(W.Envs[0]));
    const DroneRun::Result R = DroneRun::Fly(Failing, In);
    Check(!R.Outcome.Completed && Has(R.Outcome.Reason, "error code 6") && R.Outcome.StepErrors >= 1, "a failing step stops the mission with the code");
    Check(R.Outcome.DurationS < 5.0, "it stops right away (about 2 s of sim time)");
}

static void TestLostControlAbort(const World& W)
{
    // After 2000 steps the stand-in physics rolls the vehicle by 100 degrees (q = (sin 50, 0, 0, cos 50)): tilt above 80 deg.
    int Calls = 0;
    auto Tumble = [&](DroneState* S, ThermalState* T, const ControlInputs* C, const WindInput* Wd, const EnvironmentInput* E, const VehicleConfig* Cfg,
                      DebugTelemetry* Tele, FaultLatch* L, double Dt) {
        const int Code = RealStep(S, T, C, Wd, E, Cfg, Tele, L, Dt);
        if (++Calls > 2000)
        {
            S->orientation[0] = static_cast<float>(std::sin(0.8726646259971648));  // 50 deg
            S->orientation[1] = 0.0f;
            S->orientation[2] = 0.0f;
            S->orientation[3] = static_cast<float>(std::cos(0.8726646259971648));
        }
        return Code;
    };
    DroneRun::Inputs In = DroneRun::MakeInputs(W.Config, W.Start, W.Thermal, Calm(W.Envs[0]));
    const DroneRun::Result R = DroneRun::Fly(Tumble, In);
    Check(!R.Outcome.Completed && Has(R.Outcome.Reason, "lost control"), "a tumbling vehicle aborts with 'lost control'");
}

static void TestVariantApplication(const World& W)
{
    DroneRun::Inputs Base = DroneRun::MakeInputs(W.Config, W.Start, W.Thermal, W.Envs[0]);
    const DroneReport::Variant* Light = DroneReport::FindVariant("light");
    const DroneReport::Variant* Batt = DroneReport::FindVariant("battery6");
    const DroneReport::Variant* Slow = DroneReport::FindVariant("slow");
    const DroneReport::Variant* Arm = DroneReport::FindVariant("arm2mm");
    Check(Light && Batt && Slow && Arm, "variants exist");
    if (!(Light && Batt && Slow && Arm)) return;
    // Light: mass 1.516 - 0.15 = 1.366 kg; inertia scales by 1.366/1.516 = 0.90105; hover throttle = sqrt(1.366/1.516) * 0.4386224 = 0.41636 (python3).
    const DroneRun::Inputs L = DroneRun::ApplyVariant(Base, *Light);
    Check(Near(L.Config.mass_kg, 1.366, 1e-12) && Near(L.Config.inertia_kgm2[0], Base.Config.inertia_kgm2[0] * 1.366 / 1.516, 1e-12), "light: mass and inertia");
    Check(Near(L.Config.hover_throttle, std::sqrt(1.366 / 1.516) * 0.4386224371767256, 5e-4), "light: hover throttle follows sqrt(mass)");
    // Battery: 6 Ah and +0.09 kg.
    const DroneRun::Inputs Bt = DroneRun::ApplyVariant(Base, *Batt);
    Check(Bt.B.BatteryCapacityAh == 6.0 && Near(Bt.Config.mass_kg, 1.606, 1e-12), "battery6: capacity and mass");
    // Slow: cruise 3 m/s; the base is untouched.
    const DroneRun::Inputs Sl = DroneRun::ApplyVariant(Base, *Slow);
    Check(Sl.Mission.CruiseMps == 3.0 && Base.Mission.CruiseMps == 5.0, "slow: cruise 3 m/s, base unchanged");
    const DroneRun::Inputs Ar = DroneRun::ApplyVariant(Base, *Arm);
    Check(Near(Ar.B.ArmWallM, 0.002, 1e-15) && Base.B.ArmWallM == DroneRef::kFrameArmTubeWallM && Near(Ar.Config.mass_kg, Base.Config.mass_kg, 0.0), "arm2mm: wall only");
}

int main()
{
    const World W;
    TestVariantApplication(W);
    TestAxialClimbGeometry();
    TestHoverElectricsMatchHandCalc(W);
    TestEnvironmentEffect(W);
    TestStepErrorAndFailureReasons(W);
    TestLostControlAbort(W);
    TestDeterminismAndSeed(W);
    TestHeavyButFlyable(W);
    TestHeavyVehicleFails(W);
    if (g_failures == 0)
    {
        std::printf("mission_runner_test: all passed\n");
        return 0;
    }
    std::printf("mission_runner_test: %d failure(s)\n", g_failures);
    return 1;
}
