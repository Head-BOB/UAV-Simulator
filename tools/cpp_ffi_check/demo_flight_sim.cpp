// Tier: V
// Closed-loop check of the DEMO flight controller (DroneDemoControl.h) and the demo wind
// (DroneWind.h) against the REAL Rust physics (libphysics_core.so). Not flight code, and not a
// validation of the controller: it checks stability and limits (altitude held, speed reached,
// tilt bounded, no step error, deterministic), not accuracy. Pass bands are engineering choices
// for a demo, marked TODO(HUMAN-VERIFY: demo value).
//
// From the repo root (needs `cargo build --release` in physics_core/ first):
//   g++ -std=c++17 -Wall -Wextra -Werror -I UE5Project/Plugins/DronePhysicsBridge/Source/DronePhysicsBridge/Public tools/cpp_ffi_check/demo_flight_sim.cpp -o demo_flight_sim -Lphysics_core/target/release -lphysics_core -Wl,-rpath,$PWD/physics_core/target/release
//   ./demo_flight_sim
// Lives outside the plugin Source/ tree so UnrealBuildTool never compiles it.

#include "DroneDemoControl.h"
#include "DroneEnvironment.h"
#include "DroneWind.h"
#include "Generated/drone_ffi.h"

#include <cmath>
#include <cstdio>
#include <cstring>
#include <cstdint>

static int g_failures = 0;

static void Check(bool ok, const char* name)
{
    if (!ok)
    {
        std::printf("FAIL: %s\n", name);
        ++g_failures;
    }
}

constexpr double kDt = 0.002;

struct Sim
{
    DroneState State;
    ThermalState Thermal;
    VehicleConfig Config;
    EnvironmentInput Env = {0.0, 0.0};  // ISA standard day at sea level; scenarios may change it
    FaultLatch Latch;
    DebugTelemetry Tele;
    DroneDemo::Controller Ctl;
    DroneWind::Generator Gen;
    DroneWind::Params WindParams;
    DroneWind::Sample Wind;
    int BadSteps = 0;
    double MaxTilt = 0.0;
    double MinAltitude = 1e9;

    explicit Sim(const VehicleConfig& C, std::uint64_t Seed = 12345)
        : State(ffi_create_default_drone_state()), Thermal(), Config(C), Latch{0}, Tele(), Ctl(MakeTuning(C)), Gen(Seed)
    {
        std::memset(&Thermal, 0, sizeof(Thermal));
        for (double& T : Thermal.motor_temp_c) T = 20.0;
        std::memset(&Tele, 0, sizeof(Tele));
        WindParams.bGustsEnabled = false;  // scenarios opt in
        std::memset(&Wind, 0, sizeof(Wind));
    }

    static DroneDemo::Tuning MakeTuning(const VehicleConfig& C)
    {
        DroneDemo::Tuning T;
        T.HoverThrottle = C.hover_throttle;
        return T;
    }

    double Altitude() const { return -State.position[2]; }

    void Step(const DroneDemo::Command& Cmd)
    {
        DroneWind::Sample S;
        if (Gen.Step(WindParams, kDt, &S)) Wind = S;
        WindInput W = {{Wind.Total[0], Wind.Total[1], Wind.Total[2]}};

        DroneDemo::Estimate E;
        E.AltitudeM = Altitude();
        for (int I = 0; I < 3; ++I)
        {
            E.VelNedMps[I] = State.velocity[I];
            E.BodyRatesRadS[I] = State.angular_velocity[I];
        }
        for (int I = 0; I < 4; ++I) E.QuatXyzw[I] = State.orientation[I];

        DroneDemo::Output O;
        if (!Ctl.Update(E, Cmd, kDt, &O)) ++BadSteps;
        if (Ctl.GetMode() == DroneDemo::Mode::OnGround) return;  // pinned: no physics step

        ControlInputs In;
        In.throttle = static_cast<float>(O.Throttle);
        In.roll = static_cast<float>(O.Roll);
        In.pitch = static_cast<float>(O.Pitch);
        In.yaw = static_cast<float>(O.Yaw);
        const int Code = ffi_step_physics(&State, &Thermal, &In, &W, &Env, nullptr, nullptr, &Config, &Tele, &Latch, kDt);
        if (Code != 0) ++BadSteps;

        const DroneDemo::Euler Att = DroneDemo::QuatToEuler(E.QuatXyzw);
        const double Tilt = std::fmax(std::fabs(Att.RollRad), std::fabs(Att.PitchRad));
        if (Tilt > MaxTilt) MaxTilt = Tilt;
        if (Altitude() < MinAltitude) MinAltitude = Altitude();

        if (DroneDemo::ShouldTouchDown(Ctl.GetMode(), Altitude()))
        {
            // Same pinning the pawn does: level, at rest, on the ground, yaw kept.
            const DroneDemo::Euler A = DroneDemo::QuatToEuler(E.QuatXyzw);
            const double H = 0.5 * A.YawRad;
            State.position[2] = 0.0;
            for (int I = 0; I < 3; ++I) State.velocity[I] = 0.0f, State.angular_velocity[I] = 0.0f;
            State.orientation[0] = 0.0f;
            State.orientation[1] = 0.0f;
            State.orientation[2] = static_cast<float>(std::sin(H));
            State.orientation[3] = static_cast<float>(std::cos(H));
            Ctl.NotifyOnGround();
        }
    }

    void Run(double Seconds, const DroneDemo::Command& Cmd)
    {
        const int N = static_cast<int>(Seconds / kDt + 0.5);
        for (int I = 0; I < N; ++I) Step(Cmd);
    }

    double Speed() const
    {
        return std::sqrt(double(State.velocity[0]) * State.velocity[0] + double(State.velocity[1]) * State.velocity[1]);
    }
};

// The shipped test config and the demo's reference build (APC 12x6E, MN3508; masses ASSUMED): the same
// closed-loop checks run on both, so a controller that only works for one vehicle is caught.
static const char* const kConfigPaths[] = {"offline_pipeline/geometry/vehicle_config.json",
                                           "offline_pipeline/geometry/quad_ref_build_config.json"};

static bool LoadConfig(const char* Path, VehicleConfig* C)
{
    return ffi_load_vehicle_config(Path, C) == 0;
}

static DroneDemo::Command None() { return DroneDemo::Command(); }

static void TestTakeoffAndHover(const VehicleConfig& C)
{
    Sim S(C);
    Check(S.Ctl.RequestTakeoff(S.Altitude()), "takeoff accepted");
    S.Run(12.0, None());
    Check(S.BadSteps == 0, "takeoff: no step error");
    Check(std::fabs(S.Altitude() - 2.0) < 0.2, "takeoff: holds 2 m +-0.2");
    Check(S.Ctl.GetMode() == DroneDemo::Mode::Flying, "takeoff: ends in Flying");
    Check(S.MaxTilt < 0.1, "takeoff: tilt stays small");
    Check(S.Speed() < 0.1, "hover: horizontal speed near zero");
}

static void TestForwardClimbYaw(const VehicleConfig& C)
{
    Sim S(C);
    S.Ctl.RequestTakeoff(S.Altitude());
    S.Run(8.0, None());
    DroneDemo::Command Fwd;
    Fwd.ForwardMps = 2.0;
    S.Run(8.0, Fwd);
    // Heading is north (yaw 0), so forward is +North.
    Check(std::fabs(S.State.velocity[0] - 2.0) < 0.2, "forward 2 m/s reached (+-0.2)");
    Check(std::fabs(S.State.velocity[1]) < 0.2, "forward: no sideways drift");
    Check(std::fabs(S.Altitude() - 2.0) < 0.3, "forward: altitude held +-0.3");
    Check(S.MaxTilt < 0.4, "forward: tilt within limit");

    DroneDemo::Command Climb;
    Climb.ForwardMps = 0.0;
    Climb.ClimbMps = 1.0;
    S.Run(3.0, Climb);
    S.Run(6.0, None());
    Check(std::fabs(S.Altitude() - 5.0) < 0.4, "climb 1 m/s for 3 s gives 5 m +-0.4");

    DroneDemo::Command Yaw;
    Yaw.YawRateRadS = 1.0;
    S.Run(3.0, Yaw);
    Check(std::fabs(double(S.State.angular_velocity[2]) - 1.0) < 0.15, "yaw rate 1 rad/s reached");
    Check(S.BadSteps == 0, "maneuvers: no step error");
}

static void TestSidewaysAndHeadingFrame(const VehicleConfig& C)
{
    Sim S(C);
    S.Ctl.RequestTakeoff(S.Altitude());
    S.Run(8.0, None());
    // Right at 2 m/s while heading north: velocity goes to +East (NED y).
    DroneDemo::Command Right;
    Right.RightMps = 2.0;
    S.Run(8.0, Right);
    Check(std::fabs(S.State.velocity[1] - 2.0) < 0.2, "right 2 m/s: +East velocity reached (+-0.2)");
    Check(std::fabs(S.State.velocity[0]) < 0.2, "right: no north drift");
    Check(std::fabs(S.Altitude() - 2.0) < 0.3, "right: altitude held +-0.3");
    // Turn to face east (yaw +90 deg at 1 rad/s), then forward 2 m/s must move EAST: the commands
    // are in the heading frame.
    S.Run(0.2, None());
    DroneDemo::Command Turn;
    Turn.YawRateRadS = 1.0;
    S.Run(1.5707963267948966, Turn);
    S.Run(2.0, None());
    DroneDemo::Command Fwd;
    Fwd.ForwardMps = 2.0;
    S.Run(8.0, Fwd);
    Check(std::fabs(S.State.velocity[1] - 2.0) < 0.3, "facing east: forward is +East");
    Check(std::fabs(S.State.velocity[0]) < 0.3, "facing east: no north component");
    Check(S.BadSteps == 0, "sideways: no step error");
}

static void TestSteadyWind(const VehicleConfig& C)
{
    Sim S(C);
    S.WindParams.SteadySpeedMps = 8.0;
    S.WindParams.SteadyDirectionDeg = 180.0;  // blows toward south: a head wind for a north-facing quad
    S.Ctl.RequestTakeoff(S.Altitude());
    S.Run(20.0, None());
    Check(S.BadSteps == 0, "wind: no step error");
    Check(S.Speed() < 0.3, "8 m/s wind: ground speed held near zero");
    Check(std::fabs(S.Altitude() - 2.0) < 0.3, "8 m/s wind: altitude held +-0.3");
    // The wind blows toward south (-N), so holding position needs a nose-up... no: the quad must
    // lean INTO the wind (toward north) to cancel the push, i.e. pitch nose down (negative).
    const DroneDemo::Euler A = [&] { double Q[4] = {S.State.orientation[0], S.State.orientation[1], S.State.orientation[2], S.State.orientation[3]}; return DroneDemo::QuatToEuler(Q); }();
    Check(A.PitchRad < 0.0, "wind from the north: the quad leans into it (nose down)");
}

// A mountain site (2500 m, 2 C: dT +3.25 K): thin air, so hover needs more throttle. The pawn scales the
// controller's hover throttle with DroneEnv::HoverThrottleScale; the vehicle must still hold 2 m and, at hover,
// the four thrusts must add up to the weight (mass * 9.80665) within 2 %, hand: the weight is the only force.
static void TestMountainSite(const VehicleConfig& C)
{
    Sim S(C);
    S.Env.site_altitude_m = 2500.0;
    S.Env.temp_offset_c = 3.25;
    double Rho = 0.0, Scale = 0.0;
    Check(DroneEnv::DensityKgM3(2500.0, 2.0, &Rho) && DroneEnv::HoverThrottleScale(Rho, &Scale) && Scale > 1.0, "mountain: density and hover scale");
    DroneDemo::Tuning T;
    T.HoverThrottle = C.hover_throttle * Scale;
    S.Ctl = DroneDemo::Controller(T);
    S.Ctl.RequestTakeoff(S.Altitude());
    S.Run(15.0, None());
    Check(S.BadSteps == 0, "mountain: no step error");
    Check(std::fabs(S.Altitude() - 2.0) < 0.3, "mountain: holds 2 m +-0.3");
    const double Weight = C.mass_kg * 9.80665;
    double Sum = 0.0;
    for (const float F : S.Tele.motor_thrusts) Sum += F;
    Check(std::fabs(Sum - Weight) < 0.02 * Weight, "mountain: thrust balances weight at hover (+-2 %)");
}

static void TestGusts(const VehicleConfig& C)
{
    Sim S(C);
    S.WindParams.SteadySpeedMps = 5.0;
    S.WindParams.SteadyDirectionDeg = 90.0;
    S.WindParams.bGustsEnabled = true;
    S.Ctl.RequestTakeoff(S.Altitude());
    S.Run(40.0, None());
    Check(S.BadSteps == 0, "gusts: no step error");
    Check(std::fabs(S.Altitude() - 2.0) < 0.8, "gusts: altitude within +-0.8");
    Check(S.MaxTilt < 0.4, "gusts: tilt within limit");
    Check(S.Latch.physics_fault == 0, "gusts: no physics fault");
}

static void TestLanding(const VehicleConfig& C)
{
    Sim S(C);
    S.Ctl.RequestTakeoff(S.Altitude());
    S.Run(8.0, None());
    Check(S.Ctl.RequestLand(), "land accepted");
    S.Run(10.0, None());
    Check(S.Ctl.GetMode() == DroneDemo::Mode::OnGround, "landing: ends on the ground");
    Check(S.BadSteps == 0, "landing: no step error (altitude never below 0 in a step)");
    Check(S.State.position[2] == 0.0 && S.State.velocity[2] == 0.0f, "landing: pinned at rest on the ground");
    Check(!S.Ctl.RequestLand(), "land refused on the ground");
}

static void TestDeterminism(const VehicleConfig& C)
{
    DroneState A, B;
    for (int Run = 0; Run < 2; ++Run)
    {
        Sim S(C, 777);
        S.WindParams.SteadySpeedMps = 4.0;
        S.WindParams.bGustsEnabled = true;
        S.Ctl.RequestTakeoff(S.Altitude());
        DroneDemo::Command Cmd;
        Cmd.ForwardMps = 1.0;
        S.Run(10.0, Cmd);
        (Run == 0 ? A : B) = S.State;
    }
    Check(std::memcmp(&A, &B, sizeof(DroneState)) == 0, "same seed and inputs: bit-identical state");
}

int main()
{
    for (const char* Path : kConfigPaths)
    {
        VehicleConfig C;
        if (!LoadConfig(Path, &C))
        {
            std::printf("FAIL: could not load %s (run from the repo root)\n", Path);
            return 1;
        }
        std::printf("demo_flight_sim: %s\n", Path);
        TestTakeoffAndHover(C);
        TestForwardClimbYaw(C);
        TestSidewaysAndHeadingFrame(C);
        TestSteadyWind(C);
        TestMountainSite(C);
        TestGusts(C);
        TestLanding(C);
        TestDeterminism(C);
    }
    if (g_failures == 0)
    {
        std::printf("demo_flight_sim: all passed\n");
        return 0;
    }
    std::printf("demo_flight_sim: %d failure(s)\n", g_failures);
    return 1;
}
