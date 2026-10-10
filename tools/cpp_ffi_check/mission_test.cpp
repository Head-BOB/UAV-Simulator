// Tier: V
// Tests for DroneMission.h with a kinematic stand-in plant (no physics): the script's phases, corners,
// timing, cross-track distance, hover time, landing, timeout and refusal handling.
//   g++ -std=c++17 -Wall -Wextra -Werror -I UE5Project/Plugins/DronePhysicsBridge/Source/DronePhysicsBridge/Public tools/cpp_ffi_check/mission_test.cpp -o mission_test
// Expected values are hand calculations (see the comments).

#include "DroneMission.h"

#include <cmath>
#include <cstdio>
#include <string>

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

using namespace DroneMission;

// A point that moves exactly as commanded (yaw 0: forward = north, right = east). Altitude: +1 m/s in takeoff,
// follows the controller's altitude target at 1/s (max 2 m/s) while flying, -1 m/s landing, pinned at touchdown.
struct FakePlant
{
    DroneDemo::Controller Ctl;
    Script Scr;
    double T = 0.0, N = 0.0, E = 0.0, Alt = 0.0, Yaw = 0.0;
    double MinDistToCorner[4] = {1e9, 1e9, 1e9, 1e9};
    double MaxCrossTrack = 0.0;
    double PhaseStart[10];
    Phase Last = Phase::Takeoff;
    explicit FakePlant(const Params& P) : Scr(P)
    {
        for (double& S : PhaseStart) S = -1.0;
    }

    void Step(double Dt)
    {
        Pose S{T, N, E, Alt, Yaw};
        const DroneDemo::Command C = Scr.Next(S, Ctl);
        DroneDemo::Estimate Est = {Alt, {0, 0, 0}, {0, 0, 0, 1}, {0, 0, 0}};
        DroneDemo::Output Out;
        Ctl.Update(Est, C, Dt, &Out);
        N += C.ForwardMps * Dt;
        E += C.RightMps * Dt;
        switch (Ctl.GetMode())
        {
            case DroneDemo::Mode::Takeoff: Alt += 1.0 * Dt; break;
            case DroneDemo::Mode::Flying:
            {
                double V = 1.0 * (Ctl.GetAltitudeTargetM() - Alt);
                V = V > 2.0 ? 2.0 : (V < -2.0 ? -2.0 : V);
                Alt += V * Dt;
                break;
            }
            case DroneDemo::Mode::Landing: Alt -= 1.0 * Dt; break;
            default: break;
        }
        if (DroneDemo::ShouldTouchDown(Ctl.GetMode(), Alt))
        {
            Alt = 0.0;
            Ctl.NotifyOnGround();
        }
        for (int K = 0; K < 4; ++K)
        {
            double CN = 0.0, CE = 0.0;
            Scr.Corner(K, &CN, &CE);
            const double D = std::hypot(CN - N, CE - E);
            if (D < MinDistToCorner[K]) MinDistToCorner[K] = D;
        }
        const double X = Scr.CrossTrackM(Pose{T, N, E, Alt, Yaw});
        if (Scr.GetPhase() >= Phase::Leg1 && Scr.GetPhase() <= Phase::Leg4 && X > MaxCrossTrack) MaxCrossTrack = X;
        if (Scr.GetPhase() != Last)
        {
            Last = Scr.GetPhase();
            PhaseStart[static_cast<int>(Last)] = T;
        }
        T += Dt;
    }

    void RunUntilFinished(double Dt, double MaxS)
    {
        while (!Scr.Finished() && T < MaxS) Step(Dt);
    }
};

static void TestParams()
{
    Params P;
    Check(ParamsValid(P), "defaults valid");
    Params Bad = P;
    Bad.SideM = 0.0;
    Check(!ParamsValid(Bad), "zero side invalid");
    Bad = P;
    Bad.ArrivalRadiusM = 100.0;
    Check(!ParamsValid(Bad), "arrival radius >= side invalid");
    Bad = P;
    Bad.CruiseMps = std::nan("");
    Check(!ParamsValid(Bad), "NaN cruise invalid");
    Bad = P;
    Bad.HoverS = -1.0;
    Check(!ParamsValid(Bad), "negative hover invalid");
}

static void TestCorners()
{
    // Start at (10, 20), side 100: corners (110, 20), (110, 120), (10, 120), (10, 20).
    Params P;
    DroneDemo::Controller Ctl;
    Script Scr(P);
    Scr.Next(Pose{0.0, 10.0, 20.0, 0.0, 0.0}, Ctl);  // first call records the start point
    const double Want[4][2] = {{110.0, 20.0}, {110.0, 120.0}, {10.0, 120.0}, {10.0, 20.0}};
    for (int K = 0; K < 4; ++K)
    {
        double N = 0.0, E = 0.0;
        Scr.Corner(K, &N, &E);
        Check(N == Want[K][0] && E == Want[K][1], "corner coordinates");
    }
    // Out-of-range indices clamp to the first / last corner instead of reading outside the table.
    double N = 0.0, E = 0.0;
    Scr.Corner(-1, &N, &E);
    Check(N == 110.0 && E == 20.0, "corner index below 0 clamps to 0");
    Scr.Corner(9, &N, &E);
    Check(N == 10.0 && E == 20.0, "corner index above 3 clamps to 3");
}

static void TestFullMission()
{
    Params P;
    FakePlant F(P);
    F.RunUntilFinished(0.01, 400.0);
    Check(F.Scr.GetPhase() == Phase::Done, "mission reaches Done");
    Check(F.Alt == 0.0, "ends on the ground");
    // Every corner was reached to within the arrival radius (1 m): the distance test is at the step, so allow one step of motion.
    for (int K = 0; K < 4; ++K) Check(F.MinDistToCorner[K] <= P.ArrivalRadiusM + 0.06, "corner reached");
    // Leg 1 duration from rest: (100 - 5/0.5) / 5 = 18 s of cruise, then ln(5 / (0.5 * 1)) / 0.5 = ln(10)/0.5 = 4.60517 s approach = 22.605 s.
    const double Leg1 = F.PhaseStart[static_cast<int>(Phase::Leg2)] - F.PhaseStart[static_cast<int>(Phase::Leg1)];
    Check(Near(Leg1, 22.605, 0.1), "leg 1 takes 22.6 s (hand calc)");
    // The path is a straight leg along north starting at the origin: no cross-track error (kinematic plant, exact tracking).
    // Corners cut by the arrival radius: leg 2 starts up to 1 m short of corner 1, so the error stays <= 1 m + one step.
    Check(F.MaxCrossTrack <= P.ArrivalRadiusM + 0.06, "cross-track bounded by the arrival radius");
    // Hover lasts HoverS (10 s) plus at most one step.
    const double Hover = F.PhaseStart[static_cast<int>(Phase::Land)] - F.PhaseStart[static_cast<int>(Phase::Hover)];
    Check(Hover >= 10.0 && Hover <= 10.02, "hover lasts 10 s");
    // Phases occurred in order: all start times are set and non-decreasing.
    double Prev = -1.0;
    bool Ordered = true;
    for (Phase Ph : {Phase::Climb, Phase::Leg1, Phase::Leg2, Phase::Leg3, Phase::Leg4, Phase::Hover, Phase::Land, Phase::Done})
    {
        const double S = F.PhaseStart[static_cast<int>(Ph)];
        Ordered = Ordered && S >= 0.0 && S >= Prev;
        Prev = S;
    }
    Check(Ordered, "phases in order");
    // After Done the command is zero.
    const DroneDemo::Command After = F.Scr.Next(Pose{F.T, F.N, F.E, F.Alt, 0.0}, F.Ctl);
    Check(After.ForwardMps == 0.0 && After.RightMps == 0.0 && After.ClimbMps == 0.0, "zero command after Done");
}

static void TestClimb()
{
    Params P;
    FakePlant F(P);
    while (F.Scr.GetPhase() != Phase::Leg1 && F.T < 60.0) F.Step(0.01);
    Check(F.Scr.GetPhase() == Phase::Leg1, "climb ends in leg 1");
    // The altitude target stops at 20 m (the climb command stops within one step of it).
    Check(Near(F.Ctl.GetAltitudeTargetM(), 20.0, 0.06), "altitude target is 20 m");
}

static void TestCrossTrack()
{
    Params P;
    FakePlant F(P);
    while (F.Scr.GetPhase() != Phase::Leg1 && F.T < 60.0) F.Step(0.01);
    // Leg 1 runs from (0,0) to (100,0). At (50, 3) the distance to the line is 3.
    Check(Near(F.Scr.CrossTrackM(Pose{0, 50.0, 3.0, 20.0, 0.0}), 3.0, 1e-12), "cross-track 3 m beside the leg");
    // Behind the start the nearest point is the start: (-4, 3) -> 3-4-5 triangle, 5 m.
    Check(Near(F.Scr.CrossTrackM(Pose{0, -4.0, 3.0, 20.0, 0.0}), 5.0, 1e-12), "cross-track before the start is the distance to it");
    // Beyond the end: (103, 4) -> 3-4-5 from (100, 0).
    Check(Near(F.Scr.CrossTrackM(Pose{0, 103.0, 4.0, 20.0, 0.0}), 5.0, 1e-12), "cross-track past the end");
    // Drive to leg 2: segment (100,0)-(100,100); at (98, 40) the distance is 2.
    while (F.Scr.GetPhase() != Phase::Leg2 && F.T < 200.0) F.Step(0.01);
    Check(Near(F.Scr.CrossTrackM(Pose{0, 98.0, 40.0, 20.0, 0.0}), 2.0, 1e-12), "cross-track on leg 2");
}

static void TestHeadingFrame()
{
    Params P;
    FakePlant F(P);
    while (F.Scr.GetPhase() != Phase::Leg1 && F.T < 60.0) F.Step(0.01);
    // Far from the corner (100, 0): speed = min(5, 0.5 * ~100) = 5 toward north. Yaw 0: forward 5, right 0.
    DroneDemo::Command C0 = F.Scr.Next(Pose{F.T, 0.0, 0.0, 20.0, 0.0}, F.Ctl);
    Check(Near(C0.ForwardMps, 5.0, 1e-9) && Near(C0.RightMps, 0.0, 1e-9), "north at yaw 0 is forward");
    // Yaw +90 deg (nose east): north is to the LEFT: forward 0, right -5.
    DroneDemo::Command C1 = F.Scr.Next(Pose{F.T, 0.0, 0.0, 20.0, 1.5707963267948966}, F.Ctl);
    Check(Near(C1.ForwardMps, 0.0, 1e-9) && Near(C1.RightMps, -5.0, 1e-9), "north at yaw 90 deg is left");
    // 6 m from the corner: speed = 0.5 * 6 = 3 m/s.
    DroneDemo::Command C2 = F.Scr.Next(Pose{F.T, 94.0, 0.0, 20.0, 0.0}, F.Ctl);
    Check(Near(C2.ForwardMps, 3.0, 1e-9), "approach speed is gain * distance");
}

static void TestFailures()
{
    {
        Params P;
        P.MaxDurationS = 5.0;
        FakePlant F(P);
        F.RunUntilFinished(0.01, 30.0);
        Check(F.Scr.GetPhase() == Phase::Failed, "timeout fails the mission");
        Check(std::string(F.Scr.FailureReason()).find("timed out") != std::string::npos, "timeout reason");
    }
    {
        Params P;
        FakePlant F(P);
        F.Ctl.RequestTakeoff(0.0);  // already taking off: the script's request is refused
        F.Step(0.01);
        Check(F.Scr.GetPhase() == Phase::Failed && std::string(F.Scr.FailureReason()).find("refused") != std::string::npos, "refused takeoff fails");
    }
    {
        Params P;
        FakePlant F(P);
        F.Step(0.01);
        F.Scr.Abort("test abort");
        Check(F.Scr.Finished() && std::string(F.Scr.FailureReason()) == "test abort", "abort");
        const DroneDemo::Command C = F.Scr.Next(Pose{1.0, 0, 0, 0, 0}, F.Ctl);
        Check(C.ForwardMps == 0.0 && C.ClimbMps == 0.0, "no command after abort");
    }
}

static void TestLandingRefused()
{
    Params P;
    FakePlant F(P);
    while (F.Scr.GetPhase() != Phase::Hover && F.T < 200.0) F.Step(0.01);
    Check(F.Scr.GetPhase() == Phase::Hover, "reached hover");
    F.Ctl.NotifyOnGround();  // the controller is no longer flying: RequestLand will be refused
    F.RunUntilFinished(0.01, 400.0);
    Check(F.Scr.GetPhase() == Phase::Failed && std::string(F.Scr.FailureReason()).find("landing was refused") != std::string::npos, "refused landing fails");
}

static void TestSlowerCruise()
{
    Params P;
    P.CruiseMps = 3.0;
    FakePlant F(P);
    F.RunUntilFinished(0.01, 600.0);
    Check(F.Scr.GetPhase() == Phase::Done, "3 m/s mission completes");
    // Leg 1: (100 - 3/0.5) / 3 = 31.33 s cruise + ln(3 / 0.5) / 0.5 = 3.5835 s approach = 34.92 s.
    const double Leg1 = F.PhaseStart[static_cast<int>(Phase::Leg2)] - F.PhaseStart[static_cast<int>(Phase::Leg1)];
    Check(Near(Leg1, 34.92, 0.1), "leg 1 at 3 m/s takes 34.9 s (hand calc)");
}

int main()
{
    TestParams();
    TestCorners();
    TestFullMission();
    TestClimb();
    TestCrossTrack();
    TestHeadingFrame();
    TestFailures();
    TestLandingRefused();
    TestSlowerCruise();
    if (g_failures == 0)
    {
        std::printf("mission_test: all passed\n");
        return 0;
    }
    std::printf("mission_test: %d failure(s)\n", g_failures);
    return 1;
}
