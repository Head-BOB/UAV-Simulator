// Tier: V
// Unit tests for DroneDemoControl.h (DEMO controller pieces). Plain C++17, no Unreal.
// From the repo root:
//   g++ -std=c++17 -Wall -Wextra -Werror -I UE5Project/Plugins/DronePhysicsBridge/Source/DronePhysicsBridge/Public tools/cpp_ffi_check/demo_control_test.cpp -o demo_control_test
//   ./demo_control_test
// The closed-loop behaviour is checked separately in demo_flight_sim.cpp (needs the Rust library).

#include "DroneDemoControl.h"

#include <cmath>
#include <cstdio>

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

using namespace DroneDemo;

static void TestPid()
{
    // P only: 3 * 2 = 6; with an output limit of 5 it is 5; negative error mirrors.
    Pid P{3.0, 0.0, 0.0, 1.0, 10.0};
    Check(Near(P.Step(2.0, 0.1), 6.0, 1e-12), "P term");
    Pid Lim{3.0, 0.0, 0.0, 1.0, 5.0};
    Check(Near(Lim.Step(2.0, 0.1), 5.0, 1e-12), "output clamp +");
    Pid Neg{3.0, 0.0, 0.0, 1.0, 5.0};
    Check(Near(Neg.Step(-2.0, 0.1), -5.0, 1e-12), "output clamp -");

    // I only: Ki 2, error 1, dt 0.5: integral 0.5 -> out 1.0, then 1.0 -> out 2.0.
    Pid I{0.0, 2.0, 0.0, 10.0, 10.0};
    Check(Near(I.Step(1.0, 0.5), 1.0, 1e-12), "I first step");
    Check(Near(I.Step(1.0, 0.5), 2.0, 1e-12), "I second step");

    // Integral clamp: Ki 2, IntegralLimit 1 caps the integral term at 1 (integral 0.5).
    Pid IC{0.0, 2.0, 0.0, 1.0, 10.0};
    double Out = 0.0;
    for (int K = 0; K < 100; ++K) Out = IC.Step(1.0, 0.5);
    Check(Near(Out, 1.0, 1e-12), "integral clamp");

    // D only: errors 1 then 3 with dt 0.1: first step has no history (0); second 0.5 * 2 / 0.1 = 10.
    Pid D{0.0, 0.0, 0.5, 1.0, 100.0};
    Check(D.Step(1.0, 0.1) == 0.0, "D first step is zero");
    Check(Near(D.Step(3.0, 0.1), 10.0, 1e-12), "D second step");

    // Anti-windup: Kp 10 saturates immediately, so the integral must not grow.
    Pid AW{10.0, 1.0, 0.0, 5.0, 1.0};
    for (int K = 0; K < 100; ++K) AW.Step(1.0, 0.1);
    Check(AW.Integral == 0.0, "no windup while saturated");

    // Invalid inputs: 0 out, state untouched.
    Pid B{1.0, 1.0, 0.0, 5.0, 5.0};
    Check(B.Step(1.0, 0.0) == 0.0 && B.Step(std::nan(""), 0.1) == 0.0 && B.Step(1.0, -1.0) == 0.0 && B.Integral == 0.0,
          "invalid dt or error ignored");
    B.Step(1.0, 0.1);
    B.Reset();
    Check(B.Integral == 0.0 && !B.bHavePrev, "reset clears state");
}

static void TestEulerAndFrames()
{
    const double S15 = std::sin(15.0 * kPi / 180.0), C15 = std::cos(15.0 * kPi / 180.0);
    const double S30 = std::sin(30.0 * kPi / 180.0), C30 = std::cos(30.0 * kPi / 180.0);
    const double S45 = std::sqrt(0.5);

    const double Id[4] = {0, 0, 0, 1};
    Euler E = QuatToEuler(Id);
    Check(E.RollRad == 0.0 && E.PitchRad == 0.0 && E.YawRad == 0.0, "identity");

    const double Yaw90[4] = {0, 0, S45, S45};  // 90 deg about +Z (down) = nose to the east
    E = QuatToEuler(Yaw90);
    Check(Near(E.YawRad, kPi / 2, 1e-12) && Near(E.RollRad, 0.0, 1e-12) && Near(E.PitchRad, 0.0, 1e-12), "yaw 90");

    const double Roll60[4] = {S30, 0, 0, C30};  // 60 deg about +X = right wing down
    E = QuatToEuler(Roll60);
    Check(Near(E.RollRad, kPi / 3, 1e-12) && Near(E.PitchRad, 0.0, 1e-12), "roll 60");

    const double Pitch30[4] = {0, S15, 0, C15};  // 30 deg about +Y = nose up; asin(2 sin15 cos15) = asin(sin30)
    E = QuatToEuler(Pitch30);
    Check(Near(E.PitchRad, kPi / 6, 1e-12) && Near(E.RollRad, 0.0, 1e-12), "pitch 30 nose up");

    // Heading frame. Yaw 0 (north): north velocity is forward, east is right.
    double F = 0.0, R = 0.0;
    NedToHeadingFrame(1.0, 0.0, 0.0, &F, &R);
    Check(Near(F, 1.0, 1e-12) && Near(R, 0.0, 1e-12), "heading north: north is forward");
    NedToHeadingFrame(0.0, 1.0, 0.0, &F, &R);
    Check(Near(F, 0.0, 1e-12) && Near(R, 1.0, 1e-12), "heading north: east is right");
    // Yaw 90 (east): east is forward; north is to the LEFT, so right = -1.
    NedToHeadingFrame(0.0, 1.0, kPi / 2, &F, &R);
    Check(Near(F, 1.0, 1e-12) && Near(R, 0.0, 1e-12), "heading east: east is forward");
    NedToHeadingFrame(1.0, 0.0, kPi / 2, &F, &R);
    Check(Near(F, 0.0, 1e-12) && Near(R, -1.0, 1e-12), "heading east: north is left");

    Check(Near(WrapPi(2 * kPi + 0.1), 0.1, 1e-12), "wrap +2pi");
    Check(Near(WrapPi(1.5 * kPi), -0.5 * kPi, 1e-12), "wrap 270 deg is -90");
    Check(Near(WrapPi(-1.5 * kPi), 0.5 * kPi, 1e-12), "wrap -270 deg is +90");
}

static Estimate Hover(double AltM)
{
    Estimate E{};
    E.AltitudeM = AltM;
    E.QuatXyzw[3] = 1.0;
    return E;
}

static void TestModesAndThrottle()
{
    Tuning T;
    T.HoverThrottle = 0.5;
    Controller C(T);
    Output O{9, 9, 9, 9};

    // On the ground: all zeros, takeoff allowed, landing refused.
    Check(C.GetMode() == Mode::OnGround, "starts on the ground");
    Check(C.Update(Hover(0.0), Command(), 0.002, &O) && O.Throttle == 0.0 && O.Roll == 0.0 && O.Pitch == 0.0 && O.Yaw == 0.0,
          "on ground outputs zero");
    Check(!C.RequestLand(), "land refused on ground");
    Check(C.RequestTakeoff(0.0) && C.GetMode() == Mode::Takeoff, "takeoff accepted");
    Check(!C.RequestTakeoff(0.0), "second takeoff refused");

    // At the takeoff altitude, level and at rest: no vertical error, so the throttle is exactly
    // the hover throttle: 0.5 * sqrt((g + 0) / (g * 1)) = 0.5.
    Check(C.Update(Hover(2.0), Command(), 0.002, &O), "update ok");
    Check(C.GetMode() == Mode::Flying, "takeoff ends in Flying near 2 m");
    Check(Near(O.Throttle, 0.5, 1e-12), "hover throttle at zero error");
    Check(Near(O.Roll, 0.0, 1e-12) && Near(O.Pitch, 0.0, 1e-12) && Near(O.Yaw, 0.0, 1e-12), "level hover commands zero");

    // Tilt compensation: rolled 60 deg (cos = 0.5), hover target: throttle = 0.5 * sqrt(1 / 0.5) = 0.70710678.
    Estimate Tilted = Hover(2.0);
    Tilted.QuatXyzw[0] = std::sin(30.0 * kPi / 180.0);
    Tilted.QuatXyzw[3] = std::cos(30.0 * kPi / 180.0);
    Controller C2(T);
    C2.RequestTakeoff(0.0);
    C2.Update(Hover(2.0), Command(), 0.002, &O);
    C2.Update(Tilted, Command(), 0.002, &O);
    Check(Near(O.Throttle, 0.5 * std::sqrt(2.0), 1e-9), "tilt compensation 1/cos");
    // Rolled right wing down with zero commands: the controller must roll back (negative roll command).
    Check(O.Roll < 0.0, "rolled right: command rolls left");

    // Landing and touch-down rules.
    Check(C.RequestLand() && C.GetMode() == Mode::Landing, "land accepted");
    Check(ShouldTouchDown(Mode::Landing, 0.04), "touch down below 0.05 while landing");
    Check(!ShouldTouchDown(Mode::Landing, 0.06), "not yet at 0.06 m");
    Check(!ShouldTouchDown(Mode::Flying, 0.04), "flying near the ground is not a touch-down");
    Check(ShouldTouchDown(Mode::Flying, 0.0), "flying into the ground (alt 0) is pinned");
    Check(ShouldTouchDown(Mode::Takeoff, -0.1), "below ground is pinned");
    Check(!ShouldTouchDown(Mode::OnGround, -1.0), "already on ground");
    C.NotifyOnGround();
    Check(C.GetMode() == Mode::OnGround, "notify on ground");

    // Bad input: refused, output untouched.
    Output Keep{1, 2, 3, 4};
    Estimate Bad = Hover(2.0);
    Bad.VelNedMps[0] = std::nan("");
    Controller C3(T);
    C3.RequestTakeoff(0.0);
    Check(!C3.Update(Bad, Command(), 0.002, &Keep) && Keep.Throttle == 1.0, "NaN estimate rejected");
    Check(!C3.Update(Hover(2.0), Command(), 0.0, &Keep), "dt 0 rejected");
    Check(!C3.Update(Hover(2.0), Command(), 0.002, nullptr), "null output rejected");
}

int main()
{
    TestPid();
    TestEulerAndFrames();
    TestModesAndThrottle();
    if (g_failures == 0)
    {
        std::printf("demo_control_test: all passed\n");
        return 0;
    }
    std::printf("demo_control_test: %d failure(s)\n", g_failures);
    return 1;
}
