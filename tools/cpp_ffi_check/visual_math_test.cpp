// Tier: V
// Tests for DroneVisualMath.h. Plain C++17, no Unreal, no framework.
// From the repo root:
//   g++ -std=c++17 -Wall -Wextra -Werror -I UE5Project/Plugins/DronePhysicsBridge/Source/DronePhysicsBridge/Public tools/cpp_ffi_check/visual_math_test.cpp -o visual_math_test
//   ./visual_math_test
// Exit code 0 = all pass. Lives outside the plugin Source/ tree so UnrealBuildTool never compiles it.

#include "DroneVisualMath.h"

#include <cmath>
#include <cstdio>

static int g_failures = 0;
static const double kPi = 3.14159265358979323846;

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

using namespace DroneVisual;

static void TestSpin()
{
    // 6000 RPM = 100 rev/s. In 0.01 s that is exactly 1 revolution = 360 deg, which wraps to 0.
    Check(Near(AdvanceSpinDeg(0.0, 6000.0, 1.0, 0.01), 0.0, 1e-9), "6000 rpm 0.01 s is one revolution");
    // 3000 RPM = 50 rev/s; 0.01 s is half a revolution = 180 deg.
    Check(Near(AdvanceSpinDeg(0.0, 3000.0, 1.0, 0.01), 180.0, 1e-9), "3000 rpm half revolution");
    // Direction comes only from the sign: -1 gives -180, wrapped to +180.
    Check(Near(AdvanceSpinDeg(0.0, 3000.0, -1.0, 0.01), 180.0, 1e-9), "reverse half revolution wraps to 180");
    // 1000 RPM = 6000 deg/s; 1/60 s = 100 deg forward, and 260 deg when reversed (360 - 100).
    Check(Near(AdvanceSpinDeg(0.0, 1000.0, 1.0, 1.0 / 60.0), 100.0, 1e-9), "1000 rpm 1/60 s is 100 deg");
    Check(Near(AdvanceSpinDeg(0.0, 1000.0, -1.0, 1.0 / 60.0), 260.0, 1e-9), "reverse 100 deg is 260");
    // Starting angle is kept: 350 + 100 = 450 -> 90.
    Check(Near(AdvanceSpinDeg(350.0, 1000.0, 1.0, 1.0 / 60.0), 90.0, 1e-9), "wrap past 360");
    // Zero RPM: no motion (the motor is off).
    Check(AdvanceSpinDeg(123.0, 0.0, 1.0, 0.01) == 123.0, "zero rpm no motion");
    // Non-finite input leaves the angle unchanged.
    Check(AdvanceSpinDeg(10.0, std::nan(""), 1.0, 0.01) == 10.0, "NaN rpm ignored");
    Check(AdvanceSpinDeg(10.0, 1000.0, 1.0, HUGE_VAL) == 10.0, "Inf dt ignored");
    Check(std::isnan(AdvanceSpinDeg(std::nan(""), 1000.0, 1.0, 0.01)), "NaN prev is returned unchanged");
    // Result is always in [0, 360).
    const double A = AdvanceSpinDeg(359.999, 10000.0, 1.0, 0.016);
    Check(A >= 0.0 && A < 360.0, "result in range");
    // The axis constant is a unit vector (a typo here would silently shear the spin).
    const double N = std::sqrt(kSpinAxisBoneLocal[0] * kSpinAxisBoneLocal[0] + kSpinAxisBoneLocal[1] * kSpinAxisBoneLocal[1] +
                               kSpinAxisBoneLocal[2] * kSpinAxisBoneLocal[2]);
    Check(Near(N, 1.0, 1e-12), "spin axis is a unit vector");
    Check(kSpinDirectionScale == 1.0 || kSpinDirectionScale == -1.0, "direction scale is +-1");
}

static bool QuatNear(const Quat& A, const Quat& B, double tol)
{
    return Near(A.x, B.x, tol) && Near(A.y, B.y, tol) && Near(A.z, B.z, tol) && Near(A.w, B.w, tol);
}

// Rotates vector V by unit quaternion Q: V' = V + 2w(u x V) + 2 u x (u x V), u = (x, y, z).
static void Rotate(const Quat& Q, const double V[3], double Out[3])
{
    const double Ux = Q.y * V[2] - Q.z * V[1], Uy = Q.z * V[0] - Q.x * V[2], Uz = Q.x * V[1] - Q.y * V[0];
    Out[0] = V[0] + 2.0 * (Q.w * Ux + Q.y * Uz - Q.z * Uy);
    Out[1] = V[1] + 2.0 * (Q.w * Uy + Q.z * Ux - Q.x * Uz);
    Out[2] = V[2] + 2.0 * (Q.w * Uz + Q.x * Uy - Q.y * Ux);
}

static void TestSpinQuat()
{
    const double S = std::sqrt(0.5);
    const double AxisX[3] = {1.0, 0.0, 0.0}, AxisZ[3] = {0.0, 0.0, 1.0};

    // Hand value: 90 deg about +Z = (0, 0, sin45, cos45) = (0, 0, S, S).
    Check(QuatNear(QuatFromAxisAngle(AxisZ, 0.5 * kPi), Quat{0.0, 0.0, S, S}, 1e-12), "90 deg about Z");
    Check(QuatNear(QuatFromAxisAngle(AxisZ, 0.0), Quat(), 1e-12), "zero angle is identity");
    const double Zero[3] = {0.0, 0.0, 0.0};
    Check(QuatNear(QuatFromAxisAngle(Zero, 1.0), Quat(), 0.0), "zero axis is identity");
    Check(QuatNear(QuatFromAxisAngle(AxisZ, std::nan("")), Quat(), 0.0), "NaN angle is identity");
    const double Long[3] = {0.0, 0.0, 5.0};
    Check(QuatNear(QuatFromAxisAngle(Long, 0.5 * kPi), Quat{0.0, 0.0, S, S}, 1e-12), "non-unit axis is normalised");

    // Known ref rotation: 90 deg about +X, Ref = (S, 0, 0, S). Spin 90 deg about bone-local Z.
    // Hand Hamilton product Ref*Spin with Spin = (0, 0, S, S):
    //   x = w1 x2 + x1 w2 + y1 z2 - z1 y2 = 0 + S*S + 0 - 0 = 0.5
    //   y = w1 y2 - x1 z2 + y1 w2 + z1 x2 = 0 - S*S + 0 + 0 = -0.5
    //   z = w1 z2 + x1 y2 - y1 x2 + z1 w2 = S*S + 0 - 0 + 0 = 0.5
    //   w = w1 w2 - x1 x2 - y1 y2 - z1 z2 = S*S - 0 - 0 - 0 = 0.5
    const Quat Ref{S, 0.0, 0.0, S};
    const Quat Got = SpinOnRefRotation(Ref, AxisZ, 0.5 * kPi);
    Check(QuatNear(Got, Quat{0.5, -0.5, 0.5, 0.5}, 1e-12), "90 deg spin on ref rotation, hand quaternion");
    // Mutation guard: the swapped order Spin*Ref is a different rotation (it spins about the
    // PARENT's Z). Hand value: x = 0.5, y = +0.5, z = 0.5, w = 0.5. Must differ from the above.
    const Quat Swapped = QuatMul(QuatFromAxisAngle(AxisZ, 0.5 * kPi), Ref);
    Check(QuatNear(Swapped, Quat{0.5, 0.5, 0.5, 0.5}, 1e-12), "swapped order hand value");
    Check(!QuatNear(Got, Swapped, 1e-6), "swapped order is detectably different");

    // Spin axis stays fixed in the bone-local frame: Ref^-1 * Final is a pure spin about Z, so the
    // bone-local Z axis, carried to component space by Final, equals carrying it by Ref alone,
    // for every angle. Ref maps Z to (0, -1, 0) here (90 deg about X: z -> -y).
    double RefZ[3];
    Rotate(Ref, AxisZ, RefZ);
    Check(Near(RefZ[0], 0.0, 1e-12) && Near(RefZ[1], -1.0, 1e-12) && Near(RefZ[2], 0.0, 1e-12), "ref carries Z to -Y");
    for (int Deg = 0; Deg < 360; Deg += 15)
    {
        const Quat F = SpinOnRefRotation(Ref, AxisZ, Deg * kPi / 180.0);
        double FZ[3];
        Rotate(F, AxisZ, FZ);
        Check(Near(FZ[0], RefZ[0], 1e-12) && Near(FZ[1], RefZ[1], 1e-12) && Near(FZ[2], RefZ[2], 1e-12), "spin axis fixed in bone-local frame");
        // The mutated order moves this axis: it must NOT stay fixed away from 0/180 deg.
    }
    // Same fixed-axis check for a non-trivial ref and a different axis (X).
    const Quat Ref2 = QuatMul(QuatFromAxisAngle(AxisX, 0.7), QuatFromAxisAngle(AxisZ, -1.1));
    double Ref2X[3];
    Rotate(Ref2, AxisX, Ref2X);
    double FX[3];
    Rotate(SpinOnRefRotation(Ref2, AxisX, 2.3), AxisX, FX);
    Check(Near(FX[0], Ref2X[0], 1e-12) && Near(FX[1], Ref2X[1], 1e-12) && Near(FX[2], Ref2X[2], 1e-12), "axis fixed, general ref, X axis");
    // And the swapped order does NOT keep it fixed (the bug this guards against).
    double SX[3];
    Rotate(QuatMul(QuatFromAxisAngle(AxisX, 2.3), Ref2), AxisX, SX);
    Check(!(Near(SX[0], Ref2X[0], 1e-6) && Near(SX[1], Ref2X[1], 1e-6) && Near(SX[2], Ref2X[2], 1e-6)), "swapped order moves the axis");

    // Axis index and sign helpers.
    double A[3] = {9.0, 9.0, 9.0};
    Check(SpinAxisFromIndex(0, A) && A[0] == 1.0 && A[1] == 0.0 && A[2] == 0.0, "axis index 0 is X");
    Check(SpinAxisFromIndex(1, A) && A[0] == 0.0 && A[1] == 1.0 && A[2] == 0.0, "axis index 1 is Y");
    Check(SpinAxisFromIndex(2, A) && A[0] == 0.0 && A[1] == 0.0 && A[2] == 1.0, "axis index 2 is Z");
    A[0] = 9.0;
    Check(!SpinAxisFromIndex(3, A) && !SpinAxisFromIndex(-1, A) && A[0] == 9.0, "bad axis index rejected, output untouched");
    double Def[3];
    Check(SpinAxisFromIndex(kSpinAxisIndexDefault, Def) && Def[0] == kSpinAxisBoneLocal[0] && Def[1] == kSpinAxisBoneLocal[1] && Def[2] == kSpinAxisBoneLocal[2],
          "default axis index matches kSpinAxisBoneLocal");
    Check(NormalizeSpinSign(1.0) == 1.0 && NormalizeSpinSign(-1.0) == -1.0 && NormalizeSpinSign(-0.2) == -1.0 && NormalizeSpinSign(3.0) == 1.0, "spin sign is +-1");
    Check(NormalizeSpinSign(std::nan("")) == kSpinDirectionScale, "NaN sign falls back to default");

    // Direction scale: -1 mirrors the angle. 1000 rpm, 1/60 s = +100 deg; with scale -1 -> 260.
    Check(Near(AdvanceSpinDeg(0.0, 1000.0, 1.0, 1.0 / 60.0, -1.0), 260.0, 1e-9), "direction scale -1 reverses");
    // Per-motor: motor_spin [1,-1,-1,1] with equal RPM gives opposite angles on adjacent props.
    const double Spin[4] = {1.0, -1.0, -1.0, 1.0};
    const double Ang[4] = {AdvanceSpinDeg(0.0, 1000.0, Spin[0], 1.0 / 60.0), AdvanceSpinDeg(0.0, 1000.0, Spin[1], 1.0 / 60.0),
                           AdvanceSpinDeg(0.0, 1000.0, Spin[2], 1.0 / 60.0), AdvanceSpinDeg(0.0, 1000.0, Spin[3], 1.0 / 60.0)};
    Check(Near(Ang[0], 100.0, 1e-9) && Near(Ang[1], 260.0, 1e-9) && Near(Ang[2], 260.0, 1e-9) && Near(Ang[3], 100.0, 1e-9), "FR FL RR RL angles");
}

static void TestBladeFade()
{
    // Defaults: threshold 1500, hysteresis 200 -> hide at >= 1700, show again at <= 1300.
    Check(!BladesHidden(false, 1699.0), "visible below 1700");
    Check(BladesHidden(false, 1700.0), "hidden at 1700");
    Check(BladesHidden(true, 1301.0), "stays hidden above 1300");
    Check(!BladesHidden(true, 1300.0), "visible again at 1300");
    Check(!BladesHidden(false, 1500.0) && BladesHidden(true, 1500.0), "inside the band the state is kept");
    Check(!BladesHidden(true, std::nan("")), "NaN rpm keeps blades visible");
    Check(!BladesHidden(false, 5000.0, 100.0, 100.0), "threshold <= hysteresis rejected");
    Check(!BladesHidden(false, 5000.0, 1500.0, -1.0), "negative hysteresis rejected");
}

static void TestScale()
{
    double S = -1.0;
    // 0.4 / 0.1708 = 2.34192037...
    Check(ComputeUniformScale(0.4, 0.1708, &S) && Near(S, 2.3419203747072599, 1e-12), "scale 0.4 / 0.1708");
    double Untouched = 5.0;
    Check(!ComputeUniformScale(0.4, 0.0, &Untouched) && Untouched == 5.0, "zero model span rejected");
    Check(!ComputeUniformScale(-0.4, 0.17, &Untouched), "negative config span rejected");
    Check(!ComputeUniformScale(std::nan(""), 0.17, &Untouched), "NaN rejected");
    Check(!ComputeUniformScale(0.4, 0.17, nullptr), "null out rejected");

    // Shipped config motor_pos_m: FR(+0.2,+0.2) FL(+0.2,-0.2) RR(-0.2,+0.2) RL(-0.2,-0.2).
    const double Pos[4][3] = {{0.2, 0.2, 0.0}, {0.2, -0.2, 0.0}, {-0.2, 0.2, 0.0}, {-0.2, -0.2, 0.0}};
    double LR = 0.0, FR = 0.0;
    ConfigSpansM(Pos, &LR, &FR);
    Check(Near(LR, 0.4, 1e-12) && Near(FR, 0.4, 1e-12), "config spans are 0.4 m");
    // Non-square layout: left-right 0.30, front-rear 0.50 (mean of the two pairs).
    const double Rect[4][3] = {{0.25, 0.15, 0.0}, {0.25, -0.15, 0.0}, {-0.25, 0.15, 0.0}, {-0.25, -0.15, 0.0}};
    ConfigSpansM(Rect, &LR, &FR);
    Check(Near(LR, 0.3, 1e-12) && Near(FR, 0.5, 1e-12), "rectangular config spans");

    // Leftover: model front-rear 0.1453 * 2.34192037 = 0.34028...; config 0.4 -> mismatch -0.05971...
    // 0.1453 * 2.3419203747072599 = 0.340281...: -0.059719 (hand multiplication).
    Check(Near(FrontRearMismatchM(2.3419203747072599, 0.1453, 0.4), -0.05971, 1e-4), "front-rear leftover");
    Check(Near(FrontRearMismatchM(2.0, 0.2, 0.4), 0.0, 1e-12), "no leftover when proportions match");
}

static void TestOpacity()
{
    Check(MotionDiscOpacity(0.0, 10000.0) == 0.0, "off is transparent");
    Check(Near(MotionDiscOpacity(5000.0, 10000.0), 0.225, 1e-12), "half rpm gives 0.45 * 0.5");
    Check(Near(MotionDiscOpacity(10000.0, 10000.0), 0.45, 1e-12), "full rpm gives the max");
    Check(Near(MotionDiscOpacity(20000.0, 10000.0), 0.45, 1e-12), "over max is clamped");
    Check(MotionDiscOpacity(-5.0, 10000.0) == 0.0, "negative rpm is 0");
    Check(MotionDiscOpacity(5000.0, 0.0) == 0.0, "zero max rpm is 0");
    Check(MotionDiscOpacity(std::nan(""), 10000.0) == 0.0, "NaN is 0");
    Check(MotionDiscOpacity(2500.0, 10000.0) < MotionDiscOpacity(7500.0, 10000.0), "rises with rpm");
}

int main()
{
    TestSpin();
    TestSpinQuat();
    TestBladeFade();
    TestScale();
    TestOpacity();
    if (g_failures == 0)
    {
        std::printf("visual_math_test: all passed\n");
        return 0;
    }
    std::printf("visual_math_test: %d failure(s)\n", g_failures);
    return 1;
}
