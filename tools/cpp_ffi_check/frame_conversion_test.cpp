// Tier: F
// Tests for DroneNedToUnreal.h (WP-5.2, ADR-001 D5, HZ-04). Plain C++17, no Unreal, no framework.
// From the repo root:
//   g++ -std=c++17 -Wall -Wextra -Werror -I UE5Project/Plugins/DronePhysicsBridge/Source/DronePhysicsBridge/Public tools/cpp_ffi_check/frame_conversion_test.cpp -o frame_conversion_test
//   ./frame_conversion_test
// Exit code 0 = all pass. This proves the algebra only; the engine photos (B-6) are still needed.
// Lives outside the plugin Source/ tree so UnrealBuildTool never compiles it.

#include "DroneNedToUnreal.h"

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

// Rotates v by the unit quaternion (x, y, z, w) using the Hamilton matrix (right-handed rotation).
static void Rotate(const double q[4], const double v[3], double out[3])
{
    const double x = q[0], y = q[1], z = q[2], w = q[3];
    out[0] = (1 - 2 * (y * y + z * z)) * v[0] + 2 * (x * y - z * w) * v[1] + 2 * (x * z + y * w) * v[2];
    out[1] = 2 * (x * y + z * w) * v[0] + (1 - 2 * (x * x + z * z)) * v[1] + 2 * (y * z - x * w) * v[2];
    out[2] = 2 * (x * z - y * w) * v[0] + 2 * (y * z + x * w) * v[1] + (1 - 2 * (x * x + y * y)) * v[2];
}

static void ToArray(const DroneUeQuat& q, double out[4])
{
    out[0] = q.X;
    out[1] = q.Y;
    out[2] = q.Z;
    out[3] = q.W;
}

static void TestPosition()
{
    // 1.5 m * 100 = 150, -2.25 * 100 = -225, D = 10 m is down so Unreal Z = -10 * 100 = -1000. All exact in binary.
    const double ned[3] = {1.5, -2.25, 10.0};
    const DroneUeVec3 r = DroneNedToUePosition(ned);
    Check(r.X == 150.0 && r.Y == -225.0 && r.Z == -1000.0, "position (1.5,-2.25,10) m -> (150,-225,-1000) cm");
}

static void TestVector()
{
    // Axis map only: (3, -4, 14.75) -> (3, -4, -14.75). Every value is exact in binary.
    const float ned[3] = {3.0f, -4.0f, 14.75f};
    const DroneUeVec3 r = DroneNedToUeVector(ned);
    Check(r.X == 3.0 && r.Y == -4.0 && r.Z == -14.75, "vector (3,-4,14.75) -> (3,-4,-14.75)");
}

static void TestQuatComponents()
{
    // WP-5.2 formula (x,y,z,w) -> (-x,-y,z,w) with all components 0.5 (exact).
    const float ned[4] = {0.5f, 0.5f, 0.5f, 0.5f};
    const DroneUeQuat r = DroneNedToUeQuat(ned);
    Check(r.X == -0.5 && r.Y == -0.5 && r.Z == 0.5 && r.W == 0.5, "quat components (0.5 x4) -> (-0.5,-0.5,0.5,0.5)");
}

static void TestAxisCases()
{
    const double s = std::sqrt(0.5);
    const float fs = static_cast<float>(s);
    double q[4];
    double out[3];

    // Yaw +90 deg (nose turns right): NED North -> East. Unreal: S*East = +Y (right). Expect X axis -> (0,1,0).
    const float yaw[4] = {0.0f, 0.0f, fs, fs};
    ToArray(DroneNedToUeQuat(yaw), q);
    const double ex[3] = {1.0, 0.0, 0.0};
    Rotate(q, ex, out);
    Check(Near(out[0], 0.0, 1e-6) && Near(out[1], 1.0, 1e-6) && Near(out[2], 0.0, 1e-6), "yaw +90 maps UE X to UE +Y");

    // Roll +90 deg (right wing down): NED East -> Down. Unreal: S*Down = -Z. Expect Y axis -> (0,0,-1).
    const float roll[4] = {fs, 0.0f, 0.0f, fs};
    ToArray(DroneNedToUeQuat(roll), q);
    const double ey[3] = {0.0, 1.0, 0.0};
    Rotate(q, ey, out);
    Check(Near(out[0], 0.0, 1e-6) && Near(out[1], 0.0, 1e-6) && Near(out[2], -1.0, 1e-6), "roll +90 maps UE Y to UE -Z");

    // Pitch +90 deg (nose up): NED North -> (0,0,-1) up. Unreal: S*(0,0,-1) = +Z. Expect X axis -> (0,0,1).
    const float pitch[4] = {0.0f, fs, 0.0f, fs};
    ToArray(DroneNedToUeQuat(pitch), q);
    Rotate(q, ex, out);
    Check(Near(out[0], 0.0, 1e-6) && Near(out[1], 0.0, 1e-6) && Near(out[2], 1.0, 1e-6), "pitch +90 maps UE X to UE +Z");
}

static void TestMirrorIdentity()
{
    // For a general rotation, S * R(q_ned) * S * v must equal R(q_ue) * v (R_ue = S R S).
    // Tolerance is float input rounding of the quaternion (components are stored as float).
    double n = std::sqrt(0.1 * 0.1 + 0.2 * 0.2 + 0.3 * 0.3 + 0.9 * 0.9);
    const float qn[4] = {static_cast<float>(0.1 / n), static_cast<float>(0.2 / n), static_cast<float>(0.3 / n),
                         static_cast<float>(0.9 / n)};
    double qned[4] = {qn[0], qn[1], qn[2], qn[3]};
    double que[4];
    ToArray(DroneNedToUeQuat(qn), que);

    for (int axis = 0; axis < 3; ++axis)
    {
        double v[3] = {0.0, 0.0, 0.0};
        v[axis] = 1.0;
        const double sv[3] = {v[0], v[1], -v[2]};
        double rned[3];
        Rotate(qned, sv, rned);
        const double lhs[3] = {rned[0], rned[1], -rned[2]};
        double rhs[3];
        Rotate(que, v, rhs);
        Check(Near(lhs[0], rhs[0], 1e-14) && Near(lhs[1], rhs[1], 1e-14) && Near(lhs[2], rhs[2], 1e-14),
              "mirror identity S R(q_ned) S v == R(q_ue) v");
    }
}

int main()
{
    TestPosition();
    TestVector();
    TestQuatComponents();
    TestAxisCases();
    TestMirrorIdentity();
    if (g_failures != 0)
    {
        std::printf("%d check(s) failed\n", g_failures);
        return 1;
    }
    std::printf("frame_conversion_test: all checks passed\n");
    return 0;
}
