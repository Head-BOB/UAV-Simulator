// Tier: V
// Display-only math for the Unreal demo: prop spin angle and spin quaternion, uniform mesh scale,
// motion-disc opacity.
// UE-free (only <cmath>), so tools/cpp_ffi_check/visual_math_test.cpp tests it with plain g++.
// Nothing here feeds back into the physics.
#pragma once

#include <cmath>

namespace DroneVisual
{
// The spin axis of a prop bone, in the BONE's local frame, and the sign that makes a motor with
// motor_spin = +1 (reaction torque about body +Z, i.e. a propeller turning counter-clockwise seen
// from above) turn the right way on screen. Both come from the imported skeleton and Unreal's
// handedness, which cannot be checked without the engine.
// TODO(HUMAN-VERIFY): in Unreal, watch one prop (FR, motor_spin +1) at low throttle. If it turns
// about the wrong axis, change kSpinAxisBoneLocal; if it turns clockwise seen from above, change
// kSpinDirectionScale to -1.0. This is the ONLY place either is set.
constexpr double kSpinAxisBoneLocal[3] = {0.0, 0.0, 1.0};
constexpr double kSpinDirectionScale = 1.0;
// Index (0=X, 1=Y, 2=Z) of the default axis above. The console variable drone.SpinAxis starts here.
// A test checks it agrees with kSpinAxisBoneLocal.
constexpr int kSpinAxisIndexDefault = 2;

// Axis index (0=X, 1=Y, 2=Z) to a unit vector in the bone's local frame. Returns false (Out
// untouched) for any other index, so a bad console value is caught by the caller, not guessed.
inline bool SpinAxisFromIndex(int Index, double Out[3])
{
    if (!Out || Index < 0 || Index > 2) return false;
    Out[0] = Out[1] = Out[2] = 0.0;
    Out[Index] = 1.0;
    return true;
}

// Console-variable spin sign to exactly +1 or -1: any finite value >= 0 is +1, any finite value < 0
// is -1. NaN / Inf returns the default kSpinDirectionScale.
inline double NormalizeSpinSign(double Value)
{
    if (!std::isfinite(Value)) return kSpinDirectionScale;
    return Value < 0.0 ? -1.0 : 1.0;
}

// Degrees per (RPM * second): 360 deg per revolution / 60 s per minute.
constexpr double kDegPerRpmSecond = 6.0;

// Advances a prop's spin angle. RPM is unsigned (as the Rust step reports it); the direction comes
// only from SpinSign (motor_spin, +1 or -1). Result is wrapped into [0, 360). A non-finite input
// returns PrevDeg unchanged, so a bad frame never poisons the angle.
//   delta = RPM * 6 deg / (RPM s) * dt * sign * kSpinDirectionScale
// DirectionScale defaults to kSpinDirectionScale; the demo passes the drone.SpinSign value instead.
inline double AdvanceSpinDeg(double PrevDeg, double Rpm, double SpinSign, double DtS, double DirectionScale = kSpinDirectionScale)
{
    if (!std::isfinite(PrevDeg) || !std::isfinite(Rpm) || !std::isfinite(SpinSign) || !std::isfinite(DtS) || !std::isfinite(DirectionScale))
    {
        return PrevDeg;
    }
    const double Next = PrevDeg + Rpm * kDegPerRpmSecond * DtS * SpinSign * DirectionScale;
    double W = std::fmod(Next, 360.0);
    if (W < 0.0) W += 360.0;
    return W >= 360.0 ? 0.0 : W;
}

// Plain quaternion, component order (x, y, z, w) like ADR-001 and Unreal's FQuat.
struct Quat
{
    double x = 0.0, y = 0.0, z = 0.0, w = 1.0;
};

// Hamilton product A*B in (x, y, z, w) order: the rotation B first, then A, so A*B applied to a
// vector v is A(B(v)). Same convention as Unreal's FQuat::operator*.
inline Quat QuatMul(const Quat& A, const Quat& B)
{
    Quat R;
    R.x = A.w * B.x + A.x * B.w + A.y * B.z - A.z * B.y;
    R.y = A.w * B.y - A.x * B.z + A.y * B.w + A.z * B.x;
    R.z = A.w * B.z + A.x * B.y - A.y * B.x + A.z * B.w;
    R.w = A.w * B.w - A.x * B.x - A.y * B.y - A.z * B.z;
    return R;
}

// Right-handed rotation of AngleRad about Axis: (axis*sin(a/2), cos(a/2)), order (x, y, z, w).
// Axis need not be unit length. A zero or non-finite axis / angle returns the identity.
// NOTE: Unreal is left-handed, so on screen this turns the opposite way; that is what drone.SpinSign is for.
inline Quat QuatFromAxisAngle(const double Axis[3], double AngleRad)
{
    if (!Axis || !std::isfinite(AngleRad) || !std::isfinite(Axis[0]) || !std::isfinite(Axis[1]) || !std::isfinite(Axis[2]))
    {
        return Quat();
    }
    const double N = std::sqrt(Axis[0] * Axis[0] + Axis[1] * Axis[1] + Axis[2] * Axis[2]);
    if (N <= 0.0) return Quat();
    const double S = std::sin(0.5 * AngleRad) / N;
    Quat Q;
    Q.x = Axis[0] * S;
    Q.y = Axis[1] * S;
    Q.z = Axis[2] * S;
    Q.w = std::cos(0.5 * AngleRad);
    return Q;
}

// Bone rotation to write each frame: the reference-pose rotation (same space the result is written
// in, ComponentSpace in the demo) times the spin about AxisBoneLocal. The spin is on the RIGHT, so
// it acts in the bone's own local frame and the spin axis stays fixed in that frame. Writing the
// spin alone would drop the reference rotation and point the prop the wrong way.
inline Quat SpinOnRefRotation(const Quat& RefRotation, const double AxisBoneLocal[3], double AngleRad)
{
    return QuatMul(RefRotation, QuatFromAxisAngle(AxisBoneLocal, AngleRad));
}

// Uniform scale = span in the config / span in the model. Returns false (Out untouched) if either
// span is not a finite positive number.
inline bool ComputeUniformScale(double ConfigSpanM, double ModelSpanM, double* Out)
{
    if (!Out || !std::isfinite(ConfigSpanM) || !std::isfinite(ModelSpanM) || ConfigSpanM <= 0.0 || ModelSpanM <= 0.0)
    {
        return false;
    }
    *Out = ConfigSpanM / ModelSpanM;
    return true;
}

// Motor-axis spans in the config, from motor_pos_m in the config's index order (FR, FL, RR, RL):
// left-right = mean of |y_FR - y_FL| and |y_RR - y_RL|, front-rear = mean of |x_FR - x_RR| and
// |x_FL - x_RL|. Metres.
inline void ConfigSpansM(const double MotorPosM[4][3], double* LeftRight, double* FrontRear)
{
    *LeftRight = 0.5 * (std::fabs(MotorPosM[0][1] - MotorPosM[1][1]) + std::fabs(MotorPosM[2][1] - MotorPosM[3][1]));
    *FrontRear = 0.5 * (std::fabs(MotorPosM[0][0] - MotorPosM[2][0]) + std::fabs(MotorPosM[1][0] - MotorPosM[3][0]));
}

// The front-rear span the scaled model ends up with, minus the config's: metres, negative when the
// scaled model is shorter. This is the leftover that a single uniform scale cannot remove.
inline double FrontRearMismatchM(double Scale, double ModelFrontRearM, double ConfigFrontRearM)
{
    return Scale * ModelFrontRearM - ConfigFrontRearM;
}

// Motion-disc opacity: linear in RPM / max_rpm, clamped to [0, MaxOpacity].
// TODO(HUMAN-VERIFY: demo value) MaxOpacity 0.45 is a look-and-feel choice.
constexpr double kMotionDiscMaxOpacity = 0.45;

inline double MotionDiscOpacity(double Rpm, double MaxRpm)
{
    if (!std::isfinite(Rpm) || !std::isfinite(MaxRpm) || MaxRpm <= 0.0 || Rpm <= 0.0) return 0.0;
    const double Fraction = Rpm >= MaxRpm ? 1.0 : Rpm / MaxRpm;
    return Fraction * kMotionDiscMaxOpacity;
}
}  // namespace DroneVisual
