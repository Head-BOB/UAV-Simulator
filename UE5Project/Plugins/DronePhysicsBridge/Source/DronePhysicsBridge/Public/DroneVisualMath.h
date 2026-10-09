// Tier: V
// Display-only math for the Unreal demo: prop spin angle, uniform mesh scale, motion-disc opacity.
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

// Degrees per (RPM * second): 360 deg per revolution / 60 s per minute.
constexpr double kDegPerRpmSecond = 6.0;

// Advances a prop's spin angle. RPM is unsigned (as the Rust step reports it); the direction comes
// only from SpinSign (motor_spin, +1 or -1). Result is wrapped into [0, 360). A non-finite input
// returns PrevDeg unchanged, so a bad frame never poisons the angle.
//   delta = RPM * 6 deg / (RPM s) * dt * sign * kSpinDirectionScale
inline double AdvanceSpinDeg(double PrevDeg, double Rpm, double SpinSign, double DtS)
{
    if (!std::isfinite(PrevDeg) || !std::isfinite(Rpm) || !std::isfinite(SpinSign) || !std::isfinite(DtS))
    {
        return PrevDeg;
    }
    const double Next = PrevDeg + Rpm * kDegPerRpmSecond * DtS * SpinSign * kSpinDirectionScale;
    double W = std::fmod(Next, 360.0);
    if (W < 0.0) W += 360.0;
    return W >= 360.0 ? 0.0 : W;
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
