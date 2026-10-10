// Tier: V
// DEMO ONLY engineering formulas for the flight report: prop shaft power, brushless motor current and
// voltage, ESC and battery current, battery sag, arm bending stress, and the margin rule. UE-free (only
// <cmath>), tested by tools/cpp_ffi_check/electrical_test.cpp. These are textbook first-order models fed
// with CITED or ASSUMED numbers (DroneReferenceBuild.h): they are NOT validated against hardware and the
// report says so. Nothing here feeds back into the physics core.
//
// Models (n = rpm / 60 in rev/s, SI units everywhere):
//   prop shaft power    P = Cp rho n^3 D^5                (Cp from the UIUC static fit, constant: forward-flight changes are NOT modelled)
//   shaft torque        Q = P / (2 pi n)
//   motor torque const  Kt = 60 / (2 pi Kv)               [N m / A]
//   motor current       I = Q / Kt + I0                    (I0 = no-load current, held constant)
//   motor terminal volt V = rpm / Kv + I Rm                (back-EMF + resistive drop)
//   battery current     Ib = V I / (eta_esc V_batt)        per motor; the pack carries the sum
//   pack terminal volt  Vt = V_oc - I_total R_int          (flat V_oc: no discharge curve)
//   arm bending         sigma = F L (D / 2) / I_area,  I_area = pi (D^4 - d^4) / 64   (cantilever, tip load F at lever L)
//   margin              limit / peak  (> 1 is inside the limit); no load gives kNoLoadMargin
#pragma once

#include <cmath>

namespace DroneElec
{
constexpr double kPi = 3.14159265358979323846;
// Margin reported when the peak load is zero (nothing used the component): a cap, not infinity, so tables sort and print.
constexpr double kNoLoadMargin = 99.0;

inline bool Finite(double a)
{
    return std::isfinite(a);
}

// Shaft power of one prop, W. Returns false (Out untouched) for a non-finite or negative input.
inline bool ShaftPowerW(double Cp, double RhoKgM3, double Rpm, double DiameterM, double* Out)
{
    if (!Out || !Finite(Cp) || !Finite(RhoKgM3) || !Finite(Rpm) || !Finite(DiameterM)) return false;
    if (Cp < 0.0 || RhoKgM3 <= 0.0 || Rpm < 0.0 || DiameterM <= 0.0) return false;
    const double N = Rpm / 60.0;
    *Out = Cp * RhoKgM3 * N * N * N * std::pow(DiameterM, 5.0);
    return true;
}

// Shaft torque, N m. Zero RPM gives zero torque.
inline bool ShaftTorqueNm(double PowerW, double Rpm, double* Out)
{
    if (!Out || !Finite(PowerW) || !Finite(Rpm) || PowerW < 0.0 || Rpm < 0.0) return false;
    *Out = Rpm == 0.0 ? 0.0 : PowerW / (2.0 * kPi * (Rpm / 60.0));
    return true;
}

// Motor current, A: Q / Kt + I0 while the motor turns, 0 when it is off (Rpm == 0).
inline bool MotorCurrentA(double TorqueNm, double Rpm, double KvRpmPerV, double NoLoadA, double* Out)
{
    if (!Out || !Finite(TorqueNm) || !Finite(Rpm) || !Finite(KvRpmPerV) || !Finite(NoLoadA)) return false;
    if (TorqueNm < 0.0 || Rpm < 0.0 || KvRpmPerV <= 0.0 || NoLoadA < 0.0) return false;
    const double Kt = 60.0 / (2.0 * kPi * KvRpmPerV);
    *Out = Rpm == 0.0 ? 0.0 : TorqueNm / Kt + NoLoadA;
    return true;
}

// Motor terminal voltage, V: back-EMF rpm / Kv plus the winding drop I Rm.
inline bool MotorVoltageV(double Rpm, double KvRpmPerV, double CurrentA, double ResistanceOhm, double* Out)
{
    if (!Out || !Finite(Rpm) || !Finite(KvRpmPerV) || !Finite(CurrentA) || !Finite(ResistanceOhm)) return false;
    if (Rpm < 0.0 || KvRpmPerV <= 0.0 || CurrentA < 0.0 || ResistanceOhm < 0.0) return false;
    *Out = Rpm / KvRpmPerV + CurrentA * ResistanceOhm;
    return true;
}

// Battery-side current of one motor, A: V I / (eta V_batt).
inline bool BatteryCurrentA(double MotorVoltage, double MotorCurrent, double PackVoltage, double EscEfficiency, double* Out)
{
    if (!Out || !Finite(MotorVoltage) || !Finite(MotorCurrent) || !Finite(PackVoltage) || !Finite(EscEfficiency)) return false;
    if (MotorVoltage < 0.0 || MotorCurrent < 0.0 || PackVoltage <= 0.0 || EscEfficiency <= 0.0 || EscEfficiency > 1.0) return false;
    *Out = MotorVoltage * MotorCurrent / (EscEfficiency * PackVoltage);
    return true;
}

// Pack terminal voltage under load, V.
inline bool PackTerminalV(double OpenCircuitV, double TotalCurrentA, double InternalOhm, double* Out)
{
    if (!Out || !Finite(OpenCircuitV) || !Finite(TotalCurrentA) || !Finite(InternalOhm)) return false;
    if (OpenCircuitV <= 0.0 || TotalCurrentA < 0.0 || InternalOhm < 0.0) return false;
    *Out = OpenCircuitV - TotalCurrentA * InternalOhm;
    return true;
}

// Second moment of area of a hollow round tube, m^4: pi (D^4 - d^4) / 64.
inline bool TubeSecondMomentM4(double OuterM, double InnerM, double* Out)
{
    if (!Out || !Finite(OuterM) || !Finite(InnerM) || OuterM <= 0.0 || InnerM < 0.0 || InnerM >= OuterM) return false;
    *Out = kPi * (std::pow(OuterM, 4.0) - std::pow(InnerM, 4.0)) / 64.0;
    return true;
}

// Peak bending stress in a cantilever tube with a tip force, Pa: F L (D / 2) / I_area.
inline bool ArmBendingStressPa(double TipForceN, double LeverM, double OuterM, double InnerM, double* Out)
{
    double I = 0.0;
    if (!Out || !Finite(TipForceN) || !Finite(LeverM) || TipForceN < 0.0 || LeverM <= 0.0) return false;
    if (!TubeSecondMomentM4(OuterM, InnerM, &I)) return false;
    *Out = TipForceN * LeverM * (OuterM / 2.0) / I;
    return true;
}

// margin = limit / peak. A zero or negative peak (no load) gives kNoLoadMargin; the result is also capped at it.
// Returns false (Out untouched) for a non-finite value or a non-positive limit.
inline bool Margin(double Limit, double Peak, double* Out)
{
    if (!Out || !Finite(Limit) || !Finite(Peak) || Limit <= 0.0) return false;
    if (Peak <= 0.0)
    {
        *Out = kNoLoadMargin;
        return true;
    }
    const double M = Limit / Peak;
    *Out = M > kNoLoadMargin ? kNoLoadMargin : M;
    return true;
}

// Verdict from the smallest margin. Thresholds are choices (TODO(HUMAN-VERIFY: demo value)), not standards.
enum class Verdict
{
    WithinLimits,       // margin >= 1.5
    Marginal,           // 1.0 <= margin < 1.5
    NotRecommended,     // margin < 1.0
};

constexpr double kWithinLimitsMargin = 1.5;
constexpr double kMarginalMargin = 1.0;

inline Verdict VerdictFromMargin(double MinMargin)
{
    if (!Finite(MinMargin) || MinMargin < kMarginalMargin) return Verdict::NotRecommended;
    return MinMargin >= kWithinLimitsMargin ? Verdict::WithinLimits : Verdict::Marginal;
}

inline const char* VerdictText(Verdict V)
{
    switch (V)
    {
        case Verdict::WithinLimits: return "within limits";
        case Verdict::Marginal: return "marginal";
        case Verdict::NotRecommended: return "not recommended";
    }
    return "unknown";
}
}  // namespace DroneElec
