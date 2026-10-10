// Tier: V
// DEMO ONLY flight-report engine: turns a flight log into ranked component margins with causes,
// consequences, tested improvement suggestions, a per-environment verdict, and JSON / CSV / HTML text.
// UE-free (standard library only), tested by tools/cpp_ffi_check/report_test.cpp. It never feeds back
// into the physics core. Every limit carries its status: CITED (a datasheet page) or ASSUMED; every
// result is PRELIMINARY: the models are first-order, nothing is validated against hardware.
//
// margin = limit / peak load (> 1 is inside the limit). Weakest component first. A component with no
// design data is listed as NOT ASSESSED, never given a made-up margin.
#pragma once

#include "DroneElectrical.h"
#include "DroneMission.h"
#include "Generated/DroneReferenceBuild.h"

#include <algorithm>
#include <cmath>
#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <functional>
#include <string>
#include <vector>

namespace DroneReport
{
using DroneElec::Verdict;

constexpr int kMotors = 4;
constexpr double kSampleDtS = 0.02;  // the runner logs at 50 Hz

inline const char* MotorName(int I)
{
    static const char* const kNames[kMotors] = {"FR", "FL", "RR", "RL"};  // config order
    return (I >= 0 && I < kMotors) ? kNames[I] : "??";
}

// Prop bone of each motor, from DroneModelMap.json (FR = prop_1, FL = prop_4, RR = prop_2, RL = prop_3).
inline const char* PropBoneName(int I)
{
    static const char* const kBones[kMotors] = {"prop_1", "prop_4", "prop_2", "prop_3"};
    return (I >= 0 && I < kMotors) ? kBones[I] : "prop_?";
}

inline std::string Fmt(const char* Format, ...)
{
    char Buf[512];
    va_list Args;
    va_start(Args, Format);
    std::vsnprintf(Buf, sizeof(Buf), Format, Args);
    va_end(Args);
    return std::string(Buf);
}

// ---------------------------------------------------------------------------------------------
// Inputs
// ---------------------------------------------------------------------------------------------

// One logged instant (50 Hz). Plain numbers only.
struct Sample
{
    double TimeS = 0.0;
    int Phase = 0;                 // DroneMission::Phase
    double AltitudeM = 0.0;
    double TiltRad = 0.0;          // max(|roll|, |pitch|)
    double RhoKgM3 = 1.225;        // air density at the vehicle
    double ThrustN[kMotors] = {0, 0, 0, 0};
    double Rpm[kMotors] = {0, 0, 0, 0};
    double CurrentA[kMotors] = {0, 0, 0, 0};   // motor current from DroneElectrical
    double PowerW[kMotors] = {0, 0, 0, 0};     // motor electrical power V I
    double MotorTempC[kMotors] = {0, 0, 0, 0}; // core thermal model, UNVALIDATED placeholder
    double BatteryA = 0.0;         // pack current
    double PackV = 0.0;            // terminal voltage under load
    double UsedAh = 0.0;           // cumulative
    double AeroDragN = 0.0;
    double AxialClimbMps = 0.0;    // speed of the air through the props along the thrust axis, + = climbing (descent is 0: not in the data)
    double WindMps = 0.0;          // total wind magnitude
    double GustMps = 0.0;          // gust part magnitude
    double CrossTrackM = 0.0;      // during a leg, else distance to the start
    bool Saturated = false;        // any motor at or above the saturation throttle
};

struct Log
{
    std::vector<Sample> Samples;
};

// Component numbers, from the generated DroneRef constants (CITED or ASSUMED, see DroneReferenceBuild.h)
// with the vehicle values the core flew. Variants change a few of them.
struct Build
{
    double MassKg = 1.516;
    double MaxRpm = 8100.0;
    double PropCt = DroneRef::kPropCt;
    double PropCp = DroneRef::kPropCp;
    double PropDiameterM = DroneRef::kPropDiameterM;
    double MotorKv = DroneRef::kMotorKvRpmPerV;
    double MotorResistanceOhm = DroneRef::kMotorResistanceOhm;
    double MotorNoLoadA = DroneRef::kMotorNoLoadCurrentA;
    double MotorMaxCurrentA = DroneRef::kMotorMaxContinuousCurrentA;   // CITED, 180 s rating
    double MotorMaxPowerW = DroneRef::kMotorMaxPowerW;                 // CITED, 180 s rating
    double MotorTempLimitC = DroneRef::kMotorTempLimitC;               // ASSUMED
    double MotorTempBaselineC = 20.0;                                  // core thermal placeholder ambient
    double EscRatedA = DroneRef::kEscRatedCurrentA;                    // class CITED
    double EscEfficiency = DroneRef::kEscEfficiency;                   // ASSUMED
    double BatteryVoc = DroneRef::kBatteryNominalVoltageV;             // ASSUMED, flat
    double BatteryCapacityAh = DroneRef::kBatteryCapacityAh;           // ASSUMED
    double BatteryResistanceOhm = DroneRef::kBatteryInternalResistanceOhm;  // ASSUMED
    double BatteryMaxC = DroneRef::kBatteryMaxDischargeC;              // ASSUMED
    double BatteryUsable = DroneRef::kBatteryUsableFraction;           // ASSUMED
    double Cells = DroneRef::kBatteryCells;                            // ASSUMED
    double CellCutoffV = 3.3;                                          // ASSUMED common LiPo cut-off
    double ArmOuterM = DroneRef::kFrameArmTubeOuterDiameterM;          // ASSUMED
    double ArmWallM = DroneRef::kFrameArmTubeWallM;                    // ASSUMED
    double ArmAllowablePa = DroneRef::kFrameArmAllowableBendingStressPa;  // ASSUMED, not a tested value
    double ArmLeverM = 0.0;
    double MaxTiltRad = 0.5235987755982988;                            // 30 deg flight-envelope tilt, ASSUMED (the demo controller itself clamps at 20 deg)
    double SaturationThrottle = 0.98;                                  // ASSUMED
    double SaturationLimitFraction = 0.05;                             // ASSUMED
    double TrackingLimitM = 3.0;                                       // ASSUMED

    static Build Reference()
    {
        Build B;
        // Arm free length: from the plate corner (half width x sqrt 2) to the motor centre (spacing / 2 x sqrt 2).
        const double Root2 = 1.4142135623730951;
        B.ArmLeverM = (DroneRef::kFrameMotorSpacingM / 2.0 - DroneRef::kFrameBodyHalfWidthM) * Root2;
        return B;
    }
    double ArmInnerM() const { return ArmOuterM - 2.0 * ArmWallM; }
};

struct EnvInfo
{
    std::string Id = "Standard";
    std::string Name = "Standard day";
    double SiteAltitudeM = 0.0;
    double AmbientTempC = 15.0;
    double HumidityPct = 0.0;
    double WindSpeedMps = 0.0;
    double WindTowardDeg = 0.0;
    double GustSigmaMps = 0.0;
    double TempOffsetC = 0.0;
};

struct MissionOutcome
{
    bool Completed = false;
    std::string Reason;     // why it did not complete
    double DurationS = 0.0;
    int StepErrors = 0;
};

// ---------------------------------------------------------------------------------------------
// Results
// ---------------------------------------------------------------------------------------------

struct Suggestion
{
    std::string Text;
    std::string Evidence;       // what the re-flight measured, or why it was not tested
    bool Tested = false;
    bool Improves = false;
    double MarginBefore = 0.0;
    double MarginAfter = 0.0;
};

struct Finding
{
    std::string Id;           // kind, e.g. "motor_current"
    std::string Component;    // e.g. "Motor FL"
    std::string Metric;
    std::string Unit;
    std::string LimitStatus;  // CITED / ASSUMED / "controller" (the demo controller's own limit)
    bool Assessed = true;
    double Limit = 0.0;
    double PeakValue = 0.0;
    double Margin = 0.0;
    double PeakTimeS = 0.0;
    double SecondsAbove90 = 0.0;   // time at or above 90 % of the limit
    std::string PeakPhase;
    std::string Cause;
    std::string Consequence;
    std::vector<Suggestion> Suggestions;
};

struct Analysis
{
    std::vector<Finding> Findings;   // ranked: smallest margin first, not-assessed last
    double MinMargin = 0.0;
    std::string Limiting;            // component of the smallest margin
    Verdict V = Verdict::NotRecommended;
    std::string VerdictReason;
    bool Completed = false;          // the mission reached Done
};

// ---------------------------------------------------------------------------------------------
// Analysis
// ---------------------------------------------------------------------------------------------

inline const char* ConsequenceFor(const std::string& Id)
{
    if (Id == "motor_current") return "Winding heating beyond the continuous rating: shorter motor life and a risk of overheating or burnt windings; the ESC and wiring heat up too.";
    if (Id == "motor_power") return "Electrical power above the rating for a sustained time: overheating risk.";
    if (Id == "prop_thrust") return "No thrust reserve: the controller cannot recover from a gust or ask for more climb, so attitude or altitude can be lost.";
    if (Id == "esc_current") return "The ESC overheats or trips its protection: a motor can cut out in flight.";
    if (Id == "battery_current") return "The pack heats and sags; above its rated discharge it ages quickly and can fail.";
    if (Id == "battery_voltage") return "Voltage sag can reset the flight controller or trigger a low-voltage cut-off.";
    if (Id == "battery_capacity") return "Not enough energy for the mission with the reserve: a forced landing.";
    if (Id == "motor_temp") return "Motor overheating. The core thermal model is an UNVALIDATED placeholder, so treat the number as a trend only.";
    if (Id == "arm_bending") return "Arm overstress: cracking or breakage of the carbon tube at the clamp.";
    if (Id == "saturation") return "The mixer clips: the controller loses authority and tracking degrades.";
    if (Id == "tracking") return "The vehicle drifts off the planned path.";
    if (Id == "tilt") return "Attitude at the controller limit: no manoeuvre margin left.";
    if (Id == "mission") return "The mission was not completed.";
    return "";
}

// Ct(J) / Ct_static for axial inflow, from the UIUC run in DroneReferenceBuild.h. Linear in J. At J = 0 the ratio is 1
// (the static fit); between J = 0 and the first measured J it is interpolated from there; beyond the last J it holds the
// last value. Never below 0 (negative Ct is windmill state, not a thrust loss we report).
inline double AxialCtRatio(double J)
{
    if (!std::isfinite(J) || J <= 0.0) return 1.0;
    const double CtStatic = DroneRef::kPropCt;
    const double R0 = DroneRef::kPropAxialCt[0] / CtStatic;
    double R = R0;
    if (J < DroneRef::kPropAxialJ[0])
    {
        R = 1.0 + (R0 - 1.0) * (J / DroneRef::kPropAxialJ[0]);
    }
    else if (J >= DroneRef::kPropAxialJ[DroneRef::kPropAxialCount - 1])
    {
        R = DroneRef::kPropAxialCt[DroneRef::kPropAxialCount - 1] / CtStatic;
    }
    else
    {
        for (std::size_t I = 1; I < DroneRef::kPropAxialCount; ++I)
        {
            if (J <= DroneRef::kPropAxialJ[I])
            {
                const double T = (J - DroneRef::kPropAxialJ[I - 1]) / (DroneRef::kPropAxialJ[I] - DroneRef::kPropAxialJ[I - 1]);
                R = (DroneRef::kPropAxialCt[I - 1] + T * (DroneRef::kPropAxialCt[I] - DroneRef::kPropAxialCt[I - 1])) / CtStatic;
                break;
            }
        }
    }
    return R < 0.0 ? 0.0 : R;
}

// Moving average over W samples (centred at the end of the window), O(n).
inline std::vector<double> MovingAverage(const std::vector<double>& V, std::size_t W)
{
    std::vector<double> Out(V.size(), 0.0);
    if (V.empty() || W == 0) return Out;
    double Sum = 0.0;
    for (std::size_t I = 0; I < V.size(); ++I)
    {
        Sum += V[I];
        if (I >= W) Sum -= V[I - W];
        const std::size_t N = I + 1 < W ? I + 1 : W;
        Out[I] = Sum / static_cast<double>(N);
    }
    return Out;
}

// Builds a finding from a value series and a utilisation series (value / limit, or limit / value for minimums).
// LimitSeries (optional) gives the limit at each sample when it changes with time (thrust vs air density).
inline Finding FromSeries(const char* Id, const std::string& Component, const char* Metric, const char* Unit, double Limit,
                          const char* LimitStatus, const Log& L, const std::vector<double>& Value, const std::vector<double>& Util,
                          const std::vector<double>* LimitSeries = nullptr)
{
    Finding F;
    F.Id = Id;
    F.Component = Component;
    F.Metric = Metric;
    F.Unit = Unit;
    F.LimitStatus = LimitStatus;
    F.Limit = Limit;
    F.Consequence = ConsequenceFor(F.Id);
    if (Util.empty() || Util.size() != L.Samples.size())
    {
        F.Assessed = false;
        F.Cause = "no data";
        return F;
    }
    std::size_t Peak = 0;
    std::size_t Above = 0;
    for (std::size_t I = 0; I < Util.size(); ++I)
    {
        if (Util[I] > Util[Peak]) Peak = I;
        if (Util[I] >= 0.9) ++Above;
    }
    if (LimitSeries && LimitSeries->size() == Util.size()) Limit = (*LimitSeries)[Peak];
    F.Limit = Limit;
    F.PeakValue = Value[Peak];
    F.PeakTimeS = L.Samples[Peak].TimeS;
    F.SecondsAbove90 = static_cast<double>(Above) * kSampleDtS;
    F.PeakPhase = DroneMission::PhaseName(static_cast<DroneMission::Phase>(L.Samples[Peak].Phase));
    // Margin = limit / peak load = 1 / peak utilisation (no load: capped).
    double M = 0.0;
    DroneElec::Margin(1.0, Util[Peak], &M);
    F.Margin = M;

    // Cause: where the load was, how long it stayed above 90 %, which phase had most of it, and the wind then.
    int PhaseCount[16] = {0};
    double WindMax = 0.0, GustMax = 0.0;
    for (std::size_t I = 0; I < Util.size(); ++I)
    {
        if (Util[I] < 0.9) continue;
        const int P = L.Samples[I].Phase;
        if (P >= 0 && P < 16) ++PhaseCount[P];
        WindMax = std::max(WindMax, L.Samples[I].WindMps);
        GustMax = std::max(GustMax, L.Samples[I].GustMps);
    }
    const double PeakPct = 100.0 * Util[Peak];
    std::string Cause = Fmt("Peak %.4g %s = %.0f %% of the %.4g %s limit (%s) at t = %.1f s during %s.", F.PeakValue, Unit, PeakPct, Limit, Unit,
                            LimitStatus, F.PeakTimeS, F.PeakPhase.c_str());
    if (Above == 0)
    {
        Cause += " Never above 90 % of the limit.";
    }
    else
    {
        int Best = 0;
        for (int P = 1; P < 16; ++P)
            if (PhaseCount[P] > PhaseCount[Best]) Best = P;
        Cause += Fmt(" At or above 90 %% of the limit for %.1f s, mostly during %s; wind then up to %.1f m/s (gust part %.1f m/s).", F.SecondsAbove90,
                     DroneMission::PhaseName(static_cast<DroneMission::Phase>(Best)), WindMax, GustMax);
    }
    F.Cause = Cause;
    return F;
}

inline Analysis Analyze(const Log& L, const Build& B, const MissionOutcome& Out)
{
    Analysis A;
    A.Completed = Out.Completed;
    const std::size_t N = L.Samples.size();
    const std::size_t OneSecond = static_cast<std::size_t>(1.0 / kSampleDtS + 0.5);

    // Per-motor series and the worst motor for each metric (1 s moving average smooths single-sample spikes).
    auto Worst = [&](const char* Id, const char* Metric, const char* Unit, double Limit, const char* Status, auto Value, bool Smooth,
                     const char* Prefix, bool Prop) {
        Finding Best;
        bool Have = false;
        for (int M = 0; M < kMotors; ++M)
        {
            std::vector<double> V(N);
            for (std::size_t I = 0; I < N; ++I) V[I] = Value(L.Samples[I], M);
            if (Smooth) V = MovingAverage(V, OneSecond);
            std::vector<double> U(N);
            for (std::size_t I = 0; I < N; ++I) U[I] = Limit > 0.0 ? V[I] / Limit : 0.0;
            const std::string Name = Prop ? DroneReport::Fmt("Propeller %s (%s)", MotorName(M), PropBoneName(M)) : DroneReport::Fmt("%s %s", Prefix, MotorName(M));
            Finding F = FromSeries(Id, Name, Metric, Unit, Limit, Status, L, V, U);
            if (!Have || F.Margin < Best.Margin)
            {
                Best = F;
                Have = true;
            }
        }
        return Best;
    };

    A.Findings.push_back(Worst("motor_current", "motor current (1 s average)", "A", B.MotorMaxCurrentA, "CITED", [](const Sample& S, int M) { return S.CurrentA[M]; }, true, "Motor", false));
    A.Findings.push_back(Worst("motor_power", "motor electrical power (1 s average)", "W", B.MotorMaxPowerW, "CITED", [](const Sample& S, int M) { return S.PowerW[M]; }, true, "Motor", false));
    A.Findings.push_back(Worst("esc_current", "ESC output current (1 s average)", "A", B.EscRatedA, "CITED", [](const Sample& S, int M) { return S.CurrentA[M]; }, true, "ESC", false));

    // Propeller thrust headroom: limit is the full-throttle thrust at THIS air density, so it changes with altitude.
    {
        Finding Best;
        bool Have = false;
        const double Nmax = B.MaxRpm / 60.0;
        std::vector<double> Lim(N);
        for (std::size_t I = 0; I < N; ++I) Lim[I] = B.PropCt * L.Samples[I].RhoKgM3 * Nmax * Nmax * std::pow(B.PropDiameterM, 4.0);
        for (int M = 0; M < kMotors; ++M)
        {
            std::vector<double> V(N), U(N);
            for (std::size_t I = 0; I < N; ++I)
            {
                V[I] = L.Samples[I].ThrustN[M];
                U[I] = Lim[I] > 0.0 ? V[I] / Lim[I] : 0.0;
            }
            Finding F = FromSeries("prop_thrust", DroneReport::Fmt("Propeller %s (%s)", MotorName(M), PropBoneName(M)), "thrust vs full-throttle thrust", "N", 0.0,
                                   "ASSUMED", L, V, U, &Lim);
            if (!Have || F.Margin < Best.Margin)
            {
                Best = F;
                Have = true;
            }
        }
        A.Findings.push_back(Best);
    }

    // Battery.
    {
        std::vector<double> Amps(N), UAmps(N), Sag(N), USag(N), Used(N), UUsed(N);
        const double CurLimit = B.BatteryMaxC * B.BatteryCapacityAh;
        // The model's open-circuit voltage is flat, so a margin against the cut-off voltage itself would be fixed by the
        // model. What the flight changes is the SAG under load: allowed sag = V_oc - cut-off (3.3 V per cell, ASSUMED).
        const double AllowedSag = B.BatteryVoc - B.CellCutoffV * B.Cells;
        const double Usable = B.BatteryCapacityAh * B.BatteryUsable;
        for (std::size_t I = 0; I < N; ++I) Amps[I] = L.Samples[I].BatteryA;
        Amps = MovingAverage(Amps, OneSecond);
        for (std::size_t I = 0; I < N; ++I)
        {
            UAmps[I] = CurLimit > 0.0 ? Amps[I] / CurLimit : 0.0;
            Sag[I] = B.BatteryVoc - L.Samples[I].PackV;
            USag[I] = AllowedSag > 0.0 ? Sag[I] / AllowedSag : 0.0;
            Used[I] = L.Samples[I].UsedAh;
            UUsed[I] = Usable > 0.0 ? Used[I] / Usable : 0.0;
        }
        A.Findings.push_back(FromSeries("battery_current", "Battery", "pack discharge current (1 s average)", "A", CurLimit, "ASSUMED", L, Amps, UAmps));
        A.Findings.push_back(FromSeries("battery_voltage", "Battery", "voltage sag under load (allowed: open-circuit minus cut-off)", "V", AllowedSag, "ASSUMED", L, Sag, USag));
        A.Findings.push_back(FromSeries("battery_capacity", "Battery", "capacity used (usable = 80 % of rated)", "Ah", Usable, "ASSUMED", L, Used, UUsed));
    }

    // Axial-flow thrust loss: INFORMATION, not a margin. The core uses the constant static Ct; in a climb the UIUC data show
    // less thrust. The core is therefore optimistic by this much; the report says so instead of changing its numbers.
    {
        double WorstLoss = 0.0, WorstT = 0.0, WorstJ = 0.0;
        int WorstPhase = 0;
        for (const Sample& S : L.Samples)
        {
            const double Rpm = (S.Rpm[0] + S.Rpm[1] + S.Rpm[2] + S.Rpm[3]) / 4.0;
            if (Rpm < 300.0 || S.AxialClimbMps <= 0.0 || B.PropDiameterM <= 0.0) continue;
            const double J = S.AxialClimbMps / ((Rpm / 60.0) * B.PropDiameterM);
            const double Loss = 100.0 * (1.0 - AxialCtRatio(J));
            if (Loss > WorstLoss)
            {
                WorstLoss = Loss;
                WorstT = S.TimeS;
                WorstJ = J;
                WorstPhase = S.Phase;
            }
        }
        Finding F;
        F.Id = "prop_axial";
        F.Component = "Propeller thrust model (axial climb)";
        F.Metric = "not assessed (model limitation)";
        F.Unit = "%";
        F.Assessed = false;
        F.LimitStatus = "none";
        F.PeakValue = WorstLoss;
        F.PeakTimeS = WorstT;
        F.PeakPhase = DroneMission::PhaseName(static_cast<DroneMission::Phase>(WorstPhase));
        F.Cause = Fmt("The core uses a constant static thrust coefficient. In axial flow the UIUC wind-tunnel data (run near %.0f RPM, CITED) show less thrust: "
                      "up to %.1f %% less at J = %.3f (t = %.1f s, %s). The thrust margins above are optimistic by about that much in a climb; "
                      "descent and edgewise (forward) flow are not in the data.",
                      DroneRef::kPropAxialRunRpm, WorstLoss, WorstJ, WorstT, F.PeakPhase.c_str());
        A.Findings.push_back(F);
    }

    // Motor temperature: NOT ASSESSED. The core's thermal model is an UNVALIDATED placeholder (thermal_valid = 0)
    // whose constants belong to a different motor, so its temperatures are not used for a margin; the datasheet
    // 180 s current and power ratings above cover the thermal limit. The peak it reported is shown for transparency.
    {
        double PeakC = 0.0;
        for (const Sample& S : L.Samples)
            for (int M = 0; M < kMotors; ++M) PeakC = std::max(PeakC, S.MotorTempC[M]);
        Finding F;
        F.Id = "motor_temp";
        F.Component = "Motor temperature (core thermal model)";
        F.Metric = "not assessed";
        F.Unit = "C";
        F.Assessed = false;
        F.LimitStatus = "none";
        F.PeakValue = PeakC;
        F.Cause = Fmt("The core's thermal model is an UNVALIDATED placeholder (thermal_valid = 0) with constants for a different motor; it reported up to %.0f C, "
                      "which is not used. The thermal limit is covered by the CITED 180 s current and power ratings.", PeakC);
        F.Consequence = ConsequenceFor("motor_temp");
        A.Findings.push_back(F);
    }

    // Arms: cantilever tube, tip load = motor thrust + a quarter of the aero drag (arithmetic sum, conservative).
    {
        Finding Best;
        bool Have = false;
        for (int M = 0; M < kMotors; ++M)
        {
            std::vector<double> V(N), U(N);
            for (std::size_t I = 0; I < N; ++I)
            {
                double Sigma = 0.0;
                const double F = L.Samples[I].ThrustN[M] + 0.25 * L.Samples[I].AeroDragN;
                DroneElec::ArmBendingStressPa(F, B.ArmLeverM, B.ArmOuterM, B.ArmInnerM(), &Sigma);
                V[I] = Sigma / 1.0e6;  // MPa
                U[I] = B.ArmAllowablePa > 0.0 ? Sigma / B.ArmAllowablePa : 0.0;
            }
            Finding F = FromSeries("arm_bending", DroneReport::Fmt("Arm %s", MotorName(M)), "bending stress at the clamp", "MPa", B.ArmAllowablePa / 1.0e6, "ASSUMED", L, V, U);
            if (!Have || F.Margin < Best.Margin)
            {
                Best = F;
                Have = true;
            }
        }
        A.Findings.push_back(Best);
    }

    // Controller: saturation time share, path tracking, tilt.
    {
        std::vector<double> Cross(N), UCross(N), Tilt(N), UTilt(N);
        std::size_t SatCount = 0, FirstSat = N;
        int SatPhase[16] = {0};
        double SatWind = 0.0;
        for (std::size_t I = 0; I < N; ++I)
        {
            if (L.Samples[I].Saturated)
            {
                ++SatCount;
                if (FirstSat == N) FirstSat = I;
                const int P = L.Samples[I].Phase;
                if (P >= 0 && P < 16) ++SatPhase[P];
                SatWind = std::max(SatWind, L.Samples[I].WindMps);
            }
            const bool OnLeg = L.Samples[I].Phase >= static_cast<int>(DroneMission::Phase::Leg1) && L.Samples[I].Phase <= static_cast<int>(DroneMission::Phase::Leg4);
            Cross[I] = OnLeg ? L.Samples[I].CrossTrackM : 0.0;
            UCross[I] = Cross[I] / B.TrackingLimitM;
            Tilt[I] = L.Samples[I].TiltRad * 180.0 / DroneElec::kPi;
            UTilt[I] = B.MaxTiltRad > 0.0 ? L.Samples[I].TiltRad / B.MaxTiltRad : 0.0;
        }
        Finding Sat;
        Sat.Id = "saturation";
        Sat.Component = "Controller";
        Sat.Metric = "share of flight time with a motor near full throttle";
        Sat.Unit = "%";
        Sat.LimitStatus = "ASSUMED";
        Sat.Limit = 100.0 * B.SaturationLimitFraction;
        Sat.Consequence = ConsequenceFor("saturation");
        if (N == 0)
        {
            Sat.Assessed = false;
            Sat.Cause = "no data";
        }
        else
        {
            const double Fraction = static_cast<double>(SatCount) / static_cast<double>(N);
            DroneElec::Margin(B.SaturationLimitFraction, Fraction, &Sat.Margin);
            Sat.PeakValue = 100.0 * Fraction;
            Sat.SecondsAbove90 = static_cast<double>(SatCount) * kSampleDtS;
            Sat.PeakTimeS = FirstSat < N ? L.Samples[FirstSat].TimeS : 0.0;
            int Best = 0;
            for (int P = 1; P < 16; ++P)
                if (SatPhase[P] > SatPhase[Best]) Best = P;
            Sat.PeakPhase = DroneMission::PhaseName(static_cast<DroneMission::Phase>(Best));
            Sat.Cause = SatCount == 0 ? Fmt("No motor reached %.0f %% throttle (limit %.0f %% of the flight, ASSUMED).", 100.0 * B.SaturationThrottle, Sat.Limit)
                                      : Fmt("A motor was at or above %.0f %% throttle for %.1f s (%.1f %% of the flight, limit %.1f %%, ASSUMED), first at t = %.1f s, "
                                            "mostly during %s; wind then up to %.1f m/s.",
                                            100.0 * B.SaturationThrottle, Sat.SecondsAbove90, Sat.PeakValue, Sat.Limit, Sat.PeakTimeS, Sat.PeakPhase.c_str(), SatWind);
        }
        A.Findings.push_back(Sat);
        A.Findings.push_back(FromSeries("tracking", "Controller", "distance from the planned path (on legs)", "m", B.TrackingLimitM, "ASSUMED", L, Cross, UCross));
        A.Findings.push_back(FromSeries("tilt", "Controller", "tilt angle", "deg", B.MaxTiltRad * 180.0 / DroneElec::kPi, "ASSUMED", L, Tilt, UTilt));
    }

    // Frame and body: no design data, so no margin (never a made-up one).
    {
        Finding F;
        F.Id = "frame";
        F.Component = "Frame / body";
        F.Metric = "not assessed";
        F.Assessed = false;
        F.LimitStatus = "none";
        F.Cause = "No frame design data exists (only an ASSUMED plate size and mass): nothing to compare a load with.";
        A.Findings.push_back(F);
    }

    // A mission that did not complete is the first finding, margin 0.
    if (!Out.Completed)
    {
        Finding F;
        F.Id = "mission";
        F.Component = "Mission";
        F.Metric = "completion";
        F.Unit = "";
        F.LimitStatus = "controller";
        F.Limit = 1.0;
        F.PeakValue = 0.0;
        F.Margin = 0.0;
        F.PeakTimeS = Out.DurationS;
        F.Cause = Fmt("The mission did not complete: %s (after %.1f s, %d step errors).", Out.Reason.c_str(), Out.DurationS, Out.StepErrors);
        F.Consequence = ConsequenceFor("mission");
        A.Findings.push_back(F);
    }

    // Rank: assessed by margin ascending (stable), not-assessed last.
    std::stable_sort(A.Findings.begin(), A.Findings.end(), [](const Finding& X, const Finding& Y) {
        if (X.Assessed != Y.Assessed) return X.Assessed;
        return X.Margin < Y.Margin;
    });
    A.MinMargin = DroneElec::kNoLoadMargin;
    for (const Finding& F : A.Findings)
    {
        if (F.Assessed)
        {
            A.MinMargin = F.Margin;
            A.Limiting = F.Component + ": " + F.Metric;
            break;
        }
    }
    bool AnyAssessed = false;
    for (const Finding& F : A.Findings) AnyAssessed = AnyAssessed || F.Assessed;
    A.V = DroneElec::VerdictFromMargin(A.MinMargin);
    A.VerdictReason = Out.Completed ? Fmt("smallest margin %.2f (%s)", A.MinMargin, A.Limiting.c_str())
                                    : Fmt("mission not completed: %s", Out.Reason.c_str());
    if (!Out.Completed) A.V = Verdict::NotRecommended;
    if (!AnyAssessed)
    {
        A.V = Verdict::NotRecommended;
        A.VerdictReason = "no flight data to assess";
        A.Limiting = "none";
    }
    return A;
}
}  // namespace DroneReport
