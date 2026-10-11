// Tier: V
// Engine-free margins with the SAME definitions as tools/report/margins.py (the report), so the app and the report agree:
//   currents      margin = limit / peak            (zero peak = never loaded = cap)
//   temperatures  margin = (limit - ambient) / (peak - ambient), "no heating" below 0.1 K of rise (cap)
//   thrust reserve  margin = lowest max-thrust / weight ratio, limit 1.0
// A margin below 1.0 means the limit was exceeded. Limits come from powertrain_config.json (several are ASSUMED).
// Live margins use the instantaneous value instead of the run's peak. The caution threshold is an ASSUMED display rule.
#pragma once

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#include "DroneCampaignData.h"
#include "DroneReplay.h"

namespace DroneApp
{
constexpr double kMarginCap = 99.0;
constexpr double kNoHeatingRiseK = 0.1;
constexpr double kCautionMargin = 1.5;   // amber below this. ASSUMED display threshold (same value the demo used)
constexpr double kCriticalMargin = 1.0;  // red below this: the limit is exceeded

enum class Status { Ok, Caution, Critical };
inline Status StatusOfMargin(double Margin)
{
    return Margin < kCriticalMargin ? Status::Critical : (Margin < kCautionMargin ? Status::Caution : Status::Ok);
}

struct MarginRow
{
    std::string Slot;
    std::string Label;
    double Limit = 0.0;
    double Peak = 0.0;
    std::string Unit;
    double Margin = 0.0;
    bool NoHeating = false;
    std::string PeakTest;
    double PeakTimeS = 0.0;
};

namespace MarginsDetail
{
struct SlotDef
{
    const char* Slot;
    const char* Label;
    const char* PeakName;
    const char* LimitKey;
    bool IsCurrent;
};
inline const std::vector<SlotDef>& Slots()
{
    static const std::vector<SlotDef> S = {
        {"motor_current", "Motor current", "motor_current_a", "motor_max_current_a", true},
        {"esc_current", "ESC current", "motor_current_a", "esc_max_current_a", true},
        {"pack_current", "Battery pack current", "pack_current_a", "battery_max_continuous_a", true},
        {"winding_temp", "Motor winding temperature", "winding_c", "motor_winding_temp_limit_c", false},
        {"esc_temp", "ESC temperature", "esc_c", "esc_temp_limit_c", false},
        {"battery_temp", "Battery temperature", "battery_c", "battery_temp_limit_c", false},
    };
    return S;
}
}  // namespace MarginsDetail

inline double CurrentMargin(double Limit, double Peak)
{
    if (Peak <= 0.0) return kMarginCap;
    return std::min(kMarginCap, Limit / Peak);
}
inline double RiseMargin(double Limit, double Peak, double Ambient, bool& NoHeating)
{
    const double Rise = Peak - Ambient;
    NoHeating = Rise < kNoHeatingRiseK;
    if (NoHeating) return kMarginCap;
    return std::min(kMarginCap, (Limit - Ambient) / Rise);
}

// Peak margins of a whole campaign, ascending (the first row is the limiting component); ties by slot id. Returns false and Err when a
// recorded peak or a limit is missing, or a temperature limit is not above the ambient temperature.
inline bool ComputeMargins(const Campaign& C, const Json::Value& Powertrain, std::vector<MarginRow>& Out, std::string& Err)
{
    Out.clear();
    const double Ambient = C.Ambient();
    auto Extreme = [&](const char* Name, bool Max, Peak& P, std::string& Test) {
        bool Have = false;
        for (const TestData& T : C.Tests)
        {
            auto It = T.Peaks.find(Name);
            if (It == T.Peaks.end()) continue;
            if (!Have || (Max ? It->second.Value > P.Value : It->second.Value < P.Value)) { P = It->second; Test = T.Name; Have = true; }
        }
        return Have;
    };
    for (const auto& D : MarginsDetail::Slots())
    {
        Peak P;
        std::string Test;
        if (!Extreme(D.PeakName, true, P, Test)) { Err = std::string("no recorded peak ") + D.PeakName + " in any test"; return false; }
        const Json::Value* L = Powertrain.Get(D.LimitKey);
        if (L == nullptr || !L->IsNumber()) { Err = std::string("powertrain config has no ") + D.LimitKey; return false; }
        MarginRow R;
        R.Slot = D.Slot; R.Label = D.Label; R.Limit = L->number; R.Peak = P.Value; R.PeakTest = Test; R.PeakTimeS = P.TimeS;
        if (D.IsCurrent)
        {
            if (R.Limit <= 0.0) { Err = "current limit must be positive"; return false; }
            R.Unit = "A";
            R.Margin = CurrentMargin(R.Limit, R.Peak);
        }
        else
        {
            if (R.Limit <= Ambient) { Err = "temperature limit must be above the ambient temperature"; return false; }
            R.Unit = "deg C";
            R.Margin = RiseMargin(R.Limit, R.Peak, Ambient, R.NoHeating);
        }
        Out.push_back(R);
    }
    Peak P;
    std::string Test;
    if (!Extreme("min_thrust_reserve_ratio", false, P, Test)) { Err = "no recorded peak min_thrust_reserve_ratio in any test"; return false; }
    MarginRow R;
    R.Slot = "thrust_reserve"; R.Label = "Thrust reserve (max thrust / weight)"; R.Limit = 1.0; R.Peak = P.Value; R.Unit = "ratio"; R.Margin = P.Value;
    R.PeakTest = Test; R.PeakTimeS = P.TimeS;
    Out.push_back(R);
    std::sort(Out.begin(), Out.end(), [](const MarginRow& A, const MarginRow& B) { return A.Margin != B.Margin ? A.Margin < B.Margin : A.Slot < B.Slot; });
    return true;
}

// Margins at one instant of the replay, same slots and definitions, using the instantaneous worst value.
inline std::vector<MarginRow> LiveMargins(const Pose& P, const Json::Value& Powertrain, double Ambient)
{
    std::vector<MarginRow> Out;
    const double MotorA = std::max(std::max(P.CurrentA[0], P.CurrentA[1]), std::max(P.CurrentA[2], P.CurrentA[3]));
    const double Values[6] = {MotorA, MotorA, P.BatteryA, P.WindingC, P.EscC, P.BatteryC};
    std::size_t I = 0;
    for (const auto& D : MarginsDetail::Slots())
    {
        MarginRow R;
        R.Slot = D.Slot; R.Label = D.Label; R.Peak = Values[I++];
        const Json::Value* L = Powertrain.Get(D.LimitKey);
        R.Limit = (L && L->IsNumber()) ? L->number : 0.0;
        if (R.Limit <= 0.0 || (!D.IsCurrent && R.Limit <= Ambient)) { R.Margin = kMarginCap; R.Unit = "n/a"; Out.push_back(R); continue; }   // limit missing: shown as n/a by the caller
        R.Unit = D.IsCurrent ? "A" : "deg C";
        R.Margin = D.IsCurrent ? CurrentMargin(R.Limit, R.Peak) : RiseMargin(R.Limit, R.Peak, Ambient, R.NoHeating);
        Out.push_back(R);
    }
    MarginRow R;
    R.Slot = "thrust_reserve"; R.Label = "Thrust reserve (max thrust / weight)"; R.Limit = 1.0; R.Peak = P.ReserveRatio; R.Unit = "ratio"; R.Margin = P.ReserveRatio;
    Out.push_back(R);
    return Out;
}

// The smallest margin of a set (index into it), or -1 for an empty set.
inline int LimitingIndex(const std::vector<MarginRow>& Rows)
{
    int Best = -1;
    for (std::size_t I = 0; I < Rows.size(); ++I)
        if (Best < 0 || Rows[I].Margin < Rows[static_cast<std::size_t>(Best)].Margin) Best = static_cast<int>(I);
    return Best;
}
}  // namespace DroneApp
