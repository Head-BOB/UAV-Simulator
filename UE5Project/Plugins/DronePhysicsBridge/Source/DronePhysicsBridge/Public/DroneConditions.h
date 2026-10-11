// Tier: V
// Engine-free Conditions screen model: typed fields with units and ranges equal to the Rust `Conditions::validate`
// (physics_core/src/campaign.rs), nothing clamped (a bad entry is refused with a message), the test selection, and the editable
// limits of the test definitions. Writes the two files `run_campaign` reads. The Rust parser stays the authority: a CI step feeds the
// files written here to run_campaign.
#pragma once

#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <string>
#include <vector>

#include "DroneJson.h"

namespace DroneApp
{
// Limits copied from physics_core: campaign.rs (ranges) and payload.rs (MAX_PAYLOAD_KG, MAX_PAYLOAD_POSITION_M).
constexpr double kMaxPayloadKg = 50.0;
constexpr double kMaxPayloadPositionM = 2.0;

struct FieldSpec
{
    const char* Id;
    const char* Label;
    const char* Unit;
    double Lo;
    double Hi;
    bool HiExclusive;     // wind direction is 0..360 excluding 360
    const char* Default;  // text shown first
    bool Optional;        // an empty entry is allowed (battery temperature: default = ambient)
};

inline const std::vector<FieldSpec>& ConditionFields()
{
    static const std::vector<FieldSpec> F = {
        {"ambient_c", "Air temperature", "C", -40.0, 55.0, false, "20", false},
        {"site_altitude_m", "Site altitude", "m", 0.0, 4000.0, false, "0", false},
        {"wind_speed_mps", "Mean wind speed", "m/s", 0.0, 20.0, false, "0", false},
        {"wind_from_deg", "Wind from", "deg", 0.0, 360.0, true, "270", false},
        {"gust_level", "Gust level", "x mean wind", 0.0, 1.0, false, "0", false},
        {"payload_kg", "Payload mass", "kg", 0.0, kMaxPayloadKg, false, "0", false},
        {"payload_x_m", "Payload position x (forward)", "m", -kMaxPayloadPositionM, kMaxPayloadPositionM, false, "0", false},
        {"payload_y_m", "Payload position y (right)", "m", -kMaxPayloadPositionM, kMaxPayloadPositionM, false, "0", false},
        {"payload_z_m", "Payload position z (down)", "m", -kMaxPayloadPositionM, kMaxPayloadPositionM, false, "0", false},
        {"battery_soc", "Battery state of charge", "fraction", 0.0, 1.0, false, "1", false},
        {"battery_temp_c", "Battery temperature", "C", -40.0, 100.0, false, "", true},
    };
    return F;
}

// Strict number parsing: the whole text must be one finite decimal number (no hex, no nan / inf, no trailing text, no empty).
inline bool ParseStrictNumber(const std::string& Text, double& Out)
{
    if (Text.empty()) return false;
    for (const char C : Text)
        if (!((C >= '0' && C <= '9') || C == '.' || C == '-' || C == '+' || C == 'e' || C == 'E')) return false;
    char* End = nullptr;
    const double V = std::strtod(Text.c_str(), &End);
    if (End == Text.c_str() || *End != '\0' || !std::isfinite(V)) return false;
    Out = V;
    return true;
}

// The typed strings of the screen, by field id.
struct ConditionsText
{
    std::string Name = "custom";
    std::string Turbulence = "dryden";   // none | dryden | simple
    std::string Seed = "1";
    std::string Value[11];               // in ConditionFields() order

    ConditionsText()
    {
        for (std::size_t I = 0; I < ConditionFields().size(); ++I) Value[I] = ConditionFields()[I].Default;
    }
    std::string& At(const std::string& Id)
    {
        for (std::size_t I = 0; I < ConditionFields().size(); ++I)
            if (Id == ConditionFields()[I].Id) return Value[I];
        return Value[0];
    }
};

struct ConditionsValid
{
    std::string Name, Turbulence;
    std::uint64_t Seed = 1;
    double V[11] = {};
    bool HasBatteryTemp = false;
};

// Validates every field; on the first problem returns false with "<label>: <what is wrong>" (so the message can sit next to the field).
inline bool ValidateConditions(const ConditionsText& T, ConditionsValid& Out, std::string& Err)
{
    Out = ConditionsValid();
    if (T.Name.empty() || T.Name.size() > 40) { Err = "Name: 1 to 40 characters"; return false; }
    for (const char C : T.Name)
        if (!((C >= 'a' && C <= 'z') || (C >= '0' && C <= '9') || C == '_')) { Err = "Name: only a-z, 0-9 and _"; return false; }
    if (T.Turbulence != "none" && T.Turbulence != "dryden" && T.Turbulence != "simple") { Err = "Turbulence: none, dryden or simple"; return false; }
    Out.Name = T.Name;
    Out.Turbulence = T.Turbulence;
    if (T.Seed.empty() || T.Seed.size() > 18) { Err = "Seed: a whole number, 0 or more"; return false; }
    std::uint64_t Seed = 0;
    for (const char C : T.Seed)
    {
        if (C < '0' || C > '9') { Err = "Seed: a whole number, 0 or more"; return false; }
        Seed = Seed * 10 + static_cast<std::uint64_t>(C - '0');
    }
    Out.Seed = Seed;
    for (std::size_t I = 0; I < ConditionFields().size(); ++I)
    {
        const FieldSpec& F = ConditionFields()[I];
        const std::string Where = std::string(F.Label) + ": ";
        if (T.Value[I].empty() && F.Optional) { continue; }
        double V = 0.0;
        if (!ParseStrictNumber(T.Value[I], V)) { Err = Where + "enter a number"; return false; }
        const bool InRange = V >= F.Lo && (F.HiExclusive ? V < F.Hi : V <= F.Hi);
        if (!InRange)
        {
            char Buf[160];
            std::snprintf(Buf, sizeof Buf, "must be in %g to %g %s%s", F.Lo, F.Hi, F.Unit, F.HiExclusive ? " (upper end not included)" : "");
            Err = Where + Buf;
            return false;
        }
        Out.V[I] = V;
        if (std::string(F.Id) == "battery_temp_c") Out.HasBatteryTemp = true;
    }
    return true;
}

// conditions.json for run_campaign (schema_version 1, same keys as the Rust ConditionsJson).
inline std::string WriteConditionsJson(const ConditionsValid& C)
{
    auto N = [](double V) { return Json::NumberText(V); };
    // the order of ConditionFields(): 0 ambient, 1 site, 2 wind, 3 from, 4 gust, 5 payload kg, 6-8 payload xyz, 9 soc, 10 battery temp
    std::string S = "{\n  \"schema_version\": 1,\n  \"name\": " + Json::Quote(C.Name) + ",\n";
    S += "  \"ambient_c\": " + N(C.V[0]) + ",\n  \"site_altitude_m\": " + N(C.V[1]) + ",\n  \"wind_speed_mps\": " + N(C.V[2]) + ",\n";
    S += "  \"wind_from_deg\": " + N(C.V[3]) + ",\n  \"turbulence\": " + Json::Quote(C.Turbulence) + ",\n  \"gust_level\": " + N(C.V[4]) + ",\n";
    S += "  \"seed\": " + std::to_string(C.Seed) + ",\n  \"payload_kg\": " + N(C.V[5]) + ",\n";
    S += "  \"payload_position_m\": [" + N(C.V[6]) + ", " + N(C.V[7]) + ", " + N(C.V[8]) + "],\n  \"battery_soc\": " + N(C.V[9]) + ",\n";
    S += "  \"battery_temp_c\": " + (C.HasBatteryTemp ? N(C.V[10]) : std::string("null")) + "\n}\n";
    return S;
}

// ----------------------------------------------------------------------------------------------- tests and limits

struct TestInfo
{
    const char* Id;
    const char* Label;
    bool Implemented;
};
inline const std::vector<TestInfo>& TestList()
{
    static const std::vector<TestInfo> L = {{"hover_payload", "Hover with payload", true}, {"wind_gust", "Wind and gusts", true}, {"climb", "Climb", true},
                                            {"endurance", "Endurance", true}, {"thermal_soak", "Thermal soak", true}, {"max_speed", "Maximum speed", true},
                                            {"fpv_sprint", "FPV sprint", true}, {"fpv_punchout", "FPV punch-out", true}};
    return L;
}

// "--tests" argument from the selected ids (only implemented tests; order of TestList()). Empty when nothing is selected.
inline std::string TestsArgument(const std::vector<std::string>& Selected)
{
    std::string Out;
    for (const auto& T : TestList())
    {
        if (!T.Implemented) continue;
        for (const auto& S : Selected)
            if (S == T.Id) { Out += (Out.empty() ? "" : ",") + std::string(T.Id); break; }
    }
    return Out;
}

struct EditableLimit
{
    std::string Test, Key, Unit;
    double Original = 0.0;
    double Value = 0.0;
    bool Edited() const { return Value != Original; }
};

namespace ConditionsDetail
{
inline std::string UnitOfKey(const std::string& K)
{
    struct Suffix { const char* S; const char* U; };
    static const Suffix Table[] = {{"_k_per_min", "K/min"}, {"_mps2", "m/s^2"}, {"_mps", "m/s"}, {"_kg", "kg"}, {"_m", "m"}, {"_s", "s"},
                                   {"_ratio", "ratio"}, {"_fraction_limit", "fraction"}, {"_cap", "fraction"}};
    for (const auto& E : Table)
    {
        const std::string Sx = E.S;
        if (K.size() >= Sx.size() && K.compare(K.size() - Sx.size(), Sx.size(), Sx) == 0) return E.U;
    }
    return "";
}
}  // namespace ConditionsDetail

// All numeric fields of the chosen test's definition except output_interval_s (output rate, not a limit), in file order.
inline std::vector<EditableLimit> EditableLimits(const Json::Value& Definitions, const std::string& Test)
{
    std::vector<EditableLimit> Out;
    const Json::Value* D = Definitions.Get(Test);
    if (D == nullptr || !D->IsObject()) return Out;
    for (const auto& M : D->members)
    {
        if (!M.second.IsNumber() || M.first == "output_interval_s") continue;
        EditableLimit E;
        E.Test = Test; E.Key = M.first; E.Unit = ConditionsDetail::UnitOfKey(M.first); E.Original = M.second.number; E.Value = M.second.number;
        Out.push_back(E);
    }
    return Out;
}

// Applies edited values to a copy of the definitions text. Every value must be finite and not negative (the Rust parser applies the full
// per-test rules and will refuse the rest). If anything changed, definitions_version gets the suffix "-edited" (once). Returns the new text.
inline bool WriteEditedDefinitions(const std::string& OriginalText, const std::vector<EditableLimit>& Edits, std::string& OutText, bool& AnyEdited, std::string& Err)
{
    Json::ParseResult R = Json::Parse(OriginalText);
    if (!R.ok) { Err = R.error; return false; }
    AnyEdited = false;
    for (const auto& E : Edits)
    {
        if (!std::isfinite(E.Value) || E.Value < 0.0) { Err = E.Test + "." + E.Key + ": enter a number that is not negative"; return false; }
        for (auto& TM : R.value.members)
        {
            if (TM.first != E.Test || !TM.second.IsObject()) continue;
            for (auto& M : TM.second.members)
                if (M.first == E.Key && M.second.IsNumber() && M.second.number != E.Value) { M.second.number = E.Value; AnyEdited = true; }
        }
    }
    if (AnyEdited)
        for (auto& M : R.value.members)
            if (M.first == "definitions_version" && M.second.IsString())
            {
                const std::string Suffix = "-edited";
                const bool Has = M.second.text.size() >= Suffix.size() && M.second.text.compare(M.second.text.size() - Suffix.size(), Suffix.size(), Suffix) == 0;
                if (!Has) M.second.text += Suffix;
            }
    OutText = Json::Serialize(R.value);
    return true;
}

// ---- Quick profile ---------------------------------------------------------------------------------------------------------------
// Shortens the long tests for a demo. FIXED durations, listed here and nowhere else; every one is ASSUMED
// (TODO(HUMAN-VERIFY): choose by what a demo audience can wait for, not by engineering need). A value is only ever REDUCED: a limit the
// user already set lower stays. definitions_version gets the suffix "-quick" (after "-edited" if both apply), so every result file,
// the report banner ("Quick profile: shortened tests") and the Run screen say that the tests were shortened. Full is the default.
struct QuickOverride
{
    const char* Test;
    const char* Key;
    double Value;
    const char* Why;
};
inline const std::vector<QuickOverride>& QuickOverrides()
{
    static const std::vector<QuickOverride> V = {
        {"endurance", "max_duration_s", 300.0, "endurance stops after 5 min of flight instead of at the battery cutoff (up to 1 h)"},
        {"thermal_soak", "max_duration_s", 600.0, "thermal soak stops after 10 min instead of 2 h; the temperatures may not have settled"},
        {"hover_payload", "hold_s", 15.0, "each payload step is held 15 s instead of 30 s (the 10 s evaluation window is unchanged)"},
        {"wind_gust", "hold_s", 15.0, "each wind step is held 15 s instead of 30 s (the 10 s evaluation window is unchanged)"},
    };
    return V;
}

// Applies the Quick overrides to the definitions text (after the user's edits). Out = new text; Changed = any value was reduced.
inline bool ApplyQuickProfile(const std::string& DefinitionsText, std::string& OutText, std::string& Err)
{
    Json::ParseResult R = Json::Parse(DefinitionsText);
    if (!R.ok) { Err = R.error; return false; }
    for (const auto& Q : QuickOverrides())
        for (auto& TM : R.value.members)
        {
            if (TM.first != Q.Test || !TM.second.IsObject()) continue;
            for (auto& M : TM.second.members)
                if (M.first == Q.Key && M.second.IsNumber() && M.second.number > Q.Value) M.second.number = Q.Value;
        }
    for (auto& M : R.value.members)
        if (M.first == "definitions_version" && M.second.IsString())
        {
            const std::string Suffix = "-quick";
            const bool Has = M.second.text.size() >= Suffix.size() && M.second.text.compare(M.second.text.size() - Suffix.size(), Suffix.size(), Suffix) == 0;
            if (!Has) M.second.text += Suffix;
        }
    OutText = Json::Serialize(R.value);
    return true;
}
}  // namespace DroneApp
