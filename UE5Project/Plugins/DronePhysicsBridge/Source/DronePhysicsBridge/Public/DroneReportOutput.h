// Tier: V
// DEMO ONLY: improvement suggestions (each tested by re-flying the mission with the changed parameter),
// the cross-environment table, and the JSON / CSV / HTML text of the report. UE-free (standard library
// only). The caller writes the strings to files (tools: std::ofstream; Unreal: FFileHelper). No clock is
// read here: the caller passes the time text, so the same inputs give byte-identical output.
#pragma once

#include "DroneReport.h"

#include <cmath>
#include <string>
#include <vector>

namespace DroneReport
{
// ---------------------------------------------------------------------------------------------
// Suggestions
// ---------------------------------------------------------------------------------------------

struct Variant
{
    const char* Id;
    const char* Text;
    double CruiseMps;      // 0 = unchanged
    double MassDeltaKg;    // added to the vehicle mass (and inertia scaled with it by the runner)
    double BatteryAh;      // 0 = unchanged
    double ArmWallM;       // 0 = unchanged
    bool Reflight;         // true: fly the mission again; false: only re-evaluate the same log
};

// The parameter changes this demo can really re-fly (every one of them is a number in the catalog or the mission).
// TODO(HUMAN-VERIFY: demo value): the sizes of the changes are choices.
constexpr Variant kVariants[] = {
    {"slow", "Lower the cruise speed to 3 m/s", 3.0, 0.0, 0.0, 0.0, true},
    {"light", "Take 150 g off the airframe (lighter frame or payload)", 0.0, -0.15, 0.0, 0.0, true},
    {"battery6", "Fit a 6000 mAh pack (+90 g)", 0.0, 0.09, 6.0, 0.0, true},
    {"arm2mm", "Use a 2.0 mm wall arm tube (same outer diameter)", 0.0, 0.0, 0.0, 0.002, false},
};
constexpr std::size_t kVariantCount = sizeof(kVariants) / sizeof(kVariants[0]);

inline const Variant* FindVariant(const std::string& Id)
{
    for (const Variant& V : kVariants)
        if (Id == V.Id) return &V;
    return nullptr;
}

struct Rule
{
    const char* FindingId;
    const char* Variants[3];   // ids of tested variants, nullptr-terminated
    const char* Untested[2];   // suggestions with no data behind them, nullptr-terminated
};

// What could help which finding. A variant is listed only as a candidate: the re-flight decides whether it helps.
constexpr Rule kRules[] = {
    {"motor_current", {"slow", "light", nullptr}, {"A motor with a higher continuous current rating", nullptr}},
    {"motor_power", {"slow", "light", nullptr}, {"A motor with a higher power rating", nullptr}},
    {"prop_thrust", {"slow", "light", nullptr}, {"A larger-diameter propeller (needs that propeller's data)", "A higher-Kv motor (needs that motor's data)"}},
    {"esc_current", {"slow", "light", nullptr}, {"An ESC with a higher current rating", nullptr}},
    {"battery_current", {"battery6", "light", "slow"}, {"A pack with a higher discharge rating", nullptr}},
    {"battery_voltage", {"battery6", nullptr, nullptr}, {"A pack with lower internal resistance", nullptr}},
    {"battery_capacity", {"battery6", "light", "slow"}, {nullptr, nullptr}},
    {"motor_temp", {"slow", "light", nullptr}, {"Better motor cooling (not modelled)", nullptr}},
    {"arm_bending", {"arm2mm", "slow", "light"}, {nullptr, nullptr}},
    {"saturation", {"light", "slow", nullptr}, {"Propellers or motors with more thrust (needs their data)", nullptr}},
    {"tracking", {"slow", nullptr, nullptr}, {nullptr, nullptr}},
    {"tilt", {"slow", nullptr, nullptr}, {nullptr, nullptr}},
};

// Re-flies (or re-evaluates) the mission with one change. Returns false if it could not run.
using ReflightFn = std::function<bool(const Variant&, Analysis*)>;

inline const Finding* FindById(const Analysis& A, const std::string& Id)
{
    for (const Finding& F : A.Findings)
        if (F.Id == Id && F.Assessed) return &F;
    return nullptr;
}

// Adds suggestions to every assessed finding with margin below BelowMargin (at most MaxFindings of them,
// weakest first). Each variant is run once and its result reused. "Improves" needs the re-flown mission to
// complete AND the margin of the same kind of finding to rise: a change that does not help is still
// listed, marked as tried.
inline void AttachSuggestions(Analysis& A, const ReflightFn& Run, double BelowMargin = 3.0, int MaxFindings = 6)
{
    std::vector<std::pair<std::string, Analysis>> Cache;
    std::vector<std::string> Failed;
    auto Get = [&](const Variant& V) -> const Analysis* {
        for (const auto& C : Cache)
            if (C.first == V.Id) return &C.second;
        for (const std::string& F : Failed)
            if (F == V.Id) return nullptr;
        Analysis R;
        if (!Run(V, &R))
        {
            Failed.push_back(V.Id);
            return nullptr;
        }
        Cache.emplace_back(V.Id, R);
        return &Cache.back().second;
    };
    int Done = 0;
    for (Finding& F : A.Findings)
    {
        if (!F.Assessed || F.Margin >= BelowMargin || Done >= MaxFindings) continue;
        for (const Rule& R : kRules)
        {
            if (F.Id != R.FindingId) continue;
            for (const char* Id : R.Variants)
            {
                if (!Id) continue;
                const Variant* V = FindVariant(Id);
                if (!V) continue;
                Suggestion S;
                S.Text = V->Text;
                S.MarginBefore = F.Margin;
                const Analysis* After = Get(*V);
                const Finding* Match = After ? FindById(*After, F.Id) : nullptr;
                if (!After || !Match)
                {
                    S.Tested = false;
                    S.Evidence = "NOT TESTED: the re-flight could not be run";
                }
                else
                {
                    S.Tested = true;
                    S.MarginAfter = Match->Margin;
                    S.Improves = After->Completed && Match->Margin > F.Margin * (1.0 + 1e-6);
                    S.Evidence = Fmt("Re-flown with the same seed: margin %.2f -> %.2f%s.", F.Margin, Match->Margin,
                                     After->Completed ? "" : " (mission NOT completed)");
                    if (!V->Reflight) S.Evidence += " Same flight log, new arm tube.";
                }
                F.Suggestions.push_back(S);
            }
            for (const char* Text : R.Untested)
            {
                if (!Text) continue;
                Suggestion S;
                S.Text = Text;
                S.Tested = false;
                S.Evidence = "NOT TESTED: the component catalog has no data for that part.";
                F.Suggestions.push_back(S);
            }
        }
        ++Done;
    }
}

// ---------------------------------------------------------------------------------------------
// Bundle and cross-environment table
// ---------------------------------------------------------------------------------------------

struct CrossRow
{
    EnvInfo Env;
    bool Completed = false;
    double MinMargin = 0.0;
    std::string Limiting;
    Verdict V = Verdict::NotRecommended;
    std::string Reason;
    double DensityKgM3 = 0.0;
};

struct Bundle
{
    std::string GeneratedUtc;     // supplied by the caller; empty prints "not recorded"
    unsigned long long Seed = 0;
    EnvInfo Flown;
    double DensityKgM3 = 0.0;
    MissionOutcome Outcome;
    Analysis A;
    std::vector<CrossRow> Cross;
};

constexpr const char* kStatusBanner =
    "PRELIMINARY - ASSUMED INPUTS. A first-order engineering estimate from a demo simulator, not a validated analysis. "
    "Limits marked ASSUMED are placeholders, CFD/FEA-derived numbers are UNVALIDATED, nothing was checked against hardware.";

inline const char* const kLimitations[] = {
    "Limits and thresholds marked ASSUMED are placeholders (see the assumptions table); only the T-Motor motor numbers are CITED.",
    "Propeller coefficients are the constant static fit of the APC 12x6E data: forward-flight thrust loss and the data range above 6044 RPM are not modelled.",
    "Motor, ESC and battery electrics are textbook first-order models with a flat battery voltage and constant no-load current.",
    "The motor temperature comes from the core's thermal model, which is an UNVALIDATED placeholder (thermal_valid = 0).",
    "Humidity is not modelled. A surrogate aero model, if loaded, ignores the air density; otherwise drag is the core's analytic model.",
    "The wind and gust generator is a demo model, not validated turbulence. The mission uses the DEMO controller: margins describe that controller.",
    "Suggestions marked TESTED were re-flown once with the same seed and may not generalise; NOT TESTED ones have no data behind them.",
    "Frame and body are NOT ASSESSED: there is no frame design data.",
};

// ---------------------------------------------------------------------------------------------
// Text writers
// ---------------------------------------------------------------------------------------------

inline std::string EscapeJson(const std::string& S)
{
    std::string O;
    for (const char C : S)
    {
        switch (C)
        {
            case '"': O += "\\\""; break;
            case '\\': O += "\\\\"; break;
            case '\n': O += "\\n"; break;
            case '\r': O += "\\r"; break;
            case '\t': O += "\\t"; break;
            default:
                if (static_cast<unsigned char>(C) < 0x20) O += Fmt("\\u%04x", static_cast<unsigned>(static_cast<unsigned char>(C)));
                else O += C;
        }
    }
    return O;
}

inline std::string EscapeHtml(const std::string& S)
{
    std::string O;
    for (const char C : S)
    {
        switch (C)
        {
            case '&': O += "&amp;"; break;
            case '<': O += "&lt;"; break;
            case '>': O += "&gt;"; break;
            case '"': O += "&quot;"; break;
            case '\'': O += "&#39;"; break;
            default: O += C;
        }
    }
    return O;
}

// A CSV field: quoted when it holds a comma, quote or newline; quotes doubled.
inline std::string CsvField(const std::string& S)
{
    if (S.find_first_of(",\"\n\r") == std::string::npos) return S;
    std::string O = "\"";
    for (const char C : S)
    {
        if (C == '"') O += "\"\"";
        else O += C;
    }
    return O + "\"";
}

// JSON number text: 9 significant digits; NaN and Inf become null (JSON has no such numbers).
inline std::string JsonNum(double V)
{
    return std::isfinite(V) ? Fmt("%.9g", V) : std::string("null");
}

inline std::string JsonStr(const std::string& S)
{
    return "\"" + EscapeJson(S) + "\"";
}

inline std::string EnvJson(const EnvInfo& E, double Density)
{
    return "{\"id\":" + JsonStr(E.Id) + ",\"name\":" + JsonStr(E.Name) + ",\"site_altitude_m\":" + JsonNum(E.SiteAltitudeM) +
           ",\"ambient_temp_c\":" + JsonNum(E.AmbientTempC) + ",\"temp_offset_from_isa_k\":" + JsonNum(E.TempOffsetC) + ",\"humidity_pct\":" +
           JsonNum(E.HumidityPct) + ",\"humidity_modelled\":false,\"wind_speed_mps\":" + JsonNum(E.WindSpeedMps) + ",\"wind_toward_deg\":" +
           JsonNum(E.WindTowardDeg) + ",\"gust_sigma_mps\":" + JsonNum(E.GustSigmaMps) + ",\"air_density_kg_m3\":" + JsonNum(Density) + "}";
}

inline std::string ToJson(const Bundle& B)
{
    std::string J = "{\n";
    J += "  \"report\": \"UAV digital twin demo flight report\",\n";
    J += "  \"status\": " + JsonStr(kStatusBanner) + ",\n";
    J += "  \"generated_utc\": " + JsonStr(B.GeneratedUtc.empty() ? "not recorded" : B.GeneratedUtc) + ",\n";
    J += "  \"seed\": " + Fmt("%llu", B.Seed) + ",\n";
    J += "  \"environment\": " + EnvJson(B.Flown, B.DensityKgM3) + ",\n";
    J += "  \"mission\": {\"completed\":" + std::string(B.Outcome.Completed ? "true" : "false") + ",\"reason\":" + JsonStr(B.Outcome.Reason) + ",\"duration_s\":" +
         JsonNum(B.Outcome.DurationS) + ",\"step_errors\":" + Fmt("%d", B.Outcome.StepErrors) + "},\n";
    J += "  \"verdict\": {\"text\":" + JsonStr(DroneElec::VerdictText(B.A.V)) + ",\"min_margin\":" + JsonNum(B.A.MinMargin) + ",\"limiting\":" +
         JsonStr(B.A.Limiting) + ",\"reason\":" + JsonStr(B.A.VerdictReason) + "},\n";
    J += "  \"findings\": [\n";
    for (std::size_t I = 0; I < B.A.Findings.size(); ++I)
    {
        const Finding& F = B.A.Findings[I];
        J += "    {\"rank\":" + Fmt("%d", static_cast<int>(I) + 1) + ",\"id\":" + JsonStr(F.Id) + ",\"component\":" + JsonStr(F.Component) + ",\"metric\":" + JsonStr(F.Metric) +
             ",\"unit\":" + JsonStr(F.Unit) + ",\"assessed\":" + std::string(F.Assessed ? "true" : "false") + ",\"limit\":" + JsonNum(F.Limit) +
             ",\"limit_status\":" + JsonStr(F.LimitStatus) + ",\"peak\":" + JsonNum(F.PeakValue) + ",\"margin\":" + (F.Assessed ? JsonNum(F.Margin) : std::string("null")) +
             ",\"peak_time_s\":" + JsonNum(F.PeakTimeS) + ",\"seconds_at_or_above_90_percent\":" + JsonNum(F.SecondsAbove90) + ",\"phase\":" + JsonStr(F.PeakPhase) +
             ",\"cause\":" + JsonStr(F.Cause) + ",\"consequence\":" + JsonStr(F.Consequence) + ",\"suggestions\":[";
        for (std::size_t K = 0; K < F.Suggestions.size(); ++K)
        {
            const Suggestion& S = F.Suggestions[K];
            J += std::string(K ? "," : "") + "{\"text\":" + JsonStr(S.Text) + ",\"tested\":" + (S.Tested ? "true" : "false") + ",\"improves\":" + (S.Improves ? "true" : "false") +
                 ",\"margin_before\":" + (S.Tested ? JsonNum(S.MarginBefore) : std::string("null")) + ",\"margin_after\":" + (S.Tested ? JsonNum(S.MarginAfter) : std::string("null")) +
                 ",\"evidence\":" + JsonStr(S.Evidence) + "}";
        }
        J += std::string("]}") + (I + 1 < B.A.Findings.size() ? "," : "") + "\n";
    }
    J += "  ],\n  \"cross_environment\": [\n";
    for (std::size_t I = 0; I < B.Cross.size(); ++I)
    {
        const CrossRow& R = B.Cross[I];
        J += "    {\"environment\":" + EnvJson(R.Env, R.DensityKgM3) + ",\"completed\":" + (R.Completed ? "true" : "false") + ",\"min_margin\":" + JsonNum(R.MinMargin) +
             ",\"limiting\":" + JsonStr(R.Limiting) + ",\"verdict\":" + JsonStr(DroneElec::VerdictText(R.V)) + ",\"reason\":" + JsonStr(R.Reason) + "}" +
             (I + 1 < B.Cross.size() ? "," : "") + "\n";
    }
    J += "  ],\n  \"assumptions\": [\n";
    for (std::size_t I = 0; I < DroneRef::kItemCount; ++I)
    {
        const DroneRef::Item& It = DroneRef::kItems[I];
        J += "    {\"name\":" + JsonStr(It.Name) + ",\"value\":" + JsonStr(It.Value) + ",\"unit\":" + JsonStr(It.Unit) + ",\"status\":" + JsonStr(It.Status) +
             ",\"source\":" + JsonStr(It.Source) + "}" + (I + 1 < DroneRef::kItemCount ? "," : "") + "\n";
    }
    J += "  ],\n  \"limitations\": [\n";
    const std::size_t NL = sizeof(kLimitations) / sizeof(kLimitations[0]);
    for (std::size_t I = 0; I < NL; ++I) J += "    " + JsonStr(kLimitations[I]) + (I + 1 < NL ? "," : "") + "\n";
    J += "  ]\n}\n";
    return J;
}

inline std::string ToCsvFindings(const Bundle& B)
{
    std::string C = "# " + std::string(kStatusBanner) + "\n";
    C += "environment,rank,component,metric,unit,limit,limit_status,peak,margin,peak_time_s,seconds_at_or_above_90_percent,phase,assessed\n";
    for (std::size_t I = 0; I < B.A.Findings.size(); ++I)
    {
        const Finding& F = B.A.Findings[I];
        C += CsvField(B.Flown.Id) + "," + Fmt("%d", static_cast<int>(I) + 1) + "," + CsvField(F.Component) + "," + CsvField(F.Metric) + "," + CsvField(F.Unit) + "," +
             (F.Assessed ? Fmt("%.9g", F.Limit) : "") + "," + CsvField(F.LimitStatus) + "," + (F.Assessed ? Fmt("%.9g", F.PeakValue) : "") + "," +
             (F.Assessed ? Fmt("%.9g", F.Margin) : "") + "," + Fmt("%.9g", F.PeakTimeS) + "," + Fmt("%.9g", F.SecondsAbove90) + "," + CsvField(F.PeakPhase) + "," +
             (F.Assessed ? "yes" : "no") + "\n";
    }
    return C;
}

inline std::string ToCsvEnvironments(const Bundle& B)
{
    std::string C = "# " + std::string(kStatusBanner) + "\n";
    C += "environment,site_altitude_m,ambient_temp_c,wind_speed_mps,gust_sigma_mps,air_density_kg_m3,completed,min_margin,limiting_component,verdict,reason\n";
    for (const CrossRow& R : B.Cross)
    {
        C += CsvField(R.Env.Id) + "," + Fmt("%.9g", R.Env.SiteAltitudeM) + "," + Fmt("%.9g", R.Env.AmbientTempC) + "," + Fmt("%.9g", R.Env.WindSpeedMps) + "," +
             Fmt("%.9g", R.Env.GustSigmaMps) + "," + Fmt("%.9g", R.DensityKgM3) + "," + (R.Completed ? "yes" : "no") + "," + Fmt("%.9g", R.MinMargin) + "," +
             CsvField(R.Limiting) + "," + CsvField(DroneElec::VerdictText(R.V)) + "," + CsvField(R.Reason) + "\n";
    }
    return C;
}

inline const char* VerdictClass(Verdict V)
{
    return V == Verdict::WithinLimits ? "ok" : (V == Verdict::Marginal ? "warn" : "bad");
}

inline std::string ToHtml(const Bundle& B)
{
    std::string H =
        "<!doctype html>\n<html lang=\"en\"><head><meta charset=\"utf-8\"><meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">\n"
        "<title>Demo Flight Report</title>\n<style>\n"
        ":root{--bg:#0b0e13;--panel:#121722;--line:#222b3a;--text:#e8edf5;--muted:#8793a6;--accent:#38e1ff;--warn:#ffb020;--bad:#ff4d5e;--ok:#38e1ff}\n"
        "@media (prefers-color-scheme: light){:root{--bg:#f4f6fa;--panel:#fff;--line:#d5dbe6;--text:#141a26;--muted:#4b5668;--accent:#006c85;--warn:#8a5a00;--bad:#b3202f;--ok:#006c85}}\n"
        "body{margin:0;background:var(--bg);color:var(--text);font:14px/1.5 Roboto,Segoe UI,Arial,sans-serif}\n"
        "main{max-width:1100px;margin:0 auto;padding:24px 16px 48px}\n"
        "h1{font-size:24px;margin:0 0 4px}h2{font-size:16px;letter-spacing:.06em;text-transform:uppercase;color:var(--muted);margin:32px 0 8px}\n"
        ".banner{border:1px solid var(--warn);border-radius:6px;padding:10px 12px;color:var(--warn);margin:12px 0}\n"
        ".panel{background:var(--panel);border:1px solid var(--line);border-radius:6px;padding:12px 16px}\n"
        "table{border-collapse:collapse;width:100%}th,td{border-bottom:1px solid var(--line);padding:6px 8px;text-align:left;vertical-align:top}\n"
        "th{color:var(--muted);font-weight:500;font-size:12px;letter-spacing:.05em;text-transform:uppercase}\n"
        ".ok{color:var(--ok)}.warn{color:var(--warn)}.bad{color:var(--bad)}.muted{color:var(--muted)}.num{font-variant-numeric:tabular-nums;white-space:nowrap}\n"
        ".tag{font-size:11px;border:1px solid var(--line);border-radius:4px;padding:0 5px;margin-left:4px;color:var(--muted)}\n"
        "ul{margin:4px 0 0 18px;padding:0}\n</style></head><body><main>\n";
    H += "<h1>Demo flight report</h1>\n<div class=\"muted\">Generated: " + EscapeHtml(B.GeneratedUtc.empty() ? "not recorded" : B.GeneratedUtc) + " &middot; seed " +
         Fmt("%llu", B.Seed) + " &middot; environment: " + EscapeHtml(B.Flown.Name) + "</div>\n";
    H += "<div class=\"banner\">" + EscapeHtml(kStatusBanner) + "</div>\n";
    H += "<h2>Verdict</h2><div class=\"panel\"><span class=\"" + std::string(VerdictClass(B.A.V)) + "\" style=\"font-size:20px\">" + EscapeHtml(DroneElec::VerdictText(B.A.V)) +
         "</span><br>" + EscapeHtml(B.A.VerdictReason) + "<br><span class=\"muted\">Mission: " + (B.Outcome.Completed ? "completed" : "NOT completed") + ", " +
         Fmt("%.1f", B.Outcome.DurationS) + " s, " + Fmt("%d", B.Outcome.StepErrors) + " step errors. Air density " + Fmt("%.4f", B.DensityKgM3) + " kg/m&sup3;; site " +
         Fmt("%.0f", B.Flown.SiteAltitudeM) + " m, " + Fmt("%.1f", B.Flown.AmbientTempC) + " &deg;C; wind " + Fmt("%.1f", B.Flown.WindSpeedMps) + " m/s, gust &sigma; " +
         Fmt("%.1f", B.Flown.GustSigmaMps) + " m/s; humidity " + Fmt("%.0f", B.Flown.HumidityPct) + " % (shown, not modelled).</span></div>\n";
    H += "<h2>Components, weakest first</h2>\n<div class=\"panel\"><table><tr><th>#</th><th>Component</th><th>Metric</th><th class=\"num\">Peak</th><th class=\"num\">Limit</th>"
         "<th class=\"num\">Margin</th><th>Cause, consequence, suggestions</th></tr>\n";
    for (std::size_t I = 0; I < B.A.Findings.size(); ++I)
    {
        const Finding& F = B.A.Findings[I];
        const char* Cls = !F.Assessed ? "muted" : (F.Margin < DroneElec::kMarginalMargin ? "bad" : (F.Margin < DroneElec::kWithinLimitsMargin ? "warn" : "ok"));
        H += "<tr><td>" + Fmt("%d", static_cast<int>(I) + 1) + "</td><td>" + EscapeHtml(F.Component) + "</td><td>" + EscapeHtml(F.Metric) + "</td>";
        if (F.Assessed)
        {
            H += "<td class=\"num\">" + Fmt("%.4g", F.PeakValue) + " " + EscapeHtml(F.Unit) + "</td><td class=\"num\">" + Fmt("%.4g", F.Limit) + " " + EscapeHtml(F.Unit) +
                 "<span class=\"tag\">" + EscapeHtml(F.LimitStatus) + "</span></td><td class=\"num " + Cls + "\">" + Fmt("%.2f", F.Margin) + "</td>";
        }
        else
        {
            H += "<td class=\"num\">&ndash;</td><td class=\"num\">&ndash;</td><td class=\"num muted\">NOT ASSESSED</td>";
        }
        H += "<td>" + EscapeHtml(F.Cause);
        if (!F.Consequence.empty()) H += "<br><span class=\"muted\">Consequence: " + EscapeHtml(F.Consequence) + "</span>";
        if (!F.Suggestions.empty())
        {
            H += "<ul>";
            for (const Suggestion& S : F.Suggestions)
            {
                H += "<li>" + EscapeHtml(S.Text) + " <span class=\"tag\">" + (S.Tested ? (S.Improves ? "TESTED: helps" : "TESTED: no help") : "NOT TESTED") + "</span> <span class=\"muted\">" +
                     EscapeHtml(S.Evidence) + "</span></li>";
            }
            H += "</ul>";
        }
        H += "</td></tr>\n";
    }
    H += "</table></div>\n";
    H += "<h2>All environments</h2>\n<div class=\"panel\"><table><tr><th>Environment</th><th class=\"num\">Altitude</th><th class=\"num\">Temp</th><th class=\"num\">Wind / gust</th>"
         "<th class=\"num\">Density</th><th class=\"num\">Min margin</th><th>Limiting</th><th>Verdict</th></tr>\n";
    for (const CrossRow& R : B.Cross)
    {
        H += "<tr><td>" + EscapeHtml(R.Env.Name) + "</td><td class=\"num\">" + Fmt("%.0f", R.Env.SiteAltitudeM) + " m</td><td class=\"num\">" + Fmt("%.1f", R.Env.AmbientTempC) +
             " &deg;C</td><td class=\"num\">" + Fmt("%.1f / %.1f", R.Env.WindSpeedMps, R.Env.GustSigmaMps) + " m/s</td><td class=\"num\">" + Fmt("%.3f", R.DensityKgM3) +
             "</td><td class=\"num\">" + Fmt("%.2f", R.MinMargin) + "</td><td>" + EscapeHtml(R.Limiting) + "</td><td class=\"" + VerdictClass(R.V) + "\">" +
             EscapeHtml(DroneElec::VerdictText(R.V)) + (R.Completed ? "" : " (mission not completed)") + "</td></tr>\n";
    }
    H += "</table></div>\n<h2>Limitations</h2>\n<div class=\"panel\"><ul>";
    for (const char* L : kLimitations) H += std::string("<li>") + EscapeHtml(L) + "</li>";
    H += "</ul></div>\n<h2>What is real and what is assumed</h2>\n<div class=\"panel\"><table><tr><th>Item</th><th>Value</th><th>Status</th><th>Source</th></tr>\n";
    for (std::size_t I = 0; I < DroneRef::kItemCount; ++I)
    {
        const DroneRef::Item& It = DroneRef::kItems[I];
        const bool Cited = std::string(It.Status) == "CITED";
        H += "<tr><td>" + EscapeHtml(It.Name) + "</td><td class=\"num\">" + EscapeHtml(It.Value) + " " + EscapeHtml(It.Unit) + "</td><td class=\"" + (Cited ? "ok" : "warn") + "\">" +
             EscapeHtml(It.Status) + "</td><td class=\"muted\">" + EscapeHtml(It.Source) + "</td></tr>\n";
    }
    H += "</table><p class=\"muted\">Propeller data: " + EscapeHtml(DroneRef::kPropCitation) + "</p></div>\n</main></body></html>\n";
    return H;
}
}  // namespace DroneReport
