// Tier: V
// Engine-free model of one campaign result folder written by physics_core `run_campaign` (the same checks as
// tools/report/campaign_data.py). File access is injected (a ReadFile callback), so the Unreal layer reads through the engine
// and the g++ test reads through std::ifstream. Never throws; every refusal is a message.
#pragma once

#include <cmath>
#include <cstddef>
#include <functional>
#include <map>
#include <string>
#include <utility>
#include <vector>

#include "DroneJson.h"

namespace DroneApp
{
// One time series: column names and equal-length columns of doubles.
struct Series
{
    std::vector<std::string> Names;
    std::vector<std::vector<double>> Cols;

    std::size_t Rows() const { return Cols.empty() ? 0 : Cols[0].size(); }
    int Index(const std::string& Name) const
    {
        for (std::size_t I = 0; I < Names.size(); ++I)
            if (Names[I] == Name) return static_cast<int>(I);
        return -1;
    }
    const std::vector<double>* Col(const std::string& Name) const
    {
        const int I = Index(Name);
        return I < 0 ? nullptr : &Cols[static_cast<std::size_t>(I)];
    }
};

struct Peak
{
    double Value = 0.0;
    double TimeS = 0.0;
};

struct EventRow
{
    std::string Name;
    bool Occurred = false;
    double FirstS = 0.0;
    double LastS = 0.0;
    double Activations = 0.0;
    double ActiveSteps = 0.0;
};

struct TestData
{
    std::string Name;
    Json::Value Manifest;
    Json::Value Result;
    std::vector<EventRow> Events;
    std::map<std::string, Peak> Peaks;
    std::vector<std::pair<std::string, Series>> SeriesFiles;   // in result.json order
    std::string Outcome() const { return Result.Get("outcome") ? Result.Get("outcome")->AsString("") : std::string(); }
};

struct Campaign
{
    std::vector<TestData> Tests;     // in campaign_result.json order
    Json::Value Common;              // the manifest of the first test (all manifests agree on the fields that matter)
    const TestData* Find(const std::string& Name) const
    {
        for (const auto& T : Tests)
            if (T.Name == Name) return &T;
        return nullptr;
    }
    double Ambient() const
    {
        const Json::Value* V = Json::Path(Common, {"conditions", "ambient_c"});
        return V ? V->AsNumber(20.0) : 20.0;
    }
    // Value of one named metric of one test, or Default when the test or the metric is absent.
    double Metric(const std::string& Test, const std::string& Metric, double Default) const
    {
        const TestData* T = Find(Test);
        const Json::Value* M = T ? T->Result.Get("metrics") : nullptr;
        for (std::size_t I = 0; M && I < M->Size(); ++I)
        {
            const Json::Value* E = M->At(I);
            if (E->Get("name") && E->Get("name")->AsString("") == Metric) return E->Get("value") ? E->Get("value")->AsNumber(Default) : Default;
        }
        return Default;
    }
};

using ReadFile = std::function<bool(const std::string& RelPath, std::string& OutText)>;

namespace CampaignDetail
{
// Columns the replay and the margins need. roll_deg / pitch_deg exist from the P7 core on; older folders are refused.
inline const std::vector<std::string>& Required()
{
    static const std::vector<std::string> R = [] {
        std::vector<std::string> V = {"t_s", "pos_n_m", "pos_e_m", "pos_d_m", "vel_n_mps", "vel_e_mps", "vel_d_mps", "tilt_deg", "yaw_deg", "roll_deg", "pitch_deg",
                                      "wind_n_mps", "wind_e_mps", "wind_d_mps", "air_speed_mps", "thrust_reserve_ratio", "battery_v", "battery_a", "battery_soc",
                                      "battery_w", "winding_c_max", "case_c_max", "esc_c_max", "battery_c"};
        for (const char* K : {"rpm", "current_a", "torque_nm", "thrust_n"})
            for (int M = 1; M <= 4; ++M) V.push_back(std::string(K) + "_" + std::to_string(M));
        return V;
    }();
    return R;
}
inline const std::vector<std::string>& SameInAll()
{
    static const std::vector<std::string> K = {"build", "parts", "parts_db_version", "git_hash", "core_interface_version", "controller_config_version",
                                               "step_s", "drag_model", "turbulence", "conditions", "definitions_set_version"};
    return K;
}
inline bool Equal(const Json::Value& A, const Json::Value& B)
{
    if (A.type != B.type) return false;
    switch (A.type)
    {
        case Json::Value::Type::Null: return true;
        case Json::Value::Type::Bool: return A.boolean == B.boolean;
        case Json::Value::Type::Number: return A.number == B.number;
        case Json::Value::Type::String: return A.text == B.text;
        case Json::Value::Type::Array:
            if (A.items.size() != B.items.size()) return false;
            for (std::size_t I = 0; I < A.items.size(); ++I)
                if (!Equal(A.items[I], B.items[I])) return false;
            return true;
        case Json::Value::Type::Object:
            if (A.members.size() != B.members.size()) return false;
            for (const auto& M : A.members)
            {
                const Json::Value* O = B.Get(M.first);
                if (O == nullptr || !Equal(M.second, *O)) return false;
            }
            return true;
    }
    return false;
}
inline std::vector<std::string> SplitLines(const std::string& Text)
{
    std::vector<std::string> Lines;
    std::size_t Start = 0;
    while (Start < Text.size())
    {
        std::size_t End = Text.find('\n', Start);
        if (End == std::string::npos) End = Text.size();
        std::size_t E2 = End;
        if (E2 > Start && Text[E2 - 1] == '\r') --E2;
        Lines.push_back(Text.substr(Start, E2 - Start));
        Start = End + 1;
    }
    return Lines;
}
inline std::vector<std::string> SplitCommas(const std::string& Line)
{
    std::vector<std::string> Out;
    std::size_t Start = 0;
    while (true)
    {
        const std::size_t C = Line.find(',', Start);
        if (C == std::string::npos) { Out.push_back(Line.substr(Start)); break; }
        Out.push_back(Line.substr(Start, C - Start));
        Start = C + 1;
    }
    return Out;
}
}  // namespace CampaignDetail

// Parses a time-series CSV (header + rows of numbers). Refuses ragged rows and anything that is not a finite number.
inline bool ParseSeries(const std::string& Text, Series& Out, std::string& Err)
{
    const std::vector<std::string> Lines = CampaignDetail::SplitLines(Text);
    if (Lines.empty()) { Err = "empty series file"; return false; }
    Out = Series();
    Out.Names = CampaignDetail::SplitCommas(Lines[0]);
    Out.Cols.assign(Out.Names.size(), std::vector<double>());
    for (std::size_t L = 1; L < Lines.size(); ++L)
    {
        const std::vector<std::string> Cells = CampaignDetail::SplitCommas(Lines[L]);
        if (Cells.size() != Out.Names.size()) { Err = "line " + std::to_string(L + 1) + ": " + std::to_string(Cells.size()) + " cells, expected " + std::to_string(Out.Names.size()); return false; }
        for (std::size_t C = 0; C < Cells.size(); ++C)
        {
            char* End = nullptr;
            const double V = std::strtod(Cells[C].c_str(), &End);
            if (Cells[C].empty() || End == Cells[C].c_str() || *End != '\0' || !std::isfinite(V))
            {
                Err = "line " + std::to_string(L + 1) + ": " + Out.Names[C] + " is not a number";
                return false;
            }
            Out.Cols[C].push_back(V);
        }
    }
    return true;
}

inline bool RequireColumns(const Series& S, const std::string& What, std::string& Err)
{
    for (const std::string& C : CampaignDetail::Required())
        if (S.Index(C) < 0) { Err = What + " lacks column " + C + " (campaign written by an older core?)"; return false; }
    return true;
}

inline bool LoadCampaign(const ReadFile& Read, Campaign& Out, std::string& Err)
{
    Out = Campaign();
    std::string Text;
    auto LoadJson = [&](const std::string& Rel, Json::Value& V) {
        if (!Read(Rel, Text)) { Err = "cannot read " + Rel; return false; }
        Json::ParseResult R = Json::Parse(Text);
        if (!R.ok) { Err = Rel + ": " + R.error; return false; }
        V = std::move(R.value);
        return true;
    };
    Json::Value Top;
    if (!LoadJson("campaign_result.json", Top)) return false;
    if (!Top.Get("schema_version") || Top.Get("schema_version")->AsNumber(-1) != 1) { Err = "campaign_result.json schema_version is not 1"; return false; }
    const Json::Value* Tests = Top.Get("tests");
    if (Tests == nullptr || !Tests->IsArray() || Tests->Size() == 0) { Err = "campaign_result.json lists no tests"; return false; }
    for (std::size_t I = 0; I < Tests->Size(); ++I)
    {
        const Json::Value* Entry = Tests->At(I);
        const std::string Name = Entry->Get("test") ? Entry->Get("test")->AsString("") : std::string();
        if (Name.empty() || Name.find('/') != std::string::npos || Name.find("..") != std::string::npos) { Err = "bad test name in campaign_result.json"; return false; }
        TestData T;
        T.Name = Name;
        Json::Value Ev;
        if (!LoadJson(Name + "/manifest.json", T.Manifest) || !LoadJson(Name + "/result.json", T.Result) || !LoadJson(Name + "/events.json", Ev)) return false;
        for (const Json::Value* Doc : {&T.Manifest, &T.Result})
            if (!Doc->Get("schema_version") || Doc->Get("schema_version")->AsNumber(-1) != 1) { Err = Name + ": schema_version is not 1"; return false; }
        const std::string Want = Entry->Get("outcome") ? Entry->Get("outcome")->AsString("") : std::string();
        if (T.Outcome() != Want) { Err = Name + ": outcome in result.json differs from campaign_result.json"; return false; }
        const Json::Value* SeriesList = T.Result.Get("series");
        for (std::size_t K = 0; SeriesList && K < SeriesList->Size(); ++K)
        {
            const std::string File = SeriesList->At(K)->Get("file") ? SeriesList->At(K)->Get("file")->AsString("") : std::string();
            if (File.empty() || File.find('/') != std::string::npos || File.find("..") != std::string::npos) { Err = Name + ": bad series file name"; return false; }
            std::string Csv;
            if (!Read(Name + "/" + File, Csv)) { Err = "cannot read " + Name + "/" + File; return false; }
            Series S;
            if (!ParseSeries(Csv, S, Err)) { Err = Name + "/" + File + ": " + Err; return false; }
            if (!RequireColumns(S, Name + "/" + File, Err)) return false;
            const double Rows = SeriesList->At(K)->Get("rows") ? SeriesList->At(K)->Get("rows")->AsNumber(-1) : -1;
            if (static_cast<double>(S.Rows()) != Rows) { Err = Name + "/" + File + ": row count differs from result.json"; return false; }
            T.SeriesFiles.emplace_back(File, std::move(S));
        }
        const Json::Value* Peaks = Ev.Get("peaks");
        for (std::size_t K = 0; Peaks && K < Peaks->Size(); ++K)
        {
            const Json::Value* P = Peaks->At(K);
            if (P->Get("name") && P->Get("value")) T.Peaks[P->Get("name")->AsString("")] = Peak{P->Get("value")->AsNumber(0.0), P->Get("t_s") ? P->Get("t_s")->AsNumber(0.0) : 0.0};
        }
        const Json::Value* Events = Ev.Get("events");
        for (std::size_t K = 0; Events && K < Events->Size(); ++K)
        {
            const Json::Value* E = Events->At(K);
            EventRow Row;
            Row.Name = E->Get("name") ? E->Get("name")->AsString("") : std::string();
            const Json::Value* First = E->Get("first_t_s");
            Row.Occurred = First != nullptr && First->IsNumber();
            if (Row.Occurred)
            {
                Row.FirstS = First->number;
                Row.LastS = E->Get("last_t_s") ? E->Get("last_t_s")->AsNumber(First->number) : First->number;
            }
            Row.Activations = E->Get("activations") ? E->Get("activations")->AsNumber(0.0) : 0.0;
            Row.ActiveSteps = E->Get("active_steps") ? E->Get("active_steps")->AsNumber(0.0) : 0.0;
            T.Events.push_back(Row);
        }
        Out.Tests.push_back(std::move(T));
    }
    Out.Common = Out.Tests[0].Manifest;
    for (const TestData& T : Out.Tests)
        for (const std::string& K : CampaignDetail::SameInAll())
        {
            const Json::Value* A = T.Manifest.Get(K);
            const Json::Value* B = Out.Common.Get(K);
            if ((A == nullptr) != (B == nullptr) || (A && !CampaignDetail::Equal(*A, *B))) { Err = T.Name + ": manifest field " + K + " differs from the other tests"; return false; }
        }
    const Json::Value* TopBuild = Top.Get("build");
    const Json::Value* ComBuild = Out.Common.Get("build");
    if (!TopBuild || !ComBuild || TopBuild->AsString("") != ComBuild->AsString("x")) { Err = "campaign_result.json build differs from the manifests"; return false; }
    return true;
}
}  // namespace DroneApp
