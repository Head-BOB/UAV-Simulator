// Tier: V
// Engine-free view models for the Results, Compare and Report screens, read from the report.json written by tools/report/make_report.py.
// Nothing is recomputed here except display text: every number on screen is a number of the file. A value that is null in the file is
// "not available" (NaN here), never zero.
#pragma once

#include <cmath>
#include <cstdio>
#include <limits>
#include <string>
#include <vector>

#include "DroneJson.h"
#include "DroneMargins.h"

namespace DroneApp
{
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

inline std::string NumText(double V, const std::string& Unit = std::string())
{
    if (std::isnan(V)) return "n/a";
    char Buf[48];
    if (std::fabs(V) < 1e6 && V == std::floor(V)) std::snprintf(Buf, sizeof Buf, "%.0f", V);
    else std::snprintf(Buf, sizeof Buf, "%.4g", V);
    return Unit.empty() ? std::string(Buf) : std::string(Buf) + " " + Unit;
}

struct BannerRow { std::string Id, Level, Text; };
struct SummaryRow { std::string Label, Unit, Detail, ValueText; double Value = kNaN; };
struct BreakingPointRow { std::string Quantity, Unit, Criterion; double LastPass = kNaN, FirstFail = kNaN, Resolution = kNaN, Evaluations = kNaN; };
struct MetricRow { std::string Name; double Value = kNaN; };
struct TestRow
{
    std::string Test, Outcome, DefinitionVersion;
    bool HasBreakingPoint = false;
    BreakingPointRow Breaking;
    std::vector<MetricRow> Metrics;
    std::vector<std::string> Notes;
};
struct NotTestedRow { std::string Test, Status, Reason; };
struct TimelineRow { std::string Test, Event; double FirstS = kNaN, LastS = kNaN, Activations = kNaN; };
struct MomentRow { std::string Test, Moment; double Value = kNaN, TimeS = kNaN; };
struct ValidationRow { std::string Section, Id, Quantity, Unit; double Reference = kNaN, Model = kNaN, PercentError = kNaN, BandPercent = kNaN; bool OutsideBand = false, Flagged = false; };
struct ValidationSection { std::string Name, Status, Note, WorstId; double N = 0, MaxAbsPercent = kNaN, MeanAbsPercent = kNaN, OutsideBand = 0; };
struct AssumptionRow { std::string Id, Category, Item, Type, Where, Effect; };
struct EnvelopeRow { std::string ConditionSet, LimitingLabel; double LimitingMargin = kNaN, MaxPayloadKg = kNaN, MaxWindMps = kNaN, EnduranceS = kNaN, ClimbTimeS = kNaN, MinReserve = kNaN, PeakWindingC = kNaN; };

// Colour class of a delta: green better, red worse, none otherwise (no preference, unchanged, or not available).
enum class DeltaTone { None, Better, Worse };
struct DeltaCell { std::string Metric, Label, Unit; double Base = kNaN, Cand = kNaN, Delta = kNaN; DeltaTone Tone = DeltaTone::None; bool Available() const { return !std::isnan(Delta); } };
struct UpgradeRow
{
    int Rank = 0;                       // 0 = not ranked
    std::string Id, Role, PartId, Status, Note, WeakestLabel;
    double WeakestMargin = kNaN;
    std::vector<DeltaCell> Deltas;      // in the metric order of the file
    std::vector<std::string> WarningRules;
    const DeltaCell* Delta(const std::string& Metric) const { for (const auto& D : Deltas) if (D.Metric == Metric) return &D; return nullptr; }
};
struct UpgradeSection
{
    bool Run = false;
    std::string Statement, Notice, ConditionSet, LimitingSlot, BaselineWeakest;
    double BaselineLimitingMargin = kNaN;
    std::vector<UpgradeRow> Entries;
};

struct ReportData
{
    std::string Title, Build, GitHash, SelectedConditionSet, CfdStatus, CfdStatement, DragModel, PowerSource;
    std::vector<BannerRow> Banners;
    std::vector<SummaryRow> Summary;
    std::vector<TestRow> Tests;
    std::vector<NotTestedRow> NotTested;
    std::vector<TimelineRow> Events;
    std::vector<MomentRow> Moments;
    std::vector<MarginRow> Margins;           // ascending; the first is the limiting component
    std::vector<ValidationSection> ValidationSections;
    std::vector<ValidationRow> ValidationRows;
    double ValidationFlagPercent = 10.0;
    std::vector<EnvelopeRow> Envelope;
    UpgradeSection Upgrades;
    std::vector<AssumptionRow> Assumptions;
    std::vector<std::string> Commands;
    double PackW = kNaN, ShaftW = kNaN, LossW = kNaN;

    int FlaggedCount() const { int N = 0; for (const auto& R : ValidationRows) N += R.Flagged; return N; }
    bool Smoke() const { for (const auto& B : Banners) if (B.Id == "smoke") return true; return false; }
};

namespace ReportDetail
{
inline std::string Str(const Json::Value* V, const char* K) { const Json::Value* X = V ? V->Get(K) : nullptr; return X ? X->AsString("") : std::string(); }
inline double Num(const Json::Value* V, const char* K) { const Json::Value* X = V ? V->Get(K) : nullptr; return (X && X->IsNumber()) ? X->number : kNaN; }
}  // namespace ReportDetail

inline bool ParseReport(const std::string& Text, ReportData& Out, std::string& Err)
{
    using namespace ReportDetail;
    Out = ReportData();
    const Json::ParseResult R = Json::Parse(Text);
    if (!R.ok) { Err = "report.json: " + R.error; return false; }
    const Json::Value& J = R.value;
    if (Num(&J, "schema_version") != 1.0) { Err = "report.json schema_version is not 1"; return false; }
    Out.Title = Str(&J, "title");
    Out.Build = Str(&J, "build");
    auto Each = [&](const char* Key, auto&& Fn) { const Json::Value* A = J.Get(Key); for (std::size_t I = 0; A && I < A->Size(); ++I) Fn(*A->At(I)); };
    Each("banners", [&](const Json::Value& B) { Out.Banners.push_back({Str(&B, "id"), Str(&B, "level"), Str(&B, "text")}); });
    Each("summary", [&](const Json::Value& S) {
        SummaryRow Row;
        Row.Label = Str(&S, "label"); Row.Unit = Str(&S, "unit"); Row.Detail = Str(&S, "detail");
        const Json::Value* V = S.Get("value");
        if (V && V->IsNumber()) { Row.Value = V->number; Row.ValueText = NumText(V->number, Row.Unit); }
        else if (V && V->IsString()) Row.ValueText = V->text;
        else Row.ValueText = "not tested";
        Out.Summary.push_back(Row);
    });
    Each("tests", [&](const Json::Value& T) {
        TestRow Row;
        Row.Test = Str(&T, "test"); Row.Outcome = Str(&T, "outcome"); Row.DefinitionVersion = Str(&T, "definition_version");
        const Json::Value* Bp = T.Get("breaking_point");
        if (Bp && Bp->IsObject())
        {
            Row.HasBreakingPoint = true;
            Row.Breaking = {Str(Bp, "quantity"), Str(Bp, "unit"), Str(Bp, "criterion"), Num(Bp, "last_pass"), Num(Bp, "first_fail"), Num(Bp, "resolution"), Num(Bp, "evaluations")};
        }
        const Json::Value* M = T.Get("metrics");
        for (std::size_t I = 0; M && I < M->Size(); ++I) Row.Metrics.push_back({Str(M->At(I), "name"), Num(M->At(I), "value")});
        const Json::Value* N = T.Get("notes");
        for (std::size_t I = 0; N && I < N->Size(); ++I) Row.Notes.push_back(N->At(I)->AsString(""));
        Out.Tests.push_back(Row);
    });
    Each("not_built", [&](const Json::Value& N) { Out.NotTested.push_back({Str(&N, "test"), Str(&N, "status"), Str(&N, "reason")}); });
    Each("events", [&](const Json::Value& E) { Out.Events.push_back({Str(&E, "test"), Str(&E, "event"), Num(&E, "first_t_s"), Num(&E, "last_t_s"), Num(&E, "activations")}); });
    Each("key_moments", [&](const Json::Value& K) { Out.Moments.push_back({Str(&K, "test"), Str(&K, "moment"), Num(&K, "value"), Num(&K, "t_s")}); });
    Each("margins", [&](const Json::Value& M) {
        MarginRow Row;
        Row.Slot = Str(&M, "slot"); Row.Label = Str(&M, "label"); Row.Limit = Num(&M, "limit"); Row.Peak = Num(&M, "peak"); Row.Unit = Str(&M, "unit");
        Row.Margin = Num(&M, "margin"); Row.PeakTest = Str(&M, "peak_test"); Row.PeakTimeS = Num(&M, "peak_t_s");
        const Json::Value* H = M.Get("no_heating");
        Row.NoHeating = H && H->AsBool(false);
        Out.Margins.push_back(Row);
    });
    if (Out.Margins.empty()) { Err = "report.json has no margins"; return false; }
    if (const Json::Value* P = J.Get("power")) { Out.PackW = Num(P, "pack_w"); Out.ShaftW = Num(P, "shaft_w"); Out.LossW = Num(P, "loss_total_w"); Out.PowerSource = Str(P, "source"); }
    if (const Json::Value* C = J.Get("cfd")) { Out.CfdStatus = Str(C, "status"); Out.CfdStatement = Str(C, "statement"); Out.DragModel = Str(C, "drag_model"); }
    if (const Json::Value* V = J.Get("validation"))
    {
        const double Flag = Num(V, "flag_percent");
        if (!std::isnan(Flag)) Out.ValidationFlagPercent = Flag;
        const Json::Value* Secs = V->Get("sections");
        for (std::size_t I = 0; Secs && I < Secs->Size(); ++I)
        {
            const Json::Value* S = Secs->At(I);
            const Json::Value* Sm = S->Get("summary");
            ValidationSection Sec;
            Sec.Name = Str(S, "name"); Sec.Status = Str(S, "status"); Sec.Note = Str(S, "note");
            Sec.N = Num(Sm, "n"); Sec.MaxAbsPercent = Num(Sm, "max_abs_percent"); Sec.MeanAbsPercent = Num(Sm, "mean_abs_percent"); Sec.OutsideBand = Num(Sm, "n_outside_band");
            Sec.WorstId = Str(Sm, "worst_row_id");
            Out.ValidationSections.push_back(Sec);
            const Json::Value* Rows = S->Get("rows");
            for (std::size_t K = 0; Rows && K < Rows->Size(); ++K)
            {
                const Json::Value* Rw = Rows->At(K);
                ValidationRow Row;
                Row.Section = Sec.Name; Row.Id = Str(Rw, "id"); Row.Quantity = Str(Rw, "quantity"); Row.Unit = Str(Rw, "unit");
                Row.Reference = Num(Rw, "reference"); Row.Model = Num(Rw, "model"); Row.PercentError = Num(Rw, "percent_error"); Row.BandPercent = Num(Rw, "band_percent");
                Row.OutsideBand = Rw->Get("outside_band") && Rw->Get("outside_band")->AsBool(false);
                Row.Flagged = !std::isnan(Row.PercentError) && std::fabs(Row.PercentError) > Out.ValidationFlagPercent;   // strictly above, as the report
                Out.ValidationRows.push_back(Row);
            }
        }
    }
    Each("envelope", [&](const Json::Value& E) {
        EnvelopeRow Row;
        Row.ConditionSet = Str(&E, "condition_set");
        const Json::Value* L = E.Get("limiting");
        Row.LimitingLabel = Str(L, "label"); Row.LimitingMargin = Num(L, "margin");
        const Json::Value* H = E.Get("headline");
        Row.MaxPayloadKg = Num(H, "max_payload_kg"); Row.MaxWindMps = Num(H, "max_wind_mps"); Row.EnduranceS = Num(H, "endurance_s"); Row.ClimbTimeS = Num(H, "climb_time_s");
        Row.MinReserve = Num(H, "min_thrust_reserve"); Row.PeakWindingC = Num(H, "peak_winding_c");
        Out.Envelope.push_back(Row);
    });
    Each("assumptions", [&](const Json::Value& A) { Out.Assumptions.push_back({Str(&A, "id"), Str(&A, "category"), Str(&A, "item"), Str(&A, "source_type"), Str(&A, "where"), Str(&A, "effect")}); });
    if (const Json::Value* Rp = J.Get("reproducibility"))
    {
        Out.GitHash = Str(Rp, "git_hash");
        const Json::Value* Cm = Rp->Get("commands");
        for (std::size_t I = 0; Cm && I < Cm->Size(); ++I) Out.Commands.push_back(Cm->At(I)->AsString(""));
    }
    if (const Json::Value* Cd = J.Get("conditions")) Out.SelectedConditionSet = Str(Cd, "selected");
    if (const Json::Value* U = J.Get("upgrades"))
    {
        Out.Upgrades.Run = Str(U, "status") == "run";
        Out.Upgrades.Statement = Str(U, "statement");
        if (Out.Upgrades.Run)
        {
            Out.Upgrades.Notice = Str(U, "notice"); Out.Upgrades.ConditionSet = Str(U, "condition_set"); Out.Upgrades.LimitingSlot = Str(U, "limiting_slot");
            Out.Upgrades.BaselineWeakest = Str(U->Get("baseline_weakest"), "label");
            Out.Upgrades.BaselineLimitingMargin = Num(U->Get("baseline"), "limiting_margin");
            const Json::Value* Metrics = U->Get("metrics");
            const Json::Value* Ents = U->Get("entries");
            for (std::size_t I = 0; Ents && I < Ents->Size(); ++I)
            {
                const Json::Value* E = Ents->At(I);
                UpgradeRow Row;
                Row.Id = Str(E, "id"); Row.Role = Str(E, "role"); Row.PartId = Str(E, "part_id"); Row.Status = Str(E, "status"); Row.Note = Str(E, "rank_note");
                const double Rk = Num(E, "rank");
                Row.Rank = std::isnan(Rk) ? 0 : static_cast<int>(Rk);
                if (const Json::Value* W = E->Get("weakest")) { Row.WeakestLabel = Str(W, "label"); Row.WeakestMargin = Num(W, "margin"); }
                const Json::Value* W2 = E->Get("warnings");
                for (std::size_t K = 0; W2 && K < W2->Size(); ++K) Row.WarningRules.push_back(Str(W2->At(K), "rule"));
                const Json::Value* D = E->Get("deltas");
                for (std::size_t K = 0; Metrics && K < Metrics->members.size(); ++K)
                {
                    DeltaCell Cell;
                    Cell.Metric = Metrics->members[K].first;
                    Cell.Label = Str(&Metrics->members[K].second, "label");
                    Cell.Unit = Str(&Metrics->members[K].second, "unit");
                    const Json::Value* Dm = D ? D->Get(Cell.Metric) : nullptr;
                    if (Dm)
                    {
                        Cell.Base = Num(Dm, "base"); Cell.Cand = Num(Dm, "cand"); Cell.Delta = Num(Dm, "delta");
                        const Json::Value* B = Dm->Get("better");
                        if (B && B->type == Json::Value::Type::Bool) Cell.Tone = B->boolean ? DeltaTone::Better : DeltaTone::Worse;
                    }
                    Row.Deltas.push_back(Cell);
                }
                Out.Upgrades.Entries.push_back(Row);
            }
        }
    }
    return true;
}

// "+0.5" / "-10.32" / "0" / "n/a": the sign is always shown for a change.
inline std::string DeltaText(const DeltaCell& C)
{
    if (!C.Available()) return "n/a";
    const std::string T = NumText(C.Delta);
    return (C.Delta > 0.0 ? "+" : "") + T;
}
}  // namespace DroneApp
