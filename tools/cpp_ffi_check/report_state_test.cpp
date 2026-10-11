// Tier: V
// Tests for DroneReportData.h and DroneAppState.h. Plain C++17, no Unreal.
//   g++ -std=c++17 -Wall -Wextra -Werror -I UE5Project/Plugins/DronePhysicsBridge/Source/DronePhysicsBridge/Public tools/cpp_ffi_check/report_state_test.cpp -o report_state_test
//   ./report_state_test <repo root>
#include "DroneAppState.h"
#include "DroneReportData.h"

#include <cmath>
#include <cstdio>
#include <fstream>
#include <sstream>
#include <string>

using namespace DroneApp;

static int g_failures = 0;
static void Check(bool ok, const char* name)
{
    if (!ok)
    {
        std::printf("FAIL: %s\n", name);
        ++g_failures;
    }
}
static bool Near(double a, double b, double tol) { return std::fabs(a - b) <= tol; }
static std::string g_root;
static std::string ReadRel(const std::string& Rel)
{
    std::ifstream f(g_root + "/" + Rel, std::ios::binary);
    std::stringstream ss;
    ss << f.rdbuf();
    return ss.str();
}

static void TestSmokeReport()
{
    ReportData R;
    std::string Err;
    const std::string Text = ReadRel("tools/parts/builds/reference_quad/report_smoke/report.json");
    Check(ParseReport(Text, R, Err), ("smoke report parses: " + Err).c_str());
    Check(R.Build == "reference_quad" && R.GitHash == "golden" && R.SelectedConditionSet == "smoke" && R.Smoke(), "header fields and the smoke banner");
    Check(R.Margins.size() == 7 && R.Margins[0].Slot == "motor_current" && Near(R.Margins[0].Margin, 0.9595186483, 1e-8),
          "margins: the motor current limits, 23 A / 23.9704 A = 0.95952 (the FPV punch-out peak, largest over the eight tests' raw events, is above the rating)");
    for (std::size_t I = 1; I < R.Margins.size(); ++I) Check(R.Margins[I].Margin >= R.Margins[I - 1].Margin, "margins ascending");
    auto FindTest = [&](const char* Name) -> const TestRow* { for (const auto& T : R.Tests) if (T.Test == Name) return &T; return nullptr; };
    const TestRow* Hover = FindTest("hover_payload");
    const TestRow* Climb = FindTest("climb");
    Check(R.Tests.size() == 8 && R.Tests[0].Test == "climb" && R.Tests[2].Test == "fpv_punchout", "eight tests, in the report's (alphabetical) order");
    Check(Hover && Hover->HasBreakingPoint && Near(Hover->Breaking.LastPass, 2.5, 0.0) && Near(Hover->Breaking.FirstFail, 3.0, 0.0) && Hover->Breaking.Criterion == "altitude_hold" &&
              Near(Hover->Breaking.Resolution, 0.5, 0.0), "hover breaking point 2.5 / 3.0 kg");
    Check(Climb && !Climb->HasBreakingPoint && Climb->Outcome == "COMPLETED" && !Climb->Metrics.empty(), "climb has metrics and no breaking point");
    Check(R.NotTested.empty(), "no test is listed as not built any more");
    // the 6 flagged rows of the committed validation report (five motor voltage rows beyond 10 % and the worst propeller coefficient), strictly above 10
    Check(R.ValidationFlagPercent == 10.0 && R.FlaggedCount() == 6, "six validation rows above 10 %");
    double Worst = 0.0;
    for (const auto& V : R.ValidationRows) if (V.Flagged && std::fabs(V.PercentError) > Worst) Worst = std::fabs(V.PercentError);
    Check(Near(Worst, 27.8417354943, 1e-6), "worst flagged error 27.84 %");
    Check(R.ValidationSections.size() == 4 && R.ValidationSections[3].Status == "NO_PUBLISHED_DATA" && R.ValidationRows.size() == 40 + 212 + 5, "validation sections and row count");
    Check(!R.Upgrades.Run && R.Upgrades.Statement.find("No upgrade study") == 0, "no study in a single-campaign report");
    Check(R.CfdStatus == "no CFD run" && R.DragModel == "analytic_legacy", "no CFD");
    Check(R.Envelope.size() == 1 && R.Envelope[0].ConditionSet == "smoke" && Near(R.Envelope[0].MaxPayloadKg, 2.5, 0.0) && Near(R.Envelope[0].EnduranceS, 70.32, 1e-9), "envelope");
    Check(!R.Events.empty() && !R.Moments.empty() && !R.Assumptions.empty() && R.Assumptions[0].Id == "A001" && R.Commands.size() == 1, "events, moments, assumptions, commands");
    Check(Near(R.PackW, 122.822079, 1e-6) && Near(R.ShaftW + R.LossW, R.PackW, 1e-5) && R.PowerSource == "endurance/timeseries_endurance.csv", "power block");
    bool MaxSpeed = false;
    for (const auto& S : R.Summary) MaxSpeed = MaxSpeed || (S.Label == "Max speed (commanded, passing)" && S.ValueText == "12 m/s" && Near(S.Value, 12.0, 0.0) && S.Detail.find("LIMIT-BOUND") == 0);
    Check(MaxSpeed, "max speed shows 12 m/s with the LIMIT-BOUND statement");
    Check(R.Banners.size() >= 2 && [&] { for (const auto& B : R.Banners) if (B.Id == "speed_drag") return true; return false; }(), "the speed-drag banner is carried");
    // a text/number summary row
    Check(R.Summary[0].Label == "Limiting component" && R.Summary[0].ValueText == "Motor current" && R.Summary[1].ValueText == "2.5 kg", "summary value text");
    // refusals
    Check(!ParseReport("{\"schema_version\":2}", R, Err) && Err.find("schema_version") != std::string::npos, "wrong schema refused");
    Check(!ParseReport("{\"schema_version\":1}", R, Err) && Err.find("margins") != std::string::npos, "no margins refused");
    Check(!ParseReport("nope", R, Err), "bad json refused");
}

static const char* kStudy = R"({"schema_version":1,"title":"t","build":"b",
 "validation":{"flag_percent":10,"sections":[{"name":"s","status":"OK","note":"","summary":{"n":4,"max_abs_percent":10.01,"mean_abs_percent":5,"n_outside_band":0,"worst_row_id":"b"},
  "rows":[{"id":"a","quantity":"q","unit":"V","reference":1,"model":1,"percent_error":10.0,"band_percent":20,"outside_band":false},
          {"id":"b","quantity":"q","unit":"V","reference":1,"model":1,"percent_error":10.01,"band_percent":20,"outside_band":false},
          {"id":"c","quantity":"q","unit":"V","reference":1,"model":1,"percent_error":-9.99,"band_percent":20,"outside_band":false},
          {"id":"d","quantity":"q","unit":"V","reference":1,"model":1,"percent_error":null,"band_percent":20,"outside_band":false}]}]},"margins":[{"slot":"motor_current","label":"Motor current","limit":23,"peak":20,"unit":"A","margin":1.15,"peak_test":"endurance","peak_t_s":4.5,"no_heating":false}],
 "upgrades":{"status":"run","condition_set":"smoke","limiting_slot":"motor_current","notice":"SMOKE","baseline":{"limiting_margin":1.12},"baseline_weakest":{"label":"Motor current","margin":1.12,"slot":"motor_current"},
  "metrics":{"limiting_margin":{"label":"Limiting margin","unit":"ratio","higher_is_better":true},"endurance_s":{"label":"Endurance","unit":"s","higher_is_better":true},"price_usd":{"label":"Price","unit":"USD","higher_is_better":false}},
  "entries":[
   {"id":"motor__a","role":"motor","part_id":"a","status":"tested","rank":1,"rank_note":"","warnings":[{"rule":"W-X"}],"weakest":{"label":"Thrust reserve","margin":1.3},
    "deltas":{"limiting_margin":{"base":1.12,"cand":1.5,"delta":0.38,"better":true},"endurance_s":{"base":70,"cand":60,"delta":-10,"better":false},"price_usd":{"base":null,"cand":null,"delta":null,"better":null}}},
   {"id":"prop__b","role":"propeller","part_id":"b","status":"tested","rank":null,"rank_note":"propeller table is a scaled stand-in (ASSUMED): not ranked as measured","warnings":[],"weakest":null,
    "deltas":{"limiting_margin":{"base":1.12,"cand":1.1,"delta":-0.02,"better":false},"endurance_s":{"base":70,"cand":70,"delta":0,"better":null}}},
   {"id":"esc__c","role":"esc","part_id":"c","status":"rejected","rank":null,"rank_note":"not compatible: E-ESC-I","deltas":null,"weakest":null}]}})";

static void TestStudyReport()
{
    ReportData R;
    std::string Err;
    Check(ParseReport(kStudy, R, Err), ("study report parses: " + Err).c_str());
    Check(R.FlaggedCount() == 1 && R.ValidationRows.size() == 4 && R.ValidationRows[1].Flagged && !R.ValidationRows[0].Flagged && !R.ValidationRows[3].Flagged && std::isnan(R.ValidationRows[3].PercentError),
          "validation flag is strictly above 10 %: 10.0 and -9.99 are not flagged, 10.01 is, a null error is n/a and not flagged");
    const UpgradeSection& U = R.Upgrades;
    Check(U.Run && U.ConditionSet == "smoke" && U.LimitingSlot == "motor_current" && Near(U.BaselineLimitingMargin, 1.12, 0.0) && U.BaselineWeakest == "Motor current" && U.Notice == "SMOKE", "section header");
    Check(U.Entries.size() == 3 && U.Entries[0].Rank == 1 && U.Entries[1].Rank == 0 && U.Entries[2].Rank == 0, "ranks: null means not ranked (0), not rank zero of a list");
    const UpgradeRow& A = U.Entries[0];
    Check(A.Deltas.size() == 3 && A.Delta("limiting_margin") && A.Delta("limiting_margin")->Tone == DeltaTone::Better && DeltaText(*A.Delta("limiting_margin")) == "+0.38", "gain shown green with a plus");
    Check(A.Delta("endurance_s")->Tone == DeltaTone::Worse && DeltaText(*A.Delta("endurance_s")) == "-10", "a cost shown red: the gain's price stays visible");
    Check(!A.Delta("price_usd")->Available() && DeltaText(*A.Delta("price_usd")) == "n/a" && A.Delta("price_usd")->Tone == DeltaTone::None, "unknown price is n/a, not zero");
    Check(A.WeakestLabel == "Thrust reserve" && Near(A.WeakestMargin, 1.3, 0.0) && A.WarningRules.size() == 1 && A.WarningRules[0] == "W-X", "weakest component and warnings");
    Check(U.Entries[1].Note.find("scaled stand-in") != std::string::npos && U.Entries[1].Delta("endurance_s")->Tone == DeltaTone::None && DeltaText(*U.Entries[1].Delta("endurance_s")) == "0", "unranked stand-in row; zero change has no colour");
    Check(U.Entries[2].Status == "rejected" && U.Entries[2].Note == "not compatible: E-ESC-I" && !U.Entries[2].Delta("limiting_margin")->Available(), "rejected row has no numbers");
    Check(NumText(kNaN) == "n/a" && NumText(2.0, "kg") == "2 kg" && NumText(0.12345678) == "0.1235" && NumText(1234567.0) == "1.235e+06", "NumText");
}

static void TestAppState()
{
    Progress P;
    Check(CanOpen(Screen::Vehicle, P) && !CanOpen(Screen::Parts, P) && BlockedReason(Screen::Parts, P) == "Choose a vehicle first.", "only the vehicle screen at the start");
    for (Screen S : {Screen::Build, Screen::Conditions, Screen::Run, Screen::Results, Screen::Compare, Screen::Report}) Check(!CanOpen(S, P), "later screens blocked at the start");
    P.VehicleClassChosen = true;
    Check(CanOpen(Screen::Parts, P) && !CanOpen(Screen::Build, P) && BlockedReason(Screen::Build, P).find("required parts") != std::string::npos, "parts open after a vehicle, build needs all parts");
    P.PartsComplete = true;
    Check(CanOpen(Screen::Build, P) && !CanOpen(Screen::Conditions, P), "build open, conditions need a build");
    P.BuildDone = true;
    Check(CanOpen(Screen::Conditions, P) && !CanOpen(Screen::Run, P) && !CanOpen(Screen::Results, P), "conditions open, run and results need tests");
    P.CampaignDone = true;
    Check(CanOpen(Screen::Run, P) && CanOpen(Screen::Results, P) && !CanOpen(Screen::Compare, P) && !CanOpen(Screen::Report, P), "run and results open; compare and report need more");
    P.ReportDone = true;
    P.StudyDone = true;
    for (int I = 0; I < kScreenCount; ++I) Check(CanOpen(static_cast<Screen>(I), P), "everything open when everything exists");
    Check(std::string(ScreenName(Screen::Conditions)) == "Conditions" && std::string(ScreenName(static_cast<Screen>(99))).empty(), "names");
    // Next skips blocked screens; Previous steps back; ends stay
    Progress Q;
    Q.VehicleClassChosen = true;
    Check(Next(Screen::Vehicle, Q) == Screen::Parts && Next(Screen::Parts, Q) == Screen::Parts, "next stops at the first blocked screen");
    Check(Next(Screen::Report, P) == Screen::Report && Previous(Screen::Vehicle) == Screen::Vehicle && Previous(Screen::Run) == Screen::Conditions, "ends and previous");
    // changing an earlier step invalidates what was derived from it
    Progress Full = P;
    Progress A = AfterChange(Screen::Vehicle, Full);
    Check(!A.PartsComplete && !A.BuildDone && !A.ConditionsValid && !A.CampaignDone && !A.ReportDone && !A.StudyDone && A.VehicleClassChosen, "a new vehicle class clears everything derived");
    A = AfterChange(Screen::Parts, Full);
    Check(A.PartsComplete && !A.BuildDone && !A.CampaignDone, "new parts keep the class but need a new build");
    A = AfterChange(Screen::Build, Full);
    Check(A.PartsComplete && A.BuildDone == Full.BuildDone && !A.CampaignDone && !A.ReportDone, "a new build clears results");
    A = AfterChange(Screen::Conditions, Full);
    Check(A.BuildDone && !A.CampaignDone && !A.ReportDone && !A.StudyDone, "new conditions clear old results");
    A = AfterChange(Screen::Results, Full);
    Check(A.CampaignDone && A.ReportDone, "opening results changes nothing");
    ModalGate G;
    Check(!G.IsOpen() && G.Open("building") && G.IsOpen() && G.What() == "building" && !G.Open("second"), "one modal at a time");
    G.Close();
    Check(!G.IsOpen() && G.Open("second"), "reopens after close");
}

int main(int argc, char** argv)
{
    if (argc < 2)
    {
        std::printf("usage: report_state_test <repo root>\n");
        return 2;
    }
    g_root = argv[1];
    TestSmokeReport();
    TestStudyReport();
    TestAppState();
    if (g_failures == 0) std::printf("report_state_test: all checks passed\n");
    return g_failures == 0 ? 0 : 1;
}
