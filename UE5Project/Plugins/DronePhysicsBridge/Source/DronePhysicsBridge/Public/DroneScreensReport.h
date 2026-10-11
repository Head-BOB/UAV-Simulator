// Tier: V
// Engine-free layout of Results, Compare and Report: plain tables with 1 px row lines, no boxes. The report screen follows the preview's
// "Flight report" page: 56 px light title, one verdict line in amber / green, two columns, a footer line. Every number is read from
// report.json (DroneReportData.h).
#pragma once

#include <string>

#include "DroneScreensSetup.h"

namespace DroneApp
{
namespace Screens
{
inline Rgba BannerRgba(const std::string& Level) { return Level == "danger" ? Palette::Crit() : Palette::Warn(); }

// Verdict line: amber when anything needs a human look, otherwise green. Wording never says "passed".
inline void Verdict(DrawList& L, const ReportData& R, float X, float Y, float Wd)
{
    const MarginRow& Lim = R.Margins.front();
    const Status St = StatusOfMargin(Lim.Margin);
    const bool Caution = St != Status::Ok || R.Smoke() || R.FlaggedCount() > 0 || !R.Banners.empty();
    L.Text(X, Y, Wd, "Limiting component: " + Lim.Label + ", margin " + Fmt("%.2f", Lim.Margin), 34.0f, Caution ? Palette::Warn() : Palette::Ok(), Weight::Light);
    L.Text(X, Y + 46.0f, Wd, R.Smoke() ? "SMOKE definitions: not for engineering use. PRELIMINARY results from an unvalidated model." : "PRELIMINARY results from an unvalidated model; read the assumptions.", 16.0f,
           Palette::Ink70());
}

inline void BannerList(DrawList& L, const ReportData& R, float X, float& Y, float Wd, std::size_t Max)
{
    std::size_t N = 0;
    for (const auto& B : R.Banners)
    {
        if (N++ >= Max) break;
        L.Fill(X, Y + 2.0f, 4.0f, 22.0f, BannerRgba(B.Level));
        L.Text(X + 14.0f, Y, Wd - 14.0f, Shorten(B.Text, 150), 15.0f, Palette::Ink70());
        Y += 28.0f;
    }
}

// ----------------------------------------------------------------------------------------------------------------------------- 6 Results
inline void Results(DrawList& L, const AppModel& M)
{
    NavBar(L, M);
    L.Fill(0, static_cast<float>(Ui::kTopBarHeightPx), W, Hgt - static_cast<float>(Ui::kTopBarHeightPx), Palette::Report());
    const ReportData& R = M.Report;
    const float X = 96.0f;
    if (!M.Prog.ReportDone)
    {
        L.Text(X, 100.0f, 900.0f, "Results", 56.0f, Palette::Ink(), Weight::Light);
        L.Text(X, 190.0f, 1200.0f, M.Busy() ? M.BusyLabel() + " ..." : "The report has not been written yet. Open the Report screen and write it.", 20.0f, Palette::Ink70());
        StatusLine(L, M, X, 240.0f, 1500.0f);
        return;
    }
    L.Text(X, 84.0f, 900.0f, "Results", 56.0f, Palette::Ink(), Weight::Light);
    Verdict(L, R, X, 156.0f, 1700.0f);
    float By = 250.0f;
    BannerList(L, R, X, By, 1700.0f, 4);
    // left column: tests and breaking points
    float Y = std::max(By + 20.0f, 340.0f);
    const float Lw = 980.0f, Rx = 1200.0f, Rw = 624.0f;
    L.Text(X, Y, Lw, "Tests and breaking points", 15.0f, Palette::Ink70(), Weight::Medium);
    Y += 28.0f;
    for (const auto& T : R.Tests)
    {
        L.HLine(X, Y, Lw, Palette::Line());
        L.Text(X, Y + 9.0f, 220.0f, T.Test, 20.0f, Palette::Ink());
        L.Text(X + 230.0f, Y + 12.0f, 250.0f, T.Outcome, 15.0f, T.Outcome == "BREAKING_POINT_FOUND" ? Palette::Warn() : Palette::Ink70());
        if (T.HasBreakingPoint)
            L.Text(X + 490.0f, Y + 10.0f, Lw - 490.0f, NumText(T.Breaking.LastPass) + " holds, " + NumText(T.Breaking.FirstFail) + " " + T.Breaking.Unit + " fails (" + T.Breaking.Criterion +
                                                         ", step " + NumText(T.Breaking.Resolution) + ")", 16.0f, Palette::Ink70());
        else if (!T.Metrics.empty())
            L.Text(X + 490.0f, Y + 10.0f, Lw - 490.0f, T.Metrics[0].Name + " " + NumText(T.Metrics[0].Value), 16.0f, Palette::Ink70());
        Y += 42.0f;
    }
    for (const auto& N : R.NotTested)
    {
        L.HLine(X, Y, Lw, Palette::Line());
        L.Text(X, Y + 9.0f, 220.0f, N.Test, 20.0f, Palette::Ink46());
        Chip(L, X + 230.0f, Y + 10.0f, "Not implemented", Palette::Ink46());
        Y += 42.0f;
    }
    L.HLine(X, Y, Lw, Palette::Line());
    // events (first rows)
    Y += 30.0f;
    L.Text(X, Y, Lw, "Event timeline", 15.0f, Palette::Ink70(), Weight::Medium);
    Y += 28.0f;
    int Shown = 0;
    for (const auto& E : R.Events)
    {
        if (Shown++ >= 5) break;
        L.HLine(X, Y, Lw, Palette::Line());
        L.Text(X, Y + 5.0f, 200.0f, E.Test, 15.0f, Palette::Ink70());
        L.Text(X + 210.0f, Y + 4.0f, 420.0f, E.Event, 17.0f, Palette::Ink());
        L.Text(X + 640.0f, Y + 5.0f, 330.0f, NumText(E.FirstS) + " s  to  " + NumText(E.LastS) + " s", 15.0f, Palette::Ink70());
        Y += 32.0f;
    }
    if (R.Events.empty()) L.Text(X, Y, Lw, "No event flag was raised in any test.", 16.0f, Palette::Ink46());
    else if (R.Events.size() > 5) L.Text(X, Y + 6.0f, Lw, "... and " + std::to_string(R.Events.size() - 5) + " more in the report", 14.0f, Palette::Ink46());

    // right column: limiting components, validation, CFD
    float Ry = std::max(By + 20.0f, 340.0f);
    L.Text(Rx, Ry, Rw, "Limiting components (margin = limit / peak)", 15.0f, Palette::Ink70(), Weight::Medium);
    Ry += 28.0f;
    for (const auto& Mg : R.Margins)
    {
        const Status St = StatusOfMargin(Mg.Margin);
        L.HLine(Rx, Ry, Rw, Palette::Line());
        L.Text(Rx, Ry + 8.0f, 380.0f, Mg.Label, 17.0f, Palette::Ink70());
        L.Text(Rx, Ry + 6.0f, Rw, Mg.Margin >= kMarginCap ? std::string("no load") : Fmt("%.2f", Mg.Margin), 20.0f, StatusRgba(St), Weight::Regular, Align::Right);
        Ry += 36.0f;
    }
    L.HLine(Rx, Ry, Rw, Palette::Line());
    Ry += 30.0f;
    L.Text(Rx, Ry, Rw, "Validation against published data", 15.0f, Palette::Ink70(), Weight::Medium);
    Ry += 28.0f;
    for (const auto& V : R.ValidationSections)
    {
        L.HLine(Rx, Ry, Rw, Palette::Line());
        L.Text(Rx, Ry + 6.0f, 300.0f, V.Name, 16.0f, Palette::Ink70());
        L.Text(Rx, Ry + 6.0f, Rw, V.Status == "NO_PUBLISHED_DATA" ? std::string("no published data") : "largest error " + Fmt("%.1f", V.MaxAbsPercent) + " %", 16.0f,
               (!std::isnan(V.MaxAbsPercent) && V.MaxAbsPercent > R.ValidationFlagPercent) ? Palette::Warn() : Palette::Ink(), Weight::Regular, Align::Right);
        Ry += 32.0f;
    }
    L.Text(Rx, Ry + 8.0f, Rw, std::to_string(R.FlaggedCount()) + " rows differ by more than " + Fmt("%.0f", R.ValidationFlagPercent) + " % (listed in the report)", 14.0f, R.FlaggedCount() ? Palette::Warn() : Palette::Ink46());
    Ry += 44.0f;
    {
        // "no CFD run" is the only status that needs the "CFD:" prefix; every other status already says what it is
        const bool Prefixed = R.CfdStatus.compare(0, 3, "CFD") == 0;
        const Rgba C = R.CfdLevel == "danger" ? Palette::Crit() : (R.CfdLevel == "warn" ? Palette::Warn() : Palette::Ink70());
        L.Text(Rx, Ry, Rw, Shorten(Prefixed ? R.CfdStatus : "CFD: " + R.CfdStatus, 80), 15.0f, C, Weight::Medium);
        L.Text(Rx, Ry + 24.0f, Rw, Shorten(R.CfdStatement, 120), 14.0f, Palette::Ink46());
        float Cy = Ry + 48.0f;
        if (!R.CfdSanityStatus.empty() || !std::isnan(R.CfdHoldoutRmsN))
        {
            const std::string Held = std::isnan(R.CfdHoldoutRmsN) ? std::string("none") : Fmt("%.3g", R.CfdHoldoutRmsN) + " N";
            L.Text(Rx, Cy, Rw, "Held-out RMS error " + Held + "; sanity checks " + (R.CfdSanityStatus.empty() ? std::string("not reported") : R.CfdSanityStatus), 14.0f, Palette::Ink46());
            Cy += 24.0f;
        }
        if (R.CfdStatus != "no CFD run")
        {
            std::string Names;
            for (const auto& N : R.CfdImageNames) Names += (Names.empty() ? "" : ", ") + N;
            L.Text(Rx, Cy, Rw, "Flow images: " + (R.CfdImageNames.empty() ? std::string("no flow images") : Shorten(Names, 90)), 14.0f, Palette::Ink46());
        }
    }
    Button(L, X, 980.0f, 280.0f, 52.0f, "Replay a run", true, Act::Navigate, "Run");
    Button(L, X + 300.0f, 980.0f, 280.0f, 52.0f, "Report", false, Act::Navigate, "Report");
    // the flow images are files in the CFD folder (the report embeds them): the button opens that folder in the system viewer
    const bool HasImages = !R.CfdImageNames.empty() && !M.H.CfdDir.empty();
    if (HasImages) Button(L, X + 600.0f, 980.0f, 260.0f, 52.0f, "Open flow images", false, Act::OpenCfdFolder);
    StatusLine(L, M, X + (HasImages ? 890.0f : 620.0f), 994.0f, HasImages ? 800.0f : 1000.0f);
}

// ----------------------------------------------------------------------------------------------------------------------------- 7 Compare
inline void Compare(DrawList& L, const AppModel& M)
{
    NavBar(L, M);
    L.Fill(0, static_cast<float>(Ui::kTopBarHeightPx), W, Hgt - static_cast<float>(Ui::kTopBarHeightPx), Palette::Report());
    const float X = 96.0f;
    L.Text(X, 84.0f, 900.0f, "Compare", 56.0f, Palette::Ink(), Weight::Light);
    const UpgradeSection& U = M.StudyReport.Upgrades;
    if (!M.Prog.StudyDone || !U.Run)
    {
        L.Text(X, 170.0f, 1500.0f, "Compare tries every compatible single-part swap (battery, ESC, motor, propeller) under the same conditions as your tests,", 20.0f, Palette::Ink70());
        L.Text(X, 200.0f, 1500.0f, "runs the campaign for each and shows the measured change next to the original. Nothing is estimated.", 20.0f, Palette::Ink70());
        L.Text(X, 250.0f, 1500.0f, "It runs about one campaign per candidate, so it can take several minutes.", 16.0f, Palette::Ink46());
        Button(L, X, 310.0f, 340.0f, 56.0f, M.Busy() ? "Working ..." : "Run upgrade study", true, Act::RunStudy, "", !M.Busy() && M.Prog.CampaignDone);
        if (!M.Prog.CampaignDone) L.Text(X + 370.0f, 328.0f, 800.0f, "Run the tests first.", 16.0f, Palette::Warn());
        StatusLine(L, M, X, 390.0f, 1500.0f);
        return;
    }
    L.Text(X, 160.0f, 1700.0f, "Condition set " + U.ConditionSet + ". Ranked by the change of the original's weakest component (" + U.BaselineWeakest + ", margin " + Fmt("%.2f", U.BaselineLimitingMargin) + ").", 18.0f, Palette::Ink70());
    L.Text(X, 188.0f, 1700.0f, U.Notice, 14.0f, M.StudyReport.Smoke() ? Palette::Warn() : Palette::Ink46());
    const char* Cols[] = {"limiting_margin", "max_payload_kg", "max_wind_mps", "endurance_s", "peak_winding_c", "mass_kg", "price_usd"};
    const float Cx0 = 560.0f, Cw = 150.0f;
    float Y = 236.0f;
    L.Text(X, Y, 60.0f, "#", 14.0f, Palette::Ink46());
    L.Text(X + 50.0f, Y, 400.0f, "Candidate", 14.0f, Palette::Ink46());
    int C = 0;
    for (const char* Metric : Cols)
    {
        std::string Label = Metric;
        for (const auto& E : U.Entries) if (const DeltaCell* D = E.Delta(Metric)) { Label = D->Label + (D->Unit.empty() ? "" : " (" + D->Unit + ")"); break; }
        L.Text(Cx0 + static_cast<float>(C) * Cw, Y - 10.0f, Cw - 8.0f, Shorten(Label, 22), 13.0f + 1.0f, Palette::Ink46(), Weight::Regular, Align::Right);
        ++C;
    }
    L.Text(Cx0 + 7.0f * Cw + 16.0f, Y, 300.0f, "Weakest after swap", 14.0f, Palette::Ink46());
    Y += 26.0f;
    int Row = 0;
    for (const auto& E : U.Entries)
    {
        if (Row++ >= 18) break;
        L.HLine(X, Y, 1728.0f, Palette::Line());
        L.Text(X, Y + 8.0f, 50.0f, E.Rank > 0 ? std::to_string(E.Rank) : std::string("-"), 18.0f, Palette::Ink70());
        L.Text(X + 50.0f, Y + 6.0f, 430.0f, Shorten(E.Role + ": " + E.PartId, 40), 18.0f, E.Rank > 0 ? Palette::Ink() : Palette::Ink46());
        if (E.Status != "tested")
            L.Text(Cx0, Y + 9.0f, 1200.0f, E.Status == "rejected" ? E.Note : (E.Status == "not_tested" ? "Not tested: " + E.Note : E.Note), 15.0f, Palette::Ink46());
        else
        {
            int K = 0;
            for (const char* Metric : Cols)
            {
                const DeltaCell* D = E.Delta(Metric);
                const Rgba Tone = !D || !D->Available() ? Palette::Ink46() : (D->Tone == DeltaTone::Better ? Palette::Ok() : (D->Tone == DeltaTone::Worse ? Palette::Crit() : Palette::Ink70()));
                L.Text(Cx0 + static_cast<float>(K) * Cw, Y + 7.0f, Cw - 8.0f, D ? DeltaText(*D) : std::string("n/a"), 18.0f, Tone, Weight::Regular, Align::Right);
                ++K;
            }
            L.Text(Cx0 + 7.0f * Cw + 16.0f, Y + 9.0f, 330.0f, E.WeakestLabel.empty() ? std::string("n/a") : E.WeakestLabel + " " + Fmt("%.2f", E.WeakestMargin), 15.0f, Palette::Ink70());
            if (E.Rank == 0 && !E.Note.empty()) L.Text(X + 50.0f, Y + 32.0f, 1400.0f, E.Note, 12.0f + 2.0f, Palette::Warn());
        }
        Y += E.Status == "tested" && E.Rank == 0 && !E.Note.empty() ? 56.0f : 42.0f;
    }
    L.HLine(X, Y, 1728.0f, Palette::Line());
    L.Text(X, Y + 12.0f, 1700.0f, "Green: better than the original. Red: worse. A gain in one place that costs another is visible in the same row.", 14.0f, Palette::Ink46());
    Button(L, X, 980.0f, 260.0f, 52.0f, "Report", true, Act::Navigate, "Report");
    StatusLine(L, M, X + 300.0f, 994.0f, 1200.0f);
}

// ----------------------------------------------------------------------------------------------------------------------------- 8 Report
inline void ReportScreen(DrawList& L, const AppModel& M)
{
    NavBar(L, M);
    L.Fill(0, static_cast<float>(Ui::kTopBarHeightPx), W, Hgt - static_cast<float>(Ui::kTopBarHeightPx), Palette::Report());
    const float X = static_cast<float>(Ui::kReportMarginXPx);
    const ReportData& R = M.Report;
    L.Text(X, 84.0f, 900.0f, "Engineer's report", 56.0f, Palette::Ink(), Weight::Light);
    if (!M.Prog.ReportDone)
    {
        L.Text(X, 170.0f, 1500.0f, M.Busy() ? M.BusyLabel() + " ..." : "No report yet. Run the tests, then write the report.", 20.0f, Palette::Ink70());
        Button(L, X, 230.0f, 300.0f, 56.0f, "Write report", true, Act::WriteReport, "", !M.Busy() && M.Prog.CampaignDone);
        StatusLine(L, M, X, 310.0f, 1500.0f);
        return;
    }
    Verdict(L, R, X, 156.0f, 1680.0f);
    float By = 250.0f;
    BannerList(L, R, X, By, 1680.0f, 6);
    const float Y0 = std::max(By + 24.0f, 360.0f);
    const float Lw = 1000.0f, Rx = X + Lw + static_cast<float>(Ui::kReportColumnGapPx) - 40.0f, Rw = W - X - Rx;
    float Y = Y0;
    L.Text(X, Y, Lw, "Summary", 15.0f, Palette::Ink70(), Weight::Medium);
    Y += 28.0f;
    for (const auto& S : R.Summary)
    {
        L.HLine(X, Y, Lw, Palette::Line());
        L.Text(X, Y + 8.0f, 460.0f, S.Label, 20.0f, Palette::Ink());
        L.Text(X + 470.0f, Y + 8.0f, 500.0f, S.ValueText, 20.0f, S.Value != S.Value && S.ValueText == "not tested" ? Palette::Ink46() : Palette::Ink(), Weight::Regular, Align::Right);
        if (!S.Detail.empty()) L.Text(X, Y + 34.0f, Lw, Shorten(S.Detail, 110), 14.0f, Palette::Ink46());
        Y += S.Detail.empty() ? 40.0f : 58.0f;
    }
    L.HLine(X, Y, Lw, Palette::Line());
    float Ry = Y0;
    L.Text(Rx, Ry, Rw, "What this report rests on", 15.0f, Palette::Ink70(), Weight::Medium);
    Ry += 28.0f;
    int Assumed = 0, Unverified = 0;
    for (const auto& A : R.Assumptions) { Assumed += A.Type == "ASSUMED"; Unverified += A.Type == "UNVERIFIED"; }
    const std::pair<std::string, std::string> Facts[] = {{"Assumptions listed", std::to_string(R.Assumptions.size())}, {"of them ASSUMED inputs", std::to_string(Assumed)},
                                                        {"UNVERIFIED items", std::to_string(Unverified)}, {"Validation rows above 10 %", std::to_string(R.FlaggedCount())},
                                                        {"Code version", R.GitHash.empty() ? std::string("unknown") : R.GitHash}};
    for (const auto& F : Facts)
    {
        L.HLine(Rx, Ry, Rw, Palette::Line());
        L.Text(Rx, Ry + 8.0f, 300.0f, F.first, 16.0f, Palette::Ink70());
        L.Text(Rx, Ry + 6.0f, Rw, Shorten(F.second, 24), 18.0f, Palette::Ink(), Weight::Regular, Align::Right);
        Ry += 38.0f;
    }
    L.HLine(Rx, Ry, Rw, Palette::Line());
    Button(L, X, 930.0f, 260.0f, 52.0f, "Open report", true, Act::OpenReport, "", !M.ReportDir.empty());
    Button(L, X + 280.0f, 930.0f, 220.0f, 52.0f, "Open folder", false, Act::OpenFolder);
    Button(L, X + 520.0f, 930.0f, 220.0f, 52.0f, "Write again", false, Act::WriteReport, "", !M.Busy());
    L.Text(X, 1000.0f, 1500.0f, "report.html (print it to PDF from the browser), report.json and CSV files are in: " + M.ReportDir, 14.0f, Palette::Ink46());
    StatusLine(L, M, X, 1030.0f, 1500.0f);
}
}  // namespace Screens
}  // namespace DroneApp
