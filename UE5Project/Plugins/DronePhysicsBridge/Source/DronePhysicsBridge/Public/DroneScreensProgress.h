// Tier: V
// Engine-free layout of the progress indicator: a compact strip in the right side of the NavBar (56 px header), never floating over
// content. Clicking the strip expands a drop-down detail panel (below the bar); clicking outside collapses it. Auto-collapses ~4 s
// after the job finishes (see AppModel::Tick). Elapsed shown is the real wall-clock time (JobElapsedS), not the helper-reported value
// (which is sometimes 0 at the end). ETA shown after the first test, marked as estimate. Amber "No progress 60 s" + log link.
// State machine: idle (hidden) -> working (strip) -> done (chip, 4 s) -> collapsed.
// g++ -tested: StageLabel, strip layout, drop-down content.
// NOT COMPILED OR RUN by its author. TODO(HUMAN-VERIFY: Slate repaint on expand/collapse; hit region inside the NavBar overlap).
#pragma once

#include <string>

#include "DroneAppModel.h"
#include "DroneScreensSetup.h"

namespace DroneApp
{
namespace Screens
{
inline std::string TestLabelOf(const std::string& Id)
{
    for (const auto& T : TestList())
        if (Id == T.Id) return T.Label;
    return Id;
}

// "Test 3 of 8: Climb" / "Upgrade study: campaign 2 of 9" / "Writing report" / "Done".
inline std::string StageLabel(const AppModel& M)
{
    const ProgressLine& P = M.Live.Latest;
    if (!M.Busy()) return M.JobDoneOk ? "Done" : "";
    if (!M.Live.Have) return M.BusyLabel() + " ...";
    switch (P.Stage)
    {
        case ProgStage::Test: return "Test " + std::to_string(P.TestIndex + 1) + " of " + std::to_string(std::max(P.TestCount, 1)) + ": " + TestLabelOf(P.TestId);
        case ProgStage::Study: return "Upgrade study: campaign " + std::to_string(P.TestIndex + 1) + " of " + std::to_string(std::max(P.TestCount, 1));
        case ProgStage::Report: return "Writing report";
        case ProgStage::Preparing: return "Preparing ...";
        case ProgStage::Failed: return "Failed";
        case ProgStage::Done:
        case ProgStage::Unknown: break;
    }
    return M.BusyLabel() + " ...";
}

// Compact progress strip in the right portion of the NavBar (drawn AFTER NavBar fills the bar). Returns the X at which the strip starts.
// The strip is a narrow label + percent + bar, right-aligned, with a dismiss X if Done and a collapse/expand chevron.
// The drop-down panel is drawn below the bar and swallows clicks (so it does not interact with the scene).
inline void ProgressPanel(DrawList& L, const AppModel& M)
{
    if (!M.ShowProgressStrip()) return;

    const bool Running = M.Busy();
    const ProgressLine& P = M.Live.Latest;
    const double Pct = (!Running && M.JobDoneOk) ? 100.0 : (M.Live.Have ? P.PercentOverall : 0.0);
    const double ElapsedS = M.JobElapsedS;   // real wall-clock, never 0:00 after a real run

    // ---------- compact strip (inside the 56 px NavBar, right side) ----------
    // Layout from right: [X dismiss] [stage text + pct + elapsed] [small progress bar]
    constexpr float RightEdge = static_cast<float>(Ui::kDesignWidthPx) - 24.0f;
    constexpr float StripH = static_cast<float>(Ui::kTopBarHeightPx);
    const std::string Stage = StageLabel(M);
    const std::string PctStr = (!Running && M.JobDoneOk) ? "Done" : (Fmt("%.0f", Pct) + " %");
    const std::string ElStr = "  " + ClockText(ElapsedS);
    std::string EtaStr;
    if (Running && M.Live.Have && P.EtaS >= 0.0)
        EtaStr = "   left about " + ClockText(P.EtaS) + " (estimate)";
    const std::string Full = Stage + "  " + PctStr + ElStr + EtaStr;

    const float StageW = EstimateTextWidth(Full, 14.0f);
    const float BarW = 80.0f;
    const float PadX = 8.0f;
    // The dismiss X (shown when Done chip visible)
    const float XbtnW = (M.JobDoneOk && !Running) ? 20.0f : 0.0f;
    const float TotalW = XbtnW + PadX + StageW + PadX + BarW + PadX;
    const float StripX = RightEdge - TotalW;

    // Strip warning for no progress 60s (docked next to strip)
    const bool Warn = M.NoProgress();
    if (Warn)
    {
        const float WarnW = 210.0f;
        const float Wx = StripX - WarnW - PadX;
        L.Text(Wx, StripH * 0.5f - 7.0f, 130.0f, "No progress for 60 s", 14.0f, Palette::Warn(), Weight::Medium);
        L.Text(Wx + 135.0f, StripH * 0.5f - 7.0f, 65.0f, "Open log", 14.0f, Palette::Warn(), Weight::Regular);
        L.AddHit(Wx + 130.0f, 0.0f, 75.0f, StripH, Act::OpenLog, "");
    }

    // Strip hit: click -> toggle expand (when running); dismiss (when Done)
    const Rgba StripC = (M.JobDoneOk && !Running) ? Palette::Accent() : Palette::Ink();
    L.Text(StripX, StripH * 0.5f - 7.0f, StageW + PadX, Full, 14.0f, StripC, Weight::Regular);

    // Mini bar
    const float Bx = StripX + StageW + PadX;
    L.Fill(Bx, StripH * 0.5f - 2.0f, BarW, 4.0f, Palette::Line());
    if (Pct > 0.0)
        L.Fill(Bx, StripH * 0.5f - 2.0f, BarW * static_cast<float>(Pct / 100.0), 4.0f,
               (M.JobDoneOk && !Running) ? Palette::Accent() : Palette::Accent());

    // Dismiss X (Done only)
    if (XbtnW > 0.0f)
    {
        const float Xx = Bx + BarW + PadX;
        L.Text(Xx, StripH * 0.5f - 7.0f, XbtnW, "x", 14.0f, Palette::Ink70(), Weight::Regular);
        L.AddHit(Xx - 4.0f, 0.0f, XbtnW + 8.0f, StripH, Act::DismissProgress, "");
    }

    // Toggle expand on the strip itself (click anywhere not on the X)
    if (Running)
        L.AddHit(StripX - PadX, 0.0f, StageW + PadX + BarW + 2.0f * PadX, StripH, Act::ToggleProgressExpanded, "");

    // ---------- expanded drop-down (below the NavBar) ----------
    if (!M.ProgressExpanded || !Running) return;
    const std::size_t kMaxRows = 10;
    const std::size_t Rows = M.Live.Have ? std::min(P.Tests.size(), kMaxRows) : 0;
    const float PanelW = 480.0f;
    float PanelH = 80.0f + static_cast<float>(Rows) * 24.0f + (Rows > 0 ? 8.0f : 0.0f) + (Warn ? 26.0f : 0.0f) + 44.0f;
    const float Px = static_cast<float>(Ui::kDesignWidthPx) - PanelW - 24.0f;
    const float Py = StripH;
    const float Pad = 16.0f;

    L.AddHit(Px, Py, PanelW, PanelH, Act::None, "");   // swallow clicks on the panel
    L.Fill(Px, Py, PanelW, PanelH, Palette::Solid());
    L.HLine(Px, Py, PanelW, Palette::LineStrong());

    float Ry = Py + 12.0f;
    L.Text(Px + Pad, Ry, PanelW - 2.0f * Pad - 70.0f, Stage, 14.0f, Palette::Ink(), Weight::Medium);
    L.Text(Px + PanelW - Pad - 70.0f, Ry, 70.0f, Fmt("%.0f", Pct) + " %", 14.0f, Palette::Ink(), Weight::Regular, Align::Right);
    Ry += 22.0f;

    const float BarW2 = PanelW - 2.0f * Pad;
    L.Fill(Px + Pad, Ry, BarW2, 5.0f, Palette::Line());
    L.Fill(Px + Pad, Ry, BarW2 * static_cast<float>(Pct / 100.0), 5.0f, Palette::Accent());
    Ry += 14.0f;

    std::string T = "Elapsed " + ClockText(ElapsedS);
    if (M.Live.Have && P.EtaS >= 0.0) T += "     Time left about " + ClockText(P.EtaS) + " (estimate)";
    L.Text(Px + Pad, Ry, BarW2, T, 14.0f, Palette::Ink46());
    Ry += 18.0f;

    if (Warn)
    {
        L.Text(Px + Pad, Ry, 300.0f, "No progress for 60 s", 14.0f, Palette::Warn(), Weight::Medium);
        L.Text(Px + PanelW - Pad - 70.0f, Ry, 70.0f, "Open log", 14.0f, Palette::Warn(), Weight::Regular, Align::Right);
        L.AddHit(Px + PanelW - Pad - 80.0f, Ry - 4.0f, 80.0f, 22.0f, Act::OpenLog, "");
        Ry += 26.0f;
    }

    for (std::size_t I = 0; I < Rows; ++I)
    {
        const ProgTest& R = P.Tests[I];
        const bool Finished = R.State == ProgState::Done;
        const bool Playable = Finished && M.Camp.Find(R.Id) != nullptr && !M.Camp.Find(R.Id)->SeriesFiles.empty();
        const char* State = R.State == ProgState::Done ? "done" : (R.State == ProgState::Running ? "running" : (R.State == ProgState::Failed ? "failed" : "waiting"));
        const Rgba Ink = R.State == ProgState::Waiting ? Palette::Ink46() : (R.State == ProgState::Failed ? Palette::Crit() : Palette::Ink());
        L.Text(Px + Pad, Ry, 200.0f, Shorten(TestLabelOf(R.Id), 24), 14.0f, Ink, Weight::Regular);
        L.Text(Px + Pad + 204.0f, Ry, 60.0f, State, 14.0f, R.State == ProgState::Failed ? Palette::Crit() : Palette::Ink46(), Weight::Regular);
        const float Bx2 = Px + Pad + 270.0f, Bw2 = BarW2 - 270.0f - 40.0f;
        L.Fill(Bx2, Ry + 7.0f, Bw2, 3.0f, Palette::Line());
        L.Fill(Bx2, Ry + 7.0f, Bw2 * static_cast<float>(R.Percent / 100.0), 3.0f, R.State == ProgState::Failed ? Palette::Crit() : (Finished ? Palette::Ink70() : Palette::Accent()));
        if (Playable) L.AddHit(Px + Pad, Ry - 2.0f, BarW2, 22.0f, Act::PickTest, R.Id);
        Ry += 22.0f;
    }
    if (P.Tests.size() > Rows) L.Text(Px + Pad, Ry, BarW2, "+ " + std::to_string(P.Tests.size() - Rows) + " more", 14.0f, Palette::Ink46());

    // footer: auto-play + cancel
    const float Fy = Py + PanelH - 36.0f;
    if (M.Prog.RecordingsAvailable)
    {
        L.Text(Px + Pad, Fy + 4.0f, 200.0f, std::string("Auto-play ") + (M.AutoPlay ? "on" : "off"), 14.0f, M.AutoPlay ? Palette::Ink() : Palette::Ink46());
        L.AddHit(Px + Pad - 4.0f, Fy, 140.0f, 28.0f, Act::ToggleAutoPlay, "");
    }
    if (Running)
        Button(L, Px + PanelW - Pad - 100.0f, Fy, 100.0f, 28.0f, "Cancel", false, Act::CancelJob);
}
}  // namespace Screens
}  // namespace DroneApp
