// Tier: V
// Engine-free layout of the progress panel that sits over EVERY screen while a helper job runs (and says "Done" after it): overall bar,
// stage label, elapsed time, estimated time left (only after the first test has finished, marked as an estimate), one row per test with
// its own bar, a Cancel button, the auto-play switch, and an amber "No progress for 60 s" with a link to the log. Flat graphite, white
// values, one blue accent; amber only for caution. Percent is simulated time done over the planned simulated time, from the helpers.
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

inline void ProgressPanel(DrawList& L, const AppModel& M)
{
    const bool Show = M.Busy() ? true : (M.JobDoneOk && M.Live.Have);
    if (!Show) return;
    const ProgressLine& P = M.Live.Latest;
    const float X = 1440.0f, Y = static_cast<float>(Ui::kTopBarHeightPx) + 8.0f, Wd = 464.0f, Pad = 16.0f;
    constexpr std::size_t kMaxRows = 10;
    const bool Running = M.Busy();
    const std::size_t Rows = M.Live.Have ? std::min(P.Tests.size(), kMaxRows) : 0;
    const bool Warn = M.NoProgress();
    float H = 92.0f + static_cast<float>(Rows) * 24.0f + (Rows > 0 ? 8.0f : 0.0f) + (Warn ? 26.0f : 0.0f) + 44.0f;
    L.AddHit(X, Y, Wd, H, Act::None, "");   // swallows clicks on the panel (hits are searched last-added first)
    L.Fill(X, Y, Wd, H, Palette::Solid());
    L.HLine(X, Y, Wd, Palette::LineStrong());
    L.Text(X + Pad, Y + 12.0f, Wd - 2 * Pad - 70.0f, StageLabel(M), 17.0f, Palette::Ink(), Weight::Medium);
    const double Pct = (!Running && M.JobDoneOk) ? 100.0 : (M.Live.Have ? P.PercentOverall : 0.0);
    L.Text(X + Wd - Pad - 70.0f, Y + 12.0f, 70.0f, Fmt("%.0f", Pct) + " %", 17.0f, Palette::Ink(), Weight::Regular, Align::Right);
    const float BarY = Y + 42.0f, BarW = Wd - 2 * Pad;
    L.Fill(X + Pad, BarY, BarW, 6.0f, Palette::Line());
    L.Fill(X + Pad, BarY, BarW * static_cast<float>(Pct / 100.0), 6.0f, Palette::Accent());
    std::string T = "Elapsed " + ClockText(M.Live.Have ? P.ElapsedS : 0.0);
    if (Running && M.Live.Have && P.EtaS >= 0.0) T += "     Time left about " + ClockText(P.EtaS) + " (estimate)";
    L.Text(X + Pad, Y + 58.0f, BarW, T, 14.0f, Palette::Ink46());
    float Ry = Y + 84.0f;
    if (Warn)
    {
        L.Text(X + Pad, Ry, 300.0f, "No progress for 60 s", 14.0f, Palette::Warn(), Weight::Medium);
        L.Text(X + Wd - Pad - 120.0f, Ry, 120.0f, "Open log", 14.0f, Palette::Warn(), Weight::Regular, Align::Right);
        L.AddHit(X + Wd - Pad - 130.0f, Ry - 4.0f, 130.0f, 24.0f, Act::OpenLog, "");
        Ry += 26.0f;
    }
    for (std::size_t I = 0; I < Rows; ++I)
    {
        const ProgTest& R = P.Tests[I];
        const bool Finished = R.State == ProgState::Done;
        const bool Playable = Finished && M.Camp.Find(R.Id) != nullptr && !M.Camp.Find(R.Id)->SeriesFiles.empty();
        const char* State = R.State == ProgState::Done ? "done" : (R.State == ProgState::Running ? "running" : (R.State == ProgState::Failed ? "failed" : "waiting"));
        const Rgba Ink = R.State == ProgState::Waiting ? Palette::Ink46() : (R.State == ProgState::Failed ? Palette::Crit() : Palette::Ink());
        L.Text(X + Pad, Ry, 210.0f, Shorten(TestLabelOf(R.Id), 26), 14.0f, Ink, Weight::Regular);
        L.Text(X + Pad + 214.0f, Ry + 1.0f, 70.0f, State, 14.0f, R.State == ProgState::Failed ? Palette::Crit() : Palette::Ink46(), Weight::Regular);
        const float Bx = X + Pad + 290.0f, Bw = BarW - 290.0f - 44.0f;
        L.Fill(Bx, Ry + 8.0f, Bw, 4.0f, Palette::Line());
        L.Fill(Bx, Ry + 8.0f, Bw * static_cast<float>(R.Percent / 100.0), 4.0f, R.State == ProgState::Failed ? Palette::Crit() : (Finished ? Palette::Ink70() : Palette::Accent()));
        L.Text(X + Wd - Pad - 40.0f, Ry + 1.0f, 40.0f, Fmt("%.0f", R.Percent), 14.0f, Palette::Ink46(), Weight::Regular, Align::Right);
        if (Playable) L.AddHit(X + Pad, Ry - 2.0f, BarW, 24.0f, Act::PickTest, R.Id);   // a finished test opens its recording
        Ry += 24.0f;
    }
    if (P.Tests.size() > Rows) L.Text(X + Pad, Ry, BarW, "+ " + std::to_string(P.Tests.size() - Rows) + " more", 14.0f, Palette::Ink46());
    // footer: auto-play switch and Cancel
    const float Fy = Y + H - 38.0f;
    if (M.Prog.RecordingsAvailable)
    {
        L.Text(X + Pad, Fy + 6.0f, 220.0f, std::string("Auto-play ") + (M.AutoPlay ? "on" : "off"), 14.0f, M.AutoPlay ? Palette::Ink() : Palette::Ink46(), Weight::Regular);
        L.AddHit(X + Pad - 4.0f, Fy, 160.0f, 30.0f, Act::ToggleAutoPlay, "");
    }
    if (Running) Button(L, X + Wd - Pad - 110.0f, Fy, 110.0f, 30.0f, "Cancel", false, Act::CancelJob);
}
}  // namespace Screens
}  // namespace DroneApp
