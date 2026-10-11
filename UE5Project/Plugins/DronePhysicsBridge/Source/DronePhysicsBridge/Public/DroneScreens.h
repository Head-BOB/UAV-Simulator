// Tier: V
// Entry point of the engine-free UI: builds the display list of the current screen, and turns a click on it into a model action.
#pragma once

#include "DroneScreensFly.h"
#include "DroneScreensProgress.h"
#include "DroneScreensReport.h"
#include "DroneScreensRun.h"
#include "DroneScreensSetup.h"

namespace DroneApp
{
// Fills L with the current screen. Design space coordinates (1920 x 1080).
inline void BuildScreen(DrawList& L, const AppModel& M)
{
    L.Clear();
    if (M.FlyMode)
    {
        Screens::FlyScreen(L, M);
        Screens::ProgressPanel(L, M);
        return;
    }
    switch (M.Current)
    {
        case Screen::Vehicle: Screens::Vehicle(L, M); break;
        case Screen::Parts: Screens::Parts(L, M); break;
        case Screen::Build: Screens::Build(L, M); break;
        case Screen::Conditions: Screens::Conditions(L, M); break;
        case Screen::Run: Screens::Run(L, M); break;
        case Screen::Results: Screens::Results(L, M); break;
        case Screen::Report: Screens::ReportScreen(L, M); break;
    }
    Screens::ProgressPanel(L, M);   // over every screen, drawn last so it is on top and its hit regions win
}

// A click at design-space (X, Y). Returns true when it hit something. A click on the seek bar passes its fraction (0..1) to the model.
inline bool HandleClick(AppModel& M, const DrawList& L, float X, float Y)
{
    const Hit* H = L.HitAt(X, Y);
    if (H == nullptr)
    {
        M.Click(Act::ClearFocus, "");
        return false;
    }
    if (H->A == Act::SeekBar)
    {
        const float F = H->W > 0.0f ? (X - H->X) / H->W : 0.0f;
        M.Click(Act::SeekBar, std::to_string(F < 0.0f ? 0.0 : (F > 1.0f ? 1.0 : static_cast<double>(F))));
        return true;
    }
    M.Click(H->A, H->Arg);
    return true;
}
}  // namespace DroneApp
