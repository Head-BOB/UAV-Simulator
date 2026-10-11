// Tier: V
// Engine-free app flow: the eight screens in workflow order and the rules for moving between them. A screen that needs earlier work
// is not reachable until that work exists (and the reason is given), so nobody runs tests on a vehicle that was never built.
#pragma once

#include <string>

namespace DroneApp
{
enum class Screen { Vehicle = 0, Parts, Build, Conditions, Run, Results, Report };
constexpr int kScreenCount = 7;

inline const char* ScreenName(Screen S)
{
    static const char* Names[kScreenCount] = {"Vehicle", "Parts", "Build", "Conditions", "Run", "Results", "Report"};
    const int I = static_cast<int>(S);
    return (I >= 0 && I < kScreenCount) ? Names[I] : "";
}

// What exists so far in this session.
struct Progress
{
    bool VehicleClassChosen = false;
    bool PartsComplete = false;     // all required roles chosen
    bool BuildDone = false;         // the builder accepted the parts (no errors) and its files were read
    bool ConditionsValid = false;   // typed values validate and at least one implemented test is selected
    bool CampaignDone = false;      // a campaign folder was produced and loaded
    bool ReportDone = false;        // report.json exists and was read
    bool StudyDone = false;         // an upgrade study report exists
    bool RecordingsAvailable = false;   // at least one test of a running or cancelled campaign has a recording (Run screen only)
};

// Empty string when the screen can be opened; otherwise the reason (for the greyed navigation entry).
inline std::string BlockedReason(Screen S, const Progress& P)
{
    switch (S)
    {
        case Screen::Vehicle: return "";
        case Screen::Parts: return P.VehicleClassChosen ? "" : "Choose a vehicle first.";
        case Screen::Build: return P.PartsComplete ? "" : "Choose all required parts first (frame, motor, ESC, propeller, battery).";
        case Screen::Conditions: return P.BuildDone ? "" : "Build the vehicle first.";
        case Screen::Run: return (P.CampaignDone || P.RecordingsAvailable) ? "" : "Run the tests first.";
        case Screen::Results: return P.CampaignDone ? "" : "Run the tests first.";
        case Screen::Report: return P.ReportDone ? "" : "Run the tests first, then write the report.";
    }
    return "Unknown screen.";
}
inline bool CanOpen(Screen S, const Progress& P) { return BlockedReason(S, P).empty(); }

// Next screen in workflow order that can be opened, or the same screen when none can (Next at the end stays).
inline Screen Next(Screen S, const Progress& P)
{
    for (int I = static_cast<int>(S) + 1; I < kScreenCount; ++I)
        if (CanOpen(static_cast<Screen>(I), P)) return static_cast<Screen>(I);
    return S;
}
inline Screen Previous(Screen S)
{
    return static_cast<int>(S) > 0 ? static_cast<Screen>(static_cast<int>(S) - 1) : S;
}

// Going back to an earlier step invalidates what was derived from it. Returns the progress that still holds.
inline Progress AfterChange(Screen Changed, Progress P)
{
    switch (Changed)
    {
        case Screen::Vehicle: P.PartsComplete = false; [[fallthrough]];
        case Screen::Parts: P.BuildDone = false; [[fallthrough]];
        case Screen::Build: P.ConditionsValid = false; P.CampaignDone = false; P.ReportDone = false; P.StudyDone = false; P.RecordingsAvailable = false; break;
        case Screen::Conditions: P.CampaignDone = false; P.ReportDone = false; P.StudyDone = false; P.RecordingsAvailable = false; break;   // new conditions: old results do not describe them
        default: break;
    }
    return P;
}

// One modal (a file dialog, a running helper) at a time; input to the screens is ignored while it is open.
class ModalGate
{
public:
    bool Open(const std::string& What) { if (Active) return false; Active = true; Name = What; return true; }
    void Close() { Active = false; Name.clear(); }
    bool IsOpen() const { return Active; }
    const std::string& What() const { return Name; }
private:
    bool Active = false;
    std::string Name;
};
}  // namespace DroneApp
