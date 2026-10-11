// Tier: V
// Engine-free application model: all state and behaviour of the eight screens (selection, build, conditions, helper jobs, replay,
// report), driven by clicks / keys / ticks. Files and processes go through the AppIO interface, so the Unreal layer supplies the
// engine implementation and tools/cpp_ffi_check/app_flow_test.cpp supplies a std implementation that runs the REAL helpers
// (build_vehicle.py, run_campaign, make_report.py) end to end. Nothing here draws; DroneScreens*.h turns the model into a DrawList.
#pragma once

#include <algorithm>
#include <cmath>
#include <deque>
#include <map>
#include <set>
#include <string>
#include <vector>

#include "DroneAppState.h"
#include "DroneBuildFiles.h"
#include "DroneCampaignData.h"
#include "DroneCatalogue.h"
#include "DroneConditions.h"
#include "DroneDrawList.h"
#include "DroneHelpers.h"
#include "DroneMargins.h"
#include "DroneFly.h"
#include "DroneProgress.h"
#include "DroneReplay.h"
#include "DroneReportData.h"

namespace DroneApp
{
// File and process access supplied by the host (engine or test).
struct AppIO
{
    virtual ~AppIO() = default;
    virtual bool ReadText(const std::string& Path, std::string& Out) = 0;
    virtual bool WriteText(const std::string& Path, const std::string& Text) = 0;   // creates parent folders
    virtual bool Exists(const std::string& Path) = 0;
    // Starts a program with its output (stdout + stderr) going to InLogPath. Returns a handle > 0, or -1 when it could not be started.
    virtual int StartProcess(const Command& C, const std::string& InLogPath) = 0;
    // True when the process has ended (ExitCode set); false while it runs.
    virtual bool PollProcess(int Handle, int& ExitCode) = 0;
    virtual void OpenPath(const std::string& Path) = 0;   // folder or file in the system viewer
    // Optional (defaults: no progress, nothing to cancel). The host parses the running process's output with ProgressParser as it
    // arrives and returns the snapshot; CancelProcess kills the process (and its children). A cancelled process ends with a non-zero code.
    virtual bool ReadProgress(int Handle, ProgressSnapshot& Out) { (void)Handle; (void)Out; return false; }
    virtual void CancelProcess(int Handle) { (void)Handle; }
};

struct Step
{
    Command Cmd;
    Helper Kind = Helper::Build;
    std::string Log;
    int Handle = -1;
};

class AppModel
{
public:
    // ---- construction ------------------------------------------------------------------------------------------------------------
    AppModel(AppIO& InIO, HelperSettings InHelpers, std::string InWorkDir) : IO(InIO), H(std::move(InHelpers)), Work(std::move(InWorkDir)) {}

    std::string DefinitionsRel = "campaign/test_definitions.json";   // the test definitions file, relative to the repo root

    // Loads the catalogue and the test definitions. False with Message set when they are missing or malformed.
    bool Init()
    {
        std::string Text, Err;
        if (!IO.ReadText(Join(H.RepoRoot, "parts_db/parts_catalogue.json"), Text) || !Cat.Load(Text, Err)) { Message = "Parts catalogue unavailable: " + (Err.empty() ? std::string("file not found") : Err); return false; }
        if (!IO.ReadText(Join(H.RepoRoot, DefinitionsRel), DefinitionsText)) { Message = "Test definitions not found (" + DefinitionsRel + ")."; return false; }
        const Json::ParseResult D = Json::Parse(DefinitionsText);
        if (!D.ok) { Message = "Test definitions: " + D.error; return false; }
        for (const auto& T : TestList())
        {
            if (!T.Implemented) continue;
            for (auto& E : EditableLimits(D.value, T.Id)) { LimitText.push_back(NumText(E.Original)); Limits.push_back(E); }
            SelectedTests.insert(T.Id);
        }
        LimitTest = "hover_payload";
        Message.clear();
        return true;
    }

    // ---- state read by the screens -----------------------------------------------------------------------------------------------
    AppIO& IO;
    HelperSettings H;
    std::string Work;
    Screen Current = Screen::Vehicle;
    Progress Prog;
    std::string Message;                 // status line (errors and progress)
    bool MessageIsError = false;

    // Vehicle / Parts
    std::string VehicleClass;
    Catalogue Cat;
    Selection Sel;
    std::string CategoryFilter = "frame";
    std::string ManufacturerFilter;
    std::string Search;
    std::string SelectedPartId;
    int ListScroll = 0;

    // Build
    std::string BuildName = "my_build";
    BuildSummary Summary;
    std::vector<Finding> BuildFindings;
    std::vector<Primitive> Prims;
    std::string BuildDir, SpecPath;
    bool BuildAttempted = false;

    // Conditions
    ConditionsText Cond;
    std::set<std::string> SelectedTests;
    std::string DefinitionsText;
    std::vector<EditableLimit> Limits;
    std::vector<std::string> LimitText;
    std::string LimitTest;
    std::string ConditionsPath, DefinitionsPath;

    // Run (replay of the recorded campaign)
    Campaign Camp;
    std::string CampaignDir, ReplayTest;
    std::string CfdLoadError;            // why the CFD model of this build could not be used (empty = no such failure); shown amber on Conditions
    Replay Rep;
    Json::Value Powertrain;
    std::vector<MarginRow> PeakMargins;
    std::vector<MarginRow> LiveRows;
    Pose Now;
    bool ShowDetails = false, ShowControls = false;
    double RpmScale = 1.0;   // largest rpm in the loaded series, for the motor bars
    bool EditedLimits = false;

    // Results / Compare / Report
    ReportData Report;
    ReportData StudyReport;
    std::string ReportDir, StudyDir, StudyReportDir;

    // Input focus: "" or a field id ("search", "buildname", "cond:<id>", "limit:<index>").
    std::string Focus;

    // Jobs
    std::deque<Step> Queue;
    bool Busy() const { return !Queue.empty(); }

    // Progress of the running job, from the helper's own UAVPROGRESS lines (simulated time done over the planned simulated time).
    // Fly mode (the live physics pawn flies; the UE layer fills FlyT each frame and reads WindP / FlyModeSel).
    struct FlyTelemetry
    {
        double Rpm[4] = {0.0, 0.0, 0.0, 0.0};   // true per-motor rpm, order FR FL RR RL
        double AltitudeM = 0.0, SpeedMps = 0.0, RpmMean = 0.0, SocFraction = 1.0, GustMps = 0.0;
        double WindNed[3] = {0.0, 0.0, 0.0};   // the wind the physics step received
        bool Crashed = false;
        std::string StateText;                 // ON GROUND / FLYING / ...
    };
    bool FlyMode = false;
    Fly::Mode FlyModeSel = Fly::Mode::Assisted;
    Fly::WindPanel WindP;
    FlyTelemetry FlyT;
    ProgressSnapshot Live;
    double SinceProgressS = 0.0;              // real seconds since the helper last said anything
    double JobElapsedS = 0.0;                 // real wall-clock seconds since the current job chain started (reset on each new chain)
    double JobDoneSeenS = 0.0;               // real seconds since JobDoneOk became true (for auto-collapse after ~4 s)
    bool ProgressExpanded = false;           // true = drop-down detail panel open; toggles on click of the compact strip
    std::vector<std::string> Recorded;        // tests whose recording is on disk, in the order they finished
    std::set<std::string> Played;             // recordings the auto-play has already shown
    bool AutoPlay = true;
    bool QuickProfile = false;                // chosen on Conditions: shortened long tests (QuickOverrides)
    bool RunWasQuick = false;                 // the profile of the run on screen (the choice may change afterwards)
    bool Cancelled = false;
    // Repaint request: the Slate canvas repaints on input only, so every state change that is not an input (a helper ended, said
    // something, the message changed, a job started) sets this and the HUD takes it with TakeRepaint() every tick.
    bool Repaint = true;
    bool TakeRepaint() { const bool R = Repaint; Repaint = false; return R; }
    bool JobDoneOk = false;                   // the last job chain ended without error: the HUD says "Done" chip
    bool JobDoneDismissed = false;            // the Done chip was explicitly dismissed (X clicked or auto-collapsed)
    bool AutoNavigated = false;
    std::string LastLog;
    // The helper has said nothing for a minute while it was expected to (campaign, study, report).
    bool NoProgress() const
    {
        if (Queue.empty() || Queue.front().Handle < 0 || Queue.front().Kind == Helper::Build || Queue.front().Kind == Helper::Validation) return false;
        return SinceProgressS >= 60.0;
    }
    std::string RunningLog() const { return Queue.empty() ? LastLog : Queue.front().Log; }
    std::string BusyLabel() const { return Queue.empty() ? std::string() : Queue.front().Cmd.Description; }
    // True while the progress strip should be visible in the header.
    bool ShowProgressStrip() const { return Busy() || (JobDoneOk && !JobDoneDismissed); }

    // ---- derived helpers ---------------------------------------------------------------------------------------------------------
    std::string FrameClassFilter() const { return CategoryFilter == "frame" ? VehicleClass : std::string(); }
    std::vector<const Part*> VisibleParts() const { return Cat.Filter(CategoryFilter, Search, ManufacturerFilter, VehicleClass); }
    const Part* SelectedPart() const { return SelectedPartId.empty() ? nullptr : Cat.Find(SelectedPartId); }
    Totals PartTotals() const { return Sel.Sum(Cat); }
    bool ConditionsOk(std::string& Err) const { ConditionsValid V; return ValidateConditions(Cond, V, Err) && !TestsArgument(std::vector<std::string>(SelectedTests.begin(), SelectedTests.end())).empty(); }

    // ---- actions -----------------------------------------------------------------------------------------------------------------
    void Say(const std::string& Text, bool IsError = false) { Message = Text; MessageIsError = IsError; }

    void Navigate(Screen S)
    {
        const std::string Why = BlockedReason(S, Prog);
        if (!Why.empty()) { Say(Why, true); return; }
        Current = S;
        ListScroll = 0;   // a scrolled list starts at the top again on a new screen
        Focus.clear();
        Repaint = true;
        if (S == Screen::Run && Rep.TimeS() == 0.0) RefreshLive();
        Message.clear();
    }

    void Click(Act A, const std::string& Arg)
    {
        if (A != Act::FocusField) CommitFocus();
        switch (A)
        {
            case Act::Navigate: for (int I = 0; I < kScreenCount; ++I) if (Arg == ScreenName(static_cast<Screen>(I))) Navigate(static_cast<Screen>(I)); break;
            case Act::NavNext: Navigate(Next(Current, Prog)); break;
            case Act::NavPrev: Navigate(Previous(Current)); break;
            case Act::PickVehicle: PickVehicle(Arg); break;
            case Act::PickCategory: if (!IsVehiclePartCategory(Arg)) { Say("A payload is a test condition, not a part: set it on the Conditions screen.", true); break; }
                CategoryFilter = Arg; ManufacturerFilter.clear(); SelectedPartId.clear(); ListScroll = 0; break;
            case Act::PickManufacturer: ManufacturerFilter = (ManufacturerFilter == Arg) ? std::string() : Arg; ListScroll = 0; break;
            case Act::PickPart: SelectedPartId = Arg; break;
            case Act::AddPart: AddSelectedPart(); break;
            case Act::RemovePart: Sel.Clear(Arg); AfterPartsChange(); break;
            case Act::ScrollList: ListScroll = std::max(0, ListScroll + std::atoi(Arg.c_str())); break;
            case Act::FocusField: if (Focus != Arg) CommitFocus(); Focus = Arg; break;   // moving to another field commits the one left
            case Act::ClearFocus: Focus.clear(); break;
            case Act::BuildNow: StartBuild(); break;
            case Act::UseBuild: Navigate(Screen::Conditions); break;
            case Act::ToggleTest: ToggleTest(Arg); break;
            case Act::ResetLimits: ResetLimits(); break;
            case Act::SetTurbulence: if (Arg == "none" || Arg == "dryden" || Arg == "simple") Cond.Turbulence = Arg; break;
            case Act::RunTests: StartRun(false); break;
            case Act::RunAnalyticFallback: StartRun(true); break;
            case Act::OpenCfdFolder: if (!H.CfdDir.empty()) IO.OpenPath(H.CfdDir); break;
            case Act::CancelJob: CancelRunning(); break;
            case Act::OpenLog: { const std::string L = RunningLog(); if (!L.empty()) IO.OpenPath(L); break; }
            case Act::SetProfile: if (!Busy() && (Arg == "quick" || Arg == "full")) { QuickProfile = Arg == "quick"; Prog = AfterChange(Screen::Conditions, Prog); } break;
            case Act::ToggleAutoPlay: AutoPlay = !AutoPlay; break;
            case Act::ToggleProgressExpanded: ProgressExpanded = !ProgressExpanded; break;
            case Act::DismissProgress: JobDoneDismissed = true; ProgressExpanded = false; break;
            case Act::FlyIt: if (!Prog.VehicleClassChosen) { Say("Choose a vehicle first.", true); break; } FlyMode = true; FlyModeSel = Fly::Mode::Assisted; Focus.clear(); break;
            case Act::ExitFly: FlyMode = false; Focus.clear(); break;
            case Act::SetFlyMode: if (Arg == "assisted") FlyModeSel = Fly::Mode::Assisted; else if (Arg == "acro" && Fly::ModeAvailable(VehicleClass, Fly::Mode::Acro)) FlyModeSel = Fly::Mode::Acro; break;
            case Act::GustNow: WindP.GustNow(); break;
            case Act::ToggleRandomise: WindP.Randomise = !WindP.Randomise; break;
            case Act::PlayPause: Rep.Toggle(); break;
            case Act::Faster: Rep.FasterStep(); break;
            case Act::Slower: Rep.SlowerStep(); break;
            case Act::Restart: Rep.Seek(Rep.Start()); RefreshLive(); break;
            case Act::SeekBar: Rep.SeekFraction(std::atof(Arg.c_str())); RefreshLive(); break;
            case Act::PickTest: SelectReplayTest(Arg); Played.insert(Arg); if (Camp.Find(Arg) != nullptr) Rep.Play(); break;
            case Act::ToggleDetails: ShowDetails = !ShowDetails; break;
            case Act::ToggleControls: ShowControls = !ShowControls; break;
            case Act::WriteReport: StartReport(); break;
            case Act::OpenReport: if (!ReportDir.empty()) IO.OpenPath(Join(ReportDir, "report.html")); break;
            case Act::OpenFolder: if (!Work.empty()) IO.OpenPath(Work); break;
            case Act::RunStudy: StartStudy(); break;
            case Act::Dismiss: Message.clear(); break;
            case Act::None: break;
        }
    }

    // Typed text goes to the focused field only; printable ASCII (the app has no IME support yet).
    void KeyChar(char C)
    {
        if (Focus.empty() || C < 0x20 || C > 0x7E) return;
        if (std::string* F = FieldRef(Focus)) { if (F->size() < 64) F->push_back(C); AfterFieldEdit(); }
    }
    void KeyBackspace() { if (!Focus.empty()) if (std::string* F = FieldRef(Focus)) { if (!F->empty()) F->pop_back(); AfterFieldEdit(); } }
    void KeyEnter() { CommitFocus(); Focus.clear(); }
    void KeyEscape()
    {
        if (!Focus.empty()) Focus.clear();
        else if (!Message.empty()) Message.clear();
        else if (Current == Screen::Run) Navigate(Screen::Results);
    }

    // Named keys outside a text field (Run screen shortcuts).
    void KeyShortcut(const std::string& Name)
    {
        if (!Focus.empty()) return;
        if (Current != Screen::Run) return;
        if (Name == "I") ShowDetails = !ShowDetails;
        else if (Name == "Tab") ShowControls = !ShowControls;
        else if (Name == "Space") Rep.Toggle();
        else if (Name == "Equals" || Name == "Plus") Rep.FasterStep();
        else if (Name == "Minus") Rep.SlowerStep();
        else if (Name == "Home") { Rep.Seek(Rep.Start()); RefreshLive(); }
        else if (Name == "Left") { Rep.Seek(Rep.TimeS() - 5.0); RefreshLive(); }
        else if (Name == "Right") { Rep.Seek(Rep.TimeS() + 5.0); RefreshLive(); }
    }

    // Real-time tick: polls the running helper job and advances the replay.
    void Tick(double RealDt)
    {
        if (FlyMode) WindP.Advance(RealDt);
        const bool Was = Busy();
        const unsigned LinesBefore = Live.Lines;
        const std::string MessageBefore = Message;
        PollJobs();
        if (Was != Busy() || LinesBefore != Live.Lines || MessageBefore != Message || Busy()) Repaint = true;
        if (Busy() && Queue.front().Handle >= 0) { SinceProgressS += RealDt; JobElapsedS += RealDt; }
        // Auto-collapse the Done chip after ~4 real seconds.
        if (!Busy() && JobDoneOk && !JobDoneDismissed)
        {
            JobDoneSeenS += RealDt;
            if (JobDoneSeenS >= 4.0) { JobDoneDismissed = true; ProgressExpanded = false; Repaint = true; }
            else Repaint = true;   // repaint each tick so the "Done" chip stays visible
        }
        if (Was || Busy()) AutoPlayStep();
        if (Current == Screen::Run && Rep.Playing()) { Rep.Advance(RealDt); RefreshLive(); }
    }

    // ---- replay ------------------------------------------------------------------------------------------------------------------
    void RefreshLive()
    {
        if (ReplayTest.empty()) return;
        Now = Rep.Sample();
        LiveRows = LiveMargins(Now, Powertrain, Camp.Ambient());
    }

    void SelectReplayTest(const std::string& Name)
    {
        const TestData* T = Camp.Find(Name);
        if (T == nullptr || T->SeriesFiles.empty()) { Say("That test has no recorded series.", true); return; }
        std::string Err;
        if (!Rep.Load(T->SeriesFiles[0].second, Err)) { Say("Cannot replay " + Name + ": " + Err, true); return; }
        Rep.SetAutoSpeed();   // pick replay speed so test plays in ~60-90 s of real time
        ReplayTest = Name;
        RpmScale = 1.0;
        for (const char* C : {"rpm_1", "rpm_2", "rpm_3", "rpm_4"})
            if (const std::vector<double>* Col = T->SeriesFiles[0].second.Col(C))
                for (const double V : *Col) RpmScale = std::max(RpmScale, V);
        RefreshLive();
    }

    // ---- field access ------------------------------------------------------------------------------------------------------------
    std::string* FieldRef(const std::string& Id)
    {
        if (Id == "search") return &Search;
        if (Id == "buildname") return &BuildName;
        if (Id == "cond:name") return &Cond.Name;
        if (Id == "fly:speed") return &WindP.SpeedText;
        if (Id == "fly:dir") return &WindP.DirText;
        if (Id == "fly:gust") return &WindP.GustText;
        if (Id == "fly:interval") return &WindP.IntervalText;
        if (Id == "fly:temp") return &WindP.TempText;
        if (Id == "fly:payload") return &WindP.PayloadText;
        if (Id == "cond:seed") return &Cond.Seed;
        if (Id.rfind("cond:", 0) == 0) { std::string K = Id.substr(5); for (std::size_t I = 0; I < ConditionFields().size(); ++I) if (K == ConditionFields()[I].Id) return &Cond.Value[I]; return nullptr; }
        if (Id.rfind("limit:", 0) == 0) { const int I = std::atoi(Id.c_str() + 6); if (I >= 0 && I < static_cast<int>(LimitText.size())) return &LimitText[static_cast<std::size_t>(I)]; }
        return nullptr;
    }

private:
    std::string SessionDir() const { return Join(Work, "session"); }

    void PickVehicle(const std::string& Id)
    {
        for (const auto& V : VehicleClasses())
            if (V.Available && Id == V.Id)
            {
                if (VehicleClass != Id) { VehicleClass = Id; Sel.ClearAll(); Prog = AfterChange(Screen::Vehicle, Prog); }
                Prog.VehicleClassChosen = true;
                SelectedPartId.clear();
                Navigate(Screen::Parts);
                return;
            }
        Say("That vehicle is not available yet.", true);
    }

    void AfterPartsChange()
    {
        Prog.PartsComplete = Sel.Complete();
        Prog = AfterChange(Screen::Parts, Prog);
        Prog.PartsComplete = Sel.Complete();
        Summary = BuildSummary();
        BuildFindings.clear();
        Prims.clear();
        BuildAttempted = false;
    }

    void AddSelectedPart()
    {
        const Part* P = SelectedPart();
        if (P == nullptr) { Say("Pick a part first.", true); return; }
        if (!IsVehiclePartCategory(P->Category)) { Say("A payload is a test condition, not a part: set it on the Conditions screen.", true); return; }
        if (P->Category == "frame" && !VehicleClass.empty() && P->FrameClass != VehicleClass) { Say("This frame is not for the chosen vehicle class.", true); return; }
        Sel.Set(P->Category, P->Id);
        AfterPartsChange();
        Message.clear();
    }

    void ToggleTest(const std::string& Id)
    {
        for (const auto& T : TestList())
            if (Id == T.Id)
            {
                if (!T.Implemented) { Say(std::string(T.Label) + " is not implemented yet.", true); return; }
                if (SelectedTests.count(Id)) SelectedTests.erase(Id); else SelectedTests.insert(Id);
                LimitTest = Id;
                return;
            }
    }

    void ResetLimits()
    {
        for (std::size_t I = 0; I < Limits.size(); ++I) { Limits[I].Value = Limits[I].Original; LimitText[I] = NumText(Limits[I].Original); }
    }

    void CommitFocus() { if (Focus.rfind("fly:", 0) == 0) WindP.Commit(Focus.substr(4)); /* other values are parsed when used */ }
    void AfterFieldEdit() {}

    // ---- jobs --------------------------------------------------------------------------------------------------------------------
    void Enqueue(const Command& C, Helper Kind, const std::string& Tag)
    {
        Step S;
        S.Cmd = C;
        S.Kind = Kind;
        S.Log = Join(SessionDir(), "log_" + Tag + ".txt");
        Queue.push_back(S);
    }

    void StartNextIfIdle()
    {
        if (Queue.empty() || Queue.front().Handle != -1) return;
        Step& S = Queue.front();
        Live = ProgressSnapshot();
        SinceProgressS = 0.0;
        JobDoneOk = false;
        LastLog = S.Log;
        S.Handle = IO.StartProcess(S.Cmd, S.Log);
        if (S.Handle < 0)
        {
            Say(NotFoundMessage(S.Cmd), true);
            Queue.clear();
        }
        else Say(S.Cmd.Description + " ...");
    }

    void StartBuild()
    {
        if (Busy()) { Say("A job is already running.", true); return; }
        if (!Sel.Complete()) { Say("Choose all required parts first.", true); return; }
        if (!ValidBuildName(BuildName)) { Say("Build name: lower-case letters, digits and _ only (starts with a letter or digit).", true); return; }
        BuildDir = Join(SessionDir(), "build_" + BuildName);
        SpecPath = Join(SessionDir(), BuildName + ".build.json");
        if (!IO.WriteText(SpecPath, WriteBuildSpec(BuildName, VehicleClass, Sel))) { Say("Cannot write the build file.", true); return; }
        BuildAttempted = true;
        BuildFindings.clear();
        Prims.clear();
        Prog.BuildDone = false;
        Prog = AfterChange(Screen::Build, Prog);
        Prog.PartsComplete = true;
        JobElapsedS = 0.0; JobDoneSeenS = 0.0; JobDoneOk = false; JobDoneDismissed = false;
        Enqueue(BuildCommand(H, SpecPath, BuildDir), Helper::Build, "build");
        StartNextIfIdle();
    }

    bool PrepareRunFiles()
    {
        std::string Err;
        ConditionsValid V;
        if (!ValidateConditions(Cond, V, Err)) { Say(Err, true); return false; }
        const std::string Tests = TestsArgument(std::vector<std::string>(SelectedTests.begin(), SelectedTests.end()));
        if (Tests.empty()) { Say("Select at least one test.", true); return false; }
        for (std::size_t I = 0; I < Limits.size(); ++I)
        {
            double X = 0.0;
            if (!ParseStrictNumber(LimitText[I], X)) { Say("Limit " + Limits[I].Test + "." + Limits[I].Key + ": enter a number.", true); return false; }
            Limits[I].Value = X;
        }
        std::string Defs;
        if (!WriteEditedDefinitions(DefinitionsText, Limits, Defs, EditedLimits, Err)) { Say(Err, true); return false; }
        if (QuickProfile)
        {
            std::string Quick;
            if (!ApplyQuickProfile(Defs, Quick, Err)) { Say(Err, true); return false; }
            Defs = Quick;
        }
        ConditionsPath = Join(SessionDir(), "conditions.json");
        DefinitionsPath = Join(SessionDir(), "definitions.json");
        if (!IO.WriteText(ConditionsPath, WriteConditionsJson(V)) || !IO.WriteText(DefinitionsPath, Defs)) { Say("Cannot write the condition files.", true); return false; }
        Prog.ConditionsValid = true;
        return true;
    }

    // `AnalyticFallback` is true only for a click on "Run anyway on analytic drag" (shown after a CFD model failed to load).
    void StartRun(bool AnalyticFallback = false)
    {
        if (Busy()) { Say("A job is already running.", true); return; }
        if (!Prog.BuildDone) { Say("Build the vehicle first.", true); return; }
        if (AnalyticFallback && CfdLoadError.empty()) { Say("There is no failed CFD model to run around.", true); return; }
        if (!PrepareRunFiles()) return;
        CampaignDir = Join(SessionDir(), "campaign");
        Prog.CampaignDone = false; Prog.ReportDone = false; Prog.StudyDone = false;
        CfdLoadError.clear();
        RunWasQuick = QuickProfile;
        Recorded.clear();
        Played.clear();
        ReplayTest.clear();
        AutoNavigated = false;
        Cancelled = false;
        Prog.RecordingsAvailable = false;
        JobElapsedS = 0.0; JobDoneSeenS = 0.0; JobDoneOk = false; JobDoneDismissed = false;
        Enqueue(CampaignCommand(H, BuildDir, ConditionsPath, DefinitionsPath, CampaignDir, TestsArgument(std::vector<std::string>(SelectedTests.begin(), SelectedTests.end())), AnalyticFallback),
                Helper::Campaign, "campaign");
        StartNextIfIdle();
    }

    void StartReport()
    {
        if (Busy()) { Say("A job is already running.", true); return; }
        if (!Prog.CampaignDone) { Say("Run the tests first.", true); return; }
        ReportDir = Join(SessionDir(), "report");
        const std::string Val = IO.Exists(Join(BuildDir, "validation_report.json")) ? Join(BuildDir, "validation_report.json") : std::string();
        Enqueue(ReportCommand(H, BuildDir, {CampaignDir}, Val, ReportDir), Helper::Report, "report");
        StartNextIfIdle();
    }

    void StartStudy()
    {
        if (Busy()) { Say("A job is already running.", true); return; }
        if (!Prog.CampaignDone) { Say("Run the tests first; the study uses the same conditions.", true); return; }
        StudyDir = Join(SessionDir(), "study");
        StudyReportDir = Join(SessionDir(), "study_report");
        Enqueue(StudyCommand(H, SpecPath, ConditionsPath, DefinitionsPath, StudyDir), Helper::Study, "study");
        Enqueue(StudyReportCommand(H, StudyDir, StudyReportDir), Helper::Report, "study_report");   // the log name marks it as the study report
        StartNextIfIdle();
    }

    std::string ReadLog(const Step& S) { std::string T; IO.ReadText(S.Log, T); return T; }

    void PollJobs()
    {
        if (Queue.empty()) return;
        if (Queue.front().Handle == -1) { StartNextIfIdle(); return; }
        int Code = 0;
        SyncProgress(Queue.front());
        if (!IO.PollProcess(Queue.front().Handle, Code)) return;
        SyncProgress(Queue.front());   // the lines printed just before it ended
        const Step Done = Queue.front();
        Queue.pop_front();
        const std::string Log = ReadLog(Done);
        if (Cancelled)
        {
            Cancelled = false;
            Queue.clear();
            LoadRecordings();
            Say(Done.Kind == Helper::Campaign && !Recorded.empty()
                    ? "Cancelled. " + std::to_string(Recorded.size()) + " finished test(s) are kept: open Run to replay them. No report was written."
                    : "Cancelled.");
            return;
        }
        if (OnJobDone(Done, Code, Log)) { StartNextIfIdle(); JobDoneOk = Queue.empty() && !MessageIsError; }
        else Queue.clear();
    }

    // Kills the running helper. Tests that already finished stay on disk and stay replayable; the queue behind it is dropped.
    void CancelRunning()
    {
        if (Queue.empty()) return;
        Cancelled = true;
        if (Queue.front().Handle >= 0) { IO.CancelProcess(Queue.front().Handle); Say("Cancelling ..."); }
        else { Queue.clear(); Cancelled = false; Say("Cancelled."); }
    }

    // Takes the helper's progress lines; each newly finished test of a campaign is loaded and queued for the replay.
    void SyncProgress(const Step& S)
    {
        ProgressSnapshot Snap;
        if (!IO.ReadProgress(S.Handle, Snap) || !Snap.Have) return;
        if (Snap.Lines != Live.Lines) SinceProgressS = 0.0;
        Live = Snap;
        if (S.Kind != Helper::Campaign) return;
        bool New = false;
        for (const auto& Id : Snap.FinishedTests)
            if (std::find(Recorded.begin(), Recorded.end(), Id) == Recorded.end()) { Recorded.push_back(Id); New = true; }
        if (New) LoadRecordings();
    }

    // Reads the (partial) campaign folder: every finished test has its files, and the index lists the finished tests only.
    void LoadRecordings()
    {
        if (CampaignDir.empty()) return;
        ReadFile Read = [this](const std::string& Rel, std::string& Out) { return IO.ReadText(Join(CampaignDir, Rel), Out); };
        Campaign Fresh;
        std::string Err;
        if (!LoadCampaign(Read, Fresh, Err)) return;   // not written yet (or half-renamed): the next line asks again
        Camp = Fresh;
        std::string Text;
        if (Powertrain.IsNull() && IO.ReadText(Join(BuildDir, "powertrain_config.json"), Text))
        {
            const Json::ParseResult P = Json::Parse(Text);
            if (P.ok) Powertrain = P.value;
        }
        std::string MErr;
        if (!Powertrain.IsNull()) ComputeMargins(Camp, Powertrain, PeakMargins, MErr);
        for (const auto& T : Camp.Tests) if (!T.SeriesFiles.empty()) Prog.RecordingsAvailable = true;
    }

    // Auto-play: the first finished test starts at once (and opens Run from Conditions, once); when the shown one has ended, the next
    // finished test that has not been shown yet follows. Clicking a test, or turning auto-play off, takes over.
    void AutoPlayStep()
    {
        if (!AutoPlay || Recorded.empty()) return;
        auto Ready = [&](const std::string& Id) { const TestData* T = Camp.Find(Id); return T != nullptr && !T->SeriesFiles.empty(); };
        if (ReplayTest.empty() || Camp.Find(ReplayTest) == nullptr)
        {
            for (const auto& Id : Recorded)
                if (Ready(Id))
                {
                    SelectReplayTest(Id);
                    Played.insert(Id);
                    Rep.Play();
                    if (Current == Screen::Conditions && !AutoNavigated) { AutoNavigated = true; Navigate(Screen::Run); }
                    return;
                }
            return;
        }
        if (Rep.AtEnd() && !Rep.Playing())
            for (const auto& Id : Recorded)
                if (Played.count(Id) == 0 && Ready(Id))
                {
                    SelectReplayTest(Id);
                    Played.insert(Id);
                    Rep.Play();
                    return;
                }
    }

    // Returns false when the rest of the queue must be dropped.
    bool OnJobDone(const Step& S, int Code, const std::string& Log)
    {
        std::string Err;
        switch (S.Kind)
        {
            case Helper::Build:
            {
                BuildFindings = ParseBuilderOutput(Log);
                if (Code != 0)
                {
                    Say(ExitMessage(Helper::Build, Code), true);
                    return false;
                }
                std::string Side, Asm;
                if (!IO.ReadText(Join(BuildDir, "vehicle_build.json"), Side) || !IO.ReadText(Join(BuildDir, "assembly.json"), Asm) || !ParseBuildSummary(Side, Asm, Summary, Err) ||
                    !ParseAssembly(Asm, {}, Prims, Err))
                {
                    Say("The build finished but its files could not be read: " + Err, true);
                    return false;
                }
                BuildFindings = Summary.Findings;
                Prog.BuildDone = Summary.Errors() == 0;
                Say(Prog.BuildDone ? "Vehicle built." : "The build has errors.", !Prog.BuildDone);
                if (Prog.BuildDone) Enqueue(ValidationCommand(H, BuildDir), Helper::Validation, "validation");
                return Prog.BuildDone;
            }
            case Helper::Validation:
                if (Code != 0) Say(ExitMessage(Helper::Validation, Code), true);   // not fatal: the report simply has no validation table
                return true;
            case Helper::Campaign:
            {
                if (Code == 3)
                {
                    // run_campaign refused to continue because the CFD model of this build could not be used; its reason is shown, never hidden.
                    std::string StatusText;
                    CfdLoadError = "the helper gave no reason (see the log)";
                    if (IO.ReadText(Join(CampaignDir, "aero_model_status.json"), StatusText))
                    {
                        const Json::ParseResult P = Json::Parse(StatusText);
                        if (P.ok) { const Json::Value* R = P.value.Get("reason"); if (R && R->IsString() && !R->text.empty()) CfdLoadError = R->text; }
                    }
                    Say(ExitMessage(Helper::Campaign, Code), true);
                    return false;
                }
                if (Code != 0) { Say(ExitMessage(Helper::Campaign, Code), true); return false; }
                if (!LoadCampaignAndModels(Err)) { Say("The tests finished but the results could not be read: " + Err, true); return false; }
                Prog.CampaignDone = true;
                Say("Tests finished.");
                Enqueue(ReportCommand(H, BuildDir, {CampaignDir}, IO.Exists(Join(BuildDir, "validation_report.json")) ? Join(BuildDir, "validation_report.json") : std::string(), Join(SessionDir(), "report")),
                        Helper::Report, "report");
                ReportDir = Join(SessionDir(), "report");
                return true;
            }
            case Helper::Report:
            {
                if (Code != 0) { Say(ExitMessage(Helper::Report, Code) + (Code == 1 ? Log : std::string()), true); return false; }
                const bool IsStudy = S.Log.find("log_study_report") != std::string::npos;
                const std::string Dir = IsStudy ? StudyReportDir : ReportDir;
                std::string Text;
                ReportData& Target = IsStudy ? StudyReport : Report;
                if (!IO.ReadText(Join(Dir, "report.json"), Text) || !ParseReport(Text, Target, Err)) { Say("The report was written but could not be read: " + Err, true); return false; }
                if (IsStudy) { Prog.StudyDone = true; Say("Upgrade study finished."); }
                else { Prog.ReportDone = true; Say("Report written."); }
                return true;
            }
            case Helper::Study:
                if (Code != 0) { Say(ExitMessage(Helper::Study, Code), true); return false; }
                return true;
        }
        return false;
    }

    bool LoadCampaignAndModels(std::string& Err)
    {
        ReadFile Read = [this](const std::string& Rel, std::string& Out) { return IO.ReadText(Join(CampaignDir, Rel), Out); };
        if (!LoadCampaign(Read, Camp, Err)) return false;
        std::string Text;
        if (!IO.ReadText(Join(BuildDir, "powertrain_config.json"), Text)) { Err = "powertrain_config.json not found in the build folder"; return false; }
        const Json::ParseResult P = Json::Parse(Text);
        if (!P.ok) { Err = "powertrain_config.json: " + P.error; return false; }
        Powertrain = P.value;
        if (!ComputeMargins(Camp, Powertrain, PeakMargins, Err)) return false;
        std::string First;
        for (const auto& T : Camp.Tests) if (!T.SeriesFiles.empty()) { First = T.Name; break; }
        if (First.empty()) { Err = "no test recorded a time series"; return false; }
        if (ReplayTest.empty() || Camp.Find(ReplayTest) == nullptr) SelectReplayTest(First);   // keep what the auto-play is showing
        return !ReplayTest.empty();
    }
};
}  // namespace DroneApp
