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
    // Starts a program with its output (stdout + stderr) going to LogPath. Returns a handle > 0, or -1 when it could not be started.
    virtual int StartProcess(const Command& C, const std::string& LogPath) = 0;
    // True when the process has ended (ExitCode set); false while it runs.
    virtual bool PollProcess(int Handle, int& ExitCode) = 0;
    virtual void OpenPath(const std::string& Path) = 0;   // folder or file in the system viewer
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
    std::string BusyLabel() const { return Queue.empty() ? std::string() : Queue.front().Cmd.Description; }

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
        Focus.clear();
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
            case Act::PickCategory: CategoryFilter = Arg; ManufacturerFilter.clear(); SelectedPartId.clear(); ListScroll = 0; break;
            case Act::PickManufacturer: ManufacturerFilter = (ManufacturerFilter == Arg) ? std::string() : Arg; ListScroll = 0; break;
            case Act::PickPart: SelectedPartId = Arg; break;
            case Act::AddPart: AddSelectedPart(); break;
            case Act::RemovePart: Sel.Clear(Arg); AfterPartsChange(); break;
            case Act::ScrollList: ListScroll = std::max(0, ListScroll + std::atoi(Arg.c_str())); break;
            case Act::FocusField: Focus = Arg; break;
            case Act::ClearFocus: Focus.clear(); break;
            case Act::BuildNow: StartBuild(); break;
            case Act::UseBuild: Navigate(Screen::Conditions); break;
            case Act::ToggleTest: ToggleTest(Arg); break;
            case Act::ResetLimits: ResetLimits(); break;
            case Act::SetTurbulence: if (Arg == "none" || Arg == "dryden" || Arg == "simple") Cond.Turbulence = Arg; break;
            case Act::RunTests: StartRun(); break;
            case Act::PlayPause: Rep.Toggle(); break;
            case Act::Faster: Rep.FasterStep(); break;
            case Act::Slower: Rep.SlowerStep(); break;
            case Act::Restart: Rep.Seek(Rep.Start()); RefreshLive(); break;
            case Act::SeekBar: Rep.SeekFraction(std::atof(Arg.c_str())); RefreshLive(); break;
            case Act::PickTest: SelectReplayTest(Arg); break;
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
        PollJobs();
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

    void CommitFocus() { /* values are parsed when used; nothing to do while typing */ }
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
        ConditionsPath = Join(SessionDir(), "conditions.json");
        DefinitionsPath = Join(SessionDir(), "definitions.json");
        if (!IO.WriteText(ConditionsPath, WriteConditionsJson(V)) || !IO.WriteText(DefinitionsPath, Defs)) { Say("Cannot write the condition files.", true); return false; }
        Prog.ConditionsValid = true;
        return true;
    }

    void StartRun()
    {
        if (Busy()) { Say("A job is already running.", true); return; }
        if (!Prog.BuildDone) { Say("Build the vehicle first.", true); return; }
        if (!PrepareRunFiles()) return;
        CampaignDir = Join(SessionDir(), "campaign");
        Prog.CampaignDone = false; Prog.ReportDone = false; Prog.StudyDone = false;
        Enqueue(CampaignCommand(H, BuildDir, ConditionsPath, DefinitionsPath, CampaignDir, TestsArgument(std::vector<std::string>(SelectedTests.begin(), SelectedTests.end()))), Helper::Campaign, "campaign");
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
        if (!IO.PollProcess(Queue.front().Handle, Code)) return;
        const Step Done = Queue.front();
        Queue.pop_front();
        const std::string Log = ReadLog(Done);
        if (OnJobDone(Done, Code, Log)) StartNextIfIdle();
        else Queue.clear();
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
        SelectReplayTest(First);
        return !ReplayTest.empty();
    }
};
}  // namespace DroneApp
