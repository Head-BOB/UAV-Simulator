// Tier: V
// End-to-end test of the engine-free app (model + screens) against the REAL helpers: build_vehicle.py, validate_models, run_campaign,
// make_report.py. Only the 3D view and the Slate painting are not covered (they need Unreal). Plain C++17, POSIX.
//   g++ -std=c++17 -Wall -Wextra -Werror -I UE5Project/Plugins/DronePhysicsBridge/Source/DronePhysicsBridge/Public tools/cpp_ffi_check/app_flow_test.cpp -o app_flow_test
//   ./app_flow_test <repo root> <python> <run_campaign exe> <validate_models exe> <scratch dir>
#include "DroneScreens.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>

#include <sys/wait.h>

using namespace DroneApp;
namespace fs = std::filesystem;

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

// Real files, real processes (run to completion inside StartProcess; Poll reports the saved exit code).
struct StdIO : AppIO
{
    std::vector<int> Exits;
    std::vector<std::string> Opened;
    bool ReadText(const std::string& Path, std::string& Out) override
    {
        std::ifstream f(Path, std::ios::binary);
        if (!f) return false;
        std::stringstream ss;
        ss << f.rdbuf();
        Out = ss.str();
        return true;
    }
    bool WriteText(const std::string& Path, const std::string& Text) override
    {
        std::error_code Ec;
        fs::create_directories(fs::path(Path).parent_path(), Ec);
        std::ofstream f(Path, std::ios::binary);
        f << Text;
        return static_cast<bool>(f);
    }
    bool Exists(const std::string& Path) override { return fs::exists(Path); }
    int StartProcess(const Command& C, const std::string& LogPath) override
    {
        std::error_code Ec;
        fs::create_directories(fs::path(LogPath).parent_path(), Ec);
        std::string Line = "'" + C.Program + "' " + C.ArgumentString() + " > '" + LogPath + "' 2>&1";
        const int Raw = std::system(Line.c_str());
        Exits.push_back(WIFEXITED(Raw) ? WEXITSTATUS(Raw) : 255);
        return static_cast<int>(Exits.size());
    }
    bool PollProcess(int Handle, int& ExitCode) override { ExitCode = Exits[static_cast<std::size_t>(Handle - 1)]; return true; }
    void OpenPath(const std::string& Path) override { Opened.push_back(Path); }
};

static void RunToIdle(AppModel& M)
{
    for (int I = 0; I < 200 && M.Busy(); ++I) M.Tick(0.05);
}

static void Type(AppModel& M, const std::string& FieldId, const std::string& Text)
{
    M.Click(Act::FocusField, FieldId);
    std::string* F = M.FieldRef(FieldId);
    if (F) F->clear();
    for (const char C : Text) M.KeyChar(C);
    M.KeyEnter();
}

// Every screen: inside the design space, nothing under 14 px, text present.
static void CheckLayout(AppModel& M, const char* What)
{
    DrawList L;
    BuildScreen(L, M);
    std::string Name = What;
    Check(L.Count(Item::Kind::Text) > 3, (Name + ": has text").c_str());
    bool Bounds = true, Min = true;
    for (const auto& I : L.Items)
    {
        const float Mx = I.K == Item::Kind::Rect ? I.X + I.W : (I.K == Item::Kind::Line ? std::max(I.X, I.W) : I.X);
        const float My = I.K == Item::Kind::Rect ? I.Y + I.H : (I.K == Item::Kind::Line ? std::max(I.Y, I.H) : I.Y);
        if (I.X < -1.0f || I.Y < -1.0f || Mx > 1921.0f || My > 1081.0f) Bounds = false;
        if (I.K == Item::Kind::Text && I.Px < static_cast<float>(Ui::kMinTextPx)) Min = false;
        if (I.K == Item::Kind::Poly) for (const auto& P : I.Pts) if (P.X < -1.0f || P.X > 1921.0f || P.Y < -1.0f || P.Y > 1081.0f) Bounds = false;
    }
    Check(Bounds, (Name + ": every item inside 1920 x 1080").c_str());
    Check(Min, (Name + ": no text below 14 px").c_str());
    for (const auto& H : L.Hits) Check(H.X >= -1 && H.Y >= -1 && H.X + H.W <= 1921 && H.Y + H.H <= 1081 && H.W > 0 && H.H > 0, (Name + ": hit region sane").c_str());
}

int main(int argc, char** argv)
{
    if (argc < 6)
    {
        std::printf("usage: app_flow_test <repo root> <python> <run_campaign exe> <validate_models exe> <scratch dir>\n");
        return 2;
    }
    const std::string Root = argv[1], Py = argv[2], RunExe = argv[3], ValExe = argv[4], Scratch = argv[5];
    if (!fs::exists(RunExe) || !fs::exists(ValExe)) { std::printf("FAIL: build the release examples first (run_campaign, validate_models)\n"); return 1; }
    std::error_code Ec;
    fs::remove_all(Scratch, Ec);
    fs::create_directories(Scratch, Ec);

    StdIO IO;
    HelperSettings H;
    H.Python = Py; H.RepoRoot = Root; H.RunCampaignExe = RunExe; H.ValidateModelsExe = ValExe; H.Threads = 2; H.GitHash = "apptest";
    AppModel M(IO, H, Scratch);
    M.DefinitionsRel = "campaign/smoke_definitions.json";   // short tests for the check; the app ships with the full definitions
    Check(M.Init(), ("model initialises: " + M.Message).c_str());
    Check(M.Cat.Parts().size() == 56 && !M.Limits.empty() && M.LimitText.size() == M.Limits.size(), "catalogue and limits loaded");

    // ---- 1 Vehicle ----
    CheckLayout(M, "vehicle");
    Check(M.Current == Screen::Vehicle && !CanOpen(Screen::Parts, M.Prog), "starts on Vehicle, Parts locked");
    M.Click(Act::Navigate, "Parts");
    Check(M.Current == Screen::Vehicle && M.MessageIsError, "navigating to a locked screen is refused with a reason");
    M.Click(Act::PickVehicle, "");
    Check(M.Current == Screen::Vehicle && M.MessageIsError, "an unavailable vehicle is refused");
    {   // the real click path: find the Quadcopter row's hit region on the drawn screen and click inside it
        DrawList L;
        BuildScreen(L, M);
        const Hit* Row = nullptr;
        for (const auto& Ht : L.Hits) if (Ht.A == Act::PickVehicle && Ht.Arg == "quad_standard") Row = &Ht;
        Check(Row != nullptr, "the Quadcopter row has a hit region");
        if (Row) HandleClick(M, L, Row->X + 10.0f, Row->Y + 10.0f);
        bool Fpv = false, Wing = false;
        for (const auto& Ht : L.Hits) { Fpv = Fpv || Ht.Arg == "quad_fpv_5in"; Wing = Wing || (Ht.A == Act::PickVehicle && Ht.Arg.empty()); }
        Check(Fpv && !Wing, "the FPV row is clickable and the unavailable rows are not");
    }
    Check(M.Current == Screen::Parts && M.VehicleClass == "quad_standard" && M.Prog.VehicleClassChosen, "clicking the quadcopter row opens Parts");

    // ---- 2 Parts ----
    CheckLayout(M, "parts (empty)");
    M.Click(Act::AddPart, "");
    Check(M.MessageIsError && !M.Sel.Complete(), "adding with nothing picked is refused");
    M.Click(Act::Navigate, "Build");
    Check(M.Current == Screen::Parts, "Build is locked until all parts are chosen");
    const char* Roles[][2] = {{"frame", "ref_frame_quad_400"}, {"motor", "tmotor_mn3508_kv700"}, {"esc", "tmotor_air_40a_2_6s"}, {"propeller", "apc_12x6e"}, {"battery", "ref_battery_generic_4s_5000"}};
    for (const auto& R : Roles)
    {
        M.Click(Act::PickCategory, R[0]);
        Check(M.CategoryFilter == R[0] && !M.VisibleParts().empty(), "category lists parts");
        M.Click(Act::PickPart, R[1]);
        CheckLayout(M, "parts (detail)");
        M.Click(Act::AddPart, "");
    }
    Check(M.Sel.Complete() && M.Prog.PartsComplete, "five roles chosen");
    // the FPV frame must be refused for the quadcopter class
    M.Click(Act::PickCategory, "frame");
    const auto FpvFrames = M.Cat.Filter("frame", "", "", "quad_fpv_5in");
    Check(!FpvFrames.empty(), "an FPV frame exists");
    M.Click(Act::PickPart, FpvFrames[0]->Id);
    M.Click(Act::AddPart, "");
    Check(M.Sel.Get("frame") == "ref_frame_quad_400" && M.MessageIsError, "an FPV frame is refused for the quadcopter class");
    // search typed through the keyboard
    M.Click(Act::FocusField, "search");
    for (const char C : std::string("t-motor")) M.KeyChar(C);
    M.Click(Act::PickCategory, "motor");
    Check(M.Search == "t-motor" && M.VisibleParts().size() >= 5, "typed search filters (T-Motor motors)");
    M.Click(Act::FocusField, "search");
    for (int I = 0; I < 7; ++I) M.KeyBackspace();
    M.KeyEnter();
    Check(M.Search.empty() && M.Focus.empty(), "backspace and enter");
    M.KeyChar('x');
    Check(M.Search.empty(), "typing without focus does nothing");

    // ---- 3 Build (an incompatible choice first: the real builder refuses it) ----
    M.Click(Act::PickCategory, "esc");
    M.Click(Act::PickPart, "ref_esc_generic_20a");
    M.Click(Act::AddPart, "");
    M.Click(Act::Navigate, "Build");
    Check(M.Current == Screen::Build, "Build opens with a complete selection");
    M.Click(Act::BuildNow, "");
    RunToIdle(M);
    Check(!M.Prog.BuildDone && M.MessageIsError, "the builder refuses a too-small ESC");
    bool HaveEscRule = false;
    for (const auto& F : M.BuildFindings) HaveEscRule = HaveEscRule || (F.Rule == "E-ESC-I" && F.Severity == "error");
    Check(HaveEscRule, "the refusal reason E-ESC-I is shown from the builder's own output");
    CheckLayout(M, "build (refused)");
    M.Click(Act::Navigate, "Conditions");
    Check(M.Current == Screen::Build, "Conditions stays locked after a refusal");
    M.Click(Act::PickCategory, "esc");
    M.Click(Act::PickPart, "tmotor_air_40a_2_6s");
    M.Click(Act::AddPart, "");
    M.Click(Act::Navigate, "Build");
    Type(M, "buildname", "Bad Name");
    M.Click(Act::BuildNow, "");
    Check(M.MessageIsError && !M.Busy(), "an invalid build name is refused before any process starts");
    Type(M, "buildname", "app_test");
    M.Click(Act::BuildNow, "");
    Check(M.Busy(), "a build job is queued");
    RunToIdle(M);
    Check(M.Prog.BuildDone, ("the real builder accepted the build: " + M.Message).c_str());
    // the committed reference build is this build PLUS the wiring lump: 1.536 kg - the lump's catalogue mass = this build's mass
    const Part* Lump = M.Cat.Find("ref_avionics_wiring_lump");
    Check(Lump != nullptr && Near(M.Summary.MassKg, 1.536 - Lump->MassKg(), 1e-6) && M.Prims.size() == 18 && M.Summary.Errors() == 0, "mass = 1.536 kg minus the wiring lump, 18 stand-in primitives, from the real build");
    Check(IO.Exists(M.BuildDir + "/validation_report.json"), "the validation check ran and wrote its report into the build folder");
    CheckLayout(M, "build (done)");

    // ---- 4 Conditions ----
    M.Click(Act::UseBuild, "");
    Check(M.Current == Screen::Conditions, "Conditions opens after a build");
    CheckLayout(M, "conditions");
    // all eight tests exist now and start selected; toggling switches one off and on again; an id that is not in the list does nothing
    Check(M.SelectedTests.size() == 8 && M.SelectedTests.count("max_speed") == 1 && M.SelectedTests.count("fpv_sprint") == 1 && M.SelectedTests.count("fpv_punchout") == 1, "all eight tests start selected");
    M.Click(Act::ToggleTest, "max_speed");
    Check(M.SelectedTests.count("max_speed") == 0 && M.SelectedTests.size() == 7, "toggling Maximum speed switches it off");
    M.Click(Act::ToggleTest, "max_speed");
    Check(M.SelectedTests.count("max_speed") == 1, "and on again");
    M.Click(Act::ToggleTest, "no_such_test");
    Check(M.SelectedTests.size() == 8, "an unknown test id changes nothing");
    for (const char* T : {"hover_payload", "wind_gust", "endurance", "thermal_soak", "max_speed", "fpv_sprint", "fpv_punchout"}) M.Click(Act::ToggleTest, T);
    Check(M.SelectedTests.size() == 1 && M.SelectedTests.count("climb") == 1, "only the climb test is selected");
    Type(M, "cond:battery_soc", "1.5");
    M.Click(Act::RunTests, "");
    Check(M.MessageIsError && M.Message.find("Battery state of charge") == 0 && !M.Busy(), "an out-of-range value is refused with its field name, nothing is clamped");
    Type(M, "cond:battery_soc", "0.9");
    Type(M, "cond:name", "app_check");
    Type(M, "cond:ambient_c", "25");
    int Idx = -1;
    for (std::size_t I = 0; I < M.Limits.size(); ++I) if (M.Limits[I].Test == "climb" && M.Limits[I].Key == "target_height_m") Idx = static_cast<int>(I);
    Check(Idx >= 0, "climb target height is an editable limit");
    M.Click(Act::PickTest, "");
    Type(M, "limit:" + std::to_string(Idx), "30");
    M.LimitTest = "climb";
    CheckLayout(M, "conditions (edited)");
    M.Click(Act::SetTurbulence, "none");
    Check(M.Cond.Turbulence == "none", "turbulence choice");
    M.Click(Act::RunTests, "");
    Check(M.Busy() && M.EditedLimits, "tests queued with an edited limit");
    RunToIdle(M);
    Check(M.Prog.CampaignDone && M.Prog.ReportDone, ("campaign and report finished: " + M.Message).c_str());
    Check(M.Camp.Tests.size() == 1 && M.Camp.Tests[0].Name == "climb" && M.ReplayTest == "climb", "one test recorded, ready to replay");
    std::string ManifestText;
    IO.ReadText(M.CampaignDir + "/climb/manifest.json", ManifestText);
    Check(ManifestText.find("\"condition_set\": \"app_check\"") != std::string::npos && ManifestText.find("1.1.0-smoke-edited") != std::string::npos, "the manifest records the condition set and the edited definitions");
    bool EditedBanner = false;
    for (const auto& B : M.Report.Banners) EditedBanner = EditedBanner || B.Id == "edited_limits";
    Check(EditedBanner, "the report carries the 'limits were edited' banner");
    Check(M.Report.Margins.size() == 7 && std::fabs(M.PeakMargins[0].Margin - M.Report.Margins[0].Margin) < 1e-6 * std::max(1.0, M.Report.Margins[0].Margin) && M.PeakMargins[0].Slot == M.Report.Margins[0].Slot,
          "C++ margins equal the Python report's margins on this fresh run");

    // ---- 5 Run (replay) ----
    M.Click(Act::Navigate, "Run");
    Check(M.Current == Screen::Run, "Run opens once a campaign exists");
    CheckLayout(M, "run");
    {   // the caution bar appears exactly when the lowest live margin is below 1.5
        std::vector<MarginRow> Saved = M.LiveRows;
        for (auto& R : M.LiveRows) R.Margin = 2.5;
        DrawList L1;
        BuildScreen(L1, M);
        Check(L1.FindText("Caution:") == nullptr, "no caution bar when every margin is 2.5");
        M.LiveRows[2].Margin = 1.2;
        DrawList L2;
        BuildScreen(L2, M);
        Check(L2.FindText("Caution:") != nullptr && L2.FindText("margin 1.20") != nullptr, "caution bar names the limiting component and its margin when one margin is 1.2");
        M.LiveRows[2].Margin = 1.5;
        DrawList L3;
        BuildScreen(L3, M);
        Check(L3.FindText("Caution:") == nullptr, "1.5 itself is not a caution");
        M.LiveRows = Saved;
    }
    M.KeyShortcut("I");
    M.KeyShortcut("Tab");
    Check(M.ShowDetails && M.ShowControls, "I and Tab open the panels");
    CheckLayout(M, "run (both panels)");
    {
        DrawList L;
        BuildScreen(L, M);
        Check(L.FindText("Altitude") && L.FindText("Ground speed") && L.FindText("Vertical speed") && L.FindText("Heading") && L.FindText("Wind") && L.FindText("Motors") && L.FindText("Battery"),
              "seven instrument labels");
        Check(L.FindText("Details") && L.FindText("Replay") && L.FindText("Margins") && L.FindText("History"), "detail panel and controls dock");
        Check(L.FindText("Recorded run, not live physics"), "the replay disclosure is on screen");
        // click Play on the dock
        const Hit* Play = nullptr;
        for (const auto& Ht : L.Hits) if (Ht.A == Act::PlayPause) Play = &Ht;
        Check(Play != nullptr, "play button has a hit region");
        if (Play) HandleClick(M, L, Play->X + 5.0f, Play->Y + 5.0f);
        Check(M.Rep.Playing(), "clicking Play starts the replay");
        const Hit* Bar = nullptr;
        for (const auto& Ht : L.Hits) if (Ht.A == Act::SeekBar) Bar = &Ht;
        Check(Bar != nullptr, "seek bar has a hit region");
        if (Bar) HandleClick(M, L, Bar->X + Bar->W * 0.5f, Bar->Y + 5.0f);
        Check(Near(M.Rep.Fraction(), 0.5, 1e-6), "clicking the middle of the seek bar seeks to half");
        if (Bar) HandleClick(M, L, Bar->X + Bar->W * 0.25f, Bar->Y + 5.0f);
        Check(Near(M.Rep.Fraction(), 0.25, 1e-6), "a quarter of the way along the bar seeks to a quarter (the click position is used)");
    }
    const double T0 = M.Rep.TimeS();
    M.Click(Act::Faster, "");
    M.Tick(0.1);
    Check(M.Rep.TimeS() > T0 && M.Rep.Speed() == 2.0, "ticking advances the replay at the chosen speed");
    Check(Near(M.Now.TimeS, M.Rep.TimeS(), 1e-12) && Near(M.Now.PosNed[2], M.Rep.Sample().PosNed[2], 0.0), "the live values follow the replay time on every tick");
    for (int I = 0; I < 200; ++I) M.Tick(0.25);
    Check(M.Rep.AtEnd() && !M.Rep.Playing(), "the replay stops at the end");
    M.Click(Act::Restart, "");
    Check(Near(M.Rep.TimeS(), M.Rep.Start(), 0.0), "restart goes to the first sample");
    CheckLayout(M, "run (at start)");
    M.KeyShortcut("Space");
    Check(M.Rep.Playing(), "space starts the replay");
    M.KeyShortcut("Space");
    M.Click(Act::Navigate, "Results");
    M.Tick(1.0);
    Check(M.Current == Screen::Results, "results open");

    // ---- 6 Results ----
    CheckLayout(M, "results");
    {
        DrawList L;
        BuildScreen(L, M);
        Check(L.FindText("Tests and breaking points") && L.FindText("Limiting components") && L.FindText("Validation against published data") && L.FindText("CFD: no CFD run"), "results sections");
        Check(L.FindText("Not implemented") == nullptr, "nothing is listed as not implemented any more (all eight tests exist)");
    }

    // ---- 7 Compare (a hand-made study report: running a real study takes minutes) ----
    M.Click(Act::Navigate, "Compare");
    Check(M.Current == Screen::Results, "Compare is locked until a study exists");
    {
        std::string Study;
        const std::string Smoke = Root + "/tools/parts/builds/reference_quad/report_smoke/report.json";
        IO.ReadText(Smoke, Study);
        const std::size_t P = Study.find("\"upgrades\":");
        Check(P != std::string::npos, "smoke report has an upgrades member");
        const std::size_t E = Study.find("\"validation\"", P);
        const std::string Up = "\"upgrades\": {\"status\": \"run\", \"condition_set\": \"smoke\", \"limiting_slot\": \"motor_current\", \"notice\": \"SMOKE definitions: test\", \"baseline\": {\"limiting_margin\": 1.12},"
                               " \"baseline_weakest\": {\"label\": \"Motor current\", \"margin\": 1.12, \"slot\": \"motor_current\"},"
                               " \"metrics\": {\"limiting_margin\": {\"label\": \"Limiting margin\", \"unit\": \"ratio\", \"higher_is_better\": true}, \"max_payload_kg\": {\"label\": \"Max payload\", \"unit\": \"kg\", \"higher_is_better\": true}},"
                               " \"entries\": [{\"id\": \"motor__a\", \"role\": \"motor\", \"part_id\": \"a\", \"status\": \"tested\", \"rank\": 1, \"rank_note\": \"\", \"warnings\": [], \"weakest\": {\"label\": \"Thrust reserve\", \"margin\": 1.3},"
                               " \"deltas\": {\"limiting_margin\": {\"base\": 1.12, \"cand\": 1.5, \"delta\": 0.38, \"better\": true}, \"max_payload_kg\": {\"base\": 2.5, \"cand\": 2, \"delta\": -0.5, \"better\": false}}},"
                               " {\"id\": \"esc__b\", \"role\": \"esc\", \"part_id\": \"b\", \"status\": \"rejected\", \"rank\": null, \"rank_note\": \"not compatible: E-ESC-I\", \"deltas\": null, \"weakest\": null}]}, ";
        // the validation member follows upgrades in sorted key order, so the splice keeps the JSON valid
        const std::string Spliced = Study.substr(0, P) + Up + Study.substr(E);
        std::string Err;
        Check(ParseReport(Spliced, M.StudyReport, Err), ("spliced study report parses: " + Err).c_str());
    }
    M.Prog.StudyDone = true;
    M.Click(Act::Navigate, "Compare");
    Check(M.Current == Screen::Compare, "Compare opens once a study exists");
    CheckLayout(M, "compare");
    {
        DrawList L;
        BuildScreen(L, M);
        Check(L.FindText("Ranked by the change") && L.FindText("+0.38") && L.FindText("-0.5") && L.FindText("not compatible: E-ESC-I"), "deltas with signs and the rejection reason");
        const Item* Gain = L.FindText("+0.38");
        const Item* Cost = L.FindText("-0.5");
        Check(Gain && Cost && Gain->C.G > Gain->C.R && Cost->C.R > Cost->C.G, "gain is green, cost is red");
    }

    // ---- 8 Report ----
    M.Click(Act::Navigate, "Report");
    CheckLayout(M, "report");
    {
        DrawList L;
        BuildScreen(L, M);
        Check(L.FindText("Engineer's report") && L.FindText("Summary") && L.FindText("What this report rests on") && L.FindText("Open report"), "report screen");
        const Hit* Open = nullptr;
        for (const auto& Ht : L.Hits) if (Ht.A == Act::OpenReport) Open = &Ht;
        Check(Open != nullptr, "open report has a hit region");
        if (Open) HandleClick(M, L, Open->X + 3, Open->Y + 3);
        Check(!IO.Opened.empty() && IO.Opened.back().find("report/report.html") != std::string::npos, "open report asks the host to open report.html");
    }
    Check(IO.Exists(M.ReportDir + "/report.html") && IO.Exists(M.ReportDir + "/report.json"), "report files exist on disk");

    // ---- changing the vehicle invalidates everything derived ----
    M.Click(Act::Navigate, "Vehicle");
    M.Click(Act::PickVehicle, "quad_fpv_5in");
    Check(M.Current == Screen::Parts && !M.Sel.Complete() && !M.Prog.BuildDone && !M.Prog.CampaignDone && !M.Prog.ReportDone, "a new vehicle class clears build, results and report");
    CheckLayout(M, "parts (fpv)");
    M.Click(Act::PickCategory, "frame");
    for (const auto* P : M.VisibleParts()) Check(P->FrameClass == "quad_fpv_5in", "only FPV frames are listed");

    if (g_failures == 0) std::printf("app_flow_test: all checks passed\n");
    return g_failures == 0 ? 0 : 1;
}
