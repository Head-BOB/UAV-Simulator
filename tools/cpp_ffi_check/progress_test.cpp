// Tier: V
// Progress protocol and the app's use of it, without Unreal: (1) the UAVPROGRESS line parser (malformed, partial, split across chunks,
// unknown stage, clamping, non-finite numbers, overlong lines); (2) the model driven by a SCRIPTED helper: stage label, elapsed / ETA,
// recordings queued as tests finish, auto-play of the first and then the next finished test, the "No progress for 60 s" warning, Cancel
// keeping the finished tests, the Quick profile. The committed smoke campaign supplies the recordings.
//   g++ -std=c++17 -Wall -Wextra -Werror -I UE5Project/Plugins/DronePhysicsBridge/Source/DronePhysicsBridge/Public tools/cpp_ffi_check/progress_test.cpp -o progress_test
//   ./progress_test <repo root> <scratch dir>
#include "DroneScreens.h"

#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <sstream>

using namespace DroneApp;
namespace fs = std::filesystem;

static int g_failures = 0;
static void Check(bool ok, const std::string& name)
{
    if (!ok) { std::printf("FAIL: %s\n", name.c_str()); ++g_failures; }
}
static bool Near(double a, double b, double tol) { return std::fabs(a - b) <= tol; }

static std::string Line(const char* Stage, const char* Event, const char* TestId, int Index, int Count, double Overall, double PTest, double Elapsed, const char* Eta,
                        const std::string& Tests)
{
    char B[1024];
    std::snprintf(B, sizeof B,
                  "UAVPROGRESS {\"stage\": \"%s\", \"event\": \"%s\", \"test_id\": \"%s\", \"test_index\": %d, \"test_count\": %d, \"sim_s_done\": 10, \"sim_s_total\": 100, "
                  "\"percent_overall\": %.3f, \"percent_test\": %.3f, \"elapsed_s\": %.3f, \"eta_s\": %s, \"tests\": [%s]}\n",
                  Stage, Event, TestId, Index, Count, Overall, PTest, Elapsed, Eta, Tests.c_str());
    return B;
}

static void ParserTests()
{
    ProgressLine L;
    Check(ParseProgressLine(Line("test", "tick", "climb", 2, 8, 12.5, 40.0, 7.0, "null", "{\"id\": \"climb\", \"state\": \"running\", \"percent\": 40, \"sim_s\": 3}"), L), "a good line parses");
    Check(L.Stage == ProgStage::Test && L.TestId == "climb" && L.TestIndex == 2 && L.TestCount == 8 && Near(L.PercentOverall, 12.5, 1e-9) && Near(L.PercentTest, 40.0, 1e-9) &&
              L.EtaS < 0.0 && L.Tests.size() == 1 && L.Tests[0].State == ProgState::Running && Near(L.Tests[0].Percent, 40.0, 1e-9), "fields are read; eta null is 'no estimate'");
    Check(!ParseProgressLine("hello", L) && !ParseProgressLine("UAVPROGRESS ", L) && !ParseProgressLine("UAVPROGRESS {not json}", L) && !ParseProgressLine("UAVPROGRESS [1,2]", L) &&
              !ParseProgressLine("UAVPROGRESS {\"stage\": \"test\"}", L) && !ParseProgressLine("UAVPROGRESS {\"percent_overall\": 5}", L), "malformed lines are refused");
    Check(!ParseProgressLine("UAVPROGRESS {\"stage\": \"test\", \"percent_overall\": NaN}", L) && !ParseProgressLine("UAVPROGRESS {\"stage\": \"test\", \"percent_overall\": \"5\"}", L), "NaN and text numbers are refused");
    Check(ParseProgressLine("UAVPROGRESS {\"stage\": \"weird\", \"percent_overall\": 5}", L) && L.Stage == ProgStage::Unknown, "an unknown stage is kept as Unknown");
    Check(ParseProgressLine("UAVPROGRESS {\"stage\": \"test\", \"percent_overall\": 250, \"percent_test\": -3, \"elapsed_s\": -1, \"eta_s\": -5, \"test_index\": -2}", L) &&
              Near(L.PercentOverall, 100.0, 0.0) && Near(L.PercentTest, 0.0, 0.0) && Near(L.ElapsedS, 0.0, 0.0) && Near(L.EtaS, 0.0, 0.0) && L.TestIndex == 0, "clamped: percent to 0..100, times and index not negative");

    ProgressParser P;
    const std::string A = Line("test", "tick", "hover_payload", 0, 8, 1.0, 2.0, 1.0, "null", ""), B = Line("test", "test_done:climb", "climb", 2, 8, 20.0, 100.0, 5.0, "12.5", ""), C = Line("done", "campaign_done", "x", 7, 8, 90.0, 100.0, 9.0, "null", "");
    P.Feed(A.substr(0, 30));
    Check(!P.Snapshot().Have, "a partial line is held back");
    P.Feed(A.substr(30) + "plain output line\n" + B.substr(0, 50));
    Check(P.Snapshot().Have && P.Snapshot().Lines == 1 && P.Snapshot().Latest.TestId == "hover_payload", "the completed line is taken, other output ignored");
    P.Feed(B.substr(50) + "UAVPROGRESS {broken\n" + C);
    Check(P.Snapshot().Lines == 3 && P.Rejected() == 1 && P.Snapshot().Latest.Stage == ProgStage::Done && P.Snapshot().FinishedTests == std::vector<std::string>{"climb"}, "split lines join; a broken tagged line is counted and skipped; finished tests are listed once");
    P.Feed(B);
    Check(P.Snapshot().FinishedTests.size() == 1, "a repeated test_done event is not listed twice");
    ProgressParser Q;
    Q.Feed(std::string(ProgressParser::kMaxLine + 10, 'x') + "\n" + A);
    Check(Q.Snapshot().Lines == 1, "an overlong line is dropped whole and parsing recovers");
    Check(ClockText(65.0) == "1:05" && ClockText(3725.0) == "1:02:05" && ClockText(-1.0).empty(), "clock text");
}

// A helper that is never really run: the test says what it prints and when it ends.
struct ScriptIO : AppIO
{
    std::string Root;
    std::vector<std::string> Opened;
    bool Ended = false;
    int ExitCode = 0;
    ProgressParser Parser;
    bool Cancelled = false;
    std::string StartedProgram;
    bool ReadText(const std::string& P, std::string& O) override { std::ifstream f(P, std::ios::binary); if (!f) return false; std::stringstream s; s << f.rdbuf(); O = s.str(); return true; }
    bool WriteText(const std::string& P, const std::string& T) override
    {
        std::error_code Ec;
        fs::create_directories(fs::path(P).parent_path(), Ec);
        std::ofstream f(P, std::ios::binary);
        f << T;
        return static_cast<bool>(f);
    }
    bool Exists(const std::string& P) override { return fs::exists(P); }
    int StartProcess(const Command& C, const std::string&) override { StartedProgram = C.Program; Ended = false; Cancelled = false; Parser = ProgressParser(); return 1; }
    bool PollProcess(int, int& Code) override { Code = Cancelled ? -1 : ExitCode; return Ended || Cancelled; }
    void OpenPath(const std::string& P) override { Opened.push_back(P); }
    bool ReadProgress(int, ProgressSnapshot& Out) override { Out = Parser.Snapshot(); return Out.Have; }
    void CancelProcess(int) override { Cancelled = true; }
};

static bool HasText(const AppModel& M, const std::string& Needle)
{
    DrawList L;
    BuildScreen(L, M);
    return L.FindText(Needle) != nullptr;
}

int main(int argc, char** argv)
{
    if (argc < 3) { std::printf("usage: progress_test <repo root> <scratch dir>\n"); return 2; }
    const std::string Root = argv[1], Scratch = argv[2];
    ParserTests();

    std::error_code Ec;
    fs::remove_all(Scratch, Ec);
    fs::create_directories(Scratch, Ec);
    ScriptIO IO;
    HelperSettings H;
    H.RepoRoot = Root;
    H.Progress = true;
    // with --progress asked for, the helper commands carry the flag
    Check(CampaignCommand(H, "/b", "/c", "/d", "/o", "").Args.back() == "--progress" && ReportCommand(H, "/b", {"/a"}, "", "/r").Args.end()[-3] == "--progress", "campaign and report commands carry --progress when asked for");
    AppModel M(IO, H, Scratch + "/work");
    M.DefinitionsRel = "campaign/smoke_definitions.json";
    if (!M.Init()) { std::printf("init failed: %s\n", M.Message.c_str()); return 1; }

    // ---- Quick profile (definitions written for the run) ----
    {
        std::string Out, Err;
        Check(ApplyQuickProfile(M.DefinitionsText, Out, Err), "quick profile applies");
        const Json::ParseResult R = Json::Parse(Out);
        const Json::Value* E = R.value.Get("endurance");
        const Json::Value* Hp = R.value.Get("hover_payload");
        const Json::Value* V = R.value.Get("definitions_version");
        Check(R.ok && E && Hp && V && V->text.size() > 6 && V->text.substr(V->text.size() - 6) == "-quick", "definitions_version gets the -quick suffix");
        // the smoke definitions are already shorter than the quick values: nothing is ever raised
        const Json::ParseResult O = Json::Parse(M.DefinitionsText);
        Check(E->Get("max_duration_s")->number <= O.value.Get("endurance")->Get("max_duration_s")->number, "a value is only ever reduced");
        const std::string Full = "{\"definitions_version\": \"1.1.0\", \"endurance\": {\"max_duration_s\": 3600}, \"thermal_soak\": {\"max_duration_s\": 7200}, \"hover_payload\": {\"hold_s\": 30}, \"wind_gust\": {\"hold_s\": 30}}";
        Check(ApplyQuickProfile(Full, Out, Err), "quick on the full definitions");
        const Json::ParseResult Q = Json::Parse(Out);
        Check(Near(Q.value.Get("endurance")->Get("max_duration_s")->number, 300.0, 0.0) && Near(Q.value.Get("thermal_soak")->Get("max_duration_s")->number, 600.0, 0.0) &&
                  Near(Q.value.Get("hover_payload")->Get("hold_s")->number, 15.0, 0.0) && Near(Q.value.Get("wind_gust")->Get("hold_s")->number, 15.0, 0.0) &&
                  Q.value.Get("definitions_version")->text == "1.1.0-quick", "fixed durations: endurance 300 s, thermal soak 600 s, holds 15 s");
    }

    // ---- state of a built vehicle with the committed smoke campaign standing in for the helper's output ----
    const std::string B = Root + "/tools/parts/builds/reference_quad";
    M.VehicleClass = "quad_standard";
    M.Sel.Set("frame", "ref_frame_quad_400"); M.Sel.Set("motor", "tmotor_mn3508_kv700"); M.Sel.Set("esc", "tmotor_air_40a_2_6s");
    M.Sel.Set("propeller", "apc_12x6e"); M.Sel.Set("battery", "ref_battery_generic_4s_5000");
    M.Prog.VehicleClassChosen = M.Prog.PartsComplete = M.Prog.BuildDone = true;
    M.BuildDir = B;
    M.CampaignDir = B + "/campaign_smoke";
    M.Current = Screen::Conditions;
    M.QuickProfile = true;
    M.Click(Act::RunTests, "");
    Check(M.Busy() && M.RunWasQuick && IO.StartedProgram.size() > 0, "the run starts with the quick profile");
    // the helper's output folder: the committed smoke campaign stands in for the files the helper writes as tests finish
    fs::create_directories(M.CampaignDir, Ec);
    fs::copy(B + "/campaign_smoke", M.CampaignDir, fs::copy_options::recursive | fs::copy_options::overwrite_existing, Ec);
    Check(!Ec && fs::exists(M.CampaignDir + "/campaign_result.json"), "stand-in campaign folder in place");
    Check(HasText(M, "Preparing") || HasText(M, "Running the tests"), "the panel shows the job before the first line");
    // the conditions screen offers the profile and the panel is on top
    M.QuickProfile = false;

    // line 1: running
    const std::string Rows1 = "{\"id\": \"hover_payload\", \"state\": \"running\", \"percent\": 30, \"sim_s\": 12}, {\"id\": \"climb\", \"state\": \"running\", \"percent\": 60, \"sim_s\": 6}, {\"id\": \"endurance\", \"state\": \"waiting\", \"percent\": 0, \"sim_s\": 0}";
    IO.Parser.Feed(Line("test", "tick", "hover_payload", 0, 3, 18.0, 30.0, 4.0, "null", Rows1));
    M.Tick(1.0);
    Check(M.Live.Have && Screens::StageLabel(M) == "Test 1 of 3: Hover with payload", "stage label: 'Test 1 of 3: <test>'");
    Check(HasText(M, "Test 1 of 3: Hover with payload") && HasText(M, "18 %"), "the panel on the Conditions screen shows label and overall percent");
    Check(M.Live.Latest.EtaS < 0.0 && !HasText(M, "estimate"), "no time left shown before the first test has finished");

    // line 2: the climb test finished -> recording queued, auto-play starts, Run opens once
    const std::string Rows2 = "{\"id\": \"hover_payload\", \"state\": \"running\", \"percent\": 50, \"sim_s\": 20}, {\"id\": \"climb\", \"state\": \"done\", \"percent\": 100, \"sim_s\": 11}, {\"id\": \"endurance\", \"state\": \"running\", \"percent\": 5, \"sim_s\": 2}";
    IO.Parser.Feed(Line("test", "test_done:climb", "hover_payload", 0, 3, 40.0, 50.0, 9.0, "13.5", Rows2));
    M.Tick(1.0);
    Check(M.Recorded == std::vector<std::string>{"climb"} && M.Prog.RecordingsAvailable, "the finished test is queued as a recording");
    Check(M.ReplayTest == "climb" && M.Rep.Playing() && M.Current == Screen::Run, "auto-play starts the first finished test and opens Run");
    Check(HasText(M, "Recorded run, not live physics") && HasText(M, "Climb") && HasText(M, "left about 0:14 (estimate)"), "Run screen: label, test name; ETA marked as estimate after the first test");
    Check(!M.Prog.CampaignDone && !CanOpen(Screen::Results, M.Prog), "the campaign is not done: Results stays locked");

    // while the first plays, a second finishes: it does not interrupt
    const std::string Rows3 = "{\"id\": \"hover_payload\", \"state\": \"done\", \"percent\": 100, \"sim_s\": 40}, {\"id\": \"climb\", \"state\": \"done\", \"percent\": 100, \"sim_s\": 11}, {\"id\": \"endurance\", \"state\": \"running\", \"percent\": 20, \"sim_s\": 9}";
    IO.Parser.Feed(Line("test", "test_done:hover_payload", "endurance", 2, 3, 60.0, 20.0, 15.0, "10", Rows3));
    M.Tick(0.1);
    Check(M.ReplayTest == "climb" && M.Recorded.size() == 2, "a second recording is queued without interrupting the one playing");
    M.Rep.Seek(M.Rep.End());
    M.Rep.Pause();
    M.Tick(0.1);
    Check(M.ReplayTest == "hover_payload" && M.Rep.Playing(), "when the shown run has ended, auto-play moves to the next finished test");
    // the user takes over
    M.Click(Act::PickTest, "climb");
    Check(M.ReplayTest == "climb", "the user can click any finished test");
    M.Click(Act::ToggleAutoPlay, "");
    Check(!M.AutoPlay, "auto-play can be turned off");

    // silence for a minute
    for (int I = 0; I < 59; ++I) M.Tick(1.0);
    Check(!M.NoProgress(), "59 s of silence is not yet a warning");
    M.Tick(2.0);
    Check(M.NoProgress() && HasText(M, "No progress for 60 s") && HasText(M, "Open log"), "after 60 s without a line: amber 'No progress for 60 s' with a log link");
    M.Click(Act::OpenLog, "");
    Check(!IO.Opened.empty() && IO.Opened.back().find("log_campaign") != std::string::npos, "the log link opens the job's log file");

    // cancel: the helper is killed, finished tests stay, no report is started
    M.Click(Act::CancelJob, "");
    M.Tick(0.1);
    Check(IO.Cancelled && !M.Busy() && M.Message.find("Cancelled") != std::string::npos && M.Message.find("2 finished") != std::string::npos, "Cancel kills the helper and keeps the finished tests");
    Check(M.Prog.RecordingsAvailable && !M.Prog.CampaignDone && !M.Prog.ReportDone && M.Queue.empty(), "after Cancel: recordings replayable, no report job queued");
    Check(!HasText(M, "Cancel"), "no Cancel button when nothing runs");

    // state machine: idle -> working -> failed, each change asks for a repaint without any input
    M.TakeRepaint();
    Check(!M.TakeRepaint(), "repaint flag is consumed once");
    M.Click(Act::SetProfile, "full");
    M.Click(Act::RunTests, "");
    M.TakeRepaint();
    M.Tick(0.1);
    Check(M.Busy() && M.TakeRepaint(), "working: a running job keeps asking for repaints");
    IO.Parser.Feed(Line("test", "tick", "climb", 0, 1, 5.0, 5.0, 1.0, "null", ""));
    IO.Ended = true;
    IO.ExitCode = 1;   // failed: the helper's exit code ends the job and the chain
    M.Tick(0.1);
    Check(!M.Busy() && M.MessageIsError && M.TakeRepaint() && !M.TakeRepaint(), "failed on process exit: error message, repaint requested once, no input needed");
    M.Click(Act::Navigate, "Conditions");
    Check(M.TakeRepaint(), "navigation asks for a repaint");
    // a finished chain says Done
    M.Click(Act::SetProfile, "quick");
    Check(M.QuickProfile, "the profile selector works while idle");
    M.Click(Act::RunTests, "");
    IO.Parser.Feed(Line("done", "campaign_done", "x", 7, 8, 90.0, 100.0, 30.0, "null", ""));
    IO.Ended = true;
    IO.ExitCode = 1;   // the helper failed: the run is an error, not "Done"
    M.Tick(0.1);
    Check(!M.Busy() && M.MessageIsError && !M.JobDoneOk, "a failed helper is an error, never 'Done'");

    if (g_failures == 0) std::printf("progress_test: all checks passed\n");
    return g_failures == 0 ? 0 : 1;
}
