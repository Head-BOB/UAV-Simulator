// Tier: V
// CFD status path of the engine-free app against the REAL helpers (run_campaign, make_report.py): what Conditions and Results say
// when a CFD model is used, refused, or absent. Two run_campaign executables are needed: one built WITHOUT and one WITH
// `--features allow_stub_models`. Plain C++17, POSIX. The Unreal layer is not covered.
//   g++ -std=c++17 -Wall -Wextra -Werror -I UE5Project/Plugins/DronePhysicsBridge/Source/DronePhysicsBridge/Public tools/cpp_ffi_check/cfd_flow_test.cpp -o cfd_flow_test
//   ./cfd_flow_test <repo root> <python> <run_campaign STRICT exe> <run_campaign STUB-ENABLED exe> <scratch dir>
// Environment: ORT_DYLIB_PATH must name an ONNX Runtime 1.27.x (the helpers load the model with it).
#include "DroneScreens.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <memory>
#include <sstream>
#include <string>

#include <sys/wait.h>

using namespace DroneApp;
namespace fs = std::filesystem;

static int g_failures = 0;
static void Check(bool ok, const std::string& name)
{
    if (!ok)
    {
        std::printf("FAIL: %s\n", name.c_str());
        ++g_failures;
    }
}

struct StdIO : AppIO
{
    std::vector<int> Exits;
    std::vector<std::string> Opened;
    std::vector<Command> Started;
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
        Started.push_back(C);
        std::error_code Ec;
        fs::create_directories(fs::path(LogPath).parent_path(), Ec);
        const std::string Line = "'" + C.Program + "' " + C.ArgumentString() + " > '" + LogPath + "' 2>&1";
        const int Raw = std::system(Line.c_str());
        Exits.push_back(WIFEXITED(Raw) ? WEXITSTATUS(Raw) : 255);
        return static_cast<int>(Exits.size());
    }
    bool PollProcess(int Handle, int& ExitCode) override { ExitCode = Exits[static_cast<std::size_t>(Handle - 1)]; return true; }
    void OpenPath(const std::string& Path) override { Opened.push_back(Path); }
};

static void RunToIdle(AppModel& M)
{
    for (int I = 0; I < 400 && M.Busy(); ++I) M.Tick(0.05);
}

static bool HasArg(const Command& C, const std::string& A)
{
    for (const auto& X : C.Args) if (X == A) return true;
    return false;
}

// Everything drawn is inside the 1920 x 1080 design space, no text is below the minimum size, and every hit region is sane.
static void CheckBounds(const DrawList& L, const std::string& What)
{
    bool Bounds = true, Min = true, Hits = true;
    for (const auto& I : L.Items)
    {
        const float Mx = I.K == Item::Kind::Rect ? I.X + I.W : (I.K == Item::Kind::Line ? std::max(I.X, I.W) : I.X);
        const float My = I.K == Item::Kind::Rect ? I.Y + I.H : (I.K == Item::Kind::Line ? std::max(I.Y, I.H) : I.Y);
        if (I.X < -1.0f || I.Y < -1.0f || Mx > 1921.0f || My > 1081.0f) Bounds = false;
        if (I.K == Item::Kind::Text && I.Px < static_cast<float>(Ui::kMinTextPx)) Min = false;
    }
    for (const auto& H : L.Hits) if (H.X < -1 || H.Y < -1 || H.X + H.W > 1921 || H.Y + H.H > 1081 || H.W <= 0 || H.H <= 0) Hits = false;
    Check(Bounds, What + ": every item inside 1920 x 1080");
    Check(Min, What + ": no text below the minimum size");
    Check(Hits, What + ": hit regions sane");
}

static const Hit* FindHit(const DrawList& L, Act A)
{
    for (const auto& H : L.Hits) if (H.A == A) return &H;
    return nullptr;
}

static bool SameColour(const Rgba& A, const Rgba& B) { return A.R == B.R && A.G == B.G && A.B == B.B; }

// True when some text item contains `Needle` AND is drawn in `C` (a banner above the CFD block may carry similar words in another colour).
static bool HasTextInColour(const DrawList& L, const std::string& Needle, const Rgba& C)
{
    for (const auto& I : L.Items)
        if (I.K == Item::Kind::Text && I.Text.find(Needle) != std::string::npos && SameColour(I.C, C)) return true;
    return false;
}

static std::unique_ptr<AppModel> Model(StdIO& IO, HelperSettings H, const std::string& Work, const std::string& Root)
{
    auto M = std::make_unique<AppModel>(IO, H, Work);
    M->DefinitionsRel = "campaign/smoke_definitions.json";
    if (!M->Init()) { std::printf("FAIL: model init: %s\n", M->Message.c_str()); ++g_failures; }
    // a finished build is taken from the committed golden build (the build step itself is covered by app_flow_test)
    M->BuildDir = Root + "/tools/parts/builds/reference_quad";
    M->Prog.BuildDone = true;
    M->Prog.PartsComplete = true;
    M->SelectedTests = {"climb"};
    M->Current = Screen::Conditions;
    return M;
}

int main(int argc, char** argv)
{
    if (argc < 6)
    {
        std::printf("usage: cfd_flow_test <repo root> <python> <run_campaign strict exe> <run_campaign stub-enabled exe> <scratch dir>\n");
        return 2;
    }
    const std::string Root = argv[1], Py = argv[2], Strict = argv[3], Stub = argv[4], Scratch = argv[5];
    if (!fs::exists(Strict) || !fs::exists(Stub)) { std::printf("FAIL: build both run_campaign executables first\n"); return 1; }
    std::error_code Ec;
    fs::remove_all(Scratch, Ec);
    fs::create_directories(Scratch, Ec);
    const std::string Sample = Root + "/offline_pipeline/cfd/sample";
    // a CFD folder with a flow image: the committed sample plus a PNG (content is irrelevant to the app: only the name is listed)
    const std::string WithImage = Scratch + "/cfd_with_image";
    fs::create_directories(WithImage, Ec);
    for (const auto& E : fs::directory_iterator(Sample)) fs::copy_file(E.path(), WithImage + "/" + E.path().filename().string(), fs::copy_options::overwrite_existing, Ec);
    { std::ofstream P(WithImage + "/velocity_slice_v10.png", std::ios::binary); P << "\x89PNG\r\n\x1a\n"; }

    HelperSettings Base;
    Base.Python = Py; Base.RepoRoot = Root; Base.Threads = 2; Base.GitHash = "cfdflow";

    // ---- the helper commands -------------------------------------------------------------------------------------------------------
    {
        HelperSettings S = Base;
        S.RunCampaignExe = "/r/run_campaign";
        const Command A = CampaignCommand(S, "/b", "/c.json", "/d.json", "/o", "climb");
        Check(!HasArg(A, "--aero-dir") && !HasArg(A, "--analytic-fallback"), "no CFD folder: no aero option");
        S.CfdDir = "/cfd";
        const Command B = CampaignCommand(S, "/b", "/c.json", "/d.json", "/o", "climb");
        Check(HasArg(B, "--aero-dir") && !HasArg(B, "--analytic-fallback"), "CFD folder: --aero-dir, and the fallback is never set automatically");
        Check(HasArg(CampaignCommand(S, "/b", "/c.json", "/d.json", "/o", "climb", true), "--analytic-fallback"), "the explicit opt-in adds --analytic-fallback");
        Check(HasArg(ReportCommand(S, "/b", {"/a"}, "", "/rep"), "--cfd-dir"), "the report is told where the CFD folder is");
        HelperSettings P;
        std::string Err;
        Check(ParseHelperSettings(R"({"repo_root":"repo","cfd_dir":"cfd"})", P, Err) && P.CfdDir == "cfd", "helpers.json cfd_dir is read");
        ResolveHelperPaths(P, "/base");
        Check(P.CfdDir == "/base/cfd", "cfd_dir is resolved against the folder of helpers.json");
    }

    // ---- 1: synthetic fixture, stub-enabled helper -> SYNTHETIC, red -------------------------------------------------------------------
    {
        StdIO IO;
        HelperSettings H = Base;
        H.RunCampaignExe = Stub; H.CfdDir = WithImage;
        auto M = Model(IO, H, Scratch + "/w1", Root);
        M->Click(Act::RunTests, "");
        RunToIdle(*M);
        Check(M->Prog.CampaignDone && M->Prog.ReportDone && M->CfdLoadError.empty(), "synthetic: campaign and report done: " + M->Message);
        Check(M->Report.CfdStatus == "CFD surrogate: SYNTHETIC (formula, not CFD)" && M->Report.CfdLevel == "danger", "synthetic: report status and level: " + M->Report.CfdStatus);
        Check(M->Report.CfdSanityStatus == "NOT_RUN" && !std::isnan(M->Report.CfdHoldoutRmsN), "synthetic: sanity NOT_RUN and the held-out error are carried");
        Check(M->Report.CfdImageNames.size() == 1 && M->Report.CfdImageNames[0] == "velocity_slice_v10.png", "synthetic: the flow image is listed");
        M->Current = Screen::Results;
        DrawList L;
        BuildScreen(L, *M);
        CheckBounds(L, "synthetic Results");
        Check(HasTextInColour(L, "CFD surrogate: SYNTHETIC (formula, not CFD)", Palette::Crit()), "synthetic: Results shows the label in the critical colour");
        Check(L.FindText("NOT_RUN") && L.FindText("Held-out RMS error"), "synthetic: Results shows held-out error and sanity status");
        Check(L.FindText("velocity_slice_v10.png") && !L.FindText("CFD: no CFD run"), "synthetic: Results lists the image and never says no CFD run");
        const Hit* Open = FindHit(L, Act::OpenCfdFolder);
        Check(Open != nullptr, "synthetic: an Open flow images button exists");
        if (Open) { M->Click(Open->A, Open->Arg); Check(IO.Opened.size() == 1 && IO.Opened[0] == WithImage, "Open flow images opens the CFD folder"); }
    }

    // ---- 2: the same CFD folder without images -> "no flow images" -----------------------------------------------------------------------
    {
        StdIO IO;
        HelperSettings H = Base;
        H.RunCampaignExe = Stub; H.CfdDir = Sample;
        auto M = Model(IO, H, Scratch + "/w2", Root);
        M->Click(Act::RunTests, "");
        RunToIdle(*M);
        M->Current = Screen::Results;
        DrawList L;
        BuildScreen(L, *M);
        Check(L.FindText("no flow images") != nullptr && FindHit(L, Act::OpenCfdFolder) == nullptr, "no images: says so and offers no button");
    }

    // ---- 3: strict helper refuses the stub -> exit 3, amber reason, nothing ran -----------------------------------------------------------
    {
        StdIO IO;
        HelperSettings H = Base;
        H.RunCampaignExe = Strict; H.CfdDir = Sample;
        auto M = Model(IO, H, Scratch + "/w3", Root);
        M->Click(Act::RunTests, "");
        RunToIdle(*M);
        Check(IO.Exits.size() == 1 && IO.Exits[0] == 3, "strict: run_campaign exits with code 3");
        Check(!M->Prog.CampaignDone && !M->CfdLoadError.empty(), "strict: no results, the reason is kept: " + M->CfdLoadError);
        Check(M->CfdLoadError.find("stub") != std::string::npos, "strict: the reason says the model is a stub");
        Check(!HasArg(IO.Started[0], "--analytic-fallback"), "strict: the first run did not ask for the fallback");
        DrawList L;
        BuildScreen(L, *M);
        CheckBounds(L, "Conditions with a CFD load error");
        const Item* T = L.FindText("CFD model not used:");
        Check(T && SameColour(T->C, Palette::Warn()), "strict: the reason is drawn in amber on Conditions");
        const Hit* Anyway = FindHit(L, Act::RunAnalyticFallback);
        Check(Anyway != nullptr, "strict: the Run anyway button is offered");
        // ---- 4: the explicit opt-in continues on analytic drag and says so everywhere
        if (Anyway)
        {
            M->Click(Anyway->A, Anyway->Arg);
            RunToIdle(*M);
            Check(IO.Started.size() >= 2 && HasArg(IO.Started[1], "--analytic-fallback"), "fallback: the second run carries --analytic-fallback");
            Check(M->Prog.CampaignDone && M->Prog.ReportDone, "fallback: results and report exist: " + M->Message);
            Check(M->Report.CfdStatus.find("CFD model failed to load") == 0 && M->Report.CfdStatus.find("results use analytic drag") != std::string::npos &&
                      M->Report.CfdLevel == "warn", "fallback: report status says failed and analytic drag, amber: " + M->Report.CfdStatus);
            M->Current = Screen::Results;
            DrawList R;
            BuildScreen(R, *M);
            Check(HasTextInColour(R, "CFD model failed to load", Palette::Warn()), "fallback: Results shows it in amber");
            Check(!R.FindText("CFD: no CFD run"), "fallback: Results does not say no CFD run");
            CheckBounds(R, "fallback Results");
        }
    }

    // ---- 5: a failed run around the button is impossible without a failure ------------------------------------------------------------------
    {
        StdIO IO;
        HelperSettings H = Base;
        H.RunCampaignExe = Stub; H.CfdDir = Sample;
        auto M = Model(IO, H, Scratch + "/w5", Root);
        M->Click(Act::RunAnalyticFallback, "");
        Check(IO.Started.empty() && M->MessageIsError, "no failed model: the fallback button does nothing");
    }

    // ---- 6: no CFD folder at all -> the only case that may say "no CFD run" ---------------------------------------------------------------
    {
        StdIO IO;
        HelperSettings H = Base;
        H.RunCampaignExe = Stub;
        auto M = Model(IO, H, Scratch + "/w6", Root);
        M->Click(Act::RunTests, "");
        RunToIdle(*M);
        Check(M->Prog.ReportDone && !HasArg(IO.Started[0], "--aero-dir"), "no CFD folder: no aero option");
        M->Current = Screen::Results;
        DrawList L;
        BuildScreen(L, *M);
        Check(L.FindText("CFD: no CFD run") != nullptr, "no CFD folder: Results says no CFD run");
    }

    if (g_failures == 0) std::printf("cfd_flow_test: all checks passed\n");
    return g_failures == 0 ? 0 : 1;
}
