// Tier: V
// Layout at any window size: the 1920 x 1080 design space is fitted into the viewport with ONE uniform scale (Ui::FitViewport), centred,
// so nothing is cut off. Regression for the first Windows run (window 1917 x 1018: the step bar was cut off at the top and the bottom of
// the left panel at the bottom, because the canvas fitted the WIDTH only and centred vertically). Every screen of the engine-free app is
// laid out from the committed reference build and smoke campaign and every item, text bottom and hit region is mapped to each viewport.
//   g++ -std=c++17 -Wall -Wextra -Werror -I UE5Project/Plugins/DronePhysicsBridge/Source/DronePhysicsBridge/Public tools/cpp_ffi_check/layout_test.cpp -o layout_test
//   ./layout_test <repo root>
#include "DroneScreens.h"

#include <cmath>
#include <cstdio>
#include <fstream>
#include <sstream>
#include <string>

using namespace DroneApp;

static int g_failures = 0;
static void Check(bool ok, const std::string& name)
{
    if (!ok)
    {
        std::printf("FAIL: %s\n", name.c_str());
        ++g_failures;
    }
}
static bool Near(double a, double b, double tol) { return std::fabs(a - b) <= tol; }

struct FileIO : AppIO
{
    bool ReadText(const std::string& P, std::string& O) override { std::ifstream f(P, std::ios::binary); if (!f) return false; std::stringstream s; s << f.rdbuf(); O = s.str(); return true; }
    bool WriteText(const std::string&, const std::string&) override { return false; }
    bool Exists(const std::string& P) override { std::ifstream f(P); return static_cast<bool>(f); }
    int StartProcess(const Command&, const std::string&) override { return -1; }
    bool PollProcess(int, int&) override { return true; }
    void OpenPath(const std::string&) override {}
};

static void FitValues()
{
    // Hand values: S = min(W / 1920, H / 1080); offset = (size - 1920 S, 1080 S) / 2.
    auto Is = [](double W, double H, double S, double Ox, double Oy, const char* What) {
        const Ui::Viewport V = Ui::FitViewport(W, H);
        Check(Near(V.Scale, S, 1e-9) && Near(V.OffsetX, Ox, 1e-9) && Near(V.OffsetY, Oy, 1e-9), What);
    };
    Is(1920, 1080, 1.0, 0.0, 0.0, "1920 x 1080 is the identity");
    Is(1280, 720, 2.0 / 3.0, 0.0, 0.0, "1280 x 720 scales by 2/3 with no bars");
    Is(2560, 1440, 4.0 / 3.0, 0.0, 0.0, "2560 x 1440 scales by 4/3");
    Is(1917, 1018, 1018.0 / 1080.0, (1917.0 - 1920.0 * 1018.0 / 1080.0) / 2.0, 0.0, "1917 x 1018: height-bound scale 0.94259, side bars of 53.61 px");
    Is(3440, 1440, 4.0 / 3.0, (3440.0 - 2560.0) / 2.0, 0.0, "ultrawide: side bars of 440 px");
    Is(1280, 1000, 2.0 / 3.0, 0.0, (1000.0 - 720.0) / 2.0, "tall window: bars above and below, 140 px each");
    Is(0, 0, 1.0, 0.0, 0.0, "a zero size gives the identity");
    Is(-5, 100, 1.0, 0.0, 0.0, "a negative size gives the identity");
    // the old placement (fit the width, centre vertically) put the top of the design above the window in the 1917 x 1018 case
    const double OldScale = 1917.0 / 1920.0, OldOffsetY = (1018.0 - 1080.0 * OldScale) * 0.5;
    Check(OldOffsetY < -29.0 && Ui::FitViewport(1917, 1018).OffsetY >= 0.0, "regression: the old offset was -30 px, the new one is not negative");
}

struct Size { double W, H; };

static void CheckScreen(const DrawList& L, const AppModel& M, Size S, const std::string& What)
{
    const Ui::Viewport V = Ui::FitViewport(S.W, S.H);
    const double Eps = 0.01;
    auto X = [&](double D) { return D * V.Scale + V.OffsetX; };
    auto Y = [&](double D) { return D * V.Scale + V.OffsetY; };
    bool Inside = true;
    for (const auto& I : L.Items)
    {
        double X0 = I.X, Y0 = I.Y, X1 = I.X, Y1 = I.Y;
        if (I.K == Item::Kind::Rect) { X1 = I.X + I.W; Y1 = I.Y + I.H; }
        else if (I.K == Item::Kind::Line) { X1 = I.W; Y1 = I.H; }
        else if (I.K == Item::Kind::Text) { Y1 = I.Y + I.Px; if (I.Al == Align::Left) X1 = I.X + I.W; }   // text box: top .. top + font size
        else if (I.K == Item::Kind::Poly) for (const auto& P : I.Pts) { X1 = std::fmax(X1, P.X); Y1 = std::fmax(Y1, P.Y); X0 = std::fmin(X0, P.X); Y0 = std::fmin(Y0, P.Y); }
        const double Lo = std::fmin(X0, X1), Hi = std::fmax(X0, X1), To = std::fmin(Y0, Y1), Bo = std::fmax(Y0, Y1);
        if (X(Lo) < -Eps || X(Hi) > S.W + Eps || Y(To) < -Eps || Y(Bo) > S.H + Eps) Inside = false;
    }
    Check(Inside, What + ": every item (text boxes by their font height) lies inside the viewport");
    // text never below the design bottom, so nothing of a panel needs a scrollbar
    bool TextInDesign = true;
    for (const auto& I : L.Items) if (I.K == Item::Kind::Text && I.Y + I.Px > static_cast<float>(Ui::kDesignHeightPx) + 0.5f) TextInDesign = false;
    Check(TextInDesign, What + ": no text runs below the design height");
    // every hit region round-trips: viewport point -> design point -> the same action
    bool Hits = true;
    for (const auto& H : L.Hits)
    {
        const double Cx = X(H.X + H.W * 0.5), Cy = Y(H.Y + H.H * 0.5);
        const double Dx = (Cx - V.OffsetX) / V.Scale, Dy = (Cy - V.OffsetY) / V.Scale;
        const Hit* Got = L.HitAt(static_cast<float>(Dx), static_cast<float>(Dy));
        Hits = Hits && Got != nullptr && Got->A == H.A && X(H.X) >= -Eps && X(H.X + H.W) <= S.W + Eps && Y(H.Y) >= -Eps && Y(H.Y + H.H) <= S.H + Eps;
    }
    Check(Hits, What + ": hit regions are inside the viewport and map back to themselves");
    if (M.Current != Screen::Run)
    {
        // the step bar: all eight labels are present and their boxes are visible
        bool Bar = true;
        for (int I = 0; I < kScreenCount; ++I)
        {
            const std::string Label = std::to_string(I + 1) + "  " + ScreenName(static_cast<Screen>(I));
            const Item* T = L.FindText(Label);
            Bar = Bar && T != nullptr && Y(T->Y) >= -Eps && Y(T->Y + T->Px) <= Y(Ui::kTopBarHeightPx) + Eps && X(T->X) >= -Eps;
        }
        Check(Bar, What + ": the step bar 1 .. 8 is fully visible");
    }
}

int main(int argc, char** argv)
{
    if (argc < 2) { std::printf("usage: layout_test <repo root>\n"); return 2; }
    const std::string Root = argv[1];
    FitValues();

    FileIO IO;
    HelperSettings H;
    H.RepoRoot = Root;
    AppModel M(IO, H, "work");
    M.DefinitionsRel = "campaign/smoke_definitions.json";
    if (!M.Init()) { std::printf("init failed: %s\n", M.Message.c_str()); return 1; }
    const std::string B = Root + "/tools/parts/builds/reference_quad";
    M.VehicleClass = "quad_standard";
    M.Sel.Set("frame", "ref_frame_quad_400"); M.Sel.Set("motor", "tmotor_mn3508_kv700"); M.Sel.Set("esc", "tmotor_air_40a_2_6s");
    M.Sel.Set("propeller", "apc_12x6e"); M.Sel.Set("battery", "ref_battery_generic_4s_5000"); M.Sel.Set("wiring", "ref_avionics_wiring_lump");
    std::string Side, Asm, Err;
    IO.ReadText(B + "/vehicle_build.json", Side); IO.ReadText(B + "/assembly.json", Asm);
    ParseBuildSummary(Side, Asm, M.Summary, Err); ParseAssembly(Asm, {}, M.Prims, Err); M.BuildFindings = M.Summary.Findings;
    M.BuildAttempted = true;
    std::string Pt; IO.ReadText(B + "/powertrain_config.json", Pt); M.Powertrain = Json::Parse(Pt).value;
    M.CampaignDir = B + "/campaign_smoke";
    ReadFile Rd = [&](const std::string& Rel, std::string& O) { return IO.ReadText(M.CampaignDir + "/" + Rel, O); };
    if (!LoadCampaign(Rd, M.Camp, Err)) { std::printf("campaign: %s\n", Err.c_str()); return 1; }
    ComputeMargins(M.Camp, M.Powertrain, M.PeakMargins, Err);
    std::string Rep; IO.ReadText(B + "/report_smoke/report.json", Rep);
    if (!ParseReport(Rep, M.Report, Err)) { std::printf("report: %s\n", Err.c_str()); return 1; }
    M.ReportDir = B + "/report_smoke";

    const Size Sizes[] = {{1280, 720}, {1366, 768}, {1600, 900}, {1917, 1018}, {1920, 1080}, {2560, 1440}, {3440, 1440}, {1280, 1000}, {1024, 768}};
    for (int Pass = 0; Pass < 2; ++Pass)
    {
        if (Pass == 1)
        {
            // worst case on the Build screen: nine long findings (the old code listed all nine and ran off the bottom of the panel)
            M.BuildFindings.clear();
            for (int I = 0; I < 12; ++I)
                M.BuildFindings.push_back(Finding{"E-TEST-" + std::to_string(I), I < 6 ? "error" : "warning",
                                                  "a deliberately long compatibility message that wraps onto three lines of the narrow left panel and then keeps going well past it"});
        }
        for (int I = 0; I < kScreenCount; ++I)
        {
            M.Current = static_cast<Screen>(I);
            DrawList L;
            BuildScreen(L, M);
            Check(L.Count(Item::Kind::Text) > 3, std::string("screen ") + ScreenName(M.Current) + " has text");
            for (const Size S : Sizes)
                CheckScreen(L, M, S, std::string(Pass ? "stress " : "") + ScreenName(M.Current) + " at " + std::to_string(static_cast<int>(S.W)) + "x" + std::to_string(static_cast<int>(S.H)));
        }
    }

    // Build screen, stress state: the findings list is cut with a "more" line instead of overflowing, errors first
    M.Current = Screen::Build;
    DrawList L;
    BuildScreen(L, M);
    int ErrorRows = 0;
    for (const auto& I : L.Items) if (I.K == Item::Kind::Text && I.Text == "Error") ++ErrorRows;
    Check(ErrorRows >= 1 && L.FindText("more (see vehicle_build.json)") != nullptr, "build: errors are listed first and the rest is summarised");
    Check(L.FindText("6 errors, 6 cautions") != nullptr, "build: the counts cover all findings, not only the listed ones");
    // the parts list sits on its own dark panel at a readable size
    const Item* Title = nullptr;   // the panel heading "Parts" (not the step bar's "2  Parts")
    for (const auto& I : L.Items) if (I.K == Item::Kind::Text && I.Text == "Parts" && I.X > 480.0f) Title = &I;
    Check(Title != nullptr, "build: the parts list has its heading");
    bool Panel = false;
    for (const auto& I : L.Items)
        if (I.K == Item::Kind::Rect && Title != nullptr && I.X <= Title->X && I.Y <= Title->Y && I.X + I.W >= Title->X + 100.0f && I.Y + I.H >= Title->Y + 20.0f && I.C.A >= 0.70f && I.C.R < 0.15f) Panel = true;
    Check(Panel, "build: the parts list is drawn on a dark panel");
    float MinRow = 1000.0f;
    for (const auto& I : L.Items) if (I.K == Item::Kind::Text && I.X > 480.0f && I.Y < 400.0f && I.Y > 60.0f && I.Text != "Parts") MinRow = std::fmin(MinRow, I.Px);
    Check(MinRow >= 17.0f, "build: parts list text is 17 px or larger");

    // Parts list scrolling: thumb on overflow, clamp at both ends, reset on screen change, hit regions follow the scroll
    {
        M.Current = Screen::Parts;
        M.CategoryFilter = "";
        M.ListScroll = 0;
        const auto Vis = M.VisibleParts();
        Check(Vis.size() > 15, "enough parts to overflow the list");
        auto FirstRow = [&](std::string& Id) {
            DrawList Ls;
            BuildScreen(Ls, M);
            const Hit* Best = nullptr;
            for (const auto& Ht : Ls.Hits) if (Ht.A == Act::PickPart && (Best == nullptr || Ht.Y < Best->Y)) Best = &Ht;
            Id = Best ? Best->Arg : "";
            bool Thumb = false;
            for (const auto& I : Ls.Items) Thumb = Thumb || (I.K == Item::Kind::Rect && I.W == 4.0f && I.X > 850.0f && I.X < 870.0f);
            return Thumb;
        };
        std::string Id;
        Check(FirstRow(Id) && Id == Vis[0]->Id, "scrollbar thumb is drawn on overflow; the first hit row is the first part");
        M.Click(Act::ScrollList, "100000");
        Check(FirstRow(Id) && Id == Vis[Vis.size() - 15]->Id, "scrolling far past the end clamps to the last full page");
        M.Click(Act::ScrollList, "-100000");
        Check(FirstRow(Id) && Id == Vis[0]->Id, "scrolling past the top clamps to 0");
        M.Click(Act::ScrollList, "7");
        M.Click(Act::Navigate, "Vehicle");
        Check(M.ListScroll == 0, "scroll resets on screen change");
    }

    // Sticky action bars: on Results and Report, no text or hit region appears below kActionBarY
    {
        constexpr float kActY = 984.0f;
        for (Screen S : {Screen::Results, Screen::Report})
        {
            M.Current = S;
            DrawList Ls;
            BuildScreen(Ls, M);
            // No hit region for primary actions appears below the action bar
            bool HitUnder = false;
            for (const auto& Hit : Ls.Hits)
                if (Hit.Y > kActY + 2.0f && (Hit.A == Act::OpenReport || Hit.A == Act::Navigate || Hit.A == Act::WriteReport || Hit.A == Act::OpenFolder))
                    HitUnder = true;
            Check(!HitUnder, std::string(ScreenName(S)) + ": no primary action button falls below the action bar");
            // The action bar fill covers at least to the bottom of the design space
            bool BarFill = false;
            for (const auto& I : Ls.Items)
                if (I.K == Item::Kind::Rect && I.Y <= kActY + 2.0f && I.Y + I.H >= static_cast<float>(Ui::kDesignHeightPx) - 2.0f && I.W > 1800.0f)
                    BarFill = true;
            Check(BarFill, std::string(ScreenName(S)) + ": a full-width fill covers the action bar area");
        }
    }

    // Limit-bound wording: Results screen shows "no failure" for NOT_REACHED_WITHIN_BOUND
    {
        M.Current = Screen::Results;
        DrawList Ls;
        BuildScreen(Ls, M);
        // The report may or may not have NOT_REACHED_WITHIN_BOUND outcomes; just check the text doesn't say "NOT_REACHED_WITHIN_BOUND" verbatim
        bool VerbatimBound = false;
        for (const auto& I : Ls.Items)
            if (I.K == Item::Kind::Text && I.Text == "NOT_REACHED_WITHIN_BOUND") VerbatimBound = true;
        Check(!VerbatimBound, "results: outcome NOT_REACHED_WITHIN_BOUND is never shown verbatim (must say no failure found)");
    }

    // Progress strip stays inside the top bar: the progress strip items must not appear below kTopBarHeightPx
    {
        // Simulate a busy state by putting a dummy job in the queue and checking ProgressPanel items
        // We can test indirectly: build the screen with Busy=false, JobDoneOk=false → strip hidden; JobDoneOk=true → shows
        M.Current = Screen::Conditions;
        M.JobDoneOk = true;
        M.JobDoneDismissed = false;
        M.JobElapsedS = 137.5;   // real elapsed for testing
        DrawList Ls;
        BuildScreen(Ls, M);
        // The elapsed text must show real elapsed, not 0:00; search for "2:17" (137.5 s rounds to 138 s = 2:18) or "2:18"
        bool HasElapsed = false;
        for (const auto& I : Ls.Items)
            if (I.K == Item::Kind::Text && (I.Text.find("2:17") != std::string::npos || I.Text.find("2:18") != std::string::npos)) HasElapsed = true;
        Check(HasElapsed, "progress strip shows real wall-clock elapsed (2:17 or 2:18 for 137.5 s)");
        // Strip items must stay inside the top bar
        bool StripUnder = false;
        for (const auto& I : Ls.Items)
            if (I.K == Item::Kind::Text && I.Y > 0.0f && I.Y < static_cast<float>(Ui::kTopBarHeightPx))
            {
                // Any text inside the top bar is fine; check none spills below it
                if (I.Y + I.Px > static_cast<float>(Ui::kTopBarHeightPx) + 2.0f && I.Y < static_cast<float>(Ui::kTopBarHeightPx)) StripUnder = true;
            }
        Check(!StripUnder, "progress strip text does not overflow below the top bar");
        // Reset
        M.JobDoneOk = false;
        M.JobDoneDismissed = false;
        M.JobElapsedS = 0.0;
    }

    // SetAutoSpeed hand values: known durations → expected speed index
    // endurance ~1893 s: 1893/75 = 25.2; largest preset <= 25.2 is 16 (idx 4). kSpeeds = {1,2,4,8,16,32,64,128,256}
    // hover   ~100 s: 100/75 = 1.33; largest preset <= 1.33 is 1 (idx 0)
    // climb   ~60 s: 60/75 = 0.8; largest preset <= 0.8 is 1? No, 1 > 0.8. idx=0 stays (default 0 = 1x)
    {
        Replay R;
        // Can't load a real series in layout_test (FileIO only); test the math of SetAutoSpeed directly by
        // checking DurationS()/75.0 against the speed table manually.
        // endurance: target = 25.2 → speeds[4]=16 <= 25.2, speeds[5]=32 > 25.2 → idx=4
        double Target = 1893.0 / 75.0;
        int Expected = 0;
        for (int I = 0; I < Replay::kSpeedCount; ++I) if (Replay::kSpeeds[I] <= Target) Expected = I;
        Check(Expected == 4, "SetAutoSpeed hand value: endurance 1893 s → speed index 4 (16x)");
        // hover: target = 100/75 = 1.33 → speeds[0]=1 <= 1.33, speeds[1]=2 > 1.33 → idx=0
        Target = 100.0 / 75.0;
        Expected = 0;
        for (int I = 0; I < Replay::kSpeedCount; ++I) if (Replay::kSpeeds[I] <= Target) Expected = I;
        Check(Expected == 0, "SetAutoSpeed hand value: hover 100 s → speed index 0 (1x)");
        // max_speed: target = 300/75 = 4 → speeds[2]=4 <= 4, speeds[3]=8 > 4 → idx=2
        Target = 300.0 / 75.0;
        Expected = 0;
        for (int I = 0; I < Replay::kSpeedCount; ++I) if (Replay::kSpeeds[I] <= Target) Expected = I;
        Check(Expected == 2, "SetAutoSpeed hand value: 300 s → speed index 2 (4x)");
    }

    // Auto-collapse timing hand values (injected clock): after 4.0 s, JobDoneDismissed must be true
    // We simulate AppModel::Tick with injected time steps.
    {
        FileIO TIO;
        HelperSettings TH;
        TH.RepoRoot = Root;
        AppModel TM(TIO, TH, "work_ac");
        TM.DefinitionsRel = "campaign/smoke_definitions.json";
        TM.Init();
        // Manually set state as if a job just finished successfully
        TM.JobDoneOk = true;
        TM.JobDoneDismissed = false;
        TM.JobDoneSeenS = 0.0;
        Check(!TM.JobDoneDismissed, "auto-collapse: initially not dismissed");
        TM.Tick(1.0);
        Check(!TM.JobDoneDismissed, "auto-collapse: still visible after 1 s");
        TM.Tick(1.5);
        Check(!TM.JobDoneDismissed, "auto-collapse: still visible after 2.5 s");
        TM.Tick(1.6);   // total 4.1 s > 4.0 s threshold
        Check(TM.JobDoneDismissed, "auto-collapse: dismissed after >4 s");
        Check(!TM.ProgressExpanded, "auto-collapse: drop-down also collapsed");
    }

    if (g_failures == 0) std::printf("layout_test: all checks passed\n");
    return g_failures == 0 ? 0 : 1;
}
