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
        if (I.K == Item::Kind::Rect && Title != nullptr && I.X <= Title->X && I.Y <= Title->Y && I.X + I.W >= Title->X + 100.0f && I.Y + I.H >= Title->Y + 20.0f && I.C.A >= 0.99f && I.C.R < 0.15f) Panel = true;
    Check(Panel, "build: the parts list is drawn on an opaque dark panel");
    float MinRow = 1000.0f;
    for (const auto& I : L.Items) if (I.K == Item::Kind::Text && I.X > 480.0f && I.Y < 400.0f && I.Y > 60.0f && I.Text != "Parts") MinRow = std::fmin(MinRow, I.Px);
    Check(MinRow >= 17.0f, "build: parts list text is 17 px or larger");

    if (g_failures == 0) std::printf("layout_test: all checks passed\n");
    return g_failures == 0 ? 0 : 1;
}
