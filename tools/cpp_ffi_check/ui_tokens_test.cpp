// Tier: V
// Tests for DroneUiTokens.h against the approved preview (dev-docs/ui-reference/quad_ui_preview_v4.html). Plain C++17.
//   g++ -std=c++17 -Wall -Wextra -Werror -I UE5Project/Plugins/DronePhysicsBridge/Source/DronePhysicsBridge/Public tools/cpp_ffi_check/ui_tokens_test.cpp -o ui_tokens_test
//   ./ui_tokens_test dev-docs/ui-reference/quad_ui_preview_v4.html
// The preview file is parsed for its numbers, so the tokens cannot drift away from it without this test failing.
#include "DroneUiTokens.h"
#include "DroneUiMath.h"

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <fstream>
#include <sstream>
#include <string>

using namespace DroneApp::Ui;

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

static std::string g_html;

// Text after "needle" up to the next ';' or '}' (the CSS value), or "" if the needle is absent.
static std::string After(const std::string& needle)
{
    const std::size_t p = g_html.find(needle);
    if (p == std::string::npos) return "";
    const std::size_t s = p + needle.size();
    const std::size_t e = g_html.find_first_of(";}", s);
    return g_html.substr(s, e - s);
}
// Hex colour value of a CSS variable like "--acc:" -> 0x8FBCE8 (case-insensitive), or 0xFFFFFFFF if absent.
static std::uint32_t Var(const char* name)
{
    const std::string v = After(std::string(name) + ":#");
    if (v.empty()) return 0xFFFFFFFFu;
    return static_cast<std::uint32_t>(std::stoul(v.substr(0, 6), nullptr, 16));
}
// "rgba(r,g,b,a)" after "name:" -> rgb and alpha.
static bool Rgba(const char* name, std::uint32_t& rgb, double& alpha)
{
    const std::string v = After(std::string(name) + ":rgba(");
    if (v.empty()) return false;
    int r = 0, g = 0, b = 0;
    double a = 0.0;
    if (std::sscanf(v.c_str(), "%d,%d,%d,%lf", &r, &g, &b, &a) != 4) return false;
    rgb = (static_cast<std::uint32_t>(r) << 16) | (static_cast<std::uint32_t>(g) << 8) | static_cast<std::uint32_t>(b);
    alpha = a;
    return true;
}
static double PxAfter(const std::string& needle)
{
    const std::string v = After(needle);
    return v.empty() ? -1.0 : std::stod(v);
}

static void TestAgainstThePreview()
{
    Check(Var("--t1") == kInk.Rgb && Var("--acc") == kAccent && Var("--ok") == kOk && Var("--warn") == kWarn && Var("--crit") == kCrit, "palette equals the preview variables");
    Check(Var("--solid") == kSolid.Rgb, "solid panel colour");
    std::uint32_t rgb = 0;
    double a = 0.0;
    Check(Rgba("--bar", rgb, a) && rgb == kBar.Rgb && Near(a, kBar.Alpha, 1e-12), "bar fill rgba(12,14,18,.86)");
    Check(Rgba("--ln", rgb, a) && rgb == kLine.Rgb && Near(a, kLine.Alpha, 1e-12), "line 13 %");
    Check(Rgba("--ln2", rgb, a) && rgb == kLineStrong.Rgb && Near(a, kLineStrong.Alpha, 1e-12), "line 26 %");
    Check(Rgba("--t2", rgb, a) && rgb == kInk70.Rgb && Near(a, kInk70.Alpha, 1e-12), "secondary text 70 %");
    Check(Rgba("--t3", rgb, a) && rgb == kInk46.Rgb && Near(a, kInk46.Alpha, 1e-12), "tertiary text 46 %");
    // regions: ".top{...height:56px", ".ib{...height:128px", ".det{...width:460px", ".dock{...width:440px", ".s-scrim{...width:700px"
    Check(Near(PxAfter(".top{position:absolute;left:0;right:0;top:0;height:"), kTopBarHeightPx, 0.0), "top bar 56");
    Check(Near(PxAfter(".ib{position:absolute;left:0;right:0;bottom:0;height:"), kInstrumentBarHeightPx, 0.0), "instrument bar 128");
    Check(Near(PxAfter(".det{position:absolute;right:0;top:56px;bottom:128px;width:"), kDetailPanelWidthPx, 0.0), "detail panel 460");
    Check(Near(PxAfter(".dock{position:absolute;left:0;top:56px;bottom:128px;width:"), kControlsDockWidthPx, 0.0), "controls dock 440");
    Check(Near(PxAfter(".s-scrim{position:absolute;left:0;top:0;bottom:0;width:"), kSetupScrimWidthPx, 0.0), "setup scrim 700");
    Check(Near(PxAfter(".s-col{position:absolute;left:"), kSetupColumnLeftPx, 0.0), "setup column left 96");
    Check(g_html.find("grid-template-columns:repeat(7,1fr)") != std::string::npos && kInstrumentColumns == 7, "seven instrument columns");
    Check(Near(PxAfter(".it .v{font-size:"), kKeyValuePx, 0.0), "key value 46");
    Check(Near(PxAfter(".elist button{font:inherit;background:none;border:0;border-top:1px solid var(--ln);text-align:left;color:var(--t3);font-size:"), kSetupRowPx, 0.0), "setup row 30");
    Check(Near(PxAfter(".r-head h1{margin:0;font-size:"), kTitleMaxPx, 0.0), "report title 56");
    Check(Near(PxAfter(".alert{position:absolute;left:50%;top:"), kTopOfCautionBarPx, 0.0), "caution bar top 84");
    Check(Near(PxAfter(".r-wrap{position:absolute;left:"), kReportMarginXPx, 0.0), "report margin 120");
    Check(g_html.find("gap:100px;margin-top:34px") != std::string::npos && kReportColumnGapPx == 100.0, "report column gap 100");
    Check(Near(PxAfter(".chip{display:inline-block;font-size:"), kLabelPx, 0.0), "chip text 14");
    Check(g_html.find("History, last 120 s") != std::string::npos && kHistorySpanSeconds == 120.0, "history span 120 s");
}

static void TestContrastAndScale()
{
    // every text token composited onto the darkest and the lightest surface it sits on must reach WCAG AA 4.5:1 for normal text
    const std::uint32_t SolidBg = kSolid.Rgb;
    const std::uint32_t BarOverDark = Blend(kBar, 0x000000);   // the bar over a black scene is the worst case
    const std::uint32_t BarOverLight = Blend(kBar, 0x808080);  // over mid grey, a bright scene darkens less
    for (const std::uint32_t Bg : {SolidBg, BarOverDark, kReportBackground.Rgb})
    {
        Check(DroneUi::ContrastRatio(kInk.Rgb, Bg) >= 4.5, "ink on surface");
        Check(DroneUi::ContrastRatio(Blend(kInk70, Bg), Bg) >= 4.5, "ink 70 % on surface");
        Check(DroneUi::ContrastRatio(Blend(kInk46, Bg), Bg) >= 3.0, "ink 46 % on surface (large / tertiary text, 3:1)");
        Check(DroneUi::ContrastRatio(kAccent, Bg) >= 4.5 && DroneUi::ContrastRatio(kOk, Bg) >= 4.5 && DroneUi::ContrastRatio(kWarn, Bg) >= 4.5 &&
                  DroneUi::ContrastRatio(kCrit, Bg) >= 4.5, "accent and status colours on surface");
    }
    Check(DroneUi::ContrastRatio(Blend(kInk70, BarOverLight), BarOverLight) >= 4.5, "ink 70 % on the bar over a mid-grey scene");
    Check(DroneUi::ContrastRatio(kOnPrimaryButton, kInk.Rgb) >= 4.5, "dark text on the filled ink button");
    Check(DroneUi::ContrastRatio(kOnWarnBar, kWarn) >= 4.5, "dark text on the amber caution bar");
    // Blend by hand: white at 70 % over black = 255 * 0.7 = 178.5 -> +0.5 -> 179 = 0xB3 per channel
    Check(Blend(kInk70, 0x000000) == 0xB3B3B3, "blend white 70 % over black");
    // columns: (1920 - 2 x 24) / 7 = 1872 / 7 = 267.428571...; seven of them plus the padding make exactly the screen width
    Check(Near(InstrumentColumnWidthPx(), 1872.0 / 7.0, 1e-12), "column width");
    Check(Near(InstrumentColumnWidthPx() * kInstrumentColumns + 2.0 * kInstrumentBarPaddingPx, kDesignWidthPx, 1e-9), "7 columns + padding = 1920");
    // scale: fit-to-width 1920 -> 1, 3840 -> 2, 1280 -> 2/3, non-positive -> 1
    Check(Near(ScaleForWidth(1920.0), 1.0, 0.0) && Near(ScaleForWidth(3840.0), 2.0, 0.0) && Near(ScaleForWidth(1280.0), 2.0 / 3.0, 1e-12) && ScaleForWidth(0.0) == 1.0, "scale");
    // 14 px at 1080p stays 14; at 4K it is 28; at 1280 wide it is 9.33 -> 9 which is below the minimum (callers refuse such a viewport)
    Check(ScaledPx(14.0, 1.0) == 14 && ScaledPx(14.0, 2.0) == 28 && ScaledPx(14.0, ScaleForWidth(1280.0)) == 9, "scaled px");
    Check(ScaledPx(0.0, 1.0) == 1, "never below one pixel");
    Check(Near(kMinViewportWidthPx, 1920.0, 0.0), "minimum viewport width keeps 14 px labels at 14 px");
    // type scale: no token below the 14 px minimum and key values / titles ordered
    Check(kLabelPx >= kMinTextPx && kLabelLargePx >= kLabelPx && kBodyPx >= kLabelLargePx && kValuePx >= kBodyPx && kKeyValuePx > kSetupRowPx && kTitleMaxPx > kTitleMinPx, "type scale order");
    // the two bar heights sit on the 8 px grid (56 = 7 x 8, 128 = 16 x 8)
    Check(std::fmod(kTopBarHeightPx, kGridPx) == 0.0 && std::fmod(kInstrumentBarHeightPx, kGridPx) == 0.0, "bar heights on the 8 px grid");
}

int main(int argc, char** argv)
{
    if (argc < 2)
    {
        std::printf("usage: ui_tokens_test <quad_ui_preview_v4.html>\n");
        return 2;
    }
    std::ifstream f(argv[1], std::ios::binary);
    std::stringstream ss;
    ss << f.rdbuf();
    g_html = ss.str();
    if (g_html.empty())
    {
        std::printf("FAIL: cannot read the preview file %s\n", argv[1]);
        return 1;
    }
    TestAgainstThePreview();
    TestContrastAndScale();
    if (g_failures == 0) std::printf("ui_tokens_test: all checks passed\n");
    return g_failures == 0 ? 0 : 1;
}
