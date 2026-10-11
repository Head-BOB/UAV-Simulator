// Tier: V
// Design tokens of the approved UI preview v4 (dev-docs/ui-reference/quad_ui_preview_v4.html), the visual source of truth.
// Engine-free so tools/cpp_ffi_check/ui_tokens_test.cpp can compare every number with the preview file and check contrast.
// Design space is 1920 x 1080; the app scales it to the viewport (fit-to-width). Text never goes below 14 px after scaling.
// Colours are 0xRRGGBB plus an alpha where the preview uses one. Where the written spec and the preview differ (secondary text
// is "ink 70 / 46 %" in the spec, white at 70 / 46 % in the preview) the preview is followed; see dev-docs/UI-SPEC.md.
#pragma once

#include <cstdint>

namespace DroneApp
{
namespace Ui
{
constexpr double kDesignWidthPx = 1920.0;
constexpr double kDesignHeightPx = 1080.0;
constexpr double kGridPx = 8.0;
constexpr double kLinePx = 1.0;

struct Colour
{
    std::uint32_t Rgb;
    double Alpha;
};

// Text and lines (the preview's --t1, --t2, --t3, --ln, --ln2).
constexpr Colour kInk = {0xF2F4F6, 1.0};
constexpr Colour kInk70 = {0xFFFFFF, 0.70};
constexpr Colour kInk46 = {0xFFFFFF, 0.46};
constexpr Colour kLine = {0xFFFFFF, 0.13};
constexpr Colour kLineStrong = {0xFFFFFF, 0.26};
// Surfaces.
constexpr Colour kBar = {0x0C0E12, 0.86};         // top bar, instrument bar, setup scrim
constexpr Colour kSolid = {0x13171C, 1.0};        // detail panel, controls dock
constexpr Colour kReportBackground = {0x101316, 1.0};
// Status and accent: the only colours besides ink.
constexpr std::uint32_t kAccent = 0x8FBCE8;
constexpr std::uint32_t kOk = 0x5FBD84;
constexpr std::uint32_t kWarn = 0xEAB34C;
constexpr std::uint32_t kCrit = 0xE5655D;
constexpr std::uint32_t kOnPrimaryButton = 0x101318;   // text on the filled ink button
constexpr std::uint32_t kOnWarnBar = 0x15110A;         // text on the amber caution bar

// Type sizes in design px (preview values).
constexpr double kLabelPx = 14.0;       // axis labels, chips, units under values
constexpr double kLabelLargePx = 15.0;
constexpr double kBodyPx = 17.0;
constexpr double kValuePx = 20.0;
constexpr double kKeyValuePx = 46.0;    // instrument bar values, weight Light (300)
constexpr double kSetupRowPx = 30.0;    // setup list rows, weight Light
constexpr double kTitleMinPx = 24.0;
constexpr double kTitleMaxPx = 56.0;    // screen titles, weight Light
constexpr double kMinTextPx = 14.0;

// Regions of the Run screen (design px).
constexpr double kTopBarHeightPx = 56.0;
constexpr double kInstrumentBarHeightPx = 128.0;
constexpr int kInstrumentColumns = 7;           // altitude, ground speed, vertical speed, heading, wind, motors, battery
constexpr double kInstrumentBarPaddingPx = 24.0;   // left and right
constexpr double kTopBarPaddingPx = 48.0;
constexpr double kDetailPanelWidthPx = 460.0;   // right, key I
constexpr double kDetailPanelPaddingXPx = 30.0;
constexpr double kControlsDockWidthPx = 440.0;  // left, key Tab
constexpr double kControlsDockPaddingXPx = 34.0;
constexpr double kHistoryChartHeightPx = 122.0;
constexpr double kHistorySpanSeconds = 120.0;   // the "-120 s" label
constexpr double kTopOfCautionBarPx = 84.0;
// Setup / selection screens.
constexpr double kSetupScrimWidthPx = 700.0;
constexpr double kSetupColumnLeftPx = 96.0;
constexpr double kSetupColumnWidthPx = 560.0;
// Report screen.
constexpr double kReportMarginXPx = 120.0;
constexpr double kReportColumnGapPx = 100.0;

// Width of one instrument-bar column in design px.
constexpr double InstrumentColumnWidthPx(double ScreenWidthPx = kDesignWidthPx)
{
    return (ScreenWidthPx - 2.0 * kInstrumentBarPaddingPx) / kInstrumentColumns;
}

// Scale factor from design space to a viewport: fit-to-width, never below the factor that keeps 14 px text readable
// is NOT applied here (a viewport that small is refused by the caller); non-positive width gives 1.
constexpr double ScaleForWidth(double ViewportWidthPx)
{
    return ViewportWidthPx > 0.0 ? ViewportWidthPx / kDesignWidthPx : 1.0;
}

// Placement of the 1920 x 1080 design space inside a viewport of any size: ONE uniform scale that makes the whole design fit (the smaller of
// width/1920 and height/1080), centred, so nothing is cut off at the top, bottom or sides (the bars left over show the 3D scene). Fitting
// the width alone cut the step bar and the bottom of the panels off in a 1917 x 1018 window. A viewport point is  x * Scale + OffsetX.
// A non-positive size gives the identity placement. Usable down to 1280 x 720 (scale 0.667; 14 px design text is then about 9 px).
struct Viewport
{
    double Scale;
    double OffsetX;
    double OffsetY;
};
constexpr Viewport FitViewport(double WidthPx, double HeightPx)
{
    if (!(WidthPx > 0.0) || !(HeightPx > 0.0)) return Viewport{1.0, 0.0, 0.0};
    const double Sx = WidthPx / kDesignWidthPx, Sy = HeightPx / kDesignHeightPx;
    const double S = Sx < Sy ? Sx : Sy;
    return Viewport{S, (WidthPx - kDesignWidthPx * S) * 0.5, (HeightPx - kDesignHeightPx * S) * 0.5};
}

// Pixel size of a design-space font after scaling, rounded to a whole pixel; not below one pixel.
constexpr int ScaledPx(double DesignPx, double Scale)
{
    const double V = DesignPx * Scale + 0.5;
    return V < 1.0 ? 1 : static_cast<int>(V);
}

// Smallest viewport width at which a 14 px design label is still drawn at 14 px or more.
constexpr double kMinViewportWidthPx = kDesignWidthPx * (kMinTextPx / kLabelPx);   // = 1920: below it text shrinks under the minimum

// Result of scaling the colour onto a background, as 0xRRGGBB (alpha blended in sRGB, as the preview's CSS does).
constexpr std::uint32_t Blend(Colour Fg, std::uint32_t BgRgb)
{
    const double A = Fg.Alpha;
    auto Ch = [&](int Shift) {
        const double F = static_cast<double>((Fg.Rgb >> Shift) & 0xFF);
        const double B = static_cast<double>((BgRgb >> Shift) & 0xFF);
        const double V = F * A + B * (1.0 - A) + 0.5;
        return static_cast<std::uint32_t>(V);
    };
    return (Ch(16) << 16) | (Ch(8) << 8) | Ch(0);
}
}  // namespace Ui
}  // namespace DroneApp
