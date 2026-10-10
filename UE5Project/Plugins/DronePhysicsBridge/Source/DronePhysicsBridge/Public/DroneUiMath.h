// Tier: V
// DEMO ONLY: the non-Unreal logic of the in-engine UI, so tools/cpp_ffi_check/ui_math_test.cpp tests it with
// plain g++: palette and WCAG contrast, type scale and DPI scale, a critically damped smoother for gauges,
// a fixed-capacity ring buffer for charts, tape and horizon geometry, wind-arrow angle, status colours from
// margins, fixed-width number formatting, panel fades, and a 30 Hz data gate. No allocation, no clock.
// Standard library only.
#pragma once

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>

namespace DroneUi
{
constexpr double kPi = 3.14159265358979323846;

// ---------------------------------------------------------------------------------------------
// Palette (sRGB 0xRRGGBB). Design brief: graphite panels, ONE accent, amber and red only for status.
// ---------------------------------------------------------------------------------------------
constexpr std::uint32_t kPanelTop = 0x0B0E13;     // panel gradient start (drawn at ~85 % opacity)
constexpr std::uint32_t kPanelBottom = 0x121722;  // panel gradient end
constexpr std::uint32_t kHairline = 0x222B3A;     // 1 px borders
constexpr std::uint32_t kAccent = 0x38E1FF;       // electric cyan, the only accent
constexpr std::uint32_t kWarning = 0xFFB020;      // amber, status only
constexpr std::uint32_t kCritical = 0xFF4D5E;     // red, status only
constexpr std::uint32_t kText = 0xE8EDF5;         // off-white
constexpr std::uint32_t kTextMuted = 0x8793A6;    // labels

constexpr double kPanelOpacity = 0.85;
constexpr double kCornerRadiusPx = 6.0;
constexpr double kHairlinePx = 1.0;
constexpr double kGridPx = 8.0;                   // spacing grid
constexpr double kFadeSeconds = 0.15;             // panel transitions

// sRGB channel (0..255) to linear light, 0..1 (IEC 61966-2-1).
inline double SrgbToLinear(int Channel)
{
    const double C = Channel / 255.0;
    return C <= 0.04045 ? C / 12.92 : std::pow((C + 0.055) / 1.055, 2.4);
}

// WCAG 2.x relative luminance of an 0xRRGGBB colour.
inline double RelativeLuminance(std::uint32_t Rgb)
{
    return 0.2126 * SrgbToLinear(static_cast<int>((Rgb >> 16) & 0xFF)) + 0.7152 * SrgbToLinear(static_cast<int>((Rgb >> 8) & 0xFF)) +
           0.0722 * SrgbToLinear(static_cast<int>(Rgb & 0xFF));
}

// WCAG contrast ratio (L1 + 0.05) / (L2 + 0.05), lighter over darker, 1..21.
inline double ContrastRatio(std::uint32_t A, std::uint32_t B)
{
    const double La = RelativeLuminance(A), Lb = RelativeLuminance(B);
    const double Hi = La > Lb ? La : Lb, Lo = La > Lb ? Lb : La;
    return (Hi + 0.05) / (Lo + 0.05);
}

constexpr double kMinTextContrast = 4.5;  // WCAG AA for normal text

// ---------------------------------------------------------------------------------------------
// Type scale and DPI. Sizes in px at 1080p; the smallest is 14 px (readability rule).
// ---------------------------------------------------------------------------------------------
constexpr int kTypeScalePx[5] = {14, 16, 20, 28, 44};  // label, body, value, heading, hero readout
constexpr double kMinTextPx1080 = 14.0;

// Multiplier from the viewport height (px) and the platform DPI scale: 1.0 at 1080p and DPI 1. Non-finite or
// non-positive input returns 1.0.
inline double UiScale(double ViewportHeightPx, double DpiScale)
{
    if (!std::isfinite(ViewportHeightPx) || !std::isfinite(DpiScale) || ViewportHeightPx <= 0.0 || DpiScale <= 0.0) return 1.0;
    return (ViewportHeightPx / 1080.0) * DpiScale;
}

// Snap a length to the 8 px grid, rounding to nearest (halves up).
inline double SnapToGrid(double Px)
{
    return std::floor(Px / kGridPx + 0.5) * kGridPx;
}

// ---------------------------------------------------------------------------------------------
// Critically damped smoothing (gauge needles and bars). Exact solution of x'' = -2w x' - w^2 (x - target) over a
// step dt, so it is stable for any dt and never overshoots when started at rest:
//   d = x - target, e = exp(-w dt)
//   d' = (d + (v + w d) dt) e,   v' = (v - (v + w d) w dt) e
// ---------------------------------------------------------------------------------------------
struct Smoother
{
    double X = 0.0;
    double V = 0.0;
    bool bSet = false;

    // Jump to a value with no velocity (first sample, or a reset).
    void Snap(double Value)
    {
        if (!std::isfinite(Value)) return;
        X = Value;
        V = 0.0;
        bSet = true;
    }

    // Advance toward Target. Omega in 1/s (about 6 settles in half a second). Invalid input is ignored.
    void Step(double Target, double OmegaPerS, double DtS)
    {
        if (!std::isfinite(Target) || !std::isfinite(OmegaPerS) || !std::isfinite(DtS) || OmegaPerS <= 0.0 || DtS < 0.0) return;
        if (!bSet)
        {
            Snap(Target);
            return;
        }
        const double D = X - Target;
        const double E = std::exp(-OmegaPerS * DtS);
        const double Tmp = V + OmegaPerS * D;
        X = Target + (D + Tmp * DtS) * E;
        V = (V - Tmp * OmegaPerS * DtS) * E;
    }
};

// ---------------------------------------------------------------------------------------------
// Fixed-capacity ring buffer (charts): Push overwrites the oldest value, no allocation.
// ---------------------------------------------------------------------------------------------
template <std::size_t Capacity>
struct Ring
{
    static_assert(Capacity > 0, "capacity must be positive");
    double Data[Capacity] = {};
    std::size_t Head = 0;   // next write index
    std::size_t Count = 0;

    void Push(double V)
    {
        if (!std::isfinite(V)) return;  // a bad sample never reaches a chart
        Data[Head] = V;
        Head = (Head + 1) % Capacity;
        if (Count < Capacity) ++Count;
    }
    std::size_t Size() const { return Count; }
    // Oldest-first index (0 = oldest). Out of range returns 0.
    double At(std::size_t I) const
    {
        if (I >= Count) return 0.0;
        const std::size_t Start = (Head + Capacity - Count) % Capacity;
        return Data[(Start + I) % Capacity];
    }
    void Clear()
    {
        Head = 0;
        Count = 0;
    }
    // Min and max of the stored values; false if empty.
    bool MinMax(double* Lo, double* Hi) const
    {
        if (Count == 0 || !Lo || !Hi) return false;
        double L = At(0), H = At(0);
        for (std::size_t I = 1; I < Count; ++I)
        {
            const double V = At(I);
            if (V < L) L = V;
            if (V > H) H = V;
        }
        *Lo = L;
        *Hi = H;
        return true;
    }
};

// ---------------------------------------------------------------------------------------------
// Tape (altitude / speed) ticks. Screen y grows downward: a value ABOVE the centre reading is drawn above
// (negative offset), so OffsetPx = (Center - Value) * PxPerUnit.
// ---------------------------------------------------------------------------------------------
struct Tick
{
    double OffsetPx;
    double Value;
    bool bMajor;
};

// Ticks for values in [Center - HalfSpan, Center + HalfSpan] at multiples of MinorStep; every
// MinorPerMajor-th multiple is major. Writes at most MaxOut ticks (lowest value first) and returns how many.
inline int ComputeTicks(double Center, double HalfSpan, double MinorStep, int MinorPerMajor, double PxPerUnit, Tick* Out, int MaxOut)
{
    if (!Out || MaxOut <= 0 || !std::isfinite(Center) || !std::isfinite(HalfSpan) || !std::isfinite(MinorStep) || !std::isfinite(PxPerUnit)) return 0;
    if (HalfSpan <= 0.0 || MinorStep <= 0.0 || MinorPerMajor <= 0 || PxPerUnit <= 0.0) return 0;
    const double First = std::ceil((Center - HalfSpan) / MinorStep);
    const double Last = std::floor((Center + HalfSpan) / MinorStep);
    int N = 0;
    for (double K = First; K <= Last && N < MaxOut; K += 1.0)
    {
        const double Value = K * MinorStep;
        const long long Idx = static_cast<long long>(K);
        const long long Mod = Idx % MinorPerMajor;
        Out[N++] = Tick{(Center - Value) * PxPerUnit, Value, Mod == 0};
    }
    return N;
}

// ---------------------------------------------------------------------------------------------
// Artificial horizon. Local frame: x to the right, y DOWN, origin at the instrument centre. Pitch up moves the
// horizon DOWN; roll right (right wing down) makes the right end of the horizon go UP.
// ---------------------------------------------------------------------------------------------
struct Point
{
    double X;
    double Y;
};

inline void HorizonEnds(double RollRad, double PitchRad, double PxPerRad, double HalfWidthPx, Point* Left, Point* Right)
{
    if (!Left || !Right) return;
    const double Shift = PitchRad * PxPerRad;  // down
    const double C = std::cos(RollRad), S = std::sin(RollRad);
    // Local points (-W, Shift) and (+W, Shift), rotated: (x c + y s, -x s + y c).
    *Left = Point{-HalfWidthPx * C + Shift * S, HalfWidthPx * S + Shift * C};
    *Right = Point{HalfWidthPx * C + Shift * S, -HalfWidthPx * S + Shift * C};
}

// ---------------------------------------------------------------------------------------------
// Heading and wind arrow.
// ---------------------------------------------------------------------------------------------
// Degrees wrapped into [0, 360).
inline double WrapDeg360(double Deg)
{
    if (!std::isfinite(Deg)) return 0.0;
    double W = std::fmod(Deg, 360.0);
    if (W < 0.0) W += 360.0;
    return W >= 360.0 ? 0.0 : W;
}

// Angle of the wind arrow on screen, degrees clockwise from "up = the nose", in (-180, 180]. The wind
// blows TOWARD WindTowardDeg (from north, clockwise) and the nose points to HeadingDeg.
inline double WindArrowDeg(double WindTowardDeg, double HeadingDeg)
{
    double D = WrapDeg360(WindTowardDeg - HeadingDeg);
    if (D > 180.0) D -= 360.0;
    return D;
}

// ---------------------------------------------------------------------------------------------
// Status colours from margins (margin = limit / peak, > 1 is inside the limit), same thresholds as the report.
// ---------------------------------------------------------------------------------------------
enum class Status
{
    Ok,        // accent colour
    Warning,   // amber
    Critical,  // red
};

constexpr double kOkMargin = 1.5;
constexpr double kWarnMargin = 1.0;

inline Status StatusFromMargin(double Margin)
{
    if (!std::isfinite(Margin) || Margin < kWarnMargin) return Status::Critical;
    return Margin >= kOkMargin ? Status::Ok : Status::Warning;
}

// Utilisation = load / limit (0..), the inverse of the margin: warning from 1/1.5, critical from 1.
inline Status StatusFromUtilisation(double Utilisation)
{
    if (!std::isfinite(Utilisation) || Utilisation >= 1.0) return Status::Critical;
    return Utilisation >= 1.0 / kOkMargin ? Status::Warning : Status::Ok;
}

inline std::uint32_t StatusRgb(Status S)
{
    return S == Status::Ok ? kAccent : (S == Status::Warning ? kWarning : kCritical);
}

// ---------------------------------------------------------------------------------------------
// Number formatting into caller buffers (no allocation). Non-finite values print "--".
// ---------------------------------------------------------------------------------------------
// Right-aligned fixed width and decimals, e.g. FormatFixed(b, 16, 3.14159, 2, 7) -> "   3.14". Returns the length.
inline int FormatFixed(char* Buf, std::size_t Size, double Value, int Decimals, int Width)
{
    if (!Buf || Size == 0) return 0;
    if (Decimals < 0) Decimals = 0;
    if (Decimals > 6) Decimals = 6;
    if (!std::isfinite(Value)) return std::snprintf(Buf, Size, "%*s", Width, "--");
    return std::snprintf(Buf, Size, "%*.*f", Width, Decimals, Value);
}

// "MM:SS" for a time in seconds (non-negative, capped at 99:59).
inline int FormatClock(char* Buf, std::size_t Size, double Seconds)
{
    if (!Buf || Size == 0) return 0;
    if (!std::isfinite(Seconds) || Seconds < 0.0) return std::snprintf(Buf, Size, "--:--");
    long Total = static_cast<long>(Seconds);
    if (Total > 99 * 60 + 59) Total = 99 * 60 + 59;
    return std::snprintf(Buf, Size, "%02ld:%02ld", Total / 60, Total % 60);
}

// ---------------------------------------------------------------------------------------------
// Panel fade and the 30 Hz data gate.
// ---------------------------------------------------------------------------------------------
// Moves Alpha toward Target (0 or 1) linearly so a full fade takes kFadeSeconds. Result in [0, 1].
inline double FadeStep(double Alpha, double Target, double DtS)
{
    if (!std::isfinite(Alpha) || !std::isfinite(Target) || !std::isfinite(DtS) || DtS < 0.0) return Alpha;
    const double Step = DtS / kFadeSeconds;
    double A = Alpha;
    if (A < Target) A = A + Step > Target ? Target : A + Step;
    else if (A > Target) A = A - Step < Target ? Target : A - Step;
    return A < 0.0 ? 0.0 : (A > 1.0 ? 1.0 : A);
}

constexpr double kDataRateHz = 30.0;

// True when a data update is due; keeps the remainder so the long-run rate is exactly kDataRateHz.
struct DataGate
{
    double Accum = 0.0;
    bool Due(double DtS)
    {
        if (!std::isfinite(DtS) || DtS < 0.0) return false;
        Accum += DtS;
        const double Period = 1.0 / kDataRateHz;
        if (Accum < Period) return false;
        Accum -= Period;
        if (Accum >= Period) Accum = 0.0;  // after a long stall: one update, not a burst of catch-up updates
        return true;
    }
};
}  // namespace DroneUi
