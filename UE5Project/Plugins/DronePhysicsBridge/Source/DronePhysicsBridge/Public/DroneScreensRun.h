// Tier: V
// Engine-free layout of the Run screen (replay of a recorded campaign run): top bar 56 px, instrument bar 128 px in 7 equal columns,
// caution bar, detail panel (key I, 460 px, solid), replay controls dock (key Tab, 440 px, solid). The centre stays clear for the
// 3D view. Everything shown is a recorded value or a margin computed from one; nothing is simulated here.
#pragma once

#include <algorithm>
#include <cmath>
#include <string>

#include "DroneScreensFly.h"
#include "DroneScreensProgress.h"
#include "DroneScreensSetup.h"

namespace DroneApp
{
namespace Screens
{
// Heat tint for motors / ESC / battery in the 3D view and the detail panel: neutral to amber to red as the temperature rise approaches the
// limit's rise (fraction 0..1+). The same function is used by the viewer, so the colours match the numbers.
inline double HeatFraction(double TempC, double LimitC, double AmbientC)
{
    if (!(LimitC > AmbientC)) return 0.0;
    const double F = (TempC - AmbientC) / (LimitC - AmbientC);
    return F < 0.0 ? 0.0 : F;
}
inline Rgba LerpRgba(const Rgba& A, const Rgba& B, float T) { return Rgba{A.R + (B.R - A.R) * T, A.G + (B.G - A.G) * T, A.B + (B.B - A.B) * T, A.A + (B.A - A.A) * T}; }
inline Rgba HeatTint(double Fraction)
{
    if (Fraction < 0.5) return Palette::Ink70();
    if (Fraction < 0.8) return LerpRgba(Palette::Ink70(), Palette::Warn(), static_cast<float>((Fraction - 0.5) / 0.3));
    if (Fraction < 1.0) return LerpRgba(Palette::Warn(), Palette::Crit(), static_cast<float>((Fraction - 0.8) / 0.2));
    return Palette::Crit();
}

inline std::string Num1(double V) { char B[32]; std::snprintf(B, sizeof B, std::fabs(V) >= 100.0 ? "%.0f" : "%.1f", V); return B; }
inline std::string Signed1(double V) { char B[32]; std::snprintf(B, sizeof B, "%+.1f", V); return B; }

inline void Instrument(DrawList& L, int Col, const std::string& Label, const std::string& Value, const std::string& Unit, const std::string& Sub)
{
    const float Cw = static_cast<float>(Ui::InstrumentColumnWidthPx());
    const float X = static_cast<float>(Ui::kInstrumentBarPaddingPx) + Cw * static_cast<float>(Col);
    const float Y = Hgt - static_cast<float>(Ui::kInstrumentBarHeightPx);
    if (Col > 0) L.VLine(X, Y + 12.0f, static_cast<float>(Ui::kInstrumentBarHeightPx) - 24.0f, Palette::Line());
    L.Text(X + 24.0f, Y + 22.0f, Cw - 36.0f, Label, 14.0f, Palette::Ink70());
    L.Text(X + 24.0f, Y + 40.0f, Cw - 36.0f, Value, 46.0f, Palette::Ink(), Weight::Light);
    L.Text(X + 24.0f + EstimateTextWidth(Value, 46.0f) * 0.92f + 8.0f, Y + 62.0f, 80.0f, Unit, 17.0f, Palette::Ink70());
    if (!Sub.empty()) L.Text(X + 24.0f, Y + 98.0f, Cw - 36.0f, Sub, 15.0f, Palette::Ink46());
}

inline void Chart(DrawList& L, float X, float Y, float Wd, float Ht, const std::string& Title, const std::string& Unit, const std::vector<Replay::Point>& Pts, double SpanS)
{
    L.Text(X, Y, Wd, Title, 14.0f, Palette::Ink70());
    const float Top = Y + 22.0f, Bot = Y + Ht - 20.0f;
    L.HLine(X, Bot, Wd, Palette::LineStrong());
    L.VLine(X, Top, Bot - Top, Palette::Line());
    double Lo = 1e300, Hi = -1e300;
    for (const auto& P : Pts) { Lo = std::min(Lo, P.V); Hi = std::max(Hi, P.V); }
    if (Pts.empty()) { L.Text(X + 8.0f, Top + 20.0f, Wd, "no data yet", 14.0f, Palette::Ink46()); return; }
    if (Hi - Lo < 1e-9) { Lo -= 0.5; Hi += 0.5; }
    std::vector<Pt> Line;
    for (const auto& P : Pts)
        Line.push_back(Pt{X + static_cast<float>((P.T + SpanS) / SpanS) * Wd, Bot - static_cast<float>((P.V - Lo) / (Hi - Lo)) * (Bot - Top)});
    L.Poly(Line, Palette::Accent(), 1.5f);
    L.Text(X + Wd - 90.0f, Y, 90.0f, Num1(Hi) + " " + Unit, 14.0f, Palette::Ink46(), Weight::Regular, Align::Right);
    L.Text(X + Wd - 90.0f, Bot - 18.0f, 90.0f, Num1(Lo), 14.0f, Palette::Ink46(), Weight::Regular, Align::Right);
    L.Text(X, Bot + 2.0f, 100.0f, "-" + Num1(SpanS) + " s", 14.0f, Palette::Ink46());
    L.Text(X + Wd - 60.0f, Bot + 2.0f, 60.0f, "now", 14.0f, Palette::Ink46(), Weight::Regular, Align::Right);
}

inline void Run(DrawList& L, const AppModel& M)
{
    const float Top = static_cast<float>(Ui::kTopBarHeightPx), BarH = static_cast<float>(Ui::kInstrumentBarHeightPx);
    const Pose& P = M.Now;
    // top bar
    L.Fill(0, 0, W, Top, Palette::Bar());
    L.HLine(0, Top, W, Palette::Line());
    L.Text(static_cast<float>(Ui::kTopBarPaddingPx), 14.0f, 700.0f, M.ReplayTest.empty() ? "Replay" : M.ReplayTest, 20.0f, Palette::Ink(), Weight::Medium);
    L.Text(static_cast<float>(Ui::kTopBarPaddingPx) + 260.0f, 17.0f, 700.0f,
           std::string("Recorded run, not live physics   ") + (M.Rep.Playing() ? "playing " : "paused ") + Num1(M.Rep.Speed()) + "x", 17.0f, Palette::Ink70());
    {
        // what is on screen: test, simulated time, speed and the TRUE (recorded) motor rpm, on a dark strip so it reads over the sky
        const float Lx = static_cast<float>(Ui::kTopBarPaddingPx), Ly = Top + 10.0f;
        const std::string Info = (M.ReplayTest.empty() ? std::string("no test") : TestLabelOf(M.ReplayTest)) + "     sim time " + Num1(P.TimeS) + " / " + Num1(M.Rep.End()) +
                                 " s     speed " + Num1(P.GroundSpeedMps()) + " m/s     rpm " + Fmt("%.0f", P.Rpm[0]) + " / " + Fmt("%.0f", P.Rpm[1]) + " / " +
                                 Fmt("%.0f", P.Rpm[2]) + " / " + Fmt("%.0f", P.Rpm[3]);
        L.Fill(Lx - 8.0f, Ly - 4.0f, EstimateTextWidth(Info, 16.0f) + 24.0f, 30.0f, Palette::Solid());
        L.Text(Lx, Ly, 1000.0f, Info, 16.0f, Palette::Ink(), Weight::Regular);
        if (M.RunWasQuick)
        {
            L.Fill(Lx - 8.0f, Ly + 34.0f, 340.0f, 30.0f, Palette::Solid());
            Chip(L, Lx, Ly + 36.0f, "Quick profile: shortened tests", Palette::Warn());
        }
    }
    {
        const int Lim = LimitingIndex(M.LiveRows);
        if (Lim >= 0)
        {
            const MarginRow& R = M.LiveRows[static_cast<std::size_t>(Lim)];
            const Rgba C = StatusRgba(StatusOfMargin(R.Margin));
            const std::string T = R.Label + "  " + (R.Unit == "n/a" ? std::string("n/a") : Fmt("%.2f", R.Margin));
            const float Cw = Chip(L, W - static_cast<float>(Ui::kTopBarPaddingPx) - 560.0f, 17.0f, T, C);
            L.Text(W - static_cast<float>(Ui::kTopBarPaddingPx) - 560.0f + Cw + 24.0f, 17.0f, 360.0f, Num1(P.TimeS) + " / " + Num1(M.Rep.End()) + " s", 17.0f, Palette::Ink70(), Weight::Regular, Align::Right);
        }
    }
    // caution bar
    {
        const int Lim = LimitingIndex(M.LiveRows);
        if (Lim >= 0 && M.LiveRows[static_cast<std::size_t>(Lim)].Margin < kCautionMargin && M.LiveRows[static_cast<std::size_t>(Lim)].Unit != "n/a")
        {
            const MarginRow& R = M.LiveRows[static_cast<std::size_t>(Lim)];
            const std::string T = "Caution: " + R.Label + " margin " + Fmt("%.2f", R.Margin);
            const float Wd = EstimateTextWidth(T, 18.0f) + 44.0f;
            L.Fill((W - Wd) * 0.5f, static_cast<float>(Ui::kTopOfCautionBarPx), Wd, 42.0f, Palette::Warn());
            L.Text((W - Wd) * 0.5f, static_cast<float>(Ui::kTopOfCautionBarPx) + 10.0f, Wd, T, 18.0f, Palette::OnWarn(), Weight::Medium, Align::Centre);
        }
    }
    // wind of the recorded sample, and the way into fly mode
    WindRose(L, W - 120.0f, 250.0f, 54.0f, P.WindNed, 0.0);
    Button(L, W - 220.0f, Top + 8.0f, 180.0f, 36.0f, "Fly it yourself", false, Act::FlyIt);
    // instrument bar
    L.Fill(0, Hgt - BarH, W, BarH, Palette::Bar());
    L.HLine(0, Hgt - BarH, W, Palette::Line());
    const double Wind = std::hypot(P.WindNed[0], P.WindNed[1]);
    Instrument(L, 0, "Altitude", Num1(P.AltitudeM()), "m", "");
    Instrument(L, 1, "Ground speed", Num1(P.GroundSpeedMps()), "m/s", "air " + Num1(P.AirSpeedMps) + " m/s");
    Instrument(L, 2, "Vertical speed", Signed1(P.VerticalSpeedMps()), "m/s", "");
    Instrument(L, 3, "Heading", Fmt("%.0f", P.HeadingDeg()), "deg", "roll " + Signed1(P.RollDeg) + "  pitch " + Signed1(P.PitchDeg));
    Instrument(L, 4, "Wind", Num1(Wind), "m/s", "");
    {   // wind arrow relative to the nose (up = nose): direction the wind blows toward, minus heading
        const float Cw = static_cast<float>(Ui::InstrumentColumnWidthPx());
        const float Cx = static_cast<float>(Ui::kInstrumentBarPaddingPx) + Cw * 4.0f + Cw - 54.0f, Cy = Hgt - BarH + 62.0f;
        const double Rel = std::atan2(P.WindNed[1], P.WindNed[0]) - P.YawDeg * kPi / 180.0;   // 0 = along the nose
        const float Dx = static_cast<float>(std::sin(Rel)) * 20.0f, Dy = -static_cast<float>(std::cos(Rel)) * 20.0f;
        if (Wind > 0.05)
        {
            L.Line(Cx - Dx, Cy - Dy, Cx + Dx, Cy + Dy, Palette::Accent(), 2.0f);
            const float Hx = -Dy * 0.35f, Hy = Dx * 0.35f;
            L.Line(Cx + Dx, Cy + Dy, Cx + Dx * 0.4f + Hx, Cy + Dy * 0.4f + Hy, Palette::Accent(), 2.0f);
            L.Line(Cx + Dx, Cy + Dy, Cx + Dx * 0.4f - Hx, Cy + Dy * 0.4f - Hy, Palette::Accent(), 2.0f);
        }
    }
    Instrument(L, 5, "Motors", "", "", "");
    {
        const float Cw = static_cast<float>(Ui::InstrumentColumnWidthPx());
        const float X0 = static_cast<float>(Ui::kInstrumentBarPaddingPx) + Cw * 5.0f + 24.0f, Base = Hgt - 18.0f;
        for (int K = 0; K < 4; ++K)
        {
            const float Hh = static_cast<float>(std::min(1.0, P.Rpm[K] / std::max(1.0, M.RpmScale))) * 40.0f;
            L.Fill(X0 + static_cast<float>(K) * 22.0f, Base - 44.0f, 16.0f, 44.0f, Palette::Line());
            L.Fill(X0 + static_cast<float>(K) * 22.0f, Base - Hh, 16.0f, Hh, Palette::Ink());
        }
        L.Text(X0 + 100.0f, Hgt - BarH + 40.0f, 130.0f, Fmt("%.0f", (P.Rpm[0] + P.Rpm[1] + P.Rpm[2] + P.Rpm[3]) / 4.0), 28.0f, Palette::Ink(), Weight::Light);
        L.Text(X0 + 100.0f, Hgt - BarH + 76.0f, 130.0f, "mean rpm", 14.0f, Palette::Ink46());
    }
    Instrument(L, 6, "Battery", Fmt("%.1f", P.BatteryV), "V", Num1(P.BatteryA) + " A   " + Fmt("%.0f", P.BatterySoc * 100.0) + " %");
    const float Lx = M.ShowControls ? static_cast<float>(Ui::kControlsDockWidthPx) + 24.0f : static_cast<float>(Ui::kTopBarPaddingPx);
    const float Rxe = M.ShowDetails ? W - static_cast<float>(Ui::kDetailPanelWidthPx) - 24.0f : W - static_cast<float>(Ui::kTopBarPaddingPx);
    L.Text(Lx, Hgt - BarH - 34.0f, Rxe - Lx, "Space play / pause     I details     Tab replay controls     Esc results", 14.0f, Palette::Ink70(), Weight::Regular, Align::Left, true);
    L.Text(Lx, Hgt - BarH - 58.0f, Rxe - Lx,
           "Recorded run, not live physics. Drag: " + (M.Camp.Common.Get("drag_model") ? M.Camp.Common.Get("drag_model")->AsString("?") : std::string("?")) + (M.Report.Smoke() ? "   SMOKE definitions" : ""), 14.0f, Palette::Ink70(), Weight::Regular, Align::Left, true);

    // detail panel (I)
    if (M.ShowDetails)
    {
        const float Dx = W - static_cast<float>(Ui::kDetailPanelWidthPx), Pad = static_cast<float>(Ui::kDetailPanelPaddingXPx), Iw = static_cast<float>(Ui::kDetailPanelWidthPx) - 2.0f * Pad;
        L.Fill(Dx, Top, static_cast<float>(Ui::kDetailPanelWidthPx), Hgt - Top - BarH, Palette::Solid());
        L.VLine(Dx, Top, Hgt - Top - BarH, Palette::LineStrong());
        float Y = Top + 22.0f;
        L.Text(Dx + Pad, Y, Iw, "Details", 22.0f, Palette::Ink(), Weight::Medium);
        L.Text(Dx + Pad, Y + 6.0f, Iw, "I to close", 14.0f, Palette::Ink46(), Weight::Regular, Align::Right);
        Y += 44.0f;
        L.Text(Dx + Pad, Y, 200.0f, "Motors", 15.0f, Palette::Ink70(), Weight::Medium);
        for (int C = 0; C < 3; ++C) L.Text(Dx + Pad + Iw - 270.0f + static_cast<float>(C) * 90.0f, Y, 90.0f, C == 0 ? "rpm" : (C == 1 ? "A" : "N"), 14.0f, Palette::Ink46(), Weight::Regular, Align::Right);
        Y += 22.0f;
        static const char* Names[4] = {"FR", "FL", "RR", "RL"};
        for (int K = 0; K < 4; ++K)
        {
            L.HLine(Dx + Pad, Y, Iw, Palette::Line());
            L.Text(Dx + Pad, Y + 5.0f, 100.0f, Names[K], 16.0f, Palette::Ink70());
            L.Text(Dx + Pad + Iw - 270.0f, Y + 2.0f, 90.0f, Fmt("%.0f", P.Rpm[K]), 18.0f, Palette::Ink(), Weight::Regular, Align::Right);
            L.Text(Dx + Pad + Iw - 180.0f, Y + 2.0f, 90.0f, Fmt("%.1f", P.CurrentA[K]), 18.0f, Palette::Ink(), Weight::Regular, Align::Right);
            L.Text(Dx + Pad + Iw - 90.0f, Y + 2.0f, 90.0f, Fmt("%.2f", P.ThrustN[K]), 18.0f, Palette::Ink(), Weight::Regular, Align::Right);
            Y += 26.0f;
        }
        Y += 6.0f;
        L.Text(Dx + Pad, Y, 200.0f, "Power and temperatures", 15.0f, Palette::Ink70(), Weight::Medium);
        Y += 22.0f;
        const double Amb = M.Camp.Ambient();
        auto Limit = [&](const char* K) { const Json::Value* V = M.Powertrain.Get(K); return V ? V->AsNumber(0.0) : 0.0; };
        struct TRow { const char* Label; double T; double Lim; };
        const TRow Ts[4] = {{"Winding", P.WindingC, Limit("motor_winding_temp_limit_c")}, {"Motor case", P.CaseC, Limit("motor_winding_temp_limit_c")},
                            {"ESC", P.EscC, Limit("esc_temp_limit_c")}, {"Battery", P.BatteryC, Limit("battery_temp_limit_c")}};
        L.Text(Dx + Pad, Y, 200.0f, "Pack power", 16.0f, Palette::Ink70());
        L.Text(Dx + Pad, Y - 2.0f, Iw, Fmt("%.0f", P.BatteryW) + " W", 18.0f, Palette::Ink(), Weight::Regular, Align::Right);
        Y += 26.0f;
        for (const auto& T : Ts)
        {
            L.HLine(Dx + Pad, Y, Iw, Palette::Line());
            L.Fill(Dx + Pad, Y + 9.0f, 12.0f, 12.0f, HeatTint(HeatFraction(T.T, T.Lim, Amb)));
            L.Text(Dx + Pad + 24.0f, Y + 4.0f, 200.0f, T.Label, 16.0f, Palette::Ink70());
            L.Text(Dx + Pad, Y + 2.0f, Iw, Fmt("%.1f", T.T) + " C", 18.0f, Palette::Ink(), Weight::Regular, Align::Right);
            Y += 26.0f;
        }
        Y += 8.0f;
        L.Text(Dx + Pad, Y, 200.0f, "Margins", 15.0f, Palette::Ink70(), Weight::Medium);
        L.Text(Dx + Pad, Y, Iw, "limit = 1.0", 14.0f, Palette::Ink46(), Weight::Regular, Align::Right);
        Y += 22.0f;
        for (const auto& R : M.LiveRows)
        {
            const bool NA = R.Unit == "n/a";
            const Status St = StatusOfMargin(R.Margin);
            L.HLine(Dx + Pad, Y, Iw, Palette::Line());
            L.Text(Dx + Pad, Y + 3.0f, 260.0f, R.Label, 15.0f, Palette::Ink70());
            if (NA)
            {
                L.Text(Dx + Pad, Y + 2.0f, Iw, "n/a", 16.0f, Palette::Ink46(), Weight::Regular, Align::Right);
            }
            else
            {
                L.Text(Dx + Pad, Y + 2.0f, Iw - 68.0f, R.Margin >= kMarginCap ? std::string(R.NoHeating ? "no heating" : "no load") : Fmt("%.2f", R.Margin), 16.0f, Palette::Ink(), Weight::Regular, Align::Right);
                Chip(L, Dx + Pad + Iw - 56.0f, Y + 2.0f, St == Status::Ok ? "OK" : (St == Status::Caution ? "LOW" : "OVER"), StatusRgba(St));
                const float BarW = Iw - 66.0f;
                const float Frac = static_cast<float>(std::min(1.0, R.Margin / 3.0));
                L.Fill(Dx + Pad, Y + 27.0f, BarW, 3.0f, Palette::Line());
                L.Fill(Dx + Pad, Y + 27.0f, BarW * Frac, 3.0f, StatusRgba(St));
                L.Fill(Dx + Pad + BarW / 3.0f, Y + 23.0f, 1.0f, 11.0f, Palette::Ink());   // the limit: margin 1.0 on a 0..3 scale
            }
            Y += 34.0f;
        }
        Y += 6.0f;
        std::vector<Replay::Point> H1, H2;
        M.Rep.History("winding_c_max", Ui::kHistorySpanSeconds, H1);
        M.Rep.History("battery_a", Ui::kHistorySpanSeconds, H2);
        Chart(L, Dx + Pad, Y, Iw, static_cast<float>(Ui::kHistoryChartHeightPx), "History: winding temperature", "C", H1, Ui::kHistorySpanSeconds);
        Chart(L, Dx + Pad, Y + static_cast<float>(Ui::kHistoryChartHeightPx) + 4.0f, Iw, static_cast<float>(Ui::kHistoryChartHeightPx), "History: pack current", "A", H2, Ui::kHistorySpanSeconds);
    }

    // replay controls dock (Tab)
    if (M.ShowControls)
    {
        const float Dw = static_cast<float>(Ui::kControlsDockWidthPx), Pad = static_cast<float>(Ui::kControlsDockPaddingXPx), Iw = Dw - 2.0f * Pad;
        L.Fill(0, Top, Dw, Hgt - Top - BarH, Palette::Solid());
        L.VLine(Dw, Top, Hgt - Top - BarH, Palette::LineStrong());
        // Fixed footer: Results / Conditions buttons + footer note (above the instrument bar)
        const float FooterY = Hgt - BarH - 82.0f;
        L.HLine(Pad, FooterY, Iw, Palette::Line());
        Button(L, Pad, FooterY + 8.0f, Iw / 2.0f - 6.0f, 44.0f, "Results", true, Act::Navigate, "Results");
        Button(L, Pad + Iw / 2.0f + 6.0f, FooterY + 8.0f, Iw / 2.0f - 6.0f, 44.0f, "Conditions", false, Act::Navigate, "Conditions");
        L.Text(Pad, FooterY + 58.0f, Iw, "Replay only: edit conditions and run again to change a result.", 14.0f, Palette::Ink46());
        // Scrollable content area above the footer
        float Y = Top + 26.0f;
        L.Text(Pad, Y, Iw, "Replay", 24.0f, Palette::Ink(), Weight::Medium);
        L.Text(Pad, Y + 8.0f, Iw, "Tab to close", 14.0f, Palette::Ink46(), Weight::Regular, Align::Right);
        Y += 52.0f;
        L.Text(Pad, Y, Iw, "Recorded tests", 15.0f, Palette::Ink70());
        Y += 26.0f;
        for (const auto& T2 : M.Camp.Tests)
        {
            if (T2.SeriesFiles.empty() || Y >= FooterY - 4.0f) continue;
            const bool Sel = T2.Name == M.ReplayTest;
            L.HLine(Pad, Y, Iw, Palette::Line());
            L.Text(Pad, Y + 8.0f, Iw, T2.Name, 20.0f, Sel ? Palette::Ink() : Palette::Ink46());
            L.Text(Pad, Y + 12.0f, Iw, T2.Outcome(), 14.0f, Palette::Ink46(), Weight::Regular, Align::Right);
            L.AddHit(Pad, Y, Iw, 38.0f, Act::PickTest, T2.Name);
            Y += 38.0f;
        }
        L.HLine(Pad, Y, Iw, Palette::Line());
        Y += 24.0f;
        if (Y < FooterY - 110.0f)
        {
            // Show time in mm:ss format
            auto MmSs = [](double S) -> std::string {
                if (!(S >= 0.0)) return "0:00";
                const int Ss = static_cast<int>(S);
                char B[16]; std::snprintf(B, sizeof B, "%d:%02d", Ss / 60, Ss % 60);
                return B;
            };
            L.Text(Pad, Y, Iw, "Time", 15.0f, Palette::Ink70());
            const std::string TimeStr = MmSs(P.TimeS) + " / " + MmSs(M.Rep.End());
            L.Text(Pad, Y + 20.0f, Iw, TimeStr, 32.0f, Palette::Ink(), Weight::Light);
            Y += 68.0f;
            // seek bar with event ticks
            if (Y < FooterY - 60.0f)
            {
                const float Bx = Pad, By2 = Y + 14.0f;
                L.Fill(Bx, By2, Iw, 3.0f, Palette::Line());
                L.Fill(Bx, By2, Iw * static_cast<float>(M.Rep.Fraction()), 3.0f, Palette::Accent());
                L.Fill(Bx + Iw * static_cast<float>(M.Rep.Fraction()) - 1.0f, By2 - 6.0f, 3.0f, 15.0f, Palette::Ink());
                if (const TestData* TD = M.Camp.Find(M.ReplayTest))
                    for (const auto& E : TD->Events)
                        if (E.Occurred && M.Rep.DurationS() > 0.0)
                            L.Fill(Bx + Iw * static_cast<float>((E.FirstS - M.Rep.Start()) / M.Rep.DurationS()), By2 - 12.0f, 1.0f, 8.0f, Palette::Warn());
                L.AddHit(Bx, By2 - 14.0f, Iw, 34.0f, Act::SeekBar, "bar");
                Y += 52.0f;
            }
            if (Y < FooterY - 70.0f)
            {
                Button(L, Pad, Y, 190.0f, 48.0f, M.Rep.Playing() ? "Pause" : "Play", true, Act::PlayPause);
                Button(L, Pad + 204.0f, Y, 150.0f, 48.0f, "Restart", false, Act::Restart);
                Y += 70.0f;
            }
            if (Y < FooterY - 44.0f)
            {
                L.Text(Pad, Y + 8.0f, 120.0f, "Speed", 15.0f, Palette::Ink70());
                Button(L, Pad + 100.0f, Y, 56.0f, 40.0f, "-", false, Act::Slower);
                L.Text(Pad + 164.0f, Y + 6.0f, 80.0f, Num1(M.Rep.Speed()) + "x", 22.0f, Palette::Ink(), Weight::Medium, Align::Centre);
                Button(L, Pad + 252.0f, Y, 56.0f, 40.0f, "+", false, Act::Faster);
                Y += 62.0f;
            }
            if (const TestData* TD = M.Camp.Find(M.ReplayTest))
            {
                if (Y < FooterY - 30.0f)
                {
                    L.Text(Pad, Y, Iw, "Events in this run", 15.0f, Palette::Ink70());
                    Y += 24.0f;
                    int Shown = 0;
                    for (const auto& E : TD->Events)
                    {
                        if (!E.Occurred || Y >= FooterY - 4.0f) continue;
                        L.Text(Pad, Y, Iw, E.Name, 14.0f, Palette::Ink70());
                        L.Text(Pad, Y, Iw, Num1(E.FirstS) + " s", 14.0f, Palette::Ink46(), Weight::Regular, Align::Right);
                        Y += 22.0f;
                        ++Shown;
                    }
                    if (Shown == 0 && Y < FooterY - 4.0f) L.Text(Pad, Y, Iw, "none", 14.0f, Palette::Ink46());
                }
            }
        }
    }
}
}  // namespace Screens
}  // namespace DroneApp

