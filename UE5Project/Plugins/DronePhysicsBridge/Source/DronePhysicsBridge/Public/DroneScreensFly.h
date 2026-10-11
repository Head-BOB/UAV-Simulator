// Tier: V
// Engine-free layout of fly mode: the "live physics" label, the flight HUD (altitude, speed, true rpm, battery, mode), the live wind
// panel and the wind rose (also used on the Run screen). Flat graphite, white values, one blue accent, amber only for caution.
#pragma once

#include <cmath>
#include <string>

#include "DroneScreensSetup.h"

namespace DroneApp
{
namespace Screens
{
// Compass with the direction the wind blows TOWARD (north up), speed number and a gust bar. Wind in NED m/s; Gust in m/s.
inline void WindRose(DrawList& L, float Cx, float Cy, float R, const double WindNed[3], double GustMps)
{
    constexpr float kTwoPiF = static_cast<float>(2.0 * kPi);   // DroneApp::kPi (DroneReplay.h), not a local copy
    const float Bx = Cx - R - 18.0f, By = Cy - R - 26.0f, Bw = 2.0f * R + 36.0f, Bh = 2.0f * R + 96.0f;
    // Translucent tactical bezel with border lines
    L.Fill(Bx, By, Bw, Bh, ToRgba(0x0C0E12, 0.76));
    L.Line(Bx, By, Bx + Bw, By, Palette::LineStrong());
    L.Line(Bx, By + Bh, Bx + Bw, By + Bh, Palette::Line());
    L.Line(Bx, By, Bx, By + Bh, Palette::Line());
    L.Line(Bx + Bw, By, Bx + Bw, By + Bh, Palette::Line());

    // Compass ring and crosshair ticks
    std::vector<Pt> Ring;
    for (int I = 0; I <= 32; ++I) Ring.push_back(Pt{Cx + R * std::sin(kTwoPiF * static_cast<float>(I) / 32.0f), Cy - R * std::cos(kTwoPiF * static_cast<float>(I) / 32.0f)});
    L.Poly(Ring, Palette::LineStrong(), 1.5f);
    L.Line(Cx, Cy - R + 2.0f, Cx, Cy - R + 8.0f, Palette::Accent(), 2.0f); // North tick
    L.Line(Cx, Cy + R - 8.0f, Cx, Cy + R - 2.0f, Palette::LineStrong(), 1.5f); // South tick
    L.Line(Cx - R + 2.0f, Cy, Cx - R + 8.0f, Cy, Palette::LineStrong(), 1.5f); // West tick
    L.Line(Cx + R - 8.0f, Cy, Cx + R - 2.0f, Cy, Palette::LineStrong(), 1.5f); // East tick
    L.Text(Cx - 8.0f, Cy - R - 24.0f, 16.0f, "N", 14.0f, Palette::Accent(), Weight::Medium, Align::Centre);

    // Wind direction arrow (direction toward which wind blows)
    const double Speed = std::sqrt(WindNed[0] * WindNed[0] + WindNed[1] * WindNed[1]);
    if (Speed > 0.05)
    {
        const float A = static_cast<float>(std::atan2(WindNed[1], WindNed[0]));   // from north, clockwise (north = x, east = y)
        const float Tx = Cx + (R - 6.0f) * std::sin(A), Ty = Cy - (R - 6.0f) * std::cos(A);
        L.Line(Cx - 0.3f * (Tx - Cx), Cy - 0.3f * (Ty - Cy), Tx, Ty, Palette::Accent(), 3.0f);
        L.Line(Tx, Ty, Tx - 12.0f * std::sin(A - 0.45f), Ty + 12.0f * std::cos(A - 0.45f), Palette::Accent(), 3.0f);
        L.Line(Tx, Ty, Tx - 12.0f * std::sin(A + 0.45f), Ty + 12.0f * std::cos(A + 0.45f), Palette::Accent(), 3.0f);
    }
    L.Text(Cx - R, Cy + R + 6.0f, 2.0f * R, Fmt("%.1f", Speed) + " m/s", 20.0f, Palette::Ink(), Weight::Medium, Align::Centre);
    const float Gbw = 2.0f * R, Frac = static_cast<float>(std::min(1.0, std::max(0.0, GustMps / 5.0)));
    L.Fill(Cx - R, Cy + R + 36.0f, Gbw, 6.0f, Palette::Line());
    L.Fill(Cx - R, Cy + R + 36.0f, Gbw * Frac, 6.0f, GustMps > 3.0 ? Palette::Warn() : Palette::Accent());   // amber = strong gust (caution)
    L.Text(Cx - R, Cy + R + 46.0f, Gbw, "gust " + Fmt("%.1f", GustMps) + " m/s", 14.0f, Palette::Ink46(), Weight::Regular, Align::Centre);
}

inline void FlyScreen(DrawList& L, const AppModel& M)
{
    const float Top = static_cast<float>(Ui::kTopBarHeightPx);
    L.Fill(0, 0, W, Top, Palette::Bar());
    L.HLine(0, Top, W, Palette::Line());
    L.Text(48.0f, 16.0f, 220.0f, "Fly it yourself", 20.0f, Palette::Ink(), Weight::Medium);
    L.Fill(228.0f, 17.0f, 1.0f, 22.0f, Palette::LineStrong());
    L.Text(248.0f, 18.0f, 1300.0f, Fly::LiveLabel(M.VehicleClass), 15.0f, Palette::Ink70());
    Button(L, W - 200.0f, 8.0f, 160.0f, 40.0f, "Exit fly mode", false, Act::ExitFly);

    // flight HUD (bottom left): translucent aerospace glass telemetry deck
    const auto& T = M.FlyT;
    const float HudX = 40.0f, HudW = 820.0f, HudH = 158.0f;
    const float HudY = Hgt - HudH - 24.0f;
    L.Fill(HudX, HudY, HudW, HudH, ToRgba(0x0C0E12, 0.78));
    L.Line(HudX, HudY, HudX + HudW, HudY, Palette::LineStrong());
    L.Line(HudX, HudY + HudH, HudX + HudW, HudY + HudH, Palette::Line());
    L.Line(HudX, HudY, HudX, HudY + HudH, Palette::Line());
    L.Line(HudX + HudW, HudY, HudX + HudW, HudY + HudH, Palette::Line());
    L.Fill(HudX, HudY, 3.0f, HudH, Palette::Accent()); // Cyan left accent pill

    // Header strip of HUD: flight mode & state badge
    Chip(L, HudX + 16.0f, HudY + 12.0f, Fly::ModeName(M.FlyModeSel), Palette::Accent());
    Chip(L, HudX + 180.0f, HudY + 12.0f, T.Crashed ? "CRASHED" : (T.StateText.empty() ? "ACTIVE" : T.StateText), T.Crashed ? Palette::Crit() : Palette::Ok());
    L.Text(HudX + 380.0f, HudY + 14.0f, HudW - 400.0f, "rpm FR/FL/RR/RL  " + Fmt("%.0f", T.Rpm[0]) + " / " + Fmt("%.0f", T.Rpm[1]) + " / " + Fmt("%.0f", T.Rpm[2]) + " / " + Fmt("%.0f", T.Rpm[3]), 14.0f, Palette::Ink70(), Weight::Regular, Align::Right);
    L.HLine(HudX + 16.0f, HudY + 42.0f, HudW - 32.0f, Palette::Line());

    // 4 Telemetry columns
    const std::pair<const char*, std::string> Cells[] = {
        {"Altitude", Fmt("%.1f", T.AltitudeM) + " m"},
        {"Speed", Fmt("%.1f", T.SpeedMps) + " m/s"},
        {"Rpm (true)", Fmt("%.0f", T.RpmMean)},
        {"Battery", Fmt("%.0f", T.SocFraction * 100.0) + " %"}
    };
    const float ColW = (HudW - 32.0f) / 4.0f;
    for (std::size_t I = 0; I < 4; ++I)
    {
        const float Cx = HudX + 16.0f + static_cast<float>(I) * ColW;
        if (I > 0) L.VLine(Cx, HudY + 48.0f, 54.0f, Palette::Line());
        L.Text(Cx + 10.0f, HudY + 48.0f, ColW - 20.0f, Cells[I].first, 14.0f, Palette::Ink46(), Weight::Medium);
        L.Text(Cx + 10.0f, HudY + 70.0f, ColW - 20.0f, Cells[I].second, 26.0f, Palette::Ink(), Weight::Light);
    }
    L.HLine(HudX + 16.0f, HudY + 114.0f, HudW - 32.0f, Palette::Line());

    // Controls footer
    if (T.Crashed)
    {
        L.Text(HudX + 16.0f, HudY + 126.0f, HudW - 32.0f, "Crashed on ground contact: press X (or Y on the gamepad) to reset", 14.0f, Palette::Crit(), Weight::Medium);
    }
    else
    {
        L.Text(HudX + 16.0f, HudY + 126.0f, HudW - 32.0f, "W/S pitch   A/D roll   Q/E yaw   Space/R up   Shift/F down   T takeoff   G land   X reset", 14.0f, Palette::Ink46());
    }

    // wind panel (right): translucent glass environment deck
    const float Px = 1440.0f, Pw = 440.0f;
    L.Fill(Px, Top + 14.0f, Pw, 510.0f, ToRgba(0x0C0E12, 0.78));
    L.Line(Px, Top + 14.0f, Px + Pw, Top + 14.0f, Palette::LineStrong());
    L.Line(Px, Top + 14.0f + 510.0f, Px + Pw, Top + 14.0f + 510.0f, Palette::Line());
    L.Line(Px, Top + 14.0f, Px, Top + 14.0f + 510.0f, Palette::Line());
    L.Line(Px + Pw, Top + 14.0f, Px + Pw, Top + 14.0f + 510.0f, Palette::Line());
    L.Fill(Px + Pw - 3.0f, Top + 14.0f, 3.0f, 510.0f, Palette::Accent());

    L.Text(Px + 20.0f, Top + 26.0f, 260.0f, "Live wind", 18.0f, Palette::Ink(), Weight::Medium);
    Chip(L, Px + Pw - 110.0f, Top + 24.0f, "ENV DOCK", Palette::Ink70());
    L.HLine(Px + 20.0f, Top + 56.0f, Pw - 40.0f, Palette::Line());

    struct Row { const char* Label; const char* Unit; const char* Id; const std::string* Text; };
    const Row Rows[] = {{"Wind speed", "m/s", "speed", &M.WindP.SpeedText}, {"Wind from", "deg", "dir", &M.WindP.DirText},
                        {"Gust amplitude", "m/s", "gust", &M.WindP.GustText}, {"Gust interval", "s", "interval", &M.WindP.IntervalText},
                        {"Air temperature", "C", "temp", &M.WindP.TempText}, {"Payload mass", "kg", "payload", &M.WindP.PayloadText}};
    float Y = Top + 68.0f;
    for (const auto& R : Rows)
    {
        const std::string Id = std::string("fly:") + R.Id;
        L.Text(Px + 20.0f, Y + 6.0f, 190.0f, R.Label, 15.0f, Palette::Ink70());
        Field(L, Px + 220.0f, Y, 110.0f, *R.Text, 22.0f, M.Focus == Id, Id, Weight::Regular);
        L.Text(Px + 340.0f, Y + 8.0f, 60.0f, R.Unit, 14.0f, Palette::Ink46());
        L.HLine(Px + 20.0f, Y + 38.0f, Pw - 40.0f, Palette::Line());
        Y += 46.0f;
    }
    if (!M.WindP.Error.empty()) L.Text(Px + 20.0f, Y + 4.0f, Pw - 40.0f, M.WindP.Error, 14.0f, Palette::Warn());
    Y += 16.0f;

    // Wind injection buttons
    Button(L, Px + 20.0f, Y, 190.0f, 40.0f, "Gust now", true, Act::GustNow);
    Button(L, Px + 222.0f, Y, Pw - 242.0f, 40.0f, M.WindP.Randomise ? "Hold steady" : "Randomise", false, Act::ToggleRandomise);
    Y += 52.0f;

    // Flight mode buttons
    const bool Acro = Fly::ModeAvailable(M.VehicleClass, Fly::Mode::Acro);
    Button(L, Px + 20.0f, Y, 190.0f, 40.0f, "Assisted", M.FlyModeSel == Fly::Mode::Assisted, Act::SetFlyMode, "assisted");
    Button(L, Px + 222.0f, Y, Pw - 242.0f, 40.0f, Acro ? "Acro (rate)" : "Acro: FPV only", false, Act::SetFlyMode, "acro", Acro);
    if (M.WindP.GustNowTimerS > 0.0) L.Text(Px + 20.0f, Y + 48.0f, Pw - 40.0f, "Step gust on", 14.0f, Palette::Accent());

    // Tactical wind rose compass widget
    WindRose(L, Px + Pw * 0.5f, Top + 700.0f, 68.0f, T.WindNed, T.GustMps);
}
}  // namespace Screens
}  // namespace DroneApp
