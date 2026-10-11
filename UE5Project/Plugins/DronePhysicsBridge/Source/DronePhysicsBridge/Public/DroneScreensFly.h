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
    L.Fill(Cx - R - 18.0f, Cy - R - 26.0f, 2.0f * R + 36.0f, 2.0f * R + 96.0f, Palette::Solid());
    std::vector<Pt> Ring;
    for (int I = 0; I <= 32; ++I) Ring.push_back(Pt{Cx + R * std::sin(kTwoPiF * static_cast<float>(I) / 32.0f), Cy - R * std::cos(kTwoPiF * static_cast<float>(I) / 32.0f)});
    L.Poly(Ring, Palette::Ink46(), 1.5f);
    L.Text(Cx - 8.0f, Cy - R - 24.0f, 16.0f, "N", 14.0f, Palette::Ink70());
    const double Speed = std::sqrt(WindNed[0] * WindNed[0] + WindNed[1] * WindNed[1]);
    if (Speed > 0.05)
    {
        const float A = static_cast<float>(std::atan2(WindNed[1], WindNed[0]));   // from north, clockwise (north = x, east = y)
        const float Tx = Cx + (R - 6.0f) * std::sin(A), Ty = Cy - (R - 6.0f) * std::cos(A);
        L.Line(Cx - 0.3f * (Tx - Cx), Cy - 0.3f * (Ty - Cy), Tx, Ty, Palette::Accent(), 3.0f);
        L.Line(Tx, Ty, Tx - 12.0f * std::sin(A - 0.45f), Ty + 12.0f * std::cos(A - 0.45f), Palette::Accent(), 3.0f);
        L.Line(Tx, Ty, Tx - 12.0f * std::sin(A + 0.45f), Ty + 12.0f * std::cos(A + 0.45f), Palette::Accent(), 3.0f);
    }
    L.Text(Cx - R, Cy + R + 6.0f, 2.0f * R, Fmt("%.1f", Speed) + " m/s", 20.0f, Palette::Ink(), Weight::Regular, Align::Centre);
    const float Bw = 2.0f * R, Frac = static_cast<float>(std::min(1.0, std::max(0.0, GustMps / 5.0)));
    L.Fill(Cx - R, Cy + R + 36.0f, Bw, 6.0f, Palette::Line());
    L.Fill(Cx - R, Cy + R + 36.0f, Bw * Frac, 6.0f, GustMps > 3.0 ? Palette::Warn() : Palette::Accent());   // amber = strong gust (caution)
    L.Text(Cx - R, Cy + R + 46.0f, Bw, "gust " + Fmt("%.1f", GustMps) + " m/s", 14.0f, Palette::Ink46(), Weight::Regular, Align::Centre);
}

inline void FlyScreen(DrawList& L, const AppModel& M)
{
    const float Top = static_cast<float>(Ui::kTopBarHeightPx);
    L.Fill(0, 0, W, Top, Palette::Bar());
    L.HLine(0, Top, W, Palette::Line());
    L.Text(48.0f, 16.0f, 300.0f, "Fly it yourself", 20.0f, Palette::Ink(), Weight::Medium);
    L.Text(300.0f, 19.0f, 1300.0f, Fly::LiveLabel(M.VehicleClass), 16.0f, Palette::Ink70());
    Button(L, W - 220.0f, 8.0f, 180.0f, 40.0f, "Exit fly mode", false, Act::ExitFly);

    // flight HUD (bottom left)
    const auto& T = M.FlyT;
    L.Fill(32.0f, Hgt - 150.0f, 760.0f, 118.0f, Palette::Solid());
    const std::pair<const char*, std::string> Cells[] = {{"Altitude", Fmt("%.1f", T.AltitudeM) + " m"}, {"Speed", Fmt("%.1f", T.SpeedMps) + " m/s"},
                                                         {"Rpm (true)", Fmt("%.0f", T.RpmMean)}, {"Battery", Fmt("%.0f", T.SocFraction * 100.0) + " %"}};
    float X = 52.0f;
    for (const auto& C : Cells)
    {
        L.Text(X, Hgt - 138.0f, 170.0f, C.first, 14.0f, Palette::Ink46());
        L.Text(X, Hgt - 114.0f, 170.0f, C.second, 28.0f, Palette::Ink(), Weight::Light);
        X += 180.0f;
    }
    L.Text(420.0f, Hgt - 66.0f, 360.0f, "rpm FR/FL/RR/RL  " + Fmt("%.0f", T.Rpm[0]) + " / " + Fmt("%.0f", T.Rpm[1]) + " / " + Fmt("%.0f", T.Rpm[2]) + " / " + Fmt("%.0f", T.Rpm[3]), 14.0f, Palette::Ink70(), Weight::Regular, Align::Right);
    L.Text(52.0f, Hgt - 66.0f, 360.0f, std::string("Mode  ") + Fly::ModeName(M.FlyModeSel) + "     " + T.StateText, 16.0f, Palette::Ink70());
    if (T.Crashed) L.Text(52.0f, Hgt - 44.0f, 700.0f, "Crashed on ground contact: press X (or Y on the gamepad) to reset", 15.0f, Palette::Crit(), Weight::Medium);
    else L.Text(52.0f, Hgt - 44.0f, 700.0f, "W/S pitch  A/D roll  Q/E yaw  Space/R up  Shift/F down  T takeoff  G land  X reset", 14.0f, Palette::Ink46());

    // wind panel (right)
    const float Px = 1456.0f, Pw = 432.0f;
    L.Fill(Px, Top + 8.0f, Pw, 470.0f, Palette::Solid());
    L.Text(Px + 20.0f, Top + 20.0f, 300.0f, "Live wind", 18.0f, Palette::Ink(), Weight::Medium);
    struct Row { const char* Label; const char* Unit; const char* Id; const std::string* Text; };
    const Row Rows[] = {{"Wind speed", "m/s", "speed", &M.WindP.SpeedText}, {"Wind from", "deg", "dir", &M.WindP.DirText},
                        {"Gust amplitude", "m/s", "gust", &M.WindP.GustText}, {"Gust interval", "s", "interval", &M.WindP.IntervalText},
                        {"Air temperature", "C", "temp", &M.WindP.TempText}, {"Payload mass", "kg", "payload", &M.WindP.PayloadText}};
    float Y = Top + 60.0f;
    for (const auto& R : Rows)
    {
        const std::string Id = std::string("fly:") + R.Id;
        L.Text(Px + 20.0f, Y + 6.0f, 190.0f, R.Label, 15.0f, Palette::Ink70());
        Field(L, Px + 220.0f, Y, 110.0f, *R.Text, 22.0f, M.Focus == Id, Id, Weight::Regular);
        L.Text(Px + 340.0f, Y + 8.0f, 60.0f, R.Unit, 14.0f, Palette::Ink46());
        Y += 44.0f;
    }
    if (!M.WindP.Error.empty()) L.Text(Px + 20.0f, Y, Pw - 40.0f, M.WindP.Error, 14.0f, Palette::Warn());
    Y += 28.0f;
    Button(L, Px + 20.0f, Y, 180.0f, 40.0f, "Gust now", true, Act::GustNow);
    Button(L, Px + 214.0f, Y, 198.0f, 40.0f, M.WindP.Randomise ? "Hold steady" : "Randomise", false, Act::ToggleRandomise);
    Y += 56.0f;
    const bool Acro = Fly::ModeAvailable(M.VehicleClass, Fly::Mode::Acro);
    Button(L, Px + 20.0f, Y, 180.0f, 40.0f, "Assisted", M.FlyModeSel == Fly::Mode::Assisted, Act::SetFlyMode, "assisted");
    Button(L, Px + 214.0f, Y, 198.0f, 40.0f, Acro ? "Acro (rate)" : "Acro: FPV only", false, Act::SetFlyMode, "acro", Acro);
    if (M.WindP.GustNowTimerS > 0.0) L.Text(Px + 20.0f, Y + 50.0f, Pw - 40.0f, "Step gust on", 14.0f, Palette::Ink46());
    WindRose(L, Px + Pw * 0.5f, Top + 660.0f, 70.0f, T.WindNed, T.GustMps);
}
}  // namespace Screens
}  // namespace DroneApp
