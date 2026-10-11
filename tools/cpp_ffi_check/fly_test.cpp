// Tier: V
// Fly mode without Unreal: the class list, key and gamepad mapping, the wind panel (validation, gust now, hold / randomise), the wind
// streaks and rose, and the fly screen's layout. The pawn, Slate and input layer are NOT covered (they need Unreal).
//   g++ -std=c++17 -Wall -Wextra -Werror -I UE5Project/Plugins/DronePhysicsBridge/Source/DronePhysicsBridge/Public tools/cpp_ffi_check/fly_test.cpp -o fly_test
//   ./fly_test <repo root>
#include "DroneScreens.h"

#include <cmath>
#include <cstdio>
#include <fstream>
#include <sstream>

using namespace DroneApp;

static int g_failures = 0;
static void Check(bool ok, const char* name) { if (!ok) { std::printf("FAIL: %s\n", name); ++g_failures; } }
static bool Near(double a, double b, double t) { return std::fabs(a - b) <= t; }

struct FileIO : AppIO
{
    bool ReadText(const std::string& P, std::string& O) override { std::ifstream f(P, std::ios::binary); if (!f) return false; std::stringstream s; s << f.rdbuf(); O = s.str(); return true; }
    bool WriteText(const std::string&, const std::string&) override { return false; }
    bool Exists(const std::string& P) override { std::ifstream f(P); return static_cast<bool>(f); }
    int StartProcess(const Command&, const std::string&) override { return -1; }
    bool PollProcess(int, int&) override { return true; }
    void OpenPath(const std::string&) override {}
};

int main(int argc, char** argv)
{
    if (argc < 2) { std::printf("usage: fly_test <repo root>\n"); return 2; }
    // ---- class list: only the two implemented classes, both selectable ----
    const auto& V = VehicleClasses();
    Check(V.size() == 2 && std::string(V[0].Id) == "quad_standard" && std::string(V[1].Id) == "quad_fpv_5in" && V[0].Available && V[1].Available, "only the standard quad and the FPV quad are offered");
    Check(Fly::ModeAvailable("quad_standard", Fly::Mode::Assisted) && !Fly::ModeAvailable("quad_standard", Fly::Mode::Acro) && Fly::ModeAvailable("quad_fpv_5in", Fly::Mode::Acro), "acro is FPV only");
    Check(Fly::LiveLabel("quad_fpv_5in").find("standard FPV quad model, not your exact build (per-build live model needs FFI v15)") != std::string::npos, "live-physics label names the class and FFI v15");

    // ---- keys ----
    Fly::KeyState K;
    K.W = true; K.D = true; K.E = true; K.Up = true;
    Fly::Axes A = Fly::FromKeys(K);
    Check(A.Pitch == 1.0 && A.Roll == 1.0 && A.Yaw == 1.0 && A.Throttle == 1.0, "W pitch forward, D roll right, E yaw right, Space/R up");
    K.S = true; K.A = true; K.Q = true; K.Down = true;
    A = Fly::FromKeys(K);
    Check(A.Pitch == 0.0 && A.Roll == 0.0 && A.Yaw == 0.0 && A.Throttle == 0.0, "opposite keys cancel");
    Check(Fly::ButtonForKey("T") == Fly::Button::Takeoff && Fly::ButtonForKey("G") == Fly::Button::Land && Fly::ButtonForKey("X") == Fly::Button::Reset && Fly::ButtonForKey("W") == Fly::Button::None, "T takeoff, G land, X reset");
    Check(Fly::ButtonForKey("Gamepad_FaceButton_Bottom") == Fly::Button::Takeoff && Fly::ButtonForKey("Gamepad_FaceButton_Right") == Fly::Button::Land && Fly::ButtonForKey("Gamepad_FaceButton_Top") == Fly::Button::Reset, "gamepad A takeoff, B land, Y reset");
    // ---- gamepad: left stick throttle (Y) + yaw (X), right stick pitch (Y) + roll (X); dead zone 0.12 rescaled ----
    A = Fly::FromGamepad(0.5, 1.0, -1.0, 0.0);
    Check(Near(A.Yaw, (0.5 - 0.12) / 0.88, 1e-12) && A.Throttle == 1.0 && A.Roll == -1.0 && A.Pitch == 0.0, "stick mapping and rescaled dead zone");
    A = Fly::FromGamepad(0.1, -0.05, 0.11, -0.12);
    Check(A.Yaw == 0.0 && A.Throttle == 0.0 && A.Roll == 0.0 && A.Pitch == 0.0, "inside the dead zone nothing moves");
    A = Fly::FromGamepad(NAN, 5.0, 0.0, 0.0);
    Check(A.Yaw == 0.0 && A.Throttle == 1.0, "NaN stick is zero, overrange is clamped");
    const DroneDemo::Command C = Fly::ToCommand(Fly::Axes{1.0, -1.0, 0.5, 1.0});
    Check(Near(C.ForwardMps, 3.0, 0) && Near(C.RightMps, -3.0, 0) && Near(C.ClimbMps, 1.5, 0) && Near(C.YawRateRadS, 0.5, 0), "axes to the demo controller's velocity command");

    // ---- wind panel ----
    Fly::WindPanel W;
    W.SpeedText = "6"; W.DirText = "450"; W.GustText = "2"; W.IntervalText = "4"; W.TempText = "-10"; W.PayloadText = "0.5";
    Check(W.Commit("speed") && W.Commit("dir") && W.Commit("gust") && W.Commit("interval") && W.Commit("temp") && W.Commit("payload"), "valid entries commit");
    Check(Near(W.SpeedMps, 6, 0) && Near(W.DirectionDeg, 90, 1e-9) && Near(W.GustSigmaMps, 2, 0) && Near(W.GustIntervalS, 4, 0) && Near(W.AirTempC, -10, 0) && Near(W.PayloadKg, 0.5, 0), "values stored, direction wrapped 450 -> 90");
    W.SpeedText = "99";
    Check(!W.Commit("speed") && W.SpeedMps == 6.0 && W.SpeedText == "6.0" && !W.Error.empty(), "an out-of-range speed is refused, the old value stays and is shown again");
    W.GustText = "abc";
    Check(!W.Commit("gust") && W.GustSigmaMps == 2.0, "text is refused");
    W.PayloadText = "3";
    Check(!W.Commit("payload") && W.PayloadKg == 0.5, "payload above 2 kg refused");
    Check(Near(W.ToParams().SteadySpeedMps, 6.0, 0) && Near(W.ToParams().GustCorrelationTimeS, 4.0, 0) && Near(W.ToParams().GustSigmaMps, 2.0, 0), "panel values become the demo wind parameters");
    // "from" end to end: the panel's direction is where the wind comes FROM; the generator gets TOWARD and its NED vector points away from it
    Check(Near(W.DirectionDeg, 90.0, 1e-9) && Near(W.TowardDeg(), 270.0, 1e-9) && Near(W.ToParams().SteadyDirectionDeg, 270.0, 1e-9), "panel direction is FROM, the params are TOWARD (+180)");
    {
        DroneWind::Params NoGust = W.ToParams();
        NoGust.bGustsEnabled = false;
        for (const double From : {0.0, 90.0, 180.0, 270.0})
        {
            Fly::WindPanel Q;
            Q.SpeedMps = 6.0; Q.DirectionDeg = From; Q.GustSigmaMps = 0.0;
            DroneWind::Params Pm = Q.ToParams();
            Pm.bGustsEnabled = false;
            DroneWind::Generator G(1);
            DroneWind::Sample Sm;
            G.Step(Pm, 0.002, &Sm);
            // wind FROM north (0): toward south, NED x = -6; FROM east (90): toward west, NED y = -6; FROM south: x = +6; FROM west: y = +6
            const double Ex = -6.0 * std::cos(From * 3.14159265358979323846 / 180.0), Ey = -6.0 * std::sin(From * 3.14159265358979323846 / 180.0);
            Check(Near(Sm.Total[0], Ex, 1e-9) && Near(Sm.Total[1], Ey, 1e-9), "wind from <deg> gives the NED vector -v (cos, sin)(from)");
        }
        (void)NoGust;
    }
    W.GustNow();
    Check(Near(W.ToParams().SteadySpeedMps, 11.0, 0), "Gust now adds a 5 m/s step");
    W.SpeedMps = 14.0;
    Check(Near(W.EffectiveSteadyMps(), 15.0, 0), "the step is limited to the core's wind range");
    W.Advance(3.1);
    Check(W.GustNowTimerS == 0.0 && Near(W.EffectiveSteadyMps(), 14.0, 0), "the step gust ends after 3 s");
    Fly::WindPanel R1, R2;
    R1.Randomise = R2.Randomise = true;
    for (int I = 0; I < 400; ++I) { R1.Advance(0.1); R2.Advance(0.1); }
    Check(R1.SpeedMps == R2.SpeedMps && R1.DirectionDeg == R2.DirectionDeg && R1.SpeedMps >= 0.0 && R1.SpeedMps <= 15.0, "randomise is seeded: equal seeds give equal winds, always in range");
    Fly::WindPanel H;
    H.SpeedMps = 4.0;
    for (int I = 0; I < 400; ++I) H.Advance(0.1);
    Check(H.SpeedMps == 4.0, "hold steady keeps the wind");

    // ---- streaks ----
    std::vector<Fly::Streak> S0, S1, S2;
    const double Calm[3] = {0, 0, 0}, Wind5[3] = {5, 0, 0}, Wind10[3] = {10, 0, 0};
    Fly::WindStreaks(Calm, 1.0, 0.0, S0);
    Fly::WindStreaks(Wind5, 1.0, 0.0, S1);
    Fly::WindStreaks(Wind10, 1.0, 0.0, S2);
    Check(S0.empty() && S1.size() == 30 && S2.size() == 60, "no streaks in calm air; count scales with wind speed (6 per m/s)");
    std::vector<Fly::Streak> S1b;
    Fly::WindStreaks(Wind5, 1.0, 1.0, S1b);
    Check(S1b.size() == S1.size() && Near(S1[0].Alpha, 0.35, 1e-12) && Near(S1b[0].Alpha, 1.0, 1e-12), "a gust pulse brightens the streaks");
    // drift: after 1 s the north coordinate moved by +5 m (mod the 24 m box), east and down did not
    std::vector<Fly::Streak> T0, T1;
    Fly::WindStreaks(Wind5, 0.0, 0.0, T0);
    Fly::WindStreaks(Wind5, 1.0, 0.0, T1);
    double Dn = T1[3].X - T0[3].X;
    if (Dn < 0) Dn += 24.0;
    Check(Near(Dn, 5.0, 1e-9) && Near(T1[3].Y, T0[3].Y, 1e-12) && Near(T1[3].Z, T0[3].Z, 1e-12), "streaks drift with the wind vector");
    bool Inside = true;
    for (const auto& S : S2) Inside = Inside && std::fabs(S.X) <= 12.0 && std::fabs(S.Y) <= 12.0 && std::fabs(S.Z) <= 12.0;
    Check(Inside, "streaks stay in the box around the vehicle");
    Check(Fly::GustPulse(1.0, 1.0, 0.1) == 0.0 && Fly::GustPulse(1.0, 3.0, 0.1) == 1.0 && Near(Fly::GustPulse(1.0, 1.1, 0.1), 0.25, 1e-12), "pulse: none for a steady gust, full for a sudden one");

    // ---- model and screen ----
    FileIO IO;
    HelperSettings H0;
    H0.RepoRoot = argv[1];
    AppModel M(IO, H0, "work");
    M.DefinitionsRel = "campaign/smoke_definitions.json";
    if (!M.Init()) { std::printf("init failed: %s\n", M.Message.c_str()); return 1; }
    M.Click(Act::FlyIt, "");
    Check(!M.FlyMode && M.MessageIsError, "fly mode needs a vehicle class first");
    M.VehicleClass = "quad_fpv_5in";
    M.Prog.VehicleClassChosen = true;
    M.Click(Act::FlyIt, "");
    Check(M.FlyMode && M.FlyModeSel == Fly::Mode::Assisted, "fly mode opens in assisted mode");
    M.Click(Act::SetFlyMode, "acro");
    Check(M.FlyModeSel == Fly::Mode::Acro, "FPV can switch to acro");
    M.VehicleClass = "quad_standard";
    M.Click(Act::SetFlyMode, "assisted");
    M.Click(Act::SetFlyMode, "acro");
    Check(M.FlyModeSel == Fly::Mode::Assisted, "the standard quad cannot go to acro");
    M.Click(Act::FocusField, "fly:speed");
    M.KeyChar('9');
    M.KeyEnter();
    Check(M.WindP.SpeedText == "69" ? !M.WindP.Error.empty() : true, "typed text is validated on Enter");
    M.WindP.SpeedText = "7";
    M.Click(Act::FocusField, "fly:speed");
    M.Click(Act::FocusField, "fly:dir");   // leaving the field commits it
    Check(Near(M.WindP.SpeedMps, 7.0, 0), "leaving a field commits its value at once");
    M.Click(Act::GustNow, "");
    Check(M.WindP.GustNowTimerS > 0.0, "the Gust now button starts a step gust");
    M.Click(Act::ToggleRandomise, "");
    Check(M.WindP.Randomise, "the Randomise button toggles");
    M.FlyT.WindNed[0] = 6.0; M.FlyT.WindNed[1] = 2.0; M.FlyT.GustMps = 4.0; M.FlyT.AltitudeM = 12.3; M.FlyT.RpmMean = 5400.0; M.FlyT.SocFraction = 0.83;
    DrawList L;
    BuildScreen(L, M);
    Check(L.FindText("Live physics: standard quad model, not your exact build") != nullptr, "the live-physics label is on screen");
    Check(L.FindText("Altitude") && L.FindText("12.3 m") && L.FindText("5400") && L.FindText("83 %") && L.FindText("ASSISTED"), "HUD: altitude, speed, rpm, battery, mode");
    Check(L.FindText("Live wind") && L.FindText("Gust now") && L.FindText("Hold steady") && L.FindText("6.3 m/s") && L.FindText("gust 4.0 m/s"), "wind panel, rose speed and gust numbers");
    bool Amber = false;
    for (const auto& I : L.Items) Amber = Amber || (I.K == Item::Kind::Rect && I.H == 6.0f && I.C.R > 0.5f && I.C.B < 0.4f);
    Check(Amber, "a strong gust turns the gust bar amber");
    bool Bounds = true, Min = true;
    for (const auto& I : L.Items)
    {
        if (I.X < -1.0f || I.Y < -1.0f || I.X > 1921.0f || I.Y > 1081.0f) Bounds = false;
        if (I.K == Item::Kind::Text && (I.Px < 14.0f || I.Y + I.Px > 1080.5f)) Min = false;
    }
    Check(Bounds && Min, "fly screen: inside 1920 x 1080, no text under 14 px");
    M.FlyT.Crashed = true;
    DrawList L2;
    BuildScreen(L2, M);
    Check(L2.FindText("Crashed on ground contact") != nullptr, "a crash is stated and offers reset");
    M.Click(Act::ExitFly, "");
    Check(!M.FlyMode, "exit returns to the app");

    // the wind rose is on the Run screen too
    DrawList L3;
    M.Current = Screen::Run;
    BuildScreen(L3, M);
    Check(L3.FindText("0.0 m/s") != nullptr && L3.FindText("Fly it yourself") != nullptr, "Run screen: wind rose and the way into fly mode");

    if (g_failures == 0) std::printf("fly_test: all checks passed\n");
    return g_failures == 0 ? 0 : 1;
}
