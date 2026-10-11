// Tier: V
// Engine-free parts of "Fly it yourself": which vehicle classes fly and in which modes, the mapping of keys / gamepad to pilot axes and
// buttons, the live wind panel (typed values validated, gust-now step, steady or randomised), the wind streaks of the 3D scene and the
// numbers of the wind rose. The Unreal pawn only feeds these and draws them. NOT part of the physics: wind goes to the core through the
// same WindInput / EnvironmentInput the demo pawn already uses. Tested by tools/cpp_ffi_check/fly_test.cpp.
#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <string>
#include <vector>

#include "DroneDemoControl.h"
#include "DroneEnvironment.h"
#include "DroneWind.h"

namespace DroneApp
{
namespace Fly
{
enum class Mode { Assisted, Acro };

inline bool ModeAvailable(const std::string& VehicleClass, Mode M) { return M == Mode::Assisted || VehicleClass == "quad_fpv_5in"; }
inline const char* ModeName(Mode M) { return M == Mode::Assisted ? "ASSISTED (angle)" : "ACRO (rate)"; }
inline std::string ClassName(const std::string& VehicleClass) { return VehicleClass == "quad_fpv_5in" ? "FPV quad" : "quad"; }
// Always on screen in fly mode: the live model is the standard one of the class, not the user's build.
inline std::string LiveLabel(const std::string& VehicleClass)
{
    return "Live physics: standard " + ClassName(VehicleClass) + " model, not your exact build (per-build live model needs FFI v15)";
}

// Pilot axes, each -1..1: Pitch + = nose down / forward, Roll + = right, Yaw + = clockwise, Throttle + = climb.
struct Axes
{
    double Pitch = 0.0, Roll = 0.0, Yaw = 0.0, Throttle = 0.0;
};
inline double Clamp1(double V) { return std::isfinite(V) ? (V < -1.0 ? -1.0 : (V > 1.0 ? 1.0 : V)) : 0.0; }

struct KeyState
{
    bool W = false, S = false, A = false, D = false, Q = false, E = false;
    bool Up = false;     // Space or R
    bool Down = false;   // Shift or F
};
inline Axes FromKeys(const KeyState& K)
{
    Axes A;
    A.Pitch = (K.W ? 1.0 : 0.0) - (K.S ? 1.0 : 0.0);          // W forward, S back
    A.Roll = (K.D ? 1.0 : 0.0) - (K.A ? 1.0 : 0.0);           // D right, A left
    A.Yaw = (K.E ? 1.0 : 0.0) - (K.Q ? 1.0 : 0.0);            // E clockwise, Q counter-clockwise
    A.Throttle = (K.Up ? 1.0 : 0.0) - (K.Down ? 1.0 : 0.0);
    return A;
}

constexpr double kStickDeadzone = 0.12;   // TODO(HUMAN-VERIFY): demo value
inline double Deadzone(double V)
{
    const double M = std::fabs(Clamp1(V));
    return M < kStickDeadzone ? 0.0 : (Clamp1(V) < 0 ? -1.0 : 1.0) * (M - kStickDeadzone) / (1.0 - kStickDeadzone);
}
// Standard Xbox mapping, stick values -1..1 with + = right / up: left stick throttle (Y) and yaw (X), right stick pitch (Y) and roll (X).
inline Axes FromGamepad(double LeftX, double LeftY, double RightX, double RightY)
{
    Axes A;
    A.Yaw = Deadzone(LeftX);
    A.Throttle = Deadzone(LeftY);
    A.Roll = Deadzone(RightX);
    A.Pitch = Deadzone(RightY);
    return A;
}
inline Axes Sum(const Axes& A, const Axes& B) { return Axes{Clamp1(A.Pitch + B.Pitch), Clamp1(A.Roll + B.Roll), Clamp1(A.Yaw + B.Yaw), Clamp1(A.Throttle + B.Throttle)}; }

enum class Button { None, Takeoff, Land, Reset };
// Names are Unreal FKey names: keyboard T / G / X, gamepad A (face bottom) / B (face right) / Y (face top).
inline Button ButtonForKey(const std::string& Key)
{
    if (Key == "T") return Button::Takeoff;
    if (Key == "G") return Button::Land;
    if (Key == "X") return Button::Reset;
    if (Key == "Gamepad_FaceButton_Bottom") return Button::Takeoff;
    if (Key == "Gamepad_FaceButton_Right") return Button::Land;
    if (Key == "Gamepad_FaceButton_Top") return Button::Reset;
    return Button::None;
}

// Assisted (angle) mode: the demo controller takes velocity commands. Limits are the demo's own.
constexpr double kMaxHorizontalMps = 3.0, kMaxClimbMps = 1.5, kMaxYawRateRadS = 1.0;
inline DroneDemo::Command ToCommand(const Axes& A)
{
    DroneDemo::Command C;
    C.ForwardMps = Clamp1(A.Pitch) * kMaxHorizontalMps;
    C.RightMps = Clamp1(A.Roll) * kMaxHorizontalMps;
    C.ClimbMps = Clamp1(A.Throttle) * kMaxClimbMps;
    C.YawRateRadS = Clamp1(A.Yaw) * kMaxYawRateRadS;
    return C;
}

// ---- live wind panel -------------------------------------------------------------------------------------------------------------
// Typed text, validated when committed (Enter or leaving the field); an invalid entry changes nothing and says why.
struct WindPanel
{
    std::string SpeedText = "0", DirText = "0", GustText = "1", IntervalText = "2", TempText = "15", PayloadText = "0";
    double SpeedMps = 0.0, DirectionDeg = 0.0, GustSigmaMps = 1.0, GustIntervalS = 2.0, AirTempC = 15.0, PayloadKg = 0.0;
    bool Randomise = false;       // false = hold steady
    std::uint32_t Seed = 1;
    double GustNowTimerS = 0.0;   // > 0 while a step gust is on
    double RandomTimerS = 0.0;
    std::string Error;

    static constexpr double kMaxSpeedMps = 15.0;     // DroneWind::Params::MaxTotalMps
    static constexpr double kMaxGustSigmaMps = 5.0;  // TODO(HUMAN-VERIFY: demo value)
    static constexpr double kMinIntervalS = 0.5, kMaxIntervalS = 30.0;   // TODO(HUMAN-VERIFY: demo values)
    static constexpr double kMaxPayloadKg = 2.0;     // TODO(HUMAN-VERIFY): demo vehicle is about 1.5 kg
    static constexpr double kStepGustMps = 5.0;      // the "Gust now" step, added to the steady wind
    static constexpr double kStepGustS = 3.0;

    static bool Parse(const std::string& T, double& V)
    {
        if (T.empty()) return false;
        char* End = nullptr;
        V = std::strtod(T.c_str(), &End);
        return End != nullptr && *End == '\0' && std::isfinite(V);
    }
    // Validates one field by id ("speed", "dir", "gust", "interval", "temp", "payload"). False and Error set when refused.
    bool Commit(const std::string& Id)
    {
        double V = 0.0;
        const std::string* Text = Id == "speed" ? &SpeedText : Id == "dir" ? &DirText : Id == "gust" ? &GustText : Id == "interval" ? &IntervalText : Id == "temp" ? &TempText : Id == "payload" ? &PayloadText : nullptr;
        if (Text == nullptr) return false;
        if (!Parse(*Text, V)) { Error = "enter a number"; Reset(Id); return false; }
        Error.clear();
        if (Id == "speed") { if (V < 0.0 || V > kMaxSpeedMps) return Refuse(Id, "wind speed 0 .. 15 m/s"); SpeedMps = V; }
        else if (Id == "dir") { if (!DroneEnv::WrapDirectionDeg(V, &DirectionDeg)) return Refuse(Id, "direction is not a number"); }
        else if (Id == "gust") { if (V < 0.0 || V > kMaxGustSigmaMps) return Refuse(Id, "gust amplitude 0 .. 5 m/s"); GustSigmaMps = V; }
        else if (Id == "interval") { if (V < kMinIntervalS || V > kMaxIntervalS) return Refuse(Id, "gust interval 0.5 .. 30 s"); GustIntervalS = V; }
        else if (Id == "temp") { if (V < -40.0 || V > 50.0) return Refuse(Id, "air temperature -40 .. 50 C"); AirTempC = V; }
        else if (Id == "payload") { if (V < 0.0 || V > kMaxPayloadKg) return Refuse(Id, "payload 0 .. 2 kg"); PayloadKg = V; }
        return true;
    }
    void GustNow() { GustNowTimerS = kStepGustS; }
    // Meteorological convention in the UI: DirectionDeg is where the wind comes FROM (0 = from the north, blows toward the south).
    // The demo wind generator and EnvironmentInput take where it blows TOWARD (NED: north = cos, east = sin).
    double TowardDeg() const { const double D = DirectionDeg + 180.0; return D >= 360.0 ? D - 360.0 : D; }
    // Steady wind including a running step gust, never above the core's range.
    double EffectiveSteadyMps() const { return std::min(kMaxSpeedMps, SpeedMps + (GustNowTimerS > 0.0 ? kStepGustMps : 0.0)); }
    // Parameters of the demo wind generator (the same one the physics step reads).
    DroneWind::Params ToParams() const
    {
        DroneWind::Params P;
        P.SteadySpeedMps = EffectiveSteadyMps();
        P.SteadyDirectionDeg = TowardDeg();
        P.GustSigmaMps = GustSigmaMps;
        P.bGustsEnabled = GustSigmaMps > 0.0;
        P.GustCorrelationTimeS = GustIntervalS;
        return P;
    }
    // Real-time step: ends a step gust; in Randomise mode a seeded generator picks a new speed and direction every 5 to 12 s.
    void Advance(double Dt)
    {
        if (GustNowTimerS > 0.0) GustNowTimerS = std::max(0.0, GustNowTimerS - Dt);
        if (!Randomise) return;
        RandomTimerS -= Dt;
        if (RandomTimerS > 0.0) return;
        RandomTimerS = 5.0 + 7.0 * Next();
        SpeedMps = kMaxSpeedMps * 0.7 * Next();
        DirectionDeg = 360.0 * Next();
        SpeedText = Fmt1(SpeedMps);
        DirText = Fmt1(DirectionDeg);
    }

private:
    double Next()
    {
        Seed = Seed * 1664525u + 1013904223u;   // numerical-recipes LCG: seeded, deterministic
        return static_cast<double>(Seed >> 8) / static_cast<double>(1u << 24);
    }
    static std::string Fmt1(double V) { char B[32]; std::snprintf(B, sizeof B, "%.1f", V); return B; }
    bool Refuse(const std::string& Id, const char* Why) { Error = Why; Reset(Id); return false; }
    void Reset(const std::string& Id)
    {
        if (Id == "speed") SpeedText = Fmt1(SpeedMps);
        else if (Id == "dir") DirText = Fmt1(DirectionDeg);
        else if (Id == "gust") GustText = Fmt1(GustSigmaMps);
        else if (Id == "interval") IntervalText = Fmt1(GustIntervalS);
        else if (Id == "temp") TempText = Fmt1(AirTempC);
        else if (Id == "payload") PayloadText = Fmt1(PayloadKg);
    }
};

// ---- wind visuals ----------------------------------------------------------------------------------------------------------------
struct Streak
{
    double X, Y, Z;   // NED metres from the vehicle
    double LenM;      // streak length along the wind
    double Alpha;     // 0..1
};
constexpr int kMaxStreaks = 96;
constexpr double kBoxHalfM = 12.0;   // streaks live in a 24 m cube around the vehicle and wrap

inline double Hash01(std::uint32_t N)
{
    N ^= N >> 16; N *= 0x7feb352du; N ^= N >> 15; N *= 0x846ca68bu; N ^= N >> 16;
    return static_cast<double>(N >> 8) / static_cast<double>(1u << 24);
}
// Streaks drifting with the wind: count and drift speed scale with the wind speed, alpha is 0.35 plus up to 0.65 for the gust pulse
// (0..1). Deterministic in (wind, time). WindNed is the physics wind (WindSample.Total), not an estimate.
inline void WindStreaks(const double WindNed[3], double TimeS, double Pulse, std::vector<Streak>& Out)
{
    Out.clear();
    const double Speed = std::sqrt(WindNed[0] * WindNed[0] + WindNed[1] * WindNed[1] + WindNed[2] * WindNed[2]);
    if (!(Speed > 0.1)) return;
    const int N = std::min(kMaxStreaks, static_cast<int>(Speed * 6.0 + 0.5));
    const double Len = std::min(4.0, 0.25 * Speed + 0.3), A = 0.35 + 0.65 * std::min(1.0, std::max(0.0, Pulse));
    for (int I = 0; I < N; ++I)
    {
        const std::uint32_t S = static_cast<std::uint32_t>(I) * 3u + 1u;
        double P[3];
        for (int K = 0; K < 3; ++K)
        {
            const double Base = Hash01(S + static_cast<std::uint32_t>(K)) * 2.0 * kBoxHalfM;
            double V = std::fmod(Base + WindNed[K] * TimeS, 2.0 * kBoxHalfM);
            if (V < 0.0) V += 2.0 * kBoxHalfM;
            P[K] = V - kBoxHalfM;
        }
        Out.push_back(Streak{P[0], P[1], P[2], Len, A});
    }
}
// 0..1 pulse from the change of the gust magnitude: a sudden gust gives a short bright pulse, a slow drift none.
inline double GustPulse(double PrevGustMps, double GustMps, double Dt)
{
    if (!(Dt > 0.0)) return 0.0;
    const double Rate = std::fabs(GustMps - PrevGustMps) / Dt;   // m/s per s
    return std::min(1.0, Rate / 4.0);
}
}  // namespace Fly
}  // namespace DroneApp
