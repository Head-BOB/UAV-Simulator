// Tier: V
// Engine-free replay of one recorded time series: playback clock (1x..64x), interpolation at the playback time, the pose and
// instrument values for the viewer, history windows for the charts. The 3D view only shows what was recorded, so what is on
// screen is exactly what the report says. Angles: roll / pitch / yaw in degrees, aerospace ZYX (yaw, then pitch, then roll),
// quaternion order (x, y, z, w), NED world, body X forward / Y right / Z down (ADR-001).
#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <string>
#include <vector>

#include "DroneCampaignData.h"

namespace DroneApp
{
constexpr double kPi = 3.14159265358979323846;

struct Quat
{
    double X = 0.0, Y = 0.0, Z = 0.0, W = 1.0;
};

// Body-to-world quaternion (x, y, z, w) from roll, pitch, yaw in degrees (ZYX). Unit norm by construction (normalised again against rounding).
inline Quat QuatFromRollPitchYawDeg(double RollDeg, double PitchDeg, double YawDeg)
{
    const double Hr = RollDeg * kPi / 360.0, Hp = PitchDeg * kPi / 360.0, Hy = YawDeg * kPi / 360.0;
    const double Cr = std::cos(Hr), Sr = std::sin(Hr), Cp = std::cos(Hp), Sp = std::sin(Hp), Cy = std::cos(Hy), Sy = std::sin(Hy);
    Quat Q;
    Q.W = Cr * Cp * Cy + Sr * Sp * Sy;
    Q.X = Sr * Cp * Cy - Cr * Sp * Sy;
    Q.Y = Cr * Sp * Cy + Sr * Cp * Sy;
    Q.Z = Cr * Cp * Sy - Sr * Sp * Cy;
    const double N = std::sqrt(Q.X * Q.X + Q.Y * Q.Y + Q.Z * Q.Z + Q.W * Q.W);
    if (N > 0.0) { Q.X /= N; Q.Y /= N; Q.Z /= N; Q.W /= N; }
    return Q;
}

// Everything the viewer and the instruments show at one instant.
struct Pose
{
    double TimeS = 0.0;
    double PosNed[3] = {0, 0, 0};
    double VelNed[3] = {0, 0, 0};
    double RollDeg = 0, PitchDeg = 0, YawDeg = 0, TiltDeg = 0;
    Quat Q;
    double WindNed[3] = {0, 0, 0};
    double AirSpeedMps = 0;
    double Rpm[4] = {0, 0, 0, 0};
    double CurrentA[4] = {0, 0, 0, 0};
    double ThrustN[4] = {0, 0, 0, 0};
    double BatteryV = 0, BatteryA = 0, BatterySoc = 0, BatteryW = 0;
    double WindingC = 0, CaseC = 0, EscC = 0, BatteryC = 0;
    double ReserveRatio = 0;

    double AltitudeM() const { return -PosNed[2]; }
    double GroundSpeedMps() const { return std::hypot(VelNed[0], VelNed[1]); }
    double VerticalSpeedMps() const { return -VelNed[2]; }
    // Heading of the ground track in degrees 0..360 (0 = north); the body heading is YawDeg.
    double HeadingDeg() const { return std::fmod(YawDeg + 360.0, 360.0); }
};

// Shortest-way interpolation of an angle in degrees (a jump of 359 -> -359 passes through 180, not through 0).
inline double LerpAngleDeg(double A, double B, double F)
{
    double D = std::fmod(B - A, 360.0);
    if (D > 180.0) D -= 360.0;
    if (D < -180.0) D += 360.0;
    return A + D * F;
}

class Replay
{
public:
    static constexpr int kSpeedCount = 7;
    static constexpr double kSpeeds[kSpeedCount] = {1, 2, 4, 8, 16, 32, 64};
    static constexpr double kMaxRealStepS = 0.25;   // a stalled frame never jumps the playback

    bool Load(const Series& S, std::string& Err)
    {
        Data = S;
        Cols.clear();
        for (const std::string& N : CampaignDetail::Required())
        {
            const int I = S.Index(N);
            if (I < 0) { Err = "missing column " + N; return false; }
            Cols.push_back(I);
        }
        const std::vector<double>* T = S.Col("t_s");
        if (T == nullptr || T->size() < 1) { Err = "a replay needs at least one row"; return false; }
        for (std::size_t I = 1; I < T->size(); ++I)
            if (!((*T)[I] > (*T)[I - 1])) { Err = "time must increase"; return false; }
        Time = (*T)[0];
        Running = false;
        SpeedIdx = 0;
        return true;
    }

    double Start() const { return Data.Cols.empty() ? 0.0 : (*Data.Col("t_s"))[0]; }
    double End() const { return Data.Cols.empty() ? 0.0 : Data.Col("t_s")->back(); }
    double DurationS() const { return End() - Start(); }
    double TimeS() const { return Time; }
    bool Playing() const { return Running; }
    int SpeedIndex() const { return SpeedIdx; }
    double Speed() const { return kSpeeds[SpeedIdx]; }
    bool AtEnd() const { return Time >= End(); }

    void Play() { if (AtEnd()) Time = Start(); Running = true; }   // playing from the end restarts
    void Pause() { Running = false; }
    void Toggle() { if (Running) Pause(); else Play(); }
    void SetSpeedIndex(int I) { SpeedIdx = I < 0 ? 0 : (I >= kSpeedCount ? kSpeedCount - 1 : I); }
    void FasterStep() { SetSpeedIndex(SpeedIdx + 1); }
    void SlowerStep() { SetSpeedIndex(SpeedIdx - 1); }
    void Seek(double T) { Time = T < Start() ? Start() : (T > End() ? End() : T); }
    void SeekFraction(double F) { Seek(Start() + DurationS() * (F < 0 ? 0 : (F > 1 ? 1 : F))); }
    double Fraction() const { return DurationS() > 0.0 ? (Time - Start()) / DurationS() : 0.0; }

    // Advances by a real-time step; stops (and pauses) at the end of the data.
    void Advance(double RealDtS)
    {
        if (!Running || !(RealDtS > 0.0)) return;
        const double Dt = RealDtS > kMaxRealStepS ? kMaxRealStepS : RealDtS;
        Time += Dt * Speed();
        if (Time >= End()) { Time = End(); Running = false; }
    }

    Pose Sample() const { return SampleAt(Time); }

    Pose SampleAt(double T) const
    {
        const std::vector<double>& Tc = *Data.Col("t_s");
        std::size_t Hi = static_cast<std::size_t>(std::upper_bound(Tc.begin(), Tc.end(), T) - Tc.begin());
        std::size_t Lo;
        double F = 0.0;
        if (Hi == 0) { Lo = Hi = 0; }
        else if (Hi >= Tc.size()) { Lo = Hi = Tc.size() - 1; }
        else { Lo = Hi - 1; F = (T - Tc[Lo]) / (Tc[Hi] - Tc[Lo]); }
        auto V = [&](const char* Name) { const std::vector<double>& C = *Data.Col(Name); return C[Lo] + (C[Hi] - C[Lo]) * F; };
        Pose P;
        P.TimeS = T < Start() ? Start() : (T > End() ? End() : T);
        const char* Pos[3] = {"pos_n_m", "pos_e_m", "pos_d_m"};
        const char* Vel[3] = {"vel_n_mps", "vel_e_mps", "vel_d_mps"};
        const char* Wind[3] = {"wind_n_mps", "wind_e_mps", "wind_d_mps"};
        for (int I = 0; I < 3; ++I) { P.PosNed[I] = V(Pos[I]); P.VelNed[I] = V(Vel[I]); P.WindNed[I] = V(Wind[I]); }
        P.RollDeg = V("roll_deg");
        P.PitchDeg = V("pitch_deg");
        P.TiltDeg = V("tilt_deg");
        {
            const std::vector<double>& Y = *Data.Col("yaw_deg");
            P.YawDeg = LerpAngleDeg(Y[Lo], Y[Hi], F);
        }
        P.Q = QuatFromRollPitchYawDeg(P.RollDeg, P.PitchDeg, P.YawDeg);
        P.AirSpeedMps = V("air_speed_mps");
        for (int M = 0; M < 4; ++M)
        {
            const std::string K = std::to_string(M + 1);
            P.Rpm[M] = V(("rpm_" + K).c_str());
            P.CurrentA[M] = V(("current_a_" + K).c_str());
            P.ThrustN[M] = V(("thrust_n_" + K).c_str());
        }
        P.BatteryV = V("battery_v"); P.BatteryA = V("battery_a"); P.BatterySoc = V("battery_soc"); P.BatteryW = V("battery_w");
        P.WindingC = V("winding_c_max"); P.CaseC = V("case_c_max"); P.EscC = V("esc_c_max"); P.BatteryC = V("battery_c");
        P.ReserveRatio = V("thrust_reserve_ratio");
        return P;
    }

    struct Point
    {
        double T;
        double V;
    };
    // History of one column for the window [now - SpanS, now] (rows only, no interpolation), oldest first. T is seconds before now (<= 0).
    void History(const std::string& Column, double SpanS, std::vector<Point>& Out) const
    {
        Out.clear();
        const std::vector<double>* C = Data.Col(Column);
        if (C == nullptr) return;
        const std::vector<double>& Tc = *Data.Col("t_s");
        for (std::size_t I = 0; I < Tc.size(); ++I)
            if (Tc[I] <= Time && Tc[I] >= Time - SpanS) Out.push_back(Point{Tc[I] - Time, (*C)[I]});
    }

private:
    Series Data;
    std::vector<int> Cols;
    double Time = 0.0;
    bool Running = false;
    int SpeedIdx = 0;
};
}  // namespace DroneApp
