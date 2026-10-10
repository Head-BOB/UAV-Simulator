// Tier: V
// DEMO ONLY scripted mission for the flight report: takeoff, climb to 20 m, a 100 m square at 5 m/s,
// 10 s hover, landing. It only produces pilot Commands for the DEMO controller (DroneDemoControl.h); it
// has no physics and no Unreal types, so tools/cpp_ffi_check/mission_test.cpp tests it with plain g++.
// Every number is a demo choice: TODO(HUMAN-VERIFY: demo value).
//
// Geometry: the square starts at the takeoff point, corners (north, east) from the start:
//   (SideM, 0) -> (SideM, SideM) -> (0, SideM) -> (0, 0).  Heading is held (no yaw command).
// Leg command: velocity toward the corner, speed = min(CruiseMps, ApproachGain * distance), so the
// vehicle slows in a 1/ApproachGain-second exponential approach and the leg ends inside ArrivalRadiusM.
//   Time to go from rest over D metres at CruiseMps then approach to ArrivalRadiusM:
//   t = (D - CruiseMps/ApproachGain) / CruiseMps + ln(CruiseMps/(ApproachGain ArrivalRadiusM)) / ApproachGain.
#pragma once

#include "DroneDemoControl.h"

#include <cmath>

namespace DroneMission
{
enum class Phase
{
    Takeoff,
    Climb,
    Leg1,
    Leg2,
    Leg3,
    Leg4,
    Hover,
    Land,
    Done,
    Failed,
};

inline const char* PhaseName(Phase P)
{
    switch (P)
    {
        case Phase::Takeoff: return "takeoff";
        case Phase::Climb: return "climb";
        case Phase::Leg1: return "leg 1";
        case Phase::Leg2: return "leg 2";
        case Phase::Leg3: return "leg 3";
        case Phase::Leg4: return "leg 4";
        case Phase::Hover: return "hover";
        case Phase::Land: return "landing";
        case Phase::Done: return "done";
        case Phase::Failed: return "failed";
    }
    return "unknown";
}

struct Params
{
    double ClimbAltitudeM = 20.0;
    double SideM = 100.0;
    double CruiseMps = 5.0;
    double HoverS = 10.0;
    double ArrivalRadiusM = 1.0;
    double ApproachGain = 0.5;  // 1/s
    double ClimbMps = 2.0;
    double HoldMaxMps = 1.0;    // speed limit of the position hold (hover and landing)
    double MaxDurationS = 600.0;
};

inline bool ParamsValid(const Params& P)
{
    return std::isfinite(P.ClimbAltitudeM) && P.ClimbAltitudeM > 0.0 && std::isfinite(P.SideM) && P.SideM > 0.0 &&
           std::isfinite(P.CruiseMps) && P.CruiseMps > 0.0 && std::isfinite(P.HoverS) && P.HoverS >= 0.0 &&
           std::isfinite(P.ArrivalRadiusM) && P.ArrivalRadiusM > 0.0 && P.ArrivalRadiusM < P.SideM &&
           std::isfinite(P.ApproachGain) && P.ApproachGain > 0.0 && std::isfinite(P.ClimbMps) && P.ClimbMps > 0.0 &&
           std::isfinite(P.HoldMaxMps) && P.HoldMaxMps > 0.0 && std::isfinite(P.MaxDurationS) && P.MaxDurationS > 0.0;
}

struct Pose
{
    double TimeS = 0.0;
    double North = 0.0, East = 0.0;  // metres from the NED origin
    double AltitudeM = 0.0;
    double YawRad = 0.0;
};

class Script
{
public:
    explicit Script(const Params& P = Params()) : Prm(P) {}

    Phase GetPhase() const { return Ph; }
    bool Finished() const { return Ph == Phase::Done || Ph == Phase::Failed; }
    const char* FailureReason() const { return Reason; }
    double PhaseStartS() const { return PhaseT0; }

    // Corner k (0..3) in metres north/east from the start.
    void Corner(int K, double* North, double* East) const
    {
        static const int kN[4] = {1, 1, 0, 0};
        static const int kE[4] = {0, 1, 1, 0};
        const int Kc = K < 0 ? 0 : (K > 3 ? 3 : K);
        *North = OriginN + kN[Kc] * Prm.SideM;
        *East = OriginE + kE[Kc] * Prm.SideM;
    }

    // Distance from the planned path, m: to the current leg's line segment during a leg, else to the start point.
    double CrossTrackM(const Pose& S) const
    {
        if (Ph < Phase::Leg1 || Ph > Phase::Leg4)
        {
            return std::hypot(S.North - OriginN, S.East - OriginE);
        }
        const int K = static_cast<int>(Ph) - static_cast<int>(Phase::Leg1);
        double AN = OriginN, AE = OriginE, BN = 0.0, BE = 0.0;
        if (K > 0) Corner(K - 1, &AN, &AE);
        Corner(K, &BN, &BE);
        const double DN = BN - AN, DE = BE - AE;
        const double Len2 = DN * DN + DE * DE;
        double T = ((S.North - AN) * DN + (S.East - AE) * DE) / Len2;
        T = T < 0.0 ? 0.0 : (T > 1.0 ? 1.0 : T);
        return std::hypot(S.North - (AN + T * DN), S.East - (AE + T * DE));
    }

    // The pilot command for this instant. Calls RequestTakeoff / RequestLand on the controller itself.
    // After Done or Failed it returns a zero command.
    DroneDemo::Command Next(const Pose& S, DroneDemo::Controller& Ctl)
    {
        DroneDemo::Command C;
        if (Finished()) return C;
        if (!Started)
        {
            Started = true;
            OriginN = S.North;
            OriginE = S.East;
            PhaseT0 = S.TimeS;
            StartTime = S.TimeS;
            if (!Ctl.RequestTakeoff(S.AltitudeM)) return Fail("takeoff was refused", S);
        }
        if (S.TimeS - StartTime > Prm.MaxDurationS) return Fail("mission timed out", S);

        switch (Ph)
        {
            case Phase::Takeoff:
                if (Ctl.GetMode() == DroneDemo::Mode::Flying) Enter(Phase::Climb, S);
                break;
            case Phase::Climb:
                if (Ctl.GetAltitudeTargetM() >= Prm.ClimbAltitudeM - 0.05) Enter(Phase::Leg1, S);
                else C.ClimbMps = Prm.ClimbMps;
                break;
            case Phase::Leg1:
            case Phase::Leg2:
            case Phase::Leg3:
            case Phase::Leg4:
            {
                double TN = 0.0, TE = 0.0;
                Corner(LegCornerIndex(), &TN, &TE);
                C = Toward(TN, TE, S, Prm.CruiseMps);
                if (std::hypot(TN - S.North, TE - S.East) <= Prm.ArrivalRadiusM) Enter(NextAfterLeg(), S);
                break;
            }
            case Phase::Hover:
                C = Toward(OriginN, OriginE, S, Prm.HoldMaxMps);  // hold at the start point
                if (S.TimeS - PhaseT0 >= Prm.HoverS)
                {
                    if (!Ctl.RequestLand()) return Fail("landing was refused", S);
                    Enter(Phase::Land, S);
                }
                break;
            case Phase::Land:
                C = Toward(OriginN, OriginE, S, Prm.HoldMaxMps);
                if (Ctl.GetMode() == DroneDemo::Mode::OnGround) Enter(Phase::Done, S);
                break;
            default:
                break;
        }
        return C;
    }

    // The runner calls this when the vehicle crashed, a step failed, or the log overflowed.
    void Abort(const char* Why) { Ph = Phase::Failed; Reason = Why; }

private:
    DroneDemo::Command Fail(const char* Why, const Pose&)
    {
        Abort(Why);
        return DroneDemo::Command();
    }

    void Enter(Phase P, const Pose& S)
    {
        Ph = P;
        PhaseT0 = S.TimeS;
    }

    Phase NextAfterLeg() const
    {
        switch (Ph)
        {
            case Phase::Leg1: return Phase::Leg2;
            case Phase::Leg2: return Phase::Leg3;
            case Phase::Leg3: return Phase::Leg4;
            default: return Phase::Hover;
        }
    }

    // Index into Corner() for the current leg: leg 1 -> corner 0 ... leg 4 -> corner 3 (the start).
    int LegCornerIndex() const { return static_cast<int>(Ph) - static_cast<int>(Phase::Leg1); }

    // Velocity toward a point, limited to MaxMps and to ApproachGain * distance, in the heading frame.
    DroneDemo::Command Toward(double TargetN, double TargetE, const Pose& S, double MaxMps) const
    {
        const double DN = TargetN - S.North, DE = TargetE - S.East;
        const double D = std::hypot(DN, DE);
        DroneDemo::Command C;
        if (D <= 1e-9) return C;
        double Speed = Prm.ApproachGain * D;
        if (Speed > MaxMps) Speed = MaxMps;
        DroneDemo::NedToHeadingFrame(Speed * DN / D, Speed * DE / D, S.YawRad, &C.ForwardMps, &C.RightMps);
        return C;
    }

    Params Prm;
    Phase Ph = Phase::Takeoff;
    bool Started = false;
    double StartTime = 0.0;
    double PhaseT0 = 0.0;
    double OriginN = 0.0, OriginE = 0.0;
    const char* Reason = "";
};
}  // namespace DroneMission
