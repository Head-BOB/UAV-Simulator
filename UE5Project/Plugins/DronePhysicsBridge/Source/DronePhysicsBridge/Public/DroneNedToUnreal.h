// Tier: F
// NED (right-handed, Z down) to Unreal (left-handed, Z up) conversion math, ADR-001 D5.
// No Unreal includes, so tools/cpp_ffi_check/frame_conversion_test.cpp can test it with plain g++.
// FDronePhysicsBridgeModule is the only caller; nothing else may convert frames (WP-5.2).
//
// Derivation (WP-5.2): Unreal differs from NED by the mirror S = diag(1, 1, -1).
// A vector converts as v_ue = S v_ned. A rotation matrix converts as R_ue = S R_ned S.
// Because det(S) = -1, S R(axis a, angle t) S = R(axis -S a, angle t) for a unit axis a, so the
// quaternion (x, y, z, w) becomes (-x, -y, z, w) (the vector part is -S a sin(t/2)).
// Whether FQuat renders this unmirrored is NOT proven here: engine evidence is BLOCKED-HUMAN (B-6).
#pragma once

struct DroneUeVec3
{
    double X;
    double Y;
    double Z;
};

// Quaternion component order is (x, y, z, w), the same as Rust and FQuat.
struct DroneUeQuat
{
    double X;
    double Y;
    double Z;
    double W;
};

// Position: NED metres (N, E, D) -> Unreal centimetres (N*100, E*100, -D*100).
inline DroneUeVec3 DroneNedToUePosition(const double NedM[3])
{
    return DroneUeVec3{NedM[0] * 100.0, NedM[1] * 100.0, -NedM[2] * 100.0};
}

// Free vector (force N, velocity m/s): axis mapping only, unit unchanged. (x, y, z) -> (x, y, -z).
inline DroneUeVec3 DroneNedToUeVector(const float Ned[3])
{
    return DroneUeVec3{static_cast<double>(Ned[0]), static_cast<double>(Ned[1]), -static_cast<double>(Ned[2])};
}

// Rotation: NED body-to-world quaternion (x, y, z, w) -> Unreal (-x, -y, z, w), order (x, y, z, w).
inline DroneUeQuat DroneNedToUeQuat(const float NedQ[4])
{
    return DroneUeQuat{-static_cast<double>(NedQ[0]), -static_cast<double>(NedQ[1]),
                       static_cast<double>(NedQ[2]), static_cast<double>(NedQ[3])};
}
