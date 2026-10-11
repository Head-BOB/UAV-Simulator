// Tier: V
// Engine-free build files: the build spec the UI writes for tools/parts/build_vehicle.py, the builder's results the UI reads
// (vehicle_build.json estimates and findings, assembly.json), and the dimension-true primitives of the Build screen.
// Position conversion NED -> Unreal goes ONLY through DroneNedToUnreal.h (audited); sizes are lengths and need no conversion.
#pragma once

#include <cmath>
#include <set>
#include <string>
#include <vector>

#include "DroneCatalogue.h"
#include "DroneJson.h"
#include "DroneNedToUnreal.h"

namespace DroneApp
{
// Build names follow the builder's rule ^[a-z0-9][a-z0-9_]*$ (and a sane length).
inline bool ValidBuildName(const std::string& Name)
{
    if (Name.empty() || Name.size() > 40) return false;
    for (std::size_t I = 0; I < Name.size(); ++I)
    {
        const char C = Name[I];
        const bool Ok = (C >= 'a' && C <= 'z') || (C >= '0' && C <= '9') || (I > 0 && C == '_');
        if (!Ok) return false;
    }
    return true;
}

// Vehicle classes offered by the Vehicle screen. Class ids are the builder's.
struct VehicleClassInfo
{
    const char* Id;
    const char* Label;
    bool Available;
};
inline const std::vector<VehicleClassInfo>& VehicleClasses()
{
    static const std::vector<VehicleClassInfo> V = {{"quad_standard", "Quadcopter", true}, {"quad_fpv_5in", "FPV quadcopter", true}};   // only the two implemented classes are listed
    return V;
}

// The build file for build_vehicle.py: {"name", "vehicle_class", "parts": {role: id}}. Returns "" for an invalid name or an incomplete selection.
inline std::string WriteBuildSpec(const std::string& Name, const std::string& VehicleClass, const Selection& Sel)
{
    if (!ValidBuildName(Name) || VehicleClass.empty() || !Sel.Complete()) return std::string();
    std::string Out = "{\n  \"name\": " + Json::Quote(Name) + ",\n  \"vehicle_class\": " + Json::Quote(VehicleClass) + ",\n  \"parts\": {";
    bool First = true;
    for (const auto& R : Sel.Roles())   // std::map: sorted by role, deterministic
    {
        Out += std::string(First ? "\n    " : ",\n    ") + Json::Quote(R.first) + ": " + Json::Quote(R.second);
        First = false;
    }
    return Out + "\n  }\n}\n";
}

struct Finding
{
    std::string Rule, Severity, Message;
};

// What the builder estimated for a build (design-time estimates; the campaign supersedes them).
struct BuildSummary
{
    double MassKg = 0.0;
    double Cg[3] = {0, 0, 0};            // from the frame centre, NED body axes, m
    double ThrustToWeight = 0.0;
    double HoverPackCurrentA = 0.0;
    double HoverThrottle = 0.0;
    std::string PropBasis;               // UIUC_MEASURED or a scaled stand-in
    std::vector<Finding> Findings;       // errors first (the builder sorts them)
    int Errors() const { int N = 0; for (const auto& F : Findings) N += F.Severity == "error"; return N; }
    int Warnings() const { int N = 0; for (const auto& F : Findings) N += F.Severity == "warning"; return N; }
};

inline bool ParseBuildSummary(const std::string& SidecarText, const std::string& AssemblyText, BuildSummary& Out, std::string& Err)
{
    Out = BuildSummary();
    const Json::ParseResult S = Json::Parse(SidecarText), A = Json::Parse(AssemblyText);
    if (!S.ok) { Err = "vehicle_build.json: " + S.error; return false; }
    if (!A.ok) { Err = "assembly.json: " + A.error; return false; }
    const Json::Value* Est = S.value.Get("estimates");
    const Json::Value* Tot = A.value.Get("totals");
    const Json::Value* Cg = Tot ? Tot->Get("cg_from_frame_centre_m") : nullptr;
    if (!Est || !Tot || !Cg || Cg->Size() != 3 || !Tot->Get("mass_kg")) { Err = "build files lack estimates or totals"; return false; }
    auto Num = [&](const Json::Value* V, const char* K, double& Dst) {
        const Json::Value* X = V->Get(K);
        if (!X || !X->IsNumber()) { Err = std::string("missing number ") + K; return false; }
        Dst = X->number;
        return true;
    };
    if (!Num(Tot, "mass_kg", Out.MassKg) || !Num(Est, "thrust_to_weight", Out.ThrustToWeight) || !Num(Est, "pack_hover_a", Out.HoverPackCurrentA) ||
        !Num(Est, "hover_throttle", Out.HoverThrottle))
        return false;
    for (std::size_t I = 0; I < 3; ++I) Out.Cg[I] = Cg->At(I)->AsNumber(0.0);
    Out.PropBasis = Est->Get("ct_cp_basis") ? Est->Get("ct_cp_basis")->AsString("") : std::string();
    const Json::Value* F = S.value.Get("findings");
    for (std::size_t I = 0; F && I < F->Size(); ++I)
    {
        const Json::Value* E = F->At(I);
        Out.Findings.push_back(Finding{E->Get("rule") ? E->Get("rule")->AsString("") : "", E->Get("severity") ? E->Get("severity")->AsString("") : "",
                                       E->Get("message") ? E->Get("message")->AsString("") : ""});
    }
    return true;
}

enum class PrimitiveKind { Box, Cylinder, Disc };

// One dimension-true stand-in for the viewport. Size is the extent along the primitive's own X / Y / Z axes in metres
// (box dims; arm: length x width x thickness; motor can: diameter x diameter x height; prop disc: diameter x diameter x thickness).
struct Primitive
{
    std::string PartId, Role, Instance;
    PrimitiveKind Kind = PrimitiveKind::Box;
    double SizeM[3] = {0, 0, 0};
    double PosNedM[3] = {0, 0, 0};
    double YawDeg = 0.0;       // arms point along the line from the centre to the arm's own position
    bool Spins = false;
    bool Real = false;         // a real mesh exists for this part; false = dimension-true stand-in
};

constexpr double kDiscThicknessM = 0.002;   // a propeller disc is drawn this thick (display only)

// assembly.json -> primitives. Parts with visual kind "none" (cables) are skipped. RealMeshParts names parts that have a real model.
inline bool ParseAssembly(const std::string& Text, const std::set<std::string>& RealMeshParts, std::vector<Primitive>& Out, std::string& Err)
{
    Out.clear();
    const Json::ParseResult R = Json::Parse(Text);
    if (!R.ok) { Err = "assembly.json: " + R.error; return false; }
    const Json::Value* Parts = R.value.Get("parts");
    if (!Parts || !Parts->IsArray()) { Err = "assembly.json has no parts"; return false; }
    for (std::size_t I = 0; I < Parts->Size(); ++I)
    {
        const Json::Value* P = Parts->At(I);
        const Json::Value* Vis = P->Get("visual");
        const Json::Value* Pos = P->Get("position_m");
        if (!Vis || !Pos || Pos->Size() != 3 || !Vis->Get("kind")) { Err = "a part lacks visual or position"; return false; }
        const std::string Kind = Vis->Get("kind")->AsString("");
        if (Kind == "none") continue;
        Primitive Pr;
        Pr.PartId = P->Get("part_id") ? P->Get("part_id")->AsString("") : std::string();
        Pr.Role = P->Get("role") ? P->Get("role")->AsString("") : std::string();
        Pr.Instance = P->Get("instance") ? P->Get("instance")->AsString("") : std::string();
        for (std::size_t K = 0; K < 3; ++K) Pr.PosNedM[K] = Pos->At(K)->AsNumber(0.0);
        auto Dim = [&](const char* Key, double& Dst) {
            const Json::Value* V = Vis->Get(Key);
            if (!V || !V->IsNumber() || !(V->number > 0.0)) { Err = Pr.Instance + ": visual " + Key + " missing or not positive"; return false; }
            Dst = V->number;
            return true;
        };
        if (Kind == "box")
        {
            const Json::Value* D = Vis->Get("dims_m");
            if (!D || D->Size() != 3) { Err = Pr.Instance + ": box needs dims_m"; return false; }
            for (std::size_t K = 0; K < 3; ++K) Pr.SizeM[K] = D->At(K)->AsNumber(0.0);
            Pr.Kind = PrimitiveKind::Box;
        }
        else if (Kind == "tube")
        {
            if (!Dim("length_m", Pr.SizeM[0]) || !Dim("od_m", Pr.SizeM[1])) return false;
            Pr.SizeM[2] = Pr.SizeM[1];
            Pr.Kind = PrimitiveKind::Box;
        }
        else if (Kind == "plate")
        {
            if (!Dim("length_m", Pr.SizeM[0]) || !Dim("width_m", Pr.SizeM[1]) || !Dim("thickness_m", Pr.SizeM[2])) return false;
            Pr.Kind = PrimitiveKind::Box;
        }
        else if (Kind == "cylinder")
        {
            if (!Dim("diameter_m", Pr.SizeM[0]) || !Dim("height_m", Pr.SizeM[2])) return false;
            Pr.SizeM[1] = Pr.SizeM[0];
            Pr.Kind = PrimitiveKind::Cylinder;
        }
        else if (Kind == "disc")
        {
            if (!Dim("diameter_m", Pr.SizeM[0])) return false;
            Pr.SizeM[1] = Pr.SizeM[0];
            Pr.SizeM[2] = kDiscThicknessM;
            Pr.Kind = PrimitiveKind::Disc;
        }
        else { Err = Pr.Instance + ": unknown visual kind " + Kind; return false; }
        const Json::Value* Spin = Vis->Get("spin");   // the builder writes 1.0 for a part that turns (a number, not a bool)
        Pr.Spins = Spin != nullptr && ((Spin->IsNumber() && Spin->number != 0.0) || Spin->AsBool(false)) && Kind != "box";
        if ((Kind == "tube" || Kind == "plate") && (Pr.PosNedM[0] != 0.0 || Pr.PosNedM[1] != 0.0))
            Pr.YawDeg = std::atan2(Pr.PosNedM[1], Pr.PosNedM[0]) * 180.0 / 3.14159265358979323846;
        const bool Flagged = P->Get("mesh") && P->Get("mesh")->AsString("stand_in") == "real";
        Pr.Real = Flagged || RealMeshParts.count(Pr.PartId) > 0;
        Out.push_back(Pr);
    }
    if (Out.empty()) { Err = "assembly.json has no drawable parts"; return false; }
    return true;
}

// Unreal placement in centimetres via the audited converter (X north, Y east, Z up). Yaw degrees are NED heading about the down axis,
// which is also the sense of rotation about Unreal's up axis seen from above. TODO(HUMAN-VERIFY: engine evidence, same open item as B-6).
struct UePlacement
{
    double LocationCm[3];
    double SizeCm[3];
    double YawDeg;
};
inline UePlacement ToUnreal(const Primitive& P)
{
    const DroneUeVec3 L = DroneNedToUePosition(P.PosNedM);
    return UePlacement{{L.X, L.Y, L.Z}, {P.SizeM[0] * 100.0, P.SizeM[1] * 100.0, P.SizeM[2] * 100.0}, P.YawDeg};
}

struct Bounds
{
    double Min[3] = {0, 0, 0};
    double Max[3] = {0, 0, 0};
    double Radius() const { return 0.5 * std::sqrt((Max[0] - Min[0]) * (Max[0] - Min[0]) + (Max[1] - Min[1]) * (Max[1] - Min[1]) + (Max[2] - Min[2]) * (Max[2] - Min[2])); }
};
// Axis-aligned bounds of the primitive centres grown by half their largest size (good enough to frame the camera).
inline Bounds BoundsOf(const std::vector<Primitive>& Ps)
{
    Bounds B;
    bool First = true;
    for (const auto& P : Ps)
    {
        const double H = 0.5 * std::fmax(P.SizeM[0], std::fmax(P.SizeM[1], P.SizeM[2]));
        for (int K = 0; K < 3; ++K)
        {
            const double Lo = P.PosNedM[K] - H, Hi = P.PosNedM[K] + H;
            if (First || Lo < B.Min[K]) B.Min[K] = Lo;
            if (First || Hi > B.Max[K]) B.Max[K] = Hi;
        }
        First = false;
    }
    return B;
}
}  // namespace DroneApp
