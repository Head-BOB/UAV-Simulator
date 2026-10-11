// Tier: V
// Tests for DroneCatalogue.h and DroneBuildFiles.h. Plain C++17, no Unreal.
//   g++ -std=c++17 -Wall -Wextra -Werror -I UE5Project/Plugins/DronePhysicsBridge/Source/DronePhysicsBridge/Public tools/cpp_ffi_check/catalogue_build_test.cpp -o catalogue_build_test
//   ./catalogue_build_test <repo root>
#include "DroneBuildFiles.h"
#include "DroneCatalogue.h"

#include <cmath>
#include <cstdio>
#include <fstream>
#include <sstream>
#include <string>

using namespace DroneApp;

static int g_failures = 0;
static void Check(bool ok, const char* name)
{
    if (!ok)
    {
        std::printf("FAIL: %s\n", name);
        ++g_failures;
    }
}
static bool Near(double a, double b, double tol) { return std::fabs(a - b) <= tol; }
static std::string g_root;
static std::string ReadFileText(const std::string& Rel)
{
    std::ifstream f(g_root + "/" + Rel, std::ios::binary);
    std::stringstream ss;
    ss << f.rdbuf();
    return ss.str();
}

// A tiny hand-made catalogue: numbers chosen so the sums are easy by hand.
static const char* kFixture = R"({"schema_version":1,"parts":[
 {"id":"fr1","category":"frame","manufacturer":"Acme","model":"Quad 400","frame_class":"quad_standard","price_usd":50.0,"price_source_type":"CITED","verification":"Unverified","notes":"",
  "key_fields":[{"name":"mass_kg","value":0.3,"unit":"kg","source_type":"CITED"},{"name":"bounding_box_m","value":[0.2,0.2,0.1],"unit":"m","source_type":"ASSUMED"}]},
 {"id":"fr2","category":"frame","manufacturer":"Acme","model":"Five inch","frame_class":"quad_fpv_5in","price_usd":null,"verification":"Unverified","notes":"",
  "key_fields":[{"name":"mass_kg","value":0.1,"unit":"kg","source_type":"CITED"}]},
 {"id":"m1","category":"motor","manufacturer":"T-Motor","model":"MN3508 KV700","frame_class":null,"price_usd":70.0,"verification":"Unverified","notes":"",
  "key_fields":[{"name":"mass_kg","value":0.1,"unit":"kg","source_type":"CITED"},{"name":"kv_rpm_per_v","value":700,"unit":"rpm/V","source_type":"CITED"}]},
 {"id":"e1","category":"esc","manufacturer":"Acme","model":"One channel","frame_class":null,"price_usd":20.0,"verification":"Unverified","notes":"",
  "key_fields":[{"name":"mass_kg","value":0.03,"unit":"kg","source_type":"ASSUMED"},{"name":"channels","value":1,"unit":"","source_type":"CITED"}]},
 {"id":"e4","category":"esc","manufacturer":"Holy","model":"Four in one","frame_class":null,"price_usd":90.0,"verification":"Unverified","notes":"",
  "key_fields":[{"name":"mass_kg","value":0.05,"unit":"kg","source_type":"CITED"},{"name":"channels","value":4,"unit":"","source_type":"CITED"}]},
 {"id":"p1","category":"propeller","manufacturer":"APC","model":"12x6E","frame_class":null,"price_usd":5.0,"verification":"Unverified","notes":"",
  "key_fields":[{"name":"mass_kg","value":0.02,"unit":"kg","source_type":"CITED"}]},
 {"id":"b1","category":"battery","manufacturer":"Acme","model":"4S 5000","frame_class":null,"price_usd":80.0,"verification":"Unverified","notes":"",
  "key_fields":[{"name":"mass_kg","value":0.5,"unit":"kg","source_type":"CITED"}]}]})";

static void TestFilterAndSelection()
{
    Catalogue C;
    std::string Err;
    Check(C.Load(kFixture, Err), "fixture loads");
    Check(C.Parts().size() == 7 && C.Categories().size() == 5 && C.Find("m1") != nullptr && C.Find("zz") == nullptr, "counts and find");
    Check(C.Find("fr1")->Field("bounding_box_m")->ValueText == "0.2 x 0.2 x 0.1" && C.Find("m1")->Field("kv_rpm_per_v")->ValueText == "700" &&
              C.Find("m1")->Field("mass_kg")->ValueText == "0.1", "value text");
    Check(C.Filter("", "", "", "").size() == 7, "no filter returns all");
    Check(C.Filter("esc", "", "", "").size() == 2, "category filter");
    Check(C.Filter("", "t-motor", "", "").size() == 1 && C.Filter("", "T-MOTOR kv700", "", "").size() == 1, "search is case-insensitive and ANDs the words");
    Check(C.Filter("", "t-motor 12x6", "", "").empty(), "all words must match");
    Check(C.Filter("esc", "", "Holy", "").size() == 1 && C.Filter("esc", "", "Nobody", "").empty(), "manufacturer filter");
    Check(C.Filter("frame", "", "", "quad_standard").size() == 1 && C.Filter("frame", "", "", "quad_fpv_5in")[0]->Id == "fr2", "frame class filter");
    Check(C.Filter("motor", "", "", "quad_fpv_5in").size() == 1, "frame class does not filter other categories");
    Check(C.Manufacturers("esc").size() == 2 && C.Manufacturers("esc")[0] == "Acme", "manufacturers in file order");
    Check(ProvenanceChip("CITED") == "Cited" && ProvenanceChip("ASSUMED") == "Assumed" && ProvenanceChip("CALCULATED") == "Calculated" &&
              ProvenanceChip("MEASURED") == "Measured" && ProvenanceChip("???") == "Unverified", "provenance chips");

    Selection S;
    Check(!S.Complete() && S.MissingRoles().size() == 5, "empty selection is incomplete");
    S.Set("frame", "fr1"); S.Set("motor", "m1"); S.Set("esc", "e1"); S.Set("propeller", "p1");
    Check(!S.Complete() && S.MissingRoles().size() == 1 && S.MissingRoles()[0] == "battery", "one role missing");
    S.Set("battery", "b1");
    Check(S.Complete(), "five required roles complete");
    // by hand: frame 0.3 + 4 x 0.1 + 4 x 0.03 (one-channel ESC) + 4 x 0.02 + 0.5 = 0.3 + 0.4 + 0.12 + 0.08 + 0.5 = 1.40 kg
    // price: 50 + 4 x 70 + 4 x 20 + 4 x 5 + 80 = 50 + 280 + 80 + 20 + 80 = 510 USD
    Totals T = S.Sum(C);
    Check(Near(T.MassKg, 1.40, 1e-12) && T.MassKnown && Near(T.PriceUsd, 510.0, 1e-9) && T.PriceKnown && T.PartCount == 5, "totals by hand (1.40 kg, 510 USD)");
    S.Set("esc", "e4");   // a 4-in-1 ESC counts once: 0.3 + 0.4 + 0.05 + 0.08 + 0.5 = 1.33 kg; 50 + 280 + 90 + 20 + 80 = 520
    T = S.Sum(C);
    Check(Near(T.MassKg, 1.33, 1e-12) && Near(T.PriceUsd, 520.0, 1e-9), "four-in-one ESC counts once");
    S.Set("frame", "fr2");   // frame without a price: mass 0.1 + 0.4 + 0.05 + 0.08 + 0.5 = 1.13, price becomes unknown
    T = S.Sum(C);
    Check(Near(T.MassKg, 1.13, 1e-12) && !T.PriceKnown, "unknown price is flagged, not counted as zero");
    S.Set("frame", "ghost");
    T = S.Sum(C);
    Check(!T.MassKnown && !T.PriceKnown, "a part that is not in the catalogue makes the totals unknown");
    S.Clear("battery");
    Check(!S.Complete() && S.Get("battery").empty(), "clear a role");
    S.ClearAll();
    Check(S.Roles().empty(), "clear all");
    Catalogue Bad;
    Check(!Bad.Load("{\"schema_version\":2,\"parts\":[]}", Err), "schema version refused");
    Check(!Bad.Load("{\"schema_version\":1}", Err), "missing parts refused");
    Check(!Bad.Load("{\"schema_version\":1,\"parts\":[{\"category\":\"x\"}]}", Err), "part without id refused");
    Check(!Bad.Load("not json", Err), "bad json refused");
}

static void TestGeneratedCatalogue()
{
    Catalogue C;
    std::string Err;
    Check(C.Load(ReadFileText("parts_db/parts_catalogue.json"), Err), ("generated catalogue loads: " + Err).c_str());
    Check(C.Parts().size() == 56 && C.Categories().size() == 13, "56 parts in 13 categories (the P1 database)");
    const Part* M = C.Find("tmotor_mn3508_kv700");
    Check(M != nullptr && M->Manufacturer == "T-Motor" && M->Field("kv_rpm_per_v") && Near(M->Field("kv_rpm_per_v")->Number, 700.0, 0.0) && M->PriceKnown() && Near(M->PriceUsd, 69.9, 1e-12), "known motor");
    Check(M->Verification == "Unverified", "nothing is verified yet");
    // the reference build, with the counting rules, against the Python builder's mass sum: motors, props, ESC (one channel x4 if channels == 1)
    Selection S;
    S.Set("frame", "ref_frame_quad_400"); S.Set("motor", "tmotor_mn3508_kv700"); S.Set("esc", "tmotor_air_40a_2_6s"); S.Set("propeller", "apc_12x6e");
    S.Set("battery", "ref_battery_generic_4s_5000"); S.Set("wiring", "ref_avionics_wiring_lump");
    Check(S.Complete(), "reference selection complete");
    const Totals T = S.Sum(C);
    Check(T.MassKnown && T.MassKg > 1.0 && T.MassKg < 1.6, "reference mass sum is in the expected range");
    Check(!T.PriceKnown, "generic reference parts have no price: unknown");
    Check(C.Filter("frame", "", "", "quad_fpv_5in").size() >= 1 && C.Filter("frame", "", "", "quad_fpv_5in").size() < C.Filter("frame", "", "", "").size(), "FPV class sees fewer frames");
}

static void TestBuildFiles()
{
    Check(ValidBuildName("reference_quad") && ValidBuildName("a") && ValidBuildName("9lives") && !ValidBuildName("") && !ValidBuildName("_a") && !ValidBuildName("Abc") &&
              !ValidBuildName("a b") && !ValidBuildName("a-b") && !ValidBuildName("a/b") && !ValidBuildName(std::string(41, 'a')) && ValidBuildName(std::string(40, 'a')), "build name rule");
    Selection S;
    S.Set("frame", "f"); S.Set("motor", "m"); S.Set("esc", "e"); S.Set("propeller", "p"); S.Set("battery", "b");
    const std::string Want = "{\n  \"name\": \"my_build\",\n  \"vehicle_class\": \"quad_standard\",\n  \"parts\": {\n    \"battery\": \"b\",\n    \"esc\": \"e\",\n    \"frame\": \"f\",\n    \"motor\": \"m\",\n    \"propeller\": \"p\"\n  }\n}\n";
    Check(WriteBuildSpec("my_build", "quad_standard", S) == Want, "build spec text");
    Check(WriteBuildSpec("Bad Name", "quad_standard", S).empty() && WriteBuildSpec("my_build", "", S).empty(), "invalid name or class gives nothing");
    Selection Part;
    Part.Set("frame", "f");
    Check(WriteBuildSpec("my_build", "quad_standard", Part).empty(), "incomplete selection gives nothing");
    S.Set("motor", "evil\"id");
    Check(WriteBuildSpec("my_build", "quad_standard", S).find("evil\\\"id") != std::string::npos, "part ids are escaped");
    const auto& V = VehicleClasses();
    Check(V.size() == 5 && V[0].Available && V[1].Available && !V[2].Available && std::string(V[0].Id) == "quad_standard" && std::string(V[1].Id) == "quad_fpv_5in", "vehicle classes");

    // the committed reference build
    BuildSummary B;
    std::string Err;
    Check(ParseBuildSummary(ReadFileText("tools/parts/builds/reference_quad/vehicle_build.json"), ReadFileText("tools/parts/builds/reference_quad/assembly.json"), B, Err), ("summary parses: " + Err).c_str());
    const Json::Value Side = Json::Parse(ReadFileText("tools/parts/builds/reference_quad/vehicle_build.json")).value;
    Check(Near(B.MassKg, 1.536, 1e-9) && Near(B.Cg[2], 0.01002604166666667, 1e-12), "mass and CG from the assembly totals");
    Check(Near(B.ThrustToWeight, Json::Path(Side, {"estimates", "thrust_to_weight"})->number, 0.0) && Near(B.HoverPackCurrentA, 7.378293216434061, 1e-12) && Near(B.HoverThrottle, 0.3981782567296646, 1e-15), "estimates by value");
    Check(B.PropBasis == "UIUC_MEASURED" && B.Warnings() >= 1 && B.Errors() == 0, "findings counted");
    Check(!ParseBuildSummary("{}", ReadFileText("tools/parts/builds/reference_quad/assembly.json"), B, Err), "missing estimates refused");
    Check(!ParseBuildSummary("nope", "{}", B, Err), "bad json refused");

    std::vector<Primitive> Ps;
    Check(ParseAssembly(ReadFileText("tools/parts/builds/reference_quad/assembly.json"), {}, Ps, Err), ("assembly parses: " + Err).c_str());
    // 19 parts in the file, one is the wiring lump (kind none) -> 18 drawable
    Check(Ps.size() == 18, "18 drawable primitives (wiring skipped)");
    const Primitive* Body = nullptr;
    const Primitive* Arm0 = nullptr;
    const Primitive* Motor0 = nullptr;
    const Primitive* Prop0 = nullptr;
    for (const auto& P : Ps)
    {
        if (P.Instance == "body") Body = &P;
        if (P.Instance == "arm_0") Arm0 = &P;
        if (P.Instance == "motor_0") Motor0 = &P;
        if (P.Instance == "prop_0") Prop0 = &P;
    }
    Check(Body && Near(Body->SizeM[0], 0.2, 0.0) && Near(Body->SizeM[2], 0.1, 0.0) && Body->Kind == PrimitiveKind::Box && !Body->Real, "body box");
    // arm_0 at (0.1, 0.1): length 0.282843, od 0.016, heading atan2(0.1, 0.1) = 45 deg
    Check(Arm0 && Near(Arm0->SizeM[0], 0.282843, 1e-9) && Near(Arm0->SizeM[1], 0.016, 0.0) && Near(Arm0->SizeM[2], 0.016, 0.0) && Near(Arm0->YawDeg, 45.0, 1e-6), "arm tube along its diagonal");
    Check(Motor0 && Motor0->Kind == PrimitiveKind::Cylinder && Motor0->Spins && Prop0 && Prop0->Kind == PrimitiveKind::Disc && Near(Prop0->SizeM[2], kDiscThicknessM, 0.0), "motor can and prop disc");
    // conversion: NED (0.1, 0.1, 0) m -> Unreal (10, 10, 0) cm; a point 4.2 cm ABOVE the origin is NED z = -0.042 -> Unreal Z = +4.2 cm
    if (Arm0)
    {
        const UePlacement U = ToUnreal(*Arm0);
        Check(Near(U.LocationCm[0], 10.0, 1e-4) && Near(U.LocationCm[1], 10.0, 1e-4) && Near(U.SizeCm[0], 28.2843, 1e-6), "arm in Unreal cm");
    }
    Primitive Up;
    Up.PosNedM[2] = -0.042;
    Check(Near(ToUnreal(Up).LocationCm[2], 4.2, 1e-9), "up in NED (negative z) is up in Unreal (positive Z)");
    // real-mesh classification: a named part is real, the others are stand-ins
    std::vector<Primitive> Ps2;
    Check(ParseAssembly(ReadFileText("tools/parts/builds/reference_quad/assembly.json"), {"tmotor_mn3508_kv700"}, Ps2, Err), "assembly with a real part");
    int Real = 0, Stand = 0;
    for (const auto& P : Ps2) (P.Real ? Real : Stand)++;
    Check(Real == 4 && Stand == 14, "four motor cans real, the rest stand-ins");
    const Bounds Bd = BoundsOf(Ps);
    Check(Bd.Max[0] > Bd.Min[0] && Bd.Radius() > 0.2 && Bd.Radius() < 1.0, "bounds plausible for a 400 mm quad");
    // FPV plates
    std::vector<Primitive> Fpv;
    Check(ParseAssembly(ReadFileText("tools/parts/builds/fpv_5in/assembly.json"), {}, Fpv, Err), ("FPV assembly parses: " + Err).c_str());
    bool Plate = false;
    for (const auto& P : Fpv) Plate = Plate || (P.Instance == "arm_0" && P.SizeM[2] < P.SizeM[1]);
    Check(Plate, "FPV arm plate is thinner than wide");
    // an arm at (0.2, 0.1) points along atan2(0.1, 0.2) = 26.565051 deg (a diagonal arm at 45 deg cannot tell x from y)
    std::vector<Primitive> One;
    Check(ParseAssembly("{\"parts\":[{\"instance\":\"arm_x\",\"position_m\":[0.2,0.1,0],\"visual\":{\"kind\":\"tube\",\"length_m\":0.3,\"od_m\":0.016}}]}", {}, One, Err) && One.size() == 1 &&
              Near(One[0].YawDeg, 26.56505117707799, 1e-9), "arm yaw by hand for an off-diagonal arm");
    // refusals
    Check(!ParseAssembly("{\"parts\":[{\"position_m\":[0,0,0],\"visual\":{\"kind\":\"sphere\"}}]}", {}, Ps, Err) && Err.find("unknown visual kind") != std::string::npos, "unknown kind refused");
    Check(!ParseAssembly("{\"parts\":[{\"instance\":\"x\",\"position_m\":[0,0,0],\"visual\":{\"kind\":\"disc\",\"diameter_m\":-1}}]}", {}, Ps, Err), "negative size refused");
    Check(!ParseAssembly("{\"parts\":[{\"visual\":{\"kind\":\"box\"}}]}", {}, Ps, Err), "missing position refused");
    Check(!ParseAssembly("{\"parts\":[{\"position_m\":[0,0,0],\"visual\":{\"kind\":\"none\"}}]}", {}, Ps, Err), "only cables is not a drawable vehicle");
    Check(!ParseAssembly("{}", {}, Ps, Err), "no parts refused");
}

int main(int argc, char** argv)
{
    if (argc < 2)
    {
        std::printf("usage: catalogue_build_test <repo root>\n");
        return 2;
    }
    g_root = argv[1];
    TestFilterAndSelection();
    TestGeneratedCatalogue();
    TestBuildFiles();
    if (g_failures == 0) std::printf("catalogue_build_test: all checks passed\n");
    return g_failures == 0 ? 0 : 1;
}
