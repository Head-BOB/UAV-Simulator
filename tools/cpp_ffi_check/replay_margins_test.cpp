// Tier: V
// Tests for DroneJson / DroneCampaignData / DroneReplay / DroneMargins. Plain C++17, no Unreal.
//   g++ -std=c++17 -Wall -Wextra -Werror -I UE5Project/Plugins/DronePhysicsBridge/Source/DronePhysicsBridge/Public tools/cpp_ffi_check/replay_margins_test.cpp -o replay_margins_test
//   ./replay_margins_test <repo root>
// The margin cross-check compares this C++ implementation with the PYTHON report (report_smoke/report.json): two independent
// implementations of the same definitions, on the committed smoke campaign.
#include "DroneCampaignData.h"
#include "DroneMargins.h"
#include "DroneReplay.h"

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
static bool ReadAbs(const std::string& Path, std::string& Out)
{
    std::ifstream f(Path, std::ios::binary);
    if (!f) return false;
    std::stringstream ss;
    ss << f.rdbuf();
    Out = ss.str();
    return true;
}
static Json::Value LoadJsonFile(const std::string& Rel)
{
    std::string t;
    ReadAbs(g_root + "/" + Rel, t);
    return Json::Parse(t).value;
}

static const std::string kSmoke = "tools/parts/builds/reference_quad/campaign_smoke";

static void TestCsvAndLoader()
{
    Series S;
    std::string Err;
    Check(ParseSeries("a,b\n1,2.5\n3,-4e1\n", S, Err) && S.Rows() == 2 && Near((*S.Col("b"))[1], -40.0, 0.0), "csv parses");
    Check(!ParseSeries("a,b\n1,2\n3\n", S, Err) && Err.find("cells") != std::string::npos, "ragged row refused");
    Check(!ParseSeries("a,b\n1,x\n", S, Err) && Err.find("not a number") != std::string::npos, "non-number refused");
    Check(!ParseSeries("a\nnan\n", S, Err), "nan refused");
    Check(!ParseSeries("", S, Err), "empty refused");
    Check(ParseSeries("a,b\r\n1,2\r\n", S, Err) && S.Rows() == 1, "CRLF accepted");

    Campaign C;
    ReadFile Read = [](const std::string& Rel, std::string& Out) { return ReadAbs(g_root + "/" + kSmoke + "/" + Rel, Out); };
    Check(LoadCampaign(Read, C, Err), ("smoke campaign loads: " + Err).c_str());
    Check(C.Tests.size() == 8 && C.Tests[0].Name == "hover_payload" && C.Tests[5].Name == "max_speed" && C.Tests[6].Name == "fpv_sprint" && C.Tests[7].Name == "fpv_punchout" && C.Find("climb") != nullptr && C.Find("nope") == nullptr, "eight tests in order");
    Check(Near(C.Ambient(), 20.0, 0.0) && Near(C.Metric("hover_payload", "passing_payload_kg", -1), 2.5, 0.0) && C.Metric("climb", "nope", -7) == -7.0, "metrics and ambient");
    Check(C.Find("endurance")->Peaks.count("motor_current_a") == 1 && C.Find("climb")->Outcome() == "COMPLETED", "peaks and outcome");
    Check(C.Find("climb")->SeriesFiles.size() == 1 && C.Find("climb")->SeriesFiles[0].second.Rows() == 12, "climb series rows");

    // refusals through a wrapped reader
    auto Wrap = [&](const std::string& File, const std::string& From, const std::string& To) {
        return ReadFile([=](const std::string& Rel, std::string& Out) {
            if (!Read(Rel, Out)) return false;
            if (Rel == File)
            {
                const std::size_t p = Out.find(From);
                if (p != std::string::npos) Out.replace(p, From.size(), To);
            }
            return true;
        });
    };
    Campaign X;
    Check(!LoadCampaign(Wrap("climb/timeseries_climb.csv", "roll_deg", "roll_xx"), X, Err) && Err.find("roll_deg") != std::string::npos, "missing roll column refused");
    Check(!LoadCampaign(Wrap("endurance/manifest.json", "\"ambient_c\": 20", "\"ambient_c\": 25"), X, Err) && Err.find("conditions") != std::string::npos, "disagreeing manifests refused");
    Check(!LoadCampaign(Wrap("campaign_result.json", "\"schema_version\": 1", "\"schema_version\": 2"), X, Err), "schema version refused");
    Check(!LoadCampaign(Wrap("climb/result.json", "\"COMPLETED\"", "\"STEADY_STATE\""), X, Err) && Err.find("outcome") != std::string::npos, "outcome mismatch refused");
    Check(!LoadCampaign(Wrap("campaign_result.json", "\"test\": \"climb\"", "\"test\": \"../x\""), X, Err), "path traversal in a test name refused");
    Check(!LoadCampaign([](const std::string&, std::string&) { return false; }, X, Err) && Err.find("cannot read") != std::string::npos, "unreadable folder refused");
    Check(!LoadCampaign(Wrap("campaign_result.json", "\"test\": \"climb\"", "\"test\": \"a..b\""), X, Err) && Err.find("bad test name") != std::string::npos, "a test name containing .. is refused even without a slash");
}

static Series Tiny()
{
    // two rows at t = 0 and t = 2: pos_n 0 -> 10, yaw 350 -> 10 (the short way is +20 deg), roll 0 -> 20, rpm_1 1000 -> 3000
    Series S;
    for (const std::string& N : CampaignDetail::Required()) S.Names.push_back(N);
    S.Cols.assign(S.Names.size(), std::vector<double>(2, 0.0));
    auto Set = [&](const char* Name, double A, double B) { const int I = S.Index(Name); S.Cols[static_cast<std::size_t>(I)][0] = A; S.Cols[static_cast<std::size_t>(I)][1] = B; };
    Set("t_s", 0.0, 2.0); Set("pos_n_m", 0.0, 10.0); Set("pos_d_m", -5.0, -15.0); Set("yaw_deg", 350.0, 10.0); Set("roll_deg", 0.0, 20.0);
    Set("rpm_1", 1000.0, 3000.0); Set("vel_n_mps", 3.0, 3.0); Set("vel_e_mps", 4.0, 4.0); Set("vel_d_mps", -1.0, -1.0);
    return S;
}

static void TestReplay()
{
    Replay R;
    std::string Err;
    Check(R.Load(Tiny(), Err), "tiny loads");
    Check(Near(R.DurationS(), 2.0, 0.0) && Near(R.Start(), 0.0, 0.0) && Near(R.End(), 2.0, 0.0), "duration");
    // midpoint t = 1: position 5, altitude 10, rpm 2000, roll 10; yaw goes 350 -> 10 through 0: midpoint 0 (= 360 mod)
    R.Seek(1.0);
    Pose P = R.Sample();
    Check(Near(P.PosNed[0], 5.0, 1e-12) && Near(P.AltitudeM(), 10.0, 1e-12) && Near(P.Rpm[0], 2000.0, 1e-9) && Near(P.RollDeg, 10.0, 1e-12), "midpoint interpolation");
    Check(Near(std::fmod(P.YawDeg + 360.0, 360.0), 0.0, 1e-9) || Near(std::fmod(P.YawDeg + 360.0, 360.0), 360.0, 1e-9), "yaw wraps the short way");
    Check(Near(P.GroundSpeedMps(), 5.0, 1e-12) && Near(P.VerticalSpeedMps(), 1.0, 1e-12), "ground speed 3-4-5 and climb rate");
    // clamp at both ends
    Check(Near(R.SampleAt(-5.0).PosNed[0], 0.0, 0.0) && Near(R.SampleAt(99.0).PosNed[0], 10.0, 0.0), "clamped at the ends");
    // clock
    R.Seek(0.0);
    Check(!R.Playing(), "paused after load/seek");
    R.Play();
    R.Advance(0.2);
    Check(Near(R.TimeS(), 0.2, 1e-12), "1x advance");
    R.Advance(5.0);      // a stalled frame counts as 0.25 s real: 0.2 + 0.25
    Check(Near(R.TimeS(), 0.45, 1e-12), "a long frame is capped at 0.25 s");
    R.SetSpeedIndex(3);   // 8x
    R.Advance(0.1);
    Check(Near(R.TimeS(), 0.45 + 0.8, 1e-12) && Near(R.Speed(), 8.0, 0.0), "8x advance");
    R.Advance(10.0);     // a stalled frame is capped at 0.25 s real = 2 s at 8x -> reaches the end
    Check(R.AtEnd() && !R.Playing() && Near(R.TimeS(), 2.0, 0.0), "stops at the end");
    R.Play();
    Check(Near(R.TimeS(), 0.0, 0.0) && R.Playing(), "playing from the end restarts");
    R.Pause();
    R.Advance(1.0);
    Check(Near(R.TimeS(), 0.0, 0.0), "no advance while paused");
    R.SetSpeedIndex(-3);
    Check(R.SpeedIndex() == 0, "speed index clamps low");
    R.SetSpeedIndex(99);
    Check(R.SpeedIndex() == 6 && Near(R.Speed(), 64.0, 0.0), "speed index clamps high to 64x");
    R.FasterStep();
    Check(R.SpeedIndex() == 6, "faster at the top stays");
    R.SeekFraction(0.25);
    Check(Near(R.TimeS(), 0.5, 1e-12) && Near(R.Fraction(), 0.25, 1e-12), "fraction seek");
    R.SeekFraction(7.0);
    Check(R.AtEnd(), "fraction clamps");
    // Seek clamps to the data range
    R.Seek(-5.0);
    Check(Near(R.TimeS(), 0.0, 0.0), "seek before the start clamps to the start");
    R.Seek(99.0);
    Check(Near(R.TimeS(), 2.0, 0.0), "seek past the end clamps to the end");
    // history window: rows at t = 0 and 2, now = 2, span 1.5 -> only the row at t = 2 (T = 0)
    std::vector<Replay::Point> H;
    R.Seek(2.0);
    R.History("pos_n_m", 1.5, H);
    Check(H.size() == 1 && Near(H[0].T, 0.0, 0.0) && Near(H[0].V, 10.0, 0.0), "history window excludes older rows");
    R.History("pos_n_m", 5.0, H);
    Check(H.size() == 2 && Near(H[0].T, -2.0, 0.0), "history window includes both rows");
    R.History("nope", 5.0, H);
    Check(H.empty(), "unknown column gives empty history");
    // refusals
    Series Bad = Tiny();
    Bad.Cols[static_cast<std::size_t>(Bad.Index("t_s"))][1] = 0.0;
    Check(!R.Load(Bad, Err) && Err.find("increase") != std::string::npos, "non-increasing time refused");
    Series NoCol = Tiny();
    NoCol.Names[static_cast<std::size_t>(NoCol.Index("rpm_2"))] = "zz";
    Check(!R.Load(NoCol, Err), "missing column refused");
}

static void TestQuaternion()
{
    // yaw 90 deg: q = (0, 0, sin 45, cos 45)
    Quat Q = QuatFromRollPitchYawDeg(0, 0, 90);
    Check(Near(Q.X, 0, 1e-12) && Near(Q.Y, 0, 1e-12) && Near(Q.Z, std::sqrt(0.5), 1e-12) && Near(Q.W, std::sqrt(0.5), 1e-12), "yaw 90 quaternion");
    // roll 30 about X: q = (sin 15, 0, 0, cos 15)
    Q = QuatFromRollPitchYawDeg(30, 0, 0);
    Check(Near(Q.X, std::sin(15.0 * kPi / 180), 1e-12) && Near(Q.W, std::cos(15.0 * kPi / 180), 1e-12) && Near(Q.Y, 0, 1e-12) && Near(Q.Z, 0, 1e-12), "roll 30 quaternion");
    // pitch 20 about Y
    Q = QuatFromRollPitchYawDeg(0, 20, 0);
    Check(Near(Q.Y, std::sin(10.0 * kPi / 180), 1e-12) && Near(Q.W, std::cos(10.0 * kPi / 180), 1e-12), "pitch 20 quaternion");
    // unit norm for a general attitude, and the Rust extraction formulas give the angles back
    Q = QuatFromRollPitchYawDeg(25, -12, 140);
    Check(Near(Q.X * Q.X + Q.Y * Q.Y + Q.Z * Q.Z + Q.W * Q.W, 1.0, 1e-12), "unit norm");
    const double Roll = std::atan2(2 * (Q.W * Q.X + Q.Y * Q.Z), 1 - 2 * (Q.X * Q.X + Q.Y * Q.Y)) * 180 / kPi;
    const double Pitch = std::asin(2 * (Q.W * Q.Y - Q.Z * Q.X)) * 180 / kPi;
    const double Yaw = std::atan2(2 * (Q.W * Q.Z + Q.X * Q.Y), 1 - 2 * (Q.Y * Q.Y + Q.Z * Q.Z)) * 180 / kPi;
    Check(Near(Roll, 25, 1e-9) && Near(Pitch, -12, 1e-9) && Near(Yaw, 140, 1e-9), "round trip with the core's extraction formulas");
    Check(Near(LerpAngleDeg(350, 10, 0.5), 360.0, 1e-12) && Near(LerpAngleDeg(10, 350, 0.5), 0.0, 1e-12) && Near(LerpAngleDeg(0, 90, 0.5), 45.0, 1e-12), "angle lerp");
}

static void TestMarginsByHand()
{
    Check(Near(CurrentMargin(25.74, 23.0), 1.1191304347826, 1e-12) && CurrentMargin(10, 0) == kMarginCap && CurrentMargin(1000, 1) == kMarginCap, "current margin");
    bool NoHeat = true;
    Check(Near(RiseMargin(120, 70, 20, NoHeat), 2.0, 1e-12) && !NoHeat, "rise margin (120-20)/(70-20) = 2");
    Check(RiseMargin(120, 20.05, 20, NoHeat) == kMarginCap && NoHeat, "no heating below 0.1 K");
    Check(Near(RiseMargin(100, 120, 20, NoHeat), 0.8, 1e-12), "limit exceeded gives below 1");
    Check(StatusOfMargin(0.99) == Status::Critical && StatusOfMargin(1.0) == Status::Caution && StatusOfMargin(1.49) == Status::Caution &&
              StatusOfMargin(1.5) == Status::Ok && StatusOfMargin(99) == Status::Ok, "status thresholds");
    // live margins at a made-up instant: motor 20 A of 25.74, pack 60 A of 125, winding 70 C of 120 (ambient 20), reserve 2
    Pose P;
    P.CurrentA[0] = 10; P.CurrentA[2] = 20; P.BatteryA = 60; P.WindingC = 70; P.EscC = 50; P.BatteryC = 30; P.ReserveRatio = 2.0;
    const Json::Value Pt = Json::Parse("{\"motor_max_current_a\":25.74,\"esc_max_current_a\":40,\"battery_max_continuous_a\":125,\"motor_winding_temp_limit_c\":120,\"esc_temp_limit_c\":100,\"battery_temp_limit_c\":60}").value;
    const std::vector<MarginRow> L = LiveMargins(P, Pt, 20.0);
    Check(L.size() == 7 && Near(L[0].Margin, 25.74 / 20.0, 1e-12) && Near(L[1].Margin, 2.0, 1e-12) && Near(L[2].Margin, 125.0 / 60.0, 1e-12) &&
              Near(L[3].Margin, 2.0, 1e-12) && Near(L[4].Margin, 80.0 / 30.0, 1e-12) && Near(L[5].Margin, 4.0, 1e-12) && Near(L[6].Margin, 2.0, 1e-12), "live margins by hand");
    Check(LimitingIndex(L) == 0 && LimitingIndex({}) == -1, "limiting index");
    const Json::Value Partial = Json::Parse("{\"motor_max_current_a\":25.74}").value;
    const std::vector<MarginRow> L2 = LiveMargins(P, Partial, 20.0);
    Check(L2[1].Unit == "n/a" && L2[1].Margin == kMarginCap, "missing limit shown as n/a, not zero");
}

static void TestMarginsCrossCheckWithPython()
{
    Campaign C;
    std::string Err;
    ReadFile Read = [](const std::string& Rel, std::string& Out) { return ReadAbs(g_root + "/" + kSmoke + "/" + Rel, Out); };
    Check(LoadCampaign(Read, C, Err), "campaign for the cross-check");
    const Json::Value Pt = LoadJsonFile("tools/parts/builds/reference_quad/powertrain_config.json");
    const Json::Value Rep = LoadJsonFile("tools/parts/builds/reference_quad/report_smoke/report.json");
    std::vector<MarginRow> Rows;
    Check(ComputeMargins(C, Pt, Rows, Err), ("margins compute: " + Err).c_str());
    const Json::Value* Py = Rep.Get("margins");
    Check(Py != nullptr && Py->Size() == Rows.size() && Rows.size() == 7, "seven margin rows");
    for (std::size_t I = 0; Py && I < Py->Size() && I < Rows.size(); ++I)
    {
        const Json::Value* R = Py->At(I);
        Check(R->Get("slot")->AsString("") == Rows[I].Slot, ("same order: " + Rows[I].Slot).c_str());
        const double M = R->Get("margin")->AsNumber(-1);
        Check(Near(M, Rows[I].Margin, 1e-6 * std::max(1.0, std::fabs(M))), ("margin equals the Python report: " + Rows[I].Slot).c_str());
        Check(Near(R->Get("peak")->AsNumber(-1), Rows[I].Peak, 1e-6 * std::max(1.0, std::fabs(Rows[I].Peak))), ("peak equals: " + Rows[I].Slot).c_str());
        Check(R->Get("peak_test")->AsString("") == Rows[I].PeakTest, ("peak test equals: " + Rows[I].Slot).c_str());
    }
    Check(!Rows.empty() && Rows[0].Slot == "motor_current", "limiting component is the motor current");
    // refusals
    Json::Value NoLimit = Pt;
    for (auto It = NoLimit.members.begin(); It != NoLimit.members.end(); ++It)
        if (It->first == "esc_temp_limit_c") { NoLimit.members.erase(It); break; }
    Check(!ComputeMargins(C, NoLimit, Rows, Err) && Err.find("esc_temp_limit_c") != std::string::npos, "missing limit refused");
    Json::Value Low = Pt;
    for (auto& M : Low.members)
        if (M.first == "motor_winding_temp_limit_c") M.second.number = 15.0;
    Check(!ComputeMargins(C, Low, Rows, Err) && Err.find("above the ambient") != std::string::npos, "temperature limit below ambient refused");
}

static void TestRealSeriesReplay()
{
    Campaign C;
    std::string Err;
    ReadFile Read = [](const std::string& Rel, std::string& Out) { return ReadAbs(g_root + "/" + kSmoke + "/" + Rel, Out); };
    if (!LoadCampaign(Read, C, Err)) { Check(false, "campaign for the real replay"); return; }
    Replay R;
    Check(R.Load(C.Find("climb")->SeriesFiles[0].second, Err), "real climb series loads");
    const Series& S = C.Find("climb")->SeriesFiles[0].second;
    R.Seek(R.End());
    const Pose P = R.Sample();
    Check(Near(P.PosNed[2], (*S.Col("pos_d_m")).back(), 0.0) && Near(P.Rpm[1], (*S.Col("rpm_2")).back(), 0.0), "end sample equals the last row");
    Check(P.AltitudeM() > 15.0, "the climb test ends high");
    Check(Near(P.Q.X * P.Q.X + P.Q.Y * P.Q.Y + P.Q.Z * P.Q.Z + P.Q.W * P.Q.W, 1.0, 1e-12), "unit quaternion from the recorded angles");
    // determinism: the same time twice gives bit-identical samples
    Check(R.SampleAt(1.234).PosNed[0] == R.SampleAt(1.234).PosNed[0] && R.SampleAt(1.234).Rpm[2] == R.SampleAt(1.234).Rpm[2], "deterministic");
    // events: climb raised prop_outside_table early in the smoke run
    bool Found = false;
    for (const auto& E : C.Find("climb")->Events)
        if (E.Name == "prop_outside_table" && E.Occurred && Near(E.FirstS, 0.004, 1e-9)) Found = true;
    Check(Found, "event markers read from events.json");
}

int main(int argc, char** argv)
{
    if (argc < 2)
    {
        std::printf("usage: replay_margins_test <repo root>\n");
        return 2;
    }
    g_root = argv[1];
    TestCsvAndLoader();
    TestReplay();
    TestQuaternion();
    TestMarginsByHand();
    TestMarginsCrossCheckWithPython();
    TestRealSeriesReplay();
    if (g_failures == 0) std::printf("replay_margins_test: all checks passed\n");
    return g_failures == 0 ? 0 : 1;
}
