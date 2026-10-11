// Tier: V
// Tests for DroneConditions.h and DroneHelpers.h. Plain C++17, no Unreal.
//   g++ -std=c++17 -Wall -Wextra -Werror -I UE5Project/Plugins/DronePhysicsBridge/Source/DronePhysicsBridge/Public tools/cpp_ffi_check/conditions_helpers_test.cpp -o conditions_helpers_test
//   ./conditions_helpers_test <repo root>
#include "DroneConditions.h"
#include "DroneHelpers.h"

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
static std::string ReadRel(const std::string& Rel)
{
    std::ifstream f(g_root + "/" + Rel, std::ios::binary);
    std::stringstream ss;
    ss << f.rdbuf();
    return ss.str();
}

// Sets one field and says whether the whole entry validates.
static bool OkWith(const char* Id, const std::string& Text)
{
    ConditionsText T;
    T.At(Id) = Text;
    ConditionsValid V;
    std::string Err;
    return ValidateConditions(T, V, Err);
}

static void TestRanges()
{
    ConditionsText T;
    ConditionsValid V;
    std::string Err;
    Check(ValidateConditions(T, V, Err), "defaults validate");
    // range edges of the Rust validate(): inclusive ends, the wind direction excludes 360
    Check(OkWith("ambient_c", "-40") && !OkWith("ambient_c", "-40.01") && OkWith("ambient_c", "55") && !OkWith("ambient_c", "55.01"), "ambient -40..55");
    Check(OkWith("site_altitude_m", "4000") && !OkWith("site_altitude_m", "4000.1") && !OkWith("site_altitude_m", "-0.1"), "site altitude 0..4000");
    Check(OkWith("wind_speed_mps", "20") && !OkWith("wind_speed_mps", "20.01") && !OkWith("wind_speed_mps", "-1"), "wind speed 0..20");
    Check(OkWith("wind_from_deg", "0") && OkWith("wind_from_deg", "359.99") && !OkWith("wind_from_deg", "360") && !OkWith("wind_from_deg", "-1"), "wind from [0, 360)");
    Check(OkWith("gust_level", "1") && !OkWith("gust_level", "1.0001") && !OkWith("gust_level", "-0.1"), "gust 0..1");
    Check(OkWith("payload_kg", "50") && !OkWith("payload_kg", "50.1") && !OkWith("payload_kg", "-0.1"), "payload 0..50 kg");
    Check(OkWith("payload_x_m", "2") && OkWith("payload_y_m", "-2") && !OkWith("payload_z_m", "2.01") && !OkWith("payload_x_m", "-2.01"), "payload position within 2 m");
    Check(OkWith("battery_soc", "0") && OkWith("battery_soc", "1") && !OkWith("battery_soc", "1.01") && !OkWith("battery_soc", "-0.01"), "soc 0..1");
    Check(OkWith("battery_temp_c", "") && OkWith("battery_temp_c", "-40") && OkWith("battery_temp_c", "100") && !OkWith("battery_temp_c", "100.1") && !OkWith("battery_temp_c", "-41"), "battery temp optional, -40..100");
    // nothing is clamped and nothing is guessed: these are all refused
    for (const char* Bad : {"", "abc", "1e", "nan", "NaN", "inf", "-inf", "0x10", "1,5", " 5", "5 ", "5 m", "--5", "1e999", "."})
        Check(!OkWith("ambient_c", Bad), (std::string("refused: '") + Bad + "'").c_str());
    Check(OkWith("ambient_c", "1e1") && OkWith("ambient_c", "+5") && OkWith("ambient_c", "-.5"), "scientific notation, a plus sign and a leading point are accepted");
    // the message names the field and the range
    ConditionsText T2;
    T2.At("wind_speed_mps") = "25";
    Check(!ValidateConditions(T2, V, Err) && Err.find("Mean wind speed") == 0 && Err.find("0 to 20 m/s") != std::string::npos, "message names field and range");
    ConditionsText T3;
    T3.At("wind_from_deg") = "360";
    Check(!ValidateConditions(T3, V, Err) && Err.find("not included") != std::string::npos, "message mentions the excluded end");
    // seed, name, turbulence
    for (const char* Seed : {"", "abc", "-1", "1.5", "1e3", "1234567890123456789"})
    {
        ConditionsText S;
        S.Seed = Seed;
        Check(!ValidateConditions(S, V, Err), (std::string("seed refused: '") + Seed + "'").c_str());
    }
    ConditionsText S0;
    S0.Seed = "0";
    Check(ValidateConditions(S0, V, Err) && V.Seed == 0, "seed 0 allowed");
    S0.Seed = "123456789012345678";
    Check(ValidateConditions(S0, V, Err) && V.Seed == 123456789012345678ull, "18-digit seed exact");
    for (const char* Name : {"", "Hot", "a b", "a/b", "../x", "a-b"})
    {
        ConditionsText N;
        N.Name = Name;
        Check(!ValidateConditions(N, V, Err), (std::string("name refused: '") + Name + "'").c_str());
    }
    ConditionsText N2;
    N2.Name = "_ok_1";
    Check(ValidateConditions(N2, V, Err), "leading underscore is allowed by the Rust rule");
    N2.Name = std::string(41, 'a');
    Check(!ValidateConditions(N2, V, Err), "41 characters refused");
    ConditionsText Tu;
    Tu.Turbulence = "storm";
    Check(!ValidateConditions(Tu, V, Err), "unknown turbulence refused");
    Tu.Turbulence = "none";
    Check(ValidateConditions(Tu, V, Err) && V.Turbulence == "none", "turbulence none");
}

static void TestJson()
{
    ConditionsText T;
    T.Name = "hot_day";
    T.At("ambient_c") = "40";
    T.At("wind_speed_mps") = "8.5";
    T.At("payload_x_m") = "0.1";
    T.At("battery_soc") = "0.9";
    ConditionsValid V;
    std::string Err;
    Check(ValidateConditions(T, V, Err), "typed values validate");
    const std::string Want = "{\n  \"schema_version\": 1,\n  \"name\": \"hot_day\",\n  \"ambient_c\": 40,\n  \"site_altitude_m\": 0,\n  \"wind_speed_mps\": 8.5,\n"
                             "  \"wind_from_deg\": 270,\n  \"turbulence\": \"dryden\",\n  \"gust_level\": 0,\n  \"seed\": 1,\n  \"payload_kg\": 0,\n"
                             "  \"payload_position_m\": [0.1, 0, 0],\n  \"battery_soc\": 0.9,\n  \"battery_temp_c\": null\n}\n";
    Check(WriteConditionsJson(V) == Want, "conditions json text");
    T.At("battery_temp_c") = "31.5";
    Check(ValidateConditions(T, V, Err) && WriteConditionsJson(V).find("\"battery_temp_c\": 31.5") != std::string::npos, "battery temperature written when typed");
    // the text parses back to the same numbers
    const Json::ParseResult R = Json::Parse(WriteConditionsJson(V));
    Check(R.ok && Near(R.value.Get("ambient_c")->number, 40.0, 0.0) && Near(R.value.Get("payload_position_m")->At(0)->number, 0.1, 0.0) && R.value.Get("seed")->number == 1.0, "parses back");
    Check(TestsArgument({"climb", "hover_payload", "max_speed"}) == "hover_payload,climb,max_speed" && TestsArgument({}).empty() && TestsArgument({"nope"}).empty() &&
              TestsArgument({"thermal_soak", "endurance"}) == "endurance,thermal_soak", "tests argument in list order");
    const auto& L = TestList();
    bool AllImplemented = true;
    for (const auto& Ti : L) AllImplemented = AllImplemented && Ti.Implemented;
    Check(L.size() == 8 && AllImplemented && std::string(L[5].Id) == "max_speed" && std::string(L[6].Id) == "fpv_sprint" && std::string(L[7].Id) == "fpv_punchout", "test list: eight tests, all implemented");
}

static void TestLimits()
{
    const std::string Text = ReadRel("campaign/test_definitions.json");
    const Json::Value Defs = Json::Parse(Text).value;
    std::vector<EditableLimit> Hover = EditableLimits(Defs, "hover_payload");
    Check(Hover.size() == 10, "hover_payload has 10 editable numbers (output interval and version excluded)");
    auto Find = [](std::vector<EditableLimit>& V, const char* K) -> EditableLimit* { for (auto& E : V) if (E.Key == K) return &E; return nullptr; };
    Check(Find(Hover, "altitude_error_limit_m") && Near(Find(Hover, "altitude_error_limit_m")->Original, 0.5, 0.0) && Find(Hover, "altitude_error_limit_m")->Unit == "m", "altitude error limit 0.5 m");
    Check(Find(Hover, "min_thrust_reserve_ratio")->Unit == "ratio" && Find(Hover, "collective_cap_fraction_limit")->Unit == "fraction" && Find(Hover, "tolerance_kg")->Unit == "kg" &&
              Find(Hover, "hold_s")->Unit == "s" && Find(Hover, "output_interval_s") == nullptr, "units from key suffixes");
    std::vector<EditableLimit> Climb = EditableLimits(Defs, "climb");
    Check(Find(Climb, "command_climb_mps")->Unit == "m/s" && Find(Climb, "accel_limit_mps2")->Unit == "m/s^2", "climb units");
    std::vector<EditableLimit> Soak = EditableLimits(Defs, "thermal_soak");
    Check(Find(Soak, "steady_rate_k_per_min")->Unit == "K/min" && Find(Soak, "stop_on_first_limit") == nullptr && Find(Soak, "freeze_soc") == nullptr, "thermal units; booleans are not editable numbers");
    std::vector<EditableLimit> Endurance = EditableLimits(Defs, "endurance");
    Check(Endurance.size() == 7 && Find(Endurance, "mission") == nullptr, "endurance: 7 numbers (cruise altitude, loop side, loop speed, settle, max duration, cutoff confirm, crash altitude); the mission name is not editable");
    Check(EditableLimits(Defs, "no_such_test").empty(), "unknown test has no limits");
    // max_speed: start_altitude_m, start_speed_mps, step_mps, max_speed_mps, settle_s, hold_s, eval_window_s, speed_fraction_min, altitude_error_limit_m,
    // min_thrust_reserve_ratio, tolerance_mps = 11 numbers (the output interval is not editable); fpv_sprint 3; fpv_punchout 4
    std::vector<EditableLimit> Speed = EditableLimits(Defs, "max_speed");
    Check(Speed.size() == 11 && Find(Speed, "speed_fraction_min") && Near(Find(Speed, "speed_fraction_min")->Original, 0.9, 0.0) && Find(Speed, "tolerance_mps")->Unit == "m/s" &&
              Find(Speed, "min_thrust_reserve_ratio")->Unit == "ratio", "max_speed limits and units");
    Check(EditableLimits(Defs, "fpv_sprint").size() == 3 && EditableLimits(Defs, "fpv_punchout").size() == 4, "FPV manoeuvre limits");

    std::string Out, Err;
    bool Edited = true;
    Check(WriteEditedDefinitions(Text, Hover, Out, Edited, Err) && !Edited, "unchanged values are not an edit");
    Check(Json::Parse(Out).value.Get("definitions_version")->text == "1.1.0", "version unchanged without edits");
    Find(Hover, "altitude_error_limit_m")->Value = 1.0;
    Check(WriteEditedDefinitions(Text, Hover, Out, Edited, Err) && Edited, "edit applied");
    const Json::Value New = Json::Parse(Out).value;
    Check(New.Get("definitions_version")->text == "1.1.0-edited", "version gets -edited");
    Check(Near(Json::Path(New, {"hover_payload", "altitude_error_limit_m"})->number, 1.0, 0.0) && Near(Json::Path(New, {"hover_payload", "hold_s"})->number, 30.0, 0.0) &&
              Near(Json::Path(New, {"wind_gust", "position_error_limit_m"})->number, 2.0, 0.0), "only the edited value changed");
    Check(Json::Path(New, {"endurance", "mission"})->text == "square_loop" && Json::Path(New, {"thermal_soak", "freeze_soc"})->AsBool(false), "strings and booleans survive");
    // editing an already edited file does not stack suffixes
    Check(WriteEditedDefinitions(Out, Hover, Out, Edited, Err) && Json::Parse(Out).value.Get("definitions_version")->text == "1.1.0-edited", "no double suffix");
    Find(Hover, "hold_s")->Value = -1.0;
    Check(!WriteEditedDefinitions(Text, Hover, Out, Edited, Err) && Err.find("hold_s") != std::string::npos, "negative refused");
    Find(Hover, "hold_s")->Value = std::nan("");
    Check(!WriteEditedDefinitions(Text, Hover, Out, Edited, Err), "nan refused");
    Check(!WriteEditedDefinitions("not json", Hover, Out, Edited, Err), "bad definitions text refused");
    // zero is a legal value (wind_gust.start_wind_mps is 0 in the project definitions): unedited and edited-to-zero both pass
    std::vector<EditableLimit> Wind = EditableLimits(Defs, "wind_gust");
    Check(Find(Wind, "start_wind_mps") != nullptr && Near(Find(Wind, "start_wind_mps")->Original, 0.0, 0.0), "start_wind_mps is 0 in the definitions");
    Check(WriteEditedDefinitions(Text, Wind, Out, Edited, Err) && !Edited, "unchanged zero values are accepted");
    Find(Wind, "step_mps")->Value = 0.0;
    Check(WriteEditedDefinitions(Text, Wind, Out, Edited, Err) && Edited && Near(Json::Path(Json::Parse(Out).value, {"wind_gust", "step_mps"})->number, 0.0, 0.0), "an edit to zero is accepted");
    Check(EditableLimit{"t", "k", "", 1.0, 2.0}.Edited() && !EditableLimit{"t", "k", "", 1.0, 1.0}.Edited(), "Edited flag");
}

static void TestHelpers()
{
    Check(QuoteArg("abc") == "abc" && QuoteArg("a b") == "\"a b\"" && QuoteArg("") == "\"\"" && QuoteArg("a\"b") == "\"a\\\"b\"", "QuoteArg basics");
    Check(QuoteArg("C:\\x y\\") == "\"C:\\x y\\\\\"" && QuoteArg("a\\\"b") == "\"a\\\\\\\"b\"" && QuoteArg("C:\\nospace") == "C:\\nospace", "QuoteArg backslash rules");
    HelperSettings H;
    std::string Err;
    Check(ParseHelperSettings("{\"python\":\"py\",\"repo_root\":\"C:/uav sim\",\"run_campaign_exe\":\"\",\"threads\":2,\"git_hash\":\"abc\"}", H, Err) && H.Python == "py" && H.RepoRoot == "C:/uav sim" && H.Threads == 2 && H.GitHash == "abc", "settings parse");
    Check(!ParseHelperSettings("{}", H, Err) && Err.find("repo_root") != std::string::npos, "repo_root required");
    Check(ParseHelperSettings("{\"repo_root\":\"x\",\"threads\":64}", H, Err) && H.Threads == 64 && !ParseHelperSettings("{\"repo_root\":\"x\",\"threads\":65}", H, Err) &&
              !ParseHelperSettings("{\"repo_root\":\"x\",\"threads\":100}", H, Err), "threads 1..64");
    Check(!ParseHelperSettings("{\"repo_root\":\"x\",\"threads\":0}", H, Err) && !ParseHelperSettings("{\"repo_root\":\"x\",\"threads\":2.5}", H, Err) && !ParseHelperSettings("[", H, Err), "bad settings refused");
    HelperSettings S;
    S.Python = "python"; S.RepoRoot = "/r"; S.Threads = 4; S.GitHash = "g";
    const Command B = BuildCommand(S, "/w/spec.json", "/w/out");
    Check(B.Program == "python" && B.Args == std::vector<std::string>{"/r/tools/parts/build_vehicle.py", "/w/spec.json", "--out", "/w/out"}, "build command");
    const Command Cg = CampaignCommand(S, "/w/b", "/w/c.json", "/w/d.json", "/w/o", "climb,endurance");
    Check(Cg.Program == "cargo" && Cg.Args.size() == 8 + 8 + 2 && Cg.Args[0] == "run" && Cg.Args[4] == "/r/physics_core/Cargo.toml" && Cg.Args[7] == "--" && Cg.Args[8] == "/w/b" &&
              Cg.Args[Cg.Args.size() - 2] == "--tests" && Cg.Args.back() == "climb,endurance", "campaign command through cargo");
    S.RunCampaignExe = "/r/bin/run_campaign";
    const Command Ce = CampaignCommand(S, "/w/b", "/w/c.json", "/w/d.json", "/w/o", "");
    Check(Ce.Program == "/r/bin/run_campaign" && Ce.Args == std::vector<std::string>{"/w/b", "/w/c.json", "/w/d.json", "/w/o", "--threads", "4", "--git-hash", "g"}, "campaign command with a staged exe, no --tests");
    const Command R = ReportCommand(S, "/w/b", {"/w/a", "/w/z"}, "/w/v.json", "/w/rep");
    Check(R.Args == std::vector<std::string>{"/r/tools/report/make_report.py", "--build", "/w/b", "--campaign", "/w/a", "--campaign", "/w/z", "--validation", "/w/v.json", "--out", "/w/rep"}, "report command");
    Check(ReportCommand(S, "/w/b", {"/w/a"}, "", "/w/rep").Args.size() == 7, "no validation file, no flag");
    const Command St = StudyCommand(S, "/w/spec.json", "/w/c.json", "/w/d.json", "/w/study");
    Check(St.Args[0] == "/r/tools/report/run_study.py" && St.Args.back() == "4" && StudyReportCommand(S, "/w/study", "/w/rep").Args.size() == 5, "study commands");
    const Command V1 = ValidationCommand(S, "/w/b");
    Check(V1.Program == "cargo" && V1.Args.back() == "/w/b" && V1.Args[V1.Args.size() - 2] == "/w/b" && V1.Args[6] == "validate_models" && V1.Args[7] == "--", "validation command through cargo");
    S.ValidateModelsExe = "/r/bin/validate_models";
    Check(ValidationCommand(S, "/w/b").Program == "/r/bin/validate_models" && ValidationCommand(S, "/w/b").Args == std::vector<std::string>{"/w/b", "/w/b"}, "validation command with a staged exe");
    Check(ExitMessage(Helper::Validation, 2).find("exit code 2") != std::string::npos, "validation exit message");
    Command Q{"python", {"/my tools/x.py", "plain"}, ""};
    Check(Q.ArgumentString() == "\"/my tools/x.py\" plain", "argument string quoting");
    // packaged settings: relative paths resolve against the helpers.json folder; bare commands, absolute and empty values stay
    HelperSettings Rel;
    Rel.Python = "python/python.exe"; Rel.RepoRoot = "repo"; Rel.RunCampaignExe = "bin\\run_campaign.exe"; Rel.ValidateModelsExe = "";
    ResolveHelperPaths(Rel, "C:/Game/DroneData/helpers");
    Check(Rel.Python == "C:/Game/DroneData/helpers/python/python.exe" && Rel.RepoRoot == "C:/Game/DroneData/helpers/repo" && Rel.RunCampaignExe == "C:/Game/DroneData/helpers/bin\\run_campaign.exe" && Rel.ValidateModelsExe.empty(),
          "relative helper paths resolve against the base folder (repo_root \"repo\" has no separator and is still a path)");
    HelperSettings Keep;
    Keep.Python = "python"; Keep.RepoRoot = "D:\\repo"; Keep.RunCampaignExe = "/opt/bin/run"; Keep.ValidateModelsExe = "\\\\server\\share\\v.exe";
    ResolveHelperPaths(Keep, "C:/base/");
    Check(Keep.Python == "python" && Keep.RepoRoot == "D:\\repo" && Keep.RunCampaignExe == "/opt/bin/run" && Keep.ValidateModelsExe == "\\\\server\\share\\v.exe", "a bare command and absolute paths stay as they are");
    HelperSettings Bare;
    Bare.RepoRoot = "repo";
    Bare.Python = "python";
    ResolveHelperPaths(Bare, "/b");
    Check(Bare.RepoRoot == "/b/repo" && Bare.Python == "python", "a bare repo name is a path; a bare interpreter name is a command");
    Check(IsAbsolutePath("C:/x") && IsAbsolutePath("c:\\x") && IsAbsolutePath("/x") && IsAbsolutePath("\\\\s\\x") && !IsAbsolutePath("C:x") && !IsAbsolutePath("x/y") && !IsAbsolutePath("") && !IsAbsolutePath("1:/x"), "absolute path detection");
    Check(Join("/r/", "a") == "/r/a" && Join("/r", "a") == "/r/a" && Join("", "a") == "a" && Join("C:\\r\\", "a") == "C:\\r\\a", "Join");
    Check(ExitMessage(Helper::Build, 0).empty() && ExitMessage(Helper::Build, 1).find("refused") != std::string::npos && ExitMessage(Helper::Campaign, 3).find("exit code 3") != std::string::npos &&
              ExitMessage(Helper::Study, 2).find("exit code 2") != std::string::npos && ExitMessage(Helper::Report, 7).find("exit code 7") != std::string::npos, "exit messages");
    Check(NotFoundMessage(B).find("python") != std::string::npos && NotFoundMessage(B).find("helpers.json") != std::string::npos, "not found message names the program and the settings file");
    const std::vector<Finding> F = ParseBuilderOutput("WARNING W-FULL-I-MOTOR: estimated full-throttle current 27.4 A exceeds the motor's 23.0 A rating\r\nERROR   E-ESC-I: ESC too small\nINFO    I-X: note: with colon\nbuilt ref: mass 1.5 kg\n\nERROR:nope\n");
    Check(F.size() == 3 && F[0].Severity == "warning" && F[0].Rule == "W-FULL-I-MOTOR" && F[0].Message.find("27.4 A") != std::string::npos &&
              F[1].Severity == "error" && F[1].Rule == "E-ESC-I" && F[1].Message == "ESC too small" && F[2].Severity == "info" && F[2].Message == "note: with colon", "builder output parsed");
    const std::vector<Finding> G = ParseBuilderOutput("ERROR   E-A:B: the real message\n");
    Check(G.size() == 1 && G[0].Rule == "E-A:B" && G[0].Message == "the real message", "a rule id may contain a colon; the message starts after the first colon-space");
    Check(ParseBuilderOutput("").empty() && ParseBuilderOutput("nothing here").empty(), "no findings in plain text");
}

int main(int argc, char** argv)
{
    if (argc < 2)
    {
        std::printf("usage: conditions_helpers_test <repo root>\n");
        return 2;
    }
    g_root = argv[1];
    TestRanges();
    TestJson();
    TestLimits();
    TestHelpers();
    if (g_failures == 0) std::printf("conditions_helpers_test: all checks passed\n");
    return g_failures == 0 ? 0 : 1;
}
