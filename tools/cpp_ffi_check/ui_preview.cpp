// Tier: V
// Developer tool: renders every screen of the engine-free app to an SVG file from the COMMITTED smoke data, so the layout can be looked
// at (and compared with dev-docs/ui-reference/quad_ui_preview_v4.html) without Unreal. The 3D view is drawn as a flat grey placeholder.
// This is a preview of the display lists, NOT of what Slate will draw.
//   g++ -std=c++17 -Wall -Wextra -Werror -I UE5Project/Plugins/DronePhysicsBridge/Source/DronePhysicsBridge/Public tools/cpp_ffi_check/ui_preview.cpp -o ui_preview
//   ./ui_preview <repo root> <out dir>      then e.g.  rsvg-convert -w 1920 out/run.svg -o run.png
#include "DroneScreens.h"

#include <cstdio>
#include <fstream>
#include <sstream>

using namespace DroneApp;

struct FileIO : AppIO
{
    bool ReadText(const std::string& P, std::string& O) override { std::ifstream f(P, std::ios::binary); if (!f) return false; std::stringstream s; s << f.rdbuf(); O = s.str(); return true; }
    bool WriteText(const std::string&, const std::string&) override { return false; }
    bool Exists(const std::string& P) override { std::ifstream f(P); return static_cast<bool>(f); }
    int StartProcess(const Command&, const std::string&) override { return -1; }
    bool PollProcess(int, int&) override { return true; }
    void OpenPath(const std::string&) override {}
};

static std::string Esc(const std::string& S)
{
    std::string O;
    for (const char C : S) { if (C == '&') O += "&amp;"; else if (C == '<') O += "&lt;"; else if (C == '>') O += "&gt;"; else O += C; }
    return O;
}
static std::string Col(const Rgba& C)
{
    char B[96];
    std::snprintf(B, sizeof B, "rgb(%d,%d,%d)", static_cast<int>(C.R * 255 + 0.5f), static_cast<int>(C.G * 255 + 0.5f), static_cast<int>(C.B * 255 + 0.5f));
    return B;
}
static std::string Op(const Rgba& C) { char B[32]; std::snprintf(B, sizeof B, "%.3f", C.A); return B; }

static void Write(const DrawList& L, const std::string& Path)
{
    std::ofstream o(Path);
    o << "<svg xmlns='http://www.w3.org/2000/svg' width='1920' height='1080' viewBox='0 0 1920 1080' font-family='Roboto, DejaVu Sans, sans-serif'>\n";
    o << "<rect width='1920' height='1080' fill='#4b5662'/><rect y='620' width='1920' height='460' fill='#3d4651'/>\n";   // flat stand-in for the 3D scene
    for (const auto& I : L.Items)
    {
        char B[512];
        switch (I.K)
        {
            case Item::Kind::Rect:
                std::snprintf(B, sizeof B, "<rect x='%.1f' y='%.1f' width='%.1f' height='%.1f' fill='%s' fill-opacity='%s'/>\n", I.X, I.Y, I.W, I.H, Col(I.C).c_str(), Op(I.C).c_str());
                o << B; break;
            case Item::Kind::Line:
                std::snprintf(B, sizeof B, "<line x1='%.1f' y1='%.1f' x2='%.1f' y2='%.1f' stroke='%s' stroke-opacity='%s' stroke-width='%.1f'/>\n", I.X, I.Y, I.W, I.H, Col(I.C).c_str(), Op(I.C).c_str(), I.Thick);
                o << B; break;
            case Item::Kind::Poly:
            {
                o << "<polyline fill='none' stroke='" << Col(I.C) << "' stroke-width='" << I.Thick << "' points='";
                for (const auto& P : I.Pts) o << P.X << "," << P.Y << " ";
                o << "'/>\n"; break;
            }
            case Item::Kind::Text:
            {
                const char* Anchor = I.Al == Align::Left ? "start" : (I.Al == Align::Right ? "end" : "middle");
                const float X = I.Al == Align::Left ? I.X : (I.Al == Align::Right ? I.X + I.W : I.X + I.W * 0.5f);
                std::snprintf(B, sizeof B, "<text x='%.1f' y='%.1f' font-size='%.1f' font-weight='%d' text-anchor='%s' fill='%s' fill-opacity='%s'>", X, I.Y + I.Px * 0.85f, I.Px,
                              I.Wt == Weight::Light ? 300 : (I.Wt == Weight::Medium ? 500 : 400), Anchor, Col(I.C).c_str(), Op(I.C).c_str());
                o << B << Esc(I.Text) << "</text>\n"; break;
            }
        }
    }
    o << "</svg>\n";
}

int main(int argc, char** argv)
{
    if (argc < 3) { std::printf("usage: ui_preview <repo root> <out dir>\n"); return 2; }
    const std::string Root = argv[1], Out = argv[2];
    FileIO IO;
    HelperSettings H;
    H.RepoRoot = Root;
    AppModel M(IO, H, "work");
    M.DefinitionsRel = "campaign/smoke_definitions.json";
    if (!M.Init()) { std::printf("init failed: %s\n", M.Message.c_str()); return 1; }
    // state of a finished session, from the committed reference build and smoke campaign
    const std::string B = Root + "/tools/parts/builds/reference_quad";
    M.VehicleClass = "quad_standard";
    M.Sel.Set("frame", "ref_frame_quad_400"); M.Sel.Set("motor", "tmotor_mn3508_kv700"); M.Sel.Set("esc", "tmotor_air_40a_2_6s");
    M.Sel.Set("propeller", "apc_12x6e"); M.Sel.Set("battery", "ref_battery_generic_4s_5000"); M.Sel.Set("wiring", "ref_avionics_wiring_lump");
    std::string Side, Asm, Err;
    IO.ReadText(B + "/vehicle_build.json", Side); IO.ReadText(B + "/assembly.json", Asm);
    ParseBuildSummary(Side, Asm, M.Summary, Err); ParseAssembly(Asm, {}, M.Prims, Err); M.BuildFindings = M.Summary.Findings;
    M.BuildAttempted = true;
    std::string Pt; IO.ReadText(B + "/powertrain_config.json", Pt); M.Powertrain = Json::Parse(Pt).value;
    M.CampaignDir = B + "/campaign_smoke";
    ReadFile Rd = [&](const std::string& Rel, std::string& O) { return IO.ReadText(M.CampaignDir + "/" + Rel, O); };
    if (!LoadCampaign(Rd, M.Camp, Err)) { std::printf("campaign: %s\n", Err.c_str()); return 1; }
    ComputeMargins(M.Camp, M.Powertrain, M.PeakMargins, Err);
    std::string Rep; IO.ReadText(B + "/report_smoke/report.json", Rep);
    if (!ParseReport(Rep, M.Report, Err)) { std::printf("report: %s\n", Err.c_str()); return 1; }
    M.ReportDir = B + "/report_smoke";
    {   // a hand-made study section spliced into the smoke report, to show the Compare table
        const std::size_t Up = Rep.find("\"upgrades\":"), Va = Rep.find("\"validation\"", Up);
        const std::string Block = "\"upgrades\": {\"status\": \"run\", \"condition_set\": \"smoke\", \"limiting_slot\": \"motor_current\", \"notice\": \"SMOKE definitions: this comparison is not for engineering use. Preview data.\","
            " \"baseline\": {\"limiting_margin\": 1.12}, \"baseline_weakest\": {\"label\": \"Motor current\", \"margin\": 1.12, \"slot\": \"motor_current\"},"
            " \"metrics\": {\"limiting_margin\": {\"label\": \"Limiting margin\", \"unit\": \"ratio\", \"higher_is_better\": true}, \"max_payload_kg\": {\"label\": \"Max payload\", \"unit\": \"kg\", \"higher_is_better\": true},"
            " \"max_wind_mps\": {\"label\": \"Max wind\", \"unit\": \"m/s\", \"higher_is_better\": true}, \"endurance_s\": {\"label\": \"Endurance\", \"unit\": \"s\", \"higher_is_better\": true},"
            " \"peak_winding_c\": {\"label\": \"Peak winding\", \"unit\": \"C\", \"higher_is_better\": false}, \"mass_kg\": {\"label\": \"Mass\", \"unit\": \"kg\", \"higher_is_better\": false},"
            " \"price_usd\": {\"label\": \"Price\", \"unit\": \"USD\", \"higher_is_better\": false}},"
            " \"entries\": [{\"id\": \"motor__a\", \"role\": \"motor\", \"part_id\": \"tmotor_mn3508_kv380\", \"status\": \"tested\", \"rank\": 1, \"rank_note\": \"\", \"warnings\": [], \"weakest\": {\"label\": \"Thrust reserve\", \"margin\": 1.35},"
            " \"deltas\": {\"limiting_margin\": {\"base\": 1.12, \"cand\": 3.04, \"delta\": 1.91, \"better\": true}, \"max_payload_kg\": {\"base\": 2.5, \"cand\": 0.5, \"delta\": -2.0, \"better\": false}, \"max_wind_mps\": {\"base\": 12.5, \"cand\": 12.5, \"delta\": 0, \"better\": null},"
            " \"endurance_s\": {\"base\": 70.3, \"cand\": 65.8, \"delta\": -4.5, \"better\": false}, \"peak_winding_c\": {\"base\": 23.2, \"cand\": 21.3, \"delta\": -1.9, \"better\": true}, \"mass_kg\": {\"base\": 1.536, \"cand\": 1.532, \"delta\": -0.004, \"better\": true}, \"price_usd\": {\"base\": null, \"cand\": null, \"delta\": null, \"better\": null}}},"
            " {\"id\": \"prop__b\", \"role\": \"propeller\", \"part_id\": \"tmotor_cf_13x4_4\", \"status\": \"tested\", \"rank\": null, \"rank_note\": \"propeller table is a scaled stand-in (ASSUMED): not ranked as measured\", \"warnings\": [], \"weakest\": {\"label\": \"Motor current\", \"margin\": 0.85},"
            " \"deltas\": {\"limiting_margin\": {\"base\": 1.12, \"cand\": 0.85, \"delta\": -0.27, \"better\": false}, \"max_payload_kg\": {\"base\": 2.5, \"cand\": 2.5, \"delta\": 0, \"better\": null}}},"
            " {\"id\": \"esc__c\", \"role\": \"esc\", \"part_id\": \"ref_esc_generic_20a\", \"status\": \"rejected\", \"rank\": null, \"rank_note\": \"not compatible: E-ESC-I\", \"deltas\": null, \"weakest\": null}]}, ";
        std::string Study = Rep.substr(0, Up) + Block + Rep.substr(Va);
        if (!ParseReport(Study, M.StudyReport, Err)) { std::printf("study report: %s\n", Err.c_str()); return 1; }
        M.Prog.StudyDone = true;
    }
    M.SelectReplayTest("endurance");
    M.Rep.SeekFraction(0.62);
    M.RefreshLive();
    M.Prog.VehicleClassChosen = M.Prog.PartsComplete = M.Prog.BuildDone = M.Prog.ConditionsValid = M.Prog.CampaignDone = M.Prog.ReportDone = true;
    M.SelectedPartId = "tmotor_mn3508_kv700";
    M.CategoryFilter = "motor";
    M.LimitTest = "hover_payload";
    M.Focus = "cond:wind_speed_mps";
    for (int I = 0; I < kScreenCount; ++I)
    {
        M.Current = static_cast<Screen>(I);
        M.ShowDetails = M.ShowControls = (M.Current == Screen::Run);
        DrawList L;
        BuildScreen(L, M);
        std::string Name = ScreenName(M.Current);
        for (char& C : Name) C = static_cast<char>(std::tolower(static_cast<unsigned char>(C)));
        Write(L, Out + "/" + Name + ".svg");
    }
    std::printf("wrote %d svg files to %s\n", kScreenCount, Out.c_str());
    return 0;
}
