// Tier: V
// Headless deterministic mission runner and report writer (DEMO). Flies the scripted mission
// (takeoff, 20 m, 100 m square at 5 m/s, 10 s hover, landing) on the REAL Rust core for an environment
// preset, then writes the engineering report: report.json, report_findings.csv, report_environments.csv,
// report.html. PRELIMINARY: every limit is CITED or ASSUMED, nothing is validated.
//
// From the repo root (needs `cargo build --release` in physics_core/ first):
//   g++ -std=c++17 -Wall -Wextra -Werror -I UE5Project/Plugins/DronePhysicsBridge/Source/DronePhysicsBridge/Public tools/cpp_ffi_check/mission_runner.cpp -o mission_runner -Lphysics_core/target/release -lphysics_core -Wl,-rpath,$PWD/physics_core/target/release
//   ./mission_runner --out Saved/Reports [--env Beach | --all] [--seed 12345] [--utc 2026-01-01T00:00:00Z] [--config PATH]
// No clock is read: without --utc the report says "not recorded", so the same inputs give identical files.

#include "DroneMissionRunner.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <string>
#include <sys/stat.h>
#include <vector>

static bool WriteFile(const std::string& Path, const std::string& Text)
{
    std::ofstream F(Path, std::ios::binary);
    if (!F) return false;
    F << Text;
    return static_cast<bool>(F);
}

static void MakeDir(const std::string& Path)
{
    ::mkdir(Path.c_str(), 0755);  // POSIX; the Unreal side uses FFileHelper, this tool is for Linux and CI
}

int main(int argc, char** argv)
{
    std::string OutDir = "Saved/Reports";
    std::string EnvId = "Beach";
    std::string Utc;
    std::string ConfigPath = "offline_pipeline/geometry/quad_ref_build_config.json";
    unsigned long long Seed = 12345;
    bool All = false;
    for (int I = 1; I < argc; ++I)
    {
        const std::string A = argv[I];
        auto Next = [&](std::string& Out) {
            if (I + 1 >= argc)
            {
                std::fprintf(stderr, "missing value after %s\n", A.c_str());
                std::exit(2);
            }
            Out = argv[++I];
        };
        if (A == "--out") Next(OutDir);
        else if (A == "--env") Next(EnvId);
        else if (A == "--utc") Next(Utc);
        else if (A == "--config") Next(ConfigPath);
        else if (A == "--seed") { std::string S; Next(S); Seed = std::strtoull(S.c_str(), nullptr, 10); }
        else if (A == "--all") All = true;
        else
        {
            std::fprintf(stderr, "unknown argument %s\n", A.c_str());
            return 2;
        }
    }

    VehicleConfig Config;
    if (ffi_load_vehicle_config(ConfigPath.c_str(), &Config) != 0)
    {
        std::fprintf(stderr, "could not load the vehicle config '%s' (run from the repo root)\n", ConfigPath.c_str());
        return 1;
    }
    std::vector<DroneReport::EnvInfo> Envs;
    for (std::size_t I = 0; I < DroneEnvTable::kRowCount; ++I) Envs.push_back(DroneRun::EnvFromRow(DroneEnvTable::kRows[I]));

    DroneState Start = ffi_create_default_drone_state();
    ThermalState Thermal;
    std::memset(&Thermal, 0, sizeof(Thermal));
    for (double& T : Thermal.motor_temp_c) T = 20.0;

    auto Step = [](DroneState* S, ThermalState* T, const ControlInputs* C, const WindInput* W, const EnvironmentInput* E, const VehicleConfig* Cfg,
                   DebugTelemetry* Tele, FaultLatch* L, double Dt) { return ffi_step_physics(S, T, C, W, E, nullptr, nullptr, Cfg, Tele, L, Dt); };

    MakeDir(OutDir);
    int Written = 0;
    for (const DroneReport::EnvInfo& E : Envs)
    {
        if (!All && E.Id != EnvId) continue;
        const DroneReport::Bundle B = DroneRun::BuildBundle(Step, Config, Start, Thermal, E, Envs, Seed, Utc);
        const std::string Dir = OutDir + "/" + E.Id;
        MakeDir(Dir);
        const bool Ok = WriteFile(Dir + "/report.json", DroneReport::ToJson(B)) && WriteFile(Dir + "/report_findings.csv", DroneReport::ToCsvFindings(B)) &&
                        WriteFile(Dir + "/report_environments.csv", DroneReport::ToCsvEnvironments(B)) && WriteFile(Dir + "/report.html", DroneReport::ToHtml(B));
        if (!Ok)
        {
            std::fprintf(stderr, "could not write the report files in %s\n", Dir.c_str());
            return 1;
        }
        std::printf("%s: %s (min margin %.2f, %s), mission %s in %.1f s -> %s\n", E.Id.c_str(), DroneElec::VerdictText(B.A.V), B.A.MinMargin, B.A.Limiting.c_str(),
                    B.Outcome.Completed ? "completed" : "NOT completed", B.Outcome.DurationS, Dir.c_str());
        ++Written;
    }
    if (Written == 0)
    {
        std::fprintf(stderr, "no environment named '%s'\n", EnvId.c_str());
        return 2;
    }
    return 0;
}
