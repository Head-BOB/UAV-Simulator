// Tier: V
// Engine-free description of the helper programs the app starts as child processes (no FFI): the Python build / report / study tools
// and the Rust `run_campaign` example. Builds argument vectors, quotes them for a process command line, reads the settings file
// and the builder's text output, and maps exit codes to messages. Starting the process itself (FPlatformProcess) is the engine layer.
#pragma once

#include <string>
#include <vector>

#include "DroneBuildFiles.h"
#include "DroneJson.h"

namespace DroneApp
{
struct HelperSettings
{
    std::string Python = "python";       // interpreter (P8 decides how it is packaged)
    std::string RepoRoot;                // folder holding tools/, campaign/, physics_core/
    std::string RunCampaignExe;          // staged run_campaign executable; empty = `cargo run --release --example run_campaign`
    std::string ValidateModelsExe;       // staged validate_models executable; empty = `cargo run --release --example validate_models`
    int Threads = 4;
    std::string GitHash = "unknown";     // recorded in every manifest
};

// helpers.json: {"python": "...", "repo_root": "...", "run_campaign_exe": "...", "threads": 4}. Unknown keys are ignored; a missing repo_root is an error.
inline bool ParseHelperSettings(const std::string& Text, HelperSettings& Out, std::string& Err)
{
    const Json::ParseResult R = Json::Parse(Text);
    if (!R.ok) { Err = "helpers.json: " + R.error; return false; }
    HelperSettings S;
    auto Str = [&](const char* K, std::string& Dst) { const Json::Value* V = R.value.Get(K); if (V && V->IsString()) Dst = V->text; };
    Str("python", S.Python); Str("repo_root", S.RepoRoot); Str("run_campaign_exe", S.RunCampaignExe); Str("validate_models_exe", S.ValidateModelsExe); Str("git_hash", S.GitHash);
    const Json::Value* T = R.value.Get("threads");
    if (T && T->IsNumber()) { if (T->number < 1 || T->number > 64 || T->number != static_cast<int>(T->number)) { Err = "threads must be a whole number 1..64"; return false; } S.Threads = static_cast<int>(T->number); }
    if (S.RepoRoot.empty()) { Err = "helpers.json needs repo_root"; return false; }
    Out = S;
    return true;
}

// Quotes one argument for a Windows command line (CommandLineToArgvW rules): quotes only when needed, backslashes before a quote doubled.
inline std::string QuoteArg(const std::string& A)
{
    if (!A.empty() && A.find_first_of(" \t\"") == std::string::npos) return A;
    std::string Out = "\"";
    std::size_t Slashes = 0;
    for (const char C : A)
    {
        if (C == '\\') { ++Slashes; continue; }
        if (C == '"') { Out.append(Slashes * 2 + 1, '\\'); Out += '"'; }
        else { Out.append(Slashes, '\\'); Out += C; }
        Slashes = 0;
    }
    Out.append(Slashes * 2, '\\');
    return Out + "\"";
}

struct Command
{
    std::string Program;
    std::vector<std::string> Args;
    std::string Description;   // for the progress line, e.g. "Building the vehicle"
    std::string ArgumentString() const
    {
        std::string Out;
        for (std::size_t I = 0; I < Args.size(); ++I) Out += (I ? " " : "") + QuoteArg(Args[I]);
        return Out;
    }
};

inline std::string Join(const std::string& Root, const std::string& Rel) { return Root.empty() ? Rel : (Root.back() == '/' || Root.back() == '\\' ? Root + Rel : Root + "/" + Rel); }

// True for a path that starts at a root: a leading slash or backslash, a drive letter followed by a colon and a separator, or a UNC path.
inline bool IsAbsolutePath(const std::string& P)
{
    if (P.empty()) return false;
    if (P[0] == '/' || P[0] == '\\') return true;
    return P.size() >= 3 && ((P[0] >= 'A' && P[0] <= 'Z') || (P[0] >= 'a' && P[0] <= 'z')) && P[1] == ':' && (P[2] == '/' || P[2] == '\\');
}

// A packaged build ships helpers.json with RELATIVE paths ("python/python.exe", "repo", "bin/run_campaign.exe"): resolve them against the
// folder that holds helpers.json. Only the interpreter may be a bare command name with no separator ("python": the system PATH finds it);
// the repo root and the executables are always paths, so "repo" becomes <base>/repo. An absolute path is left alone; an empty value stays empty.
inline void ResolveHelperPaths(HelperSettings& S, const std::string& BaseDir)
{
    auto Fix = [&](std::string& V, bool BareIsCommand) {
        if (V.empty() || IsAbsolutePath(V)) return;
        if (BareIsCommand && V.find('/') == std::string::npos && V.find('\\') == std::string::npos) return;
        V = Join(BaseDir, V);
    };
    Fix(S.Python, true); Fix(S.RepoRoot, false); Fix(S.RunCampaignExe, false); Fix(S.ValidateModelsExe, false);
}


inline Command BuildCommand(const HelperSettings& H, const std::string& SpecFile, const std::string& OutDir)
{
    return Command{H.Python, {Join(H.RepoRoot, "tools/parts/build_vehicle.py"), SpecFile, "--out", OutDir}, "Building the vehicle"};
}

inline Command CampaignCommand(const HelperSettings& H, const std::string& BuildDir, const std::string& ConditionsFile, const std::string& DefinitionsFile,
                               const std::string& OutDir, const std::string& TestsArg)
{
    std::vector<std::string> Tail = {BuildDir, ConditionsFile, DefinitionsFile, OutDir, "--threads", std::to_string(H.Threads), "--git-hash", H.GitHash};
    if (!TestsArg.empty()) { Tail.push_back("--tests"); Tail.push_back(TestsArg); }
    if (!H.RunCampaignExe.empty()) return Command{H.RunCampaignExe, Tail, "Running the tests"};
    std::vector<std::string> Args = {"run", "--release", "--quiet", "--manifest-path", Join(H.RepoRoot, "physics_core/Cargo.toml"), "--example", "run_campaign", "--"};
    Args.insert(Args.end(), Tail.begin(), Tail.end());
    return Command{"cargo", Args, "Running the tests"};
}

// Writes validation_report.json / .csv for a build into the build folder (model-vs-published-data errors).
inline Command ValidationCommand(const HelperSettings& H, const std::string& BuildDir)
{
    if (!H.ValidateModelsExe.empty()) return Command{H.ValidateModelsExe, {BuildDir, BuildDir}, "Checking the models against published data"};
    return Command{"cargo", {"run", "--release", "--quiet", "--manifest-path", Join(H.RepoRoot, "physics_core/Cargo.toml"), "--example", "validate_models", "--", BuildDir, BuildDir},
                   "Checking the models against published data"};
}

inline Command ReportCommand(const HelperSettings& H, const std::string& BuildDir, const std::vector<std::string>& CampaignDirs, const std::string& ValidationFile, const std::string& OutDir)
{
    Command C{H.Python, {Join(H.RepoRoot, "tools/report/make_report.py"), "--build", BuildDir}, "Writing the report"};
    for (const auto& D : CampaignDirs) { C.Args.push_back("--campaign"); C.Args.push_back(D); }
    if (!ValidationFile.empty()) { C.Args.push_back("--validation"); C.Args.push_back(ValidationFile); }
    C.Args.push_back("--out"); C.Args.push_back(OutDir);
    return C;
}

inline Command StudyCommand(const HelperSettings& H, const std::string& BuildSpec, const std::string& ConditionsFile, const std::string& DefinitionsFile, const std::string& OutDir)
{
    return Command{H.Python, {Join(H.RepoRoot, "tools/report/run_study.py"), BuildSpec, "--conditions", ConditionsFile, "--definitions", DefinitionsFile, "--out", OutDir,
                              "--threads", std::to_string(H.Threads)}, "Running the upgrade study"};
}

inline Command StudyReportCommand(const HelperSettings& H, const std::string& StudyDir, const std::string& OutDir)
{
    return Command{H.Python, {Join(H.RepoRoot, "tools/report/make_report.py"), "--study", StudyDir, "--out", OutDir}, "Writing the study report"};
}

enum class Helper { Build, Validation, Campaign, Report, Study };

// What to tell the user for an exit code (0 is success). Codes are those of the tools: build_vehicle 1 = refused, 2 = stale; make_report 1 = stopped.
inline std::string ExitMessage(Helper H, int Code)
{
    if (Code == 0) return "";
    switch (H)
    {
        case Helper::Build: return Code == 1 ? "The builder refused this combination of parts; the reasons are listed." : "The builder failed (exit code " + std::to_string(Code) + ").";
        case Helper::Validation: return "The validation check failed (exit code " + std::to_string(Code) + "); the report will have no validation table.";
        case Helper::Campaign: return "The test run stopped (exit code " + std::to_string(Code) + "); no results were produced for the failed tests.";
        case Helper::Report: return Code == 1 ? "The report tool stopped: " : "The report tool failed (exit code " + std::to_string(Code) + ").";
        case Helper::Study: return Code == 1 ? "The study stopped before it could finish." : "The study failed (exit code " + std::to_string(Code) + ").";
    }
    return "";
}

// A process that could not be started at all (program not found): -1 by convention.
inline std::string NotFoundMessage(const Command& C) { return "Could not start '" + C.Program + "'. Check helpers.json (python / repo_root / run_campaign_exe)."; }

// Lines like "ERROR   E-ESC-I: message" / "WARNING E-...: message" / "INFO    I-...: message" printed by build_vehicle.py.
inline std::vector<Finding> ParseBuilderOutput(const std::string& Text)
{
    std::vector<Finding> Out;
    std::size_t Start = 0;
    while (Start <= Text.size())
    {
        std::size_t End = Text.find('\n', Start);
        if (End == std::string::npos) End = Text.size();
        std::string Line = Text.substr(Start, End - Start);
        if (!Line.empty() && Line.back() == '\r') Line.pop_back();
        Start = End + 1;
        for (const char* Sev : {"ERROR", "WARNING", "INFO"})
        {
            const std::string Tag = Sev;
            if (Line.compare(0, Tag.size(), Tag) != 0) continue;
            std::size_t P = Tag.size();
            while (P < Line.size() && Line[P] == ' ') ++P;
            const std::size_t Colon = Line.find(": ", P);
            if (Colon == std::string::npos || Colon == P) break;
            std::string Lower;
            for (const char C : Tag) Lower += static_cast<char>(C - 'A' + 'a');
            Out.push_back(Finding{Line.substr(P, Colon - P), Lower, Line.substr(Colon + 2)});
            break;
        }
        if (End == Text.size()) break;
    }
    return Out;
}
}  // namespace DroneApp
