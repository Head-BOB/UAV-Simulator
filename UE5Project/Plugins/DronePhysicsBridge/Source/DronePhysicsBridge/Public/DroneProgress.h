// Tier: V
// Engine-free reader of the helper programs' progress lines: `UAVPROGRESS {json}` printed (and flushed) by run_campaign, run_study.py
// and make_report.py with --progress. A line is data from a child process: nothing in it is trusted. Malformed or partial lines are
// skipped, numbers are clamped (percent 0..100, times not negative), an unknown stage is kept as Unknown and never moves the job to
// done. Plain C++17; tested by tools/cpp_ffi_check/progress_test.cpp (and against the real helpers by app_flow_test.cpp).
#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <string>
#include <vector>

#include "DroneJson.h"

namespace DroneApp
{
enum class ProgStage { Preparing, Test, Study, Report, Done, Failed, Unknown };

inline const char* ProgStageName(ProgStage S)
{
    switch (S)
    {
        case ProgStage::Preparing: return "preparing";
        case ProgStage::Test: return "test";
        case ProgStage::Study: return "study";
        case ProgStage::Report: return "report";
        case ProgStage::Done: return "done";
        case ProgStage::Failed: return "failed";
        case ProgStage::Unknown: return "unknown";
    }
    return "unknown";
}

enum class ProgState { Waiting, Running, Done, Failed };

struct ProgTest
{
    std::string Id;
    ProgState State = ProgState::Waiting;
    double Percent = 0.0;      // 0..100
    double SimS = 0.0;
};

struct ProgressLine
{
    ProgStage Stage = ProgStage::Unknown;
    std::string Event;         // "tick", "start", "test_done:<id>", ...
    std::string TestId;
    int TestIndex = 0;
    int TestCount = 0;
    double SimDoneS = 0.0;
    double SimTotalS = 0.0;
    double PercentOverall = 0.0;   // 0..100
    double PercentTest = 0.0;      // 0..100
    double ElapsedS = 0.0;
    double EtaS = -1.0;            // < 0: no estimate yet
    std::vector<ProgTest> Tests;
};

namespace ProgressDetail
{
inline double Clamp(double V, double Lo, double Hi) { return V < Lo ? Lo : (V > Hi ? Hi : V); }
inline bool Num(const Json::Value& O, const char* Key, double& Out)
{
    const Json::Value* V = O.Get(Key);
    if (V == nullptr || !V->IsNumber() || !std::isfinite(V->number)) return false;
    Out = V->number;
    return true;
}
inline ProgStage StageOf(const std::string& S)
{
    if (S == "preparing") return ProgStage::Preparing;
    if (S == "test") return ProgStage::Test;
    if (S == "study") return ProgStage::Study;
    if (S == "report") return ProgStage::Report;
    if (S == "done") return ProgStage::Done;
    if (S == "failed") return ProgStage::Failed;
    return ProgStage::Unknown;
}
inline ProgState StateOf(const std::string& S)
{
    if (S == "running") return ProgState::Running;
    if (S == "done") return ProgState::Done;
    if (S == "failed") return ProgState::Failed;
    return ProgState::Waiting;
}
}  // namespace ProgressDetail

// One line (with or without the trailing newline). False when it is not a UAVPROGRESS line, the JSON is broken, or a required number
// is missing or not finite; Out is untouched then.
inline bool ParseProgressLine(const std::string& Raw, ProgressLine& Out)
{
    static const std::string kTag = "UAVPROGRESS ";
    std::string Line = Raw;
    while (!Line.empty() && (Line.back() == '\n' || Line.back() == '\r')) Line.pop_back();
    if (Line.compare(0, kTag.size(), kTag) != 0) return false;
    const Json::ParseResult R = Json::Parse(Line.substr(kTag.size()));
    if (!R.ok || !R.value.IsObject()) return false;
    const Json::Value& O = R.value;
    ProgressLine L;
    const Json::Value* St = O.Get("stage");
    if (St == nullptr || !St->IsString()) return false;
    L.Stage = ProgressDetail::StageOf(St->text);
    if (const Json::Value* E = O.Get("event")) if (E->IsString()) L.Event = E->text;
    if (const Json::Value* T = O.Get("test_id")) if (T->IsString()) L.TestId = T->text;
    double V = 0.0;
    if (!ProgressDetail::Num(O, "percent_overall", V)) return false;
    L.PercentOverall = ProgressDetail::Clamp(V, 0.0, 100.0);
    if (ProgressDetail::Num(O, "percent_test", V)) L.PercentTest = ProgressDetail::Clamp(V, 0.0, 100.0);
    if (ProgressDetail::Num(O, "test_index", V)) L.TestIndex = static_cast<int>(ProgressDetail::Clamp(V, 0.0, 1000.0));
    if (ProgressDetail::Num(O, "test_count", V)) L.TestCount = static_cast<int>(ProgressDetail::Clamp(V, 0.0, 1000.0));
    if (ProgressDetail::Num(O, "sim_s_done", V)) L.SimDoneS = ProgressDetail::Clamp(V, 0.0, 1e9);
    if (ProgressDetail::Num(O, "sim_s_total", V)) L.SimTotalS = ProgressDetail::Clamp(V, 0.0, 1e9);
    if (ProgressDetail::Num(O, "elapsed_s", V)) L.ElapsedS = ProgressDetail::Clamp(V, 0.0, 1e9);
    if (ProgressDetail::Num(O, "eta_s", V)) L.EtaS = ProgressDetail::Clamp(V, 0.0, 1e9);
    if (const Json::Value* Ts = O.Get("tests"))
        for (std::size_t I = 0; Ts->IsArray() && I < Ts->Size() && I < 64; ++I)
        {
            const Json::Value* T = Ts->At(I);
            if (T == nullptr || !T->IsObject()) continue;
            ProgTest P;
            if (const Json::Value* Id = T->Get("id")) if (Id->IsString()) P.Id = Id->text;
            if (const Json::Value* S = T->Get("state")) if (S->IsString()) P.State = ProgressDetail::StateOf(S->text);
            if (ProgressDetail::Num(*T, "percent", V)) P.Percent = ProgressDetail::Clamp(V, 0.0, 100.0);
            if (ProgressDetail::Num(*T, "sim_s", V)) P.SimS = ProgressDetail::Clamp(V, 0.0, 1e9);
            L.Tests.push_back(P);
        }
    Out = L;
    return true;
}

// What the app knows about the running job.
struct ProgressSnapshot
{
    bool Have = false;
    ProgressLine Latest;
    unsigned Lines = 0;                       // valid lines seen: changes whenever the helper said something
    std::vector<std::string> FinishedTests;   // ids from "test_done:<id>" events, in the order they arrived, no duplicates
};

// Feeds raw output chunks (any size, a line may be split across chunks) and keeps the snapshot. A line without its newline yet is
// held back; one longer than kMaxLine is dropped whole. Never throws, never reads beyond what it was given.
class ProgressParser
{
public:
    static constexpr std::size_t kMaxLine = 64 * 1024;

    void Feed(const std::string& Chunk)
    {
        for (const char C : Chunk)
        {
            if (C == '\n')
            {
                if (!Overflow) Take(Pending);
                Pending.clear();
                Overflow = false;
            }
            else if (!Overflow)
            {
                Pending.push_back(C);
                if (Pending.size() > kMaxLine) { Pending.clear(); Overflow = true; }
            }
        }
    }
    const ProgressSnapshot& Snapshot() const { return Snap; }
    // Number of lines that began with the tag but could not be used.
    unsigned Rejected() const { return Bad; }

private:
    void Take(const std::string& Line)
    {
        static const std::string kEvent = "test_done:";
        ProgressLine L;
        if (!ParseProgressLine(Line, L))
        {
            if (Line.compare(0, 12, "UAVPROGRESS ") == 0) ++Bad;
            return;
        }
        Snap.Have = true;
        Snap.Latest = L;
        ++Snap.Lines;
        if (L.Event.compare(0, kEvent.size(), kEvent) == 0)
        {
            const std::string Id = L.Event.substr(kEvent.size());
            if (!Id.empty() && std::find(Snap.FinishedTests.begin(), Snap.FinishedTests.end(), Id) == Snap.FinishedTests.end()) Snap.FinishedTests.push_back(Id);
        }
    }
    std::string Pending;
    bool Overflow = false;
    unsigned Bad = 0;
    ProgressSnapshot Snap;
};

// "1:05" style time for the status line; "" for a negative (unknown) value.
inline std::string ClockText(double Seconds)
{
    if (!(Seconds >= 0.0) || Seconds > 359999.0) return std::string();
    const int S = static_cast<int>(Seconds + 0.5);
    char B[32];
    if (S >= 3600) std::snprintf(B, sizeof B, "%d:%02d:%02d", S / 3600, (S / 60) % 60, S % 60);
    else std::snprintf(B, sizeof B, "%d:%02d", S / 60, S % 60);
    return B;
}
}  // namespace DroneApp
