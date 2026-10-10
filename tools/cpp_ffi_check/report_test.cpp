// Tier: V
// Tests for DroneReport.h and DroneReportOutput.h with synthetic flight logs (no physics, no Unreal).
//   g++ -std=c++17 -Wall -Wextra -Werror -I UE5Project/Plugins/DronePhysicsBridge/Source/DronePhysicsBridge/Public tools/cpp_ffi_check/report_test.cpp -o report_test
// Expected values are hand calculations (shown) cross-checked with an independent python3 f64 run.

#include "DroneReportOutput.h"

#include <cmath>
#include <cstdio>
#include <string>

static int g_failures = 0;

static void Check(bool ok, const char* name)
{
    if (!ok)
    {
        std::printf("FAIL: %s\n", name);
        ++g_failures;
    }
}

static bool Near(double a, double b, double tol)
{
    return std::fabs(a - b) <= tol;
}

static bool Has(const std::string& S, const std::string& Sub)
{
    return S.find(Sub) != std::string::npos;
}

using namespace DroneReport;

// 100 s at 50 Hz in the Leg1 phase: every motor 15 A / 100 W / 4 N / 40 C, battery 20 A at 14.0 V, 0.5 Ah used by the end,
// 0.5 m off the path, tilt 0.1 rad, no wind, no saturation.
static Log SteadyLog()
{
    Log L;
    const std::size_t N = 5000;
    L.Samples.resize(N);
    for (std::size_t I = 0; I < N; ++I)
    {
        Sample& S = L.Samples[I];
        S.TimeS = static_cast<double>(I) * kSampleDtS;
        S.Phase = static_cast<int>(DroneMission::Phase::Leg1);
        S.RhoKgM3 = 1.225;
        for (int M = 0; M < kMotors; ++M)
        {
            S.ThrustN[M] = 4.0;
            S.Rpm[M] = 4000.0;
            S.CurrentA[M] = 15.0;
            S.PowerW[M] = 100.0;
            S.MotorTempC[M] = 40.0;
        }
        S.BatteryA = 20.0;
        S.PackV = 14.0;
        S.UsedAh = 0.5 * static_cast<double>(I + 1) / static_cast<double>(N);
        S.CrossTrackM = 0.5;
        S.TiltRad = 0.1;
    }
    return L;
}

static MissionOutcome Completed()
{
    MissionOutcome O;
    O.Completed = true;
    O.DurationS = 100.0;
    return O;
}

static const Finding* Find(const Analysis& A, const char* Id)
{
    for (const Finding& F : A.Findings)
        if (F.Id == Id) return &F;
    return nullptr;
}

static void TestSteadyMargins()
{
    const Build B = Build::Reference();
    const Analysis A = Analyze(SteadyLog(), B, Completed());
    auto M = [&](const char* Id) { const Finding* F = Find(A, Id); return F ? F->Margin : -1.0; };
    // margin = limit / peak. Motor current 23 / 15; power 510 / 100; ESC 20 / 15; battery current limit 25 C * 5 Ah = 125 A, 125 / 20.
    Check(Near(M("motor_current"), 23.0 / 15.0, 1e-12), "motor current margin 23/15");
    Check(Near(M("motor_power"), 5.1, 1e-12), "motor power margin 510/100");
    Check(Near(M("esc_current"), 20.0 / 15.0, 1e-12), "ESC margin 20/15");
    Check(Near(M("battery_current"), 125.0 / 20.0, 1e-12), "battery current margin 125/20");
    // Voltage sag: allowed = V_oc 14.8 - cut-off 3.3 V * 4 = 14.8 - 13.2 = 1.6 V; actual sag 14.8 - 14.0 = 0.8 V -> margin 2.0.
    Check(Near(M("battery_voltage"), 2.0, 1e-12), "battery sag margin 1.6/0.8");
    // Capacity: usable 5 Ah * 0.8 = 4 Ah, used 0.5 Ah -> 8.
    Check(Near(M("battery_capacity"), 8.0, 1e-12), "battery capacity margin 4/0.5");
    // Temperature: the core's placeholder thermal model is NOT used for a margin; the peak it reported (40 C here) is shown.
    Check(Find(A, "motor_temp") && !Find(A, "motor_temp")->Assessed && Near(Find(A, "motor_temp")->PeakValue, 40.0, 1e-12) && Has(Find(A, "motor_temp")->Cause, "UNVALIDATED"),
          "motor temperature is not assessed, peak shown");
    // Thrust: Tmax = Ct rho nmax^2 D^4 = 0.100256 * 1.225 * 135^2 * 0.3048^4 = 19.318519897 N; 19.3185 / 4 = 4.82963.
    Check(Near(M("prop_thrust"), 4.829629974342024, 1e-9), "prop thrust margin");
    // Arm: lever (0.2 - 0.06) sqrt 2 = 0.19799; sigma = 4 * 0.19799 * 0.008 / 1.815e-9 = 3.4907e6 Pa; 3e8 / sigma = 85.942.
    Check(Near(M("arm_bending"), 85.94217179046049, 1e-6), "arm bending margin");
    Check(Near(M("saturation"), DroneElec::kNoLoadMargin, 1e-12), "no saturation gives the cap");
    Check(Near(M("tracking"), 6.0, 1e-12), "tracking margin 3 m / 0.5 m");
    // Tilt: limit 30 deg = 0.5235987756 rad, peak 0.1 rad -> margin 5.235987756.
    Check(Near(M("tilt"), 0.5235987755982988 / 0.1, 1e-12), "tilt margin 30 deg / 0.1 rad");
    // Ranking: smallest margin first. 1.3333 (ESC), 1.5333 (motor current), 2.0 (battery sag), then 4.83 (thrust) ...
    Check(A.Findings.size() >= 4 && A.Findings[0].Id == "esc_current" && A.Findings[1].Id == "motor_current" && A.Findings[2].Id == "battery_voltage" && A.Findings[3].Id == "prop_thrust",
          "ranked weakest first");
    for (std::size_t I = 1; I < A.Findings.size(); ++I)
    {
        if (A.Findings[I].Assessed && A.Findings[I - 1].Assessed) Check(A.Findings[I - 1].Margin <= A.Findings[I].Margin, "margins are non-decreasing");
    }
    // The frame, the motor temperature and the axial-flow model note are NOT assessed and come last.
    Check(!A.Findings.back().Assessed && !A.Findings[A.Findings.size() - 2].Assessed && !A.Findings[A.Findings.size() - 3].Assessed, "not-assessed findings are last");
    Check(Near(A.MinMargin, 20.0 / 15.0, 1e-12) && A.V == Verdict::Marginal, "verdict marginal at 1.33");
    Check(A.Completed && Has(A.Limiting, "ESC"), "limiting component is an ESC");
}

static void TestCauseAndExceedance()
{
    Log L = SteadyLog();
    // Motor RR (index 2): 22 A from sample 1000 for 700 samples (14 s), in Leg2, wind 11 m/s with a 4 m/s gust part.
    for (std::size_t I = 1000; I < 1700; ++I)
    {
        L.Samples[I].CurrentA[2] = 22.0;
        L.Samples[I].Phase = static_cast<int>(DroneMission::Phase::Leg2);
        L.Samples[I].WindMps = 11.0;
        L.Samples[I].GustMps = 4.0;
    }
    const Analysis A = Analyze(L, Build::Reference(), Completed());
    const Finding* F = Find(A, "motor_current");
    Check(F != nullptr, "motor current finding exists");
    if (!F) return;
    // The 1 s average reaches 22 A at sample 1049 (t = 20.98 s). Margin 23 / 22.
    Check(F->Component == "Motor RR", "worst motor is RR (index 2)");
    Check(Near(F->PeakValue, 22.0, 1e-12) && Near(F->Margin, 23.0 / 22.0, 1e-12), "peak 22 A, margin 23/22");
    Check(Near(F->PeakTimeS, 20.98, 1e-9), "peak at 20.98 s");
    // Samples with the 1 s average at or above 0.9 * 23 = 20.7 A: 669 of them (python3 moving-average emulation) = 13.38 s.
    Check(Near(F->SecondsAbove90, 13.38, 1e-9), "13.38 s at or above 90 % of the limit");
    Check(F->PeakPhase == "leg 2", "peak during leg 2");
    Check(Has(F->Cause, "13.4 s") && Has(F->Cause, "leg 2") && Has(F->Cause, "up to 11.0 m/s") && Has(F->Cause, "gust part 4.0 m/s"), "cause names the duration, phase and wind");
    Check(Has(F->Cause, "CITED") && Has(F->Cause, "23 A"), "cause states the limit and its status");
    Check(!F->Consequence.empty(), "consequence is filled in");
}

static void TestBoundaryAndAeroLoad()
{
    // Exactly 90 % of the limit counts as "at or above 90 %": power 459 W is 459 / 510 = 0.9 of the 510 W limit.
    Log L = SteadyLog();
    for (Sample& S : L.Samples)
        for (int M = 0; M < kMotors; ++M) S.PowerW[M] = 459.0;
    const Analysis A = Analyze(L, Build::Reference(), Completed());
    const Finding* P = Find(A, "motor_power");
    Check(P && Near(P->SecondsAbove90, 100.0, 1e-9), "exactly 90 % counts: all 5000 samples = 100 s");
    // Aero drag adds a quarter of its magnitude to each arm's tip load: thrust 4 N + 8 N / 4 = 6 N -> stress x 1.5.
    Log D = SteadyLog();
    for (Sample& S : D.Samples) S.AeroDragN = 8.0;
    const Finding* Arm = Find(Analyze(D, Build::Reference(), Completed()), "arm_bending");
    Check(Arm && Near(Arm->Margin, 85.94217179046049 / 1.5, 1e-6), "arm margin with an 8 N aero load");
}

static void TestAxialLoss()
{
    // Ct(J) / Ct_static from the UIUC 4030 RPM run (python3 f64 on the same table): J = 0 -> 1; J = 0.05 interpolates between (0, 1) and the
    // first row (0.111927, 0.094848 / 0.100256 = 0.94605809): 1 - 0.05394191 * 0.05 / 0.111927 = 0.97590308.
    Check(AxialCtRatio(0.0) == 1.0 && AxialCtRatio(-1.0) == 1.0 && AxialCtRatio(std::nan("")) == 1.0, "no axial flow: ratio 1");
    Check(Near(AxialCtRatio(0.05), 0.9759030847276828, 1e-12), "ratio at J = 0.05");
    Check(Near(AxialCtRatio(0.111927), 0.9460580912863071, 1e-12), "ratio at the first measured J");
    // J = 0.15 lies between rows (0.141046, 0.091298) and (0.17154, 0.085856): ratio 0.89471011 (python3).
    Check(Near(AxialCtRatio(0.15), 0.8947101140238856, 1e-12), "ratio between rows");
    Check(AxialCtRatio(0.7) > 0.0 && AxialCtRatio(0.7) < 0.03, "beyond the last row it holds the last (near zero) value");
    // A 2 m/s climb at 3623.5 rpm: J = 2 / (60.3917 * 0.3048) = 0.108652, ratio 0.94763640, loss 5.2364 %.
    Log L = SteadyLog();
    for (std::size_t I = 1000; I < 1200; ++I)
    {
        L.Samples[I].AxialClimbMps = 2.0;
        for (int M = 0; M < kMotors; ++M) L.Samples[I].Rpm[M] = 3623.5;
        L.Samples[I].Phase = static_cast<int>(DroneMission::Phase::Climb);
    }
    const Finding* F = Find(Analyze(L, Build::Reference(), Completed()), "prop_axial");
    Check(F && !F->Assessed && Near(F->PeakValue, 5.236359606270346, 1e-6), "axial loss 5.24 % for a 2 m/s climb");
    Check(F && Has(F->Cause, "5.2 %") && Has(F->Cause, "climb") && Has(F->Cause, "optimistic"), "axial loss cause names the percentage, phase and the limitation");
    // Motors (almost) off: J is undefined, so no loss is reported (not a fake 97 %).
    Log Off = SteadyLog();
    for (Sample& S : Off.Samples)
    {
        S.AxialClimbMps = 2.0;
        for (int M = 0; M < kMotors; ++M) S.Rpm[M] = 100.0;
    }
    const Finding* OffF = Find(Analyze(Off, Build::Reference(), Completed()), "prop_axial");
    Check(OffF && OffF->PeakValue == 0.0, "no loss is reported with the motors nearly off");
    // No climb: no loss.
    const Finding* None = Find(Analyze(SteadyLog(), Build::Reference(), Completed()), "prop_axial");
    Check(None && None->PeakValue == 0.0, "no climb: no loss");
}

static void TestSaturationAndFailure()
{
    Log L = SteadyLog();
    for (std::size_t I = 0; I < 500; ++I)
    {
        L.Samples[I].Saturated = true;
        L.Samples[I].Phase = static_cast<int>(DroneMission::Phase::Climb);
    }
    const Analysis A = Analyze(L, Build::Reference(), Completed());
    const Finding* F = Find(A, "saturation");
    // 500 of 5000 samples = 10 % of the flight; limit 5 %: margin 0.05 / 0.10 = 0.5. 500 * 0.02 s = 10 s.
    Check(F && Near(F->Margin, 0.5, 1e-12) && Near(F->PeakValue, 10.0, 1e-12) && Near(F->SecondsAbove90, 10.0, 1e-12), "saturation margin 0.5 for 10 % of the flight");
    Check(F && Has(F->Cause, "10.0 s") && Has(F->Cause, "climb") && Has(F->Cause, "10.0 % of the flight"), "saturation cause");
    Check(A.V == Verdict::NotRecommended && A.Findings[0].Id == "saturation", "saturation ranks first, verdict not recommended");

    // A mission that did not complete: finding "mission" with margin 0 first; verdict not recommended.
    MissionOutcome Bad;
    Bad.Completed = false;
    Bad.Reason = "lost control (tilt 85 deg)";
    Bad.DurationS = 42.5;
    Bad.StepErrors = 2;
    const Analysis B = Analyze(SteadyLog(), Build::Reference(), Bad);
    Check(B.Findings[0].Id == "mission" && B.Findings[0].Margin == 0.0 && Has(B.Findings[0].Cause, "lost control"), "failed mission is the first finding");
    Check(B.V == Verdict::NotRecommended && Has(B.VerdictReason, "lost control") && !B.Completed, "failed mission is not recommended");
}

static void TestEmptyAndStable()
{
    Log Empty;
    const Analysis A = Analyze(Empty, Build::Reference(), Completed());
    bool AnyAssessed = false;
    for (const Finding& F : A.Findings) AnyAssessed = AnyAssessed || F.Assessed;
    Check(!AnyAssessed, "empty log: nothing assessed");
    Check(A.V == Verdict::NotRecommended && Has(A.VerdictReason, "no flight data"), "empty log: not recommended, says why");
}

static void TestMovingAverage()
{
    // Window 3: [1, 2, 3, 4] -> [1, 1.5, 2, 3]  (the first entries average what exists).
    const std::vector<double> V = MovingAverage({1.0, 2.0, 3.0, 4.0}, 3);
    Check(V.size() == 4 && Near(V[0], 1.0, 1e-15) && Near(V[1], 1.5, 1e-15) && Near(V[2], 2.0, 1e-15) && Near(V[3], 3.0, 1e-15), "moving average window 3");
    Check(MovingAverage({}, 3).empty() && MovingAverage({1.0}, 0)[0] == 0.0, "empty input and zero window");
}

static Analysis FakeVariantResult(double Margin, bool Completed)
{
    Analysis R;
    R.Completed = Completed;
    Finding F;
    F.Id = "motor_current";
    F.Component = "Motor FR";
    F.Margin = Margin;
    R.Findings.push_back(F);
    return R;
}

static void TestSuggestions()
{
    Log L = SteadyLog();
    Analysis A = Analyze(L, Build::Reference(), Completed());
    int Calls = 0;
    ReflightFn Run = [&](const Variant& V, Analysis* Out) {
        ++Calls;
        const std::string Id = V.Id;
        if (Id == "slow") { *Out = FakeVariantResult(2.0, true); return true; }
        if (Id == "light") { *Out = FakeVariantResult(1.40, true); return true; }       // worse than 23/15 = 1.533
        if (Id == "battery6") return false;                                              // could not run
        *Out = FakeVariantResult(9.0, false);                                            // better margin but the mission failed
        return true;
    };
    AttachSuggestions(A, Run, 3.0, 20);
    const Finding* F = Find(A, "motor_current");
    Check(F && F->Suggestions.size() == 3, "motor current: slow, light and one untested suggestion");
    if (F && F->Suggestions.size() == 3)
    {
        const Suggestion& Slow = F->Suggestions[0];
        Check(Slow.Tested && Slow.Improves && Near(Slow.MarginBefore, 23.0 / 15.0, 1e-12) && Near(Slow.MarginAfter, 2.0, 1e-12), "slow: tested, improves 1.53 -> 2.00");
        Check(Has(Slow.Evidence, "1.53 -> 2.00") && Has(Slow.Evidence, "same seed"), "slow: evidence text");
        const Suggestion& Light = F->Suggestions[1];
        Check(Light.Tested && !Light.Improves && Has(Light.Evidence, "1.53 -> 1.40"), "light: tried, no improvement");
        const Suggestion& Untested = F->Suggestions[2];
        Check(!Untested.Tested && Has(Untested.Evidence, "NOT TESTED") && Has(Untested.Text, "higher continuous current"), "untested suggestion is labelled");
    }
    // Each variant is run once even though several findings use it: slow + light + battery6 + (arm2mm is analysis-only but still one run).
    Check(Calls <= 4, "each variant runs at most once");
    // A finding at or above the margin threshold gets no suggestions (battery capacity margin 8.0).
    const Finding* Cap = Find(A, "battery_capacity");
    Check(Cap && Cap->Suggestions.empty(), "no suggestions above the threshold");
    // battery_voltage (margin 2.0) suggested the 6000 mAh pack, which could not run: NOT TESTED, never a made-up result.
    const Finding* V = Find(A, "battery_voltage");
    bool SawUntestedBattery = false;
    if (V)
        for (const Suggestion& S : V->Suggestions) SawUntestedBattery = SawUntestedBattery || (!S.Tested && Has(S.Evidence, "could not be run"));
    Check(SawUntestedBattery, "a variant that could not run is NOT TESTED");

    // MaxFindings limits how many findings get suggestions (weakest first): 1 -> only the weakest (esc_current, 1.33).
    Analysis A1 = Analyze(L, Build::Reference(), Completed());
    AttachSuggestions(A1, Run, 3.0, 1);
    int WithSuggestions = 0;
    for (const Finding& Fi : A1.Findings) WithSuggestions += Fi.Suggestions.empty() ? 0 : 1;
    Check(WithSuggestions == 1 && !A1.Findings[0].Suggestions.empty(), "MaxFindings 1: only the weakest finding gets suggestions");

    // A re-flight that did not complete never counts as an improvement.
    Analysis A2 = Analyze(L, Build::Reference(), Completed());
    ReflightFn Failing = [&](const Variant&, Analysis* Out) { *Out = FakeVariantResult(9.0, false); return true; };
    AttachSuggestions(A2, Failing, 3.0, 6);
    const Finding* F2 = Find(A2, "motor_current");
    bool AnyImproves = false;
    if (F2)
        for (const Suggestion& S : F2->Suggestions) AnyImproves = AnyImproves || S.Improves;
    Check(F2 && !AnyImproves, "incomplete re-flight is never an improvement");
    Check(F2 && Has(F2->Suggestions[0].Evidence, "NOT completed"), "incomplete re-flight says so");
}

static void TestEscapes()
{
    Check(EscapeJson("a\"b\\c\nd") == "a\\\"b\\\\c\\nd", "json escapes quote, backslash, newline");
    Check(EscapeJson(std::string("x\x01y")) == "x\\u0001y", "json escapes control characters");
    Check(EscapeHtml("<b>&\"'") == "&lt;b&gt;&amp;&quot;&#39;", "html escapes");
    Check(CsvField("plain") == "plain" && CsvField("a,b") == "\"a,b\"" && CsvField("say \"hi\"") == "\"say \"\"hi\"\"\"", "csv quoting");
    Check(JsonNum(NAN) == "null" && JsonNum(HUGE_VAL) == "null" && JsonNum(1.5) == "1.5", "json numbers: NaN and Inf become null");
}

static Bundle MakeBundle()
{
    Bundle B;
    B.GeneratedUtc = "2026-01-01T00:00:00Z";
    B.Seed = 12345;
    B.Flown.Id = "Mountains";
    B.Flown.Name = "Mountains <test>";
    B.Flown.SiteAltitudeM = 2500.0;
    B.DensityKgM3 = 0.9;
    B.Outcome = Completed();
    B.A = Analyze(SteadyLog(), Build::Reference(), B.Outcome);
    CrossRow R;
    R.Env = B.Flown;
    R.Completed = true;
    R.MinMargin = 1.06;
    R.Limiting = "Battery: lowest pack voltage under load";
    R.V = Verdict::Marginal;
    R.DensityKgM3 = 0.9;
    B.Cross.push_back(R);
    return B;
}

static void TestWriters()
{
    const Bundle B = MakeBundle();
    const std::string J = ToJson(B), J2 = ToJson(B);
    Check(J == J2, "JSON is deterministic");
    int Depth = 0, Min = 0;
    bool InStr = false;
    for (std::size_t I = 0; I < J.size(); ++I)
    {
        const char C = J[I];
        if (InStr)
        {
            if (C == '\\') ++I;
            else if (C == '"') InStr = false;
            continue;
        }
        if (C == '"') InStr = true;
        else if (C == '{' || C == '[') ++Depth;
        else if (C == '}' || C == ']') { --Depth; if (Depth < Min) Min = Depth; }
    }
    Check(Depth == 0 && Min == 0 && !InStr, "JSON brackets and quotes balance");
    Check(Has(J, "\"status\": \"PRELIMINARY - ASSUMED INPUTS") && Has(J, "\"humidity_modelled\":false") && Has(J, "\"generated_utc\": \"2026-01-01T00:00:00Z\""), "JSON carries the honesty banner");
    Check(Has(J, "\"rank\":1,\"id\":\"esc_current\"") && Has(J, "\"assessed\":false") && Has(J, "\"margin\":null"), "JSON ranks, and marks the frame not assessed");
    Check(Has(J, "\"status\":\"ASSUMED\"") && Has(J, "\"status\":\"CITED\"") && Has(J, "\"limitations\""), "JSON lists CITED and ASSUMED items and limitations");
    Check(Has(J, "Mountains <test>"), "JSON keeps '<' as is (it is not HTML)");

    const std::string C1 = ToCsvFindings(B), C2 = ToCsvEnvironments(B);
    Check(Has(C1, "# PRELIMINARY") && Has(C1, "environment,rank,component") && Has(C1, "Mountains,1,ESC FR"), "findings CSV");
    Check(Has(C2, "Mountains,2500,") && Has(C2, "Battery: lowest pack voltage under load,marginal"), "environments CSV");

    const std::string H = ToHtml(B);
    Check(Has(H, "<!doctype html>") && Has(H, "PRELIMINARY - ASSUMED INPUTS") && Has(H, "NOT ASSESSED") && Has(H, "Mountains &lt;test&gt;") && !Has(H, "<test>") && !Has(H, "<script"),
          "HTML is self-contained, escaped, labelled");
    Check(Has(H, ">CITED<") && Has(H, ">ASSUMED<") && Has(H, "marginal"), "HTML shows CITED and ASSUMED");
    Check(!Has(H, "src=\"http") && !Has(H, "href=\"http"), "HTML loads nothing from the network");
}

int main()
{
    TestSteadyMargins();
    TestCauseAndExceedance();
    TestBoundaryAndAeroLoad();
    TestAxialLoss();
    TestSaturationAndFailure();
    TestEmptyAndStable();
    TestMovingAverage();
    TestSuggestions();
    TestEscapes();
    TestWriters();
    if (g_failures == 0)
    {
        std::printf("report_test: all passed\n");
        return 0;
    }
    std::printf("report_test: %d failure(s)\n", g_failures);
    return 1;
}
