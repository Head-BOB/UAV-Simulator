// Tier: V
// Tests for DroneElectrical.h. Plain C++17, no Unreal, no framework.
//   g++ -std=c++17 -Wall -Wextra -Werror -I UE5Project/Plugins/DronePhysicsBridge/Source/DronePhysicsBridge/Public tools/cpp_ffi_check/electrical_test.cpp -o electrical_test
// Expected values are hand calculations (shown in the comments) cross-checked with an independent python3 f64 run.

#include "DroneElectrical.h"

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

using namespace DroneElec;

static void TestPower()
{
    double P = 0.0, Q = 0.0;
    // Cp 0.03, rho 1.2, 6000 rpm (n = 100 rev/s), D 0.3 m: P = 0.03 * 1.2 * 1e6 * 0.3^5 = 0.03 * 1.2 * 1e6 * 0.00243 = 87.48 W.
    Check(ShaftPowerW(0.03, 1.2, 6000.0, 0.3, &P) && Near(P, 87.48, 1e-9), "shaft power 87.48 W");
    // Q = P / (2 pi n) = 87.48 / 628.3185 = 0.139229 N m.
    Check(ShaftTorqueNm(P, 6000.0, &Q) && Near(Q, 0.13922874421679, 1e-12), "shaft torque");
    // Power scales with the cube of rpm: half the rpm -> 1/8 the power.
    double Half = 0.0;
    Check(ShaftPowerW(0.03, 1.2, 3000.0, 0.3, &Half) && Near(Half, P / 8.0, 1e-12), "power ~ rpm^3");
    // Zero rpm: zero power and zero torque (no division by zero).
    Check(ShaftPowerW(0.03, 1.2, 0.0, 0.3, &P) && P == 0.0 && ShaftTorqueNm(0.0, 0.0, &Q) && Q == 0.0, "zero rpm");
    // Invalid input leaves the output untouched.
    double Keep = 5.0;
    Check(!ShaftPowerW(-0.1, 1.2, 6000.0, 0.3, &Keep) && Keep == 5.0, "negative Cp rejected");
    Check(!ShaftPowerW(0.03, 0.0, 6000.0, 0.3, &Keep) && !ShaftPowerW(0.03, 1.2, 6000.0, 0.0, &Keep), "zero density or diameter rejected");
    Check(!ShaftPowerW(0.03, 1.2, std::nan(""), 0.3, &Keep) && Keep == 5.0, "NaN rpm rejected");
    Check(!ShaftPowerW(0.03, 1.2, 6000.0, 0.3, nullptr), "null out rejected");
    Check(!ShaftTorqueNm(-1.0, 6000.0, &Keep) && Keep == 5.0, "negative power rejected");
}

static void TestMotor()
{
    double I = 0.0, V = 0.0, Ib = 0.0;
    // Kt = 60 / (2 pi 700) = 0.013641852 N m/A. Q = 0.139228744 N m -> Q / Kt = 10.206 A, + I0 0.5 = 10.706 A.
    Check(MotorCurrentA(0.13922874421679, 6000.0, 700.0, 0.5, &I) && Near(I, 10.706, 1e-9), "motor current 10.706 A");
    // V = rpm / Kv + I Rm = 6000 / 700 + 10.706 * 0.072 = 8.571429 + 0.770832 = 9.342261 V.
    Check(MotorVoltageV(6000.0, 700.0, I, 0.072, &V) && Near(V, 9.342260571428572, 1e-9), "motor voltage");
    // Battery current = V I / (eta Vb) = 9.342261 * 10.706 / (0.95 * 14.8) = 100.0176 / 14.06 = 7.11367 A.
    Check(BatteryCurrentA(V, I, 14.8, 0.95, &Ib) && Near(Ib, 7.113672950050799, 1e-9), "battery current per motor");
    // Motor off: no current at all (not even the no-load current).
    Check(MotorCurrentA(0.0, 0.0, 700.0, 0.5, &I) && I == 0.0, "off motor draws nothing");
    // Torque zero but turning: only the no-load current.
    Check(MotorCurrentA(0.0, 3000.0, 700.0, 0.5, &I) && Near(I, 0.5, 1e-15), "no-load current while turning");
    double Keep = 9.0;
    Check(!MotorCurrentA(0.1, 6000.0, 0.0, 0.5, &Keep) && Keep == 9.0, "zero Kv rejected");
    Check(!MotorCurrentA(-0.1, 6000.0, 700.0, 0.5, &Keep), "negative torque rejected");
    Check(!BatteryCurrentA(9.0, 10.0, 14.8, 0.0, &Keep) && !BatteryCurrentA(9.0, 10.0, 14.8, 1.01, &Keep), "efficiency must be in (0, 1]");
    Check(!BatteryCurrentA(9.0, 10.0, 0.0, 0.95, &Keep), "zero pack voltage rejected");
    Check(!MotorVoltageV(6000.0, 700.0, -1.0, 0.072, &Keep) && Keep == 9.0, "negative current rejected");
}

static void TestPack()
{
    double V = 0.0;
    // 14.8 V open circuit, 50 A through 0.02 ohm: 14.8 - 1.0 = 13.8 V.
    Check(PackTerminalV(14.8, 50.0, 0.02, &V) && Near(V, 13.8, 1e-12), "pack sag");
    Check(PackTerminalV(14.8, 0.0, 0.02, &V) && V == 14.8, "no load: open circuit");
    double Keep = 1.0;
    Check(!PackTerminalV(0.0, 1.0, 0.02, &Keep) && !PackTerminalV(14.8, -1.0, 0.02, &Keep) && Keep == 1.0, "bad pack input rejected");
}

static void TestArm()
{
    double I = 0.0, S = 0.0;
    // D = 0.016, d = 0.013: D^4 = 6.5536e-8, d^4 = 2.8561e-8, difference 3.6975e-8, I = pi * 3.6975e-8 / 64 = 1.815006e-9 m^4.
    Check(TubeSecondMomentM4(0.016, 0.013, &I) && Near(I, 1.8150060682262915e-9, 1e-21), "tube second moment");
    // F = 20 N at L = 0.22 m: M = 4.4 N m; sigma = M (D/2) / I = 4.4 * 0.008 / 1.815006e-9 = 1.93938746e7 Pa.
    Check(ArmBendingStressPa(20.0, 0.22, 0.016, 0.013, &S) && Near(S, 19393874.55293694, 1e-3), "arm bending stress 19.39 MPa");
    // Doubling the force doubles the stress; doubling the lever doubles it too.
    double S2 = 0.0, S3 = 0.0;
    Check(ArmBendingStressPa(40.0, 0.22, 0.016, 0.013, &S2) && Near(S2, 2.0 * S, 1e-3), "stress ~ force");
    Check(ArmBendingStressPa(20.0, 0.44, 0.016, 0.013, &S3) && Near(S3, 2.0 * S, 1e-3), "stress ~ lever");
    // A thicker wall (2.0 mm -> d = 0.012) must lower the stress: I = pi (6.5536e-8 - 2.0736e-8) / 64 = 2.19911e-9 -> 1.6007e7 Pa.
    double Thick = 0.0;
    Check(ArmBendingStressPa(20.0, 0.22, 0.016, 0.012, &Thick) && Near(Thick, 4.4 * 0.008 / (kPi * (6.5536e-8 - 2.0736e-8) / 64.0), 1.0) && Thick < S, "thicker wall lowers stress");
    double Keep = 3.0;
    Check(!TubeSecondMomentM4(0.016, 0.016, &Keep) && !TubeSecondMomentM4(0.016, -0.001, &Keep) && !TubeSecondMomentM4(0.0, 0.0, &Keep) && Keep == 3.0, "bad tube rejected");
    Check(!ArmBendingStressPa(-1.0, 0.22, 0.016, 0.013, &Keep) && !ArmBendingStressPa(1.0, 0.0, 0.016, 0.013, &Keep) && Keep == 3.0, "bad arm load rejected");
}

static void TestMargin()
{
    double M = 0.0;
    // margin = limit / peak: 23 A limit, 11.5 A peak -> 2.0; at the limit -> 1.0 exactly; above -> < 1.
    Check(Margin(23.0, 11.5, &M) && M == 2.0, "margin 2");
    Check(Margin(23.0, 23.0, &M) && M == 1.0, "margin exactly 1 at the limit");
    Check(Margin(23.0, 25.3, &M) && Near(M, 23.0 / 25.3, 1e-15) && M < 1.0, "margin below 1 over the limit");
    // No load: capped at 99 (not infinity). A tiny peak is capped too.
    Check(Margin(23.0, 0.0, &M) && M == kNoLoadMargin, "zero peak gives the cap");
    Check(Margin(23.0, -1.0, &M) && M == kNoLoadMargin, "negative peak gives the cap");
    Check(Margin(23.0, 1e-9, &M) && M == kNoLoadMargin, "tiny peak is capped");
    double Keep = 7.0;
    Check(!Margin(0.0, 1.0, &Keep) && !Margin(-1.0, 1.0, &Keep) && !Margin(std::nan(""), 1.0, &Keep) && !Margin(1.0, HUGE_VAL, &Keep) && Keep == 7.0, "bad margin input rejected");
    // Verdict thresholds: >= 1.5 within, [1, 1.5) marginal, < 1 not recommended, NaN not recommended.
    Check(VerdictFromMargin(1.5) == Verdict::WithinLimits && VerdictFromMargin(99.0) == Verdict::WithinLimits, "verdict within at 1.5");
    Check(VerdictFromMargin(1.4999) == Verdict::Marginal && VerdictFromMargin(1.0) == Verdict::Marginal, "verdict marginal 1.0 to 1.5");
    Check(VerdictFromMargin(0.9999) == Verdict::NotRecommended && VerdictFromMargin(std::nan("")) == Verdict::NotRecommended, "verdict not recommended");
    Check(std::string(VerdictText(Verdict::Marginal)) == "marginal", "verdict text");
}

int main()
{
    TestPower();
    TestMotor();
    TestPack();
    TestArm();
    TestMargin();
    if (g_failures == 0)
    {
        std::printf("electrical_test: all passed\n");
        return 0;
    }
    std::printf("electrical_test: %d failure(s)\n", g_failures);
    return 1;
}
