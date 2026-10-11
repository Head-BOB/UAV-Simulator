// Tier: V
// Tests for DroneJson.h. Plain C++17, no Unreal.
//   g++ -std=c++17 -Wall -Wextra -Werror -I UE5Project/Plugins/DronePhysicsBridge/Source/DronePhysicsBridge/Public tools/cpp_ffi_check/json_test.cpp -o json_test
#include "DroneJson.h"

#include <cmath>
#include <cstdio>
#include <string>

using namespace DroneApp::Json;

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
static bool Bad(const std::string& s) { return !Parse(s).ok; }

int main()
{
    // literals and numbers: values written by hand
    Check(Parse("null").ok && Parse("null").value.IsNull(), "null");
    Check(Parse("true").value.AsBool(false) && !Parse("false").value.AsBool(true), "bools");
    Check(Near(Parse("-12.5e2").value.number, -1250.0, 0.0), "number -12.5e2 = -1250");
    Check(Near(Parse("0.000123").value.number, 0.000123, 1e-18), "small number");
    Check(Near(Parse("1E+3").value.number, 1000.0, 0.0), "exponent plus");
    Check(Parse("0").value.number == 0.0, "zero");
    // strings and escapes: \u00e9 is two UTF-8 bytes C3 A9; the surrogate pair D83D DE00 is U+1F600 = F0 9F 98 80
    Check(Parse("\"a\\n\\t\\\"b\\\\\"").value.text == "a\n\t\"b\\", "simple escapes");
    Check(Parse("\"\\u00e9\"").value.text == "\xC3\xA9", "u00e9");
    Check(Parse("\"\\ud83d\\ude00\"").value.text == "\xF0\x9F\x98\x80", "surrogate pair");
    Check(Parse("\"\\u0041\"").value.text == "A", "u0041");
    // structure
    const ParseResult R = Parse(" { \"a\" : [1, 2.5, {\"b\": null}], \"s\": \"x\", \"o\": {} } ");
    Check(R.ok, "object parses");
    Check(R.value.Get("a") != nullptr && R.value.Get("a")->Size() == 3, "array size");
    Check(Near(R.value.Get("a")->At(1)->number, 2.5, 0.0), "array element");
    Check(R.value.Get("a")->At(2)->Get("b")->IsNull(), "nested null");
    Check(R.value.Get("o") != nullptr && R.value.Get("o")->Size() == 0, "empty object");
    Check(R.value.Get("missing") == nullptr && R.value.Get("a")->At(9) == nullptr, "missing member / index is null");
    Check(R.value.Get("s")->AsString("d") == "x" && R.value.Get("a")->AsString("d") == "d", "AsString with default");
    Check(Near(R.value.Get("s")->AsNumber(7.0), 7.0, 0.0), "AsNumber default for a string");
    Check(Path(R.value, {"o"}) != nullptr && Path(R.value, {"o", "z"}) == nullptr && Path(R.value, {"s", "z"}) == nullptr, "Path");
    Check(R.value.members[0].first == "a" && R.value.members[2].first == "o", "file order kept");
    // refusals
    const char* bad[] = {"", "{", "[1,]", "{\"a\":1,}", "{\"a\" 1}", "{a:1}", "[1 2]", "01", "1.", ".5", "-", "1e", "+1", "NaN", "Infinity", "-Infinity",
                         "tru", "nul", "\"abc", "\"a\\x\"", "\"\\u12\"", "\"\\ud83d\"", "\"\\ude00\"", "[1] x", "{\"a\":1,\"a\":2}", "'a'", "// c\n1", "1e999", "\"a\nb\""};
    for (const char* b : bad) Check(Bad(b), (std::string("refuses: ") + b).c_str());
    // nesting limit: 64 levels are fine, 70 are refused
    std::string deep_ok(64, '['), deep_bad(70, '[');
    deep_ok += std::string(64, ']');
    deep_bad += std::string(70, ']');
    Check(Parse(deep_ok).ok, "64 levels allowed");
    Check(!Parse(deep_bad).ok, "70 levels refused");
    // error position: the bad character is on line 2, column 3 ("  x")
    const ParseResult E = Parse("[\n  x]");
    Check(!E.ok && E.error.rfind("2:3", 0) == 0, "error position 2:3");
    // Quote
    Check(Quote("a\"b\\c\n") == "\"a\\\"b\\\\c\\n\"", "Quote escapes");
    Check(Quote(std::string("\x01", 1)) == "\"\\u0001\"", "Quote control char");
    // quoted text parses back to itself
    const std::string odd = "x\ty\"z\\\x1f";
    Check(Parse(Quote(odd)).value.text == odd, "Quote round trip");
    // NumberText: whole numbers without a point, shortest text that reads back exactly, negative zero as 0
    Check(NumberText(8.0) == "8" && NumberText(-3.0) == "-3" && NumberText(-0.0) == "0" && NumberText(0.5) == "0.5" && NumberText(0.1) == "0.1", "NumberText simple");
    for (const double v : {1.0 / 3.0, 2.0 / 3.0, 1e-7, 123456.789012345678, 6.02214076e23, 0.1 + 0.2})
        Check(std::strtod(NumberText(v).c_str(), nullptr) == v, "NumberText round trip");
    // Serialize: exact text for a small document, and parse(serialize(x)) equals x
    const Value Doc = Parse("{\"a\":[1,2.5,{\"b\":null}],\"s\":\"x\\ny\",\"e\":[],\"o\":{},\"t\":true}").value;
    const std::string Want = "{\n  \"a\": [\n    1,\n    2.5,\n    {\n      \"b\": null\n    }\n  ],\n  \"s\": \"x\\ny\",\n  \"e\": [],\n  \"o\": {},\n  \"t\": true\n}\n";
    Check(Serialize(Doc) == Want, "Serialize exact text");
    Check(Serialize(Parse(Serialize(Doc)).value) == Serialize(Doc), "Serialize is stable under re-parse");
    if (g_failures == 0) std::printf("json_test: all checks passed\n");
    return g_failures == 0 ? 0 : 1;
}
