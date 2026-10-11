// Tier: V
// Engine-free, std-only JSON reader for the files the app reads (reports, campaign folders, build sidecars, catalogue).
// Strict: no comments, no trailing commas, no NaN / Infinity, no duplicate keys, nesting and size limits, never throws.
// Tested with plain g++ by tools/cpp_ffi_check/json_test.cpp. Numbers are doubles (the project writes f64).
#pragma once

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <initializer_list>
#include <string>
#include <utility>
#include <vector>

namespace DroneApp
{
namespace Json
{
constexpr std::size_t kMaxDepth = 64;                 // nesting limit (reports are about 8 deep)
constexpr std::size_t kMaxBytes = 128u * 1024u * 1024u;  // refuse absurd inputs before parsing

struct Value
{
    enum class Type { Null, Bool, Number, String, Array, Object };
    Type type = Type::Null;
    bool boolean = false;
    double number = 0.0;
    std::string text;
    std::vector<Value> items;                              // Array
    std::vector<std::pair<std::string, Value>> members;    // Object, in file order

    bool IsNull() const { return type == Type::Null; }
    bool IsNumber() const { return type == Type::Number; }
    bool IsString() const { return type == Type::String; }
    bool IsArray() const { return type == Type::Array; }
    bool IsObject() const { return type == Type::Object; }

    // Member of an object, or nullptr (also nullptr when this is not an object).
    const Value* Get(const std::string& Key) const
    {
        if (type != Type::Object) return nullptr;
        for (const auto& M : members)
            if (M.first == Key) return &M.second;
        return nullptr;
    }
    // Element of an array, or nullptr.
    const Value* At(std::size_t Index) const { return (type == Type::Array && Index < items.size()) ? &items[Index] : nullptr; }
    double AsNumber(double Default) const { return type == Type::Number ? number : Default; }
    const std::string& AsString(const std::string& Default) const { return type == Type::String ? text : Default; }
    bool AsBool(bool Default) const { return type == Type::Bool ? boolean : Default; }
    std::size_t Size() const { return type == Type::Array ? items.size() : (type == Type::Object ? members.size() : 0); }
};

struct ParseResult
{
    bool ok = false;
    std::string error;   // "line:col message" when !ok
    Value value;
};

namespace Detail
{
class Parser
{
public:
    explicit Parser(const std::string& Src) : S(Src) {}

    ParseResult Run()
    {
        ParseResult R;
        if (S.size() > kMaxBytes) return Fail(R, "input too large");
        SkipWs();
        if (!ParseValue(R.value, 0)) return Fail(R, Err);
        SkipWs();
        if (P != S.size()) return Fail(R, "trailing characters after the value");
        R.ok = true;
        return R;
    }

private:
    const std::string& S;
    std::size_t P = 0;
    std::string Err;

    ParseResult Fail(ParseResult& R, const std::string& Msg)
    {
        std::size_t Line = 1, Col = 1;
        for (std::size_t I = 0; I < P && I < S.size(); ++I)
        {
            if (S[I] == '\n') { ++Line; Col = 1; }
            else ++Col;
        }
        R.ok = false;
        R.value = Value();
        R.error = std::to_string(Line) + ":" + std::to_string(Col) + " " + Msg;
        return R;
    }
    bool Bad(const char* Msg) { Err = Msg; return false; }
    void SkipWs()
    {
        while (P < S.size() && (S[P] == ' ' || S[P] == '\t' || S[P] == '\n' || S[P] == '\r')) ++P;
    }
    bool Lit(const char* Word)
    {
        std::size_t I = 0;
        while (Word[I] != '\0')
        {
            if (P + I >= S.size() || S[P + I] != Word[I]) return false;
            ++I;
        }
        P += I;
        return true;
    }
    bool ParseValue(Value& Out, std::size_t Depth)
    {
        if (Depth > kMaxDepth) return Bad("nesting too deep");
        if (P >= S.size()) return Bad("unexpected end of input");
        const char C = S[P];
        if (C == '{') return ParseObject(Out, Depth);
        if (C == '[') return ParseArray(Out, Depth);
        if (C == '"') { Out.type = Value::Type::String; return ParseString(Out.text); }
        if (C == 't') { if (!Lit("true")) return Bad("bad literal"); Out.type = Value::Type::Bool; Out.boolean = true; return true; }
        if (C == 'f') { if (!Lit("false")) return Bad("bad literal"); Out.type = Value::Type::Bool; Out.boolean = false; return true; }
        if (C == 'n') { if (!Lit("null")) return Bad("bad literal"); Out.type = Value::Type::Null; return true; }
        if (C == '-' || (C >= '0' && C <= '9')) return ParseNumber(Out);
        return Bad("unexpected character");
    }
    bool ParseNumber(Value& Out)
    {
        const std::size_t Start = P;
        if (S[P] == '-') ++P;
        if (P >= S.size()) return Bad("bad number");
        if (S[P] == '0') ++P;
        else if (S[P] >= '1' && S[P] <= '9') { while (P < S.size() && S[P] >= '0' && S[P] <= '9') ++P; }
        else return Bad("bad number");
        if (P < S.size() && S[P] == '.')
        {
            ++P;
            const std::size_t D = P;
            while (P < S.size() && S[P] >= '0' && S[P] <= '9') ++P;
            if (P == D) return Bad("bad number: digits needed after the point");
        }
        if (P < S.size() && (S[P] == 'e' || S[P] == 'E'))
        {
            ++P;
            if (P < S.size() && (S[P] == '+' || S[P] == '-')) ++P;
            const std::size_t D = P;
            while (P < S.size() && S[P] >= '0' && S[P] <= '9') ++P;
            if (P == D) return Bad("bad number: digits needed in the exponent");
        }
        const std::string Tok = S.substr(Start, P - Start);
        const double V = std::strtod(Tok.c_str(), nullptr);
        if (!std::isfinite(V)) return Bad("number out of range");
        Out.type = Value::Type::Number;
        Out.number = V;
        return true;
    }
    static void AppendUtf8(std::string& Out, std::uint32_t Cp)
    {
        if (Cp < 0x80) Out += static_cast<char>(Cp);
        else if (Cp < 0x800) { Out += static_cast<char>(0xC0 | (Cp >> 6)); Out += static_cast<char>(0x80 | (Cp & 0x3F)); }
        else if (Cp < 0x10000) { Out += static_cast<char>(0xE0 | (Cp >> 12)); Out += static_cast<char>(0x80 | ((Cp >> 6) & 0x3F)); Out += static_cast<char>(0x80 | (Cp & 0x3F)); }
        else { Out += static_cast<char>(0xF0 | (Cp >> 18)); Out += static_cast<char>(0x80 | ((Cp >> 12) & 0x3F)); Out += static_cast<char>(0x80 | ((Cp >> 6) & 0x3F)); Out += static_cast<char>(0x80 | (Cp & 0x3F)); }
    }
    bool Hex4(std::uint32_t& Out)
    {
        if (P + 4 > S.size()) return false;
        std::uint32_t V = 0;
        for (int I = 0; I < 4; ++I)
        {
            const char C = S[P + static_cast<std::size_t>(I)];
            V <<= 4;
            if (C >= '0' && C <= '9') V |= static_cast<std::uint32_t>(C - '0');
            else if (C >= 'a' && C <= 'f') V |= static_cast<std::uint32_t>(C - 'a' + 10);
            else if (C >= 'A' && C <= 'F') V |= static_cast<std::uint32_t>(C - 'A' + 10);
            else return false;
        }
        P += 4;
        Out = V;
        return true;
    }
    bool ParseString(std::string& Out)
    {
        ++P;  // opening quote
        Out.clear();
        while (true)
        {
            if (P >= S.size()) return Bad("unterminated string");
            const unsigned char C = static_cast<unsigned char>(S[P++]);
            if (C == '"') return true;
            if (C < 0x20) return Bad("control character in string");
            if (C != '\\') { Out += static_cast<char>(C); continue; }
            if (P >= S.size()) return Bad("unterminated escape");
            const char E = S[P++];
            switch (E)
            {
                case '"': Out += '"'; break;
                case '\\': Out += '\\'; break;
                case '/': Out += '/'; break;
                case 'b': Out += '\b'; break;
                case 'f': Out += '\f'; break;
                case 'n': Out += '\n'; break;
                case 'r': Out += '\r'; break;
                case 't': Out += '\t'; break;
                case 'u':
                {
                    std::uint32_t Cp = 0;
                    if (!Hex4(Cp)) return Bad("bad \\u escape");
                    if (Cp >= 0xD800 && Cp <= 0xDBFF)
                    {
                        std::uint32_t Lo = 0;
                        if (P + 2 > S.size() || S[P] != '\\' || S[P + 1] != 'u') return Bad("lone surrogate");
                        P += 2;
                        if (!Hex4(Lo) || Lo < 0xDC00 || Lo > 0xDFFF) return Bad("bad surrogate pair");
                        Cp = 0x10000 + ((Cp - 0xD800) << 10) + (Lo - 0xDC00);
                    }
                    else if (Cp >= 0xDC00 && Cp <= 0xDFFF) return Bad("lone surrogate");
                    AppendUtf8(Out, Cp);
                    break;
                }
                default: return Bad("bad escape");
            }
        }
    }
    bool ParseArray(Value& Out, std::size_t Depth)
    {
        ++P;
        Out.type = Value::Type::Array;
        SkipWs();
        if (P < S.size() && S[P] == ']') { ++P; return true; }
        while (true)
        {
            Out.items.emplace_back();
            SkipWs();
            if (!ParseValue(Out.items.back(), Depth + 1)) return false;
            SkipWs();
            if (P >= S.size()) return Bad("unterminated array");
            if (S[P] == ',') { ++P; continue; }
            if (S[P] == ']') { ++P; return true; }
            return Bad("expected , or ] in array");
        }
    }
    bool ParseObject(Value& Out, std::size_t Depth)
    {
        ++P;
        Out.type = Value::Type::Object;
        SkipWs();
        if (P < S.size() && S[P] == '}') { ++P; return true; }
        while (true)
        {
            SkipWs();
            if (P >= S.size() || S[P] != '"') return Bad("expected a string key");
            std::string Key;
            if (!ParseString(Key)) return false;
            for (const auto& M : Out.members)
                if (M.first == Key) return Bad("duplicate key");
            SkipWs();
            if (P >= S.size() || S[P] != ':') return Bad("expected : after key");
            ++P;
            SkipWs();
            Out.members.emplace_back(std::move(Key), Value());
            if (!ParseValue(Out.members.back().second, Depth + 1)) return false;
            SkipWs();
            if (P >= S.size()) return Bad("unterminated object");
            if (S[P] == ',') { ++P; continue; }
            if (S[P] == '}') { ++P; return true; }
            return Bad("expected , or } in object");
        }
    }
};
}  // namespace Detail

inline ParseResult Parse(const std::string& Text)
{
    return Detail::Parser(Text).Run();
}

// Follows object keys; nullptr as soon as one is missing.
inline const Value* Path(const Value& Root, std::initializer_list<const char*> Keys)
{
    const Value* V = &Root;
    for (const char* K : Keys)
    {
        V = V->Get(K);
        if (V == nullptr) return nullptr;
    }
    return V;
}

// Escapes a string for output as a JSON string literal (including the quotes). Control characters become \u00XX.
inline std::string Quote(const std::string& S)
{
    std::string Out = "\"";
    for (const unsigned char C : S)
    {
        switch (C)
        {
            case '"': Out += "\\\""; break;
            case '\\': Out += "\\\\"; break;
            case '\n': Out += "\\n"; break;
            case '\r': Out += "\\r"; break;
            case '\t': Out += "\\t"; break;
            default:
                if (C < 0x20)
                {
                    static const char* Hex = "0123456789abcdef";
                    Out += "\\u00";
                    Out += Hex[(C >> 4) & 0xF];
                    Out += Hex[C & 0xF];
                }
                else Out += static_cast<char>(C);
        }
    }
    return Out + "\"";
}

// Shortest decimal text that reads back to exactly the same double (tries 15, 16, then 17 significant digits). Finite numbers only;
// a whole number below 1e15 prints without a decimal point.
inline std::string NumberText(double V)
{
    char Buf[40];
    if (std::fabs(V) < 1e15 && V == std::floor(V))
    {
        std::snprintf(Buf, sizeof Buf, "%.0f", V);
        return std::string(Buf) == "-0" ? std::string("0") : std::string(Buf);
    }
    for (int Digits = 15; Digits <= 17; ++Digits)
    {
        std::snprintf(Buf, sizeof Buf, "%.*g", Digits, V);
        if (std::strtod(Buf, nullptr) == V) break;
    }
    return Buf;
}

// Pretty-prints a value (2-space indent, members in stored order, trailing newline at the top level). A non-finite number is written as null
// (it cannot occur in a parsed document).
inline void SerializeInto(const Value& V, std::string& Out, int Depth)
{
    const std::string Pad(static_cast<std::size_t>(Depth + 1) * 2, ' '), PadEnd(static_cast<std::size_t>(Depth) * 2, ' ');
    switch (V.type)
    {
        case Value::Type::Null: Out += "null"; break;
        case Value::Type::Bool: Out += V.boolean ? "true" : "false"; break;
        case Value::Type::Number: Out += std::isfinite(V.number) ? NumberText(V.number) : std::string("null"); break;
        case Value::Type::String: Out += Quote(V.text); break;
        case Value::Type::Array:
            if (V.items.empty()) { Out += "[]"; break; }
            Out += "[\n";
            for (std::size_t I = 0; I < V.items.size(); ++I)
            {
                Out += Pad;
                SerializeInto(V.items[I], Out, Depth + 1);
                Out += I + 1 < V.items.size() ? ",\n" : "\n";
            }
            Out += PadEnd + "]";
            break;
        case Value::Type::Object:
            if (V.members.empty()) { Out += "{}"; break; }
            Out += "{\n";
            for (std::size_t I = 0; I < V.members.size(); ++I)
            {
                Out += Pad + Quote(V.members[I].first) + ": ";
                SerializeInto(V.members[I].second, Out, Depth + 1);
                Out += I + 1 < V.members.size() ? ",\n" : "\n";
            }
            Out += PadEnd + "}";
            break;
    }
}
inline std::string Serialize(const Value& V)
{
    std::string Out;
    SerializeInto(V, Out, 0);
    return Out + "\n";
}
}  // namespace Json
}  // namespace DroneApp
