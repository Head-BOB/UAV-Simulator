// Tier: V
// Engine-free parts catalogue model for the Parts screen: loads parts_db/parts_catalogue.json (written by
// tools/parts/export_catalogue.py), searches and filters it, keeps the build selection and the running totals. Nothing is
// computed about performance here: thrust-to-weight and hover current come from the builder (DroneBuildFiles.h).
#pragma once

#include <cctype>
#include <cmath>
#include <cstdio>
#include <limits>
#include <map>
#include <string>
#include <vector>

#include "DroneJson.h"

namespace DroneApp
{
// Provenance and status chips (one chip style, grey). Source types come from the database; the others from verification state.
inline std::string ProvenanceChip(const std::string& SourceType)
{
    if (SourceType == "CITED") return "Cited";
    if (SourceType == "ASSUMED") return "Assumed";
    if (SourceType == "CALCULATED") return "Calculated";
    if (SourceType == "MEASURED") return "Measured";
    return "Unverified";   // an unknown source type is never shown as trustworthy
}

struct KeyField
{
    std::string Name;
    std::string Unit;
    std::string SourceType;
    std::string ValueText;   // "0.468", "4", "0.14 x 0.05 x 0.02"
    double Number = std::numeric_limits<double>::quiet_NaN();   // for scalars; NaN for vectors
};

struct Part
{
    std::string Id, Category, Manufacturer, Model, FrameClass, Verification, Notes, PriceSourceType;
    double PriceUsd = std::numeric_limits<double>::quiet_NaN();   // NaN = unknown
    std::vector<KeyField> Fields;

    const KeyField* Field(const std::string& Name) const
    {
        for (const auto& F : Fields)
            if (F.Name == Name) return &F;
        return nullptr;
    }
    bool PriceKnown() const { return !std::isnan(PriceUsd); }
    double MassKg() const { const KeyField* F = Field("mass_kg"); return F ? F->Number : std::numeric_limits<double>::quiet_NaN(); }
    std::string Title() const { return Manufacturer.empty() ? Model : Manufacturer + " " + Model; }
};

namespace CatalogueDetail
{
inline std::string Num(double V)
{
    char Buf[48];
    if (std::fabs(V) < 1e15 && V == std::floor(V)) std::snprintf(Buf, sizeof Buf, "%.0f", V);
    else std::snprintf(Buf, sizeof Buf, "%.4g", V);
    return Buf;
}
inline std::string Lower(std::string S)
{
    for (char& C : S) C = static_cast<char>(std::tolower(static_cast<unsigned char>(C)));
    return S;
}
}  // namespace CatalogueDetail

constexpr int kMotorCount = 4;   // both vehicle classes are quads (same rule as tools/report/upgrades.py)

class Catalogue
{
public:
    bool Load(const std::string& Text, std::string& Err)
    {
        Parts_.clear();
        Categories_.clear();
        const Json::ParseResult R = Json::Parse(Text);
        if (!R.ok) { Err = R.error; return false; }
        const Json::Value* Sv = R.value.Get("schema_version");
        if (!Sv || Sv->AsNumber(-1) != 1) { Err = "catalogue schema_version is not 1"; return false; }
        const Json::Value* List = R.value.Get("parts");
        if (!List || !List->IsArray()) { Err = "catalogue has no parts list"; return false; }
        for (std::size_t I = 0; I < List->Size(); ++I)
        {
            const Json::Value* J = List->At(I);
            Part P;
            auto Str = [&](const char* K) { const Json::Value* V = J->Get(K); return V ? V->AsString("") : std::string(); };
            P.Id = Str("id"); P.Category = Str("category"); P.Manufacturer = Str("manufacturer"); P.Model = Str("model");
            P.FrameClass = Str("frame_class"); P.Verification = Str("verification"); P.Notes = Str("notes"); P.PriceSourceType = Str("price_source_type");
            if (P.Id.empty() || P.Category.empty()) { Err = "a catalogue part has no id or category"; return false; }
            const Json::Value* Pr = J->Get("price_usd");
            if (Pr && Pr->IsNumber()) P.PriceUsd = Pr->number;
            const Json::Value* KF = J->Get("key_fields");
            for (std::size_t K = 0; KF && K < KF->Size(); ++K)
            {
                const Json::Value* F = KF->At(K);
                KeyField Field;
                Field.Name = F->Get("name") ? F->Get("name")->AsString("") : std::string();
                Field.Unit = F->Get("unit") ? F->Get("unit")->AsString("") : std::string();
                Field.SourceType = F->Get("source_type") ? F->Get("source_type")->AsString("") : std::string();
                const Json::Value* V = F->Get("value");
                if (V && V->IsNumber()) { Field.Number = V->number; Field.ValueText = CatalogueDetail::Num(V->number); }
                else if (V && V->IsArray())
                {
                    for (std::size_t Q = 0; Q < V->Size(); ++Q)
                        Field.ValueText += (Q ? " x " : "") + CatalogueDetail::Num(V->At(Q)->AsNumber(0.0));
                }
                else Field.ValueText = "n/a";
                P.Fields.push_back(Field);
            }
            Parts_.push_back(std::move(P));
        }
        for (const Part& P : Parts_)
        {
            bool Seen = false;
            for (const auto& C : Categories_) Seen = Seen || C == P.Category;
            if (!Seen) Categories_.push_back(P.Category);
        }
        return true;
    }

    const std::vector<Part>& Parts() const { return Parts_; }
    const std::vector<std::string>& Categories() const { return Categories_; }
    const Part* Find(const std::string& Id) const
    {
        for (const auto& P : Parts_)
            if (P.Id == Id) return &P;
        return nullptr;
    }
    std::vector<std::string> Manufacturers(const std::string& Category) const
    {
        std::vector<std::string> Out;
        for (const auto& P : Parts_)
        {
            if (P.Category != Category) continue;
            bool Seen = false;
            for (const auto& M : Out) Seen = Seen || M == P.Manufacturer;
            if (!Seen) Out.push_back(P.Manufacturer);
        }
        return Out;
    }

    // Parts of one category (empty = all), every word of Search found (case-insensitive) in id / manufacturer / model, optional manufacturer,
    // optional frame class (applies to frames only, so a quad frame is never offered for the FPV class).
    std::vector<const Part*> Filter(const std::string& Category, const std::string& Search, const std::string& Manufacturer, const std::string& FrameClass) const
    {
        std::vector<std::string> Words;
        {
            std::string W;
            for (const char C : CatalogueDetail::Lower(Search))
            {
                if (C == ' ') { if (!W.empty()) Words.push_back(W); W.clear(); }
                else W += C;
            }
            if (!W.empty()) Words.push_back(W);
        }
        std::vector<const Part*> Out;
        for (const auto& P : Parts_)
        {
            if (!Category.empty() && P.Category != Category) continue;
            if (!Manufacturer.empty() && P.Manufacturer != Manufacturer) continue;
            if (!FrameClass.empty() && P.Category == "frame" && P.FrameClass != FrameClass) continue;
            const std::string Hay = CatalogueDetail::Lower(P.Id + " " + P.Manufacturer + " " + P.Model);
            bool All = true;
            for (const auto& W : Words) All = All && Hay.find(W) != std::string::npos;
            if (All) Out.push_back(&P);
        }
        return Out;
    }

private:
    std::vector<Part> Parts_;
    std::vector<std::string> Categories_;
};

// Roles the builder requires; every other role is optional.
inline const std::vector<std::string>& RequiredRoles()
{
    static const std::vector<std::string> R = {"frame", "motor", "esc", "propeller", "battery"};
    return R;
}

struct Totals
{
    double MassKg = 0.0;
    bool MassKnown = true;     // false when a selected part has no mass value
    double PriceUsd = 0.0;
    bool PriceKnown = true;    // false when any selected part has no price
    int PartCount = 0;
};

class Selection
{
public:
    void Set(const std::string& Role, const std::string& PartId) { Roles_[Role] = PartId; }
    void Clear(const std::string& Role) { Roles_.erase(Role); }
    void ClearAll() { Roles_.clear(); }
    const std::map<std::string, std::string>& Roles() const { return Roles_; }
    std::string Get(const std::string& Role) const { auto It = Roles_.find(Role); return It == Roles_.end() ? std::string() : It->second; }
    bool Complete() const
    {
        for (const auto& R : RequiredRoles())
            if (Roles_.find(R) == Roles_.end()) return false;
        return true;
    }
    std::vector<std::string> MissingRoles() const
    {
        std::vector<std::string> Out;
        for (const auto& R : RequiredRoles())
            if (Roles_.find(R) == Roles_.end()) Out.push_back(R);
        return Out;
    }

    // How many of a part the vehicle carries: motors and propellers x4; an ESC x4 if it has one channel, x1 if four; others x1.
    static int CountOf(const Part& P)
    {
        if (P.Category == "motor" || P.Category == "propeller") return kMotorCount;
        if (P.Category == "esc")
        {
            const KeyField* Ch = P.Field("channels");
            return (Ch && Ch->Number == 1.0) ? kMotorCount : 1;
        }
        return 1;
    }

    // Sum of part masses x counts, BEFORE arms, cables and layout (the builder adds those): a lower bound shown as such.
    Totals Sum(const Catalogue& C) const
    {
        Totals T;
        for (const auto& R : Roles_)
        {
            const Part* P = C.Find(R.second);
            if (P == nullptr) { T.MassKnown = false; T.PriceKnown = false; continue; }
            ++T.PartCount;
            const int N = CountOf(*P);
            if (std::isnan(P->MassKg())) T.MassKnown = false; else T.MassKg += P->MassKg() * N;
            if (!P->PriceKnown()) T.PriceKnown = false; else T.PriceUsd += P->PriceUsd * N;
        }
        return T;
    }

private:
    std::map<std::string, std::string> Roles_;
};
}  // namespace DroneApp
