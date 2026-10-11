// Tier: V
// Engine-free layout of the setup screens (navigation bar, Vehicle, Parts, Build, Conditions) as display lists, in the 1920 x 1080
// design space of the approved preview v4: dark left column over the scene, large light list rows separated by 1 px lines, details
// for the selected row only, one filled primary button, chips with 1 px outlines and no fill, tables not cards.
#pragma once

#include <cstdio>
#include <string>

#include "DroneAppModel.h"

namespace DroneApp
{
namespace Screens
{
constexpr float W = static_cast<float>(Ui::kDesignWidthPx);
constexpr float Hgt = static_cast<float>(Ui::kDesignHeightPx);

inline Rgba StatusRgba(Status S) { return S == Status::Critical ? Palette::Crit() : (S == Status::Caution ? Palette::Warn() : Palette::Ok()); }

inline std::string Fmt(const char* Format, double V)
{
    char Buf[64];
    std::snprintf(Buf, sizeof Buf, Format, V);
    return Buf;
}

// "Part title" with at most MaxChars characters (the painter clips too; this keeps list rows tidy).
inline std::string Shorten(const std::string& S, std::size_t MaxChars) { return S.size() <= MaxChars ? S : S.substr(0, MaxChars - 1) + "~"; }

// Word wrap into lines of at most MaxChars characters (words longer than that are cut). At most MaxLines lines; the last one ends in "~" if text was dropped.
inline std::vector<std::string> Wrap(const std::string& Text, std::size_t MaxChars, std::size_t MaxLines)
{
    std::vector<std::string> Lines;
    std::string Cur, Word;
    auto Flush = [&]() {
        if (Word.empty()) return;
        while (Word.size() > MaxChars) { if (!Cur.empty()) { Lines.push_back(Cur); Cur.clear(); } Lines.push_back(Word.substr(0, MaxChars)); Word = Word.substr(MaxChars); }
        if (!Cur.empty() && Cur.size() + 1 + Word.size() > MaxChars) { Lines.push_back(Cur); Cur.clear(); }
        Cur += (Cur.empty() ? "" : " ") + Word;
        Word.clear();
    };
    for (const char C : Text) { if (C == ' ') Flush(); else Word += C; }
    Flush();
    if (!Cur.empty()) Lines.push_back(Cur);
    if (Lines.size() > MaxLines) { Lines.resize(MaxLines); Lines.back() = Shorten(Lines.back() + "~~", MaxChars); }
    return Lines;
}

inline Rgba VerificationRgba(const std::string& V) { return V == "Verified" ? Palette::Ok() : Palette::Ink70(); }

// Step bar across the top of every screen except Run (which has its own top bar). Blocked steps are dim and show why on hover (not drawn).
inline void NavBar(DrawList& L, const AppModel& M)
{
    L.Fill(0, 0, W, static_cast<float>(Ui::kTopBarHeightPx), Palette::Bar());
    L.HLine(0, static_cast<float>(Ui::kTopBarHeightPx), W, Palette::Line());
    float X = static_cast<float>(Ui::kTopBarPaddingPx);
    for (int I = 0; I < kScreenCount; ++I)
    {
        const Screen S = static_cast<Screen>(I);
        const bool Open = CanOpen(S, M.Prog);
        const bool Here = S == M.Current;
        const std::string Label = std::to_string(I + 1) + "  " + ScreenName(S);
        const float Wd = EstimateTextWidth(Label, 17.0f) + 30.0f;
        L.Text(X, 15.0f, Wd, Label, 17.0f, Here ? Palette::Ink() : (Open ? Palette::Ink70() : Palette::Ink46()), Here ? Weight::Medium : Weight::Regular);
        if (Here) L.Fill(X, 52.0f, EstimateTextWidth(Label, 17.0f), 2.0f, Palette::Accent());
        if (Open && !Here) L.AddHit(X - 8.0f, 0.0f, Wd, static_cast<float>(Ui::kTopBarHeightPx), Act::Navigate, ScreenName(S));
        X += Wd + 18.0f;
    }
    if (M.Busy()) L.Text(W - 560.0f, 18.0f, 512.0f, M.BusyLabel() + " ...", 17.0f, Palette::Accent(), Weight::Regular, Align::Right);
}

// Status line under the bar (errors in the critical colour).
inline void StatusLine(DrawList& L, const AppModel& M, float X, float Y, float Wd)
{
    if (M.Message.empty() || M.Busy()) return;
    L.Text(X, Y, Wd, M.Message, 16.0f, M.MessageIsError ? Palette::Crit() : Palette::Ink70());
}

inline void Scrim(DrawList& L, float Wd = static_cast<float>(Ui::kSetupScrimWidthPx))
{
    L.Fill(0, static_cast<float>(Ui::kTopBarHeightPx), Wd, Hgt - static_cast<float>(Ui::kTopBarHeightPx), Palette::Bar());
}

// ----------------------------------------------------------------------------------------------------------------------------- 1 Vehicle
inline void Vehicle(DrawList& L, const AppModel& M)
{
    NavBar(L, M);
    Scrim(L);
    const float X = static_cast<float>(Ui::kSetupColumnLeftPx), Wd = static_cast<float>(Ui::kSetupColumnWidthPx);
    L.Text(X, 250.0f, Wd, "VEHICLE", 18.0f, Palette::Ink70(), Weight::Medium);
    L.Text(X, 282.0f, Wd, "Choose a vehicle", 56.0f, Palette::Ink(), Weight::Light);
    float Y = 396.0f;
    for (const auto& V : VehicleClasses())
    {
        const bool Sel = V.Available && M.VehicleClass == V.Id;
        L.HLine(X, Y, Wd, Palette::Line());
        L.Text(X, Y + 12.0f, Wd, V.Label, 30.0f, Sel ? Palette::Ink() : (V.Available ? Palette::Ink70() : Palette::Ink46()), Weight::Light);
        if (!V.Available) L.Text(X, Y + 22.0f, Wd, "Not available yet", 16.0f, Palette::Ink46(), Weight::Regular, Align::Right);
        else L.AddHit(X, Y, Wd, 60.0f, Act::PickVehicle, V.Id);
        Y += 60.0f;
    }
    L.HLine(X, Y, Wd, Palette::Line());
    Button(L, X, Y + 36.0f, 240.0f, 52.0f, "Continue", true, Act::NavNext, "", M.Prog.VehicleClassChosen);
    L.Text(X + 270.0f, Y + 40.0f, 300.0f, "Next: choose the parts.", 14.0f, Palette::Ink46());
    StatusLine(L, M, X, Y + 100.0f, Wd);
}

// ----------------------------------------------------------------------------------------------------------------------------- 2 Parts
inline void Parts(DrawList& L, const AppModel& M)
{
    NavBar(L, M);
    Scrim(L, 900.0f);
    L.Fill(900.0f, static_cast<float>(Ui::kTopBarHeightPx), W - 900.0f, Hgt - static_cast<float>(Ui::kTopBarHeightPx), Palette::Solid());
    const float X = 48.0f;
    // category tabs: the five roles the builder needs first, then the others; wraps onto a second row
    std::vector<std::string> Cats = RequiredRoles();
    for (const auto& C : M.Cat.Categories()) { bool Has = !IsVehiclePartCategory(C); for (const auto& R : Cats) Has = Has || R == C; if (!Has) Cats.push_back(C); }
    float Cx = X, Cy = 76.0f;
    for (const auto& C : Cats)
    {
        const bool Sel = C == M.CategoryFilter;
        const float Cw = EstimateTextWidth(C, 15.0f) + 18.0f;
        if (Cx + Cw > 860.0f) { Cx = X; Cy += 30.0f; }
        L.Text(Cx, Cy, Cw, C, 15.0f, Sel ? Palette::Ink() : Palette::Ink46(), Sel ? Weight::Medium : Weight::Regular);
        if (Sel) L.Fill(Cx, Cy + 22.0f, Cw - 14.0f, 2.0f, Palette::Accent());
        L.AddHit(Cx - 4.0f, Cy - 6.0f, Cw, 30.0f, Act::PickCategory, C);
        Cx += Cw + 6.0f;
    }
    // search + manufacturer filters
    const float Sy = Cy + 44.0f;
    L.Text(X, Sy + 6.0f, 80.0f, "Search", 14.0f, Palette::Ink46());
    Field(L, X + 70.0f, Sy, 300.0f, M.Search, 20.0f, M.Focus == "search", "search", Weight::Regular);
    float Mx = X + 400.0f;
    for (const auto& Mn : M.Cat.Manufacturers(M.CategoryFilter))
    {
        const bool Sel = Mn == M.ManufacturerFilter;
        const float Mw = EstimateTextWidth(Mn, 14.0f) + 12.0f;
        if (Mx + Mw > 860.0f) break;
        L.Text(Mx, Sy + 6.0f, Mw, Mn, 14.0f, Sel ? Palette::Accent() : Palette::Ink46(), Sel ? Weight::Medium : Weight::Regular);
        L.AddHit(Mx - 4.0f, Sy, Mw, 28.0f, Act::PickManufacturer, Mn);
        Mx += Mw + 8.0f;
    }
    // list (scrolls by whole rows)
    const std::vector<const Part*> Vis = M.VisibleParts();
    constexpr int kRows = 15;
    constexpr float kRowH = 48.0f;
    const int Start = std::min(M.ListScroll, std::max(0, static_cast<int>(Vis.size()) - kRows));
    float Y = Sy + 50.0f;
    for (int I = Start; I < static_cast<int>(Vis.size()) && I < Start + kRows; ++I)
    {
        const Part& P = *Vis[static_cast<std::size_t>(I)];
        const bool Sel = P.Id == M.SelectedPartId;
        const bool Chosen = M.Sel.Get(P.Category) == P.Id;
        L.HLine(X, Y, 800.0f, Palette::Line());
        L.Text(X, Y + 10.0f, 600.0f, Shorten(P.Title(), 44), 24.0f, Sel ? Palette::Ink() : Palette::Ink46(), Weight::Light);
        if (Chosen) L.Text(X + 640.0f, Y + 16.0f, 150.0f, "In build", 14.0f, Palette::Accent(), Weight::Medium, Align::Right);
        else L.Text(X + 640.0f, Y + 16.0f, 150.0f, P.PriceKnown() ? "$" + NumText(P.PriceUsd) : std::string("price n/a"), 14.0f, Palette::Ink46(), Weight::Regular, Align::Right);
        L.AddHit(X, Y, 800.0f, kRowH, Act::PickPart, P.Id);
        Y += kRowH;
    }
    L.HLine(X, Y, 800.0f, Palette::Line());
    if (Vis.empty()) L.Text(X, 180.0f, 700.0f, "No part matches. Clear the search or the manufacturer filter.", 18.0f, Palette::Ink70());
    L.Text(X, 1000.0f, 500.0f, std::to_string(Vis.size()) + " parts" + (Start > 0 || static_cast<int>(Vis.size()) > Start + kRows ? "   (mouse wheel scrolls)" : ""), 14.0f, Palette::Ink46());
    if (Start > 0) L.AddHit(X, 150.0f, 800.0f, 18.0f, Act::ScrollList, "-" + std::to_string(kRows));
    if (static_cast<int>(Vis.size()) > Start + kRows) L.AddHit(X, 984.0f, 800.0f, 40.0f, Act::ScrollList, std::to_string(kRows));

    // detail of the selected part (right)
    const float Dx = 960.0f, Dw = 912.0f;
    const Part* P = M.SelectedPart();
    if (P == nullptr)
    {
        L.Text(Dx, 140.0f, Dw, "Pick a part to see its details.", 20.0f, Palette::Ink70());
    }
    else
    {
        L.Text(Dx, 90.0f, Dw, P->Title(), 34.0f, Palette::Ink(), Weight::Light);
        float Cxp = Dx;
        Cxp += Chip(L, Cxp, 140.0f, P->Verification.empty() ? "Unverified" : P->Verification, VerificationRgba(P->Verification)) + 8.0f;
        Cxp += Chip(L, Cxp, 140.0f, P->Category, Palette::Ink70()) + 8.0f;
        if (!P->FrameClass.empty()) Chip(L, Cxp, 140.0f, P->FrameClass, Palette::Ink70());
        L.Text(Dx, 180.0f, Dw, "Price", 14.0f, Palette::Ink46());
        L.Text(Dx + 120.0f, 174.0f, 300.0f, P->PriceKnown() ? "$" + NumText(P->PriceUsd) : std::string("n/a"), 20.0f, Palette::Ink());
        if (P->PriceKnown()) Chip(L, Dx + 280.0f, 176.0f, ProvenanceChip(P->PriceSourceType), Palette::Ink70());
        float Ty = 262.0f;
        L.Text(Dx, Ty - 30.0f, 300.0f, "Key specifications", 15.0f, Palette::Ink70(), Weight::Medium);
        for (const auto& F : P->Fields)
        {
            L.HLine(Dx, Ty, Dw, Palette::Line());
            L.Text(Dx, Ty + 8.0f, 380.0f, F.Name, 16.0f, Palette::Ink70());
            L.Text(Dx + 380.0f, Ty + 6.0f, 300.0f, F.ValueText, 20.0f, Palette::Ink(), Weight::Regular);
            L.Text(Dx + 680.0f, Ty + 9.0f, 100.0f, F.Unit, 16.0f, Palette::Ink46());
            Chip(L, Dx + 790.0f, Ty + 6.0f, ProvenanceChip(F.SourceType), F.SourceType == "ASSUMED" ? Palette::Warn() : Palette::Ink70());
            Ty += 36.0f;
        }
        L.HLine(Dx, Ty, Dw, Palette::Line());
        if (!P->Notes.empty()) { float Ny = Ty + 12.0f; for (const auto& Ln : Wrap(P->Notes, 110, 2)) { L.Text(Dx, Ny, Dw, Ln, 14.0f, Palette::Ink46()); Ny += 20.0f; } }
        const bool Chosen = M.Sel.Get(P->Category) == P->Id;
        Button(L, Dx, 616.0f, 220.0f, 48.0f, Chosen ? "In build" : "Add to build", true, Act::AddPart, "", !Chosen);
        if (Chosen) Button(L, Dx + 240.0f, 616.0f, 200.0f, 48.0f, "Remove", false, Act::RemovePart, P->Category);
    }
    // build so far: one line per role, running totals
    const float By = 690.0f;
    L.HLine(Dx, By, Dw, Palette::LineStrong());
    L.Text(Dx, By + 10.0f, 400.0f, "Your build", 15.0f, Palette::Ink70(), Weight::Medium);
    float Ry = By + 40.0f;
    for (const auto& Role : RequiredRoles())
    {
        const Part* Ch = M.Cat.Find(M.Sel.Get(Role));
        L.Text(Dx, Ry, 140.0f, Role, 15.0f, Palette::Ink46());
        L.Text(Dx + 140.0f, Ry - 2.0f, 560.0f, Ch ? Shorten(Ch->Title(), 48) : std::string("not chosen"), 17.0f, Ch ? Palette::Ink() : Palette::Ink46());
        if (Ch) L.AddHit(Dx, Ry - 4.0f, Dw, 28.0f, Act::PickPart, Ch->Id);
        Ry += 28.0f;
    }
    const Totals T = M.PartTotals();
    L.Text(Dx, Ry + 8.0f, Dw, "Parts total " + (T.MassKnown ? Fmt("%.3f", T.MassKg) + " kg" : std::string("mass n/a")) + " (before arms and cables)    " +
                                  (T.PriceKnown ? "$" + NumText(T.PriceUsd) : std::string("price n/a")), 16.0f, Palette::Ink70());
    if (M.BuildAttempted && M.Prog.BuildDone)
        L.Text(Dx, Ry + 34.0f, Dw, "Last build: thrust-to-weight " + Fmt("%.2f", M.Summary.ThrustToWeight) + ", hover current " + Fmt("%.1f", M.Summary.HoverPackCurrentA) + " A (design estimates)", 16.0f, Palette::Ink70());
    else L.Text(Dx, Ry + 34.0f, Dw, "Thrust-to-weight, hover current and compatibility are worked out by the builder on the next screen.", 14.0f, Palette::Ink46());
    Button(L, Dx + 560.0f, 972.0f, 352.0f, 52.0f, "Continue to Build", true, Act::NavNext, "", M.Prog.PartsComplete);
    if (!M.Prog.PartsComplete) L.Text(Dx, 986.0f, 540.0f, "Missing: " + [&] { std::string S; for (const auto& R : M.Sel.MissingRoles()) S += (S.empty() ? "" : ", ") + R; return S; }(), 14.0f, Palette::Warn());
    StatusLine(L, M, Dx, 1040.0f, Dw);
}

// ----------------------------------------------------------------------------------------------------------------------------- 3 Build
inline void Build(DrawList& L, const AppModel& M)
{
    NavBar(L, M);
    const float Px = 0.0f, Pw = 480.0f;
    L.Fill(Px, static_cast<float>(Ui::kTopBarHeightPx), Pw, Hgt - static_cast<float>(Ui::kTopBarHeightPx), Palette::Bar());
    L.VLine(Pw, static_cast<float>(Ui::kTopBarHeightPx), Hgt - static_cast<float>(Ui::kTopBarHeightPx), Palette::Line());
    const float X = 40.0f, Wd = Pw - 80.0f;
    L.Text(X, 80.0f, Wd, "Build", 34.0f, Palette::Ink(), Weight::Light);
    L.Text(X, 134.0f, 120.0f, "Name", 14.0f, Palette::Ink46());
    Field(L, X + 70.0f, 126.0f, Wd - 70.0f, M.BuildName, 20.0f, M.Focus == "buildname", "buildname", Weight::Regular);
    Button(L, X, 176.0f, 200.0f, 48.0f, M.Busy() ? "Working ..." : (M.Prog.BuildDone ? "Rebuild" : "Build vehicle"), true, Act::BuildNow, "", !M.Busy() && M.Prog.PartsComplete);
    if (M.Prog.BuildDone) Button(L, X + 216.0f, 176.0f, 224.0f, 48.0f, "Conditions >", false, Act::UseBuild);
    Button(L, X, 232.0f, Wd, 36.0f, "Fly it yourself (live physics, standard model)", false, Act::FlyIt, "", M.Prog.VehicleClassChosen);
    float Y = 288.0f;
    if (M.Prog.BuildDone)
    {
        L.Text(X, Y, Wd, "Design estimates", 15.0f, Palette::Ink70(), Weight::Medium);
        Y += 30.0f;
        const std::pair<std::string, std::string> Rows[] = {
            {"Mass", Fmt("%.3f", M.Summary.MassKg) + " kg"}, {"CG below frame centre", Fmt("%.1f", M.Summary.Cg[2] * 1000.0) + " mm"},
            {"Thrust-to-weight", Fmt("%.2f", M.Summary.ThrustToWeight)}, {"Hover throttle", Fmt("%.1f", M.Summary.HoverThrottle * 100.0) + " %"},
            {"Hover pack current", Fmt("%.1f", M.Summary.HoverPackCurrentA) + " A"}, {"Propeller data", M.Summary.PropBasis}};
        for (const auto& R : Rows)
        {
            L.HLine(X, Y, Wd, Palette::Line());
            L.Text(X, Y + 6.0f, 260.0f, R.first, 15.0f, Palette::Ink70());
            L.Text(X, Y + 3.0f, Wd, R.second, 18.0f, Palette::Ink(), Weight::Regular, Align::Right);
            Y += 30.0f;
        }
        L.HLine(X, Y, Wd, Palette::Line());
        Y += 12.0f;
        L.Text(X, Y, Wd, "Estimates use the nominal pack voltage; the tests supersede them.", 14.0f, Palette::Ink46());
        Y += 30.0f;
    }
    // real meshes vs dimension-true stand-ins
    int Real = 0;
    for (const auto& P : M.Prims) Real += P.Real;
    if (!M.Prims.empty())
    {
        L.Text(X, Y, Wd, "Shown in the 3D view", 15.0f, Palette::Ink70(), Weight::Medium);
        Y += 28.0f;
        Chip(L, X, Y, Real == 0 ? "All stand-ins" : std::to_string(Real) + " real models", Palette::Ink70());
        L.Text(X + 150.0f, Y + 1.0f, Wd - 150.0f, "dimension-true boxes and cylinders", 14.0f, Palette::Ink46());
        Y += 38.0f;
    }
    // findings: errors first (the builder sorts them); the list stops above the status line instead of running off the bottom of the panel
    if (!M.BuildFindings.empty())
    {
        int Errors = 0, Cautions = 0;
        for (const auto& F : M.BuildFindings) { Errors += F.Severity == "error"; Cautions += F.Severity == "warning"; }
        L.Text(X, Y, Wd, "Compatibility   " + std::to_string(Errors) + (Errors == 1 ? " error, " : " errors, ") + std::to_string(Cautions) + (Cautions == 1 ? " caution" : " cautions"),
               15.0f, Errors > 0 ? Palette::Crit() : Palette::Ink70(), Weight::Medium);
        Y += 28.0f;
        constexpr float kFindingsBottom = 1000.0f;   // the status line sits at 1030
        int Shown = 0, Hidden = 0;
        for (const auto& F : M.BuildFindings)
        {
            if (F.Severity == "info") continue;
            const std::vector<std::string> Lines = Wrap(F.Message, 44, 3);
            const float Need = static_cast<float>(Lines.size()) * 20.0f + 10.0f;
            if (Shown >= 9 || Y + Need > kFindingsBottom) { ++Hidden; continue; }
            const Rgba C = F.Severity == "error" ? Palette::Crit() : Palette::Warn();
            L.Text(X, Y, 90.0f, F.Severity == "error" ? "Error" : "Caution", 14.0f, C, Weight::Medium);
            for (const auto& Ln : Lines) { L.Text(X + 80.0f, Y, Wd - 80.0f, Ln, 14.0f, Palette::Ink70()); Y += 20.0f; }
            Y += 10.0f;
            ++Shown;
        }
        if (Hidden > 0) L.Text(X, Y, Wd, "+ " + std::to_string(Hidden) + " more (see vehicle_build.json)", 14.0f, Palette::Ink46());
    }
    else if (M.BuildAttempted && !M.Busy() && !M.Prog.BuildDone) L.Text(X, Y, Wd, "The build did not finish; see the message below.", 15.0f, Palette::Crit());
    StatusLine(L, M, X, 1030.0f, Wd);
    // parts by role (right of the panel): on its own dark panel, so it reads over the bright sky of the 3D view
    {
        const float Lx = Pw + 24.0f, Ly = static_cast<float>(Ui::kTopBarHeightPx) + 16.0f, Lw = 680.0f, RowH = 36.0f;
        const float Lh = 52.0f + static_cast<float>(M.Sel.Roles().size()) * RowH + 12.0f;
        L.Fill(Lx, Ly, Lw, Lh, Palette::Solid());
        L.Text(Lx + 20.0f, Ly + 14.0f, 400.0f, "Parts", 18.0f, Palette::Ink70(), Weight::Medium);
        float Ry = Ly + 52.0f;
        for (const auto& [Role, Id] : M.Sel.Roles())
        {
            const Part* P = M.Cat.Find(Id);
            L.HLine(Lx + 20.0f, Ry - 4.0f, Lw - 40.0f, Palette::Line());
            L.Text(Lx + 20.0f, Ry + 2.0f, 160.0f, Role, 17.0f, Palette::Ink46(), Weight::Regular);
            L.Text(Lx + 190.0f, Ry, Lw - 210.0f, P ? Shorten(P->Title(), 38) : Id, 20.0f, Palette::Ink(), Weight::Regular);
            Ry += RowH;
        }
    }
}

// ----------------------------------------------------------------------------------------------------------------------------- 4 Conditions
inline void Conditions(DrawList& L, const AppModel& M)
{
    NavBar(L, M);
    Scrim(L, 760.0f);
    const float X = 48.0f;
    L.Text(X, 80.0f, 500.0f, "Conditions", 34.0f, Palette::Ink(), Weight::Light);
    float Y = 140.0f;
    auto Row = [&](const std::string& Label, const std::string& Unit, const std::string& FieldId, const std::string& Value, const std::string& Hint) {
        L.HLine(X, Y, 680.0f, Palette::Line());
        L.Text(X, Y + 12.0f, 230.0f, Label, 16.0f, Palette::Ink70());
        Field(L, X + 240.0f, Y + 6.0f, 150.0f, Value, 26.0f, M.Focus == FieldId, FieldId);
        L.Text(X + 410.0f, Y + 14.0f, 110.0f, Unit, 15.0f, Palette::Ink46());
        L.Text(X + 520.0f, Y + 15.0f, 160.0f, Hint, 13.0f + 1.0f, Palette::Ink46());
        Y += 50.0f;
    };
    Row("Condition set name", "", "cond:name", M.Cond.Name, "a-z 0-9 _");
    for (std::size_t I = 0; I < ConditionFields().size(); ++I)
    {
        const FieldSpec& F = ConditionFields()[I];
        char Hint[64];
        std::snprintf(Hint, sizeof Hint, "%g .. %g%s", F.Lo, F.Hi, F.Optional ? " or empty" : "");
        Row(F.Label, F.Unit, std::string("cond:") + F.Id, M.Cond.Value[I], Hint);
    }
    Row("Seed", "", "cond:seed", M.Cond.Seed, "whole number");
    // turbulence choice
    L.HLine(X, Y, 680.0f, Palette::Line());
    L.Text(X, Y + 12.0f, 230.0f, "Turbulence", 16.0f, Palette::Ink70());
    float Tx = X + 240.0f;
    for (const char* T : {"none", "dryden", "simple"})
    {
        const bool Sel = M.Cond.Turbulence == T;
        const std::string Label = std::string(T) == "dryden" ? "Dryden (unverified)" : T;
        const float Tw = EstimateTextWidth(Label, 16.0f) + 14.0f;
        L.Text(Tx, Y + 12.0f, Tw, Label, 16.0f, Sel ? Palette::Ink() : Palette::Ink46(), Sel ? Weight::Medium : Weight::Regular);
        L.AddHit(Tx - 4.0f, Y + 4.0f, Tw, 36.0f, Act::SetTurbulence, T);
        Tx += Tw + 8.0f;
    }
    L.HLine(X, Y + 50.0f, 680.0f, Palette::Line());
    // profile: Full is the default; Quick shortens the long tests with the fixed durations of QuickOverrides()
    {
        const float Py = Y + 66.0f;
        L.Text(X, Py + 4.0f, 230.0f, "Test profile", 16.0f, Palette::Ink70());
        float Px = X + 240.0f;
        for (const char* Id : {"full", "quick"})
        {
            const bool Sel = (std::string(Id) == "quick") == M.QuickProfile;
            const std::string Label = std::string(Id) == "quick" ? "Quick" : "Full";
            const float Tw = EstimateTextWidth(Label, 16.0f) + 14.0f;
            L.Text(Px, Py + 4.0f, Tw, Label, 16.0f, Sel ? Palette::Ink() : Palette::Ink46(), Sel ? Weight::Medium : Weight::Regular);
            if (Sel) L.Fill(Px, Py + 26.0f, Tw - 14.0f, 2.0f, Palette::Accent());
            L.AddHit(Px - 4.0f, Py - 2.0f, Tw, 34.0f, Act::SetProfile, Id);
            Px += Tw + 12.0f;
        }
        if (M.QuickProfile)
            L.Text(X, Py + 40.0f, 700.0f, "Quick: endurance 5 min, thermal soak 10 min, 15 s holds. Marked on Run and in the report.", 14.0f, Palette::Warn());
        else
            L.Text(X, Py + 40.0f, 700.0f, "Full: the project's test durations (endurance up to 1 h, thermal soak up to 2 h of simulated time).", 14.0f, Palette::Ink46());
    }

    // right: tests and limits (solid panel for legibility over the scene)
    L.Fill(760.0f, static_cast<float>(Ui::kTopBarHeightPx), W - 760.0f, Hgt - static_cast<float>(Ui::kTopBarHeightPx), Palette::Solid());
    const float Rx = 820.0f, Rw = 1050.0f;
    L.Text(Rx, 80.0f, 300.0f, "Tests", 34.0f, Palette::Ink(), Weight::Light);
    float Ty = 140.0f;
    for (const auto& T : TestList())
    {
        const bool On = M.SelectedTests.count(T.Id) > 0;
        const bool Shown = M.LimitTest == T.Id;
        L.HLine(Rx, Ty, Rw, Palette::Line());
        L.Line(Rx, Ty + 12.0f, Rx + 18.0f, Ty + 12.0f, Palette::Ink70()); L.Line(Rx, Ty + 30.0f, Rx + 18.0f, Ty + 30.0f, Palette::Ink70());
        L.Line(Rx, Ty + 12.0f, Rx, Ty + 30.0f, Palette::Ink70()); L.Line(Rx + 18.0f, Ty + 12.0f, Rx + 18.0f, Ty + 30.0f, Palette::Ink70());
        if (On) L.Fill(Rx + 4.0f, Ty + 16.0f, 11.0f, 11.0f, Palette::Ink());
        L.Text(Rx + 36.0f, Ty + 10.0f, 500.0f, T.Label, 22.0f, T.Implemented ? (Shown ? Palette::Ink() : Palette::Ink70()) : Palette::Ink46(), Weight::Light);
        if (!T.Implemented) Chip(L, Rx + 330.0f, Ty + 10.0f, "Not implemented", Palette::Ink46());
        L.AddHit(Rx, Ty, Rw, 40.0f, Act::ToggleTest, T.Id);
        Ty += 40.0f;
    }
    L.HLine(Rx, Ty, Rw, Palette::Line());
    // editable limits of the shown test
    Ty += 34.0f;
    L.Text(Rx, Ty, 600.0f, "Pass / fail limits and durations: " + M.LimitTest, 16.0f, Palette::Ink70(), Weight::Medium);
    if (M.EditedLimits) Chip(L, Rx + 640.0f, Ty - 2.0f, "Edited", Palette::Warn());
    Ty += 30.0f;
    for (std::size_t I = 0; I < M.Limits.size(); ++I)
    {
        const EditableLimit& E = M.Limits[I];
        if (E.Test != M.LimitTest) continue;
        const std::string Id = "limit:" + std::to_string(I);
        double Cur = 0.0;
        const bool Parsed = ParseStrictNumber(M.LimitText[I], Cur);
        const bool Edited = Parsed && Cur != E.Original;
        L.HLine(Rx, Ty, Rw, Palette::Line());
        L.Text(Rx, Ty + 8.0f, 420.0f, E.Key, 15.0f, Palette::Ink70());
        Field(L, Rx + 440.0f, Ty + 4.0f, 130.0f, M.LimitText[I], 22.0f, M.Focus == Id, Id, Weight::Regular);
        L.Text(Rx + 590.0f, Ty + 10.0f, 100.0f, E.Unit, 14.0f, Palette::Ink46());
        L.Text(Rx + 700.0f, Ty + 10.0f, 320.0f, Parsed ? (Edited ? "edited (was " + NumText(E.Original) + ")" : "project default") : std::string("not a number"), 14.0f,
               Parsed ? (Edited ? Palette::Warn() : Palette::Ink46()) : Palette::Crit());
        Ty += 36.0f;
    }
    L.HLine(Rx, Ty, Rw, Palette::Line());
    // actions
    Button(L, Rx, 960.0f, 280.0f, 52.0f, M.Busy() ? "Working ..." : "Run tests", true, Act::RunTests, "", !M.Busy());
    Button(L, Rx + 300.0f, 960.0f, 240.0f, 52.0f, "Reset limits", false, Act::ResetLimits);
    std::string Err;
    if (!M.ConditionsOk(Err)) L.Text(Rx + 570.0f, 972.0f, 480.0f, Err.empty() ? "Select at least one test." : Err, 14.0f, Palette::Warn());
    if (!M.CfdLoadError.empty())
    {
        // a CFD model that matched this build could not be used: the reason in amber, and an explicit opt-in to go on without it
        L.Text(Rx, 1030.0f, Rw, "CFD model not used: " + Shorten(M.CfdLoadError, 150), 15.0f, Palette::Warn(), Weight::Medium);
        Button(L, Rx + 560.0f, 960.0f, 300.0f, 52.0f, "Run anyway on analytic drag", false, Act::RunAnalyticFallback, "", !M.Busy());
        StatusLine(L, M, Rx, 1056.0f, Rw);
    }
    else StatusLine(L, M, Rx, 1030.0f, Rw);
}
}  // namespace Screens
}  // namespace DroneApp
