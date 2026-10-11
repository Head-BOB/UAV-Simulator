// Tier: V
// Engine-free display list: what a screen consists of (filled rectangles, lines, text, polylines) and where a click lands (hit
// regions), all in the 1920 x 1080 design space of the approved preview. The screens are built as pure functions that fill one of
// these (DroneScreens*.h); the Unreal layer only paints the list with Slate and forwards mouse / key events. This keeps every layout
// number and every piece of text testable with plain g++. Text width is not measured here (no font): the painter aligns and clips.
#pragma once

#include <cstddef>
#include <string>
#include <vector>

#include "DroneUiTokens.h"

namespace DroneApp
{
struct Rgba
{
    float R = 0, G = 0, B = 0, A = 1;   // sRGB 0..1, straight alpha
};

inline Rgba ToRgba(std::uint32_t Rgb, double Alpha = 1.0)
{
    return Rgba{static_cast<float>((Rgb >> 16) & 0xFF) / 255.0f, static_cast<float>((Rgb >> 8) & 0xFF) / 255.0f, static_cast<float>(Rgb & 0xFF) / 255.0f, static_cast<float>(Alpha)};
}
inline Rgba ToRgba(const Ui::Colour& C) { return ToRgba(C.Rgb, C.Alpha); }

namespace Palette
{
inline Rgba Ink() { return ToRgba(Ui::kInk); }
inline Rgba Ink70() { return ToRgba(Ui::kInk70); }
inline Rgba Ink46() { return ToRgba(Ui::kInk46); }
inline Rgba Line() { return ToRgba(Ui::kLine); }
inline Rgba LineStrong() { return ToRgba(Ui::kLineStrong); }
inline Rgba Bar() { return ToRgba(Ui::kBar); }
inline Rgba Solid() { return ToRgba(Ui::kSolid); }
inline Rgba Report() { return ToRgba(Ui::kReportBackground); }
inline Rgba Accent() { return ToRgba(Ui::kAccent); }
inline Rgba Ok() { return ToRgba(Ui::kOk); }
inline Rgba Warn() { return ToRgba(Ui::kWarn); }
inline Rgba Crit() { return ToRgba(Ui::kCrit); }
inline Rgba OnPrimary() { return ToRgba(Ui::kOnPrimaryButton); }
inline Rgba OnWarn() { return ToRgba(Ui::kOnWarnBar); }
}  // namespace Palette

enum class Weight { Light, Regular, Medium };
enum class Align { Left, Right, Centre };

struct Pt
{
    float X, Y;
};

struct Item
{
    enum class Kind { Rect, Line, Text, Poly };
    Kind K = Kind::Rect;
    float X = 0, Y = 0, W = 0, H = 0;   // Rect: x y w h. Line: x1 y1 x2 y2 in X Y W H. Text: anchor x, y (top of the text box) and box width W (0 = unbounded)
    Rgba C;
    float Thick = 1.0f;
    std::string Text;
    float Px = 17.0f;
    Weight Wt = Weight::Regular;
    Align Al = Align::Left;
    bool Shadow = false;                // soft text shadow, only for text drawn directly over the 3D view
    std::vector<Pt> Pts;                // Poly
};

// Actions a click or key can trigger; the model gives them their meaning. Arg carries an id (screen name, part id, field id, test id ...).
enum class Act
{
    None,
    Navigate, NavPrev, NavNext,
    PickVehicle,
    PickCategory, PickManufacturer, PickPart, AddPart, RemovePart, ScrollList,
    FocusField, ClearFocus,
    BuildNow, UseBuild,
    ToggleTest, ResetLimits, RunTests, SetTurbulence,
    PlayPause, Faster, Slower, Restart, SeekBar, PickTest, ToggleDetails, ToggleControls,
    WriteReport, OpenReport, RunStudy, OpenFolder,
    Dismiss,
};

struct Hit
{
    float X = 0, Y = 0, W = 0, H = 0;
    Act A = Act::None;
    std::string Arg;
    bool Contains(float Px, float Py) const { return Px >= X && Px < X + W && Py >= Y && Py < Y + H; }
};

class DrawList
{
public:
    std::vector<Item> Items;
    std::vector<Hit> Hits;
    std::uint64_t Version = 0;   // incremented by Clear(): painters cache converted strings per version, so painting itself does not allocate

    void Clear() { Items.clear(); Hits.clear(); ++Version; }

    void Fill(float X, float Y, float W, float H, const Rgba& C)
    {
        Item I; I.K = Item::Kind::Rect; I.X = X; I.Y = Y; I.W = W; I.H = H; I.C = C;
        Items.push_back(std::move(I));
    }
    void Line(float X1, float Y1, float X2, float Y2, const Rgba& C, float Thick = 1.0f)
    {
        Item I; I.K = Item::Kind::Line; I.X = X1; I.Y = Y1; I.W = X2; I.H = Y2; I.C = C; I.Thick = Thick;
        Items.push_back(std::move(I));
    }
    void HLine(float X, float Y, float W, const Rgba& C) { Line(X, Y, X + W, Y, C, 1.0f); }
    void VLine(float X, float Y, float H, const Rgba& C) { Line(X, Y, X, Y + H, C, 1.0f); }
    void Poly(const std::vector<Pt>& Pts, const Rgba& C, float Thick = 1.5f)
    {
        Item I; I.K = Item::Kind::Poly; I.Pts = Pts; I.C = C; I.Thick = Thick;
        Items.push_back(std::move(I));
    }
    // Text: (X, Y) is the top-left of the text box, Width the box width (alignment and clipping happen inside it; 0 = as wide as needed, left only).
    void Text(float X, float Y, float Width, const std::string& S, float Px, const Rgba& C, Weight Wt = Weight::Regular, Align Al = Align::Left, bool Shadow = false)
    {
        if (S.empty()) return;
        Item I; I.K = Item::Kind::Text; I.X = X; I.Y = Y; I.W = Width; I.Text = S; I.Px = Px; I.C = C; I.Wt = Wt; I.Al = Al; I.Shadow = Shadow;
        Items.push_back(std::move(I));
    }
    void AddHit(float X, float Y, float W, float H, Act A, const std::string& Arg = std::string())
    {
        Hit Ht; Ht.X = X; Ht.Y = Y; Ht.W = W; Ht.H = H; Ht.A = A; Ht.Arg = Arg;
        Hits.push_back(std::move(Ht));
    }

    // The topmost hit region under a point (the LAST added wins, as with painting order), or nullptr.
    const Hit* HitAt(float X, float Y) const
    {
        for (std::size_t I = Hits.size(); I-- > 0;)
            if (Hits[I].Contains(X, Y)) return &Hits[I];
        return nullptr;
    }

    // Counts, for tests.
    std::size_t Count(Item::Kind K) const { std::size_t N = 0; for (const auto& I : Items) N += I.K == K; return N; }
    // First text item whose text contains Needle (nullptr if none). For tests.
    const Item* FindText(const std::string& Needle) const
    {
        for (const auto& I : Items)
            if (I.K == Item::Kind::Text && I.Text.find(Needle) != std::string::npos) return &I;
        return nullptr;
    }
};

// ----------------------------------------------------------------------------------------------- shared widgets (as display-list builders)

// 1 px outlined chip, no fill (status chips and provenance tags). Returns its width estimate so rows can place the next element.
// Text width is estimated at 0.56 x font size per character (Roboto average); only used for layout, the painter does not depend on it.
constexpr float kAvgCharWidth = 0.56f;
inline float EstimateTextWidth(const std::string& S, float Px) { return static_cast<float>(S.size()) * Px * kAvgCharWidth; }

inline float Chip(DrawList& L, float X, float Y, const std::string& S, const Rgba& C)
{
    const float Px = static_cast<float>(Ui::kLabelPx);
    const float W = EstimateTextWidth(S, Px) + 20.0f, H = 22.0f;
    L.Line(X, Y, X + W, Y, C, 1.0f);
    L.Line(X, Y + H, X + W, Y + H, C, 1.0f);
    L.Line(X, Y, X, Y + H, C, 1.0f);
    L.Line(X + W, Y, X + W, Y + H, C, 1.0f);
    L.Text(X, Y + 2.0f, W, S, Px, C, Weight::Medium, Align::Centre);
    return W;
}

// Filled (primary) or outline (secondary) button; the hit region is registered here.
inline void Button(DrawList& L, float X, float Y, float W, float H, const std::string& S, bool Primary, Act A, const std::string& Arg = std::string(), bool Enabled = true)
{
    const Rgba Edge = Enabled ? Palette::Ink() : Palette::Ink46();
    if (Primary && Enabled) L.Fill(X, Y, W, H, Palette::Ink());
    else
    {
        L.Line(X, Y, X + W, Y, Edge); L.Line(X, Y + H, X + W, Y + H, Edge); L.Line(X, Y, X, Y + H, Edge); L.Line(X + W, Y, X + W, Y + H, Edge);
    }
    L.Text(X, Y + (H - 18.0f) * 0.5f - 2.0f, W, S, 18.0f, (Primary && Enabled) ? Palette::OnPrimary() : Edge, Weight::Medium, Align::Centre);
    if (Enabled) L.AddHit(X, Y, W, H, A, Arg);
}

// Typed value field: underline only. `Focused` draws the caret as a thin accent bar after the text (position estimated).
inline void Field(DrawList& L, float X, float Y, float W, const std::string& Value, float Px, bool Focused, const std::string& FieldId, Weight Wt = Weight::Light)
{
    L.Text(X, Y, W, Value, Px, Palette::Ink(), Wt, Align::Left);
    const float LineY = Y + Px * 1.25f;
    L.HLine(X, LineY, W, Focused ? Palette::Accent() : Palette::LineStrong());
    if (Focused)
    {
        const float CaretX = X + EstimateTextWidth(Value, Px);
        L.Fill(CaretX + 1.0f, Y + 2.0f, 2.0f, Px * 1.1f, Palette::Accent());
    }
    L.AddHit(X, Y - 4.0f, W, Px * 1.25f + 10.0f, Act::FocusField, FieldId);
}
}  // namespace DroneApp
