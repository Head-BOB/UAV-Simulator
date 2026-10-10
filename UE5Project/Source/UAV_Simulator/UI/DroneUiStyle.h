// Tier: V
// DEMO: the look of the UI in one place (design brief: glass-cockpit / motorsport telemetry). The numbers come
// from DroneUiMath.h so the contrast and scale rules are tested without Unreal. Slate API names and the
// font style names are TODO(HUMAN-VERIFY: builds on the project's Unreal version). NOT COMPILED OR RUN.
#pragma once

#include "CoreMinimal.h"
#include "DroneUiMath.h"
#include "Fonts/SlateFontInfo.h"
#include "Styling/CoreStyle.h"
#include "Styling/SlateBrush.h"
#include "Styling/SlateColor.h"
#include "Brushes/SlateRoundedBoxBrush.h"

namespace DroneUiStyle
{
inline FLinearColor Rgb(uint32 Hex, float Alpha = 1.0f)
{
	const FColor C(static_cast<uint8>((Hex >> 16) & 0xFF), static_cast<uint8>((Hex >> 8) & 0xFF), static_cast<uint8>(Hex & 0xFF), 255);
	FLinearColor L = FLinearColor::FromSRGBColor(C);
	L.A = Alpha;
	return L;
}

inline FLinearColor Accent() { return Rgb(DroneUi::kAccent); }
inline FLinearColor Warning() { return Rgb(DroneUi::kWarning); }
inline FLinearColor Critical() { return Rgb(DroneUi::kCritical); }
inline FLinearColor Text() { return Rgb(DroneUi::kText); }
inline FLinearColor Muted() { return Rgb(DroneUi::kTextMuted); }
inline FLinearColor Hairline() { return Rgb(DroneUi::kHairline); }
inline FLinearColor PanelFill() { return Rgb(DroneUi::kPanelBottom, static_cast<float>(DroneUi::kPanelOpacity)); }
inline FLinearColor StatusTint(DroneUi::Status S) { return Rgb(DroneUi::StatusRgb(S)); }

// Roboto only (the engine's default font). Sizes are the type scale in px at 1080p, multiplied by the UI scale
// by the caller. Index into DroneUi::kTypeScalePx: 0 label, 1 body, 2 value, 3 heading, 4 hero.
inline FSlateFontInfo Font(int32 ScaleIndex, bool bBold = false, float UiScale = 1.0f)
{
	const int32 Idx = FMath::Clamp(ScaleIndex, 0, 4);
	const int32 Size = FMath::Max(1, FMath::RoundToInt(DroneUi::kTypeScalePx[Idx] * UiScale));
	// TODO(HUMAN-VERIFY): "BoldCondensed" is Roboto Condensed Bold in the default composite font; fall back to "Bold".
	return FCoreStyle::GetDefaultFontStyle(bBold ? "Bold" : "Regular", Size);
}
inline FSlateFontInfo FontCondensed(int32 ScaleIndex, float UiScale = 1.0f)
{
	const int32 Idx = FMath::Clamp(ScaleIndex, 0, 4);
	return FCoreStyle::GetDefaultFontStyle("BoldCondensed", FMath::Max(1, FMath::RoundToInt(DroneUi::kTypeScalePx[Idx] * UiScale)));
}

// Rounded panel brush: ~85 % graphite fill, 1 px hairline, 6 px corners. Created once and kept (static).
inline const FSlateBrush* PanelBrush()
{
	static const FSlateRoundedBoxBrush Brush(PanelFill(), static_cast<float>(DroneUi::kCornerRadiusPx), Hairline(), static_cast<float>(DroneUi::kHairlinePx));
	return &Brush;
}

inline const FSlateBrush* CardBrush(bool bSelected)
{
	static const FSlateRoundedBoxBrush Normal(PanelFill(), static_cast<float>(DroneUi::kCornerRadiusPx), Hairline(), 1.0f);
	static const FSlateRoundedBoxBrush Selected(PanelFill(), static_cast<float>(DroneUi::kCornerRadiusPx), Accent(), 2.0f);
	return bSelected ? &Selected : &Normal;
}

inline const FSlateBrush* WhiteBrush()
{
	return FCoreStyle::Get().GetBrush("GenericWhiteBox");
}

// Spacing on the 8 px grid.
inline float Grid(float Units = 1.0f) { return static_cast<float>(DroneUi::kGridPx) * Units; }
}  // namespace DroneUiStyle
