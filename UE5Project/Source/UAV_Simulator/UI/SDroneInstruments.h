// Tier: V
// DEMO: custom-painted instruments (artificial horizon, tapes, compass with wind arrow, RPM bars, line chart,
// margin row). All drawing is FSlateDrawElement lines, boxes and text from the cached FDroneUiSnapshot: nothing
// is allocated while painting (strings are prebuilt at 30 Hz, point arrays are reserved once). NOT COMPILED OR
// RUN by its author: TODO(HUMAN-VERIFY: Slate draw-element signatures on the project's Unreal version; needs a
// build and a visual check at 1080p and 4K).
#pragma once

#include "CoreMinimal.h"
#include "DroneUiMath.h"
#include "DroneUiSnapshot.h"
#include "Widgets/SLeafWidget.h"

// Shared draw helpers (all take LOCAL coordinates inside the widget, y down).
namespace DroneDraw
{
void Line(FSlateWindowElementList& Out, int32 Layer, const FGeometry& G, const FVector2f& A, const FVector2f& B, const FLinearColor& Color, float Thickness);
void Polyline(FSlateWindowElementList& Out, int32 Layer, const FGeometry& G, const TArray<FVector2f>& Points, const FLinearColor& Color, float Thickness);
void Box(FSlateWindowElementList& Out, int32 Layer, const FGeometry& G, const FVector2f& Pos, const FVector2f& Size, const FLinearColor& Color);
void Text(FSlateWindowElementList& Out, int32 Layer, const FGeometry& G, const FVector2f& Pos, const FString& Str, const FSlateFontInfo& Font, const FLinearColor& Color);
const FString& NumberLabel(int32 Value);  // cached strings for tape labels
}  // namespace DroneDraw

class SDroneHorizon : public SLeafWidget
{
public:
	SLATE_BEGIN_ARGS(SDroneHorizon) {}
		SLATE_ARGUMENT(const FDroneUiSnapshot*, Snapshot)
		SLATE_ARGUMENT(float, UiScale)
	SLATE_END_ARGS()
	void Construct(const FArguments& InArgs);
	virtual int32 OnPaint(const FPaintArgs& Args, const FGeometry& G, const FSlateRect& Cull, FSlateWindowElementList& Out, int32 Layer, const FWidgetStyle& Style, bool bParentEnabled) const override;
	virtual bool ComputeVolatility() const override { return true; }  // repainted every frame, like a gauge
	virtual FVector2D ComputeDesiredSize(float) const override { return FVector2D(240.0 * Scale, 240.0 * Scale); }
private:
	const FDroneUiSnapshot* Snap = nullptr;
	float Scale = 1.0f;
};

// Vertical tape: altitude (m) or speed (m/s), the current value in a box at the centre.
class SDroneTape : public SLeafWidget
{
public:
	SLATE_BEGIN_ARGS(SDroneTape) {}
		SLATE_ARGUMENT(const FDroneUiSnapshot*, Snapshot)
		SLATE_ARGUMENT(bool, bAltitude)
		SLATE_ARGUMENT(float, UiScale)
	SLATE_END_ARGS()
	void Construct(const FArguments& InArgs);
	virtual int32 OnPaint(const FPaintArgs& Args, const FGeometry& G, const FSlateRect& Cull, FSlateWindowElementList& Out, int32 Layer, const FWidgetStyle& Style, bool bParentEnabled) const override;
	virtual bool ComputeVolatility() const override { return true; }  // repainted every frame, like a gauge
	virtual FVector2D ComputeDesiredSize(float) const override { return FVector2D(96.0 * Scale, 240.0 * Scale); }
private:
	const FDroneUiSnapshot* Snap = nullptr;
	bool bAlt = true;
	float Scale = 1.0f;
};

// Heading tape (horizontal) with a wind arrow circle.
class SDroneCompass : public SLeafWidget
{
public:
	SLATE_BEGIN_ARGS(SDroneCompass) {}
		SLATE_ARGUMENT(const FDroneUiSnapshot*, Snapshot)
		SLATE_ARGUMENT(float, UiScale)
	SLATE_END_ARGS()
	void Construct(const FArguments& InArgs);
	virtual int32 OnPaint(const FPaintArgs& Args, const FGeometry& G, const FSlateRect& Cull, FSlateWindowElementList& Out, int32 Layer, const FWidgetStyle& Style, bool bParentEnabled) const override;
	virtual bool ComputeVolatility() const override { return true; }  // repainted every frame, like a gauge
	virtual FVector2D ComputeDesiredSize(float) const override { return FVector2D(360.0 * Scale, 96.0 * Scale); }
private:
	const FDroneUiSnapshot* Snap = nullptr;
	float Scale = 1.0f;
	mutable TArray<FVector2f> CirclePoints;  // reserved once
};

// Four motor RPM bars (FR, FL, RR, RL) with the thrust text under each.
class SDroneMotorBars : public SLeafWidget
{
public:
	SLATE_BEGIN_ARGS(SDroneMotorBars) {}
		SLATE_ARGUMENT(const FDroneUiSnapshot*, Snapshot)
		SLATE_ARGUMENT(float, UiScale)
	SLATE_END_ARGS()
	void Construct(const FArguments& InArgs);
	virtual int32 OnPaint(const FPaintArgs& Args, const FGeometry& G, const FSlateRect& Cull, FSlateWindowElementList& Out, int32 Layer, const FWidgetStyle& Style, bool bParentEnabled) const override;
	virtual bool ComputeVolatility() const override { return true; }  // repainted every frame, like a gauge
	virtual FVector2D ComputeDesiredSize(float) const override { return FVector2D(360.0 * Scale, 128.0 * Scale); }
private:
	const FDroneUiSnapshot* Snap = nullptr;
	float Scale = 1.0f;
};

// Line chart of one ring buffer, auto-scaled with padding.
class SDroneChart : public SLeafWidget
{
public:
	enum class EKind : uint8 { Altitude, Speed, BatteryA, Throttle };
	SLATE_BEGIN_ARGS(SDroneChart) {}
		SLATE_ARGUMENT(const FDroneUiSnapshot*, Snapshot)
		SLATE_ARGUMENT(int32, Kind)
		SLATE_ARGUMENT(FString, Title)
		SLATE_ARGUMENT(float, UiScale)
	SLATE_END_ARGS()
	void Construct(const FArguments& InArgs);
	virtual int32 OnPaint(const FPaintArgs& Args, const FGeometry& G, const FSlateRect& Cull, FSlateWindowElementList& Out, int32 Layer, const FWidgetStyle& Style, bool bParentEnabled) const override;
	virtual bool ComputeVolatility() const override { return true; }  // repainted every frame, like a gauge
	virtual FVector2D ComputeDesiredSize(float) const override { return FVector2D(220.0 * Scale, 72.0 * Scale); }
private:
	const FDroneUiSnapshot* Snap = nullptr;
	EKind Kind = EKind::Altitude;
	FString Title;
	float Scale = 1.0f;
	mutable TArray<FVector2f> Points;  // reserved to the ring capacity once
};

// One component margin: label, bar to 2.0, value. Margin and label are set by the owner (5 Hz at most).
class SDroneMarginRow : public SLeafWidget
{
public:
	SLATE_BEGIN_ARGS(SDroneMarginRow) {}
		SLATE_ARGUMENT(float, UiScale)
	SLATE_END_ARGS()
	void Construct(const FArguments& InArgs);
	void Set(const FString& InLabel, double InMargin);
	virtual int32 OnPaint(const FPaintArgs& Args, const FGeometry& G, const FSlateRect& Cull, FSlateWindowElementList& Out, int32 Layer, const FWidgetStyle& Style, bool bParentEnabled) const override;
	virtual bool ComputeVolatility() const override { return true; }  // repainted every frame, like a gauge
	virtual FVector2D ComputeDesiredSize(float) const override { return FVector2D(260.0 * Scale, 28.0 * Scale); }
private:
	FString Label;
	FString ValueText;
	double Margin = 99.0;
	float Scale = 1.0f;
};
