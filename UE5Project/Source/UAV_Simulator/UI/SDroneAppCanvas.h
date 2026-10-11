// Tier: V
// The one Slate widget of the application: paints a DroneApp::DrawList (rectangles, lines, text, polylines in 1920 x 1080 design space,
// scaled to fit the width) and forwards mouse, wheel and keyboard to the engine-free model. No UMG, no Blueprint. NOT COMPILED OR RUN by
// its author: TODO(HUMAN-VERIFY: Slate draw-element and input signatures on the project's Unreal version).
#pragma once

#include "CoreMinimal.h"
#include "Widgets/SLeafWidget.h"

#include "DroneAppModel.h"
#include "DroneDrawList.h"

class SDroneAppCanvas : public SLeafWidget
{
public:
	SLATE_BEGIN_ARGS(SDroneAppCanvas) {}
		SLATE_ARGUMENT(DroneApp::AppModel*, Model)
		SLATE_ARGUMENT(const DroneApp::DrawList*, List)
		SLATE_EVENT(FSimpleDelegate, OnChanged)
	SLATE_END_ARGS()

	void Construct(const FArguments& InArgs);

	virtual int32 OnPaint(const FPaintArgs& Args, const FGeometry& G, const FSlateRect& Cull, FSlateWindowElementList& Out, int32 Layer, const FWidgetStyle& Style, bool bParentEnabled) const override;
	virtual FVector2D ComputeDesiredSize(float) const override { return FVector2D(1920.0, 1080.0); }
	virtual bool SupportsKeyboardFocus() const override { return true; }
	virtual FReply OnMouseButtonDown(const FGeometry& G, const FPointerEvent& E) override;
	virtual FReply OnMouseWheel(const FGeometry& G, const FPointerEvent& E) override;
	virtual FReply OnKeyDown(const FGeometry& G, const FKeyEvent& E) override;
	virtual FReply OnKeyChar(const FGeometry& G, const FCharacterEvent& E) override;

private:
	DroneApp::AppModel* Model = nullptr;
	const DroneApp::DrawList* List = nullptr;
	FSimpleDelegate OnChanged;
	// FString copies of the text items, rebuilt when the display list changes (not on every paint): painting does not allocate.
	mutable TArray<FString> TextCache;
	mutable uint64 CachedVersion = ~0ull;
	void RefreshTextCache() const;

	// Design space <-> local widget space: one uniform scale that fits the whole 1920 x 1080 design into the widget, centred
	// (DroneApp::Ui::FitViewport, tested without Unreal). Fitting the width alone cut the step bar off in a 1917 x 1018 window.
	static DroneApp::Ui::Viewport FitFor(const FVector2f& Size) { return DroneApp::Ui::FitViewport(static_cast<double>(Size.X), static_cast<double>(Size.Y)); }
	void Changed() const { OnChanged.ExecuteIfBound(); }
};
