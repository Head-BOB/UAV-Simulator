// Tier: V
// NOT COMPILED OR RUN by its author. See SDroneAppCanvas.h.
#include "SDroneAppCanvas.h"

#include "Fonts/FontMeasure.h"
#include "Framework/Application/SlateApplication.h"
#include "Rendering/DrawElements.h"
#include "Rendering/SlateRenderer.h"
#include "Styling/CoreStyle.h"

#include "DroneScreens.h"

namespace
{
FLinearColor ToLinear(const DroneApp::Rgba& C)
{
	FLinearColor L = FLinearColor::FromSRGBColor(FColor(static_cast<uint8>(C.R * 255.0f + 0.5f), static_cast<uint8>(C.G * 255.0f + 0.5f), static_cast<uint8>(C.B * 255.0f + 0.5f), 255));
	L.A = C.A;
	return L;
}

// Roboto only (the engine's default font). Light uses the "Light" typeface, Medium the "Bold" one (the default composite font has no
// Medium). TODO(HUMAN-VERIFY): typeface names "Light", "Regular", "Bold" in FCoreStyle's default composite font on this engine version.
FSlateFontInfo FontFor(float DesignPx, float Scale, DroneApp::Weight W)
{
	const int32 Size = FMath::Max(1, FMath::RoundToInt(DesignPx * Scale * 0.75f));   // Slate font size is in points (about 0.75 px)
	static const FName Light("Light"), Bold("Bold"), Regular("Regular");   // names created once
	const FName Face = W == DroneApp::Weight::Light ? Light : (W == DroneApp::Weight::Medium ? Bold : Regular);
	return FCoreStyle::GetDefaultFontStyle(Face, Size);
}
}  // namespace

void SDroneAppCanvas::Construct(const FArguments& InArgs)
{
	Model = InArgs._Model;
	List = InArgs._List;
	OnChanged = InArgs._OnChanged;
}

void SDroneAppCanvas::RefreshTextCache() const
{
	if (!List || CachedVersion == List->Version) return;
	TextCache.Reset(List->Items.size());
	for (const DroneApp::Item& I : List->Items) TextCache.Add(I.K == DroneApp::Item::Kind::Text ? FString(UTF8_TO_TCHAR(I.Text.c_str())) : FString());
	CachedVersion = List->Version;
}

int32 SDroneAppCanvas::OnPaint(const FPaintArgs&, const FGeometry& G, const FSlateRect&, FSlateWindowElementList& Out, int32 Layer, const FWidgetStyle&, bool) const
{
	if (!List) return Layer;
	RefreshTextCache();
	const FVector2f Size = G.GetLocalSize();
	const DroneApp::Ui::Viewport Fit = FitFor(Size);
	const float S = static_cast<float>(Fit.Scale), Ox = static_cast<float>(Fit.OffsetX), Oy = static_cast<float>(Fit.OffsetY);
	auto P = [&](float X, float Y) { return FVector2f(X * S + Ox, Y * S + Oy); };
	const FSlateBrush* White = FCoreStyle::Get().GetBrush("GenericWhiteBox");
	int32 L = Layer;
	static TArray<FVector2f> Pts;   // reused scratch (one paint thread)

	for (int32 Index = 0; Index < static_cast<int32>(List->Items.size()); ++Index)
	{
		const DroneApp::Item& I = List->Items[static_cast<size_t>(Index)];
		++L;   // painting order = list order
		switch (I.K)
		{
			case DroneApp::Item::Kind::Rect:
				FSlateDrawElement::MakeBox(Out, L, G.ToPaintGeometry(FVector2f(I.W * S, I.H * S), FSlateLayoutTransform(P(I.X, I.Y))), White, ESlateDrawEffect::None, ToLinear(I.C));
				break;
			case DroneApp::Item::Kind::Line:
				Pts.Reset();
				Pts.Add(P(I.X, I.Y));
				Pts.Add(P(I.W, I.H));
				FSlateDrawElement::MakeLines(Out, L, G.ToPaintGeometry(), Pts, ESlateDrawEffect::None, ToLinear(I.C), true, FMath::Max(1.0f, I.Thick * S));
				break;
			case DroneApp::Item::Kind::Poly:
				Pts.Reset();
				for (const DroneApp::Pt& Q : I.Pts) Pts.Add(P(Q.X, Q.Y));
				if (Pts.Num() >= 2) FSlateDrawElement::MakeLines(Out, L, G.ToPaintGeometry(), Pts, ESlateDrawEffect::None, ToLinear(I.C), true, FMath::Max(1.0f, I.Thick * S));
				break;
			case DroneApp::Item::Kind::Text:
			{
				const FString& Str = TextCache[Index];
				const FSlateFontInfo Font = FontFor(I.Px, S, I.Wt);
				float X = I.X * S + Ox;
				if (I.Al != DroneApp::Align::Left)
				{
					const TSharedRef<FSlateFontMeasure> Measure = FSlateApplication::Get().GetRenderer()->GetFontMeasureService();
					const float Tw = static_cast<float>(Measure->Measure(Str, Font).X);
					X = I.Al == DroneApp::Align::Right ? (I.X + I.W) * S + Ox - Tw : (I.X + I.W * 0.5f) * S + Ox - Tw * 0.5f;
				}
				const FVector2f At(X, I.Y * S + Oy);
				if (I.Shadow)
					FSlateDrawElement::MakeText(Out, L, G.ToPaintGeometry(G.GetLocalSize(), FSlateLayoutTransform(At + FVector2f(1.0f, 1.0f))), Str, Font, ESlateDrawEffect::None, FLinearColor(0, 0, 0, 0.6f));
				FSlateDrawElement::MakeText(Out, L + 1, G.ToPaintGeometry(G.GetLocalSize(), FSlateLayoutTransform(At)), Str, Font, ESlateDrawEffect::None, ToLinear(I.C));
				++L;
				break;
			}
		}
	}
	return L + 1;
}

FReply SDroneAppCanvas::OnMouseButtonDown(const FGeometry& G, const FPointerEvent& E)
{
	if (!Model || !List || E.GetEffectingButton() != EKeys::LeftMouseButton) return FReply::Unhandled();
	const FVector2f Local = G.AbsoluteToLocal(E.GetScreenSpacePosition());
	const FVector2f Size = G.GetLocalSize();
	const DroneApp::Ui::Viewport Fit = FitFor(Size);
	const float S = static_cast<float>(Fit.Scale);
	if (S <= 0.0f) return FReply::Unhandled();
	DroneApp::HandleClick(*Model, *List, (Local.X - static_cast<float>(Fit.OffsetX)) / S, (Local.Y - static_cast<float>(Fit.OffsetY)) / S);
	Changed();
	return FReply::Handled().SetUserFocus(AsShared(), EFocusCause::Mouse);
}

FReply SDroneAppCanvas::OnMouseWheel(const FGeometry&, const FPointerEvent& E)
{
	if (!Model || Model->Current != DroneApp::Screen::Parts) return FReply::Unhandled();
	Model->Click(DroneApp::Act::ScrollList, E.GetWheelDelta() > 0.0f ? "-3" : "3");
	Changed();
	return FReply::Handled();
}

FReply SDroneAppCanvas::OnKeyDown(const FGeometry&, const FKeyEvent& E)
{
	if (!Model) return FReply::Unhandled();
	const FKey K = E.GetKey();
	if (!Model->Focus.empty())
	{
		if (K == EKeys::BackSpace) Model->KeyBackspace();
		else if (K == EKeys::Enter) Model->KeyEnter();
		else if (K == EKeys::Escape) Model->KeyEscape();
		else return FReply::Unhandled();   // printable keys arrive through OnKeyChar
		Changed();
		return FReply::Handled();
	}
	const char* Name = nullptr;
	if (K == EKeys::Tab) Name = "Tab";
	else if (K == EKeys::SpaceBar) Name = "Space";
	else if (K == EKeys::I) Name = "I";
	else if (K == EKeys::Left) Name = "Left";
	else if (K == EKeys::Right) Name = "Right";
	else if (K == EKeys::Home) Name = "Home";
	else if (K == EKeys::Equals || K == EKeys::Add) Name = "Equals";
	else if (K == EKeys::Hyphen || K == EKeys::Subtract) Name = "Minus";
	if (K == EKeys::Escape) { Model->KeyEscape(); Changed(); return FReply::Handled(); }
	if (!Name) return FReply::Unhandled();
	Model->KeyShortcut(Name);
	Changed();
	return FReply::Handled();
}

FReply SDroneAppCanvas::OnKeyChar(const FGeometry&, const FCharacterEvent& E)
{
	if (!Model || Model->Focus.empty()) return FReply::Unhandled();
	const TCHAR C = E.GetCharacter();
	if (C < 0x20 || C > 0x7E) return FReply::Unhandled();
	Model->KeyChar(static_cast<char>(C));
	Changed();
	return FReply::Handled();
}
