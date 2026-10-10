// Tier: V
// DEMO. NOT COMPILED OR RUN by its author. See SDroneInstruments.h.
#include "SDroneInstruments.h"

#include "DroneUiStyle.h"
#include "Rendering/DrawElements.h"

namespace DroneDraw
{
void Line(FSlateWindowElementList& Out, int32 Layer, const FGeometry& G, const FVector2f& A, const FVector2f& B, const FLinearColor& Color, float Thickness)
{
	TArray<FVector2f, TInlineAllocator<2>> P;  // stack storage, no heap
	P.Add(A);
	P.Add(B);
	FSlateDrawElement::MakeLines(Out, Layer, G.ToPaintGeometry(), P, ESlateDrawEffect::None, Color, true, Thickness);
}

void Polyline(FSlateWindowElementList& Out, int32 Layer, const FGeometry& G, const TArray<FVector2f>& Points, const FLinearColor& Color, float Thickness)
{
	if (Points.Num() < 2) return;
	FSlateDrawElement::MakeLines(Out, Layer, G.ToPaintGeometry(), Points, ESlateDrawEffect::None, Color, true, Thickness);
}

void Box(FSlateWindowElementList& Out, int32 Layer, const FGeometry& G, const FVector2f& Pos, const FVector2f& Size, const FLinearColor& Color)
{
	FSlateDrawElement::MakeBox(Out, Layer, G.ToPaintGeometry(Size, FSlateLayoutTransform(Pos)), DroneUiStyle::WhiteBrush(), ESlateDrawEffect::None, Color);
}

void Text(FSlateWindowElementList& Out, int32 Layer, const FGeometry& G, const FVector2f& Pos, const FString& Str, const FSlateFontInfo& Font, const FLinearColor& Color)
{
	if (Str.IsEmpty()) return;
	FSlateDrawElement::MakeText(Out, Layer, G.ToPaintGeometry(G.GetLocalSize(), FSlateLayoutTransform(Pos)), Str, Font, ESlateDrawEffect::None, Color);
}

const FString& NumberLabel(int32 Value)
{
	// Allocates the first time a number is seen only; tape labels are integers in a small range.
	static TMap<int32, FString> Cache;
	if (const FString* Found = Cache.Find(Value)) return *Found;
	return Cache.Add(Value, FString::FromInt(Value));
}
}  // namespace DroneDraw

// ---------------------------------------------------------------------------------------------
// Horizon
// ---------------------------------------------------------------------------------------------
void SDroneHorizon::Construct(const FArguments& InArgs)
{
	Snap = InArgs._Snapshot;
	Scale = InArgs._UiScale > 0.0f ? InArgs._UiScale : 1.0f;
}

int32 SDroneHorizon::OnPaint(const FPaintArgs&, const FGeometry& G, const FSlateRect&, FSlateWindowElementList& Out, int32 Layer, const FWidgetStyle&, bool) const
{
	if (!Snap) return Layer;
	const FVector2f Size = G.GetLocalSize();
	const FVector2f C = Size * 0.5f;
	const float R = FMath::Min(Size.X, Size.Y) * 0.5f - 2.0f;
	const float PxPerRad = R * 2.2f;  // about 40 degrees across the instrument
	const double Roll = Snap->Smooth.Roll.X, Pitch = Snap->Smooth.Pitch.X;
	const int32 L = Layer + 1;

	Out.PushClip(FSlateClippingZone(G));  // TODO(HUMAN-VERIFY): a circular clip needs a mask; the square clip is used
	DroneUi::Point Left, Right;
	DroneUi::HorizonEnds(Roll, Pitch, PxPerRad, R * 1.6, &Left, &Right);
	DroneDraw::Line(Out, L, G, C + FVector2f(Left.X, Left.Y), C + FVector2f(Right.X, Right.Y), DroneUiStyle::Accent(), 2.0f);
	// Pitch ladder every 10 degrees (+-30): short bars parallel to the horizon.
	for (int32 Deg = -30; Deg <= 30; Deg += 10)
	{
		if (Deg == 0) continue;
		DroneUi::Point A, B;
		DroneUi::HorizonEnds(Roll, Pitch - Deg * DroneUi::kPi / 180.0, PxPerRad, R * 0.22, &A, &B);
		DroneDraw::Line(Out, L, G, C + FVector2f(A.X, A.Y), C + FVector2f(B.X, B.Y), DroneUiStyle::Muted(), 1.0f);
	}
	Out.PopClip();
	// Fixed aircraft symbol: wings and a centre dot.
	DroneDraw::Line(Out, L + 1, G, C + FVector2f(-R * 0.35f, 0), C + FVector2f(-R * 0.1f, 0), DroneUiStyle::Text(), 3.0f);
	DroneDraw::Line(Out, L + 1, G, C + FVector2f(R * 0.1f, 0), C + FVector2f(R * 0.35f, 0), DroneUiStyle::Text(), 3.0f);
	DroneDraw::Box(Out, L + 1, G, C - FVector2f(2, 2), FVector2f(4, 4), DroneUiStyle::Text());
	// Frame.
	DroneDraw::Line(Out, L + 1, G, FVector2f(0, 0), FVector2f(Size.X, 0), DroneUiStyle::Hairline(), 1.0f);
	DroneDraw::Line(Out, L + 1, G, FVector2f(0, Size.Y), FVector2f(Size.X, Size.Y), DroneUiStyle::Hairline(), 1.0f);
	return L + 1;
}

// ---------------------------------------------------------------------------------------------
// Tape
// ---------------------------------------------------------------------------------------------
void SDroneTape::Construct(const FArguments& InArgs)
{
	Snap = InArgs._Snapshot;
	bAlt = InArgs._bAltitude;
	Scale = InArgs._UiScale > 0.0f ? InArgs._UiScale : 1.0f;
}

int32 SDroneTape::OnPaint(const FPaintArgs&, const FGeometry& G, const FSlateRect&, FSlateWindowElementList& Out, int32 Layer, const FWidgetStyle&, bool) const
{
	if (!Snap) return Layer;
	const FVector2f Size = G.GetLocalSize();
	const float Cy = Size.Y * 0.5f;
	const double Value = bAlt ? Snap->Smooth.Altitude.X : Snap->Smooth.Speed.X;
	const double HalfSpan = bAlt ? 12.0 : 6.0;                       // metres or m/s visible each side
	const double Minor = bAlt ? 1.0 : 0.5;
	const double PxPerUnit = (Size.Y * 0.5) / HalfSpan;
	const int32 L = Layer + 1;
	DroneUi::Tick Ticks[48];
	const int32 N = DroneUi::ComputeTicks(Value, HalfSpan, Minor, bAlt ? 5 : 2, PxPerUnit, Ticks, 48);
	const float X0 = Size.X - 20.0f * Scale;  // ticks hug the right edge
	Out.PushClip(FSlateClippingZone(G));
	for (int32 I = 0; I < N; ++I)
	{
		const float Y = Cy + static_cast<float>(Ticks[I].OffsetPx);
		const float Len = (Ticks[I].bMajor ? 14.0f : 7.0f) * Scale;
		DroneDraw::Line(Out, L, G, FVector2f(X0 + 14.0f * Scale - Len, Y), FVector2f(X0 + 14.0f * Scale, Y), Ticks[I].bMajor ? DroneUiStyle::Text() : DroneUiStyle::Muted(), 1.0f);
		if (Ticks[I].bMajor)
		{
			DroneDraw::Text(Out, L, G, FVector2f(4.0f * Scale, Y - 9.0f * Scale), DroneDraw::NumberLabel(static_cast<int32>(FMath::RoundToInt(Ticks[I].Value))), DroneUiStyle::Font(0, false, Scale), DroneUiStyle::Muted());
		}
	}
	Out.PopClip();
	// Reading box at the centre.
	const FString& Reading = bAlt ? Snap->Text.Altitude : Snap->Text.Speed;
	DroneDraw::Box(Out, L + 1, G, FVector2f(0.0f, Cy - 14.0f * Scale), FVector2f(Size.X - 24.0f * Scale, 28.0f * Scale), DroneUiStyle::Rgb(DroneUi::kPanelTop, 0.95f));
	DroneDraw::Text(Out, L + 2, G, FVector2f(6.0f * Scale, Cy - 11.0f * Scale), Reading, DroneUiStyle::Font(2, true, Scale), DroneUiStyle::Accent());
	static const FString kAltTitle(TEXT("ALT m")), kSpeedTitle(TEXT("SPD m/s"));  // built once
	DroneDraw::Text(Out, L + 2, G, FVector2f(6.0f * Scale, 2.0f), bAlt ? kAltTitle : kSpeedTitle, DroneUiStyle::Font(0, false, Scale), DroneUiStyle::Muted());
	return L + 2;
}

// ---------------------------------------------------------------------------------------------
// Compass
// ---------------------------------------------------------------------------------------------
void SDroneCompass::Construct(const FArguments& InArgs)
{
	Snap = InArgs._Snapshot;
	Scale = InArgs._UiScale > 0.0f ? InArgs._UiScale : 1.0f;
	CirclePoints.Reserve(25);
}

int32 SDroneCompass::OnPaint(const FPaintArgs&, const FGeometry& G, const FSlateRect&, FSlateWindowElementList& Out, int32 Layer, const FWidgetStyle&, bool) const
{
	if (!Snap) return Layer;
	const FVector2f Size = G.GetLocalSize();
	const int32 L = Layer + 1;
	// Heading tape across the top: a tick every 5 degrees, a label every 30.
	const double Heading = Snap->Smooth.Heading.X;
	const float TapeY = 20.0f * Scale;
	const float PxPerDeg = (Size.X * 0.6f) / 90.0f;
	const float Cx = Size.X * 0.5f;
	Out.PushClip(FSlateClippingZone(G));
	for (int32 D = -50; D <= 50; D += 5)
	{
		const double Deg = FMath::RoundToDouble(Heading / 5.0) * 5.0 + D;
		const float X = Cx + static_cast<float>(Deg - Heading) * PxPerDeg;
		const int32 Wrapped = static_cast<int32>(DroneUi::WrapDeg360(Deg));
		const bool bMajor = (Wrapped % 30) == 0;
		DroneDraw::Line(Out, L, G, FVector2f(X, TapeY), FVector2f(X, TapeY - (bMajor ? 10.0f : 5.0f) * Scale), bMajor ? DroneUiStyle::Text() : DroneUiStyle::Muted(), 1.0f);
		if (bMajor) DroneDraw::Text(Out, L, G, FVector2f(X - 10.0f * Scale, TapeY + 2.0f), DroneDraw::NumberLabel(Wrapped), DroneUiStyle::Font(0, false, Scale), DroneUiStyle::Muted());
	}
	Out.PopClip();
	DroneDraw::Line(Out, L + 1, G, FVector2f(Cx, TapeY + 4.0f * Scale), FVector2f(Cx, TapeY - 14.0f * Scale), DroneUiStyle::Accent(), 2.0f);
	DroneDraw::Text(Out, L + 1, G, FVector2f(Cx + 8.0f * Scale, 0.0f), Snap->Text.Heading, DroneUiStyle::Font(1, true, Scale), DroneUiStyle::Text());

	// Wind arrow in a circle at the right: points where the wind blows TOWARD, relative to the nose (up).
	const float Radius = 26.0f * Scale;
	const FVector2f Centre(Size.X - Radius - 8.0f * Scale, Size.Y - Radius - 4.0f * Scale);
	CirclePoints.Reset();
	for (int32 I = 0; I <= 24; ++I)
	{
		const float A = static_cast<float>(I) * 2.0f * UE_PI / 24.0f;
		CirclePoints.Add(Centre + FVector2f(FMath::Sin(A), -FMath::Cos(A)) * Radius);
	}
	DroneDraw::Polyline(Out, L, G, CirclePoints, DroneUiStyle::Hairline(), 1.0f);
	const double ArrowDeg = DroneUi::WindArrowDeg(Snap->Raw.WindTowardDeg, Snap->Raw.HeadingDeg);
	const float Ar = static_cast<float>(ArrowDeg * DroneUi::kPi / 180.0);
	const FVector2f Dir(FMath::Sin(Ar), -FMath::Cos(Ar));
	const FVector2f Tip = Centre + Dir * Radius * 0.85f, Tail = Centre - Dir * Radius * 0.5f;
	const FVector2f Perp(-Dir.Y, Dir.X);
	DroneDraw::Line(Out, L + 1, G, Tail, Tip, DroneUiStyle::Accent(), 2.0f);
	DroneDraw::Line(Out, L + 1, G, Tip, Tip - Dir * 7.0f * Scale + Perp * 4.0f * Scale, DroneUiStyle::Accent(), 2.0f);
	DroneDraw::Line(Out, L + 1, G, Tip, Tip - Dir * 7.0f * Scale - Perp * 4.0f * Scale, DroneUiStyle::Accent(), 2.0f);
	DroneDraw::Text(Out, L + 1, G, FVector2f(8.0f * Scale, Size.Y - 30.0f * Scale), Snap->Text.Wind, DroneUiStyle::Font(1, false, Scale), DroneUiStyle::Text());
	return L + 1;
}

// ---------------------------------------------------------------------------------------------
// Motor bars
// ---------------------------------------------------------------------------------------------
void SDroneMotorBars::Construct(const FArguments& InArgs)
{
	Snap = InArgs._Snapshot;
	Scale = InArgs._UiScale > 0.0f ? InArgs._UiScale : 1.0f;
}

int32 SDroneMotorBars::OnPaint(const FPaintArgs&, const FGeometry& G, const FSlateRect&, FSlateWindowElementList& Out, int32 Layer, const FWidgetStyle&, bool) const
{
	if (!Snap) return Layer;
	static const FString kNames[4] = {FString(TEXT("FR")), FString(TEXT("FL")), FString(TEXT("RR")), FString(TEXT("RL"))};  // built once
	const FVector2f Size = G.GetLocalSize();
	const int32 L = Layer + 1;
	const float Gap = DroneUiStyle::Grid(1.0f);
	const float BarW = (Size.X - Gap * 5.0f) / 4.0f;
	const float BarH = Size.Y - 52.0f * Scale;
	for (int32 M = 0; M < 4; ++M)
	{
		const float X = Gap + M * (BarW + Gap);
		const double Fraction = FMath::Clamp(Snap->Smooth.Rpm[M].X, 0.0, 1.0);
		const DroneUi::Status St = DroneUi::StatusFromUtilisation(Fraction >= 0.98 ? 1.0 : Fraction);
		DroneDraw::Box(Out, L, G, FVector2f(X, 18.0f * Scale), FVector2f(BarW, BarH), DroneUiStyle::Hairline());
		const float FillH = static_cast<float>(Fraction) * BarH;
		DroneDraw::Box(Out, L + 1, G, FVector2f(X, 18.0f * Scale + BarH - FillH), FVector2f(BarW, FillH), DroneUiStyle::StatusColour(St));
		DroneDraw::Text(Out, L + 2, G, FVector2f(X, 0.0f), kNames[M], DroneUiStyle::Font(0, true, Scale), DroneUiStyle::Muted());
		DroneDraw::Text(Out, L + 2, G, FVector2f(X, 18.0f * Scale + BarH + 2.0f), Snap->Text.Rpm[M], DroneUiStyle::Font(0, false, Scale), DroneUiStyle::Text());
		DroneDraw::Text(Out, L + 2, G, FVector2f(X, 18.0f * Scale + BarH + 18.0f * Scale), Snap->Text.Thrust[M], DroneUiStyle::Font(0, false, Scale), DroneUiStyle::Muted());
	}
	return L + 2;
}

// ---------------------------------------------------------------------------------------------
// Chart
// ---------------------------------------------------------------------------------------------
void SDroneChart::Construct(const FArguments& InArgs)
{
	Snap = InArgs._Snapshot;
	Kind = static_cast<EKind>(InArgs._Kind);
	Title = InArgs._Title;
	Scale = InArgs._UiScale > 0.0f ? InArgs._UiScale : 1.0f;
	Points.Reserve(600);
}

int32 SDroneChart::OnPaint(const FPaintArgs&, const FGeometry& G, const FSlateRect&, FSlateWindowElementList& Out, int32 Layer, const FWidgetStyle&, bool) const
{
	if (!Snap) return Layer;
	const DroneUi::Ring<600>* Ring = Kind == EKind::Altitude ? &Snap->Charts.Altitude : Kind == EKind::Speed ? &Snap->Charts.Speed : Kind == EKind::BatteryA ? &Snap->Charts.BatteryA : &Snap->Charts.ThrottleMean;
	const FVector2f Size = G.GetLocalSize();
	const int32 L = Layer + 1;
	DroneDraw::Text(Out, L, G, FVector2f(0.0f, 0.0f), Title, DroneUiStyle::Font(0, false, Scale), DroneUiStyle::Muted());
	double Lo = 0.0, Hi = 1.0;
	if (!Ring->MinMax(&Lo, &Hi) || Ring->Size() < 2) return L;
	if (Hi - Lo < 1e-6) { Lo -= 0.5; Hi += 0.5; }
	const double Pad = 0.1 * (Hi - Lo);
	Lo -= Pad;
	Hi += Pad;
	const float Top = 18.0f * Scale, H = Size.Y - Top;
	Points.Reset();
	const int32 N = static_cast<int32>(Ring->Size());
	for (int32 I = 0; I < N; ++I)
	{
		const float X = Size.X * static_cast<float>(I) / static_cast<float>(FMath::Max(1, N - 1));
		const float Y = Top + H * (1.0f - static_cast<float>((Ring->At(static_cast<std::size_t>(I)) - Lo) / (Hi - Lo)));
		Points.Add(FVector2f(X, Y));
	}
	DroneDraw::Line(Out, L, G, FVector2f(0.0f, Top + H), FVector2f(Size.X, Top + H), DroneUiStyle::Hairline(), 1.0f);
	DroneDraw::Polyline(Out, L + 1, G, Points, DroneUiStyle::Accent(), 1.5f);
	return L + 1;
}

// ---------------------------------------------------------------------------------------------
// Margin row
// ---------------------------------------------------------------------------------------------
void SDroneMarginRow::Construct(const FArguments& InArgs)
{
	Scale = InArgs._UiScale > 0.0f ? InArgs._UiScale : 1.0f;
}

void SDroneMarginRow::Set(const FString& InLabel, double InMargin)
{
	Label = InLabel;
	Margin = InMargin;
	char Buf[24];
	DroneUi::FormatFixed(Buf, sizeof(Buf), InMargin >= 99.0 ? 99.0 : InMargin, 2, 5);
	ValueText = FString(UTF8_TO_TCHAR(Buf));
	Invalidate(EInvalidateWidgetReason::Paint);
}

int32 SDroneMarginRow::OnPaint(const FPaintArgs&, const FGeometry& G, const FSlateRect&, FSlateWindowElementList& Out, int32 Layer, const FWidgetStyle&, bool) const
{
	const FVector2f Size = G.GetLocalSize();
	const int32 L = Layer + 1;
	const DroneUi::Status St = DroneUi::StatusFromMargin(Margin);
	const float BarX = Size.X * 0.5f, BarW = Size.X * 0.32f, BarY = Size.Y * 0.5f - 3.0f;
	const float Fill = static_cast<float>(FMath::Clamp(Margin / 2.0, 0.0, 1.0)) * BarW;
	DroneDraw::Text(Out, L, G, FVector2f(0.0f, 2.0f), Label, DroneUiStyle::Font(0, false, Scale), DroneUiStyle::Text());
	DroneDraw::Box(Out, L, G, FVector2f(BarX, BarY), FVector2f(BarW, 6.0f), DroneUiStyle::Hairline());
	DroneDraw::Box(Out, L + 1, G, FVector2f(BarX, BarY), FVector2f(Fill, 6.0f), DroneUiStyle::StatusColour(St));
	DroneDraw::Text(Out, L + 1, G, FVector2f(BarX + BarW + 8.0f, 2.0f), ValueText, DroneUiStyle::Font(0, true, Scale), DroneUiStyle::StatusColour(St));
	return L + 1;
}
