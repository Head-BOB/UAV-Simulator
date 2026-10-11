# Tier: V
"""Tripwires for the blind-written Unreal UI layer (a tripwire, not proof; nothing here compiles Unreal code).

1. OnPaint of the canvas widget must not allocate: no NewObject / MakeShared / MakeShareable / FString construction or Printf / `new` in its body.
2. Every Unreal UI file starts with a Tier header and says it was NOT COMPILED OR RUN, so nobody mistakes it for a tested file.
3. The UI layer must not convert frames itself: the NED to Unreal conversion lives in DroneNedToUnreal.h / the bridge (no "-Z" style flips by hand).
Usage: python tools/check_ui_tripwires.py [--root DIR]
"""
import argparse
import re
import sys
from pathlib import Path

FORBIDDEN_IN_PAINT = ("NewObject", "MakeShared", "MakeShareable", "FString::Printf", "FString(", "FString Str", " new ")
NEW_FILES = ["DroneAppHUD.h", "DroneAppHUD.cpp", "DroneBuildPreview.h", "DroneBuildPreview.cpp", "UI/DroneAppIO.h", "UI/DroneAppIO.cpp", "UI/SDroneAppCanvas.h", "UI/SDroneAppCanvas.cpp"]
HAND_CONVERSION = re.compile(r"\bNedZ\b|ned\w*\[2\]\s*\*\s*-1|-\s*ned\w*\[2\]|\.Z\s*=\s*-\s*\w*[Nn]ed", re.IGNORECASE)


def function_body(text: str, signature: str) -> str | None:
    """Text between the braces of the first function whose definition line contains `signature`."""
    i = text.find(signature)
    if i < 0:
        return None
    j = text.find("{", i)
    if j < 0:
        return None
    depth, k = 0, j
    while k < len(text):
        if text[k] == "{":
            depth += 1
        elif text[k] == "}":
            depth -= 1
            if depth == 0:
                return text[j:k + 1]
        k += 1
    return None


def check(root: Path) -> list[str]:
    src = root / "UE5Project" / "Source" / "UAV_Simulator"
    errors = []
    canvas = src / "UI" / "SDroneAppCanvas.cpp"
    body = function_body(canvas.read_text(encoding="utf-8"), "int32 SDroneAppCanvas::OnPaint(") if canvas.exists() else None
    if body is None:
        errors.append("SDroneAppCanvas::OnPaint not found")
    else:
        for bad in FORBIDDEN_IN_PAINT:
            if bad in body:
                errors.append(f"OnPaint contains {bad!r} (painting must not allocate; cache conversions per display-list version)")
    for rel in NEW_FILES:
        p = src / rel
        if not p.exists():
            errors.append(f"{rel}: missing")
            continue
        text = p.read_text(encoding="utf-8")
        first = text.splitlines()[0] if text else ""
        if not first.startswith("// Tier:"):
            errors.append(f"{rel}: first line must be a Tier header")
        if "NOT COMPILED OR RUN" not in text:
            errors.append(f"{rel}: must say NOT COMPILED OR RUN")
        if HAND_CONVERSION.search(text):
            errors.append(f"{rel}: looks like a hand-written NED to Unreal conversion (use DroneNedToUnreal.h / the bridge)")
    return errors


def main(argv: list) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--root", type=Path, default=Path(__file__).resolve().parents[1])
    a = ap.parse_args(argv)
    errors = check(a.root)
    for e in errors:
        print(f"ERROR: {e}")
    print(f"ui tripwires: {len(errors)} errors")
    return 1 if errors else 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
