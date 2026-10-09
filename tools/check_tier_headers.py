# Tier: F
"""Fails if a source file lacks its `Tier:` header line (Completion Plan WP-10.11).

Rust / C++ files: the first line is `// Tier: X`.
Python files: the first line is `# Tier: X`; if line 1 is a `#!` shebang, line 2 is.
X is one of F, D, V, H (Standards Framework section 3.3). Required tier by path:
`physics_core/` is F and `offline_pipeline/` is D; elsewhere any valid tier is accepted.
The tier assignments in the headers are an AI draft until a human reviews them (HUMAN-TODO).
Usage: python tools/check_tier_headers.py [--root DIR]
"""
import argparse
import re
import sys
from pathlib import Path

VALID_TIERS = {"F", "D", "V", "H"}
SKIP_DIRS = {"__pycache__", ".venv", "target", "Generated", "Binaries", "Intermediate",
             "Saved", "DerivedDataCache", ".git", "trained_models", "node_modules"}
# (glob relative to root, comment prefix)
SCOPES = [
    ("physics_core/src/**/*.rs", "//"),
    ("physics_core/build.rs", "//"),
    ("UE5Project/Source/**/*.h", "//"),
    ("UE5Project/Source/**/*.cpp", "//"),
    ("UE5Project/Plugins/*/Source/**/*.h", "//"),
    ("UE5Project/Plugins/*/Source/**/*.cpp", "//"),
    ("tools/**/*.py", "#"),
    ("tools/**/*.cpp", "//"),
    ("scripts/**/*.py", "#"),
    ("offline_pipeline/**/*.py", "#"),
]
REQUIRED_BY_PREFIX = [("physics_core/", "F"), ("offline_pipeline/", "D")]


def tier_line_index(lines: list[str], prefix: str) -> int:
    """Index of the line that must hold the tag (1 for a Python shebang file, else 0)."""
    if prefix == "#" and lines and lines[0].startswith("#!"):
        return 1
    return 0


def check_text(rel_path: str, text: str, prefix: str) -> str | None:
    """Returns an error message, or None when the header is valid."""
    lines = text.split("\n")
    idx = tier_line_index(lines, prefix)
    line = lines[idx].rstrip("\r") if idx < len(lines) else ""
    m = re.fullmatch(re.escape(prefix) + r" Tier: ([A-Z])", line)
    if not m:
        return f"{rel_path}: line {idx + 1} must be '{prefix} Tier: X' (X in F, D, V, H), found {line!r}"
    tier = m.group(1)
    if tier not in VALID_TIERS:
        return f"{rel_path}: tier {tier!r} is not one of F, D, V, H"
    for start, required in REQUIRED_BY_PREFIX:
        if rel_path.startswith(start) and tier != required:
            return f"{rel_path}: path requires Tier {required}, found Tier {tier}"
    return None


def collect_files(root: Path) -> list[tuple[str, str]]:
    found: dict[str, str] = {}
    for pattern, prefix in SCOPES:
        for p in sorted(root.glob(pattern)):
            rel = p.relative_to(root)
            if p.is_file() and not (set(rel.parts) & SKIP_DIRS):
                found[rel.as_posix()] = prefix
    return sorted(found.items())


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--root", default=str(Path(__file__).resolve().parent.parent))
    args = ap.parse_args()
    root = Path(args.root)
    files = collect_files(root)
    if not files:
        print("ERROR: no source files found; wrong --root?")
        return 1
    errors = []
    for rel, prefix in files:
        err = check_text(rel, (root / rel).read_text(encoding="utf-8"), prefix)
        if err:
            errors.append(err)
    for e in errors:
        print(e)
    print(f"{len(files)} files checked, {len(errors)} errors")
    return 1 if errors else 0


if __name__ == "__main__":
    sys.exit(main())
