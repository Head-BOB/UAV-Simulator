# Tier: F
"""Fails if non-test Rust code binds or compares a mass/inertia identifier with a numeric literal.

Completion Plan WP-2 "Done when": no mass or inertia literal remains in the source. Mass and inertia
come from the hashed VehicleConfig only. The search covers `physics_core/src/**/*.rs`, up to the
`#[cfg(test)]` line, which must introduce `mod tests` (otherwise the file is rejected, because the
cut point would be unclear). Comments and string literals are removed first.
Flagged: an identifier containing `mass` or `inertia` (any case), optionally with a type (`: f64 =`), followed by
`:`, `=`, `==`, `<`, `>`, `<=` or `>=` and then a number, optionally after `[`, `(` or `-`.
Not flagged (known limits): a literal reached through arithmetic (`m * 2.0`), a function argument,
or a name that does not contain mass/inertia. This is a tripwire, not a proof.
Usage: python tools/check_mass_literals.py [--root DIR]
"""
import argparse
import re
import sys
from pathlib import Path

PATTERN = re.compile(
    r"\b\w*(?:mass|inertia)\w*\s*(?:(?:==|<=|>=|=|:|<|>)|:\s*[\w\[\]<>;&' ]+?=)\s*[\[\(]?\s*-?\s*\d",
    re.IGNORECASE,
)
CFG_TEST = re.compile(r"#\[cfg\(test\)\]")
MOD_TESTS = re.compile(r"#\[cfg\(test\)\]\s*mod\s+tests\b")


def strip_comments_and_strings(text: str) -> str:
    """Replaces comments and string literals by spaces, keeping newlines so line numbers hold."""
    out = []
    i, n = 0, len(text)
    while i < n:
        two = text[i : i + 2]
        if two == "//":
            while i < n and text[i] != "\n":
                out.append(" ")
                i += 1
        elif two == "/*":
            depth = 0
            while i < n:
                if text[i : i + 2] == "/*":
                    depth += 1
                    out.append("  ")
                    i += 2
                elif text[i : i + 2] == "*/":
                    depth -= 1
                    out.append("  ")
                    i += 2
                    if depth == 0:
                        break
                else:
                    out.append("\n" if text[i] == "\n" else " ")
                    i += 1
        elif text[i] == '"':
            out.append('"')
            i += 1
            while i < n and text[i] != '"':
                if text[i] == "\\" and i + 1 < n:
                    out.append(" ")
                    i += 1
                out.append("\n" if text[i] == "\n" else " ")
                i += 1
            out.append('"')
            i += 1
        else:
            out.append(text[i])
            i += 1
    return "".join(out)


def check_text(rel_path: str, text: str) -> list[str]:
    """Returns error messages for one Rust file."""
    count = len(CFG_TEST.findall(text))
    if count > 1 or (count == 1 and not MOD_TESTS.search(text)):
        return [f"{rel_path}: #[cfg(test)] must appear once, directly before 'mod tests'"]
    cut = CFG_TEST.search(text)
    body = text[: cut.start()] if cut else text
    clean = strip_comments_and_strings(body)
    errors = []
    for lineno, line in enumerate(clean.split("\n"), start=1):
        m = PATTERN.search(line)
        if m:
            errors.append(f"{rel_path}:{lineno}: mass/inertia literal: {m.group(0).strip()!r}")
    return errors


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--root", default=str(Path(__file__).resolve().parent.parent))
    args = ap.parse_args()
    root = Path(args.root)
    files = sorted((root / "physics_core" / "src").rglob("*.rs"))
    if not files:
        print("ERROR: no Rust files found; wrong --root?")
        return 1
    errors = []
    for p in files:
        errors += check_text(p.relative_to(root).as_posix(), p.read_text(encoding="utf-8"))
    for e in errors:
        print(e)
    print(f"{len(files)} files checked, {len(errors)} errors")
    return 1 if errors else 0


if __name__ == "__main__":
    sys.exit(main())
