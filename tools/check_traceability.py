# Tier: F
# tools/check_traceability.py - fails CI if TRACEABILITY.md is not honest in form.
# Checks: table format, pass rule present, status valid, every named test exists,
# every existing test is named. It cannot check that a test asserts its pass rule.
import argparse
import re
import subprocess
import sys

HEADER = (
    "| Requirement | Source | Verified By | Pass rule (tolerance) | Status | Notes |"
)
STATUSES = {"PASS", "UNVERIFIED", "FAIL"}
COLUMNS = 6
TEST_RE = re.compile(r"`((?:[a-z_0-9]+::)*tests::[a-z0-9_]+)`")


def parse_rows(text):
    """Return (rows, errors). A row is (test, pass_rule, status)."""
    lines = text.splitlines()
    try:
        start = lines.index(HEADER)
    except ValueError:
        return [], ["TRACEABILITY table header not found (expected exact header line)"]
    rows, errors = [], []
    for line in lines[start + 2 :]:
        if not line.startswith("|"):
            break
        cells = [c.strip() for c in line.strip().strip("|").split(" | ")]
        if len(cells) != COLUMNS:
            errors.append(f"row has {len(cells)} columns, expected {COLUMNS}: {line[:60]}")
            continue
        found = TEST_RE.findall(cells[2])
        if len(found) != 1:
            errors.append(f"row must name exactly one test in Verified By: {cells[0][:50]}")
            continue
        test, rule, status = found[0], cells[3], cells[4]
        if rule in ("", "-"):
            errors.append(f"{test}: empty pass rule")
        if status not in STATUSES:
            errors.append(f"{test}: status '{status}' not in {sorted(STATUSES)}")
        rows.append((test, rule, status))
    return rows, errors


def check(text, existing):
    rows, errors = parse_rows(text)
    named = [r[0] for r in rows]
    for name in sorted(set(named)):
        if name not in existing:
            errors.append(f"{name}: named in TRACEABILITY but does not exist")
        if named.count(name) > 1:
            errors.append(f"{name}: named in more than one row")
    for name in sorted(existing - set(named)):
        errors.append(f"{name}: exists but is not named in TRACEABILITY")
    return errors


def list_tests(manifest):
    proc = subprocess.run(
        ["cargo", "test", "--manifest-path", manifest, "--", "--list"],
        capture_output=True,
        text=True,
    )
    if proc.returncode != 0:
        print("cargo test --list failed:", proc.stderr[-2000:], file=sys.stderr)
        sys.exit(2)
    return {
        line.split(": ")[0]
        for line in proc.stdout.splitlines()
        if line.endswith(": test")
    }


def main(argv):
    parser = argparse.ArgumentParser()
    parser.add_argument("--matrix", default="physics_core/TRACEABILITY.md")
    parser.add_argument("--manifest", default="physics_core/Cargo.toml")
    args = parser.parse_args(argv)
    with open(args.matrix, encoding="utf-8") as f:
        text = f.read()
    errors = check(text, list_tests(args.manifest))
    if errors:
        print("TRACEABILITY check failed:", *errors, sep="\n ")
        return 1
    rows, _ = parse_rows(text)
    passed = sum(1 for r in rows if r[2] == "PASS")
    unverified = sum(1 for r in rows if r[2] == "UNVERIFIED")
    print(f"traceability OK: {len(rows)} rows, {passed} PASS, {unverified} UNVERIFIED")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
