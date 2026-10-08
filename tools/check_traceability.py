# tools/check_traceability.py - fails CI if a named test does not exist
import re, subprocess, sys

text = open("physics_core/TRACEABILITY.md").read()
named = set(re.findall(r"`([a-z_]+::tests::[a-z0-9_]+)`", text))

listing = subprocess.run(
    ["cargo", "test", "--manifest-path", "physics_core/Cargo.toml", "--", "--list"],
    capture_output=True, text=True
).stdout

existing = {l.split(": ")[0] for l in listing.splitlines() if l.endswith(": test")}
missing = sorted(n for n in named if n not in existing)

if missing:
    print("TRACEABILITY names tests that do not exist:", *missing, sep="\n ")
    sys.exit(1)

print("traceability OK:", len(named), "tests found")