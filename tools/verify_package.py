# Tier: F
"""Checks that a packaged Windows build of the demo contains what it needs at run time.

Usage (after File > Package Project > Windows, or RunUAT BuildCookRun):
  python tools/verify_package.py --package C:\\path\\to\\Packaged
It searches DIR for the staged `DroneData` folder and, next to it, checks:
  - every file in tools/demo_data_manifest.json ("data_files") is present under DroneData/<repo path> and its SHA-256
    equals the file in the repo (a stale copy fails);
  - Binaries/Win64/physics_core.dll exists (and equals physics_core/target/release/physics_core.dll when that file exists);
  - the optional files are reported but never fail the check.
Exit code 0 = everything required is there. Nothing is written.
"""
import argparse
import hashlib
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
MANIFEST = ROOT / "tools" / "demo_data_manifest.json"


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def find_project_dir(package: Path) -> Path | None:
    """The folder that holds DroneData (the packaged project folder, e.g. Windows/UAV_Simulator)."""
    for d in sorted(package.rglob("DroneData")):
        if d.is_dir():
            return d.parent
    return None


def check(package: Path, repo: Path = ROOT, manifest: Path = MANIFEST) -> list[str]:
    """Returns a list of problems (empty = OK)."""
    spec = json.loads(manifest.read_text(encoding="utf-8"))
    problems: list[str] = []
    project = find_project_dir(package)
    if project is None:
        return [f"no DroneData folder under {package}: the data files were not staged"]
    for rel in spec["data_files"]:
        staged = project / "DroneData" / rel
        source = repo / rel
        if not staged.is_file():
            problems.append(f"missing: DroneData/{rel}")
        elif source.is_file() and sha256(staged) != sha256(source):
            problems.append(f"stale (differs from the repo file): DroneData/{rel}")
    dll = project / "Binaries" / "Win64" / spec["dll"]
    if not dll.is_file():
        problems.append(f"missing: Binaries/Win64/{spec['dll']} (cargo build --release first, then package again)")
    else:
        built = repo / "physics_core" / "target" / "release" / spec["dll"]
        if built.is_file() and sha256(dll) != sha256(built):
            problems.append(f"stale: Binaries/Win64/{spec['dll']} differs from physics_core/target/release")
    return problems


def main(argv: list[str]) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--package", type=Path, required=True)
    args = ap.parse_args(argv)
    problems = check(args.package)
    spec = json.loads(MANIFEST.read_text(encoding="utf-8"))
    project = find_project_dir(args.package)
    if project is not None:
        for rel in spec.get("optional_data_files", []):
            print(f"optional {'present' if (project / 'DroneData' / rel).is_file() else 'absent '}: {rel}")
    for p in problems:
        print(f"PROBLEM: {p}")
    print("package OK" if not problems else f"{len(problems)} problem(s)")
    return 1 if problems else 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
