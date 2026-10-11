# Tier: F
"""Checks that a packaged Windows build of the demo contains what it needs at run time.

Usage (after File > Package Project > Windows, or RunUAT BuildCookRun):
  python tools/verify_package.py --package C:\\path\\to\\Packaged
It searches DIR for the staged `DroneData` folder and, next to it, checks:
  - every file in tools/demo_data_manifest.json ("data_files") is present under DroneData/<repo path> and its SHA-256
    equals the file in the repo (a stale copy fails);
  - Binaries/Win64/physics_core.dll exists (and equals physics_core/target/release/physics_core.dll when that file exists);
  - the optional files are reported but never fail the check.
  - the staged helper programs (tools/package/stage_package.py): DroneData/helpers/helpers_manifest.json, every file in it present with the
    same size and SHA-256, no unlisted file, helpers.json pointing at existing files, and for a Windows package the embeddable Python
    (python.exe, a python*._pth with the site-packages line and `import site`, the locked packages) and the two .exe files, plus the demo session.
Without --require-helpers the helper checks run only when DroneData/helpers exists (so the old demo-only package still verifies).
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


HELPERS_DEMO_FILES = ["repo/tools/parts/builds/reference_quad/vehicle_build.json", "repo/tools/parts/builds/reference_quad/campaign_smoke/campaign_result.json",
                      "repo/tools/parts/builds/reference_quad/report_smoke/report.html"]


PACKAGE_PINS = ROOT / "tools" / "package" / "pins.json"


def check_onnxruntime(helpers: Path, rel: str, pins_path: Path = PACKAGE_PINS) -> list[str]:
    """The staged ONNX Runtime library must be the pinned DLL (tools/package/pins.json onnxruntime_win_x64.dll_sha256)."""
    f = helpers / rel
    if not f.is_file():
        return [f"missing: helpers/{rel} (helpers.json names it)"]
    try:
        want = json.loads(pins_path.read_text(encoding="utf-8"))["onnxruntime_win_x64"]["dll_sha256"]
    except (OSError, ValueError, KeyError) as e:
        return [f"cannot read the onnxruntime pin from tools/package/pins.json: {e}"]
    if sha256(f) != want:
        return [f"helpers/{rel} is not the pinned ONNX Runtime (sha256 differs from tools/package/pins.json)"]
    return []


def check_helpers(project: Path) -> list[str]:
    """Problems with DroneData/helpers (the staged helper programs); empty = OK. `project` is the folder that holds DroneData."""
    helpers = project / "DroneData" / "helpers"
    manifest_path = helpers / "helpers_manifest.json"
    if not manifest_path.is_file():
        return ["missing: DroneData/helpers/helpers_manifest.json (run tools/package/stage_package.py after packaging)"]
    try:
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
        entries = manifest["files"]
    except (ValueError, KeyError) as e:
        return [f"unreadable helpers_manifest.json: {e}"]
    problems: list[str] = []
    listed = set()
    for e in entries:
        rel = e["path"]
        listed.add(rel)
        f = helpers / rel
        if not f.is_file():
            problems.append(f"missing: helpers/{rel}")
        elif f.stat().st_size != e["size"] or sha256(f) != e["sha256"]:
            problems.append(f"stale or altered (differs from the manifest): helpers/{rel}")
    for f in sorted(helpers.rglob("*")):
        rel = f.relative_to(helpers).as_posix()
        if f.is_file() and rel not in listed and rel != "helpers_manifest.json":
            problems.append(f"unlisted file (not in the manifest): helpers/{rel}")
    try:
        settings = json.loads((helpers / "helpers.json").read_text(encoding="utf-8"))
    except (OSError, ValueError) as e:
        return problems + [f"unreadable helpers.json: {e}"]
    windows = manifest.get("mode") == "windows"
    ort_key = settings.get("onnxruntime")
    if ort_key:
        problems += check_onnxruntime(helpers, ort_key)
    if settings.get("cfd_dir") and not (helpers / settings["cfd_dir"]).is_dir():
        problems.append(f"helpers.json cfd_dir: {settings['cfd_dir']!r} does not exist")
    for key in ("python", "repo_root", "run_campaign_exe", "validate_models_exe"):
        target = Path(settings.get(key, ""))
        target = target if target.is_absolute() else helpers / target
        if not settings.get(key) or not target.exists():
            problems.append(f"helpers.json {key}: {settings.get(key)!r} does not exist")
        elif windows and key == "python" and Path(settings[key]).is_absolute():
            problems.append("helpers.json python is an absolute path in a Windows package (must be relative to helpers/)")
    if windows:
        pth = sorted((helpers / "python").glob("python*._pth"))
        if not pth:
            problems.append("missing: helpers/python/python*._pth")
        else:
            text = pth[0].read_text(encoding="utf-8")
            if "Lib/site-packages" not in text.splitlines() or "import site" not in text.splitlines():
                problems.append(f"{pth[0].name} lacks the 'Lib/site-packages' or 'import site' line (the embeddable Python ignores PYTHONPATH)")
        if not (helpers / "python" / "Lib" / "site-packages" / "jsonschema").is_dir():
            problems.append("missing: helpers/python/Lib/site-packages/jsonschema (the locked wheels were not unpacked)")
        for exe in ("run_campaign.exe", "validate_models.exe"):
            if not (helpers / "bin" / exe).is_file():
                problems.append(f"missing: helpers/bin/{exe}")
    for rel in HELPERS_DEMO_FILES:
        if not (helpers / rel).is_file():
            problems.append(f"missing demo session file: helpers/{rel}")
    return problems


def require_onnxruntime(project: Path) -> list[str]:
    """A WINDOWS helpers package must ship onnxruntime.dll: without it a CFD model cannot be loaded by the helpers."""
    helpers = project / "DroneData" / "helpers"
    try:
        windows = json.loads((helpers / "helpers_manifest.json").read_text(encoding="utf-8")).get("mode") == "windows"
        declared = json.loads((helpers / "helpers.json").read_text(encoding="utf-8")).get("onnxruntime")
    except (OSError, ValueError):
        return []   # check_helpers already reports unreadable files
    if windows and not declared:
        return ["missing: helpers/bin/onnxruntime.dll (stage with --onnxruntime-zip; see UE5Project/PACKAGING_README.md)"]
    return []


def main(argv: list[str]) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--package", type=Path, required=True)
    ap.add_argument("--require-helpers", action="store_true", help="the staged helper programs must be there (the README uses this); without it they are checked only if DroneData/helpers exists")
    args = ap.parse_args(argv)
    problems = check(args.package)
    spec = json.loads(MANIFEST.read_text(encoding="utf-8"))
    project = find_project_dir(args.package)
    if project is not None and (args.require_helpers or (project / "DroneData" / "helpers").exists()):
        problems += check_helpers(project)
        problems += require_onnxruntime(project)
        for rel in spec.get("optional_data_files", []):
            print(f"optional {'present' if (project / 'DroneData' / rel).is_file() else 'absent '}: {rel}")
    for p in problems:
        print(f"PROBLEM: {p}")
    print("package OK" if not problems else f"{len(problems)} problem(s)")
    return 1 if problems else 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
