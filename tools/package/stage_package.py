# Tier: F
"""Stages the helper programs and their data into a packaged build, next to the executable, and writes a manifest of every file.

Run AFTER Unreal packaged the game (see UE5Project/PACKAGING_README.md):
  python tools/package/stage_package.py --package <packaged folder> --python-zip <python-3.12.x-embed-amd64.zip> \
         --wheels-dir <folder of downloaded wheels> --exe-dir <folder with run_campaign.exe and validate_models.exe> [--cfd-dir <dir>]
Dev mode (what the Linux tests use, NOT for a real package):
  python tools/package/stage_package.py --dest <folder> --dev-python <python> --exe-dir <dir> --exe-suffix ""

Result, under <project>/DroneData/helpers/ (or --dest/helpers/):
  python/ (embeddable zip, python312._pth with the site-packages line)  python/Lib/site-packages/ (the locked wheels, unpacked)
  repo/ (tools/parts, tools/report, parts_db, campaign, offline_pipeline/props, the golden builds incl. the demo session)
  bin/ (run_campaign, validate_models)   cfd/ (optional)   helpers.json (relative paths)   helpers_manifest.json (sha256 of every file)
Nothing is downloaded and nothing is installed: every input is checked against a pin (embeddable zip: tools/package/pins.json; wheels:
the hashes of tools/package/helpers_lock.txt). The output is deterministic (sorted, no clock, no machine path).
"""
from __future__ import annotations

import argparse
import hashlib
import json
import re
import shutil
import sys
import zipfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
PINS = Path(__file__).resolve().parent / "pins.json"
LOCK = Path(__file__).resolve().parent / "helpers_lock.txt"
MANIFEST_NAME = "helpers_manifest.json"
SCHEMA_VERSION = 1
DEFAULT_MAX_BYTES = 1024 ** 3
EXES = ("run_campaign", "validate_models")
EXE_BUILD_HINT = "cargo build --release --example run_campaign --example validate_models   (in physics_core\\, on Windows)"
# (source dir relative to the repo, destination under repo/). Everything else is left out.
TREES = [("tools/parts", "tools/parts"), ("tools/report", "tools/report"), ("parts_db", "parts_db"), ("campaign", "campaign"),
         ("offline_pipeline/props", "offline_pipeline/props")]
SKIP_DIR_NAMES = {"__pycache__", "source_cache", ".venv", "target", ".git", "node_modules"}
SKIP_FILE_NAMES = {"parts_index.sqlite"}
SKIP_SUFFIXES = (".pyc", ".pyo")
CFD_PATTERNS = ("*.onnx", "*.model.json", "geometry.json", "sweep_manifest.json", "cfd_sanity_report.json", "*.png")


class StageError(RuntimeError):
    """An input is missing, unpinned or does not match its pin. Nothing half-staged is left behind (the destination is checked first)."""


def sha256_bytes(b: bytes) -> str:
    return hashlib.sha256(b).hexdigest()


def sha256_file(p: Path) -> str:
    h = hashlib.sha256()
    with p.open("rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def wanted(rel: Path) -> bool:
    """Is this repo-relative path part of the allow-listed trees? (test files are left out: the package runs, it does not test)."""
    if any(part in SKIP_DIR_NAMES for part in rel.parts):
        return False
    if rel.name in SKIP_FILE_NAMES or rel.suffix in SKIP_SUFFIXES:
        return False
    if rel.name.startswith("test_") and rel.suffix == ".py":
        return False
    return True


def tree_files(repo: Path, src_rel: str) -> list[Path]:
    base = repo / src_rel
    if not base.is_dir():
        raise StageError(f"missing folder {src_rel} in the repository")
    return sorted(p for p in base.rglob("*") if p.is_file() and wanted(p.relative_to(repo)))


def lock_hashes(lock_text: str) -> set[str]:
    return set(re.findall(r"--hash=sha256:([0-9a-f]{64})", lock_text))


def check_pin(zip_path: Path, pins: dict) -> None:
    pin = pins["python_embeddable"]
    want = pin["sha256"]
    if not re.fullmatch(r"[0-9a-f]{64}", want):
        raise StageError(f"tools/package/pins.json has no sha256 for {pin['file']} yet (TODO(HUMAN-VERIFY)): download it, compare with python.org, write the hash into pins.json")
    if zip_path.name != pin["file"]:
        raise StageError(f"expected the file {pin['file']}, got {zip_path.name}")
    got = sha256_file(zip_path)
    if got != want:
        raise StageError(f"{zip_path.name}: sha256 {got} differs from the pin {want}")


def check_wheels(wheels_dir: Path, allowed: set[str]) -> list[Path]:
    wheels = sorted(wheels_dir.glob("*.whl"))
    if not wheels:
        raise StageError(f"no .whl files in {wheels_dir}: run the pip download command from the README first")
    for w in wheels:
        if sha256_file(w) not in allowed:
            raise StageError(f"{w.name} is not one of the files whose hash is in helpers_lock.txt (refusing an unlocked wheel)")
    return wheels


HELPER_SCRIPT_DIRS = ("../repo/tools/parts", "../repo/tools/report")   # relative to python/ (the ._pth file's folder)


def pth_text(version_tag: str) -> str:
    """python312._pth for the embeddable build. That build ignores PYTHONPATH, and a ._pth file replaces the default search path (it may also
    leave out the script's own folder), so everything the helpers import is written here: the stdlib zip, the locked packages (`import site`
    is needed for Lib/site-packages to be processed) and the two helper script folders (the helpers import each other by flat name)."""
    return f"python{version_tag}.zip\n.\nLib/site-packages\n" + "".join(d + "\n" for d in HELPER_SCRIPT_DIRS) + "\nimport site\n"


class Plan:
    """What will be written: relative path (under helpers/) -> bytes or a source Path."""

    def __init__(self) -> None:
        self.files: dict[str, Path | bytes] = {}

    def add(self, rel: str, source: Path | bytes) -> None:
        if rel in self.files:
            raise StageError(f"two inputs want to write {rel}")
        self.files[rel] = source

    def total_bytes(self) -> int:
        return sum(len(v) if isinstance(v, bytes) else v.stat().st_size for v in self.files.values())


def build_plan(repo: Path, *, exe_dir: Path, exe_suffix: str, python_zip: Path | None, wheels_dir: Path | None, dev_python: str | None,
               cfd_dir: Path | None, pins: dict, lock_text: str) -> Plan:
    plan = Plan()
    for src, dst in TREES:
        for p in tree_files(repo, src):
            plan.add(f"repo/{dst}/{p.relative_to(repo / src).as_posix()}", p)
    for name in EXES:
        exe = exe_dir / (name + exe_suffix)
        if not exe.is_file():
            raise StageError(f"missing {exe}: build it first: {EXE_BUILD_HINT}")
        plan.add(f"bin/{name}{exe_suffix}", exe)
    if cfd_dir is not None:
        found = sorted({p for pat in CFD_PATTERNS for p in cfd_dir.glob(pat) if p.is_file()})
        for p in found:
            plan.add(f"cfd/{p.name}", p)
    settings = {"python": dev_python if dev_python else "python/python.exe", "repo_root": "repo", "run_campaign_exe": f"bin/run_campaign{exe_suffix}",
                "validate_models_exe": f"bin/validate_models{exe_suffix}", "threads": 4, "git_hash": "package"}
    mode = "dev" if dev_python else "windows"
    if dev_python:
        if python_zip or wheels_dir:
            raise StageError("--dev-python cannot be combined with --python-zip / --wheels-dir")
    else:
        if python_zip is None or wheels_dir is None:
            raise StageError("a real package needs --python-zip and --wheels-dir (or --dev-python for a test stage)")
        check_pin(python_zip, pins)
        wheels = check_wheels(wheels_dir, lock_hashes(lock_text))
        tag = pins["python_embeddable"]["version_tag"]
        with zipfile.ZipFile(python_zip) as z:
            for info in sorted(z.infolist(), key=lambda i: i.filename):
                if info.is_dir() or info.filename.endswith("._pth"):
                    continue
                plan.add(f"python/{info.filename}", z.read(info.filename))
        plan.add(f"python/python{tag}._pth", pth_text(tag).encode("utf-8"))
        for w in wheels:
            with zipfile.ZipFile(w) as z:
                for info in sorted(z.infolist(), key=lambda i: i.filename):
                    if not info.is_dir():
                        plan.add(f"python/Lib/site-packages/{info.filename}", z.read(info.filename))
    plan.add("helpers.json", (json.dumps(settings, indent=2) + "\n").encode("utf-8"))
    plan.mode = mode   # type: ignore[attr-defined]
    return plan


def write_plan(plan: Plan, dest: Path) -> dict:
    helpers = dest / "helpers"
    if helpers.exists():
        raise StageError(f"{helpers} already exists: delete it (or stage into an empty package) so no stale file survives")
    entries = []
    for rel in sorted(plan.files):
        out = helpers / rel
        out.parent.mkdir(parents=True, exist_ok=True)
        src = plan.files[rel]
        if isinstance(src, bytes):
            out.write_bytes(src)
        else:
            shutil.copyfile(src, out)
            shutil.copymode(src, out)   # keeps the executable bit of a Linux test stage; on Windows the mode does not matter
        entries.append({"path": rel, "sha256": sha256_file(out), "size": out.stat().st_size})
    manifest = {"schema_version": SCHEMA_VERSION, "mode": getattr(plan, "mode", "windows"), "files": entries,
                "totals": {"files": len(entries), "bytes": sum(e["size"] for e in entries)}}
    (helpers / MANIFEST_NAME).write_text(json.dumps(manifest, indent=1, sort_keys=True) + "\n", encoding="utf-8", newline="\n")
    return manifest


def find_project_dir(package: Path) -> Path:
    for d in sorted(package.rglob("DroneData")):
        if d.is_dir():
            return d
    raise StageError(f"no DroneData folder under {package}: package the game in Unreal first (the staged data files create it)")


def main(argv: list[str]) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--package", type=Path, help="packaged folder (contains DroneData)")
    ap.add_argument("--dest", type=Path, help="stage into <dest>/helpers directly (tests)")
    ap.add_argument("--python-zip", type=Path)
    ap.add_argument("--wheels-dir", type=Path)
    ap.add_argument("--dev-python", help="path of a Python to use as is (test stage only)")
    ap.add_argument("--exe-dir", type=Path, required=True)
    ap.add_argument("--exe-suffix", default=".exe")
    ap.add_argument("--cfd-dir", type=Path)
    ap.add_argument("--repo", type=Path, default=ROOT, help=argparse.SUPPRESS)
    ap.add_argument("--max-bytes", type=int, default=DEFAULT_MAX_BYTES)
    a = ap.parse_args(argv)
    try:
        if bool(a.package) == bool(a.dest):
            raise StageError("give exactly one of --package and --dest")
        dest = find_project_dir(a.package) if a.package else a.dest
        pins = json.loads(PINS.read_text(encoding="utf-8"))
        plan = build_plan(a.repo, exe_dir=a.exe_dir, exe_suffix=a.exe_suffix, python_zip=a.python_zip, wheels_dir=a.wheels_dir, dev_python=a.dev_python,
                          cfd_dir=a.cfd_dir, pins=pins, lock_text=LOCK.read_text(encoding="utf-8"))
        if plan.total_bytes() > a.max_bytes:
            raise StageError(f"the helpers would be {plan.total_bytes()} bytes, above --max-bytes {a.max_bytes}: something unexpected is in an allow-listed folder")
        manifest = write_plan(plan, dest)
    except (StageError, OSError, zipfile.BadZipFile, json.JSONDecodeError) as e:
        print(f"STAGE STOPPED: {e}", file=sys.stderr)
        return 1
    print(f"staged {manifest['totals']['files']} files, {manifest['totals']['bytes']} bytes, mode {manifest['mode']}, into {dest / 'helpers'}")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
