# Tier: D
"""Model promotion manifest: `offline_pipeline/trained_models/active_model.json`.

Completion Plan WP-10.4 / exit gate 12: a model reaches the build only through a reviewed pull
request that changes this manifest. This tool writes one role's entry and checks the whole
manifest. It does not open a PR, copy files or touch Unreal; a human reviews and merges.

Entry fields: path (repo-relative, under offline_pipeline/trained_models/), sha256 (of the .onnx
bytes), kind ("stub" or "trained"), geometry_hash and config_hash (SHA-256 of the raw bytes of
current_geometry.json and vehicle_config.json, same method as build.rs), holdout_error.
`holdout_error` is null for a stub and a finite number >= 0 for a trained model.
TODO(HUMAN-VERIFY: WP-6/WP-7): the holdout metric name, unit and limit are not defined yet.

Usage (repo root, inside .venv):
  python offline_pipeline/promote_model.py --role aero --model offline_pipeline/trained_models/aero_stub.onnx --allow-stub
  python offline_pipeline/promote_model.py --check
Exit code 0 = ok, 1 = check failed or bad input.
"""
import argparse
import hashlib
import json
import math
import sys
from pathlib import Path, PurePosixPath
from typing import Callable

ROLES = ("aero", "fea")
KINDS = ("stub", "trained")
ENTRY_KEYS = {"path", "sha256", "kind", "geometry_hash", "config_hash", "holdout_error"}
MODEL_DIR = "offline_pipeline/trained_models"
MANIFEST_REL = f"{MODEL_DIR}/active_model.json"
GEOMETRY_REL = "offline_pipeline/geometry/current_geometry.json"
CONFIG_REL = "offline_pipeline/geometry/vehicle_config.json"
SCHEMA_VERSION = 1

MetaReader = Callable[[Path], dict]


def sha256_file(path: Path) -> str:
    """SHA-256 hex digest of the raw bytes of a file."""
    return hashlib.sha256(path.read_bytes()).hexdigest()


def canonical_json(obj: dict) -> str:
    """Sorted keys, 2-space indent, LF, trailing newline, no NaN/Inf."""
    return json.dumps(obj, indent=2, sort_keys=True, allow_nan=False) + "\n"


def read_onnx_metadata(path: Path) -> dict:
    """Returns {key: value} from the ONNX `metadata_props` (imports onnx only when called)."""
    import onnx  # local import: the pure checks below do not need it

    model = onnx.load(str(path), load_external_data=False)
    return {p.key: p.value for p in model.metadata_props}


def _is_number(x: object) -> bool:
    return isinstance(x, (int, float)) and not isinstance(x, bool)


def check_path_field(path: object) -> str | None:
    """A manifest path must be a repo-relative posix path to an .onnx under MODEL_DIR."""
    if not isinstance(path, str) or not path:
        return "path must be a non-empty string"
    p = PurePosixPath(path)
    if p.is_absolute() or ".." in p.parts or "\\" in path:
        return f"path {path!r} must be relative, posix, without '..'"
    if not path.startswith(MODEL_DIR + "/") or p.suffix != ".onnx":
        return f"path {path!r} must be an .onnx file under {MODEL_DIR}/"
    return None


def check_entry_fields(entry: object) -> list[str]:
    """Checks the shape and value rules that need no files."""
    if not isinstance(entry, dict):
        return ["entry must be an object"]
    errs = []
    if set(entry) != ENTRY_KEYS:
        extra, missing = sorted(set(entry) - ENTRY_KEYS), sorted(ENTRY_KEYS - set(entry))
        return [f"entry keys differ: extra {extra}, missing {missing}"]
    if (e := check_path_field(entry["path"])) is not None:
        errs.append(e)
    for key in ("sha256", "geometry_hash", "config_hash"):
        v = entry[key]
        if not (isinstance(v, str) and len(v) == 64 and all(c in "0123456789abcdef" for c in v)):
            errs.append(f"{key} must be 64 lowercase hex characters")
    kind, holdout = entry["kind"], entry["holdout_error"]
    if kind not in KINDS:
        errs.append(f"kind {kind!r} must be one of {KINDS}")
    elif kind == "stub" and holdout is not None:
        errs.append("a stub must have holdout_error null (it was not validated)")
    elif kind == "trained" and not (_is_number(holdout) and math.isfinite(holdout) and holdout >= 0):
        errs.append("a trained model needs a finite holdout_error >= 0")
    return errs


def check_entry(role: str, entry: object, root: Path, read_meta: MetaReader) -> list[str]:
    """Full check of one role: fields, file hash, ONNX metadata, current geometry/config hashes."""
    errs = [f"{role}: {e}" for e in check_entry_fields(entry)]
    if errs:
        return errs
    assert isinstance(entry, dict)
    model = root / entry["path"]
    if not model.is_file():
        return [f"{role}: model file {entry['path']} does not exist"]
    if sha256_file(model) != entry["sha256"]:
        errs.append(f"{role}: sha256 of {entry['path']} differs from the manifest")
    try:
        meta = read_meta(model)
    except Exception as exc:  # noqa: BLE001 - any parse failure is a failed check, never a crash
        return errs + [f"{role}: cannot read ONNX metadata of {entry['path']}: {type(exc).__name__}"]
    for key in ("kind", "geometry_hash", "config_hash"):
        if meta.get(key) != entry[key]:
            errs.append(f"{role}: ONNX metadata {key}={meta.get(key)!r} differs from manifest {entry[key]!r}")
    for key, rel in (("geometry_hash", GEOMETRY_REL), ("config_hash", CONFIG_REL)):
        current = sha256_file(root / rel)
        if entry[key] != current:
            errs.append(f"{role}: {key} differs from the current {rel} ({current[:8]}...)")
    return errs


def check_manifest(manifest: object, root: Path, read_meta: MetaReader) -> list[str]:
    """Checks the whole manifest object (not its text form)."""
    if not isinstance(manifest, dict) or set(manifest) != {"schema_version", "models"}:
        return ["manifest must have exactly the keys schema_version and models"]
    if manifest["schema_version"] != SCHEMA_VERSION or isinstance(manifest["schema_version"], bool):
        return [f"schema_version must be {SCHEMA_VERSION}"]
    models = manifest["models"]
    if not isinstance(models, dict) or set(models) != set(ROLES):
        return [f"models must have exactly the roles {list(ROLES)}"]
    errs: list[str] = []
    for role in ROLES:
        errs += check_entry(role, models[role], root, read_meta)
    return errs


def build_entry(model: Path, root: Path, read_meta: MetaReader, allow_stub: bool,
                holdout_error: float | None) -> dict:
    """Builds a manifest entry from a model file. Raises ValueError with a clear reason."""
    try:
        rel = model.resolve().relative_to(root.resolve()).as_posix()
    except ValueError as exc:
        raise ValueError(f"{model} is not inside the repo {root}") from exc
    if (e := check_path_field(rel)) is not None:
        raise ValueError(e)
    if not model.is_file():
        raise ValueError(f"{model} does not exist")
    meta = read_meta(model)
    missing = [k for k in ("kind", "geometry_hash", "config_hash") if k not in meta]
    if missing:
        raise ValueError(f"ONNX metadata lacks {missing}; the runtime would refuse this model")
    kind = meta["kind"]
    if kind == "stub" and not allow_stub:
        raise ValueError("model is kind=stub; pass --allow-stub to record it (it stays a stub)")
    if kind == "stub" and holdout_error is not None:
        raise ValueError("a stub has no holdout error; omit --holdout-error")
    entry = {"path": rel, "sha256": sha256_file(model), "kind": kind,
             "geometry_hash": meta["geometry_hash"], "config_hash": meta["config_hash"],
             "holdout_error": holdout_error}
    errs = check_entry_fields(entry)
    if errs:
        raise ValueError("; ".join(errs))
    return entry


def load_manifest(path: Path) -> dict:
    if not path.is_file():
        return {"schema_version": SCHEMA_VERSION, "models": {}}
    return json.loads(path.read_text(encoding="utf-8"))


def check_file(root: Path, read_meta: MetaReader) -> list[str]:
    """Checks active_model.json on disk, including its canonical text form."""
    path = root / MANIFEST_REL
    if not path.is_file():
        return [f"{MANIFEST_REL} does not exist"]
    text = path.read_text(encoding="utf-8")
    try:
        manifest = json.loads(text)
    except json.JSONDecodeError as exc:
        return [f"{MANIFEST_REL} is not valid JSON: {exc}"]
    errs = check_manifest(manifest, root, read_meta)
    if not errs and text != canonical_json(manifest):
        errs.append(f"{MANIFEST_REL} is not in canonical form (sorted keys, indent 2, LF, final newline)")
    return errs


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument("--repo-root", default=str(Path(__file__).resolve().parent.parent))
    ap.add_argument("--check", action="store_true", help="check the manifest and exit")
    ap.add_argument("--role", choices=ROLES)
    ap.add_argument("--model", help="path to the .onnx file to promote")
    ap.add_argument("--allow-stub", action="store_true")
    ap.add_argument("--holdout-error", type=float, default=None)
    args = ap.parse_args(argv)
    root = Path(args.repo_root)
    if args.check:
        errs = check_file(root, read_onnx_metadata)
        for e in errs:
            print(f"ERROR: {e}")
        if not errs:
            manifest = json.loads((root / MANIFEST_REL).read_text(encoding="utf-8"))
            for role in ROLES:
                if manifest["models"][role]["kind"] == "stub":
                    print(f"WARNING: {role} is kind=stub (not validated; runtime needs allow_stub_models)")
            print("active_model.json ok")
        return 1 if errs else 0
    if not (args.role and args.model):
        ap.error("give --check, or both --role and --model")
    try:
        entry = build_entry(Path(args.model), root, read_onnx_metadata, args.allow_stub, args.holdout_error)
        for key, rel in (("geometry_hash", GEOMETRY_REL), ("config_hash", CONFIG_REL)):
            if entry[key] != sha256_file(root / rel):
                raise ValueError(f"model {key} differs from the current {rel}; retrain first")
    except ValueError as exc:
        print(f"ERROR: {exc}")
        return 1
    path = root / MANIFEST_REL
    manifest = load_manifest(path)
    manifest["models"][args.role] = entry
    path.write_text(canonical_json(manifest), encoding="utf-8", newline="\n")
    print(f"wrote {args.role} entry to {MANIFEST_REL} (not committed; open a PR for review)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
