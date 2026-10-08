#!/usr/bin/env python3
"""Automated metadata tagging and dataset placement script.

Computes SHA-256 provenance of the canonical geometry definition,
embeds design revision and provenance tags, and safely places raw solver
outputs into tagged results directories with atomic rollback semantics.

Reference: UAV_Simulator_Automated_Pipeline_Supply_Chain_Addendum Section A.2
"""

import argparse
from datetime import datetime, timezone
import hashlib
import json
import os
import shutil
import subprocess
import sys
from typing import Dict, List, Optional


def compute_file_sha256(path: str) -> str:
    """Compute SHA-256 digest of specified file."""
    hasher = hashlib.sha256()
    with open(path, "rb") as f:
        while chunk := f.read(65536):
            hasher.update(chunk)
    return hasher.hexdigest()


def get_git_sha() -> str:
    """Retrieve commit SHA from environment or local repository."""
    if sha := os.environ.get("GITHUB_SHA"):
        return sha
    try:
        res = subprocess.run(
            ["git", "rev-parse", "HEAD"],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            check=True,
        )
        return res.stdout.strip()
    except (subprocess.SubprocessError, FileNotFoundError):
        return "local_dev"


def tag_file_content(src_path: str, dst_path: str, metadata: Dict[str, str]) -> None:
    """Copy content from src to dst while embedding metadata stamps."""
    ext = os.path.splitext(src_path)[1].lower()

    if ext == ".json":
        with open(src_path, "r", encoding="utf-8") as f:
            try:
                data = json.load(f)
                if isinstance(data, dict):
                    data["_provenance"] = metadata
                    with open(dst_path, "w", encoding="utf-8") as out:
                        json.dump(data, out, indent=2)
                    return
            except json.JSONDecodeError:
                pass  # Fall through to raw copy

    if ext in [".csv", ".txt", ".dat"]:
        with open(src_path, "r", encoding="utf-8", errors="replace") as f:
            original = f.read()
        header = (
            f"# UAV_SIMULATOR PROVENANCE METADATA\n"
            f"# geometry_hash: {metadata['geometry_hash']}\n"
            f"# design_revision: {metadata['design_revision']}\n"
            f"# timestamp_utc: {metadata['timestamp_utc']}\n"
            f"# git_sha: {metadata['git_sha']}\n"
        )
        with open(dst_path, "w", encoding="utf-8") as out:
            out.write(header + original)
        return

    # Binary or other formats: binary copy
    shutil.copy2(src_path, dst_path)


def tag_and_place(
    input_path: str,
    output_path: str,
    geometry_path: str,
    git_sha: Optional[str] = None,
) -> None:
    """Process raw simulation files and place tagged artifacts into destination directory.

    Maintains transactional integrity: rollback on any mid-run failure.
    """
    if not os.path.exists(geometry_path):
        raise FileNotFoundError(f"Geometry definition not found at: {geometry_path}")

    geo_hash = compute_file_sha256(geometry_path)
    geohash8 = geo_hash[:8]

    with open(geometry_path, "r", encoding="utf-8") as f:
        geo_json = json.load(f)
    design_rev = str(geo_json.get("design_revision", "1"))

    now_utc = datetime.now(timezone.utc)
    date_str = now_utc.strftime("%Y%m%d")
    timestamp_utc = now_utc.isoformat()
    effective_git_sha = git_sha or get_git_sha()

    metadata = {
        "geometry_hash": geo_hash,
        "design_revision": design_rev,
        "timestamp_utc": timestamp_utc,
        "git_sha": effective_git_sha,
    }

    os.makedirs(output_path, exist_ok=True)

    # Determine files to process
    if os.path.isfile(input_path):
        source_files = [input_path]
    elif os.path.isdir(input_path):
        source_files = [
            os.path.join(input_path, f)
            for f in sorted(os.listdir(input_path))
            if os.path.isfile(os.path.join(input_path, f))
        ]
    else:
        raise FileNotFoundError(f"Input path not found: {input_path}")

    created_files: List[str] = []

    try:
        for src_file in source_files:
            base_name, ext = os.path.splitext(os.path.basename(src_file))
            tagged_name = f"{base_name}_{geohash8}_rev{design_rev}_{date_str}{ext}"
            dest_file = os.path.join(output_path, tagged_name)

            tag_file_content(src_file, dest_file, metadata)
            created_files.append(dest_file)
            print(f"Placed: {os.path.basename(src_file)} -> {tagged_name}")

        print(f"Successfully tagged and placed {len(created_files)} result file(s).")

    except Exception as err:
        print(f"ERROR during tag_and_place: {err}. Executing rollback...", file=sys.stderr)
        for created in created_files:
            if os.path.exists(created):
                try:
                    os.remove(created)
                    print(f"Rolled back: {created}", file=sys.stderr)
                except OSError:
                    pass
        raise


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Tag and place CFD/FEA simulation datasets with canonical provenance."
    )
    parser.add_argument(
        "--input",
        required=True,
        help="Path to raw simulation result file or directory.",
    )
    parser.add_argument(
        "--output",
        required=True,
        help="Destination directory for tagged datasets.",
    )
    parser.add_argument(
        "--geometry-hash-source",
        default="offline_pipeline/geometry/current_geometry.json",
        help="Path to canonical geometry JSON (default: offline_pipeline/geometry/current_geometry.json).",
    )
    parser.add_argument(
        "--git-sha",
        help="Explicit git SHA override (defaults to GITHUB_SHA or local HEAD).",
    )

    args = parser.parse_args()

    try:
        tag_and_place(
            input_path=args.input,
            output_path=args.output,
            geometry_path=args.geometry_hash_source,
            git_sha=args.git_sha,
        )
    except Exception as err:
        print(f"Execution failed: {err}", file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":
    main()
