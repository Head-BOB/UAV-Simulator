# Tier: F
"""Downloads the pinned ONNX Runtime release asset and checks its SHA-256 before use.

Usage: python tools/fetch_onnx.py [--platform linux|windows] [--out-dir DIR]
  linux   (default) downloads the .tgz, verifies it and extracts it into the current folder (CI).
  windows downloads the .zip and verifies it only (nothing is extracted: tools/package/stage_package.py takes onnxruntime.dll from it).

Why 1.27.1: `ort 2.0.0-rc.13` (feature api-27, the crate default) is built for ONNX Runtime API 1.27 and refuses anything older
(ort-2.0.0-rc.13/src/lib.rs, load_dynamic::init: the minor version of GetVersionString() must be >= 27). 1.27.1 is the newest 1.27 patch.
"""
from __future__ import annotations

import argparse
import hashlib
import os
import sys
import tarfile
import urllib.request

VERSION = "1.27.1"
BASE_URL = f"https://github.com/microsoft/onnxruntime/releases/download/v{VERSION}/"

# SHA-256 of each asset, computed 2026-10-11 from the downloaded file. The same digests are shown by the GitHub releases API
# (`digest` field of each asset), which is the SAME origin, so this is not an independent source.
# TODO(HUMAN-VERIFY): confirm each digest against a second source (see dev-docs/HUMAN-TODO.md section 12).
ASSETS = {
    "linux": {
        "file": f"onnxruntime-linux-x64-{VERSION}.tgz",
        "sha256": "25b1ef1fea1acd210d63f8f24dc870ad6e077795ce1f54876252c6d3803c15af",
    },
    "windows": {
        "file": f"onnxruntime-win-x64-{VERSION}.zip",
        "sha256": "2e00414a63fdef0914cd5a5ede6c707844878e0c08e1b6693842f0451b2df2a1",
    },
}


def sha256_file(path: str) -> str:
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for block in iter(lambda: f.read(1 << 20), b""):
            h.update(block)
    return h.hexdigest()


def fetch_and_verify(platform: str, out_dir: str = ".") -> str:
    """Downloads and verifies the asset for `platform`; returns the path of the verified file. Exits 1 on a hash mismatch."""
    asset = ASSETS[platform]
    out_file = os.path.join(out_dir, asset["file"])
    url = BASE_URL + asset["file"]
    print(f"Downloading approved ONNX Runtime from {url}...")
    urllib.request.urlretrieve(url, out_file)
    actual = sha256_file(out_file)
    if actual != asset["sha256"]:
        os.remove(out_file)
        print(f"CRITICAL: Hash mismatch!\nExpected: {asset['sha256']}\nGot:      {actual}")
        sys.exit(1)
    print("Hash verified successfully. Supply chain secure.")
    return out_file


def main(argv: list[str]) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--platform", choices=sorted(ASSETS), default="linux")
    ap.add_argument("--out-dir", default=".")
    a = ap.parse_args(argv)
    path = fetch_and_verify(a.platform, a.out_dir)
    if a.platform == "linux":
        with tarfile.open(path, "r:gz") as t:
            t.extractall(a.out_dir, filter="data")   # "data" filter refuses absolute paths, links outside the folder and special files
        os.remove(path)
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
