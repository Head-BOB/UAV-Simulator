# Tier: D
"""Velocity-slice PNG for the report, from the raw slice a FluidX3D case writes (`slice_y_mid.f32`). No plotting library.

Input: Nx * Nz little-endian float32 values, velocity magnitude in m/s, x (downstream) fastest, then z; the plane is y = mid
(through the vehicle centre). Output: an 8-bit RGB PNG (own encoder: signature, IHDR, one IDAT, IEND, CRC-32 per chunk), the
image's top row is the largest z, x runs left to right, and a `<name>.legend.txt` with the scale.
Colour map (piecewise linear on 5 stops, an arbitrary choice): dark blue (0) - blue - cyan - yellow - red (full scale).
Cells whose speed is exactly 0 are SOLID (the vehicle) and drawn dark grey; a non-finite value is drawn magenta.
Full scale defaults to 1.5 x the freestream speed. PRELIMINARY CFD, UNVALIDATED: the legend says so.
"""
from __future__ import annotations

import argparse
import json
import math
import struct
import sys
import zlib
from pathlib import Path

import numpy as np

STOPS = ((0.0, (20, 20, 100)), (0.25, (30, 100, 200)), (0.5, (40, 200, 200)), (0.75, (240, 220, 60)), (1.0, (200, 30, 30)))
SOLID_RGB = (60, 60, 60)
BAD_RGB = (255, 0, 255)
DEFAULT_FULL_SCALE_FACTOR = 1.5
PNG_SIGNATURE = b"\x89PNG\r\n\x1a\n"


class ImageError(ValueError):
    pass


def colour(t: float) -> tuple:
    """RGB for t in [0, 1] (clamped), linear between the stops, rounded to the nearest integer."""
    t = min(max(t, 0.0), 1.0)
    for (t0, c0), (t1, c1) in zip(STOPS, STOPS[1:]):
        if t <= t1:
            w = (t - t0) / (t1 - t0)
            return tuple(int(round(c0[i] + w * (c1[i] - c0[i]))) for i in range(3))
    return STOPS[-1][1]


def read_slice(path: Path, nx: int, nz: int) -> np.ndarray:
    raw = path.read_bytes()
    if nx <= 0 or nz <= 0 or len(raw) != nx * nz * 4:
        raise ImageError(f"{path}: {len(raw)} bytes, expected {nx} x {nz} float32 = {nx * nz * 4}")
    return np.frombuffer(raw, dtype="<f4").reshape(nz, nx).astype(np.float64)


def to_rgb(values: np.ndarray, full_scale: float) -> np.ndarray:
    """(nz, nx) speeds -> (nz, nx, 3) uint8, top row = largest z."""
    if not (math.isfinite(full_scale) and full_scale > 0.0):
        raise ImageError("full scale must be finite and > 0")
    out = np.zeros(values.shape + (3,), dtype=np.uint8)
    for j in range(values.shape[0]):
        for i in range(values.shape[1]):
            v = values[j, i]
            if not math.isfinite(v):
                rgb = BAD_RGB
            elif v == 0.0:
                rgb = SOLID_RGB
            else:
                rgb = colour(v / full_scale)
            out[j, i] = rgb
    return out[::-1]


def _chunk(kind: bytes, data: bytes) -> bytes:
    return struct.pack(">I", len(data)) + kind + data + struct.pack(">I", zlib.crc32(kind + data) & 0xFFFFFFFF)


def encode_png(rgb: np.ndarray) -> bytes:
    """8-bit RGB PNG of an (h, w, 3) uint8 array: filter type 0 on every row, one IDAT."""
    if rgb.ndim != 3 or rgb.shape[2] != 3 or rgb.dtype != np.uint8:
        raise ImageError("expected an (h, w, 3) uint8 array")
    h, w = rgb.shape[:2]
    raw = b"".join(b"\x00" + rgb[r].tobytes() for r in range(h))
    ihdr = struct.pack(">IIBBBBB", w, h, 8, 2, 0, 0, 0)
    return PNG_SIGNATURE + _chunk(b"IHDR", ihdr) + _chunk(b"IDAT", zlib.compress(raw, 6)) + _chunk(b"IEND", b"")


def upscale(rgb: np.ndarray, factor: int) -> np.ndarray:
    if factor < 1:
        raise ImageError("scale must be >= 1")
    return np.repeat(np.repeat(rgb, factor, axis=0), factor, axis=1)


def legend_text(case_id: str, full_scale: float, speed: float, nx: int, nz: int, scale: int) -> str:
    stops = "; ".join(f"{t:g} -> rgb{c}" for t, c in STOPS)
    return (f"case {case_id}\nplane: y = mid plane through the vehicle centre; x (downstream) runs left to right, z up\n"
            f"slice {nx} x {nz} cells, image scale x{scale}\nvelocity magnitude: 0 .. {full_scale:.6g} m/s (freestream {speed:.6g} m/s, full scale = {DEFAULT_FULL_SCALE_FACTOR} x freestream unless set)\n"
            f"colour stops (fraction of full scale): {stops}\nsolid cells (speed exactly 0): rgb{SOLID_RGB}; non-finite: rgb{BAD_RGB}\n"
            "PRELIMINARY CFD, UNVALIDATED\n")


def make_image(slice_path: Path, nx: int, nz: int, speed: float, out: Path, case_id: str = "", full_scale: float | None = None, scale: int = 1) -> dict:
    values = read_slice(slice_path, nx, nz)
    fs = full_scale if full_scale is not None else DEFAULT_FULL_SCALE_FACTOR * speed
    rgb = upscale(to_rgb(values, fs), scale)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_bytes(encode_png(rgb))
    out.with_suffix(".legend.txt").write_text(legend_text(case_id, fs, speed, nx, nz, scale), encoding="utf-8", newline="\n")
    finite = values[np.isfinite(values)]
    return {"png": out, "width": rgb.shape[1], "height": rgb.shape[0], "full_scale_mps": fs, "max_speed_mps": float(finite.max()) if finite.size else 0.0}


def main(argv: list) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--case-dir", type=Path, help="a generated case folder with case.json and slice_y_mid.f32")
    ap.add_argument("--slice", type=Path)
    ap.add_argument("--nx", type=int)
    ap.add_argument("--nz", type=int)
    ap.add_argument("--speed", type=float)
    ap.add_argument("--full-scale", type=float)
    ap.add_argument("--scale", type=int, default=1)
    ap.add_argument("--out", type=Path, required=True)
    a = ap.parse_args(argv)
    try:
        if a.case_dir:
            case = json.loads((a.case_dir / "case.json").read_text(encoding="utf-8"))
            r = make_image(a.case_dir / "slice_y_mid.f32", case["grid"]["nx"], case["grid"]["nz"], case["speed_mps"], a.out, case["case_id"], a.full_scale, a.scale)
        else:
            if not (a.slice and a.nx and a.nz and a.speed):
                raise ImageError("give --case-dir or all of --slice --nx --nz --speed")
            r = make_image(a.slice, a.nx, a.nz, a.speed, a.out, "", a.full_scale, a.scale)
    except (ImageError, OSError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1
    print(f"wrote {r['png']} ({r['width']} x {r['height']}), full scale {r['full_scale_mps']:.4g} m/s, max {r['max_speed_mps']:.4g} m/s")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
