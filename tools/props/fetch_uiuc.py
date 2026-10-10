# Tier: D
"""Fetch, pin and convert UIUC propeller data (APC Thin Electric 12x6, Volume 4).

Source: UIUC Propeller Data Site, https://m-selig.ae.illinois.edu/props/volume-4/propDB-volume-4.html
Citation: Dantsker, Caccamo, Deters, Selig, "Performance Testing of APC Electric Fixed-Blade UAV
Propellers", AIAA Paper 2022-4020, Aviation 2022 Forum, Chicago, June 2022.
[UNVERIFIED] citation text was read from the site page by an AI; a human must check it.

Formats (read from the files, 2026-10-10):
  static file:       header `RPM CT CP`,      rows `rpm ct cp`
  wind-tunnel file:  header `J CT CP eta`,    rows `j ct cp eta`
Anything else is rejected. Every file's SHA-256 is pinned in `uiuc_pins.json` on the first fetch
(trust on first use: a human should review the pin file); a later mismatch stops the run.

Usage (from the repo root, inside .venv):
  python tools/props/fetch_uiuc.py            # fetch, pin, write offline_pipeline/props/apce_12x6.json
  python tools/props/fetch_uiuc.py --from-dir DIR   # use files already in DIR (no network)

Static fit (hover band): Ct, Cp = mean over static rows with rpm in [BAND_LO, BAND_HI].
  T = Ct * rho * n^2 * D^4 and Q = Cp * rho * n^2 * D^5 / (2 pi)  (n in rev/s), so
  kappa = Q / T = Cp * D / (2 pi * Ct)  [N*m per N = m], the core's `torque_coeff`.
"""
import argparse
import hashlib
import json
import math
import sys
import urllib.request
from pathlib import Path

BASE_URL = "https://m-selig.ae.illinois.edu/props/volume-4/data/"
STATIC_FILE = "apce_12x6_static_0629od.txt"
WIND_FILES = [
    "apce_12x6_0630od_3040.txt", "apce_12x6_0631od_4030.txt", "apce_12x6_0632od_5007.txt",
    "apce_12x6_0633od_5042.txt", "apce_12x6_0634od_5993.txt", "apce_12x6_0635od_6044.txt",
]
DIAMETER_M = 12 * 0.0254  # 12 inch nominal, exact conversion 0.0254 m/in
# Hover band: the reference build hovers at about 3500 RPM (estimate, ASSUMED).
BAND_LO_RPM = 3000.0
BAND_HI_RPM = 4500.0
CITATION = ("Dantsker, Caccamo, Deters, Selig, 'Performance Testing of APC Electric Fixed-Blade UAV "
            "Propellers', AIAA Paper 2022-4020, Aviation 2022 Forum, Chicago, June 2022 [UNVERIFIED]")
HERE = Path(__file__).resolve().parent
PIN_FILE = HERE / "uiuc_pins.json"
OUT_FILE = HERE.parent.parent / "offline_pipeline" / "props" / "apce_12x6.json"


class FormatError(ValueError):
    pass


def _numbers(line: str, count: int, where: str) -> list[float]:
    parts = line.split()
    if len(parts) != count:
        raise FormatError(f"{where}: expected {count} columns, got {len(parts)}: {line!r}")
    try:
        vals = [float(p) for p in parts]
    except ValueError as exc:
        raise FormatError(f"{where}: non-numeric value in {line!r}") from exc
    if not all(math.isfinite(v) for v in vals):
        raise FormatError(f"{where}: non-finite value in {line!r}")
    return vals


def _parse(text: str, header: list[str], name: str) -> list[list[float]]:
    lines = [ln for ln in text.splitlines() if ln.strip()]
    if not lines or lines[0].split() != header:
        got = lines[0].split() if lines else None
        raise FormatError(f"{name}: header must be {header}, found {got}")
    rows = [_numbers(ln, len(header), f"{name} line {i + 2}") for i, ln in enumerate(lines[1:])]
    if not rows:
        raise FormatError(f"{name}: no data rows")
    return rows


def parse_static(text: str, name: str = "static") -> list[dict]:
    rows = _parse(text, ["RPM", "CT", "CP"], name)
    if any(r[0] <= 0.0 for r in rows):
        raise FormatError(f"{name}: RPM must be > 0")
    return [{"rpm": r[0], "ct": r[1], "cp": r[2]} for r in rows]


def parse_wind(text: str, name: str = "wind") -> list[dict]:
    rows = _parse(text, ["J", "CT", "CP", "eta"], name)
    return [{"j": r[0], "ct": r[1], "cp": r[2], "eta": r[3]} for r in rows]


def static_fit(rows: list[dict], lo: float = BAND_LO_RPM, hi: float = BAND_HI_RPM,
               diameter_m: float = DIAMETER_M) -> dict:
    """Mean Ct, Cp over rows with lo <= rpm <= hi, and kappa = Cp*D/(2 pi Ct)."""
    band = [r for r in rows if lo <= r["rpm"] <= hi]
    if not band:
        raise FormatError(f"no static rows with rpm in [{lo}, {hi}]")
    ct = sum(r["ct"] for r in band) / len(band)
    cp = sum(r["cp"] for r in band) / len(band)
    if ct <= 0.0 or cp <= 0.0:
        raise FormatError("fitted Ct and Cp must be > 0")
    kappa = cp * diameter_m / (2.0 * math.pi * ct)
    return {"ct": ct, "cp": cp, "kappa_m": kappa, "rows_used": len(band), "band_rpm": [lo, hi]}


def sha256_hex(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def check_pin(pins: dict, name: str, digest: str) -> bool:
    """True if the pin was newly added. Raises on a mismatch (the file changed upstream)."""
    if name not in pins:
        pins[name] = digest
        return True
    if pins[name] != digest:
        raise FormatError(f"{name}: SHA-256 {digest} does not match the pin {pins[name]}")
    return False


def _download(name: str) -> bytes:
    url = BASE_URL + name
    try:
        with urllib.request.urlopen(url, timeout=60) as resp:  # noqa: S310 (fixed https URL)
            return resp.read()
    except OSError as exc:
        raise SystemExit(f"cannot download {url}: {exc}. Put the files in a folder and use --from-dir.") from exc


def build(files: dict[str, bytes], pins: dict) -> tuple[dict, bool]:
    new_pin = False
    for name, data in files.items():
        new_pin |= check_pin(pins, name, sha256_hex(data))
    static = parse_static(files[STATIC_FILE].decode("ascii"), STATIC_FILE)
    runs = {n: parse_wind(files[n].decode("ascii"), n) for n in WIND_FILES}
    out = {
        "propeller": "APC Thin Electric 12x6 (12x6E)",
        "diameter_m": DIAMETER_M,
        "source": "https://m-selig.ae.illinois.edu/props/volume-4/propDB-volume-4.html",
        "citation": CITATION,
        "status": "CITED data (UIUC measurement); fit band and use are ASSUMED, see static_fit",
        "validity": {"static_rpm": [static[0]["rpm"], static[-1]["rpm"]],
                     "note": "wind-tunnel runs only reach about 6044 RPM; above that is extrapolation"},
        "files": {n: pins[n] for n in files},
        "static": static,
        "static_fit": static_fit(static),
        "wind_tunnel": runs,
    }
    return out, new_pin


def main(argv: list[str]) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--from-dir", type=Path, help="read the data files from this folder, no network")
    args = ap.parse_args(argv)
    names = [STATIC_FILE] + WIND_FILES
    if args.from_dir:
        files = {n: (args.from_dir / n).read_bytes() for n in names}
    else:
        files = {n: _download(n) for n in names}
    pins = json.loads(PIN_FILE.read_text()) if PIN_FILE.exists() else {}
    out, new_pin = build(files, pins)
    if new_pin:
        PIN_FILE.write_text(json.dumps(pins, indent=2, sort_keys=True) + "\n")
        print(f"pinned new digests in {PIN_FILE} (trust on first use: review them)")
    OUT_FILE.parent.mkdir(parents=True, exist_ok=True)
    OUT_FILE.write_text(json.dumps(out, indent=2) + "\n")
    fit = out["static_fit"]
    print(f"wrote {OUT_FILE}: Ct={fit['ct']:.6f} Cp={fit['cp']:.6f} kappa={fit['kappa_m']:.6f} m ({fit['rows_used']} rows)")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
