# Tier: D
"""Fetch one manufacturer / authoritative document, cache it, and pin its SHA-256.

This is a deliberate, single-URL tool. It is NOT a scraper: it never follows links, and it refuses any
host that is not listed in `parts_db/allowed_sources.json` (a human-reviewed list; retailers are not on
it). The cache directory `parts_db/source_cache/` is gitignored; only the pin (url, date, sha256, size,
http status) is committed in `parts_db/source_pins.json`. Trust on first use: a later fetch whose bytes
differ from the pin is refused unless `--update` is given, and the old pin is kept in `history`.

Usage (repo root, inside .venv):
  python tools/parts/fetch_source.py URL [--note TEXT] [--update]
"""
from __future__ import annotations

import argparse
import datetime
import hashlib
import json
import sys
import urllib.parse
import urllib.request
from pathlib import Path
from typing import Callable

import partsdb as P

MAX_BYTES = 20 * 1024 * 1024
TIMEOUT_S = 30


def host_allowed(url: str, allowed: list[str]) -> bool:
    parsed = urllib.parse.urlparse(url)
    if parsed.scheme not in ("http", "https") or not parsed.hostname:
        return False
    host = parsed.hostname.lower()
    return any(host == a or host.endswith("." + a) for a in (x.lower() for x in allowed))


def cache_name(url: str) -> str:
    return hashlib.sha256(url.encode()).hexdigest()[:16]


class AllowlistRedirectHandler(urllib.request.HTTPRedirectHandler):
    """Every redirect hop must also be on the allowlist (otherwise an allowed host could hand us to any site)."""

    def __init__(self, allowed: list[str]) -> None:
        self.allowed = allowed

    def redirect_request(self, req, fp, code, msg, headers, newurl):  # noqa: ANN001 (urllib signature)
        if not host_allowed(newurl, self.allowed):
            raise ValueError(f"redirect to {newurl} refused: host is not in allowed_sources.json")
        return super().redirect_request(req, fp, code, msg, headers, newurl)


def make_opener(allowed: list[str]) -> Callable[[str], tuple[bytes, int]]:
    director = urllib.request.build_opener(AllowlistRedirectHandler(allowed))

    def opener(url: str) -> tuple[bytes, int]:
        req = urllib.request.Request(url, headers={"User-Agent": "uav-parts-db-fetch/1 (engineering datasheet pin)"})
        with director.open(req, timeout=TIMEOUT_S) as r:
            return r.read(MAX_BYTES + 1), r.status

    return opener


def fetch(url: str, db_root: Path, today: str, note: str = "", update: bool = False,
          opener: Callable[[str], tuple[bytes, int]] | None = None) -> dict:
    allowed_path = db_root / "allowed_sources.json"
    allowed = json.loads(allowed_path.read_text(encoding="utf-8"))["hosts"]
    if not host_allowed(url, allowed):
        raise ValueError(f"host of {url} is not in allowed_sources.json (retailers and unknown sites are refused)")
    data, status = (opener or make_opener(allowed))(url)
    if status != 200:
        raise ValueError(f"{url}: HTTP {status}, nothing pinned")
    if len(data) > MAX_BYTES:
        raise ValueError(f"{url}: larger than {MAX_BYTES} bytes, refused")
    sha = hashlib.sha256(data).hexdigest()
    pins = P.load_pins(db_root)
    old = pins.get(url)
    if old and old["sha256"] != sha and not update:
        raise ValueError(f"{url}: content changed since the pin ({old['sha256'][:12]} -> {sha[:12]}); use --update after review")
    cache = db_root / "source_cache"
    cache.mkdir(exist_ok=True)
    (cache / cache_name(url)).write_bytes(data)
    entry = {"retrieved_on": today, "sha256": sha, "bytes": len(data), "http_status": status,
             "note": note or (old or {}).get("note", ""), "cache_file": cache_name(url)}
    if old and old["sha256"] != sha:
        entry["history"] = (old.get("history") or []) + [{k: old[k] for k in ("retrieved_on", "sha256", "bytes")}]
    elif old and old.get("history"):
        entry["history"] = old["history"]
    pins[url] = entry
    (db_root / "source_pins.json").write_text(P.canonical_json(pins), encoding="utf-8", newline="\n")
    return entry


def main(argv: list[str]) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("url")
    ap.add_argument("--note", default="")
    ap.add_argument("--update", action="store_true")
    ap.add_argument("--db", type=Path, default=P.DEFAULT_DB)
    a = ap.parse_args(argv)
    try:
        e = fetch(a.url, a.db, datetime.date.today().isoformat(), a.note, a.update)
    except (ValueError, OSError) as ex:
        print(f"fetch failed: {ex}", file=sys.stderr)
        return 1
    print(f"pinned {a.url}\n  sha256 {e['sha256']}  bytes {e['bytes']}  retrieved_on {e['retrieved_on']}")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
