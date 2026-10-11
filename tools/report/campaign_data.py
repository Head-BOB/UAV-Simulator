# Tier: D
"""Loads one campaign result folder (written by physics_core `run_campaign`) into plain dicts, with strict checks.

Nothing here computes a physical quantity: it reads, checks that the files agree with each other, and refuses otherwise.
"""
from __future__ import annotations

import json
from pathlib import Path

SCHEMA_VERSION = 1
# Columns the report needs from every time series (the Rust SERIES_COLUMNS has more).
REQUIRED_COLUMNS = (
    ["t_s", "pos_d_m", "thrust_reserve_ratio", "battery_v", "battery_a", "battery_w", "motor_copper_w", "motor_noload_w", "esc_loss_w",
     "winding_c_max", "esc_c_max", "battery_c"]
    + [f"{k}_{i}" for k in ("rpm", "current_a", "torque_nm", "thrust_n") for i in range(1, 5)]
)
# Manifest fields that must be identical in every test of one campaign.
SAME_IN_ALL = ("build", "parts", "parts_db_version", "git_hash", "core_interface_version", "controller_config_version", "step_s",
               "drag_model", "turbulence", "conditions", "definitions_set_version")


class CampaignDataError(ValueError):
    """The folder is not a complete, consistent campaign result."""


def _json(path: Path) -> dict:
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except (OSError, ValueError) as e:
        raise CampaignDataError(f"cannot read {path}: {e}") from e


def read_series(path: Path) -> dict[str, list[float]]:
    """CSV -> column name -> list of floats. Refuses a missing required column, a ragged row or a non-number."""
    try:
        lines = path.read_text(encoding="utf-8").splitlines()
    except OSError as e:
        raise CampaignDataError(f"cannot read {path}: {e}") from e
    if not lines:
        raise CampaignDataError(f"{path} is empty")
    names = lines[0].split(",")
    missing = [c for c in REQUIRED_COLUMNS if c not in names]
    if missing:
        raise CampaignDataError(f"{path} lacks columns {missing} (campaign written by an older core?)")
    cols: dict[str, list[float]] = {n: [] for n in names}
    for k, line in enumerate(lines[1:], start=2):
        cells = line.split(",")
        if len(cells) != len(names):
            raise CampaignDataError(f"{path} line {k}: {len(cells)} cells, expected {len(names)}")
        for n, c in zip(names, cells):
            try:
                cols[n].append(float(c))
            except ValueError as e:
                raise CampaignDataError(f"{path} line {k}: {n} is not a number: {c!r}") from e
    return cols


def load_campaign(folder: Path) -> dict:
    """{dir, summary, tests: {name: {manifest, result, events, peaks, series: {file: columns}}}, common: manifest fields}."""
    folder = Path(folder)
    top = _json(folder / "campaign_result.json")
    if top.get("schema_version") != SCHEMA_VERSION:
        raise CampaignDataError(f"campaign_result.json schema_version {top.get('schema_version')!r}, expected {SCHEMA_VERSION}")
    tests: dict[str, dict] = {}
    for entry in top.get("tests", []):
        name = entry["test"]
        d = folder / name
        manifest, result, events = _json(d / "manifest.json"), _json(d / "result.json"), _json(d / "events.json")
        for what, doc in (("manifest", manifest), ("result", result)):
            if doc.get("schema_version") != SCHEMA_VERSION:
                raise CampaignDataError(f"{name}/{what}.json schema_version {doc.get('schema_version')!r}")
        if result.get("outcome") != entry.get("outcome"):
            raise CampaignDataError(f"{name}: outcome in result.json differs from campaign_result.json")
        series = {s["file"]: read_series(d / s["file"]) for s in result.get("series", [])}
        for s in result.get("series", []):
            if len(series[s["file"]]["t_s"]) != s["rows"]:
                raise CampaignDataError(f"{name}/{s['file']}: row count differs from result.json")
        peaks = {p["name"]: {"value": p["value"], "t_s": p["t_s"]} for p in events.get("peaks", [])}
        tests[name] = {"manifest": manifest, "result": result, "events": events.get("events", []), "peaks": peaks, "series": series}
    if not tests:
        raise CampaignDataError("campaign_result.json lists no tests")
    first = next(iter(tests.values()))["manifest"]
    for name, t in tests.items():
        for key in SAME_IN_ALL:
            if t["manifest"].get(key) != first.get(key):
                raise CampaignDataError(f"{name}: manifest field {key!r} differs from the other tests")
    if top.get("build") != first.get("build"):
        raise CampaignDataError("campaign_result.json build differs from the manifests")
    common = {k: first[k] for k in SAME_IN_ALL}
    # The CFD note is written only when a surrogate was involved. Every test and the campaign file must carry the same one.
    common["cfd_surrogate"] = first.get("cfd_surrogate")
    for name, t in tests.items():
        if t["manifest"].get("cfd_surrogate") != common["cfd_surrogate"]:
            raise CampaignDataError(f"{name}: manifest field 'cfd_surrogate' differs from the other tests")
    if top.get("cfd_surrogate") != common["cfd_surrogate"]:
        raise CampaignDataError("campaign_result.json cfd_surrogate differs from the manifests")
    return {"dir": folder, "summary": top, "tests": tests, "common": common}


def metric(campaign: dict, test: str, name: str):
    """Value of one named metric of one test, or None if the test or the metric is absent."""
    t = campaign["tests"].get(test)
    if t is None:
        return None
    for m in t["result"].get("metrics", []):
        if m["name"] == name:
            return m["value"]
    return None
