# Tier: D
"""Study driver: builds the baseline and every compatible single-swap candidate, runs the Rust campaign for each, writes study.json.

Usage: python tools/report/run_study.py <build.json> --conditions <file> --definitions <file> --out <study dir>
         [--threads N] [--reuse] [--all-condition-sets] [--candidates-on selected|all] [--max-campaigns N]
Then: python tools/report/make_report.py --study <study dir> --out <report dir>

The campaigns are run by `cargo run --release --example run_campaign` (physics_core). Nothing is estimated here: a candidate whose
campaign fails is recorded as not tested. No clock and no machine path is written to study.json, so it is reproducible.
"""
from __future__ import annotations

import argparse
import json
import subprocess
import sys
from pathlib import Path

import _paths
import build_vehicle as B
import partsdb as P
import upgrades as U

REPO = _paths.REPO
CORE = REPO / "physics_core"
SCHEMA_VERSION = 1
DEFAULT_MAX_CAMPAIGNS = 40


class StudyError(RuntimeError):
    """The study cannot proceed (bad input, baseline failure, too many campaigns)."""


def default_runner(cmd: list[str]) -> int:
    return subprocess.run(cmd, cwd=REPO, stdout=subprocess.DEVNULL).returncode


def git_hash() -> str:
    """HEAD hash, with '+dirty' when the working tree has changes (the report states it)."""
    def run(args):
        return subprocess.run(["git", *args], cwd=REPO, capture_output=True, text=True, check=False).stdout.strip()
    h = run(["rev-parse", "HEAD"]) or "unknown"
    return h + ("+dirty" if run(["status", "--porcelain"]) else "")


def condition_name(cond: dict, path: Path) -> str:
    return cond.get("name") or path.stem


def load_condition_sets(path: Path) -> list[dict]:
    doc = json.loads(path.read_text(encoding="utf-8"))
    if doc.get("schema_version") != 1 or not doc.get("sets"):
        raise StudyError(f"{path}: needs schema_version 1 and a non-empty 'sets' list")
    names = [s.get("name") for s in doc["sets"]]
    if None in names or len(set(names)) != len(names):
        raise StudyError(f"{path}: every set needs a unique name")
    return doc["sets"]


def campaign_command(build_dir: Path, cond_file: Path, defs_file: Path, out_dir: Path, threads: int, git: str) -> list[str]:
    return ["cargo", "run", "--release", "--quiet", "--manifest-path", str(CORE / "Cargo.toml"), "--example", "run_campaign", "--",
            str(build_dir), str(cond_file), str(defs_file), str(out_dir), "--threads", str(threads), "--git-hash", git]


def validation_command(build_dir: Path) -> list[str]:
    return ["cargo", "run", "--release", "--quiet", "--manifest-path", str(CORE / "Cargo.toml"), "--example", "validate_models", "--",
            str(build_dir), str(build_dir)]


def shown(path: Path, study: Path) -> str:
    """A path without the machine's prefix: relative to the study folder, else to the repository, else the bare name."""
    for base, prefix in ((study, "<study>/"), (REPO, "")):
        try:
            return prefix + path.resolve().relative_to(base.resolve()).as_posix()
        except ValueError:
            continue
    return path.name


def shown_cmd(cmd: list[str], study: Path) -> list[str]:
    out = []
    for a in cmd:
        out.append(shown(Path(a), study) if a.startswith("/") else a)
    return out


def plan_campaign_count(n_candidates: int, base_sets: int, cand_sets: int) -> int:
    return base_sets + n_candidates * cand_sets


def run_study(spec_path: Path, conditions_path: Path, definitions_path: Path, out: Path, *, runner=default_runner, threads: int = 1,
              reuse: bool = False, all_condition_sets: bool = False, candidates_on: str = "selected",
              condition_sets_path: Path | None = None, max_campaigns: int = DEFAULT_MAX_CAMPAIGNS, git: str | None = None,
              db_root: Path = P.DEFAULT_DB, log=print) -> dict:
    if candidates_on not in ("selected", "all"):
        raise StudyError("--candidates-on must be 'selected' or 'all'")
    spec = json.loads(spec_path.read_text(encoding="utf-8"))
    selected = json.loads(conditions_path.read_text(encoding="utf-8"))
    sel_name = condition_name(selected, conditions_path)
    sets = {sel_name: selected}
    if all_condition_sets:
        if condition_sets_path is None:
            raise StudyError("--all-condition-sets needs --condition-sets <file>")
        for s in load_condition_sets(condition_sets_path):
            sets.setdefault(s["name"], s)
    base_names = sorted(sets, key=lambda n: (n != sel_name, n))
    cand_names = base_names if candidates_on == "all" else [sel_name]
    records, _ = P.load_records(db_root)
    checked = [U.check_candidate(c, db_root) for c in U.swap_specs(spec, records)]
    buildable = [c for c in checked if c["status"] == "buildable"]
    total = plan_campaign_count(len(buildable), len(base_names), len(cand_names))
    log(f"{len(buildable)} compatible candidates, {len(checked) - len(buildable)} rejected: {total} campaigns")
    if total > max_campaigns:
        raise StudyError(f"{total} campaigns exceed --max-campaigns {max_campaigns}; raise it or narrow the study")
    git = git or git_hash()
    out.mkdir(parents=True, exist_ok=True)
    cond_dir = out / "conditions"
    cond_dir.mkdir(exist_ok=True)
    cond_files = {}
    for name in sets:
        f = cond_dir / f"{name}.json"
        f.write_text(json.dumps({**sets[name], "name": name}, indent=2) + "\n", encoding="utf-8")
        cond_files[name] = f

    def build_and_run(entry_id: str, result, names: list[str], validate: bool) -> dict:
        d = out / ("baseline" if entry_id == "baseline" else f"candidates/{entry_id}")
        B.write_outputs(result, d)
        runs = []
        for name in names:
            dest = d / "campaign" / name
            cmd = campaign_command(d, cond_files[name], definitions_path, dest, threads, git)
            if reuse and (dest / "campaign_result.json").exists():
                runs.append({"condition": name, "dir": shown(dest, out), "status": "reused", "command": shown_cmd(cmd, out)})
                continue
            rc = runner(cmd)
            runs.append({"condition": name, "dir": shown(dest, out), "status": "ok" if rc == 0 else f"failed (exit {rc})",
                         "command": shown_cmd(cmd, out)})
        if validate:
            rc = runner(validation_command(d))
            runs.append({"condition": "(validation)", "dir": shown(d, out), "status": "ok" if rc == 0 else f"failed (exit {rc})",
                         "command": shown_cmd(validation_command(d), out)})
        return {"runs": runs, "dir": shown(d, out)}

    base_res = B.build(spec, db_root)
    if not base_res.ok:
        raise StudyError("baseline build refused: " + "; ".join(f.message for f in base_res.findings if f.severity == "error"))
    base = build_and_run("baseline", base_res, base_names, True)
    bad = [r for r in base["runs"] if r["status"].startswith("failed")]
    if bad:
        raise StudyError(f"baseline run failed: {bad[0]['condition']} {bad[0]['status']}")
    entries = []
    for c in checked:
        e = {"id": c["id"], "role": c["role"], "part_id": c["part_id"], "status": c["status"], "errors": c["errors"], "warnings": c["warnings"]}
        if c["status"] == "buildable":
            e.update(build_and_run(c["id"], c["build"], cand_names, False))
            e["price_usd"] = U.parts_price_usd(c["spec"], records)
            e["status"] = "tested" if not any(r["status"].startswith("failed") for r in e["runs"]) else "not_tested"
            if e["status"] == "not_tested":
                e["reason"] = "campaign failed: " + next(r["status"] for r in e["runs"] if r["status"].startswith("failed"))
        else:
            e["status"] = "rejected"
        entries.append(e)
    study = {"schema_version": SCHEMA_VERSION, "build_spec": spec, "git_hash": git, "definitions": shown(definitions_path, out),
             "selected_condition_set": sel_name, "condition_sets": {n: sets[n] for n in base_names}, "baseline_condition_sets": base_names,
             "candidate_condition_sets": cand_names, "all_condition_sets": all_condition_sets, "threads": threads,
             "baseline": base, "baseline_price_usd": U.parts_price_usd(spec, records), "candidates": entries}
    (out / "study.json").write_text(json.dumps(study, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return study


def main(argv: list[str]) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("build_file", type=Path)
    ap.add_argument("--conditions", type=Path, required=True)
    ap.add_argument("--definitions", type=Path, required=True)
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--threads", type=int, default=1)
    ap.add_argument("--reuse", action="store_true", help="skip a campaign whose result folder already exists")
    ap.add_argument("--all-condition-sets", action="store_true", help="also run the baseline on every set of --condition-sets (default off)")
    ap.add_argument("--condition-sets", type=Path, default=REPO / "campaign" / "condition_sets.json")
    ap.add_argument("--candidates-on", choices=("selected", "all"), default="selected")
    ap.add_argument("--max-campaigns", type=int, default=DEFAULT_MAX_CAMPAIGNS)
    a = ap.parse_args(argv)
    try:
        run_study(a.build_file, a.conditions, a.definitions, a.out, threads=a.threads, reuse=a.reuse, all_condition_sets=a.all_condition_sets,
                  candidates_on=a.candidates_on, condition_sets_path=a.condition_sets, max_campaigns=a.max_campaigns)
    except (StudyError, OSError, ValueError) as e:
        print(f"STUDY STOPPED: {e}", file=sys.stderr)
        return 1
    print(f"study written to {a.out}/study.json")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
