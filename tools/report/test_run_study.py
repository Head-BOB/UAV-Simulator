# Tier: D
import io
import json
import os
import stat
import sys
import tempfile
import unittest
from pathlib import Path

import _paths
import run_study as R

SPEC = _paths.PARTS / "builds" / "reference_quad.build.json"
COND = _paths.REPO / "campaign" / "conditions_smoke.json"
DEFS = _paths.REPO / "campaign" / "smoke_definitions.json"
SETS = _paths.REPO / "campaign" / "condition_sets.json"


class FakeRunner:
    """Records commands; pretends a campaign ran by creating its result file. fail = substrings that make a command fail."""

    def __init__(self, fail=()):
        self.cmds, self.fail = [], fail

    def __call__(self, cmd):
        self.cmds.append(cmd)
        if any(f in " ".join(cmd) for f in self.fail):
            return 3
        if "run_campaign" in cmd:
            out = Path(cmd[cmd.index("--") + 4])
            out.mkdir(parents=True, exist_ok=True)
            (out / "campaign_result.json").write_text("{}")
        return 0


def go(tmp, runner, **kw):
    kw.setdefault("git", "abc123")
    kw.setdefault("log", lambda *_: None)
    return R.run_study(SPEC, COND, DEFS, Path(tmp) / "s", runner=runner, **kw)


class StudyTests(unittest.TestCase):
    def test_layout_commands_and_study_json(self):
        with tempfile.TemporaryDirectory() as t:
            r = FakeRunner()
            s = go(t, r, threads=2)
            camp = [c for c in r.cmds if "run_campaign" in c]
            n_ok = sum(e["status"] == "tested" for e in s["candidates"])
            self.assertEqual(len(camp), 1 + n_ok)                      # baseline once + each buildable candidate on the selected set
            self.assertEqual(len([c for c in r.cmds if "validate_models" in c]), 1)
            c0 = camp[0]
            self.assertEqual(c0[c0.index("--threads") + 1], "2")
            self.assertEqual(c0[c0.index("--git-hash") + 1], "abc123")
            self.assertTrue((Path(t) / "s" / "baseline" / "vehicle_config.json").exists())
            self.assertTrue((Path(t) / "s" / "conditions" / "smoke.json").exists())
            doc = json.loads((Path(t) / "s" / "study.json").read_text())
            self.assertEqual(doc, json.loads(json.dumps(s)))            # what is returned is what is written
            self.assertEqual(doc["selected_condition_set"], "smoke")
            self.assertEqual(doc["baseline_condition_sets"], ["smoke"])
            self.assertEqual({e["status"] for e in doc["candidates"]} <= {"tested", "rejected", "not_tested"}, True)
            rejected = [e for e in doc["candidates"] if e["status"] == "rejected"]
            self.assertTrue(rejected and all(e["errors"] for e in rejected))
            self.assertNotIn(str(_paths.REPO), json.dumps(doc))         # no machine path
            self.assertNotIn(str(t), json.dumps(doc))
            for e in doc["candidates"]:
                if e["status"] == "tested":
                    self.assertTrue((Path(t) / "s" / e["dir"].removeprefix("<study>/") / "vehicle_config.json").exists())

    def test_parallel_candidates_give_the_same_study_json(self):
        # --parallel only changes WHEN campaigns run, never what is written or in which order
        with tempfile.TemporaryDirectory() as t1, tempfile.TemporaryDirectory() as t2:
            a = go(t1, FakeRunner(), threads=1)
            b = go(t2, FakeRunner(), threads=1, parallel=4)
            self.assertEqual(json.dumps(a, sort_keys=True), json.dumps(b, sort_keys=True))
            self.assertEqual([e["id"] for e in a["candidates"]], [e["id"] for e in b["candidates"]])

    def test_staged_executables_replace_cargo_and_study_json_still_records_the_cargo_command(self):
        r = R.HelperRunner("/opt/run_campaign", "/opt/validate_models")
        cmd = R.campaign_command(Path("/b"), Path("/c.json"), Path("/d.json"), Path("/o"), 3, "g")
        self.assertEqual(r.real_command(cmd), ["/opt/run_campaign", "/b", "/c.json", "/d.json", "/o", "--threads", "3", "--git-hash", "g"])
        self.assertEqual(r.real_command(R.validation_command(Path("/b"))), ["/opt/validate_models", "/b", "/b"])
        self.assertEqual(R.HelperRunner().real_command(cmd), cmd)                    # no exe given: unchanged (cargo run)
        self.assertEqual(cmd[0], "cargo")                                            # the recorded command is the cargo one

    def test_progress_lines_come_from_the_campaigns_simulated_time(self):
        with tempfile.TemporaryDirectory() as t:
            fake = Path(t) / "run_campaign"
            fake.write_text(
                "#!%s\nimport json, sys, pathlib\nout = pathlib.Path(sys.argv[4]); out.mkdir(parents=True, exist_ok=True)\n"
                "for d in (50, 100):\n"
                "    print('UAVPROGRESS ' + json.dumps({'stage': 'test', 'event': 'tick', 'test_id': 'x', 'test_index': 0, 'test_count': 1,"
                " 'sim_s_done': d, 'sim_s_total': 100, 'percent_overall': 0, 'percent_test': d, 'elapsed_s': 1, 'eta_s': None, 'tests': []}), flush=True)\n"
                "print('not a progress line')\nprint('UAVPROGRESS {broken')\n"
                "(out / 'campaign_result.json').write_text('{}')\n" % sys.executable)
            fake.chmod(fake.stat().st_mode | stat.S_IEXEC)
            ok = Path(t) / "validate_models"
            ok.write_text("#!%s\nimport sys\nsys.exit(0)\n" % sys.executable)
            ok.chmod(ok.stat().st_mode | stat.S_IEXEC)
            out = io.StringIO()
            runner = R.HelperRunner(str(fake), str(ok), progress=True, out=out)
            study = go(t, runner, threads=1)
            runner.finish(True)
            lines = [json.loads(l[len("UAVPROGRESS "):]) for l in out.getvalue().splitlines() if l.startswith("UAVPROGRESS ")]
            self.assertTrue(lines)
            n_camp = 1 + sum(e["status"] == "tested" for e in study["candidates"])
            last = lines[-1]
            self.assertEqual((last["stage"], last["event"]), ("done", "study_done"))
            # every campaign reports 100 of 100 simulated seconds: all of them done = 100 % of the study's 95 % share
            self.assertAlmostEqual(last["percent_overall"], 95.0, places=2)
            self.assertAlmostEqual(last["sim_s_done"], 100.0 * n_camp, places=2)
            self.assertTrue(all(0.0 <= l["percent_overall"] <= 95.0 for l in lines))
            self.assertTrue(all(l["percent_overall"] <= m["percent_overall"] + 1e-9 for l, m in zip(lines, lines[1:])))   # never goes back
            self.assertEqual(len([l for l in lines if l["event"] == "campaign_done"]), n_camp)

    def test_condition_set_switch_off_and_on(self):
        with tempfile.TemporaryDirectory() as t:
            r = FakeRunner()
            go(t, r)
            self.assertEqual({Path(c[c.index("--") + 2]).stem for c in r.cmds if "run_campaign" in c}, {"smoke"})
        with tempfile.TemporaryDirectory() as t:
            r = FakeRunner()
            s = go(t, r, all_condition_sets=True, condition_sets_path=SETS, max_campaigns=100)
            base = [c for c in r.cmds if "run_campaign" in c and "/baseline" in c[c.index("--") + 1]]
            self.assertEqual(sorted(Path(c[c.index("--") + 2]).stem for c in base), sorted(["smoke", "standard_day", "hot_day", "hot_high_altitude", "windy_day"]))
            self.assertEqual(s["baseline_condition_sets"][0], "smoke")           # the selected set comes first
            self.assertEqual(s["candidate_condition_sets"], ["smoke"])           # candidates stay on the selected set
        with tempfile.TemporaryDirectory() as t:
            s = go(t, FakeRunner(), all_condition_sets=True, candidates_on="all", condition_sets_path=SETS, max_campaigns=200)
            n = sum(e["status"] == "tested" for e in s["candidates"])
            self.assertEqual(len(s["candidate_condition_sets"]), 5)
            self.assertEqual(sum(len(e["runs"]) for e in s["candidates"] if e["status"] == "tested"), 5 * n)

    def test_all_sets_needs_a_file_and_names_must_be_unique(self):
        with tempfile.TemporaryDirectory() as t:
            with self.assertRaises(R.StudyError):
                go(t, FakeRunner(), all_condition_sets=True)
            bad = Path(t) / "bad.json"
            bad.write_text(json.dumps({"schema_version": 1, "sets": [{"name": "a"}, {"name": "a"}]}))
            with self.assertRaises(R.StudyError):
                R.load_condition_sets(bad)
            bad.write_text(json.dumps({"schema_version": 2, "sets": [{"name": "a"}]}))
            with self.assertRaises(R.StudyError):
                R.load_condition_sets(bad)

    def test_max_campaigns_refuses_before_running_anything(self):
        with tempfile.TemporaryDirectory() as t:
            r = FakeRunner()
            with self.assertRaises(R.StudyError) as e:
                go(t, r, max_campaigns=3)
            self.assertIn("--max-campaigns", str(e.exception))
            self.assertEqual(r.cmds, [])
            self.assertEqual(R.plan_campaign_count(7, 5, 1), 12)
            self.assertEqual(R.plan_campaign_count(7, 5, 5), 40)

    def test_campaign_limit_is_inclusive(self):
        # smoke study: 8 compatible candidates + the baseline = 9 campaigns; exactly 9 is allowed, 8 is not
        with tempfile.TemporaryDirectory() as t:
            go(t, FakeRunner(), max_campaigns=9)
        with tempfile.TemporaryDirectory() as t:
            with self.assertRaises(R.StudyError):
                go(t, FakeRunner(), max_campaigns=8)

    def test_reuse_skips_existing_campaigns(self):
        with tempfile.TemporaryDirectory() as t:
            r1 = FakeRunner()
            go(t, r1)
            r2 = FakeRunner()
            s = go(t, r2, reuse=True)
            self.assertEqual([c for c in r2.cmds if "run_campaign" in c], [])
            self.assertTrue(all(run["status"] == "reused" for e in s["candidates"] if e["status"] == "tested" for run in e["runs"]))
            r3 = FakeRunner()
            go(t, r3)                                                    # without --reuse everything runs again
            self.assertEqual(len([c for c in r3.cmds if "run_campaign" in c]), len([c for c in r1.cmds if "run_campaign" in c]))

    def test_baseline_failure_stops_and_candidate_failure_is_not_tested(self):
        with tempfile.TemporaryDirectory() as t:
            with self.assertRaises(R.StudyError) as e:
                go(t, FakeRunner(fail=("/baseline",)))
            self.assertIn("baseline run failed", str(e.exception))
            self.assertFalse((Path(t) / "s" / "study.json").exists())
        with tempfile.TemporaryDirectory() as t:
            s = go(t, FakeRunner(fail=("motor__tmotor_mn3508_kv380",)))
            e = next(x for x in s["candidates"] if x["id"] == "motor__tmotor_mn3508_kv380")
            self.assertEqual(e["status"], "not_tested")
            self.assertIn("failed (exit 3)", e["reason"])
            self.assertTrue(any(x["status"] == "tested" for x in s["candidates"]))      # the others still ran

    def test_bad_inputs(self):
        with tempfile.TemporaryDirectory() as t:
            with self.assertRaises(R.StudyError):
                go(t, FakeRunner(), candidates_on="nope")
            spec = json.loads(SPEC.read_text())
            spec["parts"]["motor"] = "no_such_motor"
            p = Path(t) / "bad.build.json"
            p.write_text(json.dumps(spec))
            with self.assertRaises(R.StudyError) as e:
                R.run_study(p, COND, DEFS, Path(t) / "s2", runner=FakeRunner(), git="x", log=lambda *_: None)
            self.assertIn("baseline build refused", str(e.exception))

    def test_main_reports_a_stopped_study(self):
        with tempfile.TemporaryDirectory() as t:
            rc = R.main([str(SPEC), "--conditions", str(COND), "--definitions", str(DEFS), "--out", str(Path(t) / "s"), "--max-campaigns", "1"])
            self.assertEqual(rc, 1)

    def test_shown_hides_machine_prefixes(self):
        study = Path("/home/someone/study")
        self.assertEqual(R.shown(study / "a" / "b", study), "<study>/a/b")
        self.assertEqual(R.shown(R.REPO / "campaign" / "x.json", study), "campaign/x.json")
        self.assertEqual(R.shown(Path("/elsewhere/file.json"), study), "file.json")


if __name__ == "__main__":
    unittest.main()
