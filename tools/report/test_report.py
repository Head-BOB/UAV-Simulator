# Tier: D
import copy
import json
import shutil
import tempfile
import unittest
from html.parser import HTMLParser
from pathlib import Path

import _paths
import campaign_data as D
import make_report as MR
import render_html as RH
import testkit as K
import upgrades as U

REF = _paths.PARTS / "builds" / "reference_quad"
SMOKE = REF / "campaign_smoke"
VOID = {"meta", "br", "img", "input", "link", "hr"}


class Balance(HTMLParser):
    def __init__(self):
        super().__init__()
        self.stack, self.errors, self.ids = [], [], []

    def handle_starttag(self, tag, attrs):
        if tag not in VOID:
            self.stack.append(tag)
        for k, v in attrs:
            if k == "id":
                self.ids.append(v)

    def handle_endtag(self, tag):
        if not self.stack or self.stack[-1] != tag:
            self.errors.append(f"unbalanced </{tag}>")
        else:
            self.stack.pop()


def check_html(text):
    p = Balance()
    p.feed(text)
    return p


def raw_peak(name):
    """Largest recorded peak of one name over every test of the committed smoke campaign (read from the raw events files)."""
    return max(p["value"] for f in sorted(SMOKE.glob("*/events.json")) for p in json.loads(f.read_text())["peaks"] if p["name"] == name)


def smoke_report(**kw):
    return MR.build_report(REF, {"smoke": SMOKE}, REF / "validation_report.json", **kw)


class EndToEndSmokeTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.report, cls.plots = smoke_report()
        cls.html = RH.render_html(cls.report, cls.plots)

    def test_sections_in_order_unique_and_html_balanced(self):
        p = check_html(self.html)
        self.assertEqual(p.errors, [])
        self.assertEqual(p.stack, [])
        self.assertEqual([i for i in p.ids if i in dict(RH.SECTIONS)], [s for s, _ in RH.SECTIONS])
        self.assertEqual(len(p.ids), len(set(p.ids)))

    def test_limiting_metric_by_hand_from_the_raw_events_file(self):
        # independent of margins.py: the largest motor current peak over ALL eight tests' raw events.json files (the FPV punch-out is the highest,
        # above the 23 A rating) and the rating from the config; margin = limit / peak
        limit = json.loads((REF / "powertrain_config.json").read_text())["motor_max_current_a"]
        want = limit / raw_peak("motor_current_a")
        self.assertLess(want, 1.0)                                              # the full-throttle punch-out exceeds the motor's rating
        self.assertAlmostEqual(self.report["limiting"]["margin"], want, places=6)
        self.assertEqual(self.report["limiting"]["slot"], "motor_current")
        self.assertIn(RH.fmt(self.report["limiting"]["margin"]), self.html)

    def test_validation_flags_are_exactly_the_rows_above_ten_percent(self):
        raw = json.loads((REF / "validation_report.json").read_text())
        want = sorted(r["id"] for s in raw["sections"] for r in s["rows"] if r["percent_error"] is not None and abs(r["percent_error"]) > 10.0)
        self.assertTrue(want)                                                   # the golden does contain such rows
        self.assertEqual(sorted(f["id"] for f in self.report["validation"]["flagged"]), want)
        self.assertEqual(sum(1 for a in self.report["assumptions"] if a["category"] == "validation"), len(want))
        self.assertEqual(self.html.count('class="num flag"'), len(want))
        largest = max(abs(f["percent_error"]) for f in self.report["validation"]["flagged"])
        self.assertAlmostEqual(largest, 27.8417354943, places=6)                  # the worst propeller coefficient point

    def test_speed_tests_are_in_the_summary_and_nothing_is_listed_as_not_built(self):
        by = {x["label"]: x for x in self.report["summary"]}
        ms = by["Max speed (commanded, passing)"]
        # the reference quad's class speed demand is 12 m/s (parts_db/controller_classes.json); every level up to it passed, so the result is limit-bound
        self.assertEqual((ms["value"], ms["unit"]), (12.0, "m/s"))
        self.assertIn("LIMIT-BOUND", ms["detail"])
        self.assertIn("FPV sprint terminal speed", by)
        self.assertIn("FPV punch-out climb rate", by)
        self.assertEqual(self.report["not_built"], [])
        self.assertNotIn("Not tested", self.html)
        self.assertIn("LIMIT-BOUND", self.html)
        self.assertIn("speed_drag", {b["id"] for b in self.report["banners"]})
        env = self.report["envelope"][0]["headline"]
        self.assertEqual(env["max_speed_mps"], 12.0)

    def test_flag_boundary_in_the_html_table(self):
        r = copy.deepcopy(self.report)
        rows = r["validation"]["sections"][0]["rows"]
        for row, pe in zip(rows[:3], (10.0, 10.01, -9.99)):
            row["percent_error"] = pe
        for row in rows[3:]:
            row["percent_error"] = 0.0
        html = RH.render_html(r, self.plots)
        sec = html[html.index('<section id="validation">'):html.index('<section id="envelope">')]
        sec_first = sec[:sec.index("<h3>", sec.index("<h3>") + 4)]            # first validation section only
        self.assertEqual(sec_first.count('class="num flag"'), 1)                # only 10.01 is above 10 %

    def test_banners_and_upgrade_text_for_smoke_without_a_study(self):
        ids = {b["id"] for b in self.report["banners"]}
        self.assertTrue({"smoke", "no_cfd", "turbulence", "unverified_parts", "validation", "assumed_inputs"} <= ids)
        self.assertIn("NOT FOR ENGINEERING USE", self.html)
        self.assertIn("Turbulence: Dryden, unverified.", self.html)
        self.assertEqual(self.report["upgrades"]["status"], "not run")
        self.assertIn("No upgrade study was run", self.html)
        self.assertIn("No CFD", self.html)

    def test_numbers_in_the_html_are_the_numbers_of_the_json(self):
        for m in self.report["margins"]:
            self.assertIn(RH.fmt(m["margin"]), self.html)
            self.assertIn(RH.fmt(m["peak"], m["unit"]), self.html)
        for x in self.report["summary"]:
            if x["value"] is not None:
                self.assertIn(RH.fmt(x["value"], x.get("unit", "")), self.html)
        p = self.report["power"]
        self.assertIn(RH.fmt(p["pack_w"], "W"), self.html)
        self.assertAlmostEqual(p["shaft_w"] + p["loss_total_w"], p["pack_w"], places=5)

    def test_deterministic_and_clean_of_paths_and_clock(self):
        with tempfile.TemporaryDirectory() as t:
            a, b = Path(t) / "a", Path(t) / "b"
            for d in (a, b):
                rep, plots = smoke_report()
                RH.write_all(rep, plots, d)
            names = sorted(p.name for p in a.iterdir())
            self.assertEqual(names, ["assumptions.csv", "events.csv", "margins.csv", "report.html", "report.json", "summary.csv", "upgrades.csv", "validation.csv"])
            for n in names:
                self.assertEqual((a / n).read_bytes(), (b / n).read_bytes(), n)
                text = (a / n).read_text()
                self.assertNotIn(str(_paths.REPO), text, n)
                self.assertNotIn("/home/", text, n)
                self.assertNotIn("UTC", text, n)

    def test_html_is_self_contained(self):
        for bad in ("http://", "https://", "<script", "src=\"//", "@import"):
            self.assertNotIn(bad, self.html.replace("http://www.w3.org/2000/svg", ""))

    def test_reproducibility_hashes_every_input(self):
        files = self.report["reproducibility"]["input_sha256"]
        self.assertIn("build/vehicle_config.json", files)
        self.assertIn("campaign/smoke/climb/timeseries_climb.csv", files)
        self.assertIn("validation_report.json", files)
        import hashlib
        want = hashlib.sha256((REF / "vehicle_config.json").read_bytes()).hexdigest()
        self.assertEqual(files["build/vehicle_config.json"], want)
        self.assertEqual(self.report["reproducibility"]["seeds"], {"smoke": 1})
        self.assertEqual(self.report["reproducibility"]["git_hash"], "golden")

    def test_untrusted_text_is_escaped(self):
        r = copy.deepcopy(self.report)
        r["vehicle"]["bom"][0]["model"] = "<script>alert(1)</script>"
        r["assumptions"][0]["item"] = "<img src=x onerror=1>"
        r["banners"][0]["text"] = "</div><b>x"
        html = RH.render_html(r, self.plots)
        for raw in ("<script>alert", "<img src=x", "</div><b>x"):
            self.assertNotIn(raw, html)
        self.assertEqual(check_html(html).errors, [])


class KeyMomentTests(unittest.TestCase):
    def test_reserve_alert_is_the_first_time_below_the_threshold(self):
        rows = [{"t_s": 0.0, "thrust_reserve_ratio": 2.0}, {"t_s": 1.0, "thrust_reserve_ratio": 1.6}, {"t_s": 2.0, "thrust_reserve_ratio": 1.4},
                {"t_s": 3.0, "thrust_reserve_ratio": 1.2}]
        with tempfile.TemporaryDirectory() as t:
            c = MR.D.load_campaign(K.write_campaign(Path(t) / "c", {"climb": {"rows": rows}, "endurance": {}}))
        _, key = MR.event_rows(c)
        alerts = [k for k in key if "thrust reserve" in k["moment"]]
        # 1.6 is above 1.5; 1.4 at t = 2 s is the first below; the second test never drops (all rows 0 -> its zero rows are below 1.5 at t = 0)
        self.assertEqual([(a["test"], a["t_s"], a["value"]) for a in alerts], [("climb", 2.0, 1.4), ("endurance", 0.0, 0.0)])
        peaks = {k["moment"] for k in key if k["test"] == "climb"}
        self.assertIn("winding temperature peak", peaks)
        self.assertIn("lowest pack voltage (sag)", peaks)


class BannerTests(unittest.TestCase):
    COMMON = {"definitions_set_version": "1.0.0", "drag_model": "analytic_legacy",
              "conditions": {"turbulence": "dryden", "wind_speed_mps": 0.0}}

    def ids(self, common=None, tests=("climb",), bom=(), flags=(), pt=None):
        c = copy.deepcopy(self.COMMON)
        c.update(common or {})
        return {b["id"] for b in MR.banners(c, list(tests), list(bom), list(flags), pt or {"parameters_verified": 1})}

    def test_each_banner_appears_exactly_when_it_applies(self):
        self.assertEqual(self.ids(), {"no_cfd"})                                                      # a clean non-smoke analytic run
        self.assertIn("smoke", self.ids({"definitions_set_version": "1.0.0-smoke"}))
        self.assertIn("edited_limits", self.ids({"definitions_set_version": "1.0.0-edited"}))
        # the speed-drag banner: only when a speed test ran AND the drag is the legacy analytic model
        self.assertIn("speed_drag", self.ids(tests=("climb", "max_speed")))
        self.assertIn("speed_drag", self.ids(tests=("fpv_punchout",)))
        self.assertNotIn("speed_drag", self.ids(tests=("climb", "endurance")))
        self.assertNotIn("speed_drag", self.ids({"drag_model": "cfd_surrogate:trained:ab"}, tests=("max_speed",)))
        self.assertNotIn("edited_limits", self.ids({"definitions_set_version": "1.0.0"}))
        self.assertIn("stub_model", self.ids({"drag_model": "cfd_surrogate:stub:ab"}))
        self.assertNotIn("no_cfd", self.ids({"drag_model": "cfd_surrogate:stub:ab"}))
        self.assertIn("prelim_cfd", self.ids({"drag_model": "cfd_surrogate:trained:ab"}))
        self.assertIn("turbulence", self.ids(tests=("wind_gust",)))
        self.assertIn("turbulence", self.ids({"conditions": {"turbulence": "dryden", "wind_speed_mps": 3.0}}))
        self.assertNotIn("turbulence", self.ids({"conditions": {"turbulence": "simple", "wind_speed_mps": 3.0}}, tests=("wind_gust",)))
        self.assertIn("unverified_parts", self.ids(bom=[{"part_id": "a", "verification": "Unverified"}]))
        self.assertNotIn("unverified_parts", self.ids(bom=[{"part_id": "a", "verification": "Verified"}]))
        self.assertIn("validation", self.ids(flags=[{"id": "x"}]))
        self.assertIn("assumed_inputs", self.ids(pt={"parameters_verified": 0}))


class FormatTests(unittest.TestCase):
    def test_fmt(self):
        self.assertEqual(RH.fmt(None), "n/a")
        self.assertEqual(RH.fmt(True), "yes")
        self.assertEqual(RH.fmt(3), "3")
        self.assertEqual(RH.fmt(2.0, "kg"), "2 kg")
        self.assertEqual(RH.fmt(1.123456), "1.123")
        self.assertEqual(RH.fmt(0.000123456), "0.0001235")

    def test_csv_text_exact(self):
        self.assertEqual(RH.csv_text(["a", "b", "c"], [[1, 2.5, None], ["x,y", 0.1, "q\"r"]]), 'a,b,c\n1,2.5,\n"x,y",0.1,"q""r"\n')

    def test_round_sig(self):
        self.assertEqual(MR.round_sig({"a": [1.23456789012345, float("nan"), 3, None, True, "s"]}), {"a": [1.23456789, None, 3, None, True, "s"]})


class UpgradeSectionTests(unittest.TestCase):
    """A hand-made study folder: the smoke campaign copied for the baseline and for four candidates with edited numbers."""

    def setUp(self):
        self._t = tempfile.TemporaryDirectory()
        self.root = Path(self._t.name) / "study"
        self.base = self.root / "baseline"
        shutil.copytree(REF, self.base, ignore=shutil.ignore_patterns("campaign_smoke"))
        shutil.copytree(SMOKE, self.base / "campaign" / "smoke")
        self.entries = []

    def tearDown(self):
        self._t.cleanup()

    def candidate(self, cid, *, motor_peak=None, payload=None, endurance=None, status="tested", scaled=False, mass=None, errors=()):
        d = self.root / "candidates" / cid
        e = {"id": cid, "role": "battery", "part_id": cid.split("__")[1], "status": status, "errors": [{"rule": r, "message": r} for r in errors],
             "warnings": [], "dir": f"<study>/candidates/{cid}", "price_usd": None}
        if status in ("tested", "not_tested"):
            shutil.copytree(REF, d, ignore=shutil.ignore_patterns("campaign_smoke"))
            if scaled:
                p = d / "vehicle_build.json"
                doc = json.loads(p.read_text())
                doc["estimates"]["ct_cp_basis"] = "SCALED_STANDIN"
                p.write_text(json.dumps(doc))
            if mass is not None:
                p = d / "vehicle_config.json"
                doc = json.loads(p.read_text())
                doc["mass_kg"] = mass
                p.write_text(json.dumps(doc))
        if status == "tested":
            shutil.copytree(SMOKE, d / "campaign" / "smoke")
            if motor_peak is not None:
                for t in (d / "campaign" / "smoke").glob("*/events.json"):
                    doc = json.loads(t.read_text())
                    for pk in doc["peaks"]:
                        if pk["name"] == "motor_current_a":
                            pk["value"] = motor_peak
                    t.write_text(json.dumps(doc))
            for name, key, val in (("hover_payload", "passing_payload_kg", payload), ("endurance", "duration_s", endurance)):
                if val is not None:
                    p = d / "campaign" / "smoke" / name / "result.json"
                    doc = json.loads(p.read_text())
                    for m in doc["metrics"]:
                        if m["name"] == key:
                            m["value"] = val
                    p.write_text(json.dumps(doc))
        if status == "not_tested":
            e["reason"] = "campaign failed: failed (exit 3)"
        self.entries.append(e)

    def section(self):
        study = {"schema_version": 1, "selected_condition_set": "smoke", "candidates": self.entries}
        (self.root / "study.json").write_text(json.dumps(study))
        report, _ = MR.build_report(self.base, {"smoke": self.base / "campaign" / "smoke"}, None, None, self.root, ["cmd"])
        return report["upgrades"], report

    def test_ranking_deltas_and_flags(self):
        BASE_PEAK = raw_peak("motor_current_a")
        # baseline motor peak = the largest over all tests (read from the raw files); margin = 23 / peak
        self.candidate("battery__good", motor_peak=18.0, payload=3.0, endurance=60.0)       # margin 23 / 18 = 1.27778: gain 0.15339, endurance lost
        self.candidate("battery__worse", motor_peak=23.0)                                    # margin 1.0: loss 0.12439
        self.candidate("battery__scaled", motor_peak=10.0, scaled=True)                      # larger gain but a scaled stand-in table
        self.candidate("battery__broken", status="not_tested")
        self.candidate("battery__nope", status="rejected", errors=["E-CELLS-MOTOR"])
        u, report = self.section()
        by = {e["id"]: e for e in u["entries"]}
        self.assertEqual([e["id"] for e in u["entries"][:2]], ["battery__good", "battery__worse"])
        self.assertEqual((by["battery__good"]["rank"], by["battery__worse"]["rank"]), (1, 2))
        self.assertAlmostEqual(by["battery__good"]["deltas"]["limiting_margin"]["delta"], 23.0 / 18.0 - 23.0 / BASE_PEAK, places=6)
        self.assertAlmostEqual(by["battery__worse"]["deltas"]["limiting_margin"]["delta"], 1.0 - 23.0 / BASE_PEAK, places=6)
        # the swap's own weakest component. The motor current of "good" is capped at 18 A in every test (margin 23 / 18 = 1.2778), but the pack
        # current peak (largest over the raw events) is untouched: margin 125 / peak = 1.2385 is lower, so the PACK becomes the weakest.
        # "scaled" has 10 A (margin 2.3) and the same pack current: pack again.
        pack = 125.0 / raw_peak("pack_current_a")
        self.assertLess(pack, 23.0 / 18.0)
        self.assertEqual(by["battery__good"]["weakest"]["slot"], "pack_current")
        self.assertAlmostEqual(by["battery__good"]["weakest"]["margin"], pack, places=6)
        self.assertEqual(by["battery__scaled"]["weakest"]["slot"], "pack_current")
        self.assertIsNone(by["battery__nope"]["weakest"])
        self.assertEqual(u["baseline_weakest"]["slot"], "motor_current")
        self.assertEqual(by["battery__good"]["deltas"]["max_payload_kg"]["delta"], 0.5)      # 3.0 - 2.5
        self.assertTrue(by["battery__good"]["deltas"]["max_payload_kg"]["better"])
        self.assertAlmostEqual(by["battery__good"]["deltas"]["endurance_s"]["delta"], 60.0 - 70.32, places=9)
        self.assertFalse(by["battery__good"]["deltas"]["endurance_s"]["better"])              # the cost stays visible next to the gain
        self.assertIsNone(by["battery__scaled"]["rank"])
        self.assertIn("scaled stand-in", by["battery__scaled"]["rank_note"])
        self.assertIn("not tested", by["battery__broken"]["status"].replace("_", " "))
        self.assertIn("failed (exit 3)", by["battery__broken"]["rank_note"])
        self.assertEqual(by["battery__nope"]["rank_note"], "not compatible: E-CELLS-MOTOR")
        self.assertIn("SMOKE", u["notice"])
        html = RH.render_html(report, MR.build_plots(MR.D.load_campaign(self.base / "campaign" / "smoke")))
        self.assertEqual(check_html(html).errors, [])
        self.assertIn("battery__good", html)
        # payload +0.5 kg is better (green, with a plus sign); endurance 60 - 70.32 = -10.32 s is worse (red)
        self.assertIn('<td class="num good">+0.5</td>', html)
        self.assertIn('<td class="num bad">-10.32</td>', html)
        self.assertIn("Weakest component after the swap", html)
        csv_rows = RH.csv_files(report)["upgrades.csv"].splitlines()
        self.assertEqual(len(csv_rows), 1 + 5)

    def test_mass_delta_comes_from_the_candidate_config(self):
        base_mass = json.loads((REF / "vehicle_config.json").read_text())["mass_kg"]
        self.candidate("battery__heavy", mass=base_mass + 0.25)
        u, _ = self.section()
        d = u["entries"][0]["deltas"]["mass_kg"]
        self.assertAlmostEqual(d["delta"], 0.25, places=6)
        self.assertFalse(d["better"])

    def test_unusable_candidate_campaign_becomes_not_tested(self):
        self.candidate("battery__bad")
        (self.root / "candidates" / "battery__bad" / "campaign" / "smoke" / "climb" / "events.json").write_text("{not json")
        u, _ = self.section()
        self.assertEqual(u["entries"][0]["status"], "not_tested")
        self.assertIn("unusable", u["entries"][0]["rank_note"])


class CliTests(unittest.TestCase):
    def test_main_writes_a_report_and_refuses_bad_input(self):
        with tempfile.TemporaryDirectory() as t:
            out = Path(t) / "r"
            rc = MR.main(["--build", str(REF), "--campaign", str(SMOKE), "--validation", str(REF / "validation_report.json"), "--out", str(out)])
            self.assertEqual(rc, 0)
            rep = json.loads((out / "report.json").read_text())
            self.assertEqual(rep["conditions"]["selected"], "smoke")
            self.assertNotIn(str(_paths.REPO), rep["reproducibility"]["commands"][0])
            self.assertEqual(MR.main(["--build", str(REF), "--campaign", str(Path(t) / "missing"), "--out", str(out)]), 1)

    def test_recorded_command_has_no_out_and_no_machine_path(self):
        shown = MR.argv_shown(["--build", str(REF), "--out", "/somewhere/else", "--campaign", "/elsewhere/c"])
        self.assertEqual(shown, ["--build", "tools/parts/builds/reference_quad", "--campaign", "c"])

    def test_build_report_refuses_without_campaigns_or_with_a_wrong_selection(self):
        with self.assertRaises(MR.ReportError):
            MR.build_report(REF, {})
        with self.assertRaises(MR.ReportError):
            MR.build_report(REF, {"smoke": SMOKE}, selected="other")

    def test_synthetic_campaign_without_cfd_files(self):
        with tempfile.TemporaryDirectory() as t:
            camp = K.write_campaign(Path(t) / "c", {"climb": {}, "endurance": {}}, definitions_set="1.0.0")
            report, _ = MR.build_report(REF, {"unit": camp})
            self.assertNotIn("smoke", {b["id"] for b in report["banners"]})
            self.assertEqual(report["cfd"]["status"], "no CFD run")
            self.assertEqual(report["validation"]["status"], "not provided")
            self.assertEqual(report["limiting"]["slot"], "motor_current")


class CfdSectionTests(unittest.TestCase):
    """What the report and its banners say about CFD, driven by the campaign's own note (never by the mere presence of files)."""
    SHA = "ab" * 32
    SYNTHETIC = {"status": "CFD surrogate: SYNTHETIC (formula, not CFD)", "level": "danger", "kind": "stub", "model_sha256": SHA,
                 "provenance": "SYNTHETIC formula, not CFD | NOT VALIDATED against any measurement", "holdout_rms_n": "2.5e-9",
                 "cfd_sanity_status": "NOT_RUN", "load_error": None}
    TRAINED = {"status": "CFD surrogate: PRELIMINARY, UNVALIDATED", "level": "warn", "kind": "trained", "model_sha256": SHA,
               "provenance": "preliminary CFD, unvalidated | NOT VALIDATED against any measurement", "holdout_rms_n": "0.0125",
               "cfd_sanity_status": "NOT_RUN", "load_error": None}
    FALLBACK = {"status": "CFD model failed to load: the model is a stub; results use analytic drag", "level": "warn", "kind": None,
                "model_sha256": None, "provenance": None, "holdout_rms_n": None, "cfd_sanity_status": None, "load_error": "the model is a stub"}

    def report(self, note, drag, cfd_dir=None):
        with tempfile.TemporaryDirectory() as t:
            camp = K.write_campaign(Path(t) / "c", {"climb": {}, "endurance": {}}, definitions_set="1.0.0", drag=drag, cfd_note=note)
            return MR.build_report(REF, {"unit": camp}, cfd_dir=cfd_dir)[0]

    def banner_ids(self, rep):
        return {b["id"]: b for b in rep["banners"]}

    def test_synthetic_fixture_is_labelled_synthetic_with_the_red_banner(self):
        rep = self.report(self.SYNTHETIC, f"cfd_surrogate:stub:{self.SHA}")
        self.assertEqual(rep["cfd"]["status"], "CFD surrogate: SYNTHETIC (formula, not CFD)")
        self.assertEqual(rep["cfd"]["level"], "danger")
        self.assertEqual(rep["cfd"]["sanity"]["status"], "NOT_RUN")
        b = self.banner_ids(rep)["stub_model"]
        self.assertEqual(b["level"], "danger")
        self.assertIn("SYNTHETIC (formula, not CFD)", b["text"])
        self.assertNotIn("no_cfd", self.banner_ids(rep))

    def test_trained_model_shows_preliminary_unvalidated_heldout_error_and_not_run(self):
        rep = self.report(self.TRAINED, f"cfd_surrogate:trained:{self.SHA}")
        c = rep["cfd"]
        self.assertEqual((c["status"], c["level"]), ("CFD surrogate: PRELIMINARY, UNVALIDATED", "warn"))
        self.assertEqual(c["holdout_rms_n"], 0.0125)
        self.assertEqual(c["sanity"]["status"], "NOT_RUN")
        self.assertIn("prelim_cfd", self.banner_ids(rep))
        self.assertNotIn("stub_model", self.banner_ids(rep))

    def test_failed_load_with_explicit_fallback_is_amber_and_names_the_reason(self):
        rep = self.report(self.FALLBACK, "analytic_legacy")
        self.assertEqual(rep["cfd"]["level"], "warn")
        self.assertIn("results use analytic drag", rep["cfd"]["status"])
        self.assertIn("the model is a stub", rep["cfd"]["statement"])
        b = self.banner_ids(rep)
        self.assertEqual(b["cfd_load_failed"]["level"], "warn")
        self.assertNotIn("no_cfd", b)

    def test_cfd_files_that_the_campaign_did_not_use_are_never_called_no_cfd_run(self):
        with tempfile.TemporaryDirectory() as t:
            cfd = Path(t) / "cfd"
            cfd.mkdir()
            (cfd / "geometry.json").write_text(json.dumps({"triangles": 408, "stl_sha256": "ab" * 32}), encoding="utf-8")
            rep = self.report(None, "analytic_legacy", cfd)
        self.assertEqual(rep["cfd"]["status"], "CFD results present, NOT used by this campaign (legacy analytic drag)")
        self.assertNotEqual(rep["cfd"]["status"], "no CFD run")
        self.assertEqual(rep["cfd"]["images_statement"], "no flow images")

    def test_empty_or_missing_cfd_folder_is_the_only_way_to_no_cfd_run(self):
        with tempfile.TemporaryDirectory() as t:
            empty = Path(t) / "cfd"
            empty.mkdir()
            self.assertEqual(self.report(None, "analytic_legacy", empty)["cfd"]["status"], "no CFD run")
        self.assertEqual(self.report(None, "analytic_legacy", None)["cfd"]["status"], "no CFD run")

    def test_a_note_that_names_another_model_than_the_drag_label_is_refused(self):
        with self.assertRaises(MR.ReportError):
            self.report(self.SYNTHETIC, "cfd_surrogate:stub:" + "cd" * 32)

    def test_flow_images_are_embedded_with_their_legend(self):
        png = b"\x89PNG\r\n\x1a\nnot-a-real-image"
        with tempfile.TemporaryDirectory() as t:
            cfd = Path(t) / "cfd"
            cfd.mkdir()
            (cfd / "slice_b.png").write_bytes(png)
            (cfd / "slice_a.png").write_bytes(png)
            (cfd / "slice_a.legend.txt").write_text("0 to 15 m/s, PRELIMINARY CFD, UNVALIDATED\n", encoding="utf-8")
            rep = self.report(self.TRAINED, f"cfd_surrogate:trained:{self.SHA}", cfd)
        c = rep["cfd"]
        self.assertEqual([i["name"] for i in c["images"]], ["slice_a.png", "slice_b.png"])      # sorted: deterministic
        self.assertEqual(c["images"][0]["legend"], "0 to 15 m/s, PRELIMINARY CFD, UNVALIDATED")
        self.assertIsNone(c["images"][1]["legend"])
        self.assertTrue(c["images"][0]["data_uri"].startswith("data:image/png;base64,"))
        self.assertEqual(c["images_statement"], "2 flow image(s)")

    def test_html_shows_status_holdout_sanity_and_images_sentence(self):
        rep = self.report(self.TRAINED, f"cfd_surrogate:trained:{self.SHA}")
        html = RH.s_cfd(rep)
        for text in ("CFD surrogate: PRELIMINARY, UNVALIDATED", "held-out RMS error 0.0125 N", "NOT_RUN", "no flow images"):
            self.assertIn(text, html)
        self.assertIn('class="banner warn"', html)

    def test_tests_of_one_campaign_must_carry_the_same_note(self):
        with tempfile.TemporaryDirectory() as t:
            camp = K.write_campaign(Path(t) / "c", {"climb": {}, "endurance": {}}, drag=f"cfd_surrogate:stub:{self.SHA}", cfd_note=self.SYNTHETIC)
            m = camp / "endurance" / "manifest.json"
            doc = json.loads(m.read_text())
            del doc["cfd_surrogate"]
            m.write_text(json.dumps(doc))
            with self.assertRaises(D.CampaignDataError):
                D.load_campaign(camp)


if __name__ == "__main__":
    unittest.main()
