# Tier: D
import unittest

import _paths  # noqa: F401
import partsdb as P
import upgrades as U

import json

REF = json.loads((_paths.PARTS / "builds" / "reference_quad.build.json").read_text(encoding="utf-8"))


class SwapTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.records, _ = P.load_records()

    def test_candidate_counts_and_content_against_the_database(self):
        cands = U.swap_specs(REF, self.records)
        per_role = {r: sum(1 for c in cands if c["role"] == r) for r in U.ROLES}
        in_db = {r: sum(1 for x in self.records.values() if x["category"] == r) for r in U.ROLES}
        # one fewer than the database has in each role: the part already used is not a candidate
        for r in U.ROLES:
            self.assertEqual(per_role[r], in_db[r] - 1, r)
        for c in cands:
            changed = [k for k in REF["parts"] if c["spec"]["parts"][k] != REF["parts"][k]]
            self.assertEqual(changed, [c["role"]])                      # exactly one role changes, nothing else
            self.assertEqual(c["spec"]["parts"][c["role"]], c["part_id"])
            self.assertNotEqual(c["part_id"], REF["parts"][c["role"]])
            self.assertEqual(c["id"], f"{c['role']}__{c['part_id']}")
        self.assertEqual([c["id"] for c in cands], sorted(c["id"] for c in cands))
        self.assertEqual(REF["parts"]["motor"], "tmotor_mn3508_kv700")  # the baseline spec was not modified in place

    def test_incompatible_candidates_are_rejected_with_a_rule_and_compatible_ones_build(self):
        cands = U.swap_specs(REF, self.records)
        checked = [U.check_candidate(c) for c in cands]
        statuses = {c["status"] for c in checked}
        self.assertEqual(statuses <= {"buildable", "rejected"}, True)
        for c in checked:
            if c["status"] == "rejected":
                self.assertTrue(c["errors"] and all(e["rule"] for e in c["errors"]))
                self.assertIsNone(c["build"])
            else:
                self.assertTrue(c["build"].ok and not c["errors"])

    def test_price_by_hand(self):
        recs = {"m": {"price_usd": {"value": 50.0}}, "p": {"price_usd": {"value": 5.0}}, "b": {"price_usd": {"value": 80.0}},
                "e1": {"price_usd": {"value": 20.0}, "channels": {"value": 1}}, "e4": {"price_usd": {"value": 90.0}, "channels": {"value": 4}},
                "f": {"price_usd": {"value": 30.0}}}
        spec = {"parts": {"motor": "m", "propeller": "p", "battery": "b", "esc": "e1", "frame": "f"}}
        # 4 x 50 + 4 x 5 + 80 + 4 x 20 + 30 = 410
        self.assertEqual(U.parts_price_usd(spec, recs), 410.0)
        spec["parts"]["esc"] = "e4"                                       # a 4-in-1 ESC counts once: 200 + 20 + 80 + 90 + 30 = 420
        self.assertEqual(U.parts_price_usd(spec, recs), 420.0)
        del recs["p"]["price_usd"]
        self.assertIsNone(U.parts_price_usd(spec, recs))


BASE = {"limiting_margin": 1.2, "max_payload_kg": 2.5, "endurance_s": 1000.0, "peak_winding_c": 60.0, "mass_kg": 1.5, "price_usd": None,
        "climb_time_s": 4.0, "energy_wh": 50.0, "max_speed_mps": None, "max_wind_mps": 10.0, "min_thrust_reserve": 2.0,
        "peak_esc_c": 40.0, "peak_battery_c": 30.0}


class CompareTests(unittest.TestCase):
    def test_deltas_and_direction(self):
        cand = dict(BASE, limiting_margin=1.5, endurance_s=900.0, peak_winding_c=55.0, mass_kg=1.5, climb_time_s=5.0, energy_wh=60.0)
        d = U.deltas(BASE, cand)
        self.assertAlmostEqual(d["limiting_margin"]["delta"], 0.3, places=12)
        self.assertTrue(d["limiting_margin"]["better"])
        self.assertTrue(d["endurance_s"]["delta"] == -100.0 and d["endurance_s"]["better"] is False)     # worse: lower endurance
        self.assertEqual((d["peak_winding_c"]["delta"], d["peak_winding_c"]["better"]), (-5.0, True))    # cooler is better
        self.assertEqual((d["climb_time_s"]["delta"], d["climb_time_s"]["better"]), (1.0, False))        # slower is worse
        self.assertIsNone(d["energy_wh"]["better"])                                                      # no preference
        self.assertIsNone(d["mass_kg"]["better"])                                                        # unchanged
        self.assertIsNone(d["price_usd"]["delta"])                                                       # unknown price
        self.assertIsNone(d["max_speed_mps"]["delta"])                                                   # not tested

    def entry(self, cid, margin, status="tested", scaled=False):
        cand = dict(BASE, limiting_margin=margin)
        return {"id": cid, "status": status, "scaled_basis": scaled, "deltas": U.deltas(BASE, cand)}

    def test_ranking_by_limiting_margin_gain_with_a_gain_that_costs_endurance(self):
        a = self.entry("a", 1.5)                       # +0.3
        b = self.entry("b", 2.0)                       # +0.8, best
        b["deltas"]["endurance_s"] = U.deltas(BASE, dict(BASE, endurance_s=500.0))["endurance_s"]   # but halves the endurance
        c = self.entry("c", 1.0)                       # -0.2: ranked last of the tested
        d = self.entry("d", 3.0, scaled=True)          # large gain but ASSUMED prop table: unranked
        e = self.entry("e", 9.0, status="not_tested")
        out = U.rank([a, b, c, d, e])
        self.assertEqual([x["id"] for x in out], ["b", "a", "c", "d", "e"])
        self.assertEqual([x["rank"] for x in out], [1, 2, 3, None, None])
        self.assertFalse(out[0]["deltas"]["endurance_s"]["better"])              # the cost stays visible next to the rank
        self.assertIn("scaled stand-in", out[3]["rank_note"])
        self.assertEqual(out[4]["rank_note"], "not tested")

    def test_ties_break_by_id(self):
        out = U.rank([self.entry("z", 1.5), self.entry("m", 1.5)])
        self.assertEqual([x["id"] for x in out], ["m", "z"])

    def test_missing_limiting_margin_is_unranked(self):
        e = self.entry("a", 1.5)
        e["deltas"]["limiting_margin"]["delta"] = None
        self.assertIsNone(U.rank([e])[0]["rank"])


if __name__ == "__main__":
    unittest.main()
