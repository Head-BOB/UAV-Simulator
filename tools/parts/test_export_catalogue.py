# Tier: D
import json
import tempfile
import unittest
from pathlib import Path

import export_catalogue as X
import partsdb as P


class UnitTests(unittest.TestCase):
    def test_units_from_suffix_and_exceptions(self):
        want = {"mass_kg": "kg", "kv_rpm_per_v": "rpm/V", "thermal_resistance_k_per_w": "K/W", "projected_area_top_m2": "m^2",
                "arm_allowable_bending_stress_pa": "Pa", "winding_resistance_ohm": "ohm", "capacity_ah": "Ah", "diameter_m": "m",
                "max_current_a": "A", "voltage_per_cell_nominal_v": "V", "max_power_w": "W", "thermal_time_constant_s": "s",
                "temp_limit_c": "C", "winding_temp_limit_c": "C", "max_continuous_c": "C rate", "measured_rpm_max": "rpm",
                "cells": "", "channels": "", "efficiency": "", "blades": ""}
        for name, unit in want.items():
            self.assertEqual(X.unit_of(name), unit, name)


class CatalogueTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.cat = X.build_catalogue()
        cls.records, _ = P.load_records()

    def test_counts_per_category_equal_the_database(self):
        for cat in self.cat["categories"]:
            n_db = sum(1 for r in self.records.values() if r["category"] == cat)
            self.assertEqual(sum(1 for p in self.cat["parts"] if p["category"] == cat), n_db, cat)
        self.assertEqual(len(self.cat["parts"]), len(self.records))

    def test_a_known_part_by_hand(self):
        p = next(x for x in self.cat["parts"] if x["id"] == "tmotor_mn3508_kv700")
        rec = self.records["tmotor_mn3508_kv700"]
        kv = next(f for f in p["key_fields"] if f["name"] == "kv_rpm_per_v")
        self.assertEqual(kv["value"], rec["kv_rpm_per_v"]["value"])
        self.assertEqual((kv["unit"], kv["source_type"]), ("rpm/V", rec["kv_rpm_per_v"]["provenance"]["source_type"]))
        self.assertEqual(p["price_usd"], 69.9)
        self.assertEqual(p["verification"], "Unverified")                        # the tick file is empty today
        self.assertEqual(p["manufacturer"], "T-Motor")

    def test_every_key_field_has_a_source_type_and_a_unit_or_none(self):
        for p in self.cat["parts"]:
            self.assertTrue(p["key_fields"], p["id"])
            for f in p["key_fields"]:
                self.assertIn(f["source_type"], ("MEASURED", "CITED", "CALCULATED", "ASSUMED"), p["id"])
                self.assertIsInstance(f["unit"], str)

    def test_missing_price_is_null_not_zero(self):
        p = next(x for x in self.cat["parts"] if x["id"] == "ref_battery_generic_4s_5000")
        self.assertIsNone(p["price_usd"])
        self.assertIsNone(p["price_source_type"])

    def test_deterministic_sorted_and_the_committed_file_is_current(self):
        self.assertEqual(X.render(), X.render())
        ids = [(p["category"], p["id"]) for p in self.cat["parts"]]
        self.assertEqual(ids, sorted(ids))
        self.assertEqual(X.main(["--check"]), 0)

    def test_check_detects_a_stale_file_and_main_writes(self):
        with tempfile.TemporaryDirectory() as t:
            out = Path(t) / "c.json"
            self.assertEqual(X.main(["--out", str(out), "--check"]), 2)          # missing
            self.assertEqual(X.main(["--out", str(out)]), 0)
            self.assertEqual(X.main(["--out", str(out), "--check"]), 0)
            out.write_text(out.read_text().replace('"CITED"', '"ASSUMED"', 1))
            self.assertEqual(X.main(["--out", str(out), "--check"]), 2)
            self.assertEqual(json.loads(out.read_text())["schema_version"], 1)


if __name__ == "__main__":
    unittest.main()
