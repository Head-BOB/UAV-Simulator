# Tier: D
import tempfile
import unittest
from pathlib import Path

import assumptions as A
import campaign_data as D
import testkit as K

SIDECAR = {"assumed_inputs": [{"part_id": "bat_a", "field": "internal_resistance_ohm", "note": "typical"}],
           "builder_assumptions": ["layout slots"],
           "powertrain": {"assumed_part_inputs": [{"part_id": "bat_a", "field": "internal_resistance_ohm"},
                                                  {"part_id": "mot_a", "field": "winding_temp_limit_c"}]},
           "estimates": {"ct_cp_basis": "UIUC_MEASURED"}}
CONTROLLER = {"vehicle_class": "quad_standard", "provenance": {"source_type": "ASSUMED", "source_file": "parts_db/controller_classes.json"}}
VALIDATION = {"sections": [{"name": "motor_electrical", "rows": [
    {"id": "r1", "quantity": "motor_voltage", "percent_error": 13.98, "band_percent": 20, "outside_band": False},
    {"id": "r2", "quantity": "motor_voltage", "percent_error": -11.02, "band_percent": 20, "outside_band": False},
    {"id": "r3", "quantity": "motor_voltage", "percent_error": 10.0, "band_percent": 20, "outside_band": False},
    {"id": "r4", "quantity": "motor_voltage", "percent_error": -9.99, "band_percent": 20, "outside_band": False},
    {"id": "r5", "quantity": "x", "percent_error": None, "band_percent": 20, "outside_band": False}]}]}


def campaign(tests, **kw):
    t = tempfile.TemporaryDirectory()
    c = D.load_campaign(K.write_campaign(Path(t.name) / "c", tests, **kw))
    t.cleanup()
    return c


def items(rows):
    return [r["item"] for r in rows]


class AssumptionTests(unittest.TestCase):
    def test_validation_flags_are_above_ten_percent_only(self):
        f = A.validation_flags(VALIDATION)
        self.assertEqual([r["id"] for r in f], ["r1", "r2"])      # exactly 10.0 and -9.99 and None are not flagged
        self.assertEqual(A.validation_flags(None), [])

    def test_rows_for_a_synthetic_build(self):
        c = campaign({"climb": {}, "wind_gust": {"notes": ["inputs are ASSUMED here", "plain note"]}}, conditions=dict(K.CONDITIONS, wind_speed_mps=5))
        rows = A.collect(SIDECAR, K.POWERTRAIN, CONTROLLER, c, VALIDATION, {"bat_a": "Unverified", "mot_a": "Verified"})
        it = items(rows)
        # part inputs once per (part, field) even though two sources list the battery resistance
        self.assertEqual(sum("bat_a.internal_resistance_ohm" in i for i in it), 1)
        self.assertIn("mot_a.winding_temp_limit_c", it)
        self.assertIn("layout slots", it)
        self.assertTrue(any("parameters_verified" in i for i in it))
        self.assertTrue(any("controller class" in i for i in it))
        self.assertTrue(any(i.startswith("climb v1") for i in it) and any(i.startswith("wind_gust v1") for i in it))
        self.assertTrue(any("Dryden" in i for i in it))                        # wind 5 m/s, dryden selected
        self.assertTrue(any("legacy analytic" in i for i in it))
        self.assertIn("inputs are ASSUMED here", it)
        self.assertNotIn("plain note", it)
        self.assertIn("bat_a: Unverified", it)
        self.assertFalse(any("mot_a: " in i for i in it))                       # verified parts add nothing
        self.assertTrue(any("r1" in i for i in it) and any("r2" in i for i in it))
        self.assertEqual([r["id"] for r in rows], [f"A{i:03d}" for i in range(1, len(rows) + 1)])

    def test_rows_that_must_be_absent(self):
        # no wind, no wind test -> turbulence did not shape the result; verified basis; no validation; CFD model named instead of legacy
        c = campaign({"climb": {}}, drag="cfd_surrogate:trained:abc")
        side = dict(SIDECAR, assumed_inputs=[], builder_assumptions=[], powertrain={})
        rows = A.collect(side, dict(K.POWERTRAIN, parameters_verified=1), {"provenance": {}}, c, None, {})
        it = items(rows)
        self.assertFalse(any("Dryden" in i for i in it))
        self.assertFalse(any("legacy analytic" in i for i in it))
        self.assertTrue(any("cfd_surrogate:trained:abc" in i for i in it))
        self.assertFalse(any("parameters_verified" in i for i in it))
        self.assertFalse(any("controller" in r["category"] for r in rows))
        rows_stub = A.collect(side, K.POWERTRAIN, {"provenance": {}}, campaign({"climb": {}}, drag="cfd_surrogate:stub:abc"), None, {})
        self.assertTrue(any("STUB" in r["effect"] for r in rows_stub))

    def test_scaled_propeller_basis_and_wind_test_trigger_dryden(self):
        c = campaign({"wind_gust": {}})                                        # wind 0 but the wind test ran
        side = dict(SIDECAR, estimates={"ct_cp_basis": "SCALED_STANDIN"})
        it = items(A.collect(side, K.POWERTRAIN, CONTROLLER, c, None, {}))
        self.assertTrue(any("SCALED_STANDIN" in i for i in it))
        self.assertTrue(any("Dryden" in i for i in it))

    def test_drag_kind(self):
        self.assertEqual(A.drag_kind("analytic_legacy"), "analytic")
        self.assertEqual(A.drag_kind("cfd_surrogate:stub:ab"), "cfd_stub")
        self.assertEqual(A.drag_kind("cfd_surrogate:trained:ab"), "cfd")

    def test_unnamed_condition_set(self):
        c = campaign({"climb": {}}, conditions={k: v for k, v in K.CONDITIONS.items() if k != "condition_set"})
        it = items(A.collect(SIDECAR, K.POWERTRAIN, CONTROLLER, c, None, {}))
        self.assertTrue(any(i.startswith("condition set unnamed") for i in it))


if __name__ == "__main__":
    unittest.main()
