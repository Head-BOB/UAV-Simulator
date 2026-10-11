# Tier: D
import tempfile
import unittest
from pathlib import Path

import campaign_data as D
import testkit as K


class LoadTests(unittest.TestCase):
    def make(self, tmp, **kw):
        return K.write_campaign(Path(tmp) / "c", {"climb": {"metrics": {"time_to_target_s": 3.5}}, "endurance": {}}, **kw)

    def test_loads_and_exposes_metrics_and_peaks(self):
        with tempfile.TemporaryDirectory() as t:
            c = D.load_campaign(self.make(t))
            self.assertEqual(sorted(c["tests"]), ["climb", "endurance"])
            self.assertEqual(D.metric(c, "climb", "time_to_target_s"), 3.5)
            self.assertIsNone(D.metric(c, "climb", "nope"))
            self.assertIsNone(D.metric(c, "hover_payload", "x"))
            self.assertEqual(c["tests"]["climb"]["peaks"]["motor_current_a"], {"value": 20.0, "t_s": 1.0})
            self.assertEqual(c["common"]["conditions"]["condition_set"], "unit")
            self.assertEqual(c["tests"]["climb"]["series"]["timeseries_climb.csv"]["t_s"], [0.0, 1.0])

    def test_refuses_a_missing_required_column(self):
        with tempfile.TemporaryDirectory() as t:
            root = self.make(t)
            p = root / "climb" / "timeseries_climb.csv"
            p.write_text(p.read_text().replace("esc_loss_w", "esc_loss_x"))
            with self.assertRaises(D.CampaignDataError) as e:
                D.load_campaign(root)
            self.assertIn("esc_loss_w", str(e.exception))

    def test_refuses_ragged_rows_and_non_numbers_and_wrong_row_count(self):
        for edit, word in ((lambda s: s + "1,2\n", "cells"), (lambda s: s.replace("0.0,", "abc,", 1), "not a number")):
            with tempfile.TemporaryDirectory() as t:
                root = self.make(t)
                p = root / "climb" / "timeseries_climb.csv"
                p.write_text(edit(p.read_text()))
                with self.assertRaises(D.CampaignDataError) as e:
                    D.load_campaign(root)
                self.assertIn(word, str(e.exception))
        with tempfile.TemporaryDirectory() as t:
            root = self.make(t)
            p = root / "climb" / "timeseries_climb.csv"
            lines = p.read_text().splitlines()
            p.write_text("\n".join(lines[:-1]) + "\n")
            with self.assertRaises(D.CampaignDataError) as e:
                D.load_campaign(root)
            self.assertIn("row count", str(e.exception))

    def test_refuses_disagreeing_manifests_and_schema_and_outcome(self):
        import json
        with tempfile.TemporaryDirectory() as t:
            root = self.make(t)
            m = root / "endurance" / "manifest.json"
            d = json.loads(m.read_text())
            d["conditions"]["ambient_c"] = 25
            m.write_text(json.dumps(d))
            with self.assertRaises(D.CampaignDataError) as e:
                D.load_campaign(root)
            self.assertIn("conditions", str(e.exception))
        with tempfile.TemporaryDirectory() as t:
            root = self.make(t)
            p = root / "campaign_result.json"
            p.write_text(p.read_text().replace('"schema_version": 1', '"schema_version": 2', 1))
            with self.assertRaises(D.CampaignDataError):
                D.load_campaign(root)
        with tempfile.TemporaryDirectory() as t:
            root = self.make(t)
            p = root / "climb" / "result.json"
            p.write_text(p.read_text().replace('"COMPLETED"', '"STEADY_STATE"'))
            with self.assertRaises(D.CampaignDataError) as e:
                D.load_campaign(root)
            self.assertIn("outcome", str(e.exception))

    def test_refuses_missing_file_and_empty_campaign(self):
        with tempfile.TemporaryDirectory() as t:
            with self.assertRaises(D.CampaignDataError):
                D.load_campaign(Path(t) / "nothing")
        with tempfile.TemporaryDirectory() as t:
            root = Path(t) / "c"
            K.write_campaign(root, {})
            with self.assertRaises(D.CampaignDataError):
                D.load_campaign(root)


if __name__ == "__main__":
    unittest.main()
