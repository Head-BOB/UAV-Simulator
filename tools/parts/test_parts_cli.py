# Tier: D
"""Tests for the verification ticks / sheet and the command line."""
import contextlib
import io
import json
import unittest

import parts_cli as C
import partsdb as P
import verification as V
from testsupport import TempDb, synth


def run(args):
    out, err = io.StringIO(), io.StringIO()
    with contextlib.redirect_stdout(out), contextlib.redirect_stderr(err):
        code = C.main(args)
    return code, out.getvalue(), err.getvalue()


class VerificationTests(unittest.TestCase):
    def setUp(self):
        self.db = TempDb()
        self.addCleanup(self.db.close)
        self.rec = synth("wiring", "wire_a", mass_kg=0.12)  # wiring key fields: mass_kg only
        self.db.add(self.rec)
        self.rec2 = synth("esc", "esc_a")
        self.db.add(self.rec2)

    def test_untouched_part_is_unverified(self):
        self.assertEqual(V.part_status(self.rec, []), "Unverified")

    def test_ticking_all_key_fields_verifies(self):
        V.add_tick(self.db.root, self.rec, "mass_kg", "A. Reviewer", "2026-10-10")
        self.assertEqual(V.part_status(self.rec, V.load_ticks(self.db.root)), "Verified")

    def test_partial_ticks_are_partly_verified(self):
        V.add_tick(self.db.root, self.rec2, "mass_kg", "A. Reviewer", "2026-10-10")
        self.assertEqual(V.part_status(self.rec2, V.load_ticks(self.db.root)), "Partly verified")

    def test_changed_value_makes_the_tick_stale(self):
        V.add_tick(self.db.root, self.rec, "mass_kg", "A. Reviewer", "2026-10-10")
        changed = synth("wiring", "wire_a", mass_kg=0.13)
        ticks = V.load_ticks(self.db.root)
        self.assertEqual(V.tick_state(changed, "mass_kg", ticks), "stale")
        self.assertEqual(V.part_status(changed, ticks), "Unverified")

    def test_tick_requires_key_field_and_name(self):
        with self.assertRaises(ValueError):
            V.add_tick(self.db.root, self.rec, "price_usd", "A", "2026-10-10")
        with self.assertRaises(ValueError):
            V.add_tick(self.db.root, self.rec, "mass_kg", "  ", "2026-10-10")

    def test_motor_key_fields_are_exactly_the_declared_eight(self):
        # hand list from fields.py: the 8 values the datasheet check must cover for a motor
        self.assertEqual(V.key_fields(synth("motor")),
                         ["mass_kg", "kv_rpm_per_v", "winding_resistance_ohm", "max_current_a",
                          "max_power_w", "cells_max", "shaft_diameter_m", "bounding_box_m"])

    def test_key_fields_capped_at_eight_and_only_present(self):
        for cat in P.F.CATEGORIES:
            rec = synth(cat)
            self.assertLessEqual(len(V.key_fields(rec)), 8)
            for f in V.key_fields(rec):
                self.assertIn(f, rec)

    def test_sheet_lists_every_part_and_marks_ticks(self):
        V.add_tick(self.db.root, self.rec, "mass_kg", "A. Reviewer", "2026-10-10")
        sheet = V.render_sheet(self.db.root)
        self.assertIn("## wire_a (wiring)", sheet)
        self.assertIn("[Verified]", sheet)
        self.assertIn("- [x] `mass_kg` = 0.12", sheet)
        self.assertIn("## esc_a (esc)", sheet)
        self.assertIn("- [ ] `max_continuous_current_a`", sheet)

    def test_sheet_shows_stale_marker(self):
        V.add_tick(self.db.root, self.rec, "mass_kg", "A. Reviewer", "2026-10-10")
        self.db.add(synth("wiring", "wire_a", mass_kg=0.2))
        self.assertIn("[ ] STALE `mass_kg`".replace("[ ] STALE `", "[ ] STALE `"), V.render_sheet(self.db.root))

    def test_sheet_is_deterministic(self):
        self.assertEqual(V.render_sheet(self.db.root), V.render_sheet(self.db.root))


class CliTests(unittest.TestCase):
    def setUp(self):
        self.db = TempDb()
        self.addCleanup(self.db.close)
        self.base = ["--db", str(self.db.root)]
        self.db.add(synth("esc", "esc_a", mass_kg=0.02))
        self.db.add(synth("esc", "esc_b", mass_kg=0.04))

    def test_validate_ok_and_error_exit_codes(self):
        code, out, _ = run(self.base + ["validate"])
        self.assertEqual(code, 0)
        self.assertIn("2 records, 0 errors", out)
        bad = synth("esc", "esc_c")
        bad["efficiency"]["value"] = 5
        self.db.add(bad)
        code, _, err = run(self.base + ["validate"])
        self.assertEqual(code, 1)
        self.assertIn("ERROR esc_c", err)

    def test_search_prints_matches_with_status(self):
        code, out, _ = run(self.base + ["search", "--category", "esc", "--max-mass", "0.03"])
        self.assertEqual(code, 0)
        lines = out.strip().splitlines()
        self.assertEqual(len(lines), 1)
        self.assertTrue(lines[0].startswith("esc_a\tesc\t"))
        self.assertTrue(lines[0].endswith("Unverified"))

    def test_show_and_unknown_part(self):
        code, out, _ = run(self.base + ["show", "esc_a"])
        self.assertEqual(code, 0)
        self.assertEqual(json.loads(out)["id"], "esc_a")
        code, _, err = run(self.base + ["show", "nope"])
        self.assertEqual(code, 1)
        self.assertIn("no part", err)

    def test_add_valid_and_incomplete(self):
        src = self.db.repo / "new.json"
        src.write_text(json.dumps(synth("esc", "esc_new")), encoding="utf-8")
        code, out, _ = run(self.base + ["add", "--from", str(src)])
        self.assertEqual(code, 0)
        self.assertTrue((self.db.root / "parts" / "esc" / "esc_new.json").exists())
        rec = synth("esc", "esc_bad")
        del rec["efficiency"]["provenance"]
        src.write_text(json.dumps(rec), encoding="utf-8")
        code, _, err = run(self.base + ["add", "--from", str(src)])
        self.assertEqual(code, 1)
        self.assertIn("add refused", err)
        self.assertFalse((self.db.root / "parts" / "esc" / "esc_bad.json").exists())

    def test_add_unreadable_file(self):
        code, _, err = run(self.base + ["add", "--from", str(self.db.repo / "missing.json")])
        self.assertEqual(code, 1)
        self.assertIn("add refused", err)

    def test_schema_write_then_check(self):
        code, _, _ = run(self.base + ["schema", "--check"])
        self.assertEqual(code, 1)  # not written yet
        run(self.base + ["schema"])
        code, out, _ = run(self.base + ["schema", "--check"])
        self.assertEqual(code, 0)
        self.assertIn("up to date", out)

    def test_verify_sheet_check_detects_staleness(self):
        run(self.base + ["verify-sheet"])
        code, _, _ = run(self.base + ["verify-sheet", "--check"])
        self.assertEqual(code, 0)
        self.db.add(synth("esc", "esc_late"))
        code, _, err = run(self.base + ["verify-sheet", "--check"])
        self.assertEqual(code, 1)
        self.assertIn("stale", err)

    def test_tick_command(self):
        code, out, _ = run(self.base + ["tick", "esc_a", "mass_kg", "--by", "Rev"])
        self.assertEqual(code, 0)
        ticks = json.loads((self.db.root / "human_verification.json").read_text(encoding="utf-8"))
        self.assertEqual(ticks[0]["part_id"], "esc_a")
        self.assertEqual(ticks[0]["value"], 0.02)
        code, _, err = run(self.base + ["tick", "esc_a", "price_usd", "--by", "Rev"])
        self.assertEqual(code, 1)
        self.assertIn("tick refused", err)


if __name__ == "__main__":
    unittest.main()
