# Tier: D
"""Tests for validation_files and extract_tmotor_table: the saved MN3508 page gives the 20 published rows; refusals on bad input."""
import copy
import json
import tempfile
import unittest
from pathlib import Path

import extract_tmotor_table as X
import partsdb as P
import validation_files as VF

VALDIR = P.DEFAULT_DB / "validation"
TABLE = json.loads((VALDIR / VF.MOTOR_TABLE_FILE).read_text(encoding="utf-8"))
BANDS = json.loads((VALDIR / VF.BANDS_FILE).read_text(encoding="utf-8"))

FIXTURE = ("junk MN3508 KV700 11.1 T-MOTOR 12*4CF 50% 2.3 25.53 280 3800 10.97 42 65% 4.5 49.95 460 4900 9.21 "
           "14.8 T-MOTOR 11*3.7CF 50% 3.1 45.88 380 5300 8.28 41 100% 12.7 187.96 1110 8500 5.91 Notes: tail")


class ParseTests(unittest.TestCase):
    def test_fixture_rows_and_temperature_only_on_first_row_of_a_prop(self):
        rows = X.parse_rows(FIXTURE)
        self.assertEqual(rows, [(11.1, "12*4CF", 50.0, 2.3, 25.53, 280.0, 3800.0, 10.97, 42.0),
                                (11.1, "12*4CF", 65.0, 4.5, 49.95, 460.0, 4900.0, 9.21, None),
                                (14.8, "11*3.7CF", 50.0, 3.1, 45.88, 380.0, 5300.0, 8.28, 41.0),
                                (14.8, "11*3.7CF", 100.0, 12.7, 187.96, 1110.0, 8500.0, 5.91, None)])

    def test_missing_markers_and_garbage_are_refused(self):
        for text in ("nothing here", "MN3508 KV700 11.1 T-MOTOR 12*4CF 50% 2.3 Notes:", "MN3508 KV700 11.1 T-MOTOR 12*4CF 50% 2.3 25.53 280 3800 10.97"):
            with self.assertRaises(X.ExtractError, msg=text):
                X.parse_rows(text)
        with self.assertRaises(X.ExtractError):
            X.parse_rows("MN3508 KV700 11.1 T-MOTOR 12*4CF 50% 2.3 25.53 280 3800 10.97 bogus Notes:")

    def test_page_text_drops_scripts_and_tags(self):
        self.assertEqual(X.page_text("<p>a</p><script>var x=1;</script><b> b </b>"), " a b ")

    def test_document_ids_units_and_size_parse(self):
        rows = X.parse_rows(FIXTURE)
        pin = {"cache_file": "c", "sha256": "0" * 64, "retrieved_on": "2026-10-10", "bytes": 1}
        doc = X.to_document(rows, pin)
        self.assertEqual(doc["rows"][2]["id"], "kv700_14.8v_11x3.7cf_50pct")
        self.assertEqual((doc["rows"][2]["prop_diameter_in"], doc["rows"][2]["prop_pitch_in"]), (11.0, 3.7))
        bad = list(rows[0])
        bad[1] = "XL"
        with self.assertRaises(X.ExtractError):
            X.to_document([tuple(bad)], pin)


def page_from_table(doc, notes_before=True) -> str:
    """HTML text that carries the committed rows in the page's flat token layout (to test extract() end to end)."""
    parts = ["<html><body><p>Notes: earlier table</p>" if notes_before else "<html><body>", "<p>MN3508 KV700</p><p>MN3508 KV700"]
    last_v = last_prop = None
    for r in doc["rows"]:
        if r["supply_voltage_v"] != last_v:
            last_v, last_prop = r["supply_voltage_v"], None
            parts.append(f" {r['supply_voltage_v']:g} ")
        if r["prop"] != last_prop:
            last_prop = r["prop"]
            parts.append(f" {r['prop']} ")
        temp = "" if r["temperature_c"] is None else f" {r['temperature_c']:g}"
        parts.append(f" {r['throttle_percent']:g}% {r['current_a']:g} {r['power_w']:g} {r['thrust_g']:g} {r['rpm']:g} {r['efficiency_g_per_w']:g}{temp} ")
    parts.append(" Notes: tail</p></body></html>")
    return "".join(parts)


class ExtractEndToEndTests(unittest.TestCase):
    def make_db(self, html: str, rows_ok=True):
        import hashlib
        t = tempfile.TemporaryDirectory()
        self.addCleanup(t.cleanup)
        db = Path(t.name)
        (db / "source_cache").mkdir()
        (db / "source_cache" / "cachefile").write_bytes(html.encode("utf-8"))
        pin = {"cache_file": "cachefile", "sha256": hashlib.sha256(html.encode("utf-8")).hexdigest(), "retrieved_on": "2026-10-10", "bytes": len(html)}
        (db / "source_pins.json").write_text(json.dumps({X.PAGE_URL: pin}), encoding="utf-8")
        return db, pin

    def test_round_trip_of_the_committed_rows_through_a_synthetic_page(self):
        # the page text is rebuilt from the committed rows, with an EARLIER 'Notes:' (the real page has two): extraction must stop at the next one
        db, _ = self.make_db("<html>" + page_from_table(TABLE))
        doc = X.extract(db)
        self.assertEqual([r["id"] for r in doc["rows"]], [r["id"] for r in TABLE["rows"]])
        self.assertEqual([(r["current_a"], r["rpm"], r["temperature_c"]) for r in doc["rows"]], [(r["current_a"], r["rpm"], r["temperature_c"]) for r in TABLE["rows"]])

    def test_extract_refuses_wrong_hash_wrong_count_and_missing_cache(self):
        db, pin = self.make_db(page_from_table(TABLE))
        pins = json.loads((db / "source_pins.json").read_text())
        pins[X.PAGE_URL]["sha256"] = "0" * 64
        (db / "source_pins.json").write_text(json.dumps(pins), encoding="utf-8")
        with self.assertRaises(X.ExtractError):
            X.extract(db)
        short = copy.deepcopy(TABLE)
        short["rows"] = short["rows"][:-1]
        db2, _ = self.make_db(page_from_table(short))
        with self.assertRaises(X.ExtractError):
            X.extract(db2)
        (db2 / "source_cache" / "cachefile").unlink()
        with self.assertRaises(X.ExtractError):
            X.extract(db2)

    def test_a_stray_number_is_refused_not_taken_as_a_voltage(self):
        with self.assertRaises(X.ExtractError):
            X.parse_rows("MN3508 KV700 11.1 T-MOTOR 12*4CF 50% 2.3 25.53 280 3800 10.97 99 98 65% 4.5 49.95 460 4900 9.21 Notes:")

    def test_an_earlier_notes_marker_is_ignored(self):
        text = "Notes: earlier " + FIXTURE
        self.assertEqual(len(X.parse_rows(text)), 4)


class RealPageTests(unittest.TestCase):
    def test_committed_table_has_the_20_published_rows(self):
        self.assertEqual(len(TABLE["rows"]), 20)
        first, last = TABLE["rows"][0], TABLE["rows"][-1]
        self.assertEqual((first["supply_voltage_v"], first["prop"], first["throttle_percent"], first["current_a"], first["power_w"], first["thrust_g"], first["rpm"]),
                         (11.1, "T-MOTOR 12*4CF", 50.0, 2.3, 25.53, 280.0, 3800.0))
        self.assertEqual((last["supply_voltage_v"], last["throttle_percent"], last["current_a"], last["power_w"], last["thrust_g"], last["rpm"]),
                         (14.8, 100.0, 16.1, 238.28, 1360.0, 8100.0))
        self.assertEqual(sorted({r["supply_voltage_v"] for r in TABLE["rows"]}), [11.1, 14.8])
        # 4 groups of 5 throttle rows: (11.1 V: 12*4, 13*4.4) and (14.8 V: 11*3.7, 12*4) = 3 distinct propellers
        self.assertEqual(len({(r["supply_voltage_v"], r["prop"]) for r in TABLE["rows"]}), 4)
        self.assertEqual(len({r["prop"] for r in TABLE["rows"]}), 3)
        self.assertEqual(sum(r["temperature_c"] is not None for r in TABLE["rows"]), 4)

    def test_committed_file_passes_its_own_checks_and_the_pin(self):
        VF.check_motor_table(TABLE)
        ok, msg = X.check(P.DEFAULT_DB)
        self.assertTrue(ok, msg)

    def test_check_detects_a_stale_file_and_a_wrong_pin(self):
        with tempfile.TemporaryDirectory() as t:
            db = Path(t)
            (db / "validation").mkdir()
            (db / "source_pins.json").write_text((P.DEFAULT_DB / "source_pins.json").read_text(encoding="utf-8"), encoding="utf-8")
            self.assertFalse(X.check(db)[0])                                       # file missing
            doc = copy.deepcopy(TABLE)
            (db / "validation" / X.OUT_NAME).write_text(json.dumps(doc), encoding="utf-8")
            self.assertTrue(X.check(db)[0])                                        # no cache here: own checks + pin
            doc["source"]["sha256"] = "1" * 64
            (db / "validation" / X.OUT_NAME).write_text(json.dumps(doc), encoding="utf-8")
            self.assertFalse(X.check(db)[0])
            doc = copy.deepcopy(TABLE)
            doc["rows"].pop()
            (db / "validation" / X.OUT_NAME).write_text(json.dumps(doc), encoding="utf-8")
            self.assertFalse(X.check(db)[0])


class RowRuleTests(unittest.TestCase):
    def test_every_arithmetic_rule_refuses(self):
        row = TABLE["rows"][0]
        r = dict(row)
        r["power_w"], r["efficiency_g_per_w"] = 30.0, 280.0 / 30.0         # efficiency consistent with the wrong power: only V x I catches it
        with self.assertRaises(VF.ValidationFileError):
            VF.check_row(r)
        for key, value in (("power_w", 30.0), ("efficiency_g_per_w", 12.0), ("current_a", 0.0), ("rpm", -1.0), ("thrust_g", float("nan")), ("temperature_c", "42")):
            r = dict(row)
            r[key] = value
            with self.assertRaises(VF.ValidationFileError, msg=key):
                VF.check_row(r)
        extra = dict(row)
        extra["x"] = 1
        with self.assertRaises(VF.ValidationFileError):
            VF.check_row(extra)
        VF.check_row(row)

    def test_table_level_rules(self):
        for change in (lambda d: d.update(schema_version=2), lambda d: d["provenance"].update(source_type="ASSUMED"),
                       lambda d: d["source"].update(sha256="abc"), lambda d: d.update(rows=[]), lambda d: d["rows"].append(dict(d["rows"][0])),
                       lambda d: d.pop("motor_id")):
            d = copy.deepcopy(TABLE)
            change(d)
            with self.assertRaises(VF.ValidationFileError):
                VF.check_motor_table(d)

    def test_bands_rules(self):
        VF.check_bands(BANDS)
        for change in (lambda d: d.update(schema_version=0), lambda d: d["provenance"].update(source_type="CITED"),
                       lambda d: d["bands_percent"].update(unknown=1.0), lambda d: d["bands_percent"].update(uiuc_ct_percent=0.0),
                       lambda d: d["bands_percent"].update(uiuc_ct_percent=float("inf")), lambda d: d.update(bands_percent=[])):
            d = copy.deepcopy(BANDS)
            change(d)
            with self.assertRaises(VF.ValidationFileError):
                VF.check_bands(d)


if __name__ == "__main__":
    unittest.main()
