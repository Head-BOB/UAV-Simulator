# Tier: D
"""Tests for the CSV import route: templates, row -> record, all-or-nothing import, row/field messages."""
import csv
import io
import json
import unittest
from pathlib import Path

import fields as F
import import_parts as I
import partsdb as P
from testsupport import TempDb, synth

TOOLS = Path(__file__).resolve().parent


def csv_text(columns, rows):
    buf = io.StringIO()
    w = csv.DictWriter(buf, fieldnames=columns, lineterminator="\r\n")
    w.writeheader()
    for r in rows:
        w.writerow(r)
    return buf.getvalue()


def esc_row(rid="csv_esc", **over):
    """One complete ESC CSV row built from a known-good synthetic record (the hand-written reference)."""
    rec = synth("esc", rid)
    row = {"id": rid, "manufacturer": rec["manufacturer"], "model": rec["model"]}
    for q in F.quantity_fields("esc"):
        if q.name in rec:
            row[q.name] = json.dumps(rec[q.name]["value"])
            row[f"{q.name}__source_type"] = "ASSUMED"
            row[f"{q.name}__note"] = rec[q.name]["provenance"]["note"]
    row.update(over)
    return rec, row


class TemplateTests(unittest.TestCase):
    def test_template_files_are_up_to_date(self):
        for c in F.CATEGORIES:
            with self.subTest(category=c):
                p = I.template_path(TOOLS, c)
                self.assertTrue(p.exists(), "run: python tools/parts/import_parts.py --write-templates")
                self.assertEqual(p.read_bytes().decode("utf-8"), I.template_text(c))

    def test_template_has_a_column_for_every_schema_field(self):
        for c in F.CATEGORIES:
            cols = I.template_columns(c)
            schema_props = set(P.build_schema()["$defs"][c]["properties"])
            for name in schema_props - I.AUTO_STRINGS:
                self.assertIn(name, cols, f"{c}: {name}")
            for q in F.quantity_fields(c):
                for suffix in I.PROV_COLS:
                    self.assertIn(f"{q.name}__{suffix}", cols)

    def test_template_has_no_data_rows(self):
        self.assertEqual(I.template_text("motor").count("\n"), 1)


class RowToRecordTests(unittest.TestCase):
    def test_good_row_equals_hand_written_record(self):
        rec, row = esc_row()
        got, errs = I.row_to_record("esc", row, 2)
        self.assertEqual(errs, [])
        self.assertEqual(got, rec)

    def test_missing_value_message_has_row_and_field(self):
        _, row = esc_row(efficiency="")
        _, errs = I.row_to_record("esc", row, 7)
        self.assertIn("row 7, field efficiency: required value is empty", errs)

    def test_missing_source_type_message(self):
        _, row = esc_row(efficiency__source_type="")
        row["efficiency__source_type"] = ""
        _, errs = I.row_to_record("esc", row, 3)
        self.assertTrue(any(e.startswith("row 3, field efficiency: provenance source_type is missing") for e in errs))

    def test_missing_required_string(self):
        _, row = esc_row(manufacturer="")
        _, errs = I.row_to_record("esc", row, 4)
        self.assertIn("row 4, field manufacturer: required value is empty", errs)

    def test_unparseable_number(self):
        _, row = esc_row(channels="two")
        _, errs = I.row_to_record("esc", row, 5)
        self.assertTrue(any(e.startswith("row 5, field channels: cannot read") for e in errs))

    def test_optional_empty_is_skipped_but_orphan_provenance_is_an_error(self):
        _, row = esc_row()
        got, errs = I.row_to_record("esc", row, 2)
        self.assertNotIn("burst_current_a", got)
        self.assertEqual(errs, [])
        row["burst_current_a__note"] = "TODO(HUMAN-VERIFY) forgot the value"
        _, errs = I.row_to_record("esc", row, 2)
        self.assertTrue(any("provenance given but value is empty" in e for e in errs))

    def test_vector_cell_is_json(self):
        rec, row = esc_row()
        row["bounding_box_m"] = "[0.03, 0.02, 0.01]"
        got, errs = I.row_to_record("esc", row, 2)
        self.assertEqual(errs, [])
        self.assertEqual(got["bounding_box_m"]["value"], [0.03, 0.02, 0.01])


class ImportTests(unittest.TestCase):
    def setUp(self):
        self.db = TempDb()
        self.addCleanup(self.db.close)
        self.csv = self.db.repo / "in.csv"

    def run_import(self, rows, columns=None):
        self.csv.write_text(csv_text(columns or I.template_columns("esc"), rows), encoding="utf-8", newline="")
        return I.import_csv("esc", self.csv, self.db.root)

    def test_good_file_writes_records_identical_to_hand_written(self):
        rec1, row1 = esc_row("esc_one")
        rec2, row2 = esc_row("esc_two")
        paths, errs = self.run_import([row1, row2])
        self.assertEqual(errs, [])
        self.assertEqual(len(paths), 2)
        self.assertEqual(json.loads(paths[0].read_text(encoding="utf-8")), rec1)
        self.assertEqual(json.loads(paths[1].read_text(encoding="utf-8")), rec2)

    def test_one_bad_row_refuses_the_whole_file(self):
        _, good = esc_row("esc_good")
        _, bad = esc_row("esc_bad", efficiency="")
        paths, errs = self.run_import([good, bad])
        self.assertEqual(paths, [])
        self.assertIn("row 3, field efficiency: required value is empty", errs)
        self.assertEqual(list((self.db.root / "parts").rglob("*.json")), [])

    def test_row_numbers_count_the_header_as_line_1(self):
        _, bad = esc_row("esc_bad", channels="")
        _, errs = self.run_import([bad])
        self.assertTrue(any(e.startswith("row 2,") for e in errs))

    def test_schema_errors_get_row_prefix(self):
        _, bad = esc_row("esc_bad", efficiency="1.7")
        _, errs = self.run_import([bad])
        self.assertTrue(any(e.startswith("row 2: efficiency.value") for e in errs), errs)

    def test_duplicate_ids_in_file(self):
        _, a = esc_row("same")
        _, b = esc_row("same")
        _, errs = self.run_import([a, b])
        self.assertTrue(any("appears more than once" in e for e in errs))

    def test_id_already_in_database(self):
        P.write_record(synth("esc", "taken"), self.db.root)
        _, row = esc_row("taken")
        _, errs = self.run_import([row])
        self.assertTrue(any("already exists" in e for e in errs))

    def test_missing_required_column_refused(self):
        cols = [c for c in I.template_columns("esc") if c != "efficiency"]
        _, row = esc_row()
        row.pop("efficiency")
        paths, errs = self.run_import([row], columns=cols)
        self.assertEqual(paths, [])
        self.assertTrue(any("missing required column" in e and "efficiency" in e for e in errs))

    def test_unknown_category(self):
        self.csv.write_text("id\r\n", encoding="utf-8")
        _, errs = I.import_csv("toaster", self.csv, self.db.root)
        self.assertEqual(errs, ["unknown category 'toaster'"])


if __name__ == "__main__":
    unittest.main()
