# Tier: D
"""Tests for partsdb: schema, provenance rules, semantic rules, loading, index, search, compatibility.
Expected outcomes are stated by hand in each test (what rule must fire and why)."""
import hashlib
import json
import sqlite3
import unittest
from pathlib import Path

import fields as F
import partsdb as P
from testsupport import TempDb, deepcopy, quantity, synth

REAL = P.DEFAULT_DB
NUMERIC_CATEGORIES = sorted(F.CATEGORIES)


def errors_of(rec, db=None):
    d = db or TempDb()
    try:
        return P.validate_record(rec, d.root)[0]
    finally:
        if db is None:
            d.close()


class SchemaFileTests(unittest.TestCase):
    def test_schema_file_matches_generator(self):
        text = (REAL / "schema" / "part.schema.json").read_text(encoding="utf-8")
        self.assertEqual(text, P.canonical_json(P.build_schema()),
                         "run: python tools/parts/parts_cli.py schema")

    def test_schema_is_valid_json_schema(self):
        import jsonschema
        jsonschema.Draft202012Validator.check_schema(P.build_schema())

    def test_every_category_has_a_def_and_a_dispatch(self):
        s = P.build_schema()
        for c in F.CATEGORIES:
            self.assertIn(c, s["$defs"])
        self.assertEqual(len(s["allOf"]), len(F.CATEGORIES))


class ValidRecordTests(unittest.TestCase):
    def test_synthetic_valid_record_passes_in_every_category(self):
        # Hand check: each synthetic record satisfies every required field and the cross-field rules.
        for c in NUMERIC_CATEGORIES:
            with self.subTest(category=c):
                self.assertEqual(errors_of(synth(c)), [])


class ProvenanceRuleTests(unittest.TestCase):
    def test_bare_number_fails_for_every_required_numeric_field(self):
        for c in NUMERIC_CATEGORIES:
            for q in F.quantity_fields(c):
                if not q.required:
                    continue
                with self.subTest(category=c, field=q.name):
                    rec = synth(c)
                    rec[q.name] = rec[q.name]["value"]
                    errs = errors_of(rec)
                    self.assertTrue(any(e.startswith(f"{q.name}:") for e in errs), errs)

    def test_missing_provenance_names_the_field_path(self):
        rec = synth("motor")
        del rec["kv_rpm_per_v"]["provenance"]
        errs = errors_of(rec)
        self.assertTrue(any(e.startswith("kv_rpm_per_v:") and "provenance" in e for e in errs), errs)

    def test_missing_required_field_is_reported(self):
        rec = synth("motor")
        del rec["winding_resistance_ohm"]
        self.assertTrue(any("winding_resistance_ohm" in e for e in errors_of(rec)))

    def test_cited_without_url_fails(self):
        rec = synth("esc")
        rec["efficiency"]["provenance"] = {"source_type": "CITED", "retrieved_on": "2026-10-10"}
        self.assertTrue(any("source_url" in e for e in errors_of(rec)))

    def test_cited_without_date_fails(self):
        rec = synth("esc")
        rec["efficiency"]["provenance"] = {"source_type": "CITED", "source_url": "https://example.com/a"}
        self.assertTrue(any("retrieved_on" in e for e in errors_of(rec)))

    def test_assumed_without_todo_fails(self):
        rec = synth("esc")
        rec["efficiency"]["provenance"] = {"source_type": "ASSUMED", "note": "looks about right"}
        self.assertTrue(any("does not match" in e for e in errors_of(rec)))

    def test_assumed_without_note_fails(self):
        rec = synth("esc")
        rec["efficiency"]["provenance"] = {"source_type": "ASSUMED"}
        self.assertTrue(any("note" in e for e in errors_of(rec)))

    def test_calculated_needs_a_formula_note(self):
        rec = synth("esc")
        rec["efficiency"]["provenance"] = {"source_type": "CALCULATED", "note": "x"}
        self.assertTrue(errors_of(rec))
        rec["efficiency"]["provenance"] = {"source_type": "CALCULATED", "note": "eta = P_out / P_in with test inputs"}
        self.assertEqual(errors_of(rec), [])

    def test_measured_needs_note_and_date(self):
        rec = synth("esc")
        rec["efficiency"]["provenance"] = {"source_type": "MEASURED", "note": "bench test by hand, 2026-10-10 run 3"}
        self.assertTrue(any("retrieved_on" in e for e in errors_of(rec)))
        rec["efficiency"]["provenance"]["retrieved_on"] = "2026-10-10"
        self.assertEqual(errors_of(rec), [])

    def test_unknown_source_type_fails(self):
        rec = synth("esc")
        rec["efficiency"]["provenance"] = {"source_type": "GUESSED", "note": "TODO(HUMAN-VERIFY)"}
        self.assertTrue(errors_of(rec))

    def test_bad_sha_format_fails(self):
        rec = synth("esc")
        rec["efficiency"]["provenance"] = {"source_type": "CITED", "source_url": "https://example.com/a",
                                           "retrieved_on": "2026-10-10", "sha256_of_file": "abc"}
        self.assertTrue(any("sha256_of_file" in e for e in errors_of(rec)))

    def test_sha_without_url_fails(self):
        rec = synth("esc")
        rec["efficiency"]["provenance"] = {"source_type": "ASSUMED", "note": "TODO(HUMAN-VERIFY) x", "sha256_of_file": "0" * 64}
        self.assertTrue(any("without source_url" in e for e in errors_of(rec)))

    def test_non_http_url_fails(self):
        rec = synth("esc")
        rec["efficiency"]["provenance"] = {"source_type": "CITED", "source_url": "ftp://example.com/a",
                                           "retrieved_on": "2026-10-10"}
        self.assertTrue(any("source_url" in e for e in errors_of(rec)))

    def test_impossible_calendar_date_fails(self):
        # 2026-02-30 matches the pattern but is not a date.
        rec = synth("esc")
        rec["efficiency"]["provenance"] = {"source_type": "CITED", "source_url": "https://example.com/a",
                                           "retrieved_on": "2026-02-30"}
        self.assertTrue(any("calendar date" in e for e in errors_of(rec)))

    def test_unknown_field_is_rejected(self):
        rec = synth("esc")
        rec["secret_gain"] = 3
        self.assertTrue(any("secret_gain" in e for e in errors_of(rec)))

    def test_wrong_value_range_fails(self):
        rec = synth("esc")
        rec["efficiency"]["value"] = 1.2  # efficiency must be in (0, 1]
        self.assertTrue(any(e.startswith("efficiency.value") for e in errors_of(rec)))
        rec["efficiency"]["value"] = 0.0
        self.assertTrue(errors_of(rec))

    def test_integer_field_rejects_fraction(self):
        rec = synth("battery")
        rec["cells"]["value"] = 3.5
        self.assertTrue(any(e.startswith("cells.value") for e in errors_of(rec)))

    def test_vec3_needs_three_positive_numbers(self):
        rec = synth("motor")
        rec["bounding_box_m"]["value"] = [0.1, 0.1]
        self.assertTrue(errors_of(rec))
        rec["bounding_box_m"]["value"] = [0.1, 0.0, 0.1]
        self.assertTrue(errors_of(rec))

    def test_wrong_category_value_fails(self):
        rec = synth("motor")
        rec["category"] = "toaster"
        self.assertTrue(any("category" in e for e in errors_of(rec)))

    def test_pinned_url_requires_matching_sha(self):
        db = TempDb()
        self.addCleanup(db.close)
        url = "https://example.com/ds.pdf"
        sha = hashlib.sha256(b"datasheet").hexdigest()
        db.pins({url: {"sha256": sha, "retrieved_on": "2026-10-10", "bytes": 9, "http_status": 200, "note": ""}})
        rec = synth("esc")
        prov = {"source_type": "CITED", "source_url": url, "retrieved_on": "2026-10-10"}
        rec["efficiency"]["provenance"] = dict(prov)
        self.assertTrue(any("sha256_of_file is required" in e for e in errors_of(rec, db)))
        rec["efficiency"]["provenance"]["sha256_of_file"] = "0" * 64
        self.assertTrue(any("does not equal the pin" in e for e in errors_of(rec, db)))
        rec["efficiency"]["provenance"]["sha256_of_file"] = sha
        self.assertEqual(errors_of(rec, db), [])

    def test_unpinned_cited_url_is_a_warning_not_an_error(self):
        db = TempDb()
        self.addCleanup(db.close)
        rec = synth("esc")
        rec["efficiency"]["provenance"] = {"source_type": "CITED", "source_url": "https://example.com/x",
                                           "retrieved_on": "2026-10-10"}
        errs, warns = P.validate_record(rec, db.root)
        self.assertEqual(errs, [])
        self.assertEqual(len(warns), 1)
        self.assertIn("no fetched pin", warns[0])


class SemanticRuleTests(unittest.TestCase):
    def test_cells_min_above_max(self):
        self.assertTrue(any("cells_min" in e for e in errors_of(synth("motor", cells_min=5, cells_max=4))))
        self.assertTrue(any("cells_min" in e for e in errors_of(synth("esc", cells_min=7, cells_max=6))))

    def test_prop_range_inverted(self):
        self.assertTrue(any("prop_diameter_min_m" in e
                            for e in errors_of(synth("motor", prop_diameter_min_m=0.5, prop_diameter_max_m=0.4))))

    def test_esc_burst_below_continuous(self):
        errs = errors_of(synth("esc", burst_current_a=10.0, max_continuous_current_a=30.0))
        self.assertTrue(any("burst_current_a" in e for e in errs))

    def test_mass_without_cables_above_mass(self):
        self.assertTrue(any("mass_without_cables_kg" in e
                            for e in errors_of(synth("motor", mass_kg=0.1, mass_without_cables_kg=0.2))))

    def test_frame_tube_id_not_below_od(self):
        errs = errors_of(synth("frame", tube_od_m=0.016, tube_id_m=0.016))
        self.assertTrue(any("tube_id_m" in e and "smaller" in e for e in errs))

    def test_frame_needs_an_arm_section(self):
        errs = errors_of(synth("frame", tube_od_m=None, tube_id_m=None))
        self.assertTrue(any("arm section" in e for e in errs))

    def test_frame_plate_section_is_accepted(self):
        rec = synth("frame", tube_od_m=None, tube_id_m=None, arm_plate_width_m=0.02, arm_plate_thickness_m=0.004)
        self.assertEqual(errors_of(rec), [])

    def test_frame_half_section_rejected(self):
        self.assertTrue(any("together" in e for e in errors_of(synth("frame", tube_id_m=None))))

    def test_battery_voltage_order(self):
        # cutoff 3.3 < nominal 3.7 < max 4.2 is the only legal order
        errs = errors_of(synth("battery", voltage_per_cell_nominal_v=3.2))
        self.assertTrue(any("cutoff" in e and "nominal" in e for e in errs))

    def test_battery_voltage_order_is_strict(self):
        # equal cutoff and nominal (3.3 / 3.3) is not a legal order either
        errs = errors_of(synth("battery", voltage_per_cell_nominal_v=3.3))
        self.assertTrue(any("cutoff" in e and "nominal" in e for e in errs))
        errs = errors_of(synth("battery", voltage_per_cell_nominal_v=4.2))  # nominal == max
        self.assertTrue(any("cutoff" in e and "nominal" in e for e in errs))

    def test_ocv_curve_must_start_and_end_at_soc_limits(self):
        errs = errors_of(synth("battery", ocv_curve=[[0.1, 3.3], [1.0, 4.2]]))
        self.assertTrue(any("start at soc 0" in e for e in errs))
        errs = errors_of(synth("battery", ocv_curve=[[0.0, 3.3], [0.9, 4.2]]))
        self.assertTrue(any("end at soc 1" in e for e in errs))

    def test_ocv_curve_soc_strictly_increasing(self):
        errs = errors_of(synth("battery", ocv_curve=[[0.0, 3.3], [0.5, 3.8], [0.5, 3.9], [1.0, 4.2]]))
        self.assertTrue(any("strictly increasing" in e for e in errs))

    def test_ocv_curve_voltage_non_decreasing(self):
        errs = errors_of(synth("battery", ocv_curve=[[0.0, 3.3], [0.5, 3.9], [1.0, 3.8]]))
        self.assertTrue(any("non-decreasing" in e for e in errs))

    def test_ocv_curve_above_max_voltage(self):
        errs = errors_of(synth("battery", ocv_curve=[[0.0, 3.3], [1.0, 4.3]]))
        self.assertTrue(any("voltage_per_cell_max_v" in e for e in errs))


class PropDataTests(unittest.TestCase):
    def _db_with_file(self, content=b'{"x": 1}'):
        db = TempDb()
        self.addCleanup(db.close)
        f = db.repo / "data" / "p.json"
        f.parent.mkdir()
        f.write_bytes(content)
        return db, hashlib.sha256(content).hexdigest()

    def _uiuc(self, sha):
        rec = synth("propeller", data_basis="UIUC_MEASURED", data_file="data/p.json", data_file_sha256=sha,
                    scaled_from=None, ct_cp_scaling_assumed=None)
        return rec

    def test_matching_file_hash_passes(self):
        db, sha = self._db_with_file()
        self.assertEqual(errors_of(self._uiuc(sha), db), [])

    def test_wrong_hash_fails(self):
        db, _ = self._db_with_file()
        errs = errors_of(self._uiuc("0" * 64), db)
        self.assertTrue(any("does not match" in e for e in errs), errs)

    def test_missing_file_fails(self):
        db, sha = self._db_with_file()
        rec = self._uiuc(sha)
        rec["data_file"] = "data/nope.json"
        self.assertTrue(any("does not exist" in e for e in errors_of(rec, db)))

    def test_uiuc_without_file_fails(self):
        rec = synth("propeller", data_basis="UIUC_MEASURED", scaled_from=None, ct_cp_scaling_assumed=None)
        self.assertTrue(any("requires data_file" in e for e in errors_of(rec)))

    def test_scaled_needs_scaled_from(self):
        self.assertTrue(any("scaled_from" in e for e in errors_of(synth("propeller", scaled_from=None))))

    def test_scaled_needs_assumed_scaling_quantity(self):
        rec = synth("propeller")
        rec["ct_cp_scaling_assumed"]["provenance"] = {"source_type": "CALCULATED", "note": "ratio of diameters, test"}
        self.assertTrue(any("ASSUMED" in e for e in errors_of(rec)))
        self.assertTrue(any("ASSUMED" in e for e in errors_of(synth("propeller", ct_cp_scaling_assumed=None))))

    def test_scaled_from_must_exist_and_be_measured(self):
        db = TempDb()
        self.addCleanup(db.close)
        db.add(synth("propeller", "p_scaled", scaled_from="p_ghost"))
        errs = P.validate_db(db.root)[0]
        self.assertIn("p_scaled", errs)
        self.assertTrue(any("scaled_from" in e for e in errs["p_scaled"]))

    def test_measured_rpm_range_inverted(self):
        errs = errors_of(synth("propeller", measured_rpm_min=5000.0, measured_rpm_max=4000.0))
        self.assertTrue(any("measured_rpm_min" in e for e in errs))


class LoadingTests(unittest.TestCase):
    def setUp(self):
        self.db = TempDb()
        self.addCleanup(self.db.close)

    def test_duplicate_id_in_two_folders(self):
        self.db.add(synth("esc", "same_id"))
        other = self.db.root / "parts" / "motor"
        other.mkdir()
        # category differs from the folder, which is also an error; both ids must not load twice
        (other / "same_id.json").write_text(json.dumps(synth("esc", "same_id")), encoding="utf-8")
        recs, errs = P.load_records(self.db.root)
        self.assertEqual(list(recs), ["same_id"])
        self.assertTrue(any("not its folder" in e for e in errs))

    def test_id_must_equal_file_name(self):
        p = self.db.add(synth("esc", "right_name"))
        p.rename(p.with_name("wrong_name.json"))
        _, errs = P.load_records(self.db.root)
        self.assertTrue(any("does not equal the file name" in e for e in errs))

    def test_unknown_category_folder(self):
        (self.db.root / "parts" / "toasters").mkdir()
        _, errs = P.load_records(self.db.root)
        self.assertTrue(any("unknown category folder" in e for e in errs))

    def test_unreadable_json(self):
        d = self.db.root / "parts" / "esc"
        d.mkdir()
        (d / "bad.json").write_text("{not json", encoding="utf-8")
        _, errs = P.load_records(self.db.root)
        self.assertTrue(any("unreadable JSON" in e for e in errs))

    def test_validate_db_reports_per_record(self):
        good = synth("esc", "good_esc")
        bad = synth("esc", "bad_esc")
        bad["efficiency"]["value"] = 7
        self.db.add(good)
        self.db.add(bad)
        errs, _ = P.validate_db(self.db.root)
        self.assertEqual(sorted(errs), ["bad_esc"])


class WriteRecordTests(unittest.TestCase):
    def setUp(self):
        self.db = TempDb()
        self.addCleanup(self.db.close)

    def test_writes_canonical_file(self):
        rec = synth("esc", "new_esc")
        path = P.write_record(rec, self.db.root)
        self.assertEqual(path.read_text(encoding="utf-8"), P.canonical_json(rec))
        self.assertTrue(path.read_text(encoding="utf-8").endswith("\n"))

    def test_written_keys_are_sorted(self):
        rec = synth("esc", "new_esc")
        path = P.write_record(rec, self.db.root)
        top = [line.split('"')[1] for line in path.read_text(encoding="utf-8").splitlines() if line.startswith('  "')]
        self.assertEqual(top, sorted(top))
        self.assertEqual(top[0], "availability_url" if "availability_url" in rec else sorted(rec)[0])

    def test_refuses_incomplete_record(self):
        rec = synth("esc", "new_esc")
        del rec["efficiency"]["provenance"]
        with self.assertRaises(ValueError):
            P.write_record(rec, self.db.root)
        self.assertFalse((self.db.root / "parts" / "esc" / "new_esc.json").exists())

    def test_refuses_duplicate_id(self):
        P.write_record(synth("esc", "new_esc"), self.db.root)
        with self.assertRaises(ValueError):
            P.write_record(synth("esc", "new_esc"), self.db.root)

    def test_refuses_path_like_id(self):
        with self.assertRaises(ValueError):
            P.write_record(synth("esc", "../evil"), self.db.root)
        self.assertEqual(list(self.db.root.rglob("evil*")), [])


class IndexSearchTests(unittest.TestCase):
    """Hand-listed catalogue: masses and prices chosen so each filter has an exact answer."""

    def setUp(self):
        self.db = TempDb()
        self.addCleanup(self.db.close)
        spec = [("motor", "m_light", 0.05, 10.0), ("motor", "m_heavy", 0.15, 30.0),
                ("esc", "e_light", 0.01, 5.0), ("esc", "e_nopr", 0.02, None),
                ("battery", "b_big", 0.5, 60.0)]
        for cat, rid, mass, price in spec:
            rec = synth(cat, rid, mass_kg=mass)
            rec["manufacturer"] = "Acme" if rid.startswith("m_") else "Zed"
            rec["model"] = "Alpha-" + rid
            if price is not None:
                rec["price_usd"] = quantity(price)
            self.db.add(rec)

    def ids(self, **kw):
        return P.search(self.db.root, **kw)

    def test_no_filter_returns_everything_sorted(self):
        self.assertEqual(self.ids(), ["b_big", "e_light", "e_nopr", "m_heavy", "m_light"])

    def test_category_filter(self):
        self.assertEqual(self.ids(category="motor"), ["m_heavy", "m_light"])

    def test_mass_filter_is_inclusive(self):
        self.assertEqual(self.ids(mass_max_kg=0.05), ["e_light", "e_nopr", "m_light"])

    def test_price_filter_excludes_unpriced_parts(self):
        self.assertEqual(self.ids(price_max_usd=30.0), ["e_light", "m_heavy", "m_light"])

    def test_text_filter_all_words_must_match_case_insensitive(self):
        self.assertEqual(self.ids(text="acme alpha-m_h"), ["m_heavy"])
        self.assertEqual(self.ids(text="ZED"), ["b_big", "e_light", "e_nopr"])
        self.assertEqual(self.ids(text="acme zed"), [])

    def test_filters_combine(self):
        self.assertEqual(self.ids(category="motor", mass_max_kg=0.1, text="acme"), ["m_light"])

    def test_index_rebuilds_when_a_record_changes(self):
        self.assertEqual(self.ids(category="esc"), ["e_light", "e_nopr"])
        self.db.add(synth("esc", "e_new", mass_kg=0.03))
        self.assertEqual(self.ids(category="esc"), ["e_light", "e_new", "e_nopr"])

    def test_index_refuses_invalid_database(self):
        bad = synth("esc", "e_bad")
        bad["efficiency"]["value"] = 7
        self.db.add(bad)
        with self.assertRaises(ValueError):
            P.build_index(self.db.root)

    def test_index_content(self):
        path = P.build_index(self.db.root)
        con = sqlite3.connect(path)
        self.addCleanup(con.close)
        row = con.execute("SELECT mass_kg, price_usd FROM parts WHERE id='m_heavy'").fetchone()
        self.assertEqual(row, (0.15, 30.0))


def build_rec(cat, rid, **over):
    return synth(cat, rid, **over)


class CompatibilityTests(unittest.TestCase):
    def setUp(self):
        self.db = TempDb()
        self.addCleanup(self.db.close)
        add = self.db.add
        self.db.add_ref_prop()
        # build: a 4S pack, a 3-4S motor, a 30 A ESC, 0.3 m prop, 0.4 m frame, mount "A"
        add(build_rec("battery", "bat4s", cells=4))
        add(build_rec("battery", "bat6s", cells=6))
        add(build_rec("battery", "bat2s", cells=2, voltage_per_cell_nominal_v=3.7))
        add(build_rec("motor", "mot_a", cells_min=3, cells_max=4, mount_pattern="A", max_current_a=20.0,
                      prop_diameter_min_m=0.25, prop_diameter_max_m=0.4, shaft_diameter_m=0.004))
        add(build_rec("motor", "mot_b", cells_min=5, cells_max=6, mount_pattern="A", max_current_a=20.0))
        add(build_rec("motor", "mot_mount", cells_min=3, cells_max=4, mount_pattern="B"))
        add(build_rec("motor", "mot_hot", cells_min=3, cells_max=4, mount_pattern="A", max_current_a=40.0))
        add(build_rec("esc", "esc30", max_continuous_current_a=30.0, cells_min=3, cells_max=6))
        add(build_rec("esc", "esc10", max_continuous_current_a=10.0, cells_min=3, cells_max=6))
        add(build_rec("propeller", "prop30", diameter_m=0.30, bore_m=0.006))
        add(build_rec("propeller", "prop45", diameter_m=0.45, bore_m=0.006))
        add(build_rec("propeller", "prop_smallbore", diameter_m=0.30, bore_m=0.003))
        add(build_rec("frame", "frame_a", motor_mount_pattern="A", max_prop_diameter_m=0.4))
        self.recs, _ = P.load_records(self.db.root)

    def build(self, **kw):
        return kw

    def test_motor_voltage_range_vs_battery(self):
        # 4S pack: mot_a (3-4S) ok, mot_b (5-6S) not
        got = P.search(self.db.root, category="motor", compatible_with={"battery": "bat4s"})
        self.assertEqual(got, ["mot_a", "mot_hot", "mot_mount"])

    def test_battery_vs_motor_and_esc(self):
        # motor 3-4S and ESC 3-6S: only the 4S pack fits both; 6S is above the motor, 2S below both
        got = P.search(self.db.root, category="battery", compatible_with={"motor": "mot_a", "esc": "esc30"})
        self.assertEqual(got, ["bat4s"])

    def test_esc_cell_range_vs_battery(self):
        # esc30 accepts 3-6S: with only the ESC in the build, 2S is out, 3S/4S/6S are in
        self.db.add(build_rec("battery", "bat3s", cells=3))
        got = P.search(self.db.root, category="battery", compatible_with={"esc": "esc30"})
        self.assertEqual(got, ["bat3s", "bat4s", "bat6s"])

    def test_esc_current_must_cover_motor_max(self):
        got = P.search(self.db.root, category="esc", compatible_with={"motor": "mot_a"})
        self.assertEqual(got, ["esc30"])  # 10 A < 20 A fails; 30 A >= 20 A passes
        got = P.search(self.db.root, category="esc", compatible_with={"motor": "mot_hot"})
        self.assertEqual(got, [])  # 40 A motor: neither ESC covers it

    def test_esc_current_equal_is_allowed(self):
        self.db.add(build_rec("esc", "esc20", max_continuous_current_a=20.0, cells_min=3, cells_max=6))
        got = P.search(self.db.root, category="esc", compatible_with={"motor": "mot_a"})
        self.assertEqual(got, ["esc20", "esc30"])

    def test_prop_diameter_range_and_frame_limit(self):
        # motor range 0.25-0.4 m: 0.45 m is out; 0.30 ok. bore 0.003 < shaft 0.004 fails
        got = P.search(self.db.root, category="propeller", compatible_with={"motor": "mot_a"})
        # (test_ref_prop is the 0.30 m / 6 mm bore fixture prop, so it passes like prop30)
        self.assertEqual(got, ["prop30", "test_ref_prop"])
        # without the motor the frame limit (0.4 m) alone excludes the 0.45 m prop
        got = P.search(self.db.root, category="propeller", compatible_with={"frame": "frame_a"})
        self.assertEqual(got, ["prop30", "prop_smallbore", "test_ref_prop"])

    def test_cell_range_boundaries_are_inclusive(self):
        # mot_a accepts 3-4S: a 3S pack (lower bound) and a 4S pack (upper bound) both fit, 2S and 5S do not
        self.db.add(build_rec("battery", "bat3s", cells=3))
        self.db.add(build_rec("battery", "bat5s", cells=5))
        got = P.search(self.db.root, category="battery", compatible_with={"motor": "mot_a"})
        self.assertEqual(got, ["bat3s", "bat4s"])

    def test_prop_equal_to_frame_limit_is_allowed(self):
        self.db.add(build_rec("propeller", "prop40", diameter_m=0.40, bore_m=0.006))
        got = P.search(self.db.root, category="propeller", compatible_with={"frame": "frame_a"})
        self.assertIn("prop40", got)  # 0.40 m == frame max 0.40 m
        self.db.add(build_rec("propeller", "prop41", diameter_m=0.41, bore_m=0.006))
        got = P.search(self.db.root, category="propeller", compatible_with={"frame": "frame_a"})
        self.assertNotIn("prop41", got)

    def test_motor_mount_vs_frame(self):
        got = P.search(self.db.root, category="motor", compatible_with={"frame": "frame_a"})
        self.assertEqual(got, ["mot_a", "mot_b", "mot_hot"])

    def test_frame_vs_prop_and_motor(self):
        self.db.add(build_rec("frame", "frame_small", motor_mount_pattern="A", max_prop_diameter_m=0.2))
        self.db.add(build_rec("frame", "frame_b", motor_mount_pattern="B", max_prop_diameter_m=0.5))
        got = P.search(self.db.root, category="frame", compatible_with={"propeller": "prop30", "motor": "mot_a"})
        self.assertEqual(got, ["frame_a"])

    def test_same_category_is_skipped(self):
        got = P.search(self.db.root, category="motor", compatible_with={"motor": "mot_a"})
        self.assertEqual(got, ["mot_a", "mot_b", "mot_hot", "mot_mount"])

    def test_incompatibilities_messages_name_both_parts(self):
        msgs = P.incompatibilities(self.recs["mot_b"], self.recs["bat4s"])
        self.assertEqual(len(msgs), 1)
        self.assertIn("bat4s", msgs[0])
        self.assertIn("mot_b", msgs[0])

    def test_unrelated_categories_have_no_rule(self):
        self.assertEqual(P.incompatibilities(self.recs["bat4s"], self.recs["prop30"]), [])


class SeedCatalogueTests(unittest.TestCase):
    def test_all_seed_records_validate(self):
        errs, _ = P.validate_db(REAL)
        self.assertEqual(errs, {})

    def test_seed_records_are_canonical_text(self):
        for p in sorted((REAL / "parts").rglob("*.json")):
            with self.subTest(p=p.name):
                text = p.read_text(encoding="utf-8")
                self.assertEqual(text, P.canonical_json(json.loads(text)))
                self.assertNotIn("\r", text)

    def test_reference_build_matches_catalog_json(self):
        # Hand mapping catalog.json -> record field; values must be identical (migration, no re-typing).
        cat = json.loads((P.REPO_ROOT / "offline_pipeline/components/catalog.json").read_text())
        recs, _ = P.load_records(REAL)
        pairs = [
            ("tmotor_mn3508_kv700", "kv_rpm_per_v", cat["motor"]["kv_rpm_per_v"]["value"]),
            ("tmotor_mn3508_kv700", "winding_resistance_ohm", cat["motor"]["resistance_ohm"]["value"]),
            ("tmotor_mn3508_kv700", "no_load_current_a", cat["motor"]["no_load_current_a"]["value"]),
            ("tmotor_mn3508_kv700", "mass_kg", cat["motor"]["mass_kg"]["value"]),
            ("tmotor_mn3508_kv700", "max_current_a", cat["motor"]["max_continuous_current_a"]["value"]),
            ("tmotor_mn3508_kv700", "max_power_w", cat["motor"]["max_power_w"]["value"]),
            ("tmotor_mn3508_kv700", "cells_min", cat["motor"]["cells_min_max"]["value"][0]),
            ("tmotor_mn3508_kv700", "cells_max", cat["motor"]["cells_min_max"]["value"][1]),
            ("tmotor_mn3508_kv700", "winding_temp_limit_c", cat["motor"]["temp_limit_c"]["value"]),
            ("ref_esc_generic_20a", "max_continuous_current_a", cat["esc"]["rated_current_a"]["value"]),
            ("ref_esc_generic_20a", "mass_kg", cat["esc"]["mass_kg"]["value"]),
            ("ref_esc_generic_20a", "efficiency", cat["esc"]["efficiency"]["value"]),
            ("ref_battery_generic_4s_5000", "cells", cat["battery"]["cells"]["value"]),
            ("ref_battery_generic_4s_5000", "capacity_ah", cat["battery"]["capacity_ah"]["value"]),
            ("ref_battery_generic_4s_5000", "mass_kg", cat["battery"]["mass_kg"]["value"]),
            ("ref_battery_generic_4s_5000", "internal_resistance_ohm", cat["battery"]["internal_resistance_ohm"]["value"]),
            ("ref_battery_generic_4s_5000", "max_continuous_c", cat["battery"]["max_discharge_c"]["value"]),
            ("ref_battery_generic_4s_5000", "usable_fraction", cat["battery"]["usable_fraction"]["value"]),
            ("apc_12x6e", "diameter_m", cat["prop"]["diameter_m"]["value"]),
            ("apc_12x6e", "mass_kg", cat["prop"]["mass_kg"]["value"]),
            ("ref_frame_quad_400", "mass_kg", cat["frame"]["frame_mass_kg"]["value"]),
            ("ref_frame_quad_400", "motor_spacing_adjacent_m", cat["frame"]["motor_spacing_m"]["value"]),
            ("ref_frame_quad_400", "tube_od_m", cat["frame"]["arm_tube_outer_diameter_m"]["value"]),
            ("ref_frame_quad_400", "body_half_width_m", cat["frame"]["body_half_width_m"]["value"]),
            ("ref_frame_quad_400", "central_body_box_m", cat["frame"]["central_body_box_m"]["value"]),
            ("ref_frame_quad_400", "arm_allowable_bending_stress_pa", cat["frame"]["arm_allowable_bending_stress_pa"]["value"]),
            ("ref_avionics_wiring_lump", "mass_kg", cat["frame"]["avionics_payload_mass_kg"]["value"]),
        ]
        for rid, field, expected in pairs:
            with self.subTest(part=rid, field=field):
                self.assertEqual(recs[rid][field]["value"], expected)
        # nominal voltage per cell = 14.8 V / 4 cells = 3.7 V
        self.assertAlmostEqual(recs["ref_battery_generic_4s_5000"]["voltage_per_cell_nominal_v"]["value"], 14.8 / 4, places=12)
        # tube ID = 0.016 - 2 * 0.0015 = 0.013
        self.assertAlmostEqual(recs["ref_frame_quad_400"]["tube_id_m"]["value"], 0.013, places=9)
        # arm length = 0.4 * sqrt(2) / 2 = 0.282843 (6 d.p. stored)
        self.assertAlmostEqual(recs["ref_frame_quad_400"]["arm_length_m"]["value"], 0.282843, places=6)

    def test_prop_record_ties_to_the_uiuc_file(self):
        recs, _ = P.load_records(REAL)
        p = recs["apc_12x6e"]
        data = (P.REPO_ROOT / p["data_file"]).read_bytes()
        self.assertEqual(p["data_file_sha256"], hashlib.sha256(data).hexdigest())

    def test_assumed_values_always_carry_a_todo(self):
        recs, _ = P.load_records(REAL)
        n = 0
        for rid, rec in recs.items():
            for q in F.quantity_fields(rec["category"]):
                prov = rec.get(q.name, {}).get("provenance")
                if prov and prov["source_type"] == "ASSUMED":
                    n += 1
                    self.assertIn("TODO(HUMAN-VERIFY", prov["note"], f"{rid}.{q.name}")
        self.assertGreater(n, 0)


if __name__ == "__main__":
    unittest.main()
