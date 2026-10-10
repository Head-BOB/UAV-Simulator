# Tier: F
import json
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import gen_demo_headers as g  # noqa: E402


class GeneratorTests(unittest.TestCase):
    def test_checked_in_headers_are_current(self):
        for path, text in g.render_all().items():
            self.assertTrue(path.exists(), path)
            self.assertEqual(path.read_text(), text, f"{path.name} is stale: run python tools/gen_demo_headers.py")

    def test_camel_names(self):
        self.assertEqual(g.camel("kv_rpm_per_v"), "KvRpmPerV")
        self.assertEqual(g.camel("motor"), "Motor")
        self.assertEqual(g.camel("max_continuous_current_a"), "MaxContinuousCurrentA")

    def test_escaping(self):
        self.assertEqual(g.cpp_str('a"b\\c'), '"a\\"b\\\\c"')

    def test_reference_header_content(self):
        cat = json.loads(g.CATALOG.read_text())
        prop = json.loads(g.PROP.read_text())
        text = g.reference_header(cat, prop)
        # Spot values straight from the cited T-Motor page numbers recorded in the catalog.
        self.assertIn("constexpr double kMotorKvRpmPerV = 700.0;  // CITED", text)
        self.assertIn("constexpr double kMotorResistanceOhm = 0.072;  // CITED", text)
        self.assertIn("constexpr double kBatteryCapacityAh = 5.0;  // ASSUMED", text)
        # The 8100 rpm proxy is cited for another prop: it must be labelled ASSUMED here.
        self.assertIn("constexpr double kMotorLoadedMaxRpmProxy = 8100.0;  // ASSUMED", text)
        self.assertIn(f"constexpr double kPropCt = {prop['static_fit']['ct']!r};", text)
        # Array items appear in kItems but not as scalar constants.
        self.assertIn('"frame.central_body_box_m"', text)
        self.assertNotIn("kFrameCentralBodyBoxM =", text)

    def test_every_scalar_item_has_a_constant_and_a_status(self):
        cat = json.loads(g.CATALOG.read_text())
        text = g.reference_header(cat, json.loads(g.PROP.read_text()))
        for group, entries in cat.items():
            if group.startswith("_"):
                continue
            for key, item in entries.items():
                if isinstance(item, dict) and "value" in item:
                    self.assertIn(f'"{group}.{key}"', text)
                    self.assertIn(f'"{item["status"]}"', text)

    def test_axial_table_is_the_run_closest_to_4000_rpm_with_increasing_j(self):
        prop = json.loads(g.PROP.read_text())
        rpm, pts = g.axial_run(prop)
        self.assertEqual(rpm, 4030.0)  # |4030 - 4000| = 30 beats 3040, 5007 and the rest
        self.assertEqual(len(pts), 17)  # the run has 17 rows and strictly increasing J
        self.assertEqual(pts, sorted(pts))
        self.assertTrue(all(b[0] > a[0] for a, b in zip(pts, pts[1:])))
        text = g.reference_header(json.loads(g.CATALOG.read_text()), prop)
        self.assertIn("constexpr double kPropAxialRunRpm = 4030.0;", text)
        self.assertIn("constexpr std::size_t kPropAxialCount = 17;", text)
        self.assertIn("kPropAxialJ[] = {0.111927, 0.141046,", text)

    def test_axial_run_drops_repeated_rows(self):
        prop = {"wind_tunnel": {"a_1000.txt": [{"j": 0.1, "ct": 0.09}, {"j": 0.2, "ct": 0.08}, {"j": 0.2, "ct": 0.08}, {"j": 0.15, "ct": 0.07}]}}
        self.assertEqual(g.axial_run(prop), (1000.0, [(0.1, 0.09), (0.2, 0.08)]))
        with self.assertRaises(ValueError):
            g.axial_run({"wind_tunnel": {"no_rpm_in_name.txt": []}})

    def test_environments_header_has_five_rows_in_order(self):
        envs = json.loads(g.ENVS.read_text())
        text = g.environments_header(envs)
        ids = [e["id"] for e in envs["environments"]]
        positions = [text.index(f'{{"{i}"') for i in ids]
        self.assertEqual(positions, sorted(positions))
        self.assertEqual(text.count('{"'), 5)
        self.assertIn('{"Mountains", "Mountains", 2500.0, 2.0, 45.0, 8.0, 45.0, 2.5}', text)

    def test_check_mode_detects_a_stale_file(self):
        path = next(iter(g.render_all()))
        original = path.read_text()
        try:
            path.write_text(original + "// stale\n")
            self.assertEqual(g.main(["--check"]), 1)
        finally:
            path.write_text(original)
        self.assertEqual(g.main(["--check"]), 0)


if __name__ == "__main__":
    unittest.main()
