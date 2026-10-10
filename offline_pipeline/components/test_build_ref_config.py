# Tier: D
import json
import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import build_ref_config as b  # noqa: E402

CAT = json.loads(b.CATALOG.read_text())


class CatalogTests(unittest.TestCase):
    def test_every_field_is_cited_or_assumed(self):
        # The catalog contract: each leaf with a "value" has status CITED or ASSUMED and a source.
        for group, items in CAT.items():
            if group.startswith("_"):
                continue
            for key, item in items.items():
                if isinstance(item, dict) and "value" in item:
                    self.assertIn(item["status"], ("CITED", "ASSUMED"), f"{group}.{key}")
                    self.assertTrue(item["source"], f"{group}.{key}")
                    if item["status"] == "ASSUMED":
                        self.assertIn("TODO(HUMAN-VERIFY)", item["todo"], f"{group}.{key}")

    def test_motor_numbers_match_the_tmotor_page(self):
        m = CAT["motor"]
        self.assertEqual((m["kv_rpm_per_v"]["value"], m["resistance_ohm"]["value"], m["no_load_current_a"]["value"],
                          m["mass_kg"]["value"], m["max_continuous_current_a"]["value"], m["max_power_w"]["value"]),
                         (700, 0.072, 0.5, 0.104, 23, 510))


class MassInertiaTests(unittest.TestCase):
    def test_mass_hand_sum(self):
        # frame 0.350 + payload 0.120 + battery 0.450 + 4 x (0.104 + 0.025 + 0.020 = 0.149 -> 0.596) = 1.516 kg
        self.assertAlmostEqual(b.total_mass_kg(CAT), 1.516, places=12)

    def test_inertia_hand_calc(self):
        # Corner mass 0.149 kg at (+-0.2, +-0.2): Ixx_pts = 4 * 0.149 * 0.04 = 0.02384 (same Iyy), Izz_pts = 0.04768.
        # Box mass = 1.516 - 0.596 = 0.920 kg, a=b=0.20, h=0.10:
        #   Ixx = 0.92 * (0.04 + 0.01) / 12 = 0.0038333..., Izz = 0.92 * 0.08 / 12 = 0.0061333...
        # Total: Ixx = Iyy = 0.0276733..., Izz = 0.0538133...
        i = b.inertia_kgm2(CAT, b.motor_positions(0.4))
        self.assertAlmostEqual(i[0], 0.02384 + 0.92 * 0.05 / 12, places=12)
        self.assertAlmostEqual(i[1], i[0], places=12)
        self.assertAlmostEqual(i[2], 0.04768 + 0.92 * 0.08 / 12, places=12)

    def test_motor_positions_order_and_spacing(self):
        p = b.motor_positions(0.4)
        self.assertEqual(p, [[0.2, 0.2, 0.0], [0.2, -0.2, 0.0], [-0.2, 0.2, 0.0], [-0.2, -0.2, 0.0]])


class ClearanceTests(unittest.TestCase):
    def test_reference_build_clears(self):
        # Adjacent tip gap 0.4 - 0.3048 = 0.0952; inner tip radius sqrt(0.08) - 0.1524 = 0.28284 - 0.1524 = 0.13044;
        # body corner 0.06 * sqrt(2) = 0.08485 < 0.13044.
        r = b.check_clearance(CAT, b.motor_positions(0.4))
        self.assertAlmostEqual(r["adjacent_tip_gap_m"], 0.0952, places=9)
        self.assertAlmostEqual(r["inner_tip_radius_m"], math.sqrt(0.08) - 0.1524, places=9)

    def test_overlapping_props_rejected(self):
        cat = json.loads(json.dumps(CAT))
        cat["prop"]["diameter_m"]["value"] = 0.41
        with self.assertRaises(ValueError):
            b.check_clearance(cat, b.motor_positions(0.4))

    def test_wide_body_rejected(self):
        cat = json.loads(json.dumps(CAT))
        cat["frame"]["body_half_width_m"]["value"] = 0.2
        with self.assertRaises(ValueError):
            b.check_clearance(cat, b.motor_positions(0.4))


class HoverTests(unittest.TestCase):
    def test_hover_throttle_hand_calc(self):
        # m g = 1.516 * 9.80665 = 14.86683; D^4 = 0.3048^4 = 0.00863095; (8100/60)^2 = 18225
        # denominator = 4 * 0.100256 * 1.225 * 18225 * 0.00863095 = 77.2751...
        # throttle = sqrt(14.86683 / 77.2751) = sqrt(0.192381) = 0.43861
        t = b.hover_throttle(1.516, 0.100256, 0.3048, 8100.0)
        self.assertAlmostEqual(t, 0.43861, delta=2e-5)

    def test_heavier_needs_more_throttle(self):
        self.assertGreater(b.hover_throttle(2.0, 0.1, 0.3, 8000.0), b.hover_throttle(1.0, 0.1, 0.3, 8000.0))


class BuildTests(unittest.TestCase):
    def test_build_from_shipped_prop_json(self):
        prop = json.loads(b.PROP_JSON.read_text())
        cfg = b.build(CAT, prop)
        self.assertEqual(cfg["motor_spin"], [1.0, -1.0, -1.0, 1.0])
        self.assertEqual(cfg["prop_diameter_m"], 0.3048)
        self.assertEqual(cfg["max_rpm"], 8100.0)
        self.assertAlmostEqual(cfg["thrust_coeff"], 0.100256, places=6)
        self.assertAlmostEqual(cfg["torque_coeff"], 0.015423, delta=1e-6)
        self.assertEqual(cfg["mass_kg"], 1.516)

    def test_written_config_is_current(self):
        prop = json.loads(b.PROP_JSON.read_text())
        self.assertEqual(json.loads(b.OUT.read_text()), b.build(CAT, prop), "run build_ref_config.py")


if __name__ == "__main__":
    unittest.main()
