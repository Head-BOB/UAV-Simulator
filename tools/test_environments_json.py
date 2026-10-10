# Tier: F
"""Checks DroneEnvironments.json against the limits in DroneEnvironment.h (parsed from the C++ text),
so a preset that the demo would reject at run time fails here instead. Expected densities are
hand-derived (see the comments)."""
import json
import math
import re
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
PLUGIN = ROOT / "UE5Project" / "Plugins" / "DronePhysicsBridge"
HEADER = (PLUGIN / "Source" / "DronePhysicsBridge" / "Public" / "DroneEnvironment.h").read_text(encoding="utf-8")
DATA = json.loads((PLUGIN / "Content" / "DroneEnvironments.json").read_text(encoding="utf-8"))
EXPECTED_IDS = ["Beach", "Hills", "Mountains", "Desert", "Tropical"]
FIELDS = {"id", "name", "level_asset_path", "player_start_cm", "player_yaw_deg", "site_altitude_m", "ambient_temp_c",
          "humidity_pct", "wind_speed_mps", "wind_toward_deg", "gust_sigma_mps", "sky_tint_rgb", "fog_density",
          "light_intensity_scale"}


def limit(name: str) -> float:
    m = re.search(rf"constexpr double {name} = ([0-9.]+);", HEADER)
    if not m:
        raise AssertionError(f"{name} not found in DroneEnvironment.h")
    return float(m.group(1))


def isa_temp_c(h: float) -> float:
    return 15.0 - 0.0065 * h


class PresetTests(unittest.TestCase):
    def test_five_expected_environments_in_order(self):
        self.assertEqual([e["id"] for e in DATA["environments"]], EXPECTED_IDS)
        self.assertIn(DATA["default_environment"], EXPECTED_IDS)

    def test_fields_present_and_numeric(self):
        for e in DATA["environments"]:
            self.assertEqual(set(e), FIELDS, e["id"])
            for k in FIELDS - {"id", "name", "level_asset_path", "player_start_cm", "sky_tint_rgb"}:
                self.assertTrue(isinstance(e[k], (int, float)) and math.isfinite(e[k]), f"{e['id']}.{k}")
            self.assertEqual(len(e["player_start_cm"]), 3)
            self.assertTrue(all(0.0 <= c <= 1.0 for c in e["sky_tint_rgb"]))

    def test_every_preset_passes_the_runtime_limits(self):
        for e in DATA["environments"]:
            self.assertTrue(0.0 <= e["wind_speed_mps"] <= limit("kMaxWindSpeedMps"), e["id"])
            self.assertTrue(0.0 <= e["gust_sigma_mps"] <= limit("kMaxGustSigmaMps"), e["id"])
            self.assertTrue(0.0 <= e["site_altitude_m"] <= limit("kMaxSiteAltitudeM"), e["id"])
            self.assertTrue(0.0 <= e["humidity_pct"] <= 100.0, e["id"])
            offset = e["ambient_temp_c"] - isa_temp_c(e["site_altitude_m"])
            self.assertLessEqual(abs(offset), limit("kMaxTempOffsetC"), e["id"])

    def test_wind_direction_in_range(self):
        for e in DATA["environments"]:
            self.assertTrue(0.0 <= e["wind_toward_deg"] < 360.0, e["id"])

    def test_ids_unique_and_levels_marked_todo(self):
        ids = [e["id"] for e in DATA["environments"]]
        self.assertEqual(len(set(ids)), len(ids))
        self.assertIn("TODO(HUMAN-FILL)", DATA["_comment"])
        self.assertIn("ASSUMED", DATA["_comment"])

    def test_mountains_offset_hand_calc(self):
        # T_isa(2500) = 15 - 0.0065 * 2500 = -1.25 C; ambient 2 C -> dT = +3.25 K.
        e = next(x for x in DATA["environments"] if x["id"] == "Mountains")
        self.assertAlmostEqual(e["ambient_temp_c"] - isa_temp_c(e["site_altitude_m"]), 3.25, places=12)

    def test_header_limits_match_the_documented_values(self):
        # The Rust test types::tests::test_cpp_environment_limits_match_rust ties these to the core.
        self.assertEqual(limit("kMaxSiteAltitudeM"), 5000.0)
        self.assertEqual(limit("kMaxTempOffsetC"), 40.0)
        self.assertEqual(limit("kMaxWindSpeedMps"), 15.0)


if __name__ == "__main__":
    unittest.main()
