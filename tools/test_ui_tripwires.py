# Tier: V
import shutil
import tempfile
import unittest
from pathlib import Path

import check_ui_tripwires as T

ROOT = Path(__file__).resolve().parents[1]


class TripwireTests(unittest.TestCase):
    def copy(self):
        t = tempfile.TemporaryDirectory()
        dst = Path(t.name) / "UE5Project" / "Source" / "UAV_Simulator"
        shutil.copytree(ROOT / "UE5Project" / "Source" / "UAV_Simulator", dst)
        return t, Path(t.name)

    def test_the_repository_passes(self):
        self.assertEqual(T.check(ROOT), [])

    def test_allocation_in_paint_is_caught(self):
        t, root = self.copy()
        p = root / "UE5Project/Source/UAV_Simulator/UI/SDroneAppCanvas.cpp"
        p.write_text(p.read_text().replace("RefreshTextCache();\n\tconst FVector2f Size", "RefreshTextCache();\n\tFString Str(TEXT(\"x\"));\n\tconst FVector2f Size", 1))
        self.assertTrue(any("OnPaint contains" in e for e in T.check(root)))
        t.cleanup()

    def test_missing_header_marker_and_missing_file_are_caught(self):
        t, root = self.copy()
        p = root / "UE5Project/Source/UAV_Simulator/DroneAppHUD.h"
        p.write_text(p.read_text().replace("// Tier: V", "// header", 1).replace("NOT COMPILED OR RUN", "tested"))
        errs = T.check(root)
        self.assertTrue(any("Tier header" in e for e in errs) and any("NOT COMPILED" in e for e in errs))
        (root / "UE5Project/Source/UAV_Simulator/UI/DroneAppIO.cpp").unlink()
        self.assertTrue(any("missing" in e for e in T.check(root)))
        t.cleanup()

    def test_hand_conversion_is_caught(self):
        t, root = self.copy()
        p = root / "UE5Project/Source/UAV_Simulator/DroneBuildPreview.cpp"
        p.write_text(p.read_text() + "\nvoid X(){ double NedZ = 1; }\n")
        self.assertTrue(any("hand-written NED" in e for e in T.check(root)))
        t.cleanup()

    def test_function_body_extraction(self):
        self.assertEqual(T.function_body("void f(){ a{b} }", "void f("), "{ a{b} }")
        self.assertIsNone(T.function_body("nothing", "void f("))


if __name__ == "__main__":
    unittest.main()
