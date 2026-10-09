# Tier: F
"""Unit tests for tools/check_tier_headers.py (hand-written expected results)."""
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import check_tier_headers as cth  # noqa: E402


class CheckText(unittest.TestCase):
    def test_valid_rust(self):
        self.assertIsNone(cth.check_text("physics_core/src/a.rs", "// Tier: F\nuse x;\n", "//"))

    def test_missing_header(self):
        self.assertIn("must be", cth.check_text("tools/a.py", "import os\n", "#"))

    def test_tag_on_line_two_rejected_for_rust(self):
        self.assertIsNotNone(cth.check_text("tools/a.cpp", "#pragma once\n// Tier: F\n", "//"))

    def test_tag_on_line_three_rejected_for_python_shebang(self):
        text = "#!/usr/bin/env python3\n\n# Tier: F\n"
        self.assertIsNotNone(cth.check_text("tools/a.py", text, "#"))

    def test_python_shebang_then_tag_ok(self):
        text = "#!/usr/bin/env python3\n# Tier: D\n"
        self.assertIsNone(cth.check_text("tools/a.py", text, "#"))

    def test_unknown_tier_x_rejected(self):
        self.assertIn("not one of", cth.check_text("tools/a.py", "# Tier: X\n", "#"))

    def test_lowercase_tier_rejected(self):
        self.assertIsNotNone(cth.check_text("tools/a.py", "# Tier: f\n", "#"))

    def test_physics_core_must_be_f(self):
        self.assertIn("requires Tier F", cth.check_text("physics_core/src/a.rs", "// Tier: D\n", "//"))

    def test_offline_pipeline_must_be_d(self):
        self.assertIn("requires Tier D", cth.check_text("offline_pipeline/a.py", "# Tier: F\n", "#"))

    def test_crlf_header_accepted(self):
        self.assertIsNone(cth.check_text("tools/a.py", "# Tier: F\r\nimport os\r\n", "#"))

    def test_empty_file_rejected(self):
        self.assertIsNotNone(cth.check_text("tools/a.py", "", "#"))


class CollectFiles(unittest.TestCase):
    def test_skips_generated_and_finds_scopes(self):
        with tempfile.TemporaryDirectory() as d:
            root = Path(d)
            for rel in ["physics_core/src/a.rs", "tools/x.py",
                        "UE5Project/Plugins/P/Source/P/Public/Generated/g.h",
                        "UE5Project/Plugins/P/Source/P/Public/b.h",
                        "offline_pipeline/__pycache__/c.py"]:
                (root / rel).parent.mkdir(parents=True, exist_ok=True)
                (root / rel).write_text("x")
            got = [r for r, _ in cth.collect_files(root)]
            self.assertEqual(got, ["UE5Project/Plugins/P/Source/P/Public/b.h",
                                   "physics_core/src/a.rs", "tools/x.py"])


if __name__ == "__main__":
    unittest.main()
