# Tier: F
"""Unit tests for tools/check_mass_literals.py (hand-written inputs, expected error counts)."""
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import check_mass_literals as cml  # noqa: E402


def n_errors(code: str) -> int:
    return len(cml.check_text("x.rs", code))


class Flagged(unittest.TestCase):
    def test_struct_field_literal(self):
        self.assertEqual(n_errors("let c = Cfg { mass_kg: 1.0 };\n"), 1)

    def test_let_binding(self):
        self.assertEqual(n_errors("let mass = 1.5;\n"), 1)

    def test_inertia_array(self):
        self.assertEqual(n_errors("inertia_kgm2: [1.0, 1.0, 1.0],\n"), 1)

    def test_negative_literal(self):
        self.assertEqual(n_errors("let total_mass = -2.5;\n"), 1)

    def test_typed_const_uppercase(self):
        self.assertEqual(n_errors("const DEFAULT_INERTIA: f64 = 1.0;\n"), 1)

    def test_typed_array_let(self):
        self.assertEqual(n_errors("let inertia: [f64; 3] = [1.0, 1.0, 1.0];\n"), 1)

    def test_comparison_with_literal(self):
        self.assertEqual(n_errors("if cfg.mass_kg > 0.0 { }\n"), 1)

    def test_each_line_counted(self):
        self.assertEqual(n_errors("let mass = 1.0;\nlet inertia = 2.0;\n"), 2)


class NotFlagged(unittest.TestCase):
    def test_type_only(self):
        self.assertEqual(n_errors("pub mass_kg: f64,\n"), 0)

    def test_arithmetic_with_config_mass(self):
        self.assertEqual(n_errors("let w = config.mass_kg * 9.80665;\n"), 0)

    def test_line_comment(self):
        self.assertEqual(n_errors("// mass_kg: 1.0\n"), 0)

    def test_doc_comment(self):
        self.assertEqual(n_errors("/// mass = 1.0 kg\n"), 0)

    def test_block_comment_multiline(self):
        self.assertEqual(n_errors("/* a\n mass = 1.0\n*/ let x = 2.0;\n"), 0)

    def test_string_literal(self):
        self.assertEqual(n_errors('let s = "mass_kg: 1.0";\n'), 0)

    def test_unrelated_names(self):
        self.assertEqual(n_errors("let thrust_n = 1.0; let masses = other;\n"), 0)

    def test_test_module_is_ignored(self):
        code = "fn f() {}\n#[cfg(test)]\nmod tests {\n    fn t() { let mass = 1.0; }\n}\n"
        self.assertEqual(n_errors(code), 0)


class StructureChecks(unittest.TestCase):
    def test_code_before_cfg_test_is_scanned(self):
        code = "let mass = 1.0;\n#[cfg(test)]\nmod tests {}\n"
        self.assertEqual(n_errors(code), 1)

    def test_line_number_reported(self):
        errs = cml.check_text("x.rs", "fn a() {}\n/* c */\nlet mass = 3.0;\n")
        self.assertTrue(errs[0].startswith("x.rs:3:"))

    def test_two_cfg_test_rejected(self):
        code = "#[cfg(test)]\nmod tests {}\n#[cfg(test)]\nmod more {}\n"
        self.assertEqual(n_errors(code), 1)

    def test_cfg_test_not_before_mod_tests_rejected(self):
        self.assertEqual(n_errors("#[cfg(test)]\nfn helper() {}\n"), 1)

    # ADDED (2026-10-11): `pub(crate) mod tests` is a normal way to share test helpers between modules
    def test_pub_crate_mod_tests_is_accepted_and_the_cut_point_holds(self):
        self.assertEqual(n_errors("fn f() {}\n#[cfg(test)]\npub(crate) mod tests {\n    fn t() { let mass = 1.0; }\n}\n"), 0)
        self.assertEqual(n_errors("fn f() {}\n#[cfg(test)]\npub mod tests {}\n"), 0)

    def test_a_mass_literal_before_a_pub_crate_test_module_is_still_caught(self):
        self.assertEqual(n_errors("let mass = 1.0;\n#[cfg(test)]\npub(crate) mod tests {}\n"), 1)

    def test_two_cfg_test_are_still_refused_with_pub_crate(self):
        self.assertEqual(n_errors("#[cfg(test)]\npub(crate) mod tests {}\n#[cfg(test)]\nmod more {}\n"), 1)

    def test_pub_crate_on_something_that_is_not_mod_tests_is_refused(self):
        self.assertEqual(n_errors("#[cfg(test)]\npub(crate) mod helpers {}\n"), 1)
        self.assertEqual(n_errors("#[cfg(test)]\npub(crate) fn tests() {}\n"), 1)

    def test_a_named_constant_instead_of_a_literal_comparison_passes(self):
        self.assertEqual(n_errors("if !(m.is_finite() && m >= MIN_VEHICLE_MASS_KG) { }\n"), 0)
        self.assertEqual(n_errors("pub const MIN_VEHICLE_MASS_KG: f64 = f64::MIN_POSITIVE;\n"), 0)


class RepositoryIsClean(unittest.TestCase):
    def test_the_rust_sources_have_no_mass_or_inertia_literal(self):
        root = Path(__file__).resolve().parent.parent
        errors = []
        for p in sorted((root / "physics_core" / "src").rglob("*.rs")):
            errors += cml.check_text(p.relative_to(root).as_posix(), p.read_text(encoding="utf-8"))
        self.assertEqual(errors, [])


if __name__ == "__main__":
    unittest.main()
