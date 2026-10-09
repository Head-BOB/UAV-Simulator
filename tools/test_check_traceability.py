# Tier: F
import os
import sys
import tempfile
import unittest

sys.path.insert(0, os.path.dirname(__file__))
from check_traceability import HEADER, check, find_ignore_attributes  # noqa: E402

SEP = "| :--- | :--- | :--- | :--- | :--- | :--- |"


def table(*rows):
    return "\n".join([HEADER, SEP, *rows]) + "\n"


def row(test, rule="exact", status="PASS"):
    return f"| req | WP-1 | `{test}` | {rule} | {status} | note |"


A = "a::tests::test_a"
B = "b::tests::test_b"


class CheckTraceability(unittest.TestCase):
    def test_valid_table_has_no_errors(self):
        self.assertEqual(check(table(row(A), row(B, status="UNVERIFIED")), {A, B}), [])

    def test_empty_pass_rule_is_one_error(self):
        errors = check(table(row(A, rule=""), row(B)), {A, B})
        self.assertEqual(len(errors), 1)
        self.assertIn(A, errors[0])

    def test_invalid_status_is_one_error(self):
        errors = check(table(row(A, status="OK"), row(B)), {A, B})
        self.assertEqual(len(errors), 1)
        self.assertIn(A, errors[0])

    def test_named_test_that_does_not_exist(self):
        errors = check(table(row(A), row(B)), {A})
        self.assertEqual(len(errors), 1)
        self.assertIn(B, errors[0])

    def test_existing_test_not_named(self):
        errors = check(table(row(A)), {A, B})
        self.assertEqual(len(errors), 1)
        self.assertIn(B, errors[0])

    def test_missing_header(self):
        errors = check("| nothing |\n", {A})
        self.assertTrue(any("header" in e for e in errors))

    def test_top_level_tests_module_name_accepted(self):
        name = "tests::test_ffi_x"
        self.assertEqual(check(table(row(name)), {name}), [])

    def test_ignored_test_is_an_error(self):
        errors = check(table(row(A), row(B)), {A, B}, ignored={B})
        self.assertEqual(len(errors), 1)
        self.assertIn(B, errors[0])
        self.assertIn("ignore", errors[0])

    def test_ignore_attribute_hit_is_an_error(self):
        errors = check(table(row(A)), {A}, ignore_attrs=["src/x.rs:7"])
        self.assertEqual(len(errors), 1)
        self.assertIn("src/x.rs:7", errors[0])

    def test_no_ignored_tests_means_no_extra_errors(self):
        self.assertEqual(check(table(row(A)), {A}, ignored=set(), ignore_attrs=[]), [])


class FindIgnoreAttributes(unittest.TestCase):
    def scan(self, source):
        with tempfile.TemporaryDirectory() as d:
            with open(os.path.join(d, "x.rs"), "w", encoding="utf-8") as f:
                f.write(source)
            return [h.split(os.sep)[-1] for h in find_ignore_attributes([d])]

    def test_plain_ignore_found_with_line_number(self):
        self.assertEqual(self.scan("#[test]\n#[ignore]\nfn t() {}\n"), ["x.rs:2"])

    def test_ignore_with_reason_and_spaces_found(self):
        self.assertEqual(self.scan('#[ignore = "slow"]\n'), ["x.rs:1"])
        self.assertEqual(self.scan("#[ ignore ]\n"), ["x.rs:1"])

    def test_word_containing_ignore_is_not_flagged(self):
        self.assertEqual(self.scan("#[ignored_helper]\nfn ignore() {}\n"), [])

    def test_commented_out_ignore_is_not_flagged(self):
        self.assertEqual(self.scan("// #[ignore]\n"), [])

    def test_clean_file_has_no_hits(self):
        self.assertEqual(self.scan("#[test]\nfn t() {}\n"), [])


if __name__ == "__main__":
    unittest.main()
