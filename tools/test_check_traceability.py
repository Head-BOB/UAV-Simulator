# Tier: F
import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(__file__))
from check_traceability import HEADER, check  # noqa: E402

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


if __name__ == "__main__":
    unittest.main()
