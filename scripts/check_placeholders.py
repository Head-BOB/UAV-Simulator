#!/usr/bin/env python3
"""Scanner for placeholder physical constants across physics_core.

Enforces release-candidate gate: ensures no unmeasured PLACEHOLDER: constants
reach release builds unverified.
"""

import argparse
import os
import sys
from typing import List, Tuple


def scan_placeholders(search_path: str) -> List[Tuple[str, int, str]]:
    """Scan directory recursively for files containing literal 'PLACEHOLDER:'.

    Args:
        search_path: File or directory path to scan.

    Returns:
        List of (file_path, line_number, line_content) tuples.
    """
    matches: List[Tuple[str, int, str]] = []

    if os.path.isfile(search_path):
        target_files = [search_path]
    elif os.path.isdir(search_path):
        target_files = []
        for root, _, files in os.walk(search_path):
            for filename in sorted(files):
                if filename.endswith(".rs"):
                    target_files.append(os.path.join(root, filename))
    else:
        print(f"Error: path '{search_path}' does not exist.", file=sys.stderr)
        sys.exit(2)

    for file_path in target_files:
        try:
            with open(file_path, "r", encoding="utf-8", errors="replace") as f:
                for line_idx, line in enumerate(f, start=1):
                    if "PLACEHOLDER:" in line:
                        matches.append((file_path, line_idx, line.strip()))
        except OSError as err:
            print(f"Warning: unable to read '{file_path}': {err}", file=sys.stderr)

    return matches


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Scan codebase for unmeasured PLACEHOLDER: constant markers."
    )
    parser.add_argument(
        "--path",
        default="physics_core/src",
        help="Path to scan (default: physics_core/src)",
    )
    parser.add_argument(
        "--fail-on-match",
        action="store_true",
        help="Exit with non-zero status if any PLACEHOLDER: markers are found.",
    )

    args = parser.parse_args()

    matches = scan_placeholders(args.path)

    if matches:
        print(f"Found {len(matches)} PLACEHOLDER marker(s) in '{args.path}':")
        for file_path, line_no, content in matches:
            print(f"  {file_path}:{line_no}: {content}")

        if args.fail_on_match:
            print(
                f"\nERROR: Release gate failed. {len(matches)} unmeasured placeholder constant(s) "
                "remain in flight-critical code.",
                file=sys.stderr,
            )
            sys.exit(1)
    else:
        print(f"No PLACEHOLDER markers found in '{args.path}'. Clean.")
        sys.exit(0)


if __name__ == "__main__":
    main()
