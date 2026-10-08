#!/usr/bin/env python3
"""Checks code comments for banned AI conversational artifacts.

Enforces Section 6 of UAV_Simulator_Code_Commenting_Standards.
Scans comments in changed .rs, .cpp, and .h files for first-person narration,
unverified correctness assertions, and conversational phrases.
"""

import argparse
import os
import re
import subprocess
import sys
from typing import List, Tuple

BANNED_PATTERNS = [
    re.compile(r"\bi['’]?ve\s+updated\b", re.IGNORECASE),
    re.compile(r"\bi\s+updated\b", re.IGNORECASE),
    re.compile(r"\bi['’]?ve\s+fixed\b", re.IGNORECASE),
    re.compile(r"\bi\s+fixed\b", re.IGNORECASE),
    re.compile(r"\blet\s+me\b", re.IGNORECASE),
    re.compile(r"\bhere['’]?s\s+the\s+corrected\b", re.IGNORECASE),
    re.compile(r"\bhere\s+is\s+the\s+corrected\b", re.IGNORECASE),
    re.compile(r"\bthis\s+should\s+now\b", re.IGNORECASE),
    re.compile(r"\bthis\s+should\s+work\b", re.IGNORECASE),
    re.compile(r"\bas\s+requested\b", re.IGNORECASE),
    re.compile(r"\bper\s+your\s+instructions\b", re.IGNORECASE),
    re.compile(r"\bsorry[, ]+\s*fixed\b", re.IGNORECASE),
    re.compile(r"\bthis\s+is\s+a\s+simple\b", re.IGNORECASE),
    re.compile(r"\bthis\s+is\s+a\s+straightforward\b", re.IGNORECASE),
    re.compile(r"\bcorrectly\s+handles\s+all\s+edge\s+cases\b", re.IGNORECASE),
    re.compile(r"\bsimple\s+implementation\b", re.IGNORECASE),
    re.compile(r"\bstraightforward\s+implementation\b", re.IGNORECASE),
]

VALID_EXTENSIONS = {".rs", ".cpp", ".h", ".hpp", ".c"}


def get_changed_files() -> List[str]:
    """Retrieve list of modified/added files using git."""
    commands = [
        ["git", "diff", "--name-only", "--diff-filter=ACMRT", "origin/main...HEAD"],
        ["git", "diff", "--name-only", "--diff-filter=ACMRT", "main...HEAD"],
        ["git", "diff", "--name-only", "--diff-filter=ACMRT", "HEAD~1...HEAD"],
        ["git", "diff", "--name-only", "--diff-filter=ACMRT", "HEAD"],
    ]

    for cmd in commands:
        try:
            result = subprocess.run(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                check=True,
            )
            files = [line.strip() for line in result.stdout.splitlines() if line.strip()]
            if files:
                return files
        except subprocess.SubprocessError:
            continue

    # Fallback to git status for uncommitted changes
    try:
        result = subprocess.run(
            ["git", "status", "--porcelain"],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            check=True,
        )
        files = []
        for line in result.stdout.splitlines():
            parts = line.strip().split()
            if len(parts) >= 2:
                files.append(parts[-1])
        return files
    except subprocess.SubprocessError:
        return []


def check_file(file_path: str) -> List[Tuple[int, str, str]]:
    """Scan comments in a file for banned patterns.

    Returns list of (line_number, line_content, matched_pattern).
    """
    matches = []
    if not os.path.isfile(file_path):
        return matches

    in_block_comment = False

    try:
        with open(file_path, "r", encoding="utf-8", errors="replace") as f:
            for line_no, line in enumerate(f, start=1):
                stripped = line.strip()

                is_comment = False
                comment_text = ""

                if in_block_comment:
                    is_comment = True
                    if "*/" in stripped:
                        in_block_comment = False
                        comment_text = stripped.split("*/")[0]
                    else:
                        comment_text = stripped
                else:
                    if "/*" in stripped:
                        is_comment = True
                        if "*/" not in stripped:
                            in_block_comment = True
                        comment_text = stripped
                    elif "//" in stripped:
                        is_comment = True
                        comment_text = stripped[stripped.index("//") :]

                if is_comment:
                    for pattern in BANNED_PATTERNS:
                        if pattern.search(comment_text):
                            matches.append(
                                (line_no, stripped, pattern.pattern)
                            )
                            break
    except OSError as err:
        print(f"Warning: unable to read '{file_path}': {err}", file=sys.stderr)

    return matches


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Check code comments for banned AI conversational artifacts."
    )
    parser.add_argument(
        "--changed-files-only",
        action="store_true",
        help="Only check files changed relative to git upstream/base.",
    )
    parser.add_argument(
        "--path",
        help="Explicit file or directory path to check.",
    )
    parser.add_argument(
        "--fail-on-match",
        action="store_true",
        help="Exit with non-zero code if banned artifacts are detected.",
    )

    args = parser.parse_args()

    files_to_check: List[str] = []

    if args.path:
        if os.path.isfile(args.path):
            files_to_check = [args.path]
        elif os.path.isdir(args.path):
            for root, _, filenames in os.walk(args.path):
                for fname in sorted(filenames):
                    if os.path.splitext(fname)[1] in VALID_EXTENSIONS:
                        files_to_check.append(os.path.join(root, fname))
    elif args.changed_files_only:
        raw_files = get_changed_files()
        files_to_check = [
            f
            for f in raw_files
            if os.path.splitext(f)[1] in VALID_EXTENSIONS and os.path.isfile(f)
        ]
        print(f"Checking {len(files_to_check)} changed code file(s)...")
    else:
        # Default: scan entire repository
        for root, _, filenames in os.walk("."):
            if any(p in root.split(os.sep) for p in [".git", "target", "Intermediate", "Binaries"]):
                continue
            for fname in sorted(filenames):
                if os.path.splitext(fname)[1] in VALID_EXTENSIONS:
                    files_to_check.append(os.path.join(root, fname))

    total_violations = 0
    for fpath in files_to_check:
        matches = check_file(fpath)
        if matches:
            total_violations += len(matches)
            for line_no, content, pat in matches:
                print(f"{fpath}:{line_no}: BANNED COMMENT PATTERN [{pat}] -> {content}")

    if total_violations > 0:
        print(f"\nFound {total_violations} banned comment artifact(s).")
        if args.fail_on_match:
            print("ERROR: Comment style gate failed per Section 6 of Commenting Standards.", file=sys.stderr)
            sys.exit(1)
    else:
        print("Comment style check passed. Zero banned artifacts found.")
        sys.exit(0)


if __name__ == "__main__":
    main()
