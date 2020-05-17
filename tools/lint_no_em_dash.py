#!/usr/bin/env python3
"""
Zero Em-Dash Linter for Moonpool.
Scans source files, documentation, headers, and scripts for disallowed em-dashes
(Unicode U+2014, U+2013, or double-hyphen prose em-dashes).
"""

import os
import sys

# Allowed file extensions to scan
SCANNED_EXTENSIONS = {
    ".md", ".txt", ".c", ".h", ".py", ".sh", ".e", ".ex", ".ew", ".eu", "Makefile"
}

# Directories to skip
IGNORED_DIRS = {
    ".git", "build", "dist", ".euphoria", "__pycache__", "bin"
}

DISALLOWED_PATTERN = " " + chr(45) + chr(45) + " "

def check_file(file_path):
    violations = []
    # Avoid self-flagging on this linter tool
    if os.path.basename(file_path) == "lint_no_em_dash.py":
        return violations

    try:
        with open(file_path, "r", encoding="utf-8", errors="replace") as f:
            for line_idx, line in enumerate(f, start=1):
                # Check for unicode em-dash or en-dash
                if "\u2014" in line:
                    violations.append((line_idx, "Disallowed Unicode em-dash (U+2014) detected"))
                if "\u2013" in line:
                    violations.append((line_idx, "Disallowed Unicode en-dash (U+2013) detected"))
                
                stripped = line.strip()
                if file_path.endswith((".e", ".ex", ".ew", ".eu")):
                    if stripped.startswith("--"):
                        comment_body = stripped[2:]
                        if DISALLOWED_PATTERN in comment_body:
                            violations.append((line_idx, "Disallowed double-hyphen em-dash in comment body"))
                else:
                    if DISALLOWED_PATTERN in line:
                        violations.append((line_idx, "Disallowed double-hyphen em-dash in text"))
    except Exception as exc:
        violations.append((0, f"Failed to read file: {exc}"))
    return violations

def main():
    root_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
    total_files = 0
    total_violations = 0

    print("Running Moonpool zero em-dash typography verification...")
    for current_root, dirs, files in os.walk(root_dir):
        dirs[:] = [d for d in dirs if d not in IGNORED_DIRS]
        for fname in files:
            ext = os.path.splitext(fname)[1]
            if ext in SCANNED_EXTENSIONS or fname in SCANNED_EXTENSIONS:
                total_files += 1
                fpath = os.path.join(current_root, fname)
                rel_path = os.path.relpath(fpath, root_dir)
                v = check_file(fpath)
                if v:
                    for line_num, msg in v:
                        print(f"FAIL: {rel_path}:{line_num}: {msg}")
                        total_violations += 1

    if total_violations == 0:
        print(f"PASS: Verified {total_files} files. Zero em-dashes found.")
        sys.exit(0)
    else:
        print(f"ERROR: Found {total_violations} typography violations across {total_files} files.")
        sys.exit(1)

if __name__ == "__main__":
    main()
