#!/usr/bin/env python3
"""Merge per-package compile_commands.json files into a single workspace-level one.

colcon/CMake writes one compilation database per package under
``build/<package>/compile_commands.json`` when the workspace is configured with
``-DCMAKE_EXPORT_COMPILE_COMMANDS=ON``. clangd only looks for a database at the
project root (or in the directory given by ``--compile-commands-dir``), so the
per-package files need to be concatenated.

Usage:
    python3 scripts/merge_compile_commands.py [--build-dir build] [--output compile_commands.json]

Run this from the workspace root after every ``colcon build`` that changes
compiler flags or sources, then reload the clangd language server in VSCode
(``clangd: Restart language server``).
"""

import argparse
import glob
import json
import os
import shlex
import sys


def split_command(entry):
    """Return the compile flags of an entry as a list of tokens."""
    if "arguments" in entry:
        return list(entry["arguments"])
    command = entry.get("command", "")
    return shlex.split(command)


def ensure_std_flag(tokens, std_flag):
    """Force a single -std= flag on the compile command.

    CMake does not always emit -std= into the compile command (it can be
    carried by the CXX_STANDARD target property), and clangd then falls back
    to its default standard, which breaks C++17 code such as std::variant and
    std::is_convertible_v.
    """
    tokens = [t for t in tokens if not t.startswith("-std=")]
    # Insert the standard flag right after the compiler driver.
    insert_at = 1 if tokens else 0
    tokens.insert(insert_at, std_flag)
    return tokens


def ensure_warning_flags(tokens, warning_flags):
    """Make sure the warning flags are present in the compile command.

    clang only enables a small set of warnings by default, so constructs such
    as ``int z = 2.3;`` (a narrowing conversion) stay silent. Enabling
    -Wall/-Wextra makes clangd surface them as diagnostics in the editor.
    """
    for flag in warning_flags:
        if flag not in tokens:
            tokens.append(flag)
    return tokens


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--build-dir",
        default="build",
        help="colcon build directory containing per-package databases (default: build)",
    )
    parser.add_argument(
        "--output",
        default="compile_commands.json",
        help="path of the merged database to write (default: compile_commands.json)",
    )
    parser.add_argument(
        "--std",
        default="c++17",
        help="C++ standard to force on every entry (default: c++17)",
    )
    parser.add_argument(
        "--warnings",
        nargs="*",
        default=["-Wall", "-Wextra", "-Wpedantic"],
        help="warning flags to force on every entry (default: -Wall -Wextra -Wpedantic)",
    )
    args = parser.parse_args()

    workspace = os.path.abspath(os.getcwd())
    pattern = os.path.join(args.build_dir, "*", "compile_commands.json")
    per_package = sorted(glob.glob(pattern))

    if not per_package:
        print(
            "ERROR: no per-package compile_commands.json found under '{}'.\n"
            "       Build the workspace first, e.g.:\n"
            "       source /opt/ros/humble/setup.sh && colcon build --symlink-install \\\n"
            "           --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON".format(args.build_dir),
            file=sys.stderr,
        )
        return 1

    merged = []
    seen = set()
    for path in per_package:
        try:
            with open(path, "r", encoding="utf-8") as handle:
                entries = json.load(handle)
        except (OSError, ValueError) as exc:
            print("WARNING: skipping {}: {}".format(path, exc), file=sys.stderr)
            continue

        added = 0
        for entry in entries:
            source = os.path.abspath(entry.get("file", ""))
            if not source:
                continue
            if source in seen:
                continue
            seen.add(source)
            # Keep the file path absolute and consistent with the on-disk layout.
            entry["file"] = source
            # Normalise to the "arguments" form: clangd parses it without any
            # shell quoting ambiguity, and it lets us force the C++ standard.
            tokens = ensure_std_flag(split_command(entry), "-std={}".format(args.std))
            tokens = ensure_warning_flags(tokens, args.warnings)
            entry["arguments"] = tokens
            entry.pop("command", None)
            merged.append(entry)
            added += 1
        print("merged {:>3} entries from {}".format(added, os.path.relpath(path, workspace)))

    with open(args.output, "w", encoding="utf-8") as handle:
        json.dump(merged, handle, indent=2, sort_keys=True)
        handle.write("\n")

    print("wrote {} entries to {}".format(len(merged), os.path.relpath(args.output, workspace)))
    return 0


if __name__ == "__main__":
    sys.exit(main())
