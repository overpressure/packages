#!/usr/bin/env python3
# SPDX-License-Identifier: Unlicense
"""Shellcheck over every tracked shell script, the build's and the recipes', down to its warnings.

    python3 build/shellcheck.py

A script is one by its first line (bash, or sh as dash, shellcheck's closest to BusyBox's ash); PKGBUILDs
are left to makepkg.
"""

from __future__ import annotations

import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent


def shell(data: bytes) -> str | None:
    """Return the shell of a script by its first line, or None for anything else."""
    first = data.split(b"\n", 1)[0]
    if not first.startswith(b"#!"):
        return None
    if b"bash" in first:
        return "bash"
    if first.rstrip().endswith((b"/sh", b" sh")):
        return "dash"
    return None


def main() -> int:
    """Check the scripts; fail when one has a warning or an error."""
    names = subprocess.run(["git", "-C", str(ROOT), "ls-files", "-z"], check=True, capture_output=True).stdout  # noqa: S603, S607  # git from PATH, fixed arguments
    code = count = 0
    for name in names.decode().split("\0"):
        path = ROOT / name
        if not name or not path.is_file() or path.suffix not in ("", ".sh"):
            continue
        kind = shell(path.read_bytes()[:200])
        if kind is None:
            continue
        count += 1
        code |= subprocess.call(["shellcheck", "--severity=warning", "--shell", kind, str(path)])  # noqa: S603, S607  # shellcheck from PATH on a tracked file
    print(f"{count} scripts checked")
    return 1 if code else 0


if __name__ == "__main__":
    sys.exit(main())
