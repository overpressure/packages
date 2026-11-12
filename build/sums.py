#!/usr/bin/env python3
# SPDX-License-Identifier: Unlicense
"""Write a recipe's sha256sums: each local source's SHA-256, the others' entries as they are.

    python3 build/sums.py LAYER/RECIPE...

makepkg's updpkgsums does the same where makepkg runs; this needs only bash and Python. A downloaded
source keeps its entry (a pinned hash, or SKIP for a git source pinned by commit).
"""

from __future__ import annotations

import hashlib
import re
import shutil
import subprocess
import sys
from pathlib import Path

READ = 'source ./PKGBUILD; for v in source sha256sums; do eval "printf \'%s\\0\' \\"\\${$v[@]}\\""; echo; done'


def bash() -> str:
    """Return a bash that sources PKGBUILDs (not WSL's launcher, which runs another system)."""
    found = shutil.which("bash")
    if not found or "system32" in found.lower():
        message = "no bash (on Windows: Git Bash)"
        raise SystemExit(message)
    return found


def arrays(folder: Path) -> tuple[list[str], list[str]]:
    """Return the recipe's source and sha256sums, as bash reads them."""
    run = subprocess.run([bash(), "-c", READ], cwd=folder, capture_output=True, text=True, check=True)  # noqa: S603
    source, sums = (part.strip("\n").split("\0")[:-1] for part in run.stdout.split("\n")[:2])
    return source, sums


def local(source: str) -> bool:
    """Tell whether a source entry is a file beside the recipe."""
    return "::" not in source and "://" not in source


def write(folder: Path) -> None:
    """Rewrite FOLDER's PKGBUILD with its local sources' hashes, one entry per line."""
    source, sums = arrays(folder)
    if len(sums) != len(source):
        sums = ["SKIP"] * len(source)
    new = [
        hashlib.sha256((folder / s).read_bytes()).hexdigest() if local(s) else old
        for s, old in zip(source, sums, strict=True)
    ]
    recipe = folder / "PKGBUILD"
    text = recipe.read_text(encoding="utf-8")
    block = re.compile(r"^sha256sums=\((?:[^()]|\n)*?\)\n", re.MULTILINE)
    if len(block.findall(text)) != 1:
        message = f"{recipe}: not one sha256sums=(...) array"
        raise SystemExit(message)
    body = "sha256sums=(" + "\n            ".join(f"'{s}'" if s != "SKIP" else s for s in new) + ")\n"
    recipe.write_text(block.sub(lambda _: body, text), encoding="utf-8", newline="\n")


def main() -> None:
    """Rewrite each named recipe."""
    if len(sys.argv) < 2:
        raise SystemExit(__doc__)
    for name in sys.argv[1:]:
        write(Path(name))


if __name__ == "__main__":
    main()
