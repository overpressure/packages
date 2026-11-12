#!/usr/bin/env python3
# SPDX-License-Identifier: Unlicense
"""The packages CI builds, one job each, with the key their cached build is kept under.

    python3 build/plan.py --builder IMAGE HEADSET...

prints a JSON list of {"headset", "layer", "name", "key"}. The key is a hash of everything a package's
bytes depend on: its recipe folder, build/packages.sh, the build image and the headset's definition
(its layers, repositories, packages and pins). Recipes marked "# overpressure: local" are left out.
"""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
DEFINITION = ("layers", "pacman.conf", "packages.txt", "packages.lock")
LOCAL = "# overpressure: local"


def layers(headset: str) -> list[str]:
    """Return a headset's layers, the most general first."""
    text = (ROOT / "headsets" / headset / "layers").read_text(encoding="utf-8")
    return [s for s in (line.split("#", 1)[0].strip() for line in text.splitlines()) if s]


def digest(files: list[tuple[str, Path]], extra: str) -> str:
    """Hash FILES under their names, and EXTRA."""
    h = hashlib.sha256()
    for name, path in files:
        h.update(f"{name}\0{hashlib.sha256(path.read_bytes()).hexdigest()}\n".encode())
    h.update(extra.encode())
    return h.hexdigest()


def plan(headsets: list[str], builder: str) -> list[dict[str, str]]:
    """Return a job for each package of HEADSETS' layers."""
    jobs = []
    for headset in headsets:
        shared = [("build/packages.sh", ROOT / "build/packages.sh")]
        shared += [(f"headset/{n}", ROOT / "headsets" / headset / n) for n in DEFINITION]
        for layer in layers(headset):
            for recipe in sorted((ROOT / layer).glob("*/PKGBUILD")):
                folder = recipe.parent
                if LOCAL in recipe.read_text(encoding="utf-8"):
                    continue
                own = [
                    (f"recipe/{p.relative_to(folder).as_posix()}", p)
                    for p in sorted(folder.rglob("*"))
                    if p.is_file() and not any(part.startswith(".") for part in p.relative_to(folder).parts)
                ]
                key = digest(shared + own, f"builder {builder}\n")
                jobs.append({"headset": headset, "layer": layer, "name": folder.name, "key": key[:32]})
    return jobs


def main() -> None:
    """Print the plan."""
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--builder", required=True, help="the build image, by tag and digest")
    parser.add_argument("headsets", nargs="+")
    args = parser.parse_args()
    print(json.dumps(plan(args.headsets, args.builder), separators=(",", ":")))


if __name__ == "__main__":
    main()
