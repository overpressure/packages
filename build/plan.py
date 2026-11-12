#!/usr/bin/env python3
# SPDX-License-Identifier: Unlicense
"""The packages CI builds, one job each, with the key their cached build is kept under.

    python3 build/plan.py --builder IMAGE HEADSET...

prints a JSON list of {"headset", "layer", "name", "key"}. The key is a hash of everything a package's
bytes depend on: its recipe folder, build/packages.sh, the build image and what the build environment
takes from the headset's definition: its repositories (pacman.conf), its pins (packages.lock) and the
packages packages.txt takes from the Frame's repository, which the environment installs. The rest of
packages.txt and the layers do not change a package. Recipes marked "# overpressure: local" are left out.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import re
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
DEFINITION = ("pacman.conf", "packages.lock")
LOCAL = "# overpressure: local"


def layers(headset: str) -> list[str]:
    """Return a headset's layers, the most general first."""
    text = (ROOT / "headsets" / headset / "layers").read_text(encoding="utf-8")
    return [s for s in (line.split("#", 1)[0].strip() for line in text.splitlines()) if s]


def frame_packages(headset: str) -> list[str]:
    """Return the packages a headset's packages.txt takes from the Frame's repository (packages.sh's FRAME_REPO)."""
    script = (ROOT / "build/packages.sh").read_text(encoding="utf-8")
    found = re.search(r"^FRAME_REPO=(\S+)$", script, re.MULTILINE)
    if found is None:
        message = "build/packages.sh names no FRAME_REPO"
        raise ValueError(message)
    text = (ROOT / "headsets" / headset / "packages.txt").read_text(encoding="utf-8")
    prefix = found.group(1) + "/"
    names = (line.split("#", 1)[0].strip() for line in text.splitlines())
    return sorted(n.removeprefix(prefix) for n in names if n.startswith(prefix))


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
        frame = "".join(f"frame {name}\n" for name in frame_packages(headset))
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
                key = digest(shared + own, f"builder {builder}\n{frame}")
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
