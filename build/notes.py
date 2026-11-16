#!/usr/bin/env python3
# SPDX-License-Identifier: Unlicense
"""A release's notes, from its annotated tag and its SHA256SUMS.

    python3 build/notes.py --tag TAG --repository OWNER/NAME [--previous TAG --previous-sums FILE] SHA256SUMS

prints the notes: the tag's message; with a previous release, the commits since it and the packages
added, removed and changed against its SHA256SUMS; the commit; the pacman server and the checksums.
"""

from __future__ import annotations

import argparse
import subprocess
from pathlib import Path

SUFFIX = ".pkg.tar.zst"


def git(*args: str) -> str:
    """Return a git command's output."""
    return subprocess.run(["git", *args], capture_output=True, text=True, check=True).stdout  # noqa: S603, S607  # git from PATH, no shell


def packages(sums: str) -> dict[str, str]:
    """Return {name: version-release} of the packages a SHA256SUMS lists."""
    found = {}
    for line in sums.splitlines():
        file = line.split("  ", 1)[-1].strip()
        if file.endswith(SUFFIX):
            name, version, release, _arch = file.removesuffix(SUFFIX).rsplit("-", 3)
            found[name] = f"{version}-{release}"
    return found


def changes(old: dict[str, str], new: dict[str, str]) -> list[str]:
    """Return a line for each package added, removed or at another version, by name."""
    lines = []
    for name in sorted(old.keys() | new.keys()):
        if name not in old:
            lines.append(f"- added: {name} {new[name]}")
        elif name not in new:
            lines.append(f"- removed: {name} {old[name]}")
        elif old[name] != new[name]:
            lines.append(f"- changed: {name} {old[name]} -> {new[name]}")
    return lines


def notes(  # noqa: PLR0913  # one argument for each part of the notes
    message: str,
    *,
    commits: str | None,
    package_changes: list[str] | None,
    commit: str,
    server: str,
    sums: str,
) -> str:
    """Return the notes; COMMITS and PACKAGE_CHANGES are None for the first release."""
    parts = [message.strip()]
    if commits is not None:
        parts.append("## Changes\n\n" + (commits.strip() or "- none"))
    if package_changes is not None:
        parts.append("## Packages\n\n" + ("\n".join(package_changes) or "- none changed"))
    parts.append(f"Commit: `{commit}`")
    parts.append(
        "Packages and databases of the layers, and the copyleft packages' sources.\n"
        f"A pacman server for every layer: `Server = {server}`.\n\n```\n{sums.strip()}\n```"
    )
    return "\n\n".join(parts) + "\n"


def main() -> None:
    """Print the notes of a release."""
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--tag", required=True, help="the release's annotated tag")
    parser.add_argument("--repository", required=True, help="OWNER/NAME on GitHub")
    parser.add_argument("--previous", help="the previous release's tag; none for the first release")
    parser.add_argument("--previous-sums", type=Path, help="the previous release's SHA256SUMS")
    parser.add_argument("sums", type=Path, help="this release's SHA256SUMS")
    args = parser.parse_args()
    if bool(args.previous) != bool(args.previous_sums):
        parser.error("--previous and --previous-sums go together")
    tag = args.tag
    message = git("tag", "--list", "--format=%(contents:subject)%0a%0a%(contents:body)", tag)
    sums = args.sums.read_text(encoding="utf-8")
    commits = package_changes = None
    if args.previous:
        commits = git("log", "--no-merges", "--format=- %s", f"{args.previous}..{tag}")
        package_changes = changes(packages(args.previous_sums.read_text(encoding="utf-8")), packages(sums))
    commit = git("rev-parse", f"{tag}^{{commit}}").strip()
    server = f"https://github.com/{args.repository}/releases/download/{tag}"
    text = notes(message, commits=commits, package_changes=package_changes, commit=commit, server=server, sums=sums)
    print(text, end="")


if __name__ == "__main__":
    main()
