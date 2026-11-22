#!/usr/bin/env python3
# SPDX-License-Identifier: Unlicense
"""The release's name, and every changed recipe's version raised (overpressure's docs/releasing.md, versions).

    python3 build/versions.py next-release          the next release's name: YYYY.M.PATCH, this month's next patch
    python3 build/versions.py check [--against TAG] every recipe changed since TAG (the newest release) raises its
                                                    version; fails naming those that do not

A release is named as overpressure's: the year, the month without a leading zero and that month's next free
patch (UTC), so it sorts after every release before it, the day-named ones of November 2026 (2026.11.19,
2026.11.19.1) too. A recipe's version is its epoch, pkgver and pkgrel, compared as pacman's vercmp compares them:
a change to a recipe's folder raises pkgrel, or pkgver for a new upstream version; the project's own packages
without an upstream take the date of their change as pkgver (2026.11.22).
"""

from __future__ import annotations

import argparse
import datetime as dt
import re
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
#: The layers whose folders hold one recipe each (LAYER/NAME/PKGBUILD).
LAYERS = ("common", "sm8250", "phoenix")
_RELEASE = re.compile(r"^(\d{4})\.(\d+)\.(\d+)(?:\.(\d+))?$")
_FIELD = re.compile(r"^(epoch|pkgver|pkgrel)=['\"]?([^'\"\s#]*)['\"]?\s*(?:#.*)?$", re.MULTILINE)


def _segments(text: str) -> list[str]:
    return re.findall(r"\d+|[a-zA-Z]+", text)


def rpmvercmp(a: str, b: str) -> int:
    """Compare two version strings as pacman's rpmvercmp does: -1, 0 or 1."""
    if a == b:
        return 0
    left, right = _segments(a), _segments(b)
    for x, y in zip(left, right):
        if x.isdigit() != y.isdigit():
            return 1 if x.isdigit() else -1  # a number is newer than letters
        if x.isdigit():
            if int(x) != int(y):
                return 1 if int(x) > int(y) else -1
        elif x != y:
            return 1 if x > y else -1
    if len(left) == len(right):
        return 0
    longer, sign = (left, 1) if len(left) > len(right) else (right, -1)
    following = longer[min(len(left), len(right))]
    # More segments: newer, unless what follows is letters (1.0a is older than 1.0).
    return -sign if following.isalpha() else sign


def vercmp(a: tuple[str, str, str], b: tuple[str, str, str]) -> int:
    """Compare two (epoch, pkgver, pkgrel) as pacman's vercmp: the epoch first, then pkgver, then pkgrel."""
    epoch = (int(a[0] or 0) > int(b[0] or 0)) - (int(a[0] or 0) < int(b[0] or 0))
    return epoch or rpmvercmp(a[1], b[1]) or rpmvercmp(a[2], b[2])


def recipe_version(pkgbuild: str) -> tuple[str, str, str]:
    """Return a PKGBUILD's (epoch, pkgver, pkgrel), its plain top-level assignments; epoch "" when it has none."""
    fields = dict(_FIELD.findall(pkgbuild))
    if not fields.get("pkgver") or not fields.get("pkgrel"):
        message = "no pkgver or pkgrel assigned at the top level"
        raise ValueError(message)
    return fields.get("epoch", ""), fields["pkgver"], fields["pkgrel"]


def spelled(version: tuple[str, str, str]) -> str:
    """Return (epoch, pkgver, pkgrel) as pacman writes it: [epoch:]pkgver-pkgrel."""
    epoch, pkgver, pkgrel = version
    return f"{epoch + ':' if epoch else ''}{pkgver}-{pkgrel}"


def release_key(tag: str) -> tuple[int, int, int, int] | None:
    """Return a release tag's sorting key (2026.11.19.1 after 2026.11.19, before 2026.11.20), None for another tag."""
    match = _RELEASE.match(tag)
    return None if match is None else tuple(int(part or 0) for part in match.groups())  # ty: ignore[invalid-return-type]  # four parts


def newest_release(tags: list[str]) -> str:
    """Return the newest release among @p tags, "" for none."""
    releases = [t for t in tags if release_key(t) is not None]
    return max(releases, key=lambda t: release_key(t) or (0, 0, 0, 0), default="")


def next_release(tags: list[str], today: dt.date) -> str:
    """Return the next release's name: @p today's year and month, and the month's first patch no tag has used."""
    used = [key[2] for key in map(release_key, tags) if key is not None and key[:2] == (today.year, today.month)]
    return f"{today.year}.{today.month}.{max(used) + 1 if used else 0}"


def stale(changed: dict[str, tuple[tuple[str, str, str], tuple[str, str, str]]]) -> list[str]:
    """Return the changed recipes whose version did not rise, each with its versions; @p changed: (before, now)."""
    return [
        f"{recipe}: {spelled(before)} -> {spelled(now)}, its version must rise with its contents"
        for recipe, (before, now) in sorted(changed.items())
        if vercmp(now, before) <= 0
    ]


def git(*words: str, root: Path = ROOT) -> str:
    """Run git in @p root; return its output."""
    return subprocess.run(["git", "-C", str(root), *words], check=True, capture_output=True, text=True).stdout  # noqa: S603, S607  # git, fixed words


def changed_recipes(against: str, root: Path = ROOT) -> dict[str, tuple[tuple[str, str, str], tuple[str, str, str]]]:
    """Return every recipe whose folder changed since @p against and existed there, with both its versions."""
    found = {}
    for pkgbuild in sorted(p for layer in LAYERS for p in (root / layer).glob("*/PKGBUILD")):
        folder = pkgbuild.parent.relative_to(root).as_posix()
        diff = subprocess.run(  # noqa: S603  # git, fixed words
            ["git", "-C", str(root), "diff", "--quiet", against, "--", folder],  # noqa: S607  # git from PATH
            check=False,
        )
        if diff.returncode == 0:
            continue
        try:
            before = recipe_version(git("show", f"{against}:{folder}/PKGBUILD", root=root))
        except subprocess.CalledProcessError:
            continue  # a new recipe: no version before it
        found[folder] = (before, recipe_version(pkgbuild.read_text(encoding="utf-8")))
    return found


def main(argv: list[str] | None = None) -> int:
    """Run the command named on the command line."""
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest="command", required=True)
    naming = sub.add_parser("next-release", help="the next release's name")
    naming.add_argument("--today", type=dt.date.fromisoformat, help="the day (UTC), for tests; today")
    checking = sub.add_parser("check", help="every recipe changed since the newest release raises its version")
    checking.add_argument("--against", help="the release to compare with; the newest one")
    args = parser.parse_args(argv)
    tags = git("tag", "-l").split()
    if args.command == "next-release":
        print(next_release(tags, args.today or dt.datetime.now(dt.timezone.utc).date()))
        return 0
    against = args.against or newest_release(tags)
    if not against:
        print("no release yet: nothing to compare with")
        return 0
    changed = changed_recipes(against)
    problems = stale(changed)
    for line in problems:
        print(line, file=sys.stderr)
    print(f"{len(changed)} recipe(s) changed since {against}, {len(problems)} without a higher version")
    return 1 if problems else 0


if __name__ == "__main__":
    sys.exit(main())
