# SPDX-License-Identifier: Unlicense
"""build/notes.py: a release's notes say what changed since the previous release."""

from __future__ import annotations

import importlib.util
import shutil
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
SPEC = importlib.util.spec_from_file_location("notes", ROOT / "build/notes.py")
assert SPEC
assert SPEC.loader
notes = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(notes)

OLD = """\
aa  overpressure-common.db
bb  overpressure-gtk2-2.24.33-1-aarch64.pkg.tar.zst
cc  overpressure-gtk2-2.24.33-1.src.tar.gz
dd  pico4-base-1-3-any.pkg.tar.zst
ee  pico4-old-1-1-any.pkg.tar.zst
"""
NEW = """\
ab  overpressure-common.db
bb  overpressure-gtk2-2.24.33-1-aarch64.pkg.tar.zst
cc  overpressure-gtk2-2.24.33-1.src.tar.gz
df  pico4-base-1-4-any.pkg.tar.zst
ff  pico4-slamd-1-1-aarch64.pkg.tar.zst
"""


class Packages(unittest.TestCase):
    def test_names_and_versions_of_the_packages_only(self) -> None:
        self.assertEqual(
            notes.packages(OLD),
            {"overpressure-gtk2": "2.24.33-1", "pico4-base": "1-3", "pico4-old": "1-1"},
        )

    def test_added_removed_and_changed(self) -> None:
        self.assertEqual(
            notes.changes(notes.packages(OLD), notes.packages(NEW)),
            ["- changed: pico4-base 1-3 -> 1-4", "- removed: pico4-old 1-1", "- added: pico4-slamd 1-1"],
        )


class Notes(unittest.TestCase):
    def test_the_first_release_has_no_changes(self) -> None:
        text = notes.notes(
            "The first.", commits=None, package_changes=None, commit="c0ffee", server="https://example/dl", sums=NEW
        )
        self.assertTrue(text.startswith("The first.\n\nCommit: `c0ffee`\n\n"))
        self.assertNotIn("## Changes", text)
        self.assertNotIn("## Packages", text)
        self.assertIn("`Server = https://example/dl`", text)
        self.assertIn("```\n" + NEW.strip() + "\n```\n", text)

    def test_order_of_the_parts(self) -> None:
        text = notes.notes(
            "Tag message",
            commits="- fix: a\n",
            package_changes=["- added: b 1-1"],
            commit="c0ffee",
            server="https://example/dl",
            sums=NEW,
        )
        parts = [
            "Tag message",
            "## Changes\n\n- fix: a",
            "## Packages\n\n- added: b 1-1",
            "Commit: `c0ffee`",
            "Server =",
        ]
        positions = [text.index(part) for part in parts]
        self.assertEqual(positions, sorted(positions))


@unittest.skipUnless(shutil.which("git"), "needs git")
class FromTags(unittest.TestCase):
    def git(self, *args: str) -> None:
        identity = ["-c", "user.name=t", "-c", "user.email=t@example.invalid", "-c", "tag.gpgSign=false"]
        subprocess.run(["git", *identity, *args], cwd=self.repo, check=True, capture_output=True)  # noqa: S603, S607  # git from PATH

    def setUp(self) -> None:
        self.repo = Path(tempfile.mkdtemp())
        self.addCleanup(shutil.rmtree, self.repo, ignore_errors=True)
        self.git("init", "-q")
        for subject in ("feat: one", "fix: two", "docs: three"):
            (self.repo / "f").write_text(subject, encoding="utf-8")
            self.git("add", "f")
            self.git("commit", "-q", "--no-gpg-sign", "-m", subject)
            if subject == "feat: one":
                self.git("tag", "-a", "2026.01.01", "-m", "First")
        self.git("tag", "-a", "2026.01.02", "-m", "Second\n\nIts body.")
        (self.repo / "old").write_text(OLD, encoding="utf-8")
        (self.repo / "new").write_text(NEW, encoding="utf-8")

    def notes(self, *args: str) -> str:
        script = str(ROOT / "build/notes.py")
        run = subprocess.run(  # noqa: S603  # the script under test, no shell
            [sys.executable, script, "--repository", "o/p", *args],
            cwd=self.repo,
            capture_output=True,
            text=True,
            check=True,
        )
        return run.stdout

    def test_commits_and_packages_since_the_previous_release(self) -> None:
        text = self.notes("--tag", "2026.01.02", "--previous", "2026.01.01", "--previous-sums", "old", "new")
        self.assertTrue(text.startswith("Second\n\nIts body.\n\n## Changes\n\n- docs: three\n- fix: two\n\n"))
        self.assertIn("- changed: pico4-base 1-3 -> 1-4", text)
        self.assertIn("Server = https://github.com/o/p/releases/download/2026.01.02", text)

    def test_the_first_release(self) -> None:
        text = self.notes("--tag", "2026.01.01", "new")
        self.assertTrue(text.startswith("First\n\nCommit: `"))
