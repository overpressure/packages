# SPDX-License-Identifier: Unlicense
"""build/versions.py: pacman's version order, a recipe's version, the release's name and the stale recipes."""

from __future__ import annotations

import datetime as dt
import importlib.util
import subprocess
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
spec = importlib.util.spec_from_file_location("versions", ROOT / "build" / "versions.py")
assert spec is not None
assert spec.loader is not None
versions = importlib.util.module_from_spec(spec)
spec.loader.exec_module(versions)


class VersionsTests(unittest.TestCase):
    def test_versions_compare_as_pacman_compares_them(self):
        for older, newer in (
            ("1", "2026.11.22"),
            ("1.0", "1.0.1"),
            ("1.0a", "1.0"),
            ("1.0rc1", "1.0"),
            ("2.24.33", "2.24.34"),
            ("25.1.0", "25.1.0.1"),
            ("9", "10"),
        ):
            with self.subTest(older=older, newer=newer):
                self.assertEqual(versions.rpmvercmp(older, newer), -1)
                self.assertEqual(versions.rpmvercmp(newer, older), 1)
        self.assertEqual(versions.rpmvercmp("2.0.1", "2.0.1"), 0)
        self.assertEqual(versions.vercmp(("", "1", "4"), ("", "1", "3")), 1)
        self.assertEqual(versions.vercmp(("1", "0.1", "1"), ("", "9", "9")), 1)  # the epoch first

    def test_a_recipes_version_is_its_top_level_assignments(self):
        pkgbuild = "pkgname=x\npkgver=2026.11.22 # the change's date\npkgrel='2'\nepoch=1\nbuild() { pkgver=9; }\n"
        self.assertEqual(versions.recipe_version(pkgbuild), ("1", "2026.11.22", "2"))
        self.assertEqual(versions.spelled(("1", "2026.11.22", "2")), "1:2026.11.22-2")
        with self.assertRaises(ValueError):
            versions.recipe_version("pkgname=x\n")

    def test_the_next_release_is_this_months_next_free_patch(self):
        tags = ["2026.11.12", "2026.11.19", "2026.11.19.1", "phoenix/7.2-6", "2026.10.4"]
        self.assertEqual(versions.next_release(tags, dt.date(2026, 11, 22)), "2026.11.20")
        self.assertEqual(versions.next_release(tags, dt.date(2026, 12, 1)), "2026.12.0")
        self.assertEqual(versions.newest_release([*tags, "2026.11.20"]), "2026.11.20")
        self.assertEqual(versions.newest_release(["2026.11.19", "2026.11.19.1"]), "2026.11.19.1")
        self.assertEqual(versions.newest_release(["main"]), "")

    def test_a_changed_recipe_without_a_higher_version_is_stale(self):
        with tempfile.TemporaryDirectory() as tmp:
            repo = Path(tmp)

            def git(*words: str) -> None:
                subprocess.run(["git", "-C", tmp, *words], check=True, capture_output=True)  # noqa: S603, S607  # git

            def recipe(name: str, pkgver: str, text: str = "") -> None:
                folder = repo / "phoenix" / name
                folder.mkdir(parents=True, exist_ok=True)
                (folder / "PKGBUILD").write_text(f"pkgname={name}\npkgver={pkgver}\npkgrel=1\n", encoding="utf-8")
                (folder / "file").write_text(text, encoding="utf-8")

            git("init", "-q")
            git("config", "user.email", "test@example.invalid")
            git("config", "user.name", "test")
            recipe("pico4-raised", "1")
            recipe("pico4-stale", "1")
            recipe("pico4-same", "1")
            git("add", "-A")
            git("commit", "-q", "-m", "first")
            git("tag", "2026.11.19")
            recipe("pico4-raised", "2026.11.22", "changed")
            recipe("pico4-stale", "1", "changed")
            recipe("pico4-new", "1")
            changed = versions.changed_recipes("2026.11.19", repo)
            self.assertEqual(sorted(changed), ["phoenix/pico4-raised", "phoenix/pico4-stale"])
            self.assertEqual(
                versions.stale(changed), ["phoenix/pico4-stale: 1-1 -> 1-1, its version must rise with its contents"]
            )


if __name__ == "__main__":
    unittest.main()
