# SPDX-License-Identifier: Unlicense
"""The first-run setup's helpers (docs/session.md#first-run-setup)."""

from __future__ import annotations

import importlib.machinery
import importlib.util
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path
from unittest import mock

HERE = Path(__file__).resolve().parent
LANGUAGE = HERE / "steam-language"
TIMEZONE = HERE / "steamos-set-timezone"

_loader = importlib.machinery.SourceFileLoader("steam_language", str(LANGUAGE))
_spec = importlib.util.spec_from_loader("steam_language", _loader)
assert _spec is not None
tool = importlib.util.module_from_spec(_spec)
sys.modules[_spec.name] = tool
_loader.exec_module(tool)

#: A registry as the client writes it at its first start (made-up values).
REGISTRY = """"Registry"
{
\t"HKLM"
\t{
\t\t"Software"
\t\t{
\t\t\t"Valve"
\t\t\t{
\t\t\t\t"Steam"
\t\t\t\t{
\t\t\t\t\t"SteamPID"\t\t"1234"
\t\t\t\t}
\t\t\t}
\t\t}
\t}
\t"HKCU"
\t{
\t\t"Software"
\t\t{
\t\t\t"Valve"
\t\t\t{
\t\t\t\t"Steamsteamglobal"
\t\t\t\t{
\t\t\t\t\t"language"\t\t"english"
\t\t\t\t}
\t\t\t\t"Steam"
\t\t\t\t{
\t\t\t\t\t"language"\t\t"english"
\t\t\t\t\t"AutoLoginUser"\t\t""
\t\t\t\t\t"path"\t\t"C:\\\\a \\"b\\""
\t\t\t\t}
\t\t\t}
\t\t}
\t}
}
"""


class LanguageTests(unittest.TestCase):
    def test_both_places_change_and_nothing_else(self):
        new = tool.set_language(REGISTRY, "german")
        self.assertEqual(new.count('"language"\t\t"german"'), 2)
        self.assertNotIn('"english"', new)
        # Everything else as it was, in its order, quotes and backslashes included.
        self.assertEqual(new.replace('"german"', '"english"'), REGISTRY)

    def test_missing_places_are_made(self):
        new = tool.set_language('"Registry"\n{\n\t"HKCU"\n\t{\n\t}\n}\n', "koreana")
        root = tool.parse(new)
        for path in tool.PATHS:
            node = tool.child(root, "Registry")
            for key in path:
                node = tool.child(node, key)
            self.assertIn(("language", "koreana"), node)
        self.assertEqual(tool.parse(tool.set_language("", "thai"))[0][0], "Registry")

    def test_keys_ignore_case(self):
        new = tool.set_language(REGISTRY.replace('"Steam"\n', '"steam"\n'), "french")
        self.assertEqual(new.count('"language"\t\t"french"'), 2)

    def test_a_broken_registry_is_not_written(self):
        for text in ('"Registry"\n{\n', '"Registry"\n}\n', '"Registry"\n{\n"key"\n}\n'):
            with self.assertRaises(ValueError):
                tool.set_language(text, "german")

    def test_the_client_restarts_around_the_change(self):
        calls: list[tuple[str, ...]] = []
        with tempfile.TemporaryDirectory() as temp:
            registry = Path(temp) / "registry.vdf"
            registry.write_text(REGISTRY, encoding="utf-8")
            with (
                mock.patch.object(tool, "systemctl", side_effect=lambda *a: calls.append(a)),
                mock.patch.object(tool, "REGISTRY", registry),
                mock.patch.object(tool, "client_running", return_value=True),
            ):
                self.assertEqual(tool.main(["steam-language", "german"]), 0)
            self.assertIn('"language"\t\t"german"', registry.read_text(encoding="utf-8"))
        self.assertEqual(calls, [("stop", "steam.service"), ("start", "steam.service")])

    def test_a_client_still_to_start_only_gets_the_file(self):
        # During the first-run setup the client waits to start: it starts in the language.
        with tempfile.TemporaryDirectory() as temp:
            registry = Path(temp) / ".steam/registry.vdf"
            with (
                mock.patch.object(tool, "systemctl") as systemctl,
                mock.patch.object(tool, "REGISTRY", registry),
                mock.patch.object(tool, "client_running", return_value=False),
            ):
                self.assertEqual(tool.main(["steam-language", "japanese"]), 0)
            self.assertIn('"language"\t\t"japanese"', registry.read_text(encoding="utf-8"))
        systemctl.assert_not_called()

    def test_only_the_clients_languages(self):
        with mock.patch.object(tool, "systemctl") as systemctl:
            self.assertEqual(tool.main(["steam-language", "klingon"]), 2)
            self.assertEqual(tool.main(["steam-language"]), 2)
        systemctl.assert_not_called()


class WaitTests(unittest.TestCase):
    SCRIPT = HERE / "wait-first-run"

    def run_script(self, home: str, *args: str) -> int:
        return subprocess.run(  # noqa: S603  # bash from PATH runs the script under test
            ["bash", str(self.SCRIPT), *args],  # noqa: S607  # bash from PATH
            env={"HOME": home, "PATH": "/usr/bin:/bin"},
            capture_output=True,
            timeout=10,
            check=False,
        ).returncode

    def test_the_mark_and_a_sign_in_release_it(self):
        with tempfile.TemporaryDirectory() as home:
            mark = Path(home) / ".local/share/pico4/setup-done"
            mark.parent.mkdir(parents=True)
            mark.write_text("1\n", encoding="utf-8")
            self.assertEqual(self.run_script(home), 0)
            mark.unlink()
            registry = Path(home) / ".steam/registry.vdf"
            registry.parent.mkdir()
            registry.write_text(REGISTRY.replace('"AutoLoginUser"\t\t""', '"CompletedOOBE"\t\t"1"'), encoding="utf-8")
            self.assertEqual(self.run_script(home), 0)

    def test_the_checks_ask_whether_it_is_pending(self):
        with tempfile.TemporaryDirectory() as home:
            self.assertEqual(self.run_script(home, "--pending"), 0)
            mark = Path(home) / ".local/share/pico4/setup-done"
            mark.parent.mkdir(parents=True)
            mark.write_text("1\n", encoding="utf-8")
            self.assertEqual(self.run_script(home, "--pending"), 1)
            mark.unlink()
            registry = Path(home) / ".steam/registry.vdf"
            registry.parent.mkdir()
            registry.write_text(REGISTRY, encoding="utf-8")
            self.assertEqual(self.run_script(home, "--pending"), 0)
            registry.write_text(REGISTRY.replace('"AutoLoginUser"\t\t""', '"CompletedOOBE"\t\t"1"'), encoding="utf-8")
            self.assertEqual(self.run_script(home, "--pending"), 1)
            self.assertEqual(self.run_script(home, "--other"), 2)


class TimezoneTests(unittest.TestCase):
    def test_the_helper_takes_only_zone_names(self):
        for zone in ("", "/etc/passwd", "../../etc/passwd", "Europe/Paris;reboot", "Mars/Olympus"):
            # The helper under test, with made-up zones.
            result = subprocess.run(  # noqa: S603  # bash from PATH runs the helper
                ["bash", str(TIMEZONE), zone],  # noqa: S607  # bash from PATH
                capture_output=True,
                text=True,
                check=False,
            )
            self.assertEqual(result.returncode, 2, zone)


if __name__ == "__main__":
    unittest.main()
