# SPDX-License-Identifier: Unlicense
"""The support bundle, with fake output and fake identifiers."""

from __future__ import annotations

import importlib.machinery
import importlib.util
import tarfile
import tempfile
import unittest
from pathlib import Path
from typing import TYPE_CHECKING
from unittest.mock import patch

if TYPE_CHECKING:
    from collections.abc import Sequence

HERE = Path(__file__).resolve().parent
_loader = importlib.machinery.SourceFileLoader("pico4_support_bundle", str(HERE / "pico4-support-bundle"))
_spec = importlib.util.spec_from_loader(_loader.name, _loader)
assert _spec is not None
support = importlib.util.module_from_spec(_spec)
_loader.exec_module(support)


def member_text(archive: tarfile.TarFile, member: tarfile.TarInfo) -> str:
    """Return a regular member's text; extractfile gives None only for other kinds."""
    f = archive.extractfile(member)
    assert f is not None
    return f.read().decode()


class SupportBundleTests(unittest.TestCase):
    SECRETS = ("FAKESERIAL123", "MyHomeNetwork", "fake_account", "Fake Persona", "fake-machine-id-0123")
    TEXT = (
        "serial FAKESERIAL123 joined MyHomeNetwork from 192.168.178.23 and fd00:1234:5678:9abc::1 "
        "fe80:1:2:3:4:5:6:7 mac aa:bb:cc:dd:ee:ff\n"
        "steam 76561198000000001 [U:1:39734273] fake_account Fake Persona fake-machine-id-0123\n"
        "kept: 127.0.0.1 172.16.42.1 0.0.0.0 version 300.12.5.999 pico4-steamos\n"
    )

    def bundle(self, globs: Sequence[str] = ()) -> tuple[list[str], dict[str, str]]:
        with tempfile.TemporaryDirectory() as folder:
            output = Path(folder) / "bundle.tar.gz"
            with patch.object(support, "run", lambda *_: self.TEXT):
                names = support.bundle(output, commands={"one.txt": "fake"}, globs=globs, names=list(self.SECRETS))
            with tarfile.open(output) as archive:
                return names, {m.name: member_text(archive, m) for m in archive.getmembers()}

    def setUp(self):
        folder = tempfile.TemporaryDirectory()
        self.addCleanup(folder.cleanup)
        self.folder = Path(folder.name)

    def test_identifiers_are_masked(self):
        _, members = self.bundle()
        text = members["support/one.txt"]
        for secret in (
            *self.SECRETS,
            "192.168.178.23",
            "aa:bb:cc:dd:ee:ff",
            "76561198000000001",
            "[U:1:39734273]",
            "fe80:1:2:3:4:5:6:7",
        ):
            self.assertNotIn(secret, text)
        for kept in ("127.0.0.1", "172.16.42.1", "0.0.0.0", "300.12.5.999", "pico4-steamos"):  # noqa: S104  # text, nothing binds
            self.assertIn(kept, text)

    def test_log_files_are_masked_and_cut_to_their_newest_part(self):
        log = self.folder / "big.log"
        log.write_bytes(b"old aa:bb:cc:dd:ee:ff\n" * 1000 + b"new FAKESERIAL123\n")
        with tempfile.TemporaryDirectory() as out:
            output = Path(out) / "b.tar.gz"
            support.bundle(output, commands={}, globs=[str(log)], names=list(self.SECRETS), limit=40)
            with tarfile.open(output) as archive:
                (member,) = archive.getmembers()
                text = member_text(archive, member)
        self.assertNotIn("FAKESERIAL123", text)
        self.assertTrue(text.endswith("<name>\n"))

    def test_a_missing_part_stays_out_without_failing(self):
        names, _ = self.bundle(globs=[str(self.folder / "absent/*")])
        self.assertEqual(names, ["one.txt"])

    def test_nothing_with_tokens_is_archived(self):
        for pattern in support.LOG_GLOBS:
            for forbidden in ("config.vdf", "loginusers", "system-connections", ".ssh", "shadow"):
                self.assertNotIn(forbidden, pattern)
        text = str(support.COMMANDS)
        for forbidden in ("config.vdf", "loginusers", "system-connections", ".ssh", "shadow"):
            self.assertNotIn(forbidden, text)

    def test_the_soc_serial_is_masked_in_hex_too(self):
        # FastRPC's test signature is named after it: testsig-0x<serial in hex>.so.
        serials = {"/sys/devices/soc0/serial_number": "3735928559"}
        with (
            patch.object(support, "read", lambda path: serials.get(str(path), "")),
            patch.object(support, "run", lambda *_: ""),
        ):
            names = support.identifiers("console=x", steam=self.folder)
        self.assertIn("deadbeef", names)
        mask = support.masker(names)
        self.assertEqual(
            mask("for testsig-0xDEADBEEF.so and testsig-0xdeadbeef.so"),
            "for testsig-0x<name>.so and testsig-0x<name>.so",
        )

    def test_account_and_network_names_are_found(self):
        with tempfile.TemporaryDirectory() as folder:
            steam = Path(folder)
            (steam / "config").mkdir()
            (steam / "config/loginusers.vdf").write_text(
                '"users"\n{\n"76561198000000001"\n{\n"AccountName"\t\t"fake_account"\n'
                '"PersonaName"\t\t"Fake Persona"\n}\n}\n',
                encoding="utf-8",
            )
            with patch.object(support, "run", lambda *_: "MyHomeNetwork\n"):
                names = support.identifiers("androidboot.serialno=FAKESERIAL123 console=x", steam=steam)
        for name in ("FAKESERIAL123", "fake_account", "Fake Persona", "MyHomeNetwork"):
            self.assertIn(name, names)


if __name__ == "__main__":
    unittest.main()
