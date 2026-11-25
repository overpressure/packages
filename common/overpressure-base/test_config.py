# SPDX-License-Identifier: Unlicense
"""The system's policy files: the watchdog and SSH."""

from __future__ import annotations

import configparser
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent


def ini(name: str) -> configparser.ConfigParser:
    """Return a systemd or NetworkManager configuration file of the package, parsed."""
    config = configparser.ConfigParser()
    config.read_string((HERE / name).read_text(encoding="utf-8"))
    return config


def settings(name: str, separator: str | None = None) -> dict[str, str]:
    """Return a file's "key value" lines (sysctl's "key = value" with `separator`), comments left out."""
    found = {}
    for line in (HERE / name).read_text(encoding="utf-8").splitlines():
        if line and not line.startswith("#"):
            key, value = line.split(separator, 1)
            found[key.strip()] = value.strip()
    return found


class WatchdogTests(unittest.TestCase):
    def test_systemd_arms_the_hardware_watchdog(self):
        # qcom_wdt takes at most 30 s; systemd pings at half of it.
        self.assertEqual(ini("system.conf.d-50-overpressure.conf")["Manager"]["RuntimeWatchdogSec"], "30s")


class SshAccessTests(unittest.TestCase):
    """Keys only, on USB and Wi-Fi (docs/security.md), and no key in the package."""

    def test_sshd_takes_keys_only(self):
        sshd = {key: value for key, value in settings("sshd_config.d-50-overpressure.conf").items() if key != "HostKey"}
        self.assertEqual(sshd["PermitRootLogin"], "prohibit-password")
        self.assertEqual(sshd["PermitEmptyPasswords"], "no")
        self.assertEqual(sshd["PasswordAuthentication"], "no")
        self.assertEqual(sshd["KbdInteractiveAuthentication"], "no")
        self.assertEqual(sshd["AuthenticationMethods"], "publickey")

    def test_no_key_in_the_package(self):
        names = [
            p.name
            for p in HERE.iterdir()
            if p.name in ("authorized_keys", "id_ed25519", "id_rsa") or p.name.startswith("ssh_host_")
        ]
        self.assertEqual(names, [])


if __name__ == "__main__":
    unittest.main()
