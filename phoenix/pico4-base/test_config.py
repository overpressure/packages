# SPDX-License-Identifier: Unlicense
"""The PICO 4's policy files: charger mode, core dump limit, memory and Wi-Fi."""

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


class ChargerModeTests(unittest.TestCase):
    """Charger mode starts no Bluetooth setup: its controller refuses the address there."""

    def test_the_bluetooth_units_skip_charger_mode(self):
        for unit in ("pico4-bt-firmware.service", "pico4-bt-address.service"):
            unit_section = (HERE / unit).read_text(encoding="utf-8").split("[Service]")[0]
            self.assertIn("ConditionKernelCommandLine=!androidboot.mode=charger", unit_section.splitlines(), unit)


class CoredumpLimitTests(unittest.TestCase):
    """The limit on the core dumps that vrcompositor leaves at each SteamVR stop."""

    def test_the_dumps_have_a_disk_limit(self):
        self.assertEqual(ini("50-pico4-limit.conf")["Coredump"]["MaxUse"], "1G")

    def test_core_dumps_stay_on(self):
        coredump = ini("50-pico4-limit.conf")["Coredump"]
        self.assertNotIn("Storage", coredump)
        self.assertNotIn("ProcessSizeMax", coredump)


class MemoryTests(unittest.TestCase):
    def test_pico_os_reclaim_settings(self):
        sysctl = settings("50-pico4-memory.conf", "=")
        self.assertEqual(sysctl["vm.swappiness"], "100")
        self.assertEqual(sysctl["vm.watermark_scale_factor"], "90")
        self.assertEqual(sysctl["vm.watermark_boost_factor"], "0")
        self.assertEqual(sysctl["vm.overcommit_memory"], "1")


class WifiTests(unittest.TestCase):
    def test_connections_keep_the_address_pico_assigned(self):
        conf = ini("50-pico4-wifi-address.conf")
        self.assertEqual(conf["connection"]["wifi.cloned-mac-address"], "preserve")

    def test_network_manager_retries_a_failed_connection(self):
        # With iwd's autoconnect, one DHCP timeout leaves the headset offline for good.
        self.assertEqual(ini("50-pico4-wifi-autoconnect.conf")["device"]["wifi.iwd.autoconnect"], "false")


if __name__ == "__main__":
    unittest.main()
