# SPDX-License-Identifier: Unlicense
"""The QCA6390's addresses from persist: Wi-Fi (wlan-address) and Bluetooth (bt-address)."""

import importlib.machinery
import importlib.util
import os
import subprocess
import tempfile
import unittest
from pathlib import Path
from unittest import mock

HERE = Path(__file__).resolve().parent
_loader = importlib.machinery.SourceFileLoader("bt_address", str(HERE / "bt-address"))
_spec = importlib.util.spec_from_loader("bt_address", _loader)
assert _spec is not None
bt_address = importlib.util.module_from_spec(_spec)
_loader.exec_module(bt_address)


def run_wlan_address(tmp: str, ip: str, *args: str) -> subprocess.CompletedProcess:
    """Run wlan-address with a stand-in `ip` (a shell body) on persist's file in `tmp`."""
    bin_dir = Path(tmp, "bin")
    bin_dir.mkdir()
    Path(bin_dir, "ip").write_text(f"#!/bin/sh\n{ip}\n")
    (bin_dir / "ip").chmod(0o755)
    env = dict(os.environ, PATH=f"{bin_dir}{os.pathsep}{os.environ['PATH']}")
    return subprocess.run(  # noqa: S603  # the package's script under sh, no shell string
        ["sh", str(HERE / "wlan-address"), str(Path(tmp, "wlan_mac.bin")), *args],  # noqa: S607  # sh from PATH
        env=env,
        capture_output=True,
        text=True,
        check=False,
    )


class WifiTests(unittest.TestCase):
    def test_address_script_reads_the_first_interface(self):
        with tempfile.TemporaryDirectory() as tmp:
            Path(tmp, "wlan_mac.bin").write_bytes(b"Intf0MacAddress=00AA11BB22CC\nIntf1MacAddress=00AA11BB22CD\n")
            log = Path(tmp, "ip.log")
            r = run_wlan_address(tmp, f'echo "$@" >> {log}')
            self.assertEqual(r.returncode, 0, r.stderr)
            self.assertEqual(
                log.read_text().splitlines(),
                ["link set dev wlan0 down", "link set dev wlan0 address 00:aa:11:bb:22:cc"],
            )

    def test_profiles_lose_the_chips_changing_address(self):
        # The chip's own address differs at each boot; a profile naming it never matches again.
        with tempfile.TemporaryDirectory() as tmp:
            Path(tmp, "wlan_mac.bin").write_text("Intf0MacAddress=00AA11BB22CC\n")
            profiles = Path(tmp, "profiles")
            profiles.mkdir()
            profile = profiles / "home.nmconnection"
            profile.write_bytes(
                b"[wifi]\nband=no6\nmac-address=00:03:7F:12:C5:E0\nssid=home\n[wifi-security]\npsk=secret\n"
            )
            r = run_wlan_address(tmp, "exit 0", "wlan0", str(profiles))
            self.assertEqual(r.returncode, 0, r.stderr)
            self.assertEqual(profile.read_bytes(), b"[wifi]\nband=no6\nssid=home\n[wifi-security]\npsk=secret\n")


class BluetoothTests(unittest.TestCase):
    def test_address_command_is_the_management_interfaces(self):
        # Set Public Address (0x0039) for index 0, six address bytes least significant first.
        self.assertEqual(
            bt_address.command(0, bt_address.address("00:AA:11:BB:22:CC\n")),
            bytes.fromhex("3900 0000 0600 cc22bb11aa00"),
        )
        with self.assertRaises(ValueError):
            bt_address.address("00:AA:11")

    def test_address_reply_is_matched_by_index_and_opcode(self):
        complete = bytes.fromhex("0100 0000 0300 3900 00")
        self.assertEqual(bt_address.reply_status(complete, 0), 0)
        self.assertEqual(bt_address.reply_status(bytes.fromhex("0200 0000 0300 3900 0b"), 0), 11)
        self.assertIsNone(bt_address.reply_status(complete, 1))
        self.assertIsNone(bt_address.reply_status(bytes.fromhex("0100 0000 0300 0500 00"), 0))
        self.assertIsNone(bt_address.reply_status(bytes.fromhex("0400 0000 0000"), 0))

    def test_address_waits_until_the_controller_is_offered(self):
        # Right after the device appears its setup still runs: Invalid Index, then success.
        replies = [bytes.fromhex("0200 0000 0300 3900 11"), bytes.fromhex("0100 0000 0300 3900 00")]
        sent = []
        with tempfile.TemporaryDirectory() as tmp:
            mac = Path(tmp, "pico_bt_mac")
            mac.write_text("00:AA:11:BB:22:CC\n")
            with (
                mock.patch.object(bt_address, "control_socket", return_value=7),
                mock.patch.object(bt_address.os, "write", side_effect=lambda _fd, b: sent.append(b)),
                mock.patch.object(bt_address.os, "read", side_effect=lambda *_: replies.pop(0)),
                mock.patch.object(bt_address.select, "select", return_value=([7], [], [])),
                mock.patch.object(bt_address.time, "sleep"),
            ):
                self.assertEqual(bt_address.main(["bt-address", str(mac), "0"]), 0)
        self.assertEqual(len(sent), 2)


if __name__ == "__main__":
    unittest.main()
