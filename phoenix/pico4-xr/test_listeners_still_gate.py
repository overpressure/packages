# SPDX-License-Identifier: Unlicense
"""The on-headset listener check and the still gate, with fake output (for phoenix/pico4-xr)."""

from __future__ import annotations

import importlib.machinery
import importlib.util
import struct
import unittest
from pathlib import Path
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from types import ModuleType

HERE = Path(__file__).resolve().parent


def load(name: str) -> ModuleType:
    """Load one of the package's programs, which have no suffix, as a module."""
    loader = importlib.machinery.SourceFileLoader(name.replace("-", "_"), str(HERE / name))
    spec = importlib.util.spec_from_loader(loader.name, loader)
    assert spec is not None
    module = importlib.util.module_from_spec(spec)
    loader.exec_module(module)
    return module


listeners = load("pico4-listeners")
gate = load("pico4-still-gate")

SS = """\
tcp LISTEN 0 128 0.0.0.0:22 0.0.0.0:* users:(("sshd",pid=512,fd=3))
tcp LISTEN 0 128 [::]:22 [::]:* users:(("sshd",pid=512,fd=4))
tcp LISTEN 0 128 127.0.0.1:27060 0.0.0.0:* users:(("steam",pid=900,fd=8))
tcp LISTEN 0 128 [::1]:631 [::]:* users:(("cupsd",pid=33,fd=7))
udp UNCONN 0 0 0.0.0.0:68 0.0.0.0:* users:(("NetworkManager",pid=7,fd=20))
udp UNCONN 0 0 0.0.0.0:27036 0.0.0.0:* users:(("steam",pid=900,fd=9))
"""


class ListenerTests(unittest.TestCase):
    def test_loopback_does_not_count(self):
        found = listeners.parse(SS)
        self.assertEqual(
            [(i["process"], i["port"]) for i in found],
            [("sshd", 22), ("sshd", 22), ("NetworkManager", 68), ("steam", 27036)],
        )

    def test_the_expected_set_passes(self):
        self.assertEqual(listeners.unexpected(listeners.parse(SS)), [])

    def test_a_new_listener_is_caught(self):
        for line in (
            'tcp LISTEN 0 5 0.0.0.0:80 0.0.0.0:* users:(("httpd",pid=1,fd=1))',  # an unknown program
            'tcp LISTEN 0 5 0.0.0.0:2222 0.0.0.0:* users:(("sshd",pid=1,fd=1))',  # an unknown port
            "tcp LISTEN 0 5 0.0.0.0:23 0.0.0.0:*",
        ):  # no process known
            (bad,) = listeners.unexpected(listeners.parse(line))
            self.assertIn(str(bad["port"]), listeners.describe(bad))


class StillGateTests(unittest.TestCase):
    def memory(self, poses: list[tuple[int, float, int]]) -> bytearray:
        data = bytearray(gate.POSES + gate.POSE_SIZE * gate.POSE_COUNT)
        for index, (stamp, x, confidence) in enumerate(poses):
            at = gate.POSES + gate.POSE_SIZE * index
            struct.pack_into("<q", data, at, stamp)
            struct.pack_into("<3d", data, at + 8, x, 0.0, 0.0)
            struct.pack_into("<4d", data, at + 32, 0.0, 0.0, 0.0, 1.0)
            struct.pack_into("<I", data, at + 112, confidence)
        return data

    def test_movement_gap_and_loss(self):
        data = self.memory([(0, 0.0, 1), (5_000_000, 0.002, 1), (25_000_000, 0.001, 0)])
        times = iter([0.0, 1.0, 2.0, 9.0])

        def sleep(_: float) -> None:
            struct.pack_into("<Q", data, 32, 3)

        count, lost, furthest, angle, longest = gate.measure(data, 5, clock=lambda: next(times), sleep=sleep)
        self.assertEqual((count, lost), (3, 1))
        self.assertAlmostEqual(furthest, 0.002)
        self.assertAlmostEqual(angle, 0.0)
        self.assertEqual(longest, 20_000_000)


if __name__ == "__main__":
    unittest.main()
