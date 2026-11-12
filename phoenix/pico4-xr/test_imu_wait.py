# SPDX-License-Identifier: Unlicense
"""`pico4-imu wait`: the sensors target is reached once the IMU is listed."""

from __future__ import annotations

import argparse
import contextlib
import importlib.machinery
import importlib.util
import io
import json
import unittest
from pathlib import Path
from typing import Any

HERE = Path(__file__).resolve().parent
_loader = importlib.machinery.SourceFileLoader("pico4_imu", str(HERE / "pico4-imu"))
_spec = importlib.util.spec_from_loader("pico4_imu", _loader)
assert _spec is not None
imu = importlib.util.module_from_spec(_spec)
_loader.exec_module(imu)


class Clock:
    def __init__(self):
        self.now = 0.0

    def __call__(self) -> float:
        return self.now

    def sleep(self, seconds: float) -> None:
        self.now += seconds


class FakeSsc:
    """The sensor service: absent, then empty lookups, then the IMU (seconds of the clock)."""

    def __init__(self, clock: Clock, up_at: float, listed_at: float):
        self.clock, self.up_at, self.listed_at = clock, up_at, listed_at
        self.sock = self
        self.closed = 0

    def connect(self, timeout: float) -> FakeSsc:
        if self.clock() < self.up_at:
            self.clock.now += timeout
            msg = "SNS client service (QMI 0x190) not found"
            raise RuntimeError(msg)
        return self

    def find(self, data_type: str, timeout: float = 3.0) -> list[tuple[int, int]]:
        if self.clock() < self.listed_at:
            self.clock.now += timeout
            return []
        return [(1, 2)] if data_type in ("accel", "gyro") else []

    def close(self) -> None:
        self.closed += 1


def run(timeout: float, up_at: float, listed_at: float) -> tuple[int, dict[str, Any], FakeSsc]:
    """Run `pico4-imu wait` against the fake sensor service and clock; return its code, output and service."""
    clock = Clock()
    ssc = FakeSsc(clock, up_at, listed_at)
    out = io.StringIO()
    with contextlib.redirect_stdout(out):
        code = imu.cmd_wait(argparse.Namespace(timeout=timeout), ssc.connect, clock, clock.sleep)
    return code, json.loads(out.getvalue()), ssc


class WaitTest(unittest.TestCase):
    def test_passes_once_listed(self):
        code, result, ssc = run(120, up_at=2, listed_at=35)
        self.assertEqual(code, 0)
        self.assertEqual(result["status"], "pass")
        self.assertGreaterEqual(result["seconds"], 35)
        self.assertLess(result["seconds"], 45)
        self.assertGreater(ssc.closed, 1)  # one client per attempt, each closed

    def test_already_listed(self):
        code, result, _ = run(120, up_at=0, listed_at=0)
        self.assertEqual((code, result["seconds"]), (0, 0))

    def test_times_out_with_the_last_error(self):
        code, result, _ = run(20, up_at=0, listed_at=1000)
        self.assertEqual(code, 1)
        self.assertEqual(result["status"], "fail")
        self.assertIn("no accelerometer", result["error"])
        self.assertLess(result["seconds"], 30)
        code, result, _ = run(20, up_at=1000, listed_at=1000)
        self.assertIn("not found", result["error"])


if __name__ == "__main__":
    unittest.main()
