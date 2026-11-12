# SPDX-License-Identifier: Unlicense
"""The session's units: the start limits of SteamVR and the session, and the kernel threads' priorities."""

import configparser
import os
import subprocess
import tempfile
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent


def unit(name: str) -> configparser.ConfigParser:
    """Parse a systemd unit of the recipe, keys in their case."""
    parser = configparser.ConfigParser(strict=False, interpolation=None)
    parser.optionxform = lambda optionstr: optionstr  # keep the keys' case
    parser.read_string((HERE / name).read_text(encoding="utf-8"))
    return parser


class StartLimitTest(unittest.TestCase):
    def setUp(self):
        self.steamvr = unit("50-pico4-restart-limit.conf")
        self.session = unit("pico4-session.service")

    def test_steamvr_crash_loop_trips_its_limit(self):
        interval = int(self.steamvr["Unit"]["StartLimitIntervalSec"])
        burst = int(self.steamvr["Unit"]["StartLimitBurst"])
        restart = int(self.steamvr["Service"]["RestartSec"])
        # The bursts of a crash loop (restart wait plus a start of up to 20 s) fit the interval.
        self.assertGreaterEqual(interval, burst * (restart + 20))
        self.assertLessEqual(burst, 3)

    def test_session_crash_loop_trips_its_limit(self):
        interval = int(self.session["Unit"]["StartLimitIntervalSec"])
        burst = int(self.session["Unit"]["StartLimitBurst"])
        # A session that keeps ending by itself: its restart wait plus a start of up to 30 s.
        self.assertGreaterEqual(interval, burst * (int(self.session["Service"]["RestartSec"]) + 30))
        # A SteamVR crash loop per session: three starts about 25 s apart plus the session's restart.
        per_session = 3 * int(self.steamvr["Service"]["RestartSec"]) + 5
        self.assertGreater(interval, burst * per_session)


class KthreadPrioritiesTest(unittest.TestCase):
    def test_valve_kernel_gets_the_frames_values(self):
        with tempfile.TemporaryDirectory() as tmp:
            bin_dir = Path(tmp, "bin")
            bin_dir.mkdir()
            log = Path(tmp, "log").as_posix()
            processes = (
                "10 gpu-worker",
                "11 crtc_event:91",
                "12 atomic-worker-0",
                "13 spi0",
                "14 irq/252-soundwi",
                "15 kworker/0:1",
            )
            for name, body in (
                ("ps", "printf '%s\\n' " + " ".join(f"'{p}'" for p in processes)),
                ("chrt", f'echo "chrt $*" >> {log}'),
                # An interrupt thread refuses a new affinity.
                ("taskset", f'echo "taskset $*" >> {log}; [ "$3" != 14 ]'),
            ):
                path = bin_dir / name
                path.write_bytes(f"#!/bin/sh\n{body}\n".encode())
                path.chmod(0o755)
            workqueues = Path(tmp, "sys/bus/workqueue/devices")
            for wq in ("msm", "ring0", "ring1", "ring2", "ring3", "dsi_drm_work_0", "dsi_drm_work_1"):
                (workqueues / wq).mkdir(parents=True)
                (workqueues / wq / "prio").write_text("255\n")
            env = dict(os.environ, PICO4_SYSFS=Path(tmp, "sys").as_posix(), PATH=f"{bin_dir}:{os.environ['PATH']}")
            r = subprocess.run(  # noqa: S603  # the recipe's script under bash, no shell
                ["bash", str(HERE / "kthread-priorities")],  # noqa: S607  # bash from PATH
                env=env,
                capture_output=True,
                text=True,
                check=False,
            )
            self.assertEqual(r.returncode, 0, r.stderr)
            calls = Path(log).read_text().splitlines()
            for line in (
                "chrt -r -p 99 10",
                "chrt -r -p 99 11",
                "chrt -r -p 88 12",
                "chrt -r -p 80 13",
                "taskset -p 0x0f 13",
                "chrt -r -p 80 14",
            ):
                self.assertIn(line, calls)
            self.assertFalse([c for c in calls if c.endswith(" 15")])
            self.assertEqual((workqueues / "ring0/prio").read_text().strip(), "90")
            self.assertEqual((workqueues / "dsi_drm_work_1/prio").read_text().strip(), "90")


if __name__ == "__main__":
    unittest.main()
