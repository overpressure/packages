# SPDX-License-Identifier: Unlicense
"""session-performance: the verdict on a session's frame recording."""

import importlib.machinery
import importlib.util
import json
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path
from typing import Any

HERE = Path(__file__).resolve().parent
PATH = HERE / "session-performance"
_loader = importlib.machinery.SourceFileLoader("session_performance", str(PATH))
_spec = importlib.util.spec_from_loader("session_performance", _loader)
assert _spec is not None
perf = importlib.util.module_from_spec(_spec)
_loader.exec_module(perf)


def stats(commits: int, seconds: float, late: int, stalls: int, max_interval: float = 20.4) -> dict[str, Any]:
    """msm-frame-stats' recording of the window (only the fields the verdict reads, and some)."""
    return {
        "target": "msm",
        "commits": commits,
        "commit_hz": round((commits - 1) / seconds, 4),
        "max_interval_ms": max_interval,
        "late": late,
        "stalls": stalls,
        "vblank_hz": 89.9982,
        "gpu_jobs": commits,
        "gpu_ms": {"mean": 3.1, "p90": 4.0, "max": 9.8},
    }


# Valve's session on the Frame's compositor path: 90 commits per second over 30 s.
FRAMES = stats(commits=2700, seconds=30.0, late=2, stalls=0)


class VerdictTests(unittest.TestCase):
    def test_the_window_passes(self):
        v = perf.verdict(FRAMES, 61.5)
        self.assertEqual(v["status"], "passed", v["failures"])
        self.assertAlmostEqual(v["commit_hz"], 90.0, delta=0.05)
        self.assertEqual(v["stalls"], 0)
        self.assertAlmostEqual(v["late_percent"], 100 * 2 / 2700, places=2)
        self.assertEqual(v["gpu_ms"]["mean"], 3.1)
        self.assertEqual(v["limits"]["commit_hz_min"], 85.5)

    def test_each_limit_fails_on_its_own(self):
        cases = [
            ("rate", stats(commits=1500, seconds=30.0, late=2, stalls=0), 61.5, "commits/s"),
            ("late", stats(commits=2700, seconds=30.0, late=60, stalls=0), 61.5, "late"),
            ("stall", stats(commits=2700, seconds=30.0, late=2, stalls=1), 61.5, "above 100 ms"),
            ("heat", FRAMES, 90.5, "hottest zone"),
            ("no temperature", FRAMES, None, "temperature"),
        ]
        for name, frames, hottest, words in cases:
            with self.subTest(name):
                v = perf.verdict(frames, hottest)
                self.assertEqual(v["status"], "failed")
                self.assertEqual(len(v["failures"]), 1, v["failures"])
                self.assertIn(words, v["failures"][0])

    def test_the_rate_is_the_panels_own(self):
        # At 72 Hz (the Frame's default per-app rate) 72 commits/s pass, and a slower window at 90 Hz fails.
        at72 = dict(stats(commits=2160, seconds=30.0, late=2, stalls=0), vblank_hz=72.0059)
        v = perf.verdict(at72, 61.5)
        self.assertEqual(v["status"], "passed", v["failures"])
        self.assertEqual(v["limits"]["commit_hz_min"], 68.4)
        self.assertEqual(perf.verdict(dict(at72, vblank_hz=89.99), 61.5)["status"], "failed")

    def test_missing_or_empty_statistics_fail(self):
        self.assertEqual(perf.verdict(None, 60.0)["status"], "failed")
        self.assertIn("no commits", perf.verdict({"target": "msm", "commits": 0}, 60.0)["failures"][0])

    def test_command_line(self):
        with tempfile.TemporaryDirectory() as temp:
            frames = Path(temp) / "frames.json"
            frames.write_text(json.dumps(FRAMES))
            out = subprocess.run(  # noqa: S603  # this Python on the package's program
                [sys.executable, str(PATH), str(frames), "61.5"],
                capture_output=True,
                text=True,
                encoding="utf-8",
                check=True,
            ).stdout
            self.assertEqual(json.loads(out)["status"], "passed")
            out = subprocess.run(  # noqa: S603  # this Python on the package's program
                [sys.executable, str(PATH), str(Path(temp) / "missing"), "nan"],
                capture_output=True,
                text=True,
                encoding="utf-8",
                check=True,
            ).stdout
            self.assertEqual(json.loads(out)["status"], "failed")

    def test_up_to_the_agreed_limit_of_90_c_passes(self):
        self.assertEqual(perf.verdict(FRAMES, 90.0)["status"], "passed")
        self.assertEqual(perf.verdict(FRAMES, 90.5)["failures"], ["hottest zone 90.5 °C, above 90.0 °C"])


if __name__ == "__main__":
    unittest.main()
