# SPDX-License-Identifier: Unlicense
"""The suspend check's measurement and verdict, on synthetic recordings."""

import importlib.machinery
import importlib.util
import math
import struct
import tempfile
import unittest
import wave
from pathlib import Path

HERE = Path(__file__).resolve().parent
PATH = HERE / "pico4-suspend-check"
_loader = importlib.machinery.SourceFileLoader("pico4_suspend_check", str(PATH))
_spec = importlib.util.spec_from_loader("pico4_suspend_check", _loader)
assert _spec is not None
check = importlib.util.module_from_spec(_spec)
_loader.exec_module(check)


def recording(path: str, amplitude: float, noise: float = 0.0005) -> None:
    """Write a mono recording: the room's quiet, a tone from LEAD_SECONDS for TONE_SECONDS, the quiet again."""
    rate = check.RATE
    samples = []
    for i in range(int(check.RECORD_SECONDS * rate)):
        t = i / rate
        value = noise * math.sin(2 * math.pi * 317 * t)
        if check.LEAD_SECONDS <= t < check.LEAD_SECONDS + check.TONE_SECONDS:
            value += amplitude * math.sin(2 * math.pi * check.TONE_HZ * t)
        samples.append(int(value * 32767))
    with wave.open(path, "wb") as w:
        w.setnchannels(1)
        w.setsampwidth(2)
        w.setframerate(rate)
        w.writeframes(struct.pack(f"<{len(samples)}h", *samples))


class SuspendCheckTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.path = str(Path(self.tmp.name) / "r.wav")

    def tearDown(self):
        self.tmp.cleanup()

    def test_the_level_of_a_tone_is_its_amplitude_in_dbfs(self):
        recording(self.path, 0.01)
        run = check.measure(self.path)
        self.assertAlmostEqual(run["tone_db"], -40.0, delta=0.5)
        self.assertLess(run["room_db"], run["tone_db"] - 30)

    def test_the_same_tone_before_and_after_passes(self):
        before = {"room_db": -80.0, "tone_db": -41.2}
        after = {"room_db": -78.0, "tone_db": -41.5}
        self.assertEqual(check.verdict(before, after), [])

    def test_silent_speakers_after_the_suspend_fail(self):
        recording(self.path, 0.0)
        silent = check.measure(self.path)
        failures = check.verdict({"room_db": -80.0, "tone_db": -41.0}, silent)
        self.assertIn("no tone on the microphone after the suspend", failures)
        self.assertIn("the tone changed across the suspend", failures)

    def test_a_much_quieter_tone_after_the_suspend_fails(self):
        failures = check.verdict({"room_db": -80.0, "tone_db": -41.0}, {"room_db": -80.0, "tone_db": -50.0})
        self.assertEqual(failures, ["the tone changed across the suspend"])

    def test_pipewire_has_to_live_through_the_suspend(self):
        self.assertEqual(check.pipewire_failure("1234", "1234"), [])
        self.assertEqual(check.pipewire_failure("1234", "5678"), ["PipeWire did not survive the suspend"])
        self.assertEqual(check.pipewire_failure("1234", None), ["PipeWire did not survive the suspend"])

    def test_the_tone_file_is_the_stated_length_and_level(self):
        frames = check.tone_frames()
        self.assertEqual(len(frames), check.TONE_SECONDS * check.RATE * 4)
        peak = max(struct.unpack(f"<{len(frames) // 2}h", frames))
        self.assertAlmostEqual(peak / 32767, check.TONE_AMPLITUDE, delta=0.001)


if __name__ == "__main__":
    unittest.main()
