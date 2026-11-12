# SPDX-License-Identifier: Unlicense
"""PICO OS's shutdown at a low or hot battery (docs/cooling.md#hardware-limits)."""

from __future__ import annotations

import importlib.machinery
import importlib.util
import sys
import unittest
from pathlib import Path
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from collections.abc import Callable

PATH = Path(__file__).resolve().parent / "battery-guard"
_loader = importlib.machinery.SourceFileLoader("battery_guard", str(PATH))
_spec = importlib.util.spec_from_loader("battery_guard", _loader)
assert _spec is not None
tool = importlib.util.module_from_spec(_spec)
# Dataclasses resolve their string annotations through sys.modules.
sys.modules[_spec.name] = tool
_loader.exec_module(tool)

CHARGING, DISCHARGING, NOT_CHARGING, FULL = "Charging", "Discharging", "Not charging", "Full"


class ValuesTests(unittest.TestCase):
    def test_the_stock_values(self):
        # ExtBatteryServiceImpl: batteryLevel < 1, MIN_BATTERY_LEVEL_SHUTDOWN, postDelayed 60000.
        self.assertEqual((tool.EMPTY, tool.SCREEN_OFF_LEVEL, tool.SCREEN_OFF_DELAY), (1, 4, 60.0))
        # checkBatteryStatus's notice and shutdownIfOverTempLocked's limit, in tenths of °C.
        self.assertEqual((tool.HOT_NOTICE, tool.HOT_SHUTDOWN), (580, 590))


class HotTests(unittest.TestCase):
    def test_powers_off_above_59_degrees(self):
        guard = tool.Guard()
        for temp in (250, 580, 589, 590):
            self.assertIsNone(guard.step(80, DISCHARGING, screen_on=True, now=0.0, temp=temp))
        self.assertEqual(guard.step(80, DISCHARGING, screen_on=True, now=0.0, temp=591), "battery at 59.1 °C")

    def test_also_while_charging(self):
        reason = tool.Guard().step(80, CHARGING, screen_on=True, now=0.0, temp=600)
        self.assertEqual(reason, "battery at 60.0 °C, charging")

    def test_no_temperature_decides_nothing(self):
        self.assertIsNone(tool.Guard().step(80, DISCHARGING, screen_on=True, now=0.0, temp=None))

    def test_the_notice_once_from_58_degrees(self):
        guard = tool.Guard()
        self.assertIsNone(guard.notice(579))
        self.assertIn("58.0 °C", guard.notice(580) or "")
        self.assertIsNone(guard.notice(585))
        self.assertIsNone(guard.notice(None))
        # Cooled below it and back: a new notice.
        self.assertIsNone(guard.notice(570))
        self.assertIn("58.2 °C", guard.notice(582) or "")


class ScreenOnTests(unittest.TestCase):
    def test_powers_off_only_below_one_percent(self):
        guard = tool.Guard()
        for level in (100, 20, 5, 4, 1):
            self.assertIsNone(guard.step(level, DISCHARGING, screen_on=True, now=0.0))
        self.assertEqual(guard.step(0, DISCHARGING, screen_on=True, now=0.0), "battery at 0 %")

    def test_also_while_charging(self):
        # shouldShutdownLocked: batteryLevel < 1 whatever the charger does.
        self.assertEqual(tool.Guard().step(0, CHARGING, screen_on=True, now=0.0), "battery at 0 %, charging")
        self.assertIsNone(tool.Guard().step(1, CHARGING, screen_on=True, now=0.0))

    def test_whatever_the_cable_does(self):
        for status in (NOT_CHARGING, DISCHARGING, FULL):
            self.assertIsNotNone(tool.Guard().step(0, status, screen_on=True, now=0.0))


class ScreenOffTests(unittest.TestCase):
    def test_sixty_seconds_after_the_screen_went_off_at_four_percent(self):
        guard = tool.Guard()
        self.assertIsNone(guard.step(4, DISCHARGING, screen_on=True, now=100.0))
        self.assertIsNone(guard.step(4, DISCHARGING, screen_on=False, now=110.0))
        self.assertIsNone(guard.step(4, DISCHARGING, screen_on=False, now=169.9))
        self.assertIn("screen off for 60 s", guard.step(4, DISCHARGING, screen_on=False, now=170.0))

    def test_the_screen_on_again_cancels_it(self):
        guard = tool.Guard()
        guard.step(4, DISCHARGING, screen_on=False, now=110.0)
        self.assertIsNone(guard.step(4, DISCHARGING, screen_on=True, now=120.0))
        self.assertIsNone(guard.step(4, DISCHARGING, screen_on=True, now=200.0))

    def test_above_four_percent_nothing_is_due(self):
        guard = tool.Guard()
        guard.step(5, DISCHARGING, screen_on=False, now=110.0)
        self.assertIsNone(guard.step(5, DISCHARGING, screen_on=False, now=500.0))

    def test_a_level_that_falls_to_four_with_the_screen_off(self):
        guard = tool.Guard()
        guard.step(6, DISCHARGING, screen_on=False, now=110.0)
        self.assertIsNone(guard.step(5, DISCHARGING, screen_on=False, now=120.0))
        self.assertIn("was 6 %", guard.step(4, DISCHARGING, screen_on=False, now=130.0))

    def test_the_same_level_as_at_screen_off_waits_for_the_delay(self):
        guard = tool.Guard()
        guard.step(3, DISCHARGING, screen_on=False, now=110.0)
        self.assertIsNone(guard.step(3, DISCHARGING, screen_on=False, now=120.0))
        self.assertIn("was 3 %", guard.step(2, DISCHARGING, screen_on=False, now=130.0))

    def test_not_while_charging(self):
        guard = tool.Guard()
        self.assertIsNone(guard.step(4, CHARGING, screen_on=False, now=110.0))
        self.assertIsNone(guard.step(3, CHARGING, screen_on=False, now=500.0))

    def test_screen_off_while_charging_sets_no_delay(self):
        guard = tool.Guard()
        guard.step(4, CHARGING, screen_on=False, now=110.0)
        # Unplugged later: no delay was posted, as on PICO OS; a falling level still counts.
        self.assertIsNone(guard.step(4, DISCHARGING, screen_on=False, now=500.0))
        self.assertIsNotNone(guard.step(3, DISCHARGING, screen_on=False, now=505.0))


class CheckTests(unittest.TestCase):
    @staticmethod
    def reader(**values: str) -> Callable[[str], str | None]:
        files = {
            tool.CAPACITY: values.get("capacity"),
            tool.STATUS: values.get("status"),
            tool.BACKLIGHT: values.get("backlight"),
            tool.TEMP: values.get("temp"),
        }
        return files.get

    def test_reads_the_sysfs_files(self):
        self.assertIsNotNone(
            tool.check(tool.Guard(), 0.0, self.reader(capacity="0", status=DISCHARGING, backlight="0"))
        )
        self.assertIsNone(tool.check(tool.Guard(), 0.0, self.reader(capacity="1", status=CHARGING, backlight="4")))

    def test_the_backlight_is_the_screen(self):
        guard = tool.Guard()
        tool.check(guard, 0.0, self.reader(capacity="4", status=DISCHARGING, backlight="4"))
        self.assertFalse(guard.screen_on)
        tool.check(guard, 1.0, self.reader(capacity="4", status=DISCHARGING, backlight="0"))
        self.assertTrue(guard.screen_on)

    def test_no_panel_driver_is_a_screen_on(self):
        guard = tool.Guard()
        self.assertIsNone(tool.check(guard, 0.0, self.reader(capacity="3", status=DISCHARGING)))
        self.assertTrue(guard.screen_on)

    def test_the_temperature(self):
        said: list[str] = []
        guard = tool.Guard()
        reader = self.reader(capacity="80", status=CHARGING, backlight="0", temp="583")
        self.assertIsNone(tool.check(guard, 0.0, reader, say=said.append))
        self.assertEqual(said, ["battery-guard: battery at 58.3 °C: PICO OS powers off above 59.0 °C"])
        reader = self.reader(capacity="80", status=CHARGING, backlight="0", temp="592")
        self.assertEqual(tool.check(guard, 5.0, reader, say=said.append), "battery at 59.2 °C, charging")
        self.assertEqual(len(said), 1)

    def test_a_missing_or_odd_temperature_is_none(self):
        for text in (None, "", "x"):
            self.assertIsNone(tool.temperature(text))
        self.assertEqual(tool.temperature("-35"), -35)

    def test_an_unreadable_battery_decides_nothing(self):
        for values in ({}, {"status": DISCHARGING}, {"capacity": "0"}, {"capacity": "", "status": DISCHARGING}):
            self.assertIsNone(tool.check(tool.Guard(), 0.0, self.reader(**values)))


if __name__ == "__main__":
    unittest.main()
