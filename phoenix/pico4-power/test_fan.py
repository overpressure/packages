# SPDX-License-Identifier: Unlicense
"""pico4-fan: the stock fan service's frames, tables and controller, the sensors, the daemon and the batch rule."""

from __future__ import annotations

import contextlib
import importlib.machinery
import importlib.util
import io
import json
import os
import sys
import tempfile
import threading
import unittest
from pathlib import Path
from unittest import mock

PATH = Path(__file__).resolve().parent / "pico4-fan"
_loader = importlib.machinery.SourceFileLoader("pico4_fan", str(PATH))
_spec = importlib.util.spec_from_loader("pico4_fan", _loader)
assert _spec is not None
fan = importlib.util.module_from_spec(_spec)
# Dataclasses resolve their string annotations through sys.modules.
sys.modules[_spec.name] = fan
_loader.exec_module(fan)

LATE, EARLY = fan.PROFILES["late"], fan.PROFILES["early"]
# The 14 zones pxrfanservice reads on kona (thermal_zone1-4, 7-15, 24), by type.
STOCK_SENSORS = [
    "cpu-0-0-usr",
    "cpu-0-1-usr",
    "cpu-0-2-usr",
    "cpu-0-3-usr",
    "cpu-1-0-usr",
    "cpu-1-1-usr",
    "cpu-1-2-usr",
    "cpu-1-3-usr",
    "cpu-1-4-usr",
    "cpu-1-5-usr",
    "cpu-1-6-usr",
    "cpu-1-7-usr",
    "gpuss-0-usr",
    "gpuss-1-usr",
]
# Zone types in the headset's numbering (pico4-probe thermal.txt), up to the GPU's second sensor.
LIVE_ZONES = [
    "aoss0-usr",
    "cpu-0-0-usr",
    "cpu-0-1-usr",
    "cpu-0-2-usr",
    "cpu-0-3-usr",
    "cpuss-0-usr",
    "cpuss-1-usr",
    "cpu-1-0-usr",
    "cpu-1-1-usr",
    "cpu-1-2-usr",
    "cpu-1-3-usr",
    "cpu-1-4-usr",
    "cpu-1-5-usr",
    "cpu-1-6-usr",
    "cpu-1-7-usr",
    "gpuss-0-usr",
    "aoss-1-usr",
    "cwlan-usr",
    "video-usr",
    "ddr-usr",
    "q6-hvx-usr",
    "camera-usr",
    "cmpss-usr",
    "npu-usr",
    "gpuss-1-usr",
    "gpuss-max-step",
    "apc-0-max-step",
    "apc-1-max-step",
    "pop-mem-step",
    "cpu-0-0-step",
]


def mc(celsius: float) -> int:
    """Return a temperature in millidegrees, as the thermal zones give it."""
    return round(celsius * 1000)


def reply(*payload: int) -> bytes:
    """Build the MCU's reply frame around this payload."""
    body = bytes((fan.REPLY, *payload))
    return body + bytes((fan.xor(body),))


class FakeMcu:
    """The GD32's fan side: acknowledges SetFanSpeed, answers the queries, can fail."""

    def __init__(self, rpm_per_percent: int = 170):
        self.duty, self.rpm_per_percent = 0, rpm_per_percent
        self.frames, self.fail = [], 0
        self.stalled = False

    def __call__(self, tx: bytes, command: int) -> bytes:
        assert len(tx) == 6, tx.hex()
        assert tx[0] == fan.REQUEST, tx.hex()
        assert fan.xor(tx[:5]) == tx[5], tx.hex()
        self.frames.append(tx)
        if self.fail:
            self.fail -= 1
            msg = "no reply"
            raise TimeoutError(msg)
        cmd = tx[1]
        if cmd == fan.SET_FAN_SPEED:
            self.duty = tx[3] | tx[4] << 8
            data = b"\x79"
        elif cmd == fan.GET_FAN_SPEED:
            data = reply(self.duty & 0xFF, self.duty >> 8)
        elif cmd == fan.GET_FAN_RPM:
            rpm = 0 if self.stalled else self.duty * self.rpm_per_percent
            data = reply(rpm & 0xFF, rpm >> 8)
        else:
            msg = f"unexpected command {cmd:#x}"
            raise AssertionError(msg)
        # Through the real parser, behind a stray byte.
        return fan.find_reply(command, b"\x00" + data)

    def sets(self):
        return [f[3] for f in self.frames if f[1] == fan.SET_FAN_SPEED]


class Clock:
    """Whole cycles of PERIOD; the times are exact at whole seconds."""

    def __init__(self):
        self.ticks = 0

    def __call__(self):
        return self.ticks * fan.PERIOD


class FrameTest(unittest.TestCase):
    def test_requests_match_the_stock_service(self):
        # gd32ipdservice: SetFanSpeed A0 0E 00 lo hi xor, the queries A0 cmd 00 00 00 xor.
        self.assertEqual(fan.request(fan.SET_FAN_SPEED, 0).hex(), "a00e000000ae")
        self.assertEqual(fan.request(fan.SET_FAN_SPEED, 50).hex(), "a00e0032009c")
        self.assertEqual(fan.request(fan.SET_FAN_SPEED, 100).hex(), "a00e006400ca")
        self.assertEqual(fan.request(fan.GET_FAN_SPEED).hex(), "a00f000000af")
        self.assertEqual(fan.request(fan.GET_FAN_RPM).hex(), "a010000000b0")
        self.assertEqual(fan.request(fan.GET_MCU_VERSION).hex(), "a02600000086")
        self.assertEqual(fan.request(fan.GET_FAN_INFO).hex(), "a03000000090")

    def test_only_the_allowlist_can_be_built(self):
        built = [cmd for cmd in range(256) for duty in (None, 0) if self._builds(cmd, duty)]
        self.assertEqual(sorted(set(built)), sorted((*fan.QUERIES, fan.SET_FAN_SPEED)))
        # IPD read and move, calibration import, OTA, backlight, SetFanRPM: never.
        for cmd in (0x08, 0x07, 0x19, 0x25, 0x05, 0x29, 0x00, 0x01, 0xFF):
            with self.assertRaises(ValueError):
                fan.request(cmd)
        for duty in (101, -1, None, True, 50.0, 255):
            with self.assertRaises(ValueError):
                fan.request(fan.SET_FAN_SPEED, duty)
        with self.assertRaises(ValueError):
            fan.request(fan.GET_FAN_SPEED, 5)

    def test_the_wake_is_pico_os_and_nothing_else_of_the_backlight(self):
        # gd32ipdservice's wakeUp: SetBrightness A0 05 00 lo hi xor with level 63 (no configuration).
        self.assertEqual(fan.request(fan.SET_BRIGHTNESS, fan.WAKE_LEVEL).hex(), "a005003f009a")
        # No other level: in particular not ScreenOffForShutDown's 0, which puts the MCU to sleep.
        for level in (0, 1, 62, 64, 255, None, 63.0, True):
            with self.assertRaises(ValueError):
                fan.request(fan.SET_BRIGHTNESS, level)

    @staticmethod
    def _builds(cmd: int, duty: int | None) -> bool:
        try:
            fan.request(cmd, duty)
        except ValueError:
            return False
        return True

    def test_replies(self):
        self.assertEqual(fan.find_reply(fan.SET_FAN_SPEED, b"\x00\xff\x79"), b"\x79")
        self.assertIsNone(fan.find_reply(fan.SET_FAN_SPEED, b"\x00\xff"))
        good = reply(0x32, 0x00)
        self.assertEqual(fan.find_reply(fan.GET_FAN_SPEED, good), good)
        self.assertIsNone(fan.find_reply(fan.GET_FAN_SPEED, good[:3]))  # not yet
        self.assertIsNone(fan.find_reply(fan.GET_FAN_SPEED, good[:3] + b"\x00"))  # bad checksum
        # A stray 79 before the reply, and a payload byte that is 79 itself.
        self.assertEqual(fan.find_reply(fan.GET_FAN_SPEED, b"\x79" + good), good)
        rpm = reply(0x79, 0x17)
        self.assertEqual(fan.find_reply(fan.GET_FAN_RPM, rpm), rpm)
        mcu = FakeMcu()
        mcu.duty = 0x1779 // mcu.rpm_per_percent
        self.assertEqual(fan.Fan(mcu).rpm(), mcu.duty * mcu.rpm_per_percent)


class AsleepMcu:
    """An MCU asleep since PICO OS's shutdown: F1 to every request until PICO OS's wake gets through."""

    def __init__(self, refusals: int = 1) -> None:
        self.refusals = refusals  # wakes it answers F1 to before it wakes
        self.awake = False
        self.sent: list[bytes] = []

    def __call__(self, tx: bytes, command: int) -> bytes:
        self.sent.append(tx)
        if not self.awake:
            if command == fan.SET_BRIGHTNESS and self.refusals == 0:
                self.awake = True
                return b"\x79"
            if command == fan.SET_BRIGHTNESS:
                self.refusals -= 1
            msg = f"no reply to {command:#04x} (f1)"
            raise fan.AsleepError(msg)
        return reply(0x2D, 0x00) if command == fan.GET_FAN_SPEED else b"\x79"


class AsleepTest(unittest.TestCase):
    def test_an_asleep_mcu_is_woken_as_pico_os_does_then_asked_again(self):
        mcu, logged, pauses = AsleepMcu(refusals=1), [], []
        duty = fan.Fan(mcu, logged.append, pauses.append).duty()
        self.assertEqual(duty, 0x2D)
        wake, ask = fan.request(fan.SET_BRIGHTNESS, fan.WAKE_LEVEL), fan.request(fan.GET_FAN_SPEED)
        self.assertEqual(mcu.sent, [ask, wake, wake, ask])  # refused, two wakes (the first refused), asked again
        self.assertEqual(pauses, [fan.WAKE_PAUSE])
        self.assertEqual(len(logged), 1)
        self.assertIn("woken", logged[0])

    def test_an_mcu_that_stays_asleep_is_an_error_after_pico_os_s_tries(self):
        mcu, pauses = AsleepMcu(refusals=10**6), []
        with self.assertRaisesRegex(TimeoutError, "still answers F1"):
            fan.Fan(mcu, sleep=pauses.append).set_duty(45)
        self.assertEqual(len(pauses), fan.WAKE_TRIES)
        # Still a TimeoutError, an OSError: the daemon's failsafe and power-off handle it as before.
        self.assertTrue(issubclass(fan.AsleepError, OSError))

    def test_an_awake_mcu_is_never_woken(self):
        mcu = AsleepMcu()
        mcu.awake = True
        fan.Fan(mcu).set_duty(45)
        self.assertEqual(mcu.sent, [fan.request(fan.SET_FAN_SPEED, 45)])


class TableTest(unittest.TestCase):
    def test_tables_match_the_stock_service(self):
        # libpxrfanservice.so, FanControl::FanControl for product "Phoenix".
        self.assertEqual(EARLY.thresholds, (90000, 85000, 70000, 60000, 55000, 50000))
        self.assertEqual(EARLY.duties, (75, 60, 45, 30, 30, 30))
        self.assertEqual(LATE.thresholds, (90000, 85000, 80500, 72000, 60000, 55000))
        self.assertEqual(LATE.duties, (50, 45, 40, 35, 30, 0))

    def run_steps(self, table: object, temps: list[float], last: int | None = None) -> list[int | None]:
        out = []
        for t in temps:
            duty = fan.table_duty(table, mc(t), last)
            if duty is not None:
                last = duty
            out.append(duty)
        return out

    def test_late_rising(self):
        self.assertEqual(
            self.run_steps(LATE, [40, 55, 60, 71.9, 72, 80.5, 85, 90, 100]), [0, None, 30, 30, 35, 40, 45, 50, 50]
        )

    def test_late_falling_holds_one_step(self):
        self.assertEqual(
            self.run_steps(LATE, [89, 84.9, 84.9, 80.4, 71.9, 59.9, 54.9, 57, 60], last=50),
            [50, 45, 45, 40, 35, None, 0, None, 30],
        )

    def test_early(self):
        self.assertEqual(self.run_steps(EARLY, [45, 52, 60, 70, 85, 90]), [30, None, 30, 45, 60, 75])
        self.assertEqual(self.run_steps(EARLY, [89, 84, 69, 59, 52, 49], last=75), [75, 60, 45, 30, None, 30])

    def test_parse(self):
        self.assertEqual(fan.Table.parse("90:50, 85:45,80.5:40,72:35,60:30,55:0"), LATE)
        for bad in ("90:50", "85:45,90:50", "90:50,85:101", "90:50,85", "x:1,2:3"):
            with self.assertRaises(ValueError):
                fan.Table.parse(bad)


class ControllerTest(unittest.TestCase):
    def test_first_step_in_the_hold_band_comes_from_below(self):
        self.assertEqual(fan.Controller(LATE).step(mc(57), mc(57)), (0, "table"))
        self.assertEqual(fan.Controller(EARLY).step(mc(52), mc(52)), (30, "table"))

    def test_hold(self):
        ctl = fan.Controller(LATE)
        ctl.step(mc(61), mc(61))
        self.assertEqual(ctl.step(mc(58), mc(58)), (30, "hold"))

    def test_boost_until_the_mean_cools(self):
        ctl = fan.Controller(LATE)
        self.assertEqual(ctl.step(mc(80), mc(94.9)), (35, "table"))
        self.assertEqual(ctl.step(mc(80), mc(95)), (100, "boost"))
        self.assertEqual(ctl.step(mc(90.5), mc(93)), (100, "boost"))
        # Back on the table from full speed: one step above the band.
        self.assertEqual(ctl.step(mc(89.9), mc(89)), (50, "table"))
        self.assertEqual(ctl.step(mc(84), mc(84)), (45, "table"))

    def test_boost_off(self):
        on, off, duty = fan.parse_boost("off")
        ctl = fan.Controller(LATE, on, off, duty)
        self.assertEqual(ctl.step(mc(99), mc(105)), (50, "table"))

    def test_failsafe_without_readings(self):
        self.assertEqual(fan.Controller(LATE).step(None, None), (100, "failsafe"))
        # Only readings above 110 °C (ignored for the means), still hot.
        self.assertEqual(fan.Controller(LATE).step(None, mc(120)), (100, "boost"))

    def test_parse_boost(self):
        self.assertEqual(fan.parse_boost(None), (95000, 90000, 100))
        self.assertEqual(fan.parse_boost("93:88:80"), (93000, 88000, 80))
        for bad in ("90:95:100", "95:90:101", "95:90", "hot"):
            with self.assertRaises(ValueError):
                fan.parse_boost(bad)


class SensorTest(unittest.TestCase):
    def test_mean_over_the_window_of_valid_readings(self):
        s = fan.Sensor("cpu-1-4-usr", "/x")
        for v in (0, -5000, 115000, None):
            s.add(v)
        self.assertIsNone(s.mean())
        self.assertIsNone(s.now)
        for v in range(1, 61):
            s.add(v * 1000)
        self.assertEqual(s.mean(), sum(range(11, 61)) * 1000 // 50)
        s.add(120000)
        self.assertEqual(s.now, 120000)
        self.assertEqual(len(s.readings), fan.WINDOW)


class ZonesTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.root = Path(self.tmp.name)
        for i, kind in enumerate(LIVE_ZONES):
            zone = self.root / f"thermal_zone{i}"
            zone.mkdir()
            (zone / "type").write_text(kind + "\n")
            (zone / "temp").write_text(f"{40000 + i}\n")
        (self.root / "thermal_zone40").mkdir()
        (self.root / "thermal_zone40/type").write_text("soc\n")
        (self.root / "thermal_zone40/temp").write_text("274000\n")
        self.zones = fan.Zones(self.root)

    def tearDown(self):
        for fd in self.zones.fds.values():
            os.close(fd)
        self.tmp.cleanup()

    def test_the_stock_sensors_in_zone_order(self):
        self.assertEqual([s.name for s in self.zones.sensors()], STOCK_SENSORS)

    def test_the_same_channels_under_mainline_names(self):
        # Mainline's sm8250 zones as Valve's kernel lists them, with their TSENS channels; the
        # stock sensors are TSENS0 1-4 and 7-15 and TSENS1 8.
        mainline = [
            "aoss0-thermal",
            "cpu0-thermal",
            "cpu1-thermal",
            "cpu2-thermal",
            "cpu3-thermal",
            "cluster0-thermal",
            "cluster1-thermal",
            "cpu4-top-thermal",
            "cpu5-top-thermal",
            "cpu6-top-thermal",
            "cpu7-top-thermal",
            "cpu4-bottom-thermal",
            "cpu5-bottom-thermal",
            "cpu6-bottom-thermal",
            "cpu7-bottom-thermal",
            "gpu-top-thermal",
            "aoss1-thermal",
            "wlan-thermal",
            "video-thermal",
            "mem-thermal",
            "q6-hvx-thermal",
            "camera-thermal",
            "compute-thermal",
            "npu-thermal",
            "gpu-bottom-thermal",
            "pm8150-thermal",
        ]
        with tempfile.TemporaryDirectory() as tmp:
            for i, kind in enumerate(mainline):
                (Path(tmp) / f"thermal_zone{i}").mkdir()
                (Path(tmp) / f"thermal_zone{i}/type").write_text(kind + "\n")
                (Path(tmp) / f"thermal_zone{i}/temp").write_text("40000\n")
            zones = fan.Zones(Path(tmp))
            try:
                names = [s.name for s in zones.sensors()]
            finally:
                for fd in zones.fds.values():
                    os.close(fd)
        self.assertEqual(names, mainline[1:5] + mainline[7:16] + ["gpu-bottom-thermal"])

    def test_reads_again_through_the_open_descriptor(self):
        path = str(self.root / "thermal_zone11/temp")
        self.assertEqual(self.zones.read(path), 40011)
        Path(path).write_text("52500\n")
        self.assertEqual(self.zones.read(path), 52500)
        self.assertIsNone(self.zones.read(str(self.root / "thermal_zone99/temp")))

    def test_hottest_skips_placeholders(self):
        self.assertEqual(self.zones.hottest(), (40000 + len(LIVE_ZONES) - 1, LIVE_ZONES[-1]))


class DaemonTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.clock = Clock()
        self.mcu = FakeMcu()
        self.temps = {name: mc(40) for name in STOCK_SENSORS}
        self.logs = []
        sensors = [fan.Sensor(name, name) for name in STOCK_SENSORS]
        self.state = Path(self.tmp.name) / "state.json"
        self.daemon = fan.Daemon(
            fan.Controller(LATE),
            sensors,
            fan.Fan(self.mcu),
            self.temps.get,
            clock=self.clock,
            log=self.logs.append,
            state_path=self.state,
            profile="late",
        )

    def tearDown(self):
        self.tmp.cleanup()

    def advance(self, seconds: float) -> None:
        for _ in range(round(seconds / fan.PERIOD)):
            self.daemon.cycle()
            self.clock.ticks += 1

    def test_sends_on_change_and_every_ten_seconds(self):
        self.advance(0.2)
        self.assertEqual(self.mcu.sets(), [0])
        self.advance(9.6)
        self.assertEqual(self.mcu.sets(), [0])
        self.advance(0.4)
        self.assertEqual(self.mcu.sets(), [0, 0])
        rpm_reads = [f for f in self.mcu.frames if f[1] == fan.GET_FAN_RPM]
        self.assertEqual(len(rpm_reads), 2)

    def test_the_mean_follows_a_step_within_ten_seconds(self):
        self.advance(10)
        for name in self.temps:
            self.temps[name] = mc(61)
        self.advance(9.4)  # 47 readings at 61 °C: the means are still below 60 °C
        self.assertEqual(self.mcu.duty, 0)
        self.advance(0.2)  # the 48th
        self.assertEqual(self.mcu.duty, 30)
        self.assertIn("duty 30 % (table; hottest mean 60.2 °C, cpu-0-0-usr)", self.logs)

    def test_boost_reacts_at_once(self):
        self.advance(1)
        self.temps["gpuss-1-usr"] = mc(95.2)
        self.advance(0.2)
        self.assertEqual(self.mcu.duty, 100)
        self.assertTrue(self.daemon.last["boost"])
        self.assertEqual(self.daemon.last["hottest_now_c"], 95.2)

    def test_failed_sends_retry_each_second_and_log_once(self):
        # SetFanSpeed and the RPM query at 0 s, SetFanSpeed at 1 s and 2 s fail.
        self.mcu.fail = 4
        self.advance(0.2)
        self.assertEqual(self.mcu.sets(), [0])
        self.assertIsNone(self.daemon.sent)
        self.advance(0.6)
        self.assertEqual(len(self.mcu.sets()), 1)  # no retry before a second has passed
        self.advance(3)
        self.assertEqual(len(self.mcu.sets()), 4)  # at 1, 2 and 3 s
        self.assertEqual(self.daemon.sent, 0)
        self.assertEqual(
            [m for m in self.logs if m.startswith("MCU")],
            ["MCU: SetFanSpeed 0 %: no reply", "MCU answers again after 4 errors"],
        )
        self.assertIsNone(self.daemon.last["error"])

    def test_state_file(self):
        self.advance(1.2)
        state = json.loads(self.state.read_text())
        self.assertEqual(state["duty"], 0)
        self.assertEqual(state["profile"], "late")
        self.assertEqual(state["reason"], "table")
        self.assertEqual(state["rpm"], 0)
        self.assertEqual(set(state["sensors"]), set(STOCK_SENSORS))
        self.assertEqual(state["sensors"]["cpu-1-4-usr"], {"now_c": 40.0, "mean_c": 40.0})
        # What the checks read through `pico4-fan state`.
        self.assertEqual(fan.fan_state(self.state, now=state["time"]), {k: state[k] for k in fan.STATE_FIELDS})

    def test_overheat_warning_like_pico_os(self):
        for name in self.temps:
            self.temps[name] = mc(96)
        self.advance(119.8)
        self.assertFalse([m for m in self.logs if "overheating" in m])
        self.advance(0.4)
        self.assertEqual(len([m for m in self.logs if "overheating" in m]), 1)
        self.advance(30)
        self.assertEqual(len([m for m in self.logs if "overheating" in m]), 2)

    def test_stalled_fan_warns_once(self):
        self.mcu.stalled = True
        for name in self.temps:
            self.temps[name] = mc(65)
        self.advance(60)
        self.assertEqual(self.mcu.duty, 30)
        self.assertEqual(len([m for m in self.logs if "0 rpm" in m]), 1)

    def hot(self, celsius: float = 96) -> None:
        for name in self.temps:
            self.temps[name] = mc(celsius)

    def test_a_silent_mcu_powers_off_where_pico_os_shows_its_notice(self):
        offs = []
        self.daemon.power_off = lambda: offs.append(self.clock())
        self.hot()
        self.mcu.fail = 10**6
        self.advance(119.8)
        self.assertEqual(offs, [])
        self.advance(0.4)
        self.assertEqual(len(offs), 1)
        self.assertIn("no reply", self.daemon.last["error"])
        self.assertEqual(len([m for m in self.logs if m.startswith("powering off: the MCU is silent")]), 1)
        # Still running a repeat later: asked again, in case the first request failed.
        self.advance(30)
        self.assertEqual(len(offs), 2)

    def test_a_stalled_fan_powers_off_where_pico_os_shows_its_notice(self):
        offs = []
        self.daemon.power_off = lambda: offs.append(self.clock())
        self.mcu.stalled = True
        self.hot()
        self.advance(120.2)
        self.assertEqual(len(offs), 1)
        self.assertEqual(
            [m for m in self.logs if m.startswith("powering off")], ["powering off: the fan reports 0 rpm at 100 %"]
        )

    def test_a_cooling_fan_never_powers_off(self):
        # PICO OS shows only its notice; the kernel's critical trips end the rest.
        offs = []
        self.daemon.power_off = lambda: offs.append(self.clock())
        self.hot(105)
        self.advance(300)
        self.assertEqual(offs, [])
        self.assertTrue([m for m in self.logs if "overheating" in m])

    def test_a_silent_mcu_below_the_limit_does_not_power_off(self):
        offs = []
        self.daemon.power_off = lambda: offs.append(self.clock())
        self.mcu.fail = 10**6
        self.hot(95)  # below PICO OS's 95.1 °C
        self.advance(300)
        self.assertEqual(offs, [])
        self.assertIn("no reply", self.daemon.last["error"])

    def test_an_mcu_that_answers_again_does_not_power_off(self):
        offs = []
        self.daemon.power_off = lambda: offs.append(self.clock())
        self.hot()
        self.mcu.fail = 50  # about 50 s of failed transactions
        self.advance(120.2)
        self.assertEqual(self.mcu.fail, 0)
        self.assertEqual(offs, [])

    def test_cooling_down_restarts_the_two_minutes(self):
        offs = []
        self.daemon.power_off = lambda: offs.append(self.clock())
        self.mcu.fail = 10**6
        self.hot()
        self.advance(100)
        self.hot(40)
        self.advance(12)  # the means follow within 10 s
        self.hot()
        self.advance(100)
        self.assertEqual(offs, [])

    def test_power_off_is_systemctls(self):
        with mock.patch.object(fan.subprocess, "run") as run:
            fan.poweroff()
        self.assertEqual(run.call_args[0][0], ["systemctl", "--no-block", "poweroff"])
        with (
            mock.patch.object(fan.subprocess, "run", side_effect=OSError("no systemctl")),
            contextlib.redirect_stderr(io.StringIO()) as err,
        ):
            fan.poweroff()  # the control loop goes on
        self.assertIn("no systemctl", err.getvalue())

    def test_dry_run_never_powers_off(self):
        offs = []
        daemon = fan.Daemon(
            fan.Controller(LATE),
            [fan.Sensor("cpu-0-0-usr", "x")],
            None,
            {"x": mc(96)}.get,
            clock=self.clock,
            log=self.logs.append,
            state_path=None,
            power_off=lambda: offs.append(1),
        )
        for _ in range(round(130 / fan.PERIOD)):
            daemon.cycle()
            self.clock.ticks += 1
        self.assertEqual(offs, [])
        self.assertTrue([m for m in self.logs if "overheating" in m])

    def test_dry_run_never_touches_the_mcu(self):
        daemon = fan.Daemon(
            fan.Controller(LATE),
            [fan.Sensor("cpu-0-0-usr", "x")],
            None,
            {"x": mc(73)}.get,
            clock=self.clock,
            log=self.logs.append,
            state_path=None,
        )
        daemon.cycle()
        self.assertEqual(daemon.sent, 35)
        self.assertEqual(self.mcu.frames, [])


def synthetic_serial(ninth: str, code: str = "000", seventeenth: str = "0") -> bytes:
    """Return a made-up serial number: FanControl reads only its 9th character, the three after it and the 17th."""
    return f"00000000{ninth}{code}0000{seventeenth}00".encode()


class BatchTest(unittest.TestCase):
    def test_the_stock_rule(self):
        cases = [
            (synthetic_serial("F"), "early"),
            (synthetic_serial("H"), "late"),
            (synthetic_serial("G", "716", "Z"), "early"),  # date code below limit1
            (synthetic_serial("G", "717", "C"), "early"),  # between the limits: the 17th character
            (synthetic_serial("G", "717", "D"), "late"),
            (synthetic_serial("G", "725", "Z"), "late"),
            (synthetic_serial("G", "726", "A"), "late"),  # from limit2 on
            (b"0000000", "early"),  # too short: the zeroed buffer reads 0
            (synthetic_serial("H")[:12] + b"\n", "late"),
        ]
        for serial, batch in cases:
            with self.subTest(serial):
                self.assertEqual(fan.serial_batch(serial), batch)

    def test_cli_prints_only_the_word(self):
        serial = synthetic_serial("G", "720", "E")
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "sn"
            path.write_bytes(serial + b"\n")
            out, err = io.StringIO(), io.StringIO()
            with contextlib.redirect_stdout(out), contextlib.redirect_stderr(err):
                self.assertEqual(fan.main(["batch", str(path)]), 0)
            self.assertEqual(out.getvalue(), "late\n")
            path.write_bytes(b"\n")
            out, err = io.StringIO(), io.StringIO()
            with contextlib.redirect_stdout(out), contextlib.redirect_stderr(err):
                self.assertEqual(fan.main(["batch", str(path)]), 1)
            self.assertEqual(out.getvalue(), "")
            self.assertNotIn(serial.decode(), err.getvalue())


class FanInfoTest(unittest.TestCase):
    # GetFanInfo replies on the headset at duties 0, 30, 45 and 60 %.
    GOLDEN = (
        ((0x00, 0x00, 0x00, 0, 0, 0, 0x00, 0), {"duty": 0, "rpm": 0, "running": False}),
        ((0x1E, 0xD3, 0x12, 0, 0, 0, 0x01, 0), {"duty": 30, "rpm": 4819, "running": True}),
        ((0x2D, 0x13, 0x1B, 0, 0, 0, 0x01, 0), {"duty": 45, "rpm": 6931, "running": True}),
        ((0x3C, 0x0E, 0x23, 0, 0, 0, 0x01, 0), {"duty": 60, "rpm": 8974, "running": True}),
    )

    def test_golden_replies(self):
        for payload, want in self.GOLDEN:
            frame = fan.find_reply(fan.GET_FAN_INFO, b"\x00" + reply(*payload))
            self.assertEqual(fan.fan_info(frame), {**want, "other": "00000000"})

    def test_wrong_length(self):
        with self.assertRaises(ValueError):
            fan.fan_info(reply(1, 2, 3))

    def test_query_stays_on_the_allowlist(self):
        self.assertEqual(fan.request(fan.GET_FAN_INFO).hex(), "a030000000" + "90")


class StateTest(unittest.TestCase):
    def test_fresh_state_only(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "state.json"
            self.assertIsNone(fan.fan_state(path))
            path.write_text("{not json")
            self.assertIsNone(fan.fan_state(path))
            state = {
                "pid": 7,
                "time": 5000.0,
                "profile": "late",
                "duty": 45,
                "target": 45,
                "reason": "table",
                "boost": False,
                "hottest": "gpuss-1-usr",
                "hottest_mean_c": 86.1,
                "hottest_now_c": 87.0,
                "rpm": 7800,
                "error": None,
                "sensors": {},
            }
            path.write_text(json.dumps(state))
            self.assertEqual(
                fan.fan_state(path, now=5004.0),
                {
                    "profile": "late",
                    "duty": 45,
                    "reason": "table",
                    "boost": False,
                    "rpm": 7800,
                    "hottest": "gpuss-1-usr",
                    "hottest_mean_c": 86.1,
                    "error": None,
                },
            )
            self.assertIsNone(fan.fan_state(path, now=5005.5))  # the daemon stopped writing

    def test_cli_without_a_daemon(self):
        out = io.StringIO()
        with contextlib.redirect_stdout(out):
            self.assertEqual(fan.main(["state"]), 0)
        self.assertEqual(out.getvalue(), "null\n")


class ProfileTest(unittest.TestCase):
    def test_resolution(self):
        with tempfile.TemporaryDirectory() as tmp:
            batch = Path(tmp) / "sn"  # persist/falcon/sn
            self.assertEqual(fan.resolve_profile(None, None, batch)[:2], ("early", EARLY))
            self.assertEqual(fan.resolve_profile("auto", None, batch)[:2], ("early", EARLY))
            batch.write_bytes(b"XXXXXXXXH0000000")  # ninth character after G: the late batch
            # The early table is the default whatever the batch; auto follows the batch.
            self.assertEqual(fan.resolve_profile(None, None, batch)[:2], ("early", EARLY))
            self.assertEqual(fan.resolve_profile("auto", None, batch)[:2], ("late", LATE))
            self.assertEqual(fan.resolve_profile("late", None, batch)[:2], ("late", LATE))
            self.assertEqual(fan.resolve_profile("early", None, batch)[:2], ("early", EARLY))
            self.assertEqual(fan.resolve_profile("late", "90:60,80:30", batch)[0], "custom")
            with self.assertRaises(ValueError):
                fan.resolve_profile("pro", None, batch)

    def test_cli_rejects_out_of_range_duties(self):
        with self.assertRaises(SystemExit), contextlib.redirect_stderr(io.StringIO()):
            fan.main(["set", "101"])


@unittest.skipUnless(hasattr(os, "openpty"), "needs a pty")
class UartTest(unittest.TestCase):
    def test_transaction_over_a_pty(self):
        master, slave = os.openpty()
        received = []

        def mcu() -> None:
            buf = b""
            while len(buf) < 6:
                buf += os.read(master, 6 - len(buf))
            received.append(buf)
            os.write(master, b"\x00" + reply(0x44, 0x16))  # stray byte, then 5700 rpm

        thread = threading.Thread(target=mcu)
        thread.start()
        try:
            rpm = fan.Fan(fan.Uart(os.ttyname(slave), timeout=2.0)).rpm()
        finally:
            thread.join(5)
            os.close(master)
            os.close(slave)
        self.assertEqual(received, [fan.request(fan.GET_FAN_RPM)])
        self.assertEqual(rpm, 5700)

    def test_only_f1_is_asleep(self):
        master, slave = os.openpty()

        def mcu(answer: bytes) -> None:
            buf = b""
            while len(buf) < 6:
                buf += os.read(master, 6 - len(buf))
            os.write(master, answer)

        try:
            for answer, error in ((b"\xf1", fan.AsleepError), (b"\xf1\x00", TimeoutError), (b"", TimeoutError)):
                thread = threading.Thread(target=mcu, args=(answer,))
                thread.start()
                with self.assertRaises(error) as caught:
                    fan.Uart(os.ttyname(slave), timeout=0.3)(fan.request(fan.GET_FAN_RPM), fan.GET_FAN_RPM)
                thread.join(5)
                self.assertIs(type(caught.exception), error)
        finally:
            os.close(master)
            os.close(slave)


if __name__ == "__main__":
    unittest.main()
