# SPDX-License-Identifier: Unlicense
"""The session's units: the start limits of SteamVR and the session, and the kernel threads' priorities."""

import configparser
import os
import subprocess
import sys
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


@unittest.skipIf(sys.platform == "win32", "its fake ps, chrt and taskset go first on a POSIX PATH")
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


#: Valve's session entries (deckard-steamvr-session) and sddm's settings, laid out as the Frame's image has them:
#: /usr/lib's names Plasma, /etc's (read later) Valve's game session.
VALVE_SESSIONS = {
    "usr/share/wayland-sessions/gamescope-wayland.desktop": "[Desktop Entry]\n"
    "Exec=start-gamescope-session gamescope-session.target\n",
    "usr/share/wayland-sessions/plasma.desktop": "[Desktop Entry]\n"
    "Exec=start-gamescope-session plasma-session.target\n",
    "usr/lib/sddm/sddm.conf.d/steamos.conf": "[Theme]\nCurrent=breeze\n\n[Autologin]\nSession=plasma.desktop\n",
    "etc/sddm.conf.d/steamos.conf": "[General]\nDisplayServer=wayland\n\n[Autologin]\nRelogin=true\n"
    "Session=gamescope-wayland.desktop\n",
}
TEMP_LOGIN = "etc/sddm.conf.d/zzt-steamos-temp-login.conf"


def run_script(name: str, root: Path, *args: str, **env: str) -> subprocess.CompletedProcess[str]:
    """Run one of the recipe's scripts under bash with PICO4_SESSION_ROOT=root."""
    environ = dict(os.environ, PICO4_SESSION_ROOT=root.as_posix(), **env)
    environ.pop("PICO4_SESSION_TARGET", None)
    environ.update(env)
    return subprocess.run(  # noqa: S603  # the recipe's script under bash, no shell
        ["bash", str(HERE / name), *args],  # noqa: S607  # bash from PATH
        env=environ,
        capture_output=True,
        text=True,
        check=False,
    )


def make_root(tmp: str, files: dict[str, str]) -> Path:
    """Write a root with @p files below @p tmp."""
    root = Path(tmp)
    for path, text in files.items():
        (root / path).parent.mkdir(parents=True, exist_ok=True)
        (root / path).write_text(text, encoding="utf-8", newline="\n")
    return root


class SessionTargetTest(unittest.TestCase):
    """session-target: session.conf's target, else the one sddm's autologin starts on the Frame."""

    def target(self, files: dict[str, str], **env: str) -> str:
        with tempfile.TemporaryDirectory() as tmp:
            r = run_script("session-target", make_root(tmp, files), **env)
            self.assertEqual(r.returncode, 0, r.stderr)
            return r.stdout.strip()

    def test_game_mode_by_default(self):
        # /etc's steamos.conf comes after /usr/lib's and names Valve's game session.
        self.assertEqual(self.target(VALVE_SESSIONS), "gamescope-session.target")

    def test_switch_to_desktop(self):
        # steamos-manager's one-shot login for Steam's Switch to Desktop.
        files = {**VALVE_SESSIONS, TEMP_LOGIN: "[Autologin]\nSession=plasma.desktop\n"}
        self.assertEqual(self.target(files), "plasma-session.target")

    def test_desktop_as_the_default_login_mode(self):
        files = {**VALVE_SESSIONS, "etc/sddm.conf.d/zz-steamos-autologin.conf": "[Autologin]\nSession=plasma\n"}
        self.assertEqual(self.target(files), "plasma-session.target")

    def test_session_conf_wins(self):
        files = {
            **VALVE_SESSIONS,
            TEMP_LOGIN: "[Autologin]\nSession=plasma.desktop\n",
            "etc/pico4/session.conf": "PICO4_SESSION_TARGET=gamescope-session.target\n",
        }
        self.assertEqual(self.target(files), "gamescope-session.target")
        self.assertEqual(
            self.target(VALVE_SESSIONS, PICO4_SESSION_TARGET="plasma-session.target"), "plasma-session.target"
        )

    def test_other_sections_and_sessions(self):
        # A Session= outside [Autologin] does not count; a session that is not Valve's falls back.
        files = {**VALVE_SESSIONS, "etc/sddm.conf.d/zz-other.conf": "[X11]\nSession=plasma.desktop\n"}
        self.assertEqual(self.target(files), "gamescope-session.target")
        files = {
            **VALVE_SESSIONS,
            "usr/share/wayland-sessions/other.desktop": "[Desktop Entry]\nExec=/usr/bin/startplasma-wayland\n",
            TEMP_LOGIN: "[Autologin]\nSession=other.desktop\n",
        }
        self.assertEqual(self.target(files), "gamescope-session.target")
        self.assertEqual(self.target({}), "gamescope-session.target")


class SessionSwitchTest(unittest.TestCase):
    """session-switch: sddm's boot-time one-shot reset, and switches outside the session's start limit."""

    def switch(self, root: Path, step: str) -> subprocess.CompletedProcess[str]:
        log = root / "systemctl.log"
        fake = root / "systemctl"
        fake.write_text(f'#!/bin/sh\necho "$*" >> "{log.as_posix()}"\n', encoding="utf-8", newline="\n")
        fake.chmod(0o755)
        # The recipe's scripts are not executable in a checkout; the package installs them so.
        target = root / "session-target"
        target.write_bytes((HERE / "session-target").read_bytes())
        target.chmod(0o755)
        r = run_script(
            "session-switch",
            root,
            step,
            PICO4_SESSION_TARGET_CMD=target.as_posix(),
            PICO4_SYSTEMCTL=fake.as_posix(),
        )
        self.assertEqual(r.returncode, 0, r.stderr)
        return r

    @staticmethod
    def resets(root: Path) -> list[str]:
        log = root / "systemctl.log"
        return log.read_text(encoding="utf-8").splitlines() if log.exists() else []

    def test_first_start_of_a_boot_drops_a_stale_one_shot(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = make_root(tmp, {**VALVE_SESSIONS, TEMP_LOGIN: "[Autologin]\nSession=plasma.desktop\n"})
            self.switch(root, "start")
            self.assertFalse((root / TEMP_LOGIN).exists())
            self.assertEqual((root / "run/pico4/session-target").read_text().strip(), "gamescope-session.target")

    def test_switch_within_a_boot(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = make_root(tmp, VALVE_SESSIONS)
            self.switch(root, "start")
            # Steam's Switch to Desktop: the one-shot login, then the session ends.
            (root / TEMP_LOGIN).write_text("[Autologin]\nSession=plasma.desktop\n", encoding="utf-8")
            self.switch(root, "stop")
            self.assertEqual(self.resets(root), ["reset-failed pico4-session.service"])
            # The next start keeps it: only the first start of a boot drops it.
            self.switch(root, "start")
            self.assertTrue((root / TEMP_LOGIN).exists())
            self.assertEqual((root / "run/pico4/session-target").read_text().strip(), "plasma-session.target")

    def test_a_session_that_ends_by_itself_counts(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = make_root(tmp, VALVE_SESSIONS)
            self.switch(root, "start")
            self.switch(root, "stop")
            self.assertEqual(self.resets(root), [])


class SessionTargetUsersTest(unittest.TestCase):
    def test_the_session_ends_the_target_that_runs(self):
        # SteamVR's end and its start limit stop the running target, Steam's or Plasma's.
        for name in ("steamvr-ended", "pico4-steamvr-failed.service"):
            text = (HERE / name).read_text(encoding="utf-8")
            self.assertIn("GAMESCOPE_SESSION_TARGET:-gamescope-session.target}", text, name)
        service = unit("pico4-session.service")["Service"]
        self.assertEqual(service["ExecStartPre"], "+/usr/libexec/pico4/session-switch start")
        self.assertEqual(service["ExecStopPost"], "+/usr/libexec/pico4/session-switch stop")


if __name__ == "__main__":
    unittest.main()
