# SPDX-License-Identifier: Unlicense
"""steamvr-settings: the image's SteamVR settings reach the user's file on the data partition."""

import importlib.machinery
import importlib.util
import json
import os
import tempfile
import unittest
from pathlib import Path
from typing import Any

HERE = Path(__file__).resolve().parent
PATH = HERE / "steamvr-settings"
_loader = importlib.machinery.SourceFileLoader("steamvr_settings", str(PATH))
_spec = importlib.util.spec_from_loader("steamvr_settings", _loader)
assert _spec is not None
tool = importlib.util.module_from_spec(_spec)
_loader.exec_module(tool)


class MergeTests(unittest.TestCase):
    def setUp(self):
        self.dir = tempfile.TemporaryDirectory()
        self.image = Path(self.dir.name) / "image.vrsettings"
        self.user = Path(self.dir.name) / "config" / "steamvr.vrsettings"
        with self.image.open("w", encoding="utf-8") as f:
            json.dump(
                {
                    "steamvr": {"promptForChaperoneAtGameLaunch": True, "forcedDriver": "monado"},
                    "collisionBounds": {"CollisionBoundsStyle": 2},
                },
                f,
            )

    def tearDown(self):
        self.dir.cleanup()

    def run_tool(self) -> int:
        return tool.main(["steamvr-settings", str(self.image), str(self.user)])

    def read_user(self) -> dict[str, Any]:
        with self.user.open(encoding="utf-8") as f:
            return json.load(f)

    def test_image_keys_set_and_steamvrs_own_kept(self):
        self.user.parent.mkdir(parents=True)
        with self.user.open("w", encoding="utf-8") as f:
            json.dump(
                {
                    "steamvr": {"promptForChaperoneAtGameLaunch": False, "lastVersion": "2.17"},
                    "GpuSpeed": {"gpuSpeed0": 450},
                },
                f,
            )
        self.assertEqual(self.run_tool(), 0)
        user = self.read_user()
        self.assertTrue(user["steamvr"]["promptForChaperoneAtGameLaunch"])
        self.assertEqual(user["steamvr"]["forcedDriver"], "monado")
        self.assertEqual(user["steamvr"]["lastVersion"], "2.17")
        self.assertEqual(user["GpuSpeed"], {"gpuSpeed0": 450})
        self.assertEqual(user["collisionBounds"]["CollisionBoundsStyle"], 2)

    def test_missing_or_broken_file_gets_the_image_settings(self):
        self.assertEqual(self.run_tool(), 0)
        self.assertEqual(self.read_user()["steamvr"]["forcedDriver"], "monado")
        with self.user.open("w", encoding="utf-8") as f:
            f.write("{ half")
        self.assertEqual(self.run_tool(), 0)
        self.assertEqual(self.read_user()["collisionBounds"]["CollisionBoundsStyle"], 2)

    def test_unchanged_file_is_not_rewritten(self):
        self.run_tool()
        before = self.user.stat().st_mtime_ns
        os.utime(self.user, ns=(before - 10**9, before - 10**9))
        self.run_tool()
        self.assertEqual(self.user.stat().st_mtime_ns, before - 10**9)


class SettingsTests(unittest.TestCase):
    """The session's steamvr.vrsettings: the Frame's own settings."""

    def setUp(self):
        self.settings = json.loads((HERE / "steamvr.vrsettings").read_text(encoding="utf-8"))

    def test_frame_chaperone_settings(self):
        # The Frame driver's own room-setup settings (frame_hmd_additional.vrsettings).
        s = self.settings
        self.assertTrue(s["steamvr"]["promptForChaperoneAtGameLaunch"])
        self.assertFalse(s["steamvr"]["requireChaperoneAtGameLaunch"])
        self.assertFalse(s["steamvr"]["floorDetectAtStart"])
        self.assertEqual(s["collisionBounds"]["CollisionBoundsStyle"], 2)
        self.assertFalse(s["internal"]["autoCenterPlayArea"])
        self.assertTrue(s["dashboard"]["_inhibitRecenterOnUserInteraction"])

    def test_camera_on_as_the_frame(self):
        s = self.settings
        camera = s["camera"]
        self.assertTrue(camera["enableCamera"])
        self.assertEqual(camera["roomView"], 2)
        self.assertFalse(camera["enableCameraForCollisionBounds"])
        self.assertFalse(camera["showOnController"])
        self.assertTrue(s["steamvr"]["contentAwareRoomviewMonochromeTint"])
        self.assertFalse(s["dashboard"]["allowPassthroughPortalManager"])


if __name__ == "__main__":
    unittest.main()
