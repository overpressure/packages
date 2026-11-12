# SPDX-License-Identifier: Unlicense
"""build/plan.py: a job per package, and a key that moves with everything the package depends on."""

import importlib.util
import shutil
import tempfile
import unittest
from pathlib import Path
from unittest import mock

ROOT = Path(__file__).resolve().parent.parent
SPEC = importlib.util.spec_from_file_location("plan", ROOT / "build/plan.py")
assert SPEC and SPEC.loader
plan = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(plan)


class Plan(unittest.TestCase):
    def setUp(self) -> None:
        self.tmp = Path(tempfile.mkdtemp())
        self.addCleanup(shutil.rmtree, self.tmp)
        (self.tmp / "build").mkdir()
        (self.tmp / "build/packages.sh").write_text("build\nFRAME_REPO=frame\n")
        definition = self.tmp / "headsets/h"
        definition.mkdir(parents=True)
        (definition / "layers").write_text("# general first\ncommon\nh\n")
        for name in ("pacman.conf", "packages.lock"):
            (definition / name).write_text(f"{name}\n")
        (definition / "packages.txt").write_text("base\nframe/steamvr\nh/h-c\n")
        for layer, name in (("common", "overpressure-a"), ("common", "overpressure-b"), ("h", "h-c")):
            folder = self.tmp / layer / name
            folder.mkdir(parents=True)
            (folder / "PKGBUILD").write_text(f"pkgname={name}\n")
        local = self.tmp / "h/h-local"
        local.mkdir()
        (local / "PKGBUILD").write_text("# overpressure: local\npkgname=h-local\n")
        patcher = mock.patch.object(plan, "ROOT", self.tmp)
        patcher.start()
        self.addCleanup(patcher.stop)

    def keys(self, builder: str = "image@sha256:1") -> dict[str, str]:
        return {job["name"]: job["key"] for job in plan.plan(["h"], builder)}

    def test_a_job_per_package_without_the_local_ones(self) -> None:
        jobs = plan.plan(["h"], "image")
        self.assertEqual([(j["layer"], j["name"]) for j in jobs],
                         [("common", "overpressure-a"), ("common", "overpressure-b"), ("h", "h-c")])
        self.assertEqual({j["headset"] for j in jobs}, {"h"})

    def test_a_recipe_change_moves_only_its_key(self) -> None:
        before = self.keys()
        (self.tmp / "common/overpressure-a/fix.patch").write_text("patch\n")
        after = self.keys()
        self.assertNotEqual(before["overpressure-a"], after["overpressure-a"])
        self.assertEqual(before["overpressure-b"], after["overpressure-b"])
        self.assertEqual(before["h-c"], after["h-c"])

    def test_the_build_the_image_and_the_definition_move_every_key(self) -> None:
        before = self.keys()
        self.assertTrue(all(before[n] != k for n, k in self.keys("image@sha256:2").items()))
        for path in ("build/packages.sh", "headsets/h/packages.lock", "headsets/h/pacman.conf"):
            with self.subTest(path=path):
                old = self.keys()
                (self.tmp / path).write_text("FRAME_REPO=frame\nchanged " + path)
                self.assertTrue(all(old[n] != k for n, k in self.keys().items()))

    def test_a_frame_package_of_the_list_moves_every_key(self) -> None:
        before = self.keys()
        with (self.tmp / "headsets/h/packages.txt").open("a") as f:
            f.write("frame/gamescope  # the environment installs it\n")
        self.assertTrue(all(before[n] != k for n, k in self.keys().items()))

    def test_the_rest_of_the_list_and_the_layers_move_no_key(self) -> None:
        before = self.keys()
        with (self.tmp / "headsets/h/packages.txt").open("a") as f:
            f.write("# a new package of ours\nh/h-d\njq\n")
        (self.tmp / "headsets/h/layers").write_text("common\nh\n")
        self.assertEqual(before, self.keys())

    def test_hidden_files_do_not_count(self) -> None:
        before = self.keys()
        (self.tmp / "h/h-c/.key").write_text("from a local build\n")
        self.assertEqual(before, self.keys())


if __name__ == "__main__":
    unittest.main()
