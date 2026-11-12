# SPDX-License-Identifier: Unlicense
"""The repository's layout: layers, recipes and each headset's image definition fit together."""

import hashlib
import re
import shutil
import subprocess
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
HEADSETS = ROOT / "headsets"
# A layer's packages carry its prefix, so that a name says where it comes from.
PREFIX = {"common": "overpressure-", "sm8250": "sm8250-", "phoenix": "pico4-"}
# packages.txt lines of a layer's repository whose recipe is not here yet.
ELSEWHERE = "# elsewhere:"


def lines(path: Path) -> list[str]:
    """Return the lines of PATH without comments and blanks."""
    return [s for s in (raw.split("#", 1)[0].strip() for raw in path.read_text(encoding="utf-8").splitlines()) if s]


def layers(headset: str) -> list[str]:
    """Return a headset's layers, the most general first."""
    return lines(HEADSETS / headset / "layers")


def recipes(layer: str) -> list[Path]:
    """Return the recipe folders of LAYER."""
    return sorted(p.parent for p in (ROOT / layer).glob("*/PKGBUILD"))


def bash() -> str | None:
    """Return a bash that sources PKGBUILDs (not WSL's launcher, which runs another system)."""
    found = shutil.which("bash")
    if found and "system32" not in found.lower():
        return found
    return None


def recipe(folder: Path) -> dict[str, list[str]]:
    """Return a recipe's pkgname, source and sha256sums, as bash reads them."""
    script = 'source ./PKGBUILD; for v in pkgname source sha256sums; do eval "printf \'%s\\0\' \\"\\${$v[@]}\\""; echo; done'
    run = subprocess.run([bash(), "-c", script], cwd=folder, capture_output=True, text=True, check=True)  # noqa: S603
    names, sources, sums = (part.strip("\n").split("\0")[:-1] for part in run.stdout.split("\n")[:3])
    return {"pkgname": names, "source": sources, "sha256sums": sums}


class Layers(unittest.TestCase):
    def test_every_layer_belongs_to_a_headset(self) -> None:
        used = {layer for headset in HEADSETS.iterdir() for layer in layers(headset.name)}
        folders = {p.parent.parent.name for p in ROOT.glob("*/*/PKGBUILD")}
        self.assertEqual(folders - used, set())
        for layer in used:
            self.assertIn(layer, PREFIX, layer)

    def test_names_carry_their_layer(self) -> None:
        for layer, prefix in PREFIX.items():
            for folder in recipes(layer):
                self.assertTrue(folder.name.startswith(prefix), folder)

    def test_every_headset_names_its_layers_in_pacman_conf(self) -> None:
        for headset in HEADSETS.iterdir():
            conf = (headset / "pacman.conf").read_text(encoding="utf-8")
            sections = re.findall(r"^\[([^\]]+)\]", conf, re.MULTILINE)
            ours = [s for s in sections if s.startswith("overpressure-")]
            # The headset's own layer first, each before the distribution's repositories.
            self.assertEqual(ours, [f"overpressure-{layer}" for layer in reversed(layers(headset.name))])
            self.assertLess(sections.index(ours[-1]), sections.index("core"))
            for layer in layers(headset.name):
                self.assertIn(f"[overpressure-{layer}]\nSigLevel = Never\nServer = file:///var/cache/overpressure/{layer}\n", conf)


class Definitions(unittest.TestCase):
    def test_packages_come_from_the_headsets_layers(self) -> None:
        for headset in HEADSETS.iterdir():
            available = {f"overpressure-{layer}/{p.name}" for layer in layers(headset.name) for p in recipes(layer)}
            text = (headset / "packages.txt").read_text(encoding="utf-8").splitlines()
            ours = [raw for raw in text if raw.startswith("overpressure-")]
            for raw in ours:
                name = raw.split("#", 1)[0].strip()
                repo = name.split("/", 1)[0]
                self.assertIn(repo.removeprefix("overpressure-"), layers(headset.name), raw)
                if ELSEWHERE in raw:
                    self.assertNotIn(name, available, f"{raw}: here now, drop the mark")
                else:
                    self.assertIn(name, available, raw)
            listed = {raw.split("#", 1)[0].strip() for raw in ours}
            self.assertEqual(available - listed, set(), f"{headset.name}: recipes no image installs")

    def test_lock_lines(self) -> None:
        for headset in HEADSETS.iterdir():
            for raw in (headset / "packages.lock").read_text(encoding="utf-8").splitlines():
                self.assertRegex(raw, r"^[0-9a-f]{64}  \S+\.pkg\.tar\.zst$")

    def test_files_are_lf(self) -> None:
        run = subprocess.run(["git", "ls-files", "-z"], cwd=ROOT, capture_output=True, check=True)  # noqa: S607
        for name in run.stdout.decode().split("\0"):
            if name:
                self.assertNotIn(b"\r\n", (ROOT / name).read_bytes(), name)


class Licenses(unittest.TestCase):
    CODE = (".c", ".cpp", ".h", ".hpp", ".sh", ".py")

    def test_code_names_its_license(self) -> None:
        run = subprocess.run(["git", "ls-files", "-z"], cwd=ROOT, capture_output=True, check=True)  # noqa: S607
        for name in filter(None, run.stdout.decode().split("\0")):
            if name.endswith(self.CODE) and Path(name).name != "__init__.py":
                head = (ROOT / name).read_text(encoding="utf-8").splitlines()[:5]
                self.assertTrue(any("SPDX-License-Identifier:" in line for line in head), name)


@unittest.skipUnless(bash(), "needs bash")
class Recipes(unittest.TestCase):
    def test_name_and_local_sources(self) -> None:
        for folder in sorted(p.parent for p in ROOT.glob("*/*/PKGBUILD")):
            with self.subTest(recipe=folder.name):
                r = recipe(folder)
                self.assertEqual(r["pkgname"], [folder.name])
                self.assertEqual(len(r["source"]), len(r["sha256sums"]))
                local = set()
                for source, digest in zip(r["source"], r["sha256sums"], strict=True):
                    if "://" in source:
                        continue
                    local.add(source)
                    data = (folder / source).read_bytes()
                    self.assertEqual(digest, hashlib.sha256(data).hexdigest(), source)
                # Every file beside the recipe is one of its sources.
                beside = {p.name for p in folder.iterdir() if p.name != "PKGBUILD"}
                self.assertEqual(beside - local, set())


if __name__ == "__main__":
    unittest.main()
