# SPDX-License-Identifier: Unlicense
"""The repository's layout: layers, recipes and each headset's image definition fit together."""

from __future__ import annotations

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


def definitions() -> list[Path]:
    """Return the headsets' image definitions; a headset's folder that holds only .gitkeep has none yet."""
    return sorted(h for h in HEADSETS.iterdir() if {p.name for p in h.iterdir()} != {".gitkeep"})


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
    script = (
        'source ./PKGBUILD; for v in pkgname source sha256sums; do eval "printf \'%s\\0\' \\"\\${$v[@]}\\""; echo; done'
    )
    shell = bash()
    assert shell is not None  # the recipe tests are skipped without bash
    run = subprocess.run([shell, "-c", script], cwd=folder, capture_output=True, text=True, check=True)  # noqa: S603
    names, sources, sums = (part.strip("\n").split("\0")[:-1] for part in run.stdout.split("\n")[:3])
    return {"pkgname": names, "source": sources, "sha256sums": sums}


def tracked() -> list[str]:
    """Return the repository's tracked files."""
    run = subprocess.run(["git", "ls-files", "-z"], cwd=ROOT, capture_output=True, check=True)  # noqa: S607
    return [name for name in run.stdout.decode().split("\0") if name]


def build_text(folder: Path, sources: set[str]) -> str:
    """Return a recipe's build(), with the local scripts it runs with bash and its loops' words spelled out."""
    pkgbuild = (folder / "PKGBUILD").read_text(encoding="utf-8")
    match = re.search(r"^build\(\) \{\n(.*?)^\}", pkgbuild, re.MULTILINE | re.DOTALL)
    text = match.group(1) if match else ""
    for script in re.findall(r"\bbash (\S+)", text):
        if script in sources:
            text += (folder / script).read_text(encoding="utf-8")
    # for t in a b; do ... test_x_$t ...: test_x_a and test_x_b are named too.
    for var, words in re.findall(r"\bfor (\w+) in ([^;\n]+); do", text):
        for prefix in re.findall(rf"(\w*)\$\{{?{var}\b\}}?", text):
            text += "\n" + " ".join(prefix + word for word in words.split())
    return text


def toml_list(text: str, key: str) -> list[str]:
    """Return the strings of a TOML array of strings written one per line (pyproject.toml's)."""
    match = re.search(rf"^{re.escape(key)} = \[\n(.*?)^\]", text, re.MULTILINE | re.DOTALL)
    return re.findall(r'^\s*"([^"]+)",', match.group(1), re.MULTILINE) if match else []


class Layers(unittest.TestCase):
    def test_every_layer_belongs_to_a_headset(self) -> None:
        used = {layer for headset in definitions() for layer in layers(headset.name)}
        folders = {p.parent.parent.name for p in ROOT.glob("*/*/PKGBUILD")}
        self.assertEqual(folders - used, set())
        for layer in used:
            self.assertIn(layer, PREFIX, layer)

    def test_names_carry_their_layer(self) -> None:
        for layer, prefix in PREFIX.items():
            for folder in recipes(layer):
                self.assertTrue(folder.name.startswith(prefix), folder)

    def test_every_headset_names_its_layers_in_pacman_conf(self) -> None:
        for headset in definitions():
            conf = (headset / "pacman.conf").read_text(encoding="utf-8")
            sections = re.findall(r"^\[([^\]]+)\]", conf, re.MULTILINE)
            ours = [s for s in sections if s.startswith("overpressure-")]
            # The headset's own layer first, each before the distribution's repositories.
            self.assertEqual(ours, [f"overpressure-{layer}" for layer in reversed(layers(headset.name))])
            self.assertLess(sections.index(ours[-1]), sections.index("core"))
            for layer in layers(headset.name):
                self.assertIn(
                    f"[overpressure-{layer}]\nSigLevel = Never\nServer = file:///var/cache/overpressure/{layer}\n", conf
                )


class Definitions(unittest.TestCase):
    def test_a_headset_without_packages_yet_holds_only_gitkeep(self) -> None:
        # Its folders exist before its first packages (the layer and headsets/<codename>/), each with a .gitkeep;
        # anything more there makes it a definition or a layer that the other tests check.
        for folder in [*HEADSETS.iterdir(), *(p for p in ROOT.iterdir() if (p / ".gitkeep").exists())]:
            names = {p.name for p in folder.iterdir()}
            if ".gitkeep" in names:
                self.assertEqual(names, {".gitkeep"}, f"{folder.name}: drop the .gitkeep with the first files")

    def test_packages_come_from_the_headsets_layers(self) -> None:
        for headset in definitions():
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
        for headset in definitions():
            for raw in (headset / "packages.lock").read_text(encoding="utf-8").splitlines():
                self.assertRegex(raw, r"^[0-9a-f]{64}  \S+\.pkg\.tar\.zst$")

    def test_files_are_lf(self) -> None:
        for name in tracked():
            self.assertNotIn(b"\r\n", (ROOT / name).read_bytes(), name)

    def test_the_readme_builds_in_the_workflows_image(self) -> None:
        workflow = (ROOT / ".github/workflows/packages.yml").read_text(encoding="utf-8")
        builder = re.search(r"^  BUILDER: (\S+)$", workflow, re.MULTILINE)
        assert builder is not None
        images = re.findall(r"ghcr\.io/overpressure/builder:\S+", (ROOT / "README.md").read_text(encoding="utf-8"))
        self.assertEqual(set(images), {builder.group(1)})


class Linters(unittest.TestCase):
    def test_every_python_program_is_linted(self) -> None:
        # ruff and ty find *.py files themselves; the programs without an extension are named in pyproject.toml.
        programs = set()
        for name in tracked():
            if "." not in Path(name).name:
                head = (ROOT / name).read_bytes()[:64].split(b"\n", 1)[0]
                if head.startswith(b"#!") and b"python" in head:
                    programs.add(name)
        settings = (ROOT / "pyproject.toml").read_text(encoding="utf-8")
        self.assertEqual(set(toml_list(settings, "extend-include")), programs)
        ty = {name for name in toml_list(settings, "include") if "/" in name and "." not in Path(name).name}
        self.assertEqual(ty, programs)


class Licenses(unittest.TestCase):
    CODE = (".c", ".cpp", ".h", ".hpp", ".sh", ".py")

    def test_code_names_its_license(self) -> None:
        for name in tracked():
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

    def test_every_test_runs_in_the_build(self) -> None:
        # A test beside a recipe runs only when its build() names it: one left out would pass unseen.
        for folder in sorted(p.parent for p in ROOT.glob("*/*/PKGBUILD")):
            with self.subTest(recipe=folder.name):
                local = {s for s in recipe(folder)["source"] if "://" not in s}
                text = build_text(folder, local)
                for source in sorted(local):
                    stem, _, suffix = source.rpartition(".")
                    if source.startswith("test_") and suffix in {"py", "sh", "c", "cpp"}:
                        named = re.search(rf"(?<![\w-]){re.escape(stem)}(?![\w-])", text)
                        self.assertIsNotNone(named, f"{source}: build() does not run it")

    def test_the_tracker_and_monado_share_one_feed_layout(self) -> None:
        # pico4-slamd and Monado's PICO 4 driver talk through this shared memory: each recipe holds a copy.
        monado, slamd = (ROOT / "phoenix" / r / "pico4_slam_feed.h" for r in ("pico4-monado", "pico4-slamd"))
        self.assertEqual(monado.read_bytes(), slamd.read_bytes())


if __name__ == "__main__":
    unittest.main()
