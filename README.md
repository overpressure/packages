# overpressure packages

**TL;DR:** the Arch packages that [overpressure](https://github.com/overpressure/overpressure) builds for
its headsets' system images, and each headset's image definition. Every package owns its recipe, patches,
files and tests. The packages are arranged in layers, from what any headset running Valve's Frame
userspace needs to what one headset needs. Tagged releases publish them, built in overpressure's build
image, so that an image's build downloads them instead of compiling them.

## Layout

| Folder | What it holds | Names |
| --- | --- | --- |
| `common/` | packages for Valve's Frame userspace on any headset: GTK 2, MangoHud, gamescope, PipeWire's ALSA plugin, libdisplay-info 0.1 | `overpressure-*` |
| `sm8250/` | packages for the Snapdragon XR2 (SM8250): hexagonrpc, the DSPs' file server | `sm8250-*` |
| `phoenix/` | packages for the PICO 4: Monado with the PICO 4's driver and its tools, Basalt with the PICO 4's settings, `pico4-slamd` (the default head tracker: the host for PICO OS's libSlam on the CDSP), the compositor's swapchain layer, Steam Link's controller shim, the graphics test, and the system around Valve's userspace with its tests: `pico4-base` (preinit, storage and recovery policy, USB link, network), `pico4-power` (fan, battery guard, power key), `pico4-audio`, `pico4-xr` (DSPs, IMU, station, lenses, cameras), `pico4-session` (Valve's session and SteamVR's settings), `pico4-checks` (the acceptance checks); each one's preset enables its units | `pico4-*` |
| `hollywood/` | packages for the Meta Quest 2 ([overpressure/hollywood](https://github.com/overpressure/hollywood)): none yet, the folder holds a `.gitkeep` until its first packages | |
| `headsets/<codename>/` | a headset's image definition: `layers`, `pacman.conf`, `packages.txt`, `packages.lock`; `headsets/hollywood/` holds a `.gitkeep` until its first packages | |
| `build/packages.sh`, `build/plan.py`, `build/notes.py`, `build/versions.py` | the build of a headset's packages; the jobs and cache keys CI builds them with; a release's notes; a release's name and the check that changed recipes raise their version | |
| `tests/` | the checks of the layout, of the build plan, of the release notes, of the versions and of pico4-monado's texts | |

Each folder in a layer is one recipe, a `PKGBUILD` with the files it builds, installs or applies beside
it, the project's own code among them; a code package's unit tests run in its `build()`. A recipe pins
its upstream by version and hash or by commit, and its local files by hash: `python3 build/sums.py
LAYER/RECIPE` writes those after a change. A `docs/...` path in a recipe's comments is one of
[overpressure/phoenix's documents](https://github.com/overpressure/phoenix/tree/main/docs).

## Image definitions

`headsets/<codename>/` defines what a headset's system image installs:

- `layers`: the layers it uses, the most general first. Each layer is a pacman repository
  `overpressure-<layer>`.
- `pacman.conf`: the repositories: the headset's layers, its own first, then Valve's Holo Core aarch64
  preview (`core`, `extra`) and the Frame's hotfix repository.
- `packages.txt`: the packages, one per line; `repository/name` takes a package from that repository,
  `!name` counts as installed without being installed.
- `packages.lock`: every upstream package of the image, by SHA-256.

Each headset's own repository assembles its image from this definition (the PICO 4's: `scripts/build-userspace.sh`
in [overpressure/phoenix](https://github.com/overpressure/phoenix)), and pins the release of this repository it
builds from.

## Building

The build runs as root in overpressure's build image ([overpressure/builder](https://github.com/overpressure/builder)),
pinned by digest. Its environment is the Holo Core aarch64 preview, pinned by hash, with the packages the
recipes build against from the headset's repositories. Every one of them is pinned: by the headset's
`packages.lock` (version and SHA-256), or by the Holo snapshot when the image does not install it. The
Frame's repository is not a snapshot; when it moves, the build stops until the lock follows. The releases are built on arm64, where everything
runs natively with the environment's GCC. On an x86-64 computer the image's clang and lld compile inside
it and its other programs run under QEMU (`OVERPRESSURE_BINFMT=1` registers QEMU): packages for trying a
change, not the releases' bytes.

```sh
docker run --rm --privileged -e OVERPRESSURE_BINFMT=1 -v "$PWD:/work" -w /work \
    ghcr.io/overpressure/builder:2026.11.12@sha256:ba4a5ec8adb886b60b873b28d7d6454abd40c1a58d08a6f6fd7d5efc65d8e4b2 \
    bash build/packages.sh phoenix /var/tmp/packages /work/out
```

`out/<layer>/` then holds the layer's packages and its database `overpressure-<layer>.db`, and
`out/sources/` the complete sources of the GPL and LGPL packages; a failed build leaves its logs in
`out/logs/`. The build folder is kept: a second run rebuilds only the recipes whose folder changed.
Named packages after the folders build only those, without the databases; `--databases` writes the
databases from the packages in `out/`.
From a checkout of overpressure, `python -m overpressure container run -- bash ...` runs the same image.

```sh
python3 -m unittest -v      # the layout: layers, names, recipes' local sources, image definitions
python3 build/shellcheck.py   # every tracked shell script, the recipes' too, down to its warnings
uvx ruff@0.17.4 format --check . && uvx ruff@0.17.4 check . && uvx ty@0.0.98 check   # the Python code (pyproject.toml)
```

## Releases

Every pull request and push to `main` builds each package in its own job on an arm64 runner
([packages.yml](.github/workflows/packages.yml)); one job then writes each layer's database. A package's
key ([build/plan.py](build/plan.py)) hashes everything its bytes depend on: its recipe folder,
`build/packages.sh`, the build image's digest and what the build environment takes from the headset's
definition: `pacman.conf`, `packages.lock` and the packages `packages.txt` takes from the Frame's
repository. Adding one of the project's packages to `packages.txt` moves no other key. Each key is built once:
a key built before comes from `main`'s cache or from the artifact an earlier run of this repository left under
it (a pull request's, never a fork's), so a change builds only the packages it touches, once, on its pull
request. A release publishes those builds under a calendar version, as overpressure's releases
([versions across the project](https://github.com/overpressure/overpressure/blob/main/docs/releasing.md#versions-across-the-project)):
`YYYY.M.PATCH`, this month's next free patch (`2026.11.20`; the releases before it were named by their day, `2026.11.19`,
`2026.11.19.1`, and sort before it). A recipe changed since the newest release raises its version (`pkgrel`, or
`pkgver` for a new upstream version; the project's own packages without an upstream take the date of their change,
`pkgver=2026.11.22`), which CI checks (`build/versions.py check`). A release holds every layer's packages
and database, the copyleft packages' sources and `SHA256SUMS`. `main` changes only by pull request, only a release
started by hand releases (a date tag pushed by hand releases nothing), and released tags never move or disappear
(rulesets). The release's download folder is a pacman server for every layer:

```ini
[overpressure-common]
Server = https://github.com/overpressure/packages/releases/download/<tag>
```

A release's notes open with its tag's message, then list the commits and the packages added, removed or at
another version since the previous release, and its commit. The pull request's last job, `ci`, passes only when
every job before it passed; it is the check `main` requires.

### A release

1. Merge the pull request; its CI built every package it changed.
2. Start the release: *Actions*, *Packages*, *Run workflow* on `main`, with one line on what it brings (in a
   terminal: `gh workflow run packages.yml --ref main -f release="..."`). Anyone with write access may start one.
   The run builds as a push does (reusing what the pull request built), then tags `main`'s commit with the next
   free date (`YYYY.MM.DD`, a second one that day `YYYY.MM.DD.1`, ...) and that line and who started it as its
   message, and publishes the release. If `main` moved while it ran, it stops before tagging: start it again.
3. A consumer pins the release's tag, its commit and the SHA-256 of its `SHA256SUMS` (the PICO 4's:
   `userspace/packages.json` in [overpressure/phoenix](https://github.com/overpressure/phoenix)). A release
   changes no headset by itself: each moves its pin when it wants what the release brings.

Releases are exact pins, so there are no preview or beta channels. A preview is a CI run's `packages` artifact
(of `main` or a pull request), or a checkout of this repository built into an image (the PICO 4's
`build-userspace.sh --packages DIR`).

## License

The recipes and the project's own files are under [the Unlicense](LICENSE); a recipe's `license=` names
what the package it builds is under. Patches and files derived from an upstream are under its license:
[THIRD-PARTY.md](THIRD-PARTY.md).
