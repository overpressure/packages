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
| `phoenix/` | packages for the PICO 4: Monado with the PICO 4's driver and its tools, Basalt with the PICO 4's settings, the compositor's swapchain layer, Steam Link's controller shim, the graphics test, fan control with the battery guard and the power key | `pico4-*` |
| `headsets/<codename>/` | a headset's image definition: `layers`, `pacman.conf`, `packages.txt`, `packages.lock` | |
| `build/packages.sh`, `build/plan.py` | the build of a headset's packages; the jobs and cache keys CI builds them with | |
| `tests/` | the checks of the layout and of the build plan | |

Each folder in a layer is one recipe, a `PKGBUILD` with the files it builds, installs or applies beside
it, the project's own code among them; a code package's unit tests run in its `build()`. A recipe pins
its upstream by version and hash or by commit, and its local files by hash: `python3 build/sums.py
LAYER/RECIPE` writes those after a change.

## Image definitions

`headsets/<codename>/` defines what a headset's system image installs:

- `layers`: the layers it uses, the most general first. Each layer is a pacman repository
  `overpressure-<layer>`.
- `pacman.conf`: the repositories: the headset's layers, its own first, then Valve's Holo Core aarch64
  preview (`core`, `extra`) and the Frame's hotfix repository.
- `packages.txt`: the packages, one per line; `repository/name` takes a package from that repository,
  `!name` counts as installed without being installed.
- `packages.lock`: every upstream package of the image, by SHA-256.

overpressure's userspace build assembles the image from this definition.

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
shellcheck build/*.sh
```

## Releases

Every push and pull request builds each package in its own job on an arm64 runner
([packages.yml](.github/workflows/packages.yml)); one job then writes each layer's database. A package's
key ([build/plan.py](build/plan.py)) hashes everything its bytes depend on: its recipe folder,
`build/packages.sh`, the build image's digest and the headset's definition. A key built before comes
from the cache, so a change rebuilds only the packages it touches. A tag `YYYY.MM.DD` (or `YYYY.MM.DD.N`)
builds every package anew, without the cache, and publishes a release: every layer's packages and
database, the copyleft packages' sources and `SHA256SUMS`. Released tags never move or disappear (a
ruleset). The release's download folder is a pacman server for every layer:

```ini
[overpressure-common]
Server = https://github.com/overpressure/packages/releases/download/<tag>
```

## License

The recipes and the project's own files are under [the Unlicense](LICENSE); a recipe's `license=` names
what the package it builds is under. Patches and files derived from an upstream are under its license:
[THIRD-PARTY.md](THIRD-PARTY.md).
