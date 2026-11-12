# Third-party material

The recipes and the project's own files are under the Unlicense ([LICENSE](LICENSE)). What a recipe's
folder holds beside its PKGBUILD and changes or derives from its upstream is under that upstream's
license:

| Folder | Files | Upstream | Pinned | License |
| --- | --- | --- | --- | --- |
| `common/overpressure-gamescope/` | the patch | gamescope | 3.16.28, commit `ea3579ed6121` | BSD-2-Clause |
| `common/overpressure-spa-alsa/` | the patch | PipeWire | 1.6.8 (release archive, SHA-256 in the recipe) | MIT |
| `sm8250/sm8250-hexagonrpc/` | the patches | hexagonrpc | commit `da7a3742ab8d` | GPL-3.0-or-later |
| `phoenix/pico4-basalt/` | the patches; `pico4-vio.json` (Basalt's defaults with the PICO 4's changes) | Basalt | 2.0.1, commit `df6e970c8da7` | BSD-3-Clause |
| `phoenix/pico4-monado/` | the patches | Monado | 25.1.0, commit `3b450e37b624` | BSL-1.0 |

The project's own code that is built into another project, or beside it, takes that project's license,
as each file's `SPDX-License-Identifier` header states:

| Folder | Files | License | Note |
| --- | --- | --- | --- |
| `phoenix/pico4-monado/` | the C and C++ files: the PICO 4 driver, its tests, the OpenXR clients and tools | BSL-1.0 | built inside Monado's tree |
| `phoenix/pico4-vrlink/` | `pico4-vrlink-compat.c` | BSL-1.0 | |

The SteamVR bindings and profile in `phoenix/pico4-monado/` (the JSON files) are the project's own, under
the Unlicense.

The packages themselves are under their upstreams' licenses, which each recipe's `license=` names. A
release carries the complete sources of its GPL and LGPL packages.
