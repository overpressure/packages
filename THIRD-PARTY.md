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

The packages themselves are under their upstreams' licenses, which each recipe's `license=` names. A
release carries the complete sources of its GPL and LGPL packages.
