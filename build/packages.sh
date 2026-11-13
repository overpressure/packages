#!/usr/bin/env bash
# SPDX-License-Identifier: Unlicense
# The packages of a headset's layers, built as root in overpressure's build image
# (ghcr.io/overpressure/builder):
#
#   build/packages.sh HEADSET WORKDIR OUTDIR [PACKAGE...]
#   build/packages.sh --databases HEADSET WORKDIR OUTDIR
#
# HEADSET  headsets/HEADSET: its layers (layers), its repositories (pacman.conf), its packages
#          (packages.txt) and their pins (packages.lock), whose versions the packages build against.
# WORKDIR  the build environment and the build trees, on a Linux file system; kept, so a second run
#          rebuilds only what changed.
# OUTDIR   OUTDIR/LAYER/: the layer's packages and its database overpressure-LAYER.db; OUTDIR/sources/:
#          the complete sources of the copyleft (GPL, LGPL) packages, from makepkg --allsource.
# PACKAGE  only these packages, without the databases (default: every package of the headset's layers).
# --databases  only each layer's database, from the packages already in OUTDIR/LAYER/.
# OVERPRESSURE_REFRESH=1  the newest package databases, and what moved since the lock reported instead
#          of stopping the build (an upgrade check).
#
# The build environment is Valve/Collabora's Holo Core aarch64 preview rootfs (pinned by hash), with the
# packages the recipes build against from the headset's repositories. Each of them is pinned: by
# packages.lock, or by the Holo snapshot when the image does not install it; the build stops at any other.
# On arm64, as the releases are built, everything runs natively with the environment's own GCC. On an
# x86-64 host its programs run under QEMU and the image's clang and lld (LLVM 20) compile natively in it,
# which is their sysroot: packages for trying a change, not the releases' bytes. A package is rebuilt
# when its directory (recipe, patches, sources, files) or the compiler changes. Packages marked
# "# overpressure: local" in their PKGBUILD need the owner's files (PICO OS's) and are not built here.
set -euo pipefail
usage() { sed -n '3,/^set /{/^#/p}' "$0" >&2; exit 2; }
databases_only=0
[[ ${1:-} == --databases ]] && { databases_only=1; shift; }
[[ $# -ge 3 ]] || usage
[[ $databases_only == 0 || $# == 3 ]] || usage
headset=$1 work=$2 out=$3
shift 3
wanted=("$@")
here=$(cd "$(dirname "$0")/.." && pwd)
definition=$here/headsets/$headset
[[ -f $definition/layers ]] || { echo "no headset $headset (headsets/$headset/layers)" >&2; exit 2; }
[[ $EUID == 0 && $work = /* && $work != / ]] || { echo 'root and an absolute build folder are required' >&2; exit 2; }

HOLO_SNAPSHOT=mash-20251118.3
HOLO_URL=https://holo-packages.steamos.cloud/holo-core-aarch64-preview/$HOLO_SNAPSHOT/system.rootfs.zst
HOLO_SHA256=7e3fb88454e1ac633b7488abb72d3ca0cc7d2578a38146fdd7d58b50fcbd60bf
FRAME_REPO=deckard-arch-hotfixes-release-0.3
# What the recipes build against, installed once: their makedepends and the libraries they link.
BUILDER_PACKAGES=(base-devel git meson ninja cmake eigen openxr python-mako python-yaml
    alsa-lib pipewire-audio
    python-packaging glslang libdrm vulkan-icd-loader vulkan-headers spirv-tools zlib zstd expat libelf wayland
    wayland-protocols libx11 libxcb libxshmfence libxrandr xcb-util-keysyms opencv onetbb fmt boost ccache
    glib2-devel at-spi2-core pango cairo gdk-pixbuf2 libxcursor libxinerama libxi libxcomposite libxdamage
    shared-mime-info gtk-update-icon-cache hicolor-icon-theme glfw dbus libxkbcommon libnl pcsclite
    hwdata libavif libcap libdecor libei libinput libxmu libxres libxtst libxxf86vm luajit pipewire pixman
    sdl2 seatd xcb-util-errors xcb-util-wm xorg-xwayland libbsd libxml2-legacy
    clang lld)

mkdir -p "$work" "$out"
work=$(realpath "$work") out=$(realpath "$out")
B=$work/environment
log() { printf '>> [%4ss] %s\n' "$SECONDS" "$*"; }
SOURCE_DATE_EPOCH=${SOURCE_DATE_EPOCH:-$(git -c safe.directory='*' -C "$here" log -1 --format=%ct 2>/dev/null || date +%s)}
export SOURCE_DATE_EPOCH
mapfile -t layers < <(sed -e 's/#.*//' -e '/^[[:space:]]*$/d' "$definition/layers")

# build_jobs: one compile job per CPU, but at most one per 1.5 GiB of memory (a C++ unit of Basalt or
# Monado takes about that much).
build_jobs() {
    local cpus by_memory
    cpus=$(nproc)
    by_memory=$(awk '/^MemTotal:/ {print int($2 / 1572864)}' /proc/meminfo)
    ((by_memory >= 1)) || by_memory=1
    echo $((cpus < by_memory ? cpus : by_memory))
}
mounted=()
mnt() { mount "$@"; mounted+=("${!#}"); }
cleanup() { for ((i = ${#mounted[@]} - 1; i >= 0; i--)); do umount -l "${mounted[$i]}" 2>/dev/null || true; done; }
trap cleanup EXIT
# unmount_below DIR: detach every mount at or below DIR, left by an interrupted build, so that removing
# DIR can never reach the host's /dev or /sys.
unmount_below() {
    local p
    while read -r p; do umount -l "$p"; done < <(awk -v d="$1" '$2 == d || index($2, d "/") == 1 {print $2}' /proc/mounts | sort -r)
}
# mount_api DIR: the kernel file systems a chroot into DIR needs; /dev recursively (rootless Podman's
# placeholders), as a slave.
mount_api() {
    mnt --bind /proc "$1/proc"
    mnt --bind /sys "$1/sys"
    mnt --rbind /dev "$1/dev"
    mount --make-rslave "$1/dev"
    mnt -t devpts -o newinstance,ptmxmode=0666 devpts "$1/dev/pts"
    mnt -t tmpfs tmpfs "$1/run"
}
in_b() { chroot "$B" /usr/bin/env -i PATH=/usr/local/sbin:/usr/local/bin:/usr/bin LANG=C.UTF-8 HOME=/root \
    SOURCE_DATE_EPOCH="$SOURCE_DATE_EPOCH" "$@"; }

host_arch=$(uname -m)
if [[ $host_arch != aarch64 ]] && ! grep -qs '^flags: .*F' /proc/sys/fs/binfmt_misc/*aarch64*; then
    echo 'an aarch64 binfmt handler with the F flag is required (the build image registers one)' >&2
    exit 2
fi

# -- the build environment ----------------------------------------------------------------------
# environment: Holo's rootfs, unpacked once, with what a chroot into it needs.
environment() {
    local archive=$work/holo-$HOLO_SNAPSHOT.rootfs.zst
    if [[ ! -s $archive ]]; then
        log "downloading Holo Core aarch64 $HOLO_SNAPSHOT"
        curl -fsSL --retry 3 -o "$archive.part" "$HOLO_URL"
        mv "$archive.part" "$archive"
    fi
    printf '%s  %s\n' "$HOLO_SHA256" "$archive" | sha256sum --check --quiet
    unmount_below "$B"
    if [[ ! -f $B/.overpressure-environment ]]; then
        log "unpacking Holo Core aarch64 $HOLO_SNAPSHOT"
        rm -rf "$B"
        mkdir -p "$B"
        zstd -qdc "$archive" | tar -x --numeric-owner -C "$B"
        # What the rootfs holds is pinned by its hash.
        chroot "$B" /usr/bin/pacman -Q > "$B/.overpressure-rootfs"
        touch "$B/.overpressure-environment"
    fi
    cp -L /etc/resolv.conf "$B/etc/resolv.conf"
    mkdir -p "$B"/{proc,sys,dev,run,tmp} "$B/build"
    mount_api "$B"
}

# pins: every package of the environment is the one the pins name: packages.lock's version and, for
# what pacman downloaded, its SHA-256; a package the lock does not name is the Holo snapshot's
# (its rootfs, or its core and extra). The Frame's repository is not a snapshot: when it moves, the
# build stops here until the lock follows.
pins() {
    local sum file base name version unpinned=() rootfs snapshot
    declare -A lock_version lock_file lock_sum
    while read -r sum file; do
        base=${file%.pkg.tar.zst}
        base=${base%-*}             # the architecture
        version=${base##*-}         # the release
        base=${base%-*}
        version=${base##*-}-$version
        name=${base%-*}
        lock_version[$name]=$version lock_file[$name]=$file lock_sum[$name]=$sum
    done < "$definition/packages.lock"
    rootfs=$(cat "$B/.overpressure-rootfs")
    snapshot=$(in_b pacman -Sl core extra | awk '{print $2 " " $3}')
    while read -r name version; do
        if [[ -n ${lock_version[$name]:-} ]]; then
            if [[ ${lock_version[$name]} != "$version" ]]; then
                unpinned+=("$name $version (the lock: ${lock_version[$name]})")
            elif [[ -f $B/var/cache/pacman/pkg/${lock_file[$name]} ]] &&
                ! printf '%s  %s\n' "${lock_sum[$name]}" "$B/var/cache/pacman/pkg/${lock_file[$name]}" | sha256sum --check --quiet --status; then
                unpinned+=("$name $version (not the lock's file)")
            fi
        elif ! grep -qxF "$name $version" <<<"$rootfs" && ! grep -qxF "$name $version" <<<"$snapshot"; then
            unpinned+=("$name $version (neither in the lock nor in Holo's $HOLO_SNAPSHOT)")
        fi
    done < <(in_b pacman -Q)
    if [[ ${#unpinned[@]} -gt 0 && -n ${OVERPRESSURE_REFRESH:-} ]]; then
        printf 'moved since the lock: %s\n' "${unpinned[@]}" >&2
        log "the build environment has the newest packages"
        return
    elif [[ ${#unpinned[@]} -gt 0 ]]; then
        printf 'unpinned in the build environment: %s\n' "${unpinned[@]}" >&2
        echo "headsets/$headset/packages.lock does not pin these: update it with overpressure's userspace build (--update-lock)" >&2
        exit 1
    fi
    log "the build environment is pinned"
}

# tools: the packages the recipes build against and the compiler.
tools() {
    local frame_packages builder frame_here i f
    # The recipes build against the image's versions: what packages.txt takes from the Frame's
    # repository, the environment takes from there too (after its core and extra, as the image does).
    if ! grep -q "^\[$FRAME_REPO\]" "$B/etc/pacman.conf"; then
        printf '\n[%s]\nSigLevel = Never\n%s\n' "$FRAME_REPO" \
            "$(sed -n "/^\[$FRAME_REPO\]/,/^\$/s/^Server.*/&/p" "$definition/pacman.conf")" >> "$B/etc/pacman.conf"
    fi
    mapfile -t frame_packages < <(sed -n "s|^$FRAME_REPO/\([^[:space:]#]*\).*|\1|p" "$definition/packages.txt")
    sync=""
    [[ -s $B/var/lib/pacman/sync/$FRAME_REPO.db && -z ${OVERPRESSURE_REFRESH:-} ]] || sync=y
    builder=("${BUILDER_PACKAGES[@]}")
    for i in "${!builder[@]}"; do
        for f in "${frame_packages[@]}"; do
            [[ ${builder[i]} == "$f" ]] && builder[i]=$FRAME_REPO/$f
        done
    done
    log "the build environment's packages"
    in_b pacman "-S$sync" --noconfirm --needed "${builder[@]}" >/dev/null
    mapfile -t frame_here < <(in_b pacman -Qq "${frame_packages[@]}" 2>/dev/null)
    [[ ${#frame_here[@]} == 0 ]] || in_b pacman -S --noconfirm --needed "${frame_here[@]/#/$FRAME_REPO/}" >/dev/null
    pins
    in_b id builder >/dev/null 2>&1 || in_b useradd -m builder
    owner=$(in_b id -u builder):$(in_b id -g builder)

    compiler="GCC $(in_b gcc -dumpfullversion)"
    ccenv=""
    if [[ $host_arch == x86_64 ]]; then
        local llvm="" multiarch=/usr/lib/x86_64-linux-gnu v d
        for v in 21 20 19; do
            [[ -x /usr/lib/llvm-$v/bin/clang && -x /usr/lib/llvm-$v/bin/ld.lld ]] && { llvm=/usr/lib/llvm-$v; break; }
        done
        [[ -n $llvm ]] || { echo 'no clang and lld (LLVM 20): the build image has them' >&2; exit 2; }
        for d in "$llvm" "$multiarch"; do
            mkdir -p "$B$d"
            mnt --bind "$d" "$B$d"
            mount -o remount,bind,ro "$B$d"
        done
        # The x86-64 loader where x86-64 programs look for it.
        ln -sfn "$multiarch/ld-linux-x86-64.so.2" "$B/usr/lib/ld-linux-x86-64.so.2"
        [[ -e $B/lib64 || -L $B/lib64 ]] || ln -s usr/lib "$B/lib64"
        mkdir -p "$B/usr/local/bin"
        ln -sfn "$llvm/bin/clang" "$B/usr/local/bin/aarch64-unknown-linux-gnu-clang"
        ln -sfn "$llvm/bin/clang++" "$B/usr/local/bin/aarch64-unknown-linux-gnu-clang++"
        compiler=$("$llvm/bin/clang" --version | head -n 1)
        ccenv="OVERPRESSURE_CC=1"
    fi
    # lld links; no LTO, whose bitcode objects would need LLVM's archiver in every build system.
    mkdir -p "$B/etc/makepkg.conf.d"
    cat > "$B/etc/makepkg.conf.d/overpressure.conf" <<'EOF'
# Written by build/packages.sh: the build image's clang and lld when OVERPRESSURE_CC is set.
if [[ -n ${OVERPRESSURE_CC:-} ]]; then
    export CC=/usr/local/bin/aarch64-unknown-linux-gnu-clang CXX=/usr/local/bin/aarch64-unknown-linux-gnu-clang++
    LDFLAGS+=" -fuse-ld=lld"
    OPTIONS+=(!lto)
fi
EOF
    log "compiler: $compiler"
}

# -- the packages -------------------------------------------------------------------------------
# build LAYER NAME: makepkg LAYER/NAME into OUTDIR/LAYER, unless its directory, the makepkg settings and
# the compiler are those of the last build there.
build() {
    local layer=$1 name=$2 src=$here/$1/$2 dir=$B/build/$2 key built jobs
    key=$(cd "$src" && { find . -type f ! -path './.*' | LC_ALL=C sort | xargs sha256sum; \
        cat "$B/etc/makepkg.conf.d/overpressure.conf"; echo "$compiler"; } | sha256sum | cut -d' ' -f1)
    built=$(cat "$dir/.key" 2>/dev/null || true)
    if [[ $built != "$key" ]] || ! compgen -G "$dir/*.pkg.tar.zst" >/dev/null; then
        jobs=$(build_jobs)
        log "building $layer/$name ($compiler, $jobs jobs)"
        rm -rf "$dir"
        mkdir -p "$dir"
        cp -r "$src"/. "$dir/"
        chown -R "$owner" "$dir"
        in_b su builder -c "cd /build/$name && MAKEFLAGS=-j$jobs CMAKE_BUILD_PARALLEL_LEVEL=$jobs OVERPRESSURE_BUILD_JOBS=$jobs \
            SOURCE_DATE_EPOCH=$SOURCE_DATE_EPOCH $ccenv makepkg --noconfirm --nocheck -f" >"$dir.log" 2>&1 ||
            { tail -n 60 "$dir.log" >&2; failed "$layer/$name did not build" "$dir"; }
        # A copyleft package's complete sources (GPL, LGPL): the recipe with everything it downloaded.
        if grep -Eq '^license=\(.*GPL' "$src/PKGBUILD"; then
            in_b su builder -c "cd /build/$name && makepkg --allsource -f" >>"$dir.log" 2>&1 ||
                { tail -n 30 "$dir.log" >&2; failed "$layer/$name: no source archive" "$dir"; }
        fi
        echo "$key" > "$dir/.key"
    else
        log "$layer/$name: built already"
    fi
    mkdir -p "$out/$layer"
    rm -f "$out/$layer/$name"-[0-9]*.pkg.tar.zst "$out/sources/$name"-[0-9]*.src.tar.gz
    find "$dir" -maxdepth 1 -name '*.pkg.tar.zst' ! -name '*-debug-*' -exec cp {} "$out/$layer/" \;
    if compgen -G "$dir/*.src.tar.gz" >/dev/null; then
        mkdir -p "$out/sources"
        cp "$dir"/*.src.tar.gz "$out/sources/"
    fi
}

# failed MESSAGE DIR: stop, with the build's log and its config.log files in OUTDIR/logs.
failed() {
    local logs f
    logs=$out/logs/$(basename "$2")
    mkdir -p "$logs"
    cp "$2.log" "$logs/build.log"
    while read -r f; do
        cp "$f" "$logs/$(tr / _ <<<"${f#"$2/src/"}")"
    done < <(find "$2/src" -name config.log -size -8M 2>/dev/null)
    echo "$1 (logs: $logs)" >&2
    exit 1
}

# databases: each layer's database from the packages in OUTDIR/LAYER.
databases() {
    local layer
    for layer in "${layers[@]}"; do
        compgen -G "$out/$layer/*.pkg.tar.zst" >/dev/null || continue
        rm -rf "$B/repo/$layer"
        mkdir -p "$B/repo/$layer"
        cp "$out/$layer"/*.pkg.tar.zst "$B/repo/$layer/"
        in_b sh -c "repo-add -q /repo/$layer/overpressure-$layer.db.tar.gz /repo/$layer/*.pkg.tar.zst"
        # Files, not links: a release download has no links.
        rm -f "$out/$layer/overpressure-$layer".{db,files}*
        cp "$B/repo/$layer/overpressure-$layer".{db,files}* "$out/$layer/"
        log "$layer: $(find "$out/$layer" -name '*.pkg.tar.zst' | wc -l) packages, database overpressure-$layer"
    done
}

environment
if [[ $databases_only == 1 ]]; then
    databases
    exit 0
fi
tools
for layer in "${layers[@]}"; do
    [[ -d $here/$layer ]] || { echo "layer $layer of $headset has no folder" >&2; exit 2; }
    # A full build leaves only what the layer's recipes make; a build of named packages replaces theirs.
    [[ ${#wanted[@]} -gt 0 ]] || rm -rf "${out:?}/$layer"
    for recipe in "$here/$layer"/*/PKGBUILD; do
        [[ -e $recipe ]] || continue
        name=$(basename "$(dirname "$recipe")")
        if [[ ${#wanted[@]} -gt 0 ]] && ! printf '%s\n' "${wanted[@]}" | grep -qxF "$name"; then
            continue
        fi
        if grep -q '^# overpressure: local' "$recipe"; then
            log "$layer/$name: built on the owner's computer"
            continue
        fi
        build "$layer" "$name"
    done
done
[[ ${#wanted[@]} -gt 0 ]] || databases
