#!/bin/bash
# SPDX-License-Identifier: Unlicense
# Build pico4-slamd and the tracker root's stand-ins (docs/xr.md#head-tracking) for bionic
# (aarch64-linux-android29) with a host clang. No NDK: own entry point (crt.c) and declarations
# (bionic.h); no file of PICO's: they link against stand-ins of bionic's libc.so, libm.so and
# libdl.so (bionic-stubs.c), and on the headset the tracker's root has PICO OS's real ones.
# Runs the image-processing tests natively first.
#
#   build.sh OUTPUT_DIR FEED_DIR [LLVM_DIR]   (CLANG: the clang to use, else LLVM_DIR's or clang on PATH)
#
# FEED_DIR holds pico4_slam_feed.h, the shared memory's layout that Monado's PICO 4 driver
# (phoenix/pico4-monado, whose copy tests/test_layout.py keeps the same) and pico4-slamd share. OUTPUT_DIR receives pico4-slamd, libpico4-slam-stubs.so, libpico4-fastrpc-bridge.so and libandroid.so.
set -euo pipefail
src=$(cd "$(dirname "$0")" && pwd)
out=${1:?specify an output directory}
feed=${2:?specify the folder of pico4_slam_feed.h (overpressure/packages phoenix/pico4-monado)}
llvm=${3:-}
[[ -f $feed/pico4_slam_feed.h ]] || { echo "no pico4_slam_feed.h in $feed" >&2; exit 2; }
clang=${CLANG:-${llvm:+$llvm/bin/}clang}
[[ -x "$(command -v "$clang")" ]] || { echo "no clang ($clang)" >&2; exit 2; }
mkdir -p "$out"

# The image processing, natively on the build host.
work=$(mktemp -d)
trap 'rm -rf "$work"' EXIT
"$clang" -std=c11 -O2 -Wall -Wextra -Werror -o "$work/test_isp" "$src/test_isp.c"
"$work/test_isp"
"$clang" -std=c11 -O2 -Wall -Wextra -Werror -o "$work/test_room" "$src/test_room.c" -lm
"$work/test_room"
"$clang" -std=c11 -O2 -Wall -Wextra -Werror -o "$work/test_fastrpc_bridge" "$src/test_fastrpc_bridge.c"
"$work/test_fastrpc_bridge"

cc=("$clang" --target=aarch64-linux-android29 -fuse-ld=lld -O2 -fPIC -fno-stack-protector -nostdlib -ffreestanding
    -Wall -Wextra -Werror -I"$feed")
stubs=$work/stubs
mkdir -p "$stubs"
for lib in c m dl; do
    "${cc[@]}" -shared -Wl,-soname,lib$lib.so -Wl,--version-script="$src/bionic-stubs.map"         -DSTUB_LIB${lib^^} -o "$stubs/lib$lib.so" "$src/bionic-stubs.c"
done
libs=(-L"$stubs")
"${cc[@]}" -shared -Wl,-soname,libpico4-slam-stubs.so -o "$out/libpico4-slam-stubs.so" "$src/stubs.c" "${libs[@]}" -lc
"${cc[@]}" -shared -Wl,-soname,libandroid.so -o "$out/libandroid.so" "$src/android.c"
"${cc[@]}" -shared -Wl,-soname,libpico4-fastrpc-bridge.so -o "$out/libpico4-fastrpc-bridge.so" \
    "$src/fastrpc-bridge.c" "${libs[@]}" -lc
"${cc[@]}" -fPIE -pie -Wl,--dynamic-linker=/system/bin/bootstrap/linker64 -Wl,-z,max-page-size=4096 \
    -o "$out/pico4-slamd" "$src/crt.c" "$src/pico4-slamd.c" "$src/vst.c" "${libs[@]}" -lc -lm -ldl
