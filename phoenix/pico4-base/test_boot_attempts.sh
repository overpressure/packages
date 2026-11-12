#!/bin/bash
# SPDX-License-Identifier: Unlicense
# The installed system's count of unfinished boots (boot-attempts): two boots that never reached the
# graphical target, and the third goes to the maintenance stage 1.
set -euo pipefail
cd "$(dirname "$0")"
work=$(mktemp -d)
trap 'rm -rf "$work"' EXIT
export PICO4_BOOT_ATTEMPTS=$work/attempts
run() { bash ./boot-attempts "$@"; }
fail() { echo "boot-attempts: $*"; exit 1; }

[ "$(run)" = 0 ] || fail "a fresh data partition does not count 0"
[ -z "$(run start)" ] || fail "the first boot says something"
[ "$(run)" = 1 ] || fail "the first boot is not counted"
run finished
[ ! -e "$work/attempts" ] || fail "the graphical target does not clear the count"
run finished # nothing to clear

run start >/dev/null
[ "$(run start)" = "boot 2 since the last one that reached the graphical target" ] || fail "the second boot"
rc=0
out=$(run start) || rc=$?
[ "$rc" = 3 ] || fail "the third unfinished boot exits $rc, not 3"
[ "$out" = "the last 2 boots did not reach the graphical target" ] || fail "the third boot says: $out"
[ ! -e "$work/attempts" ] || fail "the count is not reset for the boot after the maintenance stage 1"
run start >/dev/null || fail "the boot after the maintenance stage 1 has no tries"

echo garbage > "$work/attempts"
run start >/dev/null || fail "an unreadable count stops the boot"
[ "$(run)" = 1 ] || fail "an unreadable count is not counted from 0"
rc=0
run frobnicate 2>/dev/null || rc=$?
[ "$rc" = 2 ] || fail "an unknown command exits $rc"
echo "boot-attempts: ok"
