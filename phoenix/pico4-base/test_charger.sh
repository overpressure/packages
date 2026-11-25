#!/bin/sh
# SPDX-License-Identifier: Unlicense
# The installed system's charger mode (charger): the power key's input events and the power-off without
# USB power.
set -eu
here=$(cd "$(dirname "$0")" && pwd)
work=$(mktemp -d)
trap 'rm -rf "$work"' EXIT
# A stand-in for stage 1's lib.sh, which the image takes from stage 1: its log and run folder.
mkdir "$work/lib" "$work/run"
printf 'log() { :; }\nOP_RUN=%s/run\n' "$work" > "$work/lib/lib.sh"
CHARGER_LIB=1 CHARGER_LIBDIR=$work/lib . "$here/charger"

# struct input_event on aarch64: 16 bytes of time, type, code (16 bit each), value (32 bit).
event() { # TYPE CODE VALUE, each below 256
	printf '\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000'
	printf "\\$(printf %03o "$1")\\000\\$(printf %03o "$2")\\000\\$(printf %03o "$3")\\000\\000\\000"
}
{ event 1 116 1; event 0 0 0; event 1 116 2; event 1 116 0; event 1 115 1; } > "$work/events"
keys=$(while key=$(read_key); do echo "$key"; done < "$work/events" | tr '\n' ' ')
[ "$keys" = "press other other release other " ] || { echo "read_key: $keys"; exit 1; }

# Without USB power it powers off after UNPLUGGED_OFF seconds, not before; with it, never.
mkdir "$work/bin"
printf '#!/bin/sh\necho "$*" > %s/reboot\nexit 0\n' "$work" > "$work/bin/pico4-reboot"
chmod +x "$work/bin/pico4-reboot"
PATH="$work/bin:$PATH"

# The battery in the lenses: the level and whether it charges, for SHOW_SECONDS; not twice at once.
printf '#!/bin/sh\necho "$*" >> %s/shown\nsleep 2\n' "$work" > "$work/bin/pico4-splash"
chmod +x "$work/bin/pico4-splash"
mkdir "$work/battery"
echo 42 > "$work/battery/capacity"
echo Charging > "$work/battery/status"
show_battery "$work/battery"
show_battery "$work/battery" # still showing: not again
wait "$SHOW_PID"
[ "$(cat "$work/shown")" = "--battery 42 1 4" ] || { echo "show_battery: $(cat "$work/shown")"; exit 1; }
echo Discharging > "$work/battery/status"
show_battery "$work/battery"
wait "$SHOW_PID"
[ "$(tail -n 1 "$work/shown")" = "--battery 42 0 4" ] || { echo "show_battery without the charger: $(tail -n 1 "$work/shown")"; exit 1; }
# shellcheck disable=SC2034 # the sourced charger's watch_power reads it
UNPLUGGED_OFF=2
echo 1 > "$work/online"
watch_power "$work/online" &
pid=$!
sleep 3
[ ! -e "$work/reboot" ] || { echo "powered off with USB power"; exit 1; }
echo 0 > "$work/online"
sleep 1
[ ! -e "$work/reboot" ] || { echo "powered off too early"; exit 1; }
# Then it powers off: the loop's sync can take seconds on a busy machine, so the check waits up to 30 s.
waited=0
until [ "$(cat "$work/reboot" 2>/dev/null)" = poweroff ] || [ "$waited" -ge 30 ]; do
	sleep 1
	waited=$((waited + 1))
done
kill "$pid" 2>/dev/null || true
[ "$(cat "$work/reboot" 2>/dev/null)" = poweroff ] || { echo "no power-off without USB power within 30 s"; exit 1; }
echo 'Charger mode: 6 checks passed'
