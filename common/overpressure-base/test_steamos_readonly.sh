#!/bin/bash
# SPDX-License-Identifier: Unlicense
# steamos-readonly's interface as SteamOS's (its commands, messages and exit codes), with fake commands: what
# it changes on the headset (super, the mount, the file system's size) runs only there.
set -euo pipefail
here=$(cd "$(dirname "$0")" && pwd)
tmp=$(mktemp -d)
trap 'rm -rf "$tmp"' EXIT
mkdir "$tmp/bin"
cat > "$tmp/bin/findmnt" <<'EOF'
#!/bin/bash
case "$*" in *FSTYPE*) echo "${FAKE_FSTYPE:-ext4}" ;; *) echo /dev/sda7 ;; esac
EOF
cat > "$tmp/bin/id" <<'EOF'
#!/bin/bash
echo "${FAKE_UID:-1000}"
EOF
for tool in blockdev mount resize2fs; do
	printf '#!/bin/bash\necho "%s $*" >> "%s/calls"\n' "$tool" "$tmp" > "$tmp/bin/$tool"
done
chmod +x "$tmp/bin"/*
[ ! -e /.overpressure-writable ] || { echo "SKIP: this system's root holds /.overpressure-writable"; exit 0; }

fail=0
# expect EXIT STDOUT STDERR ARGS...: the program's exit code and output.
expect() {
	local code=$1 out=$2 err=$3 got=0
	shift 3
	PATH="$tmp/bin:$PATH" bash "$here/steamos-readonly" "$@" > "$tmp/out" 2> "$tmp/err" || got=$?
	if [ "$got" != "$code" ] || [ "$(cat "$tmp/out")" != "$out" ] || [ "$(cat "$tmp/err")" != "$err" ]; then
		echo "FAIL: steamos-readonly $*: exit $got, out '$(cat "$tmp/out")', err '$(cat "$tmp/err")'"
		fail=1
	fi
}

usage="Usage: steamos-readonly enable|disable|status

Enable or disable read-only on the current running SteamOS."
expect 0 "enabled" "" status
expect 1 "$usage" ""
expect 1 "$usage" "" frobnicate
# Already read-only: a warning, nothing done, also without root, as on SteamOS.
expect 0 "" "Warning: The rootfs is already read-only!
         Nothing is performed." enable
# Making it writable needs root, which a user gets through sudo.
expect 1 "steamos-readonly needs to be run as root" "" disable
expect 1 "steamos-readonly needs to be run as root" "" toggle
FAKE_FSTYPE=btrfs expect 1 "Unrecognized root filesystem type btrfs" "" status
[ ! -e "$tmp/calls" ] || { echo "FAIL: it changed something: $(cat "$tmp/calls")"; fail=1; }
[ "$fail" = 0 ] && echo "steamos-readonly: ok"
exit "$fail"
