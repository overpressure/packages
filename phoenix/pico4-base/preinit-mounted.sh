# SPDX-License-Identifier: Unlicense
# shellcheck shell=bash
# The PICO 4's part of preinit with the data partition in place, just before systemd (overpressure-base's
# preinit sources it).
#
# PICO OS's files (overpressure/phoenix's docs/install.md#pico-oss-files): the image holds none, only links.
# Its modem, dsp and bluetooth partitions, which nothing writes, are mounted read-only as they are:
# the DSPs', the video codec's and the wireless chip's firmware, where the kernel's firmware path
# and the links point. What the install overwrote (system and vendor in super) is on the data
# partition, the copy the install writes from the headset's backup (/var/lib/pico4-stock); the
# tracker's root is completed from it, read-only, so that it looks the same to every process.
firmware=/run/pico4/firmware
for part in modem:vfat dsp:ext4 bluetooth:vfat; do
	name=${part%%:*} type=${part#*:}
	dev=$(readlink -f "/dev/block/by-name/$name" 2>/dev/null)
	opts=ro,nodev,nosuid,noexec
	[ "$type" = ext4 ] && opts=$opts,noload
	mkdir -p "$firmware/$name"
	[ -b "$dev" ] && mount -t "$type" -o "$opts" "$dev" "$firmware/$name" || log "preinit: no $name partition for its firmware"
done
printf '%s' "$firmware/modem/image" > /sys/module/firmware_class/parameters/path ||
	log "preinit: the kernel's firmware path stays the default"
stock=/var/lib/pico4-stock
island=/usr/lib/pico4-stock-slam
bind_ro() { mount --bind "$1" "$2" && mount -o remount,bind,ro "$2"; }
if [ -f "$stock/manifest.json" ]; then
	bind_ro "$stock$island/system" "$island/system" &&
		bind_ro "$stock$island/vendor" "$island/vendor" &&
		bind_ro "$firmware/dsp/cdsp" "$island/vendor/dsp/cdsp" &&
		bind_ro "$island/pico4/libandroid.so" "$island/system/lib64/libandroid.so" ||
		log "preinit: the head tracker's root is incomplete"
else
	log "preinit: no copy of PICO OS's files on the data partition (python -m overpressure dev install writes it)"
fi
# A sign of life in the lenses until pico4-splash.service takes the display; the GPU's firmware is
# there from here on.
[ -n "$OP_BOOT_UNIT" ] || /usr/bin/pico4-splash 300 </dev/null >/dev/null 2>/dev/kmsg &
