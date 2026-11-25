# SPDX-License-Identifier: Unlicense
# shellcheck shell=bash
# The PICO 4's part of preinit before the storage policy (overpressure-base's preinit sources it).
# Powered on by the USB cable: charge, as PICO OS does, until the power key starts the system.
# shellcheck disable=SC2034  # preinit reads OP_BOOT_UNIT
[ "$(cmdline_arg androidboot.mode)" = charger ] && OP_BOOT_UNIT=pico4-charger.target
mkdir -p /run/pico4
