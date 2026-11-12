// Copyright 2026, overpressure contributors.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Read the PICO 4 lens separation (IPD) from the GD32 MCU.
 *
 * The MCU on the UART behind /dev/ttyHS1 moves the lenses, drives the panel
 * backlight, runs the fan and takes firmware updates. This module only ever
 * sends its read-only "get IPD position" command; it has no way to build any
 * other frame. pico4-fan shares the port: both hold an exclusive flock() on it
 * for each transaction.
 *
 * @ingroup drv_pico4
 */

#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define PICO4_IPD_DEVICE "/dev/ttyHS1"

//! The one request this module sends: A0 08 00 00 00, XOR checksum.
extern const uint8_t pico4_ipd_get_position[6];

/*!
 * Parse a reply to the position request: 79 a_lo a_hi b_lo b_hi xor. One value
 * is the lens separation in 0.01 mm (5700..7300), the other the Hall sensor
 * ADC; they are told apart by range. Returns false for a malformed reply.
 */
bool
pico4_ipd_parse_position(const uint8_t *reply, size_t size, double *out_ipd_m);

/*!
 * Ask the MCU once. Returns 0 and the lens separation in metres, or a negative
 * errno value (-EBUSY when the port stays locked for a second).
 */
int
pico4_ipd_read(const char *device, double *out_ipd_m);

#ifdef __cplusplus
}
#endif
