// Copyright 2026, overpressure contributors.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  PICO 4 controllers: inputs and 3DoF orientation from the controller radio's packets.
 * @ingroup drv_pico4
 */

#pragma once

#include "pico4_blemcu.h"

#include "xrt/xrt_device.h"
#include "util/u_logging.h"

#ifdef __cplusplus
extern "C" {
#endif

struct pico4_controller;
struct pico4_ctrack;

/*!
 * Controller @p device (PICO4_BLEMCU_LEFT or _RIGHT). Its pose comes from the
 * controller tracker @p ctrack (borrowed, may be NULL) while that tracks it;
 * otherwise the 3DoF orientation at a fixed offset from @p head. The object
 * is shared by Monado (through the returned device) and the HMD's sensor
 * thread; each side lets go with its own call and the last one frees it.
 */
struct pico4_station;

struct pico4_controller *
pico4_controller_create(int device,
                        struct xrt_device *head,
                        struct pico4_ctrack *ctrack,
                        struct pico4_station *station,
                        enum u_logging_level log_level);

struct xrt_device *
pico4_controller_xdev(struct pico4_controller *c);

//! One parsed radio packet received at @p packet_ns (CLOCK_MONOTONIC).
void
pico4_controller_push(struct pico4_controller *c, const struct pico4_blemcu_packet *p, int64_t packet_ns);

//! The HMD side lets go (the head, the controller tracker and the station are going away).
void
pico4_controller_release(struct pico4_controller *c);

#ifdef __cplusplus
}
#endif
