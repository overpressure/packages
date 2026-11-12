// Copyright 2026, overpressure contributors.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  PICO 4 controller tracking from the LED exposures and the controller IMUs, for the
 *         driver (C interface of the tracker in pico4_ctrack_*.hpp).
 * @ingroup drv_pico4
 */

#pragma once

#include "xrt/xrt_defines.h"
#include "util/u_logging.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

struct pico4_ctrack;
struct pico4_blemcu_packet;

/*!
 * The HMD pose at a past time @p at_ns in the OpenXR convention (world Y up; HMD x right, y up,
 * z back), as the HMD device reports it; false if unknown.
 */
typedef bool (*pico4_ctrack_head_fn)(void *user, int64_t at_ns, struct xrt_pose *out);

/*!
 * Load the camera calibration and the LED models (controllers/led-models.json) from
 * @p calibration_dir and start the tracker's thread. NULL if disabled (PICO4_CTRACK=0) or
 * the calibration is missing.
 */
struct pico4_ctrack *
pico4_ctrack_create(const char *calibration_dir, pico4_ctrack_head_fn head, void *user, enum u_logging_level log_level);

//! The controllers' IMU samples of one radio packet (time of the packet's event, CLOCK_MONOTONIC).
void
pico4_ctrack_push_packet(struct pico4_ctrack *t, const struct pico4_blemcu_packet *p, int64_t packet_ns);

/*!
 * One controller-LED exposure of the four tracking cameras, in factory camera order (cam0..cam3),
 * 8 bit, 640x480, at the middle of the exposure; copied.
 */
void
pico4_ctrack_push_frames(struct pico4_ctrack *t, int64_t timestamp_ns, const uint8_t *const images[4], uint32_t stride);

/*!
 * Aim pose of a controller (PICO4_BLEMCU_LEFT or _RIGHT) at @p at_ns in the HMD's OpenXR world:
 * the pose PICO OS publishes, which it treats as the aim (pico4_aimgrip.h); false while the
 * tracker has no 6DoF pose for it.
 */
bool
pico4_ctrack_get_aim(struct pico4_ctrack *t, int device, int64_t at_ns, struct xrt_space_relation *out);

/*!
 * A controller's per-unit data, as PICO OS loads it on each connect: its calibration record from
 * the station (@p chunks, pico4_station_get_calibration; NULL if the fetch failed) gives the IMU
 * calibration and the LED positions, and @p serial (from the connect notification) picks its gyro
 * bias table, <calibration dir>/pvr/<serial>.txt. @p device is PICO4_BLEMCU_LEFT or _RIGHT.
 * Logs what was applied; parts that are missing or rejected keep their previous values.
 */
void
pico4_ctrack_set_unit(struct pico4_ctrack *t, int device, const uint8_t (*chunks)[32], const char *serial);

/*!
 * Stop solving: no more head-pose callbacks once this returns. Pushes are ignored afterwards and
 * the last poses stay readable until pico4_ctrack_destroy().
 */
void
pico4_ctrack_stop(struct pico4_ctrack *t);

void
pico4_ctrack_destroy(struct pico4_ctrack **t);

#ifdef __cplusplus
}
#endif
