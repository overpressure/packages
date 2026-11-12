// Copyright 2026, overpressure contributors.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Interface to the PICO 4 driver: the headset, its controllers and its cameras.
 * @ingroup drv_pico4
 */

#pragma once

#include "pico4_passthrough.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*!
 * @defgroup drv_pico4 PICO 4 driver
 * @ingroup drv
 *
 * @brief The original PICO 4: head tracking, lenses, controllers and cameras, with its IMU read
 * through the Qualcomm Sensor Core on the SLPI (QMI over QRTR).
 */

struct xrt_auto_prober;
struct xrt_device;

/*!
 * Auto-prober: creates the HMD on a PICO 4 (an SLPI with an SNS client service).
 *
 * @ingroup drv_pico4
 */
struct xrt_auto_prober *
pico4_create_auto_prober(void);

/*!
 * The inverse of the PICO 4 HMD's distortion function for one channel (0 red,
 * 1 green, 2 blue): the panel UV of @p view that shows render-target UV (u, v).
 * False for other devices or where no panel point shows that UV. SteamVR's Facet
 * renderer builds its mesh from this (IVRDisplayComponent_003).
 *
 * @ingroup drv_pico4
 */
bool
pico4_hmd_compute_inverse_distortion(
    struct xrt_device *xdev, uint32_t view, uint32_t channel, float u, float v, float *out_u, float *out_v);

/*!
 * Whether the PICO 4 HMD is worn, from its wear sensor (the proximity sensor on the sensor DSP,
 * PICO OS's way). False for other devices and until the sensor has reported; SteamVR's user
 * presence (/proximity) comes from this.
 *
 * @ingroup drv_pico4
 */
bool
pico4_hmd_get_presence(struct xrt_device *xdev, bool *out_worn);

/*!
 * The tracking universe of the room the PICO 4 HMD's tracker is in (docs/xr.md#room-setup): a
 * number of its map (never 0 or 1, which OpenVR reserves), so SteamVR keeps a play area per room.
 * False for other devices and until the room is settled.
 *
 * @ingroup drv_pico4
 */
bool
pico4_hmd_get_universe(struct xrt_device *xdev, uint64_t *out_universe);

/*!
 * SteamVR's camera (docs/xr.md#passthrough): the PICO 4 HMD's two forward tracking cameras.
 * False for other devices and without PICO OS's tracker; @p out describes them, upright.
 *
 * @ingroup drv_pico4
 */
bool
pico4_hmd_camera_info(struct xrt_device *xdev, struct pico4_passthrough_info *out);

//! Starts or stops handing the cameras' frames over (pico4_hmd_camera_frame).
void
pico4_hmd_camera_stream(struct xrt_device *xdev, bool on);

/*!
 * Waits up to @p timeout_ms for a frame newer than @p after and writes its luma, left camera
 * above right, into @p y (@p stride bytes a row). False on timeout or for other devices.
 *
 * @ingroup drv_pico4
 */
bool
pico4_hmd_camera_frame(struct xrt_device *xdev,
                       uint64_t after,
                       int timeout_ms,
                       uint8_t *y,
                       uint32_t stride,
                       struct pico4_passthrough_frame *out);

/*!
 * SteamVR's room setup (docs/xr.md#room-setup): the height of the floor PICO OS's tracker
 * detected, in SteamVR's raw tracking space, as the Frame's XR service reports it. False for
 * other devices and while the tracker has detected none.
 *
 * @ingroup drv_pico4
 */
bool
pico4_hmd_floor(struct xrt_device *xdev, float *out_y);

/*!
 * SteamVR's room setup begins a step that detects the floor: PICO OS's tracker detects it again
 * from what the head sees now, as each later pico4_hmd_floor then reports. Nothing for other
 * devices.
 *
 * @ingroup drv_pico4
 */
void
pico4_hmd_detect_floor(struct xrt_device *xdev);

/*!
 * The lens separation now, which pico4-lens can change during a session; false if it is unknown.
 */
bool
pico4_hmd_ipd(struct xrt_device *xdev, float *out_ipd_m);

/*!
 * SteamVR's standby: nobody wears the headset. Its compositor keeps the panels on with black
 * frames and holds the display, so the backlight goes off here.
 */
void
pico4_hmd_standby(struct xrt_device *xdev, bool standby);

/*!
 * SteamVR's 3D room view (docs/xr.md#passthrough): PICO OS's see-through depth mesh newer than
 * mesh @p after, its vertices (x, y, z; up to PICO4_DEPTH_MESH_VERTICES) into @p vertices and its
 * triangles (up to PICO4_DEPTH_MESH_INDICES indices) into @p indices, with its time and its place
 * in SteamVR's raw tracking space. False if there is none newer, or for other devices.
 *
 * @ingroup drv_pico4
 */
bool
pico4_hmd_depth_mesh(
    struct xrt_device *xdev, uint64_t after, float *vertices, uint32_t *indices, struct pico4_depth_mesh_info *out);

#ifdef __cplusplus
}
#endif
