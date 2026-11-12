// Copyright 2026, overpressure contributors.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Shared memory between the PICO 4 driver in SteamVR's vrserver and pico4-roomview, the
 *         producer of SteamVR's room view (docs/xr.md#passthrough).
 *
 * SteamVR 2.17's compositor takes the room view's frames only from a client of its private
 * passthrough interface, not from a driver. The driver owns the cameras, so it publishes the
 * forward grey pair, or the RGB camera's colour image (@ref pico4_roomview::format), here and
 * pico4-roomview hands them on. The driver creates the file (owner only,
 * as the session user) and fills it while pico4-roomview reports that the compositor wants
 * frames (@ref pico4_roomview::requested, with a fresh @ref pico4_roomview::producer_ns). A
 * frame's @ref pico4_roomview_frame::sequence is 0 while the driver writes it and is released
 * after it; a reader copies the frame and checks that the sequence is unchanged. Times are
 * CLOCK_MONOTONIC nanoseconds. Plain C with fixed-size types.
 * @ingroup drv_pico4
 */

#pragma once

#include <stdint.h>

#define PICO4_ROOMVIEW_PATH "/dev/shm/pico4-roomview"
#define PICO4_ROOMVIEW_MAGIC 0x32575652344f4350ull //!< "PCO4RVW2"
#define PICO4_ROOMVIEW_CAMS 2                      //!< Left (factory cam3), right (factory cam2).
#define PICO4_ROOMVIEW_PIXELS (640 * 480)          //!< One grey camera's upright image at most.
#define PICO4_ROOMVIEW_FORMAT_GREY 0               //!< Two 8-bit cameras, the left first.
#define PICO4_ROOMVIEW_FORMAT_NV12 1               //!< One colour camera: NV12, sRGB, full range.
#define PICO4_ROOMVIEW_RGB_WIDTH 1164              //!< The RGB camera's 2x2 cells (2328x1748 / 2).
#define PICO4_ROOMVIEW_RGB_HEIGHT 874
//! A frame's pixels: two grey cameras, or one NV12 image at the RGB camera's size.
#define PICO4_ROOMVIEW_BYTES (PICO4_ROOMVIEW_RGB_WIDTH * PICO4_ROOMVIEW_RGB_HEIGHT * 3 / 2)
#define PICO4_ROOMVIEW_FRAMES 4
#define PICO4_ROOMVIEW_PRODUCER_TIMEOUT_NS 2000000000ll

struct pico4_roomview_frame
{
	uint64_t sequence;    //!< The driver's scene set count, from 1; 0 while written.
	int64_t timestamp_ns; //!< The middle of the exposure.
	uint32_t exposure_us;
	uint32_t pose_valid;
	//! The head at the middle of the exposure in SteamVR's raw tracking space, row-major 3x4.
	double tracking_from_head[12];
	/*!
	 * PICO4_ROOMVIEW_FORMAT_GREY: 8-bit luma, @ref pico4_roomview::width a row, the left
	 * camera's height rows, then the right camera's. PICO4_ROOMVIEW_FORMAT_NV12: the luma
	 * rows, then height / 2 rows of interleaved Cb, Cr (width bytes each).
	 */
	uint8_t pixels[PICO4_ROOMVIEW_BYTES];
};

struct pico4_roomview
{
	uint64_t magic;      //!< Written last when the driver (re)initialises the file.
	uint32_t version;    //!< 2.
	uint32_t generation; //!< Bumped at each driver start.
	uint32_t width, height;
	//! Kannala-Brandt intrinsics of the upright images in pixels (OpenVR's FTheta: k1-k4); NV12: camera 0.
	double fx[PICO4_ROOMVIEW_CAMS], fy[PICO4_ROOMVIEW_CAMS], cx[PICO4_ROOMVIEW_CAMS], cy[PICO4_ROOMVIEW_CAMS];
	double k[PICO4_ROOMVIEW_CAMS][4];
	//! Each camera in the HMD frame, OpenVR's camera axes, row-major 3x4 (HmdMatrix34_t).
	double camera_to_head[PICO4_ROOMVIEW_CAMS][12];
	int64_t producer_ns; //!< pico4-roomview's last activity.
	uint32_t requested;  //!< Written by pico4-roomview: the compositor wants frames.
	uint32_t format;     //!< PICO4_ROOMVIEW_FORMAT_*.
	uint64_t frames_written;
	struct pico4_roomview_frame frames[PICO4_ROOMVIEW_FRAMES];
};
