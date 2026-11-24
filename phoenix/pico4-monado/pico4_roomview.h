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
 * after it; a reader copies the frame and checks that the sequence is unchanged. The wearer's
 * double tap on the headset (@ref pico4_roomview::taps, found by pico4_tap.h in the driver) is
 * counted here too, and pico4-roomview switches the room view for each. Times are
 * CLOCK_MONOTONIC nanoseconds. Plain C with fixed-size types.
 * @ingroup drv_pico4
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#define PICO4_ROOMVIEW_PATH "/dev/shm/pico4-roomview"
#define PICO4_ROOMVIEW_MAGIC 0x33575652344f4350ull //!< "PCO4RVW3"
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
//! A double tap older than this when pico4-roomview gets to it (the compositor was away) is not
//! switched for.
#define PICO4_ROOMVIEW_TAP_FRESH_NS 2000000000ll
//! How long the compositor has to show the room view it was told to (a double tap).
#define PICO4_ROOMVIEW_AGREEMENT_NS 2000000000ll

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
	uint32_t version;    //!< 3.
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
	/*!
	 * The wearer's double taps on the headset while worn, as PICO OS's passthrough switch: the
	 * driver counts them from the first of each driver start and stamps the last with the time of
	 * its second knock. Last in the struct: a pico4-roomview of the version before this one reads
	 * a driver that has them.
	 */
	uint32_t taps;
	int64_t last_tap_ns;
};

/*!
 * What pico4-roomview knows of the room view between the compositor's answers: after a double
 * tap told the compositor to switch, its answer follows within a moment, and a second double tap
 * in that moment switches from what it was told, not from the answer that is still to come.
 */
struct pico4_roomview_switch
{
	bool shown;      //!< The room view is shown, as far as this knows.
	int64_t told_ns; //!< When the compositor was last told; before PICO4_ROOMVIEW_AGREEMENT_NS: it agrees.
};

enum pico4_roomview_agreement
{
	PICO4_ROOMVIEW_AGREES = 0,  //!< No switch waits for the compositor's answer.
	PICO4_ROOMVIEW_WAITING = 1, //!< It has not answered the last switch yet.
	PICO4_ROOMVIEW_REFUSED = 2, //!< It did not switch in PICO4_ROOMVIEW_AGREEMENT_NS: its answer counts.
};

//! Takes the compositor's answer @p requested at @p now_ns: whether it agrees with the last switch.
static inline enum pico4_roomview_agreement
pico4_roomview_switch_follow(struct pico4_roomview_switch *s, bool requested, int64_t now_ns)
{
	if (s->told_ns == 0 || requested == s->shown) {
		s->shown = requested;
		s->told_ns = 0;
		return PICO4_ROOMVIEW_AGREES;
	}
	if (now_ns - s->told_ns > PICO4_ROOMVIEW_AGREEMENT_NS) {
		s->shown = requested;
		s->told_ns = 0;
		return PICO4_ROOMVIEW_REFUSED;
	}
	return PICO4_ROOMVIEW_WAITING;
}

//! A double tap at @p now_ns: the room view the compositor is to show from now.
static inline bool
pico4_roomview_switch_toggle(struct pico4_roomview_switch *s, int64_t now_ns)
{
	s->shown = !s->shown;
	s->told_ns = now_ns;
	return s->shown;
}
