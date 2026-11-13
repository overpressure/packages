// Copyright 2026, overpressure contributors.
// SPDX-License-Identifier: Unlicense
/*
 * PICO OS's see-through depth mesh in pico4-slamd (docs/xr.md#passthrough): libphotoid.so, which
 * PICO OS's see-through service feeds on the PICO 4 with the grey forward camera pair, the head
 * tracker's result at their time and its map points, and which returns a hemisphere mesh around
 * the head in the tracker's world. Its interface as that service uses it (reverse-engineered).
 */
#pragma once

#include <stdint.h>

struct pico4_slam_feed;

//! The tracker's result (libSlam's getAlgorithmResult), as libphotoid reads it.
#define VST_HEAD_DATA_SIZE 0x170

//! One camera image for FeedMrFrameData (0x50 bytes).
struct vst_frame
{
	const uint8_t *data;
	int32_t width, height, stride; //!< Grey: the stride must equal the width.
	int32_t format;                //!< 0: grey 8-bit; 4: NV21 (the RGB camera).
	int64_t timestamp;             //!< The tracker's clock, ns.
	int64_t exposure;              //!< ns.
	float scale;                   //!< 1.
	int32_t camera_id;             //!< 0: left, 1: right (grey pair); 6: RGB.
	const uint8_t *head;           //!< The tracker's result at the image's time (VST_HEAD_DATA_SIZE).
	const uint8_t *uv;             //!< RGB only: the second plane.
	int32_t uv_width, uv_height, uv_stride, pad;
};

//! A libc++ vector of frames, as FeedMrFrameData takes them.
struct vst_frames
{
	struct vst_frame *begin, *end, *cap;
};

//! The tracker's map points (libSlam's point callback; SetSlamPointsCloud): 32008 bytes.
#define VST_SLAM_POINTS 2000
struct vst_slam_points
{
	int32_t count, capacity; //!< capacity: VST_SLAM_POINTS.
	struct
	{
		int32_t id;
		float xyz[3];
	} pts[VST_SLAM_POINTS];
};

/*!
 * Load libphotoid and start its pipeline (/system/etc/pxr/vst_releaseA.json.cpt: depth from the
 * stereo pair by its network on the Compute DSP, a uniform hemisphere mesh). Each new mesh goes
 * into @p feed's mesh slots. 0 on success.
 */
int
vst_start(struct pico4_slam_feed *feed);

//! Feed one stereo pair (left, right: 8-bit, width x height) with the tracker's result at @p ts.
void
vst_feed(const uint8_t *left, const uint8_t *right, int width, int height, int64_t ts, int64_t exposure_ns,
         const uint8_t head[VST_HEAD_DATA_SIZE]);

//! The tracker's map points (from libSlam's point callback), while it tracks.
void
vst_points(const struct vst_slam_points *points);

void
vst_stop(void);
