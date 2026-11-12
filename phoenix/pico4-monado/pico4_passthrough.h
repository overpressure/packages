// Copyright 2026, overpressure contributors.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  PICO 4 passthrough: the two forward tracking cameras' scene frames as one upright,
 *         stacked 8-bit image for SteamVR's camera (docs/xr.md#passthrough).
 *
 * The lower pair of tracking cameras looks forward (about 40 degrees down); their images lie
 * on their sides. The camera thread hands each complete scene set's two raw images over while
 * a client streams; the client's thread turns them upright and converts them, so the camera
 * thread only copies. Plain C without Monado types: the host tests build it.
 * @ingroup drv_pico4
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define PICO4_PASSTHROUGH_CAMS 2 //!< Left (factory cam3), right (factory cam2).
#define PICO4_PASSTHROUGH_SRC_W 640
#define PICO4_PASSTHROUGH_SRC_H 480

//! The two cameras, upright, as SteamVR describes cameras.
struct pico4_passthrough_info
{
	uint32_t width, height; //!< One camera's upright image; the frame stacks left above right.
	int turn[PICO4_PASSTHROUGH_CAMS]; //!< pico4_tcam_upright's quarter turn of each camera.
	//! Kannala-Brandt intrinsics of the upright images (OpenVR's FTheta: k1-k4).
	double fx[PICO4_PASSTHROUGH_CAMS], fy[PICO4_PASSTHROUGH_CAMS];
	double cx[PICO4_PASSTHROUGH_CAMS], cy[PICO4_PASSTHROUGH_CAMS];
	double k[PICO4_PASSTHROUGH_CAMS][4];
	/*!
	 * Each camera in the HMD frame (x right, y up, z back), row-major 3x4 as OpenVR's
	 * HmdMatrix34_t, in OpenVR's camera axes (x right, y up, looking along -z).
	 */
	double camera_to_head[PICO4_PASSTHROUGH_CAMS][12];
};

//! One delivered frame.
struct pico4_passthrough_frame
{
	uint64_t sequence;     //!< Counts the scene sets handed over, from 1.
	int64_t timestamp_ns;  //!< CLOCK_MONOTONIC, the middle of the exposure.
	uint32_t exposure_us;  //!< The left camera's exposure.
};

/*!
 * PICO OS's see-through depth mesh (docs/xr.md#passthrough): the room around the head as its
 * libphotoid computes it from each pair of the forward cameras in pico4-slamd. Triangles of three
 * indices each, counter-clockwise seen from the head; at most a 128 x 128 hemisphere grid.
 */
#define PICO4_DEPTH_MESH_VERTICES 16384
#define PICO4_DEPTH_MESH_INDICES (6 * 127 * 127)
struct pico4_depth_mesh_info
{
	uint64_t id;          //!< Counts the meshes, from 1.
	int64_t timestamp_ns; //!< CLOCK_MONOTONIC, the middle of the pair's exposure.
	uint32_t vertex_count, index_count;
	//! The vertices' space in SteamVR's raw tracking space, row-major 3x4 (HmdMatrix34_t).
	double tracking_from_mesh[12];
};

struct pico4_passthrough;

/*!
 * The forward pair from the factory calibration in @p calibration_dir (pvr/mav0/camN/sensor.yaml);
 * NULL if it is missing. @p down_body and @p T_body_head (row-major 4x4, the HMD frame in the
 * body frame) place the cameras in the HMD frame.
 */
struct pico4_passthrough *
pico4_passthrough_create(const char *calibration_dir, const double down_body[3], const double T_body_head[16]);

void
pico4_passthrough_info(const struct pico4_passthrough *p, struct pico4_passthrough_info *out);

//! A client streams (or stops): only then, or while the room view takes frames, the camera thread hands frames over.
void
pico4_passthrough_set_streaming(struct pico4_passthrough *p, bool on);

bool
pico4_passthrough_streaming(const struct pico4_passthrough *p);

/*!
 * One scene set's raw images of the left and right camera (10-bit in 16, the sensor's rows as
 * read out, @p stride samples apart), its time and exposure, and the brightness gain for 8 bits.
 * Camera thread; skips the set when the client is converting the one before.
 */
void
pico4_passthrough_push(struct pico4_passthrough *p,
                       const uint16_t *left,
                       const uint16_t *right,
                       uint32_t stride,
                       int64_t timestamp_ns,
                       uint32_t exposure_lines,
                       float gain);

/*!
 * Waits up to @p timeout_ms for a set newer than @p after and writes it as 8-bit luma into @p y
 * (@p stride bytes a row; left camera on top). False on timeout.
 */
bool
pico4_passthrough_wait(struct pico4_passthrough *p,
                       uint64_t after,
                       int timeout_ms,
                       uint8_t *y,
                       uint32_t stride,
                       struct pico4_passthrough_frame *out);

//! The head pose at @p timestamp_ns in SteamVR's raw tracking space, row-major 3x4; false if unknown.
typedef bool (*pico4_passthrough_pose_fn)(void *ctx, int64_t timestamp_ns, double out_tracking_from_head[12]);

/*!
 * SteamVR's room view (pico4_roomview.h): creates the shared memory at @p path and starts a thread
 * that streams the forward pair into it while pico4-roomview asks for frames. Once per @p p; true
 * if it runs.
 */
bool
pico4_passthrough_export(struct pico4_passthrough *p, const char *path, pico4_passthrough_pose_fn pose, void *ctx);

//! Turns one raw image upright (pico4_tcam_upright's @p turn) into 8 bits: black level 16 off, times @p gain.
void
pico4_passthrough_convert(const uint16_t *src, uint32_t src_stride, int turn, float gain, uint8_t *dst,
                          uint32_t dst_stride);

/*
 * The RGB camera (IMX471, factory cam4) for SteamVR's room view, as PICO OS colours its
 * see-through: its colour image, 2x2 RGGB cells as one pixel (1164x874), in place of the grey
 * pair. SteamVR's camera component keeps the grey pair (pico4_passthrough_info).
 */
#define PICO4_PASSTHROUGH_RGB_SRC_W 2328
#define PICO4_PASSTHROUGH_RGB_SRC_H 1748
#define PICO4_PASSTHROUGH_RGB_W (PICO4_PASSTHROUGH_RGB_SRC_W / 2)
#define PICO4_PASSTHROUGH_RGB_H (PICO4_PASSTHROUGH_RGB_SRC_H / 2)
#define PICO4_PASSTHROUGH_RGB_BLACK 64 //!< 10-bit black level.

/*!
 * The room view takes the RGB camera from now on: its factory calibration (cam4, omni with
 * fisheye62) fitted to KB4 at the 2x2 cells, placed in the HMD frame. Before
 * pico4_passthrough_export; false (and the grey pair stays) without that calibration.
 */
bool
pico4_passthrough_use_rgb(struct pico4_passthrough *p, const char *calibration_dir, const double T_body_head[16]);

//! The room view's RGB camera as SteamVR describes it (camera 0; camera 1 the same); false in grey mode.
bool
pico4_passthrough_rgb_info(const struct pico4_passthrough *p, struct pico4_passthrough_info *out);

/*!
 * One RGB frame (RGGB, 10-bit in 16, @p stride samples a row), its start of frame and exposure.
 * Camera thread. True if taken: the buffer must then stay valid until pico4_passthrough_rgb_busy()
 * is false. False when nobody streams or the frame before is still being converted.
 */
bool
pico4_passthrough_push_rgb(struct pico4_passthrough *p,
                           const uint16_t *rggb,
                           uint32_t stride,
                           int64_t sof_ns,
                           uint32_t exposure_us);

//! An RGB frame handed over is still being read.
bool
pico4_passthrough_rgb_busy(struct pico4_passthrough *p);

//! The RGB frames handed over and converted so far, and the conversion time spent on them.
void
pico4_passthrough_rgb_counts(struct pico4_passthrough *p, uint64_t *taken, uint64_t *converted, int64_t *convert_ns);

/*!
 * The RGB camera's next exposure (lines of PICO4_PASSTHROUGH_RGB_LINE_NS, analog gain 16 = 1x)
 * from the newest converted frame, given the exposure it was taken with. True if it should
 * change; at most once per new frame.
 */
#define PICO4_PASSTHROUGH_RGB_LINE_NS 4425
bool
pico4_passthrough_rgb_exposure(struct pico4_passthrough *p,
                               unsigned lines,
                               unsigned gain,
                               unsigned *out_lines,
                               unsigned *out_gain);

/*!
 * RGGB 10-bit cells (@p src, @p src_stride samples a row) into NV12 (@p width x @p height, the
 * luma rows then the Cb, Cr rows, @p width bytes each): black level off, times the white balance
 * @p wb (R, G, B), through @p srgb (linear counts 0-1023 to sRGB 8 bits), full-range BT.601. The
 * channels' means before the white balance (linear counts) go to @p mean.
 */
void
pico4_passthrough_convert_rgb(const uint16_t *src,
                              uint32_t src_stride,
                              const float wb[3],
                              const uint8_t srgb[1024],
                              uint8_t *dst,
                              uint32_t width,
                              uint32_t height,
                              double mean[3]);

//! The sRGB encoding of linear counts (0 to @p white) as pico4_passthrough_convert_rgb takes it.
void
pico4_passthrough_srgb_table(uint8_t table[1024], double white);

void
pico4_passthrough_destroy(struct pico4_passthrough **p);

#ifdef __cplusplus
}
#endif
