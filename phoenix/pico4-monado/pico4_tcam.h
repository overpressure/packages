// Copyright 2026, overpressure contributors.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Factory calibration of the PICO 4 tracking cameras, refitted for Basalt.
 *
 * The factory data (pvr/mav0/camN/sensor.yaml, Kalibr style) describes each
 * OV7251 with the unified omnidirectional model plus radial-tangential
 * distortion and gives its pose in the IMU ("body") frame, which is the raw
 * axis frame of the sensor DSP's accelerometer and gyroscope. Basalt has no
 * omni model, so pico4_tcam_fit_kb4() fits the Kannala-Brandt model (KB4) to
 * rays projected with the factory model across the image. The RGB camera (cam4)
 * has the omni model with PICO's "fisheye62" distortion instead, which SteamVR's
 * room view gets through the same fit.
 *
 * @ingroup drv_pico4
 */

#pragma once

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define PICO4_TCAM_COUNT 4

//! Distortion after the omni step (sensor.yaml's distortion_model).
enum pico4_tcam_distortion
{
	PICO4_TCAM_RADTAN = 0,    //!< k1, k2 on r^2 of the omni-normalised point; p1, p2.
	PICO4_TCAM_FISHEYE62 = 1, //!< k1..k6 on theta = atan(r) of that point; p1, p2 (PICO OS's own form).
};

//! Unified omnidirectional camera with radial-tangential or fisheye62 distortion.
struct pico4_tcam_omni
{
	int width, height;
	enum pico4_tcam_distortion distortion;
	double xi, fu, fv, pu, pv;
	double k1, k2, p1, p2;
	double k3, k4, k5, k6; //!< fisheye62 only.
	//! Camera to body (IMU) transform, row-major 4x4 (Kalibr T_BS).
	double T_BS[16];
};

//! Kannala-Brandt camera: d(t) = t (1 + k1 t^2 + k2 t^4 + k3 t^6 + k4 t^8).
struct pico4_tcam_kb4
{
	double fx, fy, cx, cy;
	double k[4];
	//! Fit quality against the factory model, pixels.
	double rms_px, max_px;
	int samples;
};

//! Parse one sensor.yaml; false unless it is an omni camera (radtan or fisheye62) with T_BS.
bool
pico4_tcam_parse_yaml(const char *text, struct pico4_tcam_omni *out);

//! Project a point in camera coordinates (z forward); false if behind the model's horizon.
bool
pico4_tcam_omni_project(const struct pico4_tcam_omni *c, const double p[3], double uv[2]);

void
pico4_tcam_kb4_project(const struct pico4_tcam_kb4 *c, const double p[3], double uv[2]);

/*!
 * Fit a KB4 model with the same principal point to the factory model over the
 * rays that land in the image. Returns false if the fit does not converge.
 */
bool
pico4_tcam_fit_kb4(const struct pico4_tcam_omni *c, struct pico4_tcam_kb4 *out);

/*!
 * Describe the same camera with its image turned by 180 degrees (pixel (u, v)
 * read at (width - 1 - u, height - 1 - v)): the principal point moves and the
 * camera frame turns about its optical axis.
 */
void
pico4_tcam_rotate_180(struct pico4_tcam_kb4 *kb4, double T_BS[16], int width, int height);

/*!
 * The quarter turn (0-3, counter-clockwise as seen on screen) that brings the camera's image
 * upright: its "down" (the camera's y axis) closest to @p down_body (a body-frame direction).
 * Turn 1 reads pixel (u', v') at (v', h - 1 - u'), turn 3 at (w - 1 - v', u'); turns 1 and 3
 * swap width and height. @p kb4 and @p T_BS change to describe the turned image; the image's
 * new size goes to @p out_w and @p out_h.
 */
int
pico4_tcam_upright(struct pico4_tcam_kb4 *kb4, double T_BS[16], int width, int height, const double down_body[3],
                   int *out_w, int *out_h);

/*!
 * Whether camera b's image is upside down relative to camera a's (their image
 * y axes point more apart than together). Basalt's stereo matching tracks
 * patches between cameras and cannot follow a 180 degree turn.
 */
bool
pico4_tcam_upside_down(const double a_T_BS[16], const double b_T_BS[16]);

#ifdef __cplusplus
}
#endif
