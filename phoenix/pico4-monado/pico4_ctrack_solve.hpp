// Copyright 2026, overpressure contributors.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Pose solvers and LED association of the PICO 4 controller tracker (the approach of
 *         PICO OS's controller tracking): translation with a known rotation, the prior-rotation
 *         LED-triple search, P3P, EPnP, the shift-hypothesis association and the robust
 *         Levenberg-Marquardt refinement over all cameras.
 * @ingroup drv_pico4
 */

#pragma once

#include "pico4_ctrack_model.hpp"

#include <vector>

namespace pico4::ctrack {

/*!
 * One observation for the translation solvers: camera pose in W (rotation R_WC, centre c), the
 * unit bearing f in that camera, and the LED position P in M.
 */
struct RayObservation
{
	Mat3 R_WC;
	Vec3 c;
	Vec3 f;
	Vec3 P;
};

/*!
 * Translation of T_WM for a known rotation R_WM from two observations (solvetWoIWithPriorR_P2P_general):
 * unknowns t and the two depths; false if a depth is below 0.005 m.
 */
bool
translation_p2p(const Mat3 &R_WM, const RayObservation &a, const RayObservation &b, Vec3 &t);

//! The same from three observations (solvetWoIWithPriorR_general): depth limits 0.003 (first), 0.005 m.
bool
translation_p3(const Mat3 &R_WM, const RayObservation obs[3], Vec3 &t);

/*!
 * All P3P solutions (camera point = R * model point + t) for three unit bearings in the camera
 * frame and three model points.
 */
std::vector<Pose>
p3p(const Vec3 f[3], const Vec3 P[3]);

/*!
 * EPnP for >= 4 correspondences of normalised image points (x/z, y/z) and model points, with
 * Gauss-Newton refinement of the betas: camera <- model pose. False if degenerate.
 */
bool
epnp(const std::vector<Vec2> &uv, const std::vector<Vec3> &P, Pose &out);

//! Half-angle in degrees between the "up" (W +x) of two orientations in M: tilt only (calAngleError).
double
tilt_half_angle_deg(const Quat &a, const Quat &b);

//! Half the full relative rotation angle in degrees, folded to <= 90 (the inline P3P angle).
double
half_angle_deg(const Quat &a, const Quat &b);

//! Full rotation angle in degrees (checkOptimizeResult).
double
full_angle_deg(const Quat &a, const Quat &b);

/*!
 * Shift-hypothesis association of projected LEDs and blobs in one camera (NewMatchPoints).
 * @param proj   projected pixels of the facing LEDs, most frontal first.
 * @param blobs  blob centres.
 * @param match  per blob: index into proj, or -1.
 * @param err    per blob: distance to its (unshifted) projection, 100 if unassigned.
 */
void
match_points(const std::vector<Vec2> &proj, const std::vector<Vec2> &blobs, std::vector<int> &match,
             std::vector<double> &err);

//! One matched LED for the refinement: camera, unit bearing of the blob, LED position in M.
struct Feature
{
	int cam = 0;
	Vec3 bearing;
	Vec3 P;
	int led = -1;
	Vec2 pixel;
	double residual_px = 0; //!< after the refinement
	bool enabled = true;
};

/*!
 * Robust Levenberg-Marquardt refinement of T_HM over all cameras (PoseOptimizer): unit-sphere
 * tangent residuals, Tukey weights with a scale from the median initial error, and a soft prior
 * on the rotation (weight 0.1). Writes each feature's final error in pixels.
 * @return false if the system became singular.
 */
bool
refine_pose(const std::array<Camera, kCameras> &cams, std::vector<Feature> &feats, const Pose &prior_HM,
            Pose &T_HM, double &median_initial_px, double &median_final_px);

/*!
 * Translation with the prior rotation from 2-4 features over all cameras
 * (solvePnPWithPriorR_general_near): best pair and best triple by inliers, then mean error.
 * @param T_WH  head pose; features carry camera indices into cams.
 * @return false if no hypothesis has an inlier.
 */
bool
few_points_translation(const std::array<Camera, kCameras> &cams, const Pose &T_WH, const Quat &R_WM,
                       std::vector<Feature> &feats, bool near, Vec3 &t_WM, std::vector<double> &errors_px);

} // namespace pico4::ctrack
