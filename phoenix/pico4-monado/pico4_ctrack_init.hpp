// Copyright 2026, overpressure contributors.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Acquisition of a controller from one camera's blob cluster (the approach of PICO OS's
 *         controller tracking): the prior-rotation LED-triple search (P5P_New and P3P), the
 *         identification-table path (P5P) and the plausibility tests they share.
 * @ingroup drv_pico4
 */

#pragma once

#include "pico4_ctrack_model.hpp"
#include "pico4_ctrack_solve.hpp"

#include <functional>
#include <vector>

namespace pico4::ctrack {

//! A blob as the initialisers need it.
struct InitBlob
{
	Vec2 px;
	double size = 0;        //!< 2 x median contour distance from the centre
	double area = 0;        //!< contour polygon area
	double brightness = 0;  //!< 3x3 maximum grey around the centre
	double circularity = 0;
};

/*!
 * Cross-camera check of checkPnPByProjectArea: for camera `cam` and pose T_WM, the number of
 * facing LEDs predicted in the image and how many have a detection within 2 px.
 */
using DetectAround = std::function<void(int cam, const Pose &T_WM, int &n_visible, int &n_matched)>;

//! The other hand, as far as the initialisers look at it.
struct OtherHand
{
	//! has a Body this frame: P+0x1940, set by creatBody on success (0x305bd4) and cleared on
	//! each of its failures (0x3048f8, 0x304cac, 0x3052dc, 0x305764)
	bool active = false;
	bool locked = false;     //!< tracked and confirmed (P+0x198c)
	bool pose0_valid = false;
	Pose pose0;              //!< its last pose T_WM (P+0x950)
	Quat prior;              //!< its P+0x950 rotation while tracked (P+0x358), else its 3DoF (0x385080, 0x378e00)
	const LedModel *model = nullptr;
	Hand hand = kLeft;
	double speed = 0;        //!< |SSF velocity|
	/*
	 * Its own state. Stock fits the same blobs with the other hand by calling solveP3PWithPriorR,
	 * checkPnPByProjectArea and checkPromoteP3P on the other predictor (P+0x1938: 0x3853f8,
	 * 0x37934c, 0x37938c, 0x3793d0), so these read its fields, not this hand's.
	 */
	bool tracked = false;          //!< its P+0x358
	bool body_window_empty = true; //!< its P+0xc58 == 0
	int lost_frames = 0;           //!< its P+0x1964
	Quat q_3dof;                   //!< its P+0x460
	bool juxtaposed = false;       //!< its P+0x1b23, as its last cluster left it
	double occlusion = 0;          //!< its P+0x19e8
	DetectAround detect_around;    //!< its cross-camera check
};

//! Everything about this hand and frame the initialisers read.
struct InitContext
{
	Hand hand = kLeft;
	const LedModel *model = nullptr;
	const std::array<Camera, kCameras> *cams = nullptr;
	Pose T_WH;               //!< head pose at the image time
	Quat q_ref;              //!< reference rotation for the angle checks (6.4)
	bool tracked = false;    //!< P+0x358
	bool pose0_valid = false;
	Pose pose0;              //!< P+0x970 / the stored pose
	bool body_window_empty = true;
	int lost_frames = 0;     //!< P+0x1964
	int init_failures = 0;   //!< P+0x196c
	/*!
	 * P+0x6c1, the 30 degree angle limits. Always false on phoenix: its only setters are a gyro
	 * range event in neo3 branches (creatBody 0x3050a8, resetTrackingIfAbnormalState 0x35cd60,
	 * both behind the "neo3" type test); the constructor clears it (0x2f5e64).
	 */
	bool wide_angles = false;
	bool no_p3p = false;     //!< P+0x1975
	double occlusion = 0;    //!< P+0x19e8
	bool juxtaposed = false; //!< P+0x1b23, set per cluster by the caller
	Quat q_3dof;             //!< P+0x460, the 3DoF orientation
	bool follow = false;     //!< P+0x198e: following the other hand while lost
	Vec3 last_good_p = Vec3::Zero(); //!< P+0x13a0: position of the last good published pose
	OtherHand other;
	DetectAround detect_around; //!< optional
};

/*!
 * The context of the other hand's fit of the same blobs with its prior @p Q: its own state, with
 * the reference rotation solveP3PWithPriorR derives from it (0x37acdc..0x37ad1c).
 */
InitContext
other_hand_context(const InitContext &ctx, const Quat &Q);

//! Result of an acquisition.
struct InitResult
{
	bool ok = false;
	Pose T_WM;
	std::vector<int> ids; //!< LED per blob of the cluster (-1 unassigned)
	double error_px = 0;
	double angle_deg = 0;
	int type = -1;        //!< 2 P5P_New, 0 P5P (ID table), 1 P3P
	bool reset_tracking = false; //!< the ID-table path accepted an exact fit with a wrong IMU tilt
};

//! The three-LED search with the rotation fixed to Q (solveP3PWithPriorR); mode 5 or 3.
bool
solve_p3p_with_prior(const InitContext &ctx, const LedModel &model, const Quat &Q, int cam, const Vec2 px[3],
                     const std::vector<int> &candidates, int mode, Vec3 &t_WM, double &error_px, int ids[3],
                     double &angle_deg);

//! Photometric and footprint plausibility of a pose against the image (checkPnPByProjectArea).
bool
check_project_area(const InitContext &ctx, const LedModel &model, const Pose &T_WM, double blob_size,
                   const std::vector<InitBlob> &blobs, const std::vector<int> &ids, int cam, double E,
                   double angle, int mode);

//! Rejects a jump onto the other controller when both arms were far apart (raiseBothHandsSideways).
bool
raise_both_hands_sideways(const InitContext &ctx, const Pose &X);

//! Near the head (judgeNear).
bool
judge_near(const InitContext &ctx, const Pose &T_WM);

//! >= 5 blobs, prior rotation (initPoseUsingP5P_New).
InitResult
init_p5p_new(const InitContext &ctx, int cam, std::vector<InitBlob> blobs, const Quat &Q);

/*!
 * >= 5 blobs, identification table (initPoseUsingP5P). @p original is the cluster before the
 * redundant blobs were removed ("blobs2"), which must agree with the pose.
 */
InitResult
init_p5p_table(const InitContext &ctx, const IdTable &table, int cam, const std::vector<InitBlob> &cluster,
               const std::vector<InitBlob> &original, const Quat &Q);

//! 3 or 4 blobs (initPoseUsingP3P).
InitResult
init_p3p(const InitContext &ctx, int cam, std::vector<InitBlob> blobs, Quat Q);

//! Area of the minimum-area rectangle around points (rotating calipers on the convex hull).
double
min_area_rect_area(std::vector<Vec2> pts);

} // namespace pico4::ctrack
