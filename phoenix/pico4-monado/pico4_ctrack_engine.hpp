// Copyright 2026, overpressure contributors.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  The PICO 4 controller tracker per camera frame set (the approach of PICO OS's
 *         controller tracking): order of the hands, acquisition with a confirmation window,
 *         tracking with LED association and refinement over all cameras, re-acquisition, and
 *         the IMU fusion with its gate and lost handling.
 * @ingroup drv_pico4
 */

#pragma once

#include "pico4_ctrack_blob.hpp"
#include "pico4_ctrack_imu.hpp"
#include "pico4_ctrack_init.hpp"

#include <array>
#include <optional>
#include <vector>

namespace pico4::ctrack {

//! What happened to one hand in the last frame set (diagnostics and the replay tool).
struct HandFrameStats
{
	bool has_body = false;     //!< IMU samples for the frame interval
	bool tracked_before = false;
	int method = -1;           //!< -1 none, 0 ID-table P5P, 1 P3P, 2 P5P_New, 3 tracking
	bool init = false;         //!< acquisition ran (not tracked)
	int init_camera = -1;      //!< camera of a successful acquisition
	bool tracking_ok = false;  //!< judgeTrackingOk
	bool few_points = false;   //!< the 2-4 feature translation solver ran
	bool near = false;
	int features = 0;          //!< matched features over all cameras
	int inliers = 0;           //!< errors below the acceptance limit
	double residual_mean = 100.0;
	bool accepted = false;     //!< the fusion gate took the optical pose
	bool tracked_after = false;
	Pose prior;                //!< IMU prediction of a tracked hand at the image time
	Vec3 up_3dof = Vec3::Zero();  //!< up (world +x) in M by the 3DoF orientation of the frame
	Vec3 accel_M = Vec3::Zero();  //!< the 3DoF filter's smoothed accelerometer in M
	Pose T_WM;                 //!< optical pose of the frame (valid if method >= 0)
};

/*!
 * One controller's per-unit data, as PICO OS loads it on connect (alg_controller_init_calibPara,
 * spec 03 3/5, 6/1): the IMU calibration, the LED positions that replace the model's (normals and
 * the identification table stay), the gyro bias table named by the controller's serial.
 */
struct UnitCalibration
{
	bool imu_valid = false;
	ImuCalib imu;
	bool leds_valid = false;
	std::array<Vec3, kLeds> leds{};
	bool bias_valid = false;
	std::array<Vec3, 100> bias{};
};

class Tracker
{
public:
	Tracker(const std::array<Camera, kCameras> &cams, const LedModel &left, const LedModel &right,
	        const FusionParams &params = FusionParams{});

	Tracker(const Tracker &) = delete;
	Tracker &
	operator=(const Tracker &) = delete;

	//! One controller IMU sample; samples up to an image time must arrive before its frame set.
	void
	add_imu(Hand hand, const ImuSample &sample, const Pose *T_WH_at_sample = nullptr);

	//! A controller's per-unit data; only the valid parts are applied.
	void
	set_unit(Hand hand, const UnitCalibration &unit);

	/*!
	 * One frame set: the four tracking-camera images of the LED exposure (640x480, 8 bit, in
	 * camera order), their common timestamp and the head pose at that time.
	 */
	void
	process_frame(int64_t t_ns, const Pose &T_WH, const std::array<ImageView, kCameras> &images);

	//! The published pose of a hand, predicted to @p t_ns.
	OutputPose
	pose_at(Hand hand, int64_t t_ns) const;

	//! The published pose of a hand as of its newest IMU sample.
	const OutputPose &
	output(Hand hand) const
	{
		return hands_[hand].fusion.output();
	}

	const HandFrameStats &
	stats(Hand hand) const
	{
		return hands_[hand].stats;
	}

	bool
	tracked(Hand hand) const
	{
		return hands_[hand].fusion.tracked();
	}

private:
	struct HandState
	{
		Hand hand = kLeft;
		const LedModel *model = nullptr;
		ControllerFusion fusion;
		FramePrior prior;
		Pose pose;                //!< P+0x950: prior, then the frame's solution
		bool pose_valid = false;
		bool near = false;        //!< P+0x19e0
		bool locked = false;      //!< P+0x198c
		bool second = false;      //!< P+0x359: runs after the other tracked hand
		bool juxtaposed = false;  //!< P+0x1b23: the flag of its last acquisition cluster
		int init_failures = 9999; //!< P+0x196c
		double occlusion = 0.0;   //!< P+0x19e8
		std::array<std::vector<Vec2>, kCameras> accepted_px; //!< P+0x620
		std::array<std::vector<Vec2>, kCameras> other_px;    //!< P+0x668
		HandFrameStats stats;

		HandState(Hand h, const LedModel *m, const FusionParams &params) : hand(h), model(m), fusion(h, params) {}
	};

	std::array<Camera, kCameras> cams_;
	LedModel models_[2];
	IdTable table_;
	std::array<HandState, 2> hands_;

	// Frame set being processed.
	int64_t t_ns_ = 0;
	Pose T_WH_;
	std::array<ImageView, kCameras> images_{};
	bool bright_ = false;
	std::array<std::optional<std::vector<BlobPoint>>, kCameras> init_detections_;

	HandState &
	other(const HandState &h)
	{
		return hands_[1 - h.hand];
	}

	InitContext
	context(HandState &h, bool tracked, const Quat &Q);
	std::vector<Vec2>
	predict(const LedModel &model, const Pose &T_WM, int cam) const;
	InitResult
	acquire(HandState &h, bool tracked, const Quat &Q, int &camera);
	void
	detect_around(HandState &h, int cam, const Pose &T_WM, int &n_visible, int &n_matched);
	void
	init_controller_pose(HandState &h);
	void
	update_pose(HandState &h);
	std::vector<Feature>
	match_features(HandState &h);
	bool
	judge_tracking_ok(HandState &h, std::vector<Feature> &feats, Pose &T_WM, std::vector<double> &residuals);
	double
	occlusion_rate(const HandState &h);
};

//! The camera order of a hand's acquisition (left 3, 2, 0, 1; right 2, 3, 1, 0).
const std::array<int, kCameras> &
init_camera_order(Hand hand);

//! Convert a detected blob for the acquisition paths (brightness: 3x3 maximum in @p image).
InitBlob
to_init_blob(const ImageView &image, const BlobPoint &b);

} // namespace pico4::ctrack
