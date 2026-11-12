// Copyright 2026, overpressure contributors.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Controller IMU side of the PICO 4 controller tracker (the approach of PICO OS's
 *         controller tracking): sample intake and calibration, the 3DoF attitude filter, the
 *         15-state error-state filter fused with the optical poses, the per-frame prior, the
 *         fusion gate, loss handling and the predicted IMU-rate output pose.
 *
 * Section references ("spec 03 4/3.12") point into the private behavioural specification the
 * code was written from. All times inside are seconds on the camera clock; W has +x up.
 *
 * Threading: nothing here locks. The engine serialises every call of both hands (one lock for
 * both, since a hand reads its other hand's state).
 *
 * @ingroup drv_pico4
 */

#pragma once

#include "pico4_ctrack_types.hpp"

#include <array>
#include <deque>
#include <vector>

namespace pico4::ctrack {

using Mat15 = Eigen::Matrix<double, 15, 15>;
using Vec15 = Eigen::Matrix<double, 15, 1>;
using Mat6 = Eigen::Matrix<double, 6, 6>;
using Vec6 = Eigen::Matrix<double, 6, 1>;

//! Gravity of the IMU propagation (spec 03 5/2): W +x up, slightly different from kGravityW.
inline const Vec3 kGravityImu{9.8065, 0.0, 0.0};

/*
 *
 * Calibration
 *
 */

/*!
 * Per-unit IMU calibration (spec 03 3/3): calibrated = M (raw - b) in the raw sensor axes, then
 * the fixed permutation to M. Identity and zero by default: no record, no compiled fallback.
 */
struct ImuCalib
{
	Mat3 M_gyro = Mat3::Identity();
	Vec3 b_gyro = Vec3::Zero();
	Mat3 M_accel = Mat3::Identity();
	Vec3 b_accel = Vec3::Zero();

	/*!
	 * Load the vector form (spec 03 3/3.4): 9 matrix elements row-major, then 3 biases, for
	 * gyro and accelerometer. Every element must be finite and <= 2.0; otherwise false and the
	 * old values stay.
	 */
	bool
	load(const double gyro12[12], const double accel12[12]);

	//! Calibrated sample in M (spec 03 3/3.5): gyro rad/s, accel m/s^2.
	void
	apply(const Vec3 &gyro_raw, const Vec3 &accel_raw, Vec3 &gyro_M, Vec3 &accel_M) const;
};

/*
 *
 * 3DoF attitude filter
 *
 */

/*!
 * picosensor::OrientationController (spec 03 3/4): RK4 gyro integration, gravity window with
 * tilt correction towards W +x, online temperature-binned gyro bias, stillness, angular
 * acceleration and smoothed accelerometer. Yaw is free until corrected from the optical side.
 */
class OrientationController
{
public:
	//! @p gyro_window samples of gyro history, @p gravity_window samples of gravity window (spec 03 3/4.1).
	OrientationController(int gyro_window, int gravity_window);

	//! One sample in M (spec 03 3/4.3). False if the gyro has a NaN (sample ignored).
	bool
	track(const Vec3 &accel, const Vec3 &gyro, double t, double temperature);

	/*!
	 * applyTotalCorrect (spec 03 3/4.8): mode 0 replaces the attitude by @p q_ext, mode 1 corrects
	 * only the twist about W +x (yaw).
	 */
	void
	apply_correction(const Quat &q_ext, int mode);

	//! Rotate the attitude by @p d on the left (updatePose 0x3b3184..0x3b4550, OC+0x60).
	void
	rotate(const Quat &d)
	{
		q_ = d * q_;
	}

	//! Force a re-initialisation at the next sample (resetTrackingIfAbnormalState RESET_A).
	void
	invalidate()
	{
		initialized_ = false;
	}

	//! Set the initialized byte OC+0x00 (addSingleImu 0x361af0..0x361b00, after the yaw correction).
	void
	mark_initialized()
	{
		initialized_ = true;
	}

	Quat
	attitude() const
	{
		return q_.normalized();
	}
	//! Gravity-window confidence (OC+0x50): clamp(0.48 - 0.1 ln sigma, 0, 1) * n / N.
	double
	confidence() const
	{
		return con_;
	}
	//! Online bias valid: the gyro has been still for > 950 samples (spec 03 3/4.6); still3dof.
	bool
	still() const
	{
		return bias_valid_;
	}
	const Vec3 &
	angular_acceleration() const
	{
		return ang_acc_;
	}
	const Vec3 &
	smoothed_accel() const
	{
		return acc_s_;
	}
	bool
	initialized() const
	{
		return initialized_;
	}
	//! Gyro bias subtracted for the last sample.
	const Vec3 &
	bias() const
	{
		return bias_used_;
	}

	/*!
	 * Temperature-binned gyro bias table (spec 03 3/4.7), one entry per degree C 0..99; zero is
	 * the empty marker. Stock persists it per controller serial; here the engine may.
	 */
	const std::array<Vec3, 100> &
	bias_table() const
	{
		return table_;
	}
	//! Load a persisted table: entries above 0.15 rad/s are dropped (readBias).
	void
	set_bias_table(const std::array<Vec3, 100> &table);

private:
	void
	reset();
	void
	online_bias(const Vec3 &gyro_raw);
	void
	store_bias();
	Vec3
	current_bias() const;
	void
	gravity_update(const Vec3 &a, double dt, const Quat &dq);
	void
	tilt_correction(double dt);
	double
	quality() const;

	int gyro_cap_;
	int grav_cap_;
	bool initialized_ = false;
	double t_prev_ = 0.0;
	Quat q_ = Quat::Identity();
	double con_ = 0.0;
	Vec3 ang_acc_ = Vec3::Zero();
	Vec3 acc_s_ = Vec3::Zero();
	int mismatch_ = 0;
	double window_start_ = 0.0;
	std::deque<Vec3> gyro_hist_;
	std::deque<Vec3> acc_hist_;
	unsigned counter_ = 0;

	// Gravity estimator (picosensor::LowBandFilter).
	Quat g_qacc_ = Quat::Identity();
	Vec3 g_f_ = Vec3::Zero();
	std::deque<Vec3> g_win_;
	Vec3 g_sum_ = Vec3::Zero();
	double g_sumsq_ = 0.0;

	// Gyro bias.
	double temperature_ = 25.0;
	std::deque<Vec3> still_buf_;
	Vec3 still_sum_ = Vec3::Zero();
	Vec3 online_bias_ = Vec3::Zero();
	bool bias_valid_ = false;
	Vec3 bias_used_ = Vec3::Zero();
	std::array<Vec3, 100> table_;
	std::vector<int> bins_;
};

/*
 *
 * Error-state filter
 *
 */

/*!
 * ssf_core::SSF_EKF_Filter (spec 03 5/4): state p, v, q (M -> W), gyro bias, accel bias; error
 * state dp, dv, dtheta, db_g, db_a with the attitude error in W (q = exp(dtheta) q:
 * TAOPropagation 0x259474..0x259678, TAOChiSquare 0x25fa3c, TAOApplyCorrection 0x25b6cc).
 */
class SsfFilter
{
public:
	Vec3 p = Vec3::Zero();
	Vec3 v = Vec3::Zero();
	Quat q = Quat::Identity();
	Vec3 bg = Vec3::Zero();
	Vec3 ba = Vec3::Zero();
	Mat15 P = Mat15::Zero();

	bool initialized = false;
	double last_t = 0.0;
	Vec3 a_prev = Vec3::Zero();
	Vec3 w_prev = Vec3::Zero();

	//! Convergence mode 0/1/2 (spec 03 5/4.5).
	int mode = 0;
	//! Window statistics of the current camera frame (spec 03 5/4.6).
	double max_gyro = 0.0;
	double max_lin_acc = 0.0;
	bool restricted = false;
	bool inflate_accel = false; //!< F+0x1258 (+150 m/s^2); no writer known, never set
	bool inflate_gyro = false;  //!< F+0x1259 (+30 rad/s); no writer known, never set
	bool full_rotation_once = false;
	//! Bias-freeze countdown F+0x1254: bias gains are zero while > 0.
	int freeze = 0;
	//! Consecutive optical rejections F+0x1238.
	int rejects = 0;
	//! Motion flag F+0x4b19 from the frame's stillness test.
	bool moving = true;
	//! Snapshot taken when `restricted` became set (spec 03 5/4.6).
	Vec3 restrict_p = Vec3::Zero();
	Quat restrict_q = Quat::Identity();

	//! Last optical measurement model (TAOChiSquare).
	double chi2 = 0.0;

	//! Init / Initialize (spec 03 5/4.2): biases and their covariance carried over when initialised before.
	void
	init(double t, const Vec3 &p0, const Quat &q0, const Vec3 &v0, const Vec3 &a_m, const Vec3 &w_m);

	/*!
	 * IMUupdate / TAOPropagation (spec 03 5/4.3) with the adaptive process noise. @p inflate and
	 * @p n are the sample's status bits. False if the step was longer than 20 ms.
	 */
	bool
	propagate(double t, const Vec3 &a_m, const Vec3 &w_m, bool inflate, int n);

	//! TAOChiSquare (spec 03 5/4.4): builds R, residual and S; returns the position chi-square.
	double
	chi_square(const Vec3 &p_m, const Quat &q_m);

	//! TAOFilterMeasUpdate + TAOApplyCorrection with the model of the last chi_square().
	void
	optical_update();

	//! Convergence mode switch after an optical update (spec 03 5/4.5).
	void
	update_mode();

	//! Zero-velocity update (spec 03 5/4.7).
	void
	zupt();

	//! Occlusion position update (spec 03 5/4.8); returns its chi-square. Sets freeze = 60.
	double
	occlusion_update(const Vec3 &p_pred);

	//! Position standard deviations.
	Vec3
	sigma_p() const
	{
		return P.diagonal().head<3>().cwiseMax(0.0).cwiseSqrt();
	}

private:
	void
	correct(const Vec15 &dx);

	Eigen::Matrix<double, 6, 15> H_ = Eigen::Matrix<double, 6, 15>::Zero();
	Vec6 r_ = Vec6::Zero();
	Mat6 R_ = Mat6::Identity();
	Mat6 S_ = Mat6::Identity();
};

/*
 *
 * Output stages
 *
 */

/*!
 * centaur::HighFreqKalmanFilter::updateHighFreqKalman6Dof (spec 03 5/6): nine scalar Kalman
 * filters on position/velocity (constant velocity, 2 ms steps) and on the rotation increment
 * against the previous output; measurement noise scale * s^5.
 */
class HighFreqSmoother
{
public:
	//! Smooth p, v, q in place. @p s is the 3DoF confidence, @p rate_flag selects scale 5 instead of 20.
	bool
	update(Vec3 &p, Vec3 &v, Quat &q, double s, bool rate_flag);

	void
	reset()
	{
		initialized_ = false;
	}

private:
	bool initialized_ = false;
	Quat q_ref_ = Quat::Identity();
	std::array<double, 9> x_{};
	std::array<double, 9> P_{};
};

/*!
 * Butterworth low-pass of the per-sample path (spec 03 4/3.10): order N, pass band edge
 * 0.6498394 (2 tan(0.1 pi)), 1 dB ripple, bilinear transform; direct form II per channel.
 */
class Butterworth
{
public:
	explicit Butterworth(int order = 10);
	//! Filter one sample of channel @p ch (0..5).
	double
	filter(int ch, double x);
	const std::vector<double> &
	b() const
	{
		return b_;
	}
	const std::vector<double> &
	a() const
	{
		return a_;
	}

private:
	std::vector<double> b_, a_;
	std::array<std::vector<double>, 6> w_;
};

/*
 *
 * Fusion gate (spec 02 13.1)
 *
 */

//! How the optical pose of a frame was obtained (stock method codes).
enum class OpticalMethod
{
	kIdTable = 0, //!< re-initialisation, ID-table P5P
	kP3P = 1,     //!< re-initialisation, P3P
	kP5PNew = 2,  //!< re-initialisation, prior-rotation P5P
	kTracking = 3 //!< tracking mode (judgeTrackingOk)
};

struct GateInput
{
	Hand hand = kLeft;
	OpticalMethod method = OpticalMethod::kTracking;
	bool tracking_ok = false; //!< judgeTrackingOk accepted
	int residual_count = 0;   //!< residuals of the judge
	double residual_mean = 100.0;
	bool near = false;             //!< judgeNear
	Pose T_WH;                     //!< head pose of the frame
	Pose filter_pose;              //!< filter prior T_WM at the frame
	Vec3 p_optical = Vec3::Zero(); //!< optical position
	double chi2 = 0.0;             //!< SsfFilter::chi_square
	int consecutive_rejects = 0;
	int lost_frames = 0;
	bool lock_was_set = false; //!< high-quality lock of the previous accept
};

struct GateResult
{
	bool accept = false;
	bool force_reset = false; //!< more than 9 consecutive rejects
	bool lock = false;        //!< high-quality lock (published position = optical)
	double dp = 0.0;          //!< |p_filter - p_optical|
};

//! The accept rules of spec 02 13.1 (boxes in the head frame, chi-square, residual counters).
GateResult
fusion_gate(const GateInput &in);

/*
 *
 * Per-hand fusion
 *
 */

//! Per-sample output status (HighFreqPose +0xf0, spec 03 4/4).
enum class OutputStatus
{
	kStill = 0,
	kTracked = 1,
	kFallback = 2,
	kFallbackFlagged = 3,
	kFallbackHit = 4,
	kTrackedHit = 5,
};

//! The published pose (HighFreqPose, spec 03 4/4), predicted to `t_ns`.
struct OutputPose
{
	bool valid = false;
	int64_t t_ns = 0;      //!< prediction time: IMU sample time + 41 ms while moving
	Pose T_WM;             //!< predicted pose
	Vec3 v = Vec3::Zero(); //!< m/s in W (soft dead zone)
	Vec3 a_lin = Vec3::Zero();
	Vec3 omega = Vec3::Zero();   //!< body-frame angular velocity, rad/s
	Vec3 ang_acc = Vec3::Zero(); //!< from the 3DoF filter
	OutputStatus status = OutputStatus::kFallback;
	bool is_3dof = true;
	//! T_WM.t is a position: 6DoF, held from the last 6DoF output or the elbow model (not 0)
	bool has_position = false;
	bool still = false;
	int imu_only_steps = 0;
	int optical_state = 0; //!< 0 none, 1 optical, 2 IMU-predicted while lost
	int64_t t_unpredicted_ns = 0;
	Pose T_WM_unpredicted;
	Vec3 v_unpredicted = Vec3::Zero();
};

/*!
 * A published pose extrapolated to @p t_ns (client predict_motion_new, spec 03 6/7): constant
 * velocity and body angular velocity; no prediction beyond 1 s.
 */
OutputPose
predict_output(OutputPose o, int64_t t_ns);

//! Prior for the optical solve of one camera frame (spec 02 5.3, spec 03 6/2).
struct FramePrior
{
	bool has_body = false; //!< IMU samples exist for this frame interval (creatBody)
	bool tracked = false;  //!< the filter tracks this hand: T_WM is the prior
	Pose T_WM;
	Quat q_init = Quat::Identity(); //!< rotation prior for (re)initialisation (spec 02 7.1)
	bool moving = true;
	Vec3 v = Vec3::Zero();
	Vec3 sigma_p = Vec3::Zero();
	int lost_frames = 0;
	bool local = false; //!< prior from the local strapdown (spec 03 6/2)
};

//! Optical result of a tracked frame (updatePose) for the gate and the update.
struct OpticalReport
{
	Pose T_WM;
	OpticalMethod method = OpticalMethod::kTracking;
	bool tracking_ok = false;
	int residual_count = 0;
	double residual_mean = 100.0;
	bool near = false;
	int matched_leds = 0;
	double occlusion_rate = 0.0; //!< P+0x19e8, sum of bounding-box IoU with the other hand
};

//! Frame without an accepted optical solution (updatePose invalid branch).
struct NoOpticalReport
{
	bool near = false;
	bool force_lost = false; //!< Body force-lost flag: predict once, then reset
	double occlusion_rate = 0.0;
};

struct OpticalResult
{
	bool accepted = false;
	bool reset = false; //!< tracking was reset (rejected and lost, or forced)
	double chi2 = 0.0;
	double dp = 0.0;
};

//! Tunables whose stock value is a configuration choice or UNCERTAIN.
struct FusionParams
{
	//! Calibration record rate flag (P+0x6c0): 3DoF windows 20/50 instead of 80/100 and more.
	bool rate_flag = false;
	//! Largest IMU step before resetTracking (spec 03 4/3.5).
	double max_imu_gap = 0.035;
	//! The acc-out-of-range reset (P+0x6c2): off on stock, whose only writer is the
	//! constructor (strb wzr at 0x2f5e68); read at resetTrackingIfAbnormalState 0x35ca54.
	bool acc_range_reset = false;
	//! Publish the latest optical pose while a strike is flagged (spec 03 4/3.17, addSingleImu 0x3639cc..0x363b4c).
	bool optical_passthrough = true;
	//! Propagation and history buffer capacity (P+0x5a8): the constructor stores the int pair
	//! {0, 200} from 0xb47be8 to P+0x5a4 (str d1 at 0x2f5dd0); nothing else writes P+0x5a8.
	int buffer_capacity = 200;
	//! 3DoF-only output (trackingMode 0 / P+0x5a0).
	bool force_3dof = false;
	//! Prediction time of the "Phoenix" model string (spec 03 4/3.17).
	double prediction_time = 0.039;
	//! Butterworth order (config butterworthin).
	int butterworth_order = 10;
};

/*!
 * Everything of one hand's IMU side (ControllerPosePredictor's IMU and filter paths). Call order
 * of the engine:
 *  - add_imu() for every sample as it arrives (all samples before an image time must have been
 *    added before begin_frame() for that image);
 *  - per camera frame begin_frame(); then, if tracked, report_optical() or report_no_optical();
 *    if not tracked, report_init() or report_init_failure() after the init solve;
 *  - pose_at() at any time.
 */
class ControllerFusion
{
public:
	explicit ControllerFusion(Hand hand, const FusionParams &params = FusionParams{});

	//! The other hand (strike detection, together box, anchoring, spec 03 4/3.2, 4/3.13, 6/4).
	void
	set_other_hand(const ControllerFusion *other)
	{
		other_ = other;
	}

	//! Per-unit calibration (identity when absent).
	void
	set_calibration(const ImuCalib &calib)
	{
		calib_ = calib;
	}

	//! The controller's temperature-binned gyro bias table, for both 3DoF filters (spec 03 3/5).
	void
	set_bias_table(const std::array<Vec3, 100> &table)
	{
		oc_a_.set_bias_table(table);
		oc_b_.set_bias_table(table);
	}

	/*!
	 * One controller IMU sample (spec 03 3/2-3/3, 4/3). @p T_WH_at_sample is the head pose
	 * interpolated to the sample time (spec 03 1/4); without it the head pose of the last frame
	 * is used (arm model, together box, plausibility).
	 */
	void
	add_imu(const ImuSample &sample, const Pose *T_WH_at_sample = nullptr);

	/*!
	 * Start a camera frame (creatBody, predictControllerPose; spec 02 3.2, 5.3; spec 03 6/1-6/2):
	 * takes this frame's IMU samples, propagates the filter exactly to @p t_image_ns and returns
	 * the prior. Resets tracking if the newest IMU sample is older than 0.2 s.
	 */
	FramePrior
	begin_frame(int64_t t_image_ns, const Pose &T_WH);

	/*!
	 * Optical result of a tracked frame: fusion gate (spec 02 13.1), filter update with mode
	 * switch (spec 03 5/4.4-4.5), or on rejection loss handling (spec 03 6/4) and reset.
	 */
	OpticalResult
	report_optical(const OpticalReport &r);

	/*!
	 * No optical solution for a tracked frame: predicIfLost (spec 03 6/4). False if tracking was
	 * reset (the next frame initialises).
	 */
	bool
	report_no_optical(const NoOpticalReport &r);

	/*!
	 * Initialisation solve succeeded for this frame (spec 02 7.1): the pose enters the
	 * confirmation window; with 3 Bodies the filter is initialised (spec 03 5/4.2) with IMU
	 * replay and the hand is tracked. Returns true when the hand became tracked.
	 */
	bool
	report_init(const Pose &T_WM);

	//! Initialisation failed for this frame: the confirmation window is cleared.
	void
	report_init_failure();

	//! resetTracking (spec 03 4/6, 6/3).
	void
	reset_tracking();

	/*!
	 * The published pose extrapolated to @p t_ns (client predict_motion_new, spec 03 6/7):
	 * constant velocity and body angular velocity; no prediction beyond 1 s.
	 */
	OutputPose
	pose_at(int64_t t_ns) const;

	//! The last published pose (GetLastHighFreq).
	const OutputPose &
	output() const
	{
		return out_;
	}

	bool
	tracked() const
	{
		return tracked_;
	}
	int
	lost_frames() const
	{
		return lost_;
	}
	const SsfFilter &
	filter() const
	{
		return ssf_;
	}
	//! 3DoF orientation of the last sample of the current frame (P+0x460).
	const Quat &
	q3dof() const
	{
		return q3_frame_;
	}
	//! Bodies in the init confirmation window (P+0xc58 count; 0 = empty).
	int
	window_size() const
	{
		return int(window_.size());
	}
	//! Filter velocity in W.
	Vec3
	velocity() const
	{
		return ssf_.v;
	}
	//! Completely still since an IMU gap (P+0x1975): no P3P acquisition, prior from filtered_pose().
	bool
	complete_still() const
	{
		return complete_still_;
	}
	//! The filtered output pose, or the last acquisition (P+0xa10).
	const Pose &
	filtered_pose() const
	{
		return filtered_pose_;
	}
	//! Position of the last good published pose (P+0x13a0, written at 0x364e4c; zero until then).
	const Vec3 &
	last_good_position() const
	{
		return last_good_.T_WM.t;
	}
	//! Following the other hand while lost (P+0x198e).
	bool
	follow() const
	{
		return follow_;
	}
	const OrientationController &
	orientation() const
	{
		return rate_flag_oc();
	}
	//! Diagnostics: tracking resets by the per-sample abnormal-state check.
	int
	abnormal_resets() const
	{
		return abnormal_resets_;
	}

private:
	struct Sample
	{
		double t = 0.0;
		Vec3 gyro = Vec3::Zero();
		Vec3 accel = Vec3::Zero();
		double temperature = 0.0;
		bool flagbit = false;
		int flagrest = 0;
		bool strike = false;
		Quat q3 = Quat::Identity();
	};
	//! PicoController::PoseInfo.
	struct PoseInfo
	{
		double t = 0.0;
		Vec3 p = Vec3::Zero();
		Vec3 v = Vec3::Zero();
		Quat q = Quat::Identity();
		Vec3 accel = Vec3::Zero();
		Vec3 gyro = Vec3::Zero();
		int state = 0;
	};
	struct Body
	{
		double t = 0.0;
		Pose T_WM;
		Vec3 v = Vec3::Zero();
		std::vector<Sample> samples;
	};

	const OrientationController &
	rate_flag_oc() const
	{
		return params_.rate_flag ? oc_a_ : oc_b_;
	}
	OrientationController &
	rate_flag_oc()
	{
		return params_.rate_flag ? oc_a_ : oc_b_;
	}

	bool
	check_abnormal(const Sample &s);
	void
	process_strike(const Sample &s);
	void
	judge_long_still(const Sample &s);
	Quat
	integrate_q(const Quat &q, double dt, const Vec3 &gyro);
	PoseInfo
	one_step(const PoseInfo &prev, const Sample &s);
	void
	stand_to_move(OutputPose &pose, const Sample &s);
	void
	window_statistics(const std::vector<Sample> &samples);
	void
	publish_filter_state(int state);
	void
	push_body(const Pose &T_WM);
	bool
	predict_if_lost(bool near, double occlusion_rate);
	void
	record_anchor();
	PoseInfo
	high_freq_3dof();
	void
	sample_output(const Sample &raw, const Sample &c, PoseInfo &out, bool six_dof);
	void
	output_3dof(const Sample &s);
	void
	finish_output(OutputPose &pose, const Sample &raw, bool zero_acc = false, bool zero_ang_acc = false);
	void
	soft_reset(bool clear_buffers);

	Hand hand_;
	FusionParams params_;
	ImuCalib calib_;
	const ControllerFusion *other_ = nullptr;
	OrientationController oc_a_, oc_b_;
	SsfFilter ssf_;
	HighFreqSmoother hfkf_;
	Butterworth butter_;

	// Buffers.
	std::deque<Sample> frame_buf_; //!< samples not yet taken by a frame (history list P+0x5b0)
	std::deque<Sample> prop_buf_;  //!< corrected samples (propagation buffer P+0x5c8)
	std::vector<Sample> frame_samples_;
	std::deque<Vec3> gyro_deque_; //!< 3-deep gyro window of the IMU-rate integration

	// Per-sample state.
	Sample prev_;
	bool have_prev_ = false;
	Pose head_;       //!< head pose at the current sample
	Pose frame_head_; //!< head pose of the last frame (P+0x7a0)
	double t_last_image_ = -1e9;
	bool image_stop_latched_ = false;
	bool strike_ = false;
	int strike_count_ = 0;
	bool hit_ = false;
	double t_hit_ = -1e9;
	int hold_off_ = 0;
	int acc_out_count_ = 0;
	int warmup_ = 0;
	bool still3dof_ = false;
	bool complete_still_ = false; //!< P+0x1975
	bool still_resume_ = false;   //!< P+0x1976
	int still_count_ = 0;
	int non_still_count_ = 0;
	bool latched_ = false;
	Quat q3_snapshot_ = Quat::Identity();
	Vec3 acc_snapshot_ = Vec3::Zero();
	int ang_count_ = 0;
	int abnormal_resets_ = 0;

	// Optical side (image thread).
	bool tracked_ = false;  //!< P+0x358
	int optical_state_ = 0; //!< P+0x1794
	bool optical_new_ = false;
	PoseInfo optical_;
	bool lock_198c_ = false;  //!< an optical pose was obtained this frame
	bool yaw_pending_ = false; //!< P+0x198d: correct the 3DoF yaw at the next sample
	bool hq_lock_ = false;     //!< P+0x1cb0
	int lost_ = 1000;
	int ramp_ = 0; //!< P+0x1968
	bool near_ = false;
	bool follow_ = false;
	bool frame_40ms_ = true;
	bool face_frozen_ = false;
	Pose face_pose_H_;
	bool zupt_hold_ = false;
	Vec3 zupt_hold_p_ = Vec3::Zero();
	bool local_valid_ = false; //!< P+0x1d26
	double local_t_ = -1e9;    //!< P+0x1d28
	Vec3 local_p_ = Vec3::Zero();
	bool window_flag_ = false;    //!< P+0x1d70: a sample of the frame window had the status bit
	bool near_head_flag_ = false; //!< P+0x1d71: set while lost after such a window
	Pose frame_pose_;             //!< P+0x950: optical solution, or the filter pose while lost
	Vec3 anchor_offset_ = Vec3::Zero();      //!< P+0x1990
	Quat anchor_q_other_ = Quat::Identity(); //!< other P+0xa90
	bool anchor_valid_ = false;
	double t_frame_ = 0.0;
	Quat q3_frame_ = Quat::Identity();
	std::vector<Body> window_;
	Pose filtered_pose_; //!< P+0xa10

	// IMU-rate output.
	PoseInfo base_;
	PoseInfo running_;
	int imu_only_steps_ = 0;
	bool arm_zone_ = false;
	Vec3 fore_ = Vec3::Zero();
	Pose elbow_H_;
	Vec3 held_p_ = Vec3::Zero();
	Vec3 last_out_p_ = Vec3::Zero();
	bool has_out_p_ = false; //!< last_out_p_ holds a 6DoF output
	Vec3 head_rel_p_ = Vec3::Zero();
	bool resting_ = false;
	OutputPose last_good_;
	OutputPose out_;
};

} // namespace pico4::ctrack
