// Copyright 2026, overpressure contributors.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  The controller tracker on its own thread: IMU samples and LED frame sets arrive from
 *         the driver's sensor and camera threads, poses are read by the compositor's.
 *
 * Stock requires every IMU sample before an image time to be in the filter before that image
 * is solved. The radio delivers controller samples late and in bursts, so a frame set waits for
 * them (at most kImuWaitNs) before it is solved; samples after it wait for the solve. Only the
 * newest frame set waits: an older one still unsolved is dropped. Readers never wait for a solve:
 * each hand's published pose is copied out under a short lock and extrapolated.
 *
 * @ingroup drv_pico4
 */

#pragma once

#include "pico4_ctrack_engine.hpp"

#include <condition_variable>
#include <deque>
#include <functional>
#include <mutex>
#include <thread>

namespace pico4::ctrack {

class Runner
{
public:
	//! Head pose (body frame H in the tracker's world W, +x up) at a time; false if unknown.
	using HeadSource = std::function<bool(int64_t t_ns, Pose &T_WH)>;
	/*!
	 * Called on the tracker's thread after each publication: @p frame_t_ns is the solved frame
	 * set's time with its head pose, or 0 after IMU samples alone (diagnostics).
	 */
	using PublishFn = std::function<void(int64_t frame_t_ns, const Pose &T_WH, const HandFrameStats *stats,
	                                     const OutputPose *out)>;

	//! Longest wait for the IMU samples of a frame set.
	static constexpr int64_t kImuWaitNs = 60000000;
	//! A hand whose newest sample is this much older than a frame set is not waited for.
	static constexpr int64_t kImuStaleNs = 500000000;

	struct Stats
	{
		uint64_t frame_sets = 0;   //!< pushed
		uint64_t solved = 0;       //!< processed by the tracker
		uint64_t dropped = 0;      //!< replaced by a newer set before the solve
		uint64_t no_head = 0;      //!< skipped: no head pose at the image time
		uint64_t imu_timeouts = 0; //!< solved without all of the frame interval's IMU samples
		uint64_t imu_samples[2] = {0, 0};
		HandFrameStats last[2];
	};

	Runner(const std::array<Camera, kCameras> &cams, const LedModel &left, const LedModel &right, HeadSource head,
	       const FusionParams &params = FusionParams{}, PublishFn on_publish = nullptr);
	~Runner();

	Runner(const Runner &) = delete;
	Runner &
	operator=(const Runner &) = delete;

	//! One controller IMU sample (SI units, raw sensor axes, CLOCK_MONOTONIC).
	void
	push_imu(Hand hand, const ImuSample &sample);

	//! One LED frame set (8 bit, 640x480, factory camera order); the images are copied.
	void
	push_frames(int64_t t_ns, const std::array<ImageView, kCameras> &images);

	//! A controller's per-unit data; the tracker's thread applies it before its next work.
	void
	set_unit(Hand hand, const UnitCalibration &unit);

	//! The published pose of a hand extrapolated to @p t_ns.
	OutputPose
	pose_at(Hand hand, int64_t t_ns) const;

	Stats
	stats() const;

	//! Wait until every frame set pushed so far is solved or dropped (tests).
	void
	drain();

	//! End the thread: no more solves or head-pose calls; later pushes are ignored.
	void
	stop();

private:
	struct Queued
	{
		Hand hand;
		ImuSample sample;
	};

	Tracker tracker_;
	HeadSource head_;
	PublishFn on_publish_;

	mutable std::mutex lock_;
	std::condition_variable wake_, idle_;
	bool stop_ = false, busy_ = false;
	std::deque<Queued> imu_;
	bool unit_pending_[2] = {false, false};
	UnitCalibration unit_[2];
	int64_t newest_imu_[2] = {INT64_MIN, INT64_MIN};
	bool pending_ = false;
	int64_t pending_t_ = 0;
	std::array<std::vector<uint8_t>, kCameras> next_images_, solve_images_;
	OutputPose out_[2];
	Stats stats_;
	std::thread thread_;

	void
	run();
	bool
	imu_caught_up(int64_t t) const;
	void
	publish(int64_t frame_t_ns, const Pose &T_WH);
};

} // namespace pico4::ctrack
