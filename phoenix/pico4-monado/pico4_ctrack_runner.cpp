// Copyright 2026, overpressure contributors.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  The controller tracker on its own thread (see pico4_ctrack_runner.hpp).
 * @ingroup drv_pico4
 */

#include "pico4_ctrack_runner.hpp"

#include <chrono>
#include <cstring>

namespace pico4::ctrack {

Runner::Runner(const std::array<Camera, kCameras> &cams, const LedModel &left, const LedModel &right,
               HeadSource head, const FusionParams &params, PublishFn on_publish)
    : tracker_(cams, left, right, params), head_(std::move(head)), on_publish_(std::move(on_publish))
{
	thread_ = std::thread([this] { run(); });
}

Runner::~Runner()
{
	stop();
}

void
Runner::stop()
{
	{
		std::lock_guard<std::mutex> g(lock_);
		stop_ = true;
	}
	wake_.notify_all();
	if (thread_.joinable()) {
		thread_.join();
	}
}

void
Runner::push_imu(Hand hand, const ImuSample &sample)
{
	{
		std::lock_guard<std::mutex> g(lock_);
		if (stop_) {
			return;
		}
		imu_.push_back({hand, sample});
		newest_imu_[hand] = std::max(newest_imu_[hand], sample.t_ns);
		stats_.imu_samples[hand]++;
	}
	wake_.notify_all();
}

void
Runner::push_frames(int64_t t_ns, const std::array<ImageView, kCameras> &images)
{
	{
		std::lock_guard<std::mutex> g(lock_);
		if (stop_) {
			return;
		}
		stats_.frame_sets++;
		if (pending_) {
			stats_.dropped++;
		}
		for (int k = 0; k < kCameras; k++) {
			const ImageView &v = images[k];
			std::vector<uint8_t> &dst = next_images_[k];
			dst.resize((size_t)kImageWidth * kImageHeight);
			for (int y = 0; y < kImageHeight; y++) {
				memcpy(&dst[(size_t)y * kImageWidth], v.data + (size_t)y * v.stride, kImageWidth);
			}
		}
		pending_ = true;
		pending_t_ = t_ns;
	}
	wake_.notify_all();
}

void
Runner::set_unit(Hand hand, const UnitCalibration &unit)
{
	{
		std::lock_guard<std::mutex> g(lock_);
		if (stop_) {
			return;
		}
		unit_[hand] = unit;
		unit_pending_[hand] = true;
	}
	wake_.notify_all();
}

OutputPose
Runner::pose_at(Hand hand, int64_t t_ns) const
{
	OutputPose o;
	{
		std::lock_guard<std::mutex> g(lock_);
		o = out_[hand];
	}
	return predict_output(o, t_ns);
}

Runner::Stats
Runner::stats() const
{
	std::lock_guard<std::mutex> g(lock_);
	return stats_;
}

void
Runner::drain()
{
	std::unique_lock<std::mutex> l(lock_);
	idle_.wait(l, [this] { return !pending_ && !busy_; });
}

bool
Runner::imu_caught_up(int64_t t) const
{
	for (int h = 0; h < 2; h++) {
		// A hand that sends nothing (off, out of range) does not hold the other one up.
		if (newest_imu_[h] != INT64_MIN && newest_imu_[h] < t && newest_imu_[h] > t - kImuStaleNs) {
			return false;
		}
	}
	return true;
}

void
Runner::publish(int64_t frame_t_ns, const Pose &T_WH)
{
	OutputPose o[2] = {tracker_.output(kLeft), tracker_.output(kRight)};
	HandFrameStats s[2] = {tracker_.stats(kLeft), tracker_.stats(kRight)};
	{
		std::lock_guard<std::mutex> g(lock_);
		for (int h = 0; h < 2; h++) {
			out_[h] = o[h];
			stats_.last[h] = s[h];
		}
	}
	if (on_publish_) {
		on_publish_(frame_t_ns, T_WH, s, o);
	}
}

void
Runner::run()
{
	std::unique_lock<std::mutex> l(lock_);
	while (!stop_) {
		if (unit_pending_[0] || unit_pending_[1]) {
			bool apply[2] = {unit_pending_[0], unit_pending_[1]};
			UnitCalibration units[2] = {unit_[0], unit_[1]};
			unit_pending_[0] = unit_pending_[1] = false;
			busy_ = true;
			l.unlock();
			for (int h = 0; h < 2; h++) {
				if (apply[h]) {
					tracker_.set_unit((Hand)h, units[h]);
				}
			}
			l.lock();
			busy_ = false;
			continue;
		}
		if (!pending_ && imu_.empty()) {
			idle_.notify_all();
			wake_.wait(l);
			continue;
		}
		std::vector<Queued> batch;
		bool solve = false;
		int64_t t = 0;
		if (pending_) {
			t = pending_t_;
			auto deadline = std::chrono::steady_clock::now() + std::chrono::nanoseconds(kImuWaitNs);
			while (!stop_ && !imu_caught_up(t) && pending_ && pending_t_ == t) {
				if (wake_.wait_until(l, deadline) == std::cv_status::timeout) {
					stats_.imu_timeouts += !imu_caught_up(t);
					break;
				}
			}
			if (stop_) {
				break;
			}
			if (pending_t_ != t) {
				continue; // a newer set arrived while waiting
			}
			// This set's samples (and older ones) go in before it; later ones wait for the solve.
			for (auto it = imu_.begin(); it != imu_.end();) {
				if (it->sample.t_ns < t) {
					batch.push_back(*it);
					it = imu_.erase(it);
				} else {
					++it;
				}
			}
			std::swap(solve_images_, next_images_);
			pending_ = false;
			solve = true;
		} else {
			batch.assign(imu_.begin(), imu_.end());
			imu_.clear();
		}
		busy_ = true;
		l.unlock();

		std::stable_sort(batch.begin(), batch.end(),
		                 [](const Queued &a, const Queued &b) { return a.sample.t_ns < b.sample.t_ns; });
		for (const Queued &q : batch) {
			tracker_.add_imu(q.hand, q.sample);
		}
		bool solved = false;
		Pose T_WH;
		if (solve) {
			if (head_ && head_(t, T_WH)) {
				std::array<ImageView, kCameras> views;
				for (int k = 0; k < kCameras; k++) {
					views[k] = ImageView{solve_images_[k].data(), kImageWidth, kImageHeight, kImageWidth};
				}
				tracker_.process_frame(t, T_WH, views);
				solved = true;
			}
		}
		publish(solved ? t : 0, T_WH);

		l.lock();
		busy_ = false;
		if (solve) {
			stats_.solved += solved;
			stats_.no_head += !solved;
		}
	}
	busy_ = false;
	idle_.notify_all();
}

} // namespace pico4::ctrack
