// Copyright 2026, overpressure contributors.
// SPDX-License-Identifier: BSL-1.0
/*
 * Test of the controller tracker's thread (synthetic data, no device data): frame sets that
 * arrive before their IMU samples wait for them, a newer set replaces a waiting one, a set whose
 * samples never come is solved after the wait, a set without a head pose is skipped, and the
 * tracking result matches the engine's.
 */

#include "pico4_ctrack_calib.hpp"
#include "pico4_ctrack_runner.hpp"
#include "test_pico4_ctrack_world.hpp"

#include <chrono>
#include <cstdio>

static const int64_t kFrameNs = 40000000, kImuNs = 2000000;

static void
push_imu(Runner &r, const Motion &m, int64_t from, int64_t to)
{
	for (int64_t s = from; s <= to; s += kImuNs) {
		r.push_imu(kLeft, m.imu_at(s));
	}
}

//! The engine test's sweep through the runner, each frame set pushed before its IMU samples.
static void
test_tracking()
{
	World w;
	Runner r(w.cams, w.models[0], w.models[1], [&w](int64_t, Pose &T_WH) {
		T_WH = w.T_WH;
		return true;
	});
	Motion motion;
	const int frames = 100;
	motion.t_move = 1000000000 + 50 * kFrameNs;
	int64_t t = 1000000000;
	int tracked_at = -1, lost = 0;
	double worst_p = 0, worst_pred = 0;
	for (int f = 0; f < frames; f++) {
		r.push_frames(t, w.render(w.models[0], motion.pose_at(t)));
		push_imu(r, motion, t - kFrameNs + kImuNs, t);
		r.drain();
		Runner::Stats st = r.stats();
		if (tracked_at < 0 && st.last[kLeft].tracked_after) {
			tracked_at = f;
		}
		if (tracked_at >= 0) {
			lost += !st.last[kLeft].tracked_after;
			OutputPose o = r.pose_at(kLeft, t);
			if (o.valid && f > tracked_at + 5) {
				worst_p = std::max(worst_p, (o.T_WM_unpredicted.t - motion.pose_at(t).t).norm());
				worst_pred = std::max(worst_pred, (o.T_WM.t - motion.pose_at(t).t).norm());
			}
		}
		t += kFrameNs;
	}
	Runner::Stats st = r.stats();
	printf("tracked from frame set %d, lost %d, worst %.2f mm (extrapolated %.2f mm); %llu solved, %llu dropped, "
	       "%llu IMU timeouts\n",
	       tracked_at, lost, worst_p * 1000, worst_pred * 1000, (unsigned long long)st.solved,
	       (unsigned long long)st.dropped, (unsigned long long)st.imu_timeouts);
	EXPECT(tracked_at >= 2 && tracked_at < 30);
	EXPECT(lost == 0);
	EXPECT(worst_p < 0.005);
	EXPECT(worst_pred < 0.01);
	EXPECT(st.solved == (uint64_t)frames && st.dropped == 0 && st.imu_timeouts == 0 && st.no_head == 0);
}

static void
test_waiting()
{
	World w;
	bool head = true;
	Runner r(w.cams, w.models[0], w.models[1], [&head, &w](int64_t, Pose &T_WH) {
		T_WH = w.T_WH;
		return head;
	});
	Motion motion;
	motion.t_move = INT64_MAX / 2;
	int64_t t0 = 1000000000;
	push_imu(r, motion, t0 - kFrameNs, t0);
	r.drain();

	// Two sets while the samples of the first are still missing: the newer one replaces it.
	auto images = w.render(w.models[0], motion.rest);
	r.push_frames(t0 + kFrameNs, images);
	r.push_frames(t0 + 2 * kFrameNs, images);
	push_imu(r, motion, t0 + kImuNs, t0 + 2 * kFrameNs);
	r.drain();
	Runner::Stats st = r.stats();
	EXPECT(st.frame_sets == 2 && st.dropped == 1 && st.solved == 1 && st.imu_timeouts == 0);

	// A set whose samples never come is solved after the wait.
	auto start = std::chrono::steady_clock::now();
	r.push_frames(t0 + 3 * kFrameNs, images);
	r.drain();
	double waited = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
	st = r.stats();
	EXPECT(st.solved == 2 && st.imu_timeouts == 1);
	EXPECT(waited >= Runner::kImuWaitNs * 1e-9 * 0.9);

	// No head pose at the image time: skipped.
	head = false;
	push_imu(r, motion, t0 + 2 * kFrameNs + kImuNs, t0 + 4 * kFrameNs);
	r.push_frames(t0 + 4 * kFrameNs, images);
	r.drain();
	st = r.stats();
	EXPECT(st.solved == 2 && st.no_head == 1);

	// A hand that stopped sending long ago is not waited for.
	start = std::chrono::steady_clock::now();
	r.push_frames(t0 + 2 * Runner::kImuStaleNs, images);
	r.drain();
	waited = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
	EXPECT(waited < Runner::kImuWaitNs * 1e-9 * 0.5 && r.stats().imu_timeouts == 1);
}

static void
test_lever_arms()
{
	// The file tooling/leverarm.py writes (synthetic values).
	const char *path = "test_lever_arm.json";
	FILE *f = fopen(path, "w");
	EXPECT(f != nullptr);
	if (f == nullptr) {
		return;
	}
	fputs("{\n \"schema\": 1,\n \"source\": \"lib.so\",\n \"units\": \"m\",\n"
	      " \"convention\": \"(x, y, z) in the aim frame\",\n"
	      " \"left\": [\n  -0.005,\n  -0.01,\n  0.004\n ],\n \"right\": [\n  0.005,\n  -0.01,\n  0.004\n ]\n}\n",
	      f);
	fclose(f);
	std::array<Vec3, 2> lever;
	std::string error;
	EXPECT(load_lever_arms(path, lever, error));
	EXPECT((lever[kLeft] - Vec3(-0.005, -0.01, 0.004)).norm() < 1e-12);
	EXPECT((lever[kRight] - Vec3(0.005, -0.01, 0.004)).norm() < 1e-12);
	remove(path);
	EXPECT(!load_lever_arms(path, lever, error) && !error.empty());
}

int
main()
{
	test_tracking();
	test_waiting();
	test_lever_arms();
	if (failures == 0) {
		printf("test_pico4_ctrack_runner: all checks passed\n");
	}
	return failures == 0 ? 0 : 1;
}
