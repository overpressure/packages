// Copyright 2026, overpressure contributors.
// SPDX-License-Identifier: BSL-1.0
/*
 * End-to-end test of the controller tracker with synthetic data (no device data): rendered LED
 * images of four cameras and the matching controller IMU samples. The left controller lies still,
 * then sweeps sideways from rest (1 Hz, 20 cm) with a slight wobble; the tracker must acquire it,
 * confirm it over three frame sets, and track it to within a few millimetres.
 */

#include "test_pico4_ctrack_world.hpp"

#include <cstdlib>

int
main()
{
	World w;
	Tracker tracker(w.cams, w.models[0], w.models[1]);
	const int64_t frame_ns = 40000000, imu_ns = 2000000;
	const int still_frames = 50, moving_frames = 50;
	Motion motion;
	motion.t_move = 1000000000 + still_frames * frame_ns;
	Pose T_WM = motion.rest;
	int64_t t = 1000000000;
	int tracked_at = -1, lost = 0;
	double worst_p = 0, worst_q = 0;
	for (int f = 0; f < still_frames + moving_frames; f++) {
		for (int64_t s = t - frame_ns + imu_ns; s <= t; s += imu_ns) {
			tracker.add_imu(kLeft, motion.imu_at(s), &w.T_WH);
		}
		T_WM = motion.pose_at(t);
		tracker.process_frame(t, w.T_WH, w.render(w.models[0], T_WM));
		const HandFrameStats &st = tracker.stats(kLeft);
		if (getenv("VERBOSE") != nullptr) {
			printf("%d: body %d init %d method %d cam %d ok %d feats %d inl %d acc %d tracked %d->%d\n", f, st.has_body,
			       st.init, st.method, st.init_camera, st.tracking_ok, st.features, st.inliers, st.accepted,
			       st.tracked_before, st.tracked_after);
			const OutputPose &raw = tracker.output(kLeft);
			if (raw.valid) {
				printf("   published %+.1f ms after the sample, moved %.1f mm (v * dt %.1f mm), error there %.1f mm\n",
				       (raw.t_ns - raw.t_unpredicted_ns) * 1e-6, (raw.T_WM.t - raw.T_WM_unpredicted.t).norm() * 1000,
				       (raw.v * ((raw.t_ns - raw.t_unpredicted_ns) * 1e-9)).norm() * 1000,
				       (raw.T_WM.t - motion.pose_at(raw.t_ns).t).norm() * 1000);
			}
			if (st.method >= 0) {
				OutputPose o = tracker.pose_at(kLeft, t);
				printf("   optical %.1f mm %.2f deg; output valid %d %.1f mm %.2f deg (t %lld vs %lld)\n",
				       (st.T_WM.t - T_WM.t).norm() * 1000, full_angle_deg(st.T_WM.q, T_WM.q), o.valid,
				       (o.T_WM_unpredicted.t - T_WM.t).norm() * 1000, full_angle_deg(o.T_WM_unpredicted.q, T_WM.q),
				       (long long)o.t_unpredicted_ns, (long long)t);
			}
		}
		if (tracked_at < 0 && st.tracked_after) {
			tracked_at = f;
		}
		if (tracked_at >= 0) {
			lost += !st.tracked_after;
			OutputPose o = tracker.pose_at(kLeft, t);
			if (o.valid && f > tracked_at + 5) {
				worst_p = std::max(worst_p, (o.T_WM_unpredicted.t - T_WM.t).norm());
				worst_q = std::max(worst_q, full_angle_deg(o.T_WM_unpredicted.q, T_WM.q));
			}
		}
		t += frame_ns;
	}
	printf("tracked from frame set %d, lost %d times, worst position %.2f mm, rotation %.2f deg\n", tracked_at,
	       lost, worst_p * 1000, worst_q);
	EXPECT(tracked_at >= 2 && tracked_at < 30);
	EXPECT(lost == 0);
	EXPECT(worst_p < 0.005);
	EXPECT(worst_q < 2.0);
	EXPECT(!tracker.tracked(kRight));
	if (failures == 0) {
		printf("test_pico4_ctrack_engine: all checks passed\n");
	}
	return failures == 0 ? 0 : 1;
}
