// Copyright 2026, overpressure contributors.
// SPDX-License-Identifier: BSL-1.0
/*
 * Unit test of the controller tracker's acquisition paths with synthetic data (no device data):
 * the prior-rotation triple search, P5P_New, P3P, the footprint rectangle and the plausibility
 * tests.
 */

#include "pico4_ctrack_init.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>

using namespace pico4::ctrack;

static int failures;

#define EXPECT(cond)                                                                                                   \
	do {                                                                                                           \
		if (!(cond)) {                                                                                         \
			fprintf(stderr, "%s:%d: expected %s\n", __FILE__, __LINE__, #cond);                            \
			failures++;                                                                                    \
		}                                                                                                      \
	} while (0)

static Camera
camera(const Pose &T_HC)
{
	Camera c;
	c.xi = 0.9;
	c.fu = c.fv = 420.0;
	c.pu = 320.0;
	c.pv = 240.0;
	c.k1 = -0.15;
	c.k2 = 0.03;
	c.p1 = 0.0004;
	c.p2 = -0.0003;
	c.T_HC = T_HC;
	return c;
}

//! A synthetic LED ring (not the real model): 20 LEDs on a tilted, wavy ring, normals outward.
static LedModel
ring()
{
	LedModel m;
	for (int i = 0; i < kLeds; i++) {
		double a = 2 * M_PI * i / kLeds;
		m.position[i] = Vec3(0.045 * std::cos(a), 0.045 * std::sin(a), 0.012 * std::sin(3 * a) + 0.002 * i / kLeds);
		m.normal[i] = Vec3(std::cos(a), std::sin(a), 0.3 * std::cos(2 * a)).normalized();
	}
	return m;
}

static Quat
rot(double ax, double ay, double az)
{
	return Quat(Eigen::AngleAxisd(az, Vec3::UnitZ()) * Eigen::AngleAxisd(ay, Vec3::UnitY()) *
	            Eigen::AngleAxisd(ax, Vec3::UnitX()));
}

struct Scene
{
	LedModel model = ring();
	std::array<Camera, kCameras> cams;
	Pose truth;
	InitContext ctx;

	Scene()
	{
		cams = {camera(Pose()), camera(Pose(rot(0, -0.6, 0), Vec3(-0.05, 0, 0))),
		        camera(Pose(rot(0, 0.6, 0), Vec3(0.05, 0, 0))), camera(Pose(rot(0.6, 0, 0), Vec3(0, 0.05, 0)))};
		// Ring axis across the view of camera 0, so that half of the LEDs face it.
		truth = Pose(rot(1.4, 0.3, 0.2), Vec3(0.04, -0.03, 0.6));
		ctx.hand = kLeft;
		ctx.model = &model;
		ctx.cams = &cams;
		ctx.q_ref = truth.q;
		ctx.q_3dof = truth.q;
	}

	//! Blobs of the given LEDs in camera 0, ordered by image x like a cluster.
	std::vector<InitBlob>
	blobs(const std::vector<int> &ids, double size, std::vector<int> &order) const
	{
		std::vector<std::pair<double, int>> by_x;
		std::vector<InitBlob> out;
		for (int id : ids) {
			Vec2 px;
			cams[0].world2cam(cams[0].T_HC.inverse() * (truth * model.position[id]), px);
			by_x.emplace_back(px.x(), id);
		}
		std::sort(by_x.begin(), by_x.end());
		order.clear();
		for (auto [x, id] : by_x) {
			(void)x;
			Vec2 px;
			cams[0].world2cam(cams[0].T_HC.inverse() * (truth * model.position[id]), px);
			InitBlob b;
			b.px = px;
			b.size = size;
			b.area = size * size;
			b.brightness = 255;
			b.circularity = 0.9;
			out.push_back(b);
			order.push_back(id);
		}
		return out;
	}

	//! LEDs facing camera 0, by id.
	std::vector<int>
	facing() const
	{
		Quat q_MC = truth.q.conjugate() * cams[0].T_HC.q;
		std::vector<int> ids;
		for (auto [i, c] : visible_leds(model, q_MC, (cams[0].T_HC.inverse() * truth.t).normalized())) {
			if (c > 0.3) {
				ids.push_back(i);
			}
		}
		std::sort(ids.begin(), ids.end());
		return ids;
	}
};

//! A run of `n` facing LEDs with consecutive ids.
static std::vector<int>
consecutive(const std::vector<int> &facing, size_t n)
{
	for (size_t s = 0; s + n <= facing.size(); s++) {
		if (facing[s + n - 1] - facing[s] == (int)n - 1) {
			return std::vector<int>(facing.begin() + s, facing.begin() + s + n);
		}
	}
	return {};
}

static void
test_min_area_rect()
{
	// A 4 x 2 rectangle turned by 30 degrees, with interior points.
	std::vector<Vec2> pts;
	Eigen::Rotation2Dd r(M_PI / 6);
	for (Vec2 p : {Vec2(0, 0), Vec2(4, 0), Vec2(4, 2), Vec2(0, 2), Vec2(1, 1), Vec2(3, 0.5)}) {
		pts.push_back(r * p + Vec2(10, 20));
	}
	EXPECT(std::abs(min_area_rect_area(pts) - 8.0) < 1e-9);
	EXPECT(min_area_rect_area({Vec2(0, 0), Vec2(1, 1)}) == 0.0);
}

static void
test_triple_search()
{
	Scene s;
	std::vector<int> run = consecutive(s.facing(), 3);
	EXPECT(run.size() == 3);
	if (run.size() != 3) {
		return;
	}
	std::vector<int> order;
	std::vector<InitBlob> b = s.blobs(run, 8, order);
	Vec2 px[3] = {b[0].px, b[1].px, b[2].px};
	Vec3 t;
	double E, angle;
	int ids[3];
	EXPECT(solve_p3p_with_prior(s.ctx, s.model, s.truth.q, 0, px, s.facing(), 5, t, E, ids, angle));
	EXPECT((t - s.truth.t).norm() < 1e-6 && E < 1e-4 && angle < 1e-3);
	EXPECT(ids[0] == order[0] && ids[1] == order[1] && ids[2] == order[2]);
	// Far outside the range box around the head: rejected.
	Scene far;
	far.truth.t = Vec3(0.0, 0.0, 1.6);
	std::vector<InitBlob> bf = far.blobs(run, 8, order);
	Vec2 pf[3] = {bf[0].px, bf[1].px, bf[2].px};
	EXPECT(!solve_p3p_with_prior(far.ctx, far.model, far.truth.q, 0, pf, far.facing(), 5, t, E, ids, angle));
}

static void
test_p5p_new()
{
	Scene s;
	std::vector<int> run = consecutive(s.facing(), 5);
	EXPECT(run.size() == 5);
	if (run.size() != 5) {
		return;
	}
	std::vector<int> order;
	std::vector<InitBlob> b = s.blobs(run, 8, order);
	// A prior 3 degrees off: the triple search holds it, EPnP over five blobs removes it.
	Quat Q = s.truth.q * Quat(Eigen::AngleAxisd(3 * M_PI / 180, Vec3(0.2, 1, 0.3).normalized()));
	InitResult r = init_p5p_new(s.ctx, 0, b, Q);
	EXPECT(r.ok && r.type == 2);
	EXPECT((r.T_WM.t - s.truth.t).norm() < 1e-6 && full_angle_deg(r.T_WM.q, s.truth.q) < 1e-4);
	EXPECT(r.ids == order);

	// Dim blobs half a metre away are not a controller.
	for (InitBlob &x : b) {
		x.brightness = 120;
	}
	EXPECT(!init_p5p_new(s.ctx, 0, b, Q).ok);
}

static void
test_p3p()
{
	Scene s;
	std::vector<int> run = consecutive(s.facing(), 3);
	if (run.size() != 3) {
		EXPECT(false);
		return;
	}
	std::vector<int> order;
	std::vector<InitBlob> b = s.blobs(run, 12, order);
	InitResult r = init_p3p(s.ctx, 0, b, s.truth.q);
	EXPECT(r.ok && r.type == 1);
	EXPECT((r.T_WM.t - s.truth.t).norm() < 1e-6);
	// No P3P while the other hand forbids it.
	s.ctx.no_p3p = true;
	EXPECT(!init_p3p(s.ctx, 0, b, s.truth.q).ok);
	s.ctx.no_p3p = false;
	// Small blobs, no window, recently lost: only near the stored pose.
	std::vector<InitBlob> small = s.blobs(run, 6, order);
	if (run[0] + run[1] + run[2] != 51) {
		EXPECT(!init_p3p(s.ctx, 0, small, s.truth.q).ok);
	}
	// ... or near the last good output position (P+0x13a0).
	s.ctx.last_good_p = s.truth.t + Vec3(0, 0.3, 0.3);
	EXPECT(init_p3p(s.ctx, 0, small, s.truth.q).ok);
	s.ctx.last_good_p = s.truth.t + Vec3(0, 0.4, 0.4);
	if (run[0] + run[1] + run[2] != 51) {
		EXPECT(!init_p3p(s.ctx, 0, small, s.truth.q).ok);
	}
	s.ctx.last_good_p = Vec3::Zero();
	// The sanity does not apply while the other hand is locked, not followed and has a Body.
	if (run[0] + run[1] + run[2] != 51) {
		s.ctx.other.locked = true;
		s.ctx.other.active = true;
		EXPECT(init_p3p(s.ctx, 0, small, s.truth.q).ok);
		s.ctx.follow = true;
		EXPECT(!init_p3p(s.ctx, 0, small, s.truth.q).ok);
		s.ctx.follow = false;
		// Locked without a Body: only if it has Bodies in its window.
		s.ctx.other.active = false;
		s.ctx.other.body_window_empty = true;
		EXPECT(init_p3p(s.ctx, 0, small, s.truth.q).ok);
		s.ctx.other.body_window_empty = false;
		EXPECT(!init_p3p(s.ctx, 0, small, s.truth.q).ok);
		// Back to a fresh OtherHand's values, field by field: a moved temporary would carry its
		// unset quaternions, which GCC's -Wmaybe-uninitialized rejects.
		s.ctx.other.locked = false;
		s.ctx.other.body_window_empty = true;
		// Small-blob gate after 1501 failed acquisitions: refused, unless the other hand is
		// locked with a Body.
		s.ctx.init_failures = 1501;
		s.ctx.last_good_p = s.truth.t;
		EXPECT(!init_p3p(s.ctx, 0, small, s.truth.q).ok);
		s.ctx.other.locked = true;
		s.ctx.other.active = true;
		EXPECT(init_p3p(s.ctx, 0, small, s.truth.q).ok);
		s.ctx.other.locked = false;
		s.ctx.other.active = false;
		s.ctx.init_failures = 0;
		s.ctx.last_good_p = Vec3::Zero();
	}
	// The three most circular of four blobs, in any input order.
	std::vector<InitBlob> four = b;
	InitBlob stray = b[0];
	stray.px += Vec2(40, 25);
	stray.circularity = 0.5;
	four.insert(four.begin() + 1, stray);
	EXPECT(init_p3p(s.ctx, 0, four, s.truth.q).ok);
	std::reverse(four.begin(), four.end());
	InitResult r4 = init_p3p(s.ctx, 0, four, s.truth.q);
	EXPECT(r4.ok && (r4.T_WM.t - s.truth.t).norm() < 1e-6);
	s.ctx.pose0_valid = true;
	s.ctx.pose0 = Pose(s.truth.q, s.truth.t + Vec3(0.1, 0, 0));
	EXPECT(init_p3p(s.ctx, 0, small, s.truth.q).ok);
	// Tracked: a jump of 0.45 m upwards from the stored pose is refused.
	s.ctx.tracked = true;
	s.ctx.pose0 = Pose(s.truth.q, s.truth.t + Vec3(0.46, 0, 0));
	EXPECT(!init_p3p(s.ctx, 0, b, s.truth.q).ok);
}

/*
 * Re-acquisition of a tracked hand from three blobs of the other controller, half a metre in
 * front of camera 0: with this hand's prior the triple search finds a wrong pose next to the head
 * (the head frame is W here, camera 0 at its origin). Stock fits the same blobs with the other
 * hand on the other predictor, with its own state, and keeps ours only if that fits worse.
 */
static void
test_p3p_other_hand()
{
	Scene s;
	// The candidate LEDs of the other hand's fit come from this hand's LEDs (0x38531c): one model
	// for both hands isolates the other hand's own state.
	const LedModel &model = s.model;
	const Pose other_truth(rot(1.4, 0.3, 0.2), Vec3(0.04, -0.03, 0.45));
	const Quat Q = other_truth.q * rot(-0.2, -0.4, -1.2); // this hand's prior
	std::vector<InitBlob> b;
	for (int id : {12, 14, 17}) {
		InitBlob x;
		s.cams[0].world2cam(other_truth * model.position[id], x.px);
		x.size = 12;
		x.area = 144;
		x.brightness = 255;
		x.circularity = 0.9;
		b.push_back(x);
	}
	std::sort(b.begin(), b.end(), [](const InitBlob &l, const InitBlob &r) { return l.px.x() < r.px.x(); });

	InitContext &ctx = s.ctx;
	ctx.hand = kRight;
	ctx.tracked = true; // reInitControllerPose
	ctx.body_window_empty = false;
	ctx.q_ref = Q;
	ctx.q_3dof = Q;
	ctx.pose0_valid = true;
	ctx.pose0 = Pose(Q, Vec3(0.0, -0.05, 0.25));

	// Alone, the wrong pose 8 cm from the head passes every test: stock has no lower bound on the
	// distance to the head, and saturated blobs pass the brightness table.
	InitResult r = init_p3p(ctx, 0, b, Q);
	EXPECT(r.ok && r.T_WM.t.norm() < 0.1 && (r.T_WM.t - other_truth.t).norm() > 0.3);
	EXPECT(!(r.error_px < 0.7 && r.angle_deg < 8)); // so the other hand's fit runs
	// Unless saturated, blobs of a controller nearer than 0.5 m are too dim (0x37da64).
	std::vector<InitBlob> dim = b;
	for (InitBlob &x : dim) {
		x.brightness = 249;
	}
	EXPECT(!init_p3p(ctx, 0, dim, Q).ok);

	// The other hand, lost (not tracked, no Bodies): its prior and reference rotation are its 3DoF
	// orientation, and it explains the blobs exactly. Ours is refused.
	ctx.other.active = true;
	ctx.other.locked = false;
	ctx.other.model = &model;
	ctx.other.hand = kLeft;
	ctx.other.prior = other_truth.q;
	ctx.other.q_3dof = other_truth.q;
	ctx.other.tracked = false;
	ctx.other.body_window_empty = true;
	EXPECT(!init_p3p(ctx, 0, b, Q).ok);

	// Its fit reads its own pose0 test: tracked with its pose a metre away, its fit fails and ours
	// is kept (with this hand's state it would fail the same way, which let the wrong pose through).
	ctx.other.tracked = true;
	ctx.other.body_window_empty = false;
	ctx.other.pose0_valid = true;
	ctx.other.pose0 = Pose(other_truth.q, other_truth.t + Vec3(0, 1.0, 0));
	EXPECT(init_p3p(ctx, 0, b, Q).ok);
	ctx.other.pose0 = Pose(other_truth.q, other_truth.t + Vec3(0.1, 0, 0));
	EXPECT(!init_p3p(ctx, 0, b, Q).ok);

	// This hand's own blobs with its prior still pass with the other hand present.
	std::vector<int> run = consecutive(s.facing(), 3), order;
	EXPECT(run.size() == 3);
	if (run.size() == 3) {
		ctx.q_ref = s.truth.q;
		ctx.pose0 = Pose(s.truth.q, s.truth.t + Vec3(0.1, 0, 0));
		InitResult own = init_p3p(ctx, 0, s.blobs(run, 12, order), s.truth.q);
		EXPECT(own.ok && (own.T_WM.t - s.truth.t).norm() < 1e-6);
	}
}

static void
test_hands()
{
	Scene s;
	// Right hand near the head (pico4 W: +x up; the head frame here is W).
	s.ctx.hand = kRight;
	EXPECT(judge_near(s.ctx, Pose(Quat::Identity(), Vec3(0.0, 0.05, 0.1))));
	EXPECT(!judge_near(s.ctx, Pose(Quat::Identity(), Vec3(0.0, 0.15, 0.1))));
	s.ctx.hand = kLeft;
	EXPECT(!judge_near(s.ctx, Pose(Quat::Identity(), Vec3(0.0, 0.1, 0.1))));

	// Both arms out sideways: a pose next to the other hand, far from this hand, is refused.
	s.ctx.pose0_valid = true;
	s.ctx.pose0 = Pose(Quat::Identity(), Vec3(0, 0.8, 0.2));
	s.ctx.other.active = true;
	s.ctx.other.pose0_valid = true;
	s.ctx.other.pose0 = Pose(Quat::Identity(), Vec3(0, -0.8, 0.2));
	EXPECT(!raise_both_hands_sideways(s.ctx, Pose(Quat::Identity(), Vec3(0, -0.75, 0.2))));
	EXPECT(raise_both_hands_sideways(s.ctx, Pose(Quat::Identity(), Vec3(0, 0.7, 0.2))));
	s.ctx.other.active = false;
	EXPECT(raise_both_hands_sideways(s.ctx, Pose(Quat::Identity(), Vec3(0, -0.75, 0.2))));
}

int
main()
{
	test_min_area_rect();
	test_triple_search();
	test_p5p_new();
	test_p3p();
	test_p3p_other_hand();
	test_hands();
	if (failures == 0) {
		printf("test_pico4_ctrack_init: all checks passed\n");
	}
	return failures == 0 ? 0 : 1;
}
