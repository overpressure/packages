// Copyright 2026, overpressure contributors.
// SPDX-License-Identifier: BSL-1.0
/*
 * Unit test of the controller tracker's cameras, identification table and solvers with
 * synthetic data (no device data): camera round trip, P3P, EPnP, translation with a known
 * rotation, the shift-hypothesis association, the multi-camera refinement and the 5-LED lookup.
 */

#include "pico4_ctrack_model.hpp"
#include "pico4_ctrack_solve.hpp"

#include <cmath>
#include <cstdio>
#include <random>

using namespace pico4::ctrack;

static int failures;

#define EXPECT(cond)                                                                                                   \
	do {                                                                                                           \
		if (!(cond)) {                                                                                         \
			fprintf(stderr, "%s:%d: expected %s\n", __FILE__, __LINE__, #cond);                            \
			failures++;                                                                                    \
		}                                                                                                      \
	} while (0)

//! A camera like the PICO 4's: omni model, mild distortion, looking along its +z.
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

static void
test_camera()
{
	Camera c = camera(Pose());
	double worst = 0;
	for (double x = -0.6; x <= 0.6; x += 0.2) {
		for (double y = -0.5; y <= 0.5; y += 0.25) {
			Vec3 p(x, y, 0.8);
			Vec2 px;
			EXPECT(c.world2cam(p, px));
			worst = std::max(worst, (c.cam2world(px) - p.normalized()).norm());
		}
	}
	EXPECT(worst < 1e-9);
	Vec2 px;
	EXPECT(!c.project2d(Vec3(0, 0, -1), px) && px.x() == -1000.0);
}

static void
test_p3p_and_epnp()
{
	std::mt19937 rng(7);
	std::uniform_real_distribution<double> u(-1, 1);
	LedModel m = ring();
	double worst_p3p = 0, worst_epnp = 0;
	for (int trial = 0; trial < 50; trial++) {
		Pose T(rot(u(rng), u(rng), u(rng)), Vec3(0.1 * u(rng), 0.1 * u(rng), 0.6 + 0.3 * u(rng)));
		Vec3 f[3], P[3];
		for (int k = 0; k < 3; k++) {
			P[k] = m.position[3 * k + 1];
			f[k] = (T * P[k]).normalized();
		}
		double best = 1e9;
		for (const Pose &s : p3p(f, P)) {
			best = std::min(best, (s.t - T.t).norm() + full_angle_deg(s.q, T.q) * M_PI / 180);
		}
		worst_p3p = std::max(worst_p3p, best);

		std::vector<Vec2> uv;
		std::vector<Vec3> pts;
		for (int i = 0; i < kLeds; i += 2) {
			Vec3 q = T * m.position[i];
			uv.emplace_back(q.x() / q.z(), q.y() / q.z());
			pts.push_back(m.position[i]);
		}
		Pose e;
		EXPECT(epnp(uv, pts, e));
		worst_epnp = std::max(worst_epnp, (e.t - T.t).norm() + full_angle_deg(e.q, T.q) * M_PI / 180);
	}
	EXPECT(worst_p3p < 1e-6);
	EXPECT(worst_epnp < 1e-6);
}

static void
test_translation_with_prior()
{
	LedModel m = ring();
	Quat R = rot(0.3, -0.5, 1.1);
	Vec3 t(0.05, -0.2, 0.7);
	Pose T_WM(R, t);
	Pose cams[2] = {Pose(rot(0, 0.1, 0), Vec3(0, 0.05, 0)), Pose(rot(0, -0.1, 0.2), Vec3(0, -0.06, 0))};
	RayObservation o[3];
	for (int k = 0; k < 3; k++) {
		const Pose &C = cams[k % 2];
		Vec3 P = m.position[5 * k];
		o[k] = RayObservation{C.q.toRotationMatrix(), C.t, (C.inverse() * (T_WM * P)).normalized(), P};
	}
	Vec3 got;
	EXPECT(translation_p2p(R.toRotationMatrix(), o[0], o[1], got) && (got - t).norm() < 1e-9);
	EXPECT(translation_p3(R.toRotationMatrix(), o, got) && (got - t).norm() < 1e-9);
}

static void
test_match_points()
{
	// Projections on a grid 40 px apart; blobs shifted by (7, -5) with one missing and one extra.
	std::vector<Vec2> proj, blobs;
	for (int i = 0; i < 8; i++) {
		proj.emplace_back(200 + 40 * (i % 4), 200 + 40 * (i / 4) + 3 * i);
	}
	std::vector<int> truth;
	for (int i = 7; i >= 0; i--) {
		if (i == 2) {
			continue;
		}
		// Distinct small errors: stock's association drops entries with exactly equal distances.
		blobs.push_back(proj[i] + Vec2(7, -5) + Vec2(0.31 * std::sin(i + 1.0), -0.27 * std::cos(1.7 * i)));
		truth.push_back(i);
	}
	blobs.emplace_back(500, 100);
	truth.push_back(-1);
	std::vector<int> match;
	std::vector<double> err;
	match_points(proj, blobs, match, err);
	for (size_t b = 0; b < blobs.size(); b++) {
		EXPECT(match[b] == truth[b]);
	}
	EXPECT(err.back() == 100.0);
	EXPECT(std::abs(err[0] - (blobs[0] - proj[7]).norm()) < 1e-12);
}

static void
test_refine()
{
	LedModel m = ring();
	std::array<Camera, kCameras> cams = {camera(Pose(rot(0, 0.5, 0), Vec3(0.05, 0.05, 0))),
	                                     camera(Pose(rot(0, -0.5, 0), Vec3(-0.05, 0.05, 0))),
	                                     camera(Pose(rot(0.4, 0.3, 0), Vec3(0.05, -0.05, 0))),
	                                     camera(Pose(rot(0.4, -0.3, 0), Vec3(-0.05, -0.05, 0)))};
	Pose truth(rot(0.4, -0.2, 0.9), Vec3(0.02, -0.03, 0.6));
	std::vector<Feature> feats;
	for (int c = 0; c < kCameras; c++) {
		for (int i = 0; i < kLeds; i++) {
			Vec3 p = cams[c].T_HC.inverse() * (truth * m.position[i]);
			Vec2 px;
			if (!cams[c].world2cam(p, px) || px.x() < 0 || px.x() > 639 || px.y() < 0 || px.y() > 479) {
				continue;
			}
			Feature f;
			f.cam = c;
			f.bearing = cams[c].cam2world(px);
			f.P = m.position[i];
			f.led = i;
			f.pixel = px;
			feats.push_back(f);
		}
	}
	EXPECT(feats.size() >= 10);
	Pose start(truth.q * Quat(Eigen::AngleAxisd(0.05, Vec3(1, 2, 3).normalized())), truth.t + Vec3(0.01, -0.008, 0.015));
	// With the IMU's rotation as the prior (here the true one) a perturbed start converges.
	Pose T = start;
	double e0, e1;
	EXPECT(refine_pose(cams, feats, truth, T, e0, e1));
	EXPECT((T.t - truth.t).norm() < 1e-6 && full_angle_deg(T.q, truth.q) < 1e-3);
	EXPECT(e1 < 0.01 && e0 > 1.0);
	// A wrong prior holds the rotation near it (weight 0.1): the position still converges.
	T = start;
	EXPECT(refine_pose(cams, feats, start, T, e0, e1));
	EXPECT((T.t - truth.t).norm() < 1e-3 && full_angle_deg(T.q, truth.q) < full_angle_deg(start.q, truth.q));

	// Translation with the true rotation from three features in two cameras.
	std::vector<Feature> few;
	for (const Feature &f : feats) {
		if ((int)few.size() < 3 && (f.cam == 0 || f.cam == 1)) {
			few.push_back(f);
		}
	}
	Vec3 t;
	std::vector<double> errs;
	EXPECT(few_points_translation(cams, Pose(), truth.q, few, false, t, errs) && (t - truth.t).norm() < 1e-6);
}

static void
test_id_table()
{
	LedModel m = ring();
	IdTable table;
	table.generate(m);
	EXPECT(!table.records().empty());
	// Every record's basis maps to (-1, 0), (1, 0): check the descriptor directly.
	Vec3 f_a = Vec3(-0.1, 0.02, 1).normalized(), f_b = Vec3(0.12, -0.01, 1).normalized(), other = Vec3(0, 0.1, 1).normalized();
	Vec2 o;
	basis_descriptor(f_a, f_b, &other, 1, &o);
	Vec2 ab[2];
	Vec3 both[2] = {f_a, f_b};
	basis_descriptor(f_a, f_b, both, 2, ab);
	EXPECT((ab[0] - Vec2(-1, 0)).norm() < 1e-9 && (ab[1] - Vec2(1, 0)).norm() < 1e-9);
	// A camera rotation leaves the descriptor unchanged.
	Quat r = rot(0.2, -0.3, 0.5);
	Vec3 rf_a = r * f_a, rf_b = r * f_b, ro = r * other;
	Vec2 o2;
	basis_descriptor(rf_a, rf_b, &ro, 1, &o2);
	EXPECT((o - o2).norm() < 1e-9);

	// Look up a group as seen from a table viewpoint direction at 0.5 m, blobs in shuffled order.
	const IdRecord &rec = table.records()[table.records().size() / 3];
	Vec3 centroid = Vec3::Zero();
	for (const Vec3 &p : m.position) {
		centroid += p;
	}
	centroid /= kLeds;
	// Camera centre: on the side all five LEDs face.
	Vec3 dir = Vec3::Zero();
	for (int id : rec.ids) {
		dir += m.normal[id];
	}
	Vec3 cpos = centroid + 0.5 * dir.normalized();
	int order[5] = {3, 0, 4, 1, 2};
	std::vector<Vec3> bearings;
	for (int k : order) {
		bearings.push_back((m.position[rec.ids[k]] - cpos).normalized());
	}
	auto lists = table.lookup(bearings, false);
	bool found = false;
	for (const auto &l : lists) {
		bool all = true;
		for (auto [blob, led] : l) {
			all = all && rec.ids[order[blob]] == led;
		}
		found = found || all;
	}
	EXPECT(found);
	// The right hand mirrors: the mirrored model seen mirrored gives the same correspondence.
	std::vector<Vec3> mirrored;
	for (const Vec3 &b : bearings) {
		mirrored.emplace_back(b.x(), -b.y(), b.z());
	}
	auto lists_r = table.lookup(mirrored, true);
	found = false;
	for (const auto &l : lists_r) {
		bool all = true;
		for (auto [blob, led] : l) {
			all = all && rec.ids[order[blob]] == led;
		}
		found = found || all;
	}
	EXPECT(found);
}

int
main()
{
	test_camera();
	test_p3p_and_epnp();
	test_translation_with_prior();
	test_match_points();
	test_refine();
	test_id_table();
	if (failures == 0) {
		printf("test_pico4_ctrack_solve: all checks passed\n");
	}
	return failures == 0 ? 0 : 1;
}
