// Copyright 2026, overpressure contributors.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Acquisition of a controller from one camera (see pico4_ctrack_init.hpp).
 * @ingroup drv_pico4
 */

#include "pico4_ctrack_init.hpp"

#include "pico4_ctrack_blob.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <numeric>

namespace pico4::ctrack {

namespace {

//! Rejection log of the acquisition paths (PICO4_CTRACK_TRACE set), like stock's log lines.
bool
trace_on()
{
	static const bool on = getenv("PICO4_CTRACK_TRACE") != nullptr;
	return on;
}

#define TRACE(...)                                                                                                     \
	do {                                                                                                           \
		if (trace_on()) {                                                                                      \
			fprintf(stderr, __VA_ARGS__);                                                                  \
		}                                                                                                      \
	} while (0)

Pose
camera_pose(const InitContext &ctx, int cam)
{
	return ctx.T_WH * (*ctx.cams)[cam].T_HC;
}

//! Normalised pinhole coordinates (x/z, y/z) of a pixel's bearing.
Vec2
normalised(const Camera &c, const Vec2 &px)
{
	Vec3 f = c.cam2world(px);
	return Vec2(f.x() / f.z(), f.y() / f.z());
}

bool
in_range_box(const InitContext &ctx, const Pose &T_WM)
{
	Vec3 h = (ctx.T_WH.inverse() * T_WM).t;
	return std::abs(h.x()) < 1.2 && std::abs(h.y()) < 1.2 && std::abs(h.z()) < 0.8 && h.norm() < 1.5;
}

//! LED triples a mode-3 (P3P) result may not use: ambiguous configurations of the model.
bool
forbidden_triple(int a, int b, int c, bool juxtaposed)
{
	static const int always[6][3] = {{0, 3, 6}, {2, 4, 5}, {3, 4, 8}, {4, 5, 8}, {3, 7, 9}, {5, 7, 10}};
	static const int unless_juxtaposed[4][3] = {{6, 10, 12}, {8, 11, 12}, {9, 11, 12}, {10, 11, 12}};
	for (const auto &t : always) {
		if (t[0] == a && t[1] == b && t[2] == c) {
			return true;
		}
	}
	if (!juxtaposed) {
		for (const auto &t : unless_juxtaposed) {
			if (t[0] == a && t[1] == b && t[2] == c) {
				return true;
			}
		}
	}
	return false;
}

//! Mean blob size and centre.
void
blob_stats(const std::vector<InitBlob> &blobs, double &mean_size, Vec2 &centre)
{
	mean_size = 0;
	centre = Vec2::Zero();
	for (const InitBlob &b : blobs) {
		mean_size += b.size;
		centre += b.px;
	}
	mean_size /= std::max<size_t>(1, blobs.size());
	centre /= std::max<size_t>(1, blobs.size());
}

std::vector<int>
facing_leds(const InitContext &ctx, const LedModel &model, int cam, const Quat &Q, const Vec2 &centre)
{
	const Camera &c = (*ctx.cams)[cam];
	Quat q_MC = Q.conjugate() * ctx.T_WH.q * c.T_HC.q; // camera -> M
	std::vector<int> ids;
	for (auto [i, cosv] : visible_leds(model, q_MC, c.cam2world(centre))) {
		(void)cosv;
		ids.push_back(i);
	}
	return ids;
}

double
error_limit_by_size(double size, double smallest)
{
	return size > 20 ? 1.5 : size > 15 ? 1.2 : size > 10 ? 1.0 : smallest;
}

} // namespace


InitContext
other_hand_context(const InitContext &ctx, const Quat &Q)
{
	const OtherHand &o = ctx.other;
	InitContext c;
	c.hand = o.hand;
	c.model = o.model;
	c.cams = ctx.cams;
	c.T_WH = ctx.T_WH;
	c.tracked = o.tracked;
	c.pose0_valid = o.pose0_valid;
	c.pose0 = o.pose0;
	c.body_window_empty = o.body_window_empty;
	c.lost_frames = o.lost_frames;
	c.q_3dof = o.q_3dof;
	c.q_ref = (c.body_window_empty || c.lost_frames > 330) ? c.q_3dof : Q;
	c.juxtaposed = o.juxtaposed;
	c.occlusion = o.occlusion;
	c.detect_around = o.detect_around;
	return c;
}


/*
 *
 * The three-LED search with the prior rotation (solveP3PWithPriorR).
 *
 */

bool
solve_p3p_with_prior(const InitContext &ctx, const LedModel &model, const Quat &Q, int cam, const Vec2 px[3],
                     const std::vector<int> &candidates, int mode, Vec3 &t_WM, double &error_px, int ids[3],
                     double &angle_deg)
{
	const Camera &c = (*ctx.cams)[cam];
	Pose T_WC = camera_pose(ctx, cam);
	Quat R_CM = T_WC.q.conjugate() * Q;
	Mat3 R = R_CM.toRotationMatrix();
	Vec2 uv[3];
	for (int s = 0; s < 3; s++) {
		uv[s] = normalised(c, px[s]);
	}
	double best = 10000;
	bool kept = false;
	Vec3 best_tC;
	int n = (int)candidates.size();
	// Stage 1: every ordered LED triple, rotation fixed, translation by linear least squares.
	for (int i = 0; i < n; i++) {
		for (int j = 0; j < n; j++) {
			for (int k = 0; k < n; k++) {
				if (i == j || j == k || i == k) {
					continue;
				}
				int tri[3] = {candidates[i], candidates[j], candidates[k]};
				int s3[3] = {tri[0], tri[1], tri[2]};
				std::sort(s3, s3 + 3);
				int g1 = s3[1] - s3[0], g2 = s3[2] - s3[1];
				int gap = mode == 3 ? 5 : 4;
				if (g1 > gap || g2 > gap || (g1 == 4 && g2 == 3) || (g1 == 3 && g2 == 4)) {
					continue;
				}
				Eigen::Matrix<double, 6, 3> A;
				Eigen::Matrix<double, 6, 1> b;
				for (int s = 0; s < 3; s++) {
					const Vec3 &P = model.position[tri[s]];
					A.row(2 * s) << 1, 0, -uv[s].x();
					A.row(2 * s + 1) << 0, 1, -uv[s].y();
					b(2 * s) = (uv[s].x() * R.row(2) - R.row(0)).dot(P);
					b(2 * s + 1) = (uv[s].y() * R.row(2) - R.row(1)).dot(P);
				}
				Vec3 tC = (A.transpose() * A).ldlt().solve(A.transpose() * b);
				if (!tC.allFinite() || tC.z() < -0.075) {
					continue;
				}
				double E = 0;
				for (int s = 0; s < 3; s++) {
					Vec2 q;
					if (!c.world2cam(R * model.position[tri[s]] + tC, q)) {
						E = 1e9;
						break;
					}
					E += (q - px[s]).norm();
				}
				if (E < best) {
					best = E;
					kept = true;
					best_tC = tC;
					std::copy(tri, tri + 3, ids);
				}
			}
		}
	}
	if (!kept) {
		TRACE("  triple cam %d: no triple\n", cam);
		return false;
	}
	error_px = best;
	t_WM = T_WC.q * best_tC + T_WC.t;

	// Stage 2: P3P on the best triple as a consistency check.
	Vec3 f[3], P[3];
	for (int s = 0; s < 3; s++) {
		f[s] = c.cam2world(px[s]);
		P[s] = model.position[ids[s]];
	}
	angle_deg = 90.0;
	Vec3 t_p3p = Vec3::Constant(1e9);
	for (const Pose &T_CM : p3p(f, P)) {
		Pose T = T_WC * T_CM;
		double a = half_angle_deg(T.q, ctx.q_ref);
		if (a < angle_deg) {
			angle_deg = a;
			t_p3p = T.t;
		}
	}

	int s3[3] = {ids[0], ids[1], ids[2]};
	std::sort(s3, s3 + 3);
	if (mode == 3 && forbidden_triple(s3[0], s3[1], s3[2], ctx.juxtaposed)) {
		TRACE("  triple cam %d: forbidden %d %d %d\n", cam, s3[0], s3[1], s3[2]);
		return false;
	}
	double limit = 6.0;
	if (s3[0] + s3[1] + s3[2] < 7) {
		limit = 10.0;
	}
	if ((s3[0] == 13 && s3[1] == 15 && s3[2] == 17) || (s3[0] == 15 && s3[1] == 17 && s3[2] == 19)) {
		limit = 9.0;
	}
	if (ctx.juxtaposed) {
		if (!(angle_deg < 10.0 && error_px < 4.0)) {
			TRACE("  triple cam %d: juxtaposed E %.2f angle %.1f\n", cam, error_px, angle_deg);
			return false;
		}
	} else if (!(error_px < limit && angle_deg < 9.0)) {
		TRACE("  triple cam %d: ids %d %d %d E %.2f (< %.0f) angle %.1f (< 9)\n", cam, s3[0], s3[1], s3[2], error_px, limit,
		      angle_deg);
		return false;
	}
	if (!in_range_box(ctx, Pose(Q, t_WM))) {
		TRACE("  triple cam %d: outside the range box\n", cam);
		return false;
	}
	if ((t_p3p - t_WM).norm() >= 0.15) {
		TRACE("  triple cam %d: P3P %.3f m away\n", cam, (t_p3p - t_WM).norm());
		return false;
	}
	if (mode == 3 && ctx.tracked) {
		Vec3 d = t_WM - ctx.pose0.t;
		if (d.norm() >= 0.5 || std::abs(d.x()) >= 0.45) {
			TRACE("  triple cam %d: %.2f m from the last pose\n", cam, d.norm());
			return false;
		}
	}
	return true;
}


/*
 *
 * Plausibility tests.
 *
 */

double
min_area_rect_area(std::vector<Vec2> pts)
{
	if (pts.size() < 3) {
		return 0.0;
	}
	// Convex hull (monotone chain), then the best edge direction.
	std::sort(pts.begin(), pts.end(), [](const Vec2 &a, const Vec2 &b) {
		return a.x() < b.x() || (a.x() == b.x() && a.y() < b.y());
	});
	auto cross = [](const Vec2 &o, const Vec2 &a, const Vec2 &b) {
		return (a - o).x() * (b - o).y() - (a - o).y() * (b - o).x();
	};
	std::vector<Vec2> h(2 * pts.size());
	size_t k = 0;
	for (size_t i = 0; i < pts.size(); i++) {
		while (k >= 2 && cross(h[k - 2], h[k - 1], pts[i]) <= 0) {
			k--;
		}
		h[k++] = pts[i];
	}
	for (size_t i = pts.size() - 1, t = k + 1; i > 0; i--) {
		while (k >= t && cross(h[k - 2], h[k - 1], pts[i - 1]) <= 0) {
			k--;
		}
		h[k++] = pts[i - 1];
	}
	h.resize(k - 1);
	if (h.size() < 3) {
		return 0.0;
	}
	double best = 1e18;
	for (size_t i = 0; i < h.size(); i++) {
		Vec2 e = (h[(i + 1) % h.size()] - h[i]).normalized(), n(-e.y(), e.x());
		double lo_e = 1e18, hi_e = -1e18, lo_n = 1e18, hi_n = -1e18;
		for (const Vec2 &p : h) {
			lo_e = std::min(lo_e, p.dot(e));
			hi_e = std::max(hi_e, p.dot(e));
			lo_n = std::min(lo_n, p.dot(n));
			hi_n = std::max(hi_n, p.dot(n));
		}
		best = std::min(best, (hi_e - lo_e) * (hi_n - lo_n));
	}
	return best;
}

bool
check_project_area(const InitContext &ctx, const LedModel &model, const Pose &T_WM, double blob_size,
                   const std::vector<InitBlob> &blobs, const std::vector<int> &ids, int cam, double E,
                   double angle, int mode)
{
	const Camera &c = (*ctx.cams)[cam];
	Pose T_CM = camera_pose(ctx, cam).inverse() * T_WM;
	std::vector<Vec2> L2;
	std::array<Vec3, kLeds> L3;
	for (int i = 0; i < kLeds; i++) {
		L3[i] = T_CM * model.position[i];
		Vec2 q;
		c.world2cam(L3[i], q);
		L2.push_back(q);
	}
	double b_max = 0;
	for (const InitBlob &b : blobs) {
		b_max = std::max(b_max, b.brightness);
	}
	Vec3 h = (ctx.T_WH.inverse() * T_WM).t;
	double d = h.norm(), z = std::abs(h.z());
	// A near controller makes bright blobs.
	TRACE("  area cam %d: distance %.2f m, |z| %.2f, brightest %.0f, blob size %.1f\n", cam, d, z, b_max, blob_size);
	if ((b_max <= 249 && d < 0.5) || (b_max <= 219 && d < 0.6)) {
		TRACE("  area cam %d: too dim for the distance\n", cam);
		return false;
	}
	if (d < 0.7 && ((b_max <= 199 && z > 0.2) || (b_max <= 179 && z < 0.2))) {
		TRACE("  area cam %d: too dim for the distance\n", cam);
		return false;
	}
	if (d < 0.8 && ((b_max <= 149 && z > 0.2) || (b_max <= 109 && z < 0.2))) {
		TRACE("  area cam %d: too dim for the distance\n", cam);
		return false;
	}
	if (d < 0.9 && ((b_max <= 119 && z > 0.2) || (b_max <= 99 && z < 0.2))) {
		TRACE("  area cam %d: too dim for the distance\n", cam);
		return false;
	}
	if (b_max <= 89 && d < 1.0) {
		TRACE("  area cam %d: too dim for the distance\n", cam);
		return false;
	}
	if (std::abs(h.x()) < 0.25 && std::abs(h.y()) < 0.25 && b_max <= 249 && z < 0.45) {
		TRACE("  area cam %d: too dim for the distance\n", cam);
		return false;
	}
	// The blob of the LED nearest to the camera is the brightest and largest.
	int nearest = -1;
	double d_min_mm = 1e18;
	for (size_t b = 0; b < blobs.size() && b < ids.size(); b++) {
		if (ids[b] >= 0 && L3[ids[b]].norm() * 1000.0 < d_min_mm) {
			d_min_mm = L3[ids[b]].norm() * 1000.0;
			nearest = (int)b;
		}
	}
	if (nearest >= 0 && ctx.occlusion < 0.1 && blob_size > 3 && blob_size < 10 && d_min_mm > 300) {
		for (const InitBlob &b : blobs) {
			if (1.2 * blobs[nearest].brightness < b.brightness || 1.8 * blobs[nearest].size < b.size) {
				TRACE("  area cam %d: the nearest LED's blob is not the brightest\n", cam);
				return false;
			}
		}
	}
	std::vector<double> sizes;
	for (const InitBlob &b : blobs) {
		sizes.push_back(b.size);
	}
	std::sort(sizes.begin(), sizes.end());
	if (sizes.size() >= 3) {
		double spread = sizes[2] - sizes[0];
		double lim = d <= 0.5 ? 1e9 : d <= 0.6 ? 4.5 : d <= 0.7 ? 3.5 : d <= 0.8 ? 3.0 : d <= 0.9 ? 2.5 : 2.0;
		if (spread > lim) {
			TRACE("  area cam %d: size spread %.1f > %.1f\n", cam, spread, lim);
			return false;
		}
	}
	double area = min_area_rect_area(L2);
	double s = blob_size;
	if ((s > 0 && s <= 2 && area > 1500) || (s <= 3 && area > 2300) || (s <= 4 && area > 5500) ||
	    (s <= 5 && area > 6500)) {
		TRACE("  area cam %d: footprint %.0f px2 for blob size %.1f\n", cam, area, s);
		return false;
	}
	// A far, small controller must also show where predicted in the other cameras.
	if (blob_size < 10 && d_min_mm > 300 && !ctx.tracked && ctx.detect_around) {
		int n_vis = 0, n_match = 0;
		for (int k = 0; k < kCameras; k++) {
			if (k == cam) {
				continue;
			}
			int v = 0, m = 0;
			ctx.detect_around(k, T_WM, v, m);
			n_vis += v;
			n_match += m;
		}
		int required = 2;
		if (mode == 3) {
			if ((E < 1.5 && angle < 5) || (E < 1.0 && angle < 6)) {
				required = 1;
			}
			if (E < 0.5 && angle < 5) {
				required = 0;
			}
		}
		if (n_vis >= 1 && n_match < required) {
			TRACE("  area cam %d: other cameras %d of %d predicted LEDs seen, need %d\n", cam, n_match, n_vis, required);
			return false;
		}
	}
	return true;
}

bool
raise_both_hands_sideways(const InitContext &ctx, const Pose &X)
{
	if (!ctx.other.active || !ctx.pose0_valid || !ctx.other.pose0_valid) {
		return true;
	}
	Pose A = ctx.T_WH.inverse() * ctx.pose0, B = ctx.T_WH.inverse() * ctx.other.pose0;
	bool far_apart = A.t.normalized().dot(B.t.normalized()) < -0.6 && (A.inverse() * B).t.norm() > 1.5 &&
	                 std::abs(A.t.x() - B.t.x()) < 0.4;
	if (!far_apart) {
		return true;
	}
	return !((X.inverse() * ctx.pose0).t.norm() > 1.0 && (X.inverse() * ctx.other.pose0).t.norm() < 0.2);
}

bool
judge_near(const InitContext &ctx, const Pose &T_WM)
{
	double dmin = 1000;
	for (int k = 0; k < kCameras; k++) {
		dmin = std::min(dmin, (camera_pose(ctx, k).inverse() * T_WM).t.norm());
	}
	Vec3 h = (ctx.T_WH.inverse() * T_WM).t;
	if (ctx.hand == kRight) {
		return dmin < 0.2 && h.z() > -0.2 && std::abs(h.y()) < 0.12;
	}
	return dmin < 0.2 && h.z() > -0.18 && std::abs(h.y()) < 0.09;
}


/*
 *
 * Extension to all blobs (checkPromoteP3P).
 *
 */

namespace {

bool
promote(const InitContext &ctx, const LedModel &model, int cam, const std::vector<int> &candidates,
        const std::vector<Vec2> &px, const int ids3[3], const Pose &T_WM, std::vector<int> &ids_out, Pose &final_pose,
        double &mean_err, double blob_size, double &angle_out)
{
	if (px.size() < 5) {
		return false;
	}
	const Camera &c = (*ctx.cams)[cam];
	Pose T_WC = camera_pose(ctx, cam);
	Pose T_CM = T_WC.inverse() * T_WM;
	double gate = blob_size > 17 ? 7.0 : 5.0;
	ids_out.assign(ids3, ids3 + 3);
	std::vector<bool> used(kLeds, false);
	for (int s = 0; s < 3; s++) {
		used[ids3[s]] = true;
	}
	int assigned = 3;
	for (size_t i = 3; i < px.size(); i++) {
		int best = -1;
		double bd = 1e18;
		for (int id : candidates) {
			if (used[id]) {
				continue;
			}
			Vec2 q;
			if (!c.world2cam(T_CM * model.position[id], q)) {
				continue;
			}
			double d = (q - px[i]).norm();
			if (d < bd) {
				bd = d;
				best = id;
			}
		}
		if (best >= 0 && bd < gate) {
			ids_out.push_back(best);
			used[best] = true;
			assigned++;
		} else {
			ids_out.push_back(-1);
		}
	}
	if (assigned < 5) {
		TRACE("  promote cam %d: %d blobs assigned\n", cam, assigned);
		return false;
	}
	std::vector<Vec2> uv;
	std::vector<Vec3> P;
	for (size_t i = 0; i < px.size(); i++) {
		if (ids_out[i] >= 0) {
			uv.push_back(normalised(c, px[i]));
			P.push_back(model.position[ids_out[i]]);
		}
	}
	Pose T_CM_e;
	if (!epnp(uv, P, T_CM_e)) {
		return false;
	}
	final_pose = T_WC * T_CM_e;
	double sum = 0;
	for (size_t i = 0; i < px.size(); i++) {
		if (ids_out[i] < 0) {
			sum += -1.0; // stock quirk: unassigned blobs enter the mean as -1
			continue;
		}
		Vec2 q;
		c.world2cam(T_CM_e * model.position[ids_out[i]], q);
		sum += (q - px[i]).norm();
	}
	mean_err = sum / px.size();
	angle_out = tilt_half_angle_deg(final_pose.q, ctx.q_ref);
	double angle_limit = ctx.wide_angles ? 30.0 : ctx.body_window_empty ? 20.0 : 15.0;
	TRACE("  promote cam %d: EPnP %.2f px (< %.1f), tilt %.1f (< %.0f)\n", cam, mean_err,
	      error_limit_by_size(blob_size, 0.8), angle_out, angle_limit);
	return mean_err < error_limit_by_size(blob_size, 0.8) && angle_out < angle_limit;
}

/*!
 * One P5P_New attempt with a given model and prior; blobs already trimmed to 5. The candidate
 * LEDs come from @p vis_model: generateVisiblePoints runs on this hand's predictor also for the
 * other hand's attempt (0x3792a4).
 */
bool
p5p_new_attempt(const InitContext &ctx, const LedModel &vis_model, const LedModel &model, int cam,
                const std::vector<InitBlob> &blobs, const InitBlob *trimmed, const Quat &Q, Pose &pose_out,
                std::vector<int> &ids_out, double &err_out, double &angle_out)
{
	double mean_size;
	Vec2 centre;
	blob_stats(blobs, mean_size, centre);
	std::vector<int> visible = facing_leds(ctx, vis_model, cam, Q, centre);
	Vec2 pts3[3] = {blobs[1].px, blobs[2].px, blobs[3].px};
	Vec3 t;
	double E, angle;
	int ids3[3];
	if (!solve_p3p_with_prior(ctx, model, Q, cam, pts3, visible, 5, t, E, ids3, angle)) {
		return false;
	}
	Pose pose(Q, t);
	std::vector<int> ids_blob(blobs.size(), -1);
	ids_blob[1] = ids3[0];
	ids_blob[2] = ids3[1];
	ids_blob[3] = ids3[2];
	if (!check_project_area(ctx, model, pose, mean_size, blobs, ids_blob, cam, E, angle, 5)) {
		return false;
	}
	std::vector<Vec2> pts5 = {blobs[1].px, blobs[2].px, blobs[3].px, blobs[0].px, blobs[4].px};
	double err;
	std::vector<int> ids5;
	Pose final_pose;
	bool ok = promote(ctx, model, cam, visible, pts5, ids3, pose, ids5, final_pose, err, mean_size, angle);
	if (!ok && trimmed != nullptr) {
		pts5[4] = trimmed->px;
		ok = promote(ctx, model, cam, visible, pts5, ids3, pose, ids5, final_pose, err, mean_size, angle);
	}
	if (!ok) {
		return false;
	}
	// Back to the cluster's blob order: 0, 1, 2, 3, 4.
	ids_out = {ids5[3], ids5[0], ids5[1], ids5[2], ids5[4]};
	pose_out = final_pose;
	err_out = err;
	angle_out = angle;
	return true;
}

} // namespace


/*
 *
 * The initialisers.
 *
 */

InitResult
init_p5p_new(const InitContext &ctx, int cam, std::vector<InitBlob> blobs, const Quat &Q)
{
	InitResult res;
	res.type = 2;
	TRACE("p5p new cam %d: %zu blobs\n", cam, blobs.size());
	std::vector<InitBlob> removed;
	while (blobs.size() > 5) {
		if (blobs.front().area < blobs.back().area) {
			removed.push_back(blobs.front());
			blobs.erase(blobs.begin());
		} else {
			removed.push_back(blobs.back());
			blobs.pop_back();
		}
	}
	if (blobs.size() < 5) {
		return res;
	}
	const InitBlob *trimmed = removed.size() == 1 ? &removed[0] : nullptr;
	Pose pose;
	std::vector<int> ids;
	double err, angle;
	bool ok = p5p_new_attempt(ctx, *ctx.model, *ctx.model, cam, blobs, trimmed, Q, pose, ids, err, angle);
	if (ok && !raise_both_hands_sideways(ctx, pose)) {
		ok = false;
	}
	if (ok && ctx.other.active && !ctx.other.locked && ctx.other.model != nullptr) {
		// The same blobs explained by the other controller, with its own state: keep ours only if
		// it fits better.
		Pose po;
		std::vector<int> io;
		double eo, ao;
		InitContext octx = other_hand_context(ctx, ctx.other.prior);
		if (p5p_new_attempt(octx, *ctx.model, *ctx.other.model, cam, blobs, trimmed, ctx.other.prior, po, io, eo,
		                    ao) &&
		    !(err < eo)) {
			ok = false;
		}
	}
	if (ok) {
		res.ok = true;
		res.T_WM = pose;
		res.ids = ids;
		res.error_px = err;
		res.angle_deg = angle;
	}
	return res;
}

InitResult
init_p5p_table(const InitContext &ctx, const IdTable &table, int cam, const std::vector<InitBlob> &cluster,
               const std::vector<InitBlob> &original, const Quat &Q)
{
	InitResult res;
	res.type = 0;
	std::vector<InitBlob> blobs = cluster;
	while (blobs.size() >= 6) {
		if (blobs.front().area >= blobs.back().area) {
			blobs.pop_back();
		} else {
			blobs.erase(blobs.begin());
		}
	}
	if (blobs.size() < 5) {
		return res;
	}
	const Camera &c = (*ctx.cams)[cam];
	const LedModel &model = *ctx.model;
	double mean_size;
	Vec2 centre;
	blob_stats(blobs, mean_size, centre);
	std::vector<Vec3> bearings;
	for (const InitBlob &b : blobs) {
		bearings.push_back(c.cam2world(b.px));
	}
	auto hyps = table.lookup(bearings, ctx.hand == kRight);

	// Score the voted hypotheses (bruteForceMatchPoints_new).
	Pose T_WC = camera_pose(ctx, cam);
	Quat q_ref = (!ctx.body_window_empty && ctx.lost_frames < 331) ? Q : ctx.q_3dof;
	double best_err = 1e18;
	Pose best_pose;
	std::vector<std::pair<int, int>> best_h;
	for (const auto &h : hyps) {
		std::vector<Vec2> uv;
		std::vector<Vec3> P;
		for (auto [blob, led] : h) {
			uv.push_back(normalised(c, blobs[blob].px));
			P.push_back(model.position[led]);
		}
		Pose T_CM;
		if (!epnp(uv, P, T_CM)) {
			continue;
		}
		Pose pose = T_WC * T_CM;
		double sum = 0;
		for (auto [blob, led] : h) {
			Vec2 q;
			c.world2cam(T_CM * model.position[led], q);
			sum += (q - blobs[blob].px).norm();
		}
		double e = sum / h.size();
		if (e < best_err) {
			best_err = e;
			best_pose = pose;
			best_h = h;
		}
		if (e < 0.5 && tilt_half_angle_deg(pose.q, q_ref) < 10) {
			break; // early accept
		}
	}
	if (best_h.empty()) {
		TRACE("p5p table cam %d: %zu hypotheses, none solved\n", cam, hyps.size());
		return res;
	}
	double angle = tilt_half_angle_deg(best_pose.q, q_ref);
	double angle_limit = ctx.wide_angles ? 30.0 : ctx.tracked ? 10.0 : ctx.body_window_empty ? 20.0 : 7.0;
	double err_limit = error_limit_by_size(mean_size, 0.5);
	if (best_err < 0.08 && angle > angle_limit) {
		if (ctx.tracked && angle > 20) {
			return res;
		}
		if (angle > 20) {
			res.reset_tracking = true; // an exact optical fit with a wrong IMU tilt resets the IMU state
		}
	}
	TRACE("p5p table cam %d: %zu hypotheses, best %.3f px (< %.1f) tilt %.1f (< %.0f)\n", cam, hyps.size(), best_err,
	      err_limit, angle, angle_limit);
	if (!(best_err < 0.08 || (best_err < err_limit && angle < angle_limit))) {
		return res;
	}

	if (!in_range_box(ctx, best_pose)) {
		Vec3 h = (ctx.T_WH.inverse() * best_pose).t;
		TRACE("p5p table cam %d: outside the range box at %.2f %.2f %.2f\n", cam, h.x(), h.y(), h.z());
		return res;
	}
	std::vector<int> ids(blobs.size(), -1);
	for (auto [blob, led] : best_h) {
		ids[blob] = led;
	}
	if (!check_project_area(ctx, model, best_pose, mean_size, blobs, ids, cam, best_err, angle, 5)) {
		return res;
	}
	if (!raise_both_hands_sideways(ctx, best_pose)) {
		TRACE("p5p table cam %d: next to the other hand\n", cam);
		return res;
	}
	// The second blob set must agree with the pose.
	if (original.size() == 5) {
		if (ctx.other.active && !ctx.other.locked && angle > 5) {
			return res;
		}
	} else if (original.size() >= 6) {
		Pose T_CM = T_WC.inverse() * best_pose;
		std::vector<Vec2> proj;
		for (int i = 0; i < kLeds; i++) {
			Vec2 q;
			c.world2cam(T_CM * model.position[i], q);
			proj.push_back(q);
		}
		size_t count = 0;
		for (const InitBlob &b : original) {
			for (const Vec2 &q : proj) {
				if ((q - b.px).norm() < 1.0) {
					count++;
					break;
				}
			}
		}
		if ((original.size() == 6 && count != 6) || (original.size() >= 7 && count < 7)) {
			TRACE("p5p table cam %d: %zu of %zu blobs on predicted LEDs\n", cam, count, original.size());
			return res;
		}
	}
	if (ctx.other.active && !ctx.other.locked) {
		bool accept = best_err < 0.08 || (best_err < 0.2 && angle < 5);
		if (!accept) {
			double d = half_angle_deg(best_pose.q, ctx.other.prior);
			accept = d > 20 || angle < d;
		}
		if (!accept) {
			return res;
		}
	}
	res.ok = true;
	res.T_WM = best_pose;
	res.ids = ids;
	res.error_px = best_err;
	res.angle_deg = angle;
	return res;
}

InitResult
init_p3p(const InitContext &ctx, int cam, std::vector<InitBlob> blobs, Quat Q)
{
	InitResult res;
	res.type = 1;
	TRACE("p3p cam %d: %zu blobs%s\n", cam, blobs.size(), ctx.no_p3p ? ", not while completely still" : "");
	if (blobs.size() < 3) {
		return res;
	}
	double mean_size;
	Vec2 centre;
	blob_stats(blobs, mean_size, centre);
	/*
	 * Whether the small-blob gate (and the position sanity below) applies: with the other hand
	 * locked and not followed (its P+0x6c1 is always clear on phoenix), only while the other hand
	 * has no Body this frame but Bodies in its window (0x38410c..0x384128, 0x384c58..0x384c74;
	 * 0x384d28..0x384d48, 0x385974..0x385994); otherwise always. Then only for small blobs.
	 */
	const bool small_gate = mean_size < 10 && (!(ctx.other.locked && !ctx.follow) ||
	                                           (!ctx.other.active && !ctx.other.body_window_empty));
	if (ctx.body_window_empty) {
		Q = ctx.q_3dof;
		if (ctx.other.speed > 3.5 && !ctx.tracked) {
			return res;
		}
		// 0x384134..0x384144: the neo3 exception is not phoenix's.
		if (small_gate && !ctx.juxtaposed && ctx.init_failures >= 1501) {
			return res;
		}
	}
	if (ctx.no_p3p) {
		return res;
	}
	const LedModel &model = *ctx.model;
	std::vector<int> visible = facing_leds(ctx, model, cam, Q, centre);
	// The library's unstable std::sort with cmpByCircularity, also for three blobs (0x384560,
	// 0x2da92c; BlobPoint instantiation 0x3d582c, insertion up to 30), then the first three.
	stock_sort(blobs.begin(), blobs.end(), [](const InitBlob &a, const InitBlob &b) { return a.circularity > b.circularity; },
	           30);
	if (blobs.size() >= 4) {
		blobs.resize(3);
	}
	Vec2 px[3] = {blobs[0].px, blobs[1].px, blobs[2].px};
	Vec3 t;
	double E, angle;
	int ids3[3];
	bool ok = solve_p3p_with_prior(ctx, model, Q, cam, px, visible, 3, t, E, ids3, angle);
	Pose pose(Q, t);
	std::vector<int> ids = {ids3[0], ids3[1], ids3[2]};
	ok = ok && check_project_area(ctx, model, pose, mean_size, blobs, ids, cam, E, angle, 3);
	if (ok && !ctx.tracked && ctx.body_window_empty && small_gate && !ctx.juxtaposed && ctx.lost_frames <= 150) {
		// 0x384d6c..0x385a0c: within 0.5 m (in float) of the last good output position or of
		// pose0, whether or not they were ever set, or the triple 51.
		bool near_known = (float)(pose.t - ctx.last_good_p).norm() < 0.5f || (float)(pose.t - ctx.pose0.t).norm() < 0.5f;
		ok = near_known || ids3[0] + ids3[1] + ids3[2] == 51;
	}
	ok = ok && raise_both_hands_sideways(ctx, pose);
	if (ok && !(E < 0.7 && angle < 8) && ctx.other.active && !ctx.other.locked && ctx.other.model != nullptr) {
		// The same three blobs fitted with the other controller's model and prior, solved on the
		// other predictor (0x3853f8: its tracked flag, pose0, reference rotation and juxtaposed
		// flag); the candidates from this hand's LEDs (generateVisiblePoints on this, 0x38531c).
		std::vector<int> vis_o = facing_leds(ctx, *ctx.model, cam, ctx.other.prior, centre);
		Vec3 to;
		double Eo, ao;
		int io[3];
		InitContext octx = other_hand_context(ctx, ctx.other.prior);
		bool other_ok = solve_p3p_with_prior(octx, *ctx.other.model, ctx.other.prior, cam, px, vis_o, 3, to, Eo, io, ao);
		if (other_ok && !(Eo > E && ao > angle)) {
			TRACE("p3p cam %d: the other hand fits better, E %.2f / %.2f angle %.1f / %.1f\n", cam, E, Eo, angle, ao);
			ok = false;
		}
	}
	if (ok) {
		res.ok = true;
		res.T_WM = pose;
		res.ids = ids;
		res.error_px = E;
		res.angle_deg = angle;
	}
	return res;
}

} // namespace pico4::ctrack
