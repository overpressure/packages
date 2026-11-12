// Copyright 2026, overpressure contributors.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  The PICO 4 controller tracker per camera frame set (see pico4_ctrack_engine.hpp).
 * @ingroup drv_pico4
 */

#include "pico4_ctrack_engine.hpp"

#include <algorithm>
#include <cmath>

namespace pico4::ctrack {

namespace {

constexpr double kTrackFeatureGatePx = 20.0;
constexpr double kTrackFeatureGateNearPx = 60.0;

bool
in_image(const Vec2 &p)
{
	return p.x() >= 0 && p.x() <= kImageWidth - 1 && p.y() >= 0 && p.y() <= kImageHeight - 1;
}

//! Bounding box of the in-image points; false if there are none.
bool
bounding_box(const std::vector<Vec2> &pts, Vec2 &lo, Vec2 &hi)
{
	bool any = false;
	lo = Vec2::Constant(1e18);
	hi = Vec2::Constant(-1e18);
	for (const Vec2 &p : pts) {
		if (!in_image(p)) {
			continue;
		}
		lo = lo.cwiseMin(p);
		hi = hi.cwiseMax(p);
		any = true;
	}
	return any;
}

} // namespace

const std::array<int, kCameras> &
init_camera_order(Hand hand)
{
	static const std::array<int, kCameras> left = {3, 2, 0, 1};
	static const std::array<int, kCameras> right = {2, 3, 1, 0};
	return hand == kLeft ? left : right;
}

InitBlob
to_init_blob(const ImageView &image, const BlobPoint &b)
{
	InitBlob o;
	o.px = Vec2(b.x, b.y);
	o.size = b.size;
	o.area = b.area;
	o.brightness = neighbour_max_brightness(image, b.x, b.y);
	o.circularity = b.circularity;
	return o;
}

Tracker::Tracker(const std::array<Camera, kCameras> &cams, const LedModel &left, const LedModel &right,
                 const FusionParams &params)
    : cams_(cams), models_{left, right},
      hands_{HandState(kLeft, &models_[0], params), HandState(kRight, &models_[1], params)}
{
	// One table from the left model; the right hand mirrors its descriptors.
	table_.generate(left);
	hands_[kLeft].fusion.set_other_hand(&hands_[kRight].fusion);
	hands_[kRight].fusion.set_other_hand(&hands_[kLeft].fusion);
}

void
Tracker::add_imu(Hand hand, const ImuSample &sample, const Pose *T_WH_at_sample)
{
	hands_[hand].fusion.add_imu(sample, T_WH_at_sample);
}

void
Tracker::set_unit(Hand hand, const UnitCalibration &unit)
{
	HandState &h = hands_[hand];
	if (unit.imu_valid) {
		h.fusion.set_calibration(unit.imu);
	}
	if (unit.leds_valid) {
		// updateLedPos: the positions only (spec 03 6/1).
		models_[hand].position = unit.leds;
	}
	if (unit.bias_valid) {
		h.fusion.set_bias_table(unit.bias);
	}
}

OutputPose
Tracker::pose_at(Hand hand, int64_t t_ns) const
{
	return hands_[hand].fusion.pose_at(t_ns);
}

std::vector<Vec2>
Tracker::predict(const LedModel &model, const Pose &T_WM, int cam) const
{
	// updateLEDPos + world2cam: no visibility or z test; an invalid projection fails the image test.
	Pose T_CM = (T_WH_ * cams_[cam].T_HC).inverse() * T_WM;
	std::vector<Vec2> uv(kLeds);
	for (int i = 0; i < kLeds; i++) {
		if (!cams_[cam].world2cam(T_CM * model.position[i], uv[i])) {
			uv[i] = Vec2(-1000.0, -1000.0);
		}
	}
	return uv;
}


/*
 *
 * Frame set.
 *
 */

void
Tracker::process_frame(int64_t t_ns, const Pose &T_WH, const std::array<ImageView, kCameras> &images)
{
	t_ns_ = t_ns;
	T_WH_ = T_WH;
	images_ = images;
	bright_ = scene_is_bright(images[3]);
	for (auto &d : init_detections_) {
		d.reset();
	}

	for (HandState &h : hands_) {
		h.stats = HandFrameStats{};
		h.second = false;
		h.prior = h.fusion.begin_frame(t_ns, T_WH);
		h.stats.has_body = h.prior.has_body;
		h.stats.tracked_before = h.fusion.tracked();
		h.stats.up_3dof = h.fusion.q3dof().conjugate() * Vec3::UnitX();
		h.stats.accel_M = h.fusion.orientation().smoothed_accel();
		if (h.fusion.tracked()) {
			h.pose = h.stats.prior = h.prior.T_WM;
		}
	}

	HandState &L = hands_[kLeft], &R = hands_[kRight];
	bool tl = L.prior.has_body && L.fusion.tracked();
	bool tr = R.prior.has_body && R.fusion.tracked();
	if (tl && tr) {
		// The nearer hand first; the other one removes the blobs the first one took.
		double dl = (T_WH.inverse() * L.pose).t.norm(), dr = (T_WH.inverse() * R.pose).t.norm();
		HandState &first = dl <= dr ? L : R, &second = dl <= dr ? R : L;
		update_pose(first);
		second.second = true;
		update_pose(second);
	} else if (tl || tr) {
		HandState &t = tl ? L : R, &u = tl ? R : L;
		update_pose(t);
		if (u.prior.has_body) {
			init_controller_pose(u);
		}
	} else {
		if (R.prior.has_body) {
			init_controller_pose(R);
		}
		if (L.prior.has_body) {
			init_controller_pose(L);
		}
	}

	for (HandState &h : hands_) {
		h.stats.tracked_after = h.fusion.tracked();
	}
}


/*
 *
 * Acquisition.
 *
 */

InitContext
Tracker::context(HandState &h, bool tracked, const Quat &Q)
{
	HandState &o = other(h);
	InitContext ctx;
	ctx.hand = h.hand;
	ctx.model = h.model;
	ctx.cams = &cams_;
	ctx.T_WH = T_WH_;
	ctx.tracked = tracked;
	ctx.pose0_valid = h.pose_valid;
	ctx.pose0 = h.pose;
	ctx.body_window_empty = h.fusion.window_size() == 0;
	ctx.lost_frames = h.fusion.lost_frames();
	ctx.init_failures = h.init_failures;
	ctx.occlusion = h.occlusion;
	ctx.no_p3p = h.fusion.complete_still();
	ctx.q_3dof = h.fusion.q3dof();
	ctx.follow = h.fusion.follow();
	ctx.last_good_p = h.fusion.last_good_position();
	ctx.q_ref = (ctx.body_window_empty || ctx.lost_frames > 330) ? ctx.q_3dof : Q;
	ctx.other.active = o.prior.has_body;
	ctx.other.locked = o.fusion.tracked() && o.locked;
	ctx.other.pose0_valid = o.pose_valid;
	ctx.other.pose0 = o.pose;
	// Its P+0x950 rotation while tracked, else its 3DoF orientation (0x385080..0x385150).
	ctx.other.prior = o.fusion.tracked() ? o.pose.q : o.fusion.q3dof();
	ctx.other.model = o.model;
	ctx.other.hand = o.hand;
	ctx.other.speed = o.fusion.velocity().norm();
	ctx.other.tracked = o.fusion.tracked();
	ctx.other.body_window_empty = o.fusion.window_size() == 0;
	ctx.other.lost_frames = o.fusion.lost_frames();
	ctx.other.q_3dof = o.fusion.q3dof();
	ctx.other.juxtaposed = o.juxtaposed;
	ctx.other.occlusion = o.occlusion;
	ctx.other.detect_around = [this, &o](int cam, const Pose &T_WM, int &nv, int &nm) {
		detect_around(o, cam, T_WM, nv, nm);
	};
	ctx.detect_around = [this, &h](int cam, const Pose &T_WM, int &nv, int &nm) {
		detect_around(h, cam, T_WM, nv, nm);
	};
	return ctx;
}

void
Tracker::detect_around(HandState &h, int cam, const Pose &T_WM, int &n_visible, int &n_matched)
{
	n_visible = n_matched = 0;
	std::vector<Vec2> uv = predict(*h.model, T_WM, cam);
	if (std::count_if(uv.begin(), uv.end(), in_image) < 8) {
		return;
	}
	TrackingInput in;
	in.predicted = uv;
	in.near = h.near;
	in.t_hm = (T_WH_.inverse() * T_WM).t;
	in.near_margin = 40.0;
	TrackingBlobs tb = tracking_blobs(images_[cam], in);
	if (!tb.detected) {
		return;
	}
	// The facing LEDs (at most 14) from the view direction of the window's blobs.
	Vec2 centre = Vec2::Zero();
	for (const BlobPoint &b : tb.window_blobs) {
		centre += Vec2(b.x, b.y);
	}
	Quat q_MC = T_WM.q.conjugate() * T_WH_.q * cams_[cam].T_HC.q;
	Vec3 bearing = tb.window_blobs.empty() ? ((T_WH_ * cams_[cam].T_HC).inverse() * T_WM.t).normalized()
	                                       : cams_[cam].cam2world(centre / tb.window_blobs.size());
	std::vector<Vec2> vis;
	for (auto [i, c] : visible_leds(*h.model, q_MC, bearing)) {
		(void)c;
		if ((int)vis.size() == 14) {
			break;
		}
		if (in_image(uv[i])) {
			vis.push_back(uv[i]);
		}
	}
	n_visible = (int)vis.size();
	for (const BlobPoint &b : tb.blobs) {
		double best = 999.0;
		for (const Vec2 &p : vis) {
			best = std::min(best, (p - Vec2(b.x, b.y)).norm());
		}
		if (best < 2.0) {
			n_matched++;
		}
	}
}

InitResult
Tracker::acquire(HandState &h, bool tracked, const Quat &Q, int &camera)
{
	HandState &o = other(h);
	InitContext ctx = context(h, tracked, Q);
	for (int cam : init_camera_order(h.hand)) {
		if (!init_detections_[cam]) {
			// Detected once per frame set and shared by both hands.
			init_detections_[cam] = detect_init_blobs(images_[cam], bright_);
		}
		InitInput in;
		in.bright = bright_;
		in.near = h.near;
		in.hand = h.hand;
		in.exclude_other = o.fusion.tracked() && o.locked;
		in.other_fast = o.fusion.velocity().norm() > 5.0;
		if (in.exclude_other) {
			in.other_projected = predict(*o.model, o.pose, cam);
		}
		InitBlobs prepared = prepare_init_clusters(images_[cam], *init_detections_[cam], in);
		for (const InitCluster &cl : prepared.clusters) {
			if (cl.blobs.size() < 3) {
				continue;
			}
			std::vector<InitBlob> blobs, original;
			for (const BlobPoint &b : cl.blobs) {
				blobs.push_back(to_init_blob(images_[cam], b));
			}
			for (const BlobPoint &b : cl.original) {
				original.push_back(to_init_blob(images_[cam], b));
			}
			// P+0x1b23, written per cluster (0x389288, 0x3896bc) and read by the other hand's fits.
			ctx.juxtaposed = h.juxtaposed = cl.juxtaposed;
			InitResult r;
			switch (cl.solver) {
			case InitSolver::kJuxtaposedP3P:
				r = init_p3p(ctx, cam, std::vector<InitBlob>(blobs.begin(), blobs.begin() + 3), Q);
				if (!r.ok) {
					r = init_p3p(ctx, cam, std::vector<InitBlob>(blobs.end() - 3, blobs.end()), Q);
				}
				break;
			case InitSolver::kP5P:
				r = init_p5p_new(ctx, cam, blobs, Q);
				if (!r.ok) {
					r = init_p5p_table(ctx, table_, cam, blobs, original, Q);
				}
				break;
			case InitSolver::kP3P: r = init_p3p(ctx, cam, blobs, Q); break;
			}
			if (r.ok) {
				camera = cam;
				return r;
			}
		}
	}
	camera = -1;
	return InitResult{};
}

void
Tracker::init_controller_pose(HandState &h)
{
	h.stats.init = true;
	int cam;
	InitResult r = acquire(h, false, h.prior.q_init, cam);
	if (!r.ok) {
		h.fusion.report_init_failure();
		h.init_failures = std::min(h.init_failures + 1, 10000);
		return;
	}
	if (r.type == 0) {
		h.init_failures = 0;
	}
	if (r.reset_tracking) {
		// An exact optical fit with a wrong IMU tilt: the IMU state starts over.
		h.fusion.reset_tracking();
	}
	h.pose = r.T_WM;
	h.pose_valid = true;
	h.stats.method = r.type;
	h.stats.init_camera = cam;
	h.stats.T_WM = r.T_WM;
	InitContext ctx = context(h, false, r.T_WM.q);
	h.near = judge_near(ctx, r.T_WM);
	if (h.fusion.report_init(r.T_WM)) {
		h.locked = true;
		h.init_failures = 0;
	}
}


/*
 *
 * Tracking.
 *
 */

std::vector<Feature>
Tracker::match_features(HandState &h)
{
	HandState &o = other(h);
	std::vector<Feature> feats;
	for (int cam = 0; cam < kCameras; cam++) {
		std::vector<Vec2> uv = predict(*h.model, h.pose, cam);
		TrackingInput in;
		in.predicted = uv;
		in.near = h.near;
		in.t_hm = (T_WH_.inverse() * h.pose).t;
		in.remove_other = h.second && o.fusion.tracked() && o.locked;
		in.other_points = h.other_px[cam];
		TrackingBlobs tb = tracking_blobs(images_[cam], in);
		if (!tb.detected || tb.blobs.empty() || tb.window_blobs.empty()) {
			continue;
		}
		Vec2 centre = Vec2::Zero();
		for (const BlobPoint &b : tb.window_blobs) {
			centre += Vec2(b.x, b.y);
		}
		centre /= tb.window_blobs.size();
		Quat q_MC = h.pose.q.conjugate() * T_WH_.q * cams_[cam].T_HC.q;
		std::vector<int> ids;
		std::vector<Vec2> proj;
		for (auto [i, c] : visible_leds(*h.model, q_MC, cams_[cam].cam2world(centre))) {
			(void)c;
			ids.push_back(i);
			proj.push_back(uv[i]);
		}
		std::vector<Vec2> blobs;
		for (const BlobPoint &b : tb.blobs) {
			blobs.emplace_back(b.x, b.y);
		}
		std::vector<int> match;
		std::vector<double> err;
		match_points(proj, blobs, match, err);
		double gate = h.near ? kTrackFeatureGateNearPx : kTrackFeatureGatePx;
		for (size_t b = 0; b < blobs.size(); b++) {
			if (match[b] < 0 || err[b] > gate) {
				continue;
			}
			Feature f;
			f.cam = cam;
			f.pixel = blobs[b];
			f.bearing = cams_[cam].cam2world(blobs[b]);
			f.led = ids[match[b]];
			f.P = h.model->position[f.led];
			f.residual_px = err[b];
			feats.push_back(f);
		}
	}
	return feats;
}

bool
Tracker::judge_tracking_ok(HandState &h, std::vector<Feature> &feats, Pose &T_WM, std::vector<double> &residuals)
{
	const Pose prior = h.pose;
	int n = (int)feats.size();
	bool few = n >= 2 && n <= 4 && h.occlusion < 0.4;
	residuals.clear();
	if (few) {
		Vec3 t;
		if (!few_points_translation(cams_, T_WH_, prior.q, feats, h.near, t, residuals)) {
			return false;
		}
		T_WM = Pose(prior.q, t);
	} else {
		if (n < 2) {
			return false;
		}
		Pose prior_HM = T_WH_.inverse() * prior, T_HM = prior_HM;
		double e0, e1;
		if (!refine_pose(cams_, feats, prior_HM, T_HM, e0, e1)) {
			return false;
		}
		int good = 0;
		for (const Feature &f : feats) {
			good += f.residual_px < 1.5;
		}
		if (good >= 10) {
			for (Feature &f : feats) {
				f.enabled = f.residual_px < 1.5;
			}
			T_HM = prior_HM;
			if (!refine_pose(cams_, feats, prior_HM, T_HM, e0, e1)) {
				return false;
			}
		}
		T_WM = T_WH_ * T_HM;
		for (const Feature &f : feats) {
			if (f.enabled) {
				residuals.push_back(f.residual_px);
			}
		}
	}
	h.stats.few_points = few;

	double L = 1.5;
	// N would be 4 only if byte +0x18 of the Body's preintegration (Body+0x230, an
	// IntegrationBase) were clear (0x3ab728..0x3ab748). IMUPreIntSinceLastF (0x26feac..0x26fecc)
	// sets it to the AND of the window's IMU samples' valid byte (+0x58), which
	// updateIMULeft/Right always set to 1 (0x29212c) and nothing on the controller path clears;
	// without samples the constructor's 1 stays (0x2705d4). So N is 3 on phoenix.
	int N = 3;
	if (!h.locked && h.fusion.lost_frames() == 0) {
		L = 1.0;
	}
	if (h.near) {
		L = 7.0;
		N = few ? 3 : 2;
	}
	int low = (int)std::count_if(residuals.begin(), residuals.end(), [L](double e) { return e < L; });
	h.stats.inliers = low;
	bool ok = low >= N;
	if (ok && n <= 5 && h.occlusion > 0.8) {
		// checkOptimizeResult with the tight limits of a heavily occluded controller.
		ok = full_angle_deg(prior.q, T_WM.q) < 1.0 && (prior.t - T_WM.t).norm() < 0.003;
	}
	if (ok && !few) {
		// Inlier pixels for the other hand's blob removal.
		for (auto &v : h.accepted_px) {
			v.clear();
		}
		HandState &o = other(h);
		for (const Feature &f : feats) {
			if (f.enabled || o.occlusion > 0.4) {
				h.accepted_px[f.cam].push_back(f.pixel);
			}
		}
		o.other_px = h.accepted_px;
	}
	return ok;
}

double
Tracker::occlusion_rate(const HandState &h)
{
	// Sum over the cameras of the IoU of both hands' LED bounding boxes. Like stock, the other
	// hand is projected with this hand's LED model.
	const HandState &o = hands_[1 - h.hand];
	double sum = 0;
	for (int cam = 0; cam < kCameras; cam++) {
		Vec2 alo, ahi, blo, bhi;
		if (!bounding_box(predict(*h.model, h.pose, cam), alo, ahi) ||
		    !bounding_box(predict(*h.model, o.pose, cam), blo, bhi)) {
			continue;
		}
		Vec2 ilo = alo.cwiseMax(blo), ihi = ahi.cwiseMin(bhi);
		if (ilo.x() >= ihi.x() || ilo.y() >= ihi.y()) {
			continue;
		}
		double inter = (ihi - ilo).prod();
		double uni = (ahi - alo).prod() + (bhi - blo).prod() - inter;
		sum += uni > 0 ? inter / uni : 0.0;
	}
	return sum;
}

void
Tracker::update_pose(HandState &h)
{
	h.stats.method = -1;
	const Pose saved = h.pose;
	InitContext ctx = context(h, true, h.pose.q);
	h.near = judge_near(ctx, h.pose);

	std::vector<Feature> feats = match_features(h);
	h.stats.features = (int)feats.size();
	std::vector<double> residuals;
	Pose T_WM;
	bool track_ok = judge_tracking_ok(h, feats, T_WM, residuals);
	h.stats.tracking_ok = track_ok;
	OpticalMethod method = OpticalMethod::kTracking;
	bool ok = track_ok;
	if (track_ok) {
		h.pose = T_WM;
		h.locked = true;
	} else {
		h.locked = false;
		// reInitControllerPose: the acquisition paths with the IMU prediction as rotation prior;
		// the first success counts at once, without confirmation.
		int cam;
		InitResult r = acquire(h, true, saved.q, cam);
		if (r.ok) {
			h.pose = r.T_WM;
			h.near = judge_near(context(h, true, r.T_WM.q), r.T_WM);
			h.locked = true;
			method = static_cast<OpticalMethod>(r.type);
			ok = true;
		} else {
			h.pose = saved;
		}
	}
	if (h.near && !track_ok && (h.pose.inverse() * h.fusion.filtered_pose()).t.norm() > 0.1) {
		ok = false;
	}
	if (h.near && feats.size() == 1) {
		ok = false;
	}

	double mean = 100.0;
	if (!residuals.empty()) {
		mean = 0;
		for (double e : residuals) {
			mean += e;
		}
		mean /= residuals.size();
	}
	h.stats.near = h.near;
	h.stats.residual_mean = mean;
	if (ok) {
		h.stats.method = static_cast<int>(method);
		h.stats.T_WM = h.pose;
		OpticalReport rep;
		rep.T_WM = h.pose;
		rep.method = method;
		rep.tracking_ok = track_ok;
		rep.residual_count = (int)residuals.size();
		rep.residual_mean = mean;
		rep.near = h.near;
		rep.matched_leds = (int)feats.size();
		rep.occlusion_rate = h.occlusion;
		OpticalResult res = h.fusion.report_optical(rep);
		h.stats.accepted = res.accepted;
	} else {
		NoOpticalReport rep;
		rep.near = h.near;
		rep.occlusion_rate = h.occlusion;
		h.fusion.report_no_optical(rep);
	}

	HandState &o = other(h);
	if (h.fusion.tracked() && h.fusion.lost_frames() == 0 && o.fusion.tracked()) {
		h.occlusion = occlusion_rate(h);
	} else {
		h.occlusion = 0.0;
	}
}

} // namespace pico4::ctrack
