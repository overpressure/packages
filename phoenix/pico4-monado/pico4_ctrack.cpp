// Copyright 2026, overpressure contributors.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  PICO 4 controller tracking for the driver (see pico4_ctrack.h).
 *
 * Frames: the tracker works in the headset body frame of the factory calibration (x up, y left,
 * z back) and a world with +x up; the driver in OpenXR's (world Y up, HMD x right, y up, z back).
 * The head pose of each LED exposure comes from the HMD device, so controllers and head share
 * one world. Controller poses leave the tracker in the LED-model frame M; the published frame,
 * which PICO OS treats as the aim, is M turned like the driver's 3DoF axis mapping of the raw
 * sensor axes ("x,-z,y"), and PICO OS adds its lever arm (lever-arm.json) in that frame. The
 * driver derives the grip from it (pico4_aimgrip.h).
 *
 * @ingroup drv_pico4
 */

#include "pico4_ctrack.h"

#include "pico4_blemcu.h"
#include "pico4_ctrack_calib.hpp"
#include "pico4_ctrack_runner.hpp"

#include "util/u_debug.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <ctime>
#include <memory>
#include <string>

DEBUG_GET_ONCE_BOOL_OPTION(pico4_ctrack, "PICO4_CTRACK", true)
//! File to write one CSV row per hand after every publication (diagnostics; pico4_ctrack.h).
DEBUG_GET_ONCE_OPTION(pico4_ctrack_trace, "PICO4_CTRACK_TRACE", NULL)

using namespace pico4::ctrack;

namespace {

//! The HMD axes in the body frame and the HMD origin there (pico4_slam.h, PICO4_HMD_IN_BODY).
const Quat kQ_BH(Eigen::AngleAxisd(-M_PI / 2, Vec3::UnitZ()));
const Vec3 kP_BH(-0.025, 0.052, 0.04);
//! The tracker's world (+x up) from OpenXR's (+y up).
const Quat kQ_W_XR(Eigen::AngleAxisd(-M_PI / 2, Vec3::UnitZ()));
//! The aim frame in M: aim = (-m.y, m.x, m.z), the raw-axis mapping "x,-z,y" after the stock remap.
const Quat kQ_MA(Eigen::AngleAxisd(-M_PI / 2, Vec3::UnitZ()));

Pose
from_xrt(const xrt_pose &p)
{
	return Pose(Quat(p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z),
	            Vec3(p.position.x, p.position.y, p.position.z));
}

xrt_vec3
to_xrt(const Vec3 &v)
{
	return xrt_vec3{(float)v.x(), (float)v.y(), (float)v.z()};
}

} // namespace

struct pico4_ctrack
{
	enum u_logging_level log_level;
	pico4_ctrack_head_fn head;
	void *user;
	std::array<Vec3, 2> lever; //!< PICO OS's lever arm per hand, in the aim frame
	std::unique_ptr<Runner> runner;
	uint64_t pushed; //!< frame sets, for the status line every 10 s
	FILE *trace;      //!< PICO4_CTRACK_TRACE, written on the tracker's thread only
	std::string dir;  //!< the calibration directory (bias tables in pvr/)
};

namespace {

//! One line on both hands: tracked or not, how the last frame set was solved, where (head frame).
void
log_status(pico4_ctrack *t)
{
	Runner::Stats s = t->runner->stats();
	char hands[2][256];
	for (int h = 0; h < 2; h++) {
		const HandFrameStats &st = s.last[h];
		xrt_pose head;
		Vec3 p = Vec3::Constant(NAN);
		timespec now;
		clock_gettime(CLOCK_MONOTONIC, &now);
		int64_t now_ns = (int64_t)now.tv_sec * 1000000000 + now.tv_nsec;
		if (st.method >= 0 && t->head != nullptr && t->head(t->user, now_ns, &head)) {
			// Position relative to the current head pose, in the tracker's world axes.
			Pose T_WB = Pose(kQ_W_XR, Vec3::Zero()) * from_xrt(head) * Pose(kQ_BH, kP_BH).inverse();
			p = T_WB.inverse() * st.T_WM.t;
		}
		double tilt = NAN;
		if (st.up_3dof.norm() > 0 && st.accel_M.norm() > 0) {
			tilt = std::acos(std::clamp(st.up_3dof.normalized().dot(st.accel_M.normalized()), -1.0, 1.0)) * 180 / M_PI;
		}
		snprintf(hands[h], sizeof(hands[h]),
		         "%s %s (IMU %s), method %d, %d LEDs (%d inliers, %.2f px), at %.2f %.2f %.2f m, 3DoF up %.1f deg "
		         "from the accelerometer",
		         h == 0 ? "left" : "right", st.tracked_after ? "tracked" : "untracked",
		         st.has_body ? "on" : "off", st.method, st.features, st.inliers, st.residual_mean, p.x(), p.y(), p.z(),
		         tilt);
	}
	// Up (the world's +x) in the body frame as the head pose has it: against the head accelerometer.
	Vec3 up = Vec3::Constant(NAN);
	xrt_pose head;
	timespec now;
	clock_gettime(CLOCK_MONOTONIC, &now);
	if (t->head != nullptr && t->head(t->user, (int64_t)now.tv_sec * 1000000000 + now.tv_nsec, &head)) {
		Pose T_WB = Pose(kQ_W_XR, Vec3::Zero()) * from_xrt(head) * Pose(kQ_BH, kP_BH).inverse();
		up = T_WB.q.conjugate() * Vec3::UnitX();
	}
	U_LOG_IFL_I(t->log_level, "Controller tracking: %llu sets, %llu solved, %llu IMU timeouts; up in the body %.3f %.3f %.3f; %s; %s",
	            (unsigned long long)s.frame_sets, (unsigned long long)s.solved,
	            (unsigned long long)s.imu_timeouts, up.x(), up.y(), up.z(), hands[0], hands[1]);
}

//! One row per hand: what the tracker saw in the last frame set and what it publishes now.
void
write_trace(FILE *f, int64_t frame_t_ns, const Pose &T_WH, const HandFrameStats *st, const OutputPose *o)
{
	timespec now;
	clock_gettime(CLOCK_MONOTONIC, &now);
	long long now_ns = (long long)now.tv_sec * 1000000000 + now.tv_nsec;
	for (int h = 0; h < 2; h++) {
		const HandFrameStats &s = st[h];
		const OutputPose &p = o[h];
		const Quat &hq = T_WH.q, &q = p.T_WM.q;
		fprintf(f,
		        "%lld,%lld,%d,%.5f,%.5f,%.5f,%.6f,%.6f,%.6f,%.6f,%d,%d,%d,%d,%d,%d,%d,%.3f,%d,%d,%d,%d,%lld,"
		        "%.5f,%.5f,%.5f,%.6f,%.6f,%.6f,%.6f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n",
		        now_ns, (long long)frame_t_ns, h, T_WH.t.x(), T_WH.t.y(), T_WH.t.z(), hq.w(), hq.x(), hq.y(), hq.z(),
		        s.has_body, s.tracked_before, s.init, s.method, s.tracking_ok, s.features, s.inliers, s.residual_mean,
		        s.tracked_after, p.valid, p.is_3dof, p.has_position, (long long)p.t_ns, p.T_WM.t.x(), p.T_WM.t.y(),
		        p.T_WM.t.z(), q.w(), q.x(), q.y(), q.z(), p.v.x(), p.v.y(), p.v.z(), p.omega.x(), p.omega.y(),
		        p.omega.z());
	}
	if (frame_t_ns != 0) {
		fflush(f);
	}
}

} // namespace

extern "C" struct pico4_ctrack *
pico4_ctrack_create(const char *calibration_dir, pico4_ctrack_head_fn head, void *user, enum u_logging_level log_level)
{
	if (!debug_get_bool_option_pico4_ctrack()) {
		return nullptr;
	}
	std::array<Camera, kCameras> cams;
	LedModel left, right;
	std::array<Vec3, 2> lever;
	std::string dir = calibration_dir, error;
	if (!load_cameras(dir, cams, error) ||
	    !load_led_models(dir + "/controllers/led-models.json", left, right, error) ||
	    !load_lever_arms(dir + "/controllers/lever-arm.json", lever, error)) {
		U_LOG_IFL_E(log_level, "Controller tracking off: %s", error.c_str());
		return nullptr;
	}
	auto *t = new pico4_ctrack{log_level, head, user, lever, nullptr, 0, nullptr, dir};
	const char *trace = debug_get_option_pico4_ctrack_trace();
	if (trace != nullptr && (t->trace = fopen(trace, "w")) != nullptr) {
		fputs("now_ns,frame_ns,hand,head_x,head_y,head_z,head_qw,head_qx,head_qy,head_qz,has_body,tracked_before,"
		      "init,method,tracking_ok,features,inliers,residual_px,tracked_after,valid,is_3dof,has_position,"
		      "out_t_ns,x,y,z,qw,qx,qy,qz,vx,vy,vz,wx,wy,wz\n",
		      t->trace);
	}
	Runner::PublishFn on_publish;
	if (t->trace != nullptr) {
		on_publish = [t](int64_t frame_t_ns, const Pose &T_WH, const HandFrameStats *st, const OutputPose *o) {
			write_trace(t->trace, frame_t_ns, T_WH, st, o);
		};
	}
	t->runner = std::make_unique<Runner>(cams, left, right, [t](int64_t at_ns, Pose &T_WH) {
		xrt_pose hmd;
		if (t->head == nullptr || !t->head(t->user, at_ns, &hmd)) {
			return false;
		}
		// Body in OpenXR's world, then in the tracker's.
		Pose T_XR_B = from_xrt(hmd) * Pose(kQ_BH, kP_BH).inverse();
		T_WH = Pose(kQ_W_XR, Vec3::Zero()) * T_XR_B;
		return true;
	}, FusionParams{}, on_publish);
	U_LOG_IFL_I(log_level, "Controller tracking from the LED exposures (%s)", calibration_dir);
	return t;
}

extern "C" void
pico4_ctrack_push_packet(struct pico4_ctrack *t, const struct pico4_blemcu_packet *p, int64_t packet_ns)
{
	for (int i = 0; i < p->imu_count; i++) {
		const pico4_blemcu_imu &m = p->imu[i];
		if (m.device != PICO4_BLEMCU_LEFT && m.device != PICO4_BLEMCU_RIGHT) {
			continue;
		}
		ImuSample s;
		s.t_ns = packet_ns - (int64_t)m.offset_us * 1000;
		for (int a = 0; a < 3; a++) {
			s.gyro[a] = m.gyro[a] / PICO4_BLEMCU_GYRO_LSB_PER_DPS * M_PI / 180.0;
			s.accel[a] = m.accel[a] / PICO4_BLEMCU_ACCEL_LSB_PER_G * 9.80665;
		}
		s.temperature = m.temperature / 132.48 + 25.0;
		t->runner->push_imu(m.device == PICO4_BLEMCU_LEFT ? kLeft : kRight, s);
	}
}

extern "C" void
pico4_ctrack_push_frames(struct pico4_ctrack *t, int64_t timestamp_ns, const uint8_t *const images[4], uint32_t stride)
{
	std::array<ImageView, kCameras> views;
	for (int k = 0; k < kCameras; k++) {
		views[k] = ImageView{images[k], kImageWidth, kImageHeight, (int)stride};
	}
	t->runner->push_frames(timestamp_ns, views);
	if (++t->pushed % 250 == 0) {
		log_status(t);
	}
}

extern "C" bool
pico4_ctrack_get_aim(struct pico4_ctrack *t, int device, int64_t at_ns, struct xrt_space_relation *out)
{
	Hand hand = device == PICO4_BLEMCU_LEFT ? kLeft : kRight;
	OutputPose o = t->runner->pose_at(hand, at_ns);
	// PICO OS's client publishes the record's pose whatever its 3DoF flag (spec 03 7): a still or
	// briefly unseen controller keeps its held position. Only one never placed has none.
	if (!o.valid || !o.has_position) {
		return false;
	}
	Pose T_XR_M = Pose(kQ_W_XR.conjugate(), Vec3::Zero()) * o.T_WM;
	/*
	 * The record carries T_WM unchanged (tracker 0x2a1b0c, service 0xcc5a0). The client turns it
	 * into the Y-up world by conjugating with Rz(+90): p and q's vector part (x, y, z) ->
	 * (-y, x, z), w kept (client.so 0x13fc8, 0x143f4). That turns the body to A = M kQ_MA too, and
	 * the lever is then added as R(q_A) lever after prediction (0x1154c): in A, not in M.
	 */
	Pose T_XR_A = T_XR_M * Pose(kQ_MA, kQ_MA * t->lever[hand]);
	out->pose.orientation = xrt_quat{(float)T_XR_A.q.x(), (float)T_XR_A.q.y(), (float)T_XR_A.q.z(), (float)T_XR_A.q.w()};
	out->pose.position = to_xrt(T_XR_A.t);
	out->linear_velocity = to_xrt(kQ_W_XR.conjugate() * o.v);
	out->angular_velocity = to_xrt(T_XR_M.q * o.omega);
	out->relation_flags = (enum xrt_space_relation_flags)(
	    XRT_SPACE_RELATION_ORIENTATION_VALID_BIT | XRT_SPACE_RELATION_POSITION_VALID_BIT |
	    XRT_SPACE_RELATION_ORIENTATION_TRACKED_BIT | XRT_SPACE_RELATION_LINEAR_VELOCITY_VALID_BIT |
	    XRT_SPACE_RELATION_ANGULAR_VELOCITY_VALID_BIT | (o.is_3dof ? 0 : XRT_SPACE_RELATION_POSITION_TRACKED_BIT));
	return true;
}

extern "C" void
pico4_ctrack_set_unit(struct pico4_ctrack *t, int device, const uint8_t (*chunks)[32], const char *serial)
{
	if (t == nullptr || (device != PICO4_BLEMCU_LEFT && device != PICO4_BLEMCU_RIGHT)) {
		return;
	}
	const char *name = device == PICO4_BLEMCU_LEFT ? "left" : "right";
	UnitCalibration unit;
	std::string error, applied;
	if (chunks == nullptr) {
		U_LOG_IFL_W(t->log_level, "Controller %s: no calibration record from the station", name);
	} else if (!parse_station_calibration(chunks, unit, error)) {
		U_LOG_IFL_W(t->log_level, "Controller %s: %s", name, error.c_str());
	} else {
		applied = "IMU calibration and LED positions from the station";
	}

	// The serial names PICO OS's file; it never appears in the log.
	std::string sn = serial != nullptr ? serial : "";
	bool plain = !sn.empty() && sn.size() <= 32 &&
	             std::all_of(sn.begin(), sn.end(), [](char c) { return std::isalnum((unsigned char)c) != 0; });
	if (!plain) {
		U_LOG_IFL_W(t->log_level, "Controller %s: no serial, no gyro bias table", name);
	} else if (!load_bias_table(t->dir + "/pvr/" + sn + ".txt", unit.bias, error) &&
	           // On persist they are root's alone; a session's user reads the copy (pico4/persist).
	           !load_bias_table("/run/pico4-controllers/" + sn + ".txt", unit.bias, error)) {
		U_LOG_IFL_W(t->log_level, "Controller %s: no gyro bias table for its serial", name);
	} else {
		unit.bias_valid = true;
		int bins = 0;
		for (const Vec3 &b : unit.bias) {
			bins += !b.isZero() && b.norm() <= 0.15;
		}
		applied += std::string(applied.empty() ? "" : ", ") + "gyro bias table (" + std::to_string(bins) + " bins)";
	}
	if (!applied.empty()) {
		U_LOG_IFL_I(t->log_level, "Controller %s: %s", name, applied.c_str());
	}
	t->runner->set_unit(device == PICO4_BLEMCU_LEFT ? kLeft : kRight, unit);
}

extern "C" void
pico4_ctrack_stop(struct pico4_ctrack *t)
{
	if (t != nullptr) {
		t->runner->stop();
	}
}

extern "C" void
pico4_ctrack_destroy(struct pico4_ctrack **ptr)
{
	pico4_ctrack *t = *ptr;
	if (t == nullptr) {
		return;
	}
	Runner::Stats s = t->runner->stats();
	U_LOG_IFL_I(t->log_level,
	            "Controller tracking: %llu LED frame sets, %llu solved, %llu replaced while waiting, %llu without "
	            "a head pose, %llu solved without all IMU samples; IMU samples %llu left, %llu right",
	            (unsigned long long)s.frame_sets, (unsigned long long)s.solved, (unsigned long long)s.dropped,
	            (unsigned long long)s.no_head, (unsigned long long)s.imu_timeouts,
	            (unsigned long long)s.imu_samples[0], (unsigned long long)s.imu_samples[1]);
	t->runner.reset();
	if (t->trace != nullptr) {
		fclose(t->trace);
	}
	delete t;
	*ptr = nullptr;
}
