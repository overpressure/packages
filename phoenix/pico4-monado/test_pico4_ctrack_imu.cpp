// Copyright 2026, overpressure contributors.
// SPDX-License-Identifier: BSL-1.0
/*
 * Unit test of the controller tracker's IMU side with synthetic motion (no device data): the
 * output filter stages, calibration records, the 3DoF filter (tilt, yaw-only correction, gyro
 * bias), the error-state filter (propagation against an exact trajectory, optical update,
 * bias estimation, ZUPT), the fusion gate rules, and the per-hand fusion from initialisation
 * through tracking to loss and reset, and the 3DoF fallback through a long optical loss.
 */

#include "pico4_ctrack_calib.hpp"
#include "pico4_ctrack_imu.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

using namespace pico4::ctrack;

static int failures;

#define EXPECT(cond)                                                                                                   \
	do {                                                                                                           \
		if (!(cond)) {                                                                                         \
			fprintf(stderr, "%s:%d: expected %s\n", __FILE__, __LINE__, #cond);                            \
			failures++;                                                                                    \
		}                                                                                                      \
	} while (0)

static const double kPi = 3.14159265358979323846;
static const double kDeg = kPi / 180.0;

static uint32_t seed = 12345;

//! Roughly normal noise with standard deviation @p sigma.
static double
noise(double sigma)
{
	double sum = 0.0;
	for (int i = 0; i < 12; i++) {
		seed = seed * 1664525u + 1013904223u;
		sum += double(seed >> 8) / 16777216.0;
	}
	return (sum - 6.0) * sigma;
}

static Quat
rotvec(const Vec3 &r)
{
	const double a = r.norm();
	if (a < 1e-15) {
		return Quat::Identity();
	}
	return Quat(Eigen::AngleAxisd(a, r / a));
}

static double
angle_deg(const Quat &a, const Quat &b)
{
	return 2.0 * std::acos(std::min(1.0, std::abs(a.normalized().dot(b.normalized())))) / kDeg;
}

//! M-frame vector to raw sensor axes (inverse of sensor_to_model).
static Vec3
model_to_sensor(const Vec3 &m)
{
	return Vec3(-m.y(), m.z(), -m.x());
}

/*!
 * Exact trajectory: sinusoidal position around a centre, orientation q0 (x) exp(Omega t) with
 * a constant body rate.
 */
struct Trajectory
{
	Vec3 centre{-0.5, 0.0, -0.5};
	Vec3 amp{0.1, 0.15, 0.08};
	Vec3 freq{0.7, 0.5, 0.9}; // Hz
	Quat q0 = rotvec(Vec3(0.3, -0.2, 0.5));
	Vec3 omega{0.4, -0.3, 0.6};

	Vec3
	p(double t) const
	{
		Vec3 r;
		for (int i = 0; i < 3; i++) {
			r(i) = centre(i) + amp(i) * std::sin(2 * kPi * freq(i) * t);
		}
		return r;
	}
	Vec3
	v(double t) const
	{
		Vec3 r;
		for (int i = 0; i < 3; i++) {
			const double w = 2 * kPi * freq(i);
			r(i) = amp(i) * w * std::cos(w * t);
		}
		return r;
	}
	Vec3
	a(double t) const
	{
		Vec3 r;
		for (int i = 0; i < 3; i++) {
			const double w = 2 * kPi * freq(i);
			r(i) = -amp(i) * w * w * std::sin(w * t);
		}
		return r;
	}
	Quat
	q(double t) const
	{
		return (q0 * rotvec(omega * t)).normalized();
	}
	//! Accelerometer in M (specific force).
	Vec3
	accel(double t) const
	{
		return q(t).conjugate() * (a(t) + kGravityImu);
	}
	Pose
	pose(double t) const
	{
		return Pose(q(t), p(t));
	}
};

static Trajectory
still_trajectory()
{
	Trajectory tr;
	tr.amp.setZero();
	tr.omega.setZero();
	return tr;
}

/*
 * Output stages and calibration.
 */

static void
test_butterworth()
{
	Butterworth bw(10);
	double sb = 0.0, sa = 0.0;
	for (size_t i = 0; i < bw.b().size(); i++) {
		sb += bw.b()[i];
		sa += bw.a()[i];
	}
	EXPECT(std::abs(sb / sa - 1.0) < 1e-6); // unit gain at DC
	// A constant settles to itself; 0.25 fs is strongly attenuated, 0.02 fs passes.
	double y = 0.0;
	for (int i = 0; i < 400; i++) {
		y = bw.filter(0, 3.0);
	}
	EXPECT(std::abs(y - 3.0) < 1e-6);
	double peak_hi = 0.0, peak_lo = 0.0;
	for (int i = 0; i < 2000; i++) {
		const double hi = bw.filter(1, std::sin(2 * kPi * 0.25 * i));
		const double lo = bw.filter(2, std::sin(2 * kPi * 0.02 * i));
		if (i > 1000) {
			peak_hi = std::max(peak_hi, std::abs(hi));
			peak_lo = std::max(peak_lo, std::abs(lo));
		}
	}
	EXPECT(peak_hi < 1e-3);
	EXPECT(peak_lo > 0.95 && peak_lo < 1.05);
}

static void
test_calibration()
{
	ImuCalib c;
	Vec3 g, a;
	c.apply(Vec3(1, 2, 3), Vec3(4, 5, 6), g, a);
	EXPECT((g - Vec3(-3, -1, 2)).norm() < 1e-12); // (x, y, z) -> (-z, -x, y)
	EXPECT((a - Vec3(-6, -4, 5)).norm() < 1e-12);

	double gyro[12] = {1, 0, 0, 0, 1, 0, 0, 0, 1, 0.01, 0.02, 0.03};
	double acc[12] = {2, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0};
	EXPECT(c.load(gyro, acc));
	c.apply(Vec3(0.01, 0.02, 0.03), Vec3(1, 0, 0), g, a);
	EXPECT(g.norm() < 1e-12);
	EXPECT((a - Vec3(0, -2, 0)).norm() < 1e-12);
	acc[0] = 2.5; // above the 2.0 limit: rejected, old values kept
	EXPECT(!c.load(gyro, acc));
	EXPECT(c.M_accel(0, 0) == 2.0);
}

//! Float @p i of a station record, little-endian, as the controller sends it.
static void
put_float(uint8_t chunks[12][32], int i, float v)
{
	uint32_t u;
	std::memcpy(&u, &v, sizeof(u));
	uint8_t *b = &chunks[i / 8][(i % 8) * 4];
	for (int k = 0; k < 4; k++) {
		b[k] = uint8_t(u >> (8 * k));
	}
}

static void
test_station_record()
{
	// A made-up record in the station's layout: IMU block, 20 LEDs, imu_IR (identity).
	uint8_t chunks[12][32] = {};
	const float gyro[12] = {1.01f, 0, 0, 0, 0.99f, 0, 0, 0, 1, 0.01f, -0.02f, 0.005f};
	const float acc[12] = {1, 0, 0, 0, 1, 0, 0, 0, 1, 0.05f, 0, 0.1f};
	for (int i = 0; i < 12; i++) {
		put_float(chunks, i, gyro[i]);
		put_float(chunks, 12 + i, acc[i]);
	}
	for (int k = 0; k < 20; k++) {
		put_float(chunks, 24 + 3 * k, 0.001f * k);
		put_float(chunks, 25 + 3 * k, -0.002f * k);
		put_float(chunks, 26 + 3 * k, 0.03f);
	}
	put_float(chunks, 88 + 3, 1.0f);

	UnitCalibration u;
	std::string error;
	EXPECT(parse_station_calibration(chunks, u, error));
	EXPECT(u.imu_valid && u.leds_valid && !u.bias_valid);
	EXPECT(std::abs(u.imu.M_gyro(0, 0) - 1.01) < 1e-6 && std::abs(u.imu.b_gyro.y() + 0.02) < 1e-6);
	EXPECT(std::abs(u.imu.b_accel.z() - 0.1) < 1e-6);
	EXPECT((u.leds[19] - Vec3(0.019, -0.038, 0.03)).norm() < 1e-6);

	// Not prepared: imu_IR's fourth float is not 1, or the first IMU float is 0.
	uint8_t bad[12][32];
	std::memcpy(bad, chunks, sizeof(bad));
	put_float(bad, 91, 0.0f);
	UnitCalibration v;
	EXPECT(!parse_station_calibration(bad, v, error) && !v.imu_valid && !v.leds_valid);
	std::memcpy(bad, chunks, sizeof(bad));
	put_float(bad, 0, 0.0f);
	EXPECT(!parse_station_calibration(bad, v, error));
	// An IMU element above 2 (IMUCalib's check) or a LED coordinate above 100 (updateLedPos's).
	std::memcpy(bad, chunks, sizeof(bad));
	put_float(bad, 13, 2.5f);
	EXPECT(!parse_station_calibration(bad, v, error) && !v.imu_valid);
	std::memcpy(bad, chunks, sizeof(bad));
	put_float(bad, 30, 150.0f);
	EXPECT(!parse_station_calibration(bad, v, error) && !v.leds_valid);
}

static void
test_bias_table()
{
	std::string path = "test_pico4_bias.txt";
	FILE *f = fopen(path.c_str(), "w");
	EXPECT(f != nullptr);
	if (f == nullptr) {
		return;
	}
	for (int i = 0; i < 100; i++) {
		fprintf(f, "%f %f %f\n", i == 25 ? 0.01 : 0.0, i == 25 ? -0.02 : 0.0, i == 25 ? 0.003 : 0.0);
	}
	fclose(f);
	std::array<Vec3, 100> table;
	std::string error;
	EXPECT(load_bias_table(path, table, error));
	EXPECT((table[25] - Vec3(0.01, -0.02, 0.003)).norm() < 1e-9 && table[24].isZero());

	f = fopen(path.c_str(), "w");
	fputs("0 0 0\n0 0\n", f);
	fclose(f);
	EXPECT(!load_bias_table(path, table, error));
	remove(path.c_str());
	EXPECT(!load_bias_table(path, table, error));
}

static void
test_smoother()
{
	HighFreqSmoother hf;
	// Confidence 1: R = 20, strong smoothing of a position step; velocity passes.
	Vec3 p(0, 0, 0), v(0, 0, 0);
	Quat q = Quat::Identity();
	EXPECT(hf.update(p, v, q, 1.0, false));
	p = Vec3(0.1, 0, 0);
	v = Vec3(1, 0, 0);
	EXPECT(hf.update(p, v, q, 1.0, false));
	EXPECT(p.x() > 0.0 && p.x() < 0.01);
	EXPECT(std::abs(v.x() - 1.0) < 1e-9);
	// Confidence 0: R = 0, the measurement passes unchanged.
	HighFreqSmoother hf0;
	Vec3 p0(0, 0, 0);
	Quat q0 = Quat::Identity();
	hf0.update(p0, v, q0, 0.0, false);
	p0 = Vec3(0.3, 0.2, 0.1);
	Quat qr = rotvec(Vec3(0, 0, 0.2));
	q0 = qr;
	hf0.update(p0, v, q0, 0.0, false);
	EXPECT((p0 - Vec3(0.3, 0.2, 0.1)).norm() < 1e-9);
	EXPECT(angle_deg(q0, qr) < 1e-4);
}

/*
 * 3DoF filter.
 */

static void
test_orientation_tilt_and_rotation()
{
	// Lying tilted and still: the attitude maps the accelerometer onto W +x at once.
	OrientationController oc(80, 100);
	const Quat tilt = rotvec(Vec3(0.0, 0.4, 0.3));
	const Vec3 acc = tilt.conjugate() * kGravityImu;
	for (int i = 0; i < 300; i++) {
		oc.track(acc, Vec3::Zero(), i * 0.002, 30.0);
	}
	EXPECT(((oc.attitude() * acc).normalized() - Vec3::UnitX()).norm() < 1e-3);
	EXPECT(oc.confidence() > 0.9);

	// Rotation about the gravity axis at 1 rad/s for 2 s: the yaw follows the gyro.
	OrientationController oc2(80, 100);
	const Quat start = rotvec(Vec3(0.0, 0.2, 0.0));
	const Vec3 w_world(1.0, 0.0, 0.0);
	Quat q_prev;
	for (int i = 0; i <= 1000; i++) {
		const double t = i * 0.002;
		const Quat q = (rotvec(w_world * t) * start).normalized();
		const Vec3 gyro = q.conjugate() * w_world; // body rate
		oc2.track(q.conjugate() * kGravityImu, gyro, t, 30.0);
		if (i == 0) {
			q_prev = oc2.attitude();
		}
	}
	// Relative rotation from the first sample: 2 rad about W +x.
	const Quat rel = oc2.attitude() * q_prev.conjugate();
	EXPECT(angle_deg(rel, rotvec(Vec3(2.0, 0.0, 0.0))) < 0.5);
	// Tilt is still right.
	const Quat q_end = (rotvec(w_world * 2.0) * start).normalized();
	EXPECT(((oc2.attitude() * (q_end.conjugate() * kGravityImu)).normalized() - Vec3::UnitX()).norm() < 0.01);

	// Yaw-only correction: the twist about +x moves to the reference, the tilt stays.
	Quat before = oc.attitude();
	const Quat target = (rotvec(Vec3(0.7, 0.0, 0.0)) * before * rotvec(Vec3(0.0, 0.1, 0.0))).normalized();
	oc.apply_correction(target, 1);
	EXPECT(((oc.attitude() * acc).normalized() - Vec3::UnitX()).norm() < 1e-3);
	const Quat r = (target * oc.attitude().conjugate()).normalized();
	EXPECT(std::abs(std::atan2(r.x(), r.w())) < 1e-6); // no twist left about +x
	oc.apply_correction(target, 0);
	EXPECT(angle_deg(oc.attitude(), target) < 1e-6);
}

static void
test_orientation_bias()
{
	// A still controller with a gyro offset: after > 950 samples the bias is valid and
	// subtracted, and the attitude stops drifting.
	OrientationController oc(80, 100);
	const Vec3 bias(0.01, -0.006, 0.004);
	const Vec3 acc = kGravityImu;
	for (int i = 0; i < 2500; i++) {
		oc.track(acc + Vec3(noise(0.02), noise(0.02), noise(0.02)), bias, i * 0.002, 31.4);
		if (i == 900) {
			EXPECT(!oc.still());
		}
	}
	EXPECT(oc.still());
	EXPECT((oc.bias() - bias).norm() < 1e-6);
	EXPECT(oc.bias_table()[31].norm() > 0.0); // stored in the 31 degree bin
	const Quat q1 = oc.attitude();
	for (int i = 2500; i < 3000; i++) {
		oc.track(acc, bias, i * 0.002, 31.4);
	}
	// Yaw (twist about W +x) no longer drifts; the tilt correction may still settle.
	const Quat d = (oc.attitude() * q1.conjugate()).normalized();
	EXPECT(std::abs(2.0 * std::atan2(d.x(), d.w())) / kDeg < 0.01);
	// Motion clears the still buffer.
	oc.track(acc, Vec3(0.5, 0, 0), 3000 * 0.002, 31.4);
	EXPECT(!oc.still());
}

/*
 * Error-state filter.
 */

static void
feed(SsfFilter &f, const Trajectory &tr, double t0, double t1, double dt, const Vec3 &gyro_bias, double noise_g,
     double noise_a)
{
	for (double t = t0 + dt; t <= t1 + 1e-9; t += dt) {
		const Vec3 g = tr.omega + gyro_bias + Vec3(noise(noise_g), noise(noise_g), noise(noise_g));
		const Vec3 a = tr.accel(t) + Vec3(noise(noise_a), noise(noise_a), noise(noise_a));
		f.propagate(t, a, g, false, 0);
	}
}

static void
test_ssf_propagation()
{
	Trajectory tr;
	SsfFilter f;
	f.init(0.0, tr.p(0), tr.q(0), tr.v(0), tr.accel(0), tr.omega);
	feed(f, tr, 0.0, 1.0, 0.002, Vec3::Zero(), 0.0, 0.0);
	EXPECT((f.p - tr.p(1.0)).norm() < 1e-4);
	EXPECT((f.v - tr.v(1.0)).norm() < 1e-4);
	EXPECT(angle_deg(f.q, tr.q(1.0)) < 1e-3);
	// The covariance grows and stays symmetric.
	EXPECT(f.P(0, 0) > 1e-5);
	EXPECT((f.P - f.P.transpose()).norm() < 1e-12);
	// A step longer than 20 ms is reported.
	EXPECT(!f.propagate(1.05, tr.accel(1.05), tr.omega, false, 0));
}

static void
test_ssf_optical_update()
{
	// Start 3 cm and 3 degrees off with a gyro bias; optical poses at 25 Hz pull it in. The
	// bias converges slowly (initial sigma 0.3 mrad/s) and the mode switches after ~30 s.
	Trajectory tr;
	SsfFilter f;
	const Quat q_err = rotvec(Vec3(2.0, 1.5, -1.5) * kDeg);
	f.init(0.0, tr.p(0) + Vec3(0.03, -0.02, 0.01), tr.q(0) * q_err, tr.v(0), tr.accel(0), tr.omega);
	const Vec3 bias(0.004, -0.003, 0.002);
	double t = 0.0;
	double chi_first = -1.0;
	for (int k = 0; k < 1000; k++) {
		feed(f, tr, t, t + 0.04, 0.002, bias, 0.002, 0.02);
		t += 0.04;
		const double chi = f.chi_square(tr.p(t) + Vec3(noise(0.0005), noise(0.0005), noise(0.0005)), tr.q(t));
		if (chi_first < 0.0) {
			chi_first = chi;
		}
		f.optical_update();
		f.update_mode();
	}
	EXPECT(chi_first > 20.0); // a 3 cm jump is far outside the position noise
	EXPECT((f.p - tr.p(t)).norm() < 0.003);
	EXPECT(angle_deg(f.q, tr.q(t)) < 0.5);
	EXPECT((f.bg - bias).norm() < 0.5 * bias.norm());
	EXPECT(f.mode >= 1);
	EXPECT(f.chi_square(tr.p(t), tr.q(t)) < 12.0);

	// Bias gains frozen: the biases stay where they are.
	const Vec3 bg = f.bg, ba = f.ba;
	f.freeze = 3;
	feed(f, tr, t, t + 0.04, 0.002, bias, 0.0, 0.0);
	t += 0.04;
	f.chi_square(tr.p(t) + Vec3(0.01, 0, 0), tr.q(t));
	f.optical_update();
	EXPECT((f.bg - bg).norm() == 0.0 && (f.ba - ba).norm() == 0.0);

	// ZUPT pulls the velocity to zero without touching the attitude.
	const Quat q_before = f.q;
	f.v = Vec3(0.3, -0.2, 0.1);
	f.zupt();
	EXPECT(f.v.norm() < Vec3(0.3, -0.2, 0.1).norm());
	EXPECT(angle_deg(q_before, f.q) < 1e-9);

	// Occlusion update: position towards the anchor, chi-square returned, freeze 60.
	const Vec3 target = f.p + Vec3(0.005, 0, 0);
	const double chi = f.occlusion_update(target);
	EXPECT(chi > 0.0);
	EXPECT((f.p - target).norm() < 0.005);
	EXPECT(f.freeze == 60);
}

static void
test_ssf_one_axis_rotation()
{
	// The attitude error lives in W: only the rotation about W +x (gravity, yaw) is measured. The
	// controller is turned so that no body axis is along W +x; a 5 degree error about W x is
	// corrected by one update, one about W z hardly, whatever the body axes are.
	const Quat q_true = rotvec(Vec3(0.0, 0.0, 90.0 * kDeg)) * rotvec(Vec3(0.0, 40.0 * kDeg, 0.0));
	for (int axis : {0, 2}) {
		SsfFilter f;
		Vec3 e = Vec3::Zero();
		e(axis) = 5.0 * kDeg;
		f.init(0.0, Vec3(-0.5, 0.0, -0.5), rotvec(e) * q_true, Vec3::Zero(), Vec3(9.81, 0, 0), Vec3::Zero());
		f.chi_square(Vec3(-0.5, 0.0, -0.5), q_true);
		f.optical_update();
		const double left = angle_deg(f.q, q_true);
		if (axis == 0) {
			EXPECT(left < 2.6);
		} else {
			EXPECT(left > 4.9);
		}
	}
}

static void
test_ssf_mode_switch()
{
	// The switch to mode 1 overwrites only the 3x3 attitude block; its cross terms stay.
	SsfFilter f;
	f.init(0.0, Vec3::Zero(), Quat::Identity(), Vec3::Zero(), Vec3(9.81, 0, 0), Vec3::Zero());
	f.P.block<3, 3>(9, 9) = 1e-8 * Mat3::Identity();
	f.P(0, 6) = f.P(6, 0) = 2e-6;
	f.P(6, 7) = f.P(7, 6) = 3e-6;
	f.update_mode();
	EXPECT(f.mode == 1);
	EXPECT(f.P(6, 6) == 1e-4 && f.P(7, 7) == 1e-4 && f.P(8, 8) == 1e-4);
	EXPECT(f.P(6, 7) == 0.0 && f.P(7, 6) == 0.0);
	EXPECT(f.P(0, 6) == 2e-6 && f.P(6, 0) == 2e-6);
}

/*
 * Fusion gate.
 */

static GateInput
gate_base()
{
	GateInput in;
	in.hand = kLeft;
	in.method = OpticalMethod::kTracking;
	in.tracking_ok = true;
	in.residual_count = 4;
	in.residual_mean = 2.0;
	in.T_WH = Pose();
	in.filter_pose = Pose(Quat::Identity(), Vec3(-0.5, 0.0, -0.5)); // outside every box
	in.p_optical = Vec3(-0.44, 0.0, -0.5);                           // 6 cm off
	in.chi2 = 15.0;
	in.lost_frames = 0;
	return in;
}

static void
test_gate()
{
	GateInput in = gate_base();
	EXPECT(fusion_gate(in).accept); // chi2 < 20 and freshly tracked

	in.lost_frames = 5;
	EXPECT(!fusion_gate(in).accept); // needs >= 5 residuals below 4 px
	in.residual_count = 5;
	EXPECT(fusion_gate(in).accept);

	in = gate_base();
	in.chi2 = 25.0;
	in.p_optical += Vec3(0.06, 0, 0);
	EXPECT(!fusion_gate(in).accept);
	in.p_optical = in.filter_pose.t + Vec3(0.01, 0, 0); // dp < 5 cm and chi2 < 90
	EXPECT(fusion_gate(in).accept);
	EXPECT(std::abs(fusion_gate(in).dp - 0.01) < 1e-12);
	in.chi2 = 95.0;
	EXPECT(!fusion_gate(in).accept);
	in.residual_count = 9;
	in.residual_mean = 0.9;
	EXPECT(fusion_gate(in).accept); // tracking ok with many good residuals
	in.tracking_ok = false;
	EXPECT(!fusion_gate(in).accept);
	in.method = OpticalMethod::kP5PNew;
	EXPECT(fusion_gate(in).accept); // P5P_New re-init is always taken

	// Inside box C the dp/chi2 < 90 rule does not apply, inside box B chi2 < 1200 does.
	in = gate_base();
	in.chi2 = 50.0;
	in.filter_pose.t = Vec3(-0.1, 0.0, 0.0); // in B and C
	in.p_optical = in.filter_pose.t + Vec3(0.01, 0, 0);
	EXPECT(fusion_gate(in).accept);
	in.p_optical = in.filter_pose.t + Vec3(0.06, 0, 0);
	EXPECT(!fusion_gate(in).accept);

	// Consecutive rejects: after 3 only good solves pass, after 10 a reset is forced.
	in = gate_base();
	in.chi2 = 100.0;
	in.consecutive_rejects = 3;
	in.residual_count = 7;
	in.residual_mean = 1.1;
	EXPECT(fusion_gate(in).accept);
	in.residual_mean = 1.3;
	EXPECT(!fusion_gate(in).accept);
	in.consecutive_rejects = 10;
	GateResult g = fusion_gate(in);
	EXPECT(!g.accept && g.force_reset);

	// High-quality lock.
	in = gate_base();
	in.residual_count = 14;
	in.residual_mean = 0.5;
	EXPECT(fusion_gate(in).lock);
	in.residual_count = 12;
	in.residual_mean = 0.7;
	EXPECT(!fusion_gate(in).lock);
	in.lock_was_set = true;
	EXPECT(fusion_gate(in).lock);
}

/*
 * Per-hand fusion.
 */

struct Sim
{
	Trajectory tr;
	ControllerFusion fusion;
	double t = 0.0;
	double frame = 0.04;
	int64_t next_sample = 0;
	Vec3 gyro_bias = Vec3::Zero();

	explicit Sim(const Trajectory &trajectory, const FusionParams &params = FusionParams{})
	    : tr(trajectory), fusion(kLeft, params)
	{}

	ImuSample
	sample(double ts) const
	{
		ImuSample s;
		s.t_ns = std::llround(ts * 1e9);
		s.gyro = model_to_sensor(tr.omega + gyro_bias + Vec3(noise(0.002), noise(0.002), noise(0.002)));
		s.accel = model_to_sensor(tr.accel(ts) + Vec3(noise(0.03), noise(0.03), noise(0.03)));
		s.temperature = 30.0;
		return s;
	}

	//! IMU samples up to and including the next image time, then begin_frame.
	FramePrior
	next_frame()
	{
		const double t_img = t + frame;
		// 500 Hz, offset by 1 ms from the image times.
		while (next_sample * 0.002 + 0.001 <= t_img) {
			fusion.add_imu(sample(next_sample * 0.002 + 0.001));
			next_sample++;
		}
		t = t_img;
		return fusion.begin_frame(std::llround(t_img * 1e9), Pose());
	}

	OpticalReport
	optical(double sigma = 0.0003) const
	{
		OpticalReport r;
		r.T_WM = Pose(tr.q(t) * rotvec(Vec3(noise(0.002), noise(0.002), noise(0.002))),
		              tr.p(t) + Vec3(noise(sigma), noise(sigma), noise(sigma)));
		r.method = OpticalMethod::kTracking;
		r.tracking_ok = true;
		r.residual_count = 12;
		r.residual_mean = 0.5;
		r.matched_leds = 12;
		return r;
	}
};

static void
test_fusion_tracking()
{
	Sim sim{Trajectory{}};
	sim.gyro_bias = Vec3(0.003, -0.002, 0.001);

	// Initialisation: three confirmed frames, then tracked.
	for (int k = 0; k < 3; k++) {
		const FramePrior pr = sim.next_frame();
		EXPECT(pr.has_body && !pr.tracked);
		const bool tracked = sim.fusion.report_init(sim.tr.pose(sim.t));
		EXPECT(tracked == (k == 2));
	}
	EXPECT(sim.fusion.tracked());

	double worst_p = 0.0, worst_q = 0.0;
	int accepted = 0;
	for (int k = 0; k < 150; k++) {
		const FramePrior pr = sim.next_frame();
		EXPECT(pr.tracked);
		if (k >= 25) {
			worst_p = std::max(worst_p, (pr.T_WM.t - sim.tr.p(sim.t)).norm());
			worst_q = std::max(worst_q, angle_deg(pr.T_WM.q, sim.tr.q(sim.t)));
			EXPECT(pr.moving);
		}
		const OpticalResult res = sim.fusion.report_optical(sim.optical());
		accepted += res.accepted;
	}
	EXPECT(accepted >= 148);
	// The prior (40 ms of IMU propagation) stays within millimetres and a fraction of a degree.
	EXPECT(worst_p < 0.004);
	EXPECT(worst_q < 1.0);
	EXPECT((sim.fusion.filter().bg - sim.gyro_bias).norm() < sim.gyro_bias.norm()); // moving towards it

	// The IMU-rate output, extrapolated back to the last sample time.
	const int64_t t_last = std::llround(((sim.next_sample - 1) * 0.002 + 0.001) * 1e9);
	const OutputPose out = sim.fusion.pose_at(t_last);
	EXPECT(out.valid && !out.is_3dof);
	EXPECT(out.status == OutputStatus::kTracked);
	EXPECT(out.t_unpredicted_ns == t_last);
	EXPECT(sim.fusion.output().t_ns == t_last + 41000000);
	const double ts = t_last * 1e-9;
	EXPECT((out.T_WM.t - sim.tr.p(ts)).norm() < 0.02);
	EXPECT(angle_deg(out.T_WM.q, sim.tr.q(ts)) < 2.0);
	EXPECT((sim.fusion.output().T_WM_unpredicted.t - sim.tr.p(ts)).norm() < 0.005);
	// Prediction 41 ms ahead.
	const OutputPose ahead = sim.fusion.output();
	EXPECT((ahead.T_WM.t - sim.tr.p(ts + 0.041)).norm() < 0.02);
	// No prediction beyond 1 s.
	const OutputPose far = sim.fusion.pose_at(sim.fusion.output().t_ns + 2000000000LL);
	EXPECT((far.T_WM.t - ahead.T_WM.t).norm() == 0.0);

	// 3DoF yaw is tied to the filter after tracking.
	EXPECT(angle_deg(sim.fusion.orientation().attitude(), sim.tr.q(ts)) < 3.0);

	// A high-quality optical result does not replace the published pose (only a strike does):
	// it stays predicted 41 ms ahead of the newest sample.
	sim.next_frame();
	OpticalReport r = sim.optical(0.0);
	r.residual_count = 15;
	r.residual_mean = 0.4;
	EXPECT(sim.fusion.report_optical(r).accepted);
	const double t_next = sim.next_sample * 0.002 + 0.001;
	sim.fusion.add_imu(sim.sample(t_next));
	sim.next_sample++;
	EXPECT(sim.fusion.output().t_ns == std::llround(t_next * 1e9) + 41000000);
	EXPECT((sim.fusion.output().T_WM.t - sim.tr.p(t_next + 0.041)).norm() < 0.02);
}

static void
test_fusion_loss_still()
{
	// Lying still away from the head: kept on IMU with ZUPTs for 1.2 s, then reset.
	Sim sim{still_trajectory()};
	for (int k = 0; k < 3; k++) {
		sim.next_frame();
		sim.fusion.report_init(sim.tr.pose(sim.t));
	}
	for (int k = 0; k < 20; k++) {
		sim.next_frame();
		sim.fusion.report_optical(sim.optical());
	}
	// Lying still the output may be 3DoF, but it holds a position (published as PICO OS does).
	EXPECT(sim.fusion.output().has_position);
	EXPECT((sim.fusion.output().T_WM.t - sim.tr.p(sim.t)).norm() < 0.02);
	int kept = 0;
	for (int k = 0; k < 100; k++) {
		const FramePrior pr = sim.next_frame();
		EXPECT(pr.tracked && !pr.moving);
		NoOpticalReport r;
		if (!sim.fusion.report_no_optical(r)) {
			break;
		}
		kept++;
		EXPECT(sim.fusion.filter().v.norm() < 0.05);
	}
	EXPECT(kept == 30); // 30 frames of 40 ms = 1.2 s
	EXPECT(!sim.fusion.tracked());
	EXPECT(sim.fusion.output().has_position);
	const FramePrior pr = sim.next_frame();
	EXPECT(!pr.tracked);
	// The init prior is the 3DoF attitude: right tilt.
	const Vec3 up = pr.q_init * sim.tr.accel(sim.t);
	EXPECT((up.normalized() - Vec3::UnitX()).norm() < 0.02);

	// Re-acquisition needs three new frames.
	EXPECT(!sim.fusion.report_init(sim.tr.pose(sim.t)));
	sim.next_frame();
	sim.fusion.report_init_failure();
	sim.next_frame();
	EXPECT(!sim.fusion.report_init(sim.tr.pose(sim.t)));
	sim.next_frame();
	EXPECT(!sim.fusion.report_init(sim.tr.pose(sim.t)));
	sim.next_frame();
	EXPECT(sim.fusion.report_init(sim.tr.pose(sim.t)));
}

static void
test_fusion_loss_moving()
{
	// Moving: the position sigma limit (3 cm) or 3.0 s ends the IMU-only phase; the force-lost
	// flag ends it at once; gate rejects count towards a forced reset.
	Sim sim{Trajectory{}};
	for (int k = 0; k < 3; k++) {
		sim.next_frame();
		sim.fusion.report_init(sim.tr.pose(sim.t));
	}
	for (int k = 0; k < 50; k++) {
		sim.next_frame();
		sim.fusion.report_optical(sim.optical());
	}
	int kept = 0;
	for (int k = 0; k < 200; k++) {
		const FramePrior pr = sim.next_frame();
		EXPECT(pr.moving);
		if (!sim.fusion.report_no_optical(NoOpticalReport{})) {
			break;
		}
		kept++;
	}
	EXPECT(kept > 3 && kept <= 75);
	EXPECT(sim.fusion.output().status != OutputStatus::kTracked || !sim.fusion.tracked());

	Sim s2{Trajectory{}};
	for (int k = 0; k < 3; k++) {
		s2.next_frame();
		s2.fusion.report_init(s2.tr.pose(s2.t));
	}
	s2.next_frame();
	NoOpticalReport forced;
	forced.force_lost = true;
	EXPECT(!s2.fusion.report_no_optical(forced));
	EXPECT(!s2.fusion.tracked());

	Sim s3{Trajectory{}};
	for (int k = 0; k < 3; k++) {
		s3.next_frame();
		s3.fusion.report_init(s3.tr.pose(s3.t));
	}
	int frames = 0;
	bool reset = false;
	for (int k = 0; k < 20 && !reset; k++) {
		s3.next_frame();
		OpticalReport r = s3.optical();
		r.T_WM.t += Vec3(0.3, 0, 0); // far off: rejected every time
		r.residual_mean = 3.0;
		const OpticalResult res = s3.fusion.report_optical(r);
		EXPECT(!res.accepted);
		reset = res.reset;
		frames++;
	}
	EXPECT(reset && frames <= 11);
}

static void
test_fusion_gap_reset()
{
	// An IMU gap above 35 ms resets tracking (resetTrackingIfAbnormalState).
	Sim sim{Trajectory{}};
	for (int k = 0; k < 3; k++) {
		sim.next_frame();
		sim.fusion.report_init(sim.tr.pose(sim.t));
	}
	EXPECT(sim.fusion.tracked());
	sim.next_sample += 25; // 50 ms without samples
	sim.next_frame();
	sim.next_frame();
	EXPECT(!sim.fusion.tracked());
	EXPECT(sim.fusion.abnormal_resets() >= 1);
}

/*!
 * Rotation about a fixed point at the body rate w(t) = w0 + w1 sin(2 pi f t), integrated in 10 us
 * steps; the accelerometer sees gravity only.
 */
struct Spin
{
	Vec3 w0;
	Vec3 w1;
	double f;
	Quat q = rotvec(Vec3(0.3, -0.2, 0.5));
	double t = 0.0;

	Vec3
	rate(double ts) const
	{
		return w0 + w1 * std::sin(2.0 * kPi * f * ts);
	}
	void
	advance(double ts)
	{
		while (t < ts) {
			const double h = std::min(1e-5, ts - t);
			q = (q * rotvec(rate(t + 0.5 * h) * h)).normalized();
			t += h;
		}
	}
};

static void
test_fusion_long_loss_rotating(const Vec3 &w0, const Vec3 &w1, double f)
{
	// Tracked, then seconds without optical tracking while rotating. The fallback re-bases on the
	// picoHighFreq3dof result every sample (addSingleImu 0x360fd8..0x36107c), so the output keeps
	// following the integrated gyro instead of re-integrating a 200-sample window from the last
	// optical base. As in stock, the attitude advances every other sample (out.t is the newest
	// sample, and the next call keeps one sample after dropping t <= base.t), so a step between
	// published samples at t_a < t_b is bounded by the rotation over [t_a - dt, t_b].
	Spin spin{w0, w1, f};
	ControllerFusion fusion(kLeft);
	const Vec3 p(-0.5, 0.0, -0.5);
	const double dt = 0.002;
	int64_t n = 0;
	double t_frame = 0.0;
	bool record = false;
	std::vector<Quat> exact; // per sample index
	struct Rec
	{
		int64_t k;
		Quat q;      // unpredicted
		Quat q_pub;  // published (predicted, smoothed)
	};
	std::vector<Rec> recs;
	auto frame = [&]() {
		const double t_img = t_frame + 0.04;
		while (double(n) * dt + 0.001 <= t_img) {
			const double ts = double(n) * dt + 0.001;
			spin.advance(ts);
			exact.push_back(spin.q);
			ImuSample s;
			s.t_ns = std::llround(ts * 1e9);
			s.gyro = model_to_sensor(spin.rate(ts) + Vec3(noise(0.002), noise(0.002), noise(0.002)));
			s.accel = model_to_sensor(spin.q.conjugate() * kGravityImu +
			                          Vec3(noise(0.03), noise(0.03), noise(0.03)));
			s.temperature = 30.0;
			fusion.add_imu(s);
			const OutputPose &o = fusion.output();
			if (record && o.t_unpredicted_ns == s.t_ns && o.is_3dof && o.status != OutputStatus::kStill) {
				recs.push_back(Rec{n, o.T_WM_unpredicted.q, o.T_WM.q});
			}
			n++;
		}
		t_frame = t_img;
		return fusion.begin_frame(std::llround(t_img * 1e9), Pose());
	};
	for (int k = 0; k < 3; k++) {
		frame();
		fusion.report_init(Pose(spin.q, p));
	}
	EXPECT(fusion.tracked());
	for (int k = 0; k < 25; k++) {
		frame();
		OpticalReport r;
		r.T_WM = Pose(spin.q, p);
		r.method = OpticalMethod::kTracking;
		r.tracking_ok = true;
		r.residual_count = 12;
		r.residual_mean = 0.5;
		r.matched_leds = 12;
		fusion.report_optical(r);
	}
	record = true;
	for (int k = 0; k < 125; k++) { // 5 s without optical tracking
		const FramePrior pr = frame();
		if (pr.tracked) {
			fusion.report_no_optical(NoOpticalReport{});
		} else {
			fusion.report_init_failure();
		}
	}
	EXPECT(!fusion.tracked());
	EXPECT(recs.size() > 1500); // more than 3 s after predicIfLost gave up

	double worst_excess = 0.0, worst_pub = 0.0, worst_drift = 0.0;
	for (size_t i = 1; i < recs.size(); i++) {
		const Rec &a = recs[i - 1], &b = recs[i];
		const double allowed = angle_deg(exact[size_t(a.k - 1)], exact[size_t(b.k)]);
		worst_excess = std::max(worst_excess, angle_deg(a.q, b.q) - allowed);
		worst_pub = std::max(worst_pub, angle_deg(a.q_pub, b.q_pub) - allowed);
		// Relative to the first fallback sample, the output follows the integrated gyro.
		const Quat rel_out = recs[0].q.conjugate() * b.q;
		const Quat rel_exact = exact[size_t(recs[0].k)].conjugate() * exact[size_t(b.k)];
		worst_drift = std::max(worst_drift, angle_deg(rel_out, rel_exact));
	}
	printf("  long loss: %zu fallback samples, step excess %.4f deg (published %.4f), drift %.3f deg\n",
	       recs.size(), worst_excess, worst_pub, worst_drift);
	EXPECT(worst_excess < 0.01);
	EXPECT(worst_pub < 0.1);
	EXPECT(worst_drift < 2.0);
}

static void
test_fusion_yaw_correction_gate()
{
	// The 3DoF yaw correction runs only on the 6DoF path at a new optical pose (addSingleImu
	// 0x361aac): lying still the output is the fallback, so accepted optical poses whose yaw
	// drifts 45 degrees about W +x leave the 3DoF attitude alone. Moving, test_fusion_tracking
	// checks that the 3DoF yaw follows the filter.
	Sim sim{still_trajectory()};
	for (int k = 0; k < 3; k++) {
		sim.next_frame();
		sim.fusion.report_init(sim.tr.pose(sim.t));
	}
	for (int k = 0; k < 75; k++) { // still3dof needs more than 950 still samples
		sim.next_frame();
		sim.fusion.report_optical(sim.optical());
	}
	EXPECT(sim.fusion.output().status == OutputStatus::kStill);
	const Quat q3_before = sim.fusion.orientation().attitude();
	int accepted = 0;
	for (int k = 1; k <= 30; k++) {
		sim.next_frame();
		OpticalReport r = sim.optical(0.0);
		r.T_WM.q = rotvec(Vec3(1.5 * k * kDeg, 0.0, 0.0)) * r.T_WM.q;
		accepted += sim.fusion.report_optical(r).accepted;
	}
	sim.next_frame();
	printf("  yaw gate: status %d still %d accepted %d filter %.2f oc %.2f\n", int(sim.fusion.output().status),
	       int(sim.fusion.output().still), accepted, angle_deg(sim.fusion.filter().q, sim.tr.q(sim.t)),
	       angle_deg(sim.fusion.orientation().attitude(), q3_before));
	EXPECT(accepted >= 28);
	EXPECT(angle_deg(sim.fusion.filter().q, sim.tr.q(sim.t)) > 3.0); // the filter takes the yaw
	EXPECT(angle_deg(sim.fusion.orientation().attitude(), q3_before) < 1.0);
}

static void
test_fusion_3dof_only()
{
	FusionParams params;
	params.force_3dof = true;
	Sim sim{still_trajectory(), params};
	sim.next_frame();
	sim.next_frame();
	const OutputPose &o = sim.fusion.output();
	EXPECT(o.valid && o.is_3dof);
	EXPECT(o.T_WM.t.norm() == 0.0 && !o.has_position);
	EXPECT(((o.T_WM.q * sim.tr.accel(0)).normalized() - Vec3::UnitX()).norm() < 0.01);
}

int
main()
{
	test_butterworth();
	test_calibration();
	test_station_record();
	test_bias_table();
	test_smoother();
	test_orientation_tilt_and_rotation();
	test_orientation_bias();
	test_ssf_propagation();
	test_ssf_optical_update();
	test_ssf_one_axis_rotation();
	test_ssf_mode_switch();
	test_gate();
	test_fusion_tracking();
	test_fusion_loss_still();
	test_fusion_loss_moving();
	test_fusion_gap_reset();
	test_fusion_long_loss_rotating(Vec3(0.4, -0.3, 0.6), Vec3::Zero(), 0.0);
	test_fusion_long_loss_rotating(Vec3(0.4, -0.3, 0.6), Vec3(0.8, 0.5, -0.6), 0.7);
	test_fusion_yaw_correction_gate();
	test_fusion_3dof_only();
	if (failures) {
		fprintf(stderr, "%d failure(s)\n", failures);
		return 1;
	}
	printf("controller IMU fusion: all tests passed\n");
	return 0;
}
