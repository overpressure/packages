// Copyright 2026, overpressure contributors.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Controller IMU side of the PICO 4 controller tracker: calibration, 3DoF filter,
 *         error-state filter, per-frame prior, fusion gate, loss handling and output pose.
 * @ingroup drv_pico4
 */

#include "pico4_ctrack_imu.hpp"

#include <algorithm>
#include <cmath>
#include <complex>
#include <limits>

namespace pico4::ctrack {

namespace {

using Vec4 = Eigen::Vector4d;

constexpr double kPi = 3.14159265358979323846;

/*
 * Parameters of the per-sample path (spec 03 section 4/5).
 */
constexpr double kAccRange = 311.6;      // m/s^2, out-of-range count
constexpr double kGyroRange = 69.4;      // rad/s, logged only
constexpr double kImageStop = 0.5;       // s without an image: reset once
constexpr double kStrikeJump = 20.0;     // m/s^2 change of |a|
constexpr double kStrikeDistance = 0.1;  // m between the controllers
constexpr int kStrikeHold = 80;          // samples
constexpr double kHitDrop = 50.0;        // m/s^2 per-axis drop of |a|
constexpr double kHitHold = 0.08;        // s
constexpr double kHoldOffJump = 10.0;    // m/s^2
constexpr int kHoldOff = 20;             // samples
constexpr int kButterWarmup = 21;        // samples
constexpr int kLongStill = 501;          // samples
constexpr int kNonStillRelease = 1500;   // samples
constexpr double kHighRateMaxDt = 0.03;  // s; longer steps count as 4 ms
constexpr double kHighRateFallbackDt = 0.004;
constexpr double kRestV = 0.05;          // m/s per axis
constexpr double kRestW = 0.1;           // rad/s per axis
constexpr double kBacklash = 0.001;      // m
constexpr double kDeadzone = 0.015;      // m/s
constexpr double kPlausible = 1.5;       // m from the frame's head pose
constexpr double kStillLinAcc = 0.8;     // m/s^2 dead band of the published acceleration
const Vec3 kGravityOut{-9.8, 0.0, 0.0};  // added to R a_smoothed

/*
 * Small helpers.
 */

Mat3
skew(const Vec3 &v)
{
	Mat3 m;
	m << 0.0, -v.z(), v.y(), v.z(), 0.0, -v.x(), -v.y(), v.x(), 0.0;
	return m;
}

//! Quaternion of a rotation vector (angle = norm).
Quat
quat_exp(const Vec3 &r)
{
	const double a = r.norm();
	if (a < 1e-12) {
		return Quat(1.0, 0.5 * r.x(), 0.5 * r.y(), 0.5 * r.z()).normalized();
	}
	const Vec3 u = r / a;
	const double s = std::sin(0.5 * a);
	return Quat(std::cos(0.5 * a), u.x() * s, u.y() * s, u.z() * s);
}

Quat
quat_of(const Vec4 &c)
{
	Quat q;
	q.coeffs() = c;
	return q;
}

//! Coefficients (x, y, z, w) of 0.5 q (x) (0, w): body-frame rate.
Vec4
qdot(const Vec4 &qc, const Vec3 &w)
{
	return 0.5 * (quat_of(qc) * Quat(0.0, w.x(), w.y(), w.z())).coeffs();
}

//! Classic RK4 on q' = 0.5 q (x) (0, w) with w(0) = w0, w(h/2) = mean, w(h) = w1; no normalisation inside.
Vec4
rk4_quat(const Vec4 &q, double h, const Vec3 &w0, const Vec3 &w1, bool normalise_stages)
{
	const Vec3 wm = 0.5 * (w0 + w1);
	auto norm = [&](Vec4 x) { return normalise_stages ? Vec4(x.normalized()) : x; };
	const Vec4 k1 = qdot(q, w0);
	const Vec4 k2 = qdot(norm(q + 0.5 * h * k1), wm);
	const Vec4 k3 = qdot(norm(q + 0.5 * h * k2), wm);
	const Vec4 k4 = qdot(norm(q + h * k3), w1);
	return q + h / 6.0 * (k1 + 2.0 * k2 + 2.0 * k3 + k4);
}

bool
finite(const Vec3 &v)
{
	return v.allFinite();
}

bool
finite(const Quat &q)
{
	return q.coeffs().allFinite();
}

bool
in_box(const Vec3 &p, double x0, double x1, double y0, double y1, double z0, double z1)
{
	return p.x() > x0 && p.x() < x1 && p.y() > y0 && p.y() < y1 && p.z() > z0 && p.z() < z1;
}

//! Full rotation angle between two orientations, radians.
double
angle_between(const Quat &a, const Quat &b)
{
	const double d = std::min(1.0, std::abs(a.normalized().dot(b.normalized())));
	return 2.0 * std::acos(d);
}

//! Joseph-form covariance update with a gain whose rows may have been zeroed.
template <int M>
void
joseph_update(Mat15 &P, const Eigen::Matrix<double, 15, M> &K, const Eigen::Matrix<double, M, 15> &H,
              const Eigen::Matrix<double, M, M> &R)
{
	const Mat15 A = Mat15::Identity() - K * H;
	P = A * P * A.transpose() + K * R * K.transpose();
	P = 0.5 * (P + P.transpose()).eval();
}

} // namespace

/*
 *
 * Calibration
 *
 */

bool
ImuCalib::load(const double gyro12[12], const double accel12[12])
{
	for (int i = 0; i < 12; i++) {
		if (!std::isfinite(gyro12[i]) || gyro12[i] > 2.0 || !std::isfinite(accel12[i]) || accel12[i] > 2.0) {
			return false;
		}
	}
	for (int r = 0; r < 3; r++) {
		for (int c = 0; c < 3; c++) {
			M_gyro(r, c) = gyro12[3 * r + c];
			M_accel(r, c) = accel12[3 * r + c];
		}
	}
	b_gyro = Vec3(gyro12[9], gyro12[10], gyro12[11]);
	b_accel = Vec3(accel12[9], accel12[10], accel12[11]);
	return true;
}

void
ImuCalib::apply(const Vec3 &gyro_raw, const Vec3 &accel_raw, Vec3 &gyro_M, Vec3 &accel_M) const
{
	gyro_M = sensor_to_model(M_gyro * (gyro_raw - b_gyro));
	accel_M = sensor_to_model(M_accel * (accel_raw - b_accel));
}

/*
 *
 * OrientationController (spec 03 3/4)
 *
 */

OrientationController::OrientationController(int gyro_window, int gravity_window)
    : gyro_cap_(gyro_window), grav_cap_(gravity_window)
{
	table_.fill(Vec3::Zero());
}

void
OrientationController::set_bias_table(const std::array<Vec3, 100> &table)
{
	bins_.clear();
	for (int i = 0; i < 100; i++) {
		const bool keep = finite(table[i]) && table[i].norm() <= 0.15 && !table[i].isZero();
		table_[i] = keep ? table[i] : Vec3::Zero();
		if (keep) {
			bins_.push_back(i);
		}
	}
}

void
OrientationController::reset()
{
	gyro_hist_.clear();
	acc_hist_.clear();
	g_qacc_ = Quat::Identity();
	g_f_.setZero();
	g_win_.clear();
	g_sum_.setZero();
	g_sumsq_ = 0.0;
	counter_ = 0;
	mismatch_ = 0;
	window_start_ = t_prev_;
	ang_acc_.setZero();
	acc_s_.setZero();
	// Stock reloads the bias file here; the in-memory table is that file.
}

void
OrientationController::online_bias(const Vec3 &gyro_raw)
{
	const Vec3 gf = still_buf_.empty() ? gyro_raw : Vec3(0.4 * gyro_raw + 0.6 * still_buf_.back());
	if (gf.norm() >= 0.1 ||
	    (!still_buf_.empty() && (gf - still_sum_ / double(still_buf_.size())).norm() >= 0.02)) {
		still_buf_.clear();
		still_sum_.setZero();
		bias_valid_ = false;
	}
	still_buf_.push_back(gf);
	still_sum_ += gf;
	if (still_buf_.size() > 1000) {
		still_sum_ -= still_buf_.front();
		still_buf_.pop_front();
	}
	if (still_buf_.size() > 950) {
		online_bias_ = still_sum_ / double(still_buf_.size());
		if (still_buf_.size() == 1000) {
			store_bias();
		}
		bias_valid_ = true;
	}
}

void
OrientationController::store_bias()
{
	const int bin = std::clamp(int(std::floor(temperature_)), 0, 99);
	if (std::find(bins_.begin(), bins_.end(), bin) == bins_.end()) {
		bins_.insert(std::upper_bound(bins_.begin(), bins_.end(), bin), bin);
	}
	table_[bin] = online_bias_;
}

Vec3
OrientationController::current_bias() const
{
	if (bins_.empty()) {
		return online_bias_;
	}
	if (bins_.size() == 1) {
		return table_[bins_[0]];
	}
	// getBias 0x2c447c: the largest bin <= temperature with the index limited to 0..n-2, so the
	// highest filled bin is never selected (0x2c44f0 caps the search at n-2).
	size_t k = 0;
	for (size_t i = 0; i + 1 < bins_.size(); i++) {
		if (bins_[i] <= temperature_) {
			k = i;
		}
	}
	return table_[bins_[k]];
}

void
OrientationController::gravity_update(const Vec3 &a, double dt, const Quat &dq)
{
	if (!g_win_.empty()) {
		g_f_ = dq.conjugate() * g_f_;
		g_f_ += 2.5 * dt * (a - g_f_);
	} else {
		g_f_ = a;
	}
	g_qacc_ = (g_qacc_ * dq).normalized();
	const Vec3 gw = g_qacc_ * g_f_;
	g_win_.push_back(gw);
	g_sum_ += gw;
	g_sumsq_ += gw.squaredNorm();
	if (int(g_win_.size()) > grav_cap_) {
		g_sum_ -= g_win_.front();
		g_sumsq_ -= g_win_.front().squaredNorm();
		g_win_.pop_front();
	}
}

double
OrientationController::quality() const
{
	const double n = double(g_win_.size());
	if (n <= 0.0) {
		return 0.0;
	}
	const Vec3 m = g_sum_ / n;
	const double var = g_sumsq_ / n - m.squaredNorm();
	const double sigma = var >= 1e-5 ? std::sqrt(var) : 1e-5;
	return std::clamp(0.48 - 0.1 * std::log(sigma), 0.0, 1.0) * n / double(grav_cap_);
}

void
OrientationController::tilt_correction(double dt)
{
	const size_t n = g_win_.size();
	if (n == 0) {
		return;
	}
	const Vec3 m = g_sum_ / double(n);
	const Vec3 g = q_.normalized() * (g_qacc_.conjugate() * m);
	const Quat c = Quat::FromTwoVectors(g, Vec3::UnitX());
	const double qual = quality();
	if (n == 1) {
		q_ = c * q_;
	} else if (std::abs(c.w()) < 0.99875 && qual > 0.75) {
		q_ = c * q_;
	} else if (qual > 0.5) {
		const double s = c.w() >= 0.0 ? 1.0 : -1.0;
		const Vec4 blend = (1.0 - 0.2 * dt) * Quat::Identity().coeffs() + s * 0.2 * dt * c.coeffs();
		q_ = quat_of(blend.normalized()) * q_;
	}
}

bool
OrientationController::track(const Vec3 &accel, const Vec3 &gyro, double t, double temperature)
{
	temperature_ = temperature;
	if (!finite(gyro)) {
		return false;
	}
	online_bias(gyro);
	const Vec3 b = current_bias();
	bias_used_ = b;
	const Vec3 w = finite(b) ? Vec3(gyro - b) : gyro;
	const double dt = t - t_prev_;
	if (dt > 0.2) {
		initialized_ = false;
	}
	t_prev_ = t;

	if (!initialized_) {
		reset();
		initialized_ = true;
		gyro_hist_.push_back(w);
		acc_hist_.push_back(accel);
		gravity_update(accel, 0.0, Quat::Identity());
		const Vec3 a_hat = accel.norm() > 1e-9 ? Vec3(accel.normalized()) : Vec3::UnitX();
		Vec3 r1 = Vec3::UnitY() - a_hat * a_hat.y();
		if (r1.norm() < 1e-6) {
			r1 = Vec3::UnitZ() - a_hat * a_hat.z();
		}
		r1.normalize();
		Mat3 R;
		R.row(0) = a_hat;
		R.row(1) = r1;
		R.row(2) = a_hat.cross(r1);
		q_ = Quat(R);
		return true;
	}

	counter_++;
	gyro_hist_.push_back(w);
	if (int(gyro_hist_.size()) > gyro_cap_) {
		gyro_hist_.pop_front();
	}
	acc_hist_.push_back(accel);
	if (acc_hist_.size() > 4) {
		acc_hist_.pop_front();
	}

	Quat dq = Quat::Identity();
	if (w.squaredNorm() >= 1e-7) {
		dq = quat_exp(w * dt);
	}
	gravity_update(accel, dt, dq);
	if (gyro_hist_.size() < 3) {
		q_ = q_ * dq;
	} else {
		const Vec3 &w0 = gyro_hist_[gyro_hist_.size() - 2];
		q_ = quat_of(rk4_quat(q_.coeffs(), dt, w0, w, false));
	}
	tilt_correction(dt);
	con_ = quality();
	if ((counter_ & 0xffu) == 0) {
		q_.normalize();
	}

	ang_acc_.setZero();
	if (int(gyro_hist_.size()) == gyro_cap_ && gyro_hist_.size() >= 12 && dt > 0.0) {
		Vec3 slope = Vec3::Zero();
		const size_t base = gyro_hist_.size() - 12;
		for (int k = 0; k < 12; k++) {
			slope += (k - 5.5) / 143.0 * gyro_hist_[base + k];
		}
		ang_acc_ = slope / dt;
	}
	acc_s_.setZero();
	if (acc_hist_.size() == 4) {
		acc_s_ = 0.7 * acc_hist_[3] + 0.4 * acc_hist_[2] + 0.1 * acc_hist_[1] - 0.2 * acc_hist_[0];
	}
	const double an = acc_s_.norm();
	if (an > 9.6 && an < 10.0) {
		const Vec3 v = (q_.normalized() * acc_s_).normalized();
		if (Quat::FromTwoVectors(v, Vec3::UnitX()).w() < 0.9) {
			mismatch_++;
		}
	}
	if (t - window_start_ > 1.0) {
		if (mismatch_ >= 401) {
			initialized_ = false;
		}
		window_start_ = t;
		mismatch_ = 0;
	}
	return true;
}

void
OrientationController::apply_correction(const Quat &q_ext, int mode)
{
	const Quat qn = q_.normalized();
	const Quat r = q_ext.normalized() * qn.conjugate();
	if (mode == 0) {
		q_ = r * qn;
	} else {
		const double theta = std::atan2(r.x(), r.w());
		q_ = Quat(std::cos(theta), std::sin(theta), 0.0, 0.0) * qn;
	}
	q_.normalize();
}

/*
 *
 * SSF error-state filter (spec 03 5/4)
 *
 */

void
SsfFilter::init(double t, const Vec3 &p0, const Quat &q0, const Vec3 &v0, const Vec3 &a_m, const Vec3 &w_m)
{
	const bool keep = initialized;
	const Eigen::Matrix<double, 6, 6> P_bias = P.block<6, 6>(9, 9);
	p = p0;
	v = v0;
	q = q0.normalized();
	if (!keep) {
		bg.setZero();
		ba.setZero();
	}
	P.setZero();
	P.diagonal() << 1e-5, 1e-5, 1e-5, 1e-3, 1e-3, 1e-3, 1e-4, 1e-4, 1e-4, 1e-7, 1e-7, 1e-7, 1e-4, 1e-4, 1e-4;
	if (keep) {
		P.block<6, 6>(9, 9) = P_bias;
	}
	last_t = t;
	a_prev = a_m;
	w_prev = w_m;
	initialized = true;
}

bool
SsfFilter::propagate(double t, const Vec3 &a_m, const Vec3 &w_m, bool inflate, int n)
{
	if (!initialized) {
		return true;
	}
	const double dt = t - last_t;
	if (dt <= 0.0) {
		// Not specified: an out-of-order or repeated sample only replaces the stored sample.
		a_prev = a_m;
		w_prev = w_m;
		return true;
	}
	last_t = t;
	if (dt > 5.0) {
		a_prev = a_m;
		w_prev = w_m;
		return true;
	}
	const bool ok = dt <= 0.02;
	const Vec3 &g = kGravityImu;

	const Vec3 w0 = w_prev - bg, w1 = w_m - bg, wm = 0.5 * (w0 + w1);
	const Vec3 a0 = a_prev - ba, a1 = a_m - ba, am = 0.5 * (a0 + a1);

	// RK4 of q, v, p with the samples interpolated linearly; q re-normalised after each stage.
	const Vec4 qc = q.coeffs();
	const Vec4 k1q = qdot(qc, w0);
	const Vec3 k1v = quat_of(qc) * a0 - g;
	const Vec3 k1p = v;
	const Vec4 q2 = (qc + 0.5 * dt * k1q).normalized();
	const Vec3 v2 = v + 0.5 * dt * k1v;
	const Vec4 k2q = qdot(q2, wm);
	const Vec3 k2v = quat_of(q2) * am - g;
	const Vec4 q3 = (qc + 0.5 * dt * k2q).normalized();
	const Vec3 v3 = v + 0.5 * dt * k2v;
	const Vec4 k3q = qdot(q3, wm);
	const Vec3 k3v = quat_of(q3) * am - g;
	const Vec4 q4 = (qc + dt * k3q).normalized();
	const Vec3 v4 = v + dt * k3v;
	const Vec4 k4q = qdot(q4, w1);
	const Vec3 k4v = quat_of(q4) * a1 - g;

	p += dt / 6.0 * (k1p + 2.0 * v2 + 2.0 * v3 + v4);
	v += dt / 6.0 * (k1v + 2.0 * k2v + 2.0 * k3v + k4v);
	q = quat_of((qc + dt / 6.0 * (k1q + 2.0 * k2q + 2.0 * k3q + k4q)).normalized());

	// Transition of the W-frame error state as TAOPropagation 0x259244..0x259678 stores it: R0 is
	// the attitude at the step start (previous state q, 0x259244), a the specific force of the
	// third RK stage in W (k3 + g, 0x2592e0..0x259304); the higher-order terms carry no 1/2 or 1/6.
	const Mat3 R0 = quat_of(qc).toRotationMatrix();
	const Mat3 Sa = skew(quat_of(q3) * am);
	const Mat3 SaR = Sa * R0;
	const double dt2 = dt * dt;
	Mat15 Fd = Mat15::Identity();
	Fd.block<3, 3>(0, 3) = dt * Mat3::Identity();
	Fd.block<3, 3>(0, 6) = -dt2 * Sa;
	Fd.block<3, 3>(0, 9) = dt2 * dt * SaR;
	Fd.block<3, 3>(0, 12) = -dt2 * R0;
	Fd.block<3, 3>(3, 6) = -dt * Sa;
	Fd.block<3, 3>(3, 9) = dt2 * SaR;
	Fd.block<3, 3>(3, 12) = -dt * R0;
	Fd.block<3, 3>(6, 9) = -dt * R0;

	// Adaptive noise.
	double sa = mode == 0 ? 0.1 : 0.05;
	double sg = 0.005;
	if (restricted) {
		sa += std::max(max_lin_acc, 1.0);
		sg = max_gyro > 0.1 ? max_gyro + 0.005 : 0.105;
	}
	if (inflate) {
		sg *= 1.0 + n;
		sa *= 1.0 + 2.0 * n;
	}
	const double wbar = wm.norm();
	const double abar = (q * am - g).norm();
	sg += 0.0005 * abar + 0.015 * wbar;
	if (wm.cwiseAbs().maxCoeff() > 60.0) {
		sg += 15.0;
	}
	sa += 0.015 * abar;
	if (am.cwiseAbs().maxCoeff() > 300.0) {
		sa += 30.0;
	}
	if (inflate_accel) {
		sa += 150.0;
	}
	if (inflate_gyro) {
		sg += 30.0;
	}
	if (dt > 0.01) {
		sa += 0.1 * max_lin_acc;
		sg += 0.1 * max_gyro;
	}

	Vec15 Qd = Vec15::Zero();
	Qd.segment<3>(3).setConstant((sa * dt) * (sa * dt));
	Qd.segment<3>(6).setConstant((sg * dt) * (sg * dt));
	if (mode == 2) {
		for (int i = 0; i < 3; i++) {
			if (P(9 + i, 9 + i) < 1e-5) {
				Qd(9 + i) = 1e-10 * dt * dt;
			}
			if (P(12 + i, 12 + i) < 1e-3) {
				Qd(12 + i) = 1e-8 * dt * dt;
			}
		}
	}
	P = Fd * P * Fd.transpose();
	P.diagonal() += Qd;
	P = 0.5 * (P + P.transpose()).eval();

	a_prev = a_m;
	w_prev = w_m;
	return ok;
}

double
SsfFilter::chi_square(const Vec3 &p_m, const Quat &q_m)
{
	double rp, r0, r1, r2;
	if (restricted) {
		rp = 4e-4;
		r0 = r1 = r2 = 100.0;
	} else {
		rp = 4e-6;
		r0 = 1e-4;
		r1 = r2 = 100.0;
		if (mode != 0 && full_rotation_once) {
			r0 = r1 = r2 = 4e-4;
			full_rotation_once = false;
		}
	}
	R_.setZero();
	R_.diagonal() << rp, rp, rp, r0, r1, r2;

	// World-frame error q_m (x) q^-1 (TAOChiSquare 0x25fa3c..0x25fb28); H is the constant
	// [I 0 0 0 0; 0 0 I 0 0] of the constructor (SSF_EKF_Filter 0x25dda4..0x25ddd4), so component 0
	// is the rotation about W +x (gravity): position and yaw are measured.
	const Quat qe = q_m.normalized() * q.conjugate();
	const double sgn = qe.w() >= 0.0 ? 1.0 : -1.0;
	r_.head<3>() = p_m - p;
	r_.tail<3>() = 2.0 * sgn * qe.vec();

	H_.setZero();
	H_.block<3, 3>(0, 0) = Mat3::Identity();
	H_.block<3, 3>(3, 6) = Mat3::Identity();
	S_ = H_ * P * H_.transpose() + R_;
	const Vec3 r_p = r_.head<3>();
	chi2 = r_p.dot(S_.topLeftCorner<3, 3>().ldlt().solve(r_p));
	return chi2;
}

void
SsfFilter::correct(const Vec15 &dx)
{
	p += dx.segment<3>(0);
	v += dx.segment<3>(3);
	bg = (bg + dx.segment<3>(9)).cwiseMax(-0.02).cwiseMin(0.02);
	ba = (ba + dx.segment<3>(12)).cwiseMax(-0.2).cwiseMin(0.2);
	const Vec3 dth = dx.segment<3>(6);
	const double h2 = dth.squaredNorm() / 4.0;
	Quat dq;
	if (h2 < 1.0) {
		dq = Quat(std::sqrt(1.0 - h2), 0.5 * dth.x(), 0.5 * dth.y(), 0.5 * dth.z());
	} else {
		dq = Quat(1.0, 0.5 * dth.x(), 0.5 * dth.y(), 0.5 * dth.z()).normalized();
	}
	// The correction multiplies from the left (TAOApplyCorrection 0x25b6cc..0x25b74c).
	q = (dq * q).normalized();
}

void
SsfFilter::optical_update()
{
	Eigen::Matrix<double, 15, 6> K = (S_.partialPivLu().solve(H_ * P)).transpose();
	const bool hard = max_lin_acc >= 25.0 || max_gyro >= 5.0;
	if (hard) {
		K.middleRows<3>(9).setZero();
	}
	if (mode == 0 || hard) {
		K.middleRows<3>(12).setZero();
	}
	if (freeze > 0 || restricted) {
		K.middleRows<6>(9).setZero();
	}
	const Vec15 dx = K * r_;
	joseph_update<6>(P, K, H_, R_);
	correct(dx);
}

void
SsfFilter::update_mode()
{
	const Vec15 d = P.diagonal();
	if (mode == 0) {
		if (d(9) < 4e-8 && d(10) < 4e-8 && d(11) < 4e-8) {
			mode = 1;
			// Only the 3x3 attitude block is overwritten; its cross terms stay
			// (updatePose 0x3b3a5c..0x3b3a94, stores F+0x550..0x650).
			P.block<3, 3>(6, 6) = 1e-4 * Mat3::Identity();
		}
	} else if (mode == 1) {
		if (d(12) < 2.5e-5 && d(13) < 2.5e-5 && d(14) < 2.5e-5) {
			mode = 2;
		}
	}
}

void
SsfFilter::zupt()
{
	Eigen::Matrix<double, 3, 15> H = Eigen::Matrix<double, 3, 15>::Zero();
	H.block<3, 3>(0, 3) = Mat3::Identity();
	const Mat3 R = 1e-4 * Mat3::Identity();
	const Mat3 S = H * P * H.transpose() + R;
	Eigen::Matrix<double, 15, 3> K = (S.ldlt().solve(H * P)).transpose();
	K.topRows<3>().setZero();
	K.bottomRows<9>().setZero();
	const Vec15 dx = K * (-v);
	joseph_update<3>(P, K, H, R);
	correct(dx);
}

double
SsfFilter::occlusion_update(const Vec3 &p_pred)
{
	Eigen::Matrix<double, 3, 15> H = Eigen::Matrix<double, 3, 15>::Zero();
	H.block<3, 3>(0, 0) = Mat3::Identity();
	const Mat3 R = 1e-4 * Mat3::Identity();
	const Mat3 S = H * P * H.transpose() + R;
	const Vec3 r = p_pred - p;
	const double chi = r.dot(S.ldlt().solve(r));
	Eigen::Matrix<double, 15, 3> K = (S.ldlt().solve(H * P)).transpose();
	K.bottomRows<9>().setZero();
	const Vec15 dx = K * r;
	joseph_update<3>(P, K, H, R);
	correct(dx);
	freeze = 60;
	return chi;
}

/*
 *
 * Output smoother (spec 03 5/6)
 *
 */

bool
HighFreqSmoother::update(Vec3 &p, Vec3 &v, Quat &q, double s, bool rate_flag)
{
	if (!initialized_) {
		q_ref_ = q;
	}
	Quat dq = q_ref_.conjugate() * q;
	if (dq.w() < 0.0) {
		dq.coeffs() *= -1.0;
	}
	const double ac = std::acos(std::clamp(dq.w(), -1.0, 1.0));
	Vec3 r = 2.0 * ac / (std::sin(ac) + 1.745e-7) * dq.vec();
	if (!finite(r)) {
		r.setZero();
	}
	const std::array<double, 9> z{p.x(), v.x(), p.y(), v.y(), p.z(), v.z(), r.x(), r.y(), r.z()};
	if (!initialized_) {
		x_ = z;
		P_.fill(0.0);
		initialized_ = true;
	}
	const double r_pos = std::pow(s, 5) * (rate_flag ? 5.0 : 20.0);
	const double r_rot = 0.8 * r_pos;
	auto kf = [&](int i, double pred, double Q, double R) {
		P_[i] += Q;
		const double den = P_[i] + R;
		const double K = den > 0.0 ? P_[i] / den : 1.0;
		x_[i] = pred + K * (z[i] - pred);
		P_[i] *= 1.0 - K;
	};
	for (int j = 0; j < 3; j++) {
		const int ip = 2 * j, iv = 2 * j + 1;
		const double pred_p = x_[ip] + x_[iv] * 0.002;
		const double pred_v = x_[iv];
		kf(ip, pred_p, 0.1, r_pos);
		kf(iv, pred_v, 0.1, 0.0);
	}
	for (int i = 6; i < 9; i++) {
		kf(i, x_[i], 0.5, r_rot);
	}
	for (double x : x_) {
		if (!std::isfinite(x)) {
			initialized_ = false;
			return false;
		}
	}
	p = Vec3(x_[0], x_[2], x_[4]);
	v = Vec3(x_[1], x_[3], x_[5]);
	q = (q_ref_ * quat_exp(Vec3(x_[6], x_[7], x_[8]))).normalized();
	q_ref_ = q;
	return true;
}

/*
 *
 * Butterworth (spec 03 4/3.10)
 *
 */

Butterworth::Butterworth(int order)
{
	using C = std::complex<double>;
	const int N = std::max(1, order);
	const double Wp = 0.6498394;
	const double Wn = Wp / std::pow(std::pow(10.0, 0.1) - 1.0, 1.0 / (2.0 * N));
	// Analog poles, then the bilinear transform s = 2 (z - 1) / (z + 1).
	std::vector<C> a(1, C(1.0, 0.0));
	C den(1.0, 0.0);
	for (int k = 0; k < N; k++) {
		const C pole = Wn * std::exp(C(0.0, kPi * (2.0 * k + N + 1) / (2.0 * N)));
		const C zk = (2.0 + pole) / (2.0 - pole);
		den *= (2.0 - pole);
		std::vector<C> next(a.size() + 1, C(0.0, 0.0));
		for (size_t i = 0; i < a.size(); i++) {
			next[i] += a[i];
			next[i + 1] -= zk * a[i];
		}
		a = next;
	}
	const double gain = (std::pow(Wn, N) / den).real();
	b_.assign(N + 1, 0.0);
	a_.assign(N + 1, 0.0);
	double binom = 1.0;
	for (int i = 0; i <= N; i++) {
		b_[i] = gain * binom;
		binom = binom * (N - i) / (i + 1);
		a_[i] = a[i].real();
	}
	for (auto &w : w_) {
		w.assign(N + 1, 0.0);
	}
}

double
Butterworth::filter(int ch, double x)
{
	std::vector<double> &w = w_[ch];
	const size_t n = b_.size();
	for (size_t i = n - 1; i > 0; i--) {
		w[i] = w[i - 1];
	}
	double w0 = x;
	for (size_t i = 1; i < n; i++) {
		w0 -= a_[i] * w[i];
	}
	w[0] = w0;
	double y = 0.0;
	for (size_t i = 0; i < n; i++) {
		y += b_[i] * w[i];
	}
	return y;
}

/*
 *
 * Fusion gate (spec 02 13.1)
 *
 */

GateResult
fusion_gate(const GateInput &in)
{
	GateResult res;
	const Pose E = in.T_WH.inverse() * in.filter_pose;
	const Vec3 &h = E.t;
	const bool in_a = in_box(h, -0.3, 0.3, -0.2, 0.3, -0.6, 0.0);
	const bool in_b = in_box(h, -0.25, 0.25, -0.25, 0.35, -0.3, 0.3);
	const bool in_c = in_box(h, -0.25, 0.05, -0.12, 0.22, -0.1, 0.2);
	// Reference orientation in the head frame, w first as stored at 0xb4c978..0xb4c990 (the
	// product at updatePose 0x3b0dc0 pairs the first constant with w).
	const Quat ref = Quat(-0.41378566, 0.73169416, 0.49452433, 0.2210221).normalized();
	// Full rotation angle: 2 atan2(|vec|, |w|) of the relative rotation (updatePose 0x3b0e44,
	// doubled at 0x3b1568) against 1.0 rad.
	const double ang = angle_between(E.q, ref);
	res.dp = (in.filter_pose.t - in.p_optical).norm();
	const int n = in.residual_count;
	const double e = in.residual_mean;

	bool accept;
	if (in.method == OpticalMethod::kP5PNew) {
		accept = true;
	} else if (ang < 1.0 && in_a && in.near && in.hand == kRight) {
		accept = true;
	} else if (in.tracking_ok && n >= 9 && e < 1.0) {
		accept = true;
	} else if (in_b && res.dp < 0.05 && in.chi2 < 1200.0) {
		accept = true;
	} else if (!in_c && ((res.dp < 0.05 && in.chi2 < 90.0) || in.chi2 < 12.0)) {
		accept = true;
	} else if (in.consecutive_rejects > 2) {
		if (in.consecutive_rejects > 9) {
			res.force_reset = true;
			accept = false;
		} else {
			accept = n >= 7 && e < 1.2;
		}
	} else if (in.chi2 >= 20.0) {
		accept = false;
	} else if (in.lost_frames < 4) {
		accept = true;
	} else {
		accept = n >= 5 && e < 4.0;
	}
	res.accept = accept;
	res.lock = accept && ((n >= 14 && e < 0.6) || (in.lock_was_set && n >= 11 && e < 0.8));
	return res;
}

/*
 *
 * ControllerFusion
 *
 */

ControllerFusion::ControllerFusion(Hand hand, const FusionParams &params)
    : hand_(hand), params_(params), oc_a_(20, 50), oc_b_(80, 100), butter_(params.butterworth_order)
{
	fore_ = hand == kRight ? Vec3(0.0, -0.04, 0.13) : Vec3(0.0, 0.04, 0.13);
}

void
ControllerFusion::soft_reset(bool clear_buffers)
{
	// RESET_A/B/C of resetTrackingIfAbnormalState (spec 03 4/3.5).
	optical_state_ = 0;
	optical_new_ = false;
	tracked_ = false;
	lock_198c_ = false;
	window_.clear();
	if (clear_buffers) {
		prop_buf_.clear();
		frame_buf_.clear();
	}
}

bool
ControllerFusion::check_abnormal(const Sample &s)
{
	bool ok = true;
	if ((s.accel.cwiseAbs().array() >= kAccRange).any()) {
		acc_out_count_++;
	}
	if (params_.acc_range_reset && acc_out_count_ >= 21 && lost_ >= 4) {
		// Off by default: P+0x6c2 is only cleared by the constructor (0x2f5e68). The counter
		// restarts after the reset.
		soft_reset(false);
		prop_buf_.clear();
		oc_a_.invalidate();
		oc_b_.invalidate();
		acc_out_count_ = 0;
		abnormal_resets_++;
		ok = false;
	}
	(void)kGyroRange; // out-of-range gyro is only logged on phoenix
	const double dt = have_prev_ ? s.t - prev_.t : 1e9;
	if (dt <= 0.0 || std::abs(dt) > params_.max_imu_gap) {
		soft_reset(true);
		warmup_ = 0;
		if (still_count_ > 1500 || std::abs(dt) > 0.1) {
			complete_still_ = true;
			still_resume_ = true;
			non_still_count_ = 0;
		}
		if (have_prev_) {
			abnormal_resets_++;
		}
		ok = false;
	}
	if (s.t - t_last_image_ > kImageStop && !image_stop_latched_) {
		soft_reset(false);
		warmup_ = 0;
		image_stop_latched_ = true;
	}
	return ok;
}

void
ControllerFusion::process_strike(const Sample &s)
{
	if (other_ != nullptr && have_prev_) {
		const double d = (ssf_.p - other_->ssf_.p).norm();
		const Vec3 u1 = ssf_.v.norm() > 0.0 ? Vec3(ssf_.v.normalized()) : Vec3::Zero();
		const Vec3 u2 = other_->ssf_.v.norm() > 0.0 ? Vec3(other_->ssf_.v.normalized()) : Vec3::Zero();
		const double jump = std::abs(s.accel.norm() - prev_.accel.norm());
		if (jump > kStrikeJump && d < kStrikeDistance && u1.dot(u2) < -0.5 && strike_count_ == 0) {
			strike_ = true;
		}
	}
	if (strike_) {
		if (strike_count_ <= kStrikeHold && !frame_buf_.empty()) {
			strike_count_++;
		} else {
			strike_ = false;
			strike_count_ = 0;
		}
	}
}

void
ControllerFusion::judge_long_still(const Sample &s)
{
	if (still3dof_) {
		still_count_++;
		non_still_count_ = 0;
		if (tracked_) {
			complete_still_ = false;
		}
	} else {
		still_count_ = 0;
		non_still_count_++;
		if (non_still_count_ > kNonStillRelease || tracked_) {
			complete_still_ = false;
		}
	}
	if (params_.rate_flag) {
		complete_still_ = false;
	}
	if (still_count_ >= kLongStill && !latched_) {
		latched_ = true;
		q3_snapshot_ = s.q3;
		acc_snapshot_ = s.accel;
		ang_count_ = 0;
	}
}

Quat
ControllerFusion::integrate_q(const Quat &q, double dt, const Vec3 &gyro)
{
	gyro_deque_.push_back(gyro);
	if (gyro_deque_.size() > 3) {
		gyro_deque_.pop_front();
	}
	if (gyro_deque_.size() < 3) {
		const double wn = gyro.norm();
		if (wn * dt == 0.0) {
			return q;
		}
		return (q * quat_exp(gyro * dt)).normalized();
	}
	return quat_of(rk4_quat(q.coeffs(), dt, gyro_deque_[1], gyro_deque_[2], false).normalized());
}

ControllerFusion::PoseInfo
ControllerFusion::one_step(const PoseInfo &prev, const Sample &c)
{
	double dt = c.t - prev.t;
	if (dt > kHighRateMaxDt) {
		dt = kHighRateFallbackDt;
	}
	PoseInfo out = prev;
	out.q = integrate_q(prev.q, dt, c.gyro);
	if (ssf_.restricted) {
		out.v.setZero();
		out.p = prev.p;
		out.accel = prev.q.conjugate() * kGravityImu;
		out.gyro.setZero();
	} else {
		const Vec3 a_w = out.q * c.accel;
		out.v = prev.v + (a_w - kGravityImu) * dt;
		out.p = prev.p + 0.5 * (out.v + prev.v) * dt;
		out.accel = c.accel;
		out.gyro = c.gyro;
	}
	out.t = c.t;
	return out;
}

ControllerFusion::PoseInfo
ControllerFusion::high_freq_3dof()
{
	// picoHighFreq3dof 0x366898, called for every sample: with fewer than two buffered samples the
	// base is returned as it is; otherwise the attitude is re-integrated from the base time to the
	// newest sample, starting from the gyro interpolated to the base time between the first two
	// samples (0x366a3c..0x366ab8), one first-order step q (x) (1, w_mean dt / 2) per sample pair,
	// normalised (0x366abc..0x366c9c). Position stays, velocity is zero (0x366e20), and only the
	// newest gyro enters the 3-deep deque (0x366cf4..0x366df4). The base is only read; add_imu
	// re-bases on the result, so with out.t = newest t the next call integrates every other sample.
	if (prop_buf_.size() < 2) {
		return base_;
	}
	const Sample &s0 = prop_buf_[0];
	const Sample &s1 = prop_buf_[1];
	const double w = (s1.t - base_.t) / (s1.t - s0.t);
	Vec3 g_prev = w * s0.gyro + (1.0 - w) * s1.gyro;
	double t_prev = base_.t;
	Quat d = Quat::Identity();
	for (size_t i = 1; i < prop_buf_.size(); i++) {
		const Sample &e = prop_buf_[i];
		const Vec3 h = 0.25 * (e.t - t_prev) * (g_prev + e.gyro);
		d = d * Quat(1.0, h.x(), h.y(), h.z());
		if (d.squaredNorm() > 0.0) {
			d.normalize();
		}
		g_prev = e.gyro;
		t_prev = e.t;
	}
	const Sample &last = prop_buf_.back();
	gyro_deque_.push_back(last.gyro);
	if (gyro_deque_.size() > 3) {
		gyro_deque_.pop_front();
	}
	PoseInfo out = base_;
	out.t = last.t;
	out.v.setZero();
	out.q = base_.q * d;
	out.accel = last.accel;
	out.gyro = last.gyro;
	return out;
}

void
ControllerFusion::add_imu(const ImuSample &raw, const Pose *T_WH_at_sample)
{
	if (!finite(raw.gyro) || !finite(raw.accel)) {
		return; // dropped (spec 03 3/2)
	}
	Sample s;
	calib_.apply(raw.gyro, raw.accel, s.gyro, s.accel);
	if (!finite(s.gyro) || !finite(s.accel)) {
		return;
	}
	s.t = double(raw.t_ns) * 1e-9;
	s.temperature = raw.temperature;
	s.flagbit = (raw.status & 1) != 0;
	s.flagrest = raw.status >> 1;
	head_ = T_WH_at_sample != nullptr ? *T_WH_at_sample : frame_head_;

	// Strike, hit and hold-off (spec 03 4/3.2-3.4).
	process_strike(s);
	if (have_prev_ && (std::abs(prev_.accel.x()) - std::abs(s.accel.x()) > kHitDrop ||
	                   std::abs(prev_.accel.y()) - std::abs(s.accel.y()) > kHitDrop)) {
		hit_ = true;
		t_hit_ = s.t;
	}
	if (s.t - t_hit_ >= kHitHold) {
		hit_ = false;
	}
	if (have_prev_ && (prev_.accel - s.accel).cwiseAbs().maxCoeff() > kHoldOffJump) {
		hold_off_ = kHoldOff;
	} else if (hold_off_ >= 1) {
		hold_off_ -= 1;
	}
	s.strike = strike_;

	auto keep_as_previous = [&]() {
		prev_ = s;
		have_prev_ = true;
	};
	if (!check_abnormal(s)) {
		keep_as_previous();
		return;
	}

	// 3DoF attitude (spec 03 4/3.6).
	OrientationController &oc = rate_flag_oc();
	oc.track(s.accel, s.gyro, s.t, s.temperature);
	if (!finite(oc.attitude()) || !std::isfinite(oc.confidence()) || !finite(oc.angular_acceleration()) ||
	    !finite(oc.smoothed_accel())) {
		keep_as_previous();
		return;
	}
	s.q3 = oc.attitude();
	still3dof_ = oc.still();
	judge_long_still(s);

	frame_buf_.push_back(s);
	while (int(frame_buf_.size()) > params_.buffer_capacity) {
		frame_buf_.pop_front();
	}

	if (params_.force_3dof) {
		output_3dof(s);
		keep_as_previous();
		return;
	}

	// Butterworth (only used with the status flag or a strike), SSF biases (spec 03 4/3.9-3.10).
	Sample c = s;
	const double in[6] = {s.accel.x(), s.accel.y(), s.accel.z(), s.gyro.x(), s.gyro.y(), s.gyro.z()};
	double f[6];
	for (int i = 0; i < 6; i++) {
		f[i] = butter_.filter(i, in[i]);
	}
	if (warmup_ >= kButterWarmup && (s.flagbit || strike_)) {
		c.accel = Vec3(f[0], f[1], f[2]);
		c.gyro = Vec3(f[3], f[4], f[5]);
	}
	warmup_++;
	c.accel -= ssf_.ba;
	c.gyro -= ssf_.bg;
	if (!strike_) {
		prop_buf_.push_back(c);
		while (int(prop_buf_.size()) > params_.buffer_capacity) {
			prop_buf_.pop_front();
		}
	}

	// Re-anchoring on a new optical pose (spec 03 4/3.11).
	if (complete_still_ && non_still_count_ <= 9) {
		base_.p = last_good_.T_WM.t;
		base_.q = last_good_.T_WM.q;
		base_.t = prev_.t;
	}
	bool new_opt = false;
	if (optical_new_ && (!still3dof_ || (still_resume_ && lock_198c_))) {
		base_ = optical_;
		base_.accel = c.accel;
		base_.gyro = c.gyro;
		optical_new_ = false;
		new_opt = true;
	}
	if (strike_) {
		base_ = optical_;
		base_.t = s.t;
		hfkf_.reset();
	}

	// Propagation selector (spec 03 4/3.12).
	auto drop_old = [&]() {
		while (!prop_buf_.empty() && prop_buf_.front().t <= base_.t) {
			prop_buf_.pop_front();
		}
	};
	PoseInfo out;
	bool six = false;
	if (optical_state_ != 0 && ((!still3dof_ && !complete_still_) || still_resume_)) {
		still_resume_ = false;
		if (new_opt && !prop_buf_.empty()) {
			// The replay advances the base itself (base := out after every step,
			// 0x360dac..0x360e08), and the IMU-only step count becomes the number of replayed
			// samples (0x360d4c, 0x360e2c, 0x361640).
			drop_old();
			for (const Sample &e : prop_buf_) {
				base_ = one_step(base_, e);
			}
			out = base_;
			imu_only_steps_ = int(prop_buf_.size());
		} else {
			out = one_step(running_, c);
			imu_only_steps_++;
		}
		six = true;
	} else {
		drop_old();
		out = high_freq_3dof();
	}

	if (six) {
		// A non-finite p, q or v re-bases on the optical pose at the sample time and outputs that
		// base (0x3617b4..0x3618d0, 0x361930..0x361aa0). The fallback path does not pass here.
		if (!finite(out.p) || !finite(out.q) || !finite(out.v)) {
			base_ = optical_;
			base_.t = s.t;
			out = base_;
		}
		// 3DoF yaw tied to the optical solution (spec 02 5.1, spec 03 3/4.8): on the 6DoF path at
		// a new optical pose with P+0x198d set, mode 1 with the output attitude, then OC+0x00 := 1
		// (0x361aac..0x361b00). P+0x198d is not cleared here; the frame handlers set it.
		if (new_opt && yaw_pending_) {
			oc.apply_correction(out.q, 1);
			oc.mark_initialized();
		}
	}

	// The fallback replaces out.p by the held or arm-model position; the result becomes the new
	// base (0x360fd8..0x36107c, both branches; base+0x68 stays), so the next picoHighFreq3dof
	// continues from it. Only the 6DoF path updates the running pose (0x361bc0); under a strike
	// ("phoenix") it is the optical pose at the sample time instead (0x361c74..0x361d70).
	sample_output(s, c, out, six);
	if (!six) {
		base_ = out;
	} else if (strike_) {
		running_ = optical_;
		running_.t = s.t;
	} else {
		running_ = out;
	}
	keep_as_previous();
}

void
ControllerFusion::stand_to_move(OutputPose &pose, const Sample &s)
{
	// Release of the long-still latch (spec 03 4/3.15).
	if (!latched_) {
		return;
	}
	const double ang_q = angle_between(s.q3, q3_snapshot_) * 180.0 / kPi;
	double ang_a = 0.0;
	if (s.accel.norm() > 0.0 && acc_snapshot_.norm() > 0.0) {
		ang_a = std::acos(std::clamp(s.accel.normalized().dot(acc_snapshot_.normalized()), -1.0, 1.0)) * 180.0 / kPi;
	}
	if (ang_a > 25.0) {
		ang_count_++;
	}
	if ((ang_q >= 30.0 && ang_count_ >= 100) || ang_q > 20.0) {
		latched_ = false;
		ang_count_ = 0;
	} else {
		pose.status = OutputStatus::kStill;
	}
}

void
ControllerFusion::sample_output(const Sample &s, const Sample &c, PoseInfo &out, bool six)
{
	OutputPose pose;
	pose.t_ns = std::llround(s.t * 1e9);
	pose.optical_state = base_.state;
	pose.imu_only_steps = imu_only_steps_;
	if (six) {
		// 6DoF output (spec 03 4/3.13).
		const Pose T_hc = head_.inverse() * Pose(out.q, out.p);
		head_rel_p_ = T_hc.t;
		resting_ = (out.v.cwiseAbs().array() < kRestV).all() && (c.gyro.cwiseAbs().array() < kRestW).all();
		bool together = false;
		if (other_ != nullptr) {
			auto box = [](const Vec3 &r) { return in_box(r, -0.2, 0.02, 0.0, 0.12, -0.8, 0.03); };
			together = box(head_rel_p_) && box(other_->head_rel_p_) &&
			           std::abs(head_rel_p_.y() - other_->head_rel_p_.y()) < 0.03 && resting_ &&
			           other_->resting_;
		}
		for (int i = 0; i < 3; i++) {
			if (out.p(i) > held_p_(i) + kBacklash) {
				held_p_(i) = out.p(i) - kBacklash;
			} else if (out.p(i) < held_p_(i) - kBacklash) {
				held_p_(i) = out.p(i) + kBacklash;
			}
		}
		Vec3 v = out.v;
		for (int i = 0; i < 3; i++) {
			if (std::abs(v(i)) <= kDeadzone) {
				v(i) = (v(i) >= 0.0 ? 1.0 : -1.0) * v(i) * v(i) / kDeadzone;
			}
		}
		pose.T_WM = Pose(out.q, together ? held_p_ : out.p);
		pose.v = v;
		pose.omega = c.gyro;
		pose.is_3dof = false;
		pose.status = (hit_ || strike_) ? OutputStatus::kTrackedHit : OutputStatus::kTracked;
		if (still3dof_) {
			pose.status = OutputStatus::kStill;
		}
		last_out_p_ = pose.T_WM.t;
		has_out_p_ = true;
		pose.has_position = true;
	} else {
		// Fallback when not optically tracked (spec 03 4/3.14).
		Vec3 p = last_out_p_;
		pose.has_position = has_out_p_;
		if (arm_zone_ && !still3dof_ && !complete_still_) {
			p = (head_ * elbow_H_).t - out.q * fore_;
			pose.has_position = true;
		}
		out.p = p; // 0x360fc8 (hold), 0x361628 (arm model)
		pose.T_WM = Pose(out.q, p);
		pose.v = out.v;
		pose.omega = c.gyro;
		pose.is_3dof = true;
		pose.status = OutputStatus::kFallback;
		if (still3dof_) {
			pose.status = OutputStatus::kStill;
			pose.T_WM.q = last_good_.T_WM.q;
		} else if (hit_) {
			pose.status = OutputStatus::kFallbackHit;
		}
	}

	// Arm model (spec 03 4/3.15).
	const Pose T_hc = head_.inverse() * pose.T_WM;
	const Vec3 &r = T_hc.t;
	arm_zone_ = false;
	if (r.z() > -0.15 && std::abs(r.z()) < 0.35 && std::abs(r.y()) < 0.5 && !follow_) {
		arm_zone_ = true;
		fore_ = hand_ == kRight ? Vec3(0.0, -0.04, 0.13) : Vec3(0.0, 0.04, 0.13);
		if (r.z() > 0.1) {
			const bool far = hand_ == kRight ? r.y() < -0.3 : r.y() > 0.4;
			fore_.z() += far ? 0.2 : 0.12;
		}
		// P+0xb40 is an SE3 whose translation is P+0xb60 (the forearm offset, addSingleImu
		// 0x3629b0..0x3629d4) and whose rotation only the constructor writes (identity).
		elbow_H_ = T_hc * Pose(Quat::Identity(), fore_);
	}
	stand_to_move(pose, s);

	// A strike while not tracked or lost for more than 3 frames (the else of the optical
	// passthrough test, 0x363a84..0x363ae4) resets tracking, clears the propagation and history
	// lists, and zeroes the velocity, the smoothed acceleration used for aLin and the OC rate
	// vector (0x363b50..0x363c98). P+0x1da8 / P+0x1dab there only choose the log sink.
	bool zero_acc = false, zero_ang_acc = false;
	if (strike_ && !(tracked_ && lost_ <= 3)) {
		reset_tracking();
		prop_buf_.clear();
		frame_buf_.clear();
		pose.v.setZero();
		zero_acc = true;
		zero_ang_acc = true;
	}

	pose.t_unpredicted_ns = pose.t_ns;
	pose.T_WM_unpredicted = pose.T_WM;
	pose.v_unpredicted = pose.v;

	// Output-time prediction and smoothing (spec 03 4/3.17), moving only.
	const OrientationController &oc = rate_flag_oc();
	const double t_pred = params_.prediction_time;
	// Only a moving sample without a strike is predicted: a strike branches from 0x363d74 straight
	// to the strike block 0x3639e8 (passthrough / reset), skipping the prediction and the smoother
	// 0x363e08..0x364d40, which itself continues at 0x3639e8 (0x364d60, 0x364dd8).
	if (!still3dof_ && !complete_still_ && !strike_) {
		double kV = s.flagbit ? 0.3 : 3.0;
		double kW = s.flagbit ? 0.05 : 0.1;
		const double gain = oc.confidence();
		if (near_) {
			kW = 0.05 * (1.0 - gain) + 0.01;
			kV = 0.5 * (1.0 - gain) + 0.05;
		}
		double extra = 0.002;
		if (params_.rate_flag) {
			extra = 0.005;
			kW = 0.05;
			kV = 1.0;
		}
		const double T = t_pred + extra;
		Vec3 wbar = Vec3::Zero();
		for (const Vec3 &w : gyro_deque_) {
			wbar += w;
		}
		if (!gyro_deque_.empty()) {
			wbar /= double(gyro_deque_.size());
		}
		pose.omega = wbar;
		const double hT = std::min(T, kV * pose.v.norm());
		const double hR = std::min(T, kW * wbar.norm());
		const double ramp = std::min(ramp_ / 50.0, 1.0);
		Vec3 p = pose.T_WM.t + pose.v * (ramp * hT);
		Quat q = pose.T_WM.q;
		if (wbar.norm() > 0.0) {
			q = q * quat_exp(wbar.normalized() * (hR * wbar.norm()));
		}
		Vec3 v = pose.v;
		if (!hfkf_.update(p, v, q, gain, params_.rate_flag)) {
			return; // nothing published for this sample
		}
		pose.T_WM = Pose(q, p);
		pose.v = v;
		pose.t_ns += std::llround(T * 1e9);
	}
	// The optical passthrough runs on phoenix only while a strike is flagged (P+0x1b20, tested at
	// addSingleImu 0x3639e8 / 0x363d74 before the override at 0x363a84), not under the
	// high-quality lock.
	if (params_.optical_passthrough && strike_ && tracked_ && lost_ <= 3) {
		if (lost_ <= 1) {
			pose.T_WM = Pose(optical_.q, optical_.p);
			pose.v = optical_.v;
		} else {
			pose.T_WM = last_good_.T_WM;
		}
		pose.omega.setZero();
		pose.t_ns = std::llround(s.t * 1e9) + std::llround(t_pred * 1e9);
		zero_acc = true; // 0x363b2c, 0x363b44
	}
	finish_output(pose, s, zero_acc, zero_ang_acc);
}

void
ControllerFusion::finish_output(OutputPose &pose, const Sample &s, bool zero_acc, bool zero_ang_acc)
{
	// Final checks and publication (spec 03 4/3.18).
	const OrientationController &oc = rate_flag_oc();
	if (!finite(pose.T_WM.t) || !finite(pose.T_WM.q) || !finite(pose.v)) {
		if (last_good_.valid) {
			out_ = last_good_;
		}
		return;
	}
	if ((frame_head_.inverse() * pose.T_WM).t.norm() > kPlausible && last_good_.valid) {
		pose.T_WM.t = last_good_.T_WM.t;
		pose.v = last_good_.v;
	} else {
		last_good_ = pose;
		last_good_.valid = true;
	}
	// The strike passthrough and reset zero the smoothed acceleration and the OC rate vector
	// before this point (0x363b2c..0x363b44, 0x363c7c..0x363c98; used at 0x364fa4, 0x365088).
	Vec3 a_lin = pose.T_WM.q * (zero_acc ? Vec3::Zero() : oc.smoothed_accel()) + kGravityOut;
	pose.ang_acc = zero_ang_acc ? Vec3::Zero() : oc.angular_acceleration();
	if (pose.status == OutputStatus::kStill) {
		pose.ang_acc.setZero();
		if (a_lin.norm() < kStillLinAcc) {
			a_lin.setZero();
		}
	}
	pose.a_lin = a_lin;
	pose.still = still3dof_;
	pose.valid = true;
	(void)s;
	out_ = pose;
}

void
ControllerFusion::output_3dof(const Sample &s)
{
	// 3DoF-only output (spec 03 4/3.16).
	OutputPose pose;
	pose.t_ns = std::llround(s.t * 1e9);
	pose.T_WM = Pose(s.q3, Vec3::Zero());
	pose.omega = s.gyro - rate_flag_oc().bias();
	pose.is_3dof = true;
	pose.status = still3dof_ ? OutputStatus::kStill : OutputStatus::kFallback;
	stand_to_move(pose, s);
	pose.t_unpredicted_ns = pose.t_ns;
	pose.T_WM_unpredicted = pose.T_WM;
	finish_output(pose, s);
}

void
ControllerFusion::window_statistics(const std::vector<Sample> &samples)
{
	// predictControllerPose pre-scan (spec 03 6/2, 5/4.6) with the state at the frame start.
	const bool was_restricted = ssf_.restricted;
	ssf_.restricted = false;
	double max_w = 0.0, max_a = 0.0;
	Vec3 sw = Vec3::Zero(), sw2 = Vec3::Zero(), sa = Vec3::Zero(), sa2 = Vec3::Zero();
	for (const Sample &s : samples) {
		const Vec3 wr = s.gyro - ssf_.bg;
		const Vec3 ar = ssf_.q * (s.accel - ssf_.ba) - kGravityImu;
		max_w = std::max(max_w, wr.norm());
		max_a = std::max(max_a, ar.norm());
		const bool acc_sat = (s.accel.cwiseAbs().array() > 300.0).any();
		const bool gyro_sat = (s.gyro.cwiseAbs().array() > 60.0).any();
		if (acc_sat || gyro_sat) {
			ssf_.freeze = 8;
			ssf_.full_rotation_once = true;
		}
		if (s.strike) {
			ssf_.restricted = true;
		}
		sw += wr;
		sw2 += wr.cwiseProduct(wr);
		sa += ar;
		sa2 += ar.cwiseProduct(ar);
	}
	if (ssf_.restricted && !was_restricted) {
		ssf_.restrict_p = ssf_.p;
		ssf_.restrict_q = ssf_.q;
	}
	const double n = std::max<double>(1.0, double(samples.size()));
	const Vec3 mw = sw / n, ma = sa / n;
	const Vec3 vw = sw2 / n - mw.cwiseProduct(mw), va = sa2 / n - ma.cwiseProduct(ma);
	// The three "< 0.25" criteria are |v| per axis of the filter velocity at the frame start
	// (predictControllerPose 0x399238..0x39924c, 0x399340; scan before the propagation, 0x3993d8).
	const bool still = (mw.cwiseAbs().array() < 0.2).all() && (vw.array() < 0.0256).all() &&
	                   (ma.cwiseAbs().array() < 1.0).all() && (va.array() < 0.64).all() &&
	                   (ssf_.v.cwiseAbs().array() < 0.25).all();
	ssf_.moving = !still;
	ssf_.max_gyro = max_w;
	ssf_.max_lin_acc = max_a;
}

FramePrior
ControllerFusion::begin_frame(int64_t t_image_ns, const Pose &T_WH)
{
	const double t = double(t_image_ns) * 1e-9;
	const double t_prev_frame = t_last_image_;
	const double dt = t - t_prev_frame;
	// Frame interval class (spec 03 6/1): the loss timer counts 40 or 33 ms per frame.
	if (dt > 0.031 && dt < 0.035) {
		frame_40ms_ = false;
	} else if (dt > 0.038 && dt < 0.042) {
		frame_40ms_ = true;
	}
	t_last_image_ = t;
	image_stop_latched_ = false;
	frame_head_ = T_WH;
	t_frame_ = t;

	// creatBody (spec 02 3.2): the samples t_prev <= ts < t.
	frame_samples_.clear();
	while (!frame_buf_.empty() && frame_buf_.front().t < t) {
		if (frame_buf_.front().t >= t_prev_frame) {
			frame_samples_.push_back(frame_buf_.front());
		}
		frame_buf_.pop_front();
	}

	FramePrior pr;
	pr.lost_frames = lost_;
	if (frame_samples_.empty()) {
		pr.has_body = false;
		pr.tracked = tracked_;
		return pr;
	}
	if (have_prev_ && prev_.t < t - 0.2) {
		reset_tracking();
	}
	pr.has_body = true;
	q3_frame_ = frame_samples_.back().q3;

	if (tracked_) {
		// predictControllerPose (spec 03 6/2): the filter lands exactly on the image time.
		window_statistics(frame_samples_);
		bool flag = false;
		for (const Sample &s : frame_samples_) {
			flag = flag || s.flagbit;
		}
		// The status bit of the window (P+0x1d70, predictControllerPose 0x398ef0 / 0x3990ac) only
		// inflates the process noise; the local strapdown needs P+0x1d26 and an anchor less than
		// 0.05 s old (0x399384..0x3993ac), anchored at the last accepted optical position.
		window_flag_ = flag;
		const bool local = local_valid_ && t - local_t_ < 0.05;
		Quat q_loc = ssf_.q;
		Vec3 v_loc = ssf_.v;
		Vec3 p_loc = local_p_;
		for (size_t i = 0; i < frame_samples_.size(); i++) {
			const Sample &s = frame_samples_[i];
			const double ti = i + 1 == frame_samples_.size() ? t : s.t;
			ssf_.propagate(ti, s.accel, s.gyro, flag, s.flagrest);
			if (local) {
				const double dtl = ti - local_t_;
				if (dtl > 1e-7 && dtl < 0.4) {
					q_loc = (q_loc * quat_exp(s.gyro * dtl)).normalized();
					const Vec3 dv = (q_loc * s.accel - kGravityImu) * dtl;
					p_loc += (v_loc + 0.5 * dv) * dtl;
					v_loc += dv;
					local_t_ = ti;
				}
			}
		}
		if (local) {
			local_p_ = p_loc;
		}
		pr.tracked = true;
		pr.local = local;
		pr.T_WM = local ? Pose(q_loc, p_loc) : Pose(ssf_.q, ssf_.p);
		frame_pose_ = pr.T_WM;
		pr.q_init = pr.T_WM.q;
		pr.moving = ssf_.moving;
		pr.v = ssf_.v;
		pr.sigma_p = ssf_.sigma_p();
		return pr;
	}

	// initControllerPose 0x39dd3c..0x39dd58: a window of 3 or more Bodies left from tracking is
	// cleared first, so re-acquisition needs 3 new frames; 0x39dc3c clears P+0x1d26 and P+0x1d70.
	if (window_.size() >= 3) {
		window_.clear();
	}
	local_valid_ = false;
	window_flag_ = false;

	// Rotation prior for initialisation (spec 02 7.1).
	if (!window_.empty()) {
		Quat q = window_.back().T_WM.q;
		double tp = window_.back().t;
		for (const Sample &s : frame_samples_) {
			q = (q * quat_exp((s.gyro - ssf_.bg) * (s.t - tp))).normalized();
			tp = s.t;
		}
		pr.q_init = (q * quat_exp((frame_samples_.back().gyro - ssf_.bg) * (t - tp))).normalized();
	} else {
		pr.q_init = q3_frame_;
	}
	if (complete_still_) {
		pr.q_init = filtered_pose_.q;
	}
	pr.T_WM = Pose(pr.q_init, filtered_pose_.t);
	return pr;
}

void
ControllerFusion::publish_filter_state(int state)
{
	optical_.t = t_frame_;
	optical_.p = ssf_.p;
	optical_.v = ssf_.v;
	optical_.q = ssf_.q;
	optical_.state = state;
	optical_new_ = true;
	optical_state_ = state;
	filtered_pose_ = Pose(ssf_.q, ssf_.p);
}

OpticalResult
ControllerFusion::report_optical(const OpticalReport &r)
{
	OpticalResult res;
	if (!tracked_) {
		return res;
	}
	near_ = r.near;
	lock_198c_ = true;
	yaw_pending_ = false;
	frame_pose_ = r.T_WM;

	GateInput in;
	in.hand = hand_;
	in.method = r.method;
	in.tracking_ok = r.tracking_ok;
	in.residual_count = r.residual_count;
	in.residual_mean = r.residual_mean;
	in.near = r.near;
	in.T_WH = frame_head_;
	in.filter_pose = Pose(ssf_.q, ssf_.p);
	in.p_optical = r.T_WM.t;
	in.chi2 = ssf_.chi_square(r.T_WM.t, r.T_WM.q);
	in.consecutive_rejects = ssf_.rejects;
	in.lost_frames = lost_;
	in.lock_was_set = hq_lock_;
	const GateResult g = fusion_gate(in);
	res.chi2 = in.chi2;
	res.dp = g.dp;
	hq_lock_ = false;

	if (g.force_reset) {
		reset_tracking();
		record_anchor();
		res.reset = true;
		return res;
	}
	if (g.accept) {
		if (ssf_.freeze > 0) {
			ssf_.freeze--;
		}
		ssf_.optical_update();
		ssf_.update_mode();
		ssf_.rejects = 0;
		hq_lock_ = g.lock;
		lost_ = 0;
		ramp_ = std::min(ramp_ + 1, 1000);
		yaw_pending_ = r.method == OpticalMethod::kTracking && r.tracking_ok;
		local_valid_ = true;
		local_t_ = t_frame_;
		local_p_ = r.T_WM.t;
		near_head_flag_ = false; // updatePose 0x3b61a0
		face_frozen_ = false;
		zupt_hold_ = false;
		publish_filter_state(1);
		push_body(Pose(ssf_.q, ssf_.p));
		record_anchor();
		res.accepted = true;
		return res;
	}
	ssf_.rejects++;
	// The reject tail clears P+0x1d26 (updatePose 0x3b6150..0x3b6168): no local position handover.
	local_valid_ = false;
	if (r.matched_leds >= 6) {
		// UNCERTAIN (spec 02 5.1): stock applies this in the branch of updatePose that clears the
		// window and the lost counter instead of predicting (0x3b311c..0x3b45d4, entered when the
		// Body flag at Body+0x230 -> +0x18 is clear); which rejected frames take it is not traced.
		// The rotation is the solution against the 3DoF attitude of the frame, applied to the
		// current 3DoF attitude: q3 <- (q_opt (x) q3_frame^-1) (x) q3 (0x3b3184..0x3b4550).
		rate_flag_oc().rotate(r.T_WM.q.normalized() * q3_frame_.conjugate());
	}
	if (!predict_if_lost(r.near, r.occlusion_rate)) {
		reset_tracking();
		res.reset = true;
	}
	record_anchor();
	return res;
}

bool
ControllerFusion::report_no_optical(const NoOpticalReport &r)
{
	if (!tracked_) {
		return false;
	}
	near_ = r.near;
	lock_198c_ = false;
	yaw_pending_ = false;
	hq_lock_ = false;
	const bool keep = predict_if_lost(r.near, r.occlusion_rate);
	if (r.force_lost || !keep) {
		reset_tracking();
		record_anchor();
		return false;
	}
	record_anchor();
	return true;
}

void
ControllerFusion::record_anchor()
{
	// End of every updatePose of a tracked hand (0x3b55ec..0x3b5634): while the other hand is
	// tracked and this one is lost for fewer than P+0x1984 frames, the offset of this hand's frame
	// pose (P+0x970) from the other's and the other's frame attitude (other P+0xa90 := P+0x950)
	// are kept for predicIfLost. P+0x1984 has no constant writer; 3 as in the follow rule.
	if (other_ != nullptr && other_->tracked_ && lost_ < 3) {
		anchor_offset_ = frame_pose_.t - other_->frame_pose_.t;
		anchor_q_other_ = other_->frame_pose_.q;
		anchor_valid_ = true;
	}
}

bool
ControllerFusion::predict_if_lost(bool near, double occlusion_rate)
{
	// predicIfLost, phoenix path (spec 03 6/4).
	(void)near;
	if (lost_ == 0 && std::abs(t_frame_ - local_t_) < 0.01 && local_valid_) {
		ssf_.p = local_p_;
	}
	if (window_flag_) {
		near_head_flag_ = true; // 0x3b9afc..0x3b9b08
	}
	if (strike_) {
		strike_ = false;
		strike_count_ = 0;
		return false;
	}
	follow_ = other_ != nullptr && other_->tracked_ && occlusion_rate >= 0.4 && other_->lost_ < 3;
	if (follow_) {
		// The reject counter is cleared only while following the other hand (0x3b96a8).
		ssf_.rejects = 0;
	}
	if (follow_ && lost_ >= 4 && anchor_valid_) {
		// The other hand's frame pose (P+0x950/0x970) carries the offset (0x3b96b8..0x3b9828).
		const Quat q_rel = other_->frame_pose_.q * anchor_q_other_.conjugate();
		const Vec3 p_pred = other_->frame_pose_.t + q_rel * anchor_offset_;
		if (ssf_.occlusion_update(p_pred) > 9.0) {
			return false;
		}
	}
	const Pose T(ssf_.q, ssf_.p);
	frame_pose_ = T; // P+0x950 (0x3b98d8..0x3b9900)
	const Pose T_hm = frame_head_.inverse() * T;
	const Vec3 &h = T_hm.t;
	const bool inbox = in_box(h, -0.28, 0.1, -0.3, 0.4, -0.2, 0.4);
	const bool near_face = in_box(h, -0.08, 0.05, -0.1, 0.2, -0.05, 0.12);
	if (near_face && hold_off_ >= 1 && !face_frozen_) {
		face_frozen_ = true;
		face_pose_H_ = T_hm;
	}
	const bool moving = ssf_.moving;
	if (!follow_) {
		if (!inbox && !moving) {
			ssf_.zupt();
			zupt_hold_ = false;
		}
		if (inbox && !moving) {
			if (!zupt_hold_) {
				zupt_hold_ = true;
				zupt_hold_p_ = ssf_.p;
				ssf_.v.setZero();
			}
			ssf_.zupt();
			ssf_.p = zupt_hold_p_;
		}
	}
	ramp_ = std::min(ramp_ + 1, 1000);
	if (ssf_.restricted) {
		if ((ssf_.p - ssf_.restrict_p).norm() > 0.02) {
			ssf_.p = ssf_.restrict_p;
		}
		if (angle_between(ssf_.restrict_q, ssf_.q) > 0.04) {
			ssf_.q = ssf_.restrict_q;
		}
		ssf_.v.setZero();
	}
	publish_filter_state(2);
	push_body(Pose(ssf_.q, ssf_.p));
	if (near_face && face_frozen_) {
		optical_.p = (frame_head_ * face_pose_H_).t;
	}

	double sigma_max, t_max;
	if (inbox) {
		// 0.01 m after a status-bit window while lost (P+0x1d71), else 0.02 m (0x3ba714..0x3ba72c).
		sigma_max = near_head_flag_ ? 0.01 : 0.02;
		t_max = 2.0;
	} else {
		sigma_max = moving ? 0.03 : 0.012;
		t_max = moving ? 3.0 : 1.2;
	}
	const Vec3 var = ssf_.P.diagonal().head<3>();
	if ((var.array() > sigma_max * sigma_max).any()) {
		return false;
	}
	const double period = frame_40ms_ ? 0.04 : 0.033;
	if (period * lost_ >= t_max && !follow_) {
		return false;
	}
	// While following from the fourth lost frame on, the counter stays (0x3ba6d0..0x3ba6f8).
	if (!(follow_ && lost_ >= 4)) {
		lost_++;
	}
	return true;
}

void
ControllerFusion::push_body(const Pose &T_WM)
{
	// The sliding window of Bodies (slideWindowCalculate, spec 03 6/5): kept while tracked too.
	Body b;
	b.t = t_frame_;
	b.T_WM = T_WM;
	b.samples = frame_samples_;
	window_.push_back(b);
	while (window_.size() > 3) {
		window_.erase(window_.begin());
	}
}

bool
ControllerFusion::report_init(const Pose &T_WM)
{
	if (tracked_) {
		return true;
	}
	if (frame_samples_.empty()) {
		window_.clear();
		return false;
	}
	push_body(T_WM);
	ramp_ = std::min(ramp_ + 1, 1000);
	filtered_pose_ = T_WM;

	if (window_.size() >= 2) {
		// Window velocity of the last pair (slideWindowCalculate, spec 03 6/5).
		Body &b0 = window_[window_.size() - 2];
		Body &b1 = window_.back();
		const double dt = b1.t - b0.t;
		if (dt > 0.0) {
			Vec3 v = (b1.T_WM.t - b0.T_WM.t) / dt;
			if (v.norm() > 5.0) {
				v *= 5.0 / v.norm();
			}
			Quat dR = Quat::Identity();
			Vec3 dv_pre = Vec3::Zero();
			double tp = b0.t;
			for (size_t i = 0; i < b1.samples.size(); i++) {
				const Sample &s = b1.samples[i];
				const double ti = i + 1 == b1.samples.size() ? b1.t : s.t;
				const double h = ti - tp;
				if (h <= 0.0) {
					continue;
				}
				dv_pre += dR * (s.accel - ssf_.ba) * h;
				dR = (dR * quat_exp((s.gyro - ssf_.bg) * h)).normalized();
				tp = ti;
			}
			const Quat q_mid = b0.T_WM.q.slerp(0.5, b1.T_WM.q);
			const Vec3 dv = q_mid * dv_pre - kGravityImu * dt;
			b0.v = v - 0.5 * dv;
			b1.v = v + 0.5 * dv;
		}
	}
	if (window_.size() < 3) {
		return false;
	}

	// Filter initialisation from the confirmation window (spec 02 7.1, spec 03 5/4.2).
	const Body &b0 = window_[0];
	ssf_.init(b0.t, b0.T_WM.t, b0.T_WM.q, b0.v, b0.samples.back().accel, b0.samples.back().gyro);
	ssf_.freeze = 8;
	for (size_t k = 1; k < window_.size(); k++) {
		const Body &bk = window_[k];
		window_statistics(bk.samples);
		bool flag = false;
		for (const Sample &s : bk.samples) {
			flag = flag || s.flagbit;
		}
		for (size_t i = 0; i < bk.samples.size(); i++) {
			const Sample &s = bk.samples[i];
			ssf_.propagate(i + 1 == bk.samples.size() ? bk.t : s.t, s.accel, s.gyro, flag, s.flagrest);
		}
		ssf_.chi_square(bk.T_WM.t, bk.T_WM.q);
		ssf_.optical_update();
	}
	tracked_ = true;
	lost_ = 0;
	ssf_.rejects = 0;
	lock_198c_ = true;
	hq_lock_ = false;
	local_valid_ = false;
	near_head_flag_ = false; // initControllerPose 0x3a1fc0, after Init
	publish_filter_state(1);
	return true;
}

void
ControllerFusion::report_init_failure()
{
	window_.clear();
}

void
ControllerFusion::reset_tracking()
{
	tracked_ = false;
	optical_state_ = 0;
	optical_new_ = false;
	lock_198c_ = false;
	yaw_pending_ = false;
	hq_lock_ = false;
	ramp_ = 0;
	hfkf_.reset();
	acc_out_count_ = 0;
	zupt_hold_ = false;
	ssf_.rejects = 0;
	local_valid_ = false;
	face_frozen_ = false;
	follow_ = false;
	// The window of Bodies stays (resetTracking 0x3671b4..0x367238 only re-bases each Body's pose
	// on its own stored pose, which is the one pose a Body holds here); the next init clears it
	// (begin_frame). The anchor offset P+0x1990 is not touched.
}

OutputPose
ControllerFusion::pose_at(int64_t t_ns) const
{
	return predict_output(out_, t_ns);
}

OutputPose
predict_output(OutputPose o, int64_t t_ns)
{
	if (!o.valid) {
		return o;
	}
	const double dt = double(t_ns - o.t_ns) * 1e-9;
	if (dt > 1.0) {
		o.t_ns = t_ns;
		return o;
	}
	const double wn = o.omega.norm();
	if (wn > 0.001) {
		o.T_WM.q = (o.T_WM.q * quat_exp(o.omega * dt)).normalized();
	}
	o.T_WM.t += o.v * dt;
	o.t_ns = t_ns;
	return o;
}

} // namespace pico4::ctrack
