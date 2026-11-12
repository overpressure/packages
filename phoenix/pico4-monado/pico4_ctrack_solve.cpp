// Copyright 2026, overpressure contributors.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Pose solvers and LED association (see pico4_ctrack_solve.hpp).
 * @ingroup drv_pico4
 */

#include "pico4_ctrack_solve.hpp"

#include <Eigen/Dense>
#include <Eigen/Eigenvalues>

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>

namespace pico4::ctrack {

/*
 *
 * Translation with a known rotation.
 *
 */

bool
translation_p2p(const Mat3 &R_WM, const RayObservation &a, const RayObservation &b, Vec3 &t)
{
	// t - lambda_k d_k = c_k - R P_k, unknowns (t, lambda_a, lambda_b).
	Eigen::Matrix<double, 6, 5> A = Eigen::Matrix<double, 6, 5>::Zero();
	Eigen::Matrix<double, 6, 1> rhs;
	const RayObservation *o[2] = {&a, &b};
	Vec3 d[2];
	for (int k = 0; k < 2; k++) {
		d[k] = o[k]->R_WC * o[k]->f;
		A.block<3, 3>(3 * k, 0) = Mat3::Identity();
		A.block<3, 1>(3 * k, 3 + k) = -d[k];
		rhs.segment<3>(3 * k) = o[k]->c - R_WM * o[k]->P;
	}
	Eigen::Matrix<double, 5, 1> x = (A.transpose() * A).ldlt().solve(A.transpose() * rhs);
	if (!x.allFinite() || x(3) < 0.005 || x(4) < 0.005) {
		return false;
	}
	t = Vec3::Zero();
	for (int k = 0; k < 2; k++) {
		t += o[k]->c + x(3 + k) * d[k] - R_WM * o[k]->P;
	}
	t /= 2.0;
	return true;
}

bool
translation_p3(const Mat3 &R_WM, const RayObservation obs[3], Vec3 &t)
{
	// lambda_a d_a - lambda_k d_k = c_k - c_a - R (P_k - P_a) for k = b, c: t eliminated.
	Vec3 d[3];
	for (int k = 0; k < 3; k++) {
		d[k] = obs[k].R_WC * obs[k].f;
	}
	Eigen::Matrix<double, 6, 3> A = Eigen::Matrix<double, 6, 3>::Zero();
	Eigen::Matrix<double, 6, 1> rhs;
	for (int k = 1; k < 3; k++) {
		A.block<3, 1>(3 * (k - 1), 0) = d[0];
		A.block<3, 1>(3 * (k - 1), k) = -d[k];
		rhs.segment<3>(3 * (k - 1)) = obs[k].c - obs[0].c - R_WM * (obs[k].P - obs[0].P);
	}
	Vec3 lambda = (A.transpose() * A).ldlt().solve(A.transpose() * rhs);
	if (!lambda.allFinite() || lambda(0) < 0.003 || lambda(1) < 0.005 || lambda(2) < 0.005) {
		return false;
	}
	t = Vec3::Zero();
	for (int k = 0; k < 3; k++) {
		t += obs[k].c + lambda(k) * d[k] - R_WM * obs[k].P;
	}
	t /= 3.0;
	return true;
}


/*
 *
 * P3P (Grunert) and absolute orientation.
 *
 */

namespace {

Pose
kabsch(const Vec3 src[3], const Vec3 dst[3])
{
	Vec3 cs = (src[0] + src[1] + src[2]) / 3.0, cd = (dst[0] + dst[1] + dst[2]) / 3.0;
	Mat3 H = Mat3::Zero();
	for (int i = 0; i < 3; i++) {
		H += (src[i] - cs) * (dst[i] - cd).transpose();
	}
	Eigen::JacobiSVD<Mat3> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
	Mat3 D = Mat3::Identity();
	D(2, 2) = (svd.matrixV() * svd.matrixU().transpose()).determinant() < 0 ? -1.0 : 1.0;
	Mat3 R = svd.matrixV() * D * svd.matrixU().transpose();
	return Pose(Quat(R), cd - R * cs);
}

std::vector<double>
real_roots(const double c[5])
{
	// Companion matrix of c[0] x^4 + ... + c[4].
	std::vector<double> out;
	if (std::abs(c[0]) < 1e-14) {
		return out;
	}
	Eigen::Matrix4d C = Eigen::Matrix4d::Zero();
	for (int i = 0; i < 4; i++) {
		C(0, i) = -c[i + 1] / c[0];
	}
	C(1, 0) = C(2, 1) = C(3, 2) = 1.0;
	Eigen::EigenSolver<Eigen::Matrix4d> es(C, false);
	for (int i = 0; i < 4; i++) {
		std::complex<double> r = es.eigenvalues()(i);
		// A double root comes out as a pair with a small imaginary part; keep its real part.
		if (std::abs(r.imag()) > 1e-4 * (1.0 + std::abs(r.real()))) {
			continue;
		}
		// Polish with Newton steps on the quartic itself: the eigenvalues carry the rounding of
		// the companion matrix (and differ with the compiler's contraction of multiply-adds).
		double x = r.real();
		for (int k = 0; k < 4; k++) {
			double p = c[0], d = 0.0;
			for (int j = 1; j < 5; j++) {
				d = d * x + p;
				p = p * x + c[j];
			}
			if (std::abs(d) < 1e-300) {
				break;
			}
			x -= p / d;
		}
		out.push_back(x);
	}
	return out;
}

} // namespace

std::vector<Pose>
p3p(const Vec3 f[3], const Vec3 P[3])
{
	std::vector<Pose> out;
	double a2 = (P[1] - P[2]).squaredNorm(), b2 = (P[0] - P[2]).squaredNorm(), c2 = (P[0] - P[1]).squaredNorm();
	if (b2 < 1e-18) {
		return out;
	}
	double ca = f[1].dot(f[2]), cb = f[0].dot(f[2]), cg = f[0].dot(f[1]);
	double q = (a2 - c2) / b2, r = (a2 + c2) / b2;
	double coeff[5] = {
	    (q - 1) * (q - 1) - 4 * c2 / b2 * ca * ca,
	    4 * (q * (1 - q) * cb - (1 - r) * ca * cg + 2 * c2 / b2 * ca * ca * cb),
	    2 * (q * q - 1 + 2 * q * q * cb * cb + 2 * (b2 - c2) / b2 * ca * ca - 4 * r * ca * cb * cg +
	         2 * (b2 - a2) / b2 * cg * cg),
	    4 * (-q * (1 + q) * cb + 2 * a2 / b2 * cg * cg * cb - (1 - r) * ca * cg),
	    (1 + q) * (1 + q) - 4 * a2 / b2 * cg * cg,
	};
	for (double v : real_roots(coeff)) {
		if (v <= 0) {
			continue;
		}
		double den = 2 * (cg - v * ca);
		if (std::abs(den) < 1e-12) {
			continue;
		}
		double u = ((q - 1) * v * v - 2 * q * cb * v + 1 + q) / den;
		double s1sq = b2 / (1 + v * v - 2 * v * cb);
		if (u <= 0 || s1sq <= 0) {
			continue;
		}
		double s1 = std::sqrt(s1sq);
		Vec3 cam[3] = {s1 * f[0], u * s1 * f[1], v * s1 * f[2]};
		out.push_back(kabsch(P, cam));
	}
	return out;
}


/*
 *
 * EPnP (Lepetit, Moreno-Noguer, Fua) with K = identity on normalised image points.
 *
 */

namespace {

struct Epnp
{
	int n = 0;
	std::vector<Vec3> P;
	std::vector<Vec2> uv;
	Vec3 cw[4];                        //!< control points, model frame
	std::vector<Eigen::Vector4d> alpha; //!< barycentric coordinates

	void
	choose_control_points()
	{
		cw[0] = Vec3::Zero();
		for (const Vec3 &p : P) {
			cw[0] += p;
		}
		cw[0] /= n;
		Eigen::MatrixXd A(n, 3);
		for (int i = 0; i < n; i++) {
			A.row(i) = (P[i] - cw[0]).transpose();
		}
		Eigen::SelfAdjointEigenSolver<Mat3> es(A.transpose() * A);
		for (int k = 0; k < 3; k++) {
			double s = std::sqrt(std::max(es.eigenvalues()(2 - k), 0.0) / n);
			cw[k + 1] = cw[0] + s * es.eigenvectors().col(2 - k);
		}
	}

	void
	compute_alphas()
	{
		Mat3 C;
		for (int k = 0; k < 3; k++) {
			C.col(k) = cw[k + 1] - cw[0];
		}
		Mat3 Ci = C.inverse();
		alpha.resize(n);
		for (int i = 0; i < n; i++) {
			Vec3 a = Ci * (P[i] - cw[0]);
			alpha[i] = Eigen::Vector4d(1.0 - a.sum(), a(0), a(1), a(2));
		}
	}

	Eigen::MatrixXd
	m_matrix() const
	{
		Eigen::MatrixXd M = Eigen::MatrixXd::Zero(2 * n, 12);
		for (int i = 0; i < n; i++) {
			for (int j = 0; j < 4; j++) {
				M(2 * i, 3 * j) = alpha[i](j);
				M(2 * i, 3 * j + 2) = -alpha[i](j) * uv[i].x();
				M(2 * i + 1, 3 * j + 1) = alpha[i](j);
				M(2 * i + 1, 3 * j + 2) = -alpha[i](j) * uv[i].y();
			}
		}
		return M;
	}

	//! Camera-frame control points from betas and the null-space vectors (columns of V, 12 x 4).
	void
	control_points(const Eigen::Matrix<double, 12, 4> &V, const Eigen::Vector4d &beta, Vec3 cc[4]) const
	{
		Eigen::Matrix<double, 12, 1> x = V * beta;
		for (int j = 0; j < 4; j++) {
			cc[j] = x.segment<3>(3 * j);
		}
	}

	double
	pose_from(const Vec3 cc[4], Pose &out) const
	{
		std::vector<Vec3> pc(n);
		for (int i = 0; i < n; i++) {
			pc[i] = Vec3::Zero();
			for (int j = 0; j < 4; j++) {
				pc[i] += alpha[i](j) * cc[j];
			}
		}
		// Points behind the camera: flip the solution.
		if (pc[0].z() < 0) {
			for (Vec3 &p : pc) {
				p = -p;
			}
		}
		Vec3 m_w = Vec3::Zero(), m_c = Vec3::Zero();
		for (int i = 0; i < n; i++) {
			m_w += P[i];
			m_c += pc[i];
		}
		m_w /= n;
		m_c /= n;
		Mat3 H = Mat3::Zero();
		for (int i = 0; i < n; i++) {
			H += (pc[i] - m_c) * (P[i] - m_w).transpose();
		}
		Eigen::JacobiSVD<Mat3> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
		Mat3 R = svd.matrixU() * svd.matrixV().transpose();
		if (R.determinant() < 0) {
			Mat3 U = svd.matrixU();
			U.col(2) *= -1;
			R = U * svd.matrixV().transpose();
		}
		Vec3 t = m_c - R * m_w;
		out = Pose(Quat(R), t);
		double err = 0;
		for (int i = 0; i < n; i++) {
			Vec3 q = R * P[i] + t;
			err += (Vec2(q.x() / q.z(), q.y() / q.z()) - uv[i]).norm();
		}
		return err / n;
	}
};

//! The six squared control-point distances as linear functions of the beta products (L_6x10).
void
l6x10(const Eigen::Matrix<double, 12, 4> &V, Eigen::Matrix<double, 6, 10> &L)
{
	static const int pairs[6][2] = {{0, 1}, {0, 2}, {0, 3}, {1, 2}, {1, 3}, {2, 3}};
	for (int i = 0; i < 6; i++) {
		int a = pairs[i][0], b = pairs[i][1];
		Vec3 dv[4];
		for (int k = 0; k < 4; k++) {
			dv[k] = V.block<3, 1>(3 * a, 3 - k) - V.block<3, 1>(3 * b, 3 - k);
		}
		// beta order: b11 b12 b22 b13 b23 b33 b14 b24 b34 b44 (vectors k = 0..3 = smallest first).
		L(i, 0) = dv[0].dot(dv[0]);
		L(i, 1) = 2 * dv[0].dot(dv[1]);
		L(i, 2) = dv[1].dot(dv[1]);
		L(i, 3) = 2 * dv[0].dot(dv[2]);
		L(i, 4) = 2 * dv[1].dot(dv[2]);
		L(i, 5) = dv[2].dot(dv[2]);
		L(i, 6) = 2 * dv[0].dot(dv[3]);
		L(i, 7) = 2 * dv[1].dot(dv[3]);
		L(i, 8) = 2 * dv[2].dot(dv[3]);
		L(i, 9) = dv[3].dot(dv[3]);
	}
}

void
gauss_newton_betas(const Eigen::Matrix<double, 6, 10> &L, const Eigen::Matrix<double, 6, 1> &rho,
                   Eigen::Vector4d &b)
{
	for (int it = 0; it < 5; it++) {
		Eigen::Matrix<double, 6, 4> J;
		Eigen::Matrix<double, 6, 1> r;
		for (int i = 0; i < 6; i++) {
			const auto &l = L.row(i);
			J(i, 0) = 2 * l(0) * b(0) + l(1) * b(1) + l(3) * b(2) + l(6) * b(3);
			J(i, 1) = l(1) * b(0) + 2 * l(2) * b(1) + l(4) * b(2) + l(7) * b(3);
			J(i, 2) = l(3) * b(0) + l(4) * b(1) + 2 * l(5) * b(2) + l(8) * b(3);
			J(i, 3) = l(6) * b(0) + l(7) * b(1) + l(8) * b(2) + 2 * l(9) * b(3);
			r(i) = rho(i) - (l(0) * b(0) * b(0) + l(1) * b(0) * b(1) + l(2) * b(1) * b(1) + l(3) * b(0) * b(2) +
			                 l(4) * b(1) * b(2) + l(5) * b(2) * b(2) + l(6) * b(0) * b(3) + l(7) * b(1) * b(3) +
			                 l(8) * b(2) * b(3) + l(9) * b(3) * b(3));
		}
		Eigen::Vector4d step = J.colPivHouseholderQr().solve(r);
		if (!step.allFinite()) {
			return;
		}
		b += step;
	}
}

} // namespace

bool
epnp(const std::vector<Vec2> &uv, const std::vector<Vec3> &P, Pose &out)
{
	Epnp e;
	e.n = (int)P.size();
	if (e.n < 4 || uv.size() != P.size()) {
		return false;
	}
	e.P = P;
	e.uv = uv;
	e.choose_control_points();
	e.compute_alphas();
	Eigen::MatrixXd M = e.m_matrix();
	Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 12, 12>> es(M.transpose() * M);
	// Columns 0..3 of V: the eigenvectors of the four smallest eigenvalues, largest of those first.
	Eigen::Matrix<double, 12, 4> V;
	for (int k = 0; k < 4; k++) {
		V.col(k) = es.eigenvectors().col(3 - k);
	}
	Eigen::Matrix<double, 6, 10> L;
	l6x10(V, L);
	Eigen::Matrix<double, 6, 1> rho;
	static const int pairs[6][2] = {{0, 1}, {0, 2}, {0, 3}, {1, 2}, {1, 3}, {2, 3}};
	for (int i = 0; i < 6; i++) {
		rho(i) = (e.cw[pairs[i][0]] - e.cw[pairs[i][1]]).squaredNorm();
	}

	double best = std::numeric_limits<double>::infinity();
	auto attempt = [&](Eigen::Vector4d b) {
		gauss_newton_betas(L, rho, b);
		Eigen::Vector4d beta(b(3), b(2), b(1), b(0)); // V columns are ordered largest of the four first
		Vec3 cc[4];
		e.control_points(V, beta, cc);
		Pose p;
		double err = e.pose_from(cc, p);
		if (std::isfinite(err) && err < best) {
			best = err;
			out = p;
		}
	};
	// Approximation 1: b11 b12 b13 b14.
	{
		Eigen::Matrix<double, 6, 4> A;
		A << L.col(0), L.col(1), L.col(3), L.col(6);
		Eigen::Vector4d x = A.colPivHouseholderQr().solve(rho);
		Eigen::Vector4d b = Eigen::Vector4d::Zero();
		if (x(0) < 0) {
			b << std::sqrt(-x(0)), -x(1) / std::sqrt(-x(0)), -x(2) / std::sqrt(-x(0)), -x(3) / std::sqrt(-x(0));
		} else {
			b << std::sqrt(x(0)), x(1) / std::sqrt(x(0)), x(2) / std::sqrt(x(0)), x(3) / std::sqrt(x(0));
		}
		attempt(b);
	}
	// Approximation 2: b11 b12 b22.
	{
		Eigen::Matrix<double, 6, 3> A;
		A << L.col(0), L.col(1), L.col(2);
		Vec3 x = A.colPivHouseholderQr().solve(rho);
		Eigen::Vector4d b = Eigen::Vector4d::Zero();
		if (x(0) < 0) {
			b(0) = std::sqrt(-x(0));
			b(1) = x(2) < 0 ? std::sqrt(-x(2)) : 0.0;
		} else {
			b(0) = std::sqrt(x(0));
			b(1) = x(2) > 0 ? std::sqrt(x(2)) : 0.0;
		}
		if (x(1) < 0) {
			b(0) = -b(0);
		}
		attempt(b);
	}
	// Approximation 3: b11 b12 b22 b13 b23.
	{
		Eigen::Matrix<double, 6, 5> A;
		A << L.col(0), L.col(1), L.col(2), L.col(3), L.col(4);
		Eigen::Matrix<double, 5, 1> x = A.colPivHouseholderQr().solve(rho);
		Eigen::Vector4d b = Eigen::Vector4d::Zero();
		if (x(0) < 0) {
			b(0) = std::sqrt(-x(0));
			b(1) = x(2) < 0 ? std::sqrt(-x(2)) : 0.0;
		} else {
			b(0) = std::sqrt(x(0));
			b(1) = x(2) > 0 ? std::sqrt(x(2)) : 0.0;
		}
		if (x(1) < 0) {
			b(0) = -b(0);
		}
		b(2) = std::abs(b(0)) > 1e-12 ? x(3) / b(0) : 0.0;
		attempt(b);
	}
	return std::isfinite(best);
}


/*
 *
 * Angles.
 *
 */

double
tilt_half_angle_deg(const Quat &a, const Quat &b)
{
	Vec3 u1 = b.conjugate() * Vec3::UnitX(), u2 = a.conjugate() * Vec3::UnitX();
	Quat s = Quat::FromTwoVectors(u1, u2);
	return std::acos(std::clamp(std::abs(s.w()), 0.0, 1.0)) * 57.296;
}

double
half_angle_deg(const Quat &a, const Quat &b)
{
	Quat r = a.conjugate() * b;
	double w = r.w();
	double ang = w <= -1.0 ? 180.0007 : std::acos(std::clamp(w, -1.0, 1.0)) * 57.296;
	return ang >= 90.0 ? 180.0 - ang : ang;
}

double
full_angle_deg(const Quat &a, const Quat &b)
{
	Quat r = a.conjugate() * b;
	return 2.0 * std::atan2(r.vec().norm(), std::abs(r.w())) * 180.0 / M_PI;
}


/*
 *
 * Association (NewMatchPoints).
 *
 */

void
match_points(const std::vector<Vec2> &proj, const std::vector<Vec2> &blobs, std::vector<int> &match,
             std::vector<double> &err)
{
	size_t np = proj.size(), nb = blobs.size();
	match.assign(nb, -1);
	err.assign(nb, 100.0);
	if (np == 0 || nb == 0) {
		return;
	}
	// The gate from the projected LED spacing.
	double dmin = 100.0;
	for (size_t j = 0; j < np; j++) {
		double m = 100.0;
		for (size_t k = 0; k < np; k++) {
			if (k != j) {
				m = std::min(m, (proj[j] - proj[k]).norm());
			}
		}
		dmin = std::min(dmin, m);
	}
	double th = dmin < 30 ? 0.7 * dmin + 12 : 0.25 * dmin + 25;
	double g = 0.2 * th;
	std::vector<size_t> valid;
	for (size_t i = 0; i < nb; i++) {
		for (size_t j = 0; j < np; j++) {
			if ((blobs[i] - proj[j]).norm() < th) {
				valid.push_back(i);
				break;
			}
		}
	}
	size_t n = std::min(np, valid.size());
	auto finish = [&](const std::vector<std::pair<size_t, size_t>> &pairs) {
		for (auto [b, p] : pairs) {
			match[b] = (int)p;
			err[b] = (blobs[b] - proj[p]).norm();
		}
	};
	if (n == 0) {
		return;
	}
	if (n == 1) {
		// The single nearest pair, if clearly better than the runner-up.
		double best = 1e9, second = 1e9;
		std::pair<size_t, size_t> bp;
		for (size_t i : valid) {
			for (size_t j = 0; j < np; j++) {
				double d = (blobs[i] - proj[j]).norm();
				if (d < best) {
					second = best;
					best = d;
					bp = {i, j};
				} else if (d < second) {
					second = d;
				}
			}
		}
		if (best < 0.2 * th && second > 0.4 * th) {
			finish({bp});
		}
		return;
	}
	if (n == 2) {
		// The displacement between two blobs against the displacement between two projections.
		double best = 1e9, prev = 1e9;
		std::vector<std::pair<size_t, size_t>> bestp;
		for (size_t a = 0; a < valid.size(); a++) {
			for (size_t b = 0; b < valid.size(); b++) {
				if (a == b) {
					continue;
				}
				Vec2 db = blobs[valid[b]] - blobs[valid[a]];
				for (size_t p = 0; p < np; p++) {
					for (size_t q = 0; q < np; q++) {
						if (p == q) {
							continue;
						}
						double d = (db - (proj[q] - proj[p])).norm();
						if (d < best) {
							prev = best;
							best = d;
							bestp = {{valid[a], p}, {valid[b], q}};
						} else if (d < prev) {
							prev = d;
						}
					}
				}
			}
		}
		if (!bestp.empty() && best < 0.2 * th && prev > 0.4 * th &&
		    (blobs[bestp[0].first] - proj[bestp[0].second]).norm() < th) {
			finish(bestp);
		}
		return;
	}
	size_t K = n >= 5 ? (2 * n) / 3 : 3;
	double best_score = 10000.0;
	std::vector<std::pair<size_t, size_t>> best_pairs;
	bool done = false;
	for (size_t j = 0; j < np && !done; j++) {
		for (size_t i : valid) {
			if ((blobs[i] - proj[j]).norm() >= th) {
				continue;
			}
			Vec2 shift = blobs[i] - proj[j];
			std::multimap<double, std::pair<size_t, size_t>> dist;
			std::map<double, bool> seen; // equal distances are dropped, as stock's map does
			for (size_t b : valid) {
				for (size_t p = 0; p < np; p++) {
					double d = (blobs[b] - (proj[p] + shift)).norm();
					if (d < g && !seen.count(d)) {
						seen[d] = true;
						dist.emplace(d, std::make_pair(b, p));
					}
				}
			}
			if (dist.size() < K) {
				continue;
			}
			size_t c12 = 0, c06 = 0;
			for (auto &kv : dist) {
				c12 += kv.first < 0.12 * th;
				c06 += kv.first < 0.06 * th;
			}
			bool very_good = (n >= 5 && c12 >= n - 1) || (n >= 5 && c06 >= K) ||
			                 (n == 4 && (c12 == 4 || c06 > 2)) || (n == 3 && c06 == 3);
			// Greedy one-to-one assignment in increasing distance.
			std::vector<std::pair<size_t, size_t>> pairs;
			std::vector<bool> used_b(nb, false), used_p(np, false);
			double score = 0;
			size_t limit = very_good ? n : K;
			for (auto &kv : dist) {
				auto [b, p] = kv.second;
				if (used_b[b] || used_p[p]) {
					continue;
				}
				used_b[b] = used_p[p] = true;
				pairs.emplace_back(b, p);
				if (pairs.size() <= K) {
					score += kv.first;
				}
				if (pairs.size() >= limit) {
					break;
				}
			}
			if (very_good) {
				best_pairs = pairs;
				done = true;
				break;
			}
			if (pairs.size() >= K && score < best_score) {
				best_score = score;
				// Complete the assignment up to n with the remaining entries.
				for (auto &kv : dist) {
					if (pairs.size() >= n) {
						break;
					}
					auto [b, p] = kv.second;
					if (!used_b[b] && !used_p[p]) {
						used_b[b] = used_p[p] = true;
						pairs.emplace_back(b, p);
					}
				}
				best_pairs = pairs;
			}
		}
	}
	// A blob keeps its LED only if it lies within the inlier radius after the shift; stock
	// re-checks with the shifted distance, which is what entered the map.
	finish(best_pairs);
}


/*
 *
 * Refinement over all cameras (PoseOptimizer).
 *
 */

namespace {

Mat3
skew(const Vec3 &v)
{
	Mat3 m;
	m << 0, -v.z(), v.y(), v.z(), 0, -v.x(), -v.y(), v.x(), 0;
	return m;
}

Quat
exp_so3(const Vec3 &w)
{
	double th = w.norm();
	if (th < 1e-12) {
		return Quat(1, 0.5 * w.x(), 0.5 * w.y(), 0.5 * w.z()).normalized();
	}
	return Quat(Eigen::AngleAxisd(th, w / th));
}

Vec3
log_so3(const Quat &q)
{
	Eigen::AngleAxisd aa(q.normalized());
	double a = aa.angle();
	if (a > M_PI) {
		a -= 2 * M_PI;
	}
	return a * aa.axis();
}

//! Inverse right Jacobian of SO(3).
Mat3
jr_inv(const Vec3 &w)
{
	double th = w.norm();
	Mat3 W = skew(w);
	if (th < 1e-6) {
		return Mat3::Identity() + 0.5 * W;
	}
	return Mat3::Identity() + 0.5 * W +
	       (1.0 / (th * th) - (1.0 + std::cos(th)) / (2.0 * th * std::sin(th))) * W * W;
}

constexpr double kTukey = 4.6851;
constexpr double kPriorWeight = 0.1;

struct Residual
{
	Eigen::Vector2d r;
	Eigen::Matrix<double, 2, 6> J; //!< w.r.t. (dw in M, dt in H)
	double err_norm;               //!< normalised-plane error
};

//! Tangent-plane residual of one feature and its Jacobian.
Residual
residual(const Camera &cam, const Feature &f, const Pose &T_HM)
{
	Pose T_CH = cam.T_HC.inverse();
	Vec3 pM = f.P;
	Vec3 pH = T_HM * pM;
	Vec3 p = T_CH * pH;
	double pn = p.norm();
	Vec3 u = p / pn;
	Vec3 fb = f.bearing.normalized();
	Vec3 b1 = Vec3::UnitZ() - Vec3::UnitZ().dot(fb) * fb;
	if (b1.norm() < 1e-9) {
		b1 = Vec3::UnitX() - Vec3::UnitX().dot(fb) * fb;
	}
	b1.normalize();
	Vec3 b2 = fb.cross(b1);
	Eigen::Matrix<double, 2, 3> B;
	B.row(0) = b1.transpose();
	B.row(1) = b2.transpose();
	Residual out;
	out.r = B * (fb - u);
	// d(-u)/dp = -(I - u u^T) / |p|; dp/dpH = R_CH; dpH/d(dw) = -R_HM [P]x (right perturbation); dpH/dt = I.
	Mat3 du = -(Mat3::Identity() - u * u.transpose()) / pn;
	Mat3 R_CH = T_CH.q.toRotationMatrix(), R_HM = T_HM.q.toRotationMatrix();
	out.J.block<2, 3>(0, 0) = B * du * R_CH * (-R_HM * skew(pM));
	out.J.block<2, 3>(0, 3) = B * du * R_CH;
	Vec2 a, b;
	cam.project2d(f.bearing, a);
	cam.project2d(p, b);
	out.err_norm = (a - b).norm();
	return out;
}

} // namespace

bool
refine_pose(const std::array<Camera, kCameras> &cams, std::vector<Feature> &feats, const Pose &prior_HM,
            Pose &T_HM, double &median_initial_px, double &median_final_px)
{
	auto median = [](std::vector<double> v) {
		if (v.empty()) {
			return 0.0;
		}
		std::nth_element(v.begin(), v.begin() + v.size() / 2, v.end());
		return v[v.size() / 2];
	};
	std::vector<double> e0;
	for (const Feature &f : feats) {
		if (f.enabled) {
			e0.push_back(residual(cams[f.cam], f, T_HM).err_norm);
		}
	}
	if (e0.empty()) {
		return false;
	}
	double sigma0 = 1.48 * median(e0);
	median_initial_px = median(e0) * cams[0].error_multiplier();

	auto chi2_of = [&](const Pose &T, double sigma, bool with_prior, Eigen::Matrix<double, 6, 6> *H,
	                   Eigen::Matrix<double, 6, 1> *g) {
		double chi2 = 0;
		if (H) {
			H->setZero();
			g->setZero();
		}
		for (const Feature &f : feats) {
			if (!f.enabled) {
				continue;
			}
			Residual r = residual(cams[f.cam], f, T);
			double x = sigma > 0 ? r.err_norm / sigma : 0.0;
			double w = x <= kTukey ? std::pow(1.0 - x * x / (kTukey * kTukey), 2) : 0.0;
			chi2 += w * r.r.squaredNorm();
			if (H) {
				*H += w * r.J.transpose() * r.J;
				*g += w * r.J.transpose() * r.r;
			}
		}
		if (with_prior) {
			Vec3 e = log_so3(prior_HM.q.conjugate() * T.q);
			chi2 += kPriorWeight * e.squaredNorm();
			if (H) {
				Mat3 J = jr_inv(e);
				H->block<3, 3>(0, 0) += kPriorWeight * J.transpose() * J;
				g->segment<3>(0) += kPriorWeight * J.transpose() * e;
			}
		}
		return chi2;
	};

	// Stock quirk: the initial chi2 carries no prior term.
	double sigma = sigma0;
	Eigen::Matrix<double, 6, 6> H;
	Eigen::Matrix<double, 6, 1> g;
	double chi2 = chi2_of(T_HM, sigma, false, &H, &g);
	chi2_of(T_HM, sigma, true, &H, &g);
	double mu = 1e-4 * H.diagonal().maxCoeff(), nu = 2.0;
	for (int it = 0; it < 10; it++) {
		if (it == 5) {
			sigma = 0.85 / cams[0].error_multiplier();
		}
		bool accepted = false;
		for (int rej = 0; rej < 5 && !accepted; rej++) {
			Eigen::Matrix<double, 6, 6> Hd = H;
			for (int i = 0; i < 6; i++) {
				Hd(i, i) *= 1.0 + mu;
			}
			Eigen::Matrix<double, 6, 1> x = Hd.ldlt().solve(-g);
			if (!x.allFinite()) {
				return false; // "Matrix is close to singular"
			}
			Pose T_new(T_HM.q * exp_so3(x.segment<3>(0)), T_HM.t + x.segment<3>(3));
			Eigen::Matrix<double, 6, 6> Hn;
			Eigen::Matrix<double, 6, 1> gn;
			double chi2_new = chi2_of(T_new, sigma, true, &Hn, &gn);
			double rho = (chi2 - chi2_new) / (x.dot(mu * x - g) + 1e-6);
			if (rho > 0) {
				T_HM = T_new;
				chi2 = chi2_new;
				H = Hn;
				g = gn;
				mu *= std::clamp(1.0 - std::pow(2.0 * rho - 1.0, 3), 1.0 / 3.0, 2.0 / 3.0);
				nu = 2.0;
				accepted = true;
				if (x.cwiseAbs().maxCoeff() <= 1e-6) {
					it = 10;
				}
			} else {
				mu *= nu;
				nu *= 2.0;
			}
		}
		if (!accepted) {
			break;
		}
	}
	std::vector<double> e1;
	for (Feature &f : feats) {
		Residual r = residual(cams[f.cam], f, T_HM);
		f.residual_px = r.err_norm * cams[f.cam].error_multiplier();
		if (f.enabled) {
			e1.push_back(f.residual_px);
		}
	}
	median_final_px = median(e1);
	return true;
}


/*
 *
 * Translation with the prior rotation from few points (solvePnPWithPriorR_general_near).
 *
 */

bool
few_points_translation(const std::array<Camera, kCameras> &cams, const Pose &T_WH, const Quat &R_WM,
                       std::vector<Feature> &feats, bool near, Vec3 &t_WM, std::vector<double> &errors_px)
{
	// Observations: skip a camera with a single feature unless near.
	int per_cam[kCameras] = {0};
	for (const Feature &f : feats) {
		per_cam[f.cam]++;
	}
	std::vector<int> use;
	for (size_t i = 0; i < feats.size(); i++) {
		if ((per_cam[feats[i].cam] > 1 || near) && feats[i].residual_px <= 50.0) {
			use.push_back((int)i);
		}
	}
	auto obs_of = [&](int i) {
		const Camera &c = cams[feats[i].cam];
		Pose T_WC = T_WH * c.T_HC;
		return RayObservation{T_WC.q.toRotationMatrix(), T_WC.t, feats[i].bearing, feats[i].P};
	};
	Mat3 R = R_WM.toRotationMatrix();
	auto score = [&](const Vec3 &t, double gate, double inlier, int &inliers, double &mean) {
		inliers = 0;
		double sum = 0;
		for (int i : use) {
			const Camera &c = cams[feats[i].cam];
			Pose T_CM = (T_WH * c.T_HC).inverse() * Pose(R_WM, t);
			Vec2 a, b;
			c.project2d(feats[i].bearing, a);
			c.project2d(T_CM * feats[i].P, b);
			double e = c.error_multiplier() * (a - b).norm();
			if (e < gate && e < inlier) {
				inliers++;
				sum += e;
			}
		}
		mean = inliers ? sum / inliers : 1e9;
	};
	struct Best
	{
		int inliers = 0;
		double mean = 1e9;
		Vec3 t;
	} pair_best, triple_best;
	double inlier = near ? 10.0 : 1.0;
	for (size_t a = 0; a < use.size(); a++) {
		for (size_t b = a + 1; b < use.size(); b++) {
			Vec3 t;
			if (!translation_p2p(R, obs_of(use[a]), obs_of(use[b]), t)) {
				continue;
			}
			int n;
			double m;
			score(t, near ? 20.0 : 10.0, inlier, n, m);
			if (n > pair_best.inliers || (n == pair_best.inliers && m < pair_best.mean)) {
				pair_best = {n, m, t};
			}
			for (size_t c = b + 1; c < use.size(); c++) {
				RayObservation o[3] = {obs_of(use[a]), obs_of(use[b]), obs_of(use[c])};
				if (!translation_p3(R, o, t)) {
					continue;
				}
				score(t, near ? 100.0 : 10.0, inlier, n, m);
				if (n > triple_best.inliers || (n == triple_best.inliers && m < triple_best.mean)) {
					triple_best = {n, m, t};
				}
			}
		}
	}
	const Best *best = nullptr;
	if (pair_best.inliers > triple_best.inliers) {
		best = &pair_best;
	} else if (triple_best.inliers > pair_best.inliers) {
		best = &triple_best;
	} else {
		best = pair_best.mean <= triple_best.mean ? &pair_best : &triple_best;
	}
	if (best->inliers == 0) {
		return false;
	}
	t_WM = best->t;
	errors_px.clear();
	for (int i : use) {
		const Camera &c = cams[feats[i].cam];
		Pose T_CM = (T_WH * c.T_HC).inverse() * Pose(R_WM, t_WM);
		Vec2 a, b;
		c.project2d(feats[i].bearing, a);
		c.project2d(T_CM * feats[i].P, b);
		errors_px.push_back(c.error_multiplier() * (a - b).norm());
	}
	return true;
}

} // namespace pico4::ctrack
