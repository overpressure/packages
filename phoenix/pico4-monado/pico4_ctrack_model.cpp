// Copyright 2026, overpressure contributors.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Cameras, LED models, visibility and the identification table (see pico4_ctrack_model.hpp).
 * @ingroup drv_pico4
 */

#include "pico4_ctrack_model.hpp"

#include "pico4_ctrack_blob.hpp" // stock_sort

#include <algorithm>
#include <cmath>
#include <map>

namespace pico4::ctrack {

/*
 *
 * Camera.
 *
 */

Vec2
Camera::distort(const Vec2 &m) const
{
	double x = m.x(), y = m.y(), r2 = x * x + y * y;
	double rad = k1 * r2 + k2 * r2 * r2;
	return Vec2(x + x * rad + 2 * p1 * x * y + p2 * (r2 + 2 * x * x),
	            y + y * rad + p1 * (r2 + 2 * y * y) + 2 * p2 * x * y);
}

bool
Camera::project2d(const Vec3 &p, Vec2 &out) const
{
	double den = p.z() + xi * p.norm();
	if (!(den > 1e-12)) {
		out = Vec2(-1000.0, -1000.0);
		return false;
	}
	out = Vec2(p.x() / den, p.y() / den);
	return true;
}

bool
Camera::world2cam(const Vec3 &p, Vec2 &out) const
{
	Vec2 m;
	if (!project2d(p, m)) {
		out = m;
		return false;
	}
	Vec2 d = distort(m);
	out = Vec2(fu * d.x() + pu, fv * d.y() + pv);
	return true;
}

Vec3
Camera::cam2world(const Vec2 &px) const
{
	Vec2 target((px.x() - pu) / fu, (px.y() - pv) / fv), m = target;
	// Fixed-point undistortion; the distortion of these lenses is mild over the image.
	for (int i = 0; i < 20; i++) {
		m -= distort(m) - target;
	}
	double r2 = m.squaredNorm();
	double f = (xi + std::sqrt(1.0 + (1.0 - xi * xi) * r2)) / (1.0 + r2);
	return Vec3(f * m.x(), f * m.y(), f - xi).normalized();
}

double
Camera::error_multiplier() const
{
	return std::abs(fu);
}


/*
 *
 * Visibility.
 *
 */

std::vector<std::pair<int, double>>
visible_leds(const LedModel &model, const Quat &q_MC, const Vec3 &b)
{
	Vec3 c = -(q_MC * b);
	std::vector<std::pair<int, double>> out;
	for (int i = 0; i < kLeds; i++) {
		double cos_a = model.normal[i].dot((c - model.position[i]).normalized());
		if (cos_a >= 0.0) {
			out.emplace_back(i, cos_a);
		}
	}
	std::stable_sort(out.begin(), out.end(), [](const auto &a, const auto &b) { return a.second > b.second; });
	return out;
}


/*
 *
 * Identification table.
 *
 */

void
basis_descriptor(const Vec3 &f_a, const Vec3 &f_b, const Vec3 *others, int n, Vec2 *out)
{
	// The bisector of the basis bearings onto the optical axis, then their difference onto +x.
	Quat r1 = Quat::FromTwoVectors((f_a + f_b).normalized(), Vec3::UnitZ());
	Vec3 a = r1 * f_a, b = r1 * f_b;
	double angle = std::atan2(b.y() - a.y(), b.x() - a.x());
	Quat r = Quat(Eigen::AngleAxisd(-angle, Vec3::UnitZ())) * r1;
	auto plane = [&](const Vec3 &f) {
		Vec3 g = r * f;
		return Vec2(g.x() / g.z(), g.y() / g.z());
	};
	Vec2 pa = plane(f_a), pb = plane(f_b);
	double s = 2.0 / (pb - pa).norm();
	for (int i = 0; i < n; i++) {
		out[i] = s * plane(others[i]);
	}
}

namespace {

//! The 44 LED groups of five whose image patterns the table holds.
const int kGroups[44][5] = {
    {0, 1, 2, 3, 4},      {0, 1, 2, 3, 6},      {0, 1, 2, 3, 8},      {0, 1, 2, 10, 11},    {0, 1, 6, 7, 8},
    {0, 1, 10, 11, 12},   {0, 1, 11, 12, 13},   {0, 11, 12, 13, 14},  {0, 11, 12, 13, 15},  {0, 13, 14, 15, 17},
    {0, 14, 15, 16, 18},  {1, 2, 3, 4, 6},      {1, 2, 3, 6, 8},      {1, 2, 6, 8, 10},     {1, 8, 9, 10, 11},
    {1, 9, 10, 11, 12},   {3, 8, 9, 10, 11},    {3, 9, 10, 11, 12},   {5, 6, 7, 8, 9},      {5, 7, 9, 12, 14},
    {5, 8, 9, 10, 12},    {6, 7, 8, 9, 10},     {6, 7, 8, 9, 12},     {6, 8, 10, 11, 13},   {7, 8, 9, 10, 11},
    {7, 9, 12, 14, 16},   {8, 9, 10, 11, 12},   {8, 9, 10, 11, 13},   {8, 10, 11, 13, 15},  {9, 10, 11, 12, 13},
    {9, 12, 14, 16, 18},  {10, 11, 12, 13, 14}, {10, 11, 13, 15, 17}, {11, 12, 13, 14, 15}, {11, 12, 13, 14, 16},
    {11, 12, 14, 16, 18}, {11, 13, 15, 17, 19}, {12, 13, 14, 15, 16}, {12, 14, 15, 16, 18}, {12, 14, 16, 17, 18},
    {13, 14, 15, 16, 17}, {13, 14, 15, 16, 18}, {14, 15, 16, 17, 18}, {15, 16, 17, 18, 19},
};
constexpr int kViewpoints = 200;
constexpr double kViewRadius = 0.6;   //!< metres around the LED centroid
constexpr double kMatchRadius = 0.21; //!< per descriptor point, basis length 2

} // namespace

void
IdTable::generate(const LedModel &left)
{
	Vec3 centroid = Vec3::Zero();
	for (const Vec3 &p : left.position) {
		centroid += p;
	}
	centroid /= kLeds;
	std::vector<IdRecord> records;
	const double golden = M_PI * (3.0 - std::sqrt(5.0));
	for (int k = 0; k < kViewpoints; k++) {
		double y = 1.0 - 2.0 * k / (kViewpoints - 1);
		double r = std::sqrt(std::max(0.0, 1.0 - y * y)), phi = k * golden;
		Vec3 c = centroid + kViewRadius * Vec3(r * std::cos(phi), y, r * std::sin(phi));
		std::vector<IdRecord> view;
		for (const auto &g : kGroups) {
			bool facing = true;
			for (int i : g) {
				facing = facing && left.normal[i].dot(c - left.position[i]) > 0.0;
			}
			if (!facing) {
				continue;
			}
			for (int ia = 0; ia < 5; ia++) {
				for (int ib = 0; ib < 5; ib++) {
					if (ia == ib) {
						continue;
					}
					IdRecord rec;
					rec.ids[0] = g[ia];
					rec.ids[1] = g[ib];
					Vec3 other[3];
					for (int i = 0, n = 0; i < 5; i++) {
						if (i != ia && i != ib) {
							rec.ids[2 + n] = g[i];
							other[n++] = (left.position[g[i]] - c).normalized();
						}
					}
					Vec2 pts[3];
					basis_descriptor((left.position[g[ia]] - c).normalized(),
					                 (left.position[g[ib]] - c).normalized(), other, 3, pts);
					std::copy(pts, pts + 3, rec.pts.begin());
					view.push_back(rec);
				}
			}
		}
		std::sort(view.begin(), view.end(), [](const IdRecord &a, const IdRecord &b) { return a.ids < b.ids; });
		records.insert(records.end(), view.begin(), view.end());
	}
	load(std::move(records));
}

void
IdTable::load(std::vector<IdRecord> records)
{
	records_ = std::move(records);
	build_grid();
}

void
IdTable::build_grid()
{
	// Equal-frequency bins of the first descriptor point, x and y separately.
	auto edges = [&](int axis) {
		std::vector<double> v;
		for (const IdRecord &r : records_) {
			v.push_back(r.pts[0][axis]);
		}
		// Ascending: initP5PList's std::sort (0x2851c4) has the inlined comparator a < b (0x28526c).
		std::sort(v.begin(), v.end());
		std::vector<double> e;
		if (v.empty()) {
			return e;
		}
		size_t step = v.size() / kBins;
		for (int k = 0; k < kBins; k++) {
			e.push_back(v[std::min(v.size() - 1, k * step)]);
		}
		e.push_back(v.back() + 1e-8);
		return e;
	};
	edges_x_ = edges(0);
	edges_y_ = edges(1);
	grid_.assign(kBins * kBins, {});
	for (size_t i = 0; i < records_.size(); i++) {
		int bx = bin(edges_x_, records_[i].pts[0].x()), by = bin(edges_y_, records_[i].pts[0].y());
		grid_[bx * kBins + by].push_back((int)i);
	}
}

int
IdTable::bin(const std::vector<double> &edges, double v) const
{
	if (edges.empty() || v < edges[0] + 1e-8) {
		return 0;
	}
	auto it = std::upper_bound(edges.begin(), edges.end(), v); // first edge greater than v
	int k = it == edges.end() ? kBins : (int)(it - edges.begin()) - 1;
	return std::min(std::max(k, 0), kBins - 1);
}

std::vector<std::vector<std::pair<int, int>>>
IdTable::lookup(const std::vector<Vec3> &bearings, bool mirror, int max_lists) const
{
	std::vector<std::vector<std::pair<int, int>>> out;
	int n = (int)bearings.size();
	if (n < 5 || records_.empty()) {
		return out;
	}
	using Vote = std::vector<std::pair<int, int>>;
	std::vector<std::pair<Vote, int>> votes; // in the order first found
	std::map<Vote, size_t> index;
	const double r2 = kMatchRadius * kMatchRadius;
	for (int i = 0; i < n; i++) {
		for (int j = i + 1; j < n; j++) {
			std::vector<int> others;
			std::vector<Vec3> rest;
			for (int o = 0; o < n; o++) {
				if (o != i && o != j) {
					others.push_back(o);
					rest.push_back(bearings[o]);
				}
			}
			std::vector<Vec2> q(rest.size());
			basis_descriptor(bearings[i], bearings[j], rest.data(), (int)rest.size(), q.data());
			if (mirror) {
				for (Vec2 &p : q) {
					p.y() = -p.y();
				}
			}
			for (size_t k1 = 0; k1 < others.size(); k1++) {
				// Records whose first point lies within the radius: the grid cells the disc touches.
				const Vec2 &c = q[k1];
				int x0 = bin(edges_x_, c.x() - kMatchRadius), x1 = bin(edges_x_, c.x() + kMatchRadius);
				int y0 = bin(edges_y_, c.y() - kMatchRadius), y1 = bin(edges_y_, c.y() + kMatchRadius);
				std::vector<int> cand1;
				for (int bx = x0; bx <= x1; bx++) {
					for (int by = y0; by <= y1; by++) {
						double nx = std::clamp(c.x(), edges_x_[bx], edges_x_[bx + 1]);
						double ny = std::clamp(c.y(), edges_y_[by], edges_y_[by + 1]);
						if ((Vec2(nx, ny) - c).squaredNorm() > r2) {
							continue; // the disc misses this cell
						}
						for (int r : grid_[bx * kBins + by]) {
							if ((records_[r].pts[0] - c).squaredNorm() <= r2) {
								cand1.push_back(r);
							}
						}
					}
				}
				for (size_t k2 = 0; k2 < others.size(); k2++) {
					if (k2 == k1) {
						continue;
					}
					std::vector<int> cand2;
					for (int r : cand1) {
						if ((records_[r].pts[1] - q[k2]).squaredNorm() <= r2) {
							cand2.push_back(r);
						}
					}
					for (size_t k3 = 0; k3 < others.size(); k3++) {
						if (k3 == k1 || k3 == k2) {
							continue;
						}
						for (int r : cand2) {
							if ((records_[r].pts[2] - q[k3]).squaredNorm() > r2) {
								continue;
							}
							const auto &ids = records_[r].ids;
							Vote v = {{i, ids[0]}, {j, ids[1]}, {others[k1], ids[2]}, {others[k2], ids[3]},
							          {others[k3], ids[4]}};
							std::sort(v.begin(), v.end());
							auto it = index.find(v);
							if (it != index.end()) {
								votes[it->second].second++;
							} else {
								index.emplace(v, votes.size());
								votes.emplace_back(v, 1);
							}
						}
					}
				}
			}
		}
	}
	// Comparator 0x338178 is count(a) > count(b); the sort is the stock std::sort of idvote
	// (0x339b04, insertion sort only up to 6 elements), so equal counts end up in its order.
	stock_sort(votes.begin(), votes.end(), [](const auto &a, const auto &b) { return a.second > b.second; }, 6);
	for (size_t k = 0; k < votes.size() && (int)k < max_lists; k++) {
		out.push_back(votes[k].first);
	}
	return out;
}

} // namespace pico4::ctrack
