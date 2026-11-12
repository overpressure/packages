// Copyright 2026, overpressure contributors.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Controller LED blob detection of the PICO 4 controller tracker.
 * @ingroup drv_pico4
 */

#include "pico4_ctrack_blob.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <utility>

namespace pico4::ctrack {

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kFourPi = 12.566370614359172;
//! Stock "mean" divisor offset (n + this) of the near-path blob size and of computeObjPointAngle.
constexpr double kMeanEpsilon = 1.7453292855e-7;

//! Chain-code directions in the order E, NE, N, NW, W, SW, S, SE (image y grows downwards).
constexpr int kDirX[8] = {1, 1, 0, -1, -1, -1, 0, 1};
constexpr int kDirY[8] = {0, -1, -1, -1, 0, 1, 1, 1};

//! Border-following marks: an ordinary border pixel, and one whose east neighbour is background.
constexpr int8_t kMarkBorder = 2;
constexpr int8_t kMarkRightBorder = (int8_t)(2 - 128);

int
clampi(int v, int lo, int hi)
{
	return v < lo ? lo : (v > hi ? hi : v);
}

double
point_distance(double ax, double ay, double bx, double by)
{
	double dx = ax - bx, dy = ay - by;
	return std::sqrt(dx * dx + dy * dy);
}

/*!
 * Follow the outer border that starts at @p start (a 1-pixel whose west neighbour is 0),
 * marking it and appending its points. The neighbour search starts at the west and turns
 * clockwise to find the second border pixel, then each step searches counter-clockwise from
 * the direction back to the previous pixel.
 */
void
follow_border(int stride, int8_t *start, Point2i pt, ChainApprox approx, Point2i offset, std::vector<Point2i> &out)
{
	int step[16];
	for (int i = 0; i < 8; i++) {
		step[i] = step[i + 8] = kDirY[i] * stride + kDirX[i];
	}

	int dir = 4;
	int8_t *second = nullptr;
	do {
		dir = (dir - 1) & 7;
		second = start + step[dir];
	} while (*second == 0 && dir != 4);

	if (dir == 4) {
		// An isolated pixel: a one-point contour.
		*start = kMarkRightBorder;
		out.push_back(Point2i{pt.x + offset.x, pt.y + offset.y});
		return;
	}

	int8_t *cur = start;
	int written_dir = dir ^ 4;
	for (;;) {
		int back = dir;
		int s = back;
		int8_t *next = nullptr;
		while (s < 15) {
			next = cur + step[++s];
			if (*next != 0) {
				break;
			}
		}
		// The east neighbour was examined and found empty when the search wrapped past it.
		bool east_empty = s > 8;
		s &= 7;

		if (east_empty) {
			*cur = kMarkRightBorder;
		} else if (*cur == 1) {
			*cur = kMarkBorder;
		}

		if (approx == ChainApprox::kNone || s != written_dir) {
			out.push_back(Point2i{pt.x + offset.x, pt.y + offset.y});
			written_dir = s;
		}
		pt.x += kDirX[s];
		pt.y += kDirY[s];

		if (next == start && cur == second) {
			break;
		}
		cur = next;
		dir = (s + 4) & 7;
	}
}

std::vector<BlobPoint>
shifted(std::vector<BlobPoint> blobs, float dx, float dy)
{
	for (BlobPoint &b : blobs) {
		b.x += dx;
		b.y += dy;
	}
	return blobs;
}

std::vector<Point2f>
centres(const std::vector<BlobPoint> &blobs)
{
	std::vector<Point2f> pts;
	pts.reserve(blobs.size());
	for (const BlobPoint &b : blobs) {
		pts.push_back(Point2f{b.x, b.y});
	}
	return pts;
}

float
mean_x(const std::vector<BlobPoint> &blobs)
{
	if (blobs.empty()) {
		return 0.0f;
	}
	// setBolbsPriority's centroid: the sum of x / n in float (0x385e74..0x385e84).
	const float n = (float)blobs.size();
	float sum = 0.0f;
	for (const BlobPoint &b : blobs) {
		sum += b.x / n;
	}
	return sum;
}

//! Grey value with the coordinates clamped into the image (the stock code reads unchecked).
int
pixel_clamped(const ImageView &img, int x, int y)
{
	return img.at(clampi(x, 0, img.width - 1), clampi(y, 0, img.height - 1));
}

/*!
 * Remove blobs on static lights: a light is valid when 0 <= u <= 639 and 0 <= v <= 639 (stock
 * quirk: the second limit should be 479); a blob closer than 10 px to a valid light is dropped
 * and counted on the first such light.
 */
void
remove_static_lights(std::vector<BlobPoint> &blobs, const std::vector<Vec2> &lights, std::vector<int> *hits)
{
	std::vector<BlobPoint> kept;
	kept.reserve(blobs.size());
	for (const BlobPoint &b : blobs) {
		bool drop = false;
		for (size_t i = 0; i < lights.size() && !drop; i++) {
			const Vec2 &l = lights[i];
			// Stock quirk: v is tested against 639, not 479.
			if (!(l.x() >= 0.0 && l.x() <= 639.0 && l.y() >= 0.0 && l.y() <= 639.0)) {
				continue;
			}
			if (point_distance(b.x, b.y, l.x(), l.y()) < 10.0) {
				drop = true;
				if (hits != nullptr) {
					(*hits)[i]++;
				}
			}
		}
		if (!drop) {
			kept.push_back(b);
		}
	}
	blobs = std::move(kept);
}

//! A small deterministic generator (multiply-with-carry, the scheme of OpenCV's cv::RNG).
struct Mwc
{
	uint64_t state = 0xffffffffULL;

	uint32_t
	next()
	{
		state = (uint64_t)(uint32_t)state * 4164903690ULL + (state >> 32);
		return (uint32_t)state;
	}

	double
	uniform01()
	{
		return next() * (1.0 / 4294967296.0);
	}
};

double
squared(double v)
{
	return v * v;
}

} // namespace

/*
 *
 * Detector.
 *
 */

std::vector<uint8_t>
box_mean(const ImageView &src, int block)
{
	const int w = src.width, h = src.height, r = block / 2;
	std::vector<uint32_t> rows((size_t)w * (size_t)h);
	for (int y = 0; y < h; y++) {
		const uint8_t *row = src.data + (size_t)y * (size_t)src.stride;
		uint32_t *dst = rows.data() + (size_t)y * (size_t)w;
		uint32_t s = 0;
		for (int i = -r; i <= r; i++) {
			s += row[clampi(i, 0, w - 1)];
		}
		dst[0] = s;
		for (int x = 1; x < w; x++) {
			s += row[clampi(x + r, 0, w - 1)];
			s -= row[clampi(x - r - 1, 0, w - 1)];
			dst[x] = s;
		}
	}

	// OpenCV's 8-bit box filter sums in 16 bits and divides in 16.16 fixed point.
	const int divisor = block * block;
	const double exact = 65536.0 / divisor;
	int scale = (int)std::floor(exact);
	int bias = divisor / 2;
	if (exact - scale < 0.5) {
		bias++;
	} else {
		scale++;
	}

	std::vector<uint8_t> mean((size_t)w * (size_t)h);
	for (int x = 0; x < w; x++) {
		uint32_t s = 0;
		for (int j = -r; j <= r; j++) {
			s += rows[(size_t)clampi(j, 0, h - 1) * (size_t)w + (size_t)x];
		}
		mean[(size_t)x] = (uint8_t)(((int)s + bias) * scale >> 16);
		for (int y = 1; y < h; y++) {
			s += rows[(size_t)clampi(y + r, 0, h - 1) * (size_t)w + (size_t)x];
			s -= rows[(size_t)clampi(y - r - 1, 0, h - 1) * (size_t)w + (size_t)x];
			mean[(size_t)y * (size_t)w + (size_t)x] = (uint8_t)(((int)s + bias) * scale >> 16);
		}
	}
	return mean;
}

std::vector<int8_t>
binarise(const ImageView &src, bool adaptive, int threshold, int *block_out)
{
	const int w = src.width, h = src.height, pw = w + 2;
	std::vector<int8_t> buf((size_t)pw * (size_t)(h + 2), 0);
	int block = 0;
	if (!adaptive) {
		for (int y = 0; y < h; y++) {
			int8_t *dst = buf.data() + (size_t)(y + 1) * (size_t)pw + 1;
			for (int x = 0; x < w; x++) {
				dst[x] = src.at(x, y) > threshold ? 1 : 0;
			}
		}
	} else {
		block = (w > 15 && h > 15) ? 11 : 5;
		std::vector<uint8_t> mean = box_mean(src, block);
		for (int y = 0; y < h; y++) {
			int8_t *dst = buf.data() + (size_t)(y + 1) * (size_t)pw + 1;
			const uint8_t *m = mean.data() + (size_t)y * (size_t)w;
			for (int x = 0; x < w; x++) {
				// A saturating 8-bit add in the stock vector path; the same result as in int.
				dst[x] = (int)src.at(x, y) > (int)m[x] + kAdaptiveOffset ? 1 : 0;
			}
		}
	}
	if (block_out != nullptr) {
		*block_out = block;
	}
	return buf;
}

std::vector<std::vector<Point2i>>
find_outer_contours(int8_t *buf, int w, int h, ChainApprox approx, Point2i offset)
{
	std::vector<std::vector<Point2i>> contours;
	for (int y = 1; y < h - 1; y++) {
		int8_t *row = buf + (size_t)y * (size_t)w;
		// Column of the last border mark met on this row (column 0 is the zero frame).
		int last_mark = 0;
		int prev = 0;
		for (int x = 1; x < w - 1; x++) {
			int p = row[x];
			if (p == prev) {
				continue;
			}
			if (prev == 0 && p == 1) {
				// An outer border starts here unless the scan is inside a traced blob (its
				// left border, a positive mark, was the last mark met): blobs in holes are skipped.
				if (row[last_mark] <= 0) {
					std::vector<Point2i> c;
					follow_border(w, row + x, Point2i{x, y}, approx, offset, c);
					contours.push_back(std::move(c));
					// Continue after the start pixel with its new mark; last_mark stays.
					prev = row[x];
					continue;
				}
			} else if (p == 0 && prev >= 1) {
				// A hole starts (not traced in this mode).
				if (prev != 1) {
					last_mark = x - 1;
				}
			}
			prev = p;
			if (p != 0 && p != 1) {
				last_mark = x;
			}
		}
	}
	return contours;
}

ContourMoments
contour_moments(const std::vector<Point2i> &c)
{
	ContourMoments m;
	const size_t n = c.size();
	if (n == 0) {
		return m;
	}
	// Green's theorem over the closed polygon, edge by edge from the last point.
	double a00 = 0, a10 = 0, a01 = 0, a20 = 0, a11 = 0, a02 = 0;
	double xp = c[n - 1].x, yp = c[n - 1].y;
	for (size_t i = 0; i < n; i++) {
		double xc = c[i].x, yc = c[i].y;
		double cross = xp * yc - xc * yp;
		a00 += cross;
		a10 += cross * (xp + xc);
		a01 += cross * (yp + yc);
		a20 += cross * (xp * xp + xp * xc + xc * xc);
		a11 += cross * (2.0 * xp * yp + xp * yc + xc * yp + 2.0 * xc * yc);
		a02 += cross * (yp * yp + yp * yc + yc * yc);
		xp = xc;
		yp = yc;
	}
	if (std::fabs(a00) <= (double)std::numeric_limits<float>::epsilon()) {
		return m;
	}
	const double sign = a00 > 0 ? 1.0 : -1.0;
	m.m00 = sign * a00 / 2.0;
	m.m10 = sign * a10 / 6.0;
	m.m01 = sign * a01 / 6.0;
	m.m20 = sign * a20 / 12.0;
	m.m11 = sign * a11 / 24.0;
	m.m02 = sign * a02 / 12.0;
	const double cx = m.m10 / m.m00, cy = m.m01 / m.m00;
	m.mu20 = m.m20 - m.m10 * cx;
	m.mu11 = m.m11 - m.m10 * cy;
	m.mu02 = m.m02 - m.m01 * cy;
	return m;
}

double
closed_arc_length(const std::vector<Point2i> &c)
{
	double perimeter = 0.0;
	if (c.empty()) {
		return perimeter;
	}
	Point2i prev = c.back();
	for (const Point2i &p : c) {
		// Single-precision step lengths, accumulated in double (as OpenCV's arcLength).
		float dx = (float)(p.x - prev.x), dy = (float)(p.y - prev.y);
		perimeter += std::sqrt(dx * dx + dy * dy);
		prev = p;
	}
	return perimeter;
}

double
polygon_area(const std::vector<Point2i> &c)
{
	if (c.size() < 3) {
		return 0.0;
	}
	double a = 0.0;
	Point2i prev = c.back();
	for (const Point2i &p : c) {
		a += (double)prev.x * p.y - (double)p.x * prev.y;
		prev = p;
	}
	return std::fabs(a) / 2.0;
}

std::vector<Point2i>
convex_hull(std::vector<Point2i> pts)
{
	std::sort(pts.begin(), pts.end(),
	          [](const Point2i &a, const Point2i &b) { return a.x < b.x || (a.x == b.x && a.y < b.y); });
	pts.erase(std::unique(pts.begin(), pts.end(),
	                      [](const Point2i &a, const Point2i &b) { return a.x == b.x && a.y == b.y; }),
	          pts.end());
	if (pts.size() < 3) {
		return pts;
	}
	auto turn = [](const Point2i &o, const Point2i &a, const Point2i &b) {
		return (int64_t)(a.x - o.x) * (b.y - o.y) - (int64_t)(a.y - o.y) * (b.x - o.x);
	};
	std::vector<Point2i> hull(2 * pts.size());
	size_t k = 0;
	for (size_t i = 0; i < pts.size(); i++) {
		while (k >= 2 && turn(hull[k - 2], hull[k - 1], pts[i]) <= 0) {
			k--;
		}
		hull[k++] = pts[i];
	}
	for (size_t i = pts.size() - 1, lower = k + 1; i-- > 0;) {
		while (k >= lower && turn(hull[k - 2], hull[k - 1], pts[i]) <= 0) {
			k--;
		}
		hull[k++] = pts[i];
	}
	hull.resize(k - 1);
	return hull;
}

std::vector<BlobCandidate>
filter_contours(const std::vector<std::vector<Point2i>> &contours, const DetectorParams &params, int block)
{
	std::vector<BlobCandidate> out;
	for (const std::vector<Point2i> &c : contours) {
		const ContourMoments m = contour_moments(c);
		BlobCandidate cand;
		cand.area = m.m00;

		if (params.filter_by_area) {
			// Only the adaptive mode with block 11 keeps the limit; the others allow 9 x.
			const float max_area = block == 11 ? params.max_area : 9.0f * params.max_area;
			if (m.m00 < params.min_area || m.m00 >= max_area) {
				continue;
			}
		}

		if (params.filter_by_circularity) {
			const double perimeter = closed_arc_length(c);
			const double ratio = kFourPi * m.m00 / (perimeter * perimeter);
			if (ratio < params.min_circularity || ratio >= params.max_circularity) {
				continue;
			}
			cand.circularity = ratio;
		}

		if (params.filter_by_inertia) {
			// Ratio of the smaller to the larger principal moment of inertia.
			const double spread = std::sqrt(squared(m.mu20 - m.mu02) + 4.0 * m.mu11 * m.mu11);
			double ratio = 1.0;
			if (spread > 0.01) {
				const double mid = 0.5 * (m.mu20 + m.mu02);
				ratio = (mid - 0.5 * spread) / (mid + 0.5 * spread);
			}
			if (ratio < params.min_inertia_ratio || ratio >= params.max_inertia_ratio) {
				continue;
			}
			cand.confidence = ratio * ratio;
		}

		// Convexity is always computed, but a filter only when enabled.
		const double hull_area = polygon_area(convex_hull(c));
		cand.convexity = polygon_area(c) / hull_area;
		if (params.filter_by_convexity &&
		    (cand.convexity < params.min_convexity || cand.convexity >= params.max_convexity)) {
			continue;
		}

		if (m.m00 == 0.0) {
			continue;
		}
		cand.x = m.m10 / m.m00;
		cand.y = m.m01 / m.m00;

		if (params.filter_by_color) {
			// The stock test compares the 0/1 buffer with blobColor (255): it never matches.
			continue;
		}

		std::vector<double> dists;
		dists.reserve(c.size());
		for (const Point2i &p : c) {
			dists.push_back(point_distance(cand.x, cand.y, p.x, p.y));
		}
		std::sort(dists.begin(), dists.end());
		cand.radius = (dists[(dists.size() - 1) / 2] + dists[dists.size() / 2]) / 2.0;
		out.push_back(cand);
	}
	return out;
}

std::vector<BlobPoint>
detect_blobs(const ImageView &image, bool adaptive, int threshold, const DetectorParams &params)
{
	std::vector<BlobPoint> out;
	if (image.data == nullptr || image.width <= 0 || image.height <= 0) {
		return out;
	}
	// The stock work buffer is 642 x 482: larger inputs are not supported.
	if (image.width > kBlobImageWidth || image.height > kBlobImageHeight || params.min_repeatability <= 0 ||
	    !(params.threshold_step > 0.0f)) {
		return out;
	}

	// Groups of candidates across passes, each sorted by radius.
	std::vector<std::vector<BlobCandidate>> groups;
	for (float t = params.min_threshold; t < params.max_threshold; t += params.threshold_step) {
		// Stock quirk: the pass variable t is never used; every pass binarises with the
		// threshold argument (one pass with the stock parameters).
		int block = 0;
		std::vector<int8_t> buf = binarise(image, adaptive, threshold, &block);
		const std::vector<std::vector<Point2i>> contours =
		    find_outer_contours(buf.data(), image.width + 2, image.height + 2, ChainApprox::kNone, Point2i{-1, -1});
		const std::vector<BlobCandidate> cands = filter_contours(contours, params, block);

		std::vector<std::vector<BlobCandidate>> fresh;
		for (const BlobCandidate &c : cands) {
			bool joined = false;
			for (std::vector<BlobCandidate> &g : groups) {
				const BlobCandidate &median = g[g.size() / 2];
				const double d = point_distance(median.x, median.y, c.x, c.y);
				if (d < params.min_dist_between_blobs || d < median.radius || d < c.radius) {
					auto at = std::upper_bound(
					    g.begin(), g.end(), c.radius,
					    [](double r, const BlobCandidate &member) { return r < member.radius; });
					g.insert(at, c);
					joined = true;
					break;
				}
			}
			if (!joined) {
				fresh.push_back(std::vector<BlobCandidate>{c});
			}
		}
		// Candidates of one pass never merge with each other.
		groups.insert(groups.end(), fresh.begin(), fresh.end());
	}

	for (const std::vector<BlobCandidate> &g : groups) {
		if ((int)g.size() < params.min_repeatability) {
			continue;
		}
		double sx = 0, sy = 0, weight = 0;
		for (const BlobCandidate &c : g) {
			sx += c.confidence * c.x;
			sy += c.confidence * c.y;
			weight += c.confidence;
		}
		const BlobCandidate &median = g[g.size() / 2];
		BlobPoint b;
		b.x = (float)(sx * (1.0 / weight));
		b.y = (float)(sy * (1.0 / weight));
		b.size = (float)median.radius * 2.0f;
		b.area = (float)median.area;
		b.circularity = (float)median.circularity;
		b.convexity = (float)median.convexity;
		out.push_back(b);
	}
	return out;
}

/*
 *
 * Geometry helpers.
 *
 */

void
RotatedRect::points(Point2f pts[4]) const
{
	// cv::RotatedRect::points (0x9ad580): one float operation per statement, in its order.
	const double rad = (double)angle * kPi / 180.0;
	const float half_cos = (float)std::cos(rad) * 0.5f;
	const float half_sin = (float)std::sin(rad) * 0.5f;
	const float sh = half_sin * height, cw = half_cos * width;
	const float ch = half_cos * height, sw = half_sin * width;
	float t = center.x - sh;
	pts[0].x = t - cw;
	t = ch + center.y;
	pts[0].y = t - sw;
	t = sh + center.x;
	pts[1].x = t - cw;
	t = center.y - ch;
	pts[1].y = t - sw;
	const float cx2 = center.x + center.x, cy2 = center.y + center.y;
	pts[2].x = cx2 - pts[0].x;
	pts[2].y = cy2 - pts[0].y;
	pts[3].x = cx2 - pts[1].x;
	pts[3].y = cy2 - pts[1].y;
}

RectI
RotatedRect::bounding_rect() const
{
	Point2f p[4];
	points(p);
	float x0 = p[0].x, x1 = p[0].x, y0 = p[0].y, y1 = p[0].y;
	for (int i = 1; i < 4; i++) {
		x0 = std::min(x0, p[i].x);
		x1 = std::max(x1, p[i].x);
		y0 = std::min(y0, p[i].y);
		y1 = std::max(y1, p[i].y);
	}
	RectI r;
	r.x = (int)std::floor(x0);
	r.y = (int)std::floor(y0);
	r.width = (int)std::ceil(x1) - r.x + 1;
	r.height = (int)std::ceil(y1) - r.y + 1;
	return r;
}

namespace {

int
sign_of(float v)
{
	return (v > 0.0f ? 1 : 0) - (v < 0.0f ? 1 : 0);
}

bool
same_bits(const Point2f &a, const Point2f &b)
{
	uint32_t ax, ay, bx, by;
	std::memcpy(&ax, &a.x, 4);
	std::memcpy(&ay, &a.y, 4);
	std::memcpy(&bx, &b.x, 4);
	std::memcpy(&by, &b.y, 4);
	return ax == bx && ay == by;
}

/*!
 * One monotone chain of the stock hull (float Sklansky scan, 0x7dd6ac): walk the x-sorted points
 * @p s from @p from towards @p to (inclusive), skipping points that move vertically in direction
 * @p skip_dy and keeping a turn of sign @p keep_turn. Returns the chain's indices into @p s.
 */
std::vector<int>
hull_chain(const std::vector<Point2f> &s, int from, int to, int skip_dy, int keep_turn)
{
	if (from == to || (s[(size_t)from].x == s[(size_t)to].x && s[(size_t)from].y == s[(size_t)to].y)) {
		return {from};
	}
	const int step = to > from ? 1 : -1;
	const int stop = to + step;
	// a, b, c: the last two kept points and the candidate; st ends with the candidate.
	int a = from, b = from + step, c = b + step;
	std::vector<int> st = {a, b, c};
	while (c != stop) {
		const Point2f &pa = s[(size_t)a], &pb = s[(size_t)b], &pc = s[(size_t)c];
		const float dy_bc = pc.y - pb.y;
		if (sign_of(dy_bc) == skip_dy) {
			c += step;
			st.back() = c;
			continue;
		}
		const float dx_ab = pb.x - pa.x;
		const float dy_ab = pb.y - pa.y;
		const float dx_bc = pc.x - pb.x;
		const float t1 = dx_bc * dy_ab;
		const float t2 = dy_bc * dx_ab;
		const float turn = t1 - t2;
		if (sign_of(turn) == keep_turn && (dx_ab != 0.0f || dy_ab != 0.0f)) {
			a = b;
			b = c;
			c += step;
			st.push_back(c);
		} else if (a == from) {
			// b is not on the hull and has no predecessor to fall back to: replace it by c.
			b = c;
			c += step;
			st[1] = b;
			st[2] = c;
		} else {
			// Drop b and retest c against the previous pair.
			const size_t n = st.size();
			b = a;
			a = st[n - 4];
			st[n - 2] = c;
			st.pop_back();
		}
	}
	st.pop_back(); // the candidate past the end
	return st;
}

/*!
 * Convex hull in the order of the stock cv::convexHull(points, hull, clockwise = true) for float
 * points (OpenCV 3.4.0 in the library, 0x7dc7f4): sort by x then y, an upper and a lower pair of
 * monotone chains between the lowest-x and highest-x points, joined at the first highest-y and
 * the first lowest-y point.
 */
std::vector<Point2f>
stock_convex_hull(std::vector<Point2f> s)
{
	const int n = (int)s.size();
	if (n == 0) {
		return {};
	}
	std::sort(s.begin(), s.end(), [](const Point2f &p, const Point2f &q) { return p.x < q.x || (p.x == q.x && p.y < q.y); });
	int lowest = 0, highest = 0; // first index of the minimum and maximum y
	for (int i = 1; i < n; i++) {
		if (s[(size_t)lowest].y > s[(size_t)i].y) {
			lowest = i;
		}
		if (s[(size_t)highest].y < s[(size_t)i].y) {
			highest = i;
		}
	}
	if (same_bits(s[0], s[(size_t)n - 1])) {
		return {s[0]};
	}
	std::vector<int> order;
	// Upper part: the chain from the first point, then the one from the last point backwards.
	const std::vector<int> up_l = hull_chain(s, 0, highest, -1, 1);
	const std::vector<int> up_r = hull_chain(s, n - 1, highest, -1, -1);
	for (size_t i = 0; i + 1 < up_l.size(); i++) {
		order.push_back(up_l[i]);
	}
	for (size_t i = up_r.size() - 1; i > 0; i--) {
		order.push_back(up_r[i]);
	}
	const int stop_idx = up_r.size() > 2 ? up_r[1] : up_l.size() > 2 ? up_l[up_l.size() - 2] : -1;
	// Lower part, clockwise: the chain from the last point first.
	const std::vector<int> lo_a = hull_chain(s, n - 1, lowest, 1, 1);
	const std::vector<int> lo_b = hull_chain(s, 0, lowest, 1, -1);
	size_t na = lo_a.size(), nb = lo_b.size();
	if (stop_idx >= 0) {
		const int check = na > 2 ? lo_a[1] : na + nb > 2 ? lo_b[2 - na] : -1;
		if (check == stop_idx || (check >= 0 && same_bits(s[(size_t)check], s[(size_t)stop_idx]))) {
			// All points on one line: the lower part mirrors the upper one.
			na = std::min<size_t>(na, 2);
			nb = std::min<size_t>(nb, 2);
		}
	}
	for (size_t i = 0; i + 1 < na; i++) {
		order.push_back(lo_a[i]);
	}
	for (size_t i = nb - 1; i > 0; i--) {
		order.push_back(lo_b[i]);
	}
	std::vector<Point2f> hull;
	hull.reserve(order.size());
	for (int i : order) {
		hull.push_back(s[(size_t)i]);
	}
	return hull;
}

//! sqrt(x^2 + y^2) of float components in double, squares and sum as separate operations.
double
hypot_d(float x, float y)
{
	const double x2 = (double)x * x, y2 = (double)y * y;
	const double sum = x2 + y2;
	return std::sqrt(sum);
}

//! Stock angle conversion (0x864c14): float radians times 180, then divided by pi in double.
float
degrees_of(float rad)
{
	const float scaled = rad * 180.0f;
	return (float)((double)scaled / kPi);
}

} // namespace

RotatedRect
min_area_rect(const std::vector<Point2f> &pts)
{
	// cv::minAreaRect of the bundled OpenCV 3.4.0 (0x8643f4): the clockwise hull above, then
	// rotating calipers in float. Ties between equal areas take the last rectangle (fcmp + b.hi
	// at 0x864aac), and the angle is that of the caliper's base direction, so it is not
	// normalised to a fixed range.
	RotatedRect rr;
	const std::vector<Point2f> h = stock_convex_hull(pts);
	const int n = (int)h.size();
	if (n == 0) {
		return rr;
	}
	if (n == 1) {
		rr.center = h[0];
		return rr;
	}
	if (n == 2) {
		const float sx = h[0].x + h[1].x, sy = h[0].y + h[1].y;
		rr.center = Point2f{sx * 0.5f, sy * 0.5f};
		const float dx = h[1].x - h[0].x, dy = h[1].y - h[0].y;
		rr.width = (float)hypot_d(dx, dy);
		rr.angle = degrees_of((float)std::atan2((double)dy, (double)dx));
		return rr;
	}

	// Edge vectors, their inverse lengths and the extreme points (first index on ties).
	std::vector<Point2f> edge((size_t)n);
	std::vector<float> inv_len((size_t)n);
	int min_x = 0, max_x = 0, min_y = 0, max_y = 0;
	for (int i = 0; i < n; i++) {
		const Point2f &p = h[(size_t)i];
		if (h[(size_t)min_x].x > p.x) {
			min_x = i;
		}
		if (h[(size_t)max_x].x < p.x) {
			max_x = i;
		}
		if (h[(size_t)max_y].y < p.y) {
			max_y = i;
		}
		if (h[(size_t)min_y].y > p.y) {
			min_y = i;
		}
		const Point2f &q = h[(size_t)((i + 1) % n)];
		const float ex = q.x - p.x, ey = q.y - p.y;
		edge[(size_t)i] = Point2f{ex, ey};
		inv_len[(size_t)i] = (float)(1.0 / hypot_d(ex, ey));
	}
	// Winding of the hull from the first non-zero cross product with the last edge.
	float wind = 0.0f;
	{
		const double lx = edge[(size_t)n - 1].x;
		double ly = edge[(size_t)n - 1].y;
		for (int i = 0; i < n && wind == 0.0f; i++) {
			const double c1 = lx * edge[(size_t)i].y, c2 = ly * edge[(size_t)i].x;
			const double cross = c1 - c2;
			if (cross != 0.0) {
				wind = cross > 0.0 ? 1.0f : -1.0f;
			}
			ly = edge[(size_t)i].y;
		}
	}

	// Calipers on the four extreme points; (ba, bb) is the base direction.
	int cal[4] = {min_y, max_x, max_y, min_x};
	float ba = wind, bb = 0.0f;
	float best_area = std::numeric_limits<float>::max();
	int best_c0 = 0, best_c3 = 0;
	float best_a = 0.0f, best_b = 0.0f, best_w = 0.0f, best_h = 0.0f;
	for (int k = 0; k < n; k++) {
		// Cosine between each caliper side and the hull edge leaving its point.
		float cosine[4];
		for (int s = 0; s < 4; s++) {
			const Point2f &e = edge[(size_t)cal[s]];
			float dot;
			if (s == 0) {
				const float u = ba * e.x, v = bb * e.y;
				dot = u + v;
			} else if (s == 1) {
				const float u = e.y * ba, v = bb * e.x;
				dot = u - v;
			} else if (s == 2) {
				const float u = -(ba * e.x), v = bb * e.y;
				dot = u - v;
			} else {
				const float u = bb * e.x, v = ba * e.y;
				dot = u - v;
			}
			cosine[s] = dot * inv_len[(size_t)cal[s]];
		}
		int lead = 0;
		float best_cos = cosine[0];
		for (int s = 1; s < 4; s++) {
			if (cosine[s] > best_cos) {
				best_cos = cosine[s];
				lead = s;
			}
		}
		// Turn the calipers onto that edge.
		const int pi = cal[lead];
		const float lx = edge[(size_t)pi].x * inv_len[(size_t)pi];
		const float ly = inv_len[(size_t)pi] * edge[(size_t)pi].y;
		switch (lead) {
		case 0: ba = lx, bb = ly; break;
		case 1: ba = ly, bb = -lx; break;
		case 2: ba = -lx, bb = -ly; break;
		default: ba = -ly, bb = lx; break;
		}
		cal[lead] = cal[lead] + 1 == n ? 0 : cal[lead] + 1;

		const Point2f &p0 = h[(size_t)cal[0]], &p1 = h[(size_t)cal[1]], &p2 = h[(size_t)cal[2]],
		              &p3 = h[(size_t)cal[3]];
		const float wx = p1.x - p3.x, wy = p1.y - p3.y;
		const float hx = p2.x - p0.x, hy = p2.y - p0.y;
		const float w1 = ba * wx, w2 = bb * wy;
		const float width = w1 + w2;
		const float h1 = bb * hx, h2 = ba * hy;
		const float height = h2 - h1;
		const float area = width * height;
		if (!(area > best_area)) {
			best_area = area;
			best_c0 = cal[0];
			best_c3 = cal[3];
			best_a = ba;
			best_b = bb;
			best_w = width;
			best_h = height;
		}
	}

	// Corner where the base side through point c0 meets the side through point c3.
	const float A = best_a, B = best_b;
	const Point2f &q3 = h[(size_t)best_c3], &q0 = h[(size_t)best_c0];
	const float aa = A * A, bb2 = B * B;
	const float norm = aa + bb2;
	const float inv = 1.0f / norm;
	const float c1a = A * q3.x, c1b = B * q3.y;
	const float c1 = c1a + c1b;
	const float c2a = A * q0.y, c2b = q0.x * B;
	const float c2 = c2a - c2b;
	const float ox1 = A * c1, ox2 = B * c2;
	const float oy1 = A * c2, oy2 = c1 * B;
	const float ox = inv * (ox1 - ox2);
	const float oy = inv * (oy1 + oy2);
	const float sx = best_w * A, sy = B * best_w;           // the width side
	const float tx = best_h * -B, ty = best_h * A;          // the height side
	const float mx = (sx + tx) * 0.5f, my = (sy + ty) * 0.5f;
	rr.center = Point2f{ox + mx, oy + my};
	rr.width = (float)hypot_d(sx, sy);
	rr.height = (float)hypot_d(tx, ty);
	rr.angle = degrees_of((float)std::atan2((double)sy, (double)sx));
	return rr;
}

bool
calc_safe_rect(const RotatedRect &rr, int cols, int rows, double margin, RectF &out)
{
	const RectI br = rr.bounding_rect();
	const double half = std::trunc(margin / 2.0);
	double left = br.x - 5.0 - half;
	double top = br.y - 5.0 - half;
	const double w = br.width + 10.0 + margin;
	const double h = br.height + 10.0 + margin;
	// Stock quirk: the far edges are inclusive coordinates used as exclusive ones (one pixel lost).
	const double right = (cols > left + w ? left + w : cols) - 1.0;
	const double bottom = (rows > top + h ? top + h : rows) - 1.0;
	left = std::max(left, 0.0);
	top = std::max(top, 0.0);
	const double width = right - left, height = bottom - top;
	if (width <= 0.0 || height <= 0.0) {
		return false;
	}
	out = RectF{(float)left, (float)top, (float)width, (float)height};
	return true;
}

RectI
round_rect(const RectF &r)
{
	return RectI{(int)std::lrint(r.x), (int)std::lrint(r.y), (int)std::lrint(r.width), (int)std::lrint(r.height)};
}

int
neighbour_max_brightness(const ImageView &img, float x, float y)
{
	// The stock loop hard-codes 640 x 480.
	const int cx = std::min((int)std::lround(x), kBlobImageWidth - 1);
	const int cy = std::min((int)std::lround(y), kBlobImageHeight - 1);
	int best = 0;
	for (int yy = cy - 1; yy <= cy + 1; yy++) {
		for (int xx = cx - 1; xx <= cx + 1; xx++) {
			if (xx < 0 || yy < 0 || xx >= std::min(img.width, kBlobImageWidth) ||
			    yy >= std::min(img.height, kBlobImageHeight)) {
				continue;
			}
			best = std::max(best, (int)img.at(xx, yy));
		}
	}
	return best;
}

float
dist_p2p(const BlobPoint &a, const BlobPoint &b)
{
	return (float)point_distance(a.x, a.y, b.x, b.y);
}

float
dist_p2l(Point2f p, Point2f a, Point2f b)
{
	// getDist_P2L (0x2daa58): |A p.x + B p.y + C| / sqrt(A^2 + B^2) in float, one operation per
	// statement so that no compiler fuses a multiply-add the stock code does not.
	const float A = b.y - a.y;
	const float B = a.x - b.x;
	const float c1 = a.y * b.x, c2 = b.y * a.x;
	const float C = c1 - c2;
	const float ax = A * p.x, by = B * p.y;
	const float lin = ax + by;
	const float num = std::fabs(C + lin);
	const float aa = A * A, bb = B * B;
	const float len2 = aa + bb;
	return num / std::sqrt(len2);
}

std::vector<int>
partition_labels(int n, const std::function<bool(int, int)> &same, int *classes)
{
	std::vector<int> parent((size_t)std::max(n, 0));
	for (int i = 0; i < n; i++) {
		parent[(size_t)i] = i;
	}
	auto root = [&](int i) {
		while (parent[(size_t)i] != i) {
			parent[(size_t)i] = parent[(size_t)parent[(size_t)i]];
			i = parent[(size_t)i];
		}
		return i;
	};
	for (int i = 0; i < n; i++) {
		for (int j = 0; j < n; j++) {
			if (i == j || !same(i, j)) {
				continue;
			}
			const int a = root(i), b = root(j);
			if (a != b) {
				parent[(size_t)std::max(a, b)] = std::min(a, b);
			}
		}
	}
	std::vector<int> labels((size_t)std::max(n, 0));
	std::vector<int> label_of_root((size_t)std::max(n, 0), -1);
	int count = 0;
	for (int i = 0; i < n; i++) {
		const int r = root(i);
		if (label_of_root[(size_t)r] < 0) {
			label_of_root[(size_t)r] = count++;
		}
		labels[(size_t)i] = label_of_root[(size_t)r];
	}
	if (classes != nullptr) {
		*classes = count;
	}
	return labels;
}

void
sort_blob_points(std::vector<BlobPoint> &blobs)
{
	if (blobs.size() < 2) {
		return;
	}
	Point2f c[4];
	min_area_rect(centres(blobs)).points(c);
	// Edge lengths from corner 0 as 0x37530c..0x375348: float differences, double squares.
	auto edge = [&](const Point2f &q) {
		const double dx = (double)(c[0].x - q.x), dy = (double)(c[0].y - q.y);
		const double dx2 = dx * dx, dy2 = dy * dy;
		return std::sqrt(dx2 + dy2);
	};
	const double d1 = edge(c[1]);
	const double d3 = edge(c[3]);
	// The line through corner 0 and corner 1 if that edge is strictly shorter, else corner 3
	// (fcmp + b.pl at 0x37534c: a tie takes corner 3).
	const bool use1 = d1 < d3;
	const Point2f near_corner = use1 ? c[1] : c[3];
	const Point2f far_corner = use1 ? c[3] : c[1];
	const bool degenerate = std::min(d1, d3) < 1e-6;
	double ux = (double)far_corner.x - c[0].x, uy = (double)far_corner.y - c[0].y;
	const double ulen = std::sqrt(ux * ux + uy * uy);
	if (ulen > 0) {
		ux /= ulen;
		uy /= ulen;
	}
	auto key = [&](const BlobPoint &b) {
		const Point2f p{b.x, b.y};
		if (!degenerate) {
			return dist_p2l(p, c[0], near_corner);
		}
		// Collinear centres: the short edge has no direction (NaN keys in the stock code, which
		// leave the order unchanged); use the distance along the long edge instead, which is
		// what the short-edge distance measures.
		return (float)std::fabs(((double)p.x - c[0].x) * ux + ((double)p.y - c[0].y) * uy);
	};
	// (blob index, float distance) pairs in index order, sorted ascending by the distance with
	// the stock std::sort of pair<int, float> (0x37559c; insertion sort up to 6 elements).
	std::vector<std::pair<int, float>> keyed;
	keyed.reserve(blobs.size());
	for (size_t i = 0; i < blobs.size(); i++) {
		keyed.emplace_back((int)i, key(blobs[i]));
	}
	stock_sort(keyed.begin(), keyed.end(), [](const auto &a, const auto &b) { return a.second < b.second; }, 6);
	std::vector<BlobPoint> out;
	out.reserve(blobs.size());
	for (const auto &k : keyed) {
		out.push_back(blobs[(size_t)k.first]);
	}
	blobs = std::move(out);
}

/*
 *
 * Shared stages.
 *
 */

void
refound_blob_points(const ImageView &img, std::vector<BlobPoint> &blobs, bool near)
{
	if (blobs.empty()) {
		return;
	}
	float sum = 0.0f;
	for (const BlobPoint &b : blobs) {
		sum += b.size;
	}
	// 1.5 x the float mean, in double (0x38ff70).
	const double limit = 1.5 * (double)(sum / (float)blobs.size());

	std::vector<BlobPoint> out;
	out.reserve(blobs.size());
	for (const BlobPoint &b : blobs) {
		if ((double)b.size <= limit) {
			out.push_back(b);
			continue;
		}
		const float s = b.size;
		// Window of +- size around the blob in float, clipped like calcSafeRect (far edges lose
		// a pixel), as 0x390048..0x3900c8.
		const float two = s + s;
		const float wx0 = b.x - s, wy0 = b.y - s;
		const float x_end = two + wx0, y_end = two + wy0;
		const float right = x_end < (float)img.width ? x_end - 1.0f : (float)(img.width - 1);
		const float bottom = y_end < (float)img.height ? y_end - 1.0f : (float)(img.height - 1);
		const float left = std::max(wx0, 0.0f), top = std::max(wy0, 0.0f);
		const float w = right - left, h = bottom - top;
		const bool outside = left > (float)(img.width - 1) || top >= (float)(img.height - 1);
		const RectI win = round_rect(RectF{left, top, w, h});
		if (outside || !(w > 0.0f) || !(h > 0.0f) || win.width <= 0 || win.height <= 0) {
			// Stock quirk: a large blob whose window is empty is dropped, not kept.
			continue;
		}

		// Saturated cores: pixel > 240, outer contours.
		const int pw = win.width + 2;
		std::vector<int8_t> buf((size_t)pw * (size_t)(win.height + 2), 0);
		for (int y = 0; y < win.height; y++) {
			for (int x = 0; x < win.width; x++) {
				buf[(size_t)(y + 1) * (size_t)pw + (size_t)(x + 1)] = img.at(win.x + x, win.y + y) > 240 ? 1 : 0;
			}
		}
		const std::vector<std::vector<Point2i>> cores =
		    find_outer_contours(buf.data(), pw, win.height + 2, ChainApprox::kSimple, Point2i{-1, -1});

		std::vector<Point2f> centre;
		for (const std::vector<Point2i> &c : cores) {
			if (near && c.size() < 2) {
				continue;
			}
			int x0 = c[0].x, x1 = x0, y0 = c[0].y, y1 = y0;
			for (const Point2i &p : c) {
				x0 = std::min(x0, p.x);
				x1 = std::max(x1, p.x);
				y0 = std::min(y0, p.y);
				y1 = std::max(y1, p.y);
			}
			// Centre of the bounding rect: x + w * 0.5 in double, as float (0x39026c..0x390294),
			// so half pixels are kept.
			centre.push_back(Point2f{(float)(x0 + (x1 - x0 + 1) * 0.5), (float)(y0 + (y1 - y0 + 1) * 0.5)});
		}
		if (centre.size() < 2) {
			out.push_back(b);
			continue;
		}
		for (const Point2f &c : centre) {
			BlobPoint piece;
			// Stock quirk: the pieces are placed from the unrounded float window corner, not
			// from the rounded ROI the cores were found in (s13/s14 at 0x3903a0).
			piece.x = left + c.x;
			piece.y = top + c.y;
			piece.size = 0.5f * s;
			out.push_back(piece);
		}
	}
	blobs = std::move(out);
}

void
remove_blobs_near_points(std::vector<BlobPoint> &blobs, const std::vector<Vec2> &points, float radius_factor)
{
	if (points.empty()) {
		return;
	}
	std::vector<BlobPoint> kept;
	kept.reserve(blobs.size());
	for (const BlobPoint &b : blobs) {
		const double r = (double)(radius_factor * b.size);
		bool drop = false;
		for (const Vec2 &p : points) {
			if (point_distance(b.x, b.y, p.x(), p.y()) < r) {
				drop = true;
				break;
			}
		}
		if (!drop) {
			kept.push_back(b);
		}
	}
	blobs = std::move(kept);
}

/*
 *
 * Tracking path.
 *
 */

TrackingBlobs
tracking_blobs(const ImageView &image, const TrackingInput &in)
{
	TrackingBlobs out;
	std::vector<Point2f> uv;
	for (size_t i = 0; i < in.predicted.size(); i++) {
		const Vec2 &p = in.predicted[i];
		if (p.x() >= 0.0 && p.x() <= 639.0 && p.y() >= 0.0 && p.y() <= 479.0) {
			out.in_image.push_back((int)i);
			uv.push_back(Point2f{(float)p.x(), (float)p.y()});
		}
	}
	if (uv.empty()) {
		return out;
	}

	// Window around the predictions; the stock code ignores calcSafeRect's result and keeps a
	// zeroed rectangle on failure, which the width test then rejects.
	RectF rf;
	const double margin = in.near ? in.near_margin : 0.0;
	calc_safe_rect(min_area_rect(uv), image.width, image.height, margin, rf);
	if (rf.width <= 3.0f) {
		return out;
	}
	out.roi = round_rect(rf);
	out.detected = true;
	const ImageView win = image.window(out.roi);

	const double d = in.t_hm.norm();
	std::vector<BlobPoint> blobs;
	if (in.near) {
		out.threshold = 250;
		blobs = detect_blobs(win, false, 250);
		float sum = 0.0f;
		for (const BlobPoint &b : blobs) {
			sum += b.size;
		}
		// No or small blobs at 250: the controller is not that bright, use the local mean.
		if (sum / ((double)blobs.size() + kMeanEpsilon) < 10.0) {
			out.adaptive = true;
			out.threshold = 0;
			blobs = detect_blobs(win, true, 70);
		}
	} else if (d < 0.4) {
		out.threshold = 150;
		blobs = detect_blobs(win, false, 150);
	} else {
		out.adaptive = true;
		blobs = detect_blobs(win, true, 70);
		// Far and straight ahead: only round blobs.
		if (std::fabs(in.t_hm.z()) > 0.3 && std::fabs(in.t_hm.y()) < 0.15 && std::fabs(in.t_hm.x()) < 0.15) {
			blobs.erase(std::remove_if(blobs.begin(), blobs.end(),
			                           [](const BlobPoint &b) { return !((double)b.circularity >= 0.4); }),
			            blobs.end());
		}
	}

	refound_blob_points(win, blobs, in.near);
	blobs = shifted(std::move(blobs), (float)out.roi.x, (float)out.roi.y);

	if (in.near) {
		// Near: a blob must be at least its size away from every image border.
		blobs.erase(std::remove_if(blobs.begin(), blobs.end(),
		                           [](const BlobPoint &b) {
			                           return !(b.x >= b.size && 639.0f - b.x >= b.size && b.y >= b.size &&
			                                    479.0f - b.y >= b.size);
		                           }),
		            blobs.end());
	}

	out.window_blobs = blobs;
	if (in.remove_other) {
		remove_blobs_near_points(blobs, in.other_points, 1.0f);
	}
	out.blobs = std::move(blobs);
	return out;
}

/*
 *
 * (Re)initialisation path.
 *
 */

bool
scene_is_bright(const ImageView &cam3, int threshold, double *mean_out)
{
	auto mean = [&](int x0, int y0) {
		uint64_t sum = 0;
		for (int y = y0; y < y0 + 120; y++) {
			for (int x = x0; x < x0 + 160; x++) {
				sum += cam3.at(x, y);
			}
		}
		return (double)sum / (160.0 * 120.0);
	};
	if (cam3.width < 639 || cam3.height < 479) {
		return false;
	}
	// Top-left corner and (479, 359, 160, 120), which ends one pixel short of the bottom right.
	const double m1 = mean(0, 0), m2 = mean(479, 359);
	if (mean_out != nullptr) {
		*mean_out = (m1 + m2) / 2.0;
	}
	return m1 > threshold && m2 > threshold;
}

std::vector<BlobPoint>
detect_init_blobs(const ImageView &full, bool bright)
{
	return detect_blobs(full, bright, 70);
}

void
delete_some_blob_points(const ImageView &full, std::vector<BlobPoint> &blobs, bool near)
{
	std::vector<BlobPoint> kept;
	kept.reserve(blobs.size());
	for (const BlobPoint &b : blobs) {
		const int bright = neighbour_max_brightness(full, b.x, b.y);
		const float c = b.circularity, s = b.size, half = s / 2.0f;
		bool drop = false;
		if (c < 0.25f && !near) {
			drop = true;
		} else if ((double)c < 0.7 && s > 10.0f &&
		           // Stock quirk: 640 - y where 640 - x is meant; the right border never fires.
		           ((640.0f - b.y) < half || b.x < half || b.y < half || (480.0f - b.y) < half)) {
			drop = true;
		} else if (bright <= 239 && (double)c < 0.3) {
			drop = true;
		} else if (bright <= 234 && s > 5.0f) {
			drop = true;
		} else if (bright < 80) {
			drop = true;
		}
		if (!drop) {
			kept.push_back(b);
		}
	}
	blobs = std::move(kept);
}

bool
cluster_predicate(ClusterPredicate p, const BlobPoint &a, const BlobPoint &b)
{
	const float m = std::max(a.size, b.size);
	float cut = 7.0f;
	if (p != ClusterPredicate::kA) {
		// Stricter near the top of the image.
		if (a.y < 120.0f || b.y < 120.0f) {
			cut = 7.0f;
		} else if (a.y < 240.0f || b.y < 240.0f) {
			cut = 10.0f;
		} else {
			cut = 15.0f;
		}
	}
	const float hi = p == ClusterPredicate::kC ? 250.0f : 150.0f;
	const float cap = m > cut ? hi : 50.0f;
	return dist_p2p(a, b) < std::min(10.0f * m, cap);
}

std::vector<std::vector<BlobPoint>>
classify_blob_points(const ImageView &full, const std::vector<BlobPoint> &blobs, bool bright)
{
	const int n = (int)blobs.size();
	auto run = [&](ClusterPredicate p, int &classes) {
		return partition_labels(
		    n, [&](int i, int j) { return cluster_predicate(p, blobs[(size_t)i], blobs[(size_t)j]); }, &classes);
	};
	int classes = 0;
	std::vector<int> labels = run(bright ? ClusterPredicate::kA : ClusterPredicate::kB, classes);

	if (n > 0) {
		double size = 0, light = 0;
		for (const BlobPoint &b : blobs) {
			size += b.size;
			light += neighbour_max_brightness(full, b.x, b.y);
		}
		// Large, saturated blobs everywhere (a near controller): wider clusters.
		if (size / n > 10.0 && light / n > 240.0) {
			labels = run(ClusterPredicate::kC, classes);
		}
	}

	std::vector<std::vector<BlobPoint>> groups((size_t)classes);
	for (int i = 0; i < n; i++) {
		groups[(size_t)labels[(size_t)i]].push_back(blobs[(size_t)i]);
	}
	return groups;
}

std::vector<BlobPoint>
refind_blob_points(const ImageView &full, const std::vector<BlobPoint> &blobs, bool bright, bool near)
{
	std::vector<BlobPoint> out;
	for (const std::vector<BlobPoint> &g : classify_blob_points(full, blobs, bright)) {
		// Stock quirk: clusters of a single blob are dropped entirely.
		if (g.size() < 2) {
			continue;
		}
		double size = 0, light = 0;
		for (const BlobPoint &b : g) {
			size += b.size;
			light += neighbour_max_brightness(full, b.x, b.y);
		}
		size /= (double)g.size();
		light /= (double)g.size();
		const bool big_bright = size > 10.0 && light > 240.0;

		RectF rf;
		if (!calc_safe_rect(min_area_rect(centres(g)), full.width, full.height, big_bright ? 100.0 : 15.0, rf)) {
			continue;
		}
		const RectI roi = round_rect(rf);
		if (roi.width <= 0 || roi.height <= 0) {
			continue;
		}
		const ImageView win = full.window(roi);
		std::vector<BlobPoint> found;
		if (big_bright) {
			found = detect_blobs(win, false, 210);
		} else if (!near || size < 10.0) {
			found = detect_blobs(win, true, 70);
		} else {
			found = detect_blobs(win, false, 150);
		}
		for (const BlobPoint &b : shifted(std::move(found), (float)roi.x, (float)roi.y)) {
			out.push_back(b);
		}
	}
	return out;
}

void
check_blob_points(std::vector<std::vector<BlobPoint>> &groups)
{
	for (std::vector<BlobPoint> &g : groups) {
		const int n = (int)g.size();
		if (n == 0) {
			continue;
		}
		std::vector<BlobPoint> sorted = g;
		// Ascending size with the stock std::sort of BlobPoint (0x3d4608; insertion sort up to 30).
		stock_sort(sorted.begin(), sorted.end(), [](const BlobPoint &a, const BlobPoint &b) { return a.size < b.size; }, 30);
		const int max_size = (int)sorted.back().size;
		if (n != 5 && n >= 4 && max_size >= 16) {
			// Drop blobs much smaller than the largest, but keep enough for P5P (5) or P3P (3).
			std::vector<BlobPoint> kept;
			for (int i = 0; i < n; i++) {
				const BlobPoint &b = sorted[(size_t)i];
				const int total = (n - i) + (int)kept.size();
				if ((double)b.size > 0.4 * max_size || total == 5 || total == 3) {
					kept.push_back(b);
				}
			}
			g = std::move(kept);
		}
		// Near-duplicates: drop a blob if a later one lies within 1 px.
		std::vector<BlobPoint> unique;
		for (size_t i = 0; i < g.size(); i++) {
			bool dup = false;
			for (size_t j = i + 1; j < g.size() && !dup; j++) {
				dup = dist_p2p(g[i], g[j]) < 1.0f;
			}
			if (!dup) {
				unique.push_back(g[i]);
			}
		}
		g = std::move(unique);
	}
}

void
check_blob_points_brightness(const ImageView &full, std::vector<std::vector<BlobPoint>> &groups, int led_count)
{
	std::vector<std::vector<BlobPoint>> out;
	for (std::vector<BlobPoint> g : groups) {
		if (g.size() < 3 || g.size() > 30) {
			continue;
		}
		// cmpByCircularity with the stock std::sort of BlobPoint (0x3744a0; insertion sort up to 30).
		stock_sort(g.begin(), g.end(), [](const BlobPoint &a, const BlobPoint &b) { return a.circularity > b.circularity; },
		           30);
		std::vector<std::pair<BlobPoint, int>> lit;
		int saturated = 0;
		for (const BlobPoint &b : g) {
			const int v = neighbour_max_brightness(full, b.x, b.y);
			lit.emplace_back(b, v);
			saturated += v >= 251 ? 1 : 0;
		}
		// More blobs than LEDs without enough saturated ones: not a controller.
		if (saturated <= 14 && (int)g.size() > led_count) {
			continue;
		}
		// cmpByBrightNess with the stock std::sort of pair<BlobPoint, int> (0x37488c; insertion sort
		// only up to 6): equal brightnesses (255 is common) end up in its order.
		stock_sort(lit.begin(), lit.end(), [](const auto &a, const auto &b) { return a.second > b.second; }, 6);

		// The three brightest must be separate LEDs: dark between neighbours along the long axis.
		std::vector<BlobPoint> three;
		for (size_t i = 0; i < 3; i++) {
			three.push_back(lit[i].first);
		}
		sort_blob_points(three);
		bool separate = true;
		for (size_t i = 0; i + 1 < three.size() && separate; i++) {
			const BlobPoint &a = three[i], &b = three[i + 1];
			const int mx = (int)std::lround((a.x + b.x) / 2.0f);
			const int my = (int)std::lround((a.y + b.y) / 2.0f);
			const int mid = (pixel_clamped(full, mx, my) + pixel_clamped(full, mx - 1, my) +
			                 pixel_clamped(full, mx + 1, my) + pixel_clamped(full, mx, my - 1) +
			                 pixel_clamped(full, mx, my + 1)) /
			                5;
			const int dimmer =
			    std::min(neighbour_max_brightness(full, a.x, a.y), neighbour_max_brightness(full, b.x, b.y));
			separate = dimmer - mid > 79;
		}
		if (!separate) {
			continue;
		}
		std::vector<BlobPoint> ordered;
		for (const auto &p : lit) {
			ordered.push_back(p.first);
		}
		out.push_back(std::move(ordered));
	}
	groups = std::move(out);
}

std::vector<int>
two_means(const std::vector<Point2f> &pts)
{
	const size_t n = pts.size();
	std::vector<int> best_labels(n, 0);
	if (n < 2) {
		return best_labels;
	}
	Mwc rng;
	double best_compactness = std::numeric_limits<double>::infinity();
	auto d2 = [](const Point2f &a, double x, double y) { return squared(a.x - x) + squared(a.y - y); };

	for (int attempt = 0; attempt < 3; attempt++) {
		// k-means++ seeding: the second centre is drawn by squared distance, best of 3 trials.
		double cx[2], cy[2];
		const size_t first = rng.next() % n;
		cx[0] = pts[first].x;
		cy[0] = pts[first].y;
		std::vector<double> dist(n);
		double sum = 0;
		for (size_t i = 0; i < n; i++) {
			dist[i] = d2(pts[i], cx[0], cy[0]);
			sum += dist[i];
		}
		double best_sum = std::numeric_limits<double>::infinity();
		size_t second = first;
		for (int trial = 0; trial < 3; trial++) {
			double r = rng.uniform01() * sum;
			size_t idx = 0;
			for (; idx + 1 < n; idx++) {
				if ((r -= dist[idx]) <= 0) {
					break;
				}
			}
			double s = 0;
			for (size_t i = 0; i < n; i++) {
				s += std::min(dist[i], d2(pts[i], pts[idx].x, pts[idx].y));
			}
			if (s < best_sum) {
				best_sum = s;
				second = idx;
			}
		}
		cx[1] = pts[second].x;
		cy[1] = pts[second].y;

		std::vector<int> labels(n, 0);
		auto assign = [&]() {
			for (size_t i = 0; i < n; i++) {
				labels[i] = d2(pts[i], cx[1], cy[1]) < d2(pts[i], cx[0], cy[0]) ? 1 : 0;
			}
		};
		for (int iter = 0; iter < 10; iter++) {
			assign();
			double sx[2] = {0, 0}, sy[2] = {0, 0};
			int cnt[2] = {0, 0};
			for (size_t i = 0; i < n; i++) {
				sx[labels[i]] += pts[i].x;
				sy[labels[i]] += pts[i].y;
				cnt[labels[i]]++;
			}
			double shift = 0;
			for (int k = 0; k < 2; k++) {
				double nx = cx[k], ny = cy[k];
				if (cnt[k] > 0) {
					nx = sx[k] / cnt[k];
					ny = sy[k] / cnt[k];
				} else {
					// Empty cluster: take the point farthest from the other centre.
					size_t far = 0;
					for (size_t i = 1; i < n; i++) {
						if (d2(pts[i], cx[1 - k], cy[1 - k]) > d2(pts[far], cx[1 - k], cy[1 - k])) {
							far = i;
						}
					}
					nx = pts[far].x;
					ny = pts[far].y;
				}
				shift = std::max(shift, squared(nx - cx[k]) + squared(ny - cy[k]));
				cx[k] = nx;
				cy[k] = ny;
			}
			if (shift <= 0.1 * 0.1) {
				break;
			}
		}
		assign();
		double compactness = 0;
		for (size_t i = 0; i < n; i++) {
			compactness += d2(pts[i], cx[labels[i]], cy[labels[i]]);
		}
		if (compactness < best_compactness) {
			best_compactness = compactness;
			best_labels = labels;
		}
	}
	return best_labels;
}

void
delete_redundant_points(const ImageView &full, std::vector<std::vector<BlobPoint>> &groups)
{
	for (std::vector<BlobPoint> &g : groups) {
		const size_t n = g.size();
		if (n < 10 || n > 15) {
			continue;
		}
		// sortBlobPoints runs on a copy (0x375888); the cluster itself is replaced only when it
		// is thinned out, and otherwise keeps its order.
		std::vector<BlobPoint> s = g;
		sort_blob_points(s);
		const RotatedRect rr = min_area_rect(centres(s));
		const float aspect =
		    (rr.width == 0.0f || rr.height == 0.0f) ? 1.0f : std::max(rr.width / rr.height, rr.height / rr.width);

		if (aspect < 2.0f) {
			// Compact: two controllers' worth of blobs? Keep the tighter half.
			const std::vector<int> labels = two_means(centres(s));
			std::vector<BlobPoint> half[2];
			for (size_t i = 0; i < n; i++) {
				half[labels[i]].push_back(s[i]);
			}
			if (half[0].empty()) {
				g = half[1];
			} else if (half[1].empty()) {
				g = half[0];
			} else {
				const RotatedRect r0 = min_area_rect(centres(half[0]));
				const RotatedRect r1 = min_area_rect(centres(half[1]));
				const float a0 = r0.width * r0.height, a1 = r1.width * r1.height;
				// Equal areas leave the cluster unchanged (fcmp + b.pl, b.le at 0x376064..0x3760c0).
				if (a0 < a1) {
					if (half[0].size() >= 4) {
						g = half[0];
					}
				} else if (a0 > a1) {
					if (half[1].size() >= 4) {
						g = half[1];
					}
				}
			}
			continue;
		}

		// Elongated: the rectangle edge the blobs hug (float sums, the first of equal sums),
		// then drop the farthest outliers.
		Point2f c[4];
		rr.points(c);
		float best = 9999.0f;
		int edge = -1;
		for (int e = 0; e < 4; e++) {
			float sum = 0.0f;
			for (const BlobPoint &b : s) {
				sum += dist_p2l(Point2f{b.x, b.y}, c[e], c[(e + 1) % 4]);
			}
			if (sum < best) {
				best = sum;
				edge = e;
			}
		}
		if (edge < 0) {
			continue;
		}
		// (blob index, float distance) in index order, sorted with the stock std::sort of
		// pair<int, float> (0x37688c).
		std::vector<std::pair<int, float>> by_dist;
		for (size_t i = 0; i < n; i++) {
			by_dist.emplace_back((int)i, dist_p2l(Point2f{s[i].x, s[i].y}, c[edge], c[(edge + 1) % 4]));
		}
		stock_sort(by_dist.begin(), by_dist.end(), [](const auto &a, const auto &b) { return a.second < b.second; }, 6);
		const int k = (int)n / 2 + 1;
		int running = 0;
		for (int i = 0; i < k; i++) {
			// Stock: a float addition truncated to an integer after each step (0x37696c).
			running = (int)(by_dist[(size_t)i].second + (float)running);
		}
		const double limit = 1.2 * (double)((float)running / (float)k);

		size_t keep = n;
		int bright = 0;
		for (int i = (int)n - 1; i >= 0; i--) {
			const BlobPoint &b = s[(size_t)by_dist[(size_t)i].first];
			if (neighbour_max_brightness(full, b.x, b.y) > 250) {
				bright++;
			}
			// This blob already counts towards the four bright ones (cinc + b.hi at 0x376c34).
			if (bright > 3) {
				break;
			}
			if ((double)by_dist[(size_t)i].second > limit && keep + 4 > n) {
				// Stock quirk: removes the last remaining entry, which is this blob only while
				// every blob after it was removed (always so with the ascending order).
				keep--;
			}
		}
		std::vector<BlobPoint> rest;
		for (size_t i = 0; i < keep; i++) {
			rest.push_back(s[(size_t)by_dist[i].first]);
		}
		g = std::move(rest);
	}
}

InitBlobs
prepare_init_clusters(const ImageView &full, const std::vector<BlobPoint> &detected, const InitInput &in)
{
	InitBlobs out;
	std::vector<BlobPoint> blobs = detected;

	if (in.exclude_other) {
		std::vector<Vec2> other;
		for (const Vec2 &p : in.other_projected) {
			if (p.x() >= 0.0 && p.x() <= 639.0 && p.y() >= 0.0 && p.y() <= 479.0) {
				other.push_back(p);
			}
		}
		remove_blobs_near_points(blobs, other, in.other_fast ? 3.0f : 1.0f);
	}

	out.static_hits.assign(in.static_lights.size(), 0);
	remove_static_lights(blobs, in.static_lights, &out.static_hits);

	if (!in.bright) {
		// Fixed threshold 70 merges neighbours: re-detect per cluster with its own threshold.
		delete_some_blob_points(full, blobs, in.near);
		blobs = refind_blob_points(full, blobs, false, in.near);
	}
	remove_static_lights(blobs, in.static_lights, nullptr);
	delete_some_blob_points(full, blobs, in.near);
	out.blobs = blobs;

	// The stock code forces the bright flag here: predicate P_A (or P_C).
	std::vector<std::vector<BlobPoint>> groups = classify_blob_points(full, blobs, true);
	groups.erase(std::remove_if(groups.begin(), groups.end(),
	                            [](const std::vector<BlobPoint> &g) { return g.size() < 3; }),
	             groups.end());
	check_blob_points(groups);
	check_blob_points_brightness(full, groups, in.led_count);
	std::vector<std::vector<BlobPoint>> filtered = groups;
	delete_redundant_points(full, filtered);

	std::vector<std::pair<std::vector<BlobPoint>, std::vector<BlobPoint>>> pairs;
	for (size_t i = 0; i < groups.size(); i++) {
		pairs.emplace_back(filtered[i], groups[i]);
	}
	// Larger filtered clusters first: the inlined comparator 0x3dc0e0..0x3dc110 is
	// size(a.first) > size(b.first) (cmp + b.ls to the "not less" branch). Both sorts are the
	// stock std::sort of these pairs (insertion sort only up to 6).
	stock_sort(pairs.begin(), pairs.end(), [](const auto &a, const auto &b) { return a.first.size() > b.first.size(); },
	           6);
	// setBolbsPriority (0x385da8): by the filtered cluster's centroid x, descending for the
	// right hand (P+0x1924 != 0, 0x3daa64) and ascending for the left (0x3d9868), so each hand
	// starts on its side.
	const bool descending = in.hand != 0;
	std::vector<std::pair<float, size_t>> by_x;
	for (size_t i = 0; i < pairs.size(); i++) {
		by_x.emplace_back(mean_x(pairs[i].first), i);
	}
	stock_sort(by_x.begin(), by_x.end(),
	           [descending](const auto &a, const auto &b) { return descending ? a.first > b.first : a.first < b.first; },
	           6);
	{
		std::vector<std::pair<std::vector<BlobPoint>, std::vector<BlobPoint>>> ordered;
		ordered.reserve(pairs.size());
		for (const auto &k : by_x) {
			ordered.push_back(std::move(pairs[k.second]));
		}
		pairs = std::move(ordered);
	}

	for (auto &p : pairs) {
		if (p.first.size() < 3) {
			continue;
		}
		InitCluster c;
		c.blobs = std::move(p.first);
		c.original = std::move(p.second);
		for (const BlobPoint &b : c.blobs) {
			c.n255 += neighbour_max_brightness(full, b.x, b.y) > 254 ? 1 : 0;
		}
		c.juxtaposed = c.n255 >= 14 && c.blobs.size() >= 17;
		if (c.juxtaposed) {
			// By y with the stock std::sort of BlobPoint (0x3dd248; insertion sort up to 30).
			stock_sort(c.blobs.begin(), c.blobs.end(), [](const BlobPoint &a, const BlobPoint &b) { return a.y < b.y; },
			           30);
			c.solver = InitSolver::kJuxtaposedP3P;
		} else {
			// Every other cluster is ordered along its long axis before the size test, the P3P
			// ones included (0x389694..0x389698, then |cluster| >= 5 at 0x389830).
			sort_blob_points(c.blobs);
			c.solver = c.blobs.size() >= 5 ? InitSolver::kP5P : InitSolver::kP3P;
		}
		out.clusters.push_back(std::move(c));
	}
	return out;
}

} // namespace pico4::ctrack
