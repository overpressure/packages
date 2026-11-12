// Copyright 2026, overpressure contributors.
// SPDX-License-Identifier: BSL-1.0
/*
 * Unit test of the controller LED blob detection on synthetic images: the box mean and its
 * rounding, border following (order, holes, 8-connectivity), polygon moments and the
 * detector's filters, fixed and local-mean thresholds on neighbouring LEDs, splitting of merged
 * blobs, the rotated-rectangle helpers, the tracking path's window, thresholds and removals,
 * and the init path's filters, clustering and cluster order.
 */

#include "pico4_ctrack_blob.hpp"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <algorithm>
#include <vector>

static const double kPi = 3.14159265358979323846;

using namespace pico4::ctrack;

static int failures;

#define EXPECT(cond)                                                                                                   \
	do {                                                                                                           \
		if (!(cond)) {                                                                                         \
			fprintf(stderr, "%s:%d: expected %s\n", __FILE__, __LINE__, #cond);                            \
			failures++;                                                                                    \
		}                                                                                                      \
	} while (0)

#define EXPECT_NEAR(a, b, tol) EXPECT(std::fabs((double)(a) - (double)(b)) <= (tol))

//! A synthetic 8-bit image.
struct Img
{
	int w, h;
	std::vector<uint8_t> px;

	Img(int w_ = 640, int h_ = 480, uint8_t fill = 0) : w(w_), h(h_), px((size_t)w_ * (size_t)h_, fill) {}

	ImageView
	view() const
	{
		return ImageView{px.data(), w, h, w};
	}

	uint8_t &
	at(int x, int y)
	{
		return px[(size_t)y * (size_t)w + (size_t)x];
	}

	void
	disc(double cx, double cy, double r, uint8_t v)
	{
		for (int y = (int)std::floor(cy - r) - 1; y <= (int)std::ceil(cy + r) + 1; y++) {
			for (int x = (int)std::floor(cx - r) - 1; x <= (int)std::ceil(cx + r) + 1; x++) {
				if (x < 0 || y < 0 || x >= w || y >= h) {
					continue;
				}
				if ((x - cx) * (x - cx) + (y - cy) * (y - cy) <= r * r && at(x, y) < v) {
					at(x, y) = v;
				}
			}
		}
	}

	void
	rect(int x0, int y0, int rw, int rh, uint8_t v)
	{
		for (int y = y0; y < y0 + rh; y++) {
			for (int x = x0; x < x0 + rw; x++) {
				at(x, y) = v;
			}
		}
	}

	//! Add a Gaussian spot (saturating).
	void
	gauss(double cx, double cy, double sigma, double peak)
	{
		for (int y = 0; y < h; y++) {
			for (int x = 0; x < w; x++) {
				const double d2 = (x - cx) * (x - cx) + (y - cy) * (y - cy);
				if (d2 > 25 * sigma * sigma) {
					continue;
				}
				const double v = at(x, y) + peak * std::exp(-d2 / (2 * sigma * sigma));
				at(x, y) = (uint8_t)std::min(255.0, std::round(v));
			}
		}
	}
};

//! A zero-framed 0/1 buffer from rows of '#' and '.'.
static std::vector<int8_t>
buffer_from(const std::vector<const char *> &rows, int &w, int &h)
{
	const int iw = (int)std::string(rows[0]).size(), ih = (int)rows.size();
	w = iw + 2;
	h = ih + 2;
	std::vector<int8_t> buf((size_t)w * (size_t)h, 0);
	for (int y = 0; y < ih; y++) {
		for (int x = 0; x < iw; x++) {
			buf[(size_t)(y + 1) * (size_t)w + (size_t)(x + 1)] = rows[(size_t)y][x] == '#' ? 1 : 0;
		}
	}
	return buf;
}

static std::vector<std::vector<Point2i>>
contours_of(const std::vector<const char *> &rows, ChainApprox approx = ChainApprox::kNone)
{
	int w, h;
	std::vector<int8_t> buf = buffer_from(rows, w, h);
	return find_outer_contours(buf.data(), w, h, approx, Point2i{-1, -1});
}

static BlobPoint
blob(float x, float y, float size, float circ = 0.9f)
{
	BlobPoint b;
	b.x = x;
	b.y = y;
	b.size = size;
	b.circularity = circ;
	return b;
}

static void
test_box_mean()
{
	Img flat(40, 30, 77);
	for (uint8_t v : box_mean(flat.view(), 11)) {
		EXPECT(v == 77);
	}

	// The sliding sums match a direct sum with replicated edges, divided in 16.16 fixed point.
	Img noise(37, 23);
	uint32_t s = 1;
	for (uint8_t &v : noise.px) {
		s = s * 1664525u + 1013904223u;
		v = (uint8_t)(s >> 24);
	}
	for (int block : {5, 11}) {
		const int scale = block == 11 ? 542 : 2621, bias = block == 11 ? 60 : 13;
		const std::vector<uint8_t> m = box_mean(noise.view(), block);
		bool same = true;
		for (int y = 0; y < noise.h; y++) {
			for (int x = 0; x < noise.w; x++) {
				int sum = 0;
				for (int j = -block / 2; j <= block / 2; j++) {
					for (int i = -block / 2; i <= block / 2; i++) {
						sum += noise.at(std::min(std::max(x + i, 0), noise.w - 1),
						                std::min(std::max(y + j, 0), noise.h - 1));
					}
				}
				same = same && m[(size_t)y * (size_t)noise.w + (size_t)x] == (uint8_t)((sum + bias) * scale >> 16);
			}
		}
		EXPECT(same);
	}
	// The fixed-point division rounds 1391 / 121 = 11.496 up to 12, unlike round().
	EXPECT(((1391 + 60) * 542 >> 16) == 12);

	// A window never reads outside itself: a bright column just outside does not leak in.
	Img wide(30, 30, 10);
	for (int y = 0; y < 30; y++) {
		wide.at(20, y) = 250;
	}
	const ImageView left = wide.view().window(RectI{0, 0, 20, 30});
	for (uint8_t v : box_mean(left, 11)) {
		EXPECT(v == 10);
	}
}

static void
test_contours()
{
	// A 3x3 square: starts top-left, down the left side first, every border pixel once.
	auto c = contours_of({"....", ".###", ".###", ".###"});
	EXPECT(c.size() == 1);
	if (c.size() == 1) {
		const int expect[8][2] = {{1, 1}, {1, 2}, {1, 3}, {2, 3}, {3, 3}, {3, 2}, {3, 1}, {2, 1}};
		EXPECT(c[0].size() == 8);
		for (size_t i = 0; i < 8 && i < c[0].size(); i++) {
			EXPECT(c[0][i].x == expect[i][0] && c[0][i].y == expect[i][1]);
		}
	}
	// The same with direction-change points only: the four corners.
	c = contours_of({"###", "###", "###"}, ChainApprox::kSimple);
	EXPECT(c.size() == 1 && c[0].size() == 4);

	// An isolated pixel is a one-point contour.
	c = contours_of({"...", ".#.", "..."});
	EXPECT(c.size() == 1 && c[0].size() == 1 && c[0][0].x == 1 && c[0][0].y == 1);

	// A one-pixel-wide line is traced there and back (repeats included).
	c = contours_of({"####"});
	EXPECT(c.size() == 1 && c[0].size() == 6);

	// Diagonal neighbours are connected (8-connectivity).
	c = contours_of({"##..", "##..", "..##", "..##"});
	EXPECT(c.size() == 1);

	// A blob inside a hole is skipped; a blob to the right of the ring on the same rows is not.
	c = contours_of({
	    "#######......",
	    "#.....#......",
	    "#.....#...##.",
	    "#..#..#...##.",
	    "#.....#......",
	    "#.....#......",
	    "#######......",
	});
	EXPECT(c.size() == 2);
	if (c.size() == 2) {
		EXPECT(c[0][0].x == 0 && c[0][0].y == 0);
		EXPECT(c[1][0].x == 10 && c[1][0].y == 2);
	}

	// Raster order of each blob's first pixel.
	c = contours_of({"....##", "##..##", "##...."});
	EXPECT(c.size() == 2 && c[0][0].x == 4 && c[1][0].x == 0);
}

static void
test_moments()
{
	auto area_of = [](const std::vector<const char *> &rows) {
		auto c = contours_of(rows);
		return c.empty() ? -1.0 : contour_moments(c[0]).m00;
	};
	EXPECT_NEAR(area_of({"#"}), 0.0, 1e-12);
	EXPECT_NEAR(area_of({"#####"}), 0.0, 1e-12);
	EXPECT_NEAR(area_of({"##", "##"}), 1.0, 1e-12);
	EXPECT_NEAR(area_of({"#.", "##"}), 0.5, 1e-12);
	EXPECT_NEAR(area_of({"###", "###", "###"}), 4.0, 1e-12);

	auto c = contours_of({"###", "###", "###"});
	const ContourMoments m = contour_moments(c[0]);
	EXPECT_NEAR(m.m10 / m.m00, 1.0, 1e-12);
	EXPECT_NEAR(m.m01 / m.m00, 1.0, 1e-12);
	EXPECT_NEAR(closed_arc_length(c[0]), 8.0, 1e-12);
	EXPECT_NEAR(polygon_area(convex_hull(c[0])), 4.0, 1e-12);
}

static void
test_filters()
{
	// Area: the adaptive block-11 limit is 3000, the fixed limit 9 x 3000.
	std::vector<Point2i> square;
	for (int i = 0; i < 60; i++) {
		square.push_back(Point2i{0, i});
	}
	for (int i = 0; i < 60; i++) {
		square.push_back(Point2i{i, 60});
	}
	for (int i = 60; i > 0; i--) {
		square.push_back(Point2i{60, i});
	}
	for (int i = 60; i > 0; i--) {
		square.push_back(Point2i{i, 0});
	}
	const DetectorParams params;
	EXPECT(filter_contours({square}, params, 11).empty());
	EXPECT(filter_contours({square}, params, 5).size() == 1);
	EXPECT(filter_contours({square}, params, 0).size() == 1);

	// Circularity >= 0.1: a 2-pixel-wide bar of 40 fails (0.077), a 3-pixel-wide one passes (0.146).
	Img bars(100, 100);
	bars.rect(10, 10, 40, 2, 200);
	bars.rect(10, 50, 40, 3, 200);
	const std::vector<BlobPoint> b = detect_blobs(bars.view(), false, 100);
	EXPECT(b.size() == 1);
	if (b.size() == 1) {
		EXPECT_NEAR(b[0].y, 51.0, 1e-4);
		EXPECT_NEAR(b[0].area, 78.0, 1e-4);
		EXPECT_NEAR(b[0].circularity, 12.566370614359172 * 78.0 / (82.0 * 82.0), 1e-5);
	}
}

static void
test_detect_fixed()
{
	Img img;
	img.disc(100, 100, 4, 200);
	img.disc(130, 90, 4, 200);
	img.rect(300, 300, 1, 1, 255); // one pixel: area 0, dropped
	img.rect(400, 300, 2, 2, 255); // 2 x 2: area 1
	img.rect(500, 300, 3, 3, 150); // exactly at the threshold: not set
	img.rect(200, 200, 3, 3, 200); // two squares touching diagonally: one blob
	img.rect(203, 203, 3, 3, 200);
	const std::vector<BlobPoint> b = detect_blobs(img.view(), false, 150);
	EXPECT(b.size() == 4);
	if (b.size() == 4) {
		// Raster order of the first pixel: (130, 86) before (100, 96).
		EXPECT_NEAR(b[0].x, 130.0, 1e-4);
		EXPECT_NEAR(b[0].y, 90.0, 1e-4);
		EXPECT_NEAR(b[1].x, 100.0, 1e-4);
		EXPECT(b[0].size > 6.0f && b[0].size < 9.0f);
		EXPECT(b[0].circularity > 0.7f);
		EXPECT(b[0].brightness == -1 && b[0].color == 255.0f);
		EXPECT_NEAR(b[2].x, 202.5, 1e-4);
		EXPECT_NEAR(b[3].area, 1.0, 1e-6);
		EXPECT_NEAR(b[3].x, 400.5, 1e-4);
	}

	// Windows give window coordinates.
	const std::vector<BlobPoint> w = detect_blobs(img.view().window(RectI{90, 80, 50, 30}), false, 150);
	EXPECT(w.size() == 2 && std::fabs(w[0].x - 40.0f) < 1e-4f && std::fabs(w[0].y - 10.0f) < 1e-4f);

	// Oversized input is refused (the stock buffer is 642 x 482).
	Img big(641, 10);
	EXPECT(detect_blobs(big.view(), false, 10).empty());
}

static void
test_detect_adaptive()
{
	// Two LEDs 7 px apart in a common glow: a threshold of 70 merges them, the local mean does not.
	Img img;
	img.gauss(200, 200, 6.0, 120);
	img.gauss(196.5, 200, 1.0, 255);
	img.gauss(203.5, 200, 1.0, 255);
	const std::vector<BlobPoint> fixed = detect_blobs(img.view(), false, 70);
	EXPECT(fixed.size() == 1);
	const std::vector<BlobPoint> local = detect_blobs(img.view(), true, 70);
	EXPECT(local.size() == 2);
	if (local.size() == 2) {
		EXPECT_NEAR(local[0].x + local[1].x, 400.0, 0.2);
		EXPECT_NEAR(std::fabs(local[0].x - local[1].x), 7.0, 0.6);
	}
	// A uniform bright area is not a blob in adaptive mode.
	Img wall(100, 100, 0);
	wall.rect(20, 20, 60, 60, 120);
	for (const BlobPoint &b : detect_blobs(wall.view(), true, 70)) {
		EXPECT(b.circularity < 0.1f || b.area < 3000.0f);
	}
	// Small windows use a 5 x 5 block.
	int block = 0;
	(void)binarise(Img(15, 40).view(), true, 0, &block);
	EXPECT(block == 5);
	(void)binarise(Img(16, 16).view(), true, 0, &block);
	EXPECT(block == 11);
}

static void
test_geometry()
{
	// The stock cv::minAreaRect representation (OpenCV 3.4.0 in the library): an axis-aligned
	// box comes out at -90 degrees with the sides swapped, a triangle as the last of its equal
	// rectangles; golden values from the machine code (0x8643f4).
	RotatedRect r = min_area_rect({{0, 0}, {10, 1}, {5, 8}});
	EXPECT_NEAR(r.angle, -84.2894058, 1e-4);
	EXPECT_NEAR(r.width, 7.46277905, 1e-5);
	EXPECT_NEAR(r.height, 10.0498753, 1e-5);
	EXPECT_NEAR(r.center.x, 4.62871313, 1e-5);
	EXPECT_NEAR(r.center.y, 4.21287155, 1e-5);
	Point2f corner[4];
	r.points(corner);
	EXPECT_NEAR(corner[0].x, 9.25742626, 1e-4);
	EXPECT_NEAR(corner[0].y, 8.4257431, 1e-4);
	EXPECT_NEAR(corner[2].x, 0.0, 1e-4);
	EXPECT_NEAR(corner[3].y, 1.0, 1e-4);

	r = min_area_rect({{0, 0}, {10, 0}, {10, 4}, {0, 4}, {5, 2}});
	EXPECT(r.angle == -90.0f && r.width == 4.0f && r.height == 10.0f);
	EXPECT_NEAR(r.width * r.height, 40.0, 1e-3);
	EXPECT_NEAR(r.center.x, 5.0, 1e-4);
	EXPECT_NEAR(r.center.y, 2.0, 1e-4);
	RectI br = r.bounding_rect();
	// Floating-point noise of the corners can widen it by a pixel (as in OpenCV).
	EXPECT(br.x >= -1 && br.x <= 0 && br.y >= -1 && br.y <= 0);
	EXPECT(br.x + br.width >= 11 && br.x + br.width <= 12 && br.y + br.height >= 5 && br.y + br.height <= 6);

	// A square turned by 30 degrees.
	std::vector<Point2f> sq;
	for (int k = 0; k < 4; k++) {
		const double a = 30.0 * kPi / 180.0 + k * kPi / 2.0;
		sq.push_back(Point2f{(float)(50 + 10 * std::cos(a)), (float)(50 + 10 * std::sin(a))});
	}
	r = min_area_rect(sq);
	EXPECT_NEAR(r.width, 10 * std::sqrt(2.0), 1e-3);
	EXPECT_NEAR(r.height, 10 * std::sqrt(2.0), 1e-3);
	EXPECT_NEAR(r.angle, -15.0, 1e-3);

	// calcSafeRect: grow by 5 (+ margin / 2), clip, the far edge loses a pixel.
	RectF out;
	EXPECT(calc_safe_rect(min_area_rect({{100, 100}}), 640, 480, 0.0, out));
	EXPECT(out.x == 95 && out.y == 95 && out.width == 10 && out.height == 10);
	EXPECT(calc_safe_rect(min_area_rect({{100, 100}}), 640, 480, 100.0, out));
	EXPECT(out.x == 45 && out.width == 110);
	EXPECT(calc_safe_rect(min_area_rect({{2, 2}}), 640, 480, 0.0, out));
	EXPECT(out.x == 0 && out.width == 7);
	EXPECT(calc_safe_rect(min_area_rect({{639, 479}}), 640, 480, 0.0, out));
	EXPECT(out.x == 634 && out.width == 5 && out.y == 474 && out.height == 5);
	RectF untouched{1, 2, 3, 4};
	EXPECT(!calc_safe_rect(min_area_rect({{700, 100}}), 640, 480, 0.0, untouched));
	EXPECT(untouched.x == 1 && untouched.width == 3);

	// Brightness: 3x3 maximum around the rounded centre, half away from zero.
	Img img;
	img.at(11, 10) = 200;
	EXPECT(neighbour_max_brightness(img.view(), 9.5f, 10.0f) == 200);
	EXPECT(neighbour_max_brightness(img.view(), 9.4f, 10.0f) == 0);
	img.at(639, 479) = 99;
	EXPECT(neighbour_max_brightness(img.view(), 700.0f, 900.0f) == 99);

	EXPECT_NEAR(dist_p2l(Point2f{5, 3}, Point2f{0, 0}, Point2f{10, 0}), 3.0, 1e-9);
	EXPECT(std::isnan(dist_p2l(Point2f{5, 3}, Point2f{1, 1}, Point2f{1, 1})));

	// Partition: a chain is one class; labels in order of first appearance.
	const std::vector<int> pos = {100, 0, 5, 10};
	int classes = 0;
	const std::vector<int> labels =
	    partition_labels(4, [&](int i, int j) { return std::abs(pos[(size_t)i] - pos[(size_t)j]) <= 5; }, &classes);
	EXPECT(classes == 2 && labels[0] == 0 && labels[1] == 1 && labels[2] == 1 && labels[3] == 1);

	// sortBlobPoints orders along the long axis.
	std::vector<BlobPoint> line = {blob(30, 15, 4), blob(0, 0, 4), blob(50, 25, 4), blob(10, 5, 4), blob(41, 19, 4)};
	sort_blob_points(line);
	const bool up = line[0].x < line[4].x;
	for (size_t i = 0; i + 1 < line.size(); i++) {
		EXPECT(up ? line[i].x < line[i + 1].x : line[i].x > line[i + 1].x);
	}
	// Three blobs: the stock rectangle (last of the equal ones, corner 0 at (9.26, 8.43)) has
	// its short edge through (10, 1), so the order is (10, 1), (5, 8), (0, 0).
	std::vector<BlobPoint> tri = {blob(0, 0, 4), blob(5, 8, 4), blob(10, 1, 4)};
	sort_blob_points(tri);
	EXPECT(tri[0].x == 10 && tri[1].x == 5 && tri[2].x == 0);
}

static void
test_stock_sort()
{
	// Sorts like std::sort.
	srand(7);
	for (int round = 0; round < 200; round++) {
		std::vector<int> v((size_t)(rand() % 1200));
		for (int &x : v) {
			x = rand() % 50;
		}
		std::vector<int> want = v;
		std::sort(want.begin(), want.end());
		stock_sort(v.begin(), v.end(), [](int a, int b) { return a < b; }, round % 2 ? 6 : 30);
		EXPECT(v == want);
	}
	// Equal keys: up to the insertion limit in input order; above it in the order of the libc++
	// introsort, traced by hand: the guard exchange of the first element with c moves b ahead of a.
	std::vector<std::pair<char, int>> e = {{'a', 1}, {'b', 1}, {'c', 0}, {'d', 1}, {'e', 1}, {'f', 1}, {'g', 1}};
	auto by_key = [](const auto &x, const auto &y) { return x.second < y.second; };
	std::vector<std::pair<char, int>> small(e.begin(), e.begin() + 6);
	stock_sort(small.begin(), small.end(), by_key, 6);
	EXPECT(small[0].first == 'c' && small[1].first == 'a' && small[2].first == 'b');
	stock_sort(e.begin(), e.end(), by_key, 6);
	std::string order;
	for (const auto &x : e) {
		order += x.first;
	}
	EXPECT(order == "cbadefg");
	std::vector<std::pair<char, int>> e30 = {{'a', 1}, {'b', 1}, {'c', 0}, {'d', 1}, {'e', 1}, {'f', 1}, {'g', 1}};
	stock_sort(e30.begin(), e30.end(), by_key, 30);
	EXPECT(e30[0].first == 'c' && e30[1].first == 'a' && e30[2].first == 'b');
}

static void
test_refound()
{
	// Four LEDs and one blob of two merged LEDs (saturated cores, a 200 bridge).
	Img img;
	for (int k = 0; k < 4; k++) {
		img.disc(100 + 30 * k, 100, 3, 255);
	}
	img.rect(296, 297, 16, 7, 200);
	img.disc(300, 300, 2, 255);
	img.disc(310, 300, 2, 255);
	std::vector<BlobPoint> b = detect_blobs(img.view(), false, 150);
	EXPECT(b.size() == 5);
	const float merged_size = b.size() == 5 ? b[4].size : 0.0f;
	// The window corner in float and as the rounded ROI the cores are searched in.
	const float left = b.size() == 5 ? b[4].x - merged_size : 0.0f;
	const float top = b.size() == 5 ? b[4].y - merged_size : 0.0f;
	const float roi_x = std::nearbyint(left), roi_y = std::nearbyint(top);
	refound_blob_points(img.view(), b, false);
	EXPECT(b.size() == 6);
	if (b.size() == 6) {
		// Bounding-box centres x + w / 2 of the 5 x 5 cores (298 + 2.5 in the image), placed
		// from the unrounded window corner (stock quirk).
		EXPECT_NEAR(b[4].x, left + (298.0f - roi_x) + 2.5f, 1e-4);
		EXPECT_NEAR(b[5].x, left + (308.0f - roi_x) + 2.5f, 1e-4);
		EXPECT_NEAR(b[4].y, top + (298.0f - roi_y) + 2.5f, 1e-4);
		EXPECT_NEAR(b[4].size, 0.5f * merged_size, 1e-5);
		EXPECT(b[4].area == 0.0f && b[4].circularity == 0.0f && b[4].brightness == -1);
	}

	// A large blob with a single core is kept as it is.
	Img one;
	for (int k = 0; k < 4; k++) {
		one.disc(100 + 30 * k, 100, 3, 255);
	}
	one.disc(300, 300, 9, 200);
	one.disc(300, 300, 2, 255);
	b = detect_blobs(one.view(), false, 150);
	refound_blob_points(one.view(), b, false);
	EXPECT(b.size() == 5);

	// Stock quirk: a large blob whose window lies outside the image is dropped.
	std::vector<BlobPoint> far = {blob(10, 10, 2), blob(10, 30, 2), blob(10, 50, 2), blob(900, 900, 40)};
	refound_blob_points(one.view(), far, false);
	EXPECT(far.size() == 3);
}

static void
test_tracking()
{
	// Six LEDs in the image, predicted exactly.
	Img img;
	std::vector<Vec2> pred;
	for (int k = 0; k < 6; k++) {
		const double x = 280 + 16 * k, y = 230 + 4 * (k % 2);
		img.gauss(x, y, 1.2, 400);
		pred.push_back(Vec2(x, y));
	}
	pred.push_back(Vec2(-1000, -1000)); // behind the camera
	pred.push_back(Vec2(700, 200));     // outside

	TrackingInput in;
	in.predicted = pred;
	in.t_hm = Vec3(0.1, 0.1, -0.8); // far: local mean
	TrackingBlobs t = tracking_blobs(img.view(), in);
	EXPECT(t.detected && t.adaptive && t.in_image.size() == 6);
	EXPECT(t.blobs.size() == 6);
	// Window: bounding box of the predictions (+1 inclusive) grown by 5, far edge - 1.
	EXPECT(t.roi.x == 275 && t.roi.y == 225 && t.roi.width == 80 + 1 + 10 - 1 && t.roi.height == 4 + 1 + 10 - 1);
	// Raster order of the blobs' first pixels: the upper row first.
	for (size_t k = 0; k < t.blobs.size(); k++) {
		const Vec2 &p = pred[k < 3 ? 2 * k : 2 * (k - 3) + 1];
		EXPECT_NEAR(t.blobs[k].x, p.x(), 1e-4);
		EXPECT_NEAR(t.blobs[k].y, p.y(), 1e-4);
	}

	in.t_hm = Vec3(0.1, 0.1, -0.2); // closer than 0.4 m: fixed 150
	t = tracking_blobs(img.view(), in);
	EXPECT(!t.adaptive && t.threshold == 150 && t.blobs.size() == 6);

	// Near: fixed 250; these small LEDs (mean size < 10) fall back to the local mean.
	in.near = true;
	t = tracking_blobs(img.view(), in);
	EXPECT(t.adaptive && t.roi.x == 225 && t.blobs.size() == 6);

	// The other hand's accepted pixel removes the blob on it (after the window list).
	in.near = false;
	in.remove_other = true;
	in.other_points = {Vec2(296.5, 234.2)};
	t = tracking_blobs(img.view(), in);
	EXPECT(t.window_blobs.size() == 6 && t.blobs.size() == 5);

	// Near: a blob closer to a border than its size is dropped.
	Img edge;
	edge.gauss(3, 240, 1.2, 400);
	edge.gauss(20, 240, 1.2, 400);
	TrackingInput near_in;
	near_in.predicted = {Vec2(3, 240), Vec2(20, 240)};
	near_in.near = true;
	t = tracking_blobs(edge.view(), near_in);
	EXPECT(t.blobs.size() == 1 && std::fabs(t.blobs[0].x - 20.0f) < 0.3f);

	// Far and straight ahead: only round blobs (circularity >= 0.4).
	Img bar;
	bar.gauss(300, 240, 1.2, 400);
	bar.rect(320, 240, 20, 3, 255);
	TrackingInput ahead;
	ahead.predicted = {Vec2(300, 240), Vec2(330, 241)};
	ahead.t_hm = Vec3(0.0, 0.0, -0.6);
	EXPECT(tracking_blobs(bar.view(), ahead).blobs.size() == 1);
	ahead.t_hm = Vec3(0.2, 0.0, -0.6);
	EXPECT(tracking_blobs(bar.view(), ahead).blobs.size() == 2);

	// Nothing predicted in the image: no detection.
	TrackingInput none;
	none.predicted = {Vec2(-5, 10)};
	t = tracking_blobs(img.view(), none);
	EXPECT(!t.detected && t.blobs.empty());
}

static void
test_init_stages()
{
	Img img;
	img.disc(100, 100, 2, 255);
	img.disc(635, 240, 2, 255);
	img.disc(300, 475, 2, 255);
	img.disc(400, 100, 2, 60);
	std::vector<BlobPoint> b = {
	    blob(100, 100, 4, 0.2f),  // rule 1: not round (kept when near)
	    blob(635, 240, 12, 0.5f), // rule 2 should fire at the right border; stock quirk: it does not
	    blob(300, 475, 12, 0.5f), // rule 2 at the bottom border
	    blob(400, 100, 4, 0.9f),  // rule 5: dim
	};
	std::vector<BlobPoint> far = b;
	delete_some_blob_points(img.view(), far, false);
	EXPECT(far.size() == 1 && far[0].x == 635.0f);
	std::vector<BlobPoint> near = b;
	delete_some_blob_points(img.view(), near, true);
	EXPECT(near.size() == 2 && near[0].x == 100.0f);

	// Cluster predicates.
	EXPECT(cluster_predicate(ClusterPredicate::kA, blob(0, 0, 5), blob(49, 0, 5)));
	EXPECT(!cluster_predicate(ClusterPredicate::kA, blob(0, 0, 5), blob(51, 0, 5)));
	EXPECT(cluster_predicate(ClusterPredicate::kA, blob(0, 300, 8), blob(79, 300, 8)));
	EXPECT(!cluster_predicate(ClusterPredicate::kB, blob(0, 300, 8), blob(79, 300, 8)));
	EXPECT(cluster_predicate(ClusterPredicate::kB, blob(0, 100, 8), blob(79, 100, 8)));
	EXPECT(cluster_predicate(ClusterPredicate::kC, blob(0, 100, 20), blob(199, 100, 20)));

	// checkBlobPoint: keep the large ones plus enough small ones for 5; drop near-duplicates.
	std::vector<std::vector<BlobPoint>> g = {
	    {blob(0, 0, 2), blob(10, 0, 20), blob(20, 0, 3), blob(30, 0, 20), blob(40, 0, 2), blob(50, 0, 20),
	     blob(60, 0, 20)},
	    {blob(0, 0, 4), blob(0.5f, 0, 4), blob(10, 0, 4)},
	};
	check_blob_points(g);
	EXPECT(g[0].size() == 5 && g[0][0].size == 3.0f);
	EXPECT(g[1].size() == 2 && g[1][0].x == 0.5f);

	// checkBlobPointUseBrightNess: dark gaps between the three brightest are required.
	Img lit;
	lit.disc(100, 100, 2, 255);
	lit.disc(120, 100, 2, 255);
	lit.disc(140, 100, 2, 250);
	lit.disc(300, 100, 2, 255);
	lit.disc(310, 100, 2, 255);
	lit.disc(320, 100, 2, 255);
	lit.rect(300, 99, 20, 3, 200); // bright bridge
	g = {{blob(100, 100, 4), blob(140, 100, 4), blob(120, 100, 4)},
	     {blob(300, 100, 4), blob(310, 100, 4), blob(320, 100, 4)},
	     {blob(100, 100, 4), blob(120, 100, 4)}};
	check_blob_points_brightness(lit.view(), g, 20);
	EXPECT(g.size() == 1 && g[0].size() == 3 && g[0][2].x == 140.0f);

	// deleteRedundantPoints, elongated: the blobs off the line go.
	std::vector<std::vector<BlobPoint>> lines(1);
	for (int k = 0; k < 10; k++) {
		lines[0].push_back(blob(100 + 10 * k, 100, 4));
	}
	lines[0].push_back(blob(120, 130, 4));
	lines[0].push_back(blob(150, 130, 4));
	Img dark;
	delete_redundant_points(dark.view(), lines);
	EXPECT(lines[0].size() == 10);
	for (const BlobPoint &p : lines[0]) {
		EXPECT(p.y == 100.0f);
	}

	// Compact: two-means, the tighter half stays.
	std::vector<std::vector<BlobPoint>> two(1);
	for (int k = 0; k < 6; k++) {
		two[0].push_back(blob(100 + 3 * (k % 3), 100 + 3 * (k / 3), 4));
		two[0].push_back(blob(100 + 20 * (k % 3), 130 + 20 * (k / 3), 4));
	}
	delete_redundant_points(dark.view(), two);
	EXPECT(two[0].size() == 6);
	for (const BlobPoint &p : two[0]) {
		EXPECT(p.y < 110.0f);
	}
}

//! Draw a controller-like cluster: saturated LEDs with dark gaps.
static std::vector<Vec2>
draw_cluster(Img &img, double x0, double y0, int cols, int rows, double step)
{
	std::vector<Vec2> pts;
	for (int r = 0; r < rows; r++) {
		for (int c = 0; c < cols; c++) {
			const double x = x0 + step * c + 3 * r, y = y0 + step * r;
			img.gauss(x, y, 1.0, 600);
			pts.push_back(Vec2(x, y));
		}
	}
	return pts;
}

static void
test_init_path()
{
	Img img;
	draw_cluster(img, 120, 200, 3, 2, 14);                          // 6 LEDs on the left
	const std::vector<Vec2> right = draw_cluster(img, 440, 220, 3, 2, 14); // 6 LEDs on the right
	draw_cluster(img, 300, 400, 3, 1, 14);                          // 3 LEDs at the bottom
	img.gauss(320, 60, 1.0, 600);                                   // a static light

	EXPECT(!scene_is_bright(img.view()));
	const std::vector<BlobPoint> det = detect_init_blobs(img.view(), false);
	EXPECT(det.size() == 16);

	InitInput in;
	in.static_lights = {Vec2(321, 61), Vec2(320, 600)}; // the second is valid only by the 639 quirk
	InitBlobs out = prepare_init_clusters(img.view(), det, in);
	EXPECT(out.static_hits.size() == 2 && out.static_hits[0] == 1 && out.static_hits[1] == 0);
	EXPECT(out.blobs.size() == 15);
	EXPECT(out.clusters.size() == 3);
	if (out.clusters.size() == 3) {
		// Left hand: ascending x.
		EXPECT(out.clusters[0].blobs[0].x < 200 && out.clusters[0].solver == InitSolver::kP5P);
		EXPECT(out.clusters[1].solver == InitSolver::kP3P && out.clusters[1].blobs.size() == 3);
		EXPECT(out.clusters[2].blobs[0].x > 400 && out.clusters[2].original.size() == 6);
		EXPECT(out.clusters[0].n255 == 6 && !out.clusters[0].juxtaposed);
	}

	// Right hand: descending x.
	in.hand = kRight;
	out = prepare_init_clusters(img.view(), det, in);
	EXPECT(out.clusters.size() == 3 && out.clusters[0].blobs[0].x > 400);

	// The other hand's predicted LEDs remove its cluster.
	in.exclude_other = true;
	in.other_projected = right;
	in.other_projected.push_back(Vec2(130, 900)); // off the image: ignored
	out = prepare_init_clusters(img.view(), det, in);
	EXPECT(out.clusters.size() == 2);
	for (const InitCluster &c : out.clusters) {
		EXPECT(c.blobs[0].x < 400);
	}

	// Both controllers merged into one saturated cluster of 18: juxtaposed, ordered by y.
	Img both;
	draw_cluster(both, 200, 200, 6, 3, 14);
	const std::vector<BlobPoint> det2 = detect_init_blobs(both.view(), false);
	out = prepare_init_clusters(both.view(), det2, InitInput());
	EXPECT(out.clusters.size() == 1);
	if (out.clusters.size() == 1) {
		const InitCluster &c = out.clusters[0];
		EXPECT(c.blobs.size() == 18 && c.n255 == 18 && c.juxtaposed && c.solver == InitSolver::kJuxtaposedP3P);
		for (size_t i = 0; i + 1 < c.blobs.size(); i++) {
			EXPECT(c.blobs[i].y <= c.blobs[i + 1].y);
		}
	}

	// Bright scene: both corners of cam3 above 15.
	Img room(640, 480, 20);
	double mean = 0;
	EXPECT(scene_is_bright(room.view(), 15, &mean) && mean == 20.0);
	room.rect(0, 0, 160, 120, 10);
	EXPECT(!scene_is_bright(room.view()));
}

int
main()
{
	test_box_mean();
	test_contours();
	test_moments();
	test_filters();
	test_detect_fixed();
	test_detect_adaptive();
	test_geometry();
	test_stock_sort();
	test_refound();
	test_tracking();
	test_init_stages();
	test_init_path();
	if (failures != 0) {
		fprintf(stderr, "%d failures\n", failures);
		return 1;
	}
	printf("blob detection: all tests passed\n");
	return 0;
}
