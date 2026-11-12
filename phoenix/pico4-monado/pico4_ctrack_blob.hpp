// Copyright 2026, overpressure contributors.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Controller LED blob detection of the PICO 4 controller tracker.
 *
 * Reimplements the blob stage of PICO OS's controller tracking from a behavioural description:
 * - the detector (fixed or local-mean binarisation, outer 8-connected contours, polygon moments,
 *   area and circularity filters, a centroid of the contour polygon),
 * - the tracking path: a window around the predicted LED pixels, the threshold chosen by
 *   distance, splitting of merged blobs at their saturated cores, border and other-hand rules,
 * - the (re)initialisation path: full-image detection, removal of the other hand's and static
 *   lights' blobs, re-detection per cluster, clustering, the cluster filters, their priority and
 *   the blob order handed to the pose initialisers.
 *
 * Images are 8-bit grey, row-major with a stride, at most 640 x 480 (the stock detector's work
 * buffer). Pixel (c, r) has its centre at (c, r). Camera indices are the factory cam<k>; the
 * scene-brightness test reads cam3.
 *
 * Behaviour of the stock code that looks unintended is kept and marked "stock quirk" at the
 * place it happens. No OpenCV: the needed pieces (box filter, border following, moments,
 * minimum-area rectangle, partition, two-means) are implemented here.
 *
 * @ingroup drv_pico4
 */

#pragma once

#include "pico4_ctrack_types.hpp"

#include <cstddef>
#include <cstdint>
#include <functional>
#include <iterator>
#include <utility>
#include <vector>

namespace pico4::ctrack {

/*
 *
 * The stock library's std::sort.
 *
 */

namespace stock_sort_detail {

//! Order three elements with at most two exchanges; returns the number of exchanges.
template <typename It, typename Less>
unsigned
order3(It a, It b, It c, Less &less)
{
	if (!less(*b, *a)) {
		if (!less(*c, *b)) {
			return 0;
		}
		std::iter_swap(b, c);
		if (less(*b, *a)) {
			std::iter_swap(a, b);
			return 2;
		}
		return 1;
	}
	if (less(*c, *b)) {
		std::iter_swap(a, c);
		return 1;
	}
	std::iter_swap(a, b);
	if (less(*c, *b)) {
		std::iter_swap(b, c);
		return 2;
	}
	return 1;
}

//! order3, then sink @p d into place.
template <typename It, typename Less>
unsigned
order4(It a, It b, It c, It d, Less &less)
{
	unsigned n = order3(a, b, c, less);
	if (less(*d, *c)) {
		std::iter_swap(c, d);
		n++;
		if (less(*c, *b)) {
			std::iter_swap(b, c);
			n++;
			if (less(*b, *a)) {
				std::iter_swap(a, b);
				n++;
			}
		}
	}
	return n;
}

//! order4, then sink @p e into place.
template <typename It, typename Less>
unsigned
order5(It a, It b, It c, It d, It e, Less &less)
{
	unsigned n = order4(a, b, c, d, less);
	if (less(*e, *d)) {
		std::iter_swap(d, e);
		n++;
		if (less(*d, *c)) {
			std::iter_swap(c, d);
			n++;
			if (less(*c, *b)) {
				std::iter_swap(b, c);
				n++;
				if (less(*b, *a)) {
					std::iter_swap(a, b);
					n++;
				}
			}
		}
	}
	return n;
}

//! Move *@p from down past every greater element before it (the range starts at @p first).
template <typename It, typename Less>
void
sink(It first, It from, Less &less)
{
	auto value = std::move(*from);
	It hole = from;
	It prev = from;
	do {
		--prev;
		*hole = std::move(*prev);
		hole = prev;
	} while (hole != first && less(value, *std::prev(hole)));
	*hole = std::move(value);
}

//! Insertion sort whose first three elements are ordered by order3.
template <typename It, typename Less>
void
insertion_sort(It first, It last, Less &less)
{
	It sorted_end = first + 2;
	order3(first, first + 1, sorted_end, less);
	for (It i = sorted_end + 1; i != last; ++i) {
		if (less(*i, *(i - 1))) {
			sink(first, i, less);
		}
	}
}

//! Insertion sort that gives up after 8 moved elements; true when the range ended sorted.
template <typename It, typename Less>
bool
insertion_sort_bounded(It first, It last, Less &less)
{
	switch (last - first) {
	case 0:
	case 1: return true;
	case 2:
		if (less(*(last - 1), *first)) {
			std::iter_swap(first, last - 1);
		}
		return true;
	case 3: order3(first, first + 1, last - 1, less); return true;
	case 4: order4(first, first + 1, first + 2, last - 1, less); return true;
	case 5: order5(first, first + 1, first + 2, first + 3, last - 1, less); return true;
	default: break;
	}
	order3(first, first + 1, first + 2, less);
	int moved = 0;
	for (It i = first + 3; i != last; ++i) {
		if (less(*i, *(i - 1))) {
			sink(first, i, less);
			if (++moved == 8) {
				return i + 1 == last;
			}
		}
	}
	return true;
}

} // namespace stock_sort_detail

/*!
 * The unstable std::sort of the stock library (the NDK's libc++ introsort), so that elements
 * with equal keys end up in the same order as on PICO OS. Read from its instantiations (for
 * example 0x3d8600 for sortBlobPoints, 0x339b04 for getP5PList): ranges of up to 5 elements by
 * fixed exchanges, up to @p small elements by insertion sort, else a median-of-3 (of 5 from
 * 1000 elements) partition that falls back to a bounded insertion sort after a partition
 * without exchanges and recurses into the smaller part.
 *
 * @param small  Length up to which the instantiation sorts by insertion: 30 when the element
 *               type is trivially copyable in the stock build (a plain struct such as
 *               Blob::BlobPoint), 6 otherwise (std::pair and vector-holding types). Every
 *               instantiation's length test shows its value.
 */
template <typename It, typename Less>
void
stock_sort(It first, It last, Less less, std::ptrdiff_t small)
{
	using namespace stock_sort_detail;
	while (true) {
		const std::ptrdiff_t len = last - first;
		switch (len) {
		case 0:
		case 1: return;
		case 2:
			if (less(*(last - 1), *first)) {
				std::iter_swap(first, last - 1);
			}
			return;
		case 3: order3(first, first + 1, last - 1, less); return;
		case 4: order4(first, first + 1, first + 2, last - 1, less); return;
		case 5: order5(first, first + 1, first + 2, first + 3, last - 1, less); return;
		default: break;
		}
		if (len <= small) {
			insertion_sort(first, last, less);
			return;
		}

		// Pivot: median of first, middle and last (of five spread elements from 1000 on).
		It m = first + len / 2;
		const It lm1 = last - 1;
		unsigned swaps = len >= 1000 ? order5(first, first + len / 4, m, m + len / 4, lm1, less)
		                             : order3(first, m, lm1, less);

		It i = first;
		It j = lm1;
		bool restart = false;
		if (!less(*i, *m)) {
			// *first is not below the pivot: find an element below it to guard the scan.
			while (true) {
				if (i == --j) {
					// Nothing below the pivot: split off the elements equal to *first instead.
					++i;
					j = last;
					if (!less(*first, *--j)) {
						while (true) {
							if (i == j) {
								return; // all equal
							}
							if (less(*first, *i)) {
								std::iter_swap(i, j);
								++i;
								break;
							}
							++i;
						}
					}
					if (i == j) {
						return;
					}
					while (true) {
						while (!less(*first, *i)) {
							++i;
						}
						while (less(*first, *--j)) {
						}
						if (i >= j) {
							break;
						}
						std::iter_swap(i, j);
						++i;
					}
					first = i;
					restart = true;
					break;
				}
				if (less(*j, *m)) {
					std::iter_swap(i, j);
					swaps++;
					break;
				}
			}
		}
		if (restart) {
			continue;
		}

		++i;
		if (i < j) {
			while (true) {
				while (less(*i, *m)) {
					++i;
				}
				while (!less(*--j, *m)) {
				}
				if (i > j) {
					break;
				}
				std::iter_swap(i, j);
				swaps++;
				if (m == i) {
					m = j;
				}
				++i;
			}
		}
		if (i != m && less(*m, *i)) {
			std::iter_swap(i, m);
			swaps++;
		}

		if (swaps == 0) {
			// Already partitioned without exchanges: maybe both halves are nearly sorted.
			const bool left_done = insertion_sort_bounded(first, i, less);
			if (insertion_sort_bounded(i + 1, last, less)) {
				if (left_done) {
					return;
				}
				last = i;
				continue;
			}
			if (left_done) {
				first = i + 1;
				continue;
			}
		}
		if (i - first < last - i) {
			stock_sort(first, i, less, small);
			first = i + 1;
		} else {
			stock_sort(i + 1, last, less, small);
			last = i;
		}
	}
}

constexpr int kBlobImageWidth = 640;
constexpr int kBlobImageHeight = 480;

struct Point2i
{
	int x = 0;
	int y = 0;
};

struct Point2f
{
	float x = 0.0f;
	float y = 0.0f;
};

struct RectI
{
	int x = 0;
	int y = 0;
	int width = 0;
	int height = 0;
};

struct RectF
{
	float x = 0.0f;
	float y = 0.0f;
	float width = 0.0f;
	float height = 0.0f;
};

//! A non-owning view of an 8-bit grey image (or a window of one, sharing its stride).
struct ImageView
{
	const uint8_t *data = nullptr;
	int width = 0;
	int height = 0;
	int stride = 0;

	uint8_t
	at(int x, int y) const
	{
		return data[(size_t)y * (size_t)stride + (size_t)x];
	}

	//! The window @p r, which must lie inside the image.
	ImageView
	window(const RectI &r) const
	{
		return ImageView{data + (size_t)r.y * (size_t)stride + (size_t)r.x, r.width, r.height, stride};
	}
};

/*!
 * One detected LED blob, as the stock detector fills it. Coordinates are those of the image
 * the detector ran on until a caller shifts them (the tracking and init paths return full-image
 * coordinates).
 */
struct BlobPoint
{
	float x = 0.0f;           //!< Centroid of the contour polygon.
	float y = 0.0f;
	float size = 0.0f;        //!< Diameter: twice the median distance of the contour points from the centroid.
	float area = 0.0f;        //!< Area of the contour polygon (0 for split pieces).
	float circularity = 0.0f; //!< 4 pi area / perimeter^2 (0 for split pieces).
	float color = 255.0f;     //!< Constant 255 (unused on phoenix).
	float convexity = 0.0f;   //!< Area / convex hull area (0 for split pieces).
	uint8_t flag0 = 0;        //!< Never written on the phoenix path.
	uint8_t flag1 = 0;
	int brightness = -1; //!< -1; consumers compute the 3x3 maximum from the image instead.
};

//! The stock detector parameters (an OpenCV SimpleBlobDetector-like set); both stock instances use the defaults.
struct DetectorParams
{
	float threshold_step = 10.0f; //!< Pass loop step; the passes all use the threshold argument.
	float min_threshold = 80.0f;
	float max_threshold = 85.0f;
	int min_repeatability = 1;
	float min_dist_between_blobs = 4.0f;
	bool filter_by_color = false;
	uint8_t blob_color = 255;
	bool filter_by_area = true;
	float min_area = 0.49f;
	float max_area = 3000.0f; //!< In fixed mode (and adaptive with block 5) the limit is 9 x this.
	bool filter_by_circularity = true;
	float min_circularity = 0.1f;
	float max_circularity = 3.402823466e+38f;
	bool filter_by_inertia = false;
	float min_inertia_ratio = 0.1f;
	float max_inertia_ratio = 3.402823466e+38f;
	bool filter_by_convexity = false;
	float min_convexity = 0.95f;
	float max_convexity = 3.402823466e+38f;
};

/*
 *
 * Detector (stock Blob::BlobDetector::detect).
 *
 */

//! Offset C of the local-mean threshold: a pixel is set when it exceeds its box mean by more than 30.
constexpr int kAdaptiveOffset = 30;

/*!
 * Detect blobs in @p image (a full image or a window of one, at most 640 x 480).
 *
 * @param adaptive  false: pixel > @p threshold; true: pixel > 11x11 box mean + 30 (5x5 when the
 *                  image is 15 pixels or less in either direction), @p threshold is ignored.
 * @return          Blobs in the image's coordinates, in the raster order of each blob's first
 *                  pixel. Empty for an empty or oversized image.
 */
std::vector<BlobPoint>
detect_blobs(const ImageView &image, bool adaptive, int threshold, const DetectorParams &params = DetectorParams());

/*
 * Detector building blocks, exposed for the tests and for reuse.
 */

/*!
 * Box mean of @p src over block x block pixels, replicating the image's own edge pixels (never
 * reading outside the view), rounded to 8 bits the way OpenCV's 8-bit box filter does
 * (16-bit sums, fixed-point division). Row-major, stride = width.
 */
std::vector<uint8_t>
box_mean(const ImageView &src, int block);

/*!
 * Binarise @p src into a zero-framed buffer of (width + 2) x (height + 2) bytes: 1 where the
 * pixel is set, 0 elsewhere and on the frame.
 */
std::vector<int8_t>
binarise(const ImageView &src, bool adaptive, int threshold, int *block_out = nullptr);

enum class ChainApprox
{
	kNone,   //!< Every visited border pixel, repeats included.
	kSimple, //!< Only the points where the chain direction changes.
};

/*!
 * Outer contours of a zero-framed 0/1 buffer (padded size @p w x @p h), 8-connected, in
 * raster order of their first pixel; blobs inside holes of another blob are skipped. The
 * buffer receives the border-following marks. Points are reported as padded position + @p offset.
 */
std::vector<std::vector<Point2i>>
find_outer_contours(int8_t *buf, int w, int h, ChainApprox approx, Point2i offset);

//! Spatial and central moments of the closed polygon through the contour points (area positive).
struct ContourMoments
{
	double m00 = 0, m10 = 0, m01 = 0, m20 = 0, m11 = 0, m02 = 0;
	double mu20 = 0, mu11 = 0, mu02 = 0;
};

ContourMoments
contour_moments(const std::vector<Point2i> &c);

//! Perimeter of the closed polygon through the points.
double
closed_arc_length(const std::vector<Point2i> &c);

//! Absolute area of the polygon through the points.
double
polygon_area(const std::vector<Point2i> &c);

//! Convex hull (counter-clockwise in a y-up sense, no collinear points).
std::vector<Point2i>
convex_hull(std::vector<Point2i> pts);

/*!
 * Candidate filter of the detector for the contours in a binarised buffer (stock
 * findBlobs_neon). @p block is the adaptive block size (11 or 5) or 0 in fixed mode: only
 * block 11 keeps the max-area limit, the others allow 9 x max area.
 */
struct BlobCandidate
{
	double x = 0, y = 0, radius = 0, confidence = 1, area = 0, circularity = 0, convexity = 0;
};

std::vector<BlobCandidate>
filter_contours(const std::vector<std::vector<Point2i>> &contours, const DetectorParams &params, int block);

/*
 *
 * Geometry helpers (OpenCV-like rotated rectangle, partition).
 *
 */

/*!
 * A rotated rectangle as cv::RotatedRect: angle in degrees of the width side, height side
 * perpendicular to it. min_area_rect() yields the representation the stock library's bundled
 * OpenCV 3.4.0 yields (mostly in [-90, 0], but not normalised).
 */
struct RotatedRect
{
	Point2f center;
	float width = 0.0f;
	float height = 0.0f;
	float angle = 0.0f;

	//! Corners in OpenCV's RotatedRect::points order and float arithmetic (0x9ad580).
	void
	points(Point2f pts[4]) const;

	//! Integer bounding rectangle of the corners: floor of the minimum, ceil of the maximum, inclusive.
	RectI
	bounding_rect() const;
};

/*!
 * Minimum-area enclosing rectangle exactly as the stock cv::minAreaRect (OpenCV 3.4.0,
 * 0x8643f4): the clockwise convex hull in cv::convexHull's order (starting at the lowest-x
 * point), rotating calipers in float, the last of equal areas. Its corner order decides
 * sortBlobPoints' direction and, for clusters of three, which rectangle among equal ones (and so
 * the order) is taken.
 */
RotatedRect
min_area_rect(const std::vector<Point2f> &pts);

/*!
 * Stock calcSafeRect: the bounding rectangle of @p rr grown by 5 + floor(margin / 2) on each
 * side and clipped to cols x rows; the far edges lose one pixel (stock quirk).
 * @return false (and @p out untouched) when the result is empty.
 */
bool
calc_safe_rect(const RotatedRect &rr, int cols, int rows, double margin, RectF &out);

//! RectF -> RectI by rounding each component (OpenCV's Rect2f to Rect conversion).
RectI
round_rect(const RectF &r);

/*!
 * Brightness of a blob: the maximum grey value of the 3x3 window around its rounded centre
 * (half away from zero, clamped to at most 639 / 479), skipping pixels outside the image.
 */
int
neighbour_max_brightness(const ImageView &img, float x, float y);

//! Euclidean distance of two blob centres.
float
dist_p2p(const BlobPoint &a, const BlobPoint &b);

//! Distance from @p p to the infinite line through @p a and @p b in float, as the stock code (NaN if a == b).
float
dist_p2l(Point2f p, Point2f a, Point2f b);

/*!
 * Equivalence classes of @p n elements under @p same (cv::partition semantics): connected
 * components of the relation, labelled 0.. in order of each class's first element.
 */
std::vector<int>
partition_labels(int n, const std::function<bool(int, int)> &same, int *classes = nullptr);

/*!
 * Stock sortBlobPoints (0x375118): ascending by the distance from the short edge at corner 0 of
 * the minimum-area rectangle of the centres, i.e. along the long axis; equal distances in the
 * order of the stock std::sort.
 */
void
sort_blob_points(std::vector<BlobPoint> &blobs);

/*
 *
 * Shared stages.
 *
 */

/*!
 * Stock reFoundBlobPoint: replace each blob larger than 1.5 x the mean size by the saturated
 * cores (> 240) inside a window of +- size around it, when there are two or more; such a blob
 * is dropped when its window is empty (stock quirk). @p near ignores one-point cores.
 * Coordinates of @p blobs are those of @p img.
 */
void
refound_blob_points(const ImageView &img, std::vector<BlobPoint> &blobs, bool near);

//! Drop every blob whose centre is closer than radius_factor x its size to one of @p points.
void
remove_blobs_near_points(std::vector<BlobPoint> &blobs, const std::vector<Vec2> &points, float radius_factor);

/*
 *
 * Tracking path (stock getProjectPointPairsAndBlobPoints + the other-hand removal of
 * updateMatchPoints), one camera.
 *
 */

struct TrackingInput
{
	/*!
	 * Predicted pixel of every LED in this camera, index = LED id: world2cam of the LED
	 * position in the camera frame under the prior pose (no visibility or z test; the camera
	 * model's invalid result, e.g. (-1000, -1000), simply fails the in-image test).
	 */
	std::vector<Vec2> predicted;
	//! judgeNear() of this hand (controller within 0.2 m of a camera and inside the near box).
	bool near = false;
	//! Controller position in the head frame in metres: translation of inv(T_WH) * T_WM (prior).
	Vec3 t_hm = Vec3::Zero();
	//! ROI margin when near: 100 on the tracking path, 40 in checkPnPByProjectArea.
	double near_margin = 100.0;
	/*!
	 * Other-hand removal is active: this hand is the second one this frame and the other hand
	 * is tracked and locked (stock predictor+0x359, other+0x358, other+0x198c).
	 */
	bool remove_other = false;
	//! The other hand's accepted pixels in this camera (stock other+0x668[cam]).
	std::vector<Vec2> other_points;
};

struct TrackingBlobs
{
	std::vector<int> in_image; //!< LED ids predicted inside the image, ascending.
	RectI roi;                 //!< Detection window (empty if no detection ran).
	bool detected = false;     //!< false: no LED in the image or the window was 3 px wide or less.
	bool adaptive = false;     //!< Final binarisation of the window.
	int threshold = 0;         //!< Fixed threshold used (0 when adaptive).
	/*!
	 * The window's blobs as getProjectPointPairsAndBlobPoints returns them, in image
	 * coordinates. Their centroid is the view direction of generateVisiblePoints, which
	 * builds the (predicted pixel, LED id) pairs of NewMatchPoints.
	 */
	std::vector<BlobPoint> window_blobs;
	//! window_blobs after the other-hand removal: the blob list of NewMatchPoints.
	std::vector<BlobPoint> blobs;
};

TrackingBlobs
tracking_blobs(const ImageView &image, const TrackingInput &in);

/*
 *
 * (Re)initialisation path (stock comControllerPoseForTracking, blob part), one camera.
 *
 */

//! Stock scene-brightness flag from cam3: both corner means (160 x 120) above @p threshold.
bool
scene_is_bright(const ImageView &cam3, int threshold = 15, double *mean_out = nullptr);

//! Full-image detection of the init path: adaptive when the scene is bright, else pixel > 70.
std::vector<BlobPoint>
detect_init_blobs(const ImageView &full, bool bright);

struct InitInput
{
	bool bright = false; //!< scene_is_bright() of this frame set (stock predictor+0x1971).
	bool near = false;   //!< judgeNear() of this hand.
	//! Hand index (stock predictor+0x1924): 0 orders clusters by ascending x, others descending.
	int hand = kLeft;
	int led_count = 20; //!< LED traces of the model.
	/*!
	 * Other-hand exclusion is active: the other hand tracks (stock: other+0x1940 with body state
	 * 0 or 2, or other+0x358 and other+0x198c).
	 */
	bool exclude_other = false;
	//! The other hand moves fast (norm of its filter vector other->[0xd08]+0x128 > 5): radius 3 x size.
	bool other_fast = false;
	//! The other hand's predicted LED pixels in this camera (raw world2cam; the in-image test is applied here).
	std::vector<Vec2> other_projected;
	//! Static world lights projected into this camera (raw; the stock validity test is applied here).
	std::vector<Vec2> static_lights;
};

enum class InitSolver
{
	kJuxtaposedP3P, //!< Both controllers merged: blobs sorted by y, P3P on the first and the last three.
	kP5P,           //!< >= 5 blobs: initPoseUsingP5P_New, then initPoseUsingP5P (ID table) with original.
	kP3P,           //!< 3 or 4 blobs: initPoseUsingP3P.
};

struct InitCluster
{
	//! The filtered cluster, ordered for its solver (by y when juxtaposed, else sortBlobPoints).
	std::vector<BlobPoint> blobs;
	//! The cluster before deleteRedundantPoints ("blobs2" of initPoseUsingP5P), brightness order.
	std::vector<BlobPoint> original;
	int n255 = 0;            //!< Blobs of the filtered cluster with a 3x3 maximum > 254.
	bool juxtaposed = false; //!< n255 >= 14 and >= 17 blobs (stock P+0x1b23 for this cluster).
	InitSolver solver = InitSolver::kP3P;
};

struct InitBlobs
{
	std::vector<BlobPoint> blobs; //!< The camera's blob list after preparation (frame.blobs).
	std::vector<int> static_hits; //!< Blobs removed per static light in the first pass.
	std::vector<InitCluster> clusters; //!< Priority order: try them in this order, first success wins.
};

/*!
 * Prepare the init path's clusters for one camera. @p detected is detect_init_blobs() of
 * this camera's image; the stock code detects once per frame and shares the list between
 * both hands.
 */
InitBlobs
prepare_init_clusters(const ImageView &full, const std::vector<BlobPoint> &detected, const InitInput &in);

/*
 * Init-path stages, exposed for the tests.
 */

//! Stock deleteSomeBolbPoint: drop blobs that do not look like LEDs (circularity, size, brightness).
void
delete_some_blob_points(const ImageView &full, std::vector<BlobPoint> &blobs, bool near);

enum class ClusterPredicate
{
	kA, //!< size cut 7, cap 150 / 50.
	kB, //!< size cut by y (7 / 10 / 15), cap 150 / 50.
	kC, //!< size cut by y, cap 250 / 50.
};

//! Two blobs belong to one cluster: distance < min(10 m, cap), m = the larger size.
bool
cluster_predicate(ClusterPredicate p, const BlobPoint &a, const BlobPoint &b);

/*!
 * Stock classifyBlobPoint: partition with P_A (bright) or P_B, re-partitioned with P_C when the
 * mean size exceeds 10 and the mean brightness 240 over all blobs.
 */
std::vector<std::vector<BlobPoint>>
classify_blob_points(const ImageView &full, const std::vector<BlobPoint> &blobs, bool bright);

//! Stock reFindBlobPoint: re-detect each cluster of two or more blobs with its own threshold.
std::vector<BlobPoint>
refind_blob_points(const ImageView &full, const std::vector<BlobPoint> &blobs, bool bright, bool near);

//! Stock checkBlobPoint: drop small blobs of large-blob clusters, then near-duplicates.
void
check_blob_points(std::vector<std::vector<BlobPoint>> &groups);

//! Stock checkBlobPointUseBrightNess: keep clusters of 3..30 separate bright LEDs, brightness order.
void
check_blob_points_brightness(const ImageView &full, std::vector<std::vector<BlobPoint>> &groups, int led_count);

/*!
 * Stock deleteRedundantPoints: thin out clusters of 10..15 blobs (two-means for compact
 * clusters, a line fit for elongated ones).
 */
void
delete_redundant_points(const ImageView &full, std::vector<std::vector<BlobPoint>> &groups);

/*!
 * Two-means of 2-D points (k-means++ seeding, 3 attempts, 10 iterations, epsilon 0.1). The
 * stock code seeds from OpenCV's global RNG and is not reproducible; this one is deterministic.
 */
std::vector<int>
two_means(const std::vector<Point2f> &pts);

} // namespace pico4::ctrack
