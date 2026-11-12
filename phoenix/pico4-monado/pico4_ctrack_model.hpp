// Copyright 2026, overpressure contributors.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Cameras, LED models, LED visibility and the 5-LED identification table of the
 *         PICO 4 controller tracker (the approach of PICO OS's controller tracking).
 *
 * The LED models (20 positions and unit normals per hand, frame M) come from the headset's own
 * firmware at build time (tooling/ledmodel.py). The identification table is not shipped: it is
 * generated from the left model by the rule PICO OS's table was built with (see
 * IdTable::generate) and shared by both hands, the right hand mirroring its descriptors.
 *
 * @ingroup drv_pico4
 */

#pragma once

#include "pico4_ctrack_types.hpp"

#include <array>
#include <utility>
#include <vector>

namespace pico4::ctrack {

constexpr int kLeds = 20;
constexpr int kCameras = 4;
constexpr int kImageWidth = 640;
constexpr int kImageHeight = 480;

/*!
 * A tracking camera: the unified omnidirectional (Mei) model with radial-tangential distortion
 * of the factory sensor.yaml, and its pose in the headset body frame (T_BS, body <- camera).
 */
struct Camera
{
	double xi = 0, fu = 0, fv = 0, pu = 0, pv = 0;
	double k1 = 0, k2 = 0, p1 = 0, p2 = 0;
	Pose T_HC;

	//! Normalised image plane of a camera-frame point: (x, y) / (z + xi |p|); false behind the model.
	bool
	project2d(const Vec3 &p, Vec2 &out) const;

	//! Pixel of a camera-frame point (distortion applied); false if project2d fails.
	bool
	world2cam(const Vec3 &p, Vec2 &out) const;

	//! Unit bearing in the camera frame of a pixel.
	Vec3
	cam2world(const Vec2 &px) const;

	//! Pixels per unit of the normalised plane, to express normalised errors in pixels.
	double
	error_multiplier() const;

private:
	Vec2
	distort(const Vec2 &m) const;
};

//! One hand's LEDs in the model frame M: positions in metres and unit normals.
struct LedModel
{
	std::array<Vec3, kLeds> position{};
	std::array<Vec3, kLeds> normal{};
};

/*!
 * LEDs facing a camera, most frontal first (generateVisiblePoints): the camera is assumed 1 m
 * from the controller along the bearing `b` (camera frame) of the blob centroid; an LED faces
 * it when normal . normalize(c - P) >= 0 with c = -(q_MC b). No occlusion or image test.
 * @param q_MC  rotation camera -> M.
 * @return (LED, cosine) pairs.
 */
std::vector<std::pair<int, double>>
visible_leds(const LedModel &model, const Quat &q_MC, const Vec3 &b);

//! One row of the identification table: three points in the basis frame of LEDs (a, b), and the five LEDs.
struct IdRecord
{
	std::array<Vec2, 3> pts;
	std::array<int, 5> ids; //!< a, b, then c < d < e
};

/*!
 * Descriptor of 2+3 bearings in the basis frame of the first two (section "ID table" of the
 * tracker): rotate so that normalize(f_a + f_b) is the optical axis and f_b - f_a points along
 * +x, pinhole-project, and scale so that a -> (-1, 0) and b -> (1, 0). Invariant to the camera's
 * rotation and to scale.
 */
void
basis_descriptor(const Vec3 &f_a, const Vec3 &f_b, const Vec3 *others, int n, Vec2 *out);

//! A geometric hash of 5-LED image patterns (GenerateP5PList): table, equal-frequency grid, lookup.
class IdTable
{
public:
	//! Build the table from the left model: 200 viewpoints on a 0.6 m sphere, the 44 LED groups.
	void
	generate(const LedModel &left);

	//! Use given records instead (for tests and comparisons with a stored table).
	void
	load(std::vector<IdRecord> records);

	const std::vector<IdRecord> &
	records() const
	{
		return records_;
	}

	/*!
	 * Up to `max_lists` voted correspondences (blob index, LED) of 5 blobs, most votes first, for
	 * the unit bearings of >= 5 blobs in one camera. `mirror` (right hand) negates the descriptor y.
	 */
	std::vector<std::vector<std::pair<int, int>>>
	lookup(const std::vector<Vec3> &bearings, bool mirror, int max_lists = 10) const;

private:
	static constexpr int kBins = 200;
	std::vector<IdRecord> records_;
	std::vector<double> edges_x_, edges_y_;
	std::vector<std::vector<int>> grid_; //!< kBins * kBins cells of record indices, by the first point
	void
	build_grid();
	int
	bin(const std::vector<double> &edges, double v) const;
};

} // namespace pico4::ctrack
