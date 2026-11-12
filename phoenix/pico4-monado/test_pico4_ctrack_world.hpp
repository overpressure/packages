// Copyright 2026, overpressure contributors.
// SPDX-License-Identifier: BSL-1.0
/*
 * Synthetic world of the controller tracker's tests (no device data): four cameras like a
 * headset's, a synthetic LED controller, rendered LED images and a test motion with its IMU
 * samples.
 */

#pragma once

#include "pico4_ctrack_engine.hpp"

#include <cmath>
#include <cstdio>
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

static Quat
rot(double ax, double ay, double az)
{
	return Quat(Eigen::AngleAxisd(az, Vec3::UnitZ()) * Eigen::AngleAxisd(ay, Vec3::UnitY()) *
	            Eigen::AngleAxisd(ax, Vec3::UnitX()));
}

static Camera
camera(const Pose &T_HC)
{
	Camera c;
	c.xi = 0.9;
	c.fu = c.fv = 420.0;
	c.pu = 320.0;
	c.pv = 240.0;
	c.k1 = -0.15;
	c.k2 = 0.03;
	c.p1 = 0.0004;
	c.p2 = -0.0003;
	c.T_HC = T_HC;
	return c;
}

//! A synthetic controller (not the real model): 20 LEDs on a wavy arc, normals outward.
static LedModel
ring(bool mirror)
{
	LedModel m;
	for (int i = 0; i < kLeds; i++) {
		// An open arc of 300 degrees: LEDs with neighbouring ids are neighbours, 19 and 0 are not.
		double a = 2 * M_PI * i / (kLeds + 4);
		// No symmetry: radius and height vary irregularly along the arc.
		double r = 0.065 + 0.008 * std::sin(1.3 * a + 0.4);
		m.position[i] = Vec3(r * std::cos(a), r * std::sin(a), 0.015 * std::sin(1.7 * a + 0.3) + 0.008 * std::cos(2.9 * a));
		m.normal[i] = Vec3(std::cos(a), std::sin(a), 0.3 * std::cos(2 * a)).normalized();
		if (mirror) {
			m.position[i].y() = -m.position[i].y();
			m.normal[i].y() = -m.normal[i].y();
		}
	}
	return m;
}

//! Raw sensor axes of a vector in M: the inverse of sensor_to_model.
static Vec3
model_to_sensor(const Vec3 &m)
{
	return Vec3(-m.y(), m.z(), -m.x());
}

struct World
{
	std::array<Camera, kCameras> cams;
	LedModel models[2] = {ring(false), ring(true)};
	Pose T_WH; // head: H +x is up, +z forward
	std::array<std::vector<uint8_t>, kCameras> pixels;

	World()
	{
		// Four cameras looking forward, splayed outwards and downwards like a headset's.
		cams = {camera(Pose(rot(0.3, -0.35, 0), Vec3(0.02, 0.04, 0.0))),
		        camera(Pose(rot(0.3, 0.35, 0), Vec3(0.02, -0.04, 0.0))),
		        camera(Pose(rot(-0.4, -0.45, 0), Vec3(-0.02, 0.05, 0.0))),
		        camera(Pose(rot(-0.4, 0.45, 0), Vec3(-0.02, -0.05, 0.0)))};
	}

	//! LED images of a controller pose: a Gaussian spot per LED facing the camera.
	std::array<ImageView, kCameras>
	render(const LedModel &model, const Pose &T_WM)
	{
		std::array<ImageView, kCameras> views;
		for (int k = 0; k < kCameras; k++) {
			std::vector<uint8_t> &img = pixels[k];
			img.assign((size_t)kImageWidth * kImageHeight, 4);
			Pose T_CM = (T_WH * cams[k].T_HC).inverse() * T_WM;
			for (int i = 0; i < kLeds; i++) {
				Vec3 p = T_CM * model.position[i];
				Vec3 n = T_CM.q * model.normal[i];
				Vec2 px;
				if (n.dot(-p.normalized()) < 0.3 || !cams[k].world2cam(p, px)) {
					continue;
				}
				for (int y = (int)px.y() - 4; y <= (int)px.y() + 4; y++) {
					for (int x = (int)px.x() - 4; x <= (int)px.x() + 4; x++) {
						if (x < 0 || y < 0 || x >= kImageWidth || y >= kImageHeight) {
							continue;
						}
						double d2 = (Vec2(x, y) - px).squaredNorm();
						int v = img[(size_t)y * kImageWidth + x] + (int)(320.0 * std::exp(-d2 / (2 * 0.9 * 0.9)));
						img[(size_t)y * kImageWidth + x] = (uint8_t)std::min(v, 255);
					}
				}
			}
			views[k] = ImageView{img.data(), kImageWidth, kImageHeight, kImageWidth};
		}
		return views;
	}
};

//! The test motion: still, then a sideways sweep from rest (1 Hz, 20 cm) with a slight wobble.
struct Motion
{
	// The left controller 0.5 m ahead, 0.15 m below and 0.1 m beside the head, its arc axis about level.
	Pose rest{rot(-M_PI / 2 + 0.25, 0.3, 0.15), Vec3(-0.15, 0.1, 0.5)};
	Vec3 amplitude{0.02, 0.1, 0.0};
	double omega = 2 * M_PI;
	int64_t t_move = 0;
	// Position and wobble about a fixed axis in M start from rest (the IMU sees every change of
	// velocity); a held controller never keeps the gyro at zero.
	Vec3 wobble_axis = Vec3(0.3, 1.0, -0.5).normalized();
	double wobble = 0.15;

	Pose
	pose_at(int64_t s) const
	{
		Pose p = rest;
		if (s > t_move) {
			double x = omega * (s - t_move) * 1e-9;
			p.t += amplitude * (1 - std::cos(x));
			p.q = rest.q * Quat(Eigen::AngleAxisd(wobble * (1 - std::cos(x)), wobble_axis));
		}
		return p;
	}

	//! The controller IMU sample at @p s: the accelerometer measures acceleration minus gravity (W +x up), in M.
	ImuSample
	imu_at(int64_t s) const
	{
		Vec3 a_W = Vec3::Zero(), w_M = Vec3::Zero();
		if (s > t_move) {
			double x = omega * (s - t_move) * 1e-9;
			a_W = amplitude * omega * omega * std::cos(x);
			w_M = wobble_axis * wobble * omega * std::sin(x);
		}
		ImuSample m;
		m.t_ns = s;
		m.accel = model_to_sensor(pose_at(s).q.conjugate() * (a_W + kGravityW));
		m.gyro = model_to_sensor(w_M);
		m.temperature = 30.0;
		return m;
	}
};

