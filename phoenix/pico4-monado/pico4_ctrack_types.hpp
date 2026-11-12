// Copyright 2026, overpressure contributors.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Shared types of the PICO 4 controller tracker (LED constellation + controller IMU).
 *
 * The tracker reimplements the approach of PICO OS's controller tracking. Frames:
 * - W: the tracking world of the head pose given with each camera frame; +x is up
 *   (gravity is (9.81, 0, 0) in W).
 * - H: the headset body frame (the head tracker's IMU body, as in the factory sensor.yaml T_BS).
 * - C_k: tracking camera k (factory cam<k>); T_HC_k is its sensor.yaml T_BS.
 * - M: the controller's LED-model frame, which is also its IMU frame after the fixed axis
 *   permutation (x, y, z) -> (-z, -x, y) of the raw sensor axes. No lever arm.
 *
 * @ingroup drv_pico4
 */

#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <cstdint>

namespace pico4::ctrack {

using Vec2 = Eigen::Vector2d;
using Vec3 = Eigen::Vector3d;
using Mat3 = Eigen::Matrix3d;
using Quat = Eigen::Quaterniond;

//! Gravity in W, whose +x is up.
inline const Vec3 kGravityW{9.81, 0.0, 0.0};

enum Hand
{
	kLeft = 0,
	kRight = 1,
};

//! A rigid transform: maps points of its child frame into its parent frame (p_parent = q * p + t).
struct Pose
{
	Quat q = Quat::Identity();
	Vec3 t = Vec3::Zero();

	Pose() = default;
	Pose(const Quat &q_, const Vec3 &t_) : q(q_.normalized()), t(t_) {}

	Vec3
	operator*(const Vec3 &p) const
	{
		return q * p + t;
	}

	Pose
	operator*(const Pose &o) const
	{
		return Pose(q * o.q, q * o.t + t);
	}

	Pose
	inverse() const
	{
		Quat qi = q.conjugate();
		return Pose(qi, -(qi * t));
	}
};

//! Raw axes of the controller IMU to the LED-model frame M: (x, y, z) -> (-z, -x, y).
inline Vec3
sensor_to_model(const Vec3 &v)
{
	return Vec3(-v.z(), -v.x(), v.y());
}

/*!
 * One controller IMU sample as the radio delivers it, already in SI units: gyro in rad/s and
 * accelerometer in m/s^2 in the raw sensor axes (pico4_blemcu.h scales), temperature in deg C.
 * t_ns is CLOCK_MONOTONIC: the sample's station time on the camera clock.
 */
struct ImuSample
{
	int64_t t_ns = 0;
	Vec3 gyro = Vec3::Zero();
	Vec3 accel = Vec3::Zero();
	double temperature = 0.0;
	//! Status byte of the IMU record (data byte 15): bit 0 "flagbit", the rest "flagrest"; 0 so far.
	uint8_t status = 0;
};

} // namespace pico4::ctrack
