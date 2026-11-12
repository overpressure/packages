// Copyright 2026, overpressure contributors.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  The controller tracker's calibration: the factory tracking cameras (sensor.yaml) and
 *         the LED models and lever arms extracted from the headset's firmware (led-models.json,
 *         lever-arm.json).
 * @ingroup drv_pico4
 */

#pragma once

#include "pico4_ctrack_engine.hpp"
#include "pico4_ctrack_model.hpp"

#include <array>
#include <cstdint>
#include <string>

namespace pico4::ctrack {

/*!
 * The four tracking cameras from @p dir (pvr/mav0/cam<k>/sensor.yaml): the omnidirectional model
 * and T_BS, the camera's pose in the headset body frame. False with a message in @p error.
 */
bool
load_cameras(const std::string &dir, std::array<Camera, kCameras> &cams, std::string &error);

//! Both LED models with their normals (led-models.json, tooling/ledmodel.py). False with a message in @p error.
bool
load_led_models(const std::string &path, LedModel &left, LedModel &right, std::string &error);

/*!
 * PICO OS's lever arm of each hand in the aim frame, metres (lever-arm.json, tooling/leverarm.py),
 * indexed by Hand. False with a message in @p error.
 */
bool
load_lever_arms(const std::string &path, std::array<Vec3, 2> &lever, std::string &error);

/*!
 * A controller's calibration record as the station sends it (12 chunks of 32 bytes,
 * pico4_station_get_calibration), checked and split as PICO OS does:
 *
 * - the record is valid when imu_IR's fourth float is 1 and the first IMU float is not 0
 *   (CalibrationData::PrepareCalibrationData, both within 1e-8);
 * - chunks 1-3: the IMU block, 24 little-endian floats (ImuCalib::load's checks; applied only
 *   if both matrices' first elements are not 0);
 * - chunks 4-11: the 20 LED positions (finite, at most 100: updateLedPos);
 * - chunk 12: imu_IR, 7 floats, which the tracker does not use.
 *
 * False with a message in @p error if the record is invalid; @p unit's IMU and LED parts are
 * then left unset.
 */
bool
parse_station_calibration(const uint8_t chunks[12][32], UnitCalibration &unit, std::string &error);

/*!
 * A controller's gyro bias table (PICO OS's /mnt/vendor/pvr/<serial>.txt, readBias): 100 lines of
 * three numbers, rad/s at 0..99 degrees C. False with a message in @p error.
 */
bool
load_bias_table(const std::string &path, std::array<Vec3, 100> &table, std::string &error);

} // namespace pico4::ctrack
