// Copyright 2026, overpressure contributors.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  The controller tracker's calibration (see pico4_ctrack_calib.hpp).
 * @ingroup drv_pico4
 */

#include "pico4_ctrack_calib.hpp"

#include "pico4_tcam.h"

#include <cmath>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iterator>
#include <vector>

namespace pico4::ctrack {

namespace {

bool
read_text(const std::string &path, std::string &out)
{
	std::ifstream f(path, std::ios::binary);
	if (!f) {
		return false;
	}
	out.assign(std::istreambuf_iterator<char>(f), std::istreambuf_iterator<char>());
	return true;
}

//! @p n numbers after position @p pos of @p text (brackets, commas and white space skipped).
bool
numbers_after(const std::string &text, size_t pos, int n, std::vector<double> &out)
{
	out.clear();
	const char *p = text.c_str() + pos;
	while (*p != '\0' && (int)out.size() < n) {
		if (strchr("[], \t\r\n", *p) != nullptr) {
			p++;
			continue;
		}
		char *end;
		double v = strtod(p, &end);
		if (end == p) {
			return false;
		}
		out.push_back(v);
		p = end;
	}
	return (int)out.size() == n;
}

} // namespace

bool
load_cameras(const std::string &dir, std::array<Camera, kCameras> &cams, std::string &error)
{
	for (int k = 0; k < kCameras; k++) {
		std::string path = dir + "/pvr/mav0/cam" + std::to_string(k) + "/sensor.yaml", text;
		pico4_tcam_omni o;
		if (!read_text(path, text)) {
			error = "cannot read " + path;
			return false;
		}
		if (!pico4_tcam_parse_yaml(text.c_str(), &o) || o.width != kImageWidth || o.height != kImageHeight) {
			error = path + ": not a 640x480 omni camera";
			return false;
		}
		Camera &c = cams[k];
		c.xi = o.xi;
		c.fu = o.fu;
		c.fv = o.fv;
		c.pu = o.pu;
		c.pv = o.pv;
		c.k1 = o.k1;
		c.k2 = o.k2;
		c.p1 = o.p1;
		c.p2 = o.p2;
		Mat3 R;
		R << o.T_BS[0], o.T_BS[1], o.T_BS[2], o.T_BS[4], o.T_BS[5], o.T_BS[6], o.T_BS[8], o.T_BS[9], o.T_BS[10];
		c.T_HC = Pose(Quat(R), Vec3(o.T_BS[3], o.T_BS[7], o.T_BS[11]));
	}
	return true;
}

bool
load_led_models(const std::string &path, LedModel &left, LedModel &right, std::string &error)
{
	std::string text;
	if (!read_text(path, text)) {
		error = "cannot read " + path;
		return false;
	}
	size_t normals = text.find("\"normals\"");
	size_t pl = text.find("\"left\""), pr = text.find("\"right\"");
	size_t nl = normals == std::string::npos ? normals : text.find("\"left\"", normals);
	size_t nr = normals == std::string::npos ? normals : text.find("\"right\"", normals);
	std::vector<double> v;
	auto fill = [&](size_t key, std::array<Vec3, kLeds> &dst) {
		if (key == std::string::npos || !numbers_after(text, text.find(':', key) + 1, 3 * kLeds, v)) {
			return false;
		}
		for (int i = 0; i < kLeds; i++) {
			dst[i] = Vec3(v[3 * i], v[3 * i + 1], v[3 * i + 2]);
		}
		return true;
	};
	if (pl > normals || pr > normals || !fill(pl, left.position) || !fill(pr, right.position) ||
	    !fill(nl, left.normal) || !fill(nr, right.normal)) {
		error = path + ": no LED models with normals";
		return false;
	}
	return true;
}

bool
load_lever_arms(const std::string &path, std::array<Vec3, 2> &lever, std::string &error)
{
	std::string text;
	if (!read_text(path, text)) {
		error = "cannot read " + path;
		return false;
	}
	std::vector<double> v;
	const char *keys[2] = {"\"left\"", "\"right\""};
	for (int h = 0; h < 2; h++) {
		size_t key = text.find(keys[h]);
		if (key == std::string::npos || !numbers_after(text, text.find(':', key) + 1, 3, v)) {
			error = path + ": no lever arms";
			return false;
		}
		lever[h] = Vec3(v[0], v[1], v[2]);
	}
	return true;
}

namespace {

//! Float @p i of the record (little-endian), counting from the start of chunk 1.
float
record_float(const uint8_t chunks[12][32], int i)
{
	const uint8_t *b = &chunks[i / 8][(i % 8) * 4];
	uint32_t u = (uint32_t)b[0] | ((uint32_t)b[1] << 8) | ((uint32_t)b[2] << 16) | ((uint32_t)b[3] << 24);
	float f;
	std::memcpy(&f, &u, sizeof(f));
	return f;
}

} // namespace

bool
parse_station_calibration(const uint8_t chunks[12][32], UnitCalibration &unit, std::string &error)
{
	const int kImu = 0, kLedFloats = 3 * 8, kImuIr = 11 * 8;
	if (!(std::fabs(record_float(chunks, kImuIr + 3) - 1.0) < 1e-8) || !(std::fabs(record_float(chunks, kImu)) >= 1e-8)) {
		error = "calibration record not prepared (imu_IR[3] != 1 or imu[0] == 0)";
		return false;
	}
	double g[12], a[12];
	for (int i = 0; i < 12; i++) {
		g[i] = record_float(chunks, kImu + i);
		a[i] = record_float(chunks, kImu + 12 + i);
	}
	unit.imu_valid = g[0] != 0.0 && a[0] != 0.0 && unit.imu.load(g, a);

	unit.leds_valid = true;
	for (int k = 0; k < kLeds; k++) {
		for (int c = 0; c < 3; c++) {
			const double v = record_float(chunks, kLedFloats + 3 * k + c);
			unit.leds_valid = unit.leds_valid && std::isfinite(v) && v <= 100.0;
			unit.leds[k][c] = v;
		}
	}
	if (!unit.imu_valid || !unit.leds_valid) {
		error = std::string("calibration record rejected:") + (unit.imu_valid ? "" : " IMU") + (unit.leds_valid ? "" : " LEDs");
		return false;
	}
	return true;
}

bool
load_bias_table(const std::string &path, std::array<Vec3, 100> &table, std::string &error)
{
	std::string text;
	if (!read_text(path, text)) {
		error = "cannot read " + path;
		return false;
	}
	std::vector<double> v;
	if (!numbers_after(text, 0, 300, v)) {
		error = path + ": not 100 lines of three numbers";
		return false;
	}
	for (int i = 0; i < 100; i++) {
		table[i] = Vec3(v[3 * i], v[3 * i + 1], v[3 * i + 2]);
	}
	return true;
}

} // namespace pico4::ctrack
