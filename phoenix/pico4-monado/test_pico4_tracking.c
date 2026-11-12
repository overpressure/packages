// Copyright 2026, overpressure contributors.
// SPDX-License-Identifier: BSL-1.0
/*
 * Unit test of the tracking-camera helpers with synthetic values (no device
 * data): the station frame builder (golden frames from the Python tool that
 * was verified on hardware, and the allowlist), the vibration frames (golden
 * frames from the documented layout), the factory calibration
 * parser, the Kannala-Brandt refit, the controller packet parser, the
 * camera metadata line, the room view's shared frames, and the pairing
 * frames and control socket (with a fake station).
 */

#include "pico4_blemcu.h"
#include "pico4_cam.h"
#include "pico4_passthrough.h"
#include "pico4_roomview.h"
#include "pico4_station.h"
#include "pico4_tcam.h"

#include <errno.h>
#include <fcntl.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/un.h>
#include <time.h>
#include <unistd.h>

static int failures;

#define EXPECT(cond)                                                                                                   \
	do {                                                                                                           \
		if (!(cond)) {                                                                                         \
			fprintf(stderr, "%s:%d: expected %s\n", __FILE__, __LINE__, #cond);                            \
			failures++;                                                                                    \
		}                                                                                                      \
	} while (0)

static void
test_station(void)
{
	uint8_t f[PICO4_STATION_FRAME];
	const uint8_t init[4] = {0x01, 0x00, 0x02, 0x00}, on[3] = {0x02, 0x00, 0x01}, fps[3] = {0x01, 0x00, 0x01};
	const uint8_t ack[2] = {0x12, 0x34};

	EXPECT(pico4_station_frame(f, PICO4_STATION_CMD_CAMERA, init, sizeof(init)));
	EXPECT(f[0] == 0xAA && f[1] == 0x01 && f[2] == 0x04 && f[3] == 0x01 && f[5] == 0x02 && f[39] == 0x77);
	EXPECT(pico4_station_frame(f, PICO4_STATION_CMD_CAMERA, on, sizeof(on)));
	EXPECT(f[3] == 0x02 && f[5] == 0x01 && f[39] == 0x06);
	EXPECT(pico4_station_frame(f, PICO4_STATION_CMD_CAMERA_FPS, fps, sizeof(fps)));
	EXPECT(f[2] == 0x19 && f[39] == 0x00);
	EXPECT(pico4_station_frame(f, PICO4_STATION_CMD_ACK, ack, sizeof(ack)));
	EXPECT(f[2] == 0xFA && f[3] == 0x12 && f[4] == 0x34 && f[39] == 0x28);

	// The controllers' slow advertising as PICO OS's goToSleep sends it, and only that.
	const uint8_t slow_adv[2] = {0, 0}, slow_tracker[2] = {4, 0}, slow_other[2] = {0, 1};
	EXPECT(pico4_station_frame(f, PICO4_STATION_CMD_SLOW_ADV, slow_adv, sizeof(slow_adv)));
	EXPECT(f[0] == 0xAA && f[1] == 0x01 && f[2] == 0x0D && f[3] == 0 && f[4] == 0 && f[39] == 0x0A);
	EXPECT(!pico4_station_frame(f, PICO4_STATION_CMD_SLOW_ADV, slow_tracker, sizeof(slow_tracker)));
	EXPECT(!pico4_station_frame(f, PICO4_STATION_CMD_SLOW_ADV, slow_other, sizeof(slow_other)));
	EXPECT(!pico4_station_frame(f, PICO4_STATION_CMD_SLOW_ADV, slow_adv, 1));

	// Nothing outside the allowlist: other commands, camera ops and payload shapes.
	const uint8_t version[1] = {0};
	const uint8_t cam_fps[3] = {0x07, 0x00, 0x01}, bad_stream[3] = {0x02, 0x00, 0x03}, short_init[2] = {0x01, 0x00};
	const uint8_t bad_rate[3] = {0x03, 0x00, 0x01};
	EXPECT(!pico4_station_frame(f, 0x01, version, sizeof(version)));
	for (int cmd = 0; cmd < 256; cmd++) {
		if (cmd != PICO4_STATION_CMD_CAMERA && cmd != PICO4_STATION_CMD_CAMERA_FPS && cmd != PICO4_STATION_CMD_ACK) {
			EXPECT(!pico4_station_frame(f, (uint8_t)cmd, ack, sizeof(ack)));
		}
	}
	EXPECT(!pico4_station_frame(f, PICO4_STATION_CMD_CAMERA, cam_fps, sizeof(cam_fps)));
	EXPECT(!pico4_station_frame(f, PICO4_STATION_CMD_CAMERA, bad_stream, sizeof(bad_stream)));
	EXPECT(!pico4_station_frame(f, PICO4_STATION_CMD_CAMERA, short_init, sizeof(short_init)));
	uint8_t para[28] = {0x03};
	EXPECT(pico4_station_frame(f, PICO4_STATION_CMD_CAMERA, para, sizeof(para)));
	EXPECT(!pico4_station_frame(f, PICO4_STATION_CMD_CAMERA, para, 27));
	EXPECT(!pico4_station_frame(f, PICO4_STATION_CMD_CAMERA_FPS, bad_rate, sizeof(bad_rate)));
	EXPECT(!pico4_station_frame(f, PICO4_STATION_CMD_ACK, init, sizeof(init)));

	// CameraSetPara: scene setting for cameras 0-3, PICO OS's 50 Hz value (52) and the event
	// index, then the LED setting.
	const struct pico4_station_exposure scene = {300, 64}, leds = {4, 26}, long_leds = {50, 64};
	const struct pico4_station_exposure same[4] = {scene, scene, scene, scene};
	const uint8_t golden[28] = {0x03, 0x2C, 0x01, 0x40, 0x2C, 0x01, 0x40, 0x2C, 0x01, 0x40, 0x2C, 0x01, 0x40, 0x34, 0, 7,
	                            0x04, 0x00, 0x1A, 0x04, 0x00, 0x1A, 0x04, 0x00, 0x1A, 0x04, 0x00, 0x1A};
	uint8_t built[28];
	pico4_station_camera_para_each(built, same, leds, 7);
	EXPECT(memcmp(built, golden, sizeof(golden)) == 0);
	EXPECT(pico4_station_frame(f, PICO4_STATION_CMD_CAMERA, built, sizeof(built)));
	// The event index counts 0-9.
	pico4_station_camera_para_each(built, same, leds, 13);
	EXPECT(built[15] == 3);
	// The second set stays within PICO OS's LED limits (10 lines, gain 32).
	pico4_station_camera_para_each(built, same, long_leds, 0);
	EXPECT(built[16] == 10 && built[17] == 0 && built[18] == 32 && built[25] == 10 && built[27] == 32);
	EXPECT(built[1] == 0x2C && built[2] == 0x01 && built[3] == 0x40);
	// The modules' OTP ids as PICO OS's station service reads them: AA 01 04 05 01, then 05 02.
	const uint8_t otp_read[2] = {0x05, 0x01}, otp_result[2] = {0x05, 0x02}, otp_init[2] = {0x05, 0x03};
	EXPECT(pico4_station_frame(f, PICO4_STATION_CMD_CAMERA, otp_read, 2));
	EXPECT(f[0] == 0xAA && f[1] == 0x01 && f[2] == 0x04 && f[3] == 0x05 && f[4] == 0x01 && f[5] == 0);
	EXPECT(f[39] == pico4_station_crc8(f, 39));
	EXPECT(pico4_station_frame(f, PICO4_STATION_CMD_CAMERA, otp_result, 2) && f[4] == 0x02);
	EXPECT(!pico4_station_frame(f, PICO4_STATION_CMD_CAMERA, otp_init, 2)); // only the two PICO OS sends
	uint8_t otp[PICO4_STATION_FRAME] = {0xBB, 0x01, 0x04, 0x05, 9, 9, 0, 8};
	uint8_t ids[4] = {0, 0, 7, 0};
	EXPECT(pico4_station_parse_otp(otp, ids) && ids[0] == 9 && ids[1] == 9 && ids[2] == 7 && ids[3] == 8);
	otp[3] = 0x03;
	EXPECT(!pico4_station_parse_otp(otp, ids));

	// PICO OS's auto exposure: a setting per camera, in camera order (cameras 2 and 3 share one).
	const struct pico4_station_exposure each[4] = {{263, 48}, {737, 240}, {13, 16}, {13, 16}};
	pico4_station_camera_para_each(built, each, leds, 1);
	EXPECT(built[1] == 0x07 && built[2] == 0x01 && built[3] == 48);
	EXPECT(built[4] == 0xE1 && built[5] == 0x02 && built[6] == 240);
	EXPECT(built[7] == 13 && built[8] == 0 && built[9] == 16 && built[10] == 13 && built[12] == 16);
	EXPECT(built[13] == 0x34 && built[15] == 1 && built[16] == 4 && built[18] == 26);

	// Replies: header and CRC.
	uint8_t reply[PICO4_STATION_FRAME] = {0xBB, 0x01, 0x04, 0x01, 0x01};
	reply[39] = pico4_station_crc8(reply, 39);
	EXPECT(pico4_station_valid(reply));
	reply[10] ^= 1;
	EXPECT(!pico4_station_valid(reply));
}

//! A whole frame: AA, target, command, payload, zeros, CRC at byte 39.
static bool
frame_is(const uint8_t f[PICO4_STATION_FRAME], const uint8_t *head, size_t n, uint8_t crc)
{
	for (size_t i = n; i < 39; i++) {
		if (f[i] != 0) {
			return false;
		}
	}
	return memcmp(f, head, n) == 0 && f[39] == crc;
}

static void
test_station_vibrate(void)
{
	uint8_t f[PICO4_STATION_FRAME];

	// Plain motor command to the left controller: play, 200 × 50 % = 100, 300 ms.
	const uint8_t left[7] = {0xAA, 0x02, 0x01, 0x01, 0x64, 0x2C, 0x01};
	EXPECT(pico4_station_vibrate(f, PICO4_STATION_HAND_LEFT, 200, 50, 300));
	EXPECT(frame_is(f, left, sizeof(left), 0x81));

	// To the right controller: full level, 1000 ms.
	const uint8_t right[7] = {0xAA, 0x03, 0x01, 0x01, 0xFF, 0xE8, 0x03};
	EXPECT(pico4_station_vibrate(f, PICO4_STATION_HAND_RIGHT, 255, 100, 1000));
	EXPECT(frame_is(f, right, sizeof(right), 0xFC));

	// Level 0 stops: an all-zero payload, whatever else was asked.
	const uint8_t stop_left[3] = {0xAA, 0x02, 0x01}, stop_right[3] = {0xAA, 0x03, 0x01};
	EXPECT(pico4_station_vibrate(f, PICO4_STATION_HAND_LEFT, 0, 75, 500));
	EXPECT(frame_is(f, stop_left, sizeof(stop_left), 0x14));
	EXPECT(pico4_station_vibrate(f, PICO4_STATION_HAND_RIGHT, 0, 75, 500));
	EXPECT(frame_is(f, stop_right, sizeof(stop_right), 0xD3));

	// Clamps: level 255, amplitude 255, duration u16.
	const uint8_t clamped[7] = {0xAA, 0x03, 0x01, 0x01, 0xFF, 0xFF, 0xFF};
	EXPECT(pico4_station_vibrate(f, PICO4_STATION_HAND_RIGHT, 1000, 100, 70000));
	EXPECT(frame_is(f, clamped, sizeof(clamped), 0xAF));
	// The amplitude truncates: 3 × 75 / 100 = 2; 1 × 50 / 100 = 0 still plays, as PICO OS does.
	EXPECT(pico4_station_vibrate(f, PICO4_STATION_HAND_LEFT, 3, 75, 20) && f[3] == 1 && f[4] == 2);
	EXPECT(pico4_station_vibrate(f, PICO4_STATION_HAND_LEFT, 1, 50, 20) && f[3] == 1 && f[4] == 0);

	// Refused: other targets and a scale above 100 %; nothing is written.
	memset(f, 0x5A, sizeof(f));
	EXPECT(!pico4_station_vibrate(f, (enum pico4_station_hand)0, 100, 50, 100));
	EXPECT(!pico4_station_vibrate(f, (enum pico4_station_hand)3, 100, 50, 100));
	EXPECT(!pico4_station_vibrate(f, PICO4_STATION_HAND_LEFT, 100, 101, 100));
	EXPECT(f[0] == 0x5A && f[39] == 0x5A);

	// To the station, 01 (firmware version) and 07 (IMU stream) stay refused.
	const uint8_t motor[4] = {0x01, 0x64, 0x2C, 0x01}, event[6] = {0x01, 0x64, 0x2C, 0x01, 0xA0, 0x00};
	EXPECT(!pico4_station_frame(f, PICO4_STATION_CMD_MOTOR, motor, sizeof(motor)));
	EXPECT(!pico4_station_frame(f, 0x07, event, sizeof(event)));

	// The calibration query: golden frames from pico4-station (controller_query_frame).
	const uint8_t calib_left[4] = {0xAA, 0x02, 0x02, 0x01}, calib_right[4] = {0xAA, 0x03, 0x02, 0x0C};
	EXPECT(pico4_station_calibration_query(f, PICO4_STATION_HAND_LEFT, 1));
	EXPECT(frame_is(f, calib_left, sizeof(calib_left), 0x62));
	EXPECT(pico4_station_calibration_query(f, PICO4_STATION_HAND_RIGHT, 12));
	EXPECT(frame_is(f, calib_right, sizeof(calib_right), 0xC4));
	// Refused: chunk 0 or beyond 12, another target, and command 02 to the station with a chunk index
	// (to the station, 02 is pairing and takes a controller's mask, 1 or 2, only).
	memset(f, 0x5A, sizeof(f));
	EXPECT(!pico4_station_calibration_query(f, PICO4_STATION_HAND_LEFT, 0));
	EXPECT(!pico4_station_calibration_query(f, PICO4_STATION_HAND_LEFT, 13));
	EXPECT(!pico4_station_calibration_query(f, PICO4_STATION_HAND_RIGHT, 257));
	EXPECT(!pico4_station_calibration_query(f, (enum pico4_station_hand)3, 1));
	EXPECT(f[0] == 0x5A && f[39] == 0x5A);
	const uint8_t chunk[1] = {3};
	EXPECT(!pico4_station_frame(f, PICO4_STATION_CMD_CALIBRATION, chunk, sizeof(chunk)));

	// OpenXR mapping: XR_MIN_HAPTIC_DURATION is 500 ms, at least 10 ms, level = amplitude × 255.
	unsigned level = 99, ms = 99;
	pico4_station_haptic_map(1.0f, -1, &level, &ms);
	EXPECT(level == 255 && ms == 500);
	pico4_station_haptic_map(0.25f, 123456789, &level, &ms);
	EXPECT(level == 63 && ms == 123);
	pico4_station_haptic_map(0.5f, 2500000, &level, &ms);
	EXPECT(level == 127 && ms == 10);
	pico4_station_haptic_map(0.5f, 999999, &level, &ms); // under 1 ms: stop
	EXPECT(level == 0 && ms == 0);
	pico4_station_haptic_map(0.0f, 1000000000, &level, &ms);
	EXPECT(level == 0 && ms == 0);
	pico4_station_haptic_map(NAN, 1000000000, &level, &ms);
	EXPECT(level == 0 && ms == 0);
	pico4_station_haptic_map(2.0f, INT64_MAX, &level, &ms); // XR_INFINITE_DURATION
	EXPECT(level == 255 && ms == 65535);
}

// Same structure as the factory files; made-up numbers of the same magnitude.
static const char *yaml = "%YAML:1.0\n"
                          "sensor_type: camera\n"
                          "T_BS:\n"
                          "  cols: 4\n"
                          "  rows: 4\n"
                          "  data: [ 0.0, 0.0, 1.0, 0.01,\n"
                          "          1.0, 0.0, 0.0, 0.10,\n"
                          "          0.0, 1.0, 0.0, -0.02,\n"
                          "          0.0, 0.0, 0.0, 1.0]\n"
                          "rate_hz: 30\n"
                          "resolution: [640, 480]\n"
                          "camera_model: omni\n"
                          "intrinsics: [3.5,1050.0,1052.0,321.0,242.0] #xi fu fv pu pv\n"
                          "distortion_model: radtan\n"
                          "distortion_coefficients: [1.2,15.5,0.001,-0.002]";

static void
test_calibration(void)
{
	struct pico4_tcam_omni c;
	struct pico4_tcam_kb4 k;

	EXPECT(pico4_tcam_parse_yaml(yaml, &c));
	EXPECT(c.width == 640 && c.height == 480);
	EXPECT(c.xi == 3.5 && c.fu == 1050.0 && c.fv == 1052.0 && c.pu == 321.0 && c.pv == 242.0);
	EXPECT(c.k1 == 1.2 && c.k2 == 15.5 && c.p1 == 0.001 && c.p2 == -0.002);
	EXPECT(c.T_BS[2] == 1.0 && c.T_BS[3] == 0.01 && c.T_BS[7] == 0.10 && c.T_BS[11] == -0.02 && c.T_BS[15] == 1.0);

	// Other models are refused rather than misread.
	char other[2048];
	snprintf(other, sizeof(other), "%s", yaml);
	memcpy(strstr(other, "omni"), "pinh", 4);
	EXPECT(!pico4_tcam_parse_yaml(other, &c));
	EXPECT(pico4_tcam_parse_yaml(yaml, &c));

	// The optical axis lands on the principal point in both models.
	double axis[3] = {0, 0, 1}, uv[2];
	EXPECT(pico4_tcam_omni_project(&c, axis, uv));
	EXPECT(fabs(uv[0] - 321.0) < 1e-9 && fabs(uv[1] - 242.0) < 1e-9);

	EXPECT(pico4_tcam_fit_kb4(&c, &k));
	printf("KB4 fit: f %.2f %.2f, k %.4f %.4f %.4f %.4f, rms %.3f px, max %.3f px over %d rays\n", k.fx, k.fy,
	       k.k[0], k.k[1], k.k[2], k.k[3], k.rms_px, k.max_px, k.samples);
	EXPECT(k.cx == 321.0 && k.cy == 242.0);
	EXPECT(k.samples > 1000);
	EXPECT(k.rms_px < 0.5);
	EXPECT(k.max_px < 2.0);

	// Spot check off axis: 40 degrees right of the axis.
	double p[3] = {sin(0.698), 0, cos(0.698)}, a[2], b[2];
	EXPECT(pico4_tcam_omni_project(&c, p, a));
	pico4_tcam_kb4_project(&k, p, b);
	EXPECT(fabs(a[0] - b[0]) < 1.0 && fabs(a[1] - b[1]) < 1.0);

	// The RGB camera's model: omni with PICO's fisheye62 (k1..k6 on the angle of the
	// omni-normalised point, tangential terms on that point). Made-up values; the expected
	// pixel comes from an independent evaluation of the same formula.
	static const char rgb_yaml[] = "T_BS:\n"
	                               "  data: [ 0.0, -1.0, 0.0, 0.05,\n"
	                               "          -1.0, 0.0, 0.0, -0.02,\n"
	                               "          0.0, 0.0, -1.0, 0.01,\n"
	                               "          0.0, 0.0, 0.0, 1.0]\n"
	                               "resolution: [2328, 1748]\n"
	                               "camera_model: omni\n"
	                               "intrinsics: [0.77,1550.0,1551.0,1160.0,875.0]\n"
	                               "distortion_model: fisheye62\n"
	                               "distortion_coefficients: [0.1,-0.05,0.02,-0.01,0.003,-0.0005,0.0002,-0.0001]";
	struct pico4_tcam_omni rgb;
	EXPECT(pico4_tcam_parse_yaml(rgb_yaml, &rgb));
	EXPECT(rgb.distortion == PICO4_TCAM_FISHEYE62 && rgb.width == 2328 && rgb.height == 1748);
	EXPECT(rgb.k1 == 0.1 && rgb.k6 == -0.0005 && rgb.p1 == 0.0002 && rgb.p2 == -0.0001);
	double q[3] = {0.3, -0.2, 0.9}, quv[2];
	EXPECT(pico4_tcam_omni_project(&rgb, q, quv));
	EXPECT(fabs(quv[0] - 1439.252672) < 1e-5 && fabs(quv[1] - 688.721360) < 1e-5);
	EXPECT(pico4_tcam_omni_project(&rgb, axis, uv));
	EXPECT(fabs(uv[0] - 1160.0) < 1e-9 && fabs(uv[1] - 875.0) < 1e-9);
	struct pico4_tcam_kb4 rk;
	EXPECT(pico4_tcam_fit_kb4(&rgb, &rk));
	printf("RGB KB4 fit: rms %.3f px, max %.3f px over %d rays\n", rk.rms_px, rk.max_px, rk.samples);
	EXPECT(rk.samples > 1000);

	// A camera turned by 180 degrees sees a body point at the mirrored pixel.
	struct pico4_tcam_kb4 r = k;
	double T[16];
	memcpy(T, c.T_BS, sizeof(T));
	pico4_tcam_rotate_180(&r, T, c.width, c.height);
	EXPECT(!pico4_tcam_upside_down(c.T_BS, c.T_BS));
	EXPECT(pico4_tcam_upside_down(c.T_BS, T));
	double body[3] = {0.3, -0.2, 1.5}, pc[3], pr[3], u0[2], u1[2];
	for (int i = 0; i < 3; i++) {
		// Camera coordinates: R^T (p - t), R and t from the row-major T_BS.
		pc[i] = pr[i] = 0;
		for (int j = 0; j < 3; j++) {
			pc[i] += c.T_BS[j * 4 + i] * (body[j] - c.T_BS[j * 4 + 3]);
			pr[i] += T[j * 4 + i] * (body[j] - T[j * 4 + 3]);
		}
	}
	pico4_tcam_kb4_project(&k, pc, u0);
	pico4_tcam_kb4_project(&r, pr, u1);
	EXPECT(fabs(u1[0] - (c.width - 1 - u0[0])) < 1e-6 && fabs(u1[1] - (c.height - 1 - u0[1])) < 1e-6);

	// Upright for passthrough: each quarter turn sees a body point where the pixel mapping says.
	for (int t = 0; t < 4; t++) {
		// down_body along the camera's y, x, -y and -x picks turns 0, 1, 2 and 3.
		const int axis_col = t % 2 == 0 ? 1 : 0;
		const double sign = t < 2 ? 1 : -1;
		double down[3];
		for (int i = 0; i < 3; i++) {
			down[i] = sign * c.T_BS[i * 4 + axis_col];
		}
		struct pico4_tcam_kb4 q = k;
		memcpy(T, c.T_BS, sizeof(T));
		int w, h;
		EXPECT(pico4_tcam_upright(&q, T, c.width, c.height, down, &w, &h) == t);
		EXPECT(t % 2 == 0 ? (w == 640 && h == 480) : (w == 480 && h == 640));
		for (int i = 0; i < 3; i++) {
			pr[i] = 0;
			for (int j = 0; j < 3; j++) {
				pr[i] += T[j * 4 + i] * (body[j] - T[j * 4 + 3]);
			}
		}
		pico4_tcam_kb4_project(&q, pr, u1);
		// The source pixel of (u1) by the documented mapping is u0.
		double su = t == 0 ? u1[0] : t == 1 ? u1[1] : t == 2 ? c.width - 1 - u1[0] : c.width - 1 - u1[1];
		double sv = t == 0 ? u1[1] : t == 1 ? c.height - 1 - u1[0] : t == 2 ? c.height - 1 - u1[1] : u1[0];
		EXPECT(fabs(su - u0[0]) < 1e-6 && fabs(sv - u0[1]) < 1e-6);
	}
}

static void
test_blemcu(void)
{
	// Same layout as the radio packets: an IMU record with a time offset (right
	// controller), a tracker record to skip, keys and battery of the left one.
	uint8_t pkt[200] = {0x5A, 0x01, 0x00, 0x00, 0x78, 0x56, 0x34, 0x12, 0x01, 0x00, 0x00, 0x00};
	const uint8_t records[] = {
	    0x17, 0x00, 0x82, 0x80, 0x07,                         // IMU, right, 1920 us earlier
	    0xF5, 0xFF, 0xFC, 0xFF, 0x05, 0x00,                   // gyro -11 -4 5
	    0xC7, 0x03, 0xAB, 0x00, 0x0F, 0xFF,                   // accel 967 171 -241
	    0x0A, 0xFF, 0xA1, 0x00, 0xD0, 0x07,                   // temperature, counter, range
	    0x06, 0x00, 0x03, 0x01, 0x02, 0x03,                   // tracker 3: ignored
	    0x0A, 0x01, 0x01, 0x40, 0x10, 0xF0, 0x80, 0xA4, 0x15, 0x02, // keys, left
	    0x04, 0x02, 0x01, 0x05,                               // battery, left
	};
	memcpy(pkt + 13, records, sizeof(records));
	pkt[12] = sizeof(records);

	struct pico4_blemcu_packet p;
	EXPECT(pico4_blemcu_parse(pkt, sizeof(pkt), &p));
	EXPECT(p.station_time_us == 0x112345678ULL);
	EXPECT(p.imu_count == 1 && p.imu[0].device == PICO4_BLEMCU_RIGHT && p.imu[0].offset_us == 1920);
	EXPECT(p.imu[0].gyro[0] == -11 && p.imu[0].gyro[1] == -4 && p.imu[0].gyro[2] == 5);
	EXPECT(p.imu[0].accel[0] == 967 && p.imu[0].accel[1] == 171 && p.imu[0].accel[2] == -241);
	EXPECT(p.imu[0].temperature == -246 && p.imu[0].counter == 161);
	EXPECT(p.key_count == 1 && p.keys[0].device == PICO4_BLEMCU_LEFT);
	EXPECT(p.keys[0].trigger == 0x40 && p.keys[0].stick_y == 0x10 && p.keys[0].stick_x == 0xF0);
	EXPECT(p.keys[0].grip == 0x80);
	EXPECT(p.keys[0].buttons == (PICO4_BLEMCU_BTN_TRIGGER | PICO4_BLEMCU_BTN_STICK | PICO4_BLEMCU_BTN_AX));
	EXPECT(p.keys[0].touch == (PICO4_BLEMCU_TOUCH_TRIGGER | PICO4_BLEMCU_TOUCH_AX | PICO4_BLEMCU_TOUCH_STICK));
	EXPECT(p.battery_count == 1 && p.battery[0].device == PICO4_BLEMCU_LEFT && p.battery[0].level == 5);

	// Scales as PICO OS derives them: 2000 deg/s and 16 g over 32768, both doubled for the
	// PICO 4's controllers. A raw gyro of 8192 is 1000 deg/s, an accelerometer of 1024 1 g.
	EXPECT(fabs(PICO4_BLEMCU_GYRO_LSB_PER_DPS - 32768.0 / (2 * 2000.0)) < 1e-12);
	EXPECT(fabs(PICO4_BLEMCU_ACCEL_LSB_PER_G - 32768.0 / (2 * 16.0)) < 1e-12);
	EXPECT(fabs(8192 / PICO4_BLEMCU_GYRO_LSB_PER_DPS - 1000.0) < 1e-9);
	EXPECT(fabs(1024 / PICO4_BLEMCU_ACCEL_LSB_PER_G - 1.0) < 1e-12);

	// Not a radio packet, or records running past the length.
	pkt[0] = 0x5B;
	EXPECT(!pico4_blemcu_parse(pkt, sizeof(pkt), &p));
	pkt[0] = 0x5A;
	EXPECT(!pico4_blemcu_parse(pkt, 20, &p));
	pkt[13] = 0x30; // first record claims more bytes than there are
	EXPECT(pico4_blemcu_parse(pkt, sizeof(pkt), &p) && p.imu_count == 0 && p.key_count == 0);
}

static void
test_station_connection(void)
{
	// A connect notification with the layout the station sends (A5 02), made-up contents.
	uint8_t f[PICO4_STATION_FRAME] = {0xA5, 0x02, 0x02, 'P', 'R', '0', '3', '2', '6', 0x11, '0', '3', 0x06};
	memcpy(f + 13, "TESTSERIAL0000001", 17);
	f[39] = pico4_station_crc8(f, 39);
	enum pico4_station_hand hand = 0;
	struct pico4_station_controller c;
	EXPECT(pico4_station_parse_connection(f, &hand, &c) == 1);
	EXPECT(hand == PICO4_STATION_HAND_RIGHT && c.connected);
	EXPECT(strcmp(c.software, "PR0326") == 0 && strcmp(c.serial, "TESTSERIAL0000001") == 0);

	// Connected but invalid (status 2) is no connection; a disconnect names the hand.
	f[9] = 0x12;
	EXPECT(pico4_station_parse_connection(f, &hand, &c) == -1);
	const uint8_t off[4] = {0xA5, 0x03, 0x01, 0x05};
	memset(f, 0, sizeof(f));
	memcpy(f, off, sizeof(off));
	EXPECT(pico4_station_parse_connection(f, &hand, &c) == 0 && hand == PICO4_STATION_HAND_LEFT);
	f[2] = 3;
	EXPECT(pico4_station_parse_connection(f, &hand, &c) == -1);
	f[0] = 0xBB;
	f[2] = 1;
	EXPECT(pico4_station_parse_connection(f, &hand, &c) == -1);
}

static void
test_blemcu_order(void)
{
	// Samples stamped at or after "now" are dropped, the others keep their order.
	struct pico4_blemcu_packet p = {0};
	const uint32_t offsets[] = {4000, 0, 2000};
	for (int i = 0; i < 3; i++) {
		p.imu[i].device = PICO4_BLEMCU_RIGHT;
		p.imu[i].offset_us = offsets[i];
	}
	p.imu_count = 3;
	EXPECT(pico4_blemcu_drop_future(&p, 10000000, 10000000) == 1);
	EXPECT(p.imu_count == 2 && p.imu[0].offset_us == 4000 && p.imu[1].offset_us == 2000);
	EXPECT(pico4_blemcu_drop_future(&p, 10000000, 7000000) == 1 && p.imu_count == 1);
	EXPECT(pico4_blemcu_drop_future(&p, 10000000, 20000000) == 0 && p.imu_count == 1);

	// Duplicates and backward steps are rejected and counted.
	struct pico4_blemcu_order o = {0};
	EXPECT(!pico4_blemcu_order_fresh(&o, 1000));
	EXPECT(pico4_blemcu_order_accept(&o, 1000000));
	EXPECT(!pico4_blemcu_order_accept(&o, 1000000));
	EXPECT(!pico4_blemcu_order_accept(&o, 998000));
	EXPECT(pico4_blemcu_order_accept(&o, 1002000));
	EXPECT(o.newest_ns == 1002000 && o.rejected == 2);

	// Fresh until the newest sample is 0.2 s old.
	EXPECT(pico4_blemcu_order_fresh(&o, 1002000 + PICO4_BLEMCU_STALE_NS - 1));
	EXPECT(!pico4_blemcu_order_fresh(&o, 1002000 + PICO4_BLEMCU_STALE_NS));
}

//! A metadata line with gain @p gain, exposure @p lines and phase register @p phase as 10-bit samples.
static void
meta_line(uint8_t *line, unsigned gain, unsigned lines, unsigned phase)
{
	uint16_t w[PICO4_CAM_WIDTH] = {0};
	w[4] = (uint16_t)(gain << 2);
	w[6] = (uint16_t)(((lines >> 8) & 0xf) << 2);
	w[7] = (uint16_t)((lines & 0xff) << 2);
	w[82] = (uint16_t)(phase << 2);
	for (int i = 0; i < PICO4_CAM_WIDTH; i++) {
		line[2 * i] = (uint8_t)(w[i] & 0xff);
		line[2 * i + 1] = (uint8_t)(w[i] >> 8);
	}
}

static void
test_meta(void)
{
	uint8_t line[2 * PICO4_CAM_WIDTH];
	struct pico4_cam_meta m;

	// Values seen on the headset: a scene frame at 300 lines and gain 32, phase register 0x51.
	meta_line(line, 32, 300, 0x51);
	EXPECT(line[8] == 0x80 && line[12] == 0x04 && line[14] == 0xb0 && line[164] == 0x44 && line[165] == 0x01);
	EXPECT(pico4_cam_meta_decode(line, sizeof(line), &m));
	EXPECT(m.gain == 32 && m.exposure_lines == 300 && m.phase == PICO4_CAM_PHASE_SCENE && m.tag == 5);
	// A controller-LED frame: 4 lines, gain 26, phase register 0x72.
	meta_line(line, 26, 4, 0x72);
	EXPECT(pico4_cam_meta_decode(line, sizeof(line), &m));
	EXPECT(m.gain == 26 && m.exposure_lines == 4 && m.phase == PICO4_CAM_PHASE_LEDS && m.tag == 7);
	// Long exposures use the upper bits.
	meta_line(line, 248, 570, 0x01);
	EXPECT(pico4_cam_meta_decode(line, sizeof(line), &m) && m.exposure_lines == 570 && m.gain == 248);
	// No known phase, a sample above 10 bits (scene data, not metadata), a short line.
	meta_line(line, 32, 300, 0x53);
	EXPECT(!pico4_cam_meta_decode(line, sizeof(line), &m));
	meta_line(line, 32, 300, 0x51);
	line[9] = 0x04; // word 4 = 0x480
	EXPECT(!pico4_cam_meta_decode(line, sizeof(line), &m));
	meta_line(line, 32, 300, 0x51);
	EXPECT(!pico4_cam_meta_decode(line, 82 * 2, &m));
	EXPECT(!pico4_cam_meta_decode(NULL, sizeof(line), &m));
}

static void
test_median(void)
{
	// One camera covered (dark) or saturated does not move the group's signal much.
	const float covered[4] = {240, 3, 255, 250}, saturated[4] = {240, 1000, 255, 250}, odd[3] = {9, 1, 5};
	EXPECT(fabsf(pico4_cam_median(covered, 4) - 245.0f) < 0.01f);
	EXPECT(fabsf(pico4_cam_median(saturated, 4) - 252.5f) < 0.01f);
	EXPECT(fabsf(pico4_cam_median(odd, 3) - 5.0f) < 0.01f);
	EXPECT(pico4_cam_median(odd, 0) == 0.0f);
}

static void
test_set_assembly(void)
{
	const int64_t ms = 1000000, window = 4 * ms;
	struct pico4_cam_assembly a = {0};
	bool dropped;
	// Four cameras within 4 ms of the first: one complete set.
	EXPECT(pico4_cam_assembly_add(&a, 0, 100 * ms, window, 4, &dropped) == PICO4_CAM_SET_ADDED && !dropped);
	EXPECT(pico4_cam_assembly_add(&a, 2, 101 * ms, window, 4, &dropped) == PICO4_CAM_SET_ADDED);
	EXPECT(pico4_cam_assembly_add(&a, 2, 102 * ms, window, 4, &dropped) == PICO4_CAM_SET_DUPLICATE);
	EXPECT(pico4_cam_assembly_add(&a, 1, 103 * ms, window, 4, &dropped) == PICO4_CAM_SET_ADDED);
	EXPECT(pico4_cam_assembly_add(&a, 3, 99 * ms, window, 4, &dropped) == PICO4_CAM_SET_COMPLETE && !dropped);
	pico4_cam_assembly_reset(&a);
	// Camera 3 misses the pulse: the next pulse's first frame drops the incomplete set and
	// starts a new one, which camera 3 then completes.
	EXPECT(pico4_cam_assembly_add(&a, 0, 140 * ms, window, 4, &dropped) == PICO4_CAM_SET_ADDED);
	EXPECT(pico4_cam_assembly_add(&a, 1, 140 * ms, window, 4, &dropped) == PICO4_CAM_SET_ADDED);
	EXPECT(pico4_cam_assembly_add(&a, 2, 141 * ms, window, 4, &dropped) == PICO4_CAM_SET_ADDED && !dropped);
	EXPECT(pico4_cam_assembly_add(&a, 1, 180 * ms, window, 4, &dropped) == PICO4_CAM_SET_ADDED && dropped);
	EXPECT(a.have == 1 && a.ts == 180 * ms && a.slot[1] && !a.slot[0]);
	// A frame of the dropped pulse arriving late is skipped, not mixed into the new set.
	EXPECT(pico4_cam_assembly_add(&a, 3, 141 * ms, window, 4, &dropped) == PICO4_CAM_SET_LATE && !dropped);
	EXPECT(pico4_cam_assembly_add(&a, 0, 180 * ms, window, 4, &dropped) == PICO4_CAM_SET_ADDED);
	EXPECT(pico4_cam_assembly_add(&a, 2, 181 * ms, window, 4, &dropped) == PICO4_CAM_SET_ADDED);
	EXPECT(pico4_cam_assembly_add(&a, 3, 182 * ms, window, 4, &dropped) == PICO4_CAM_SET_COMPLETE);
}

static void
test_set_identity(void)
{
	struct pico4_cam_set_check c = {0};
	struct pico4_cam_meta m[4] = {{32, 300, 1, 5}, {32, 300, 1, 5}, {32, 300, 1, 5}, {32, 300, 1, 5}};
	char why[160];
	// Counters started at different times; scene frames every other pulse.
	uint64_t ids[4] = {100, 41, 7, 260};
	EXPECT(pico4_cam_set_consistent(&c, ids, m, 4, why, sizeof(why)) && why[0] == '\0');
	for (int i = 0; i < 4; i++) {
		ids[i] += 2;
	}
	EXPECT(pico4_cam_set_consistent(&c, ids, m, 4, why, sizeof(why)));
	// Camera 2's frame from the pulse before (a late frame mixed in): dropped.
	ids[0] += 2, ids[1] += 2, ids[3] += 2;
	EXPECT(!pico4_cam_set_consistent(&c, ids, m, 4, why, sizeof(why)) && strstr(why, "camera 2") != NULL);
	// Next pulse in step again: accepted with the old offsets.
	ids[0] += 2, ids[1] += 2, ids[2] += 4, ids[3] += 2;
	EXPECT(pico4_cam_set_consistent(&c, ids, m, 4, why, sizeof(why)) && why[0] == '\0');
	// Camera 1's counter skipped one for good: the first set is dropped, the second adopts it.
	ids[0] += 2, ids[1] += 3, ids[2] += 2, ids[3] += 2;
	EXPECT(!pico4_cam_set_consistent(&c, ids, m, 4, why, sizeof(why)));
	for (int i = 0; i < 4; i++) {
		ids[i] += 2;
	}
	EXPECT(pico4_cam_set_consistent(&c, ids, m, 4, why, sizeof(why)) && strstr(why, "adopted") != NULL);
	for (int i = 0; i < 4; i++) {
		ids[i] += 2;
	}
	EXPECT(pico4_cam_set_consistent(&c, ids, m, 4, why, sizeof(why)) && why[0] == '\0');
	// A camera that took the station's new exposure one frame later: two exposures in one set.
	for (int i = 0; i < 4; i++) {
		ids[i] += 2;
	}
	m[3].exposure_lines = 320;
	EXPECT(!pico4_cam_set_consistent(&c, ids, m, 4, why, sizeof(why)) && strstr(why, "camera 3 exposed 320") != NULL);
	m[3].exposure_lines = 300;
	m[1].gain = 48;
	EXPECT(!pico4_cam_set_consistent(&c, ids, m, 4, why, sizeof(why)) && strstr(why, "gain 48") != NULL);
	// Exposed each on its own, the frames of one pulse differ; their counters still decide.
	c.per_camera_exposure = true;
	EXPECT(pico4_cam_set_consistent(&c, ids, m, 4, why, sizeof(why)) && why[0] == '\0');
	ids[2] += 1;
	EXPECT(!pico4_cam_set_consistent(&c, ids, m, 4, why, sizeof(why)) && strstr(why, "camera 2") != NULL);
}

/*!
 * The RGB camera's cells into NV12: black level off, white balance, sRGB, full-range BT.601.
 * Expected values from an independent evaluation of the same steps (sRGB of 200/959 is 126).
 */
static void
test_rgb_convert(void)
{
	enum { W = 4, H = 2 }; // output pixels; the source has twice as many in each direction
	static uint16_t src[2 * H][2 * W];
	for (int y = 0; y < 2 * H; y++) {
		for (int x = 0; x < 2 * W; x++) {
			// RGGB: R at even/even, G at even/odd and odd/even, B at odd/odd.
			src[y][x] = (uint16_t)(64 + ((y % 2) == 0 ? ((x % 2) == 0 ? 200 : 400) : ((x % 2) == 0 ? 400 : 100)));
		}
	}
	uint8_t table[1024], nv12[W * H * 3 / 2];
	pico4_passthrough_srgb_table(table, 959);
	EXPECT(table[0] == 0 && table[959] == 255 && table[1023] == 255 && table[200] == 126);
	const float unity[3] = {1, 1, 1};
	double mean[3];
	pico4_passthrough_convert_rgb(&src[0][0], 2 * W, unity, table, nv12, W, H, mean);
	EXPECT(mean[0] == 200 && mean[1] == 400 && mean[2] == 100);
	EXPECT(nv12[0] == 150 && nv12[W * H - 1] == 150);
	EXPECT(nv12[W * H] == 95 && nv12[W * H + 1] == 111); // Cb, Cr of the first 2x2 block
	// Grey-world gains make the cells grey: no colour left.
	const float grey[3] = {2, 1, 4};
	pico4_passthrough_convert_rgb(&src[0][0], 2 * W, grey, table, nv12, W, H, mean);
	EXPECT(nv12[0] == 173 && nv12[W * H] == 128 && nv12[W * H + 1] == 128);
}

static void
test_passthrough(void)
{
	// A 640x480 raw image whose samples encode their own position: 16 (black) + x / 4 + y / 4.
	static uint16_t raw[PICO4_PASSTHROUGH_SRC_W * PICO4_PASSTHROUGH_SRC_H];
	static uint8_t out[PICO4_PASSTHROUGH_SRC_W * PICO4_PASSTHROUGH_SRC_H];
	for (int y = 0; y < PICO4_PASSTHROUGH_SRC_H; y++) {
		for (int x = 0; x < PICO4_PASSTHROUGH_SRC_W; x++) {
			raw[y * PICO4_PASSTHROUGH_SRC_W + x] = (uint16_t)(16 + 4 * ((x + y) % 64));
		}
	}
	raw[0] = 16 + 400; // the first sample: 100 at gain 1
	// Turn 1 (480 wide): (u, v) reads (v, h - 1 - u), so source (0, 0) lands at (479, 0).
	pico4_passthrough_convert(raw, PICO4_PASSTHROUGH_SRC_W, 1, 1.0f, out, 480);
	EXPECT(out[479] == 100);
	// Turn 3: (u, v) reads (w - 1 - v, u), so source (0, 0) lands at (0, 639).
	pico4_passthrough_convert(raw, PICO4_PASSTHROUGH_SRC_W, 3, 1.0f, out, 480);
	EXPECT(out[639 * 480] == 100);
	// Turn 0 at gain 2, with the black level off: 400 * 2 / 4 = 200; saturating at 255.
	pico4_passthrough_convert(raw, PICO4_PASSTHROUGH_SRC_W, 0, 2.0f, out, 640);
	EXPECT(out[0] == 200);
	pico4_passthrough_convert(raw, PICO4_PASSTHROUGH_SRC_W, 0, 4.0f, out, 640);
	EXPECT(out[0] == 255 && out[1] == 4);
}

static bool
fixed_pose(void *ctx, int64_t timestamp_ns, double out[12])
{
	(void)ctx;
	const double m[12] = {1, 0, 0, 0.5, 0, 1, 0, 1.5, 0, 0, 1, (double)(timestamp_ns % 1000)};
	memcpy(out, m, sizeof(m));
	return true;
}

static int64_t
now_ns(void)
{
	struct timespec ts;
	clock_gettime(CLOCK_MONOTONIC, &ts);
	return (int64_t)ts.tv_sec * 1000000000ll + ts.tv_nsec;
}

/*!
 * Pushes the pair until the driver's export thread reaches @p written frames, for up to a second.
 * A push that finds the export thread holding the lock is dropped by design (the camera thread
 * never waits), which happens now and then on a slow or emulated build machine.
 */
static bool
push_until_written(struct pico4_passthrough *p,
                   struct pico4_roomview *rv,
                   const uint16_t *left,
                   const uint16_t *right,
                   int64_t timestamp_ns,
                   uint64_t written)
{
	// Again only after 250 ms: a second push before the first is written would replace it.
	for (int i = 0; i < 1000; i++) {
		if (i % 250 == 0 && __atomic_load_n(&rv->frames_written, __ATOMIC_ACQUIRE) < written) {
			pico4_passthrough_push(p, left, right, PICO4_PASSTHROUGH_SRC_W, timestamp_ns, 100, 1.0f);
		}
		if (__atomic_load_n(&rv->frames_written, __ATOMIC_ACQUIRE) >= written) {
			return true;
		}
		usleep(1000);
	}
	return false;
}

static void
test_roomview(void)
{
	// The forward pair's calibration as the factory directory holds it.
	char dir[] = "/tmp/pico4-roomview-test-XXXXXX";
	EXPECT(mkdtemp(dir) != NULL);
	char path[512];
	for (int cam = 2; cam <= 3; cam++) {
		snprintf(path, sizeof(path), "%s/pvr", dir);
		mkdir(path, 0700);
		snprintf(path, sizeof(path), "%s/pvr/mav0", dir);
		mkdir(path, 0700);
		snprintf(path, sizeof(path), "%s/pvr/mav0/cam%d", dir, cam);
		mkdir(path, 0700);
		snprintf(path, sizeof(path), "%s/pvr/mav0/cam%d/sensor.yaml", dir, cam);
		FILE *f = fopen(path, "w");
		EXPECT(f != NULL);
		fputs(yaml, f);
		fclose(f);
	}
	const double down[3] = {0, 0, -1};
	const double head[16] = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
	struct pico4_passthrough *p = pico4_passthrough_create(dir, down, head);
	EXPECT(p != NULL);
	if (p == NULL) {
		return;
	}
	snprintf(path, sizeof(path), "%s/roomview", dir);
	EXPECT(pico4_passthrough_export(p, path, fixed_pose, NULL));
	EXPECT(pico4_passthrough_export(p, path, fixed_pose, NULL)); // once per passthrough
	int fd = open(path, O_RDWR);
	struct pico4_roomview *rv = mmap(NULL, sizeof(*rv), PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
	close(fd);
	EXPECT(rv != MAP_FAILED);
	EXPECT(rv->magic == PICO4_ROOMVIEW_MAGIC && rv->version == 2 && rv->generation == 1);
	EXPECT(rv->format == PICO4_ROOMVIEW_FORMAT_GREY);
	struct pico4_passthrough_info info;
	pico4_passthrough_info(p, &info);
	EXPECT(rv->width == info.width && rv->height == info.height && rv->fx[0] == info.fx[0]);
	EXPECT(rv->camera_to_head[1][3] == info.camera_to_head[1][3]);

	static uint16_t left[PICO4_PASSTHROUGH_SRC_W * PICO4_PASSTHROUGH_SRC_H];
	static uint16_t right[PICO4_PASSTHROUGH_SRC_W * PICO4_PASSTHROUGH_SRC_H];
	for (size_t i = 0; i < sizeof(left) / sizeof(left[0]); i++) {
		left[i] = 16 + 400;  // 100 at gain 1
		right[i] = 16 + 200; // 50
	}
	// Nobody asks: the camera thread hands nothing over.
	usleep(100 * 1000);
	EXPECT(!pico4_passthrough_streaming(p));
	// pico4-roomview asks for frames.
	__atomic_store_n(&rv->producer_ns, now_ns(), __ATOMIC_RELEASE);
	__atomic_store_n(&rv->requested, 1u, __ATOMIC_RELEASE);
	for (int i = 0; i < 200 && !pico4_passthrough_streaming(p); i++) {
		usleep(1000);
	}
	EXPECT(pico4_passthrough_streaming(p));
	EXPECT(push_until_written(p, rv, left, right, 123456789, 1));
	const struct pico4_roomview_frame *f = &rv->frames[0];
	EXPECT(f->sequence == 1 && f->timestamp_ns == 123456789 && f->pose_valid == 1);
	EXPECT(f->tracking_from_head[3] == 0.5 && f->tracking_from_head[11] == 789);
	// The two cameras one after the other, each width x height.
	EXPECT(f->pixels[0] == 100 && f->pixels[(size_t)info.width * info.height - 1] == 100);
	EXPECT(f->pixels[(size_t)info.width * info.height] == 50);
	EXPECT(push_until_written(p, rv, left, right, 223456789, 2));
	EXPECT(rv->frames[1].sequence == 2);
	// A producer that went quiet stops the stream.
	__atomic_store_n(&rv->producer_ns, now_ns() - 3 * PICO4_ROOMVIEW_PRODUCER_TIMEOUT_NS, __ATOMIC_RELEASE);
	for (int i = 0; i < 500 && pico4_passthrough_streaming(p); i++) {
		usleep(1000);
	}
	EXPECT(!pico4_passthrough_streaming(p));
	pico4_passthrough_destroy(&p);
	munmap(rv, sizeof(*rv));
	// A restart keeps the file and starts a new generation.
	p = pico4_passthrough_create(dir, down, head);
	EXPECT(p != NULL && pico4_passthrough_export(p, path, NULL, NULL));
	fd = open(path, O_RDONLY);
	rv = mmap(NULL, sizeof(*rv), PROT_READ, MAP_SHARED, fd, 0);
	close(fd);
	EXPECT(rv != MAP_FAILED && rv->generation == 2 && rv->frames[0].sequence == 0);
	munmap(rv, sizeof(*rv));
	pico4_passthrough_destroy(&p);
	unlink(path);
	for (int cam = 2; cam <= 3; cam++) {
		snprintf(path, sizeof(path), "%s/pvr/mav0/cam%d/sensor.yaml", dir, cam);
		unlink(path);
		snprintf(path, sizeof(path), "%s/pvr/mav0/cam%d", dir, cam);
		rmdir(path);
	}
	snprintf(path, sizeof(path), "%s/pvr/mav0", dir);
	rmdir(path);
	snprintf(path, sizeof(path), "%s/pvr", dir);
	rmdir(path);
	rmdir(dir);
}

static void
test_station_pairing_frames(void)
{
	// The bytes pico4-station sends (its golden frames): AA 01 cmd mask, zeros, CRC.
	uint8_t f[PICO4_STATION_FRAME];
	EXPECT(pico4_station_pairing_frame(f, PICO4_STATION_PAIR, PICO4_STATION_HAND_LEFT));
	EXPECT(f[0] == 0xAA && f[1] == 0x01 && f[2] == 0x02 && f[3] == 0x01 && f[39] == 0x32);
	for (int i = 4; i < 39; i++) {
		EXPECT(f[i] == 0);
	}
	EXPECT(pico4_station_pairing_frame(f, PICO4_STATION_PAIR, PICO4_STATION_HAND_RIGHT) && f[39] == 0x10);
	EXPECT(pico4_station_pairing_frame(f, PICO4_STATION_UNPAIR, PICO4_STATION_HAND_LEFT) && f[2] == 0x03 &&
	       f[39] == 0x47);
	EXPECT(pico4_station_pairing_frame(f, PICO4_STATION_PAIR_STOP, PICO4_STATION_HAND_RIGHT) && f[2] == 0x08 &&
	       f[39] == 0x79);
	// One controller's mask only: no tracker bits, both hands or nothing; no other command.
	EXPECT(!pico4_station_pairing_frame(f, PICO4_STATION_PAIR, (enum pico4_station_hand)3));
	EXPECT(!pico4_station_pairing_frame(f, PICO4_STATION_PAIR, (enum pico4_station_hand)4));
	EXPECT(!pico4_station_pairing_frame(f, PICO4_STATION_PAIR, (enum pico4_station_hand)0));
	EXPECT(!pico4_station_pairing_frame(f, (enum pico4_station_pairing)0x09, PICO4_STATION_HAND_LEFT));
	const uint8_t address[7] = {0x01, 1, 2, 3, 4, 5, 6}, mask[1] = {0x10};
	EXPECT(!pico4_station_frame(f, PICO4_STATION_UNPAIR, address, sizeof(address)));
	EXPECT(!pico4_station_frame(f, PICO4_STATION_PAIR, mask, sizeof(mask)));
}

static void
test_station_requests(void)
{
	struct pico4_station_request r;
	EXPECT(pico4_station_parse_request("pair left", 9, &r) && r.kind == PICO4_STATION_REQUEST_PAIR &&
	       r.hand == PICO4_STATION_HAND_LEFT && r.seconds == PICO4_STATION_PAIR_DEFAULT_S);
	EXPECT(pico4_station_parse_request("pair right 90\n", 14, &r) && r.hand == PICO4_STATION_HAND_RIGHT &&
	       r.seconds == 90);
	EXPECT(pico4_station_parse_request("unpair right", 12, &r) && r.kind == PICO4_STATION_REQUEST_UNPAIR);
	EXPECT(pico4_station_parse_request("status", 6, &r) && r.kind == PICO4_STATION_REQUEST_STATUS);
	const char *bad[] = {"", "pair", "pair both", "pair left 0", "pair left 601", "pair left -5", "pair left 5s",
	                     "unpair left 5", "status left", "pair left 5 6", "reset", "pair\tleft", "pairing left"};
	for (size_t i = 0; i < sizeof(bad) / sizeof(bad[0]); i++) {
		EXPECT(!pico4_station_parse_request(bad[i], strlen(bad[i]), &r));
	}
}

//! A station for the control socket: answers pairing commands, a controller that connects on demand.
struct fake_station
{
	int answer; //!< What pairing commands return.
	int ops[8], hands[8], count;
	struct pico4_station_controller controllers[2];
};

static int
fake_pairing(void *ctx, enum pico4_station_pairing op, enum pico4_station_hand hand)
{
	struct fake_station *s = ctx;
	if (s->count < 8) {
		s->ops[s->count] = op;
		s->hands[s->count++] = hand;
	}
	return s->answer;
}

static void
fake_get_controller(void *ctx, enum pico4_station_hand hand, struct pico4_station_controller *out)
{
	*out = ((struct fake_station *)ctx)->controllers[hand - 1];
}

//! Connect to the socket, send @p request and let the control take it.
static int
control_client(struct pico4_station_control *c, const char *path, const char *request, char *log, size_t size)
{
	int fd = socket(AF_UNIX, SOCK_SEQPACKET | SOCK_CLOEXEC, 0);
	struct sockaddr_un addr = {.sun_family = AF_UNIX};
	snprintf(addr.sun_path, sizeof(addr.sun_path), "%s", path);
	EXPECT(fd >= 0 && connect(fd, (const struct sockaddr *)&addr, sizeof(addr)) == 0);
	EXPECT(send(fd, request, strlen(request), 0) == (ssize_t)strlen(request));
	log[0] = '\0';
	pico4_station_control_step(c, 1000, log, size);
	return fd;
}

//! The next answer on @p fd, "" when the control hung up.
static void
control_answer(int fd, char *out, size_t size)
{
	ssize_t n = recv(fd, out, size - 1, MSG_DONTWAIT);
	out[n > 0 ? n : 0] = '\0';
}

static void
test_station_control(void)
{
	char dir[] = "/tmp/pico4-station-test.XXXXXX", path[64], log[128], got[128];
	EXPECT(mkdtemp(dir) != NULL);
	snprintf(path, sizeof(path), "%s/pico4-station", dir);
	struct fake_station s = {.answer = 0};
	struct pico4_station_ops ops = {fake_pairing, fake_get_controller, &s};
	struct pico4_station_control *c = NULL;
	EXPECT(pico4_station_control_open(&c, path, ops) == 0);
	struct stat sb;
	EXPECT(stat(path, &sb) == 0 && (sb.st_mode & 0777) == 0600);
	EXPECT(pico4_station_control_step(c, 0, log, sizeof(log)) == 0); // nobody there

	// Pairing: entered, then the controller connects (a new connection), then pairing stops.
	s.controllers[0].connects = 3;
	int fd = control_client(c, path, "pair left 30", log, sizeof(log));
	control_answer(fd, got, sizeof(got));
	EXPECT(strcmp(got, "pairing left") == 0 && strcmp(log, "pairing left") == 0);
	EXPECT(pico4_station_control_step(c, 2000, log, sizeof(log)) == 0); // still waiting
	s.controllers[0].connected = true;
	s.controllers[0].connects = 4;
	snprintf(s.controllers[0].serial, sizeof(s.controllers[0].serial), "TESTSERIAL0000001");
	EXPECT(pico4_station_control_step(c, 3000, log, sizeof(log)) > 0);
	control_answer(fd, got, sizeof(got));
	EXPECT(strcmp(got, "paired left TESTSERIAL0000001") == 0);
	EXPECT(s.count == 2 && s.ops[0] == PICO4_STATION_PAIR && s.ops[1] == PICO4_STATION_PAIR_STOP &&
	       s.hands[1] == PICO4_STATION_HAND_LEFT);
	control_answer(fd, got, sizeof(got));
	EXPECT(got[0] == '\0'); // hung up
	close(fd);

	// A controller connected before pairing does not count; the time runs out and pairing stops.
	s.count = 0;
	fd = control_client(c, path, "pair left 2", log, sizeof(log));
	control_answer(fd, got, sizeof(got));
	EXPECT(pico4_station_control_step(c, 2500, log, sizeof(log)) == 0);
	EXPECT(pico4_station_control_step(c, 3000, log, sizeof(log)) > 0);
	control_answer(fd, got, sizeof(got));
	EXPECT(strcmp(got, "timeout left") == 0 && s.count == 2 && s.ops[1] == PICO4_STATION_PAIR_STOP);
	close(fd);

	// A client that hangs up stops the pairing; nothing is sent to it.
	s.count = 0;
	fd = control_client(c, path, "pair right", log, sizeof(log));
	close(fd);
	EXPECT(pico4_station_control_step(c, 1100, log, sizeof(log)) > 0);
	EXPECT(strcmp(log, "cancelled right") == 0 && s.count == 2 && s.ops[1] == PICO4_STATION_PAIR_STOP);

	// Refused and unanswered commands; a refused pairing has no stop.
	s.count = 0;
	s.answer = -EIO;
	fd = control_client(c, path, "pair right", log, sizeof(log));
	control_answer(fd, got, sizeof(got));
	EXPECT(strcmp(got, "refused right") == 0 && s.count == 1);
	close(fd);
	s.answer = -ETIMEDOUT;
	fd = control_client(c, path, "unpair left", log, sizeof(log));
	control_answer(fd, got, sizeof(got));
	EXPECT(strcmp(got, "noreply left") == 0);
	close(fd);
	s.answer = 0;
	fd = control_client(c, path, "unpair left", log, sizeof(log));
	control_answer(fd, got, sizeof(got));
	EXPECT(strcmp(got, "unpaired left") == 0 && s.ops[s.count - 1] == PICO4_STATION_UNPAIR);
	close(fd);

	// Status, and requests it does not know.
	fd = control_client(c, path, "status", log, sizeof(log));
	control_answer(fd, got, sizeof(got));
	EXPECT(strcmp(got, "left connected TESTSERIAL0000001\nright disconnected") == 0);
	EXPECT(log[0] == '\0'); // asked every few seconds: not logged
	close(fd);
	s.count = 0;
	fd = control_client(c, path, "pair both", log, sizeof(log));
	control_answer(fd, got, sizeof(got));
	EXPECT(strcmp(got, "error request") == 0 && s.count == 0);
	close(fd);

	// A connection that sends nothing is dropped after its time.
	fd = socket(AF_UNIX, SOCK_SEQPACKET | SOCK_CLOEXEC, 0);
	struct sockaddr_un addr = {.sun_family = AF_UNIX};
	snprintf(addr.sun_path, sizeof(addr.sun_path), "%s", path);
	EXPECT(connect(fd, (const struct sockaddr *)&addr, sizeof(addr)) == 0);
	EXPECT(pico4_station_control_step(c, 0, log, sizeof(log)) == 0);
	EXPECT(pico4_station_control_step(c, PICO4_STATION_REQUEST_MS, log, sizeof(log)) == 0);
	control_answer(fd, got, sizeof(got));
	EXPECT(got[0] == '\0');
	close(fd);

	// Closing removes the socket and stops a pairing in progress.
	s.count = 0;
	fd = control_client(c, path, "pair left", log, sizeof(log));
	pico4_station_control_close(c);
	EXPECT(s.count == 2 && s.ops[1] == PICO4_STATION_PAIR_STOP);
	EXPECT(access(path, F_OK) != 0);
	close(fd);
	rmdir(dir);
}

int
main(void)
{
	test_station();
	test_station_vibrate();
	test_station_connection();
	test_station_pairing_frames();
	test_station_requests();
	test_station_control();
	test_calibration();
	test_blemcu();
	test_blemcu_order();
	test_meta();
	test_median();
	test_set_assembly();
	test_set_identity();
	test_rgb_convert();
	test_passthrough();
	test_roomview();
	if (failures != 0) {
		fprintf(stderr, "%d failure(s)\n", failures);
		return 1;
	}
	printf("ok\n");
	return 0;
}
