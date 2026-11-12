// Copyright 2026, overpressure contributors.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  PICO 4 6DoF head tracking through Monado's SLAM tracker (see pico4_slam.h).
 *
 * The four OV7251s (pico4_cam.h) only send frames while the controller radio
 * pulses their shared FSIN line (pico4_station.h), so all cameras expose
 * together. The station alternates two exposures: every other frame is a
 * short one in which only the controllers' infrared LEDs show (for
 * controller tracking). Each frame's first line is the sensor's metadata:
 * its exposure phase and the exposure and gain it was taken with
 * (pico4_cam_meta_decode). The LED frames are kept out of the tracker by that
 * phase and go, as sets of the four cameras, to the controller tracker
 * (pico4_ctrack.h) when there is one. Frames
 * of one pulse are grouped by their start-of-frame time, converted from 10 to
 * 8 bits (black level removed, slowly adapting digital gain) and pushed to the
 * tracker in camera order with one timestamp, as Basalt requires: the middle
 * of their exposures. The station also sets the exposure (the sensors' own
 * registers are ignored in this mode); its default of about 3 ms leaves
 * indoor scenes nearly black, so a slow auto-exposure loop raises it.
 *
 * Basalt's queues are bounded and block their producers when its optical flow
 * falls behind. Two threads therefore feed the tracker: one takes the newest
 * frame set from the camera thread (an older one still waiting is dropped),
 * the other drains a ring buffer of IMU samples from the HMD's IMU thread.
 * Neither capture nor the 3DoF and controller paths ever wait for Basalt, and
 * IMU samples keep flowing while a frame push blocks: Basalt's estimator waits
 * for them before its optical flow can take the next frame, so one thread
 * feeding both deadlocked. IMU samples arrive from
 * the HMD's sensor thread in the same body frame as the factory extrinsics.
 *
 * The default tracker is PICO OS's own (PICO4_SLAM_BACKEND=stock, docs/xr.md#head-tracking):
 * pico4-slamd runs it in its bionic root with its front end on the Compute DSP. This module
 * then writes each scene set's raw 10-bit frames (factory camera order, as read out) and every
 * IMU sample into the shared memory of pico4_slam_feed.h and takes the tracker's IMU-rate poses
 * back; Basalt is not started. Frame times come from the CSID's hardware start of frame on
 * CLOCK_MONOTONIC (patch 0016), the clock the IMU's samples are mapped to.
 *
 * @ingroup drv_pico4
 */

#include "pico4_slam.h"
#include "pico4_slam_feed.h"
#include "pico4_fusion.h"

#include "xrt/xrt_config_build.h"

#ifdef XRT_FEATURE_SLAM

#include "pico4_cam.h"
#include "pico4_ctrack.h"
#include "pico4_passthrough.h"
#include "pico4_station.h"
#include "pico4_tcam.h"

#include "xrt/xrt_frame.h"
#include "xrt/xrt_tracking.h"

#include "math/m_api.h"
#include "os/os_threading.h"
#include "os/os_time.h"
#include "tracking/t_tracking.h"
#include "util/u_debug.h"
#include "util/u_frame.h"
#include "util/u_misc.h"

#include <assert.h>
#include <errno.h>
#include <fcntl.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sched.h>
#include <sys/mman.h>
#include <sys/resource.h>
#include <sys/stat.h>
#include <sys/syscall.h>
#include <unistd.h>

DEBUG_GET_ONCE_BOOL_OPTION(pico4_slam, "PICO4_SLAM", true)
//! "stock": PICO OS's tracker through pico4-slamd; "basalt": Monado's SLAM tracker.
DEBUG_GET_ONCE_OPTION(pico4_slam_backend, "PICO4_SLAM_BACKEND", "stock")
//! Test only: restart the camera capture every N seconds, as after a stuck link (0: never).
DEBUG_GET_ONCE_NUM_OPTION(pico4_capture_restart_test_s, "PICO4_CAPTURE_RESTART_TEST_S", 0)
/*!
 * Factory cameras (cam0..cam3) given to the tracker, in tracker order. Basalt matches the
 * first camera against the others; all four keep up on the XR2 and see the most.
 */
DEBUG_GET_ONCE_OPTION(pico4_slam_cams, "PICO4_SLAM_CAMS", "0,1,2,3")
/*!
 * Sensor slot of each factory camera cam0..cam3 (top left, top right,
 * bottom right, bottom left as worn), found by covering one lens at a time.
 */
DEBUG_GET_ONCE_OPTION(pico4_tcam_slots, "PICO4_TCAM_SLOTS", "0,2,1,3")
//! Turn images that are upside down relative to the first camera's (the lower pair is mounted 180 degrees apart).
DEBUG_GET_ONCE_BOOL_OPTION(pico4_slam_upright, "PICO4_SLAM_UPRIGHT", true)
/*!
 * Use every Nth scene frame set (25 Hz: the station's 50 Hz alternates scene
 * and LED exposures) and IMU sample (1024 Hz). Basalt's optical flow keeps up
 * with 25 Hz on the XR2; it does not need the full IMU rate.
 */
DEBUG_GET_ONCE_NUM_OPTION(pico4_slam_divisor, "PICO4_SLAM_DIVISOR", 1)
DEBUG_GET_ONCE_NUM_OPTION(pico4_slam_imu_divisor, "PICO4_SLAM_IMU_DIVISOR", 2)
//! Directory to save the frames of every 500th set as PGM (diagnostics).
DEBUG_GET_ONCE_OPTION(pico4_slam_dump, "PICO4_SLAM_DUMP", NULL)
//! Mean 8-bit brightness the digital gain aims for.
DEBUG_GET_ONCE_NUM_OPTION(pico4_slam_brightness, "PICO4_SLAM_BRIGHTNESS", 70)
//! Longest exposure in sensor lines (19.07 us each): with the 9 ms readout a frame must fit
//! the 20 ms sync period, and longer exposures smear more when moving.
DEBUG_GET_ONCE_NUM_OPTION(pico4_slam_max_exposure, "PICO4_SLAM_MAX_EXPOSURE", 400)
//! Frame times at the middle of the exposure rather than at the start of frame (push_group).
DEBUG_GET_ONCE_BOOL_OPTION(pico4_slam_exposure_centre, "PICO4_SLAM_EXPOSURE_CENTRE", true)
/*!
 * SteamVR's room view in colour from the RGB camera (sensor slot 4), as PICO OS colours its
 * see-through, in place of the grey forward pair (docs/xr.md#passthrough). PICO4_ROOMVIEW_RGB=0
 * keeps the grey pair.
 */
DEBUG_GET_ONCE_BOOL_OPTION(pico4_roomview_rgb, "PICO4_ROOMVIEW_RGB", true)
#define LINE_NS 19069                 //!< One sensor line (stock).
#define SOF_AFTER_EXPOSURE_NS 52000   //!< Start-of-frame interrupt after the exposure's end (stock).
#define SOF_AFTER_LINES_NS 133483     //!< Seven more lines the stock service takes off (stock).
//! The stock tracker's poses are predicted at most this far past the newest one (as PICO OS).
#define STOCK_PREDICT_MAX_NS (45 * U_TIME_1MS_IN_NS)
//! PICO OS's auto exposure in pico4-slamd counts while its heartbeat is this fresh.
#define STOCK_AE_HEARTBEAT_NS (U_TIME_1S_IN_NS)
//! CPUs (bit mask) Basalt may use; 0 = no restriction. Default: all but cores 6 and 7.
DEBUG_GET_ONCE_NUM_OPTION(pico4_slam_cpus, "PICO4_SLAM_CPUS", 0x3f)
/*!
 * The head IMU's noise model for Basalt, continuous-time: accelerometer noise
 * density, accelerometer bias random walk, gyro noise density, gyro bias random
 * walk (Basalt multiplies the densities by the square root of the IMU rate).
 */
DEBUG_GET_ONCE_OPTION(pico4_slam_imu_noise, "PICO4_SLAM_IMU_NOISE", "0.016,0.001,0.0016,0.0001")
/*!
 * Basalt's VIO settings (a full VioConfig JSON: Basalt's defaults with the PICO 4's changes,
 * docs/xr.md); empty for Basalt's built-in defaults. The calibration comes from the driver.
 */
DEBUG_GET_ONCE_OPTION(pico4_slam_vio_config, "PICO4_SLAM_VIO_CONFIG", "/usr/share/pico4-basalt/pico4-vio.json")

#define SLAM_DEBUG(s, ...) U_LOG_IFL_D(s->log_level, __VA_ARGS__)
#define SLAM_INFO(s, ...) U_LOG_IFL_I(s->log_level, __VA_ARGS__)
#define SLAM_WARN(s, ...) U_LOG_IFL_W(s->log_level, __VA_ARGS__)
#define SLAM_ERROR(s, ...) U_LOG_IFL_E(s->log_level, __VA_ARGS__)

#define STATION_HZ 50.0
#define SCENE_HZ (STATION_HZ / 2) //!< Every other frame is a controller-LED exposure.
//! OV7251 10-bit black level.
#define BLACK_LEVEL 16
//! Auto exposure: mean signal (10-bit, above black) it aims for, and its limits.
#define AE_TARGET 250.0f
#define AE_MIN_LINES 10u
/*!
 * 15.5x, the sensors' highest analog gain. At 4x and 400 lines a moderately lit room
 * stayed near black (mean 5-10 of 255) and Basalt drifted or lost track; gain adds
 * noise but, unlike a longer exposure, no motion blur.
 */
#define AE_MAX_GAIN 248u
#define AE_SETS 25        //!< Sets per adjustment (half a second).
#define IMU_RING 2048
#define TRACKER_NICE 19 //!< Basalt and its feeders, below the compositor and the app (0).
//! Time Basalt gets at shutdown to process the frame sets still queued (10 at most).
#define TRACKER_DRAIN_NS (1000 * U_TIME_1MS_IN_NS)

//! Frames of one FSIN pulse start within this window.
#define GROUP_WINDOW_NS (4 * U_TIME_1MS_IN_NS)

static const struct xrt_vec3 P_BODY_HMD = PICO4_HMD_IN_BODY;
//! Basalt's world has Z up, OpenXR's Y up: -90 degrees about X.
static const struct xrt_quat Q_XR_W = {-0.70710678f, 0, 0, 0.70710678f};
//! The HMD axes (x right, y up, z back) in the body frame (x up, y left, z back).
static const struct xrt_quat Q_B_H = {0, 0, -0.70710678f, 0.70710678f};

struct pico4_slam
{
	enum u_logging_level log_level;

	struct xrt_frame_context xfctx;
	struct xrt_tracked_slam *tracker;
	struct xrt_slam_sinks *sinks;
	struct t_slam_calibration calib;

	struct pico4_station *station;
	struct pico4_cam *cam;
	struct pico4_cam_config cam_config; //!< As opened: the capture restarts with it.
	//! A restart of the capture asked for by the stall check, done by the camera thread.
	const char *restart_why;
	uint64_t capture_restarts;
	struct os_thread_helper oth;

	int cam_count;           //!< Cameras in the tracker.
	int index_of_slot[PICO4_CAM_NUM]; //!< Tracker index of each sensor slot, -1 if unused.
	bool rotated[PICO4_TCAM_COUNT];   //!< Image turned by 180 degrees (pico4_tcam_upside_down).
	unsigned divisor;

	//! The frame set being collected.
	struct
	{
		struct pico4_cam_assembly assembly; //!< Which cameras it has, by start of frame.
		struct xrt_frame *frames[PICO4_TCAM_COUNT];
		int64_t taken[PICO4_TCAM_COUNT]; //!< Middle of each frame's own exposure.
		int64_t sof[PICO4_TCAM_COUNT];   //!< Each frame's start of frame.
		float signal[PICO4_TCAM_COUNT];  //!< Mean signal above black of each frame (10-bit).
		struct pico4_cam_meta applied;   //!< The exposure the set was taken with.
		uint64_t frame_id[PICO4_TCAM_COUNT];          //!< Each frame's counter (V4L2 sequence).
		struct pico4_cam_meta meta[PICO4_TCAM_COUNT]; //!< Each frame's metadata.
	} group;
	//! What each camera's scene frames did since no set completed (scene_stall).
	struct
	{
		int64_t last_complete_ns, logged_ns;
		uint64_t frames[PICO4_TCAM_COUNT], late[PICO4_TCAM_COUNT], duplicate[PICO4_TCAM_COUNT];
		uint64_t sets_started;
		int64_t sof[PICO4_TCAM_COUNT];
		struct pico4_cam_meta meta[PICO4_TCAM_COUNT];
	} stall;
	//! Frame identity of the sets (pico4_cam_set_consistent).
	struct pico4_cam_set_check identity;
	uint64_t sets_pushed, sets_dropped, sets_busy, sequence;
	//! Complete sets dropped because their frames differ in exposure or frame identity.
	uint64_t sets_inconsistent;
	//! Frames skipped by their metadata: controller-LED or other phases, undecodable metadata.
	uint64_t frames_leds, frames_other, frames_bad;

	//! The controller tracker (borrowed) and the LED exposure set being collected for it.
	struct pico4_ctrack *ctrack;
	int factory_of_slot[PICO4_CAM_NUM]; //!< Factory camera of each sensor slot, -1 if none.
	struct
	{
		struct pico4_cam_assembly assembly;
		uint8_t *images[PICO4_TCAM_COUNT]; //!< 640x480, factory camera order.
		int64_t taken[PICO4_TCAM_COUNT];
	} leds;
	uint64_t led_sets, led_sets_dropped;
	int64_t max_spread_ns; //!< Largest spread of exposure middles within a set since the last report.

	//! Frame sets for the tracker, guarded by push_oth's lock.
	struct os_thread_helper push_oth;
	struct xrt_frame *mail[PICO4_TCAM_COUNT];
	bool mail_full;
	//! IMU samples for the tracker, guarded by imu_oth's lock.
	struct os_thread_helper imu_oth;
	struct xrt_imu_sample imu_ring[IMU_RING];
	unsigned imu_head, imu_len;
	uint64_t imu_overflow;

	//! Auto exposure: current station settings and the signal collected since.
	unsigned exposure_lines, exposure_gain, max_exposure;
	double ae_signal;
	unsigned ae_count;
	//! Newest IMU sample, to report the camera-IMU time skew.
	int64_t last_imu_ns;
	unsigned imu_divisor;
	uint64_t imu_count;
	//! The IMU rate declared to the tracker, before the divisor, and its measurement.
	double imu_hz;
	int64_t rate_start_ns;
	uint64_t rate_samples;
	bool rate_reported;
	float gain, brightness;

	//! CLOCK_MONOTONIC minus QTimer ns, for the hardware start-of-frame stamps.
	int64_t qtimer_offset_ns, qtimer_measured_ns, qtimer_first_offset_ns;
	int64_t slept_ns; //!< pico4_slept_ns() when the offset was taken.

	//! PICO OS's tracker through pico4-slamd (PICO4_SLAM_BACKEND=stock).
	bool stock;
	struct pico4_slam_feed *feed; //!< NULL until pico4-slamd's shared memory is there.
	uint32_t stock_generation;    //!< Ours, from the attach.
	uint32_t stock_room_logged;   //!< The room state last logged.
	uint32_t mesh_asking;         //!< Depth meshes asked for (pico4_slam_depth_mesh), as last logged.
	uint64_t stock_ae_seen;       //!< pico4-slamd's auto-exposure results taken (ae_written).
	bool stock_ae_logged;         //!< Whose auto exposure runs, as last logged.
	int64_t feed_retry_ns;
	uint64_t stock_sets, stock_imu, stock_lost_feed;
	//! SteamVR's camera: the forward pair's scene sets while a client streams (NULL without).
	struct pico4_passthrough *passthrough;
	//! The room view's RGB camera (PICO4_ROOMVIEW_RGB): the frame the passthrough reads, if any.
	bool rgb, rgb_holding;
	bool rgb_paused;        //!< Its stream paused at the sensor while nobody shows it.
	bool rgb_pause_refused; //!< The sensor refused a pause or resume: it streams on.
	int64_t rgb_wanted_ns;  //!< When the room view last asked for frames.
	struct pico4_cam_frame rgb_held;
	unsigned rgb_lines, rgb_gain; //!< Its exposure as last set.
	uint64_t rgb_frames, rgb_taken, rgb_exposures;
};

static void
stock_publish(struct pico4_slam *s);

//! Comma-separated indices below @p limit; returns the count, 0 on a parse error.
static int
parse_index_list(const char *text, int *out, int max, int limit)
{
	int n = 0;
	for (const char *p = text; *p != '\0' && n < max;) {
		char *end;
		long v = strtol(p, &end, 10);
		if (end == p || v < 0 || v >= limit) {
			return 0;
		}
		out[n++] = (int)v;
		p = *end == ',' ? end + 1 : end;
	}
	return n;
}

static char *
read_file(const char *path)
{
	FILE *f = fopen(path, "rb");
	if (f == NULL) {
		return NULL;
	}
	char *text = calloc(1, 1 << 16);
	if (text != NULL && fread(text, 1, (1 << 16) - 1, f) == 0) {
		free(text);
		text = NULL;
	}
	fclose(f);
	return text;
}

static bool
load_calibration(struct pico4_slam *s, const char *dir, const int *cams)
{
	double first_T_BS[16];
	s->calib.cam_count = s->cam_count;
	for (int i = 0; i < s->cam_count; i++) {
		char path[512];
		struct pico4_tcam_omni omni;
		struct pico4_tcam_kb4 kb4;
		snprintf(path, sizeof(path), "%s/pvr/mav0/cam%d/sensor.yaml", dir, cams[i]);
		char *text = read_file(path);
		bool ok = text != NULL && pico4_tcam_parse_yaml(text, &omni) && pico4_tcam_fit_kb4(&omni, &kb4);
		free(text);
		if (!ok) {
			SLAM_ERROR(s, "No usable camera calibration in %s", path);
			return false;
		}
		SLAM_INFO(s, "cam%d: KB4 refit of the factory omni model, %.2f px RMS (max %.2f) over %d rays", cams[i],
		          kb4.rms_px, kb4.max_px, kb4.samples);
		if (i == 0) {
			memcpy(first_T_BS, omni.T_BS, sizeof(first_T_BS));
		} else if (debug_get_bool_option_pico4_slam_upright() && pico4_tcam_upside_down(first_T_BS, omni.T_BS)) {
			// The lower pair is mounted 180 degrees apart; turn this image upright for stereo matching.
			pico4_tcam_rotate_180(&kb4, omni.T_BS, omni.width, omni.height);
			s->rotated[i] = true;
			SLAM_INFO(s, "cam%d: image turned by 180 degrees to match cam%d", cams[i], cams[0]);
		}

		struct t_slam_camera_calibration *c = &s->calib.cams[i];
		c->base.image_size_pixels.w = (uint32_t)omni.width;
		c->base.image_size_pixels.h = (uint32_t)omni.height;
		c->base.intrinsics[0][0] = kb4.fx;
		c->base.intrinsics[0][2] = kb4.cx;
		c->base.intrinsics[1][1] = kb4.fy;
		c->base.intrinsics[1][2] = kb4.cy;
		c->base.intrinsics[2][2] = 1;
		c->base.distortion_model = T_DISTORTION_FISHEYE_KB4;
		c->base.kb4.k1 = kb4.k[0];
		c->base.kb4.k2 = kb4.k[1];
		c->base.kb4.k3 = kb4.k[2];
		c->base.kb4.k4 = kb4.k[3];
		c->frequency = SCENE_HZ / s->divisor;

		// T_BS is the camera's pose in the body (IMU) frame: Basalt's T_i_c.
		const double *t = omni.T_BS;
		struct xrt_matrix_3x3 rot = {{(float)t[0], (float)t[1], (float)t[2], (float)t[4], (float)t[5], (float)t[6],
		                              (float)t[8], (float)t[9], (float)t[10]}};
		struct xrt_pose pose = {.position = {(float)t[3], (float)t[7], (float)t[11]}};
		math_quat_from_matrix_3x3(&rot, &pose.orientation);
		math_quat_normalize(&pose.orientation);
		math_matrix_4x4_isometry_from_pose(&pose, &c->T_imu_cam);
	}

	// The noise model (PICO4_SLAM_IMU_NOISE); Basalt estimates the biases.
	double n[4];
	const char *spec = debug_get_option_pico4_slam_imu_noise();
	if (sscanf(spec, "%lf,%lf,%lf,%lf", &n[0], &n[1], &n[2], &n[3]) != 4 || !(n[0] > 0 && n[1] > 0 && n[2] > 0 && n[3] > 0)) {
		SLAM_ERROR(s, "PICO4_SLAM_IMU_NOISE must be four positive numbers, not '%s'", spec);
		return false;
	}
	SLAM_INFO(s, "IMU noise: accelerometer %g (bias walk %g), gyro %g (bias walk %g)", n[0], n[1], n[2], n[3]);
	struct t_inertial_calibration accel = {
	    .transform = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}},
	    .bias_std = {n[1], n[1], n[1]},
	    .noise_std = {n[0], n[0], n[0]},
	};
	struct t_inertial_calibration gyro = {
	    .transform = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}},
	    .bias_std = {n[3], n[3], n[3]},
	    .noise_std = {n[2], n[2], n[2]},
	};
	s->calib.imu.base.accel = accel;
	s->calib.imu.base.gyro = gyro;
	s->calib.imu.frequency = s->imu_hz / s->imu_divisor;
	return true;
}

static void
drop_group(struct pico4_slam *s)
{
	for (int i = 0; i < PICO4_TCAM_COUNT; i++) {
		xrt_frame_reference(&s->group.frames[i], NULL);
	}
	pico4_cam_assembly_reset(&s->group.assembly);
}

//! Mean of a sparse grid of 10-bit samples.
static float
sample_mean(const struct pico4_cam_frame *f)
{
	uint64_t sum = 0, n = 0;
	for (uint32_t y = 8; y < f->height; y += 16) {
		const uint16_t *line = (const uint16_t *)((const uint8_t *)f->data + (size_t)y * f->stride);
		for (uint32_t x = 8; x < f->width; x += 16) {
			sum += line[x] & 0x3ff;
			n++;
		}
	}
	return n > 0 ? (float)sum / (float)n : 0;
}

static struct xrt_frame *
convert(struct pico4_slam *s, const struct pico4_cam_frame *f, bool rotate)
{
	struct xrt_frame *xf = NULL;
	u_frame_create_one_off(XRT_FORMAT_L8, f->width, f->height, &xf);
	if (xf == NULL) {
		return NULL;
	}
	// 10 -> 8 bits above the black level, with the gain in 1/256 steps.
	uint32_t g = (uint32_t)(s->gain * 256.0f);
	for (uint32_t y = 0; y < f->height; y++) {
		const uint16_t *in = (const uint16_t *)((const uint8_t *)f->data + (size_t)y * f->stride);
		uint8_t *out = xf->data + (size_t)(rotate ? f->height - 1 - y : y) * xf->stride;
		for (uint32_t x = 0; x < f->width; x++) {
			uint32_t raw = in[x] & 0x3ff;
			uint32_t v = raw > BLACK_LEVEL ? ((raw - BLACK_LEVEL) * g) >> 10 : 0;
			out[rotate ? f->width - 1 - x : x] = (uint8_t)(v > 255 ? 255 : v);
		}
	}
	return xf;
}

static void
dump_frame(const struct xrt_frame *xf, const char *dir, uint64_t set, int index)
{
	char path[512];
	snprintf(path, sizeof(path), "%s/set%llu-cam%d.pgm", dir, (unsigned long long)set, index);
	FILE *f = fopen(path, "wb");
	if (f == NULL) {
		return;
	}
	fprintf(f, "P5\n%u %u\n255\n", xf->width, xf->height);
	for (uint32_t y = 0; y < xf->height; y++) {
		fwrite(xf->data + (size_t)y * xf->stride, 1, xf->width, f);
	}
	fclose(f);
}

static void
push_group(struct pico4_slam *s)
{
	const char *dump = debug_get_option_pico4_slam_dump();
	s->sequence++;
	if (s->stock) {
		stock_publish(s);
		s->sets_pushed++;
		drop_group(s);
		return;
	}
	if (dump != NULL && s->sets_pushed % 500 == 100) {
		for (int i = 0; i < s->cam_count; i++) {
			dump_frame(s->group.frames[i], dump, s->sets_pushed, i);
		}
	}
	os_thread_helper_lock(&s->push_oth);
	if (s->mail_full) {
		s->sets_busy++; // the tracker has not taken the previous set yet
		for (int i = 0; i < s->cam_count; i++) {
			xrt_frame_reference(&s->mail[i], NULL);
		}
	}
	// One time for the set, as Basalt requires: the mean of the cameras' exposure middles.
	int64_t sum = 0, lo = INT64_MAX, hi = INT64_MIN;
	for (int i = 0; i < s->cam_count; i++) {
		int64_t t = s->group.taken[i];
		sum += t - s->group.taken[0];
		lo = t < lo ? t : lo;
		hi = t > hi ? t : hi;
	}
	int64_t taken = s->group.taken[0] + sum / s->cam_count;
	s->max_spread_ns = hi - lo > s->max_spread_ns ? hi - lo : s->max_spread_ns;
	for (int i = 0; i < s->cam_count; i++) {
		struct xrt_frame *xf = s->group.frames[i];
		xf->timestamp = taken;
		xf->source_timestamp = taken;
		xf->source_sequence = s->sequence;
		xf->source_id = i;
		s->mail[i] = xf; // the reference moves to the mailbox
		s->group.frames[i] = NULL;
	}
	s->mail_full = true;
	os_thread_helper_signal_locked(&s->push_oth);
	os_thread_helper_unlock(&s->push_oth);
	s->sets_pushed++;
	if (s->sets_pushed % 250 == 1) {
		int64_t imu = __atomic_load_n(&s->last_imu_ns, __ATOMIC_RELAXED);
		SLAM_INFO(s,
		          "frame set %llu: frame - newest IMU %.1f ms, latency %.1f ms, exposure %u lines gain %u, "
		          "digital gain %.1f; exposure middles within %.2f ms; skipped %llu LED, %llu other, "
		          "%llu undecodable frames; %llu incomplete and %llu inconsistent sets dropped",
		          (unsigned long long)s->sets_pushed, (s->group.assembly.ts - imu) / 1e6,
		          (os_monotonic_get_ns() - s->group.assembly.ts) / 1e6, s->exposure_lines, s->exposure_gain, s->gain,
		          s->max_spread_ns / 1e6, (unsigned long long)s->frames_leds, (unsigned long long)s->frames_other,
		          (unsigned long long)s->frames_bad, (unsigned long long)s->sets_dropped,
		          (unsigned long long)s->sets_inconsistent);
		s->max_spread_ns = 0;
	}
	drop_group(s);
}

/*!
 * Station exposure from the signal of the scene cameras (their median), exposure
 * first, then gain. @p applied is the exposure that signal was taken with: a command
 * still on its way does not count.
 */
static void
auto_exposure(struct pico4_slam *s, float signal, const struct pico4_cam_meta *applied)
{
	s->ae_signal += signal;
	if (++s->ae_count < AE_SETS) {
		return;
	}
	float mean = (float)(s->ae_signal / s->ae_count);
	s->ae_signal = 0;
	s->ae_count = 0;

	float total = (float)applied->exposure_lines * (float)applied->gain;
	if (total <= 0.0f) {
		total = (float)s->exposure_lines * (float)s->exposure_gain;
	}
	float want = total * AE_TARGET / (mean > 1.0f ? mean : 1.0f);
	if (want > total * 4) {
		want = total * 4; // the response is not quite linear; approach in steps
	}
	unsigned lines = (unsigned)(want / 16.0f), gain = 16;
	if (lines > s->max_exposure) {
		lines = s->max_exposure;
		gain = (unsigned)(want / (float)lines);
		gain = gain > AE_MAX_GAIN ? AE_MAX_GAIN : gain < 16 ? 16 : gain;
	}
	lines = lines < AE_MIN_LINES ? AE_MIN_LINES : lines;
	float change = (float)lines * (float)gain / total;
	if (change > 0.9f && change < 1.1f) {
		return;
	}
	if (pico4_station_camera_set_exposure(s->station, lines, gain) == 0) {
		SLAM_DEBUG(s, "exposure %u lines, gain %u (signal %.0f)", lines, gain, mean);
		s->exposure_lines = lines;
		s->exposure_gain = gain;
	}
}

/*!
 * The middle of a frame's exposure. These global-shutter sensors expose before
 * the readout that the start-of-frame interrupt marks; PICO's stock tracking
 * takes 52 us and half the frame's own exposure off that time. Up to 4 ms, and
 * changing with the exposure, otherwise put the images out of step with the IMU
 * while the head moves.
 */
static int64_t
exposure_middle(int64_t sof_ns, const struct pico4_cam_meta *m)
{
	if (!debug_get_bool_option_pico4_slam_exposure_centre()) {
		return sof_ns;
	}
	return sof_ns - SOF_AFTER_EXPOSURE_NS - SOF_AFTER_LINES_NS - (((int64_t)LINE_NS * m->exposure_lines) >> 1);
}

//! QTimer ticks (19.2 MHz) in ns, without overflow.
static uint64_t
qtimer_to_ns(uint64_t ticks)
{
	return (ticks / 12) * 625 + (ticks % 12) * 625 / 12;
}

/*!
 * CLOCK_MONOTONIC minus QTimer ns: the narrowest of a few (monotonic, arch counter) pairs. On
 * the XR2 the arch counter is the QTimer, as in pico4_hmd.c for the IMU's timestamps.
 */
static int64_t
qtimer_offset(void)
{
	int64_t best = INT64_MAX, offset = 0;
	for (int i = 0; i < 8; i++) {
		uint64_t ticks = 0;
		int64_t before = os_monotonic_get_ns();
#if defined(__aarch64__)
		__asm__ volatile("isb; mrs %0, cntvct_el0" : "=r"(ticks)::"memory");
#endif
		int64_t after = os_monotonic_get_ns();
		if (after - before < best) {
			best = after - before;
			offset = before + (after - before) / 2 - (int64_t)qtimer_to_ns(ticks);
		}
	}
	return offset;
}

/*!
 * A frame's start of frame on the IMU's clock, from the buffer's hardware stamp (patch 0016).
 */
static int64_t
frame_sof_ns(struct pico4_slam *s, const struct pico4_cam_frame *f)
{
	// Taken again each second: timesyncd steers CLOCK_MONOTONIC against the QTimer (pico4_hmd.c).
	// And at once after a suspend, through which the QTimer runs on (pico4_slept_ns()).
	int64_t now = os_monotonic_get_ns(), slept = pico4_slept_ns();
	if (now - s->qtimer_measured_ns >= U_TIME_1S_IN_NS || slept - s->slept_ns > PICO4_SLEEP_DETECT_NS) {
		s->qtimer_offset_ns = qtimer_offset();
		s->qtimer_measured_ns = now;
		s->slept_ns = slept;
	}
	return f->sof_qtimer_ns + s->qtimer_offset_ns;
}

//! A module id PICO OS's station service keeps (camera_otp.json): 8 or 9.
static bool
otp_valid(const uint8_t ids[PICO4_SLAM_FEED_CAMS])
{
	for (int k = 0; k < PICO4_SLAM_FEED_CAMS; k++) {
		if ((ids[k] & 0xfe) != 8) {
			return false;
		}
	}
	return true;
}

/*!
 * The cameras' module ids for pico4-slamd's lens shading, as PICO OS's station service gets them:
 * the saved ones if they are valid, else read from the modules' OTP through the station (once per
 * generation; the cameras run by now). pico4-slamd saves what it may keep.
 */
static void
stock_camera_otp(struct pico4_slam *s, struct pico4_slam_feed *feed)
{
	uint8_t saved[PICO4_SLAM_FEED_CAMS];
	memcpy(saved, feed->otp_saved, sizeof(saved));
	uint32_t next = __atomic_load_n(&feed->generation, __ATOMIC_ACQUIRE) + 1;
	if (otp_valid(saved)) {
		SLAM_INFO(s, "Camera module ids %u,%u,%u,%u (saved)", saved[0], saved[1], saved[2], saved[3]);
		feed->otp_state = 0;
	} else {
		uint8_t ids[PICO4_SLAM_FEED_CAMS] = {0};
		int r = s->station != NULL ? pico4_station_camera_otp(s->station, ids) : -ENODEV;
		SLAM_INFO(s, "Camera module ids from the OTP: %u,%u,%u,%u (%s)", ids[0], ids[1], ids[2], ids[3],
		          r == 0 ? "station answered" : strerror(-r));
		memcpy(feed->otp_read, ids, sizeof(ids));
		feed->otp_state = r == 0 ? 1 : 2;
	}
	__atomic_store_n(&feed->otp_generation, next, __ATOMIC_RELEASE);
}

/*!
 * Maps pico4-slamd's shared memory once it is there (camera thread; retried once a second) and
 * starts a new generation, which gives the driver a tracker of its own.
 */
static void
stock_attach(struct pico4_slam *s)
{
	if (__atomic_load_n(&s->feed, __ATOMIC_ACQUIRE) != NULL) {
		return;
	}
	int64_t now = os_monotonic_get_ns();
	if (now < s->feed_retry_ns) {
		return;
	}
	s->feed_retry_ns = now + U_TIME_1S_IN_NS;
	int fd = open(PICO4_SLAM_FEED_PATH, O_RDWR | O_CLOEXEC);
	if (fd < 0) {
		if (s->stock_lost_feed++ % 30 == 0) {
			SLAM_WARN(s, "No %s (%s): is pico4-slamd running?", PICO4_SLAM_FEED_PATH, strerror(errno));
		}
		return;
	}
	struct stat st;
	void *map = MAP_FAILED;
	if (fstat(fd, &st) == 0 && (size_t)st.st_size >= sizeof(struct pico4_slam_feed)) {
		map = mmap(NULL, sizeof(struct pico4_slam_feed), PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
	}
	close(fd);
	if (map == MAP_FAILED) {
		return;
	}
	struct pico4_slam_feed *feed = map;
	if (__atomic_load_n(&feed->magic, __ATOMIC_ACQUIRE) != PICO4_SLAM_FEED_MAGIC) {
		munmap(map, sizeof(struct pico4_slam_feed));
		return;
	}
	stock_camera_otp(s, feed);
	uint32_t generation = __atomic_add_fetch(&feed->generation, 1, __ATOMIC_ACQ_REL);
	s->stock_generation = generation;
	__atomic_store_n(&s->feed, feed, __ATOMIC_RELEASE);
	SLAM_INFO(s, "PICO OS's tracker through pico4-slamd (generation %u)", generation);
}

//! One scene frame's raw rows into the set being collected in the shared memory.
static void
stock_copy_frame(struct pico4_slam *s, const struct pico4_cam_frame *f)
{
	stock_attach(s);
	struct pico4_slam_feed *feed = s->feed;
	int k = s->factory_of_slot[f->slot];
	if (feed == NULL || k < 0 || f->width != PICO4_SLAM_FEED_W || f->height < PICO4_SLAM_FEED_H) {
		return;
	}
	uint64_t w = __atomic_load_n(&feed->sets_written, __ATOMIC_RELAXED);
	struct pico4_slam_feed_set *set = &feed->sets[w % PICO4_SLAM_FEED_SETS];
	for (uint32_t y = 0; y < PICO4_SLAM_FEED_H; y++) {
		memcpy(set->pixels[k] + (size_t)y * PICO4_SLAM_FEED_W, (const uint8_t *)f->data + (size_t)y * f->stride,
		       PICO4_SLAM_FEED_W * sizeof(uint16_t));
	}
}

//! Completes the collected set (times and exposures) and hands it to pico4-slamd.
static void
stock_publish(struct pico4_slam *s)
{
	struct pico4_slam_feed *feed = s->feed;
	if (feed == NULL) {
		return;
	}
	uint64_t w = __atomic_load_n(&feed->sets_written, __ATOMIC_RELAXED);
	struct pico4_slam_feed_set *set = &feed->sets[w % PICO4_SLAM_FEED_SETS];
	// As the stock service: the earliest exposure middle stands for all four.
	int64_t first = INT64_MAX;
	for (int i = 0; i < s->cam_count; i++) {
		first = s->group.taken[i] < first ? s->group.taken[i] : first;
	}
	for (int slot = 0; slot < PICO4_CAM_NUM; slot++) {
		int k = s->factory_of_slot[slot], i = s->index_of_slot[slot];
		if (k < 0 || i < 0) {
			continue;
		}
		set->sof_ns[k] = s->group.sof[i];
		set->exposure_lines[k] = s->group.meta[i].exposure_lines;
		set->gain[k] = s->group.meta[i].gain;
		set->frame_id[k] = s->group.frame_id[i];
	}
	set->ts_ns = first;
	__atomic_store_n(&feed->sets_written, w + 1, __ATOMIC_RELEASE);
	// The forward pair for SteamVR's camera, from the copies just made (bottom left cam3, right cam2).
	if (pico4_passthrough_streaming(s->passthrough)) {
		pico4_passthrough_push(s->passthrough, set->pixels[3], set->pixels[2], PICO4_SLAM_FEED_W, first,
		                       set->exposure_lines[3], s->gain);
	}
	s->stock_sets++;
	if (s->stock_sets % 250 == 1) {
		uint64_t poses = __atomic_load_n(&feed->poses_written, __ATOMIC_ACQUIRE);
		const struct pico4_slam_feed_pose *p = poses > 0 ? &feed->poses[(poses - 1) % PICO4_SLAM_FEED_POSES] : NULL;
		SLAM_INFO(s,
		          "frame set %llu to pico4-slamd: exposure %u lines gain %u, %llu IMU samples, %llu poses (newest "
		          "%.1f ms old, confidence %u, error %d); %llu incomplete and "
		          "%llu inconsistent sets dropped; clock offset moved %.2f ms since the start",
		          (unsigned long long)s->stock_sets, s->exposure_lines, s->exposure_gain,
		          (unsigned long long)s->stock_imu, (unsigned long long)poses,
		          p != NULL ? (os_monotonic_get_ns() - p->ts_ns) / 1e6 : -1.0, p != NULL ? p->confidence : 0,
		          p != NULL ? p->error : 0, (unsigned long long)s->sets_dropped,
		          (unsigned long long)s->sets_inconsistent,
		          (double)(s->qtimer_offset_ns - s->qtimer_first_offset_ns) / 1e6);
		if (s->rgb) {
			uint64_t taken, converted;
			int64_t convert_ns;
			pico4_passthrough_rgb_counts(s->passthrough, &taken, &converted, &convert_ns);
			SLAM_INFO(s,
			          "RGB camera: %llu frames, %llu to the room view, %llu converted (%.1f ms each); exposure %u "
			          "lines gain %u, %llu changes",
			          (unsigned long long)s->rgb_frames, (unsigned long long)taken, (unsigned long long)converted,
			          converted > 0 ? convert_ns / 1e6 / converted : 0.0, s->rgb_lines, s->rgb_gain,
			          (unsigned long long)s->rgb_exposures);
		}
	}
}

//! The HMD frame in the body frame (row-major 4x4): its axes are Q_B_H's columns, its origin P_BODY_HMD.
static void
body_head(double T_body_head[16], double down_body[3])
{
	const struct xrt_vec3 axes[3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
	struct xrt_vec3 col[3];
	for (int i = 0; i < 3; i++) {
		math_quat_rotate_vec3(&Q_B_H, &axes[i], &col[i]);
	}
	const double T[16] = {col[0].x, col[1].x, col[2].x, P_BODY_HMD.x, col[0].y, col[1].y, col[2].y, P_BODY_HMD.y,
	                      col[0].z, col[1].z, col[2].z, P_BODY_HMD.z, 0,        0,        0,        1};
	memcpy(T_body_head, T, sizeof(T));
	down_body[0] = -col[1].x;
	down_body[1] = -col[1].y;
	down_body[2] = -col[1].z;
}

//! SteamVR's camera from the forward pair (docs/xr.md#passthrough), placed in the HMD frame.
static struct pico4_passthrough *
create_passthrough(struct pico4_slam *s, const char *calibration_dir)
{
	double T_body_head[16], down_body[3];
	body_head(T_body_head, down_body);
	struct pico4_passthrough *p = pico4_passthrough_create(calibration_dir, down_body, T_body_head);
	if (p == NULL) {
		SLAM_WARN(s, "No passthrough: no calibration of the forward cameras in %s", calibration_dir);
		return NULL;
	}
	struct pico4_passthrough_info info;
	pico4_passthrough_info(p, &info);
	for (int i = 0; i < PICO4_PASSTHROUGH_CAMS; i++) {
		const double *m = info.camera_to_head[i];
		SLAM_INFO(s, "Passthrough %s camera: %ux%u upright (quarter turn %d), at %.3f %.3f %.3f m, looking %.2f %.2f %.2f",
		          i == 0 ? "left" : "right", info.width, info.height, info.turn[i], m[3], m[7], m[11], -m[2],
		          -m[6], -m[10]);
	}
	return p;
}

/*!
 * Whether pico4-slamd runs PICO OS's auto exposure for this driver generation and still runs: then
 * the driver's own stays off, and the cameras of a set may differ in exposure.
 */
static bool
stock_ae_active(struct pico4_slam *s)
{
	struct pico4_slam_feed *feed = s->feed;
	if (feed == NULL || __atomic_load_n(&feed->ae_generation, __ATOMIC_ACQUIRE) != s->stock_generation) {
		return false;
	}
	uint64_t beat = __atomic_load_n(&feed->slamd_heartbeat_ns, __ATOMIC_RELAXED);
	return os_monotonic_get_ns() - (int64_t)beat < STOCK_AE_HEARTBEAT_NS;
}

//! Sends the exposure PICO OS's auto exposure published last, once (camera thread).
static void
stock_ae_apply(struct pico4_slam *s)
{
	struct pico4_slam_feed *feed = s->feed;
	uint64_t w = __atomic_load_n(&feed->ae_written, __ATOMIC_ACQUIRE);
	if (w == s->stock_ae_seen) {
		return;
	}
	struct pico4_station_exposure scene[PICO4_SLAM_FEED_CAMS];
	for (int k = 0; k < PICO4_SLAM_FEED_CAMS; k++) {
		scene[k].lines = feed->ae_lines[k];
		scene[k].gain = feed->ae_gain[k];
	}
	__atomic_thread_fence(__ATOMIC_ACQUIRE);
	if (__atomic_load_n(&feed->ae_written, __ATOMIC_RELAXED) != w) {
		return; // rewritten while read: the next set takes it
	}
	s->stock_ae_seen = w;
	if (pico4_station_camera_set_each(s->station, scene) == 0) {
		SLAM_DEBUG(s, "exposure %u/%u %u/%u %u/%u %u/%u (PICO OS's auto exposure)", scene[0].lines, scene[0].gain,
		           scene[1].lines, scene[1].gain, scene[2].lines, scene[2].gain, scene[3].lines, scene[3].gain);
		s->exposure_lines = scene[0].lines;
		s->exposure_gain = scene[0].gain;
	}
}

/*!
 * The time that groups the frames of one FSIN pulse: the start of frame less half the exposure.
 * The sensors centre their exposures on the pulse, so with exposures that differ per camera
 * (PICO OS's auto exposure) their starts of frame differ by half the difference: 308 against
 * 737 lines put one camera 4.1 ms after the other, outside the group window, and no set formed.
 */
static int64_t
pulse_key(int64_t sof_ns, const struct pico4_cam_meta *m)
{
	return sof_ns - (((int64_t)LINE_NS * m->exposure_lines) >> 1);
}

/*!
 * A controller-LED exposure for the controller tracker: the four cameras of one pulse, in
 * factory order, 10 -> 8 bits by dropping the two lowest bits as pico4-cam --record-leds stores
 * them (the tracker's blob thresholds are for these), at the mean of their exposure middles.
 */
static void
handle_led_frame(struct pico4_slam *s, const struct pico4_cam_frame *f, const struct pico4_cam_meta *meta)
{
	int k = s->factory_of_slot[f->slot];
	if (s->ctrack == NULL || k < 0 || f->width != 640 || f->height != 480) {
		return;
	}
	bool dropped;
	int64_t sof = frame_sof_ns(s, f);
	enum pico4_cam_assembly_result added = pico4_cam_assembly_add(&s->leds.assembly, k, pulse_key(sof, meta),
	                                                              GROUP_WINDOW_NS, PICO4_TCAM_COUNT, &dropped);
	if (dropped) {
		s->led_sets_dropped++;
	}
	if (added == PICO4_CAM_SET_LATE || added == PICO4_CAM_SET_DUPLICATE) {
		return;
	}
	uint8_t *out = s->leds.images[k];
	for (uint32_t y = 0; y < f->height; y++) {
		const uint16_t *in = (const uint16_t *)((const uint8_t *)f->data + (size_t)y * f->stride);
		for (uint32_t x = 0; x < f->width; x++) {
			out[(size_t)y * f->width + x] = (uint8_t)((in[x] & 0x3ff) >> 2);
		}
	}
	s->leds.taken[k] = exposure_middle(sof, meta);
	if (added == PICO4_CAM_SET_COMPLETE) {
		int64_t sum = 0;
		for (int i = 0; i < PICO4_TCAM_COUNT; i++) {
			sum += s->leds.taken[i] - s->leds.taken[0];
		}
		pico4_ctrack_push_frames(s->ctrack, s->leds.taken[0] + sum / PICO4_TCAM_COUNT,
		                         (const uint8_t *const *)s->leds.images, 640);
		s->led_sets++;
		pico4_cam_assembly_reset(&s->leds.assembly);
	}
}

/*!
 * No scene set completed for a second while scene frames arrive: what each camera's frames did
 * (camera thread). Logged once a second while it lasts.
 */
static void
scene_stall(struct pico4_slam *s)
{
	int64_t now = os_monotonic_get_ns();
	if (s->stall.last_complete_ns == 0) {
		s->stall.last_complete_ns = now;
	}
	if (now - s->stall.last_complete_ns < U_TIME_1S_IN_NS || now - s->stall.logged_ns < U_TIME_1S_IN_NS) {
		return;
	}
	s->stall.logged_ns = now;
	int64_t first = INT64_MAX;
	for (int i = 0; i < s->cam_count; i++) {
		first = s->stall.frames[i] != 0 && s->stall.sof[i] < first ? s->stall.sof[i] : first;
	}
	char line[512];
	int n = 0;
	for (int i = 0; i < s->cam_count && n < (int)sizeof(line); i++) {
		n += snprintf(line + n, sizeof(line) - (size_t)n,
		              "%scam %d: %llu frames, %llu late, %llu again, SOF +%.2f ms, %u lines gain %u", i ? "; " : "", i,
		              (unsigned long long)s->stall.frames[i], (unsigned long long)s->stall.late[i],
		              (unsigned long long)s->stall.duplicate[i],
		              s->stall.frames[i] != 0 ? (s->stall.sof[i] - first) / 1e6 : -1.0, s->stall.meta[i].exposure_lines,
		              s->stall.meta[i].gain);
	}
	SLAM_WARN(s, "no complete scene set for %.1f s (%llu started): %s", (now - s->stall.last_complete_ns) / 1e9,
	          (unsigned long long)s->stall.sets_started, line);
	bool silent = false, some = false;
	for (int i = 0; i < s->cam_count; i++) {
		silent = silent || s->stall.frames[i] == 0;
		some = some || s->stall.frames[i] != 0;
	}
	if (silent && some) {
		// One camera's stream stuck while the others run.
		s->restart_why = "a camera delivers no frames";
	} else if (now - s->stall.last_complete_ns >= (int64_t)3 * U_TIME_1S_IN_NS) {
		s->restart_why = "no complete scene set for 3 s";
	} else if (stock_ae_active(s)) {
		// The auto exposure runs on complete sets: whatever keeps them from forming, one exposure
		// for all cameras (the median of theirs) lets them form again, and it can come down.
		unsigned lines[PICO4_TCAM_COUNT], gains[PICO4_TCAM_COUNT];
		int n_cams = s->cam_count < PICO4_TCAM_COUNT ? s->cam_count : PICO4_TCAM_COUNT;
		for (int i = 0; i < n_cams; i++) {
			lines[i] = s->stall.meta[i].exposure_lines;
			gains[i] = s->stall.meta[i].gain;
		}
		for (int i = 1; i < n_cams; i++) {
			for (int j = i; j > 0 && lines[j] < lines[j - 1]; j--) {
				unsigned t = lines[j];
				lines[j] = lines[j - 1];
				lines[j - 1] = t;
			}
			for (int j = i; j > 0 && gains[j] < gains[j - 1]; j--) {
				unsigned t = gains[j];
				gains[j] = gains[j - 1];
				gains[j - 1] = t;
			}
		}
		if (n_cams > 0 && pico4_station_camera_set_exposure(s->station, lines[n_cams / 2], gains[n_cams / 2]) == 0) {
			SLAM_WARN(s, "one exposure for all cameras: %u lines gain %u", lines[n_cams / 2], gains[n_cams / 2]);
		}
	}
	memset(s->stall.frames, 0, sizeof(s->stall.frames));
	memset(s->stall.late, 0, sizeof(s->stall.late));
	memset(s->stall.duplicate, 0, sizeof(s->stall.duplicate));
	s->stall.sets_started = 0;
}

/*!
 * An RGB frame for the room view: handed to the passthrough, which converts it on its own thread
 * and lets go of it (rgb_service); dropped while it still converts the one before.
 */
static void
handle_rgb_frame(struct pico4_slam *s, const struct pico4_cam_frame *f)
{
	s->rgb_frames++;
	// The frame before may be converted already: back to the camera first, so this one is taken.
	if (s->rgb_holding && !pico4_passthrough_rgb_busy(s->passthrough)) {
		pico4_cam_release(s->cam, &s->rgb_held);
		s->rgb_holding = false;
	}
	uint32_t exposure_us = (uint32_t)((uint64_t)s->rgb_lines * PICO4_CAM_RGB_LINE_NS / 1000);
	if (!s->rgb_holding && f->sof_monotonic_ns != 0 &&
	    pico4_passthrough_push_rgb(s->passthrough, f->data, f->stride / 2, frame_sof_ns(s, f), exposure_us)) {
		s->rgb_held = *f;
		s->rgb_holding = true;
		s->rgb_taken++;
		return;
	}
	pico4_cam_release(s->cam, f);
}

//! Camera thread: the RGB frame back once converted, and the exposure the newest one asks for.
static void
rgb_service(struct pico4_slam *s)
{
	if (!s->rgb) {
		return;
	}
	if (s->rgb_holding && !pico4_passthrough_rgb_busy(s->passthrough)) {
		pico4_cam_release(s->cam, &s->rgb_held);
		s->rgb_holding = false;
	}
	// The sensor streams only while the room view shows it, and 5 s on after that.
	int64_t now = os_monotonic_get_ns();
	if (pico4_passthrough_streaming(s->passthrough)) {
		s->rgb_wanted_ns = now;
	}
	bool want = now - s->rgb_wanted_ns < (int64_t)5 * U_TIME_1S_IN_NS;
	if (s->cam != NULL && want == s->rgb_paused && !s->rgb_pause_refused) {
		int r = pico4_cam_set_rgb_streaming(s->cam, want);
		if (r == 0) {
			s->rgb_paused = !want;
			SLAM_INFO(s, "RGB camera %s", want ? "streams (the room view shows it)" : "paused (nobody shows it)");
		} else {
			s->rgb_pause_refused = true;
			SLAM_WARN(s, "RGB camera: %s refused (%d), it stays as it is", want ? "resume" : "pause", r);
		}
	}
	unsigned lines, gain;
	if (s->cam != NULL &&
	    pico4_passthrough_rgb_exposure(s->passthrough, s->rgb_lines, s->rgb_gain, &lines, &gain) &&
	    pico4_cam_set_rgb_exposure(s->cam, lines, gain) == 0) {
		s->rgb_lines = lines;
		s->rgb_gain = gain;
		s->rgb_exposures++;
	}
}

//! Before the capture closes: the passthrough must be done with the RGB frame it reads.
static void
rgb_let_go(struct pico4_slam *s)
{
	for (int i = 0; i < 200 && pico4_passthrough_rgb_busy(s->passthrough); i++) {
		os_nanosleep(U_TIME_1MS_IN_NS);
	}
	s->rgb_holding = false; // the close frees its buffer
}

static void
handle_frame(struct pico4_slam *s, const struct pico4_cam_frame *f)
{
	if (f->slot == PICO4_CAM_RGB_SLOT) {
		handle_rgb_frame(s, f);
		return;
	}
	if (f->sof_monotonic_ns == 0) {
		return;
	}
	struct pico4_cam_meta meta;
	if (!pico4_cam_meta_decode(f->meta, f->stride, &meta)) {
		s->frames_bad++;
		return;
	}
	if (meta.phase != PICO4_CAM_PHASE_SCENE) {
		if (meta.phase == PICO4_CAM_PHASE_LEDS) {
			s->frames_leds++;
			handle_led_frame(s, f, &meta);
		} else {
			s->frames_other++;
		}
		return; // a controller-LED exposure: fed to Basalt, it runs away
	}
	int index = s->index_of_slot[f->slot];
	if (index < 0) {
		return;
	}
	int64_t ts = frame_sof_ns(s, f);
	s->stall.frames[index]++;
	s->stall.sof[index] = ts;
	s->stall.meta[index] = meta;
	if (s->group.assembly.have == 0) {
		s->stall.sets_started++;
	}
	bool dropped;
	enum pico4_cam_assembly_result added = pico4_cam_assembly_add(&s->group.assembly, index, pulse_key(ts, &meta),
	                                                              GROUP_WINDOW_NS, s->cam_count, &dropped);
	if (dropped) {
		s->sets_dropped++; // a camera missed the pulse of the set before
		for (int i = 0; i < PICO4_TCAM_COUNT; i++) {
			xrt_frame_reference(&s->group.frames[i], NULL); // that set's frames
		}
	}
	if (added == PICO4_CAM_SET_LATE || added == PICO4_CAM_SET_DUPLICATE) {
		(added == PICO4_CAM_SET_LATE ? s->stall.late : s->stall.duplicate)[index]++;
		scene_stall(s);
		return;
	}
	if (s->stock) {
		stock_copy_frame(s, f);
	} else {
		s->group.frames[index] = convert(s, f, s->rotated[index]);
	}
	s->group.taken[index] = exposure_middle(ts, &meta);
	s->group.sof[index] = ts;
	s->group.signal[index] = sample_mean(f) - BLACK_LEVEL;
	s->group.frame_id[index] = f->frame_id;
	s->group.meta[index] = meta;
	if (index == 0) {
		s->group.applied = meta;
	}
	if (!s->stock && s->group.frames[index] == NULL) {
		drop_group(s);
		return;
	}
	if (added != PICO4_CAM_SET_COMPLETE) {
		scene_stall(s);
	}
	if (added == PICO4_CAM_SET_COMPLETE) {
		s->stall.last_complete_ns = os_monotonic_get_ns();
		// Four frames of one exposure pulse, or none: a mixed set would give Basalt two scenes.
		bool stock_ae = s->stock && stock_ae_active(s);
		if (stock_ae != s->stock_ae_logged) {
			SLAM_INFO(s, "auto exposure: %s", stock_ae ? "PICO OS's, through pico4-slamd" : "the driver's");
			s->stock_ae_logged = stock_ae;
		}
		s->identity.per_camera_exposure = stock_ae;
		char why[160];
		bool consistent = pico4_cam_set_consistent(&s->identity, s->group.frame_id, s->group.meta, s->cam_count,
		                                           why, sizeof(why));
		if (!consistent) {
			s->sets_inconsistent++;
			SLAM_WARN(s, "frame set dropped: %s (%llu so far)", why, (unsigned long long)s->sets_inconsistent);
			drop_group(s);
			return;
		}
		if (why[0] != '\0') {
			SLAM_INFO(s, "%s", why);
		}
		/*
		 * The median of the cameras steers the exposure and the digital gain of the next
		 * sets: with the first camera alone, one facing fabric close by drove the exposure up
		 * until the other three saturated.
		 */
		float signal = pico4_cam_median(s->group.signal, s->cam_count);
		if (stock_ae) {
			stock_ae_apply(s);
		} else {
			auto_exposure(s, signal, &s->group.applied);
		}
		float mean8 = signal / 4.0f;
		float want = mean8 > 0.5f ? s->brightness / mean8 : 8.0f;
		want = want < 1.0f ? 1.0f : want > 8.0f ? 8.0f : want;
		s->gain += 0.05f * (want - s->gain);
		if (s->sequence % s->divisor == 0) {
			push_group(s);
		} else {
			s->sequence++;
			drop_group(s);
		}
	}
}

/*!
 * Set the calling thread's nice value; returns the previous one. Basalt's
 * threads run below the compositor and the IMU thread: without a config file
 * its TBB pool takes every core, and at equal priority the compositor missed
 * frames.
 */
static int
thread_nice(int value)
{
	id_t tid = (id_t)syscall(SYS_gettid);
	errno = 0;
	int before = getpriority(PRIO_PROCESS, tid);
	if (errno != 0) {
		before = 0;
	}
	setpriority(PRIO_PROCESS, tid, value); // lowering it back needs CAP_SYS_NICE (root here)
	return before;
}

/*!
 * Restrict the calling thread to the CPUs in PICO4_SLAM_CPUS (a bit mask,
 * default cores 0-5: the XR2's four small and two of its big cores), saving
 * the previous set. TBB sizes its pool from the CPU set it starts on, so
 * Basalt then leaves the two fastest cores to the compositor and the app.
 */
static bool
tracker_cpus(cpu_set_t *before)
{
	unsigned long mask = (unsigned long)debug_get_num_option_pico4_slam_cpus();
	cpu_set_t set;
	CPU_ZERO(&set);
	for (int cpu = 0; cpu < (int)(8 * sizeof(mask)) && cpu < CPU_SETSIZE; cpu++) {
		if (mask & (1ul << cpu)) {
			CPU_SET(cpu, &set);
		}
	}
	if (mask == 0 || sched_getaffinity(0, sizeof(*before), before) != 0) {
		return false;
	}
	CPU_AND(&set, &set, before);
	return CPU_COUNT(&set) > 0 && sched_setaffinity(0, sizeof(set), &set) == 0;
}

//! Feeds frame sets to the tracker; may block on it.
static void *
push_thread(void *ptr)
{
	struct pico4_slam *s = ptr;
	struct xrt_frame *frames[PICO4_TCAM_COUNT] = {0};

	thread_nice(TRACKER_NICE);
	os_thread_helper_name(&s->push_oth, "PICO 4 SLAM frames");
	os_thread_helper_lock(&s->push_oth);
	while (os_thread_helper_is_running_locked(&s->push_oth)) {
		if (!s->mail_full) {
			os_thread_helper_wait_locked(&s->push_oth);
			continue;
		}
		for (int i = 0; i < s->cam_count; i++) {
			frames[i] = s->mail[i];
			s->mail[i] = NULL;
		}
		s->mail_full = false;
		os_thread_helper_unlock(&s->push_oth);

		for (int i = 0; i < s->cam_count; i++) {
			xrt_sink_push_frame(s->sinks->cams[i], frames[i]);
			xrt_frame_reference(&frames[i], NULL);
		}
		os_thread_helper_lock(&s->push_oth);
	}
	os_thread_helper_unlock(&s->push_oth);
	return NULL;
}

//! Feeds IMU samples to the tracker, independently of the frames.
static void *
imu_push_thread(void *ptr)
{
	struct pico4_slam *s = ptr;
	struct xrt_imu_sample imu[IMU_RING];

	thread_nice(TRACKER_NICE);
	os_thread_helper_name(&s->imu_oth, "PICO 4 SLAM IMU");
	os_thread_helper_lock(&s->imu_oth);
	while (os_thread_helper_is_running_locked(&s->imu_oth)) {
		if (s->imu_len == 0) {
			os_thread_helper_wait_locked(&s->imu_oth);
			continue;
		}
		unsigned n = s->imu_len;
		for (unsigned i = 0; i < n; i++) {
			imu[i] = s->imu_ring[(s->imu_head + i) % IMU_RING];
		}
		s->imu_head = (s->imu_head + n) % IMU_RING;
		s->imu_len = 0;
		os_thread_helper_unlock(&s->imu_oth);

		for (unsigned i = 0; i < n; i++) {
			xrt_sink_push_imu(s->sinks->imu, &imu[i]);
		}
		os_thread_helper_lock(&s->imu_oth);
	}
	os_thread_helper_unlock(&s->imu_oth);
	return NULL;
}

/*!
 * The capture again from the start, as pico4_slam_create starts it: the station's camera sync off,
 * the capture closed and opened, the sync on, the exposure set (camera thread). A stream that
 * stays stuck recovers only this way.
 */
static bool
restart_capture(struct pico4_slam *s, const char *why)
{
	s->capture_restarts++;
	SLAM_WARN(s, "Restarting the camera capture (%llu so far): %s", (unsigned long long)s->capture_restarts, why);
	pico4_station_camera_stop(s->station);
	rgb_let_go(s);
	// The RGB camera starts again where its auto exposure was.
	s->cam_config.rgb_exposure_lines = s->rgb ? s->rgb_lines : 0;
	s->cam_config.rgb_gain = s->rgb ? s->rgb_gain : 0;
	s->rgb_paused = false; // a new capture streams from its start
	pico4_cam_close(s->cam);
	s->cam = NULL;
	drop_group(s);
	pico4_cam_assembly_reset(&s->leds.assembly);
	int r;
	if ((r = pico4_cam_open(&s->cam, &s->cam_config)) < 0 || (r = pico4_cam_start(s->cam)) < 0) {
		SLAM_ERROR(s, "The tracking cameras did not start again: %s", strerror(-r));
		pico4_cam_close(s->cam);
		s->cam = NULL;
		return false;
	}
	// The first restart of a session: the station refuses (EIO) the start after the stop, and
	// takes it after a second stop (measured, every time); later restarts take it at once.
	for (int tries = 0; (r = pico4_station_camera_start(s->station)) < 0 && tries < 4; tries++) {
		pico4_station_camera_stop(s->station);
		os_nanosleep(200 * U_TIME_1MS_IN_NS);
	}
	if (r < 0) {
		SLAM_ERROR(s, "The controller radio refused camera sync again: %s", strerror(-r));
		return false;
	}
	pico4_station_camera_set_exposure(s->station, s->exposure_lines, s->exposure_gain);
	s->stall.last_complete_ns = os_monotonic_get_ns();
	return true;
}

static void *
camera_thread(void *ptr)
{
	struct pico4_slam *s = ptr;
	struct pico4_cam_frame f;
	uint64_t timeouts = 0;
	int64_t restarted_ns = 0, slept_ns = pico4_slept_ns();
	// The RGB camera streams on its own: the tracking cameras' silence is watched apart from it.
	int64_t tracking_frame_ns = os_monotonic_get_ns();

	os_thread_helper_name(&s->oth, "PICO 4 cameras");
	os_thread_helper_lock(&s->oth);
	while (os_thread_helper_is_running_locked(&s->oth)) {
		os_thread_helper_unlock(&s->oth);
		// The capture streamed into a suspend: a new capture after it, as after an ISP error.
		int64_t slept = pico4_slept_ns();
		if (slept - slept_ns > PICO4_SLEEP_DETECT_NS) {
			SLAM_INFO(s, "The system slept %.1f s: the capture starts again", (double)(slept - slept_ns) / 1e9);
			s->restart_why = "the system slept";
		}
		slept_ns = slept;
		const char *why = s->restart_why;
		rgb_service(s);
		int r = s->cam != NULL ? pico4_cam_next(s->cam, &f, 200) : -ENODEV;
		if (r == 0 && f.slot == PICO4_CAM_RGB_SLOT) {
			handle_frame(s, &f);
			if (os_monotonic_get_ns() - tracking_frame_ns > U_TIME_1S_IN_NS) {
				why = "no tracking camera frames for 1 s";
			}
		} else if (r == 0) {
			timeouts = 0;
			tracking_frame_ns = os_monotonic_get_ns();
			handle_frame(s, &f);
			// With the RGB camera the frames are held (its own until converted): the rest at once.
			if (s->cam_config.hold_frames && f.slot != PICO4_CAM_RGB_SLOT) {
				pico4_cam_release(s->cam, &f);
			}
		} else if (r == -ETIMEDOUT) {
			if (timeouts++ % 25 == 0) {
				SLAM_ERROR(s, "No camera frames for 200 ms");
			}
			why = timeouts >= 5 ? "no camera frames for 1 s" : why;
		} else if (r != -EINTR) {
			why = r == -ENODEV ? "the capture did not start" : "every camera failed";
			os_nanosleep(U_TIME_1MS_IN_NS * 100);
		}
		// At most once in 5 s: the restart itself takes a moment, and a station that refuses
		// should not be hammered.
		int64_t now = os_monotonic_get_ns();
		int64_t test_s = debug_get_num_option_pico4_capture_restart_test_s();
		if (test_s > 0 && now - restarted_ns >= test_s * U_TIME_1S_IN_NS && restarted_ns != 0) {
			why = "PICO4_CAPTURE_RESTART_TEST_S";
		} else if (restarted_ns == 0) {
			restarted_ns = now; // the test's first restart one period after the start
		}
		if (why != NULL && now - restarted_ns >= (int64_t)5 * U_TIME_1S_IN_NS) {
			s->restart_why = NULL;
			restarted_ns = now;
			timeouts = 0;
			restart_capture(s, why);
			tracking_frame_ns = os_monotonic_get_ns();
		}
		os_thread_helper_lock(&s->oth);
	}
	os_thread_helper_unlock(&s->oth);
	return NULL;
}

static void
cam_log(void *user, enum pico4_cam_log_level level, const char *msg)
{
	struct pico4_slam *s = user;
	if (level == PICO4_CAM_LOG_ERROR) {
		SLAM_ERROR(s, "%s", msg);
	} else {
		SLAM_DEBUG(s, "%s", msg);
	}
}

struct pico4_slam *
pico4_slam_create(const char *calibration_dir,
                  struct pico4_station *station,
                  double imu_hz,
                  struct pico4_ctrack *ctrack,
                  enum u_logging_level log_level)
{
	if (!debug_get_bool_option_pico4_slam() || station == NULL) {
		return NULL;
	}
	struct pico4_slam *s = U_TYPED_CALLOC(struct pico4_slam);
	int cams[PICO4_TCAM_COUNT], slots[PICO4_TCAM_COUNT];
	s->log_level = log_level;
	s->ctrack = ctrack;
	s->imu_hz = imu_hz;
	s->gain = 2.0f;
	s->exposure_lines = 300;
	s->exposure_gain = 16;
	s->max_exposure = (unsigned)debug_get_num_option_pico4_slam_max_exposure();
	s->brightness = (float)debug_get_num_option_pico4_slam_brightness();
	long divisor = debug_get_num_option_pico4_slam_divisor();
	s->divisor = divisor < 1 ? 1 : (unsigned)divisor;
	long imu_divisor = debug_get_num_option_pico4_slam_imu_divisor();
	s->imu_divisor = imu_divisor < 1 ? 1 : (unsigned)imu_divisor;
	s->cam_count = parse_index_list(debug_get_option_pico4_slam_cams(), cams, PICO4_TCAM_COUNT, PICO4_TCAM_COUNT);
	if (s->cam_count < 2 ||
	    parse_index_list(debug_get_option_pico4_tcam_slots(), slots, PICO4_TCAM_COUNT, PICO4_CAM_NUM) !=
	        PICO4_TCAM_COUNT) {
		SLAM_ERROR(s, "PICO4_SLAM_CAMS needs two or more cameras 0-3, PICO4_TCAM_SLOTS four slots");
		free(s);
		return NULL;
	}
	for (int i = 0; i < PICO4_CAM_NUM; i++) {
		s->index_of_slot[i] = -1;
		s->factory_of_slot[i] = -1;
	}
	for (int i = 0; i < s->cam_count; i++) {
		s->index_of_slot[slots[cams[i]]] = i;
	}
	for (int k = 0; k < PICO4_TCAM_COUNT; k++) {
		s->factory_of_slot[slots[k]] = k;
	}
	if (!load_calibration(s, calibration_dir, cams)) {
		free(s);
		return NULL;
	}
	s->qtimer_offset_ns = s->qtimer_first_offset_ns = qtimer_offset();
	s->slept_ns = pico4_slept_ns();
	s->qtimer_measured_ns = os_monotonic_get_ns();
	s->stock = strcmp(debug_get_option_pico4_slam_backend(), "basalt") != 0;
	if (s->stock && s->cam_count != PICO4_TCAM_COUNT) {
		SLAM_ERROR(s, "PICO OS's tracker needs all four cameras (PICO4_SLAM_CAMS=0,1,2,3)");
		free(s);
		return NULL;
	}
	// The first frame attaches to pico4-slamd (stock_copy_frame): a driver whose cameras fail
	// to start, because another one has them, must not restart that one's tracker.
	if (s->stock) {
		s->passthrough = create_passthrough(s, calibration_dir);
		goto cameras;
	}

	// pico4-basalt reads it when it creates the tracker (basalt-vio-config.patch).
	const char *vio = debug_get_option_pico4_slam_vio_config();
	if (vio != NULL && *vio != '\0' && access(vio, R_OK) == 0) {
		setenv("PICO4_BASALT_VIO_CONFIG", vio, 1);
		SLAM_INFO(s, "Basalt VIO settings from %s", vio);
	} else {
		unsetenv("PICO4_BASALT_VIO_CONFIG");
	}
	struct t_slam_tracker_config config;
	t_slam_fill_default_config(&config);
	config.cam_count = s->cam_count;
	config.slam_calib = &s->calib;
	config.submit_from_start = true;
	/*
	 * Between Basalt's poses (25 Hz), predict the orientation from the gyro and the
	 * position from Basalt's velocity plus the mean acceleration since (above
	 * SLAM_ACCEL_DEADBAND, see monado-slam-prediction.patch): positions trailed a moving
	 * head by 31 ms instead of 65 ms without the acceleration. The default dead reckoning
	 * integrates the accelerometer fully: its positions strayed 50-270 mm from Basalt's.
	 * A stalled tracker reports untracked poses after 500 ms, and the HMD falls back to
	 * 3DoF. The environment's SLAM_PREDICTION_TYPE still overrides this.
	 */
	if (getenv("SLAM_PREDICTION_TYPE") == NULL) {
		config.prediction = SLAM_PRED_ACCEL_GYRO;
	}
	// Basalt's threads, its TBB workers included, inherit this thread's nice value and CPU set.
	int nice_before = thread_nice(TRACKER_NICE);
	cpu_set_t cpus_before;
	bool pinned = tracker_cpus(&cpus_before);
	bool started = t_slam_create(&s->xfctx, &config, &s->tracker, &s->sinks) == 0 && t_slam_start(s->tracker) == 0;
	if (pinned) {
		sched_setaffinity(0, sizeof(cpus_before), &cpus_before);
	}
	thread_nice(nice_before);
	if (!started) {
		SLAM_ERROR(s, "Could not start the SLAM tracker (is Basalt's libbasalt.so installed?)");
		goto fail;
	}

cameras:
	s->station = station;
	for (int k = 0; s->ctrack != NULL && k < PICO4_TCAM_COUNT; k++) {
		s->leds.images[k] = U_TYPED_ARRAY_CALLOC(uint8_t, 640 * 480);
	}
	int r;
	struct pico4_cam_config cc;
	pico4_cam_config_init(&cc);
	cc.verbose = log_level <= U_LOGGING_DEBUG ? PICO4_CAM_LOG_INFO : PICO4_CAM_LOG_ERROR;
	cc.log = cam_log;
	cc.log_user = s;
	// The RGB camera is in the same capture.
	bool rgb = s->passthrough != NULL && debug_get_bool_option_pico4_roomview_rgb();
	if (rgb) {
		cc.slots |= 1u << PICO4_CAM_RGB_SLOT;
		cc.hold_frames = true;
		cc.rgb_exposure_lines = 2260; // 1/100 s at 4x, until the auto exposure took over
		cc.rgb_gain = 64;
	}
	s->cam_config = cc;
	r = pico4_cam_open(&s->cam, &cc);
	if (r == 0) {
		r = pico4_cam_start(s->cam);
	}
	if (r < 0 && rgb) {
		SLAM_WARN(s, "The cameras did not start with the RGB camera (%s): the room view stays grey", strerror(-r));
		pico4_cam_close(s->cam);
		s->cam = NULL;
		rgb = false;
		cc.slots &= ~(1u << PICO4_CAM_RGB_SLOT);
		cc.hold_frames = false;
		s->cam_config = cc;
		r = pico4_cam_open(&s->cam, &cc);
		if (r == 0) {
			r = pico4_cam_start(s->cam);
		}
	}
	if (r < 0) {
		SLAM_ERROR(s, "Tracking cameras failed to start: %s", strerror(-r));
		goto fail;
	}
	if (rgb) {
		double T_body_head[16], down_body[3];
		body_head(T_body_head, down_body);
		if (pico4_passthrough_use_rgb(s->passthrough, calibration_dir, T_body_head)) {
			struct pico4_passthrough_info info;
			pico4_passthrough_rgb_info(s->passthrough, &info);
			const double *m = info.camera_to_head[0];
			SLAM_INFO(s, "Room view from the RGB camera: %ux%u, at %.3f %.3f %.3f m, looking %.2f %.2f %.2f",
			          info.width, info.height, m[3], m[7], m[11], -m[2], -m[6], -m[10]);
		} else {
			SLAM_WARN(s, "No calibration of the RGB camera (cam4) in %s: the room view stays grey", calibration_dir);
		}
	}
	s->rgb = rgb;
	s->rgb_lines = cc.rgb_exposure_lines;
	s->rgb_gain = cc.rgb_gain;
	// A timeout here would leave the head in 3DoF for the whole session. After one the radio refuses
	// (EIO) every further start, also after a stop (provoked): only the reset that readies it for
	// camera sync at the driver's start helps, and the controllers are back within a second.
	for (int tries = 0; (r = pico4_station_camera_start(s->station)) < 0 && tries < 3; tries++) {
		SLAM_WARN(s, "The controller radio refused camera sync (%s): resetting it", strerror(-r));
		int reset = pico4_station_reset(s->station);
		if (reset < 0) {
			SLAM_WARN(s, "The controller radio's reset: %s", strerror(-reset));
		}
	}
	if (r < 0) {
		SLAM_ERROR(s, "The controller radio refused camera sync: %s", strerror(-r));
		goto fail;
	}
	pico4_station_camera_set_exposure(s->station, s->exposure_lines, s->exposure_gain);
	// The stock tracker gets frames and IMU samples straight from the capture and IMU threads.
	if ((!s->stock && (os_thread_helper_init(&s->imu_oth) != 0 ||
	                   os_thread_helper_start(&s->imu_oth, imu_push_thread, s) != 0 ||
	                   os_thread_helper_init(&s->push_oth) != 0 ||
	                   os_thread_helper_start(&s->push_oth, push_thread, s) != 0)) ||
	    os_thread_helper_init(&s->oth) != 0 || os_thread_helper_start(&s->oth, camera_thread, s) != 0) {
		SLAM_ERROR(s, "Could not start the camera threads");
		goto fail;
	}
	SLAM_INFO(s, "SLAM tracking (%s) with %d cameras at %.1f Hz", s->stock ? "PICO OS's tracker" : "Basalt",
	          s->cam_count, SCENE_HZ / s->divisor);
	return s;

fail:
	pico4_slam_destroy(&s);
	return NULL;
}

//! Measures the rate the IMU samples arrive at over 10 s, and reports it once or when it strays.
static void
measure_imu_rate(struct pico4_slam *s, int64_t timestamp_ns)
{
	if (s->rate_start_ns == 0 || timestamp_ns < s->rate_start_ns) {
		s->rate_start_ns = timestamp_ns;
		s->rate_samples = 0;
		return;
	}
	s->rate_samples++;
	int64_t span = timestamp_ns - s->rate_start_ns;
	if (span < 10 * (int64_t)1000000000) {
		return;
	}
	double hz = (double)s->rate_samples * 1e9 / (double)span;
	bool strays = fabs(hz - s->imu_hz) > 0.05 * s->imu_hz;
	if (!s->rate_reported || strays) {
		U_LOG_IFL(strays ? U_LOGGING_WARN : U_LOGGING_INFO, s->log_level,
		          "IMU at %.1f Hz measured, %.1f Hz declared; the tracker gets every %u. sample", hz,
		          s->imu_hz, s->imu_divisor);
		s->rate_reported = true;
	}
	s->rate_start_ns = timestamp_ns;
	s->rate_samples = 0;
}

void
pico4_slam_push_imu(
    struct pico4_slam *s, int64_t timestamp_ns, const float accel[3], const float gyro[3], float temperature_c)
{
	measure_imu_rate(s, timestamp_ns);
	if (s->stock) {
		// Every sample, as PICO OS feeds its tracker (only this thread writes the ring).
		struct pico4_slam_feed *feed = __atomic_load_n(&s->feed, __ATOMIC_ACQUIRE);
		if (feed != NULL) {
			uint64_t w = __atomic_load_n(&feed->imu_written, __ATOMIC_RELAXED);
			struct pico4_slam_feed_imu *e = &feed->imu[w % PICO4_SLAM_FEED_IMU];
			e->ts_ns = timestamp_ns;
			memcpy(e->gyro, gyro, sizeof(e->gyro));
			memcpy(e->accel, accel, sizeof(e->accel));
			e->temperature = temperature_c;
			__atomic_store_n(&feed->imu_written, w + 1, __ATOMIC_RELEASE);
			s->stock_imu++;
		}
		__atomic_store_n(&s->last_imu_ns, timestamp_ns, __ATOMIC_RELAXED);
		return;
	}
	if (s->imu_count++ % s->imu_divisor != 0) {
		return;
	}
	struct xrt_imu_sample sample = {
	    .timestamp_ns = timestamp_ns,
	    .accel_m_s2 = {accel[0], accel[1], accel[2]},
	    .gyro_rad_secs = {gyro[0], gyro[1], gyro[2]},
	};
	os_thread_helper_lock(&s->imu_oth);
	if (s->imu_len == IMU_RING) {
		// The tracker is seconds behind; drop the oldest sample.
		s->imu_head = (s->imu_head + 1) % IMU_RING;
		s->imu_len--;
		s->imu_overflow++;
	}
	s->imu_ring[(s->imu_head + s->imu_len) % IMU_RING] = sample;
	s->imu_len++;
	os_thread_helper_signal_locked(&s->imu_oth);
	os_thread_helper_unlock(&s->imu_oth);
	__atomic_store_n(&s->last_imu_ns, timestamp_ns, __ATOMIC_RELAXED);
}

//! A tracked stock pose not older than this counts; older, the head is lost (3DoF in the HMD).
#define STOCK_STALE_NS (200 * U_TIME_1MS_IN_NS)

//! The stock tracker's newest pose, if it tracks; false while lost, stale or not attached.
static bool
stock_newest(struct pico4_slam *s, struct pico4_slam_feed_pose *out)
{
	struct pico4_slam_feed *feed = __atomic_load_n(&s->feed, __ATOMIC_ACQUIRE);
	if (feed == NULL) {
		return false;
	}
	for (int attempt = 0; attempt < 3; attempt++) {
		uint64_t n = __atomic_load_n(&feed->poses_written, __ATOMIC_ACQUIRE);
		if (n == 0) {
			return false;
		}
		*out = feed->poses[(n - 1) % PICO4_SLAM_FEED_POSES];
		// The entry stays valid until the writer laps the ring.
		if (__atomic_load_n(&feed->poses_written, __ATOMIC_ACQUIRE) - n < PICO4_SLAM_FEED_POSES - 1) {
			return out->confidence > 0 && os_monotonic_get_ns() - out->ts_ns < STOCK_STALE_NS;
		}
	}
	return false;
}

/*!
 * A stock pose in the room's world (docs/xr.md#room-setup): once pico4-slamd has found that the
 * tracker relocalised in a saved room, the session's world is turned about its up axis (x) and
 * shifted onto the room's, so SteamVR's tracking space, and the play area kept in it, stay where
 * they were in every session in that room. Body rates stay as they are.
 */
static void
stock_in_room(struct pico4_slam *s, struct pico4_slam_feed_pose *p)
{
	struct pico4_slam_feed *feed = s->feed;
	uint32_t state = __atomic_load_n(&feed->room_state, __ATOMIC_ACQUIRE);
	if (feed->room_generation != s->stock_generation) {
		return;
	}
	if (state != 0 && __atomic_exchange_n(&s->stock_room_logged, state, __ATOMIC_RELAXED) != state) {
		SLAM_INFO(s, "Room: %s (map %.40s)", state == 1 ? "the saved room, its world from now on" : "a room of its own",
		          feed->room_uuid);
	}
	if (state != 1) {
		return;
	}
	double c = cos(feed->room_yaw), sn = sin(feed->room_yaw);
	double y = p->position[1], z = p->position[2];
	p->position[0] += feed->room_t[0];
	p->position[1] = c * y - sn * z + feed->room_t[1];
	p->position[2] = sn * y + c * z + feed->room_t[2];
	double vy = p->velocity[1], vz = p->velocity[2];
	p->velocity[1] = c * vy - sn * vz;
	p->velocity[2] = sn * vy + c * vz;
	// The turn as a quaternion about x, on the world side: q = r * q.
	double rw = cos(feed->room_yaw / 2), rx = sin(feed->room_yaw / 2);
	double w = p->orientation[0], x = p->orientation[1], qy = p->orientation[2], qz = p->orientation[3];
	p->orientation[0] = rw * w - rx * x;
	p->orientation[1] = rw * x + rx * w;
	p->orientation[2] = rw * qy - rx * qz;
	p->orientation[3] = rw * qz + rx * qy;
}

bool
pico4_slam_room(struct pico4_slam *s, char out_map[40])
{
	struct pico4_slam_feed *feed = s != NULL ? __atomic_load_n(&s->feed, __ATOMIC_ACQUIRE) : NULL;
	if (feed == NULL || __atomic_load_n(&feed->room_state, __ATOMIC_ACQUIRE) == 0 ||
	    feed->room_generation != s->stock_generation || feed->room_uuid[0] == '\0') {
		return false;
	}
	memcpy(out_map, feed->room_uuid, 40);
	out_map[39] = '\0';
	return true;
}

void
pico4_slam_detect_floor(struct pico4_slam *s)
{
	struct pico4_slam_feed *feed = s != NULL ? __atomic_load_n(&s->feed, __ATOMIC_ACQUIRE) : NULL;
	if (feed == NULL || !s->stock) {
		return;
	}
	__atomic_fetch_add(&feed->floor_requests, 1, __ATOMIC_ACQ_REL);
	SLAM_INFO(s, "Floor: the tracker's ground detection asked to run again");
}

bool
pico4_slam_floor(struct pico4_slam *s, float *out_y)
{
	struct pico4_slam_feed_pose p;
	if (s == NULL || !s->stock || !stock_newest(s, &p) || p.floor_status != 1) {
		return false;
	}
	// The floor lies floor_height below the world's origin along its up axis (x), which is
	// OpenXR's y; in a saved room the room's shift moves it (stock_in_room).
	struct pico4_slam_feed *feed = s->feed;
	double y = -p.floor_height;
	if (__atomic_load_n(&feed->room_state, __ATOMIC_ACQUIRE) == 1 && feed->room_generation == s->stock_generation) {
		y += feed->room_t[0];
	}
	*out_y = (float)y;
	return true;
}

static_assert(PICO4_DEPTH_MESH_VERTICES == PICO4_SLAM_FEED_MESH_VERTICES, "the feed's mesh size");
static_assert(PICO4_DEPTH_MESH_INDICES == PICO4_SLAM_FEED_MESH_INDICES, "the feed's mesh size");

/*!
 * libphotoid writes its vertices in the tracker's world turned by a fixed quarter turn about z,
 * (x, y, z) = (-y, x, z) of the world (measured: gravity along its y): with the tracker's axes
 * (x up, y left, z back) these are the HMD axes of stock_to_openxr. So the vertices are in the
 * OpenXR world as they are; in a saved room they get the room's turn and shift as the poses.
 */
bool
pico4_slam_depth_mesh(
    struct pico4_slam *s, uint64_t after, float *vertices, uint32_t *indices, struct pico4_depth_mesh_info *out)
{
	struct pico4_slam_feed *feed = s != NULL && s->stock ? __atomic_load_n(&s->feed, __ATOMIC_ACQUIRE) : NULL;
	// Only while the cameras are shown (the room view asks for frames): asking keeps pico4-slamd
	// feeding its depth pipeline, which otherwise idles.
	const uint32_t asking = feed != NULL && pico4_passthrough_streaming(s->passthrough);
	if (s != NULL && __atomic_exchange_n(&s->mesh_asking, asking, __ATOMIC_RELAXED) != asking) {
		SLAM_INFO(s, "Depth mesh: %s", asking ? "the cameras are shown, asking pico4-slamd for meshes" : "not asked for");
	}
	if (!asking) {
		return false;
	}
	__atomic_store_n(&feed->mesh_wanted_ns, (uint64_t)os_monotonic_get_ns(), __ATOMIC_RELEASE);
	for (int attempt = 0; attempt < 3; attempt++) {
		const uint64_t n = __atomic_load_n(&feed->meshes_written, __ATOMIC_ACQUIRE);
		if (n <= after) {
			return false;
		}
		const struct pico4_slam_feed_mesh *m = &feed->meshes[(n - 1) % PICO4_SLAM_FEED_MESHES];
		const uint32_t nv = m->vertex_count, ni = m->index_count;
		if (nv > PICO4_DEPTH_MESH_VERTICES || ni > PICO4_DEPTH_MESH_INDICES) {
			return false;
		}
		out->id = n;
		out->timestamp_ns = m->ts_ns;
		out->vertex_count = nv;
		out->index_count = ni;
		memcpy(vertices, m->vertices, (size_t)nv * 3 * sizeof(float));
		memcpy(indices, m->indices, (size_t)ni * sizeof(uint32_t));
		// The writer takes this slot again only after it has published the next mesh.
		if (__atomic_load_n(&feed->meshes_written, __ATOMIC_ACQUIRE) != n) {
			continue;
		}
		if (__atomic_load_n(&feed->room_state, __ATOMIC_ACQUIRE) == 1 && feed->room_generation == s->stock_generation) {
			// The room's turn about the tracker's up axis (x), then its shift, in OpenXR axes:
			// tracker (x, y, z) = (y, -x, z) of the vertex.
			const double c = cos(feed->room_yaw), sn = sin(feed->room_yaw);
			for (uint32_t i = 0; i < nv; i++) {
				float *v = &vertices[3 * i];
				const double ty = -v[0], tz = v[2];
				const double up = v[1] + feed->room_t[0];
				const double left = c * ty - sn * tz + feed->room_t[1];
				const double back = sn * ty + c * tz + feed->room_t[2];
				v[0] = (float)-left;
				v[1] = (float)up;
				v[2] = (float)back;
			}
		}
		return true;
	}
	return false;
}

/*!
 * The stock tracker's pose at @p at_ns, which lies before its newest pose (an LED exposure for the
 * controller tracker): interpolated between the two ring entries around it. False if the ring no
 * longer holds that time, or either entry is untracked.
 */
static bool
stock_pose_at(struct pico4_slam *s, int64_t at_ns, struct pico4_slam_feed_pose *out)
{
	struct pico4_slam_feed *feed = __atomic_load_n(&s->feed, __ATOMIC_ACQUIRE);
	if (feed == NULL) {
		return false;
	}
	uint64_t n = __atomic_load_n(&feed->poses_written, __ATOMIC_ACQUIRE);
	// Searched from the newest back; the oldest quarter of the ring is left to the writer.
	uint64_t oldest = n > PICO4_SLAM_FEED_POSES * 3 / 4 ? n - PICO4_SLAM_FEED_POSES * 3 / 4 : 0;
	for (uint64_t i = n; i > oldest + 1; i--) {
		const struct pico4_slam_feed_pose *b = &feed->poses[(i - 1) % PICO4_SLAM_FEED_POSES];
		if (b->ts_ns > at_ns) {
			continue;
		}
		struct pico4_slam_feed_pose lo = *b, hi = feed->poses[i % PICO4_SLAM_FEED_POSES];
		if (i == n || __atomic_load_n(&feed->poses_written, __ATOMIC_ACQUIRE) - oldest >= PICO4_SLAM_FEED_POSES ||
		    lo.confidence == 0 || hi.confidence == 0 || hi.ts_ns <= lo.ts_ns) {
			return false;
		}
		double t = (double)(at_ns - lo.ts_ns) / (double)(hi.ts_ns - lo.ts_ns);
		// Neighbouring IMU samples (1 ms apart): a normalised linear blend is exact enough.
		double dot = 0;
		for (int k = 0; k < 4; k++) {
			dot += lo.orientation[k] * hi.orientation[k];
		}
		double sign = dot < 0 ? -1 : 1, norm = 0;
		*out = lo;
		out->ts_ns = at_ns;
		for (int k = 0; k < 3; k++) {
			out->position[k] = lo.position[k] + t * (hi.position[k] - lo.position[k]);
			out->velocity[k] = lo.velocity[k] + t * (hi.velocity[k] - lo.velocity[k]);
			out->angular[k] = lo.angular[k] + t * (hi.angular[k] - lo.angular[k]);
		}
		for (int k = 0; k < 4; k++) {
			out->orientation[k] = lo.orientation[k] + t * (sign * hi.orientation[k] - lo.orientation[k]);
			norm += out->orientation[k] * out->orientation[k];
		}
		norm = sqrt(norm);
		for (int k = 0; k < 4; k++) {
			out->orientation[k] /= norm;
		}
		return true;
	}
	return false;
}

/*!
 * The stock tracker's pose in OpenXR. Its world and body frames are the factory body frame's
 * axes (x up, y left, z back), as Basalt's body frame: PICO OS's client maps both with the HMD
 * axes (x right = -y, y up = x, z back = z), the inverse of Q_B_H on each side. @p dt_ns
 * predicts the pose that far along its velocities.
 */
static void
stock_to_openxr(const struct pico4_slam_feed_pose *p, int64_t dt_ns, struct xrt_space_relation *out,
                struct xrt_vec3 *out_imu_position)
{
	const struct xrt_quat q_h_b = {-Q_B_H.x, -Q_B_H.y, -Q_B_H.z, Q_B_H.w};
	struct xrt_quat q_s = {(float)p->orientation[1], (float)p->orientation[2], (float)p->orientation[3],
	                       (float)p->orientation[0]};
	math_quat_normalize(&q_s);
	struct xrt_vec3 pos_s = {(float)p->position[0], (float)p->position[1], (float)p->position[2]};
	struct xrt_vec3 vel_s = {(float)p->velocity[0], (float)p->velocity[1], (float)p->velocity[2]};
	struct xrt_vec3 rate_b = {(float)p->angular[0], (float)p->angular[1], (float)p->angular[2]};

	struct xrt_quat t, q;
	math_quat_rotate(&q_h_b, &q_s, &t);
	math_quat_rotate(&t, &Q_B_H, &q);
	struct xrt_vec3 lever, hmd_s, rate_s, hmd, vel, rate;
	math_quat_rotate_vec3(&q_s, &P_BODY_HMD, &lever);
	hmd_s = (struct xrt_vec3){pos_s.x + lever.x, pos_s.y + lever.y, pos_s.z + lever.z};
	math_quat_rotate_vec3(&q_h_b, &hmd_s, &hmd);
	math_quat_rotate_vec3(&q_h_b, &vel_s, &vel);
	math_quat_rotate_vec3(&q_s, &rate_b, &rate_s);
	math_quat_rotate_vec3(&q_h_b, &rate_s, &rate);
	if (out_imu_position != NULL) {
		math_quat_rotate_vec3(&q_h_b, &pos_s, out_imu_position);
	}

	if (dt_ns > 0) {
		float dt = (float)dt_ns / (float)U_TIME_1S_IN_NS;
		float speed = sqrtf(rate.x * rate.x + rate.y * rate.y + rate.z * rate.z);
		if (speed * dt > 1e-6f) {
			float half = 0.5f * speed * dt, k = sinf(half) / speed;
			struct xrt_quat dq = {rate.x * k, rate.y * k, rate.z * k, cosf(half)};
			struct xrt_quat before = q;
			math_quat_rotate(&dq, &before, &q); // world rates turn from the left
		}
		hmd.x += vel.x * dt;
		hmd.y += vel.y * dt;
		hmd.z += vel.z * dt;
	}
	math_quat_normalize(&q);
	out->pose.orientation = q;
	out->pose.position = hmd;
	out->linear_velocity = vel;
	out->angular_velocity = rate;
	out->relation_flags = (enum xrt_space_relation_flags)(
	    XRT_SPACE_RELATION_ORIENTATION_VALID_BIT | XRT_SPACE_RELATION_POSITION_VALID_BIT |
	    XRT_SPACE_RELATION_ORIENTATION_TRACKED_BIT | XRT_SPACE_RELATION_POSITION_TRACKED_BIT |
	    XRT_SPACE_RELATION_LINEAR_VELOCITY_VALID_BIT | XRT_SPACE_RELATION_ANGULAR_VELOCITY_VALID_BIT);
}

bool
pico4_slam_fuses_imu(const struct pico4_slam *s)
{
	return s != NULL && s->stock;
}

bool
pico4_slam_get_head_pose(struct pico4_slam *s, int64_t at_ns, struct xrt_space_relation *out)
{
	if (s->stock) {
		struct pico4_slam_feed_pose p;
		if (!stock_newest(s, &p)) {
			return false;
		}
		// A time already past (the controller tracker's LED exposures): the pose then.
		if (at_ns < p.ts_ns) {
			struct pico4_slam_feed_pose then;
			if (stock_pose_at(s, at_ns, &then)) {
				stock_in_room(s, &then);
				stock_to_openxr(&then, 0, out, NULL);
				return true;
			}
		}
		int64_t dt = at_ns - p.ts_ns;
		dt = dt < 0 ? 0 : dt > STOCK_PREDICT_MAX_NS ? STOCK_PREDICT_MAX_NS : dt;
		stock_in_room(s, &p);
		stock_to_openxr(&p, dt, out, NULL);
		return true;
	}
	struct xrt_space_relation rel = XRT_SPACE_RELATION_ZERO;
	xrt_tracked_slam_get_tracked_pose(s->tracker, at_ns, &rel);
	if (!(rel.relation_flags & XRT_SPACE_RELATION_POSITION_TRACKED_BIT)) {
		return false;
	}

	// Basalt: body (IMU) pose in a Z-up world. OpenXR: HMD pose in a Y-up world.
	struct xrt_quat q;
	struct xrt_vec3 offset, p;

	math_quat_rotate(&Q_XR_W, &rel.pose.orientation, &q);
	math_quat_rotate(&q, &Q_B_H, &out->pose.orientation);
	math_quat_normalize(&out->pose.orientation);

	math_quat_rotate_vec3(&rel.pose.orientation, &P_BODY_HMD, &offset);
	p = (struct xrt_vec3){rel.pose.position.x + offset.x, rel.pose.position.y + offset.y,
	                      rel.pose.position.z + offset.z};
	math_quat_rotate_vec3(&Q_XR_W, &p, &out->pose.position);

	out->relation_flags = (enum xrt_space_relation_flags)(
	    XRT_SPACE_RELATION_ORIENTATION_VALID_BIT | XRT_SPACE_RELATION_POSITION_VALID_BIT |
	    XRT_SPACE_RELATION_ORIENTATION_TRACKED_BIT | XRT_SPACE_RELATION_POSITION_TRACKED_BIT);
	return true;
}

bool
pico4_slam_get_latest(struct pico4_slam *s, struct pico4_fusion_slam *out)
{
	if (s->stock) {
		struct pico4_slam_feed_pose p;
		struct xrt_space_relation r = XRT_SPACE_RELATION_ZERO;
		if (!stock_newest(s, &p)) {
			return false;
		}
		stock_in_room(s, &p);
		stock_to_openxr(&p, 0, &r, &out->position);
		out->ts_ns = p.ts_ns;
		out->orientation = r.pose.orientation;
		out->velocity = r.linear_velocity;
		return true;
	}
	int64_t ts;
	struct xrt_space_relation rel = XRT_SPACE_RELATION_ZERO;
	if (!t_slam_get_latest_pose(s->tracker, &ts, &rel)) {
		return false;
	}
	struct xrt_quat q;
	out->ts_ns = ts;
	math_quat_rotate(&Q_XR_W, &rel.pose.orientation, &q);
	math_quat_rotate(&q, &Q_B_H, &out->orientation);
	math_quat_normalize(&out->orientation);
	math_quat_rotate_vec3(&Q_XR_W, &rel.pose.position, &out->position);
	math_quat_rotate_vec3(&Q_XR_W, &rel.linear_velocity, &out->velocity);
	return true;
}

void
pico4_slam_reset(struct pico4_slam *s)
{
	if (s->stock) {
		return; // PICO OS's tracker relocalises in its own map
	}
	t_slam_reset(s->tracker);
}

void
pico4_slam_destroy(struct pico4_slam **ptr)
{
	struct pico4_slam *s = *ptr;
	if (s == NULL) {
		return;
	}
	// A failed create gets here with helpers never initialised; Monado asserts on those.
	if (s->oth.initialized) {
		os_thread_helper_destroy(&s->oth);
	}
	// The frame feeder may be inside a push; the IMU feeder keeps Basalt draining until it returns.
	if (s->push_oth.initialized) {
		os_thread_helper_destroy(&s->push_oth);
	}
	/*
	 * Let Basalt work through the frames it has queued before it stops: its
	 * optical flow carries on after the estimator has finished and then
	 * predicts from an invalid state (an abort in Sophus or a crash, seen when
	 * the optical flow lagged behind).
	 */
	if (s->tracker != NULL && s->sets_pushed > 0) {
		os_nanosleep(TRACKER_DRAIN_NS);
	}
	if (s->imu_oth.initialized) {
		os_thread_helper_destroy(&s->imu_oth);
	}
	for (int i = 0; i < PICO4_TCAM_COUNT; i++) {
		xrt_frame_reference(&s->mail[i], NULL);
	}
	if (s->station != NULL && s->cam != NULL) {
		pico4_station_camera_stop(s->station);
	}
	rgb_let_go(s);
	pico4_cam_close(s->cam);
	drop_group(s);
	xrt_frame_context_destroy_nodes(&s->xfctx);
	SLAM_INFO(s,
	          "SLAM: %llu frame sets, %llu incomplete, %llu skipped while the tracker was busy, "
	          "%llu IMU samples dropped; %llu LED sets to the controller tracker, %llu incomplete",
	          (unsigned long long)s->sets_pushed, (unsigned long long)s->sets_dropped,
	          (unsigned long long)s->sets_busy, (unsigned long long)s->imu_overflow,
	          (unsigned long long)s->led_sets, (unsigned long long)s->led_sets_dropped);
	if (s->stock) {
		SLAM_INFO(s, "PICO OS's tracker: %llu frame sets and %llu IMU samples to pico4-slamd",
		          (unsigned long long)s->stock_sets, (unsigned long long)s->stock_imu);
	}
	if (s->feed != NULL) {
		munmap(s->feed, sizeof(struct pico4_slam_feed));
	}
	for (int k = 0; k < PICO4_TCAM_COUNT; k++) {
		free(s->leds.images[k]);
	}
	pico4_passthrough_destroy(&s->passthrough);
	free(s);
	*ptr = NULL;
}

struct pico4_passthrough *
pico4_slam_passthrough(struct pico4_slam *s)
{
	return s != NULL ? s->passthrough : NULL;
}

#else // XRT_FEATURE_SLAM

bool
pico4_slam_fuses_imu(const struct pico4_slam *s)
{
	(void)s;
	return false;
}

struct pico4_slam *
pico4_slam_create(const char *calibration_dir,
                  struct pico4_station *station,
                  double imu_hz,
                  struct pico4_ctrack *ctrack,
                  enum u_logging_level log_level)
{
	(void)calibration_dir;
	(void)station;
	(void)imu_hz;
	(void)ctrack;
	(void)log_level;
	return NULL;
}

void
pico4_slam_push_imu(
    struct pico4_slam *s, int64_t timestamp_ns, const float accel[3], const float gyro[3], float temperature_c)
{
	(void)s;
	(void)timestamp_ns;
	(void)accel;
	(void)gyro;
	(void)temperature_c;
}

bool
pico4_slam_get_head_pose(struct pico4_slam *s, int64_t at_ns, struct xrt_space_relation *out)
{
	(void)s;
	(void)at_ns;
	(void)out;
	return false;
}

bool
pico4_slam_get_latest(struct pico4_slam *s, struct pico4_fusion_slam *out)
{
	(void)s;
	(void)out;
	return false;
}

void
pico4_slam_reset(struct pico4_slam *s)
{
	(void)s;
}

void
pico4_slam_destroy(struct pico4_slam **s)
{
	*s = NULL;
}

struct pico4_passthrough *
pico4_slam_passthrough(struct pico4_slam *s)
{
	(void)s;
	return NULL;
}

bool
pico4_slam_room(struct pico4_slam *s, char out_map[40])
{
	(void)s;
	(void)out_map;
	return false;
}

void
pico4_slam_detect_floor(struct pico4_slam *s)
{
	(void)s;
}

bool
pico4_slam_floor(struct pico4_slam *s, float *out_y)
{
	(void)s;
	(void)out_y;
	return false;
}

bool
pico4_slam_depth_mesh(
    struct pico4_slam *s, uint64_t after, float *vertices, uint32_t *indices, struct pico4_depth_mesh_info *out)
{
	(void)s;
	(void)after;
	(void)vertices;
	(void)indices;
	(void)out;
	return false;
}

#endif // XRT_FEATURE_SLAM
