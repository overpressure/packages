// Copyright 2026, overpressure contributors.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  PICO 4 HMD: head tracking from the IMU behind the sensor DSP and the tracking cameras.
 *
 * Accelerometer and gyroscope samples of the ICM-42688 come from the Qualcomm
 * Sensor Core (see pico4_ssc.c) with QTimer timestamps. They are rotated into
 * the HMD frame, fused with m_imu_3dof and pushed into a relation history in
 * the CLOCK_MONOTONIC timeline that Monado uses. With SLAM (pico4_slam.c) the
 * raw samples also go to the visual-inertial tracker. Its poses arrive about
 * 100 ms late at 25 Hz; pico4_fusion.c keeps the head pose at the IMU's rate
 * and corrects it with them. The 3DoF orientation covers the time before that.
 *
 * The HMD also owns the controller radio ("station", pico4_station.h): it
 * resets it once, which makes the controllers reconnect and lets SLAM start
 * the cameras' frame sync, and acknowledges its frames from a thread. The
 * controllers' packets arrive through the same sensor service (the "ble_mcu"
 * sensor) and go to the two controller devices (pico4_controller.c).
 *
 * @ingroup drv_pico4
 */

#include "pico4_blemcu.h"
#include "pico4_cam.h"
#include "pico4_controller.h"
#include "pico4_ctrack.h"
#include "pico4_fusion.h"
#include "pico4_interface.h"
#include "pico4_ipd.h"
#include "pico4_lens.h"
#include "pico4_roomview.h"
#include "pico4_slam.h"
#include "pico4_ssc.h"
#include "pico4_station.h"

#include "xrt/xrt_defines.h"
#include "xrt/xrt_device.h"
#include "xrt/xrt_prober.h"

#include "os/os_threading.h"
#include "os/os_time.h"

#include "math/m_api.h"
#include "math/m_imu_3dof.h"
#include "math/m_mathinclude.h" // IWYU pragma: keep
#include "math/m_relation_history.h"
#include "math/m_vec3.h"

#include "util/u_debug.h"
#include "util/u_device.h"
#include "util/u_distortion_mesh.h"
#include "util/u_logging.h"
#include "util/u_misc.h"
#include "util/u_time.h"
#include "util/u_var.h"
#include "util/u_visibility_mask.h"

#include <dirent.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <math.h>
#include <string.h>
#include <unistd.h>

DEBUG_GET_ONCE_LOG_OPTION(pico4_log, "PICO4_LOG", U_LOGGING_INFO)
DEBUG_GET_ONCE_NUM_OPTION(pico4_imu_rate, "PICO4_IMU_RATE", 1000)
//! The tracking cameras' rate: 50 Hz as stock, or PICO OS's other rate, 60 Hz (scene frames at
//! 30 Hz instead of 25 for the room view; docs/xr.md#passthrough).
DEBUG_GET_ONCE_NUM_OPTION(pico4_camera_hz, "PICO4_CAMERA_HZ", 50)
DEBUG_GET_ONCE_OPTION(pico4_calibration, "PICO4_CALIBRATION_DIR", "/usr/share/qcom/pico4/calibration")
/*!
 * Correct chromatic aberration in the distortion pass. Off by default, as on
 * PICO OS (sdk_chromaticStaus 0): three separate texture reads per pixel cost
 * the layer squasher the memory bandwidth it needs for 90 Hz.
 */
DEBUG_GET_ONCE_BOOL_OPTION(pico4_lens_ca, "PICO4_LENS_CA", false)
//! Fuse the SLAM poses with the IMU (pico4_fusion.c) rather than taking Monado's prediction of them.
DEBUG_GET_ONCE_BOOL_OPTION(pico4_fusion, "PICO4_FUSION", true)
DEBUG_GET_ONCE_NUM_OPTION(pico4_fusion_position_ms, "PICO4_FUSION_POSITION_MS", 150)
DEBUG_GET_ONCE_NUM_OPTION(pico4_fusion_smoothing_ms, "PICO4_FUSION_SMOOTHING_MS", 50)
/*!
 * Hand out the head pose this much later than asked for (fusion only), predicted from the
 * IMU-rate state. SteamVR asks for the present and extrapolates by about 11 ms, while a pose
 * handed out reaches the vblank 12 ms later, plus the display's lead (latency probe, docs/xr.md);
 * the rest showed as lag on head turns and as the world swimming with small head motion.
 */
DEBUG_GET_ONCE_NUM_OPTION(pico4_predict_ms, "PICO4_PREDICT_MS", 12)
/*!
 * Latency probe, with the fusion: periods of this many milliseconds cycle
 * through four phases. In phase 1 the head pose handed out is turned by 5 degrees about
 * the vertical (velocity unchanged); in phase 3 it reports a turn of 30 deg/s about the
 * vertical without turning. The display layer's motion log shows when the step reaches
 * the panels and how far SteamVR extrapolates the reported turn. Only for a headset
 * lying still.
 */
DEBUG_GET_ONCE_NUM_OPTION(pico4_latency_probe_ms, "PICO4_LATENCY_PROBE_MS", 0)
//! Test of the runaway path: reset the SLAM tracker once, this many seconds after the fusion started.
DEBUG_GET_ONCE_NUM_OPTION(pico4_slam_reset_after_s, "PICO4_SLAM_RESET_AFTER_S", 0)
//! Directory for fusion_slam.csv (each SLAM pose) and fusion_pose.csv (each pose handed out).
DEBUG_GET_ONCE_OPTION(pico4_fusion_csv, "PICO4_FUSION_CSV", NULL)

#define P4_DEBUG(h, ...) U_LOG_XDEV_IFL_D(&h->base, h->log_level, __VA_ARGS__)
#define P4_INFO(h, ...) U_LOG_XDEV_IFL_I(&h->base, h->log_level, __VA_ARGS__)
#define P4_WARN(h, ...) U_LOG_XDEV_IFL_W(&h->base, h->log_level, __VA_ARGS__)
#define P4_ERROR(h, ...) U_LOG_XDEV_IFL_E(&h->base, h->log_level, __VA_ARGS__)

//! QTimer rate of the SSC timestamps; also the AP's arch counter on this SoC.
#define QTIMER_HZ 19200000ULL

//! Accelerometer/gyroscope lookups of 3 s each before giving up (DSP still starting).
#define PICO4_LOOKUP_ATTEMPTS 5
//! The headset's battery in the power-supply class.
#define PICO4_BATTERY "/sys/class/power_supply/battery"
//! The panels' backlight (the panel driver's).
#define PICO4_BACKLIGHT_POWER "/sys/class/backlight/pico4-panel/bl_power"
//! Gyro samples ignored after enabling (the accelerometer settles), then averaged for the start pose.
#define PICO4_WARMUP_SKIP 100
#define PICO4_WARMUP_AVERAGE 100
//! The ICM-42688's temperature stream: its fastest rate (it offers 1 and 5 Hz).
#define PICO4_TEMPERATURE_HZ 5.0f

//! Samples per stillness window of the gyro bias estimate (a quarter second at 1 kHz).
#define PICO4_STILL_WINDOW 256

/*!
 * The PICO 4 HMD.
 *
 * @implements xrt_device
 */
struct pico4_hmd
{
	struct xrt_device base;

	struct pico4_ssc *ssc;
	struct pico4_ssc_suid accel_suid, gyro_suid, blemcu_suid, temperature_suid, proximity_suid;
	bool has_blemcu, has_temperature, has_proximity;
	//! The wear sensor: -1 not reported yet, 0 off the head, 1 worn (pico4_hmd_get_presence).
	int worn;
	//! The ICM-42688's die temperature, °C, NaN until its first report (PICO OS's tracker uses it).
	float temperature_c;
	//! The IMU stream is lost and being reconnected: the head pose is untracked meanwhile.
	bool imu_stale;
	struct os_thread_helper oth;

	//! Controller radio, NULL if unavailable; its own thread acknowledges frames.
	struct pico4_station *station;
	struct os_thread_helper station_oth;
	//! Pairing while the session runs (pico4-station, the onboarding): served by the station thread.
	struct pico4_station_control *station_control;
	struct pico4_controller *controllers[2];
	uint64_t blemcu_packets;
	uint64_t blemcu_future; //!< Controller IMU samples stamped after their arrival, dropped.
	/*!
	 * The ble_mcu event time minus the packet's station time (both QTimer), over the packets since
	 * the last log line: the controller samples are timed from the event time (station thread
	 * logs nothing of it; the IMU thread logs every 30 s).
	 */
	int64_t blemcu_clock_min_ns, blemcu_clock_max_ns;
	uint64_t blemcu_clock_n;
	int64_t blemcu_clock_log_ns;
	//! Station thread: per hand, connected, and the connection whose unit data went to the tracker.
	bool controller_connected[2];
	uint32_t unit_connects[2];
	//! Controller tracking from the LED exposures, NULL if unavailable.
	struct pico4_ctrack *ctrack;

	//! CLOCK_MONOTONIC ns minus the QTimer time in ns.
	int64_t qtimer_offset_ns;

	struct m_imu_3dof fusion;
	bool fusion_started;
	uint32_t warmup_samples;
	struct xrt_vec3 warmup_accel;

	//! Gyro bias, learned while the headset is still (see update_bias()).
	struct xrt_vec3 gyro_bias;
	bool gyro_bias_valid;
	struct
	{
		struct xrt_vec3 gyro_sum, gyro_sq, accel_sum, accel_sq;
		uint32_t n;
	} still;
	struct xrt_vec3 last_accel;
	float last_accel_raw[3];
	bool have_accel;

	//! Visual-inertial tracking, NULL when unavailable.
	struct pico4_slam *slam;
	struct m_relation_history *relation_hist;
	/*!
	 * While SLAM has no current pose: its newest position is held, and the 3DoF
	 * orientation is turned into SLAM's frame by the rotation between the two
	 * last seen together (their headings differ).
	 */
	struct xrt_vec3 slam_position;
	struct xrt_quat slam_from_3dof;
	bool have_slam_position;

	//! The head pose at the IMU's rate with the SLAM poses as corrections; used when use_track.
	struct pico4_fusion track;
	bool use_track;
	FILE *slam_csv, *pose_csv;
	int64_t slam_reset_ns; //!< When the SLAM tracker was last told to reset.
	int64_t track_started_ns;
	bool reset_tested;
	int64_t probe_start_ns; //!< First pose query of the latency probe.
	int64_t lead_ns, lead_read_ns; //!< The display layer's vsync lead, and when it was read.

	//! Lens model per eye from the factory data, if found.
	struct pico4_lens lens[2];
	struct pico4_lens_ca ca;
	bool has_lens;

	//! Lens separation read from the IPD MCU, metres.
	double ipd_m; //!< Written by the IPD thread, read with relaxed atomics.
	struct os_thread_helper ipd_oth;

	//! The newest gyro sample's time: the 3DoF filter needs each one later than the last.
	int64_t last_gyro_ns;

	// Statistics, also shown in the debug GUI.
	uint64_t gyro_samples;
	uint64_t accel_samples;
	int64_t last_latency_ns;

	enum u_logging_level log_level;
};

static inline struct pico4_hmd *
pico4_hmd(struct xrt_device *xdev)
{
	return (struct pico4_hmd *)xdev;
}

static uint64_t
qtimer_to_ns(uint64_t ticks)
{
	// 1e9 / 19.2e6 = 625 / 12, split to stay exact and avoid overflow.
	return (ticks / 12) * 625 + (ticks % 12) * 625 / 12;
}

#if defined(__aarch64__)
static uint64_t
read_counter(void)
{
	uint64_t value;
	__asm__ volatile("isb; mrs %0, cntvct_el0" : "=r"(value)::"memory");
	return value;
}

static uint64_t
counter_frequency(void)
{
	uint64_t value;
	__asm__ volatile("mrs %0, cntfrq_el0" : "=r"(value));
	return value;
}
#endif

/*!
 * Offset between the SSC timestamps and CLOCK_MONOTONIC. On the XR2 the arch
 * counter is the QTimer, so the narrowest of a few (counter, monotonic) pairs
 * gives it directly; elsewhere it is set from the first sample's arrival.
 */
static bool
calibrate_clock(struct pico4_hmd *h)
{
#if defined(__aarch64__)
	if (counter_frequency() != QTIMER_HZ) {
		P4_ERROR(h, "Arch counter runs at %llu Hz, not the QTimer rate", (unsigned long long)counter_frequency());
		return false;
	}
	int64_t best_window = INT64_MAX;
	for (int i = 0; i < 16; i++) {
		int64_t before = os_monotonic_get_ns();
		uint64_t ticks = read_counter();
		int64_t after = os_monotonic_get_ns();
		if (after - before < best_window) {
			best_window = after - before;
			h->qtimer_offset_ns = before + (after - before) / 2 - (int64_t)qtimer_to_ns(ticks);
		}
	}
	return true;
#else
	return false;
#endif
}

//! IMU axes into the HMD frame (x right, y up, z back): from the factory camera extrinsics.
static struct xrt_vec3
imu_to_hmd(const float v[3])
{
	return (struct xrt_vec3){-v[1], v[0], v[2]};
}

/*!
 * Orientation (world from HMD) that takes the measured gravity reaction @p a,
 * which points up when still, to world +Y; no rotation about the vertical.
 */
static struct xrt_quat
rotation_to_up(const struct xrt_vec3 *a)
{
	float len = sqrtf(a->x * a->x + a->y * a->y + a->z * a->z);
	if (len < 1e-3f) {
		return (struct xrt_quat)XRT_QUAT_IDENTITY;
	}
	struct xrt_vec3 n = {a->x / len, a->y / len, a->z / len};
	// Half-way quaternion between n and up: axis n x up, w = 1 + n . up.
	struct xrt_quat q = {-n.z, 0.0f, n.x, 1.0f + n.y};
	if (q.w < 1e-6f) {
		return (struct xrt_quat){1.0f, 0.0f, 0.0f, 0.0f}; // upside down: 180 degrees about X
	}
	math_quat_normalize(&q);
	return q;
}

static float
window_max_std(const struct xrt_vec3 *sum, const struct xrt_vec3 *sq, uint32_t n)
{
	float mx = sum->x / n, my = sum->y / n, mz = sum->z / n;
	float vx = sq->x / n - mx * mx, vy = sq->y / n - my * my, vz = sq->z / n - mz * mz;
	return sqrtf(fmaxf(fmaxf(vx, vy), fmaxf(vz, 0.0f)));
}

/*!
 * The SSC's "gyro" stream keeps a bias of a few mrad/s, enough to tilt the
 * fusion away from gravity faster than m_imu_3dof pulls it back. Learn it over
 * windows in which gyro and accelerometer are both quiet and the rate is small:
 * a still headset, not a slow deliberate turn.
 */
static void
update_bias(struct pico4_hmd *h, const struct xrt_vec3 *gyro, const struct xrt_vec3 *accel)
{
	struct xrt_vec3 g2 = {gyro->x * gyro->x, gyro->y * gyro->y, gyro->z * gyro->z};
	struct xrt_vec3 a2 = {accel->x * accel->x, accel->y * accel->y, accel->z * accel->z};
	math_vec3_accum(gyro, &h->still.gyro_sum);
	math_vec3_accum(&g2, &h->still.gyro_sq);
	math_vec3_accum(accel, &h->still.accel_sum);
	math_vec3_accum(&a2, &h->still.accel_sq);
	if (++h->still.n < PICO4_STILL_WINDOW) {
		return;
	}

	uint32_t n = h->still.n;
	struct xrt_vec3 mean = h->still.gyro_sum;
	math_vec3_scalar_mul(1.0f / n, &mean);
	struct xrt_vec3 a_mean = h->still.accel_sum;
	math_vec3_scalar_mul(1.0f / n, &a_mean);
	float g_std = window_max_std(&h->still.gyro_sum, &h->still.gyro_sq, n);
	float a_std = window_max_std(&h->still.accel_sum, &h->still.accel_sq, n);
	float rate = m_vec3_len(mean);
	float g = m_vec3_len(a_mean);
	memset(&h->still, 0, sizeof(h->still));

	if (g_std > 0.01f || a_std > 0.05f || rate > 0.02f || fabsf(g - (float)MATH_GRAVITY_M_S2) > 0.3f) {
		return;
	}
	if (!h->gyro_bias_valid) {
		h->gyro_bias = mean;
		h->gyro_bias_valid = true;
	} else {
		h->gyro_bias.x += 0.2f * (mean.x - h->gyro_bias.x);
		h->gyro_bias.y += 0.2f * (mean.y - h->gyro_bias.y);
		h->gyro_bias.z += 0.2f * (mean.z - h->gyro_bias.z);
	}
}

static bool
suid_equal(const struct pico4_ssc_suid *a, const struct pico4_ssc_suid *b)
{
	return a->low == b->low && a->high == b->high;
}

//! A new SLAM pose for the fusion; logs what it made of it.
static void
track_slam_pose(struct pico4_hmd *h, const struct pico4_fusion_slam *m)
{
	enum pico4_fusion_result r = pico4_fusion_slam(&h->track, m);
	int64_t now = os_monotonic_get_ns();
	if (r == PICO4_FUSION_STARTED) {
		h->track_started_ns = now;
		P4_INFO(h, "6DoF: fusing SLAM poses with the IMU, the first %.0f ms after its images",
		        (now - m->ts_ns) / 1e6);
	} else if (r == PICO4_FUSION_RESET) {
		P4_WARN(h, "6DoF: following SLAM after a gap, %.3f m and %.1f deg off", h->track.last_error_m,
		        h->track.last_error_rad * 180.0f / (float)M_PI);
	} else if (r == PICO4_FUSION_ANCHORED) {
		P4_INFO(h, "6DoF: SLAM's world re-anchored (it jumped, or a new map after a reset)");
	} else if (r == PICO4_FUSION_RUNAWAY) {
		P4_WARN(h, "6DoF: SLAM ran away (%.2f m/s, its velocity %.2f m/s off the IMU's): holding the position",
		        h->track.last_speed, h->track.last_innovation);
	}
	long test_s = debug_get_num_option_pico4_slam_reset_after_s();
	if (test_s > 0 && !h->reset_tested && h->track.started && now - h->track_started_ns > test_s * U_TIME_1S_IN_NS) {
		P4_WARN(h, "6DoF: resetting the SLAM tracker as a test (PICO4_SLAM_RESET_AFTER_S)");
		pico4_slam_reset(h->slam);
		h->reset_tested = true;
	}
	// A runaway tracker rarely recovers by itself: start a new map, again while it stays lost.
	if (h->track.lost && now - h->slam_reset_ns > 5 * (int64_t)U_TIME_1S_IN_NS) {
		P4_WARN(h, "6DoF: resetting the SLAM tracker");
		pico4_slam_reset(h->slam);
		pico4_fusion_tracker_reset(&h->track, now);
		h->slam_reset_ns = now;
	}
	if (h->slam_csv != NULL) {
		fprintf(h->slam_csv, "%lld,%lld,%d,%.4f,%.3f,%.4f,%.4f,%.4f,%.3f,%.3f\n", (long long)m->ts_ns,
		        (long long)now, (int)r, h->track.last_error_m, h->track.last_error_rad * 180.0f / (float)M_PI,
		        m->position.x, m->position.y, m->position.z, h->track.last_innovation, h->track.last_speed);
	}
}

/*!
 * The controller packets' event time against their station time: PICO OS times the
 * samples from the station time, the driver from the event time, which agree while this stays
 * within a microsecond. Logged every 30 s as the range since the last line.
 */
static void
log_blemcu_clock(struct pico4_hmd *h, int64_t diff_ns, int64_t now_ns)
{
	if (h->blemcu_clock_n == 0 || diff_ns < h->blemcu_clock_min_ns) {
		h->blemcu_clock_min_ns = diff_ns;
	}
	if (h->blemcu_clock_n == 0 || diff_ns > h->blemcu_clock_max_ns) {
		h->blemcu_clock_max_ns = diff_ns;
	}
	h->blemcu_clock_n++;
	if (h->blemcu_clock_log_ns == 0) {
		h->blemcu_clock_log_ns = now_ns;
	} else if (now_ns - h->blemcu_clock_log_ns >= 30 * (int64_t)U_TIME_1S_IN_NS) {
		P4_INFO(h, "Controller packets: event time minus station time %.2f..%.2f us over %llu packets",
		        h->blemcu_clock_min_ns / 1e3, h->blemcu_clock_max_ns / 1e3, (unsigned long long)h->blemcu_clock_n);
		h->blemcu_clock_n = 0;
		h->blemcu_clock_log_ns = now_ns;
	}
}

/*!
 * The wear state for the session's microphone mute (pico4-micmute): $XDG_RUNTIME_DIR/pico4-worn,
 * "1" worn or "0" off the head, written atomically on each change.
 */
static void
publish_worn(int worn)
{
	const char *dir = getenv("XDG_RUNTIME_DIR");
	char path[512], tmp[520];
	snprintf(path, sizeof(path), "%s/pico4-worn", dir != NULL ? dir : "/run");
	snprintf(tmp, sizeof(tmp), "%s.new", path);
	FILE *f = fopen(tmp, "we");
	if (f == NULL) {
		return;
	}
	fputs(worn ? "1\n" : "0\n", f);
	if (fclose(f) == 0) {
		rename(tmp, path);
	}
}

static void
handle_sample(struct pico4_hmd *h, const struct pico4_ssc_sample *s)
{
	if (s->proximity) {
		// As PICO OS's sensor HAL: only events of high accuracy (status 3) count; 1 is near.
		if (h->has_proximity && s->count >= 3 && s->values[2] == 3.0f && suid_equal(&s->suid, &h->proximity_suid)) {
			int worn = s->values[0] >= 0.5f;
			if (__atomic_exchange_n(&h->worn, worn, __ATOMIC_RELAXED) != worn) {
				P4_INFO(h, "Wear sensor: %s (raw %.0f)", worn ? "worn" : "off the head", s->values[1]);
				publish_worn(worn);
			}
		}
		return;
	}
	if (h->has_temperature && s->count >= 1 && suid_equal(&s->suid, &h->temperature_suid)) {
		h->temperature_c = s->values[0]; // read below, on the same (IMU) thread
		return;
	}
	if (s->count < 3) {
		return;
	}
	int64_t ts = (int64_t)qtimer_to_ns(s->ticks) + h->qtimer_offset_ns;
	if (h->has_blemcu && suid_equal(&s->suid, &h->blemcu_suid)) {
		struct pico4_blemcu_packet p;
		if (s->raw_len > 0 && pico4_blemcu_parse(s->raw, s->raw_len, &p)) {
			h->blemcu_packets++;
			h->blemcu_future += (uint64_t)pico4_blemcu_drop_future(&p, ts, (int64_t)os_monotonic_get_ns());
			log_blemcu_clock(h, (int64_t)qtimer_to_ns(s->ticks) - (int64_t)p.station_time_us * 1000, ts);
			for (int i = 0; i < 2; i++) {
				if (h->controllers[i] != NULL) {
					pico4_controller_push(h->controllers[i], &p, ts);
				}
			}
			if (h->ctrack != NULL) {
				pico4_ctrack_push_packet(h->ctrack, &p, ts);
			}
		}
		return;
	}
	struct xrt_vec3 v = imu_to_hmd(s->values);

	if (suid_equal(&s->suid, &h->accel_suid)) {
		h->last_accel = v;
		memcpy(h->last_accel_raw, s->values, sizeof(h->last_accel_raw));
		h->have_accel = true;
		h->accel_samples++;
		return;
	}
	if (!suid_equal(&s->suid, &h->gyro_suid) || !h->have_accel) {
		return;
	}
	// Monado's 3DoF filter asserts on a step back, which killed vrserver after a resume.
	if (ts <= h->last_gyro_ns) {
		return;
	}
	h->last_gyro_ns = ts;
	h->gyro_samples++;
	h->last_latency_ns = os_monotonic_get_ns() - ts;
	if (h->slam != NULL) {
		pico4_slam_push_imu(h->slam, ts, h->last_accel_raw, s->values, h->temperature_c);
	}

	update_bias(h, &v, &h->last_accel);
	if (h->gyro_bias_valid) {
		math_vec3_subtract(&h->gyro_bias, &v);
	}

	if (!h->fusion_started) {
		/*
		 * m_imu_3dof starts at identity and only slowly pulls towards gravity,
		 * and the first accelerometer samples after enabling are not settled
		 * yet (observed: 2 degrees off). Skip those, average the next ones and
		 * start the fusion level with that gravity mapped to +Y.
		 */
		h->warmup_samples++;
		if (h->warmup_samples > PICO4_WARMUP_SKIP) {
			math_vec3_accum(&h->last_accel, &h->warmup_accel);
		}
		if (h->warmup_samples < PICO4_WARMUP_SKIP + PICO4_WARMUP_AVERAGE) {
			return;
		}
		h->fusion.rot = rotation_to_up(&h->warmup_accel);
		h->fusion_started = true;
		P4_DEBUG(h, "fusion started after %u samples", h->warmup_samples);
	}
	m_imu_3dof_update(&h->fusion, (uint64_t)ts, &h->last_accel, &v);

	struct xrt_space_relation rel = XRT_SPACE_RELATION_ZERO;
	if (h->use_track) {
		pico4_fusion_imu(&h->track, ts, &h->fusion.rot, &v, &h->last_accel);
		struct pico4_fusion_slam m;
		if (pico4_slam_get_latest(h->slam, &m) && m.ts_ns > h->track.seen_ts_ns) {
			track_slam_pose(h, &m);
		}
		pico4_fusion_relation(&h->track, &rel);
	} else {
		rel.relation_flags = (enum xrt_space_relation_flags)(XRT_SPACE_RELATION_ORIENTATION_VALID_BIT |
		                                                      XRT_SPACE_RELATION_ORIENTATION_TRACKED_BIT |
		                                                      XRT_SPACE_RELATION_ANGULAR_VELOCITY_VALID_BIT);
		rel.pose.orientation = h->fusion.rot;
		math_quat_rotate_derivative(&h->fusion.rot, &v, &rel.angular_velocity);
	}
	m_relation_history_push(h->relation_hist, &rel, ts);

	if (h->gyro_samples % 5000 == 1 && h->log_level <= U_LOGGING_DEBUG) {
		// Tilt of the head's up axis in the world, and of the measured gravity in the head.
		struct xrt_vec3 up = {0, 1, 0}, head_up;
		math_quat_rotate_vec3(&h->fusion.rot, &up, &head_up);
		float a = sqrtf(h->last_accel.x * h->last_accel.x + h->last_accel.y * h->last_accel.y +
		                h->last_accel.z * h->last_accel.z);
		P4_DEBUG(h, "samples %llu, latency %.2f ms, fusion tilt %.3f deg, accel tilt %.3f deg, bias %s (%.4f %.4f %.4f) rad/s",
		         (unsigned long long)h->gyro_samples, h->last_latency_ns / 1e6,
		         acosf(fminf(1.0f, head_up.y)) * 180.0f / (float)M_PI,
		         acosf(fminf(1.0f, h->last_accel.y / a)) * 180.0f / (float)M_PI, h->gyro_bias_valid ? "learned" : "none",
         h->gyro_bias.x, h->gyro_bias.y, h->gyro_bias.z);
	}
}

//! No IMU samples for this long: the stream is lost (it delivers 500-1000 per second).
#define IMU_LOST_NS (1000 * U_TIME_1MS_IN_NS)

static bool
imu_thread_running(struct pico4_hmd *h)
{
	os_thread_helper_lock(&h->oth);
	bool running = os_thread_helper_is_running_locked(&h->oth);
	os_thread_helper_unlock(&h->oth);
	return running;
}

/*!
 * The IMU's die temperature, which PICO OS's tracker takes with every IMU sample (its gyro bias
 * table is binned by it); on PICO OS the sensor DSP inserts it into each gyro sample. Without
 * it the tracker gets a fixed value (pico4-slamd). On the IMU thread, which owns the client.
 */
static void
setup_temperature(struct pico4_hmd *h, int timeout_ms)
{
	h->has_temperature = pico4_ssc_find(h->ssc, "sensor_temperature", &h->temperature_suid, timeout_ms) == 0 &&
	                     pico4_ssc_enable(h->ssc, &h->temperature_suid, PICO4_TEMPERATURE_HZ) >= 0;
	if (!h->has_temperature) {
		P4_WARN(h, "No IMU temperature from the sensor service");
	}
}

/*!
 * The wear sensor (STK3x1x proximity on the sensor DSP), enabled as PICO OS enables it: the
 * on-change request with an empty payload; a standard request crashed the SLPI. On the IMU
 * thread, which owns the client.
 */
static void
setup_proximity(struct pico4_hmd *h, int timeout_ms)
{
	h->has_proximity = pico4_ssc_find(h->ssc, "proximity", &h->proximity_suid, timeout_ms) == 0 &&
	                   pico4_ssc_enable_on_change(h->ssc, &h->proximity_suid) >= 0;
	if (!h->has_proximity) {
		P4_WARN(h, "No wear sensor from the sensor service");
	}
}

/*!
 * The IMU stream is gone, typically because the sensor DSP restarted: its service, the
 * sensor UIDs and the QTimer mapping may all have changed. Report the head pose untracked,
 * then open a new client, find the sensors again, map the clock and subscribe, until it
 * works or the device goes away. The station and the cameras are not touched.
 */
static void
recover_imu(struct pico4_hmd *h, float rate, const char *why)
{
	__atomic_store_n(&h->imu_stale, true, __ATOMIC_RELEASE);
	P4_ERROR(h, "IMU stream lost (%s): head pose untracked until it is back", why);
	int64_t start = os_monotonic_get_ns();
	pico4_ssc_close(&h->ssc);
	// Steps of a second at most: a service stop must not wait long for the sensor DSP.
	static const char *const types[2] = {"accel", "gyro"};
	struct pico4_ssc_suid suids[2];
	for (int attempt = 1; imu_thread_running(h); attempt++) {
		os_nanosleep(U_TIME_1S_IN_NS / 2);
		if (pico4_ssc_subscribe(&h->ssc, types, suids, 2, rate, 1000) == 0 && calibrate_clock(h)) {
			h->accel_suid = suids[0];
			h->gyro_suid = suids[1];
			if (h->has_blemcu && (pico4_ssc_find(h->ssc, "ble_mcu", &h->blemcu_suid, 1000) != 0 ||
			                      pico4_ssc_enable(h->ssc, &h->blemcu_suid, rate) < 0)) {
				P4_ERROR(h, "The controller stream did not come back with the IMU");
			}
			setup_temperature(h, 1000);
			setup_proximity(h, 1000);
			__atomic_store_n(&h->imu_stale, false, __ATOMIC_RELEASE);
			P4_INFO(h, "IMU stream back after %.1f s (attempt %d), QTimer offset %lld ns",
			        (double)(os_monotonic_get_ns() - start) / U_TIME_1S_IN_NS, attempt,
			        (long long)h->qtimer_offset_ns);
			return;
		}
		pico4_ssc_close(&h->ssc);
	}
}

static void *
imu_thread(void *ptr)
{
	struct pico4_hmd *h = (struct pico4_hmd *)ptr;
	struct pico4_ssc_sample samples[64];
	float rate = (float)debug_get_num_option_pico4_imu_rate();
	int64_t last_data_ns = os_monotonic_get_ns(), clock_ns = last_data_ns;
	int64_t slept_ns = pico4_slept_ns();

	os_thread_helper_name(&h->oth, "PICO 4 IMU");
	os_thread_helper_lock(&h->oth);
	while (os_thread_helper_is_running_locked(&h->oth)) {
		os_thread_helper_unlock(&h->oth);
		int n = pico4_ssc_read(h->ssc, 100, samples, (int)ARRAY_SIZE(samples));
		int64_t now = os_monotonic_get_ns();
		if (n > 0) {
			last_data_ns = now;
		}
		/*
		 * timesyncd steers CLOCK_MONOTONIC against the QTimer (15 ppm measured): an offset taken
		 * once is 0.3 s off after five hours, and the poses then look stale. Taken again each
		 * second, on the only thread that maps the sensor DSP's times.
		 */
		if (now - clock_ns >= U_TIME_1S_IN_NS) {
			calibrate_clock(h);
			clock_ns = now;
		}
		/*
		 * After a suspend the QTimer is ahead of CLOCK_MONOTONIC by the time slept: the offset is
		 * taken again before these samples are mapped, or they would land that far in the future
		 * and the next ones step back.
		 */
		int64_t slept = pico4_slept_ns();
		if (slept - slept_ns > PICO4_SLEEP_DETECT_NS) {
			calibrate_clock(h);
			clock_ns = now;
			P4_INFO(h, "The system slept %.1f s: QTimer offset taken again", (double)(slept - slept_ns) / 1e9);
		}
		slept_ns = slept;
		bool gone = n == -ECONNRESET, silent = now - last_data_ns > IMU_LOST_NS;
		if (n < 0 && !gone && !silent) {
			P4_ERROR(h, "Reading the sensor service failed: %s", strerror(-n));
			os_nanosleep(U_TIME_1MS_IN_NS * 100);
		}
		for (int i = 0; i < n; i++) {
			handle_sample(h, &samples[i]);
		}
		if (gone || silent) {
			recover_imu(h, rate, gone ? "the sensor service went away" : "no samples for a second");
			last_data_ns = os_monotonic_get_ns();
		}
		os_thread_helper_lock(&h->oth);
	}
	os_thread_helper_unlock(&h->oth);
	return NULL;
}

/*!
 * Controller connections, as PICO OS's station service handles them: on each connect, the
 * controller's calibration record from the station and its serial go to the tracker.
 */
static void
handle_connections(struct pico4_hmd *h)
{
	for (int i = 0; i < 2; i++) {
		const enum pico4_station_hand hand = i == 0 ? PICO4_STATION_HAND_LEFT : PICO4_STATION_HAND_RIGHT;
		const char *name = i == 0 ? "left" : "right";
		struct pico4_station_controller c;
		pico4_station_get_controller(h->station, hand, &c);
		if (c.connected != h->controller_connected[i]) {
			h->controller_connected[i] = c.connected;
			if (c.connected) {
				P4_INFO(h, "Controller %s connected (software %s)", name, c.software);
			} else {
				P4_INFO(h, "Controller %s disconnected", name);
			}
		}
		struct pico4_ctrack *ctrack = __atomic_load_n(&h->ctrack, __ATOMIC_ACQUIRE);
		if (!c.connected || c.connects == h->unit_connects[i] || ctrack == NULL) {
			continue;
		}
		h->unit_connects[i] = c.connects;
		uint8_t chunks[PICO4_STATION_CALIBRATION_CHUNKS][PICO4_STATION_CALIBRATION_CHUNK];
		int r = pico4_station_get_calibration(h->station, hand, chunks);
		if (r < 0) {
			P4_WARN(h, "Controller %s: calibration record unavailable (%s)", name, strerror(-r));
		}
		pico4_ctrack_set_unit(ctrack, i == 0 ? PICO4_BLEMCU_LEFT : PICO4_BLEMCU_RIGHT,
		                      r < 0 ? NULL : (const uint8_t(*)[32])chunks, c.serial);
	}
}

static void *
station_thread(void *ptr)
{
	struct pico4_hmd *h = (struct pico4_hmd *)ptr;

	os_thread_helper_name(&h->station_oth, "PICO 4 station");
	os_thread_helper_lock(&h->station_oth);
	while (os_thread_helper_is_running_locked(&h->station_oth)) {
		os_thread_helper_unlock(&h->station_oth);
		if (pico4_station_poll(h->station, 100) < 0) {
			os_nanosleep(U_TIME_1MS_IN_NS * 100);
		}
		handle_connections(h);
		char line[128];
		if (pico4_station_control_step(h->station_control, (int64_t)(os_monotonic_get_ns() / U_TIME_1MS_IN_NS), line,
		                               sizeof(line)) > 0) {
			P4_INFO(h, "Controller pairing: %s", line);
		}
		os_thread_helper_lock(&h->station_oth);
	}
	os_thread_helper_unlock(&h->station_oth);
	return NULL;
}

/*!
 * The lens separation while the session runs: pico4-lens moves the lenses. A move ends within
 * 0.05 mm of its target and readings jitter by 0.02 mm, so only a real move counts.
 */
static void *
ipd_thread(void *ptr)
{
	struct pico4_hmd *h = (struct pico4_hmd *)ptr;

	os_thread_helper_name(&h->ipd_oth, "PICO 4 lenses");
	os_thread_helper_lock(&h->ipd_oth);
	while (os_thread_helper_is_running_locked(&h->ipd_oth)) {
		os_thread_helper_unlock(&h->ipd_oth);
		os_nanosleep(U_TIME_1S_IN_NS);
		double now, known;
		__atomic_load(&h->ipd_m, &known, __ATOMIC_RELAXED);
		if (pico4_ipd_read(PICO4_IPD_DEVICE, &now) == 0 && fabs(now - known) > 0.0002) {
			__atomic_store(&h->ipd_m, &now, __ATOMIC_RELAXED);
			P4_INFO(h, "IPD %.2f mm", now * 1000);
		}
		os_thread_helper_lock(&h->ipd_oth);
	}
	os_thread_helper_unlock(&h->ipd_oth);
	return NULL;
}

static void
set_backlight(struct pico4_hmd *h, bool on)
{
	int fd = open(PICO4_BACKLIGHT_POWER, O_WRONLY | O_CLOEXEC);
	if (fd < 0) {
		return; // No panel driver.
	}
	// FB_BLANK_UNBLANK or FB_BLANK_POWERDOWN.
	if (write(fd, on ? "0" : "4", 1) != 1) {
		P4_WARN(h, "Backlight %s: %s", on ? "on" : "off", strerror(errno));
	}
	close(fd);
}

//! Reset the controller radio: the controllers reconnect and camera sync becomes possible.
static void
setup_station(struct pico4_hmd *h)
{
	int r = pico4_station_open(&h->station, NULL);
	if (r == 0) {
		pico4_station_set_camera_hz(h->station, (unsigned)debug_get_num_option_pico4_camera_hz());
		r = pico4_station_reset(h->station);
	}
	if (r == 0) {
		int c = pico4_station_control_open(&h->station_control, NULL, pico4_station_ops(h->station));
		if (c < 0) {
			P4_WARN(h, "Controller pairing socket: %s; pairing needs the session stopped", strerror(-c));
			h->station_control = NULL;
		}
	}
	if (r == 0 && (os_thread_helper_init(&h->station_oth) != 0 ||
	               os_thread_helper_start(&h->station_oth, station_thread, h) != 0)) {
		r = -ENOMEM;
	}
	if (r < 0) {
		P4_ERROR(h, "Controller radio %s: no controllers, no camera sync",
		         r == -EBUSY ? "in use by another runtime (SteamVR's driver_monado?)" : strerror(-r));
		pico4_station_control_close(h->station_control);
		h->station_control = NULL;
		pico4_station_close(h->station);
		h->station = NULL;
	}
}

/*!
 * The headset's battery, from its power supply (the PMIC's fuel gauge):
 * capacity in percent and the charger's status.
 */
static xrt_result_t
pico4_hmd_get_battery_status(struct xrt_device *xdev, bool *out_present, bool *out_charging, float *out_charge)
{
	(void)xdev;
	char capacity[16] = {0}, status[32] = {0};
	FILE *f = fopen(PICO4_BATTERY "/capacity", "re");
	bool ok = f != NULL && fgets(capacity, sizeof(capacity), f) != NULL;
	if (f != NULL) {
		fclose(f);
	}
	f = fopen(PICO4_BATTERY "/status", "re");
	if (f != NULL) {
		if (fgets(status, sizeof(status), f) == NULL) {
			status[0] = '\0';
		}
		fclose(f);
	}
	*out_present = ok;
	*out_charge = ok ? (float)atoi(capacity) / 100.f : 0.f;
	*out_charging = strncmp(status, "Charging", 8) == 0 || strncmp(status, "Full", 4) == 0;
	return XRT_SUCCESS;
}

static void
pico4_hmd_destroy(struct xrt_device *xdev)
{
	struct pico4_hmd *h = pico4_hmd(xdev);

	// A failed create gets here with helpers never initialised; Monado asserts on those.
	if (h->oth.initialized) {
		os_thread_helper_destroy(&h->oth);
	}
	if (h->ipd_oth.initialized) {
		os_thread_helper_destroy(&h->ipd_oth);
	}
	set_backlight(h, true);
	// Its thread asks for head poses, SLAM's included; SLAM's camera thread still pushes to it.
	pico4_ctrack_stop(h->ctrack);
	pico4_slam_destroy(&h->slam);
	if (h->station_oth.initialized) {
		os_thread_helper_destroy(&h->station_oth);
	}
	// A pairing still running stops before the station sleeps.
	pico4_station_control_close(h->station_control);
	// The controllers let go of the station (haptics) before it closes.
	for (int i = 0; i < 2; i++) {
		if (h->controllers[i] != NULL) {
			pico4_controller_release(h->controllers[i]);
		}
	}
	// Nobody serves the controllers now: they wait for the next start in slow advertising.
	int station_sleep = pico4_station_sleep(h->station);
	if (station_sleep < 0) {
		P4_WARN(h, "Controller radio to sleep: %s", strerror(-station_sleep));
	}
	pico4_station_close(h->station);
	// No frames (SLAM) or packets (IMU thread) any more, and the controllers let go of it.
	pico4_ctrack_destroy(&h->ctrack);
	if (h->ssc != NULL) {
		pico4_ssc_disable(h->ssc, &h->accel_suid);
		pico4_ssc_disable(h->ssc, &h->gyro_suid);
		if (h->has_blemcu) {
			pico4_ssc_disable(h->ssc, &h->blemcu_suid);
		}
		if (h->has_temperature) {
			pico4_ssc_disable(h->ssc, &h->temperature_suid);
		}
		if (h->has_proximity) {
			pico4_ssc_disable(h->ssc, &h->proximity_suid);
		}
		pico4_ssc_close(&h->ssc);
	}
	u_var_remove_root(h);
	if (h->slam_csv != NULL) {
		fclose(h->slam_csv);
	}
	if (h->pose_csv != NULL) {
		fclose(h->pose_csv);
	}
	m_imu_3dof_close(&h->fusion);
	m_relation_history_destroy(&h->relation_hist);
	u_device_free(&h->base);
}

//! The IMU-rate head pose that takes the SLAM poses as corrections (pico4_fusion.c).
static void
setup_track(struct pico4_hmd *h)
{
	if (!debug_get_bool_option_pico4_fusion()) {
		P4_INFO(h, "6DoF from Monado's prediction of the SLAM poses (PICO4_FUSION=0)");
		return;
	}
	const float hmd_in_body[3] = PICO4_HMD_IN_BODY;
	struct pico4_fusion_config c = {
	    .lever = imu_to_hmd(hmd_in_body),
	    .position_s = (float)debug_get_num_option_pico4_fusion_position_ms() / 1000.0f,
	    .velocity_s = (float)debug_get_num_option_pico4_fusion_position_ms() / 1000.0f,
	    .orientation_s = 0.5f,
	    .smoothing_s = (float)debug_get_num_option_pico4_fusion_smoothing_ms() / 1000.0f,
	    .gate_m = 0.15f,
	    .gate_rad = 10.0f * (float)M_PI / 180.0f,
	    .persist = 5,
	    .stale_ns = 500 * U_TIME_1MS_IN_NS,
	    .runaway_window_ns = 500 * U_TIME_1MS_IN_NS,
	    .runaway_mps = 0.5f,
	    .max_speed = 4.0f,
	    .still_mps = 0.15f,
	    .runaway_poses = 3,
	    .relock_poses = 5,
	};
	if (c.position_s <= 0.0f || c.smoothing_s <= 0.0f) {
		P4_ERROR(h, "PICO4_FUSION_POSITION_MS and PICO4_FUSION_SMOOTHING_MS must be positive");
		return;
	}
	pico4_fusion_init(&h->track, &c);
	h->use_track = true;

	const char *dir = debug_get_option_pico4_fusion_csv();
	if (dir != NULL) {
		char path[512];
		snprintf(path, sizeof(path), "%s/fusion_slam.csv", dir);
		h->slam_csv = fopen(path, "w");
		snprintf(path, sizeof(path), "%s/fusion_pose.csv", dir);
		h->pose_csv = fopen(path, "w");
		if (h->slam_csv == NULL || h->pose_csv == NULL) {
			P4_ERROR(h, "Cannot write the fusion CSVs to %s", dir);
		} else {
			// SteamVR may end the driver without destroying it: write each line as it comes.
			setvbuf(h->slam_csv, NULL, _IOLBF, 0);
			setvbuf(h->pose_csv, NULL, _IOLBF, 0);
			fprintf(h->slam_csv, "#image_ns,now_ns,result,error_m,error_deg,x,y,z,innovation_mps,speed_mps\n");
			fprintf(h->pose_csv, "#at_ns,now_ns,x,y,z,qx,qy,qz,qw,flags,vx,vy,vz,wx,wy,wz\n");
		}
	}
}

/*!
 * The display's vsync lead: pico4-swapchain-usage signals each vblank this much early and
 * publishes the lead it applies in $XDG_RUNTIME_DIR/pico4-display-lead-us (microseconds).
 * SteamVR takes the early signal for the vblank and predicts its poses for that much
 * earlier, so the lead is added to the prediction; 0 without the layer (Monado's own
 * compositor). Read at most once a second.
 */
static int64_t
display_lead_ns(struct pico4_hmd *h)
{
	int64_t now = (int64_t)os_monotonic_get_ns();
	int64_t lead = __atomic_load_n(&h->lead_ns, __ATOMIC_RELAXED);
	if (now - __atomic_load_n(&h->lead_read_ns, __ATOMIC_RELAXED) < U_TIME_1S_IN_NS) {
		return lead;
	}
	__atomic_store_n(&h->lead_read_ns, now, __ATOMIC_RELAXED);
	char path[512];
	const char *dir = getenv("XDG_RUNTIME_DIR");
	long long us = 0;
	snprintf(path, sizeof(path), "%s/pico4-display-lead-us", dir != NULL ? dir : "/run");
	FILE *f = fopen(path, "re");
	if (f != NULL) {
		if (fscanf(f, "%lld", &us) != 1 || us < 0 || us > 20000) {
			us = 0;
		}
		fclose(f);
	}
	if (us * 1000 != lead) {
		P4_INFO(h, "Display lead %lld us (pico4-swapchain-usage): poses predicted that much further", us);
		__atomic_store_n(&h->lead_ns, us * 1000, __ATOMIC_RELAXED);
	}
	return us * 1000;
}

//! The relation handed out for @p at_ns, to PICO4_FUSION_CSV's fusion_pose.csv when set.
static void
log_pose(struct pico4_hmd *h, int64_t at_ns, const struct xrt_space_relation *rel)
{
	if (h->pose_csv == NULL) {
		return;
	}
	const struct xrt_pose *p = &rel->pose;
	const struct xrt_vec3 *v = &rel->linear_velocity, *w = &rel->angular_velocity;
	fprintf(h->pose_csv, "%lld,%lld,%.5f,%.5f,%.5f,%.6f,%.6f,%.6f,%.6f,%d,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n",
	        (long long)at_ns, (long long)os_monotonic_get_ns(), p->position.x, p->position.y, p->position.z,
	        p->orientation.x, p->orientation.y, p->orientation.z, p->orientation.w, (int)rel->relation_flags, v->x,
	        v->y, v->z, w->x, w->y, w->z);
}

//! PICO4_LATENCY_PROBE_MS: known steps in the handed-out pose, for a headset lying still.
static void
latency_probe(struct pico4_hmd *h, int64_t at_timestamp_ns, struct xrt_space_relation *rel)
{
	int64_t probe_ns = debug_get_num_option_pico4_latency_probe_ms() * U_TIME_1MS_IN_NS;
	if (probe_ns <= 0) {
		return;
	}
	if (h->probe_start_ns == 0) {
		h->probe_start_ns = at_timestamp_ns;
		P4_INFO(h,
		        "latency probe: phases of %lld ns from %lld ns (CLOCK_MONOTONIC): 1 = 5 deg step, "
		        "3 = 30 deg/s reported turn",
		        (long long)probe_ns, (long long)at_timestamp_ns);
	}
	int64_t phase = ((at_timestamp_ns - h->probe_start_ns) / probe_ns) % 4;
	if (phase == 1) {
		const struct xrt_quat step = {0, 0.04361939f, 0, 0.99904822f}; // 5 degrees about +Y
		struct xrt_quat q = rel->pose.orientation;
		math_quat_rotate(&step, &q, &rel->pose.orientation);
	} else if (phase == 3) {
		rel->angular_velocity.y += 0.52359878f; // 30 deg/s about +Y
		rel->relation_flags |= XRT_SPACE_RELATION_ANGULAR_VELOCITY_VALID_BIT;
	}
}

static xrt_result_t
get_head_pose(struct xrt_device *xdev,
              enum xrt_input_name name,
              int64_t at_timestamp_ns,
              struct xrt_space_relation *out_relation)
{
	struct pico4_hmd *h = pico4_hmd(xdev);

	if (name != XRT_INPUT_GENERIC_HEAD_POSE) {
		U_LOG_XDEV_UNSUPPORTED_INPUT(&h->base, h->log_level, name);
		return XRT_ERROR_INPUT_UNSUPPORTED;
	}

	if (h->use_track) {
		// Interpolated between IMU steps, or predicted from the newest one.
		int64_t ahead = debug_get_num_option_pico4_predict_ms() * U_TIME_1MS_IN_NS + display_lead_ns(h);
		m_relation_history_get(h->relation_hist, at_timestamp_ns + ahead, out_relation);
		latency_probe(h, at_timestamp_ns, out_relation);
		log_pose(h, at_timestamp_ns, out_relation);
		return XRT_SUCCESS;
	}

	struct xrt_space_relation rel = XRT_SPACE_RELATION_ZERO;
	struct xrt_space_relation imu = XRT_SPACE_RELATION_ZERO;
	m_relation_history_get(h->relation_hist, at_timestamp_ns, &imu);
	bool imu_valid = (imu.relation_flags & XRT_SPACE_RELATION_ORIENTATION_VALID_BIT) != 0;
	if (imu_valid) {
		math_quat_normalize(&imu.pose.orientation);
	}

	// PICO OS's tracker predicts like the fusion's track: with the display lead for applications.
	int64_t lead = pico4_slam_fuses_imu(h->slam)
	                   ? debug_get_num_option_pico4_predict_ms() * U_TIME_1MS_IN_NS + display_lead_ns(h)
	                   : 0;
	if (h->slam != NULL && pico4_slam_get_head_pose(h->slam, at_timestamp_ns + lead, &rel)) {
		h->slam_position = rel.pose.position;
		if (imu_valid) {
			struct xrt_quat inverse;
			math_quat_invert(&imu.pose.orientation, &inverse);
			math_quat_rotate(&rel.pose.orientation, &inverse, &h->slam_from_3dof);
			math_quat_normalize(&h->slam_from_3dof);
			h->have_slam_position = true;
		}
		*out_relation = rel;
		latency_probe(h, at_timestamp_ns, out_relation);
		log_pose(h, at_timestamp_ns, out_relation);
		return XRT_SUCCESS;
	}
	rel = imu;
	if (h->have_slam_position && imu_valid) {
		// SLAM lost or stalled: the 3DoF orientation in SLAM's frame, at SLAM's last position.
		math_quat_rotate(&h->slam_from_3dof, &imu.pose.orientation, &rel.pose.orientation);
		math_quat_normalize(&rel.pose.orientation);
		math_quat_rotate_vec3(&h->slam_from_3dof, &imu.angular_velocity, &rel.angular_velocity);
		rel.pose.position = h->slam_position;
		rel.relation_flags |= XRT_SPACE_RELATION_POSITION_VALID_BIT;
	}
	*out_relation = rel;
	return XRT_SUCCESS;
}

/*!
 * The head pose at an LED exposure for the controller tracker (its own thread): the HMD's track
 * without the display lead it adds for applications, or SLAM's pose, or the 3DoF orientation.
 */
static bool
ctrack_head(void *user, int64_t at_ns, struct xrt_pose *out)
{
	struct pico4_hmd *h = user;
	struct xrt_space_relation rel = XRT_SPACE_RELATION_ZERO;
	if (h->use_track || h->slam == NULL || !pico4_slam_get_head_pose(h->slam, at_ns, &rel)) {
		m_relation_history_get(h->relation_hist, at_ns, &rel);
	}
	if (!(rel.relation_flags & XRT_SPACE_RELATION_ORIENTATION_VALID_BIT)) {
		return false;
	}
	*out = rel.pose;
	math_quat_normalize(&out->orientation);
	return true;
}

//! The head pose; while the IMU stream is being reconnected, the last one, untracked.
static xrt_result_t
pico4_hmd_get_tracked_pose(struct xrt_device *xdev,
                           enum xrt_input_name name,
                           int64_t at_timestamp_ns,
                           struct xrt_space_relation *out_relation)
{
	xrt_result_t ret = get_head_pose(xdev, name, at_timestamp_ns, out_relation);
	if (ret == XRT_SUCCESS && __atomic_load_n(&pico4_hmd(xdev)->imu_stale, __ATOMIC_ACQUIRE)) {
		out_relation->relation_flags = (enum xrt_space_relation_flags)(
		    out_relation->relation_flags &
		    ~(XRT_SPACE_RELATION_ORIENTATION_TRACKED_BIT | XRT_SPACE_RELATION_POSITION_TRACKED_BIT));
	}
	return ret;
}

static xrt_result_t
pico4_hmd_get_visibility_mask(struct xrt_device *xdev,
                              enum xrt_visibility_mask_type type,
                              uint32_t view_index,
                              struct xrt_visibility_mask **out_mask)
{
	struct xrt_fov fov = xdev->hmd->distortion.fov[view_index];
	u_visibility_mask_get_default(type, &fov, out_mask);
	return XRT_SUCCESS;
}

static xrt_result_t
pico4_hmd_get_view_poses(struct xrt_device *xdev,
                         const struct xrt_vec3 *default_eye_relation,
                         int64_t at_timestamp_ns,
                         enum xrt_view_type view_type,
                         uint32_t view_count,
                         struct xrt_space_relation *out_head_relation,
                         struct xrt_fov *out_fovs,
                         struct xrt_pose *out_poses)
{
	struct pico4_hmd *h = pico4_hmd(xdev);

	// The lenses (and panels) sit where the motor put them.
	struct xrt_vec3 eye_relation = *default_eye_relation;
	double ipd_m;
	__atomic_load(&h->ipd_m, &ipd_m, __ATOMIC_RELAXED);
	if (ipd_m > 0) {
		eye_relation = (struct xrt_vec3){(float)ipd_m, 0, 0};
	}
	return u_device_get_view_poses(xdev, &eye_relation, at_timestamp_ns, view_type, view_count,
	                               out_head_relation, out_fovs, out_poses);
}

/*!
 * Panel position (u, v in [0, 1], v down) to texture coordinates per colour,
 * through the factory lens model. Past the lens circle the coordinates leave
 * the texture quickly, which the compositor samples as black.
 */
static xrt_result_t
pico4_hmd_compute_distortion(
    struct xrt_device *xdev, uint32_t view, float u, float v, struct xrt_uv_triplet *out_result)
{
	struct pico4_hmd *h = pico4_hmd(xdev);
	const struct xrt_fov *fov = &xdev->hmd->distortion.fov[view];
	double tan_left = tan(fov->angle_left), tan_right = tan(fov->angle_right);
	double tan_up = tan(fov->angle_up), tan_down = tan(fov->angle_down);
	double tans[3][2];

	pico4_lens_tan(&h->lens[view], &h->ca, (u - 0.5) * PICO4_LENS_EYE_M, (v - 0.5) * PICO4_LENS_EYE_M, tans);
	struct xrt_vec2 *channels[3] = {&out_result->r, &out_result->g, &out_result->b};
	for (int c = 0; c < 3; c++) {
		// Tan-angles are right and down here; texture v runs down from the top edge.
		channels[c]->x = (float)((tans[c][0] - tan_left) / (tan_right - tan_left));
		channels[c]->y = (float)((tans[c][1] + tan_up) / (tan_up - tan_down));
	}
	return XRT_SUCCESS;
}

bool
pico4_hmd_compute_inverse_distortion(
    struct xrt_device *xdev, uint32_t view, uint32_t channel, float u, float v, float *out_u, float *out_v)
{
	// Only for this driver's HMD: the inverse of pico4_hmd_compute_distortion.
	if (xdev == NULL || xdev->compute_distortion != pico4_hmd_compute_distortion || view > 1 || channel > 2) {
		return false;
	}
	struct pico4_hmd *h = pico4_hmd(xdev);
	const struct xrt_fov *fov = &xdev->hmd->distortion.fov[view];
	double tan_left = tan(fov->angle_left), tan_right = tan(fov->angle_right);
	double tan_up = tan(fov->angle_up), tan_down = tan(fov->angle_down);
	double tx = tan_left + u * (tan_right - tan_left);
	double ty = v * (tan_up - tan_down) - tan_up;
	double x, y;
	if (!pico4_lens_inverse(&h->lens[view], &h->ca, (int)channel, tx, ty, &x, &y)) {
		return false;
	}
	*out_u = (float)(x / PICO4_LENS_EYE_M + 0.5);
	*out_v = (float)(y / PICO4_LENS_EYE_M + 0.5);
	return true;
}

static char *
read_text(const char *dir, const char *name)
{
	char path[512];
	snprintf(path, sizeof(path), "%s/%s", dir, name);
	FILE *f = fopen(path, "rb");
	if (f == NULL) {
		return NULL;
	}
	char *text = NULL;
	if (fseek(f, 0, SEEK_END) == 0) {
		long size = ftell(f);
		rewind(f);
		text = size > 0 && size < (1 << 20) ? calloc(1, (size_t)size + 1) : NULL;
		if (text != NULL && fread(text, 1, (size_t)size, f) != (size_t)size) {
			free(text);
			text = NULL;
		}
	}
	fclose(f);
	return text;
}

//! Factory lens data (pvr/OpticalCalibrationData.json) and the stock chromatic aberration terms.
static void
load_lens(struct pico4_hmd *h)
{
	const char *dir = debug_get_option_pico4_calibration();
	char *optical = read_text(dir, "pvr/OpticalCalibrationData.json");
	char *config = read_text(dir, "pxrconfig/SDK_Lens_ConfigFile.txt");

	h->has_lens = optical != NULL && pico4_lens_parse_optical(optical, h->lens);
	h->ca = (struct pico4_lens_ca){{0}};
	if (!h->has_lens) {
		P4_ERROR(h, "No lens data in %s: no distortion correction, provisional 104 degree field of view", dir);
	} else if (!debug_get_bool_option_pico4_lens_ca()) {
		P4_INFO(h, "Chromatic aberration correction off (PICO4_LENS_CA=1 turns it on)");
	} else if (config == NULL || !pico4_lens_parse_ca(config, &h->ca)) {
		h->ca = (struct pico4_lens_ca){{0}};
		P4_INFO(h, "No chromatic aberration terms in %s, correcting geometry only", dir);
	}
	free(optical);
	free(config);
}

//! Display description: two 2160x2160 panels side by side.
static bool
setup_display(struct pico4_hmd *h)
{
	const int panel = 2160;
	// Without the factory lens data: approximately the stock runtime's symmetric field of view.
	const double fov = 104.0 * (M_PI / 180.0);

	h->base.hmd->blend_modes[0] = XRT_BLEND_MODE_OPAQUE;
	h->base.hmd->blend_mode_count = 1;
	h->base.hmd->screens[0].w_pixels = panel * 2;
	h->base.hmd->screens[0].h_pixels = panel;
	h->base.hmd->screens[0].nominal_frame_interval_ns = time_s_to_ns(1.0f / 90.0f);
	for (int eye = 0; eye < 2; eye++) {
		h->base.hmd->views[eye].display.w_pixels = panel;
		h->base.hmd->views[eye].display.h_pixels = panel;
		h->base.hmd->views[eye].viewport.x_pixels = eye * panel;
		h->base.hmd->views[eye].viewport.y_pixels = 0;
		h->base.hmd->views[eye].viewport.w_pixels = panel;
		h->base.hmd->views[eye].viewport.h_pixels = panel;
		h->base.hmd->views[eye].rot = u_device_rotation_ident;
		struct xrt_fov *f = &h->base.hmd->distortion.fov[eye];
		if (h->has_lens) {
			double left, right, up, down;
			pico4_lens_fov(&h->lens[eye], &left, &right, &up, &down);
			*f = (struct xrt_fov){(float)left, (float)right, (float)up, (float)down};
			P4_INFO(h, "Eye %d: field of view left %.2f right %.2f up %.2f down %.2f degrees", eye,
			        left * 180 / M_PI, right * 180 / M_PI, up * 180 / M_PI, down * 180 / M_PI);
		} else if (!math_compute_fovs(1, 0.5, fov, 1, 0.5, fov, f)) {
			return false;
		}
	}
	if (h->has_lens) {
		h->base.compute_distortion = pico4_hmd_compute_distortion;
		u_distortion_mesh_fill_in_compute(&h->base);
	} else {
		u_distortion_mesh_set_none(&h->base);
	}
	return true;
}

static struct xrt_device *
pico4_hmd_create(void)
{
	enum u_device_alloc_flags flags =
	    (enum u_device_alloc_flags)(U_DEVICE_ALLOC_HMD | U_DEVICE_ALLOC_TRACKING_NONE);
	struct pico4_hmd *h = U_DEVICE_ALLOCATE(struct pico4_hmd, flags, 1, 0);
	h->log_level = debug_get_log_option_pico4_log();
	h->temperature_c = NAN;
	h->worn = -1;

	h->base.update_inputs = u_device_noop_update_inputs;
	h->base.get_tracked_pose = pico4_hmd_get_tracked_pose;
	h->base.get_view_poses = pico4_hmd_get_view_poses;
	h->base.get_visibility_mask = pico4_hmd_get_visibility_mask;
	h->base.destroy = pico4_hmd_destroy;
	h->base.get_battery_status = pico4_hmd_get_battery_status;
	h->base.name = XRT_DEVICE_GENERIC_HMD;
	h->base.device_type = XRT_DEVICE_TYPE_HMD;
	h->base.inputs[0].name = XRT_INPUT_GENERIC_HEAD_POSE;
	h->base.supported.orientation_tracking = true;
	h->base.supported.position_tracking = false;
	snprintf(h->base.str, XRT_DEVICE_NAME_LEN, "PICO 4");
	snprintf(h->base.serial, XRT_DEVICE_NAME_LEN, "PICO 4 (SSC IMU)");

	m_relation_history_create(&h->relation_hist);
	m_imu_3dof_init(&h->fusion, M_IMU_3DOF_USE_GRAVITY_DUR_20MS);
	load_lens(h);
	int ipd_ret = pico4_ipd_read(PICO4_IPD_DEVICE, &h->ipd_m);
	if (ipd_ret < 0) {
		h->ipd_m = 0;
		P4_INFO(h, "No IPD from the MCU (%s), using the runtime default", strerror(-ipd_ret));
	} else {
		P4_INFO(h, "IPD %.2f mm", h->ipd_m * 1000);
		if (os_thread_helper_init(&h->ipd_oth) != 0 || os_thread_helper_start(&h->ipd_oth, ipd_thread, h) != 0) {
			P4_WARN(h, "The IPD stays as it is for this session");
		}
	}
	set_backlight(h, true); // A session that ended in standby without closing.
	if (os_thread_helper_init(&h->oth) != 0 || !setup_display(h)) {
		P4_ERROR(h, "Failed to set up the device");
		goto fail;
	}

	int ret = pico4_ssc_open(&h->ssc, 3000);
	if (ret < 0) {
		P4_ERROR(h, "No SNS client service (is the SLPI up and pico4-sensorspd running?): %s", strerror(-ret));
		goto fail;
	}
	// Shortly after the DSP boots, lookups can come back empty for several seconds.
	bool found = false;
	for (int attempt = 0; attempt < PICO4_LOOKUP_ATTEMPTS && !found; attempt++) {
		found = pico4_ssc_find(h->ssc, "accel", &h->accel_suid, 3000) == 0 &&
		        pico4_ssc_find(h->ssc, "gyro", &h->gyro_suid, 3000) == 0;
	}
	if (!found) {
		P4_ERROR(h, "The sensor service reports no accelerometer/gyroscope");
		goto fail;
	}
	if (!calibrate_clock(h)) {
		P4_ERROR(h, "Cannot map sensor timestamps to CLOCK_MONOTONIC");
		goto fail;
	}
	// Looked up before the IMU thread reads the client; enabled with the other streams.
	h->has_temperature = pico4_ssc_find(h->ssc, "sensor_temperature", &h->temperature_suid, 3000) == 0;
	h->has_proximity = pico4_ssc_find(h->ssc, "proximity", &h->proximity_suid, 3000) == 0;

	// The controllers' packets come through the same service, when the radio is up.
	setup_station(h);
	if (h->station != NULL && pico4_ssc_find(h->ssc, "ble_mcu", &h->blemcu_suid, 3000) == 0) {
		h->has_blemcu = true;
		// The station thread hands each controller's unit data to it once it exists.
		__atomic_store_n(&h->ctrack,
		                 pico4_ctrack_create(debug_get_option_pico4_calibration(), ctrack_head, h, h->log_level),
		                 __ATOMIC_RELEASE);
		h->controllers[0] = pico4_controller_create(PICO4_BLEMCU_LEFT, &h->base, h->ctrack, h->station, h->log_level);
		h->controllers[1] = pico4_controller_create(PICO4_BLEMCU_RIGHT, &h->base, h->ctrack, h->station, h->log_level);
	}

	/*
	 * The sensor DSP runs the head IMU at the next supported output data rate above
	 * the request: 500 Hz at 512 Hz, 512 or 1000 Hz at 1024 Hz (docs/xr.md).
	 */
	float rate = (float)debug_get_num_option_pico4_imu_rate();
	double delivered_hz = rate > 500.0f ? 1024.0 : 512.0;

	// Before the IMU thread, which feeds it.
	h->slam =
	    pico4_slam_create(debug_get_option_pico4_calibration(), h->station, delivered_hz, h->ctrack, h->log_level);
	h->base.supported.position_tracking = h->slam != NULL;
	if (h->slam != NULL) {
		snprintf(h->base.serial, XRT_DEVICE_NAME_LEN, "PICO 4 (SLAM)");
		if (pico4_slam_fuses_imu(h->slam)) {
			P4_INFO(h, "6DoF from PICO OS's tracker, which fuses the IMU itself");
			const char *dir = debug_get_option_pico4_fusion_csv();
			if (dir != NULL) {
				char path[512];
				snprintf(path, sizeof(path), "%s/fusion_pose.csv", dir);
				h->pose_csv = fopen(path, "w");
				if (h->pose_csv != NULL) {
					setvbuf(h->pose_csv, NULL, _IOLBF, 0);
					fprintf(h->pose_csv, "#at_ns,now_ns,x,y,z,qx,qy,qz,qw,flags,vx,vy,vz,wx,wy,wz\n");
				}
			}
		} else {
			setup_track(h);
		}
	}

	if (pico4_ssc_enable(h->ssc, &h->accel_suid, rate) < 0 || pico4_ssc_enable(h->ssc, &h->gyro_suid, rate) < 0 ||
	    os_thread_helper_start(&h->oth, imu_thread, h) != 0) {
		P4_ERROR(h, "Failed to start the IMU stream");
		goto fail;
	}
	if (h->has_blemcu && pico4_ssc_enable(h->ssc, &h->blemcu_suid, rate) < 0) {
		P4_ERROR(h, "Failed to start the controller stream");
	}
	if (h->has_temperature && pico4_ssc_enable(h->ssc, &h->temperature_suid, PICO4_TEMPERATURE_HZ) < 0) {
		h->has_temperature = false;
	}
	if (!h->has_temperature) {
		P4_WARN(h, "No IMU temperature from the sensor service");
	}
	if (h->has_proximity && pico4_ssc_enable_on_change(h->ssc, &h->proximity_suid) < 0) {
		h->has_proximity = false;
	}
	if (!h->has_proximity) {
		P4_WARN(h, "No wear sensor from the sensor service");
	}
	P4_INFO(h, "IMU streaming at %.0f Hz requested, QTimer offset %lld ns", rate,
	        (long long)h->qtimer_offset_ns);

	u_var_add_root(h, "PICO 4", true);
	u_var_add_log_level(h, &h->log_level, "log_level");
	u_var_add_ro_u64(h, &h->gyro_samples, "gyro_samples");
	u_var_add_ro_u64(h, &h->accel_samples, "accel_samples");
	u_var_add_ro_u64(h, &h->blemcu_packets, "controller_packets");
	u_var_add_ro_u64(h, &h->blemcu_future, "controller_samples_from_the_future");
	u_var_add_ro_i64_ns(h, &h->last_latency_ns, "last_latency");
	m_imu_3dof_add_vars(&h->fusion, h, "");
	if (h->use_track) {
		u_var_add_ro_u64(h, &h->track.corrected, "slam_corrections");
		u_var_add_ro_u64(h, &h->track.rejected, "slam_outliers");
		u_var_add_ro_u64(h, &h->track.resets, "slam_resets");
		u_var_add_ro_u64(h, &h->track.anchors, "slam_anchors");
		u_var_add_ro_u64(h, &h->track.runaways, "slam_runaways");
		u_var_add_ro_f32(h, &h->track.last_error_m, "slam_error_m");
	}
	return &h->base;

fail:
	pico4_hmd_destroy(&h->base);
	return NULL;
}


/*
 *
 * Auto-prober.
 *
 */

//! A headset with a sensor DSP: a remoteproc named "slpi".
static bool
pico4_has_sensor_dsp(void)
{
	DIR *dir = opendir("/sys/class/remoteproc");
	if (dir == NULL) {
		return false;
	}
	bool found = false;
	struct dirent *e;
	while (!found && (e = readdir(dir)) != NULL) {
		char path[300], name[16] = {0};
		snprintf(path, sizeof(path), "/sys/class/remoteproc/%s/name", e->d_name);
		FILE *f = fopen(path, "r");
		if (f == NULL) {
			continue;
		}
		found = fgets(name, sizeof(name), f) != NULL && strcmp(name, "slpi\n") == 0;
		fclose(f);
	}
	closedir(dir);
	return found;
}

static int
pico4_autoprobe(struct xrt_auto_prober *xap,
                cJSON *attached_data,
                bool no_hmds,
                struct xrt_prober *xp,
                struct xrt_device **out_xdevs)
{
	// Only on a headset with a sensor DSP; the device create checks the service.
	if (no_hmds || !pico4_has_sensor_dsp()) {
		return 0;
	}
	struct xrt_device *xdev = pico4_hmd_create();
	if (xdev == NULL) {
		return 0;
	}
	struct pico4_hmd *h = pico4_hmd(xdev);
	int count = 0;
	out_xdevs[count++] = xdev;
	for (int i = 0; i < 2; i++) {
		if (h->controllers[i] != NULL) {
			out_xdevs[count++] = pico4_controller_xdev(h->controllers[i]);
		}
	}
	return count;
}

static void
pico4_auto_prober_destroy(struct xrt_auto_prober *xap)
{
	free(xap);
}

struct xrt_auto_prober *
pico4_create_auto_prober(void)
{
	struct xrt_auto_prober *xap = U_TYPED_CALLOC(struct xrt_auto_prober);
	xap->name = "PICO 4";
	xap->destroy = pico4_auto_prober_destroy;
	xap->lelo_dallas_autoprobe = pico4_autoprobe;
	return xap;
}

bool
pico4_hmd_get_presence(struct xrt_device *xdev, bool *out_worn)
{
	// Only our HMD has the wear sensor; other devices of the same process answer false.
	if (xdev == NULL || xdev->get_tracked_pose != pico4_hmd_get_tracked_pose) {
		return false;
	}
	int worn = __atomic_load_n(&pico4_hmd(xdev)->worn, __ATOMIC_RELAXED);
	if (worn < 0) {
		return false;
	}
	*out_worn = worn != 0;
	return true;
}

/*!
 * The head at the middle of the exposure @p timestamp_ns, in the space driver_monado reports to
 * SteamVR: the tracking origin's offset (Monado's 1.6 m) applied, as on every reported pose
 * (world from driver is the identity). Without it the 3D room view placed each frame 1.6 m below
 * the head and the image stretched towards the top and bottom; the 2D view only uses rotation.
 * Without the prediction the reported poses carry: with it the compositor placed each camera
 * frame as if taken that much later, and the room trailed the head's turns.
 */
static bool
roomview_pose(void *ctx, int64_t timestamp_ns, double out[12])
{
	struct xrt_device *xdev = ctx;
	struct xrt_pose tracked;
	if (!ctrack_head(pico4_hmd(xdev), timestamp_ns, &tracked)) {
		return false;
	}
	struct xrt_pose pose = tracked;
	if (xdev->tracking_origin != NULL) {
		math_pose_transform(&xdev->tracking_origin->initial_offset, &tracked, &pose);
		math_quat_normalize(&pose.orientation);
	}
	const struct xrt_quat q = pose.orientation; // normalized by ctrack_head
	const double x = q.x, y = q.y, z = q.z, w = q.w;
	const double m[12] = {
	    1 - 2 * (y * y + z * z), 2 * (x * y - z * w),     2 * (x * z + y * w),     pose.position.x,
	    2 * (x * y + z * w),     1 - 2 * (x * x + z * z), 2 * (y * z - x * w),     pose.position.y,
	    2 * (x * z - y * w),     2 * (y * z + x * w),     1 - 2 * (x * x + y * y), pose.position.z,
	};
	memcpy(out, m, sizeof(m));
	return true;
}

bool
pico4_hmd_camera_info(struct xrt_device *xdev, struct pico4_passthrough_info *out)
{
	if (xdev == NULL || xdev->get_tracked_pose != pico4_hmd_get_tracked_pose) {
		return false;
	}
	struct pico4_passthrough *p = pico4_slam_passthrough(pico4_hmd(xdev)->slam);
	if (p == NULL) {
		return false;
	}
	pico4_passthrough_info(p, out);
	// SteamVR's camera component asks: its room view takes frames from pico4-roomview, which
	// gets them from here (pico4_roomview.h).
	if (!pico4_passthrough_export(p, PICO4_ROOMVIEW_PATH, roomview_pose, xdev)) {
		P4_WARN(pico4_hmd(xdev), "No room view: %s not shared (%s)", PICO4_ROOMVIEW_PATH, strerror(errno));
	}
	return true;
}

void
pico4_hmd_camera_stream(struct xrt_device *xdev, bool on)
{
	if (xdev == NULL || xdev->get_tracked_pose != pico4_hmd_get_tracked_pose) {
		return;
	}
	struct pico4_passthrough *p = pico4_slam_passthrough(pico4_hmd(xdev)->slam);
	if (p != NULL) {
		pico4_passthrough_set_streaming(p, on);
		P4_INFO(pico4_hmd(xdev), "Passthrough cameras %s", on ? "streaming" : "stopped");
	}
}

bool
pico4_hmd_camera_frame(struct xrt_device *xdev,
                       uint64_t after,
                       int timeout_ms,
                       uint8_t *y,
                       uint32_t stride,
                       struct pico4_passthrough_frame *out)
{
	if (xdev == NULL || xdev->get_tracked_pose != pico4_hmd_get_tracked_pose) {
		return false;
	}
	struct pico4_passthrough *p = pico4_slam_passthrough(pico4_hmd(xdev)->slam);
	return p != NULL && pico4_passthrough_wait(p, after, timeout_ms, y, stride, out);
}

bool
pico4_hmd_depth_mesh(
    struct xrt_device *xdev, uint64_t after, float *vertices, uint32_t *indices, struct pico4_depth_mesh_info *out)
{
	if (xdev == NULL || xdev->get_tracked_pose != pico4_hmd_get_tracked_pose ||
	    !pico4_slam_depth_mesh(pico4_hmd(xdev)->slam, after, vertices, indices, out)) {
		return false;
	}
	// The vertices are in the head pose's world: SteamVR's raw tracking space is that world with
	// the tracking origin's offset applied, as on every reported pose (roomview_pose).
	struct xrt_pose offset = XRT_POSE_IDENTITY;
	if (xdev->tracking_origin != NULL) {
		offset = xdev->tracking_origin->initial_offset;
		math_quat_normalize(&offset.orientation);
	}
	const double x = offset.orientation.x, y = offset.orientation.y, z = offset.orientation.z,
	             w = offset.orientation.w;
	const double m[12] = {
	    1 - 2 * (y * y + z * z), 2 * (x * y - z * w),     2 * (x * z + y * w),     offset.position.x,
	    2 * (x * y + z * w),     1 - 2 * (x * x + z * z), 2 * (y * z - x * w),     offset.position.y,
	    2 * (x * z - y * w),     2 * (y * z + x * w),     1 - 2 * (x * x + y * y), offset.position.z,
	};
	memcpy(out->tracking_from_mesh, m, sizeof(m));
	return true;
}

bool
pico4_hmd_floor(struct xrt_device *xdev, float *out_y)
{
	float y;
	if (xdev == NULL || xdev->get_tracked_pose != pico4_hmd_get_tracked_pose ||
	    !pico4_slam_floor(pico4_hmd(xdev)->slam, &y)) {
		return false;
	}
	// A point of the floor in SteamVR's raw tracking space: the tracking origin's offset applied,
	// as on every reported pose.
	const struct xrt_vec3 floor = {0, y, 0};
	struct xrt_vec3 point = floor;
	if (xdev->tracking_origin != NULL) {
		struct xrt_pose offset = xdev->tracking_origin->initial_offset;
		math_quat_normalize(&offset.orientation);
		math_pose_transform_point(&offset, &floor, &point);
	}
	*out_y = point.y;
	return true;
}

bool
pico4_hmd_ipd(struct xrt_device *xdev, float *out_ipd_m)
{
	if (xdev == NULL || xdev->get_tracked_pose != pico4_hmd_get_tracked_pose) {
		return false;
	}
	double ipd_m;
	__atomic_load(&pico4_hmd(xdev)->ipd_m, &ipd_m, __ATOMIC_RELAXED);
	*out_ipd_m = (float)ipd_m;
	return ipd_m > 0;
}

void
pico4_hmd_standby(struct xrt_device *xdev, bool standby)
{
	if (xdev != NULL && xdev->get_tracked_pose == pico4_hmd_get_tracked_pose) {
		set_backlight(pico4_hmd(xdev), !standby);
	}
}

void
pico4_hmd_detect_floor(struct xrt_device *xdev)
{
	if (xdev != NULL && xdev->get_tracked_pose == pico4_hmd_get_tracked_pose) {
		pico4_slam_detect_floor(pico4_hmd(xdev)->slam);
	}
}

bool
pico4_hmd_get_universe(struct xrt_device *xdev, uint64_t *out_universe)
{
	char map[40];
	if (xdev == NULL || xdev->get_tracked_pose != pico4_hmd_get_tracked_pose ||
	    !pico4_slam_room(pico4_hmd(xdev)->slam, map)) {
		return false;
	}
	// FNV-1a of the map's identity.
	uint64_t h = 14695981039346656037ull;
	for (const char *c = map; *c != '\0'; c++) {
		h = (h ^ (uint8_t)*c) * 1099511628211ull;
	}
	// Not 0 or 1, which vrserver gives other meanings, nor 424242, the Frame's universe without
	// a tracker map, which vrserver never saves.
	*out_universe = h > 1 && h != 424242 ? h : h + 2;
	return true;
}
