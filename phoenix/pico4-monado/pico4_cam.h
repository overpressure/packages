// Copyright 2026, overpressure contributors.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Capture from the four OV7251 tracking cameras and the IMX471 RGB camera through V4L2.
 *
 * One pipeline per camera through CAMSS, with raw sensor data (no pixel pipeline, no firmware).
 * Sensor slots 0-3 are the OV7251s (left, right, "slave", "master" modules), slot 4 the IMX471.
 *
 * Plain C11 on Linux headers. Not thread-safe: one thread owns a pico4_cam. Functions return 0
 * or a negative errno.
 *
 * @ingroup drv_pico4
 */

#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

#ifdef __cplusplus
extern "C" {
#endif

#define PICO4_CAM_NUM 4      //!< OV7251 tracking cameras, sensor slots 0-3.
#define PICO4_CAM_WIDTH 640  //!< Image width in pixels.
#define PICO4_CAM_HEIGHT 480 //!< Scene rows reported to callers.
/*!
 * The IMX471 RGB camera, sensor slot 4: 2328x1748 RAW10 (RGGB, the 2x2-binned array),
 * free-running at 50 fps on four lanes, with no metadata line. It streams through the same
 * pico4_cam as the tracking cameras.
 */
#define PICO4_CAM_RGB_SLOT 4
#define PICO4_CAM_RGB_WIDTH 2328
#define PICO4_CAM_RGB_HEIGHT 1748
#define PICO4_CAM_RGB_BLACK 64     //!< Black level of the 10-bit samples.
#define PICO4_CAM_RGB_LINE_NS 4425 //!< One line of the 50 fps mode (frame length 4520).
#define PICO4_CAM_RGB_MAX_LINES 4510 //!< Longest exposure in lines: frame length less 10.
#define PICO4_CAM_SLOTS 5          //!< Sensor slots this module drives: the OV7251s and the IMX471.
#define PICO4_CAM_RING_MIN 4
#define PICO4_CAM_RING_MAX 8
enum pico4_cam_log_level
{
	PICO4_CAM_LOG_ERROR = 0,
	PICO4_CAM_LOG_INFO = 1,
};

typedef void (*pico4_cam_log_fn)(void *user, enum pico4_cam_log_level level, const char *msg);

struct pico4_cam_config
{
	unsigned slots;               //!< Bitmask of sensor slots 0-4 (4: the RGB camera).
	unsigned ring;                //!< Buffers per camera, 4-8.
	unsigned rgb_exposure_lines;  //!< The RGB camera's exposure (4-PICO4_CAM_RGB_MAX_LINES), 0: 1/100 s.
	unsigned rgb_gain;            //!< Its analog gain, 16 = 1x (16-256), 0: 1x.
	bool hold_frames;             //!< Frames stay owned until pico4_cam_release(); else the next call releases.
	int verbose;                  //!< Highest pico4_cam_log_level that is logged.
	pico4_cam_log_fn log;         //!< NULL logs to stderr.
	void *log_user;
};

struct pico4_cam_frame
{
	int slot;                  //!< Sensor slot 0-4.
	uint64_t frame_id;         //!< The stream's frame counter: the buffer's V4L2 sequence number.
	int64_t sof_boottime_ns;   //!< CLOCK_BOOTTIME at the hardware's start of frame.
	int64_t sof_monotonic_ns;  //!< The same instant on CLOCK_MONOTONIC.
	int64_t sof_qtimer_ns;     //!< The same instant in QTimer ns (the IMU's clock).
	uint32_t width;            //!< 640 (the RGB camera: 2328).
	uint32_t height;           //!< Scene rows from data on (480, fewer if the buffer is shorter; RGB: 1748).
	uint32_t lines;            //!< Lines in the buffer, the metadata line included (481; RGB: 1748).
	uint32_t stride;           //!< Bytes per line. RAW10, LSB-aligned in 16 bits.
	const void *meta;          //!< The buffer's first line: the OV7251's metadata (pico4_cam_meta_decode); RGB: the image.
	const void *data;          //!< First scene row, after the metadata line; valid until the frame is released.
	size_t size;               //!< stride * lines, from meta.
	unsigned buffer;           //!< Ring index (internal, used by pico4_cam_release()).
};

/*!
 * What the sensor applied to a frame, from its metadata line: OV7251 register
 * values carried as 10-bit samples (shifted up by two bits). PICO OS reads the
 * same words. The exposure phase follows the station's schedule.
 */
struct pico4_cam_meta
{
	unsigned gain;           //!< Analog gain, 16 = 1x.
	unsigned exposure_lines; //!< Exposure in sensor lines.
	unsigned phase;          //!< 1: scene (head tracking), 2: controller LEDs, 4: other.
	unsigned tag;            //!< The phase register's upper bits; their meaning is not known.
};

#define PICO4_CAM_PHASE_SCENE 1u
#define PICO4_CAM_PHASE_LEDS 2u

//! Decodes a metadata line of @p bytes; false if it is too short or holds no known phase.
static inline bool
pico4_cam_meta_decode(const void *line, size_t bytes, struct pico4_cam_meta *out)
{
	const uint8_t *b = (const uint8_t *)line;
	if (line == NULL || bytes < 83 * 2) {
		return false;
	}
#define PICO4_CAM_META_WORD(i) ((unsigned)b[2 * (i)] | ((unsigned)b[2 * (i) + 1] << 8))
	unsigned gain = PICO4_CAM_META_WORD(4), exp_hi = PICO4_CAM_META_WORD(6), exp_lo = PICO4_CAM_META_WORD(7);
	unsigned phase = PICO4_CAM_META_WORD(82);
#undef PICO4_CAM_META_WORD
	if ((gain | exp_hi | exp_lo | phase) > 0x3ff) {
		return false; // not 10-bit samples
	}
	out->gain = gain >> 2;
	out->exposure_lines = ((exp_hi << 6) & 0xf00) | ((exp_lo >> 2) & 0xff);
	out->phase = (phase >> 2) & 0xf;
	out->tag = (phase >> 2) >> 4;
	return out->phase == 1 || out->phase == 2 || out->phase == 4;
}

/*!
 * The median of @p n signals (n <= 8): what the cameras of one set see as a group. One
 * covered or saturated camera does not move it, unlike the mean or the first camera alone.
 */
static inline float
pico4_cam_median(const float *v, int n)
{
	float s[8];
	if (n <= 0) {
		return 0.0f;
	}
	n = n > 8 ? 8 : n;
	for (int i = 0; i < n; i++) {
		int j = i;
		for (; j > 0 && s[j - 1] > v[i]; j--) {
			s[j] = s[j - 1];
		}
		s[j] = v[i];
	}
	return n % 2 ? s[n / 2] : 0.5f * (s[n / 2 - 1] + s[n / 2]);
}

/*!
 * Assembly of one set of scene frames, one per camera, by start of frame: all sensors start
 * on the same FSIN pulse, so a set's frames start within @p window_ns of its first one. A
 * frame after that window starts the next set and drops the incomplete one (a camera missed
 * the pulse); one before it belongs to a set already given up; a second frame of a camera
 * in the same set is a duplicate.
 */
struct pico4_cam_assembly
{
	int64_t ts;   //!< Start of frame of the set's first frame.
	int have;     //!< Frames in the set.
	bool slot[8]; //!< Which cameras the set has.
};

enum pico4_cam_assembly_result
{
	PICO4_CAM_SET_ADDED,     //!< In the set, which still misses frames.
	PICO4_CAM_SET_COMPLETE,  //!< In the set, which now has all @p count frames.
	PICO4_CAM_SET_LATE,      //!< From a set already given up: skip it.
	PICO4_CAM_SET_DUPLICATE, //!< The camera is in the set already: skip it.
};

static inline void
pico4_cam_assembly_reset(struct pico4_cam_assembly *a)
{
	a->have = 0;
	for (int i = 0; i < 8; i++) {
		a->slot[i] = false;
	}
}

/*!
 * Add camera @p index's frame (start of frame @p ts) to the set of @p count cameras.
 * @p out_dropped is set when the frame dropped an incomplete set before it.
 */
static inline enum pico4_cam_assembly_result
pico4_cam_assembly_add(
    struct pico4_cam_assembly *a, int index, int64_t ts, int64_t window_ns, int count, bool *out_dropped)
{
	*out_dropped = false;
	if (a->have != 0 && ts > a->ts + window_ns) {
		*out_dropped = true;
		pico4_cam_assembly_reset(a);
	}
	if (a->have != 0 && ts < a->ts - window_ns) {
		return PICO4_CAM_SET_LATE;
	}
	if (a->have == 0) {
		a->ts = ts;
	}
	if (a->slot[index]) {
		return PICO4_CAM_SET_DUPLICATE;
	}
	a->slot[index] = true;
	a->have++;
	return a->have == count ? PICO4_CAM_SET_COMPLETE : PICO4_CAM_SET_ADDED;
}

/*!
 * Frame identity across one set of scene frames. All sensors start their frames on the
 * station's FSIN pulse, so each camera's frame counter keeps a fixed offset from the
 * first camera's; a frame at another offset is from another pulse. An offset that changes
 * and then holds for PICO4_CAM_OFFSET_ADOPT sets is taken as the new one (a counter that
 * skipped).
 */
#define PICO4_CAM_OFFSET_ADOPT 2

struct pico4_cam_set_check
{
	bool have;
	int64_t offset[8];  //!< frame_id[i] - frame_id[0] of the accepted sets.
	int64_t pending[8]; //!< A different offset seen in the last sets...
	int pending_sets;   //!< ...this many in a row.
	//! The cameras are exposed each on its own (PICO OS's auto exposure): only the counters count.
	bool per_camera_exposure;
};

/*!
 * Whether @p n frames (n <= 8) form one exposure: the same exposure and analog gain in
 * each frame's metadata, and every camera's frame counter at its usual offset. False
 * with the reason in @p why otherwise; also @p why, with true, when a new offset is adopted.
 */
static inline bool
pico4_cam_set_consistent(struct pico4_cam_set_check *c,
                         const uint64_t *frame_id,
                         const struct pico4_cam_meta *meta,
                         int n,
                         char *why,
                         size_t why_len)
{
	why[0] = '\0';
	n = n > 8 ? 8 : n;
	for (int i = 1; i < n && !c->per_camera_exposure; i++) {
		if (meta[i].exposure_lines != meta[0].exposure_lines || meta[i].gain != meta[0].gain) {
			snprintf(why, why_len, "camera %d exposed %u lines at gain %u, camera 0 %u lines at gain %u", i,
			         meta[i].exposure_lines, meta[i].gain, meta[0].exposure_lines, meta[0].gain);
			return false;
		}
	}
	int64_t offset[8];
	int moved = -1;
	for (int i = 0; i < n; i++) {
		offset[i] = (int64_t)(frame_id[i] - frame_id[0]);
		if (c->have && offset[i] != c->offset[i] && moved < 0) {
			moved = i;
		}
	}
	if (!c->have || moved < 0) {
		for (int i = 0; i < n; i++) {
			c->offset[i] = offset[i];
		}
		c->have = true;
		c->pending_sets = 0;
		return true;
	}
	bool same = c->pending_sets > 0;
	for (int i = 0; same && i < n; i++) {
		same = c->pending[i] == offset[i];
	}
	c->pending_sets = same ? c->pending_sets + 1 : 1;
	for (int i = 0; i < n; i++) {
		c->pending[i] = offset[i];
	}
	if (c->pending_sets >= PICO4_CAM_OFFSET_ADOPT) {
		snprintf(why, why_len, "camera %d frame counter moved from offset %lld to %lld, adopted", moved,
		         (long long)c->offset[moved], (long long)offset[moved]);
		for (int i = 0; i < n; i++) {
			c->offset[i] = offset[i];
		}
		c->pending_sets = 0;
		return true;
	}
	snprintf(why, why_len, "camera %d frame %llu is at offset %lld from camera 0, not %lld", moved,
	         (unsigned long long)frame_id[moved], (long long)offset[moved], (long long)c->offset[moved]);
	return false;
}

//! Per-slot state for diagnostics.
struct pico4_cam_slot_info
{
	int phy;            //!< CSIPHY index.
	unsigned lane_assign;
	uint64_t errors;
};

struct pico4_cam;

//! Defaults: all four slots, ring of 8.
void
pico4_cam_config_init(struct pico4_cam_config *cfg);

/*!
 * Sets up one V4L2 pipeline per selected camera through CAMSS, with its buffers. The sensors
 * power up and streaming starts with pico4_cam_start().
 */
int
pico4_cam_open(struct pico4_cam **out, const struct pico4_cam_config *cfg);

//! Stream on for every selected camera.
int
pico4_cam_start(struct pico4_cam *cam);

/*!
 * Waits up to @p timeout_ms (negative: forever) for the next frame from any
 * camera. Returns 0, -ETIMEDOUT, -EINTR (signal) or another negative errno.
 */
int
pico4_cam_next(struct pico4_cam *cam, struct pico4_cam_frame *out, int timeout_ms);

//! Gives a frame's buffer back to its camera (only needed with hold_frames).
int
pico4_cam_release(struct pico4_cam *cam, const struct pico4_cam_frame *frame);

//! Stream off. Safe to call twice.
int
pico4_cam_stop(struct pico4_cam *cam);

//! Stops if needed, releases every buffer, closes all nodes.
void
pico4_cam_close(struct pico4_cam *cam);

//! Copies the per-slot state of @p slot (0-4).
int
pico4_cam_get_slot_info(const struct pico4_cam *cam, int slot, struct pico4_cam_slot_info *out);

/*!
 * The RGB camera's exposure while it streams: @p lines (4-PICO4_CAM_RGB_MAX_LINES, of
 * PICO4_CAM_RGB_LINE_NS each) and analog @p gain (16 = 1x, up to 256 = 16x). The IMX471 runs
 * free and owns its exposure, unlike the OV7251s, whose exposure the station sets.
 */
int
pico4_cam_set_rgb_exposure(struct pico4_cam *cam, unsigned lines, unsigned gain);

/*!
 * Pauses (or resumes) the RGB camera's stream at the sensor while the capture runs on, for
 * when nobody shows its frames. Its buffers stay queued and its pipeline stays up.
 */
int
pico4_cam_set_rgb_streaming(struct pico4_cam *cam, bool on);

/*!
 * CLOCK_BOOTTIME less CLOCK_MONOTONIC, ns: it grows by the time the system was suspended. The
 * QTimer, which stamps the IMU samples and the camera frames, runs on through a suspend while
 * CLOCK_MONOTONIC stands, so a QTimer-to-monotonic offset taken before it is that much off after.
 */
int64_t
pico4_slept_ns(void);

//! A growth of pico4_slept_ns() beyond this is a suspend (timesyncd's steering does not change it).
#define PICO4_SLEEP_DETECT_NS 10000000

#ifdef __cplusplus
}
#endif
