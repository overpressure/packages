// Copyright 2026, overpressure contributors.
// SPDX-License-Identifier: Unlicense
/*
 * pico4-slamd: PICO OS's head tracker (libSlam.so, its front end on the Compute DSP) for the
 * PICO 4 driver, in the tracker's bionic root (docs/xr.md#head-tracking).
 *
 *   pico4-slamd [GID]   (in the root, through /usr/libexec/pico4/stock-slam)
 *
 * Creates the shared memory of pico4_slam_feed.h (group GID, mode 0660) and waits for the
 * driver. Each driver generation gets its own tracker: when the driver starts again, the process
 * exits and systemd starts a fresh one. The tracker is fed as PICO OS's tracking service feeds
 * it, from two threads as there: every IMU sample {gyro, accel, temperature} with its time, and
 * the pose after it; each frame set, once the IMU is past it (the camera's latency does that on
 * PICO OS), as four 640x480 8-bit images after the stock image processing (isp.h), with the
 * earliest exposure middle for all four; then start6DOF. Times are CLOCK_MONOTONIC ns throughout.
 * The lens-shading table follows the camera modules' vendor, as on PICO OS (lsc_table);
 * PICO4_SLAMD_LSC names another.
 *
 * As PICO OS's tracking service, a tracker whose confidence fell from 1 to 0 and stays 0 for
 * more than 10 s (PICO4_SLAMD_RESTART_S; 0 turns it off) is stopped and started again in the
 * process: libSlam can stay in its relocalisation for good after a loss.
 */
#include "bionic.h"
#include "isp.h"
#include "room.h"
#include "pico4_slam_feed.h"
#include "vst.h"

#include <stdbool.h>

#define CONFIG_DIR "/system/etc/pvr/config/"
//! The camera modules' ids where PICO OS's station service keeps them (the data partition, stock-slam).
#define OTP_PATH "/data/stationservice/camera_otp.json"
//! A set goes in once the IMU is this far past it.
#define SET_DELAY_NS 30000000
//! Temperature for samples without one: the gyro bias table's middle.
#define DEFAULT_TEMPERATURE 30.0
//! The range libSlam accepts as a temperature (°C); outside it the tracker ignores the value.
#define TEMPERATURE_MIN -30.0
#define TEMPERATURE_MAX 100.0

struct vec
{
	void *begin, *end, *cap; // libc++ std::vector
};
typedef void *(*create_fn)(const char *);
typedef int (*set_fn)(void *, const char *, const char *);
typedef int (*start_fn)(void *, const char *);
typedef void (*image_fn)(void *, int, const struct vec *);
typedef void (*imu_fn)(void *, const struct vec *, int64_t);
typedef int (*result_fn)(void *, void *);
typedef void (*register_fn)(void (*)(void));
typedef int (*points_register_fn)(void (*)(const struct vst_slam_points *));
typedef void (*prestop_fn)(void *);
typedef int (*stop_fn)(void *, const char *);
typedef void (*destroy_fn)(void *);
typedef int (*relocinfo_fn)(void *, void *);

/*
 * The room (docs/xr.md#room-setup), where PICO OS's tracker keeps it: /data/misc/user/0, bound
 * from the data partition by stock-slam. The tracker saves its map at the stop only with a
 * boundary in stdata.txt (in this session's world), and after relocalising in a saved room it
 * writes that room's boundary in this session's world to stdataforalgorithm.txt. Every map here
 * is saved with its room's reference shape (room.h: its id in its sides) in the room's world: the
 * tracker's copy of it after a relocalisation then gives the room and the transform from this
 * session's world to the room's, whichever saved map it found and whatever it names it. The play
 * area itself is SteamVR's, kept per room's universe.
 */
#define BOUNDARY_DIR "/data/misc/user/0/boundary/"
#define SESSION_BOUNDARY BOUNDARY_DIR "stdata.txt"
#define RELOCALISED_BOUNDARY BOUNDARY_DIR "stdataforalgorithm.txt"
//! PICO OS's tracker tries to relocalise for 8 s; with no relocalisation this long after its first
//! frames (PICO4_SLAMD_RELOCALISE_S) the session is a new room until a later one.
#define RELOCALISE_DEFAULT_S 10
//! PICO OS's tracking service restarts a tracker lost for longer than this (restart_tracking_sec).
#define RESTART_DEFAULT_S 10

static volatile int g_stop;
static char g_text[4 << 20];
static long
read_text(const char *path);

static void
on_stop_signal(int sig)
{
	(void)sig;
	g_stop = 1;
}

static double g_seen[ROOM_MAX_POINTS][3];

//! Writes a boundary file (tracker-world points), atomically; 0 or -1.
static int
write_boundary(const char *path, const double (*pts)[3], int n)
{
	static char text[ROOM_MAX_POINTS * 64 + 64];
	int len = snprintf(text, sizeof(text), "isSaveMap 1\nvertexcount %d\n", 2 * n);
	for (int i = 0; i < n && len < (int)sizeof(text) - 64; i++) {
		double abc[3];
		room_to_file(pts[i], abc);
		len += snprintf(text + len, sizeof(text) - (size_t)len, "(%f, %f, %f)\n", abc[0], abc[1], abc[2]);
	}
	char tmp[128];
	snprintf(tmp, sizeof(tmp), "%s.new", path);
	int fd = open(tmp, O_WRONLY | O_CREAT | O_TRUNC | O_CLOEXEC, 0644);
	if (fd < 0) {
		return -1;
	}
	int ok = write(fd, text, (size_t)len) == len;
	close(fd);
	return ok && rename(tmp, path) == 0 ? 0 : -1;
}

//! A boundary file's points (tracker world); their count, or -1.
static int
read_boundary(const char *path, double (*out)[3])
{
	long len = read_text(path);
	return len > 0 ? room_parse(g_text, len, out, ROOM_MAX_POINTS) : -1;
}

/*
 * Rooms from before the ids (the plain square) keep the identity their universe came from, the
 * map name the tracker reported then: one line per room, its id in hex and that identity.
 */
#define ROOM_TABLE BOUNDARY_DIR "pico4_rooms.txt"

//! The identity of room @p id, which the driver's universe comes from: its table entry or "room-ID".
static void
room_identity(uint64_t id, char *out, size_t size)
{
	snprintf(out, size, "room-%09llx", (unsigned long long)id);
	long len = read_text(ROOM_TABLE);
	for (long i = 0; i < len;) {
		long end = i;
		while (end < len && g_text[end] != '\n') {
			end++;
		}
		uint64_t v = 0;
		long k = i;
		for (; k < end && g_text[k] != ' '; k++) {
			char c = g_text[k];
			v = v << 4 | (uint64_t)(c >= 'a' ? c - 'a' + 10 : c - '0');
		}
		if (v == id && k < end && end - k - 1 > 0 && (size_t)(end - k - 1) < size) {
			memcpy(out, g_text + k + 1, (size_t)(end - k - 1));
			out[end - k - 1] = '\0';
			return;
		}
		i = end + 1;
	}
}

//! Adds @p id's identity to the table (atomically, with the lines there).
static void
room_table_add(uint64_t id, const char *identity)
{
	long len = read_text(ROOM_TABLE);
	static char text[1 << 16];
	if (len < 0 || len > (long)sizeof(text) - 128) {
		len = 0;
	}
	memcpy(text, g_text, (size_t)len);
	len += snprintf(text + len, sizeof(text) - (size_t)len, "%09llx %s\n", (unsigned long long)id, identity);
	int fd = open(ROOM_TABLE ".new", O_WRONLY | O_CREAT | O_TRUNC | O_CLOEXEC, 0644);
	if (fd >= 0) {
		int ok = write(fd, text, (size_t)len) == len;
		close(fd);
		if (ok) {
			rename(ROOM_TABLE ".new", ROOM_TABLE);
		}
	}
}

//! A new room id: random, never the plain square's.
static uint64_t
new_room_id(void)
{
	uint64_t id = 0;
	int fd = open("/dev/urandom", O_RDONLY | O_CLOEXEC);
	if (fd >= 0) {
		if (read(fd, &id, sizeof(id)) != (ssize_t)sizeof(id)) {
			id = 0;
		}
		close(fd);
	}
	if (id == 0) {
		id = (uint64_t)now_ns(CLOCK_MONOTONIC) * 6364136223846793005ull;
	}
	id &= ROOM_ID_MASK;
	return id == ROOM_ID_SQUARE ? id ^ 1 : id;
}

/*!
 * The room after start6DOF, once PICO OS's tracker has had its time to relocalise: in a saved room
 * its id from the shape of the tracker's copy of its boundary, the transform to its world, and the
 * shape again in this session's world so the stop saves the map with it; else a new room with a
 * new id, whose map is saved with its shape as it is. The driver's universe comes from the room's
 * identity (room_identity), not from the tracker's map name, which a merge can change.
 */
static void
settle_room(struct pico4_slam_feed *feed, uint32_t generation, void *h, relocinfo_fn relocinfo)
{
	int seen_n = read_boundary(RELOCALISED_BOUNDARY, g_seen);
	uint8_t info[0x48];
	memset(info, 0, sizeof(info));
	int info_ret = relocinfo != 0 ? relocinfo(h, info) : -99;
	int32_t info_status;
	memcpy(&info_status, info + 0x1c, 4);
	char map[sizeof(feed->room_uuid)];
	memcpy(map, info + 0x20, sizeof(map) - 1);
	map[sizeof(map) - 1] = '\0';
	// The driver keeps the previous room (and universe) while the new one is written.
	__atomic_store_n(&feed->room_state, 0, __ATOMIC_RELEASE);
	fprintf(stderr, "pico4-slamd: map %s (getRelocMapInfo %d, status %d); relocalised boundary %d points\n", map,
	        info_ret, info_status, seen_n);
	uint32_t state = 2;
	uint64_t id = 0;
	double shape[4][3];
	if (seen_n == 4 && room_decode((const double(*)[3])g_seen, 4, &id, sqrt) == 0) {
		room_shape(id, shape);
		double yaw, t[3];
		double rms = room_fit((const double(*)[3])g_seen, (const double(*)[3])shape, 4, &yaw, t, atan2, sin, cos, sqrt);
		fprintf(stderr,
		        "pico4-slamd: relocalised in a saved room (id %09llx): turn %.2f deg, shift %.3f %.3f %.3f m, residual "
		        "%.4f m\n",
		        (unsigned long long)id, yaw * 57.29577951308232, t[0], t[1], t[2], rms);
		if (rms >= 0 && rms < 0.05) {
			feed->room_yaw = yaw;
			memcpy(feed->room_t, t, sizeof(t));
			state = 1;
			if (id == ROOM_ID_SQUARE) {
				// A room from before the ids: a new id and shape in the same room world, and its
				// universe kept (the identity it came from, the map name the tracker reports).
				id = new_room_id();
				room_table_add(id, map);
				room_shape(id, shape);
				double c = cos(yaw), sn = sin(yaw);
				for (int i = 0; i < 4; i++) {
					room_to_session(shape[i], c, sn, t, g_seen[i]);
				}
				fprintf(stderr, "pico4-slamd: the room now has id %09llx (universe from %s)\n", (unsigned long long)id,
				        map);
			}
			write_boundary(SESSION_BOUNDARY, (const double(*)[3])g_seen, 4);
		}
	} else if (seen_n > 0) {
		fprintf(stderr, "pico4-slamd: the relocalised boundary is no room shape\n");
	}
	if (state != 1) {
		id = new_room_id();
		room_shape(id, shape);
		fprintf(stderr, "pico4-slamd: a new room (id %09llx); its map is saved at the stop\n", (unsigned long long)id);
		write_boundary(SESSION_BOUNDARY, (const double(*)[3])shape, 4);
	}
	room_identity(id, feed->room_uuid, sizeof(feed->room_uuid));
	feed->room_generation = generation;
	__atomic_store_n(&feed->room_state, state, __ATOMIC_RELEASE);
}

static struct isp_tables g_tables;
static uint16_t g_raw[PICO4_SLAM_FEED_CAMS][PICO4_SLAM_FEED_W * PICO4_SLAM_FEED_H];
static uint8_t g_images[PICO4_SLAM_FEED_CAMS][PICO4_SLAM_FEED_W * PICO4_SLAM_FEED_H];

/*
 * The see-through's depth mesh (vst.h), fed as PICO OS's see-through service feeds it: each pair
 * of the forward cameras with the tracker's result at the pair's time, which the IMU thread keeps
 * in a ring (a result per sample; the frame thread looks it up 30 ms later), and the tracker's map
 * points, which libSlam hands its point callback on its own thread.
 */
#define HEADS 256
static struct
{
	uint8_t res[HEADS][VST_HEAD_DATA_SIZE];
	uint64_t written; //!< Results written; the slot of result n is n % HEADS.
} g_heads;
static struct vst_slam_points g_points, g_points_fed;
static int g_points_lock, g_points_new;
static bool g_vst;

//! libSlam's point callback: the newest map points, for the next pair.
static void
on_points(const struct vst_slam_points *points)
{
	if (__atomic_exchange_n(&g_points_lock, 1, __ATOMIC_ACQUIRE)) {
		return; // the frame thread is copying the last ones
	}
	memcpy(&g_points, points, sizeof(g_points));
	g_points_new = 1;
	__atomic_store_n(&g_points_lock, 0, __ATOMIC_RELEASE);
}

//! The tracker's result nearest to @p ts into @p out; false when the ring has none near it.
static bool
head_at(int64_t ts, uint8_t out[VST_HEAD_DATA_SIZE])
{
	uint64_t w = __atomic_load_n(&g_heads.written, __ATOMIC_ACQUIRE);
	int64_t best = INT64_MAX;
	for (uint64_t i = w > HEADS - 8 ? w - (HEADS - 8) : 0; i < w; i++) {
		int64_t t;
		memcpy(&t, g_heads.res[i % HEADS], 8);
		int64_t d = t > ts ? t - ts : ts - t;
		if (d < best) {
			best = d;
			memcpy(out, g_heads.res[i % HEADS], VST_HEAD_DATA_SIZE);
		}
	}
	// The IMU thread may have overwritten the slot meanwhile: the copy's own time must match.
	int64_t t;
	memcpy(&t, out, 8);
	return best < 5000000 && (t > ts ? t - ts : ts - t) == best;
}

//! Feed the forward pair (cameras 3 and 2: left and right as PICO OS's see-through names them).
static void
vst_feed_set(int64_t ts, int64_t exposure_ns)
{
	static uint8_t head[VST_HEAD_DATA_SIZE];
	if (!head_at(ts, head)) {
		return;
	}
	vst_feed(g_images[3], g_images[2], PICO4_SLAM_FEED_W, PICO4_SLAM_FEED_H, ts, exposure_ns, head);
	// The map points with each pair, while the tracker tracks (as PICO OS's service).
	if (head[0x40] == 1 && head[0x41] == 1 && !__atomic_exchange_n(&g_points_lock, 1, __ATOMIC_ACQUIRE)) {
		if (g_points_new) {
			memcpy(&g_points_fed, &g_points, sizeof(g_points));
			g_points_new = 0;
		}
		__atomic_store_n(&g_points_lock, 0, __ATOMIC_RELEASE);
		vst_points(&g_points_fed);
	}
}

/*
 * PICO OS's auto exposure of the head cameras (docs/xr.md#auto-exposure): its
 * libExposureImageProcess.so, called as its tracking service's calcExpo thread calls it, on a
 * set's four images after black level and lens shading, before the gamma table, with the
 * exposure they were taken with. Cameras 2 and 3 share one setting. The result goes to the driver,
 * which owns the station, unless it is what the set was taken with or what was sent last.
 */
typedef int (*expo_fn)(const void *, const void *, const void *, const void *, uint16_t *, uint8_t *, void *, int,
                       int, int);
//! The cameras' frame rate as PICO OS passes it (scene sets come at half of it); 60 when the sets
//! come at 30 Hz (the station at 60 Hz, PICO4_CAMERA_HZ in the driver).
#define AE_FPS 50
#define AE_BIT_WIDTH 16 //!< 10-bit samples in 16 bits.
#define AE_FLAG_DETECT 1 //!< Start the mains-flicker detection, as PICO OS does once at the start.

struct ae_ctx
{
	struct pico4_slam_feed *feed;
	expo_fn predict;
	int ready; //!< 1: g_linear and lines/gain hold a set for the auto exposure (frame thread -> AE thread).
	int fps;   //!< The cameras' rate, from the sets' spacing (frame thread -> AE thread).
	uint32_t lines[PICO4_SLAM_FEED_CAMS], gain[PICO4_SLAM_FEED_CAMS];
	int stop;
	uint64_t runs, sent;
};
static uint16_t g_linear[PICO4_SLAM_FEED_CAMS][PICO4_SLAM_FEED_W * PICO4_SLAM_FEED_H];

static void *
ae_thread(void *arg)
{
	struct ae_ctx *c = arg;
	struct pico4_slam_feed *feed = c->feed;
	int flags = AE_FLAG_DETECT, last_ret = 2, have_sent = 0;
	uint16_t sent_lines[4] = {0};
	uint8_t sent_gains[4] = {0};
	while (!__atomic_load_n(&c->stop, __ATOMIC_ACQUIRE)) {
		if (!__atomic_load_n(&c->ready, __ATOMIC_ACQUIRE)) {
			usleep(2000);
			continue;
		}
		uint16_t lines[8] = {0};
		uint8_t gains[8] = {0};
		for (int k = 0; k < 4; k++) {
			lines[k] = (uint16_t)c->lines[k];
			gains[k] = (uint8_t)(c->gain[k] > 255 ? 255 : c->gain[k]);
		}
		struct vec unused = {0, 0, 0};
		int ret = c->predict(g_linear[0], g_linear[1], g_linear[2], g_linear[3], lines, gains, &unused, __atomic_load_n(&c->fps, __ATOMIC_RELAXED),
		                     AE_BIT_WIDTH, flags);
		flags = 0;
		c->runs++;
		if (ret != last_ret) {
			fprintf(stderr, "pico4-slamd: auto exposure %s\n",
			        ret == 0   ? "detecting the mains frequency"
			        : ret == 1 ? "found the frame rate off the mains frequency (flicker); the station keeps its rate"
			                   : "running");
			last_ret = ret;
		}
		int own = 1, same = have_sent;
		for (int k = 0; k < 4; k++) {
			own = own && lines[k] == c->lines[k] && gains[k] == c->gain[k];
			same = same && lines[k] == sent_lines[k] && gains[k] == sent_gains[k];
		}
		if (!own && !same) {
			for (int k = 0; k < 4; k++) {
				feed->ae_lines[k] = lines[k];
				feed->ae_gain[k] = gains[k];
				sent_lines[k] = lines[k];
				sent_gains[k] = gains[k];
			}
			have_sent = 1;
			__atomic_fetch_add(&feed->ae_written, 1, __ATOMIC_RELEASE);
			if (c->sent++ % 50 == 0) {
				fprintf(stderr, "pico4-slamd: exposure %u/%u %u/%u %u/%u %u/%u (lines/gain per camera), %llu of %llu sets\n",
				        lines[0], gains[0], lines[1], gains[1], lines[2], gains[2], lines[3], gains[3],
				        (unsigned long long)c->sent, (unsigned long long)c->runs);
			}
		}
		__atomic_store_n(&c->ready, 0, __ATOMIC_RELEASE);
	}
	return 0;
}

//! PICO OS's auto exposure, unless PICO4_SLAMD_AE=0 leaves it to the driver; 0 when it cannot load.
static expo_fn
load_ae(void)
{
	const char *env = getenv("PICO4_SLAMD_AE");
	if (env != NULL && strcmp(env, "0") == 0) {
		fprintf(stderr, "pico4-slamd: auto exposure left to the driver (PICO4_SLAMD_AE=0)\n");
		return 0;
	}
	void *lib = dlopen("libExposureImageProcess.so", RTLD_NOW);
	expo_fn predict = lib != 0 ? (expo_fn)dlsym(lib, "ExposurePredict") : 0;
	if (predict == 0) {
		fprintf(stderr, "pico4-slamd: no auto exposure (%s); the driver's runs\n", lib != 0 ? "ExposurePredict" : dlerror());
	}
	return predict;
}

static void
on_result(void)
{
}

//! The whole file into g_text; its length, or -1.
static long
read_text(const char *path)
{
	int fd = open(path, O_RDONLY | O_CLOEXEC);
	if (fd < 0) {
		return -1;
	}
	long len = 0;
	ssize_t r;
	while (len < (long)sizeof(g_text) && (r = read(fd, g_text + len, sizeof(g_text) - (size_t)len)) > 0) {
		len += r;
	}
	close(fd);
	return len;
}

static int
load_table(const char *path, uint32_t *out, int count)
{
	long len = read_text(path);
	return len >= 0 && isp_parse_numbers(g_text, len, out, count) == count ? 0 : -1;
}

//! The saved module ids (camera_otp.json) into @p ids; zeros where the file has none.
static void
read_saved_ids(uint8_t ids[4])
{
	long len = read_text(OTP_PATH);
	for (int k = 0; k < 4; k++) {
		int v = len > 0 ? isp_otp_id(g_text, len, k) : -1;
		ids[k] = v > 0 ? (uint8_t)v : 0;
	}
}

/*!
 * The cameras' module ids as PICO OS's station service hands them out: the saved ones if valid,
 * else what the driver read from the modules' OTP for this generation, saved when PICO OS would
 * keep them. Camera 0's id; 0 if there is none.
 */
static int
camera_vendor(struct pico4_slam_feed *feed, uint32_t generation)
{
	uint8_t ids[4];
	memcpy(ids, feed->otp_saved, sizeof(ids));
	const char *from = "saved";
	if (!isp_otp_valid(ids)) {
		if (__atomic_load_n(&feed->otp_generation, __ATOMIC_ACQUIRE) == generation && feed->otp_state == 1) {
			memcpy(ids, feed->otp_read, sizeof(ids));
			from = "the modules' OTP";
			char json[128];
			int n = isp_otp_json(ids, json, sizeof(json));
			int fd = isp_otp_valid(ids) ? open(OTP_PATH ".new", O_WRONLY | O_CREAT | O_TRUNC | O_CLOEXEC, 0644) : -1;
			if (fd >= 0) {
				int ok = write(fd, json, (size_t)n) == n;
				close(fd);
				ok = ok && rename(OTP_PATH ".new", OTP_PATH) == 0;
				fprintf(stderr, "pico4-slamd: camera_otp %s\n", ok ? "saved" : "not saved");
				if (ok) {
					memcpy(feed->otp_saved, ids, sizeof(ids)); // the driver's next start reads no OTP
				}
			}
		} else {
			memset(ids, 0, sizeof(ids));
			from = "none: the station gave no ids";
		}
	}
	fprintf(stderr, "pico4-slamd: camera module ids %u,%u,%u,%u (%s)\n", ids[0], ids[1], ids[2], ids[3], from);
	return ids[0];
}

/*!
 * The lens-shading table as PICO OS chooses it: by camera 0's module (vendor) id. PICO4_SLAMD_LSC
 * names a table instead; NULL: PICO OS calls the vendor illegal and shades nothing.
 */
static const char *
lsc_table(int vendor)
{
	const char *lsc = getenv("PICO4_SLAMD_LSC");
	if (lsc != NULL && *lsc != '\0') {
		return lsc;
	}
	const char *file = isp_lsc_file(vendor);
	fprintf(stderr, "pico4-slamd: camera vendor %d: %s\n", vendor, file != NULL ? file : "no lens shading");
	return file;
}

static int
load_tables(int vendor)
{
	static uint32_t tmp[ISP_W * ISP_H];
	const char *lsc = lsc_table(vendor);
	if (lsc == NULL) {
		for (int i = 0; i < ISP_W * ISP_H; i++) {
			g_tables.lsc[i] = 128; // 1x
		}
	} else {
		char path[256];
		snprintf(path, sizeof(path), CONFIG_DIR "%s", lsc);
		if (load_table(path, tmp, ISP_W * ISP_H) < 0) {
			fprintf(stderr, "pico4-slamd: cannot read the lens-shading table %s\n", path);
			return -1;
		}
		for (int i = 0; i < ISP_W * ISP_H; i++) {
			g_tables.lsc[i] = (uint16_t)tmp[i];
		}
	}
	if (load_table(CONFIG_DIR "config_gma10.txt", tmp, 1024) < 0) {
		fprintf(stderr, "pico4-slamd: cannot read " CONFIG_DIR "config_gma10.txt\n");
		return -1;
	}
	for (int i = 0; i < 1024; i++) {
		g_tables.gamma[i] = (uint8_t)(tmp[i] > 255 ? 255 : tmp[i]);
	}
	return 0;
}

static struct pico4_slam_feed *
open_feed(int gid)
{
	int fd = open(PICO4_SLAM_FEED_PATH, O_RDWR | O_CREAT | O_CLOEXEC, 0660);
	if (fd < 0 || ftruncate(fd, sizeof(struct pico4_slam_feed)) < 0) {
		fprintf(stderr, "pico4-slamd: cannot create " PICO4_SLAM_FEED_PATH "\n");
		return 0;
	}
	if (gid >= 0) {
		fchown(fd, (uid_t)-1, (gid_t)gid);
	}
	fchmod(fd, 0660);
	struct pico4_slam_feed *feed = mmap(0, sizeof(*feed), PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
	close(fd);
	if (feed == (void *)-1) {
		return 0;
	}
	if (feed->magic != PICO4_SLAM_FEED_MAGIC) {
		memset(feed, 0, sizeof(*feed));
		feed->version = 1;
		__atomic_store_n(&feed->magic, PICO4_SLAM_FEED_MAGIC, __ATOMIC_RELEASE);
	}
	return feed;
}

static void
put32(uint8_t *p, uint32_t v)
{
	memcpy(p, &v, 4);
}

static void
put64(uint8_t *p, uint64_t v)
{
	memcpy(p, &v, 8);
}

/*
 * The frame thread, as PICO OS's checkIsSync thread: each scene set goes in once the IMU thread
 * has fed samples this far past it. updateImage hands the set to the Compute DSP and waits for
 * it; fed from the IMU's thread, the samples piled up behind each set and the poses came in
 * bursts at 25 Hz, up to 20 ms late.
 */
struct frames_ctx
{
	struct pico4_slam_feed *feed;
	void *h;
	image_fn image;
	uint64_t sr;
	int64_t imu_fed_ts; //!< Written by the IMU thread: the newest sample fed.
	uint64_t sets_fed, skipped;
	int stop; //!< Set by the IMU thread before the tracker's stop.
	struct ae_ctx *ae; //!< Or NULL.
	int64_t last_set_ts; //!< The set before, for the cameras' rate.
};

static void *
frame_thread(void *arg)
{
	struct frames_ctx *c = arg;
	struct pico4_slam_feed *feed = c->feed;
	static uint8_t items[4 * 0x48];
	struct vec frames = {items, items + 4 * 0x48, items + 4 * 0x48};
	while (!__atomic_load_n(&c->stop, __ATOMIC_ACQUIRE)) {
		uint64_t sw = __atomic_load_n(&feed->sets_written, __ATOMIC_ACQUIRE);
		if (sw - c->sr > PICO4_SLAM_FEED_SETS - 1) {
			__atomic_fetch_add(&c->skipped, sw - c->sr - (PICO4_SLAM_FEED_SETS - 1), __ATOMIC_RELAXED);
			c->sr = sw - (PICO4_SLAM_FEED_SETS - 1);
		}
		const struct pico4_slam_feed_set *s = &feed->sets[c->sr % PICO4_SLAM_FEED_SETS];
		if (c->sr == sw || s->ts_ns + SET_DELAY_NS > __atomic_load_n(&c->imu_fed_ts, __ATOMIC_ACQUIRE)) {
			usleep(1000);
			continue;
		}
		int64_t ts = s->ts_ns;
		uint32_t lines[4], gain[4];
		int64_t sof[4];
		memcpy(g_raw, s->pixels, sizeof(g_raw));
		memcpy(lines, s->exposure_lines, sizeof(lines));
		memcpy(gain, s->gain, sizeof(gain));
		memcpy(sof, s->sof_ns, sizeof(sof));
		// The driver may have reused the slot while it was copied.
		if (__atomic_load_n(&feed->sets_written, __ATOMIC_ACQUIRE) - c->sr >= PICO4_SLAM_FEED_SETS) {
			__atomic_fetch_add(&c->skipped, 1, __ATOMIC_RELAXED);
			c->sr++;
			continue;
		}
		uint64_t n = __atomic_load_n(&c->sets_fed, __ATOMIC_RELAXED);
		// The auto exposure takes the set when it is done with the one before.
		bool to_ae = c->ae != 0 && !__atomic_load_n(&c->ae->ready, __ATOMIC_ACQUIRE);
		for (int k = 0; k < 4; k++) {
			isp_process(&g_tables, g_raw[k], PICO4_SLAM_FEED_W, g_images[k], to_ae ? g_linear[k] : 0);
			uint8_t *it = items + k * 0x48;
			memset(it, 0, 0x48);
			it[0] = (uint8_t)k;
			put32(it + 0x04, PICO4_SLAM_FEED_W);
			put32(it + 0x08, PICO4_SLAM_FEED_H);
			put32(it + 0x14, lines[k]);
			put32(it + 0x18, gain[k]);
			put64(it + 0x20, (uint64_t)sof[k]);
			put64(it + 0x28, (uint64_t)ts);
			put64(it + 0x30, 13 + n);
			put32(it + 0x38, PICO4_SLAM_FEED_W * PICO4_SLAM_FEED_H);
			const uint8_t *px = g_images[k];
			memcpy(it + 0x40, &px, 8);
		}
		// Sets come at half the cameras' rate: 40 ms apart at 50 Hz, 33 ms at 60 Hz.
		if (c->ae != 0 && c->last_set_ts != 0 && ts - c->last_set_ts > 25000000 && ts - c->last_set_ts < 45000000) {
			__atomic_store_n(&c->ae->fps, ts - c->last_set_ts < 36700000 ? 60 : AE_FPS, __ATOMIC_RELAXED);
		}
		c->last_set_ts = ts;
		if (to_ae) {
			memcpy(c->ae->lines, lines, sizeof(lines));
			memcpy(c->ae->gain, gain, sizeof(gain));
			__atomic_store_n(&c->ae->ready, 1, __ATOMIC_RELEASE);
		}
		c->image(c->h, 0, &frames);
		// The depth mesh only while the driver asks for it (SteamVR's room view).
		if (g_vst && (uint64_t)now_ns(CLOCK_MONOTONIC) - __atomic_load_n(&feed->mesh_wanted_ns, __ATOMIC_ACQUIRE) <
		                 1000000000ull) {
			vst_feed_set(ts, lines[3]);
		}
		__atomic_store_n(&c->sets_fed, n + 1, __ATOMIC_RELAXED);
		c->sr++;
	}
	return 0;
}

//! libSlam's interface, as PICO OS's tracking service calls it.
struct tracker
{
	create_fn create;
	set_fn set;
	start_fn start;
	register_fn reg;
	points_register_fn points; //!< Or NULL.
	prestop_fn prestop;
	stop_fn stop;
	destroy_fn destroy;
};

//! As PICO OS's service starts it: the IMU's model, start, the result callback, then 6DoF.
static void *
tracker_start(const struct tracker *t, struct pico4_slam_feed *feed)
{
	// The tracker's copy of a saved room's boundary is this session's, if it relocalises.
	unlink(RELOCALISED_BOUNDARY);
	void *h = t->create("");
	if (h == 0) {
		return 0;
	}
	t->set(h, "headSensorModel", "icm4x6xx");
	t->start(h, "");
	t->reg(on_result);
	if (g_vst && t->points != 0) {
		t->points(on_points);
	}
	__atomic_store_n(&feed->room_state, 0, __ATOMIC_RELEASE);
	t->set(h, "start6DOF", "6");
	t->set(h, "bigRoom", "0"); // as PICO OS without large-space mode
	// The tracker's ground detection (setPlaneDetect 1): the floor's height and status in every
	// result (+0x48, +0x50), which PICO OS's see-through takes as its ground (docs/xr.md#passthrough).
	t->set(h, "plane", "1");
	return h;
}

//! As PICO OS's service stops it: the map is saved (with a boundary) on the way.
static void
tracker_stop(const struct tracker *t, void *h)
{
	t->prestop(h);
	t->stop(h, "non_sleep");
	t->destroy(h);
}

int
main(int argc, char **argv, char **envp)
{
	(void)envp;
	int gid = argc > 1 ? atoi(argv[1]) : -1;
	struct pico4_slam_feed *feed = open_feed(gid);
	if (feed == 0) {
		return 1;
	}
	// The saved module ids for the driver, which reads the OTP when they are not valid.
	uint8_t saved[4];
	read_saved_ids(saved);
	memcpy(feed->otp_saved, saved, sizeof(saved));
	// Wait for a driver, then serve its generation for as long as it lasts.
	uint32_t generation;
	while ((generation = __atomic_load_n(&feed->generation, __ATOMIC_ACQUIRE)) == 0 ||
	       __atomic_load_n(&feed->imu_written, __ATOMIC_ACQUIRE) == 0) {
		feed->slamd_heartbeat_ns = (uint64_t)now_ns(CLOCK_MONOTONIC);
		usleep(100000);
	}
	if (load_tables(camera_vendor(feed, generation)) < 0) {
		return 1;
	}

	void *lib = dlopen("libSlam.so", RTLD_NOW);
	if (!lib) {
		fprintf(stderr, "pico4-slamd: %s\n", dlerror());
		return 1;
	}
	const struct tracker t = {
	    .create = (create_fn)dlsym(lib, "createHandle"),
	    .set = (set_fn)dlsym(lib, "setAlgorithmParameters"),
	    .start = (start_fn)dlsym(lib, "startAlgorithm"),
	    .reg = (register_fn)dlsym(lib, "AlgorithmResultCallbackRegister"),
	    .points = (points_register_fn)dlsym(lib, "AlgorithmPointCallbackRegister"),
	    .prestop = (prestop_fn)dlsym(lib, "preStopAlgorithm"),
	    .stop = (stop_fn)dlsym(lib, "stopAlgorithm"),
	    .destroy = (destroy_fn)dlsym(lib, "destroyHandle"),
	};
	image_fn image = (image_fn)dlsym(lib, "updateImage");
	imu_fn imu = (imu_fn)dlsym(lib, "update6AxisIMU");
	result_fn result = (result_fn)dlsym(lib, "getAlgorithmResult");
	relocinfo_fn relocinfo = (relocinfo_fn)dlsym(lib, "getRelocMapInfo");
	if (!t.create || !t.set || !t.start || !image || !imu || !result || !t.reg || !t.prestop || !t.stop ||
	    !t.destroy) {
		fprintf(stderr, "pico4-slamd: libSlam.so lacks a function of its interface\n");
		return 1;
	}
	// The see-through's depth mesh.
	g_vst = vst_start(feed) == 0;
	void *h = tracker_start(&t, feed);
	if (h == 0) {
		fprintf(stderr, "pico4-slamd: createHandle failed\n");
		return 1;
	}
	int64_t started = now_ns(CLOCK_MONOTONIC), reloc_checked = 0;
	bool room_settled = false, relocalised = false;
	// The tracker's time to relocalise runs from its first frame set, which the frame thread hands
	// over only with the IMU past it: the cameras can take 10 s to start after a failed sync.
	const char *relocalise_env = getenv("PICO4_SLAMD_RELOCALISE_S");
	const int64_t relocalise_ns = (int64_t)(relocalise_env != 0 ? atoi(relocalise_env) : RELOCALISE_DEFAULT_S) * 1000000000LL;
	uint64_t sets_at_start = 0;
	int64_t input_since = 0;
	uint64_t floor_requests = __atomic_load_n(&feed->floor_requests, __ATOMIC_ACQUIRE);
	const char *restart_env = getenv("PICO4_SLAMD_RESTART_S");
	const int64_t restart_ns = (int64_t)(restart_env != 0 ? atoi(restart_env) : RESTART_DEFAULT_S) * 1000000000LL;
	// The watchdog: the confidence before, and since when it is 0 after a 1 (0: not lost).
	int last_confidence = 0;
	int64_t lost_since = 0;
	uint64_t restarts = 0;
	// The ground the tracker reports (+0x48 height, +0x50 status), logged when it changes.
	int last_plane_status = -1;
	double last_plane_height = 0;
	signal(SIGTERM, on_stop_signal);
	signal(SIGINT, on_stop_signal);
	fprintf(stderr, "pico4-slamd: tracking for driver generation %u\n", generation);

	static struct frames_ctx frames;
	frames.feed = feed;
	frames.h = h;
	frames.image = image;
	frames.sr = __atomic_load_n(&feed->sets_written, __ATOMIC_ACQUIRE);
	static struct ae_ctx ae;
	ae.feed = feed;
	ae.predict = load_ae();
	ae.fps = AE_FPS;
	pthread_t ae_th;
	if (ae.predict != 0 && pthread_create(&ae_th, 0, ae_thread, &ae) == 0) {
		frames.ae = &ae;
		__atomic_store_n(&feed->ae_generation, generation, __ATOMIC_RELEASE);
	}
	pthread_t thread;
	if (pthread_create(&thread, 0, frame_thread, &frames) != 0) {
		fprintf(stderr, "pico4-slamd: cannot start the frame thread\n");
		return 1;
	}

	// The IMU thread, as PICO OS's threadReader: every sample and its pose, as soon as it is there.
	uint8_t res[0x170];
	double sample[7];
	struct vec samples = {sample, sample + 7, sample + 7};
	uint64_t ir = __atomic_load_n(&feed->imu_written, __ATOMIC_ACQUIRE);
	uint64_t imu_fed = 0, skipped = 0, tracked = 0;
	int64_t last_imu = 0, report = now_ns(CLOCK_MONOTONIC) + 10000000000LL;
	int64_t cpu0 = now_ns(CLOCK_PROCESS_CPUTIME_ID), wall0 = now_ns(CLOCK_MONOTONIC);
	bool known = false;
	// The longest time between two poses in each report: bursts show here.
	int64_t last_pose_wall = 0, longest_gap = 0;
	for (;;) {
		int64_t now = now_ns(CLOCK_MONOTONIC);
		feed->slamd_heartbeat_ns = (uint64_t)now;
		if (g_stop || __atomic_load_n(&feed->generation, __ATOMIC_ACQUIRE) != generation) {
			// As PICO OS's service stops it: the map is saved (with a boundary) on the way.
			fprintf(stderr, "pico4-slamd: %s; stopping the tracker\n",
			        g_stop ? "asked to stop" : "the driver started again, a new tracker follows");
			fflush(stderr);
			__atomic_store_n(&frames.stop, 1, __ATOMIC_RELEASE);
			pthread_join(thread, 0);
			if (frames.ae != 0) {
				__atomic_store_n(&feed->ae_generation, 0, __ATOMIC_RELEASE); // the driver's own again
				__atomic_store_n(&ae.stop, 1, __ATOMIC_RELEASE);
				pthread_join(ae_th, 0);
			}
			vst_stop();
			tracker_stop(&t, h);
			fprintf(stderr, "pico4-slamd: tracker stopped\n");
			fflush(stderr);
			exit(0);
		}
		// The room at the tracker's relocalisation, once its copy of the saved room's boundary is
		// written, and a new room if there was none in time; a later relocalisation moves the
		// session into the saved room's world then. The copy is the sign: tracker_start removes it
		// and the tracker writes it when it relocalises. libSlam's RelocResultCallbackRegister only
		// calls back with a flag in its configuration set, and does not here.
		if (now - reloc_checked > 100000000LL) {
			reloc_checked = now;
			if (!relocalised && read_boundary(RELOCALISED_BOUNDARY, g_seen) == 4) {
				relocalised = true;
				fprintf(stderr, "pico4-slamd: the tracker relocalised in a saved map %.1f s after its start\n",
				        (double)(now - started) / 1e9);
				settle_room(feed, generation, h, relocinfo);
				room_settled = true;
			}
			// SteamVR's room setup asks for the floor: the tracker's ground detection runs
			// again (setPlaneDetect 1; each plane it fits then replaces the last), so the floor
			// comes from what the head sees now, not from the first second after the start.
			const uint64_t fr = __atomic_load_n(&feed->floor_requests, __ATOMIC_ACQUIRE);
			if (fr != floor_requests) {
				floor_requests = fr;
				t.set(h, "plane", "1");
				fprintf(stderr, "pico4-slamd: ground detection again (room setup)\n");
			}
		}
		if (input_since == 0 && __atomic_load_n(&frames.sets_fed, __ATOMIC_RELAXED) != sets_at_start) {
			input_since = now;
		}
		if (!room_settled && input_since != 0 && now - input_since > relocalise_ns) {
			settle_room(feed, generation, h, relocinfo);
			room_settled = true;
		}
		uint64_t iw = __atomic_load_n(&feed->imu_written, __ATOMIC_ACQUIRE);
		if (iw - ir > PICO4_SLAM_FEED_IMU) {
			skipped += iw - ir - PICO4_SLAM_FEED_IMU;
			ir = iw - PICO4_SLAM_FEED_IMU;
		}
		if (ir == iw) {
			usleep(200);
			continue;
		}
		const struct pico4_slam_feed_imu e = feed->imu[ir % PICO4_SLAM_FEED_IMU];
		ir++;
		if (e.ts_ns <= last_imu) {
			continue; // the tracker needs increasing times
		}
		last_imu = e.ts_ns;
		for (int k = 0; k < 3; k++) {
			sample[k] = e.gyro[k];
			sample[3 + k] = e.accel[k];
		}
		// The IMU's own temperature, as PICO OS's sensor DSP inserts it (NaN: none yet).
		known = e.temperature >= TEMPERATURE_MIN && e.temperature <= TEMPERATURE_MAX;
		sample[6] = known ? e.temperature : DEFAULT_TEMPERATURE;
		imu(h, &samples, e.ts_ns);
		imu_fed++;
		__atomic_store_n(&frames.imu_fed_ts, e.ts_ns, __ATOMIC_RELEASE);
		memset(res, 0, sizeof(res));
		if (result(h, res) == 0) {
			if (g_vst) {
				uint64_t hw = g_heads.written;
				memcpy(g_heads.res[hw % HEADS], res, sizeof(res));
				__atomic_store_n(&g_heads.written, hw + 1, __ATOMIC_RELEASE);
			}
			uint64_t w = __atomic_load_n(&feed->poses_written, __ATOMIC_RELAXED);
			struct pico4_slam_feed_pose *p = &feed->poses[w % PICO4_SLAM_FEED_POSES];
			memcpy(&p->ts_ns, res, 8);
			memcpy(p->position, res + 0x08, 24);
			memcpy(p->orientation, res + 0x20, 32);
			memcpy(p->velocity, res + 0xe0, 24);
			memcpy(p->angular, res + 0x110, 24);
			p->type = res[0x40];
			p->confidence = res[0x41];
			memcpy(&p->error, res + 0x44, 4);
			double plane_height;
			memcpy(&plane_height, res + 0x48, 8);
			const int plane_status = res[0x50];
			p->floor_status = (uint32_t)plane_status;
			p->floor_height = plane_height;
			const double plane_moved = plane_height - last_plane_height;
			if (plane_status != last_plane_status ||
			    (plane_status != 0 && (plane_moved > 0.01 || plane_moved < -0.01))) {
				fprintf(stderr, "pico4-slamd: ground status %d, height %.3f m (head at %.3f %.3f %.3f)\n",
				        plane_status, plane_height, p->position[0], p->position[1], p->position[2]);
				last_plane_status = plane_status;
				last_plane_height = plane_height;
			}
			__atomic_store_n(&feed->poses_written, w + 1, __ATOMIC_RELEASE);
			tracked += p->confidence > 0;
			// PICO OS's watchdog: a drop from 1 to 0 starts the clock (CLOCK_BOOTTIME, as there);
			// any confidence clears it.
			if (p->confidence == 0 && last_confidence == 1) {
				lost_since = now_ns(CLOCK_BOOTTIME);
				fprintf(stderr, "pico4-slamd: tracking lost (error 0x%x)\n", (unsigned)p->error);
			} else if (p->confidence != 0) {
				lost_since = 0;
			}
			last_confidence = p->confidence;
			int64_t wall = now_ns(CLOCK_MONOTONIC);
			if (last_pose_wall != 0 && wall - last_pose_wall > longest_gap) {
				longest_gap = wall - last_pose_wall;
			}
			last_pose_wall = wall;
		}
		if (restart_ns > 0 && lost_since != 0 && now_ns(CLOCK_BOOTTIME) - lost_since > restart_ns) {
			// As PICO OS's restartTracking: stop the tracker and start it again. A new world, which
			// relocalises in the room as at the start.
			lost_since = 0;
			restarts++;
			fprintf(stderr, "pico4-slamd: lost for over %lld s; restarting the tracker (restart %llu)\n",
			        (long long)(restart_ns / 1000000000LL), (unsigned long long)restarts);
			fflush(stderr);
			__atomic_store_n(&frames.stop, 1, __ATOMIC_RELEASE);
			pthread_join(thread, 0);
			tracker_stop(&t, h);
			h = tracker_start(&t, feed);
			if (h == 0) {
				fprintf(stderr, "pico4-slamd: createHandle failed on the restart\n");
				return 1;
			}
			started = now_ns(CLOCK_MONOTONIC);
			room_settled = false;
			relocalised = false;
			sets_at_start = __atomic_load_n(&frames.sets_fed, __ATOMIC_RELAXED);
			input_since = 0;
			last_confidence = 0;
			frames.h = h;
			frames.sr = __atomic_load_n(&feed->sets_written, __ATOMIC_ACQUIRE);
			__atomic_store_n(&frames.stop, 0, __ATOMIC_RELEASE);
			if (pthread_create(&thread, 0, frame_thread, &frames) != 0) {
				fprintf(stderr, "pico4-slamd: cannot start the frame thread again\n");
				return 1;
			}
			fprintf(stderr, "pico4-slamd: tracker started again\n");
			fflush(stderr);
		}
		if (now >= report) {
			double cpu = (double)(now_ns(CLOCK_PROCESS_CPUTIME_ID) - cpu0) / (double)(now - wall0);
			fprintf(stderr,
			        "pico4-slamd: %llu frame sets, %llu IMU samples, %llu tracked poses, %llu skipped; IMU at "
			        "%.1f C%s; pose %.1f ms after its sample, longest gap between poses %.1f ms; CPU %.0f %% of "
			        "one core\n",
			        (unsigned long long)__atomic_load_n(&frames.sets_fed, __ATOMIC_RELAXED),
			        (unsigned long long)imu_fed, (unsigned long long)tracked,
			        (unsigned long long)(skipped + __atomic_load_n(&frames.skipped, __ATOMIC_RELAXED)), sample[6],
			        known ? "" : " (none reported, default)", (double)(now_ns(CLOCK_MONOTONIC) - e.ts_ns) / 1e6,
			        (double)longest_gap / 1e6, 100.0 * cpu);
			longest_gap = 0;
			fflush(stderr);
			report = now + 10000000000LL;
		}
	}
}
