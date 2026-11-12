// SPDX-License-Identifier: BSL-1.0
/*
 * pico4-cam: the four OV7251 tracking cameras and the IMX471 RGB camera through V4L2
 * (src/monado/pico4_cam.c).
 *
 *   pico4-cam capture [--slots 0,1,2,3] [--frames N] [--out DIR]
 *                     [--ring N] [--timeout MS]
 *                     [--no-station] [--no-station-reset] [--stats-log FILE]
 *                     [--exposure LINES [--gain G]]
 *                     [--record-leds FILE] [--record-scene FILE]
 *                     [--record-imu FILE]
 *       Streams until every selected camera delivered N frames (default 30),
 *       saves an early scene frame of each camera as DIR/camS.pgm (8 bits, the
 *       640x480 scene rows) and DIR/camS.raw16 (the whole buffer, little-endian
 *       16-bit samples: the sensor's metadata line, then the scene rows), and
 *       prints a JSON summary: per slot the frame count, the frame rate from
 *       start-of-frame timestamps, whether those increase, min/mean/max sample
 *       values and the frames of each exposure phase from the metadata line. The station (controller radio) triggers the frames: it
 *       is power-cycled first (unless --no-station-reset) and told to start
 *       once the sensors stream. It refuses unless all four sensors are on, so
 *       --slots only selects which cameras are reported. --stats-log writes
 *       "slot sof_boottime_ns mean latency_ns frame phase lines gain peak
 *       saturated" per frame (mean of a sparse grid; latency from start of
 *       frame to delivery; the brightest sample and the samples at 1023;
 *       the applied exposure from the metadata line, phase 0 if it does not
 *       decode), for example to find which slot goes dark when a lens is
 *       covered.
 *       --exposure (sensor lines, at most about 570) and --gain (16 = 1x)
 *       ask the station for that exposure; the saved frame is then the
 *       tenth or eleventh, after the setting took effect. Every other frame
 *       is the station's short controller-LED exposure; the saved frame is a
 *       scene frame by its metadata (or the brighter of two if it has none).
 *       --record-leds appends every controller-LED exposure of the selected
 *       cameras to FILE, for the controller tracking work: per frame a 48-byte
 *       header ("P4LF", version 1, slot, the frame counter, start of frame
 *       on CLOCK_BOOTTIME and CLOCK_MONOTONIC in ns, the applied exposure lines
 *       and gain, width, height; little-endian) and the scene rows as 8 bits
 *       (the 10-bit samples shifted right by two). --record-scene writes the
 *       scene exposures (the head-tracking frames) the same way.
 *       --record-imu appends the controllers' radio packets (the sensor
 *       service's ble_mcu events, with their IMU and input records) and the
 *       head IMU to FILE: records of an 8-byte header ("P4L", kind I, H or C,
 *       version 1, a pad byte, the length that follows as u16), I = the event's
 *       QTimer ticks (u64) and the raw packet, H = a head IMU event's QTimer
 *       ticks (u64), the sensor (u8: 0 accelerometer, 1 gyroscope) and its three
 *       values (float, m/s^2 or rad/s, raw axes = the factory body frame), C
 *       every 100 ms = a CLOCK_MONOTONIC time in ns and the QTimer at that
 *       moment (u64 each), to put both on one clock. The head IMU runs at 1 kHz
 *       as in the driver: without another sensor streaming, the sensor DSP slows
 *       ble_mcu to about 90 events per second a few minutes after it started.
 *
 *   pico4-cam rgb [--frames N] [--out DIR] [--exposure LINES] [--gain G]
 *       The RGB camera alone (slot 4, 2328x1748 RGGB at 50 fps, free-running: no
 *       station); see cmd_rgb().
 *
 * -v (before or after the command) logs progress to stderr. Exit status 0 means
 * success: every camera delivered N frames with increasing timestamps.
 */

#define _GNU_SOURCE // sigaction, mkdir under -std=c11

#include "pico4_cam.h"
#include "pico4_ssc.h"
#include "pico4_station.h"

#include <errno.h>
#include <pthread.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <time.h>

static volatile sig_atomic_t g_stop;
static volatile int g_station_stop;
static unsigned g_save_frame = 2; //!< Which frame of each camera to save (or the next, if brighter).

static void
on_signal(int sig)
{
	(void)sig;
	g_stop = 1;
}

static void
usage(FILE *f)
{
	fputs("usage: pico4-cam [-v] capture [--slots 0,1,2,3] [--frames N] [--out DIR]\n"
	      "                 [--ring N] [--timeout MS] [--no-station]\n"
	      "                 [--no-station-reset] [--stats-log FILE] [--exposure LINES [--gain G]]\n"
	      "                 [--record-leds FILE] [--record-scene FILE] [--record-imu FILE]\n"
	      "       pico4-cam [-v] rgb [--frames N] [--out DIR] [--exposure LINES] [--gain G]\n"
	      "                 [--step-exposure LINES [--step-gain G]]\n",
	      f);
}

static void
json_str(const char *s)
{
	putchar('"');
	for (; *s != '\0'; s++) {
		unsigned char ch = (unsigned char)*s;
		if (ch == '"' || ch == '\\') {
			printf("\\%c", ch);
		} else if (ch < 0x20) {
			printf("\\u%04x", ch);
		} else {
			putchar(ch);
		}
	}
	putchar('"');
}

static bool
parse_uint(const char *s, unsigned long max, unsigned long *out)
{
	char *end = NULL;
	unsigned long v;
	errno = 0;
	v = strtoul(s, &end, 0);
	if (errno != 0 || end == s || *end != '\0' || v > max) {
		return false;
	}
	*out = v;
	return true;
}

static bool
parse_slots(const char *s, unsigned *mask)
{
	char buf[64], *save = NULL, *tok;
	unsigned m = 0;
	snprintf(buf, sizeof(buf), "%s", s);
	for (tok = strtok_r(buf, ",", &save); tok != NULL; tok = strtok_r(NULL, ",", &save)) {
		unsigned long v;
		if (!parse_uint(tok, PICO4_CAM_NUM - 1, &v)) {
			return false;
		}
		m |= 1u << v;
	}
	*mask = m;
	return m != 0;
}


/*
 *
 * capture
 *
 */

struct slot_stats
{
	unsigned frames;
	unsigned with_ts;
	bool monotonic;
	int64_t first_ts, last_ts, prev_ts;
	uint64_t first_frame_id, last_frame_id;
	uint64_t sum;
	uint64_t count;
	unsigned min, max;
	bool saved;
	double saved_mean;
	char pgm[256], raw[256];
	int save_error;
	unsigned phases[3]; //!< Frames by metadata: scene, controller LEDs, other or undecodable.
};

static unsigned
sample(const uint8_t *line, unsigned x)
{
	return (unsigned)line[2 * x] | ((unsigned)line[2 * x + 1] << 8);
}

//! Every frame: all pixels of the first one, every second row and column of the rest.
static void
add_stats(struct slot_stats *st, const struct pico4_cam_frame *f, bool full)
{
	unsigned step = full ? 1 : 2;
	for (unsigned y = 0; y < f->height; y += step) {
		const uint8_t *line = (const uint8_t *)f->data + (size_t)y * f->stride;
		for (unsigned x = 0; x < f->width; x += step) {
			unsigned v = sample(line, x);
			st->sum += v;
			st->min = v < st->min ? v : st->min;
			st->max = v > st->max ? v : st->max;
		}
		st->count += (f->width + step - 1) / step;
	}
}

static int
save_frame(struct slot_stats *st, const struct pico4_cam_frame *f, const char *dir)
{
	FILE *pgm, *raw;
	uint8_t row[PICO4_CAM_WIDTH];

	snprintf(st->pgm, sizeof(st->pgm), "%s/cam%d.pgm", dir, f->slot);
	snprintf(st->raw, sizeof(st->raw), "%s/cam%d.raw16", dir, f->slot);
	pgm = fopen(st->pgm, "wbe");
	if (pgm == NULL) {
		return -errno;
	}
	fprintf(pgm, "P5\n%u %u\n255\n", f->width, f->height);
	for (unsigned y = 0; y < f->height; y++) {
		const uint8_t *line = (const uint8_t *)f->data + (size_t)y * f->stride;
		for (unsigned x = 0; x < f->width; x++) {
			unsigned v = sample(line, x) >> 2; // 10-bit samples, LSB-aligned
			row[x] = (uint8_t)(v > 255 ? 255 : v);
		}
		fwrite(row, 1, f->width, pgm);
	}
	if (fclose(pgm) != 0) {
		return -errno;
	}
	raw = fopen(st->raw, "wbe");
	if (raw == NULL) {
		return -errno;
	}
	for (unsigned y = 0; y < f->lines; y++) {
		fwrite((const uint8_t *)f->meta + (size_t)y * f->stride, 2, f->width, raw);
	}
	if (fclose(raw) != 0) {
		return -errno;
	}
	st->saved = true;
	return 0;
}

//! Mean of a sparse grid of samples.
static double
grid_mean(const struct pico4_cam_frame *f)
{
	uint64_t sum = 0, n = 0;
	for (unsigned y = 8; y < f->height; y += 16) {
		const uint8_t *line = (const uint8_t *)f->data + (size_t)y * f->stride;
		for (unsigned x = 8; x < f->width; x += 16) {
			sum += sample(line, x);
			n++;
		}
	}
	return n > 0 ? (double)sum / (double)n : 0.0;
}

//! The brightest sample and how many reach the 10-bit maximum (saturated controller LEDs).
static unsigned
frame_peak(const struct pico4_cam_frame *f, unsigned *out_saturated)
{
	unsigned peak = 0, saturated = 0;
	for (unsigned y = 0; y < f->height; y++) {
		const uint8_t *line = (const uint8_t *)f->data + (size_t)y * f->stride;
		for (unsigned x = 0; x < f->width; x++) {
			unsigned v = sample(line, x);
			peak = v > peak ? v : peak;
			saturated += v >= 1023;
		}
	}
	*out_saturated = saturated;
	return peak;
}

static void
log_frame(FILE *log, const struct pico4_cam_frame *f)
{
	struct timespec ts;
	clock_gettime(CLOCK_BOOTTIME, &ts);
	long long now = (long long)ts.tv_sec * 1000000000LL + ts.tv_nsec;
	struct pico4_cam_meta m = {0};
	if (!pico4_cam_meta_decode(f->meta, f->stride, &m)) {
		m.phase = 0;
	}
	unsigned saturated, peak = frame_peak(f, &saturated);
	fprintf(log, "%d %lld %.2f %lld %llu %u %u %u %u %u\n", f->slot, (long long)f->sof_boottime_ns, grid_mean(f),
	        now - (long long)f->sof_boottime_ns, (unsigned long long)f->frame_id, m.phase, m.exposure_lines,
	        m.gain, peak, saturated);
}

//! One exposure for --record-leds or --record-scene (format in the header comment); false on a write error.
static bool
record_frame(FILE *out, const struct pico4_cam_frame *f, const struct pico4_cam_meta *m)
{
	uint8_t head[48] = {'P', '4', 'L', 'F', 1, (uint8_t)f->slot};
	uint64_t id = f->frame_id, boot = (uint64_t)f->sof_boottime_ns, mono = (uint64_t)f->sof_monotonic_ns;
	uint16_t lines = (uint16_t)m->exposure_lines, gain = (uint16_t)m->gain;
	uint16_t width = (uint16_t)f->width, height = (uint16_t)f->height;
	memcpy(head + 8, &id, 8);
	memcpy(head + 16, &boot, 8);
	memcpy(head + 24, &mono, 8);
	memcpy(head + 32, &lines, 2);
	memcpy(head + 34, &gain, 2);
	memcpy(head + 36, &width, 2);
	memcpy(head + 38, &height, 2);
	if (fwrite(head, sizeof(head), 1, out) != 1) {
		return false;
	}
	uint8_t row[PICO4_CAM_WIDTH];
	for (unsigned y = 0; y < f->height; y++) {
		const uint8_t *line = (const uint8_t *)f->data + (size_t)y * f->stride;
		for (unsigned x = 0; x < f->width && x < PICO4_CAM_WIDTH; x++) {
			row[x] = (uint8_t)(sample(line, x) >> 2);
		}
		if (fwrite(row, 1, f->width, out) != f->width) {
			return false;
		}
	}
	return true;
}

static void
on_frame(struct slot_stats *st, const struct pico4_cam_frame *f, const char *dir)
{
	// The station alternates scene and controller-LED exposures; the metadata says which this is.
	struct pico4_cam_meta m;
	bool decoded = pico4_cam_meta_decode(f->meta, f->stride, &m);
	st->phases[!decoded ? 2 : m.phase == PICO4_CAM_PHASE_SCENE ? 0 : m.phase == PICO4_CAM_PHASE_LEDS ? 1 : 2]++;
	if (dir != NULL && (st->frames == g_save_frame || st->frames == g_save_frame + 1)) {
		double mean = grid_mean(f);
		bool scene = decoded ? m.phase == PICO4_CAM_PHASE_SCENE : st->frames == g_save_frame || mean > st->saved_mean;
		if (scene) {
			st->saved_mean = mean;
			st->save_error = save_frame(st, f, dir);
		}
	}
	add_stats(st, f, st->frames == 0);
	if (f->sof_boottime_ns != 0) {
		if (st->with_ts == 0) {
			st->first_ts = f->sof_boottime_ns;
			st->first_frame_id = f->frame_id;
		} else if (f->sof_boottime_ns <= st->prev_ts) {
			st->monotonic = false;
		}
		st->prev_ts = st->last_ts = f->sof_boottime_ns;
		st->last_frame_id = f->frame_id;
		st->with_ts++;
	}
	st->frames++;
}

//! The AP's counter, the QTimer that also stamps the sensor service's events (aarch64 only).
static uint64_t
qtimer_ticks(void)
{
#if defined(__aarch64__)
	uint64_t value;
	__asm__ volatile("isb; mrs %0, cntvct_el0" : "=r"(value)::"memory");
	return value;
#else
	return 0;
#endif
}

//! Appends one --record-imu record: "P4LI", "P4LH" or "P4LC", version 1, a pad byte, the length of what follows (u16).
static bool
record_imu_item(FILE *out, char kind, const void *data, uint16_t len)
{
	uint8_t head[8] = {'P', '4', 'L', (uint8_t)kind, 1, 0};
	memcpy(head + 6, &len, 2);
	return fwrite(head, sizeof(head), 1, out) == 1 && (len == 0 || fwrite(data, len, 1, out) == 1);
}

struct imu_recorder
{
	FILE *out;
	int error; //!< 0, or the negative errno that ended the recording.
	uint64_t packets;
};

static bool
suid_is(const struct pico4_ssc_sample *s, const struct pico4_ssc_suid *suid)
{
	return s->suid.low == suid->low && s->suid.high == suid->high;
}

/*!
 * --record-imu: the controllers' ble_mcu packets and the head IMU through the sensor
 * service, while the station runs. Each packet record (I) holds its event time in QTimer
 * ticks (u64) and the raw packet; each head record (H) its event time, the sensor and its
 * three values; every 100 ms a clock record (C) holds a CLOCK_MONOTONIC time in ns and the
 * QTimer read between two such readings (u64 each, the pair with the shortest window).
 */
static void *
imu_record_thread(void *arg)
{
	struct imu_recorder *rec = arg;
	struct pico4_ssc *ssc = NULL;
	// ble_mcu, accel, gyro: the head IMU keeps the DSP from slowing ble_mcu down.
	struct pico4_ssc_suid suids[3];
	static const char *const types[3] = {"ble_mcu", "accel", "gyro"};
	int r = -ETIMEDOUT;
	// The station's controllers show up in the sensor service once it runs.
	for (int attempt = 0; attempt < 10 && !g_station_stop && r < 0; attempt++) {
		r = pico4_ssc_subscribe(&ssc, types, suids, 3, 1000.0f, 1000);
	}
	if (r < 0) {
		rec->error = r;
		return NULL;
	}
	struct pico4_ssc_sample samples[32];
	struct timespec last_clock = {0};
	while (!g_station_stop && rec->error == 0) {
		struct timespec now;
		clock_gettime(CLOCK_MONOTONIC, &now);
		if ((now.tv_sec - last_clock.tv_sec) * 1000000000LL + (now.tv_nsec - last_clock.tv_nsec) > 100000000LL) {
			uint64_t pair[2] = {0, 0};
			int64_t best = INT64_MAX;
			for (int i = 0; i < 8; i++) {
				struct timespec a, b;
				clock_gettime(CLOCK_MONOTONIC, &a);
				uint64_t ticks = qtimer_ticks();
				clock_gettime(CLOCK_MONOTONIC, &b);
				int64_t ta = a.tv_sec * 1000000000LL + a.tv_nsec, tb = b.tv_sec * 1000000000LL + b.tv_nsec;
				if (tb - ta < best) {
					best = tb - ta;
					pair[0] = (uint64_t)(ta + (tb - ta) / 2);
					pair[1] = ticks;
				}
			}
			if (!record_imu_item(rec->out, 'C', pair, sizeof(pair))) {
				rec->error = -EIO;
			}
			last_clock = now;
		}
		int n = pico4_ssc_read(ssc, 100, samples, (int)(sizeof(samples) / sizeof(samples[0])));
		if (n < 0) {
			rec->error = n;
			break;
		}
		for (int i = 0; i < n && rec->error == 0; i++) {
			const struct pico4_ssc_sample *s = &samples[i];
			if (suid_is(s, &suids[0])) {
				uint8_t item[8 + PICO4_SSC_RAW_MAX];
				memcpy(item, &s->ticks, 8);
				memcpy(item + 8, s->raw, s->raw_len);
				if (!record_imu_item(rec->out, 'I', item, (uint16_t)(8 + s->raw_len))) {
					rec->error = -EIO;
				}
				rec->packets++;
			} else if ((suid_is(s, &suids[1]) || suid_is(s, &suids[2])) && s->count >= 3) {
				uint8_t item[8 + 1 + 12];
				memcpy(item, &s->ticks, 8);
				item[8] = suid_is(s, &suids[1]) ? 0 : 1;
				memcpy(item + 9, s->values, 12);
				if (!record_imu_item(rec->out, 'H', item, sizeof(item))) {
					rec->error = -EIO;
				}
			}
		}
	}
	for (int i = 0; i < 3; i++) {
		pico4_ssc_disable(ssc, &suids[i]);
	}
	pico4_ssc_close(&ssc);
	return NULL;
}

//! Acknowledges the station's frames while the capture runs.
static void *
station_thread(void *arg)
{
	while (!g_station_stop) {
		if (pico4_station_poll(arg, 100) < 0) {
			break;
		}
	}
	return NULL;
}

static int
cmd_capture(struct pico4_cam_config *cfg, int argc, char **argv)
{
	const char *dir = ".";
	FILE *stats_log = NULL, *led_record = NULL, *scene_record = NULL;
	struct imu_recorder imu = {0};
	pthread_t imu_thread;
	bool imu_running = false;
	unsigned long exposure = 0, gain = 16;
	unsigned long frames = 30, timeout = 3000;
	struct slot_stats st[PICO4_CAM_NUM];
	struct pico4_cam *cam = NULL;
	struct sigaction sa;
	const char *error = NULL;
	bool ok = true, first = true, use_station = true, station_reset = true, station_started = false;
	struct pico4_station *station = NULL;
	pthread_t poller;
	bool polling = false;
	int r;

	for (int i = 0; i < argc; i++) {
		const char *a = argv[i], *v = i + 1 < argc ? argv[i + 1] : NULL;
		unsigned long n;
		if (strcmp(a, "--no-station") == 0) {
			use_station = false;
			continue;
		}
		if (strcmp(a, "--no-station-reset") == 0) {
			station_reset = false;
			continue;
		}
		if (v == NULL) {
			usage(stderr);
			return 2;
		}
		i++;
		if (strcmp(a, "--slots") == 0 && parse_slots(v, &cfg->slots)) {
		} else if (strcmp(a, "--frames") == 0 && parse_uint(v, 1000000, &frames) && frames > 0) {
		} else if (strcmp(a, "--out") == 0) {
			dir = v;
		} else if (strcmp(a, "--exposure") == 0 && parse_uint(v, 1700, &exposure)) {
		} else if (strcmp(a, "--gain") == 0 && parse_uint(v, 1023, &gain) && gain >= 16) {
		} else if (strcmp(a, "--stats-log") == 0) {
			if (stats_log != NULL) {
				fclose(stats_log);
			}
			stats_log = fopen(v, "we");
			if (stats_log == NULL) {
				fprintf(stderr, "pico4-cam: cannot write %s: %s\n", v, strerror(errno));
				return 1;
			}
		} else if (strcmp(a, "--record-imu") == 0) {
			if (imu.out != NULL) {
				fclose(imu.out);
			}
			imu.out = fopen(v, "we");
			if (imu.out == NULL) {
				fprintf(stderr, "pico4-cam: cannot write %s: %s\n", v, strerror(errno));
				return 1;
			}
		} else if (strcmp(a, "--record-leds") == 0 || strcmp(a, "--record-scene") == 0) {
			FILE **out = a[9] == 'l' ? &led_record : &scene_record;
			if (*out != NULL) {
				fclose(*out);
			}
			*out = fopen(v, "we");
			if (*out == NULL) {
				fprintf(stderr, "pico4-cam: cannot write %s: %s\n", v, strerror(errno));
				return 1;
			}
		} else if (strcmp(a, "--ring") == 0 && parse_uint(v, PICO4_CAM_RING_MAX, &n)) {
			cfg->ring = (unsigned)n;
		} else if (strcmp(a, "--timeout") == 0 && parse_uint(v, 600000, &timeout)) {
		} else {
			fprintf(stderr, "pico4-cam: bad option %s %s\n", a, v);
			usage(stderr);
			return 2;
		}
	}
	if (mkdir(dir, 0755) < 0 && errno != EEXIST) {
		fprintf(stderr, "pico4-cam: cannot create %s: %s\n", dir, strerror(errno));
		return 1;
	}

	memset(st, 0, sizeof(st));
	for (int s = 0; s < PICO4_CAM_NUM; s++) {
		st[s].monotonic = true;
		st[s].min = ~0u;
	}
	// No SA_RESTART: a signal ends the wait in pico4_cam_next() and the capture tears down.
	memset(&sa, 0, sizeof(sa));
	sa.sa_handler = on_signal;
	sigemptyset(&sa.sa_mask);
	sigaction(SIGINT, &sa, NULL);
	sigaction(SIGTERM, &sa, NULL);

	if (use_station) {
		r = pico4_station_open(&station, NULL);
		if (r == 0 && station_reset) {
			r = pico4_station_reset(station);
		}
		if (r < 0) {
			fprintf(stderr, "pico4-cam: station: %s\n", strerror(-r));
			error = "station unavailable";
		}
	}
	r = error != NULL ? -ENODEV : pico4_cam_open(&cam, cfg);
	if (r == 0) {
		r = pico4_cam_start(cam);
		if (r < 0) {
			error = "start failed";
		}
	} else if (error == NULL) {
		error = "open failed";
	}
	if (r == 0 && station != NULL) {
		r = pico4_station_camera_start(station);
		if (r < 0) {
			fprintf(stderr, "pico4-cam: station camera start: %s\n", strerror(-r));
			error = "station refused frame sync (are all four sensors streaming?)";
		} else {
			station_started = true;
			if (exposure != 0) {
				g_save_frame = 10;
				pico4_station_camera_set_exposure(station, (unsigned)exposure, (unsigned)gain);
			}
			polling = pthread_create(&poller, NULL, station_thread, station) == 0;
		}
	}
	// With --no-station someone else runs the station; its controllers still reach the sensor service.
	if (r == 0 && imu.out != NULL) {
		imu_running = pthread_create(&imu_thread, NULL, imu_record_thread, &imu) == 0;
	}
	while (r == 0 && !g_stop) {
		struct pico4_cam_frame f;
		bool all = true;
		for (int s = 0; s < PICO4_CAM_NUM; s++) {
			all = all && (!((cfg->slots >> s) & 1) || st[s].frames >= frames);
		}
		if (all) {
			break;
		}
		r = pico4_cam_next(cam, &f, (int)timeout);
		if (r == -EINTR) {
			r = 0;
			continue; // g_stop is set
		}
		if (r < 0) {
			error = r == -ETIMEDOUT ? "timeout waiting for frames" : "capture failed";
			break;
		}
		if (st[f.slot].frames < frames) {
			on_frame(&st[f.slot], &f, dir);
			if (stats_log != NULL) {
				log_frame(stats_log, &f);
			}
			struct pico4_cam_meta m;
			bool decoded = (led_record != NULL || scene_record != NULL) && pico4_cam_meta_decode(f.meta, f.stride, &m);
			if (decoded && led_record != NULL && m.phase == PICO4_CAM_PHASE_LEDS && !record_frame(led_record, &f, &m)) {
				error = "writing the LED record failed";
				r = -EIO;
			}
			if (decoded && scene_record != NULL && m.phase == PICO4_CAM_PHASE_SCENE &&
			    !record_frame(scene_record, &f, &m)) {
				error = "writing the scene record failed";
				r = -EIO;
			}
		}
	}
	if (g_stop && error == NULL) {
		error = "interrupted";
	}

	g_station_stop = 1;
	if (polling) {
		pthread_join(poller, NULL);
	}
	if (imu_running) {
		pthread_join(imu_thread, NULL);
	}
	if (station_started) {
		pico4_station_camera_stop(station);
	}
	printf("{\n  \"station\": %s,\n  \"frames_requested\": %lu,\n", use_station ? "true" : "false", frames);
	printf("  \"slots\": [");
	for (int s = 0; s < PICO4_CAM_NUM; s++) {
		const struct slot_stats *t = &st[s];
		struct pico4_cam_slot_info si;
		double span, fps = 0.0;
		if (!((cfg->slots >> s) & 1)) {
			continue;
		}
		memset(&si, 0, sizeof(si));
		if (cam != NULL) {
			pico4_cam_get_slot_info(cam, s, &si);
		}
		span = (double)(t->last_ts - t->first_ts) / 1e9;
		if (t->with_ts > 1 && span > 0.0) {
			fps = (t->with_ts - 1) / span;
		}
		printf("%s\n    {\"slot\": %d, \"frames\": %u, \"errors\": %llu, \"with_timestamp\": %u, ", first ? "" : ",",
		       s, t->frames, (unsigned long long)si.errors, t->with_ts);
		printf("\"measured_fps\": %.3f, \"timestamps_monotonic\": %s, ", fps, t->monotonic ? "true" : "false");
		printf("\"sof_frames_spanned\": %llu, ",
		       (unsigned long long)(t->with_ts > 0 ? t->last_frame_id - t->first_frame_id + 1 : 0));
		printf("\"first_sof_boottime_ns\": %lld, ", (long long)t->first_ts);
		if (t->count > 0) {
			printf("\"min\": %u, \"mean\": %.2f, \"max\": %u, ", t->min, (double)t->sum / (double)t->count, t->max);
		} else {
			printf("\"min\": null, \"mean\": null, \"max\": null, ");
		}
		printf("\"pgm\": ");
		json_str(t->saved ? t->pgm : "");
		printf(", \"raw\": ");
		json_str(t->saved ? t->raw : "");
		if (t->save_error != 0) {
			printf(", \"save_error\": ");
			json_str(strerror(-t->save_error));
		}
		printf(", \"phases\": {\"scene\": %u, \"leds\": %u, \"other\": %u}", t->phases[0], t->phases[1],
		       t->phases[2]);
		printf(", \"csiphy\": %d, \"lane_assign\": %u}", si.phy, si.lane_assign);
		ok = ok && t->frames >= frames && t->monotonic && t->save_error == 0;
		first = false;
	}
	ok = ok && error == NULL;
	printf("\n  ],\n");
	if (imu.out != NULL) {
		printf("  \"imu_packets\": %llu,\n  \"imu_error\": ", (unsigned long long)imu.packets);
		if (imu.error != 0 || !imu_running) {
			json_str(imu_running ? strerror(-imu.error) : "not started");
		} else {
			printf("null");
		}
		printf(",\n");
		ok = ok && imu_running && imu.error == 0 && imu.packets > 0;
	}
	printf("  \"error\": ");
	if (error != NULL) {
		json_str(error);
	} else {
		printf("null");
	}
	printf(",\n  \"ok\": %s\n}\n", ok ? "true" : "false");
	fflush(stdout);
	pico4_cam_close(cam);
	pico4_station_close(station);
	if (stats_log != NULL) {
		fclose(stats_log);
	}
	if (led_record != NULL && fclose(led_record) != 0) {
		ok = false;
	}
	if (scene_record != NULL && fclose(scene_record) != 0) {
		ok = false;
	}
	if (imu.out != NULL && fclose(imu.out) != 0) {
		ok = false;
	}
	return ok ? 0 : 1;
}

//! Mean of one Bayer channel (row and column parity) over a sparse grid, black level removed.
static double
bayer_mean(const struct pico4_cam_frame *f, unsigned dy, unsigned dx)
{
	double sum = 0;
	unsigned n = 0;
	for (unsigned y = dy; y < f->height; y += 32) {
		const uint16_t *row = (const uint16_t *)((const uint8_t *)f->data + (size_t)y * f->stride);
		for (unsigned x = dx; x < f->width; x += 32) {
			sum += (double)row[x] - PICO4_CAM_RGB_BLACK;
			n++;
		}
	}
	return n > 0 ? sum / n : 0;
}

/*!
 * The RGB frame as DIR/rgb.raw16 (the buffer, little-endian 16-bit RGGB samples) and
 * DIR/rgb.ppm: each 2x2 cell one pixel (1164x874), black level removed, the channels
 * balanced to their means (grey world) and scaled so their mean is a quarter of white.
 */
static int
save_rgb(const struct pico4_cam_frame *f, const char *dir, const double mean[4])
{
	char path[512];
	snprintf(path, sizeof(path), "%s/rgb.raw16", dir);
	FILE *out = fopen(path, "wb");
	if (out == NULL || fwrite(f->data, 1, f->size, out) != f->size) {
		if (out != NULL) {
			fclose(out);
		}
		return -1;
	}
	fclose(out);
	double g = (mean[1] + mean[2]) / 2;
	double scale[3] = {mean[0] > 1 ? g / mean[0] : 1, 1, mean[3] > 1 ? g / mean[3] : 1};
	double level = g > 1 ? 255.0 / 4 / g : 1;
	snprintf(path, sizeof(path), "%s/rgb.ppm", dir);
	out = fopen(path, "wb");
	if (out == NULL) {
		return -1;
	}
	unsigned w = f->width / 2, h = f->height / 2;
	fprintf(out, "P6\n%u %u\n255\n", w, h);
	uint8_t *line = malloc((size_t)w * 3);
	for (unsigned y = 0; y < h && line != NULL; y++) {
		const uint16_t *r0 = (const uint16_t *)((const uint8_t *)f->data + (size_t)(2 * y) * f->stride);
		const uint16_t *r1 = (const uint16_t *)((const uint8_t *)f->data + (size_t)(2 * y + 1) * f->stride);
		for (unsigned x = 0; x < w; x++) {
			double v[3] = {r0[2 * x] - (double)PICO4_CAM_RGB_BLACK,
			               (r0[2 * x + 1] + r1[2 * x]) / 2.0 - PICO4_CAM_RGB_BLACK,
			               r1[2 * x + 1] - (double)PICO4_CAM_RGB_BLACK};
			for (int c = 0; c < 3; c++) {
				double o = v[c] * scale[c] * level;
				line[3 * x + c] = (uint8_t)(o < 0 ? 0 : o > 255 ? 255 : o);
			}
		}
		fwrite(line, 3, w, out);
	}
	free(line);
	fclose(out);
	return 0;
}

/*!
 * pico4-cam rgb [--frames N] [--out DIR] [--exposure LINES] [--gain G]: the IMX471 alone
 * (slot 4, free-running, no station), N frames (default 100); a JSON summary with the
 * frame rate from start-of-frame timestamps and the mean of each Bayer channel; the last
 * frame saved as DIR/rgb.raw16 and DIR/rgb.ppm when --out is given.
 */
static int
cmd_rgb(struct pico4_cam_config *cfg, int argc, char **argv)
{
	unsigned long frames = 100, v, step_lines = 0, step_gain = 16;
	const char *dir = NULL;
	for (int i = 0; i < argc; i++) {
		const char *a = argv[i], *val = i + 1 < argc ? argv[i + 1] : NULL;
		if (strcmp(a, "--frames") == 0 && val != NULL && parse_uint(val, 100000, &frames) && frames > 0) {
			i++;
		} else if (strcmp(a, "--out") == 0 && val != NULL) {
			dir = val;
			i++;
		} else if (strcmp(a, "--exposure") == 0 && val != NULL && parse_uint(val, PICO4_CAM_RGB_MAX_LINES, &v)) {
			cfg->rgb_exposure_lines = (unsigned)v;
			i++;
		} else if (strcmp(a, "--gain") == 0 && val != NULL && parse_uint(val, 256, &v)) {
			cfg->rgb_gain = (unsigned)v;
			i++;
		} else if (strcmp(a, "--step-exposure") == 0 && val != NULL &&
		           parse_uint(val, PICO4_CAM_RGB_MAX_LINES, &step_lines)) {
			i++;
		} else if (strcmp(a, "--step-gain") == 0 && val != NULL && parse_uint(val, 256, &step_gain)) {
			i++;
		} else {
			usage(stderr);
			return 2;
		}
	}
	cfg->slots = 1u << PICO4_CAM_RGB_SLOT;
	struct pico4_cam *cam;
	int r = pico4_cam_open(&cam, cfg);
	if (r < 0) {
		fprintf(stderr, "pico4-cam: open: %s\n", strerror(-r));
		return 1;
	}
	struct sigaction sa = {.sa_handler = on_signal};
	sigaction(SIGINT, &sa, NULL);
	sigaction(SIGTERM, &sa, NULL);
	r = pico4_cam_start(cam);
	unsigned long got = 0, timeouts = 0, backwards = 0;
	int64_t first_sof = 0, last_sof = 0;
	// Each half's mean level and brightest sample (sparse grid), to see an exposure step take effect.
	double half_mean[2] = {0}, half_peak[2] = {0};
	unsigned long half_n[2] = {0};
	int step_rc = 0;
	double mean[4] = {0};
	struct pico4_cam_frame f;
	while (r == 0 && got < frames && !g_stop) {
		int n = pico4_cam_next(cam, &f, 1000);
		if (n == -ETIMEDOUT) {
			if (++timeouts >= 3) {
				break;
			}
			continue;
		}
		if (n < 0) {
			r = n;
			break;
		}
		got++;
		unsigned half = got > frames / 2;
		if (half == 1 && half_n[1] == 0 && step_lines != 0) {
			step_rc = pico4_cam_set_rgb_exposure(cam, (unsigned)step_lines, (unsigned)step_gain);
		}
		double fm = 0, fp = 0;
		unsigned fn = 0;
		for (unsigned y = 0; y < f.height; y += 16) {
			const uint16_t *row = (const uint16_t *)((const uint8_t *)f.data + (size_t)y * f.stride);
			for (unsigned x = 0; x < f.width; x += 16) {
				fm += row[x];
				fp = row[x] > fp ? row[x] : fp;
				fn++;
			}
		}
		half_mean[half] += fn > 0 ? fm / fn : 0;
		half_peak[half] += fp;
		half_n[half]++;
		if (f.sof_boottime_ns != 0) {
			backwards += last_sof != 0 && f.sof_boottime_ns <= last_sof;
			first_sof = first_sof != 0 ? first_sof : f.sof_boottime_ns;
			last_sof = f.sof_boottime_ns;
		}
		if (got == frames || g_stop) {
			for (int c = 0; c < 4; c++) {
				mean[c] = bayer_mean(&f, (unsigned)c / 2, (unsigned)c % 2);
			}
			if (dir != NULL) {
				mkdir(dir, 0755);
				if (save_rgb(&f, dir, mean) < 0) {
					fprintf(stderr, "pico4-cam: could not save to %s\n", dir);
				}
			}
		}
		pico4_cam_release(cam, &f);
	}
	double fps = got > 1 && last_sof > first_sof ? (got - 1) * 1e9 / (double)(last_sof - first_sof) : 0;
	for (int h = 0; h < 2; h++) {
		half_mean[h] = half_n[h] > 0 ? half_mean[h] / half_n[h] : 0;
		half_peak[h] = half_n[h] > 0 ? half_peak[h] / half_n[h] : 0;
	}
	printf("{\"frames\": %lu, \"fps\": %.2f, \"timestamps_backwards\": %lu, \"timeouts\": %lu, "
	       "\"mean_r\": %.1f, \"mean_gr\": %.1f, \"mean_gb\": %.1f, \"mean_b\": %.1f, "
	       "\"first_half\": {\"mean\": %.2f, \"peak\": %.1f}, \"second_half\": {\"mean\": %.2f, \"peak\": %.1f}, "
	       "\"step_rc\": %d}\n",
	       got, fps, backwards, timeouts, mean[0], mean[1], mean[2], mean[3], half_mean[0], half_peak[0],
	       half_mean[1], half_peak[1], step_rc);
	pico4_cam_stop(cam);
	pico4_cam_close(cam);
	return r == 0 && got == frames && backwards == 0 ? 0 : 1;
}

int
main(int argc, char **argv)
{
	struct pico4_cam_config cfg;
	int i = 1;

	pico4_cam_config_init(&cfg);
	for (; i < argc && argv[i][0] == '-'; i++) {
		if (strcmp(argv[i], "-v") == 0) {
			cfg.verbose = PICO4_CAM_LOG_INFO;
		} else if (strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "--help") == 0) {
			usage(stdout);
			return 0;
		} else {
			usage(stderr);
			return 2;
		}
	}
	if (i >= argc) {
		usage(stderr);
		return 2;
	}
	// The verbosity flag may also follow the command.
	for (int j = i + 1; j < argc; j++) {
		if (strcmp(argv[j], "-v") == 0) {
			cfg.verbose = PICO4_CAM_LOG_INFO;
			memmove(&argv[j], &argv[j + 1], (size_t)(argc - j - 1) * sizeof(argv[0]));
			argc--;
			j--;
		}
	}

	if (strcmp(argv[i], "capture") == 0) {
		return cmd_capture(&cfg, argc - i - 1, argv + i + 1);
	}
	if (strcmp(argv[i], "rgb") == 0) {
		return cmd_rgb(&cfg, argc - i - 1, argv + i + 1);
	}
	usage(stderr);
	return 2;
}
