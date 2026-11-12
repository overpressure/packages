// Copyright 2026, overpressure contributors.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  PICO 4 passthrough frames (see pico4_passthrough.h).
 * @ingroup drv_pico4
 */

#ifndef _GNU_SOURCE
#define _GNU_SOURCE // pthread_setaffinity_np
#endif

#include "pico4_passthrough.h"
#include "pico4_roomview.h"
#include "pico4_tcam.h"

#include <errno.h>
#include <fcntl.h>
#include <math.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <time.h>
#include <unistd.h>

#define BLACK_LEVEL 16
//! Factory cameras of the forward pair: bottom left, bottom right.
static const int FACTORY[PICO4_PASSTHROUGH_CAMS] = {3, 2};
//! Who wants frames (pico4_passthrough::streaming).
#define STREAM_CLIENT 1u
#define STREAM_ROOMVIEW 2u

struct pico4_passthrough
{
	struct pico4_passthrough_info info;
	pthread_mutex_t lock;
	pthread_cond_t cond;
	unsigned streaming; //!< STREAM_* bits.
	//! The newest set, guarded by lock.
	uint16_t *raw[PICO4_PASSTHROUGH_CAMS];
	struct pico4_passthrough_frame frame;
	float gain;

	//! The room view's export (pico4_passthrough_export).
	struct pico4_roomview *roomview;
	pthread_t export_thread;
	bool exporting, export_stop;
	pico4_passthrough_pose_fn pose;
	void *pose_ctx;

	//! The room view's RGB camera (pico4_passthrough_use_rgb); the fields below guarded by lock.
	bool rgb;
	struct pico4_passthrough_info rgb_info;
	uint8_t srgb[1024];
	const uint16_t *rgb_src; //!< The frame handed over, valid while rgb_pending.
	uint32_t rgb_stride;
	int64_t rgb_sof_ns;
	uint32_t rgb_exposure_us;
	bool rgb_pending;
	uint64_t rgb_taken, rgb_converted, rgb_exposed; //!< Frames handed over, converted, exposed for.
	int64_t rgb_convert_ns; //!< Conversion time spent so far.
	float wb[3];        //!< White balance gains R, G, B (grey world, smoothed).
	double rgb_mean[3]; //!< The newest frame's channel means before the white balance.
};

//! Exposure the room view's RGB camera aims at: 18 % grey of white, in linear counts (white 959).
#define RGB_TARGET (0.18 * (1023 - PICO4_PASSTHROUGH_RGB_BLACK))
#define RGB_FLICKER_LINES 2260 //!< 1/100 s: a whole mains-flicker period at 50 Hz.
#define RGB_MAX_LINES 4510     //!< The 50 fps mode's longest exposure.
#define RGB_MAX_GAIN 256       //!< 16x analog.
#define RGB_READOUT_NS (1748ll * PICO4_PASSTHROUGH_RGB_LINE_NS)

static bool
wait_rgb(struct pico4_passthrough *p, uint64_t after, int timeout_ms, uint8_t *nv12, struct pico4_passthrough_frame *out);

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

//! a = b * c for row-major 4x4 rigid transforms.
static void
mul44(double a[16], const double b[16], const double c[16])
{
	for (int r = 0; r < 4; r++) {
		for (int col = 0; col < 4; col++) {
			double v = 0;
			for (int i = 0; i < 4; i++) {
				v += b[r * 4 + i] * c[i * 4 + col];
			}
			a[r * 4 + col] = v;
		}
	}
}

//! The inverse of a row-major rigid transform.
static void
invert_rigid(double out[16], const double t[16])
{
	memset(out, 0, 16 * sizeof(double));
	for (int r = 0; r < 3; r++) {
		for (int c = 0; c < 3; c++) {
			out[r * 4 + c] = t[c * 4 + r];
		}
		out[r * 4 + 3] = -(t[0 * 4 + r] * t[3] + t[1 * 4 + r] * t[7] + t[2 * 4 + r] * t[11]);
	}
	out[15] = 1;
}

struct pico4_passthrough *
pico4_passthrough_create(const char *calibration_dir, const double down_body[3], const double T_body_head[16])
{
	struct pico4_passthrough *p = calloc(1, sizeof(*p));
	if (p == NULL) {
		return NULL;
	}
	double T_head_body[16];
	invert_rigid(T_head_body, T_body_head);
	// OpenVR's camera axes (x right, y up, looking along -z) from the computer-vision ones.
	static const double CV_GL[16] = {1, 0, 0, 0, 0, -1, 0, 0, 0, 0, -1, 0, 0, 0, 0, 1};
	for (int i = 0; i < PICO4_PASSTHROUGH_CAMS; i++) {
		char path[512];
		snprintf(path, sizeof(path), "%s/pvr/mav0/cam%d/sensor.yaml", calibration_dir, FACTORY[i]);
		char *text = read_file(path);
		struct pico4_tcam_omni omni;
		struct pico4_tcam_kb4 kb4;
		bool ok = text != NULL && pico4_tcam_parse_yaml(text, &omni) && pico4_tcam_fit_kb4(&omni, &kb4) &&
		          omni.width == PICO4_PASSTHROUGH_SRC_W && omni.height == PICO4_PASSTHROUGH_SRC_H;
		free(text);
		if (!ok) {
			free(p->raw[0]);
			free(p);
			return NULL;
		}
		int w, h;
		p->info.turn[i] = pico4_tcam_upright(&kb4, omni.T_BS, omni.width, omni.height, down_body, &w, &h);
		p->info.width = (uint32_t)w;
		p->info.height = (uint32_t)h;
		p->info.fx[i] = kb4.fx;
		p->info.fy[i] = kb4.fy;
		p->info.cx[i] = kb4.cx;
		p->info.cy[i] = kb4.cy;
		memcpy(p->info.k[i], kb4.k, sizeof(kb4.k));
		double T_head_cam[16], T_gl[16];
		mul44(T_head_cam, T_head_body, omni.T_BS);
		mul44(T_gl, T_head_cam, CV_GL);
		memcpy(p->info.camera_to_head[i], T_gl, 12 * sizeof(double));
		p->raw[i] = calloc(PICO4_PASSTHROUGH_SRC_W * PICO4_PASSTHROUGH_SRC_H, sizeof(uint16_t));
		if (p->raw[i] == NULL) {
			free(p->raw[0]);
			free(p);
			return NULL;
		}
	}
	pthread_mutex_init(&p->lock, NULL);
	pthread_condattr_t attr;
	pthread_condattr_init(&attr);
	pthread_condattr_setclock(&attr, CLOCK_MONOTONIC);
	pthread_cond_init(&p->cond, &attr);
	pthread_condattr_destroy(&attr);
	return p;
}

void
pico4_passthrough_info(const struct pico4_passthrough *p, struct pico4_passthrough_info *out)
{
	*out = p->info;
}

static void
set_stream_bit(struct pico4_passthrough *p, unsigned bit, bool on)
{
	if (on) {
		__atomic_fetch_or(&p->streaming, bit, __ATOMIC_RELEASE);
	} else {
		__atomic_fetch_and(&p->streaming, ~bit, __ATOMIC_RELEASE);
	}
}

void
pico4_passthrough_set_streaming(struct pico4_passthrough *p, bool on)
{
	set_stream_bit(p, STREAM_CLIENT, on);
}

bool
pico4_passthrough_streaming(const struct pico4_passthrough *p)
{
	return p != NULL && __atomic_load_n(&p->streaming, __ATOMIC_ACQUIRE) != 0;
}

void
pico4_passthrough_push(struct pico4_passthrough *p,
                       const uint16_t *left,
                       const uint16_t *right,
                       uint32_t stride,
                       int64_t timestamp_ns,
                       uint32_t exposure_lines,
                       float gain)
{
	// With the RGB camera the room view takes no grey sets: only a streaming client does.
	unsigned want = __atomic_load_n(&p->streaming, __ATOMIC_ACQUIRE);
	if ((p->rgb ? (want & STREAM_CLIENT) == 0 : want == 0) || pthread_mutex_trylock(&p->lock) != 0) {
		return; // nobody wants them, or the client is converting the set before: never wait here
	}
	const uint16_t *src[PICO4_PASSTHROUGH_CAMS] = {left, right};
	for (int i = 0; i < PICO4_PASSTHROUGH_CAMS; i++) {
		for (uint32_t row = 0; row < PICO4_PASSTHROUGH_SRC_H; row++) {
			memcpy(p->raw[i] + row * PICO4_PASSTHROUGH_SRC_W, src[i] + (size_t)row * stride,
			       PICO4_PASSTHROUGH_SRC_W * sizeof(uint16_t));
		}
	}
	p->frame.sequence++;
	p->frame.timestamp_ns = timestamp_ns;
	p->frame.exposure_us = (uint32_t)((uint64_t)exposure_lines * 19069 / 1000); // 19.07 us lines
	p->gain = gain;
	pthread_cond_broadcast(&p->cond);
	pthread_mutex_unlock(&p->lock);
}

void
pico4_passthrough_convert(const uint16_t *src, uint32_t src_stride, int turn, float gain, uint8_t *dst,
                          uint32_t dst_stride)
{
	const int w = PICO4_PASSTHROUGH_SRC_W, h = PICO4_PASSTHROUGH_SRC_H;
	const int dw = turn % 2 == 0 ? w : h, dh = turn % 2 == 0 ? h : w;
	const uint32_t g = (uint32_t)(gain * 256.0f);
	for (int v = 0; v < dh; v++) {
		uint8_t *out = dst + (size_t)v * dst_stride;
		for (int u = 0; u < dw; u++) {
			// The source pixel of (u, v), as pico4_tcam_upright describes the turns.
			int su = turn == 0 ? u : turn == 1 ? v : turn == 2 ? w - 1 - u : w - 1 - v;
			int sv = turn == 0 ? v : turn == 1 ? h - 1 - u : turn == 2 ? h - 1 - v : u;
			uint32_t s = src[(size_t)sv * src_stride + (size_t)su] & 0x3ff;
			s = s > BLACK_LEVEL ? s - BLACK_LEVEL : 0;
			s = (s * g) >> 10; // 10 bits to 8, times the gain
			out[u] = (uint8_t)(s > 255 ? 255 : s);
		}
	}
}

bool
pico4_passthrough_wait(struct pico4_passthrough *p,
                       uint64_t after,
                       int timeout_ms,
                       uint8_t *y,
                       uint32_t stride,
                       struct pico4_passthrough_frame *out)
{
	struct timespec until;
	clock_gettime(CLOCK_MONOTONIC, &until);
	until.tv_sec += timeout_ms / 1000;
	until.tv_nsec += (long)(timeout_ms % 1000) * 1000000L;
	if (until.tv_nsec >= 1000000000L) {
		until.tv_sec++;
		until.tv_nsec -= 1000000000L;
	}
	pthread_mutex_lock(&p->lock);
	while (p->frame.sequence <= after) {
		if (pthread_cond_timedwait(&p->cond, &p->lock, &until) == ETIMEDOUT) {
			pthread_mutex_unlock(&p->lock);
			return false;
		}
	}
	for (int i = 0; i < PICO4_PASSTHROUGH_CAMS; i++) {
		pico4_passthrough_convert(p->raw[i], PICO4_PASSTHROUGH_SRC_W, p->info.turn[i], p->gain,
		                          y + (size_t)i * p->info.height * stride, stride);
	}
	*out = p->frame;
	pthread_mutex_unlock(&p->lock);
	return true;
}

static int64_t
monotonic_ns(void)
{
	struct timespec ts;
	clock_gettime(CLOCK_MONOTONIC, &ts);
	return (int64_t)ts.tv_sec * 1000000000ll + ts.tv_nsec;
}

//! Streams the forward pair into the room view's ring while its producer asks for frames.
static void *
export_run(void *arg)
{
	struct pico4_passthrough *p = arg;
	struct pico4_roomview *rv = p->roomview;
	uint64_t after = 0;
	if (p->rgb) {
		// An RGB frame (8 MB of samples) converts in about 10 ms on the Cortex-A77 cores 4-6 and
		// in 22 ms on the little cores at their usual clock (measured): longer than a frame's 20 ms.
		cpu_set_t big;
		CPU_ZERO(&big);
		for (int cpu = 4; cpu <= 6; cpu++) {
			CPU_SET(cpu, &big);
		}
		pthread_setaffinity_np(pthread_self(), sizeof(big), &big);
	}
	while (!__atomic_load_n(&p->export_stop, __ATOMIC_ACQUIRE)) {
		int64_t seen = __atomic_load_n(&rv->producer_ns, __ATOMIC_ACQUIRE);
		bool want = __atomic_load_n(&rv->requested, __ATOMIC_ACQUIRE) != 0 &&
		            monotonic_ns() - seen < PICO4_ROOMVIEW_PRODUCER_TIMEOUT_NS;
		set_stream_bit(p, STREAM_ROOMVIEW, want);
		if (!want) {
			usleep(50 * 1000);
			continue;
		}
		uint64_t n = __atomic_load_n(&rv->frames_written, __ATOMIC_RELAXED);
		struct pico4_roomview_frame *f = &rv->frames[n % PICO4_ROOMVIEW_FRAMES];
		__atomic_store_n(&f->sequence, 0, __ATOMIC_RELEASE);
		__atomic_thread_fence(__ATOMIC_SEQ_CST);
		struct pico4_passthrough_frame frame;
		if (p->rgb ? !wait_rgb(p, after, 200, f->pixels, &frame)
		           : !pico4_passthrough_wait(p, after, 200, f->pixels, p->info.width, &frame)) {
			continue;
		}
		after = frame.sequence;
		f->timestamp_ns = frame.timestamp_ns;
		f->exposure_us = frame.exposure_us;
		f->pose_valid = p->pose != NULL && p->pose(p->pose_ctx, frame.timestamp_ns, f->tracking_from_head);
		__atomic_store_n(&f->sequence, frame.sequence, __ATOMIC_RELEASE);
		__atomic_store_n(&rv->frames_written, n + 1, __ATOMIC_RELEASE);
	}
	set_stream_bit(p, STREAM_ROOMVIEW, false);
	return NULL;
}

bool
pico4_passthrough_export(struct pico4_passthrough *p, const char *path, pico4_passthrough_pose_fn pose, void *ctx)
{
	if (p->exporting) {
		return true;
	}
	// Grey: the frames stack the two cameras (pico4_passthrough_wait writes left above right).
	// Colour: one NV12 image of the RGB camera, the same for both cameras.
	const struct pico4_passthrough_info *info = p->rgb ? &p->rgb_info : &p->info;
	if ((size_t)info->width * info->height > PICO4_ROOMVIEW_PIXELS && !p->rgb) {
		return false;
	}
	int fd = open(path, O_RDWR | O_CREAT | O_CLOEXEC, 0600);
	if (fd < 0) {
		return false;
	}
	if (ftruncate(fd, sizeof(struct pico4_roomview)) != 0) {
		close(fd);
		return false;
	}
	struct pico4_roomview *rv = mmap(NULL, sizeof(*rv), PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
	close(fd);
	if (rv == MAP_FAILED) {
		return false;
	}
	// Kept in place across starts (pico4-roomview keeps its mapping); a new generation each start.
	uint32_t generation = rv->magic == PICO4_ROOMVIEW_MAGIC ? rv->generation + 1 : 1;
	__atomic_store_n(&rv->magic, 0, __ATOMIC_RELEASE);
	rv->version = 2;
	rv->generation = generation;
	rv->format = p->rgb ? PICO4_ROOMVIEW_FORMAT_NV12 : PICO4_ROOMVIEW_FORMAT_GREY;
	rv->width = info->width;
	rv->height = info->height;
	for (int i = 0; i < PICO4_ROOMVIEW_CAMS; i++) {
		rv->fx[i] = info->fx[i];
		rv->fy[i] = info->fy[i];
		rv->cx[i] = info->cx[i];
		rv->cy[i] = info->cy[i];
		memcpy(rv->k[i], info->k[i], sizeof(rv->k[i]));
		memcpy(rv->camera_to_head[i], info->camera_to_head[i], sizeof(rv->camera_to_head[i]));
	}
	for (int i = 0; i < PICO4_ROOMVIEW_FRAMES; i++) {
		rv->frames[i].sequence = 0;
	}
	__atomic_store_n(&rv->magic, PICO4_ROOMVIEW_MAGIC, __ATOMIC_RELEASE);
	p->roomview = rv;
	p->pose = pose;
	p->pose_ctx = ctx;
	p->export_stop = false;
	if (pthread_create(&p->export_thread, NULL, export_run, p) != 0) {
		munmap(rv, sizeof(*rv));
		p->roomview = NULL;
		return false;
	}
	p->exporting = true;
	return true;
}

void
pico4_passthrough_srgb_table(uint8_t table[1024], double white)
{
	for (int i = 0; i < 1024; i++) {
		double v = i / white;
		v = v >= 1 ? 1 : v <= 0.0031308 ? 12.92 * v : 1.055 * pow(v, 1 / 2.4) - 0.055;
		table[i] = (uint8_t)lround(v * 255);
	}
}

bool
pico4_passthrough_use_rgb(struct pico4_passthrough *p, const char *calibration_dir, const double T_body_head[16])
{
	char path[512];
	snprintf(path, sizeof(path), "%s/pvr/mav0/cam4/sensor.yaml", calibration_dir);
	char *text = read_file(path);
	struct pico4_tcam_omni omni;
	struct pico4_tcam_kb4 kb4;
	bool ok = text != NULL && pico4_tcam_parse_yaml(text, &omni) && pico4_tcam_fit_kb4(&omni, &kb4) &&
	          omni.width == PICO4_PASSTHROUGH_RGB_SRC_W && omni.height == PICO4_PASSTHROUGH_RGB_SRC_H;
	free(text);
	if (!ok || p->exporting) {
		return false;
	}
	double T_head_body[16], T_head_cam[16], T_gl[16];
	static const double CV_GL[16] = {1, 0, 0, 0, 0, -1, 0, 0, 0, 0, -1, 0, 0, 0, 0, 1};
	invert_rigid(T_head_body, T_body_head);
	mul44(T_head_cam, T_head_body, omni.T_BS);
	mul44(T_gl, T_head_cam, CV_GL);
	struct pico4_passthrough_info *i = &p->rgb_info;
	memset(i, 0, sizeof(*i));
	i->width = PICO4_PASSTHROUGH_RGB_W;
	i->height = PICO4_PASSTHROUGH_RGB_H;
	for (int c = 0; c < PICO4_PASSTHROUGH_CAMS; c++) {
		// One pixel per 2x2 cell: cell x covers sensor columns 2x and 2x + 1, centred on 2x + 0.5.
		i->fx[c] = kb4.fx / 2;
		i->fy[c] = kb4.fy / 2;
		i->cx[c] = (kb4.cx - 0.5) / 2;
		i->cy[c] = (kb4.cy - 0.5) / 2;
		memcpy(i->k[c], kb4.k, sizeof(kb4.k));
		memcpy(i->camera_to_head[c], T_gl, 12 * sizeof(double));
	}
	pico4_passthrough_srgb_table(p->srgb, 1023 - PICO4_PASSTHROUGH_RGB_BLACK);
	p->wb[0] = p->wb[1] = p->wb[2] = 1;
	p->rgb = true;
	return true;
}

bool
pico4_passthrough_rgb_info(const struct pico4_passthrough *p, struct pico4_passthrough_info *out)
{
	if (p == NULL || !p->rgb) {
		return false;
	}
	*out = p->rgb_info;
	return true;
}

bool
pico4_passthrough_push_rgb(struct pico4_passthrough *p,
                           const uint16_t *rggb,
                           uint32_t stride,
                           int64_t sof_ns,
                           uint32_t exposure_us)
{
	if (p == NULL || !p->rgb || !pico4_passthrough_streaming(p)) {
		return false;
	}
	// The export thread holds the lock only for its bookkeeping, never while it converts.
	pthread_mutex_lock(&p->lock);
	bool taken = !p->rgb_pending;
	if (taken) {
		p->rgb_src = rggb;
		p->rgb_stride = stride;
		p->rgb_sof_ns = sof_ns;
		p->rgb_exposure_us = exposure_us;
		p->rgb_taken++;
		__atomic_store_n(&p->rgb_pending, true, __ATOMIC_RELEASE);
		pthread_cond_broadcast(&p->cond);
	}
	pthread_mutex_unlock(&p->lock);
	return taken;
}

bool
pico4_passthrough_rgb_busy(struct pico4_passthrough *p)
{
	return p != NULL && __atomic_load_n(&p->rgb_pending, __ATOMIC_ACQUIRE);
}

void
pico4_passthrough_rgb_counts(struct pico4_passthrough *p, uint64_t *taken, uint64_t *converted, int64_t *convert_ns)
{
	pthread_mutex_lock(&p->lock);
	*taken = p->rgb_taken;
	*converted = p->rgb_converted;
	*convert_ns = p->rgb_convert_ns;
	pthread_mutex_unlock(&p->lock);
}

void
pico4_passthrough_convert_rgb(const uint16_t *src,
                              uint32_t src_stride,
                              const float wb[3],
                              const uint8_t srgb[1024],
                              uint8_t *dst,
                              uint32_t width,
                              uint32_t height,
                              double mean[3])
{
	// White balance in 8.8 fixed point. Two output rows at a time: each row's cells into sRGB
	// R, G, B and luma, then one row of Cb, Cr from the 2x2 blocks of both. The means come from
	// every fourth row, which is enough for the exposure and the white balance.
	const int32_t gr = (int32_t)(wb[0] * 256), gg = (int32_t)(wb[1] * 256), gb = (int32_t)(wb[2] * 256);
	uint64_t sum[3] = {0}, counted = 0;
	uint8_t rows[2][PICO4_PASSTHROUGH_RGB_W * 3];
	uint8_t *uv = dst + (size_t)width * height;
	for (uint32_t y = 0; y + 1 < height && width <= PICO4_PASSTHROUGH_RGB_W; y += 2) {
		for (uint32_t dy = 0; dy < 2; dy++) {
			const uint16_t *r0 = src + (size_t)(2 * (y + dy)) * src_stride;
			const uint16_t *r1 = r0 + src_stride;
			uint8_t *rgb = rows[dy], *luma = dst + (size_t)(y + dy) * width;
			const bool sample = ((y + dy) & 3) == 0;
			uint32_t sr = 0, sg = 0, sb = 0;
			for (uint32_t x = 0; x < width; x++) {
				int32_t r = (int32_t)r0[2 * x] - PICO4_PASSTHROUGH_RGB_BLACK;
				int32_t g = (((int32_t)r0[2 * x + 1] + (int32_t)r1[2 * x]) >> 1) - PICO4_PASSTHROUGH_RGB_BLACK;
				int32_t b = (int32_t)r1[2 * x + 1] - PICO4_PASSTHROUGH_RGB_BLACK;
				r = r < 0 ? 0 : r;
				g = g < 0 ? 0 : g;
				b = b < 0 ? 0 : b;
				sr += (uint32_t)r;
				sg += (uint32_t)g;
				sb += (uint32_t)b;
				r = (r * gr) >> 8;
				g = (g * gg) >> 8;
				b = (b * gb) >> 8;
				const uint8_t R = srgb[r > 1023 ? 1023 : r], G = srgb[g > 1023 ? 1023 : g], B = srgb[b > 1023 ? 1023 : b];
				rgb[3 * x] = R;
				rgb[3 * x + 1] = G;
				rgb[3 * x + 2] = B;
				luma[x] = (uint8_t)((77 * R + 150 * G + 29 * B + 128) >> 8);
			}
			if (sample) {
				sum[0] += sr;
				sum[1] += sg;
				sum[2] += sb;
				counted += width;
			}
		}
		uint8_t *o = uv + (size_t)(y / 2) * width;
		for (uint32_t x = 0; x + 1 < width; x += 2) {
			const uint8_t *a = rows[0] + 3 * x, *b = rows[1] + 3 * x;
			int32_t r = a[0] + a[3] + b[0] + b[3], g = a[1] + a[4] + b[1] + b[4], bl = a[2] + a[5] + b[2] + b[5];
			// Full-range BT.601 on the 2x2 block's mean (the sums are four times it).
			int32_t cb = 128 + ((-43 * r - 85 * g + 128 * bl + 512) >> 10);
			int32_t cr = 128 + ((128 * r - 107 * g - 21 * bl + 512) >> 10);
			o[x] = (uint8_t)(cb < 0 ? 0 : cb > 255 ? 255 : cb);
			o[x + 1] = (uint8_t)(cr < 0 ? 0 : cr > 255 ? 255 : cr);
		}
	}
	for (int c = 0; c < 3; c++) {
		mean[c] = counted > 0 ? (double)sum[c] / counted : 0;
	}
}

bool
pico4_passthrough_rgb_exposure(struct pico4_passthrough *p,
                               unsigned lines,
                               unsigned gain,
                               unsigned *out_lines,
                               unsigned *out_gain)
{
	if (p == NULL || !p->rgb || pthread_mutex_lock(&p->lock) != 0) {
		return false;
	}
	bool fresh = p->rgb_converted > p->rgb_exposed;
	p->rgb_exposed = p->rgb_converted;
	double y = 0.299 * p->rgb_mean[0] * p->wb[0] + 0.587 * p->rgb_mean[1] * p->wb[1] +
	           0.114 * p->rgb_mean[2] * p->wb[2];
	pthread_mutex_unlock(&p->lock);
	if (!fresh || gain < 16) {
		return false;
	}
	// Half of the step to the target per frame, at most a factor of two either way.
	double ratio = sqrt(RGB_TARGET / (y > 0.5 ? y : 0.5));
	ratio = ratio > 2 ? 2 : ratio < 0.5 ? 0.5 : ratio;
	double exposure = lines * (gain / 16.0) * ratio; // in lines at 1x
	// The light first goes into a flicker-free 1/100 s, then into gain, then into a longer exposure.
	double l = exposure < RGB_FLICKER_LINES ? exposure : RGB_FLICKER_LINES;
	double g = 16 * exposure / (l < 4 ? 4 : l);
	if (g > RGB_MAX_GAIN) {
		g = RGB_MAX_GAIN;
		l = exposure / (RGB_MAX_GAIN / 16.0);
		l = l > RGB_MAX_LINES ? RGB_MAX_LINES : l;
	}
	unsigned nl = (unsigned)lround(l < 4 ? 4 : l), ng = (unsigned)lround(g < 16 ? 16 : g);
	// Within 5 % of the current exposure: no change, so that the sensor is not written every frame.
	double now = lines * (gain / 16.0), next = nl * (ng / 16.0);
	if (fabs(next - now) <= 0.05 * now) {
		return false;
	}
	*out_lines = nl;
	*out_gain = ng;
	return true;
}

/*!
 * Waits up to @p timeout_ms for an RGB frame newer than @p after, converts it into @p nv12 and
 * lets the camera thread have its buffer back; then the grey-world white balance follows it.
 */
static bool
wait_rgb(struct pico4_passthrough *p, uint64_t after, int timeout_ms, uint8_t *nv12, struct pico4_passthrough_frame *out)
{
	struct timespec until;
	clock_gettime(CLOCK_MONOTONIC, &until);
	until.tv_sec += timeout_ms / 1000;
	until.tv_nsec += (long)(timeout_ms % 1000) * 1000000L;
	if (until.tv_nsec >= 1000000000L) {
		until.tv_sec++;
		until.tv_nsec -= 1000000000L;
	}
	pthread_mutex_lock(&p->lock);
	while (!p->rgb_pending || p->rgb_taken <= after) {
		if (pthread_cond_timedwait(&p->cond, &p->lock, &until) == ETIMEDOUT) {
			pthread_mutex_unlock(&p->lock);
			return false;
		}
	}
	const uint16_t *src = p->rgb_src;
	const uint32_t stride = p->rgb_stride;
	const float wb[3] = {p->wb[0], p->wb[1], p->wb[2]};
	out->sequence = p->rgb_taken;
	out->exposure_us = p->rgb_exposure_us;
	// The middle row's exposure middle: rows are read out after the start of frame, each
	// exposed for exposure_us before it.
	out->timestamp_ns = p->rgb_sof_ns + RGB_READOUT_NS / 2 - (int64_t)p->rgb_exposure_us * 500;
	pthread_mutex_unlock(&p->lock);

	double mean[3];
	int64_t t0 = monotonic_ns();
	pico4_passthrough_convert_rgb(src, stride, wb, p->srgb, nv12, PICO4_PASSTHROUGH_RGB_W, PICO4_PASSTHROUGH_RGB_H,
	                              mean);

	pthread_mutex_lock(&p->lock);
	p->rgb_convert_ns += monotonic_ns() - t0;
	memcpy(p->rgb_mean, mean, sizeof(mean));
	p->rgb_converted = out->sequence;
	// Grey world: the gains that make the frame's mean grey, followed slowly.
	if (mean[0] > 1 && mean[1] > 1 && mean[2] > 1) {
		const double target[3] = {mean[1] / mean[0], 1, mean[1] / mean[2]};
		for (int c = 0; c < 3; c += 2) {
			double t = target[c] < 0.5 ? 0.5 : target[c] > 4 ? 4 : target[c];
			p->wb[c] += (float)(0.05 * (t - p->wb[c]));
		}
	}
	__atomic_store_n(&p->rgb_pending, false, __ATOMIC_RELEASE);
	pthread_mutex_unlock(&p->lock);
	return true;
}

void
pico4_passthrough_destroy(struct pico4_passthrough **pp)
{
	struct pico4_passthrough *p = *pp;
	if (p == NULL) {
		return;
	}
	if (p->exporting) {
		__atomic_store_n(&p->export_stop, true, __ATOMIC_RELEASE);
		pthread_join(p->export_thread, NULL);
		munmap(p->roomview, sizeof(*p->roomview));
	}
	pthread_mutex_destroy(&p->lock);
	pthread_cond_destroy(&p->cond);
	for (int i = 0; i < PICO4_PASSTHROUGH_CAMS; i++) {
		free(p->raw[i]);
	}
	free(p);
	*pp = NULL;
}
