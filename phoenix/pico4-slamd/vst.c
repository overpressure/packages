// Copyright 2026, overpressure contributors.
// SPDX-License-Identifier: Unlicense
/*
 * PICO OS's see-through depth mesh (vst.h), called as PICO OS's see-through service calls it on
 * the PICO 4: the mesh buffers, their lock and the ready callback before the start; then per
 * stereo pair the two grey images with the tracker's result at their time, and the tracker's map
 * points while it tracks.
 */
#include "bionic.h"
#include "pico4_slam_feed.h"
#include "vst.h"

#include <stdbool.h>

typedef int (*start_fn)(void);
typedef int (*stop_fn)(void);
typedef void (*feed_fn)(const struct vst_frames *, const struct vst_frames *, const void *);
typedef int (*points_fn)(const struct vst_slam_points *);
typedef void (*mesh_buffer_fn)(float *, uint32_t *, int32_t *, int32_t *, uint8_t *, int64_t *);
typedef void (*callback_fn)(void (*)(void));
typedef void (*lock_fn)(int (*)(void), int (*)(void));
typedef int (*version_fn)(void);

//! What the stock service gives it: about 5 MB each for the vertices and the indices.
#define MESH_BYTES (5 * 1024 * 1024)

static struct
{
	feed_fn feed_fn;
	points_fn points;
	stop_fn stop;
	bool running;
	float *vertices;
	uint32_t *indices;
	int32_t vertex_bytes, index_bytes;
	uint8_t have_read;
	int64_t mesh_ts;
	int lock;
	struct pico4_slam_feed *feed;
	uint64_t meshes, fed, point_sets, too_big;
	int64_t latency_sum, report_at;
} g_vst;

static int
mesh_lock(void)
{
	while (__atomic_exchange_n(&g_vst.lock, 1, __ATOMIC_ACQUIRE)) {
		usleep(100);
	}
	return 0;
}

static int
mesh_unlock(void)
{
	__atomic_store_n(&g_vst.lock, 0, __ATOMIC_RELEASE);
	return 0;
}

/*!
 * The pipeline's thread, after each pair: a new mesh (have_read 0) goes into the feed's free slot.
 * Every 10 s the rate and the time from the pair to its mesh go to the log.
 */
static void
mesh_ready(void)
{
	mesh_lock();
	if (g_vst.have_read != 0) {
		mesh_unlock();
		return;
	}
	g_vst.have_read = 1;
	const uint32_t n = (uint32_t)g_vst.vertex_bytes / 12, idx = (uint32_t)g_vst.index_bytes / 4;
	if (n > PICO4_SLAM_FEED_MESH_VERTICES || idx > PICO4_SLAM_FEED_MESH_INDICES) {
		g_vst.too_big++;
		mesh_unlock();
		return;
	}
	struct pico4_slam_feed *feed = g_vst.feed;
	const uint64_t w = feed->meshes_written;
	struct pico4_slam_feed_mesh *m = &feed->meshes[w % PICO4_SLAM_FEED_MESHES];
	m->ts_ns = g_vst.mesh_ts;
	m->vertex_count = n;
	m->index_count = idx;
	memcpy(m->vertices, g_vst.vertices, (size_t)n * 12);
	memcpy(m->indices, g_vst.indices, (size_t)idx * 4);
	mesh_unlock();
	__atomic_store_n(&feed->meshes_written, w + 1, __ATOMIC_RELEASE);

	const int64_t now = now_ns(CLOCK_MONOTONIC);
	g_vst.meshes++;
	g_vst.latency_sum += now - m->ts_ns;
	if (now >= g_vst.report_at) {
		fprintf(stderr,
		        "pico4-slamd: %llu depth meshes (%u vertices, %u triangles), %.0f ms after their pair; %llu pairs "
		        "and %llu point sets fed, %llu meshes too big\n",
		        (unsigned long long)g_vst.meshes, n, idx / 3,
		        g_vst.meshes ? (double)g_vst.latency_sum / (double)g_vst.meshes / 1e6 : 0.0,
		        (unsigned long long)g_vst.fed, (unsigned long long)g_vst.point_sets, (unsigned long long)g_vst.too_big);
		fflush(stderr);
		g_vst.report_at = now + 10000000000LL;
	}
}

int
vst_start(struct pico4_slam_feed *feed)
{
	g_vst.feed = feed;
	void *lib = dlopen("libphotoid.so", RTLD_NOW);
	if (lib == 0) {
		fprintf(stderr, "pico4-slamd: no see-through depth mesh: %s\n", dlerror());
		return -1;
	}
	start_fn start = (start_fn)dlsym(lib, "StartVstPipeline");
	mesh_buffer_fn mesh_buffer = (mesh_buffer_fn)dlsym(lib, "SetMeshBuffer");
	callback_fn callback = (callback_fn)dlsym(lib, "RegisterVSTCallback");
	lock_fn lock = (lock_fn)dlsym(lib, "SetMeshLockFunc");
	version_fn version = (version_fn)dlsym(lib, "LibraryVersion");
	g_vst.feed_fn = (feed_fn)dlsym(lib, "FeedMrFrameData");
	g_vst.points = (points_fn)dlsym(lib, "SetSlamPointsCloud");
	g_vst.stop = (stop_fn)dlsym(lib, "StopVstPipeline");
	if (!start || !mesh_buffer || !callback || !lock || !g_vst.feed_fn || !g_vst.points || !g_vst.stop) {
		fprintf(stderr, "pico4-slamd: libphotoid.so lacks a function of its interface\n");
		return -1;
	}
	g_vst.vertices = malloc(MESH_BYTES);
	g_vst.indices = malloc(MESH_BYTES);
	if (g_vst.vertices == 0 || g_vst.indices == 0) {
		return -1;
	}
	// The buffers first: the pipeline writes its first mesh through them without a check.
	mesh_buffer(g_vst.vertices, g_vst.indices, &g_vst.vertex_bytes, &g_vst.index_bytes, &g_vst.have_read,
	            &g_vst.mesh_ts);
	lock(mesh_lock, mesh_unlock);
	callback(mesh_ready);
	int ok = start();
	fprintf(stderr, "pico4-slamd: see-through depth mesh (libphotoid %d): %s\n", version ? version() : -1,
	        ok == 1 ? "started" : "did not start");
	g_vst.running = ok == 1;
	return g_vst.running ? 0 : -1;
}

/*
 * libphotoid takes the head and the map points in a world with y up and turns them a quarter
 * turn about z into its own, libSlam's x-up world, the inverse of the turn on its mesh output.
 * Measured: fed libSlam's x-up result unchanged, a head turned 39 degrees to the right put the
 * hemisphere 39 degrees below the horizon instead. So both go in turned: (x, y, z) -> (-y, x, z),
 * the orientation's vector part alike.
 */
static void
to_y_up(double v[3])
{
	const double x = v[0];
	v[0] = -v[1];
	v[1] = x;
}

void
vst_feed(const uint8_t *left, const uint8_t *right, int width, int height, int64_t ts, int64_t exposure_ns,
         const uint8_t result[VST_HEAD_DATA_SIZE])
{
	if (!g_vst.running) {
		return;
	}
	static uint8_t head[VST_HEAD_DATA_SIZE];
	memcpy(head, result, sizeof(head));
	double position[3], orientation[4];
	memcpy(position, head + 0x08, sizeof(position));
	memcpy(orientation, head + 0x20, sizeof(orientation)); // w, x, y, z
	to_y_up(position);
	to_y_up(orientation + 1);
	memcpy(head + 0x08, position, sizeof(position));
	memcpy(head + 0x20, orientation, sizeof(orientation));
	static struct vst_frame grey[2];
	for (int i = 0; i < 2; i++) {
		struct vst_frame *f = &grey[i];
		memset(f, 0, sizeof(*f));
		f->data = i == 0 ? left : right;
		f->width = width;
		f->height = height;
		f->stride = width;
		f->format = 0;
		f->timestamp = ts;
		f->exposure = exposure_ns;
		f->scale = 1.0f;
		f->camera_id = i;
		f->head = head;
	}
	struct vst_frames pair = {grey, grey + 2, grey + 2};
	struct vst_frames none = {0, 0, 0};
	g_vst.feed_fn(&pair, &none, 0);
	g_vst.fed++;
}

void
vst_points(const struct vst_slam_points *points)
{
	if (g_vst.running && points->count >= 1 && points->count <= VST_SLAM_POINTS) {
		static struct vst_slam_points turned;
		memcpy(&turned, points, sizeof(turned));
		for (int32_t i = 0; i < turned.count; i++) {
			float *p = turned.pts[i].xyz;
			const float x = p[0];
			p[0] = -p[1];
			p[1] = x;
		}
		g_vst.points(&turned);
		g_vst.point_sets++;
	}
}

void
vst_stop(void)
{
	if (g_vst.running) {
		g_vst.running = false;
		g_vst.stop();
	}
}
