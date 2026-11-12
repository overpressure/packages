// Copyright 2026, overpressure contributors.
// SPDX-License-Identifier: BSL-1.0
/*
 * Unit test of the head pose filter with synthetic motion: a 1 kHz IMU with
 * noise and bias, and tracker poses at 25 Hz that arrive 120 ms after their
 * images, with 2 mm, 3 cm/s and 0.2 degree noise. The output must follow the
 * motion within millimetres and never change its velocity abruptly (a visible
 * jump); a tracker that jumps, runs away or starts a new map must not move it.
 */

#include "pico4_fusion.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

static int failures;

#define EXPECT(cond)                                                                                                   \
	do {                                                                                                           \
		if (!(cond)) {                                                                                         \
			fprintf(stderr, "%s:%d: expected %s\n", __FILE__, __LINE__, #cond);                            \
			failures++;                                                                                    \
		}                                                                                                      \
	} while (0)

#define MS 1000000LL
#define LATENCY_MS 120
#define NEVER 99.0

static const struct pico4_fusion_config config = {
    .lever = {-0.052f, -0.025f, 0.04f},
    .position_s = 0.15f,
    .velocity_s = 0.15f,
    .orientation_s = 0.5f,
    .smoothing_s = 0.05f,
    .gate_m = 0.15f,
    .gate_rad = 0.17f,
    .persist = 5,
    .stale_ns = 500 * MS,
    .runaway_window_ns = 500 * MS,
    .runaway_mps = 0.5f,
    .max_speed = 4.0f,
    .still_mps = 0.15f,
    .runaway_poses = 3,
    .relock_poses = 5,
};

static uint32_t seed = 12345;

//! Roughly normal noise with standard deviation @p sigma.
static float
noise(float sigma)
{
	float sum = 0.0f;
	for (int i = 0; i < 12; i++) {
		seed = seed * 1664525u + 1013904223u;
		sum += (float)(seed >> 8) / 16777216.0f;
	}
	return (sum - 6.0f) * sigma;
}

static struct xrt_quat
yaw_q(float yaw)
{
	return (struct xrt_quat){0, sinf(yaw / 2), 0, cosf(yaw / 2)};
}

static struct xrt_quat
mul(struct xrt_quat a, struct xrt_quat b)
{
	return (struct xrt_quat){
	    a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
	    a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
	    a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
	    a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
	};
}

static struct xrt_vec3
rotate(struct xrt_quat q, struct xrt_vec3 v)
{
	struct xrt_vec3 t = {2 * (q.y * v.z - q.z * v.y), 2 * (q.z * v.x - q.x * v.z), 2 * (q.x * v.y - q.y * v.x)};
	return (struct xrt_vec3){v.x + q.w * t.x + (q.y * t.z - q.z * t.y), v.y + q.w * t.y + (q.z * t.x - q.x * t.z),
	                         v.z + q.w * t.z + (q.x * t.y - q.y * t.x)};
}

//! The true motion: swaying along x and bobbing along y, the head turned by a fixed yaw.
struct truth
{
	struct xrt_vec3 p, v, a;
	struct xrt_quat q;
};

static struct truth
truth_at(double t, float amplitude)
{
	const double w = 6.283185307179586 * 0.7; // 0.7 Hz
	struct truth s;
	s.p = (struct xrt_vec3){(float)(1.0 + amplitude * sin(w * t)), (float)(1.6 + 0.3 * amplitude * sin(2 * w * t)),
	                        -0.5f};
	s.v = (struct xrt_vec3){(float)(amplitude * w * cos(w * t)), (float)(0.6 * amplitude * w * cos(2 * w * t)), 0};
	s.a = (struct xrt_vec3){(float)(-amplitude * w * w * sin(w * t)),
	                        (float)(-1.2 * amplitude * w * w * sin(2 * w * t)), 0};
	s.q = yaw_q(0.4f);
	return s;
}

//! What happens to the tracker; times in seconds of image time.
struct scenario
{
	double seconds;
	float amplitude; //!< Of the true motion, m.
	float yaw_3dof;  //!< The 3DoF fusion's heading off the tracker's.
	double drop_from, drop_to; //!< No poses for these images.
	double bad_from, bad_to;   //!< Poses 0.5 m off.
	double drift_from;         //!< From here the tracker accelerates at 2 m/s^2 that the IMU never feels.
	double creep_from;         //!< From here the tracker moves at a steady 0.3 m/s that the IMU never feels.
	double reset_at;           //!< The tracker is reset: a new map, turned 70 degrees and moved.
};

struct run
{
	float rms_error;   //!< Of the output against the truth, after the first second.
	float final_error; //!< At the end.
	float max_kink;    //!< Largest change of the output's velocity in one step, m/s: a visible jump.
	float max_turn;    //!< Largest rotation in one step, degrees (the true motion has none).
	bool tracking;     //!< At the end.
	int results[7];
};

static struct run
simulate(const struct scenario *sc)
{
	struct pico4_fusion *f = calloc(1, sizeof(*f));
	pico4_fusion_init(f, &config);
	struct run r = {0};
	double sum = 0;
	int n = 0;
	struct xrt_vec3 last[2] = {{0}};
	int have_last = 0;
	struct xrt_quat last_q = XRT_QUAT_IDENTITY;
	const struct xrt_quat new_map = yaw_q(1.22f);
	const struct xrt_vec3 new_origin = {-3.0f, 0.2f, 2.0f};

	for (int64_t ms = 0; ms < (int64_t)(sc->seconds * 1000); ms++) {
		int64_t ts = ms * MS;
		struct truth s = truth_at(ms / 1000.0, sc->amplitude);
		struct xrt_quat q3 = mul(yaw_q(sc->yaw_3dof), s.q);
		struct xrt_vec3 force = s.a;
		force.y += 9.80665f;
		struct xrt_quat inv = {-s.q.x, -s.q.y, -s.q.z, s.q.w};
		struct xrt_vec3 accel = rotate(inv, force);
		accel.x += noise(0.02f) + 0.03f; // noise and a bias
		accel.y += noise(0.02f);
		accel.z += noise(0.02f);
		struct xrt_vec3 gyro = {0, 0, 0};
		pico4_fusion_imu(f, ts, &q3, &gyro, &accel);
		if (ms == (int64_t)(sc->reset_at * 1000)) {
			pico4_fusion_tracker_reset(f, ts);
		}

		// A pose for the images taken LATENCY_MS ago, every 40 ms.
		int64_t taken = ms - LATENCY_MS;
		double taken_s = taken / 1000.0;
		if (taken >= 0 && taken % 40 == 0 && !(taken_s >= sc->drop_from && taken_s < sc->drop_to)) {
			struct truth then = truth_at(taken_s, sc->amplitude);
			// Tracker orientations scatter by about 0.2 degrees.
			float tilt = noise(0.0035f);
			struct xrt_quat wobble = {sinf(tilt / 2), 0, 0, cosf(tilt / 2)};
			struct pico4_fusion_slam m = {.ts_ns = taken * MS, .orientation = mul(then.q, wobble)};
			m.position = (struct xrt_vec3){then.p.x + noise(0.002f), then.p.y + noise(0.002f),
			                               then.p.z + noise(0.002f)};
			m.velocity = (struct xrt_vec3){then.v.x + noise(0.03f), then.v.y + noise(0.03f),
			                               then.v.z + noise(0.03f)};
			if (taken_s >= sc->bad_from && taken_s < sc->bad_to) {
				m.position.x += 0.5f;
			}
			if (taken_s >= sc->reset_at) {
				m.orientation = mul(new_map, m.orientation);
				m.position = rotate(new_map, m.position);
				m.position = (struct xrt_vec3){m.position.x + new_origin.x, m.position.y + new_origin.y,
				                               m.position.z + new_origin.z};
				m.velocity = rotate(new_map, m.velocity);
			} else if (taken_s >= sc->drift_from) {
				float d = (float)(taken_s - sc->drift_from);
				m.position.x += d * d; // 2 m/s^2
				m.velocity.x += 2.0f * d;
			} else if (taken_s >= sc->creep_from) {
				m.position.z += 0.3f * (float)(taken_s - sc->creep_from);
				m.velocity.z += 0.3f;
			}
			r.results[pico4_fusion_slam(f, &m)]++;
		}

		struct xrt_space_relation rel;
		pico4_fusion_relation(f, &rel);
		if (!(rel.relation_flags & XRT_SPACE_RELATION_POSITION_VALID_BIT)) {
			continue;
		}
		if (have_last == 2 && ms > 1000) {
			struct xrt_vec3 p = rel.pose.position;
			float kx = p.x - 2 * last[1].x + last[0].x, ky = p.y - 2 * last[1].y + last[0].y,
			      kz = p.z - 2 * last[1].z + last[0].z;
			float kink = sqrtf(kx * kx + ky * ky + kz * kz) * 1000.0f; // per millisecond step
			r.max_kink = kink > r.max_kink ? kink : r.max_kink;
			// The vector part of last^-1 * now: acos near 1 drowns in float rounding.
			struct xrt_quat c = rel.pose.orientation, l = last_q;
			double vx = l.w * c.x - l.x * c.w - l.y * c.z + l.z * c.y;
			double vy = l.w * c.y + l.x * c.z - l.y * c.w - l.z * c.x;
			double vz = l.w * c.z - l.x * c.y + l.y * c.x - l.z * c.w;
			float turn = (float)(2.0 * asin(fmin(1.0, sqrt(vx * vx + vy * vy + vz * vz))) * 57.29578);
			r.max_turn = turn > r.max_turn ? turn : r.max_turn;
		}
		last_q = rel.pose.orientation;
		last[0] = last[1];
		last[1] = rel.pose.position;
		have_last = have_last < 2 ? have_last + 1 : 2;
		if (ms > 1000) {
			// The IMU's position: the output less the lever turned into the world.
			struct xrt_vec3 lw = rotate(rel.pose.orientation, config.lever);
			float ex = rel.pose.position.x - lw.x - s.p.x, ey = rel.pose.position.y - lw.y - s.p.y,
			      ez = rel.pose.position.z - lw.z - s.p.z;
			sum += ex * ex + ey * ey + ez * ez;
			n++;
			r.final_error = sqrtf(ex * ex + ey * ey + ez * ez);
			float dq = fabsf(rel.pose.orientation.y * s.q.y + rel.pose.orientation.w * s.q.w);
			EXPECT(dq > 0.99996f); // within about 1 degree of the true heading
			if (dq <= 0.99996f) {
				break;
			}
		}
	}
	r.rms_error = n > 0 ? sqrtf((float)(sum / n)) : -1.0f;
	r.tracking = pico4_fusion_tracking(f);
	free(f);
	return r;
}

static const struct scenario plain = {
    .seconds = 5.0,
    .drop_from = NEVER,
    .bad_from = NEVER,
    .drift_from = NEVER,
    .creep_from = NEVER,
    .reset_at = NEVER,
};

static void
test_still(void)
{
	struct run r = simulate(&plain);
	EXPECT(r.results[PICO4_FUSION_STARTED] == 1);
	EXPECT(r.results[PICO4_FUSION_OUTLIER] == 0);
	EXPECT(r.results[PICO4_FUSION_RUNAWAY] == 0);
	EXPECT(r.rms_error >= 0 && r.rms_error < 0.005f);
	EXPECT(r.max_kink < 0.01f);
	EXPECT(r.max_turn < 0.002f); // no steps from the tracker's scatter
	printf("still: error %.2f mm rms, largest velocity step %.4f m/s, largest turn %.4f deg\n", r.rms_error * 1e3,
	       r.max_kink, r.max_turn);
}

static void
test_moving_with_heading_offset(void)
{
	// 25 cm swaying at 0.7 Hz (1.1 m/s at the centre); the 3DoF heading 30 degrees off.
	struct scenario sc = plain;
	sc.seconds = 8.0;
	sc.amplitude = 0.25f;
	sc.yaw_3dof = 0.52f;
	struct run r = simulate(&sc);
	EXPECT(r.results[PICO4_FUSION_OUTLIER] == 0);
	EXPECT(r.results[PICO4_FUSION_RUNAWAY] == 0);
	EXPECT(r.results[PICO4_FUSION_ANCHORED] == 0);
	EXPECT(r.rms_error >= 0 && r.rms_error < 0.006f);
	EXPECT(r.max_kink < 0.03f); // the true motion changes velocity by up to 0.012 m/s per step
	printf("moving: error %.2f mm rms, largest velocity step %.4f m/s\n", r.rms_error * 1e3, r.max_kink);
}

static void
test_gap(void)
{
	// Moving, no tracker pose for 300 ms: the IMU carries the position.
	struct scenario sc = plain;
	sc.amplitude = 0.25f;
	sc.drop_from = 2.0;
	sc.drop_to = 2.3;
	struct run r = simulate(&sc);
	EXPECT(r.results[PICO4_FUSION_RESET] == 0);
	EXPECT(r.results[PICO4_FUSION_RUNAWAY] == 0);
	EXPECT(r.rms_error >= 0 && r.rms_error < 0.008f);
	EXPECT(r.max_kink < 0.03f);
	printf("gap: error %.2f mm rms, largest velocity step %.4f m/s\n", r.rms_error * 1e3, r.max_kink);
}

static void
test_jumps(void)
{
	// Two poses 0.5 m off are skipped.
	struct scenario sc = plain;
	sc.seconds = 4.0;
	sc.amplitude = 0.1f;
	sc.bad_from = 2.0;
	sc.bad_to = 2.07;
	struct run r = simulate(&sc);
	EXPECT(r.results[PICO4_FUSION_OUTLIER] == 2);
	EXPECT(r.results[PICO4_FUSION_ANCHORED] == 0);
	EXPECT(r.max_kink < 0.03f);
	// Off for a second, the fifth re-anchors the tracker's world, twice: the view stays.
	sc.bad_to = 3.0;
	r = simulate(&sc);
	EXPECT(r.results[PICO4_FUSION_OUTLIER] == 8);
	EXPECT(r.results[PICO4_FUSION_ANCHORED] == 2);
	EXPECT(r.max_kink < 0.03f);
	EXPECT(r.final_error < 0.02f);
	printf("jumps: final error %.1f mm, largest velocity step %.4f m/s\n", r.final_error * 1e3, r.max_kink);
}

static void
test_runaway_and_reset(void)
{
	// The tracker drifts away from 2 s on; the user stands still. The position is held.
	struct scenario sc = plain;
	sc.seconds = 3.5;
	sc.drift_from = 2.0;
	struct run r = simulate(&sc);
	EXPECT(r.results[PICO4_FUSION_RUNAWAY] == 1);
	EXPECT(r.results[PICO4_FUSION_ANCHORED] == 0);
	EXPECT(!r.tracking);
	EXPECT(r.final_error < 0.08f);
	EXPECT(r.max_kink < 0.05f);
	printf("runaway: held %.1f mm from the truth, largest velocity step %.4f m/s\n", r.final_error * 1e3,
	       r.max_kink);

	// Reset at 4 s: a new map turned 70 degrees; the filter re-anchors and tracks the motion again.
	sc.seconds = 7.0;
	sc.amplitude = 0.0f;
	sc.reset_at = 4.0;
	r = simulate(&sc);
	EXPECT(r.results[PICO4_FUSION_RUNAWAY] == 1);
	EXPECT(r.results[PICO4_FUSION_ANCHORED] == 1);
	EXPECT(r.tracking);
	EXPECT(r.final_error < 0.08f);
	EXPECT(r.max_kink < 0.05f);
	printf("reset: final error %.1f mm, largest velocity step %.4f m/s\n", r.final_error * 1e3, r.max_kink);

	// A steady creep while the IMU says the head rests: held as well.
	sc = plain;
	sc.seconds = 4.0;
	sc.creep_from = 2.0;
	r = simulate(&sc);
	EXPECT(r.results[PICO4_FUSION_RUNAWAY] == 1);
	EXPECT(!r.tracking);
	EXPECT(r.final_error < 0.08f);
	printf("creep: held %.1f mm from the truth\n", r.final_error * 1e3);
}

int
main(void)
{
	test_still();
	test_moving_with_heading_offset();
	test_gap();
	test_jumps();
	test_runaway_and_reset();
	if (failures > 0) {
		fprintf(stderr, "%d failures\n", failures);
		return 1;
	}
	printf("pico4 fusion tests passed\n");
	return 0;
}
