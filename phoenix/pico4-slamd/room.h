// Copyright 2026, overpressure contributors.
// SPDX-License-Identifier: Unlicense
/*
 * The room's boundary as PICO OS's tracker keeps it (docs/xr.md#room-setup): its boundary files
 * ("isSaveMap 1", "vertexcount 2N", then N lines "(a, b, c)", b up: the tracker world's (x, y, z)
 * is (b, -a, c); the tracker's own copy after a relocalisation has no isSaveMap line and counts
 * N), and the transform that takes this session's world back to the room's, from the
 * room's boundary and the tracker's copy of it in this session after a relocalisation. Both worlds
 * are gravity-aligned (+x up), so the transform is a turn about up and a shift. Plain C with the
 * math functions passed in: the host tests build it too.
 */
#pragma once
#include <stdint.h>

#define ROOM_MAX_POINTS 512

/*
 * A room's id, in the shape of the reference quadrilateral its maps carry as their boundary: the
 * tracker keeps a map's boundary through every save and merge, whatever it names the map, and a
 * turn and a shift keep the sides' lengths. Three sides of ROOM_SIDE_MIN + v * ROOM_SIDE_STEP
 * metres, v 12 bits each, 1.5 m below the start (tracker world: x up), in the (y, z) plane:
 * (0, 0), (l0, 0), (l0, l1), (l0 - l2, l1), centred on (l0 / 2, l1 / 2). The plain 2 m square of
 * the rooms from before the ids is ROOM_ID_SQUARE.
 */
#define ROOM_SIDE_MIN 1.5
#define ROOM_SIDE_STEP 0.0002
#define ROOM_HEIGHT -1.5
#define ROOM_ID_MASK ((1ull << 36) - 1)
#define ROOM_ID_SQUARE (2500ull << 24 | 2500ull << 12 | 2500ull)

//! The reference shape of room @p id (tracker world).
static inline void
room_shape(uint64_t id, double out[4][3])
{
	double l0 = ROOM_SIDE_MIN + (double)((id >> 24) & 0xfff) * ROOM_SIDE_STEP;
	double l1 = ROOM_SIDE_MIN + (double)((id >> 12) & 0xfff) * ROOM_SIDE_STEP;
	double l2 = ROOM_SIDE_MIN + (double)(id & 0xfff) * ROOM_SIDE_STEP;
	const double yz[4][2] = {{0, 0}, {l0, 0}, {l0, l1}, {l0 - l2, l1}};
	for (int i = 0; i < 4; i++) {
		out[i][0] = ROOM_HEIGHT;
		out[i][1] = yz[i][0] - l0 / 2;
		out[i][2] = yz[i][1] - l1 / 2;
	}
}

/*!
 * The id a reference shape carries, from its sides (in any world, turned and shifted): 0 and
 * @p out_id, or -1 if the points are not such a shape. @p sqrt_fn is libm's.
 */
static inline int
room_decode(const double (*p)[3], int n, uint64_t *out_id, double (*sqrt_fn)(double))
{
	if (n != 4) {
		return -1;
	}
	double d[4];
	for (int i = 0; i < 4; i++) {
		double dy = p[(i + 1) % 4][1] - p[i][1], dz = p[(i + 1) % 4][2] - p[i][2];
		d[i] = sqrt_fn(dy * dy + dz * dz);
	}
	uint64_t v[3];
	double l[3];
	for (int k = 0; k < 3; k++) {
		double f = (d[k] - ROOM_SIDE_MIN) / ROOM_SIDE_STEP;
		if (f < -0.25 || f > 4095.25) {
			return -1;
		}
		v[k] = (uint64_t)(f + 0.5);
		double off = f - (double)v[k];
		if (off > 0.25 || off < -0.25) { // more than 50 um off a step
			return -1;
		}
		l[k] = ROOM_SIDE_MIN + (double)v[k] * ROOM_SIDE_STEP;
	}
	double e = sqrt_fn((l[0] - l[2]) * (l[0] - l[2]) + l[1] * l[1]) - d[3];
	if (e > 0.001 || e < -0.001) {
		return -1;
	}
	*out_id = v[0] << 24 | v[1] << 12 | v[2];
	return 0;
}

/*!
 * The inverse of room_fit's transform: the session's point of room point @p w (w = R s + t, R a
 * turn of @p yaw about x), with @p c = cos(yaw) and @p sn = sin(yaw).
 */
static inline void
room_to_session(const double w[3], double c, double sn, const double t[3], double out[3])
{
	double y = w[1] - t[1], z = w[2] - t[2];
	out[0] = w[0] - t[0];
	out[1] = c * y + sn * z;
	out[2] = -sn * y + c * z;
}

//! A tracker-world point (x up) from a boundary file's (a, b, c), and back.
static inline void
room_from_file(const double abc[3], double out[3])
{
	out[0] = abc[1];
	out[1] = -abc[0];
	out[2] = abc[2];
}

static inline void
room_to_file(const double xyz[3], double out[3])
{
	out[0] = -xyz[1];
	out[1] = xyz[0];
	out[2] = xyz[2];
}

//! Parses a boundary file's points (tracker world); their count, or -1 if malformed.
static inline int
room_parse(const char *text, long len, double (*out)[3], int max)
{
	long i = 0;
	int n = 0, declared = -1;
	while (i < len) {
		long end = i;
		while (end < len && text[end] != '\n') {
			end++;
		}
		const char *l = text + i;
		long ll = end - i;
		if (ll > 12 && l[0] == 'v' && l[1] == 'e' && l[2] == 'r' && l[3] == 't') {
			declared = 0;
			for (long k = 0; k < ll; k++) {
				if (l[k] >= '0' && l[k] <= '9') {
					declared = declared * 10 + (l[k] - '0');
				}
			}
		} else if (ll > 0 && l[0] == '(') {
			if (n >= max) {
				return -1;
			}
			double v[3];
			int got = 0;
			long k = 1;
			while (k < ll && got < 3) {
				// One decimal number: sign, digits, point, digits (the files hold %f values).
				while (k < ll && !(l[k] == '-' || (l[k] >= '0' && l[k] <= '9') || l[k] == '.')) {
					k++;
				}
				if (k >= ll) {
					break;
				}
				double sign = 1, val = 0, scale = 0;
				if (l[k] == '-') {
					sign = -1;
					k++;
				}
				for (; k < ll && ((l[k] >= '0' && l[k] <= '9') || l[k] == '.'); k++) {
					if (l[k] == '.') {
						scale = 1;
					} else if (scale > 0) {
						scale *= 10;
						val += (l[k] - '0') / scale;
					} else {
						val = val * 10 + (l[k] - '0');
					}
				}
				v[got++] = sign * val;
			}
			if (got != 3) {
				return -1;
			}
			room_from_file(v, out[n++]);
		}
		i = end + 1;
	}
	return n > 0 && (declared == 2 * n || declared == n) ? n : -1;
}

/*!
 * The rigid transform (turn @p out_yaw about the up axis x, then @p out_t) that takes session
 * points @p s onto the room's @p w (same order): w = R s + t. Its RMS residual in metres, or -1
 * for fewer than 3 points. @p atan2_fn, @p sin_fn, @p cos_fn and @p sqrt_fn are libm's.
 */
static inline double
room_fit(const double (*s)[3],
         const double (*w)[3],
         int n,
         double *out_yaw,
         double out_t[3],
         double (*atan2_fn)(double, double),
         double (*sin_fn)(double),
         double (*cos_fn)(double),
         double (*sqrt_fn)(double))
{
	if (n < 3) {
		return -1;
	}
	double cs[3] = {0, 0, 0}, cw[3] = {0, 0, 0};
	for (int i = 0; i < n; i++) {
		for (int k = 0; k < 3; k++) {
			cs[k] += s[i][k] / n;
			cw[k] += w[i][k] / n;
		}
	}
	// The turn about x in the (y, z) plane: atan2 of the summed cross and dot products.
	double cross = 0, dot = 0;
	for (int i = 0; i < n; i++) {
		double sy = s[i][1] - cs[1], sz = s[i][2] - cs[2];
		double wy = w[i][1] - cw[1], wz = w[i][2] - cw[2];
		cross += sy * wz - sz * wy;
		dot += sy * wy + sz * wz;
	}
	double yaw = atan2_fn(cross, dot), c = cos_fn(yaw), sn = sin_fn(yaw);
	out_t[0] = cw[0] - cs[0];
	out_t[1] = cw[1] - (c * cs[1] - sn * cs[2]);
	out_t[2] = cw[2] - (sn * cs[1] + c * cs[2]);
	double err = 0;
	for (int i = 0; i < n; i++) {
		double p[3] = {s[i][0] + out_t[0], c * s[i][1] - sn * s[i][2] + out_t[1], sn * s[i][1] + c * s[i][2] + out_t[2]};
		for (int k = 0; k < 3; k++) {
			err += (p[k] - w[i][k]) * (p[k] - w[i][k]);
		}
	}
	*out_yaw = yaw;
	return sqrt_fn(err / n);
}
