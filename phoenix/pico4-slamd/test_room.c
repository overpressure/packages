// Copyright 2026, overpressure contributors.
// SPDX-License-Identifier: Unlicense
/*
 * room.h against hand-made boundaries: the file format both ways (b up), a malformed file, and
 * the fit of a turned and shifted copy. Built and run natively by src/stockslam/build.sh.
 */
#include "room.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

static int failures;
static const double PI = 3.14159265358979323846;

#define EXPECT(cond, ...)                                                                                              \
	do {                                                                                                           \
		if (!(cond)) {                                                                                         \
			fprintf(stderr, "test_room: " __VA_ARGS__);                                                    \
			fprintf(stderr, "\n");                                                                         \
			failures++;                                                                                    \
		}                                                                                                      \
	} while (0)

int
main(void)
{
	// As PICO OS writes it: (a, b, c) with b up, -> tracker world (b, -a, c).
	const char file[] = "isSaveMap 1\nvertexcount 6\n(1.000000, -1.500000, 2.000000)\n"
	                    "(-1.000000, -1.500000, 2.000000)\n(0.500000, -1.400000, -0.250000)\n";
	double p[ROOM_MAX_POINTS][3];
	int n = room_parse(file, (long)strlen(file), p, ROOM_MAX_POINTS);
	EXPECT(n == 3, "count %d", n);
	EXPECT(p[0][0] == -1.5 && p[0][1] == -1.0 && p[0][2] == 2.0, "point 0 %f %f %f", p[0][0], p[0][1], p[0][2]);
	EXPECT(fabs(p[2][0] + 1.4) < 1e-9 && fabs(p[2][1] + 0.5) < 1e-9 && fabs(p[2][2] + 0.25) < 1e-9, "point 2");
	double abc[3];
	room_to_file(p[1], abc);
	EXPECT(abc[0] == -1.0 && abc[1] == -1.5 && abc[2] == 2.0, "back to the file");
	const char wrong[] = "isSaveMap 1\nvertexcount 8\n(1, 2, 3)\n";
	EXPECT(room_parse(wrong, (long)strlen(wrong), p, ROOM_MAX_POINTS) == -1, "count mismatch");
	EXPECT(room_parse("", 0, p, ROOM_MAX_POINTS) == -1, "empty");
	// The tracker's copy after a relocalisation (stdataforalgorithm.txt) counts the points once.
	const char seen[] = "vertexcount 2\n(1.000276, -1.498821, -0.999374)\n(-0.999723, -1.498821, -1.000548)\n";
	n = room_parse(seen, (long)strlen(seen), p, ROOM_MAX_POINTS);
	EXPECT(n == 2 && fabs(p[1][1] - 0.999723) < 1e-9, "relocalised copy %d", n);

	// A square boundary, turned by 37 degrees about up and shifted: the fit gets both back.
	double s[4][3] = {{-1.5, -1, -1}, {-1.5, 1, -1}, {-1.5, 1, 1}, {-1.5, -1, 1}}, w[4][3];
	double yaw = 37 * PI / 180, t[3] = {0.1, 0.7, -0.4};
	for (int i = 0; i < 4; i++) {
		w[i][0] = s[i][0] + t[0];
		w[i][1] = cos(yaw) * s[i][1] - sin(yaw) * s[i][2] + t[1];
		w[i][2] = sin(yaw) * s[i][1] + cos(yaw) * s[i][2] + t[2];
	}
	double got_yaw, got_t[3];
	double rms = room_fit((const double(*)[3])s, (const double(*)[3])w, 4, &got_yaw, got_t, atan2, sin, cos, sqrt);
	EXPECT(rms >= 0 && rms < 1e-9, "rms %g", rms);
	EXPECT(fabs(got_yaw - yaw) < 1e-9, "yaw %f", got_yaw * 180 / PI);
	EXPECT(fabs(got_t[0] - t[0]) < 1e-9 && fabs(got_t[1] - t[1]) < 1e-9 && fabs(got_t[2] - t[2]) < 1e-9, "shift");
	EXPECT(room_fit((const double(*)[3])s, (const double(*)[3])w, 2, &got_yaw, got_t, atan2, sin, cos, sqrt) < 0,
	       "two points");

	// End to end, with the files as libSlam writes and reads them: its saveMap writer prints a
	// world point (x, y, z) as (-y, x, z), its readers store (a, b, c) as (b, -a, c). A session
	// whose world is the room's turned by -100 degrees and shifted (as relocalisations report):
	// the tracker writes the room's square in the session's world, and the fit must give back the
	// transform that takes a session point (a head pose) into the room's world.
	double room[4][3] = {{-1.5, -1, -1}, {-1.5, 1, -1}, {-1.5, 1, 1}, {-1.5, -1, 1}};
	const double turn = -100 * PI / 180, shift[3] = {0.004, 0.075, 1.427};
	char copy[512];
	int len = snprintf(copy, sizeof(copy), "vertexcount 4\n");
	for (int i = 0; i < 4; i++) {
		// The session's point of room[i]: the inverse of w = R s + t.
		double y = room[i][1] - shift[1], z = room[i][2] - shift[2];
		double sx = room[i][0] - shift[0], sy = cos(turn) * y + sin(turn) * z, sz = -sin(turn) * y + cos(turn) * z;
		len += snprintf(copy + len, sizeof(copy) - (size_t)len, "(%f, %f, %f)\n", -sy, sx, sz);
	}
	double seen_pts[ROOM_MAX_POINTS][3];
	EXPECT(room_parse(copy, len, seen_pts, ROOM_MAX_POINTS) == 4, "libSlam's copy");
	rms = room_fit((const double(*)[3])seen_pts, (const double(*)[3])room, 4, &got_yaw, got_t, atan2, sin, cos, sqrt);
	EXPECT(rms >= 0 && rms < 1e-5, "copy rms %g", rms);
	EXPECT(fabs(got_yaw - turn) < 1e-5, "copy turn %f, not %f", got_yaw * 180 / PI, turn * 180 / PI);
	EXPECT(fabs(got_t[0] - shift[0]) < 1e-5 && fabs(got_t[1] - shift[1]) < 1e-5 && fabs(got_t[2] - shift[2]) < 1e-5,
	       "copy shift");

	// A room's id in its shape: the plain square of the rooms from before the ids, and a new id,
	// both back from a copy written as libSlam writes it after a turn and a shift.
	double sq[4][3];
	room_shape(ROOM_ID_SQUARE, sq);
	const double old[4][3] = {{-1.5, -1, -1}, {-1.5, 1, -1}, {-1.5, 1, 1}, {-1.5, -1, 1}};
	EXPECT(memcmp(sq, old, sizeof(old)) == 0, "the square's shape");
	uint64_t ids[3] = {ROOM_ID_SQUARE, 0x9a3c5f0e1ull, ROOM_ID_MASK};
	for (int k = 0; k < 3; k++) {
		double shape[4][3];
		room_shape(ids[k], shape);
		len = snprintf(copy, sizeof(copy), "vertexcount 4\n");
		for (int i = 0; i < 4; i++) {
			double s_pt[3];
			room_to_session(shape[i], cos(turn), sin(turn), shift, s_pt);
			// libSlam keeps the points as floats.
			len += snprintf(copy + len, sizeof(copy) - (size_t)len, "(%f, %f, %f)\n", (float)-s_pt[1], (float)s_pt[0],
			                (float)s_pt[2]);
		}
		uint64_t got = 0;
		EXPECT(room_parse(copy, len, seen_pts, ROOM_MAX_POINTS) == 4, "id copy");
		EXPECT(room_decode((const double(*)[3])seen_pts, 4, &got, sqrt) == 0 && got == ids[k], "id %llx back as %llx",
		       (unsigned long long)ids[k], (unsigned long long)got);
		rms = room_fit((const double(*)[3])seen_pts, (const double(*)[3])shape, 4, &got_yaw, got_t, atan2, sin, cos,
		               sqrt);
		EXPECT(rms >= 0 && rms < 1e-5 && fabs(got_yaw - turn) < 1e-5, "id shape fit %g", rms);
	}
	// Not a room shape: a boundary someone drew.
	const double drawn[4][3] = {{-1.5, 0, 0}, {-1.5, 3.1, 0.2}, {-1.5, 2.7, 2.9}, {-1.5, -0.4, 2.5}};
	uint64_t none;
	EXPECT(room_decode(drawn, 4, &none, sqrt) == -1, "a drawn boundary decoded");
	EXPECT(room_decode(drawn, 3, &none, sqrt) == -1, "three points decoded");

	if (failures == 0) {
		printf("test_room: passed\n");
	}
	return failures == 0 ? 0 : 1;
}
