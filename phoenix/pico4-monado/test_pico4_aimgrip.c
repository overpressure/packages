// Copyright 2026, overpressure contributors.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Host test of the aim-to-grip transform (pico4_aimgrip.h) with synthetic values.
 */

#include "pico4_aimgrip.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

static int failures;

#define CHECK(cond)                                                                                                    \
	do {                                                                                                           \
		if (!(cond)) {                                                                                         \
			fprintf(stderr, "%s:%d: %s\n", __FILE__, __LINE__, #cond);                                     \
			failures++;                                                                                    \
		}                                                                                                      \
	} while (0)

static bool
near(float a, float b)
{
	return fabsf(a - b) < 1e-5f;
}

static bool
near3(struct xrt_vec3 a, struct xrt_vec3 b)
{
	return near(a.x, b.x) && near(a.y, b.y) && near(a.z, b.z);
}

//! Rotation by @p deg about a unit axis.
static struct xrt_quat
about(float deg, float x, float y, float z)
{
	float h = deg * (float)M_PI / 360.0f;
	return (struct xrt_quat){x * sinf(h), y * sinf(h), z * sinf(h), cosf(h)};
}

static void
test_parse(void)
{
	// cos/sin of 15 degrees: a 30-degree turn about x, stored w first; the right hand differs.
	const char *text = "{\"left\": {\"position\": [0.001, 0.01, 0.05], \"orientation\": [0.9659258, 0.2588190, 0, 0]},"
	                   " \"right\": {\"position\": [-0.002, 0.01, 0.04], \"orientation\": [1, 0, 0, 0]}}";
	struct xrt_pose t;
	char error[128];
	CHECK(pico4_aimgrip_parse(text, true, &t, error, sizeof(error)));
	CHECK(near3(t.position, (struct xrt_vec3){0.001f, 0.01f, 0.05f}));
	CHECK(near(t.orientation.w, 0.9659258f) && near(t.orientation.x, 0.2588190f));
	CHECK(near(t.orientation.y, 0) && near(t.orientation.z, 0));
	CHECK(pico4_aimgrip_parse(text, false, &t, error, sizeof(error)));
	CHECK(near(t.position.x, -0.002f) && near(t.orientation.w, 1) && near(t.orientation.x, 0));

	const char *bad[] = {
	    "",
	    "{\"left\": {\"position\": [0, 0, 0]}}",
	    "{\"left\": {\"position\": [0, 0], \"orientation\": [1, 0, 0, 0]}}",
	    "{\"left\": {\"position\": [0, 0, 0], \"orientation\": [1, 1, 0, 0]}}",
	    "{\"left\": {\"position\": [0, 0, 0.25], \"orientation\": [1, 0, 0, 0]}}",
	    "{\"left\": {\"position\": [0, \"0\", 0], \"orientation\": [1, 0, 0, 0]}}",
	};
	for (size_t i = 0; i < sizeof(bad) / sizeof(bad[0]); i++) {
		CHECK(!pico4_aimgrip_parse(bad[i], true, &t, error, sizeof(error)));
	}
}

static void
test_apply(void)
{
	// Aim turned 90 degrees about +y (its -z points along world -x) and moving.
	struct xrt_space_relation aim = {0};
	aim.pose.position = (struct xrt_vec3){1, 2, 3};
	aim.pose.orientation = about(90, 0, 1, 0);
	aim.linear_velocity = (struct xrt_vec3){0.5f, 0, 0};
	aim.angular_velocity = (struct xrt_vec3){0, 2, 0};
	aim.relation_flags = XRT_SPACE_RELATION_ORIENTATION_VALID_BIT | XRT_SPACE_RELATION_POSITION_VALID_BIT |
	                     XRT_SPACE_RELATION_LINEAR_VELOCITY_VALID_BIT |
	                     XRT_SPACE_RELATION_ANGULAR_VELOCITY_VALID_BIT;
	// Grip 5 cm behind the aim point, its -z pitched up 30 degrees in the aim frame.
	struct xrt_pose t = {about(30, 1, 0, 0), {0, 0, 0.05f}};

	struct xrt_space_relation grip;
	pico4_aimgrip_apply(&t, &aim, &grip);
	// The lever arm R(q_aim) p_T: +z of the aim is world +x.
	CHECK(near3(grip.pose.position, (struct xrt_vec3){1.05f, 2, 3}));
	// grip = aim * T: the grip's -z is the aim's -z pitched up, then turned with the aim.
	struct xrt_vec3 fwd = pico4_aimgrip_rotate(grip.pose.orientation, (struct xrt_vec3){0, 0, -1});
	float s = sinf(30 * (float)M_PI / 180), c = cosf(30 * (float)M_PI / 180);
	CHECK(near3(fwd, (struct xrt_vec3){-c, s, 0}));
	// Back again with the inverse transform.
	struct xrt_pose inv;
	inv.orientation = (struct xrt_quat){-t.orientation.x, -t.orientation.y, -t.orientation.z, t.orientation.w};
	inv.position = pico4_aimgrip_rotate(inv.orientation, (struct xrt_vec3){0, 0, -0.05f});
	struct xrt_space_relation back;
	pico4_aimgrip_apply(&inv, &grip, &back);
	CHECK(near3(back.pose.position, aim.pose.position));
	CHECK(fabsf(fabsf(pico4_aimgrip_qmul(back.pose.orientation,
	                                     (struct xrt_quat){-aim.pose.orientation.x, -aim.pose.orientation.y,
	                                                       -aim.pose.orientation.z, aim.pose.orientation.w})
	                       .w) -
	            1) < 1e-5f);
	// omega x r = (0, 2, 0) x (0.05, 0, 0) = (0, 0, -0.1); the angular velocity is the aim's.
	CHECK(near3(grip.linear_velocity, (struct xrt_vec3){0.5f, 0, -0.1f}));
	CHECK(near3(grip.angular_velocity, aim.angular_velocity));
	CHECK(grip.relation_flags == aim.relation_flags);

	// Without a linear velocity (the 3DoF placeholder) none is made up.
	aim.relation_flags &= ~XRT_SPACE_RELATION_LINEAR_VELOCITY_VALID_BIT;
	aim.linear_velocity = (struct xrt_vec3){0, 0, 0};
	pico4_aimgrip_apply(&t, &aim, &grip);
	CHECK(near3(grip.linear_velocity, (struct xrt_vec3){0, 0, 0}));
	CHECK(!(grip.relation_flags & XRT_SPACE_RELATION_LINEAR_VELOCITY_VALID_BIT));
}

int
main(void)
{
	test_parse();
	test_apply();
	if (failures != 0) {
		fprintf(stderr, "%d failures\n", failures);
		return 1;
	}
	printf("pico4_aimgrip: ok\n");
	return 0;
}
