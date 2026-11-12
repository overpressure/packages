// Copyright 2026, overpressure contributors.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  PICO OS's aim-to-grip transform of the PICO 4 controllers. Header-only, so that the
 *         package build tests it without Monado's libraries.
 *
 * PICO OS publishes a controller's tracking pose as its aim pose and derives the grip pose from
 * it. GetControllerTrackingDataGripPose (libtrackingclient.pxr.so) calls
 * pvr::TrackingClient::transform_pose(hand, in, out) (0x41e0c), which composes
 *
 *     q_grip = q_aim * q_T,   p_grip = p_aim + R(q_aim) p_T
 *
 * with T the block of the hand (0 left, 1 right) that pvr::TrackingClient::getAim2GripTransform
 * (0x420d4) fetched from the tracking service: 14 doubles, per hand a position x, y, z and a
 * quaternion w, x, y, z, stored into six_dof +0x08..+0x38 (position, then w at +0x20). The
 * service's pvr::ConfigManager::getAim2GripTrans (libpvrtrackingservice.so 0xa5820) parses them
 * from sys_tracking_aim_to_grip_transform or falls back to built-in values, and logs them as
 * "trans left ..., right ...". tooling/aimgrip.py extracts these into
 * calibration/controllers/aim-to-grip.json. Stock moves the pose only; here the grip's linear
 * velocity also gets the lever arm, omega x R(q_aim) p_T.
 *
 * @ingroup drv_pico4
 */

#pragma once

#include "xrt/xrt_defines.h"

#include <cjson/cJSON.h>

#include <math.h>
#include <stdbool.h>
#include <stdio.h>

//! The grip point lies within this distance of the aim point (metres).
#define PICO4_AIMGRIP_MAX_OFFSET_M 0.2
//! Stored quaternions are unit quaternions; anything further off is not one.
#define PICO4_AIMGRIP_UNIT_TOLERANCE 1e-3

static inline struct xrt_quat
pico4_aimgrip_qmul(struct xrt_quat a, struct xrt_quat b)
{
	return (struct xrt_quat){
	    a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
	    a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
	    a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
	    a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
	};
}

static inline struct xrt_vec3
pico4_aimgrip_cross(struct xrt_vec3 a, struct xrt_vec3 b)
{
	return (struct xrt_vec3){a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}

//! v rotated by the unit quaternion q: v + 2w (u x v) + 2 u x (u x v), u the vector part.
static inline struct xrt_vec3
pico4_aimgrip_rotate(struct xrt_quat q, struct xrt_vec3 v)
{
	struct xrt_vec3 u = {q.x, q.y, q.z};
	struct xrt_vec3 c = pico4_aimgrip_cross(u, v);
	struct xrt_vec3 cc = pico4_aimgrip_cross(u, c);
	return (struct xrt_vec3){v.x + 2 * (q.w * c.x + cc.x), v.y + 2 * (q.w * c.y + cc.y),
	                         v.z + 2 * (q.w * c.z + cc.z)};
}

//! Up to @p count finite numbers of the JSON array @p array into @p out; false unless exactly @p count.
static inline bool
pico4_aimgrip_numbers(const cJSON *array, double *out, int count)
{
	if (!cJSON_IsArray(array) || cJSON_GetArraySize(array) != count) {
		return false;
	}
	int i = 0;
	const cJSON *item;
	cJSON_ArrayForEach(item, array)
	{
		if (!cJSON_IsNumber(item) || !isfinite(item->valuedouble)) {
			return false;
		}
		out[i++] = item->valuedouble;
	}
	return i == count;
}

/*!
 * The transform of one hand from the text of calibration/controllers/aim-to-grip.json
 * ({"left"|"right": {"position": [x, y, z], "orientation": [w, x, y, z]}}), checked like the
 * extraction checks it; false with the reason in @p error otherwise.
 */
static inline bool
pico4_aimgrip_parse(const char *json_text, bool left, struct xrt_pose *out, char *error, size_t error_len)
{
	cJSON *root = cJSON_Parse(json_text);
	const cJSON *hand = cJSON_GetObjectItemCaseSensitive(root, left ? "left" : "right");
	double p[3], q[4];
	bool ok = pico4_aimgrip_numbers(cJSON_GetObjectItemCaseSensitive(hand, "position"), p, 3) &&
	          pico4_aimgrip_numbers(cJSON_GetObjectItemCaseSensitive(hand, "orientation"), q, 4);
	cJSON_Delete(root);
	if (!ok) {
		snprintf(error, error_len, "no finite %s position and orientation", left ? "left" : "right");
		return false;
	}
	double norm = sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
	if (fabs(norm - 1.0) > PICO4_AIMGRIP_UNIT_TOLERANCE) {
		snprintf(error, error_len, "the %s orientation is not a unit quaternion", left ? "left" : "right");
		return false;
	}
	double offset = sqrt(p[0] * p[0] + p[1] * p[1] + p[2] * p[2]);
	if (offset >= PICO4_AIMGRIP_MAX_OFFSET_M) {
		snprintf(error, error_len, "the %s grip is %.3f m from the aim", left ? "left" : "right", offset);
		return false;
	}
	out->position = (struct xrt_vec3){(float)p[0], (float)p[1], (float)p[2]};
	out->orientation = (struct xrt_quat){(float)(q[1] / norm), (float)(q[2] / norm), (float)(q[3] / norm),
	                                     (float)(q[0] / norm)};
	return true;
}

/*!
 * The grip relation from the aim relation @p aim: pose aim * @p aim_to_grip; with valid
 * velocities, the linear velocity of the grip point adds omega x R(q_aim) p_T (both in the base
 * frame, as xrt_space_relation keeps them). Angular velocity and flags stay the aim's.
 */
static inline void
pico4_aimgrip_apply(const struct xrt_pose *aim_to_grip, const struct xrt_space_relation *aim,
                    struct xrt_space_relation *grip)
{
	*grip = *aim;
	struct xrt_vec3 r = pico4_aimgrip_rotate(aim->pose.orientation, aim_to_grip->position);
	grip->pose.position =
	    (struct xrt_vec3){aim->pose.position.x + r.x, aim->pose.position.y + r.y, aim->pose.position.z + r.z};
	struct xrt_quat q = pico4_aimgrip_qmul(aim->pose.orientation, aim_to_grip->orientation);
	float n = sqrtf(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
	grip->pose.orientation = (struct xrt_quat){q.x / n, q.y / n, q.z / n, q.w / n};
	const enum xrt_space_relation_flags velocities =
	    XRT_SPACE_RELATION_LINEAR_VELOCITY_VALID_BIT | XRT_SPACE_RELATION_ANGULAR_VELOCITY_VALID_BIT;
	if ((aim->relation_flags & velocities) == velocities) {
		struct xrt_vec3 lever = pico4_aimgrip_cross(aim->angular_velocity, r);
		grip->linear_velocity.x += lever.x;
		grip->linear_velocity.y += lever.y;
		grip->linear_velocity.z += lever.z;
	}
}
