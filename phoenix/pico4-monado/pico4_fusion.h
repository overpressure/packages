// Copyright 2026, overpressure contributors.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  PICO 4 head pose: the 1 kHz IMU propagates it, delayed SLAM poses correct it.
 *
 * The visual-inertial tracker (Basalt) delivers 25 poses a second, each about
 * 100 ms after its images were taken. Extrapolating that far from its pose
 * turns the noise of a 25 Hz velocity into jumps of centimetres. This filter
 * keeps its own state at the head IMU's rate instead:
 *
 * - Orientation: the 3DoF fusion's, turned into the world by an offset (mostly
 *   heading). It turns towards the tracker's newest at every IMU step, slowly:
 *   following each tracker pose at once made the view step 25 times a second.
 * - Position and velocity of the IMU: integrated from the accelerometer.
 *   A tracker pose is compared with the state at the time its images were
 *   taken (a history of one second), and position and velocity are pulled
 *   towards it, then and since. The jump this makes in the output is not shown
 *   at once: it fades in over a short time, without a step in the velocity.
 *
 * The tracker's world is mapped onto the filter's by an anchor (heading and
 * offset). When the tracker jumps and stays there, or starts a new map after a
 * reset, the anchor moves instead of the view. When the tracker runs away (its
 * velocity changes in ways the IMU did not feel, it moves while the IMU says
 * the head is still, or it moves impossibly fast), the filter holds the
 * position until the tracker delivers consistent poses again for a while, and
 * re-anchors there. Without tracker poses the position is held too.
 *
 * Plain C without Monado's math library, so that it is tested on its own.
 *
 * @ingroup drv_pico4
 */

#pragma once

#include "xrt/xrt_defines.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

//! IMU steps kept for delayed tracker poses: one second at 1 kHz.
#define PICO4_FUSION_HISTORY 1024
//! Tracker poses kept for the runaway check: over a second at 25 Hz.
#define PICO4_FUSION_POSES 32

//! A tracker pose, at the time its images were taken, in the tracker's world. World Y up, HMD axes.
struct pico4_fusion_slam
{
	int64_t ts_ns;
	struct xrt_quat orientation; //!< World from HMD.
	struct xrt_vec3 position;    //!< The IMU's position.
	struct xrt_vec3 velocity;    //!< The IMU's velocity, world frame.
};

struct pico4_fusion_config
{
	struct xrt_vec3 lever;  //!< From the IMU to the HMD origin, HMD frame, metres.
	float position_s;       //!< Time constant pulling the position to the tracker's.
	float velocity_s;       //!< Time constant pulling the velocity to the tracker's.
	float orientation_s;    //!< Time constant turning the orientation offset to the tracker's.
	float smoothing_s;      //!< Time constant over which a correction reaches the output.
	float gate_m, gate_rad; //!< Larger disagreements are outliers, unless they persist.
	uint32_t persist;       //!< Outliers in a row after which the tracker is re-anchored.
	int64_t stale_ns;       //!< Without a tracker pose for this long, the position is held.

	int64_t runaway_window_ns; //!< Time over which the tracker's velocity change is checked.
	float runaway_mps;         //!< Its largest disagreement with the IMU's, m/s.
	float max_speed;           //!< Faster tracker poses are impossible for a head, m/s.
	float still_mps;           //!< Largest tracker speed while the IMU has been still for the window, m/s.
	uint32_t runaway_poses;    //!< Implausible poses in a row that make a runaway.
	uint32_t relock_poses;     //!< Consistent poses in a row that end one.
};

enum pico4_fusion_result
{
	PICO4_FUSION_IGNORED,   //!< Not newer than the last, or outside the history.
	PICO4_FUSION_STARTED,   //!< The first pose: the position starts there.
	PICO4_FUSION_CORRECTED, //!< Blended in.
	PICO4_FUSION_OUTLIER,   //!< Too far off or implausible, skipped.
	PICO4_FUSION_RESET,     //!< Taken over in full after the tracker was stale.
	PICO4_FUSION_ANCHORED,  //!< The tracker's world mapped anew onto the filter's state.
	PICO4_FUSION_RUNAWAY,   //!< The tracker ran away: the position is held.
};

struct pico4_fusion_step
{
	int64_t ts_ns;
	struct xrt_quat q3dof;
	struct xrt_vec3 p, v;
	struct xrt_vec3 imu_dv; //!< Velocity change from the accelerometer alone since the start.
	int64_t still_since_ns; //!< Since when the IMU has felt neither turning nor acceleration.
};

//! A tracker pose kept for the runaway check, in the filter's world.
struct pico4_fusion_seen
{
	int64_t ts_ns;
	struct xrt_vec3 p, v, imu_dv;
};

struct pico4_fusion
{
	struct pico4_fusion_config c;

	//! State at each IMU step, oldest first from ring[head - count].
	struct pico4_fusion_step ring[PICO4_FUSION_HISTORY];
	uint32_t head, count;

	//! The tracker's recent poses, oldest first from poses[pose_head - pose_count].
	struct pico4_fusion_seen poses[PICO4_FUSION_POSES];
	uint32_t pose_head, pose_count;

	//! Newest IMU step: time, 3DoF orientation, gyro (HMD frame).
	int64_t ts_ns;
	struct xrt_quat q3dof;
	struct xrt_vec3 gyro;

	struct xrt_quat offset; //!< The filter's world from the 3DoF fusion's world.
	struct xrt_quat target; //!< The offset the tracker's newest pose measured.
	//! The filter's world from the tracker's: heading, then offset.
	struct xrt_quat anchor_q;
	struct xrt_vec3 anchor_p;
	struct xrt_vec3 p, v;   //!< The IMU's position and velocity.
	struct xrt_vec3 imu_dv; //!< Accelerometer-only velocity change since the start.
	int64_t dv_since_ns;    //!< imu_dv is integrated in the filter's world from here on.
	int64_t still_since_ns; //!< Since when the IMU has been still.
	/*!
	 * Correction not shown yet, and its rate. Both decay to zero critically
	 * damped, so a correction bends the output's path instead of kinking it.
	 */
	struct xrt_vec3 pending, pending_rate;
	bool started;
	bool lost;              //!< The tracker ran away: hold until it is consistent again.
	int64_t ignore_until_ns; //!< While lost, poses of images before this are skipped.
	int64_t slam_ts_ns;     //!< Image time of the last pose taken.
	int64_t seen_ts_ns;     //!< Image time of the newest pose offered.
	uint32_t outliers;      //!< Outliers in a row.
	uint32_t implausible;   //!< Implausible poses in a row.
	uint32_t consistent;    //!< While lost: consistent poses in a row.

	// Statistics.
	uint64_t corrected, rejected, resets, anchors, runaways;
	float last_error_m, last_error_rad;
	float last_innovation; //!< The newest velocity disagreement with the IMU, m/s.
	float last_speed;      //!< The tracker's newest speed, m/s.
};

void
pico4_fusion_init(struct pico4_fusion *f, const struct pico4_fusion_config *c);

//! One IMU step: the 3DoF orientation (world from HMD), gyro (rad/s) and accelerometer (m/s^2) in the HMD frame.
void
pico4_fusion_imu(struct pico4_fusion *f,
                 int64_t ts_ns,
                 const struct xrt_quat *q3dof,
                 const struct xrt_vec3 *gyro,
                 const struct xrt_vec3 *accel);

enum pico4_fusion_result
pico4_fusion_slam(struct pico4_fusion *f, const struct pico4_fusion_slam *m);

/*!
 * The tracker was told to reset at @p now_ns: while lost, its poses of images
 * before then belong to the old map and are skipped.
 */
void
pico4_fusion_tracker_reset(struct pico4_fusion *f, int64_t now_ns);

//! The HMD's pose at the newest IMU step; a position once the tracker has started it.
void
pico4_fusion_relation(const struct pico4_fusion *f, struct xrt_space_relation *out);

//! Whether the tracker currently bounds the position (not stale, not lost).
bool
pico4_fusion_tracking(const struct pico4_fusion *f);

#ifdef __cplusplus
}
#endif
