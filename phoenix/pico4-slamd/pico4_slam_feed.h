// Copyright 2026, overpressure contributors.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Shared memory between the PICO 4 driver and pico4-slamd, which runs PICO OS's head
 *         tracker (libSlam) in its bionic root (docs/xr.md#head-tracking).
 *
 * pico4-slamd creates the file (root:video, 0660) and keeps it; the driver maps it and is the
 * only writer of the frame and IMU rings, pico4-slamd the only writer of the pose ring. Each
 * ring entry is written first and its count released after it; a reader that falls behind by
 * a whole ring skips ahead. Times are CLOCK_MONOTONIC nanoseconds. The driver bumps
 * @ref pico4_slam_feed::generation when it (re)starts; pico4-slamd then starts a new tracker.
 * Plain C with fixed-size types: the same header builds against glibc and bionic.
 * @ingroup drv_pico4
 */

#pragma once

#include <stdint.h>

#define PICO4_SLAM_FEED_PATH "/dev/shm/pico4-slam"
#define PICO4_SLAM_FEED_MAGIC 0x37444546344f4350ull //!< "PCO4FED7"
#define PICO4_SLAM_FEED_W 640
#define PICO4_SLAM_FEED_H 480
#define PICO4_SLAM_FEED_CAMS 4
#define PICO4_SLAM_FEED_IMU 8192
#define PICO4_SLAM_FEED_SETS 4
#define PICO4_SLAM_FEED_POSES 4096
#define PICO4_SLAM_FEED_MESHES 2
#define PICO4_SLAM_FEED_MESH_VERTICES 16384 //!< PICO OS's hemisphere: a 128 x 128 grid at most.
#define PICO4_SLAM_FEED_MESH_INDICES (6 * 127 * 127)

//! One head IMU sample: the sensor DSP's raw axes (the factory body frame), m/s^2 and rad/s.
struct pico4_slam_feed_imu
{
	int64_t ts_ns;
	float gyro[3];
	float accel[3];
	float temperature; //!< Degrees Celsius; NaN if unknown.
	uint32_t pad;
};

//! One scene exposure of the four tracking cameras, in factory camera order (cam0..cam3).
struct pico4_slam_feed_set
{
	int64_t ts_ns;                               //!< Middle of the exposure, the earliest of the four.
	int64_t sof_ns[PICO4_SLAM_FEED_CAMS];        //!< Each frame's start of frame.
	uint32_t exposure_lines[PICO4_SLAM_FEED_CAMS];
	uint32_t gain[PICO4_SLAM_FEED_CAMS];         //!< Analog gain, 16 = 1x.
	uint64_t frame_id[PICO4_SLAM_FEED_CAMS];
	//! 10-bit samples in 16 bits, the sensor's rows as read out (no rotation), metadata row excluded.
	uint16_t pixels[PICO4_SLAM_FEED_CAMS][PICO4_SLAM_FEED_W * PICO4_SLAM_FEED_H];
};

//! One pose of the tracker, at an IMU sample's time, in its own world (body frame at the start).
struct pico4_slam_feed_pose
{
	int64_t ts_ns;
	double position[3];      //!< The IMU's position, metres.
	double orientation[4];   //!< World from body, w x y z.
	double velocity[3];      //!< World frame, m/s.
	double angular[3];       //!< Body rates without bias, rad/s.
	uint32_t confidence;     //!< 0: lost or not started; above: tracked.
	int32_t error;           //!< The tracker's error code.
	uint32_t type;
	/*!
	 * The tracker's ground detection (docs/xr.md#room-setup): 0 none, 1 the floor it detected
	 * (or 1.3 m below the head when its detection found no plane), 2 and 3 the height a saved
	 * map carries. The floor lies @ref floor_height below the world's origin.
	 */
	uint32_t floor_status;
	double floor_height;
};

/*!
 * PICO OS's see-through depth mesh (docs/xr.md#passthrough): the room around the head as its
 * libphotoid computes it from one pair of the forward cameras, in the tracker's world.
 */
struct pico4_slam_feed_mesh
{
	int64_t ts_ns;          //!< The pair's time.
	uint32_t vertex_count;
	uint32_t index_count;   //!< Triangles: three indices each.
	float vertices[PICO4_SLAM_FEED_MESH_VERTICES][3]; //!< The tracker's world, metres.
	uint32_t indices[PICO4_SLAM_FEED_MESH_INDICES];
};

struct pico4_slam_feed
{
	uint64_t magic;
	uint32_t version, generation; //!< generation: bumped by the driver at each start.
	uint64_t imu_written, sets_written, poses_written;
	uint64_t slamd_heartbeat_ns; //!< pico4-slamd's last activity.
	/*!
	 * The room (docs/xr.md#room-setup): after the tracker relocalised in a saved room, the
	 * transform from this session's world to the room's (a turn of @ref room_yaw about the
	 * world's up axis x, then @ref room_t), written by pico4-slamd before @ref room_state.
	 * 0 not known (yet), 1 relocalised, 2 a room of its own (no saved room, or no match).
	 */
	uint32_t room_state;
	uint32_t room_generation; //!< The driver generation the room state belongs to.
	double room_yaw;
	double room_t[3];
	char room_uuid[40]; //!< The tracker's map (getRelocMapInfo), the room's identity.
	/*!
	 * PICO OS's auto exposure of the head cameras (docs/xr.md#auto-exposure): pico4-slamd runs it
	 * on the sets and publishes the scene exposure it wants per camera (factory order, gain 16 =
	 * 1x), written before @ref ae_written is bumped. While @ref ae_generation is the driver's own
	 * and pico4-slamd's heartbeat is fresh, the driver sends these and runs no auto exposure.
	 */
	uint32_t ae_generation;
	uint32_t ae_lines[PICO4_SLAM_FEED_CAMS];
	uint32_t ae_gain[PICO4_SLAM_FEED_CAMS];
	uint64_t ae_written;
	/*!
	 * The tracking cameras' module ids (lens shading, docs/xr.md#head-tracking), as PICO OS's
	 * station service keeps them: pico4-slamd puts the saved ones in @ref otp_saved; when they
	 * are not all 8 or 9, the driver (the station's owner) reads them from the modules' OTP
	 * before it starts a generation, into @ref otp_read with @ref otp_state 1 (2: no answer) and
	 * @ref otp_generation the generation it then starts; pico4-slamd saves valid ones.
	 */
	uint8_t otp_saved[PICO4_SLAM_FEED_CAMS];
	uint8_t otp_read[PICO4_SLAM_FEED_CAMS];
	uint32_t otp_state;
	uint32_t otp_generation;
	struct pico4_slam_feed_imu imu[PICO4_SLAM_FEED_IMU];
	struct pico4_slam_feed_pose poses[PICO4_SLAM_FEED_POSES];
	struct pico4_slam_feed_set sets[PICO4_SLAM_FEED_SETS];
	/*!
	 * The depth mesh, written by pico4-slamd into slot @ref meshes_written % 2 before the count
	 * is bumped: a reader takes the newest slot and checks the count again after its copy. The
	 * driver stamps @ref mesh_wanted_ns whenever it asks for a mesh (SteamVR's room view reads
	 * them); pico4-slamd feeds the depth pipeline only within a second of it.
	 */
	uint64_t mesh_wanted_ns;
	uint64_t meshes_written;
	struct pico4_slam_feed_mesh meshes[PICO4_SLAM_FEED_MESHES];
	/*!
	 * SteamVR's room setup asks for the floor (docs/xr.md#room-setup): the driver bumps this
	 * when a step that detects the floor begins, and pico4-slamd starts the tracker's ground
	 * detection again, so the floor comes from what the head sees then.
	 */
	uint64_t floor_requests;
};
