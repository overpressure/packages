// Copyright 2026, overpressure contributors.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  PICO 4 controllers (see pico4_controller.h).
 *
 * Inputs come from the key records of the radio packets. The pose comes from
 * the controller tracker (pico4_ctrack.h: LED exposures and the controller IMU)
 * while it tracks the controller. Otherwise the orientation comes from the
 * controller's IMU records through m_imu_3dof, started level with the measured
 * gravity, and the position is a fixed offset from the head, turned with its
 * heading. Either is the aim pose, as PICO OS publishes it; the grip pose is
 * the aim composed with PICO OS's aim-to-grip transform (pico4_aimgrip.h) from
 * calibration/controllers/aim-to-grip.json, or the aim pose without that file.
 *
 * @ingroup drv_pico4
 */

#include "pico4_controller.h"
#include "pico4_aimgrip.h"
#include "pico4_ctrack.h"
#include "pico4_station.h"

#include "math/m_api.h"
#include "math/m_imu_3dof.h"
#include "math/m_mathinclude.h" // IWYU pragma: keep
#include "math/m_relation_history.h"
#include "math/m_vec3.h"
#include "os/os_threading.h"
#include "util/u_debug.h"
#include "util/u_device.h"
#include "util/u_misc.h"
#include "util/u_time.h"

#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <string.h>

/*!
 * Raw sensor axes to the aim frame (x right, y up, z back), as "x,y,z" with
 * optional minus signs, for example "-y,x,z".
 */
DEBUG_GET_ONCE_OPTION(pico4_controller_axes_left, "PICO4_CONTROLLER_AXES_LEFT", "x,-z,y")
DEBUG_GET_ONCE_OPTION(pico4_controller_axes_right, "PICO4_CONTROLLER_AXES_RIGHT", "x,-z,y")
//! The HMD's calibration directory (pico4_hmd.c), for controllers/aim-to-grip.json.
DEBUG_GET_ONCE_OPTION(pico4_controller_calibration, "PICO4_CALIBRATION_DIR", "/usr/share/qcom/pico4/calibration")

#define C_DEBUG(c, ...) U_LOG_XDEV_IFL_D(&c->base, c->log_level, __VA_ARGS__)
#define C_INFO(c, ...) U_LOG_XDEV_IFL_I(&c->base, c->log_level, __VA_ARGS__)

#define GRAVITY 9.80665
#define WARMUP_SAMPLES 100

enum
{
	IN_AX_CLICK,
	IN_AX_TOUCH,
	IN_BY_CLICK,
	IN_BY_TOUCH,
	IN_MENU_CLICK,
	IN_SYSTEM_CLICK,
	IN_TRIGGER_CLICK,
	IN_TRIGGER_VALUE,
	IN_TRIGGER_TOUCH,
	IN_STICK_CLICK,
	IN_STICK_TOUCH,
	IN_STICK,
	IN_SQUEEZE_CLICK,
	IN_SQUEEZE_VALUE,
	IN_GRIP_POSE,
	IN_AIM_POSE,
	IN_COUNT,
};

//! The top of the battery byte's scale.
#define BATTERY_FULL 5

struct pico4_controller
{
	struct xrt_device base;
	int device;
	struct xrt_device *head;
	struct pico4_ctrack *ctrack; //!< Guarded by lock; NULL once the HMD let go.
	struct pico4_station *station; //!< Haptics; guarded by lock, NULL once the HMD let go.
	enum u_logging_level log_level;

	struct os_mutex lock;
	int refs; //!< Monado's device and the HMD's sensor thread.

	//! Axis mapping: aim axis i takes raw axis axes[i] times signs[i].
	int axes[3];
	float signs[3];

	//! The grip in the aim frame; without it the grip pose is the aim pose.
	bool has_aim_to_grip;
	struct xrt_pose aim_to_grip;

	struct m_imu_3dof fusion;
	uint32_t warmup;
	struct xrt_vec3 warmup_accel;
	struct m_relation_history *history;

	struct pico4_blemcu_keys keys;
	int64_t keys_ns;
	struct pico4_blemcu_order order; //!< The 3DoF orientation's samples.
	uint64_t imu_samples, key_packets;
	uint64_t vibrations; //!< Motor frames sent; guarded by lock.
	int battery;         //!< The raw level of the last battery record, -1 before the first.
};

static inline struct pico4_controller *
pico4_controller(struct xrt_device *xdev)
{
	return (struct pico4_controller *)xdev;
}

static void
parse_axes(const char *text, int axes[3], float signs[3])
{
	int n = 0;
	for (const char *p = text; *p != '\0' && n < 3; p++) {
		float sign = 1.0f;
		if (*p == '-') {
			sign = -1.0f;
			p++;
		}
		if (*p < 'x' || *p > 'z') {
			break;
		}
		axes[n] = *p - 'x';
		signs[n] = sign;
		n++;
		while (p[1] != '\0' && p[1] != ',') {
			p++;
		}
		if (p[1] == ',') {
			p++;
		}
	}
	if (n != 3) {
		for (int i = 0; i < 3; i++) {
			axes[i] = i;
			signs[i] = 1.0f;
		}
	}
}

//! This hand's aim-to-grip transform from @p path; false with the reason in @p error.
static bool
load_aim_to_grip(struct pico4_controller *c, const char *path, char *error, size_t error_len)
{
	FILE *f = fopen(path, "re");
	if (f == NULL) {
		snprintf(error, error_len, "%s: %s", path, strerror(errno));
		return false;
	}
	char text[16384];
	size_t n = fread(text, 1, sizeof(text) - 1, f);
	bool complete = feof(f) != 0;
	fclose(f);
	if (!complete) {
		snprintf(error, error_len, "%s: not read to the end", path);
		return false;
	}
	text[n] = '\0';
	return pico4_aimgrip_parse(text, c->device == PICO4_BLEMCU_LEFT, &c->aim_to_grip, error, error_len);
}

static struct xrt_vec3
to_aim(const struct pico4_controller *c, const int16_t raw[3], double scale)
{
	float v[3];
	for (int i = 0; i < 3; i++) {
		v[i] = (float)(raw[c->axes[i]] * scale) * c->signs[i];
	}
	return (struct xrt_vec3){v[0], v[1], v[2]};
}

//! Orientation that takes the measured gravity reaction @p a (up when still) to world +Y.
static struct xrt_quat
level_from_gravity(const struct xrt_vec3 *a)
{
	struct xrt_vec3 n = *a, up = {0, 1, 0};
	math_vec3_normalize(&n);
	struct xrt_quat q;
	math_quat_from_vec_a_to_vec_b(&n, &up, &q);
	return q;
}

static void
push_imu(struct pico4_controller *c, const struct pico4_blemcu_imu *imu, int64_t ts)
{
	// The 3DoF filter integrates from one sample to the next: a duplicate or backward step
	// would turn it by nothing or backwards. The tracker gets them and starts over, as stock.
	if (!pico4_blemcu_order_accept(&c->order, ts)) {
		return;
	}
	struct xrt_vec3 accel = to_aim(c, imu->accel, GRAVITY / PICO4_BLEMCU_ACCEL_LSB_PER_G);
	struct xrt_vec3 gyro = to_aim(c, imu->gyro, (M_PI / 180.0) / PICO4_BLEMCU_GYRO_LSB_PER_DPS);
	c->imu_samples++;

	if (c->warmup < WARMUP_SAMPLES) {
		math_vec3_accum(&accel, &c->warmup_accel);
		if (++c->warmup == WARMUP_SAMPLES) {
			c->fusion.rot = level_from_gravity(&c->warmup_accel);
		}
		return;
	}
	m_imu_3dof_update(&c->fusion, (uint64_t)ts, &accel, &gyro);

	struct xrt_space_relation rel = XRT_SPACE_RELATION_ZERO;
	rel.relation_flags = (enum xrt_space_relation_flags)(XRT_SPACE_RELATION_ORIENTATION_VALID_BIT |
	                                                      XRT_SPACE_RELATION_ORIENTATION_TRACKED_BIT |
	                                                      XRT_SPACE_RELATION_ANGULAR_VELOCITY_VALID_BIT);
	rel.pose.orientation = c->fusion.rot;
	math_quat_rotate_derivative(&c->fusion.rot, &gyro, &rel.angular_velocity);
	m_relation_history_push(c->history, &rel, ts);
}

void
pico4_controller_push(struct pico4_controller *c, const struct pico4_blemcu_packet *p, int64_t packet_ns)
{
	os_mutex_lock(&c->lock);
	for (int i = 0; i < p->imu_count; i++) {
		if (p->imu[i].device == c->device) {
			push_imu(c, &p->imu[i], packet_ns - (int64_t)p->imu[i].offset_us * 1000);
		}
	}
	for (int i = 0; i < p->key_count; i++) {
		if (p->keys[i].device == c->device) {
			c->keys = p->keys[i];
			c->keys_ns = packet_ns;
			c->key_packets++;
		}
	}
	for (int i = 0; i < p->battery_count; i++) {
		if (p->battery[i].device == c->device && p->battery[i].level != c->battery) {
			c->battery = p->battery[i].level;
			C_INFO(c, "Battery level %d (raw)", c->battery);
		}
	}
	os_mutex_unlock(&c->lock);
}

static void
unref(struct pico4_controller *c)
{
	os_mutex_lock(&c->lock);
	int left = --c->refs;
	os_mutex_unlock(&c->lock);
	if (left > 0) {
		return;
	}
	os_mutex_destroy(&c->lock);
	m_imu_3dof_close(&c->fusion);
	m_relation_history_destroy(&c->history);
	u_device_free(&c->base);
}

void
pico4_controller_release(struct pico4_controller *c)
{
	os_mutex_lock(&c->lock);
	c->head = NULL;
	c->ctrack = NULL;
	c->station = NULL;
	os_mutex_unlock(&c->lock);
	unref(c);
}

static void
pico4_controller_destroy(struct xrt_device *xdev)
{
	struct pico4_controller *c = pico4_controller(xdev);
	C_INFO(c, "%llu IMU samples (%llu out of order), %llu key packets, %llu vibration frames",
	       (unsigned long long)c->imu_samples, (unsigned long long)c->order.rejected,
	       (unsigned long long)c->key_packets, (unsigned long long)c->vibrations);
	unref(c);
}

static xrt_result_t
pico4_controller_update_inputs(struct xrt_device *xdev)
{
	struct pico4_controller *c = pico4_controller(xdev);
	os_mutex_lock(&c->lock);
	struct pico4_blemcu_keys k = c->keys;
	int64_t ts = c->keys_ns;
	os_mutex_unlock(&c->lock);

	struct xrt_input *in = c->base.inputs;
	for (int i = 0; i < IN_COUNT; i++) {
		in[i].timestamp = ts;
	}
	in[IN_AX_CLICK].value.boolean = (k.buttons & PICO4_BLEMCU_BTN_AX) != 0;
	in[IN_AX_TOUCH].value.boolean = (k.touch & PICO4_BLEMCU_TOUCH_AX) != 0;
	in[IN_BY_CLICK].value.boolean = (k.buttons & PICO4_BLEMCU_BTN_BY) != 0;
	in[IN_BY_TOUCH].value.boolean = (k.touch & PICO4_BLEMCU_TOUCH_BY) != 0;
	in[IN_MENU_CLICK].value.boolean = (k.buttons & PICO4_BLEMCU_BTN_APP) != 0;
	in[IN_SYSTEM_CLICK].value.boolean = (k.buttons & PICO4_BLEMCU_BTN_HOME) != 0;
	in[IN_TRIGGER_CLICK].value.boolean = (k.buttons & PICO4_BLEMCU_BTN_TRIGGER) != 0;
	in[IN_TRIGGER_VALUE].value.vec1.x = k.trigger / 255.0f;
	in[IN_TRIGGER_TOUCH].value.boolean = (k.touch & PICO4_BLEMCU_TOUCH_TRIGGER) != 0;
	in[IN_STICK_CLICK].value.boolean = (k.buttons & PICO4_BLEMCU_BTN_STICK) != 0;
	in[IN_STICK_TOUCH].value.boolean = (k.touch & PICO4_BLEMCU_TOUCH_STICK) != 0;
	// Without a key packet yet the stick bytes are 0; report it centred.
	float sx = ts != 0 ? (k.stick_x - 128) / 127.0f : 0.0f, sy = ts != 0 ? (k.stick_y - 128) / 127.0f : 0.0f;
	in[IN_STICK].value.vec2.x = sx < -1.0f ? -1.0f : sx;
	in[IN_STICK].value.vec2.y = sy < -1.0f ? -1.0f : sy;
	in[IN_SQUEEZE_CLICK].value.boolean = (k.buttons & PICO4_BLEMCU_BTN_GRIP) != 0;
	in[IN_SQUEEZE_VALUE].value.vec1.x = k.grip / 255.0f;
	return XRT_SUCCESS;
}

//! 3DoF orientation at a fixed offset from the head, turned with its heading.
static void
placeholder_pose(struct pico4_controller *c, int64_t at_timestamp_ns, struct xrt_space_relation *out)
{
	struct xrt_space_relation rel = XRT_SPACE_RELATION_ZERO;
	os_mutex_lock(&c->lock);
	struct pico4_blemcu_order order = c->order;
	os_mutex_unlock(&c->lock);
	if (pico4_blemcu_order_fresh(&order, at_timestamp_ns)) {
		m_relation_history_get(c->history, at_timestamp_ns, &rel);
	} else if (order.newest_ns != 0) {
		// The controller stopped sending: hold its last orientation instead of extrapolating
		// the last turn, untracked.
		m_relation_history_get(c->history, order.newest_ns, &rel);
		rel.relation_flags = (enum xrt_space_relation_flags)(rel.relation_flags &
		                                                      XRT_SPACE_RELATION_ORIENTATION_VALID_BIT);
		rel.angular_velocity = (struct xrt_vec3){0, 0, 0};
	}
	if (!(rel.relation_flags & XRT_SPACE_RELATION_ORIENTATION_VALID_BIT)) {
		rel.pose.orientation = (struct xrt_quat)XRT_QUAT_IDENTITY;
	}
	math_quat_normalize(&rel.pose.orientation);

	// In front of and below the head, turned with the head's heading.
	struct xrt_vec3 offset = {c->device == PICO4_BLEMCU_LEFT ? -0.18f : 0.18f, -0.35f, -0.30f};
	struct xrt_space_relation head = XRT_SPACE_RELATION_ZERO;
	os_mutex_lock(&c->lock);
	struct xrt_device *hd = c->head;
	os_mutex_unlock(&c->lock);
	if (hd != NULL) {
		xrt_device_get_tracked_pose(hd, XRT_INPUT_GENERIC_HEAD_POSE, at_timestamp_ns, &head);
	}
	struct xrt_quat yaw = XRT_QUAT_IDENTITY;
	if (head.relation_flags & XRT_SPACE_RELATION_ORIENTATION_VALID_BIT) {
		struct xrt_vec3 fwd = {0, 0, -1}, f;
		math_quat_rotate_vec3(&head.pose.orientation, &fwd, &f);
		float heading = atan2f(-f.x, -f.z);
		yaw = (struct xrt_quat){0, sinf(heading / 2), 0, cosf(heading / 2)};
	}
	math_quat_rotate_vec3(&yaw, &offset, &rel.pose.position);
	if (head.relation_flags & XRT_SPACE_RELATION_POSITION_VALID_BIT) {
		math_vec3_accum(&head.pose.position, &rel.pose.position);
	}
	rel.relation_flags = (enum xrt_space_relation_flags)(rel.relation_flags | XRT_SPACE_RELATION_POSITION_VALID_BIT |
	                                                      XRT_SPACE_RELATION_ORIENTATION_VALID_BIT);
	*out = rel;
}

static xrt_result_t
pico4_controller_get_tracked_pose(struct xrt_device *xdev,
                                  enum xrt_input_name name,
                                  int64_t at_timestamp_ns,
                                  struct xrt_space_relation *out_relation)
{
	struct pico4_controller *c = pico4_controller(xdev);
	if (name != XRT_INPUT_PICO4_GRIP_POSE && name != XRT_INPUT_PICO4_AIM_POSE) {
		U_LOG_XDEV_UNSUPPORTED_INPUT(&c->base, c->log_level, name);
		return XRT_ERROR_INPUT_UNSUPPORTED;
	}

	// The aim pose; the grip follows from it.
	struct xrt_space_relation rel = XRT_SPACE_RELATION_ZERO;
	// The tracker is read under the lock: the HMD destroys it only after pico4_controller_release.
	os_mutex_lock(&c->lock);
	bool tracked = c->ctrack != NULL && pico4_ctrack_get_aim(c->ctrack, c->device, at_timestamp_ns, &rel);
	os_mutex_unlock(&c->lock);
	if (!tracked) {
		placeholder_pose(c, at_timestamp_ns, &rel);
	}
	if (name == XRT_INPUT_PICO4_GRIP_POSE && c->has_aim_to_grip) {
		pico4_aimgrip_apply(&c->aim_to_grip, &rel, out_relation);
	} else {
		*out_relation = rel;
	}
	return XRT_SUCCESS;
}

static xrt_result_t
pico4_controller_set_output(struct xrt_device *xdev, enum xrt_output_name name, const struct xrt_output_value *value)
{
	struct pico4_controller *c = pico4_controller(xdev);
	if (name != XRT_OUTPUT_NAME_PICO4_HAPTIC || value->type != XRT_OUTPUT_VALUE_TYPE_VIBRATION) {
		return XRT_ERROR_OUTPUT_UNSUPPORTED;
	}
	// One motor frame per request, mapped as PICO OS maps OpenXR haptics; level 0 stops.
	unsigned level, duration_ms;
	pico4_station_haptic_map(value->vibration.amplitude, value->vibration.duration_ns, &level, &duration_ms);
	enum pico4_station_hand hand =
	    c->device == PICO4_BLEMCU_LEFT ? PICO4_STATION_HAND_LEFT : PICO4_STATION_HAND_RIGHT;
	// The station is used under the lock: the HMD closes it only after pico4_controller_release.
	os_mutex_lock(&c->lock);
	int r = c->station != NULL ? pico4_station_motor(c->station, hand, level, duration_ms) : 0;
	c->vibrations += c->station != NULL && r == 0;
	os_mutex_unlock(&c->lock);
	if (r < 0) {
		C_DEBUG(c, "vibration not sent: %d", r);
	}
	return XRT_SUCCESS;
}

/*!
 * The controller's charge. The level is one byte; PICO OS's controller service counts 1 as low, and
 * its settings draw six steps, 0 to 5, so a level is a fifth of a full charge.
 */
static xrt_result_t
pico4_controller_get_battery_status(struct xrt_device *xdev, bool *out_present, bool *out_charging, float *out_charge)
{
	struct pico4_controller *c = pico4_controller(xdev);
	os_mutex_lock(&c->lock);
	int level = c->battery;
	os_mutex_unlock(&c->lock);
	*out_present = level >= 0 && level <= BATTERY_FULL;
	*out_charging = false;
	*out_charge = *out_present ? (float)level / BATTERY_FULL : 0.f;
	return XRT_SUCCESS;
}

struct pico4_controller *
pico4_controller_create(int device,
                        struct xrt_device *head,
                        struct pico4_ctrack *ctrack,
                        struct pico4_station *station,
                        enum u_logging_level log_level)
{
	enum u_device_alloc_flags flags = U_DEVICE_ALLOC_TRACKING_NONE;
	struct pico4_controller *c = U_DEVICE_ALLOCATE(struct pico4_controller, flags, IN_COUNT, 1);
	// Poses are relative to the head's (tracked or placeholder): the same origin and offset.
	c->base.tracking_origin = head->tracking_origin;
	bool left = device == PICO4_BLEMCU_LEFT;
	c->device = device;
	c->head = head;
	c->ctrack = ctrack;
	c->station = station;
	c->log_level = log_level;
	c->refs = 2;
	os_mutex_init(&c->lock);
	parse_axes(left ? debug_get_option_pico4_controller_axes_left() : debug_get_option_pico4_controller_axes_right(),
	           c->axes, c->signs);
	char path[512], error[640];
	snprintf(path, sizeof(path), "%s/controllers/aim-to-grip.json", debug_get_option_pico4_controller_calibration());
	c->has_aim_to_grip = load_aim_to_grip(c, path, error, sizeof(error));
	if (!c->has_aim_to_grip) {
		U_LOG_IFL_W(log_level, "PICO 4 %s controller: grip pose = aim pose (%s)", left ? "left" : "right", error);
	}
	c->battery = -1;
	m_imu_3dof_init(&c->fusion, M_IMU_3DOF_USE_GRAVITY_DUR_20MS);
	m_relation_history_create(&c->history);

	c->base.name = XRT_DEVICE_PICO4_CONTROLLER;
	c->base.device_type = left ? XRT_DEVICE_TYPE_LEFT_HAND_CONTROLLER : XRT_DEVICE_TYPE_RIGHT_HAND_CONTROLLER;
	c->base.update_inputs = pico4_controller_update_inputs;
	c->base.get_battery_status = pico4_controller_get_battery_status;
	c->base.get_tracked_pose = pico4_controller_get_tracked_pose;
	c->base.set_output = pico4_controller_set_output;
	c->base.destroy = pico4_controller_destroy;
	c->base.supported.orientation_tracking = true;
	c->base.supported.position_tracking = ctrack != NULL;
	snprintf(c->base.str, XRT_DEVICE_NAME_LEN, "PICO 4 %s controller", left ? "left" : "right");
	snprintf(c->base.serial, XRT_DEVICE_NAME_LEN, "PICO 4 %s controller", left ? "left" : "right");

	const enum xrt_input_name names[IN_COUNT] = {
	    [IN_AX_CLICK] = left ? XRT_INPUT_PICO4_X_CLICK : XRT_INPUT_PICO4_A_CLICK,
	    [IN_AX_TOUCH] = left ? XRT_INPUT_PICO4_X_TOUCH : XRT_INPUT_PICO4_A_TOUCH,
	    [IN_BY_CLICK] = left ? XRT_INPUT_PICO4_Y_CLICK : XRT_INPUT_PICO4_B_CLICK,
	    [IN_BY_TOUCH] = left ? XRT_INPUT_PICO4_Y_TOUCH : XRT_INPUT_PICO4_B_TOUCH,
	    [IN_MENU_CLICK] = XRT_INPUT_PICO4_MENU_CLICK,
	    [IN_SYSTEM_CLICK] = XRT_INPUT_PICO4_SYSTEM_CLICK,
	    [IN_TRIGGER_CLICK] = XRT_INPUT_PICO4_TRIGGER_CLICK,
	    [IN_TRIGGER_VALUE] = XRT_INPUT_PICO4_TRIGGER_VALUE,
	    [IN_TRIGGER_TOUCH] = XRT_INPUT_PICO4_TRIGGER_TOUCH,
	    [IN_STICK_CLICK] = XRT_INPUT_PICO4_THUMBSTICK_CLICK,
	    [IN_STICK_TOUCH] = XRT_INPUT_PICO4_THUMBSTICK_TOUCH,
	    [IN_STICK] = XRT_INPUT_PICO4_THUMBSTICK,
	    [IN_SQUEEZE_CLICK] = XRT_INPUT_PICO4_SQUEEZE_CLICK,
	    [IN_SQUEEZE_VALUE] = XRT_INPUT_PICO4_SQUEEZE_VALUE,
	    [IN_GRIP_POSE] = XRT_INPUT_PICO4_GRIP_POSE,
	    [IN_AIM_POSE] = XRT_INPUT_PICO4_AIM_POSE,
	};
	for (int i = 0; i < IN_COUNT; i++) {
		c->base.inputs[i].name = names[i];
		c->base.inputs[i].active = true;
	}
	c->base.outputs[0].name = XRT_OUTPUT_NAME_PICO4_HAPTIC;
	return c;
}

struct xrt_device *
pico4_controller_xdev(struct pico4_controller *c)
{
	return &c->base;
}
