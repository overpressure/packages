// Copyright 2026, overpressure contributors.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  PICO 4 head pose: the 1 kHz IMU propagates it, delayed SLAM poses correct it.
 * @ingroup drv_pico4
 */

#include "pico4_fusion.h"

#include <math.h>
#include <string.h>

#define GRAVITY 9.80665f
#define HOLD_S 0.05f //!< Velocity decay while the position is held.
//! The IMU counts as still below these: a head at rest, not one walking or turning.
#define STILL_ACCEL 0.3f
#define STILL_GYRO 0.15f

static struct xrt_vec3
v_add(struct xrt_vec3 a, struct xrt_vec3 b)
{
	return (struct xrt_vec3){a.x + b.x, a.y + b.y, a.z + b.z};
}

static struct xrt_vec3
v_sub(struct xrt_vec3 a, struct xrt_vec3 b)
{
	return (struct xrt_vec3){a.x - b.x, a.y - b.y, a.z - b.z};
}

static struct xrt_vec3
v_scale(struct xrt_vec3 a, float s)
{
	return (struct xrt_vec3){a.x * s, a.y * s, a.z * s};
}

static struct xrt_vec3
v_cross(struct xrt_vec3 a, struct xrt_vec3 b)
{
	return (struct xrt_vec3){a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}

static float
v_len(struct xrt_vec3 a)
{
	return sqrtf(a.x * a.x + a.y * a.y + a.z * a.z);
}

static struct xrt_quat
q_mul(struct xrt_quat a, struct xrt_quat b)
{
	return (struct xrt_quat){
	    a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
	    a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
	    a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
	    a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
	};
}

static struct xrt_quat
q_conj(struct xrt_quat q)
{
	return (struct xrt_quat){-q.x, -q.y, -q.z, q.w};
}

static float
q_dot(struct xrt_quat a, struct xrt_quat b)
{
	return a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w;
}

static struct xrt_quat
q_normalize(struct xrt_quat q)
{
	float n = sqrtf(q_dot(q, q));
	if (n < 1e-9f) {
		return (struct xrt_quat)XRT_QUAT_IDENTITY;
	}
	return (struct xrt_quat){q.x / n, q.y / n, q.z / n, q.w / n};
}

static struct xrt_vec3
q_rotate(struct xrt_quat q, struct xrt_vec3 v)
{
	struct xrt_vec3 u = {q.x, q.y, q.z};
	struct xrt_vec3 t = v_scale(v_cross(u, v), 2.0f);
	return v_add(v_add(v, v_scale(t, q.w)), v_cross(u, t));
}

//! From @p a towards @p b by @p k (0..1), the shorter way; the angles here are small.
static struct xrt_quat
q_blend(struct xrt_quat a, struct xrt_quat b, float k)
{
	if (q_dot(a, b) < 0.0f) {
		b = (struct xrt_quat){-b.x, -b.y, -b.z, -b.w};
	}
	return q_normalize((struct xrt_quat){a.x + k * (b.x - a.x), a.y + k * (b.y - a.y), a.z + k * (b.z - a.z),
	                                     a.w + k * (b.w - a.w)});
}

static float
q_angle(struct xrt_quat a, struct xrt_quat b)
{
	float d = fabsf(q_dot(a, b));
	return 2.0f * acosf(d > 1.0f ? 1.0f : d);
}

//! The rotation about the vertical (Y) in @p q: the twist of a swing-twist split.
static struct xrt_quat
q_yaw(struct xrt_quat q)
{
	return q_normalize((struct xrt_quat){0.0f, q.y, 0.0f, q.w});
}

static struct pico4_fusion_step *
step_at(struct pico4_fusion *f, uint32_t i)
{
	return &f->ring[(f->head + PICO4_FUSION_HISTORY - f->count + i) % PICO4_FUSION_HISTORY];
}

//! The state at @p ts_ns, interpolated between IMU steps; false outside the history.
static bool
lookup(struct pico4_fusion *f, int64_t ts_ns, struct pico4_fusion_step *out)
{
	if (f->count < 2 || ts_ns < step_at(f, 0)->ts_ns || ts_ns > step_at(f, f->count - 1)->ts_ns) {
		return false;
	}
	uint32_t lo = 0, hi = f->count - 1; // ts(lo) <= ts_ns <= ts(hi)
	while (hi - lo > 1) {
		uint32_t mid = (lo + hi) / 2;
		if (step_at(f, mid)->ts_ns <= ts_ns) {
			lo = mid;
		} else {
			hi = mid;
		}
	}
	const struct pico4_fusion_step *a = step_at(f, lo), *b = step_at(f, hi);
	float u = b->ts_ns > a->ts_ns ? (float)(ts_ns - a->ts_ns) / (float)(b->ts_ns - a->ts_ns) : 0.0f;
	out->ts_ns = ts_ns;
	out->q3dof = q_blend(a->q3dof, b->q3dof, u);
	out->p = v_add(a->p, v_scale(v_sub(b->p, a->p), u));
	out->v = v_add(a->v, v_scale(v_sub(b->v, a->v), u));
	out->imu_dv = v_add(a->imu_dv, v_scale(v_sub(b->imu_dv, a->imu_dv), u));
	out->still_since_ns = b->still_since_ns;
	return true;
}

static struct pico4_fusion_seen *
seen_at(struct pico4_fusion *f, uint32_t i)
{
	return &f->poses[(f->pose_head + PICO4_FUSION_POSES - f->pose_count + i) % PICO4_FUSION_POSES];
}

static void
remember(struct pico4_fusion *f, int64_t ts_ns, struct xrt_vec3 p, struct xrt_vec3 v, struct xrt_vec3 imu_dv)
{
	f->poses[f->pose_head] = (struct pico4_fusion_seen){ts_ns, p, v, imu_dv};
	f->pose_head = (f->pose_head + 1) % PICO4_FUSION_POSES;
	if (f->pose_count < PICO4_FUSION_POSES) {
		f->pose_count++;
	}
}

/*!
 * How far the tracker's velocity changed, over the runaway window up to @p ts_ns,
 * from what the accelerometer measured over the same time. A head cannot change
 * its velocity without the IMU feeling it. Negative without an older pose.
 */
static float
innovation(struct pico4_fusion *f, int64_t ts_ns, struct xrt_vec3 v, struct xrt_vec3 imu_dv)
{
	for (uint32_t i = f->pose_count; i-- > 0;) {
		const struct pico4_fusion_seen *s = seen_at(f, i);
		if (s->ts_ns > ts_ns - f->c.runaway_window_ns) {
			continue;
		}
		if (s->ts_ns < ts_ns - 3 * f->c.runaway_window_ns || s->ts_ns < f->dv_since_ns) {
			break;
		}
		return v_len(v_sub(v_sub(v, s->v), v_sub(imu_dv, s->imu_dv)));
	}
	return -1.0f;
}

//! A tracker pose in the filter's world.
static struct pico4_fusion_slam
to_world(const struct pico4_fusion *f, const struct pico4_fusion_slam *m)
{
	return (struct pico4_fusion_slam){
	    .ts_ns = m->ts_ns,
	    .orientation = q_normalize(q_mul(f->anchor_q, m->orientation)),
	    .position = v_add(q_rotate(f->anchor_q, m->position), f->anchor_p),
	    .velocity = q_rotate(f->anchor_q, m->velocity),
	};
}

/*!
 * Maps the tracker's world anew so that @p m lands on the filter's state at its
 * time (@p then): the filter's heading and position stay, the tracker's change.
 * Both worlds have gravity along -Y, so only the heading turns.
 */
static void
anchor_at(struct pico4_fusion *f, const struct pico4_fusion_slam *m, const struct pico4_fusion_step *then)
{
	struct xrt_quat filter_q = q_mul(f->offset, then->q3dof);
	f->anchor_q = q_yaw(q_mul(filter_q, q_conj(m->orientation)));
	f->anchor_p = v_sub(then->p, q_rotate(f->anchor_q, m->position));
	f->pose_count = 0;
	f->outliers = 0;
	f->implausible = 0;
	f->consistent = 0;
	f->anchors++;
}

void
pico4_fusion_init(struct pico4_fusion *f, const struct pico4_fusion_config *c)
{
	memset(f, 0, sizeof(*f));
	f->c = *c;
	f->offset = (struct xrt_quat)XRT_QUAT_IDENTITY;
	f->target = f->offset;
	f->anchor_q = f->offset;
	f->q3dof = f->offset;
}

bool
pico4_fusion_tracking(const struct pico4_fusion *f)
{
	return f->started && !f->lost && f->ts_ns - f->slam_ts_ns <= f->c.stale_ns;
}

void
pico4_fusion_tracker_reset(struct pico4_fusion *f, int64_t now_ns)
{
	f->ignore_until_ns = now_ns;
}

void
pico4_fusion_imu(struct pico4_fusion *f,
                 int64_t ts_ns,
                 const struct xrt_quat *q3dof,
                 const struct xrt_vec3 *gyro,
                 const struct xrt_vec3 *accel)
{
	if (f->count > 0 && ts_ns <= f->ts_ns) {
		return; // the history must stay in order
	}
	float dt = f->count > 0 ? (float)(ts_ns - f->ts_ns) * 1e-9f : 0.0f;
	if (f->started && dt > 0.0f && dt < 0.05f) {
		// World acceleration: the specific force turned into the world, less gravity's reaction.
		struct xrt_vec3 a = q_rotate(q_mul(f->offset, *q3dof), *accel);
		a.y -= GRAVITY;
		f->imu_dv = v_add(f->imu_dv, v_scale(a, dt));
		if (v_len(a) > STILL_ACCEL || v_len(*gyro) > STILL_GYRO) {
			f->still_since_ns = ts_ns;
		}
		if (pico4_fusion_tracking(f)) {
			f->p = v_add(f->p, v_add(v_scale(f->v, dt), v_scale(a, 0.5f * dt * dt)));
			f->v = v_add(f->v, v_scale(a, dt));
		} else {
			// No tracker to bound the accelerometer's drift: come to rest and hold.
			f->v = v_scale(f->v, expf(-dt / HOLD_S));
			f->p = v_add(f->p, v_scale(f->v, dt));
		}
		f->offset = q_blend(f->offset, f->target, 1.0f - expf(-dt / f->c.orientation_s));
		// Critically damped: pending'' = -2 w pending' - w^2 pending.
		float w = 1.0f / f->c.smoothing_s;
		struct xrt_vec3 pa = v_add(v_scale(f->pending_rate, -2.0f * w), v_scale(f->pending, -w * w));
		f->pending_rate = v_add(f->pending_rate, v_scale(pa, dt));
		f->pending = v_add(f->pending, v_scale(f->pending_rate, dt));
	}
	f->ts_ns = ts_ns;
	f->q3dof = *q3dof;
	f->gyro = *gyro;

	struct pico4_fusion_step *s = &f->ring[f->head];
	s->ts_ns = ts_ns;
	s->q3dof = *q3dof;
	s->p = f->p;
	s->v = f->v;
	s->imu_dv = f->imu_dv;
	s->still_since_ns = f->still_since_ns;
	f->head = (f->head + 1) % PICO4_FUSION_HISTORY;
	if (f->count < PICO4_FUSION_HISTORY) {
		f->count++;
	}
}

/*!
 * Moves the state at @p ts_ns by @p dp and @p dv, and every later step with it:
 * the integration is linear in position and velocity, so a correction at that
 * time shifts the position later by dp + dv * elapsed. The output keeps its
 * position and velocity: the correction comes through pending.
 */
static void
correct_since(struct pico4_fusion *f, int64_t ts_ns, struct xrt_vec3 dp, struct xrt_vec3 dv)
{
	for (uint32_t i = f->count; i-- > 0;) {
		struct pico4_fusion_step *s = step_at(f, i);
		if (s->ts_ns < ts_ns) {
			break;
		}
		s->p = v_add(s->p, v_add(dp, v_scale(dv, (float)(s->ts_ns - ts_ns) * 1e-9f)));
		s->v = v_add(s->v, dv);
	}
	struct xrt_vec3 now = v_add(dp, v_scale(dv, (float)(f->ts_ns - ts_ns) * 1e-9f));
	f->p = v_add(f->p, now);
	f->v = v_add(f->v, dv);
	f->pending = v_sub(f->pending, now);
	f->pending_rate = v_sub(f->pending_rate, dv);
}

//! Re-anchors on @p m and takes its velocity: the position stays, the motion continues.
static enum pico4_fusion_result
relock(struct pico4_fusion *f, const struct pico4_fusion_slam *m, const struct pico4_fusion_step *then)
{
	anchor_at(f, m, then);
	struct pico4_fusion_slam w = to_world(f, m);
	correct_since(f, m->ts_ns, (struct xrt_vec3){0, 0, 0}, v_sub(w.velocity, then->v));
	f->target = q_normalize(q_mul(w.orientation, q_conj(then->q3dof)));
	remember(f, m->ts_ns, w.position, w.velocity, then->imu_dv);
	f->lost = false;
	f->slam_ts_ns = m->ts_ns;
	return PICO4_FUSION_ANCHORED;
}

enum pico4_fusion_result
pico4_fusion_slam(struct pico4_fusion *f, const struct pico4_fusion_slam *m)
{
	struct pico4_fusion_step then;
	if (m->ts_ns <= f->seen_ts_ns) {
		return PICO4_FUSION_IGNORED; // seen already
	}
	f->seen_ts_ns = m->ts_ns;
	if (!lookup(f, m->ts_ns, &then) || (f->lost && m->ts_ns < f->ignore_until_ns)) {
		return PICO4_FUSION_IGNORED;
	}

	if (!f->started) {
		// The filter's world starts as the tracker's.
		f->offset = q_normalize(q_mul(m->orientation, q_conj(then.q3dof)));
		f->target = f->offset;
		f->p = m->position;
		f->v = m->velocity;
		for (uint32_t i = f->count; i-- > 0;) {
			struct pico4_fusion_step *s = step_at(f, i);
			if (s->ts_ns < m->ts_ns) {
				break;
			}
			s->p = v_add(m->position, v_scale(m->velocity, (float)(s->ts_ns - m->ts_ns) * 1e-9f));
			s->v = m->velocity;
			f->p = s->p;
		}
		remember(f, m->ts_ns, m->position, m->velocity, then.imu_dv);
		f->started = true;
		f->still_since_ns = f->ts_ns;
		f->dv_since_ns = f->ts_ns;
		f->slam_ts_ns = m->ts_ns;
		return PICO4_FUSION_STARTED;
	}

	struct pico4_fusion_slam w = to_world(f, m);
	float inn = innovation(f, m->ts_ns, w.velocity, then.imu_dv);
	f->last_innovation = inn > 0.0f ? inn : 0.0f;
	f->last_speed = v_len(w.velocity);
	bool still = f->started && m->ts_ns - then.still_since_ns >= f->c.runaway_window_ns;
	bool implausible = f->last_speed > f->c.max_speed || inn > f->c.runaway_mps ||
	                   (still && f->last_speed > f->c.still_mps);

	if (f->lost) {
		// Hold until the tracker is consistent again: plausible, and moving the way its velocity says.
		if (f->pose_count > 0) {
			const struct pico4_fusion_seen *prev = seen_at(f, f->pose_count - 1);
			float dt = (float)(m->ts_ns - prev->ts_ns) * 1e-9f;
			struct xrt_vec3 expected = v_add(prev->p, v_scale(v_add(prev->v, w.velocity), 0.5f * dt));
			implausible = implausible || v_len(v_sub(w.position, expected)) > f->c.gate_m || dt > 0.2f;
		}
		if (implausible) {
			f->consistent = 0;
			f->pose_count = 0;
		}
		remember(f, m->ts_ns, w.position, w.velocity, then.imu_dv);
		// Consistent long enough to check its velocity against the IMU's over the window.
		if (implausible || ++f->consistent < f->c.relock_poses || inn < 0.0f) {
			f->rejected++;
			return PICO4_FUSION_OUTLIER;
		}
		return relock(f, m, &then);
	}

	if (implausible) {
		if (++f->implausible >= f->c.runaway_poses) {
			f->lost = true;
			f->runaways++;
			f->implausible = 0;
			f->outliers = 0;
			f->consistent = 0;
			f->pose_count = 0;
			return PICO4_FUSION_RUNAWAY;
		}
		f->rejected++;
		return PICO4_FUSION_OUTLIER;
	}
	f->implausible = 0;

	struct xrt_quat measured = q_normalize(q_mul(w.orientation, q_conj(then.q3dof)));
	struct xrt_vec3 dp = v_sub(w.position, then.p), dv = v_sub(w.velocity, then.v);
	f->last_error_m = v_len(dp);
	f->last_error_rad = q_angle(measured, f->offset);
	bool stale = f->ts_ns - f->slam_ts_ns > f->c.stale_ns;
	if (!stale && (f->last_error_m > f->c.gate_m || f->last_error_rad > f->c.gate_rad)) {
		if (++f->outliers < f->c.persist) {
			f->rejected++;
			return PICO4_FUSION_OUTLIER;
		}
		// The tracker jumped and stayed there: move its world, not the view.
		return relock(f, m, &then);
	}
	remember(f, m->ts_ns, w.position, w.velocity, then.imu_dv);

	float k_p = 1.0f, k_v = 1.0f;
	if (!stale) {
		float dt = (float)(m->ts_ns - f->slam_ts_ns) * 1e-9f;
		dt = dt < 0.001f ? 0.001f : dt > 0.5f ? 0.5f : dt;
		k_p = 1.0f - expf(-dt / f->c.position_s);
		k_v = 1.0f - expf(-dt / f->c.velocity_s);
	}
	correct_since(f, m->ts_ns, v_scale(dp, k_p), v_scale(dv, k_v));
	f->target = measured;
	f->outliers = 0;
	f->slam_ts_ns = m->ts_ns;
	if (stale) {
		f->resets++;
		return PICO4_FUSION_RESET;
	}
	f->corrected++;
	return PICO4_FUSION_CORRECTED;
}

void
pico4_fusion_relation(const struct pico4_fusion *f, struct xrt_space_relation *out)
{
	struct xrt_space_relation rel = XRT_SPACE_RELATION_ZERO;
	struct xrt_quat q = q_normalize(q_mul(f->offset, f->q3dof));
	struct xrt_vec3 w = q_rotate(q, f->gyro);
	rel.pose.orientation = q;
	rel.angular_velocity = w;
	rel.relation_flags = (enum xrt_space_relation_flags)(XRT_SPACE_RELATION_ORIENTATION_VALID_BIT |
	                                                      XRT_SPACE_RELATION_ORIENTATION_TRACKED_BIT |
	                                                      XRT_SPACE_RELATION_ANGULAR_VELOCITY_VALID_BIT);
	if (f->started) {
		struct xrt_vec3 lever = q_rotate(q, f->c.lever);
		rel.pose.position = v_add(v_add(f->p, f->pending), lever);
		rel.linear_velocity = v_add(v_add(f->v, f->pending_rate), v_cross(w, lever));
		rel.relation_flags |= XRT_SPACE_RELATION_POSITION_VALID_BIT | XRT_SPACE_RELATION_LINEAR_VELOCITY_VALID_BIT;
		if (pico4_fusion_tracking(f)) {
			rel.relation_flags |= XRT_SPACE_RELATION_POSITION_TRACKED_BIT;
		}
	}
	*out = rel;
}
