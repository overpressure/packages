// Copyright 2026, overpressure contributors.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  PICO 4: the wearer's double tap on the headset, from its accelerometer (see pico4_tap.h).
 *
 * @ingroup drv_pico4
 */

#include "pico4_tap.h"

#include <math.h>
#include <string.h>

const struct pico4_tap_config pico4_tap_default_config = {
    .gravity_s = 0.3f,
    .envelope_s = 0.15f,
    .spike_mpss = 15.0f,
    .release_mpss = 6.0f,
    .quiet_mpss = 4.0f,
    .width_ns = 60 * 1000 * 1000,
    .rise_ns = 20 * 1000 * 1000,
    .gap_min_ns = 80 * 1000 * 1000,
    .gap_max_ns = 400 * 1000 * 1000,
    .lockout_ns = 700 * 1000 * 1000,
    .peak_ratio = 4.0f,
};

//! The share of a step of @p dt_ns that a low-pass of @p seconds follows in one sample.
static float
follow(int64_t dt_ns, float seconds)
{
	return (float)dt_ns / ((float)dt_ns + seconds * 1e9f);
}

void
pico4_tap_init(struct pico4_tap *t, const struct pico4_tap_config *cfg)
{
	memset(t, 0, sizeof(*t));
	t->cfg = *cfg;
}

void
pico4_tap_reset(struct pico4_tap *t)
{
	t->state = PICO4_TAP_IDLE;
	t->started = false;
	t->settled = false;
}

bool
pico4_tap_feed(struct pico4_tap *t, int64_t ts_ns, float x, float y, float z)
{
	const struct pico4_tap_config *c = &t->cfg;
	const int64_t dt_ns = ts_ns - t->last_ns;
	if (!t->started || dt_ns <= 0 || dt_ns > c->gap_max_ns) {
		// The first sample, or a gap (the system slept): start over, taking this sample for rest.
		pico4_tap_reset(t);
		t->gravity[0] = x;
		t->gravity[1] = y;
		t->gravity[2] = z;
		t->envelope = 0.0f;
		t->started = true;
		t->last_ns = ts_ns;
		return false;
	}
	t->last_ns = ts_ns;

	const float lx = x - t->gravity[0], ly = y - t->gravity[1], lz = z - t->gravity[2];
	const float linear = sqrtf(lx * lx + ly * ly + lz * lz);
	// While a spike is being judged, gravity stays what it was: the spike is not to pull it. At all other
	// times it follows, however strong the signal: a headset knocked or laid down turned over rests on
	// another gravity, and it has to be learnt before any tap is found.
	// The motion envelope takes a spike as the spike level at most: a hard knock is no movement.
	if (t->state != PICO4_TAP_FIRST && t->state != PICO4_TAP_LAST) {
		const float a = follow(dt_ns, c->gravity_s);
		t->gravity[0] += lx * a;
		t->gravity[1] += ly * a;
		t->gravity[2] += lz * a;
	}
	const float tracked = linear < c->spike_mpss ? linear : c->spike_mpss;
	t->envelope += (tracked - t->envelope) * follow(dt_ns, c->envelope_s);

	switch (t->state) {
	case PICO4_TAP_IDLE:
		// Whether the headset moves is judged once a spike has ended: then a second one may count.
		if (linear >= c->spike_mpss) {
			t->start_ns = ts_ns;
			t->peak = linear;
			t->state = PICO4_TAP_FIRST;
		}
		break;
	case PICO4_TAP_FIRST:
	case PICO4_TAP_LAST:
		if (linear > t->peak) {
			if (ts_ns - t->start_ns > c->rise_ns) {
				// Still growing this late: a slow movement, no knock.
				t->until_ns = ts_ns + c->width_ns;
				t->state = PICO4_TAP_LOCKOUT;
				break;
			}
			t->peak = linear;
		}
		if (ts_ns - t->start_ns > c->width_ns) {
			// A sustained acceleration is no tap, and the headset was moving: let it settle.
			t->until_ns = ts_ns + c->width_ns;
			t->state = PICO4_TAP_LOCKOUT;
		} else if (linear < c->release_mpss) {
			if (t->state == PICO4_TAP_FIRST) {
				t->first_ns = t->start_ns;
				t->first_peak = t->peak;
				t->settled = true;
				t->state = PICO4_TAP_SECOND;
			} else {
				const float ratio = t->peak / t->first_peak;
				t->until_ns = ts_ns + c->lockout_ns;
				t->state = PICO4_TAP_LOCKOUT;
				return ratio >= 1.0f / c->peak_ratio && ratio <= c->peak_ratio;
			}
		}
		break;
	case PICO4_TAP_SECOND:
		if (t->envelope >= c->quiet_mpss) {
			// The wearer moves: the first tap counts for nothing, whatever comes now.
			t->state = PICO4_TAP_IDLE;
			break;
		}
		if (linear >= c->spike_mpss) {
			// Above the spike level again: the first tap's ringing, or the second tap. A ringing that
			// is still up when the gap opens is no second tap, so the signal has to have come down.
			const bool settled = t->settled;
			const int64_t since_ns = ts_ns - t->first_ns;
			t->settled = false;
			if (settled && since_ns >= c->gap_min_ns) {
				if (since_ns <= c->gap_max_ns) {
					t->start_ns = ts_ns;
					t->peak = linear;
					t->state = PICO4_TAP_LAST;
				} else {
					// Too late for the second tap: this spike is a new first one.
					t->start_ns = ts_ns;
					t->peak = linear;
					t->state = PICO4_TAP_FIRST;
				}
			}
		} else if (linear < c->release_mpss) {
			t->settled = true;
		}
		break;
	case PICO4_TAP_LOCKOUT:
		if (ts_ns >= t->until_ns) {
			t->state = PICO4_TAP_IDLE;
		}
		break;
	default: break;
	}
	return false;
}
