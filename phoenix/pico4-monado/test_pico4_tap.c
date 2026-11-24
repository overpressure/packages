// Copyright 2026, overpressure contributors.
// SPDX-License-Identifier: BSL-1.0
/*
 * Unit test of the double-tap detector with a synthetic accelerometer: the ICM-42688's
 * 1024 Hz, gravity, noise of its floor at rest, and scenes built of knocks (half sine
 * pulses), ramps, pushes and waves (walking, running, shaking the head). Two knocks close
 * together out of rest count once; one knock, a knock with ringing, ramps, pushes, waves, a
 * knock while moving, knocks too far apart or too unlike each other count never.
 *
 * Each rule of the detector decides at least one scene here: break one, and a check fails.
 */

#include "pico4_tap.h"

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
#define DT (1000000000LL / 1024) //!< The ICM-42688's period at the 1000 Hz request.
#define G 9.80665f

static uint32_t seed = 20261124;

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

//! Uniformly from 0 to 1.
static float
chance(void)
{
	seed = seed * 1664525u + 1013904223u;
	return (float)(seed >> 8) / 16777216.0f;
}

enum kind
{
	PULSE, //!< A knock: a half sine of its length.
	RAMP,  //!< A push's slow start: a straight rise to its top over its length, then nothing.
	HOLD,  //!< A push: a constant acceleration.
	WAVE,  //!< A wave of one frequency.
	TURN,  //!< Gravity turns over from here on: the headset is laid down the other way.
	AGAIN, //!< The sample at this time reaches the detector twice.
};

struct event
{
	enum kind kind;
	double from_s;   //!< Start, s.
	double to_s;     //!< End of a push or a wave, s.
	double length_s; //!< A knock's or a ramp's length, s.
	float amp;       //!< m/s².
	float axis[3];   //!< Where it points.
	float hz;        //!< A wave's frequency.
};

static struct event
knock(double at_s, float amp)
{
	return (struct event){.kind = PULSE, .from_s = at_s, .length_s = 0.008, .amp = amp, .axis = {1, 0, 0}};
}

//! A hard knock, twice as long, at the accelerometer's limit of 8 g.
static struct event
thump(double at_s)
{
	return (struct event){.kind = PULSE, .from_s = at_s, .length_s = 0.016, .amp = 8 * G, .axis = {1, 0, 0}};
}

static struct event
ramp(double at_s, float amp)
{
	return (struct event){.kind = RAMP, .from_s = at_s, .length_s = 0.05, .amp = amp, .axis = {1, 0, 0}};
}

static struct event
push(double from_s, double to_s, float amp)
{
	return (struct event){.kind = HOLD, .from_s = from_s, .to_s = to_s, .amp = amp, .axis = {1, 0, 0}};
}

static struct event
wave(double from_s, double to_s, float amp, float hz)
{
	return (struct event){.kind = WAVE, .from_s = from_s, .to_s = to_s, .amp = amp, .axis = {0.6f, 0.8f, 0}, .hz = hz};
}

static struct event
turn(double at_s)
{
	return (struct event){.kind = TURN, .from_s = at_s};
}

static struct event
again(double at_s)
{
	return (struct event){.kind = AGAIN, .from_s = at_s};
}

//! Everything the accelerometer reads besides gravity and noise.
struct scene
{
	const struct event *events;
	int count;
	double seconds;
	double gap_at_s, gap_s; //!< No samples between these times, as over a suspend.
	double reset_at_s;      //!< The detector is reset here.
};

struct result
{
	int taps;
	double first_s, last_s; //!< Of the detections, s; negative without one.
};

//! The scenes' acceleration at @p t, s, into @p out.
static void
scene_at(const struct scene *sc, double t, float out[3])
{
	out[0] = out[1] = out[2] = 0.0f;
	for (int i = 0; i < sc->count; i++) {
		const struct event *e = &sc->events[i];
		if (e->kind == PULSE || e->kind == RAMP) {
			const double t2 = t - e->from_s;
			if (t2 >= 0 && t2 < e->length_s) {
				const float s = e->kind == PULSE ? e->amp * sinf((float)(M_PI * t2 / e->length_s))
				                                 : e->amp * (float)(t2 / e->length_s);
				for (int a = 0; a < 3; a++) {
					out[a] += s * e->axis[a];
				}
			}
		} else if ((e->kind == HOLD || e->kind == WAVE) && t >= e->from_s && t < e->to_s) {
			const float s = e->kind == WAVE ? e->amp * sinf((float)(2 * M_PI * e->hz * (t - e->from_s))) : e->amp;
			for (int a = 0; a < 3; a++) {
				out[a] += s * e->axis[a];
			}
		}
	}
}

//! The scene through the detector: what it finds, and when. Gravity along @p up.
static struct result
run(const struct scene *sc, const float up[3])
{
	struct pico4_tap tap;
	pico4_tap_init(&tap, &pico4_tap_default_config);
	struct result r = {.taps = 0, .first_s = -1, .last_s = -1};
	const float norm = sqrtf(up[0] * up[0] + up[1] * up[1] + up[2] * up[2]);
	double turn_at_s = -1, again_at_s = -1;
	for (int i = 0; i < sc->count; i++) {
		if (sc->events[i].kind == TURN) {
			turn_at_s = sc->events[i].from_s;
		} else if (sc->events[i].kind == AGAIN) {
			again_at_s = sc->events[i].from_s;
		}
	}
	double reset_at_s = sc->reset_at_s;
	for (int64_t k = 0; k * DT < (int64_t)(sc->seconds * 1e9); k++) {
		const double t = (double)(k * DT) / 1e9;
		if (sc->gap_s > 0 && t >= sc->gap_at_s && t < sc->gap_at_s + sc->gap_s) {
			continue;
		}
		if (reset_at_s >= 0 && t >= reset_at_s) {
			pico4_tap_reset(&tap);
			reset_at_s = -1;
		}
		float a[3];
		scene_at(sc, t, a);
		const float sign = t >= turn_at_s ? -1.0f : 1.0f;
		for (int axis = 0; axis < 3; axis++) {
			a[axis] += sign * G * up[axis] / norm + noise(0.02f);
		}
		bool found = pico4_tap_feed(&tap, k * DT, a[0], a[1], a[2]);
		if (t >= again_at_s && again_at_s >= 0) {
			again_at_s = -1;
			const bool twice = pico4_tap_feed(&tap, k * DT, a[0], a[1], a[2]);
			found = found || twice;
		}
		if (found) {
			if (r.taps == 0) {
				r.first_s = t;
			}
			r.last_s = t;
			r.taps++;
		}
	}
	return r;
}

//! A knock of 8 ms at @p s ends a double tap at its first sample under the release level, 7.8 ms
//! after it began: @p t is that sample's time.
#define ENDS_AT(t, s) ((t) > (s) + 0.0073 && (t) < (s) + 0.0083)

//! One double tap, found as the second of two knocks @p second_s ends.
static bool
found_at(const struct result *r, double second_s)
{
	return r->taps == 1 && ENDS_AT(r->first_s, second_s);
}

static const float up_right[3] = {0, 1, 0}, up_tilted[3] = {0.3f, 0.9f, 0.316f};

static void
test_rest_and_single_knocks(void)
{
	for (int tilt = 0; tilt < 2; tilt++) {
		struct result r = run(&(struct scene){.seconds = 30}, tilt ? up_tilted : up_right);
		EXPECT(r.taps == 0);
	}
	const struct event one[] = {knock(2.0, 25)};
	const struct event light[] = {knock(2.0, 12)};
	const struct event far[] = {knock(2.0, 25), knock(3.5, 25), knock(5.0, 25)};
	for (int tilt = 0; tilt < 2; tilt++) {
		const float *up = tilt ? up_tilted : up_right;
		EXPECT(run(&(struct scene){.events = one, .count = 1, .seconds = 5}, up).taps == 0);
		EXPECT(run(&(struct scene){.events = light, .count = 1, .seconds = 5}, up).taps == 0);
		EXPECT(run(&(struct scene){.events = far, .count = 3, .seconds = 7}, up).taps == 0);
	}
}

//! Two knocks, the first at 2 s and the second of @p second m/s² at @p second_s.
static struct result
pair_at(double second_s, float second)
{
	const struct event e[] = {knock(2.0, 25), knock(second_s, second)};
	return run(&(struct scene){.events = e, .count = 2, .seconds = 5}, up_right);
}

//! Two knocks @p apart_s, the first at 2 s, the second of @p second m/s².
static struct result
pair(double apart_s, float second)
{
	return pair_at(2.0 + apart_s, second);
}

static void
test_double_taps(void)
{
	for (int tilt = 0; tilt < 2; tilt++) {
		const float *up = tilt ? up_tilted : up_right;
		const struct event e[] = {knock(2.0, 25), knock(2.25, 25)};
		const struct result r = run(&(struct scene){.events = e, .count = 2, .seconds = 5}, up);
		EXPECT(found_at(&r, 2.25));
	}
	// The gap runs from the first knock's spike to the second's, 80 ms to 400 ms. One sample outside
	// either end, 79.1 ms and 400.4 ms, is no double tap.
	EXPECT(pair(0.080, 25).taps == 1);
	EXPECT(pair(0.395, 25).taps == 1);
	EXPECT(pair_at(2.0787, 25).taps == 0);
	EXPECT(pair_at(2.399, 25).taps == 1);
	EXPECT(pair_at(2.4, 25).taps == 0);
	// Hard knocks, soft ones and unlike ones: the peaks of a double tap's differ by less than four times.
	EXPECT(pair(0.25, 80).taps == 1);
	EXPECT(pair(0.25, 17).taps == 1);
	EXPECT(pair(0.25, 120).taps == 0);
	EXPECT(pair(0.25, 5).taps == 0);
}

static void
test_slow_and_sustained(void)
{
	// A push that builds up over 50 ms is no knock, twice over: its top comes too late. A knock 90 ms
	// after its top is none either: the refused spike locks out for a width.
	const struct event ramps[] = {ramp(2.0, 30), ramp(2.25, 30)};
	const struct event ramps_close[] = {ramp(2.0, 30), ramp(2.125, 30)};
	const struct event ramp_then_knock[] = {ramp(2.0, 30), knock(2.14, 25)};
	// A knock as the headset is laid down turned: the rest of gravity that stays is no spike for
	// good, and a double tap is found afterwards.
	const struct event turned[] = {knock(2.0, 25), push(2.0, 10, 8), knock(3.0, 25), knock(3.25, 25)};
	// A knock 52 ms after a push of 61 ms is not the start of a double tap: the headset was moving,
	// and it has to settle for a width.
	const struct event push_then_knocks[] = {push(2.0, 2.061, 25), knock(2.113, 25), knock(2.363, 25)};
	// Hard knocks are no movement.
	const struct event thumps[] = {thump(2.0), thump(2.25)};
	EXPECT(run(&(struct scene){.events = ramps, .count = 2, .seconds = 5}, up_right).taps == 0);
	EXPECT(run(&(struct scene){.events = ramps_close, .count = 2, .seconds = 5}, up_right).taps == 0);
	EXPECT(run(&(struct scene){.events = ramp_then_knock, .count = 2, .seconds = 5}, up_right).taps == 0);
	EXPECT(run(&(struct scene){.events = push_then_knocks, .count = 3, .seconds = 5}, up_right).taps == 0);
	EXPECT(run(&(struct scene){.events = turned, .count = 4, .seconds = 5}, up_right).taps == 1);
	EXPECT(run(&(struct scene){.events = thumps, .count = 2, .seconds = 5}, up_right).taps == 1);
}

static void
test_ringing_bounces_and_more_knocks(void)
{
	// The first knock's bounce and a long ringing are no second knock; the real one still counts.
	const struct event bounce[] = {knock(2.0, 25), knock(2.03, 20), knock(2.25, 25)};
	const struct result r = run(&(struct scene){.events = bounce, .count = 3, .seconds = 5}, up_right);
	EXPECT(found_at(&r, 2.25));
	const struct event ringing[] = {knock(2.0, 25), wave(2.008, 2.09, 20, 60)};
	EXPECT(run(&(struct scene){.events = ringing, .count = 2, .seconds = 5}, up_right).taps == 0);
	// A bounce, and the headset trembling on after it: the signal has to come down before a knock
	// counts again.
	const struct event trembling[] = {knock(2.0, 25), knock(2.03, 20), push(2.03, 2.1, 9), knock(2.09, 25)};
	EXPECT(run(&(struct scene){.events = trembling, .count = 4, .seconds = 5}, up_right).taps == 0);
	// Three knocks and four: one double tap, the rest is inside the lockout after it.
	const struct event three[] = {knock(2.0, 25), knock(2.25, 25), knock(2.5, 25)};
	const struct event four[] = {knock(2.0, 25), knock(2.25, 25), knock(2.5, 25), knock(2.75, 25)};
	EXPECT(run(&(struct scene){.events = three, .count = 3, .seconds = 5}, up_right).taps == 1);
	EXPECT(run(&(struct scene){.events = four, .count = 4, .seconds = 5}, up_right).taps == 1);
	// A double tap long after in the scene's time scale is found again.
	const struct event again_[] = {knock(2.0, 25), knock(2.25, 25), knock(5.0, 25), knock(5.25, 25)};
	const struct result later = run(&(struct scene){.events = again_, .count = 4, .seconds = 8}, up_right);
	EXPECT(later.taps == 2 && later.first_s < 3);
	EXPECT(later.taps == 2 && ENDS_AT(later.last_s, 5.25));
}

static void
test_no_knocks_in_movement(void)
{
	const struct event walk[] = {wave(0, 20, 4, 2)};
	const struct event run_[] = {wave(0, 20, 14, 3)};
	const struct event shake[] = {wave(2, 5, 30, 4)};
	const struct event shake_hard[] = {wave(2, 5, 60, 2.5)};
	const struct event nod[] = {wave(2, 2.5, 30, 2), wave(2.25, 2.75, 30, 2)};
	// Pushes, 100 ms each, are no knocks, in pairs or one followed by a knock; the detector finds a
	// double tap after one.
	const struct event pushes[] = {push(2.0, 2.1, 25), push(2.15, 2.25, 25), push(3.0, 3.1, 25), knock(3.35, 25)};
	const struct event after_push[] = {push(2.0, 2.1, 25), knock(3.0, 25), knock(3.25, 25)};
	const struct event while_running[] = {wave(0, 6, 14, 3), knock(3.0, 25), knock(3.25, 25)};
	// A knock while walking counts: the walking is under the envelope's limit.
	const struct event walking[] = {wave(0, 6, 4, 2), knock(3.0, 25), knock(3.25, 25)};
	EXPECT(run(&(struct scene){.events = walk, .count = 1, .seconds = 20}, up_right).taps == 0);
	EXPECT(run(&(struct scene){.events = run_, .count = 1, .seconds = 20}, up_right).taps == 0);
	EXPECT(run(&(struct scene){.events = shake, .count = 1, .seconds = 6}, up_right).taps == 0);
	EXPECT(run(&(struct scene){.events = shake_hard, .count = 1, .seconds = 6}, up_right).taps == 0);
	EXPECT(run(&(struct scene){.events = nod, .count = 2, .seconds = 4}, up_right).taps == 0);
	EXPECT(run(&(struct scene){.events = pushes, .count = 4, .seconds = 5}, up_right).taps == 0);
	EXPECT(run(&(struct scene){.events = after_push, .count = 3, .seconds = 5}, up_right).taps == 1);
	EXPECT(run(&(struct scene){.events = while_running, .count = 3, .seconds = 7}, up_right).taps == 0);
	EXPECT(run(&(struct scene){.events = walking, .count = 3, .seconds = 7}, up_right).taps == 1);
}

static void
test_turns_gaps_and_resets(void)
{
	// The headset is laid down the other way: gravity is learnt again, and a double tap counts.
	const struct event turned[] = {turn(2.0), knock(3.0, 25), knock(3.25, 25)};
	const struct result r = run(&(struct scene){.events = turned, .count = 3, .seconds = 5}, up_right);
	EXPECT(found_at(&r, 3.25));
	// No samples for three seconds, as over a suspend, and the headset turned meanwhile: the detector
	// starts over, and works on.
	const struct event e[] = {knock(1.0, 25), knock(1.25, 25), turn(3.0), knock(6.0, 25), knock(6.25, 25)};
	struct result later = run(&(struct scene){.events = e, .count = 5, .seconds = 8, .gap_at_s = 2, .gap_s = 3}, up_right);
	EXPECT(later.taps == 2 && later.first_s < 2 && ENDS_AT(later.last_s, 6.25));
	// The system sleeps while a knock is being judged, or while the first waits for the second: the
	// knock is not finished by the first sample after the wake, and the first is not waiting any more.
	const struct event mid[] = {knock(2.0, 25), knock(2.25, 25)};
	EXPECT(run(&(struct scene){.events = mid, .count = 2, .seconds = 8, .gap_at_s = 2.252, .gap_s = 3}, up_right).taps == 0);
	EXPECT(run(&(struct scene){.events = mid, .count = 2, .seconds = 8, .gap_at_s = 2.02, .gap_s = 3}, up_right).taps == 0);
	// A reset forgets a first knock that is still waiting for its second.
	const struct event after[] = {knock(2.0, 25), knock(2.25, 25), knock(4.0, 25), knock(4.25, 25)};
	later = run(&(struct scene){.events = after, .count = 4, .seconds = 6, .reset_at_s = 2.1}, up_right);
	EXPECT(later.taps == 1 && ENDS_AT(later.last_s, 4.25));
	// One sample reaching the detector twice starts it over: the second knock is a first one.
	const struct event twice[] = {knock(2.0, 25), again(2.1), knock(2.25, 25)};
	EXPECT(run(&(struct scene){.events = twice, .count = 3, .seconds = 5}, up_right).taps == 0);
	// Two samples of one time, and one that goes back: neither counts, and the detector works on.
	const struct event one[] = {knock(3.0, 25), knock(3.25, 25)};
	for (int back = 0; back < 2; back++) {
		struct pico4_tap tap;
		pico4_tap_init(&tap, &pico4_tap_default_config);
		EXPECT(!pico4_tap_feed(&tap, 1000 * MS, 0, G, 0));
		EXPECT(!pico4_tap_feed(&tap, back ? 999 * MS : 1000 * MS, 0, G, 0));
		struct result found = {.taps = 0};
		for (int64_t k = 0; k * DT < 5 * 1000 * MS; k++) {
			const double t = (double)(k * DT) / 1e9;
			float a[3];
			scene_at(&(struct scene){.events = one, .count = 2}, t, a);
			for (int axis = 0; axis < 3; axis++) {
				a[axis] += G * up_right[axis] + noise(0.02f);
			}
			if (pico4_tap_feed(&tap, 2000 * MS + k * DT, a[0], a[1], a[2])) {
				found.taps++;
				found.last_s = t;
			}
		}
		EXPECT(found.taps == 1 && ENDS_AT(found.last_s, 3.25));
	}
}

static void
test_random_knocks(void)
{
	// Alone: each knock at least a second from the last. Together: pairs of random strength and spacing
	// inside the bounds, a second and a half apart.
	for (int round = 0; round < 20; round++) {
		struct event e[200];
		int n = 0;
		double t = 1;
		int expect = 0;
		while (t < 60) {
			e[n++] = knock(t, 20 + 40 * chance());
			if (round % 2) {
				e[n++] = knock(t + 0.1 + 0.25 * chance(), 20 + 40 * chance());
				expect++;
				t += 1.5;
			} else {
				t += 1 + chance();
			}
		}
		const struct result r = run(&(struct scene){.events = e, .count = n, .seconds = 61}, up_tilted);
		EXPECT(r.taps == expect);
	}
}

int
main(void)
{
	test_rest_and_single_knocks();
	test_double_taps();
	test_slow_and_sustained();
	test_ringing_bounces_and_more_knocks();
	test_no_knocks_in_movement();
	test_turns_gaps_and_resets();
	test_random_knocks();

	if (failures == 0) {
		printf("test_pico4_tap: all checks passed\n");
	}
	return failures == 0 ? 0 : 1;
}
