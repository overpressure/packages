// Copyright 2026, overpressure contributors.
// SPDX-License-Identifier: BSL-1.0
/*
 * The onboarding's pages, layout and decisions (pico4-onboarding.cpp), apart from SteamVR and
 * drawing so that the tests can reach them.
 */
#pragma once

#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#define PICO4_TOUR_W 1000
#define PICO4_TOUR_H 600

//! Seconds after the start before missing controllers count as missing (they reconnect by themselves).
#define PICO4_TOUR_CONTROLLER_WAIT_S 20.0
//! How long each hands-free pairing listens, as `pico4-station pair` by default.
#define PICO4_TOUR_PAIR_S 60

enum pico4_tour_page
{
	PICO4_TOUR_CONTROLLERS,
	PICO4_TOUR_SAFETY,
	PICO4_TOUR_CONTROLS,
	PICO4_TOUR_PLAYSPACE,
	PICO4_TOUR_DONE,
	PICO4_TOUR_PAGES,
};

// The pages' texts, in every language, are in pico4-onboarding.strings; what to press to pair one
// controller is PICO OS's instruction (its station service listens meanwhile).

enum pico4_tour_button
{
	PICO4_TOUR_NONE = -1,
	PICO4_TOUR_SKIP,
	PICO4_TOUR_PAIR, //!< On the controllers page, with one controller missing.
	PICO4_TOUR_NEXT,
};

// The buttons' row, in the page's pixels, y down.
#define PICO4_TOUR_BUTTON_Y 480.0
#define PICO4_TOUR_BUTTON_H 80.0
#define PICO4_TOUR_BUTTON_W 260.0
#define PICO4_TOUR_SKIP_X 40.0
#define PICO4_TOUR_PAIR_X 370.0
#define PICO4_TOUR_NEXT_X 700.0

static inline enum pico4_tour_button
pico4_tour_hit(double x, double y, bool pair_shown)
{
	if (y < PICO4_TOUR_BUTTON_Y || y >= PICO4_TOUR_BUTTON_Y + PICO4_TOUR_BUTTON_H) {
		return PICO4_TOUR_NONE;
	}
	if (x >= PICO4_TOUR_SKIP_X && x < PICO4_TOUR_SKIP_X + PICO4_TOUR_BUTTON_W) {
		return PICO4_TOUR_SKIP;
	}
	if (pair_shown && x >= PICO4_TOUR_PAIR_X && x < PICO4_TOUR_PAIR_X + PICO4_TOUR_BUTTON_W) {
		return PICO4_TOUR_PAIR;
	}
	if (x >= PICO4_TOUR_NEXT_X && x < PICO4_TOUR_NEXT_X + PICO4_TOUR_BUTTON_W) {
		return PICO4_TOUR_NEXT;
	}
	return PICO4_TOUR_NONE;
}

/*!
 * Whether Steam's out-of-box experience is done: `CompletedOOBE` set to 1 in the client's
 * `registry.vdf` (`~/.steam/registry.vdf`), as the client keeps it (docs/session.md).
 */
static inline bool
pico4_tour_oobe_done(const char *vdf)
{
	const char *key = "\"CompletedOOBE\"";
	for (const char *p = vdf != NULL ? strstr(vdf, key) : NULL; p != NULL; p = strstr(p + 1, key)) {
		const char *v = p + strlen(key);
		while (*v == ' ' || *v == '\t') {
			v++;
		}
		if (strncmp(v, "\"1\"", 3) == 0) {
			return true;
		}
	}
	return false;
}

/*!
 * The controller to pair hands-free: with no controller connected the laser cannot reach the
 * pages, so the onboarding lets the station listen for one hand after the other. 1 left, 2 right,
 * 0 none (a controller is connected). @p last is the hand tried last (0 none yet).
 */
static inline int
pico4_tour_hands_free(bool left, bool right, int last)
{
	if (left || right) {
		return 0;
	}
	return last == 1 ? 2 : 1;
}

//! The controller the Pair button pairs: the missing one when the other one points; 0 for none.
static inline int
pico4_tour_pair_button(bool left, bool right)
{
	return left == right ? 0 : left ? 2 : 1;
}

/*!
 * Whether the tour starts by itself at this start: once, after Steam's OOBE, until it is done or
 * skipped (its mark on the data partition).
 */
static inline bool
pico4_tour_starts(bool oobe_done, bool done_mark)
{
	return oobe_done && !done_mark;
}

//! The page after @p page; PICO4_TOUR_PAGES after the last.
static inline enum pico4_tour_page
pico4_tour_next(enum pico4_tour_page page)
{
	return page + 1 < PICO4_TOUR_PAGES ? (enum pico4_tour_page)(page + 1) : PICO4_TOUR_PAGES;
}

/*!
 * One answer of the driver's pairing socket (pico4_station.h) for @p hand ("left" or "right"):
 * 1 paired, 0 still pairing, -1 failed (refused, no answer, timeout, an error).
 */
static inline int
pico4_tour_pair_answer(const char *answer)
{
	if (strncmp(answer, "paired ", 7) == 0) {
		return 1;
	}
	return strncmp(answer, "pairing ", 8) == 0 ? 0 : -1;
}

/*!
 * The driver's status answer ("left connected SERIAL\nright disconnected"): each controller's
 * state; false for anything else.
 */
static inline bool
pico4_tour_parse_status(const char *answer, bool *left, bool *right)
{
	const char *r = strstr(answer, "\nright ");
	if (strncmp(answer, "left ", 5) != 0 || r == NULL) {
		return false;
	}
	*left = strncmp(answer + 5, "connected", 9) == 0;
	*right = strncmp(r + 7, "connected", 9) == 0;
	return true;
}
