// Copyright 2026, overpressure contributors.
// SPDX-License-Identifier: BSL-1.0
/*
 * The setup panel's size and its row of buttons (pico4-onboarding.cpp), apart from SteamVR and
 * drawing so that the tests can reach them; its pages are in pico4_setup.h.
 */
#pragma once

#include <stdbool.h>

#define PICO4_TOUR_W 1000
#define PICO4_TOUR_H 600

// The pages' texts, in every language, are in pico4-onboarding.strings; what to press to pair one
// controller is PICO OS's instruction (its station service listens meanwhile).

enum pico4_tour_button
{
	PICO4_TOUR_NONE = -1,
	PICO4_TOUR_SKIP,
	PICO4_TOUR_PAIR, //!< The middle button: Pair, Refresh, Turn on Wi-Fi.
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
