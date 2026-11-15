// Copyright 2026, overpressure contributors.
// SPDX-License-Identifier: BSL-1.0
/*
 * Overpressure Settings' layout and decisions (overpressure-settings.cpp), apart from SteamVR and
 * drawing so that the tests can reach them: a menu on the left, the chosen page on the right. Its
 * pages are what the PICO 4 cannot do the Steam Frame's way: the lenses have no wheel
 * (pico4_lens_panel.h), and SteamVR cannot pair its controllers.
 */
#pragma once

#include "pico4_lens_panel.h"

#include <stdbool.h>

// The whole tab, in its pixels, y down; a page is PICO4_PANEL_W x PICO4_PANEL_H from its origin.
#define PICO4_SETTINGS_W 1100
#define PICO4_SETTINGS_H 620
#define PICO4_SETTINGS_MENU_W 300.0
#define PICO4_SETTINGS_PAGE_X 300.0
#define PICO4_SETTINGS_PAGE_Y 100.0
#define PICO4_SETTINGS_TITLE_Y 30.0 //!< the page's title, above its origin

enum pico4_settings_page
{
	PICO4_SETTINGS_NONE = -1,
	PICO4_SETTINGS_LENSES,
	PICO4_SETTINGS_CONTROLLERS,
	PICO4_SETTINGS_PAGES,
};

// The menu's entries.
#define PICO4_SETTINGS_ENTRY_X 16.0
#define PICO4_SETTINGS_ENTRY_W 268.0
#define PICO4_SETTINGS_ENTRY_Y 110.0
#define PICO4_SETTINGS_ENTRY_H 64.0
#define PICO4_SETTINGS_ENTRY_STEP 76.0

//! The menu's entry at (@p x, @p y) of the tab.
static inline enum pico4_settings_page
pico4_settings_menu_hit(double x, double y)
{
	if (x < PICO4_SETTINGS_ENTRY_X || x >= PICO4_SETTINGS_ENTRY_X + PICO4_SETTINGS_ENTRY_W || y < PICO4_SETTINGS_ENTRY_Y) {
		return PICO4_SETTINGS_NONE;
	}
	const int i = (int)((y - PICO4_SETTINGS_ENTRY_Y) / PICO4_SETTINGS_ENTRY_STEP);
	const double inside = y - PICO4_SETTINGS_ENTRY_Y - i * PICO4_SETTINGS_ENTRY_STEP;
	return i < PICO4_SETTINGS_PAGES && inside < PICO4_SETTINGS_ENTRY_H ? (enum pico4_settings_page)i : PICO4_SETTINGS_NONE;
}

// The controllers' page, in the page's pixels from its 30 px margin: a row for each hand with its
// Pair button, the instructions under them, and on the panel in front of the wearer a Close button.
#define PICO4_CONTROLLERS_MARGIN 30.0
#define PICO4_CONTROLLERS_W (PICO4_PANEL_W - 2 * PICO4_CONTROLLERS_MARGIN)
#define PICO4_CONTROLLERS_ROW_Y 0.0
#define PICO4_CONTROLLERS_ROW_H 84.0
#define PICO4_CONTROLLERS_ROW_STEP 100.0
#define PICO4_CONTROLLERS_PAIR_X 548.0
#define PICO4_CONTROLLERS_PAIR_W 180.0
#define PICO4_CONTROLLERS_PAIR_INSET 12.0
#define PICO4_CONTROLLERS_TEXT_Y 220.0
#define PICO4_CONTROLLERS_CLOSE_Y 410.0
#define PICO4_CONTROLLERS_CLOSE_H 70.0

enum pico4_controllers_widget
{
	PICO4_CONTROLLERS_NONE = -1,
	PICO4_CONTROLLERS_PAIR_LEFT,
	PICO4_CONTROLLERS_PAIR_RIGHT,
	PICO4_CONTROLLERS_CLOSE,
};

/*!
 * The widget at (@p x, @p y) of the controllers' page: a hand's Pair button only where it is
 * shown (that controller missing, no pairing running), Close only on the panel in front.
 */
static inline enum pico4_controllers_widget
pico4_controllers_hit(double x, double y, bool pair_left, bool pair_right, bool close)
{
	if (x < PICO4_CONTROLLERS_PAIR_X || x >= PICO4_CONTROLLERS_PAIR_X + PICO4_CONTROLLERS_PAIR_W) {
		return PICO4_CONTROLLERS_NONE;
	}
	for (int hand = 0; hand < 2; hand++) {
		const double top = PICO4_CONTROLLERS_ROW_Y + hand * PICO4_CONTROLLERS_ROW_STEP + PICO4_CONTROLLERS_PAIR_INSET;
		if ((hand == 0 ? pair_left : pair_right) && y >= top && y < top + PICO4_CONTROLLERS_ROW_H - 2 * PICO4_CONTROLLERS_PAIR_INSET) {
			return hand == 0 ? PICO4_CONTROLLERS_PAIR_LEFT : PICO4_CONTROLLERS_PAIR_RIGHT;
		}
	}
	if (close && y >= PICO4_CONTROLLERS_CLOSE_Y && y < PICO4_CONTROLLERS_CLOSE_Y + PICO4_CONTROLLERS_CLOSE_H) {
		return PICO4_CONTROLLERS_CLOSE;
	}
	return PICO4_CONTROLLERS_NONE;
}

/*!
 * Whether the controllers' panel comes up in front of the wearer with the station listening
 * hands-free: no controller connected (as far as the driver said) and no pairing running, the
 * setup over (it pairs itself while it runs) and the controllers given @p wait_s since the start
 * to reconnect by themselves.
 */
static inline bool
pico4_settings_rescue(bool setup_pending, bool known, bool left, bool right, int pairing, double since_start, double wait_s)
{
	return !setup_pending && known && !left && !right && pairing == 0 && since_start >= wait_s;
}
