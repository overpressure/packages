// Copyright 2026, overpressure contributors.
// SPDX-License-Identifier: BSL-1.0
/*
 * The lens panel's layout and presets (pico4-lens-panel.cpp), apart from SteamVR and drawing so
 * that the tests can reach them.
 */
#pragma once

#include <stdbool.h>
#include <stdio.h>

#define PICO4_PANEL_W 800
#define PICO4_PANEL_H 500
#define PICO4_PANEL_MM_MIN 62.0
#define PICO4_PANEL_MM_MAX 72.0
#define PICO4_PANEL_STEP_MM 0.5
#define PICO4_PANEL_PRESETS 4
#define PICO4_PANEL_HOLD_S 1.0 //!< Holding a preset this long stores the separation in it.

enum pico4_panel_widget
{
	PICO4_PANEL_NONE = -1,
	PICO4_PANEL_PRESET0 = 0, //!< ..PICO4_PANEL_PRESET0 + PICO4_PANEL_PRESETS - 1
	PICO4_PANEL_MINUS = PICO4_PANEL_PRESETS,
	PICO4_PANEL_PLUS,
	PICO4_PANEL_SLIDER,
};

// In the panel's pixels, y down.
#define PICO4_PANEL_BUTTON_X0 30.0 //!< The minus button; the plus button mirrors it.
#define PICO4_PANEL_BUTTON_W 80.0
#define PICO4_PANEL_SLIDER_X0 130.0
#define PICO4_PANEL_SLIDER_X1 670.0
#define PICO4_PANEL_ROW1_Y 190.0
#define PICO4_PANEL_ROW1_H 80.0
#define PICO4_PANEL_PRESET_X0 30.0
#define PICO4_PANEL_PRESET_W 170.0
#define PICO4_PANEL_PRESET_GAP 20.0
#define PICO4_PANEL_ROW2_Y 310.0
#define PICO4_PANEL_ROW2_H 110.0

static inline enum pico4_panel_widget
pico4_panel_hit(double x, double y)
{
	if (y >= PICO4_PANEL_ROW1_Y && y < PICO4_PANEL_ROW1_Y + PICO4_PANEL_ROW1_H) {
		if (x >= PICO4_PANEL_BUTTON_X0 && x < PICO4_PANEL_BUTTON_X0 + PICO4_PANEL_BUTTON_W) {
			return PICO4_PANEL_MINUS;
		}
		if (x >= PICO4_PANEL_W - PICO4_PANEL_BUTTON_X0 - PICO4_PANEL_BUTTON_W && x < PICO4_PANEL_W - PICO4_PANEL_BUTTON_X0) {
			return PICO4_PANEL_PLUS;
		}
		if (x >= PICO4_PANEL_SLIDER_X0 - 10 && x <= PICO4_PANEL_SLIDER_X1 + 10) {
			return PICO4_PANEL_SLIDER;
		}
	}
	if (y >= PICO4_PANEL_ROW2_Y && y < PICO4_PANEL_ROW2_Y + PICO4_PANEL_ROW2_H && x >= PICO4_PANEL_PRESET_X0) {
		int i = (int)((x - PICO4_PANEL_PRESET_X0) / (PICO4_PANEL_PRESET_W + PICO4_PANEL_PRESET_GAP));
		double inside = x - PICO4_PANEL_PRESET_X0 - i * (PICO4_PANEL_PRESET_W + PICO4_PANEL_PRESET_GAP);
		if (i < PICO4_PANEL_PRESETS && inside < PICO4_PANEL_PRESET_W) {
			return (enum pico4_panel_widget)(PICO4_PANEL_PRESET0 + i);
		}
	}
	return PICO4_PANEL_NONE;
}

static inline double
pico4_panel_clamp(double mm)
{
	return mm < PICO4_PANEL_MM_MIN ? PICO4_PANEL_MM_MIN : mm > PICO4_PANEL_MM_MAX ? PICO4_PANEL_MM_MAX : mm;
}

//! The slider's value at @p x, in steps of 0.1 mm.
static inline double
pico4_panel_slider_mm(double x)
{
	double t = (x - PICO4_PANEL_SLIDER_X0) / (PICO4_PANEL_SLIDER_X1 - PICO4_PANEL_SLIDER_X0);
	int tenths = (int)(10 * (PICO4_PANEL_MM_MIN + t * (PICO4_PANEL_MM_MAX - PICO4_PANEL_MM_MIN)) + 0.5);
	return pico4_panel_clamp(tenths / 10.0);
}

static inline double
pico4_panel_slider_x(double mm)
{
	double t = (pico4_panel_clamp(mm) - PICO4_PANEL_MM_MIN) / (PICO4_PANEL_MM_MAX - PICO4_PANEL_MM_MIN);
	return PICO4_PANEL_SLIDER_X0 + t * (PICO4_PANEL_SLIDER_X1 - PICO4_PANEL_SLIDER_X0);
}

//! One separation in mm per line; a line outside the range, or a missing one, is an empty preset (0).
static inline void
pico4_panel_presets_load(const char *path, double presets[PICO4_PANEL_PRESETS])
{
	FILE *f = fopen(path, "r");
	for (int i = 0; i < PICO4_PANEL_PRESETS; i++) {
		double mm = 0;
		presets[i] = f != NULL && fscanf(f, "%lf", &mm) == 1 && mm >= PICO4_PANEL_MM_MIN && mm <= PICO4_PANEL_MM_MAX ? mm : 0;
	}
	if (f != NULL) {
		fclose(f);
	}
}

static inline bool
pico4_panel_presets_save(const char *path, const double presets[PICO4_PANEL_PRESETS])
{
	FILE *f = fopen(path, "w");
	if (f == NULL) {
		return false;
	}
	for (int i = 0; i < PICO4_PANEL_PRESETS; i++) {
		fprintf(f, "%.2f\n", presets[i]);
	}
	return fclose(f) == 0;
}
