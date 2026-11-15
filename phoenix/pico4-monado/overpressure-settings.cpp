// Copyright 2026, overpressure contributors.
// SPDX-License-Identifier: BSL-1.0
/*
 * overpressure-settings: Overpressure Settings, a tab in SteamVR's dashboard (docs/session.md#settings)
 * for what the PICO 4 cannot do the Steam Frame's way, laid out as SteamVR's own settings: a menu on
 * the left, the chosen page on the right (pico4_settings.h).
 *
 * - Lens distance. The lenses have no wheel, and SteamVR shows the IPD when it changes but has no
 *   control that sets it: a slider, buttons for half a millimetre, and four presets for people
 *   taking turns (a press moves the lenses, holding one stores the distance in it). pico4-lens
 *   moves the lenses, and the driver hands the new distance to SteamVR.
 * - Controllers. SteamVR cannot pair the PICO 4's controllers: each one's state, and Pair for a
 *   missing one, through the driver's pairing socket (pico4_station.h), while the page says what
 *   to press. With no controller connected 20 s after the start, the laser cannot reach anything:
 *   the page comes up in front of the wearer and the station listens for one controller after the
 *   other. pico4-onboarding pairs them itself during the first-run setup.
 *
 * In Steam's language, with the texts of pico4-onboarding.strings. Nothing of Valve's is linked:
 * libopenvr_api.so is opened at run time.
 *
 *   overpressure-settings [--render DIR [LANGUAGE]]
 *
 * --render draws the pages into DIR as PNG files, without SteamVR, in LANGUAGE (Steam's name).
 */
#include "pico4_pairing.h"
#include "pico4_panels.hpp"
#include "pico4_settings.h"

#include <fcntl.h>
#include <sys/wait.h>

#include <cstdlib>

namespace {

using namespace pico4_overlay;
using namespace pico4_panels;

volatile sig_atomic_t g_stop;

void
stop_requested(int)
{
	g_stop = 1;
}

//! The panel in front of the wearer: the controllers' page under its title.
constexpr int kFrontW = PICO4_PANEL_W;
constexpr int kFrontH = (int)PICO4_SETTINGS_PAGE_Y + PICO4_PANEL_H;
//! Seconds both controllers stay shown on the panel in front before it goes.
constexpr double kBothShown = 3.0;

struct Lenses
{
	double mm = 0;      //!< What the driver reports; 0 before the first reading.
	double target = 0;  //!< Where the lenses are going, or a slider being dragged; 0: nowhere.
	double presets[PICO4_PANEL_PRESETS] = {};
	int pressed = PICO4_PANEL_NONE;
	bool stored = false; //!< The pressed preset was just stored.
	double pressed_at = 0;
	const char *message = nullptr; //!< a failure's text key
	pid_t mover = -1;
	double pending = 0; //!< A move asked for while another one runs.
};

struct Controllers
{
	bool known = false; //!< The driver answered a status.
	bool left = false, right = false;
	int pairing = 0; //!< The hand being paired: 1 left, 2 right.
	int last_hands_free = 0;
	bool hands_free = false; //!< Pairing started because no controller was there.
	const char *message = nullptr; //!< a failure's text key
	int pressed = PICO4_CONTROLLERS_NONE;
};

struct State
{
	enum pico4_settings_page page = PICO4_SETTINGS_LENSES;
	int pressed_entry = PICO4_SETTINGS_NONE;
	Lenses lens;
	Controllers ctl;
};

std::string
presets_path()
{
	const std::string config = home_path("/.config"), dir = config + "/pico4-lens";
	mkdir(config.c_str(), 0755);
	mkdir(dir.c_str(), 0755);
	return dir + "/presets";
}

void
start_move(Lenses &l, double mm)
{
	if (l.mover > 0) {
		l.pending = mm;
		return;
	}
	char value[16];
	snprintf(value, sizeof(value), "%.2f", mm);
	pid_t pid = fork();
	if (pid == 0) {
		int null = open("/dev/null", O_WRONLY);
		dup2(null, 1);
		execlp("pico4-lens", "pico4-lens", "set", value, (char *)nullptr);
		_exit(127);
	}
	if (pid < 0) {
		l.message = "lens_move_failed";
		return;
	}
	l.mover = pid;
	l.target = mm;
	l.message = nullptr;
}

//! True when the move ended in this call.
bool
reap_mover(Lenses &l)
{
	int status;
	if (l.mover <= 0 || waitpid(l.mover, &status, WNOHANG) != l.mover) {
		return false;
	}
	l.mover = -1;
	if (!(WIFEXITED(status) && WEXITSTATUS(status) == 0)) {
		l.message = "lens_not_reached";
	}
	if (l.pending > 0) {
		const double next = l.pending;
		l.pending = 0;
		start_move(l, next);
	}
	return true;
}

// -- Drawing --------------------------------------------------------------------------------------

const double kWhite[3] = {1, 1, 1}, kDim[3] = {0.65, 0.67, 0.70}, kText[3] = {0.85, 0.87, 0.90};

void
colour(cairo_t *c, const double rgb[3])
{
	cairo_set_source_rgb(c, rgb[0], rgb[1], rgb[2]);
}

void
box(cairo_t *c, double x, double y, double w, double h, bool pressed, bool selected, double r = 14)
{
	rounded(c, x, y, w, h, r);
	if (pressed) {
		cairo_set_source_rgb(c, 0.20, 0.44, 0.72);
	} else if (selected) {
		cairo_set_source_rgb(c, 0.16, 0.36, 0.60);
	} else {
		cairo_set_source_rgb(c, 0.22, 0.24, 0.27);
	}
	cairo_fill(c);
}

//! @p text centred in the box at (@p x, @p y, @p w, @p h).
void
centred(cairo_t *c, const std::string &text, double x, double y, double w, double h, double size, bool bold)
{
	const double tall = pico4_text::draw(c, text, x, 0, w, size, bold, pico4_text::kCenter, true);
	pico4_text::draw(c, text, x, y + (h - tall) / 2, w, size, bold, pico4_text::kCenter);
}

void
button(cairo_t *c, double x, double y, double w, double h, const std::string &label, bool pressed, bool primary)
{
	box(c, x, y, w, h, pressed, primary);
	colour(c, kWhite);
	centred(c, label, x + 8, y, w - 16, h, 26, true);
}

//! The lens distance page, in the page's pixels.
void
draw_lenses(cairo_t *c, const Lenses &l, const Texts &t)
{
	const double shown = l.target > 0 ? l.target : l.mm;
	char line[64];
	colour(c, kWhite);
	snprintf(line, sizeof(line), shown > 0 ? "%.1f mm" : "--", shown);
	centred(c, line, 0, 70, PICO4_PANEL_W, 100, 70, true);

	const double row = PICO4_PANEL_ROW1_Y, h = PICO4_PANEL_ROW1_H, w = PICO4_PANEL_BUTTON_W;
	const double plus_x = PICO4_PANEL_W - PICO4_PANEL_BUTTON_X0 - w;
	box(c, PICO4_PANEL_BUTTON_X0, row, w, h, l.pressed == PICO4_PANEL_MINUS, false);
	box(c, plus_x, row, w, h, l.pressed == PICO4_PANEL_PLUS, false);
	colour(c, kWhite);
	centred(c, "\xE2\x88\x92", PICO4_PANEL_BUTTON_X0, row, w, h, 52, true);
	centred(c, "+", plus_x, row, w, h, 52, true);

	// The slider: a track, its filled part and a knob at the distance.
	const double track = row + h / 2;
	rounded(c, PICO4_PANEL_SLIDER_X0, track - 5, PICO4_PANEL_SLIDER_X1 - PICO4_PANEL_SLIDER_X0, 10, 5);
	cairo_set_source_rgb(c, 0.30, 0.33, 0.37);
	cairo_fill(c);
	if (shown > 0) {
		const double knob = pico4_panel_slider_x(shown);
		rounded(c, PICO4_PANEL_SLIDER_X0, track - 5, knob - PICO4_PANEL_SLIDER_X0, 10, 5);
		cairo_set_source_rgb(c, 0.27, 0.56, 0.90);
		cairo_fill(c);
		cairo_arc(c, knob, track, l.pressed == PICO4_PANEL_SLIDER ? 22 : 18, 0, 2 * M_PI);
		cairo_set_source_rgb(c, 1, 1, 1);
		cairo_fill(c);
	}
	colour(c, kDim);
	centred(c, "62", PICO4_PANEL_SLIDER_X0 - 22, row + h - 4, 60, 24, 18, false);
	centred(c, "72", PICO4_PANEL_SLIDER_X1 - 38, row + h - 4, 60, 24, 18, false);

	for (int i = 0; i < PICO4_PANEL_PRESETS; i++) {
		const double x = PICO4_PANEL_PRESET_X0 + i * (PICO4_PANEL_PRESET_W + PICO4_PANEL_PRESET_GAP);
		const double y = PICO4_PANEL_ROW2_Y;
		const bool here = l.presets[i] > 0 && shown > 0 && fabs(l.presets[i] - shown) < 0.15;
		box(c, x, y, PICO4_PANEL_PRESET_W, PICO4_PANEL_ROW2_H, l.pressed == PICO4_PANEL_PRESET0 + i, here);
		char number[8];
		snprintf(number, sizeof(number), "%d", i + 1);
		cairo_set_source_rgb(c, 0.75, 0.78, 0.82);
		centred(c, fill(t.our("lens_preset"), number), x + 6, y + 12, PICO4_PANEL_PRESET_W - 12, 30, 22, false);
		colour(c, kWhite);
		if (l.pressed == PICO4_PANEL_PRESET0 + i && l.stored) {
			centred(c, t.our("lens_stored"), x + 6, y + 50, PICO4_PANEL_PRESET_W - 12, 46, 28, true);
		} else if (l.presets[i] > 0) {
			snprintf(number, sizeof(number), "%.1f", l.presets[i]);
			centred(c, number, x + 6, y + 50, PICO4_PANEL_PRESET_W - 12, 46, 36, true);
		} else {
			cairo_set_source_rgb(c, 0.55, 0.58, 0.62);
			centred(c, t.our("lens_empty"), x + 6, y + 50, PICO4_PANEL_PRESET_W - 12, 46, 24, false);
		}
	}

	if (l.message != nullptr) {
		cairo_set_source_rgb(c, 0.95, 0.55, 0.45);
	} else {
		colour(c, kDim);
	}
	pico4_text::draw(c, t.our(l.message != nullptr ? l.message : "lens_hint"), 30, 440, PICO4_PANEL_W - 60, 22, false,
	                 pico4_text::kCenter);
}

//! The controllers' page, in the page's pixels; @p close for the panel in front of the wearer.
void
draw_controllers(cairo_t *c, const Controllers &s, const Texts &t, bool close, int close_pressed)
{
	for (int hand = 1; hand <= 2; hand++) {
		const bool on = hand == 1 ? s.left : s.right;
		const double y = PICO4_CONTROLLERS_ROW_Y + (hand - 1) * PICO4_CONTROLLERS_ROW_STEP;
		rounded(c, 0, y, PICO4_CONTROLLERS_W, PICO4_CONTROLLERS_ROW_H, 12);
		cairo_set_source_rgb(c, 0.17, 0.18, 0.21);
		cairo_fill(c);
		const std::string state = !s.known ? "..." : on ? t.our("controller_connected")
		                                     : s.pairing == hand ? t.our("controller_waiting")
		                                                          : t.our("controller_missing");
		cairo_set_source_rgb(c, on ? 0.55 : 0.95, on ? 0.85 : 0.75, on ? 0.55 : 0.45);
		const std::string text = fill(t.our(hand == 1 ? "controller_left" : "controller_right"), state);
		const double tall = pico4_text::draw(c, text, 24, 0, PICO4_CONTROLLERS_PAIR_X - 40, 28, false, pico4_text::kStart, true);
		pico4_text::draw(c, text, 24, y + (PICO4_CONTROLLERS_ROW_H - tall) / 2, PICO4_CONTROLLERS_PAIR_X - 40, 28, false);
		if (s.known && !on && s.pairing == 0) {
			button(c, PICO4_CONTROLLERS_PAIR_X, y + PICO4_CONTROLLERS_PAIR_INSET, PICO4_CONTROLLERS_PAIR_W,
			       PICO4_CONTROLLERS_ROW_H - 2 * PICO4_CONTROLLERS_PAIR_INSET, t.our("pair"),
			       s.pressed == (hand == 1 ? PICO4_CONTROLLERS_PAIR_LEFT : PICO4_CONTROLLERS_PAIR_RIGHT), false);
		}
	}
	colour(c, kText);
	double y = PICO4_CONTROLLERS_TEXT_Y;
	const double w = PICO4_CONTROLLERS_W;
	const int missing = pico4_pairing_missing(s.left, s.right);
	if (s.pairing != 0) {
		y += pico4_text::draw(c, t.our(s.pairing == 1 ? "controller_pair_left" : "controller_pair_right"), 0, y, w, 26, false) + 12;
		pico4_text::draw(c, t.our("controller_pair_how"), 0, y, w, 26, false);
	} else if (s.left && s.right) {
		pico4_text::draw(c, t.our("controllers_both"), 0, y, w, 26, false);
	} else if (missing != 0) {
		pico4_text::draw(c, t.our(missing == 1 ? "controller_point_left" : "controller_point_right"), 0, y, w, 26, false);
	}
	if (s.message != nullptr) {
		cairo_set_source_rgb(c, 0.95, 0.55, 0.45);
		pico4_text::draw(c, t.our(s.message), 0, close ? PICO4_CONTROLLERS_CLOSE_Y + 20 : PICO4_CONTROLLERS_CLOSE_Y, close ? PICO4_CONTROLLERS_PAIR_X - 20 : w,
		                 22, false);
	}
	if (close) {
		button(c, PICO4_CONTROLLERS_PAIR_X, PICO4_CONTROLLERS_CLOSE_Y, PICO4_CONTROLLERS_PAIR_W, PICO4_CONTROLLERS_CLOSE_H,
		       t.our("close"), close_pressed == PICO4_CONTROLLERS_CLOSE, true);
	}
}

const char *const kPageTitles[PICO4_SETTINGS_PAGES] = {"setup_ipd_title", "controllers_title"};

void
title(cairo_t *c, const std::string &text, double x, double w)
{
	colour(c, kWhite);
	pico4_text::line(c, text, x, PICO4_SETTINGS_TITLE_Y, w, 36, true);
}

void
draw_tab(cairo_surface_t *surface, const State &s, const Texts &t)
{
	cairo_t *c = cairo_create(surface);
	cairo_set_source_rgb(c, 0.11, 0.12, 0.14);
	cairo_paint(c);
	// The menu.
	cairo_rectangle(c, 0, 0, PICO4_SETTINGS_MENU_W, PICO4_SETTINGS_H);
	cairo_set_source_rgb(c, 0.08, 0.09, 0.11);
	cairo_fill(c);
	colour(c, kWhite);
	pico4_text::draw(c, t.our("settings_title"), 24, 30, PICO4_SETTINGS_MENU_W - 48, 26, true);
	for (int i = 0; i < PICO4_SETTINGS_PAGES; i++) {
		const double y = PICO4_SETTINGS_ENTRY_Y + i * PICO4_SETTINGS_ENTRY_STEP;
		if (s.pressed_entry == i || s.page == i) {
			box(c, PICO4_SETTINGS_ENTRY_X, y, PICO4_SETTINGS_ENTRY_W, PICO4_SETTINGS_ENTRY_H, s.pressed_entry == i, true, 10);
		}
		colour(c, s.page == i ? kWhite : kText);
		const std::string label = t.our(kPageTitles[i]);
		const double tall = pico4_text::draw(c, label, 0, 0, PICO4_SETTINGS_ENTRY_W - 40, 24, false, pico4_text::kStart, true);
		pico4_text::draw(c, label, PICO4_SETTINGS_ENTRY_X + 20, y + (PICO4_SETTINGS_ENTRY_H - tall) / 2,
		                 PICO4_SETTINGS_ENTRY_W - 40, 24, s.page == i);
	}
	// The page.
	title(c, t.our(kPageTitles[s.page]), PICO4_SETTINGS_PAGE_X + 30, PICO4_PANEL_W - 60);
	cairo_save(c);
	cairo_translate(c, PICO4_SETTINGS_PAGE_X, PICO4_SETTINGS_PAGE_Y);
	if (s.page == PICO4_SETTINGS_LENSES) {
		draw_lenses(c, s.lens, t);
	} else {
		cairo_translate(c, PICO4_CONTROLLERS_MARGIN, 0);
		draw_controllers(c, s.ctl, t, false, PICO4_CONTROLLERS_NONE);
	}
	cairo_restore(c);
	cairo_destroy(c);
	cairo_surface_flush(surface);
}

void
draw_front(cairo_surface_t *surface, const Controllers &s, const Texts &t)
{
	cairo_t *c = cairo_create(surface);
	cairo_set_source_rgb(c, 0.11, 0.12, 0.14);
	cairo_paint(c);
	title(c, t.our("controllers_title"), 30, kFrontW - 60);
	cairo_translate(c, PICO4_CONTROLLERS_MARGIN, PICO4_SETTINGS_PAGE_Y);
	draw_controllers(c, s, t, true, s.pressed);
	cairo_destroy(c);
	cairo_surface_flush(surface);
}

//! The tab's icon: three sliders.
void
draw_thumbnail(cairo_surface_t *surface)
{
	cairo_t *c = cairo_create(surface);
	cairo_set_source_rgb(c, 0.11, 0.12, 0.14);
	cairo_paint(c);
	const double knobs[3] = {90, 170, 120};
	for (int i = 0; i < 3; i++) {
		const double y = 68 + i * 60;
		rounded(c, 40, y - 6, 176, 12, 6);
		cairo_set_source_rgb(c, 0.55, 0.58, 0.62);
		cairo_fill(c);
		cairo_arc(c, knobs[i], y, 20, 0, 2 * M_PI);
		cairo_set_source_rgb(c, 1, 1, 1);
		cairo_fill(c);
	}
	cairo_destroy(c);
	cairo_surface_flush(surface);
}

// -- Input ----------------------------------------------------------------------------------------

//! A mouse event on the lens distance page at (@p x, @p y) of the page; true when it changed it.
bool
lenses_event(Lenses &l, const vr::VREvent_t &e, double x, double y)
{
	switch (e.eventType) {
	case vr::VREvent_MouseButtonDown:
		l.pressed = pico4_panel_hit(x, y);
		l.pressed_at = now_s();
		l.stored = false;
		if (l.pressed == PICO4_PANEL_SLIDER) {
			l.target = pico4_panel_slider_mm(x);
		}
		return true;
	case vr::VREvent_MouseMove:
		if (l.pressed == PICO4_PANEL_SLIDER) {
			l.target = pico4_panel_slider_mm(x);
			return true;
		}
		return false;
	case vr::VREvent_MouseButtonUp: {
		const double from = l.target > 0 ? l.target : l.mm;
		const int was = l.pressed;
		const bool stored = l.stored;
		l.pressed = PICO4_PANEL_NONE;
		l.stored = false;
		if (from <= 0) {
			return true;
		}
		if (was == PICO4_PANEL_MINUS) {
			start_move(l, pico4_panel_clamp(from - PICO4_PANEL_STEP_MM));
		} else if (was == PICO4_PANEL_PLUS) {
			start_move(l, pico4_panel_clamp(from + PICO4_PANEL_STEP_MM));
		} else if (was == PICO4_PANEL_SLIDER) {
			start_move(l, l.target);
		} else if (was >= PICO4_PANEL_PRESET0 && was < PICO4_PANEL_PRESET0 + PICO4_PANEL_PRESETS && !stored &&
		           l.presets[was] > 0) {
			start_move(l, l.presets[was]);
		}
		return true;
	}
	default: return false;
	}
}

/*!
 * A mouse event on the controllers' page at (@p x, @p y) of the page; returns the widget let go
 * on (the one pressed), PICO4_CONTROLLERS_NONE otherwise.
 */
int
controllers_event(Controllers &s, const vr::VREvent_t &e, double x, double y, bool close)
{
	const bool pair = s.known && s.pairing == 0;
	const int at = pico4_controllers_hit(x, y, pair && !s.left, pair && !s.right, close);
	if (e.eventType == vr::VREvent_MouseButtonDown) {
		s.pressed = at;
	} else if (e.eventType == vr::VREvent_MouseButtonUp) {
		const int was = s.pressed;
		s.pressed = PICO4_CONTROLLERS_NONE;
		return was == at ? was : PICO4_CONTROLLERS_NONE;
	}
	return PICO4_CONTROLLERS_NONE;
}

int
render(const char *dir, const char *language)
{
	Texts t;
	t.load(language != nullptr ? language : "english");
	int r = 0;
	auto save = [&](cairo_surface_t *surface, const char *name) {
		const std::string file = std::string(dir) + "/" + name + ".png";
		r |= cairo_surface_write_to_png(surface, file.c_str()) != CAIRO_STATUS_SUCCESS;
	};
	cairo_surface_t *tab = cairo_image_surface_create(CAIRO_FORMAT_ARGB32, PICO4_SETTINGS_W, PICO4_SETTINGS_H);
	cairo_surface_t *front = cairo_image_surface_create(CAIRO_FORMAT_ARGB32, kFrontW, kFrontH);
	State s;
	s.lens.mm = 64.0;
	s.lens.presets[0] = 63.5;
	s.lens.presets[1] = 64.0;
	draw_tab(tab, s, t);
	save(tab, "settings-lenses");
	s.page = PICO4_SETTINGS_CONTROLLERS;
	s.ctl.known = true;
	s.ctl.left = true;
	draw_tab(tab, s, t);
	save(tab, "settings-controllers");
	s.ctl.left = false;
	s.ctl.pairing = 1;
	s.ctl.hands_free = true;
	draw_front(front, s.ctl, t);
	save(front, "settings-front-pairing");
	cairo_surface_destroy(tab);
	cairo_surface_destroy(front);
	return r;
}

} // namespace

int
main(int argc, char **argv)
{
	if ((argc == 3 || argc == 4) && strcmp(argv[1], "--render") == 0) {
		return render(argv[2], argc == 4 ? argv[3] : nullptr);
	}
	signal(SIGPIPE, SIG_IGN);
	signal(SIGTERM, stop_requested);
	signal(SIGINT, stop_requested);

	Vr vrs;
	const int opened = vr_open("overpressure-settings", vrs, &g_stop);
	if (opened != 0) {
		return opened == 2 ? 0 : 1;
	}
	vr::IVROverlay *overlay = vrs.overlay;
	vr::IVRSystem *system = vrs.system;

	Texts texts;
	std::string language = "english";
	{
		char lang[32];
		const std::string vdf = registry();
		if (pico4_setup_registry_language(vdf.c_str(), lang, sizeof(lang))) {
			language = lang;
		}
		texts.load(language);
	}
	vr::VROverlayHandle_t tab, thumbnail, front;
	if (overlay->CreateDashboardOverlay("overpressure.settings", texts.our("settings_title").c_str(), &tab, &thumbnail) !=
	        vr::VROverlayError_None ||
	    overlay->CreateOverlay("overpressure.settings.controllers", "Overpressure controllers", &front) != vr::VROverlayError_None) {
		fprintf(stderr, "overpressure-settings: no overlays\n");
		return 1;
	}
	const vr::HmdVector2_t tab_scale = {{(float)PICO4_SETTINGS_W, (float)PICO4_SETTINGS_H}};
	const vr::HmdVector2_t front_scale = {{(float)kFrontW, (float)kFrontH}};
	overlay->SetOverlayWidthInMeters(tab, 2.2f);
	overlay->SetOverlayInputMethod(tab, vr::VROverlayInputMethod_Mouse);
	overlay->SetOverlayMouseScale(tab, &tab_scale);
	overlay->SetOverlayWidthInMeters(front, 0.96f);
	overlay->SetOverlayInputMethod(front, vr::VROverlayInputMethod_Mouse);
	overlay->SetOverlayMouseScale(front, &front_scale);
	// The laser reaches the panel outside the dashboard too, and it is drawn over Steam's window.
	overlay->SetOverlayFlag(front, vr::VROverlayFlags_MakeOverlaysInteractiveIfVisible, true);
	overlay->SetOverlaySortOrder(front, 100);

	cairo_surface_t *tab_surface = cairo_image_surface_create(CAIRO_FORMAT_ARGB32, PICO4_SETTINGS_W, PICO4_SETTINGS_H);
	cairo_surface_t *front_surface = cairo_image_surface_create(CAIRO_FORMAT_ARGB32, kFrontW, kFrontH);
	Gpu gpu;
	Texture tab_texture, front_texture, thumbnail_texture;
	if (!gpu_create(gpu, vrs.compositor, "overpressure-settings") || !texture_create(gpu, thumbnail_texture, 256, 256) ||
	    !texture_create(gpu, tab_texture, PICO4_SETTINGS_W, PICO4_SETTINGS_H) || !texture_create(gpu, front_texture, kFrontW, kFrontH)) {
		fprintf(stderr, "overpressure-settings: no Vulkan textures\n");
		return 1;
	}
	{
		cairo_surface_t *small = cairo_image_surface_create(CAIRO_FORMAT_ARGB32, 256, 256);
		draw_thumbnail(small);
		if (!texture_show(gpu, thumbnail_texture, small, overlay, thumbnail)) {
			fprintf(stderr, "overpressure-settings: thumbnail not accepted\n");
		}
		cairo_surface_destroy(small);
	}
	printf("overpressure-settings: in the dashboard\n");
	fflush(stdout);

	State s;
	Lenses &l = s.lens;
	Controllers &k = s.ctl;
	const std::string path = presets_path();
	pico4_panel_presets_load(path.c_str(), l.presets);
	Front placement;
	Request status, pair;
	bool front_shown = false;
	bool setup_pending = true; //!< the first-run setup still to come or running: it pairs itself
	const double started = now_s();
	double next_read = 0, next_status = 0, next_registry = 0, both_since = 0;
	bool tab_dirty = true, front_dirty = false, drawn = false, running = true;

	auto start_pairing = [&](int hand, bool hands_free) {
		char text[32];
		snprintf(text, sizeof(text), "pair %s %d", hand == 1 ? "left" : "right", PICO4_PAIRING_LISTEN_S);
		k.hands_free = hands_free;
		if (pair.start(text)) {
			k.pairing = hand;
			k.message = nullptr;
		} else {
			k.message = "controller_radio";
		}
		tab_dirty = front_dirty = true;
	};
	auto show_front = [&](bool on) {
		if (on && !front_shown) {
			placement.place(overlay, system, front);
			overlay->ShowOverlay(front);
			printf("overpressure-settings: the controllers in front of the wearer\n");
			fflush(stdout);
		} else if (!on && front_shown) {
			overlay->HideOverlay(front);
		}
		front_shown = on;
		both_since = 0;
		front_dirty = on;
	};

	while (running && !g_stop) {
		const double now = now_s();
		vr::VREvent_t e;
		while (overlay->PollNextOverlayEvent(tab, &e, sizeof(e))) {
			const double x = e.data.mouse.x, y = PICO4_SETTINGS_H - e.data.mouse.y; // SteamVR's y points up
			const double px = x - PICO4_SETTINGS_PAGE_X, py = y - PICO4_SETTINGS_PAGE_Y;
			if (e.eventType == vr::VREvent_MouseButtonDown && x < PICO4_SETTINGS_MENU_W) {
				s.pressed_entry = pico4_settings_menu_hit(x, y);
				tab_dirty = true;
				continue;
			}
			if (e.eventType == vr::VREvent_MouseButtonUp && s.pressed_entry != PICO4_SETTINGS_NONE) {
				if (s.pressed_entry == pico4_settings_menu_hit(x, y)) {
					s.page = (enum pico4_settings_page)s.pressed_entry;
				}
				s.pressed_entry = PICO4_SETTINGS_NONE;
				tab_dirty = true;
				continue;
			}
			if (s.page == PICO4_SETTINGS_LENSES) {
				tab_dirty = lenses_event(l, e, px, py) || tab_dirty;
			} else if (e.eventType == vr::VREvent_MouseButtonDown || e.eventType == vr::VREvent_MouseButtonUp) {
				const int done = controllers_event(k, e, px - PICO4_CONTROLLERS_MARGIN, py, false);
				if (done == PICO4_CONTROLLERS_PAIR_LEFT || done == PICO4_CONTROLLERS_PAIR_RIGHT) {
					start_pairing(done == PICO4_CONTROLLERS_PAIR_LEFT ? 1 : 2, false);
				}
				tab_dirty = true;
			}
		}
		while (overlay->PollNextOverlayEvent(front, &e, sizeof(e))) {
			if (e.eventType != vr::VREvent_MouseButtonDown && e.eventType != vr::VREvent_MouseButtonUp) {
				continue;
			}
			const double x = e.data.mouse.x - PICO4_CONTROLLERS_MARGIN, y = kFrontH - e.data.mouse.y - PICO4_SETTINGS_PAGE_Y;
			const int done = controllers_event(k, e, x, y, true);
			if (done == PICO4_CONTROLLERS_PAIR_LEFT || done == PICO4_CONTROLLERS_PAIR_RIGHT) {
				start_pairing(done == PICO4_CONTROLLERS_PAIR_LEFT ? 1 : 2, false);
			} else if (done == PICO4_CONTROLLERS_CLOSE) {
				show_front(false);
			}
			front_dirty = true;
		}
		while (system->PollNextEvent(&e, sizeof(e))) {
			if (e.eventType == vr::VREvent_Quit) {
				system->AcknowledgeQuit_Exiting();
				running = false;
			} else if (e.eventType == vr::VREvent_TrackedDeviceUserInteractionStarted &&
			           e.trackedDeviceIndex == vr::k_unTrackedDeviceIndex_Hmd && front_shown) {
				// Put on (the wear sensor): the panel came up wherever the headset lay; in front again.
				placement.place(overlay, system, front);
			}
		}
		const bool visible = overlay->IsOverlayVisible(tab);

		// The lenses: a held preset stores the distance, a finished move, the driver's distance.
		if (l.pressed >= PICO4_PANEL_PRESET0 && l.pressed < PICO4_PANEL_PRESET0 + PICO4_PANEL_PRESETS && !l.stored &&
		    now - l.pressed_at >= PICO4_PANEL_HOLD_S && l.mm > 0) {
			l.presets[l.pressed] = round(l.mm * 10) / 10;
			l.stored = true;
			if (!pico4_panel_presets_save(path.c_str(), l.presets)) {
				l.message = "lens_store_failed";
			}
			tab_dirty = true;
		}
		tab_dirty = reap_mover(l) || tab_dirty;
		if (now >= next_read) {
			next_read = now + (visible ? 0.25 : 2.0);
			vr::ETrackedPropertyError pe;
			const float m = system->GetFloatTrackedDeviceProperty(vr::k_unTrackedDeviceIndex_Hmd, vr::Prop_UserIpdMeters_Float, &pe);
			if (pe == vr::TrackedProp_Success && m > 0.05f && fabs(m * 1000 - l.mm) >= 0.05) {
				l.mm = m * 1000;
				tab_dirty = true;
			}
			// The driver has caught up with the finished move.
			if (l.mover <= 0 && l.pressed != PICO4_PANEL_SLIDER && l.target > 0 && now - l.pressed_at > 2.5) {
				l.target = 0;
				tab_dirty = true;
			}
		}

		// Steam's language and the setup's state, from the client's registry and the setup's mark.
		if (now >= next_registry) {
			next_registry = now + 5;
			const std::string vdf = registry();
			setup_pending = pico4_setup_starts(pico4_pairing_oobe_done(vdf.c_str()), mark_exists("setup-done"));
			char lang[32];
			const std::string now_language = pico4_setup_registry_language(vdf.c_str(), lang, sizeof(lang)) ? lang : language;
			if (now_language != language || !texts.steam_found) {
				language = now_language;
				texts.load(language);
				overlay->SetOverlayName(tab, texts.our("settings_title").c_str());
				tab_dirty = front_dirty = true;
			}
		}

		// The controllers' state from the driver, while no pairing holds its socket; during the setup
		// only while the tab shows (the setup asks it itself).
		if (!pair.active() && !status.active() && now >= next_status && (!setup_pending || visible)) {
			next_status = now + 2;
			status.start("status");
		}
		if (status.poll() && status.done) {
			bool left, right;
			if (!status.answers.empty() && pico4_pairing_parse_status(status.answers[0].c_str(), &left, &right)) {
				if (!k.known || left != k.left || right != k.right) {
					tab_dirty = front_dirty = true;
				}
				k.known = true;
				k.left = left;
				k.right = right;
			}
			status.answers.clear();
		}
		if (pair.poll() && pair.done) {
			const int result = pair.answers.empty() ? -1 : pico4_pairing_answer(pair.answers.back().c_str());
			if (result == 1) {
				// Connected now, before the next status: hands-free pairing must not take the other hand.
				(k.pairing == 1 ? k.left : k.right) = true;
			} else if (!k.hands_free) {
				k.message = "controller_pair_failed";
			}
			k.pairing = 0;
			next_status = 0; // the new state, now
			tab_dirty = front_dirty = true;
		}
		// Hands-free pairing: nothing can point at a panel while no controller is there.
		if (pico4_settings_rescue(setup_pending, k.known, k.left, k.right, k.pairing, now - started, PICO4_PAIRING_WAIT_S)) {
			show_front(true);
			k.last_hands_free = pico4_pairing_hands_free(false, false, k.last_hands_free);
			start_pairing(k.last_hands_free, true);
		}
		// Both there: the panel in front goes after a moment.
		if (front_shown && k.left && k.right) {
			if (both_since == 0) {
				both_since = now;
			} else if (now - both_since >= kBothShown) {
				show_front(false);
			}
		}
		if (front_shown) {
			placement.follow(overlay, system, front, now);
		}

		// Once before anyone looks, so that the tab never opens blank.
		if (tab_dirty && (visible || !drawn)) {
			drawn = true;
			draw_tab(tab_surface, s, texts);
			if (!texture_show(gpu, tab_texture, tab_surface, overlay, tab)) {
				fprintf(stderr, "overpressure-settings: image not accepted\n");
			}
			tab_dirty = false;
		}
		if (front_dirty && front_shown) {
			draw_front(front_surface, k, texts);
			if (!texture_show(gpu, front_texture, front_surface, overlay, front)) {
				fprintf(stderr, "overpressure-settings: image not accepted\n");
			}
			front_dirty = false;
		}
		usleep(visible || front_shown ? 16000 : 200000);
	}
	pair.finish();
	status.finish();
	cairo_surface_destroy(tab_surface);
	cairo_surface_destroy(front_surface);
	overlay->DestroyOverlay(front);
	overlay->DestroyOverlay(tab);
	vrs.shutdown();
	return 0;
}
