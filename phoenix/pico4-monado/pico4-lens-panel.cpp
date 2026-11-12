// Copyright 2026, overpressure contributors.
// SPDX-License-Identifier: BSL-1.0
/*
 * pico4-lens-panel: the lens distance in SteamVR's dashboard (docs/xr.md#lens-distortion-and-ipd).
 *
 * SteamVR shows the IPD when it changes but has no control that sets it. This panel has one, for
 * people taking turns with the headset: a slider, buttons for half a millimetre, and four presets
 * (a press moves the lenses, holding one stores the distance in it). pico4-lens moves the lenses,
 * and the driver hands the new distance to SteamVR, whose own display shows it. Nothing of Valve's
 * is linked: libopenvr_api.so is opened at run time.
 *
 * The images go to SteamVR as Vulkan textures: with one through SetOverlayRaw or SetOverlayFromFile
 * alive, the compositor crashes in its cleanup at every stop (docs/session.md).
 *
 *   pico4-lens-panel [--open] [--render FILE.png MM]
 *
 * --open opens the dashboard on the panel. --render draws it into a file, without SteamVR.
 */
#include "pico4_lens_panel.h"
#include "pico4_overlay.hpp"

#include <fcntl.h>
#include <sys/stat.h>
#include <sys/wait.h>

#include <cstdlib>

namespace {

using namespace pico4_overlay;

volatile sig_atomic_t g_stop;

void
stop_requested(int)
{
	g_stop = 1;
}

struct State
{
	double mm = 0;             //!< What the driver reports; 0 before the first reading.
	double target = 0;         //!< Where the lenses are going, or a slider being dragged; 0: nowhere.
	double presets[PICO4_PANEL_PRESETS] = {};
	int pressed = PICO4_PANEL_NONE;
	bool stored = false;       //!< The pressed preset was just stored.
	double pressed_at = 0;
	std::string message;
	pid_t mover = -1;
	double pending = 0;        //!< A move asked for while another one runs.
};

std::string
presets_path()
{
	const char *home = getenv("HOME");
	std::string dir = std::string(home != nullptr ? home : "/tmp") + "/.config/pico4-lens";
	mkdir((std::string(home != nullptr ? home : "/tmp") + "/.config").c_str(), 0755);
	mkdir(dir.c_str(), 0755);
	return dir + "/presets";
}

void
start_move(State &s, double mm)
{
	if (s.mover > 0) {
		s.pending = mm;
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
		s.message = "Could not move the lenses";
		return;
	}
	s.mover = pid;
	s.target = mm;
	s.message.clear();
}

void
reap_mover(State &s)
{
	int status;
	if (s.mover <= 0 || waitpid(s.mover, &status, WNOHANG) != s.mover) {
		return;
	}
	s.mover = -1;
	if (!(WIFEXITED(status) && WEXITSTATUS(status) == 0)) {
		s.message = "The lenses did not reach it";
	}
	if (s.pending > 0) {
		double next = s.pending;
		s.pending = 0;
		start_move(s, next);
	}
}

void
button(cairo_t *c, double x, double y, double w, double h, bool pressed, bool selected)
{
	rounded(c, x, y, w, h, 14);
	if (pressed) {
		cairo_set_source_rgb(c, 0.20, 0.44, 0.72);
	} else if (selected) {
		cairo_set_source_rgb(c, 0.16, 0.32, 0.50);
	} else {
		cairo_set_source_rgb(c, 0.22, 0.24, 0.27);
	}
	cairo_fill(c);
}

void
draw(cairo_surface_t *surface, const State &s)
{
	cairo_t *c = cairo_create(surface);
	cairo_set_source_rgb(c, 0.11, 0.12, 0.14);
	cairo_paint(c);

	double shown = s.target > 0 ? s.target : s.mm;
	char line[64];
	cairo_set_source_rgb(c, 0.70, 0.73, 0.77);
	text_centered(c, "Lens distance", PICO4_PANEL_W / 2.0, 52, 26, CAIRO_FONT_WEIGHT_NORMAL);
	cairo_set_source_rgb(c, 1, 1, 1);
	snprintf(line, sizeof(line), shown > 0 ? "%.1f mm" : "--", shown);
	text_centered(c, line, PICO4_PANEL_W / 2.0, 140, 76, CAIRO_FONT_WEIGHT_BOLD);

	const double row = PICO4_PANEL_ROW1_Y, h = PICO4_PANEL_ROW1_H, w = PICO4_PANEL_BUTTON_W;
	const double plus_x = PICO4_PANEL_W - PICO4_PANEL_BUTTON_X0 - w;
	button(c, PICO4_PANEL_BUTTON_X0, row, w, h, s.pressed == PICO4_PANEL_MINUS, false);
	button(c, plus_x, row, w, h, s.pressed == PICO4_PANEL_PLUS, false);
	cairo_set_source_rgb(c, 1, 1, 1);
	text_centered(c, "-", PICO4_PANEL_BUTTON_X0 + w / 2, row + 57, 52, CAIRO_FONT_WEIGHT_BOLD);
	text_centered(c, "+", plus_x + w / 2, row + 57, 52, CAIRO_FONT_WEIGHT_BOLD);

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
		cairo_arc(c, knob, track, s.pressed == PICO4_PANEL_SLIDER ? 22 : 18, 0, 2 * M_PI);
		cairo_set_source_rgb(c, 1, 1, 1);
		cairo_fill(c);
	}
	cairo_set_source_rgb(c, 0.60, 0.63, 0.67);
	text_centered(c, "62", PICO4_PANEL_SLIDER_X0 + 8, row + h + 12, 18, CAIRO_FONT_WEIGHT_NORMAL);
	text_centered(c, "72", PICO4_PANEL_SLIDER_X1 - 8, row + h + 12, 18, CAIRO_FONT_WEIGHT_NORMAL);

	for (int i = 0; i < PICO4_PANEL_PRESETS; i++) {
		const double x = PICO4_PANEL_PRESET_X0 + i * (PICO4_PANEL_PRESET_W + PICO4_PANEL_PRESET_GAP);
		const double y = PICO4_PANEL_ROW2_Y;
		const bool here = s.presets[i] > 0 && shown > 0 && fabs(s.presets[i] - shown) < 0.15;
		button(c, x, y, PICO4_PANEL_PRESET_W, PICO4_PANEL_ROW2_H, s.pressed == PICO4_PANEL_PRESET0 + i, here);
		cairo_set_source_rgb(c, 0.75, 0.78, 0.82);
		snprintf(line, sizeof(line), "User %d", i + 1);
		text_centered(c, line, x + PICO4_PANEL_PRESET_W / 2, y + 40, 22, CAIRO_FONT_WEIGHT_NORMAL);
		cairo_set_source_rgb(c, 1, 1, 1);
		if (s.pressed == PICO4_PANEL_PRESET0 + i && s.stored) {
			text_centered(c, "Stored", x + PICO4_PANEL_PRESET_W / 2, y + 82, 30, CAIRO_FONT_WEIGHT_BOLD);
		} else if (s.presets[i] > 0) {
			snprintf(line, sizeof(line), "%.1f", s.presets[i]);
			text_centered(c, line, x + PICO4_PANEL_PRESET_W / 2, y + 82, 36, CAIRO_FONT_WEIGHT_BOLD);
		} else {
			cairo_set_source_rgb(c, 0.55, 0.58, 0.62);
			text_centered(c, "empty", x + PICO4_PANEL_PRESET_W / 2, y + 80, 24, CAIRO_FONT_WEIGHT_NORMAL);
		}
	}

	cairo_set_source_rgb(c, s.message.empty() ? 0.60 : 0.95, s.message.empty() ? 0.63 : 0.55, s.message.empty() ? 0.67 : 0.45);
	text_centered(c, s.message.empty() ? "Press a user to move the lenses. Hold it to store the distance." : s.message.c_str(),
	              PICO4_PANEL_W / 2.0, 466, 22, CAIRO_FONT_WEIGHT_NORMAL);
	cairo_destroy(c);
	cairo_surface_flush(surface);
}

void
draw_thumbnail(cairo_surface_t *surface)
{
	cairo_t *c = cairo_create(surface);
	cairo_set_source_rgb(c, 0.11, 0.12, 0.14);
	cairo_paint(c);
	cairo_set_line_width(c, 12);
	cairo_set_source_rgb(c, 1, 1, 1);
	cairo_arc(c, 76, 128, 44, 0, 2 * M_PI);
	cairo_stroke(c);
	cairo_arc(c, 180, 128, 44, 0, 2 * M_PI);
	cairo_stroke(c);
	cairo_destroy(c);
	cairo_surface_flush(surface);
}

bool
handle_event(State &s, const vr::VREvent_t &e)
{
	const double x = e.data.mouse.x, y = PICO4_PANEL_H - e.data.mouse.y; // SteamVR's y points up.
	switch (e.eventType) {
	case vr::VREvent_MouseButtonDown: {
		s.pressed = pico4_panel_hit(x, y);
		s.pressed_at = now_s();
		s.stored = false;
		if (s.pressed == PICO4_PANEL_SLIDER) {
			s.target = pico4_panel_slider_mm(x);
		}
		return true;
	}
	case vr::VREvent_MouseMove:
		if (s.pressed == PICO4_PANEL_SLIDER) {
			s.target = pico4_panel_slider_mm(x);
			return true;
		}
		return false;
	case vr::VREvent_MouseButtonUp: {
		const double from = s.target > 0 ? s.target : s.mm;
		const int was = s.pressed;
		const bool stored = s.stored;
		s.pressed = PICO4_PANEL_NONE;
		s.stored = false;
		if (from <= 0) {
			return true;
		}
		if (was == PICO4_PANEL_MINUS) {
			start_move(s, pico4_panel_clamp(from - PICO4_PANEL_STEP_MM));
		} else if (was == PICO4_PANEL_PLUS) {
			start_move(s, pico4_panel_clamp(from + PICO4_PANEL_STEP_MM));
		} else if (was == PICO4_PANEL_SLIDER) {
			start_move(s, s.target);
		} else if (was >= PICO4_PANEL_PRESET0 && was < PICO4_PANEL_PRESET0 + PICO4_PANEL_PRESETS && !stored &&
		           s.presets[was] > 0) {
			start_move(s, s.presets[was]);
		}
		return true;
	}
	default: return false;
	}
}

int
render(const char *file, double mm)
{
	State s;
	s.mm = mm;
	s.presets[0] = 63.5;
	s.presets[1] = mm;
	cairo_surface_t *surface = cairo_image_surface_create(CAIRO_FORMAT_ARGB32, PICO4_PANEL_W, PICO4_PANEL_H);
	draw(surface, s);
	int r = cairo_surface_write_to_png(surface, file) == CAIRO_STATUS_SUCCESS ? 0 : 1;
	cairo_surface_destroy(surface);
	return r;
}

} // namespace

int
main(int argc, char **argv)
{
	if (argc == 4 && strcmp(argv[1], "--render") == 0) {
		return render(argv[2], atof(argv[3]));
	}
	signal(SIGPIPE, SIG_IGN);
	signal(SIGTERM, stop_requested);
	signal(SIGINT, stop_requested);

	Vr vrs;
	const int opened = vr_open("pico4-lens-panel", vrs, &g_stop);
	if (opened != 0) {
		return opened == 2 ? 0 : 1;
	}
	vr::IVROverlay *overlay = vrs.overlay;
	vr::IVRSystem *system = vrs.system;
	vr::VROverlayHandle_t main_overlay, thumbnail;
	vr::EVROverlayError oe = overlay->CreateDashboardOverlay("pico4.lens.panel", "Lens distance", &main_overlay, &thumbnail);
	if (oe != vr::VROverlayError_None) {
		fprintf(stderr, "pico4-lens-panel: dashboard overlay: %d\n", oe);
		return 1;
	}
	overlay->SetOverlayWidthInMeters(main_overlay, 2.0f);
	overlay->SetOverlayInputMethod(main_overlay, vr::VROverlayInputMethod_Mouse);
	vr::HmdVector2_t scale = {{(float)PICO4_PANEL_W, (float)PICO4_PANEL_H}};
	overlay->SetOverlayMouseScale(main_overlay, &scale);

	State s;
	const std::string path = presets_path();
	pico4_panel_presets_load(path.c_str(), s.presets);
	cairo_surface_t *surface = cairo_image_surface_create(CAIRO_FORMAT_ARGB32, PICO4_PANEL_W, PICO4_PANEL_H);
	Gpu gpu;
	Texture main_texture, thumbnail_texture;
	if (!gpu_create(gpu, vrs.compositor, "pico4-lens-panel") || !texture_create(gpu, thumbnail_texture, 256, 256) ||
	    !texture_create(gpu, main_texture, PICO4_PANEL_W, PICO4_PANEL_H)) {
		fprintf(stderr, "pico4-lens-panel: no Vulkan textures\n");
		return 1;
	}
	{
		cairo_surface_t *small = cairo_image_surface_create(CAIRO_FORMAT_ARGB32, 256, 256);
		draw_thumbnail(small);
		if (!texture_show(gpu, thumbnail_texture, small, overlay, thumbnail)) {
			fprintf(stderr, "pico4-lens-panel: thumbnail not accepted\n");
		}
		cairo_surface_destroy(small);
	}
	if (argc == 2 && strcmp(argv[1], "--open") == 0) {
		overlay->ShowDashboard("pico4.lens.panel");
	}
	printf("pico4-lens-panel: in the dashboard\n");
	fflush(stdout);

	bool dirty = true, running = true, drawn = false;
	double next_read = 0;
	while (running && !g_stop) {
		vr::VREvent_t e;
		while (overlay->PollNextOverlayEvent(main_overlay, &e, sizeof(e))) {
			dirty = handle_event(s, e) || dirty;
		}
		while (system->PollNextEvent(&e, sizeof(e))) {
			if (e.eventType == vr::VREvent_Quit) {
				system->AcknowledgeQuit_Exiting();
				running = false;
			}
		}
		const double now = now_s();
		if (s.pressed >= PICO4_PANEL_PRESET0 && s.pressed < PICO4_PANEL_PRESET0 + PICO4_PANEL_PRESETS && !s.stored &&
		    now - s.pressed_at >= PICO4_PANEL_HOLD_S && s.mm > 0) {
			s.presets[s.pressed] = round(s.mm * 10) / 10;
			s.stored = true;
			if (!pico4_panel_presets_save(path.c_str(), s.presets)) {
				s.message = "Could not store it";
			}
			dirty = true;
		}
		const pid_t mover = s.mover;
		reap_mover(s);
		dirty = dirty || mover != s.mover;
		if (now >= next_read) {
			next_read = now + 0.25;
			vr::ETrackedPropertyError pe;
			const float m = system->GetFloatTrackedDeviceProperty(vr::k_unTrackedDeviceIndex_Hmd,
			                                                      vr::Prop_UserIpdMeters_Float, &pe);
			if (pe == vr::TrackedProp_Success && m > 0.05f && fabs(m * 1000 - s.mm) >= 0.05) {
				s.mm = m * 1000;
				dirty = true;
			}
			// The driver has caught up with the finished move.
			if (s.mover <= 0 && s.pressed != PICO4_PANEL_SLIDER && s.target > 0 && now - s.pressed_at > 2.5) {
				s.target = 0;
				dirty = true;
			}
		}
		const bool visible = overlay->IsOverlayVisible(main_overlay);
		// Once before anyone looks, as soon as the distance is known, so that the tab never opens blank.
		if (dirty && (visible || (!drawn && s.mm > 0))) {
			drawn = true;
			draw(surface, s);
			if (!texture_show(gpu, main_texture, surface, overlay, main_overlay)) {
				fprintf(stderr, "pico4-lens-panel: image not accepted\n");
			}
			dirty = false;
		}
		usleep(visible ? 16000 : 200000);
	}
	cairo_surface_destroy(surface);
	overlay->DestroyOverlay(main_overlay);
	vrs.shutdown();
	return 0;
}
