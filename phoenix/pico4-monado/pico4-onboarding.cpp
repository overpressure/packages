// Copyright 2026, overpressure contributors.
// SPDX-License-Identifier: BSL-1.0
/*
 * pico4-onboarding: the first run in the headset (docs/session.md#onboarding).
 *
 * Steam runs its own setup only on Valve's factory image; on an installed system with no saved
 * account it goes to the sign-in, and the Frame's guided tour does not run (device type 541). This
 * covers both for the PICO 4:
 *
 * - The setup, before the first sign-in, once: Steam's welcome, its language (the client restarts
 *   in it) and time zone (the system's), Wi-Fi, the controllers and the lens distance, in the
 *   chosen language: the client's own words where it has them, ours otherwise (pico4_setup.h,
 *   pico4-onboarding.strings). Finishing or skipping it leaves ~/.local/share/pico4/setup-done.
 * - Controllers. With no controller connected 20 s after the start, the laser cannot reach
 *   anything: the panel comes up in front of the wearer and the station listens for one
 *   controller after the other, through the driver's pairing socket (pico4_station.h), while the
 *   panel says what to press. With one controller connected, its laser pairs the other.
 * - The introduction after the first sign-in, once: health and safety, the laser and the PICO
 *   button, the playspace. Skipping or finishing it leaves ~/.local/share/pico4/onboarding-done;
 *   the dashboard's Introduction tab shows it again.
 *
 *   pico4-onboarding [--render DIR [LANGUAGE]]
 *
 * --render draws every page into DIR as PNG files, without SteamVR, in LANGUAGE (Steam's name).
 */
#include "pico4_onboarding.h"
#include "pico4_overlay.hpp"
#include "pico4_setup.h"
#include "pico4_text.hpp"

#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/un.h>
#include <sys/wait.h>

#include <algorithm>
#include <cstdlib>
#include <fstream>
#include <sstream>

namespace {

using namespace pico4_overlay;

volatile sig_atomic_t g_stop;

void
stop_requested(int)
{
	g_stop = 1;
}

std::string
home_path(const char *rest)
{
	const char *home = getenv("HOME");
	return std::string(home != nullptr ? home : "/tmp") + rest;
}

std::string
read_file(const std::string &path, size_t limit = 8u << 20)
{
	std::ifstream in(path, std::ios::binary);
	std::string text;
	if (in) {
		std::stringstream s;
		s << in.rdbuf();
		text = s.str();
	}
	if (text.size() > limit) {
		text.clear();
	}
	return text;
}

bool
mark_exists(const char *name)
{
	return access(home_path((std::string("/.local/share/pico4/") + name).c_str()).c_str(), F_OK) == 0;
}

void
set_mark(const char *name)
{
	mkdir(home_path("/.local").c_str(), 0755);
	mkdir(home_path("/.local/share").c_str(), 0755);
	mkdir(home_path("/.local/share/pico4").c_str(), 0755);
	std::ofstream(home_path((std::string("/.local/share/pico4/") + name).c_str())) << "1\n";
}

std::string
registry()
{
	return read_file(home_path("/.steam/registry.vdf"));
}

//! The system's time zone: /etc/localtime's target below zoneinfo, "" when it has none.
std::string
system_zone()
{
	char target[256];
	const ssize_t n = readlink("/etc/localtime", target, sizeof(target) - 1);
	if (n <= 0) {
		return "";
	}
	target[n] = '\0';
	const char *z = strstr(target, "zoneinfo/");
	return z != nullptr ? z + 9 : "";
}

//! A child process with its output collected and its input written at the start, without waiting.
struct Job
{
	pid_t pid = -1;
	int out = -1;
	std::string output;
	int status = -1;

	bool running() const { return pid > 0; }

	//! Start @p argv with @p input on its standard input (a secret goes only there).
	bool start(const std::vector<std::string> &argv, const std::string &input = std::string())
	{
		if (running()) {
			return false;
		}
		int in_pipe[2], out_pipe[2];
		if (pipe2(in_pipe, O_CLOEXEC) != 0) {
			return false;
		}
		if (pipe2(out_pipe, O_CLOEXEC) != 0) {
			close(in_pipe[0]);
			close(in_pipe[1]);
			return false;
		}
		std::vector<char *> args;
		for (const std::string &a : argv) {
			args.push_back(const_cast<char *>(a.c_str()));
		}
		args.push_back(nullptr);
		pid = fork();
		if (pid == 0) {
			dup2(in_pipe[0], 0);
			dup2(out_pipe[1], 1);
			dup2(out_pipe[1], 2);
			execvp(args[0], args.data());
			_exit(127);
		}
		close(in_pipe[0]);
		close(out_pipe[1]);
		if (pid < 0) {
			close(in_pipe[1]);
			close(out_pipe[0]);
			return false;
		}
		if (!input.empty() && write(in_pipe[1], input.data(), input.size()) != (ssize_t)input.size()) {
			fprintf(stderr, "pico4-onboarding: %s: its input not written\n", argv[0].c_str());
		}
		close(in_pipe[1]);
		out = out_pipe[0];
		fcntl(out, F_SETFL, fcntl(out, F_GETFL) | O_NONBLOCK);
		output.clear();
		status = -1;
		return true;
	}

	//! Take its output; true when it ended in this call (status then holds its exit status).
	bool poll()
	{
		if (!running()) {
			return false;
		}
		char buf[1024];
		ssize_t n;
		while ((n = read(out, buf, sizeof(buf))) > 0) {
			output.append(buf, (size_t)n);
		}
		int st;
		if (waitpid(pid, &st, WNOHANG) != pid) {
			return false;
		}
		while ((n = read(out, buf, sizeof(buf))) > 0) {
			output.append(buf, (size_t)n);
		}
		close(out);
		out = -1;
		pid = -1;
		status = WIFEXITED(st) ? WEXITSTATUS(st) : 128;
		return true;
	}
};

//! One request to the driver's pairing socket, read without waiting.
struct Request
{
	int fd = -1;
	std::vector<std::string> answers;
	bool done = false;

	//! Connect and send; false when the driver has no socket (no session, or it is starting).
	bool start(const std::string &text)
	{
		const char *runtime = getenv("XDG_RUNTIME_DIR");
		sockaddr_un addr{};
		addr.sun_family = AF_UNIX;
		snprintf(addr.sun_path, sizeof(addr.sun_path), "%s/%s", runtime != nullptr ? runtime : "/run/user/1000",
		         "pico4-station");
		// Without waiting: a driver busy with another request leaves the connection in its backlog.
		fd = socket(AF_UNIX, SOCK_SEQPACKET | SOCK_NONBLOCK | SOCK_CLOEXEC, 0);
		if (fd < 0 || connect(fd, reinterpret_cast<const sockaddr *>(&addr), sizeof(addr)) != 0 ||
		    send(fd, text.data(), text.size(), MSG_NOSIGNAL) != (ssize_t)text.size()) {
			finish();
			return false;
		}
		done = false;
		answers.clear();
		return true;
	}

	//! Take what arrived; true when something did.
	bool poll()
	{
		bool any = false;
		char buf[256];
		while (fd >= 0) {
			ssize_t n = recv(fd, buf, sizeof(buf) - 1, MSG_DONTWAIT);
			if (n > 0) {
				buf[n] = '\0';
				answers.emplace_back(buf);
				any = true;
			} else if (n == 0 || (errno != EAGAIN && errno != EWOULDBLOCK)) {
				finish();
				done = true;
				any = true;
			} else {
				break;
			}
		}
		return any;
	}

	void finish()
	{
		if (fd >= 0) {
			close(fd);
		}
		fd = -1;
	}

	bool active() const { return fd >= 0; }
};

//! The texts in the current language: ours from the strings file, the rest the Steam client's own.
struct Texts
{
	std::string language = "english";
	std::string ours;
	std::string ui, shared, ui_en, shared_en; //!< the client's localization files
	std::string module;                       //!< its language-screen texts, every language
	std::string steam = home_path("/.local/share/Steam");
	bool steam_found = false;

	//! The client's language-screen module, found by its content (its file name is a build's).
	void find_module()
	{
		DIR *d = opendir((steam + "/steamui").c_str());
		if (d == nullptr) {
			return;
		}
		while (dirent *e = readdir(d)) {
			const std::string name = e->d_name;
			if (name.size() < 4 || name.compare(name.size() - 3, 3, ".js") != 0) {
				continue;
			}
			const std::string text = read_file(steam + "/steamui/" + name, 256u << 10);
			if (text.find("Steam_Language_Selection_Label:") != std::string::npos) {
				module = text;
				break;
			}
		}
		closedir(d);
	}

	void load(const std::string &lang)
	{
		language = pico4_setup_language_index(lang.c_str()) >= 0 ? lang : "english";
		const char *path = getenv("PICO4_ONBOARDING_STRINGS");
		ours = read_file(path != nullptr ? path : "/usr/share/pico4/onboarding.strings");
		const std::string loc = steam + "/steamui/localization/";
		ui = read_file(loc + "steamui_" + language + "-json.js");
		shared = read_file(loc + "shared_" + language + "-json.js");
		ui_en = read_file(loc + "steamui_english-json.js");
		shared_en = read_file(loc + "shared_english-json.js");
		if (module.empty()) {
			find_module();
		}
		steam_found = !ui_en.empty();
		pico4_text::add_steam_fonts(steam);
	}

	std::string our(const char *key) const
	{
		char out[1024];
		if (pico4_setup_string(ours.c_str(), language.c_str(), key, out, sizeof(out)) ||
		    pico4_setup_string(ours.c_str(), "english", key, out, sizeof(out))) {
			return out;
		}
		return key;
	}

	//! The client's text @p token in the current language, else in English, else @p fallback.
	std::string steam_text(const char *token, const char *fallback) const
	{
		char out[1024];
		for (const std::string *file : {&ui, &shared, &ui_en, &shared_en}) {
			if (pico4_setup_steam_token(file->c_str(), token, out, sizeof(out))) {
				return out;
			}
		}
		return fallback;
	}

	//! A language-screen text of @p lang (its welcome, its own name), else @p fallback.
	std::string language_text(const char *lang, const char *key, const char *fallback) const
	{
		char out[512];
		return pico4_setup_language_text(module.c_str(), lang, key, out, sizeof(out)) ? out : fallback;
	}
};

//! @p pattern with its first %s or %1$s replaced by @p value.
std::string
fill(std::string pattern, const std::string &value)
{
	for (const char *mark : {"%1$s", "%s"}) {
		const size_t at = pattern.find(mark);
		if (at != std::string::npos) {
			return pattern.replace(at, strlen(mark), value);
		}
	}
	return pattern;
}

struct Setup
{
	bool active = false; //!< the setup's pages instead of the introduction
	enum pico4_setup_page page = PICO4_SETUP_WELCOME;
	int list_page = 0;
	int language = 0;
	int zone = 0;
	std::vector<pico4_setup_network> nets;
	bool scanned = false, wifi_on = true;
	std::string wifi_message; //!< the list's note: the network just joined
	enum pico4_setup_wifi_step wifi = PICO4_WIFI_LIST;
	std::string ssid;      //!< the network being joined
	bool hidden = false;   //!< that network hides its name
	bool secured = false;  //!< that network asks for a password
	char entry[PICO4_SETUP_TEXT] = {}; //!< the name or password being typed
	bool reveal = false;   //!< the password shown as typed
	bool retry = false;    //!< the password again, after a refused one
	enum pico4_setup_join join = PICO4_JOIN_OK;
	double step_at = 0;    //!< when the Wi-Fi step began (the spinner, the joined sign's time)
	bool keyboard = false; //!< SteamVR's keyboard is up for the field
	int typed = 0;         //!< characters the keyboard sent since it came up
	int ipd = -1, ipd_target = -1;
	std::string message;   //!< a page's note (the language's restart, a failure)
	bool message_error = false;
	int pressed_row = -1, pressed_arrow = 0, pressed_ipd = 0, pressed_field = 0;
	double next_scan = 0;
	Job language_job, zone_job, scan_job, connect_job, ipd_job, radio_job, forget_job;
};

struct State
{
	enum pico4_tour_page page = PICO4_TOUR_CONTROLLERS;
	bool shown = false;     //!< The panel in front of the wearer.
	bool in_tab = false;    //!< The pages run in the dashboard's tab (started from it).
	bool tour = false;      //!< The pages past the controllers follow.
	bool known = false;     //!< The driver answered a status.
	bool left = false, right = false;
	int pairing = 0;        //!< The hand being paired: 1 left, 2 right.
	int last_hands_free = 0;
	bool hands_free = false; //!< Pairing started because no controller was there.
	std::string message;
	int pressed = PICO4_TOUR_NONE;
	Setup setup;
};

// -- Drawing --------------------------------------------------------------------------------------

constexpr double kTextX = 60, kTextW = PICO4_TOUR_W - 120;

void
background(cairo_t *c)
{
	cairo_set_source_rgb(c, 0.11, 0.12, 0.14);
	cairo_paint(c);
}

//! The page's title on one line: smaller where it would not fit (a network's name in a sentence).
void
title(cairo_t *c, const std::string &text)
{
	cairo_set_source_rgb(c, 1, 1, 1);
	double size = 44;
	while (size > 30 && pico4_text::draw(c, text, kTextX, 40, kTextW, size, true, pico4_text::kStart, true) > size * 1.5) {
		size -= 2;
	}
	pico4_text::line(c, text, kTextX, 40 + (44 - size) / 2, kTextW, size, true);
}

void
button(cairo_t *c, double x, const std::string &label, bool pressed, bool primary)
{
	rounded(c, x, PICO4_TOUR_BUTTON_Y, PICO4_TOUR_BUTTON_W, PICO4_TOUR_BUTTON_H, 16);
	if (pressed) {
		cairo_set_source_rgb(c, 0.20, 0.44, 0.72);
	} else if (primary) {
		cairo_set_source_rgb(c, 0.16, 0.36, 0.60);
	} else {
		cairo_set_source_rgb(c, 0.22, 0.24, 0.27);
	}
	cairo_fill(c);
	cairo_set_source_rgb(c, 1, 1, 1);
	const double h = pico4_text::draw(c, label, x + 10, 0, PICO4_TOUR_BUTTON_W - 20, 28, true, pico4_text::kCenter, true);
	pico4_text::draw(c, label, x + 10, PICO4_TOUR_BUTTON_Y + (PICO4_TOUR_BUTTON_H - h) / 2, PICO4_TOUR_BUTTON_W - 20, 28,
	                 true, pico4_text::kCenter);
}

//! A paragraph from @p y down; returns the next y.
double
paragraph(cairo_t *c, const std::string &text, double y, double size)
{
	return y + pico4_text::draw(c, text, kTextX, y, kTextW, size, false) + size * 0.5;
}

//! The controllers' state, what to press and the pairing's note, from @p y down.
double
controllers(cairo_t *c, const State &s, const Texts &t, double y)
{
	for (int hand = 1; hand <= 2; hand++) {
		const bool on = hand == 1 ? s.left : s.right;
		const std::string state = !s.known ? "..." : on ? t.our("controller_connected")
		                                     : s.pairing == hand ? t.our("controller_waiting")
		                                                          : t.our("controller_missing");
		cairo_set_source_rgb(c, on ? 0.55 : 0.95, on ? 0.85 : 0.75, on ? 0.55 : 0.45);
		y = paragraph(c, fill(t.our(hand == 1 ? "controller_left" : "controller_right"), state), y, 30);
	}
	y += 6;
	cairo_set_source_rgb(c, 0.85, 0.87, 0.90);
	const int pair_hand = pico4_tour_pair_button(s.left, s.right);
	if (s.pairing != 0) {
		y = paragraph(c, t.our(s.pairing == 1 ? "controller_pair_left" : "controller_pair_right"), y, 28);
		y = paragraph(c, t.our("controller_pair_how"), y, 28);
	} else if (s.left && s.right) {
		y = paragraph(c, t.our("controllers_both"), y, 28);
	} else if (pair_hand != 0) {
		y = paragraph(c, t.our(pair_hand == 1 ? "controller_point_left" : "controller_point_right"), y, 28);
	}
	return y;
}

void
note(cairo_t *c, const std::string &text, bool error)
{
	if (text.empty()) {
		return;
	}
	if (error) {
		cairo_set_source_rgb(c, 0.95, 0.55, 0.45);
	} else {
		cairo_set_source_rgb(c, 0.60, 0.80, 0.95);
	}
	pico4_text::line(c, text, kTextX, 440, kTextW, 24, false);
}

void
draw_tour(cairo_t *c, const State &s, const Texts &t)
{
	static const char *const titles[PICO4_TOUR_PAGES] = {"controllers_title", "safety_title", "controls_title",
	                                                      "playspace_title", "done_title"};
	static const char *const lines[PICO4_TOUR_PAGES][4] = {
	    {nullptr},
	    {"safety_space", "safety_light", "safety_sun", "safety_breaks"},
	    {"controls_point", "controls_menu", "controls_recenter", nullptr},
	    {"playspace_new", "playspace_later", nullptr, nullptr},
	    {"done_again", nullptr, nullptr, nullptr},
	};
	title(c, t.our(titles[s.page]));
	double y = 130;
	if (s.page == PICO4_TOUR_CONTROLLERS) {
		controllers(c, s, t, y);
	} else {
		cairo_set_source_rgb(c, 0.85, 0.87, 0.90);
		for (int l = 0; l < 4 && lines[s.page][l] != nullptr; l++) {
			y = paragraph(c, t.our(lines[s.page][l]), y, 30);
		}
	}
	note(c, s.message, true);
	const int pair_hand = pico4_tour_pair_button(s.left, s.right);
	const bool last = s.page == PICO4_TOUR_DONE || (!s.tour && s.page == PICO4_TOUR_CONTROLLERS);
	button(c, PICO4_TOUR_SKIP_X, t.our(s.tour ? "skip" : "close"), s.pressed == PICO4_TOUR_SKIP, false);
	if (s.page == PICO4_TOUR_CONTROLLERS && pair_hand != 0 && s.pairing == 0) {
		button(c, PICO4_TOUR_PAIR_X, t.our("pair"), s.pressed == PICO4_TOUR_PAIR, false);
	}
	button(c, PICO4_TOUR_NEXT_X, last ? t.steam_text("Button_Done", "Done") : t.steam_text("Button_Next", "Next"),
	       s.pressed == PICO4_TOUR_NEXT, true);
}

//! The list's count of rows on @p page.
int
list_count(const Setup &u)
{
	switch (u.page) {
	case PICO4_SETUP_LANGUAGE: return PICO4_SETUP_LANGUAGES;
	case PICO4_SETUP_TIMEZONE: return PICO4_SETUP_ZONES;
	case PICO4_SETUP_WIFI: // and the hidden network; past the list, none
		return u.wifi_on && u.wifi == PICO4_WIFI_LIST ? (int)u.nets.size() + 1 : 0;
	default: return 0;
	}
}

void
arrows(cairo_t *c, const Setup &u, int count)
{
	const int pages = pico4_setup_list_pages(count);
	if (pages <= 1) {
		return;
	}
	const double down_y = PICO4_SETUP_LIST_Y + PICO4_SETUP_ROWS * PICO4_SETUP_ROW_H - PICO4_SETUP_ARROW_H;
	for (int d : {-1, 1}) {
		const bool can = d < 0 ? u.list_page > 0 : u.list_page < pages - 1;
		const double y = d < 0 ? PICO4_SETUP_LIST_Y : down_y;
		rounded(c, PICO4_SETUP_ARROW_X, y, PICO4_SETUP_ARROW_W, PICO4_SETUP_ARROW_H, 14);
		if (u.pressed_arrow == d) {
			cairo_set_source_rgb(c, 0.20, 0.44, 0.72);
		} else {
			cairo_set_source_rgb(c, 0.22, 0.24, 0.27);
		}
		cairo_fill(c);
		cairo_set_source_rgba(c, 1, 1, 1, can ? 1 : 0.3);
		const double cx = PICO4_SETUP_ARROW_X + PICO4_SETUP_ARROW_W / 2, cy = y + PICO4_SETUP_ARROW_H / 2;
		cairo_move_to(c, cx - 18, cy - 9 * d);
		cairo_line_to(c, cx + 18, cy - 9 * d);
		cairo_line_to(c, cx, cy + 12 * d);
		cairo_close_path(c);
		cairo_fill(c);
	}
	char text[32];
	snprintf(text, sizeof(text), "%d / %d", u.list_page + 1, pages);
	cairo_set_source_rgb(c, 0.65, 0.67, 0.70);
	pico4_text::draw(c, text, PICO4_SETUP_ARROW_X - 10, PICO4_SETUP_LIST_Y + 160, PICO4_SETUP_ARROW_W + 20, 20, false,
	                 pico4_text::kCenter);
}

//! Signal bars (0-4) at (@p x, @p y): four rising bars, the strong ones lit.
void
signal_bars(cairo_t *c, double x, double y, int bars)
{
	for (int b = 0; b < 4; b++) {
		const double h = 6 + b * 5;
		cairo_rectangle(c, x + b * 8, y + 22 - h, 5, h);
		cairo_set_source_rgba(c, 1, 1, 1, b < bars ? 0.95 : 0.25);
		cairo_fill(c);
	}
}

//! A padlock at (@p x, @p y), for a network that asks for a password.
void
padlock(cairo_t *c, double x, double y)
{
	cairo_set_source_rgba(c, 1, 1, 1, 0.85);
	cairo_set_line_width(c, 2.5);
	cairo_arc_negative(c, x + 8, y + 9, 5, 0, M_PI);
	cairo_stroke(c);
	rounded(c, x, y + 9, 16, 12, 2);
	cairo_fill(c);
}

/*!
 * One list row: highlighted when chosen, its main text, under it a smaller @p sub line, at the
 * right a smaller @p side text, and for a network its signal and its lock.
 */
void
row(cairo_t *c, int r, const std::string &main, const std::string &sub, const std::string &side, bool chosen,
    bool pressed, int bars = -1, bool lock = false)
{
	const double y = PICO4_SETUP_LIST_Y + r * PICO4_SETUP_ROW_H;
	rounded(c, PICO4_SETUP_LIST_X, y + 3, PICO4_SETUP_LIST_W, PICO4_SETUP_ROW_H - 6, 10);
	if (pressed) {
		cairo_set_source_rgb(c, 0.20, 0.44, 0.72);
	} else if (chosen) {
		cairo_set_source_rgb(c, 0.16, 0.36, 0.60);
	} else {
		cairo_set_source_rgb(c, 0.17, 0.18, 0.21);
	}
	cairo_fill(c);
	double right = PICO4_SETUP_LIST_X + PICO4_SETUP_LIST_W - 16;
	if (lock) {
		right -= 16;
		padlock(c, right, y + 17);
		right -= 12;
	}
	if (bars >= 0) {
		right -= 29;
		signal_bars(c, right, y + 16, bars);
		right -= 12;
	}
	const double side_w = side.empty() ? 0 : 260;
	if (!side.empty()) {
		cairo_set_source_rgb(c, 0.75, 0.78, 0.82);
		pico4_text::line(c, side, right - side_w, y + 19, side_w, 19, false, false);
	}
	const double x = PICO4_SETUP_LIST_X + 20, w = right - side_w - 10 - x;
	cairo_set_source_rgb(c, 1, 1, 1);
	if (sub.empty()) {
		pico4_text::line(c, main, x, y + 14, w, 25, chosen, false);
	} else {
		pico4_text::line(c, main, x, y + 6, w, 23, chosen, false);
		cairo_set_source_rgb(c, 0.75, 0.78, 0.82);
		pico4_text::line(c, sub, x, y + 34, w, 17, false, false);
	}
}

//! Whether the input page's text can be sent: a name for a hidden network, a usable password.
bool
entry_ok(const Setup &u)
{
	return u.wifi == PICO4_WIFI_NAME ? u.entry[0] != '\0' : pico4_setup_password_ok(u.entry, u.hidden && !u.secured);
}

//! The attempt's sign, centred at (@p x, @p y): a turning arc while joining, a tick, or a cross.
void
join_sign(cairo_t *c, enum pico4_setup_wifi_step step, double x, double y, double since)
{
	cairo_new_path(c); // no line from where the text before ended
	cairo_set_line_width(c, 9);
	cairo_set_line_cap(c, CAIRO_LINE_CAP_ROUND);
	if (step == PICO4_WIFI_JOINING) {
		cairo_set_source_rgba(c, 1, 1, 1, 0.15);
		cairo_arc(c, x, y, 44, 0, 2 * M_PI);
		cairo_stroke(c);
		const double a = std::fmod(since * 1.6, 1.0) * 2 * M_PI; // a turn in 0.6 s
		cairo_set_source_rgb(c, 0.40, 0.65, 0.95);
		cairo_arc(c, x, y, 44, a, a + 1.6);
		cairo_stroke(c);
		return;
	}
	const bool ok = step == PICO4_WIFI_JOINED;
	cairo_set_source_rgb(c, ok ? 0.45 : 0.90, ok ? 0.80 : 0.40, ok ? 0.50 : 0.35);
	cairo_arc(c, x, y, 44, 0, 2 * M_PI);
	cairo_stroke(c);
	cairo_new_path(c);
	if (ok) {
		cairo_move_to(c, x - 20, y + 2);
		cairo_line_to(c, x - 5, y + 17);
		cairo_line_to(c, x + 22, y - 14);
	} else {
		cairo_move_to(c, x - 16, y - 16);
		cairo_line_to(c, x + 16, y + 16);
		cairo_move_to(c, x + 16, y - 16);
		cairo_line_to(c, x - 16, y + 16);
	}
	cairo_stroke(c);
}

/*!
 * The Wi-Fi page past its list: the hidden network's name or the password in a field, typed on
 * SteamVR's keyboard; the attempt, with its sign and Steam's words for its end.
 */
void
draw_wifi_step(cairo_t *c, const State &s, const Texts &t, double now)
{
	const Setup &u = s.setup;
	std::string back = t.steam_text("Button_Back", "Back"), next;
	if (u.wifi == PICO4_WIFI_NAME || u.wifi == PICO4_WIFI_PASSWORD) {
		const bool name = u.wifi == PICO4_WIFI_NAME;
		title(c, name ? t.steam_text("Settings_Internet_Enter_Network_Credentials", "Enter your network information")
		              : fill(t.steam_text(u.retry ? "Settings_Internet_Reenter_The_Password_For_Network"
		                                          : "Settings_Internet_Enter_The_Password_For_Network",
		                                  u.retry ? "Re-enter password for %1$s" : "Enter password for %1$s"),
		                     u.ssid));
		cairo_set_source_rgb(c, 0.75, 0.78, 0.82);
		pico4_text::line(c,
		                 name ? t.steam_text("Settings_Internet_Enter_NetworkName", "Network Name")
		                      : t.steam_text("Settings_Internet_Enter_Password", "Password"),
		                 PICO4_SETUP_FIELD_X, PICO4_SETUP_FIELD_Y - 36, PICO4_SETUP_FIELD_W, 22, false);
		rounded(c, PICO4_SETUP_FIELD_X, PICO4_SETUP_FIELD_Y, PICO4_SETUP_FIELD_W, PICO4_SETUP_FIELD_H, 12);
		cairo_set_source_rgb(c, 0.07, 0.08, 0.09);
		cairo_fill_preserve(c);
		if (u.keyboard || u.pressed_field == 1) {
			cairo_set_source_rgb(c, 0.40, 0.65, 0.95);
		} else {
			cairo_set_source_rgb(c, 0.35, 0.37, 0.40);
		}
		cairo_set_line_width(c, 3);
		cairo_stroke(c);
		std::string shown = u.entry;
		if (!name && !u.reveal) {
			shown.clear();
			for (size_t i = pico4_setup_text_chars(u.entry); i > 0; i--) {
				shown += "\xE2\x97\x8F"; // a dot for each character
			}
		}
		const double tx = PICO4_SETUP_FIELD_X + 24, tw = PICO4_SETUP_FIELD_W - 48, ty = PICO4_SETUP_FIELD_Y + 22;
		cairo_set_source_rgb(c, 1, 1, 1);
		const double w = shown.empty() ? 0 : pico4_text::field(c, shown, tx, ty, tw, 30);
		if (u.keyboard && std::fmod(now, 1.0) < 0.6) {
			// The caret, blinking while the keyboard is up.
			cairo_rectangle(c, tx + w + 3, PICO4_SETUP_FIELD_Y + 18, 3, PICO4_SETUP_FIELD_H - 36);
			cairo_fill(c);
		}
		if (!name) {
			// The show-password switch: a box, ticked when the password shows.
			const double bx = PICO4_SETUP_FIELD_X, by = PICO4_SETUP_SHOW_Y + 10;
			rounded(c, bx, by, 30, 30, 6);
			cairo_set_source_rgb(c, u.pressed_field == 2 ? 0.20 : 0.22, u.pressed_field == 2 ? 0.44 : 0.24,
			                     u.pressed_field == 2 ? 0.72 : 0.27);
			cairo_fill(c);
			if (u.reveal) {
				cairo_set_source_rgb(c, 1, 1, 1);
				cairo_set_line_width(c, 4);
				cairo_move_to(c, bx + 7, by + 15);
				cairo_line_to(c, bx + 13, by + 22);
				cairo_line_to(c, bx + 24, by + 8);
				cairo_stroke(c);
			}
			cairo_set_source_rgb(c, 0.85, 0.87, 0.90);
			pico4_text::line(c, t.steam_text("Login_ShowPassword", "Show Password"), bx + 44, by + 3,
			                 PICO4_SETUP_SHOW_W - 44, 22, false);
		}
		if (u.retry && !name) {
			note(c, t.steam_text("Login_PasswordMismatch", "Incorrect Password"), true);
		}
		next = t.steam_text("Settings_Internet_Connect", "Connect");
		if (name) {
			next = t.steam_text("Button_Continue", "Continue");
		}
		button(c, PICO4_TOUR_SKIP_X, back, s.pressed == PICO4_TOUR_SKIP, false);
		button(c, PICO4_TOUR_NEXT_X, next, s.pressed == PICO4_TOUR_NEXT, entry_ok(u));
		return;
	}
	title(c, u.ssid);
	join_sign(c, u.wifi, PICO4_TOUR_W / 2.0, 230, now - u.step_at);
	std::string text;
	switch (u.wifi) {
	case PICO4_WIFI_JOINING:
		text = fill(t.steam_text("Settings_Internet_Connecting_To_Network", "Connecting to %1$s..."), u.ssid);
		break;
	case PICO4_WIFI_JOINED: text = t.steam_text("Settings_Internet_Connection_Successful", "Connection successful!"); break;
	default:
		text = t.steam_text("Settings_Internet_Failed_To_Connect", "Failed to Connect");
		break;
	}
	cairo_set_source_rgb(c, 1, 1, 1);
	pico4_text::draw(c, text, kTextX, 310, kTextW, 30, true, pico4_text::kCenter);
	if (u.wifi != PICO4_WIFI_FAILED) {
		return;
	}
	const std::string why =
	    u.join == PICO4_JOIN_PASSWORD
	        ? t.steam_text("Login_PasswordMismatch", "Incorrect Password")
	    : u.join == PICO4_JOIN_NOT_FOUND
	        ? fill(t.steam_text("Settings_Internet_Network_Not_Found", "Network not found: %1$s"), u.ssid)
	        : t.steam_text("Settings_Internet_Failed_To_Connect_To_Network", "Failed to connect to the network.");
	cairo_set_source_rgb(c, 0.95, 0.55, 0.45);
	pico4_text::draw(c, why, kTextX, 360, kTextW, 26, false, pico4_text::kCenter);
	button(c, PICO4_TOUR_SKIP_X, back, s.pressed == PICO4_TOUR_SKIP, false);
	button(c, PICO4_TOUR_NEXT_X, t.steam_text("Button_TryAgain", "Try Again"), s.pressed == PICO4_TOUR_NEXT, true);
}

void
draw_setup(cairo_t *c, const State &s, const Texts &t, double now)
{
	const Setup &u = s.setup;
	if (u.page == PICO4_SETUP_WIFI && u.wifi != PICO4_WIFI_LIST) {
		draw_wifi_step(c, s, t, now);
		return;
	}
	const int first = u.list_page * PICO4_SETUP_ROWS;
	const int count = list_count(u);
	std::string middle;
	switch (u.page) {
	case PICO4_SETUP_WELCOME: {
		title(c, t.language_text(t.language.c_str(), "Steam_Welcome", "Welcome"));
		cairo_set_source_rgb(c, 0.85, 0.87, 0.90);
		double y = paragraph(c, t.our("setup_welcome"), 130, 28);
		y = controllers(c, s, t, y + 6);
		if (s.left || s.right) {
			cairo_set_source_rgb(c, 0.85, 0.87, 0.90);
			paragraph(c, t.our("setup_welcome_point"), y, 26);
		}
		break;
	}
	case PICO4_SETUP_LANGUAGE:
		title(c, t.language_text(t.language.c_str(), "Steam_Language_Selection_Label", "Select a language"));
		for (int r = 0; r < PICO4_SETUP_ROWS && first + r < count; r++) {
			const pico4_setup_language &l = pico4_setup_languages[first + r];
			const std::string own = t.language_text(l.steam, "Language_Name", l.english);
			row(c, r, own, "", own == l.english ? "" : l.english, first + r == u.language, u.pressed_row == r);
		}
		break;
	case PICO4_SETUP_TIMEZONE:
		title(c, t.steam_text("Settings_TimeAndDate_ChooseTimezone_OOBE", "Choose your timezone"));
		for (int r = 0; r < PICO4_SETUP_ROWS && first + r < count; r++) {
			const pico4_setup_zone &z = pico4_setup_zones[first + r];
			char offset[16];
			pico4_setup_offset_text(z.offset, offset, sizeof(offset));
			const std::string region = t.steam_text((std::string("TimeZoneRegion_") + z.region).c_str(), z.zone);
			const std::string cities = t.steam_text((std::string("TimeZoneCities_") + z.cities).c_str(), "");
			row(c, r, region, std::string(offset) + (cities.empty() ? "" : "  \xC2\xB7  " + cities), "", first + r == u.zone,
			    u.pressed_row == r);
		}
		break;
	case PICO4_SETUP_WIFI:
		title(c, t.steam_text("Login_Welcome_ChooseYourNetwork", "Choose your network"));
		middle = u.wifi_on ? t.steam_text("Settings_Internet_Refresh", "Refresh") : t.our("setup_wifi_on");
		if (!u.wifi_on) {
			cairo_set_source_rgb(c, 0.85, 0.87, 0.90);
			paragraph(c, t.our("setup_wifi_off"), 140, 28);
			break;
		}
		if (u.scanned && u.nets.empty() && u.list_page == 0) {
			cairo_set_source_rgb(c, 0.85, 0.87, 0.90);
			pico4_text::line(c, t.steam_text("Login_NoNetworksFound", "No networks found"), PICO4_SETUP_LIST_X + 20,
			                 PICO4_SETUP_LIST_Y + PICO4_SETUP_ROW_H + 11, PICO4_SETUP_LIST_W, 26, false);
		}
		for (int r = 0; r < PICO4_SETUP_ROWS && first + r < count; r++) {
			const int i = first + r;
			if (i == (int)u.nets.size()) {
				row(c, r, t.steam_text("Settings_Internet_Hidden_Network", "Hidden Network"), "", "", false,
				    u.pressed_row == r);
				continue;
			}
			const pico4_setup_network &n = u.nets[(size_t)i];
			const std::string side = n.active ? t.steam_text("Login_Welcome_Connected", "Connected") : "";
			row(c, r, n.ssid, "", side, n.active, u.pressed_row == r, pico4_setup_bars(n.signal), n.secured);
		}
		break;
	case PICO4_SETUP_CONTROLLERS:
		title(c, t.our("controllers_title"));
		controllers(c, s, t, 130);
		if (pico4_tour_pair_button(s.left, s.right) != 0 && s.pairing == 0) {
			middle = t.our("pair");
		}
		break;
	case PICO4_SETUP_IPD: {
		title(c, t.our("setup_ipd_title"));
		cairo_set_source_rgb(c, 0.85, 0.87, 0.90);
		paragraph(c, t.our("setup_ipd_text"), 120, 26);
		for (int d : {-1, 1}) {
			const double x = d < 0 ? PICO4_SETUP_IPD_LESS_X : PICO4_SETUP_IPD_MORE_X;
			rounded(c, x, PICO4_SETUP_IPD_Y, PICO4_SETUP_IPD_W, PICO4_SETUP_IPD_H, 16);
			if (u.pressed_ipd == d) {
				cairo_set_source_rgb(c, 0.20, 0.44, 0.72);
			} else {
				cairo_set_source_rgb(c, 0.22, 0.24, 0.27);
			}
			cairo_fill(c);
			cairo_set_source_rgb(c, 1, 1, 1);
			pico4_text::draw(c, d < 0 ? "\xE2\x88\x92" : "+", x, PICO4_SETUP_IPD_Y + 22, PICO4_SETUP_IPD_W, 64, true,
			                 pico4_text::kCenter);
		}
		char mm[32];
		const int shown = u.ipd_target >= 0 ? u.ipd_target : u.ipd;
		if (shown >= 0) {
			snprintf(mm, sizeof(mm), "%d.%d mm", shown / 100, shown % 100 / 10);
		} else {
			snprintf(mm, sizeof(mm), "...");
		}
		cairo_set_source_rgb(c, 1, 1, 1);
		pico4_text::draw(c, mm, 330, PICO4_SETUP_IPD_Y + 28, 340, 56, true, pico4_text::kCenter);
		cairo_set_source_rgb(c, 0.65, 0.67, 0.70);
		pico4_text::draw(c, t.our("setup_ipd_range"), kTextX, 380, kTextW, 22, false, pico4_text::kCenter);
		break;
	}
	case PICO4_SETUP_READY:
		title(c, t.our("setup_ready_title"));
		cairo_set_source_rgb(c, 0.85, 0.87, 0.90);
		paragraph(c, fill(t.our("setup_ready_text"), t.steam_text("Button_Done", "Done")), 130, 30);
		break;
	case PICO4_SETUP_PAGES: break;
	}
	arrows(c, u, count);
	if (u.page == PICO4_SETUP_WIFI) {
		note(c, u.wifi_message.empty() ? t.our("setup_wifi_later") : u.wifi_message, false);
	} else if (!s.message.empty()) {
		note(c, s.message, true);
	} else if (!u.message.empty()) {
		note(c, u.message, u.message_error);
	} else if (u.page == PICO4_SETUP_TIMEZONE) {
		note(c, t.our("setup_timezone_note"), false);
	}
	button(c, PICO4_TOUR_SKIP_X,
	       u.page == PICO4_SETUP_WELCOME ? t.our("skip") : t.steam_text("Button_Back", "Back"),
	       s.pressed == PICO4_TOUR_SKIP, false);
	if (!middle.empty()) {
		button(c, PICO4_TOUR_PAIR_X, middle, s.pressed == PICO4_TOUR_PAIR, false);
	}
	button(c, PICO4_TOUR_NEXT_X,
	       u.page == PICO4_SETUP_READY ? t.steam_text("Button_Done", "Done") : t.steam_text("Button_Continue", "Continue"),
	       s.pressed == PICO4_TOUR_NEXT, true);
}

void
draw(cairo_surface_t *surface, const State &s, const Texts &t, double now = 0)
{
	cairo_t *c = cairo_create(surface);
	background(c);
	if (s.setup.active) {
		draw_setup(c, s, t, now);
	} else {
		draw_tour(c, s, t);
	}
	cairo_destroy(c);
	cairo_surface_flush(surface);
}

void
draw_start(cairo_surface_t *surface, const Texts &t, bool pressed)
{
	cairo_t *c = cairo_create(surface);
	background(c);
	cairo_set_source_rgb(c, 1, 1, 1);
	pico4_text::draw(c, t.our("start_title"), kTextX, 100, kTextW, 48, true, pico4_text::kCenter);
	cairo_set_source_rgb(c, 0.85, 0.87, 0.90);
	pico4_text::draw(c, t.our("start_text"), kTextX, 200, kTextW, 28, false, pico4_text::kCenter);
	button(c, PICO4_TOUR_NEXT_X - 330, t.our("start_show"), pressed, true);
	cairo_destroy(c);
	cairo_surface_flush(surface);
}

void
draw_thumbnail(cairo_surface_t *surface)
{
	cairo_t *c = cairo_create(surface);
	background(c);
	cairo_set_source_rgb(c, 1, 1, 1);
	pico4_text::draw(c, "?", 0, 40, 256, 150, true, pico4_text::kCenter);
	cairo_destroy(c);
	cairo_surface_flush(surface);
}

//! How far in front of the wearer the panel stands: before Steam's window, which follows the view.
constexpr double kPanelDistance = 1.3;

//! Where the panel stands: its centre, in the standing universe; valid once placed while tracked.
struct Placement
{
	double x = 0, y = 0, z = 0;
	bool valid = false;
};

//! The panel in front of the wearer, level, facing them, where they look now.
void
place_in_front(vr::IVROverlay *overlay, vr::IVRSystem *system, vr::VROverlayHandle_t handle, Placement &at)
{
	vr::TrackedDevicePose_t pose;
	system->GetDeviceToAbsoluteTrackingPose(vr::TrackingUniverseStanding, 0, &pose, 1);
	if (!pose.bPoseIsValid) {
		// Head-locked until the head is tracked.
		vr::HmdMatrix34_t m = {{{1, 0, 0, 0}, {0, 1, 0, 0}, {0, 0, 1, (float)-kPanelDistance}}};
		overlay->SetOverlayTransformTrackedDeviceRelative(handle, vr::k_unTrackedDeviceIndex_Hmd, &m);
		at.valid = false;
		return;
	}
	const vr::HmdMatrix34_t &h = pose.mDeviceToAbsoluteTracking;
	double fx = -h.m[0][2], fz = -h.m[2][2]; // the head's forward, level
	const double n = std::sqrt(fx * fx + fz * fz);
	if (n < 1e-3) {
		fx = 0, fz = -1;
	} else {
		fx /= n, fz /= n;
	}
	// The panel's +z (its front) points back at the wearer: a turn about up by atan2(-fx, -fz).
	const double yaw = std::atan2(-fx, -fz), cy = std::cos(yaw), sy = std::sin(yaw);
	at = {h.m[0][3] + kPanelDistance * fx, h.m[1][3], h.m[2][3] + kPanelDistance * fz, true};
	vr::HmdMatrix34_t m = {{{(float)cy, 0, (float)sy, (float)at.x}, {0, 1, 0, (float)at.y}, {(float)-sy, 0, (float)cy, (float)at.z}}};
	overlay->SetOverlayTransformAbsolute(handle, vr::TrackingUniverseStanding, &m);
}

/*!
 * Whether the panel at @p at has left the wearer's view, as Steam's window would follow: more than
 * 30 degrees aside of where they look, or more than 0.3 m above or below their eyes. Then it is
 * placed in front again; while they look at it, it stays (the laser needs it still).
 */
bool
out_of_view(vr::IVRSystem *system, const Placement &at)
{
	vr::TrackedDevicePose_t pose;
	system->GetDeviceToAbsoluteTrackingPose(vr::TrackingUniverseStanding, 0, &pose, 1);
	if (!pose.bPoseIsValid) {
		return false;
	}
	if (!at.valid) {
		return true; // placed head-locked before tracking: into the room now
	}
	const vr::HmdMatrix34_t &h = pose.mDeviceToAbsoluteTracking;
	const double fx = -h.m[0][2], fz = -h.m[2][2], dx = at.x - h.m[0][3], dz = at.z - h.m[2][3];
	const double nf = std::sqrt(fx * fx + fz * fz), nd = std::sqrt(dx * dx + dz * dz);
	if (nf < 1e-3 || nd < 1e-3) {
		return false;
	}
	const double cosine = (fx * dx + fz * dz) / (nf * nd);
	return cosine < std::cos(30.0 * M_PI / 180.0) || std::fabs(at.y - h.m[1][3]) > 0.3;
}

int
render(const char *dir, const char *language)
{
	Texts t;
	t.load(language != nullptr ? language : "english");
	cairo_surface_t *surface = cairo_image_surface_create(CAIRO_FORMAT_ARGB32, PICO4_TOUR_W, PICO4_TOUR_H);
	int r = 0;
	auto save = [&](const State &s, const char *name) {
		draw(surface, s, t);
		const std::string file = std::string(dir) + "/" + name + ".png";
		r |= cairo_surface_write_to_png(surface, file.c_str()) != CAIRO_STATUS_SUCCESS;
	};
	State s;
	s.known = true;
	s.tour = true;
	const struct
	{
		const char *name;
		enum pico4_tour_page page;
		bool left, right;
		int pairing;
	} pages[] = {
	    {"controllers-none", PICO4_TOUR_CONTROLLERS, false, false, 1},
	    {"controllers-one", PICO4_TOUR_CONTROLLERS, true, false, 0},
	    {"controllers-both", PICO4_TOUR_CONTROLLERS, true, true, 0},
	    {"safety", PICO4_TOUR_SAFETY, true, true, 0},
	    {"controls", PICO4_TOUR_CONTROLS, true, true, 0},
	    {"playspace", PICO4_TOUR_PLAYSPACE, true, true, 0},
	    {"done", PICO4_TOUR_DONE, true, true, 0},
	};
	for (const auto &p : pages) {
		s.page = p.page;
		s.left = p.left;
		s.right = p.right;
		s.pairing = p.pairing;
		save(s, p.name);
	}
	// The setup's pages, with made-up networks and a lens distance.
	State u;
	u.known = true;
	u.left = true;
	u.setup.active = true;
	u.setup.language = std::max(0, pico4_setup_language_index(t.language.c_str()));
	u.setup.zone = pico4_setup_zone_index("Europe/Paris");
	u.setup.scanned = true;
	const char *nets = "*:Home:82:WPA2\n :Neighbours:61:WPA2\n :Cafe:35:\n :Office 5G:12:WPA2 WPA3\n";
	pico4_setup_network found[8];
	u.setup.nets.assign(found, found + pico4_setup_parse_networks(nets, found, 8));
	u.setup.ipd = 6350;
	const struct
	{
		const char *name;
		enum pico4_setup_page page;
		int list_page;
	} setup_pages[] = {
	    {"setup-welcome", PICO4_SETUP_WELCOME, 0},   {"setup-language", PICO4_SETUP_LANGUAGE, 0},
	    {"setup-language-2", PICO4_SETUP_LANGUAGE, 1}, {"setup-timezone", PICO4_SETUP_TIMEZONE, 3},
	    {"setup-wifi", PICO4_SETUP_WIFI, 0},          {"setup-controllers", PICO4_SETUP_CONTROLLERS, 0},
	    {"setup-ipd", PICO4_SETUP_IPD, 0},            {"setup-ready", PICO4_SETUP_READY, 0},
	};
	for (const auto &p : setup_pages) {
		u.setup.page = p.page;
		u.setup.list_page = p.list_page;
		u.setup.message = p.page == PICO4_SETUP_LANGUAGE ? t.our("setup_language_note") : "";
		save(u, p.name);
	}
	// The Wi-Fi page's steps past its list.
	Setup &w = u.setup;
	w.page = PICO4_SETUP_WIFI;
	w.wifi_message = t.steam_text("Settings_Internet_Connection_Successful", "Connection successful!");
	save(u, "setup-wifi-joined-list");
	w.wifi_message.clear();
	w.ssid = "Neighbours";
	w.secured = true;
	w.wifi = PICO4_WIFI_PASSWORD;
	w.keyboard = true;
	save(u, "setup-wifi-password-empty");
	snprintf(w.entry, sizeof(w.entry), "correct horse");
	save(u, "setup-wifi-password");
	w.reveal = true;
	save(u, "setup-wifi-password-shown");
	w.reveal = false;
	w.retry = true;
	w.keyboard = false;
	snprintf(w.entry, sizeof(w.entry), "%s", "a long passphrase that no longer fits the field");
	save(u, "setup-wifi-password-again");
	w.retry = false;
	w.hidden = true;
	w.secured = false;
	w.wifi = PICO4_WIFI_NAME;
	snprintf(w.entry, sizeof(w.entry), "Basement");
	save(u, "setup-wifi-name");
	w.hidden = false;
	w.entry[0] = '\0';
	w.wifi = PICO4_WIFI_JOINING;
	w.step_at = 0;
	draw(surface, u, t, 0.2);
	r |= cairo_surface_write_to_png(surface, (std::string(dir) + "/setup-wifi-joining.png").c_str()) != CAIRO_STATUS_SUCCESS;
	w.wifi = PICO4_WIFI_JOINED;
	save(u, "setup-wifi-joined");
	w.wifi = PICO4_WIFI_FAILED;
	w.join = PICO4_JOIN_PASSWORD;
	save(u, "setup-wifi-failed-password");
	w.join = PICO4_JOIN_NOT_FOUND;
	save(u, "setup-wifi-failed-missing");
	w.join = PICO4_JOIN_FAILED;
	save(u, "setup-wifi-failed");
	w.wifi = PICO4_WIFI_LIST;
	w.wifi_on = false;
	save(u, "setup-wifi-off");
	draw_start(surface, t, false);
	r |= cairo_surface_write_to_png(surface, (std::string(dir) + "/start.png").c_str()) != CAIRO_STATUS_SUCCESS;
	cairo_surface_destroy(surface);
	return r;
}

//! Seconds from the system UI's overlay to its dashboard mailbox taking messages (2 s measured).
constexpr double kSystemUiSettle = 5.0;

//! Seconds for the keyboard's page to load once started, and for its layout once shown (under 1 s measured).
constexpr double kKeyboardLoad = 2.0;

/*
 * SteamVR's switches for the setup, which its scene application may not throw itself: through
 * pico4-steamvr-switch, a short background application, waited for (at most 15 s).
 */
void
steamvr_switch(const char *what, bool on)
{
	Job job;
	if (!job.start({"pico4-steamvr-switch", what, on ? "on" : "off"})) {
		fprintf(stderr, "pico4-onboarding: pico4-steamvr-switch did not start\n");
		return;
	}
	for (int i = 0; i < 1500 && !job.poll(); i++) {
		usleep(10000);
	}
	if (job.running()) {
		kill(job.pid, SIGKILL);
		while (!job.poll()) {
			usleep(10000);
		}
	}
	printf("pico4-onboarding: %s", job.output.c_str());
	fflush(stdout);
}

/*
 * The setup's own image for the scene: an even dark room, which the room view covers. As a scene
 * application the setup keeps SteamVR's dashboard and its home away while it runs.
 */
struct Scene
{
	Texture eye;
	bool ready = false;

	bool create(const Gpu &gpu)
	{
		if (!texture_create(gpu, eye, 16, 16)) {
			return false;
		}
		cairo_surface_t *surface = cairo_image_surface_create(CAIRO_FORMAT_ARGB32, 16, 16);
		cairo_t *c = cairo_create(surface);
		cairo_set_source_rgb(c, 0.07, 0.08, 0.10);
		cairo_paint(c);
		cairo_destroy(c);
		ready = texture_upload(gpu, eye, surface);
		cairo_surface_destroy(surface);
		return ready;
	}

	//! One frame: wait for SteamVR's frame timing, then the image to both eyes.
	void frame(const Gpu &gpu, vr::IVRCompositor *compositor)
	{
		vr::TrackedDevicePose_t poses[vr::k_unMaxTrackedDeviceCount];
		compositor->WaitGetPoses(poses, vr::k_unMaxTrackedDeviceCount, nullptr, 0);
		vr::VRVulkanTextureData_t data = vulkan_data(gpu, eye);
		vr::Texture_t texture{&data, vr::TextureType_Vulkan, vr::ColorSpace_Auto};
		compositor->Submit(vr::Eye_Left, &texture);
		compositor->Submit(vr::Eye_Right, &texture);
	}
};

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

	// The setup runs as SteamVR's scene application, the introduction as a background one.
	const std::string start_registry = registry();
	const bool scene = pico4_setup_starts(pico4_tour_oobe_done(start_registry.c_str()), mark_exists("setup-done"));
	Vr vrs;
	const int opened =
	    vr_open("pico4-onboarding", vrs, &g_stop, scene ? vr::VRApplication_Scene : vr::VRApplication_Background);
	if (opened != 0) {
		return opened == 2 ? 0 : 1;
	}
	vr::IVROverlay *overlay = vrs.overlay;
	vr::IVRSystem *system = vrs.system;

	Texts texts;
	{
		char lang[32];
		const std::string vdf = registry();
		texts.load(pico4_setup_registry_language(vdf.c_str(), lang, sizeof(lang)) ? lang : "english");
	}
	vr::VROverlayHandle_t front, tab, thumbnail;
	if (overlay->CreateOverlay("pico4.onboarding", "PICO 4 introduction", &front) != vr::VROverlayError_None ||
	    overlay->CreateDashboardOverlay("pico4.onboarding.tab", texts.our("start_title").c_str(), &tab, &thumbnail) !=
	        vr::VROverlayError_None) {
		fprintf(stderr, "pico4-onboarding: no overlays\n");
		return 1;
	}
	vr::HmdVector2_t scale = {{(float)PICO4_TOUR_W, (float)PICO4_TOUR_H}};
	for (vr::VROverlayHandle_t h : {front, tab}) {
		overlay->SetOverlayInputMethod(h, vr::VROverlayInputMethod_Mouse);
		overlay->SetOverlayMouseScale(h, &scale);
	}
	overlay->SetOverlayWidthInMeters(front, 1.2f);
	overlay->SetOverlayWidthInMeters(tab, 2.0f);
	// The laser reaches the panel outside the dashboard too, and it is drawn over Steam's window.
	overlay->SetOverlayFlag(front, vr::VROverlayFlags_MakeOverlaysInteractiveIfVisible, true);
	overlay->SetOverlaySortOrder(front, 100);

	cairo_surface_t *surface = cairo_image_surface_create(CAIRO_FORMAT_ARGB32, PICO4_TOUR_W, PICO4_TOUR_H);
	Gpu gpu;
	Texture front_texture, tab_texture, thumbnail_texture;
	if (!gpu_create(gpu, vrs.compositor, "pico4-onboarding") || !texture_create(gpu, front_texture, PICO4_TOUR_W, PICO4_TOUR_H) ||
	    !texture_create(gpu, tab_texture, PICO4_TOUR_W, PICO4_TOUR_H) || !texture_create(gpu, thumbnail_texture, 256, 256)) {
		fprintf(stderr, "pico4-onboarding: no Vulkan textures\n");
		return 1;
	}
	{
		cairo_surface_t *small = cairo_image_surface_create(CAIRO_FORMAT_ARGB32, 256, 256);
		draw_thumbnail(small);
		texture_show(gpu, thumbnail_texture, small, overlay, thumbnail);
		cairo_surface_destroy(small);
		draw_start(surface, texts, false);
		texture_show(gpu, tab_texture, surface, overlay, tab);
	}
	Scene room;
	if (scene) {
		vrs.compositor->SetTrackingSpace(vr::TrackingUniverseStanding);
		if (!room.create(gpu)) {
			fprintf(stderr, "pico4-onboarding: no image for the scene\n");
		}
	}
	printf("pico4-onboarding: ready (%s)\n", scene ? "the setup, as SteamVR's scene" : "background");
	fflush(stdout);
	fflush(stdout);

	State s;
	Setup &u = s.setup;
	Placement placement;
	double away_since = 0; //!< when the panel left the view, 0 while in it
	Request status, pair;
	const double started = now_s();
	double next_status = 0, next_oobe = 0, next_texts = 0, next_view_check = 0, next_frame_draw = 0;
	double systemui_since = 0; //!< when SteamVR's system UI was first seen, 0 before
	double keyboard_warm_at = 0, keyboard_hide_at = 0; //!< the keyboard's first showing, 0 when done
	bool view_pending = false; //!< the setup runs; its room view and closed dashboard still to come
	bool oobe = false, mark = mark_exists("onboarding-done"), setup_mark = mark_exists("setup-done");
	bool dirty = true, running = true, tab_pressed = false, again = false;
	// The client's texts follow its first start (it unpacks itself), and the language a page set.
	std::string texts_language = texts.language;
	bool texts_steam = texts.steam_found;

	auto show = [&](bool tour) {
		s.tour = tour;
		s.page = PICO4_TOUR_CONTROLLERS;
		s.message.clear();
		if (!s.shown) {
			place_in_front(overlay, system, front, placement);
			away_since = 0;
			overlay->ShowOverlay(front);
			s.shown = true;
			printf("pico4-onboarding: shown (%s)\n", tour ? "introduction" : "controllers or setup");
			fflush(stdout);
		}
		dirty = true;
	};
	auto hide = [&](bool finished) {
		printf("pico4-onboarding: %s %s\n", finished ? "finished" : "closed", u.active ? "setup" : "introduction");
		fflush(stdout);
		if (u.active) {
			if (finished) {
				// SteamVR as it was, then the client: the mark lets it start.
				if (!view_pending) {
					steamvr_switch("roomview", false);
				}
				view_pending = false;
				steamvr_switch("expect-steam", true);
				set_mark("setup-done");
				// The introduction comes from a background application: a new start of this one.
				again = scene;
				running = !scene;
				setup_mark = true;
			}
			u.active = false;
		} else if (finished && s.tour) {
			set_mark("onboarding-done");
			mark = true;
		}
		if (s.in_tab) {
			s.in_tab = false;
			draw_start(surface, texts, false);
			texture_show(gpu, tab_texture, surface, overlay, tab);
		} else {
			overlay->HideOverlay(front);
			s.shown = false;
		}
		s.tour = false;
	};
	auto start_pairing = [&](int hand) {
		char text[32];
		snprintf(text, sizeof(text), "pair %s %d", hand == 1 ? "left" : "right", PICO4_TOUR_PAIR_S);
		if (pair.start(text)) {
			s.pairing = hand;
			s.message.clear();
		} else {
			s.message = texts.our("controller_radio");
		}
		dirty = true;
	};
	auto scan = [&](bool rescan) {
		if (!u.scan_job.running()) {
			u.scan_job.start({"nmcli", "-t", "-f", "IN-USE,SSID,SIGNAL,SECURITY", "device", "wifi", "list", "--rescan",
			                  rescan ? "yes" : "auto"});
		}
		u.next_scan = now_s() + 10;
	};
	/*
	 * SteamVR's keyboard for the field, in its minimal mode: it sends each key to the panel
	 * (VREvent_KeyboardCharInput) and keeps no text of its own, so the field on the panel is the
	 * only one, and what it shows is what is sent.
	 */
	auto keyboard = [&](bool open) {
		if (!open) {
			if (u.keyboard) {
				overlay->HideKeyboard();
				printf("pico4-onboarding: keyboard closed, %d characters typed\n", u.typed);
				fflush(stdout);
			}
			u.keyboard = false;
			return;
		}
		const bool password = u.wifi == PICO4_WIFI_PASSWORD;
		const std::string label = password ? texts.steam_text("Settings_Internet_Enter_Password", "Password")
		                                   : texts.steam_text("Settings_Internet_Enter_NetworkName", "Network Name");
		const vr::EVROverlayError e = overlay->ShowKeyboardForOverlay(
		    s.in_tab ? tab : front, password ? vr::k_EGamepadTextInputModePassword : vr::k_EGamepadTextInputModeNormal,
		    vr::k_EGamepadTextInputLineModeSingleLine, vr::KeyboardFlag_Minimal | vr::KeyboardFlag_Modal, label.c_str(),
		    PICO4_SETUP_TEXT - 1, "", 0);
		u.keyboard = e == vr::VROverlayError_None;
		u.typed = 0;
		if (!u.keyboard) {
			fprintf(stderr, "pico4-onboarding: no keyboard: %d\n", e);
		}
	};
	//! A Wi-Fi step; the typed text goes when the field's step is left.
	auto wifi_step = [&](enum pico4_setup_wifi_step step) {
		const bool typing = step == PICO4_WIFI_NAME || step == PICO4_WIFI_PASSWORD;
		if (!typing || step != u.wifi) {
			keyboard(false);
		}
		if (!typing && step != PICO4_WIFI_JOINING) {
			memset(u.entry, 0, sizeof(u.entry));
		}
		if (typing && step != u.wifi) {
			memset(u.entry, 0, sizeof(u.entry));
			u.reveal = false;
		}
		u.wifi = step;
		u.step_at = now_s();
		u.pressed_field = 0;
		dirty = true;
		if (typing) {
			keyboard(true);
		}
	};
	//! Join u.ssid: with the password on nmcli's input only, never on a command line.
	auto connect_network = [&]() {
		std::vector<std::string> argv = {"nmcli", "--wait", "45"};
		const bool password = u.entry[0] != '\0';
		if (password) {
			argv.push_back("--ask");
		}
		argv.insert(argv.end(), {"device", "wifi", "connect", u.ssid});
		if (u.hidden) {
			argv.insert(argv.end(), {"hidden", "yes"});
		}
		const std::string input = password ? std::string(u.entry) + "\n" : std::string();
		if (u.connect_job.start(argv, input)) {
			wifi_step(PICO4_WIFI_JOINING);
		}
		memset(u.entry, 0, sizeof(u.entry));
		dirty = true;
	};
	//! The input page's text sent: the hidden network's name leads to its password.
	auto submit_entry = [&]() {
		if (!entry_ok(u)) {
			return;
		}
		if (u.wifi == PICO4_WIFI_NAME) {
			u.ssid = u.entry;
			u.retry = false;
			wifi_step(PICO4_WIFI_PASSWORD);
		} else {
			keyboard(false);
			connect_network();
		}
	};
	auto enter = [&](enum pico4_setup_page page) {
		keyboard(false);
		memset(u.entry, 0, sizeof(u.entry));
		u.wifi = PICO4_WIFI_LIST;
		u.page = page;
		u.pressed_row = -1;
		u.message.clear();
		u.message_error = false;
		s.message.clear();
		switch (page) {
		case PICO4_SETUP_LANGUAGE: u.list_page = pico4_setup_list_page(u.language); break;
		case PICO4_SETUP_TIMEZONE:
			u.zone = pico4_setup_zone_index(system_zone().c_str());
			u.list_page = pico4_setup_list_page(u.zone);
			break;
		case PICO4_SETUP_WIFI:
			u.list_page = 0;
			u.wifi_message.clear();
			if (!u.radio_job.running()) {
				u.radio_job.start({"nmcli", "-t", "radio", "wifi"});
			}
			scan(false);
			break;
		case PICO4_SETUP_IPD:
			u.ipd_target = -1;
			if (!u.ipd_job.running()) {
				u.ipd_job.start({"pico4-lens"});
			}
			break;
		default: break;
		}
		dirty = true;
	};
	auto press_setup = [&](int button) {
		if (u.page == PICO4_SETUP_WIFI && u.wifi != PICO4_WIFI_LIST) {
			// Past the list: Back to it, the main button sends the field or tries again.
			if (u.wifi == PICO4_WIFI_JOINING || u.wifi == PICO4_WIFI_JOINED) {
				return;
			}
			if (button == PICO4_TOUR_SKIP) {
				wifi_step(PICO4_WIFI_LIST);
				scan(false);
			} else if (button == PICO4_TOUR_NEXT && u.wifi == PICO4_WIFI_FAILED) {
				u.retry = u.join == PICO4_JOIN_PASSWORD;
				if (u.secured || u.hidden) {
					wifi_step(PICO4_WIFI_PASSWORD);
				} else {
					connect_network();
				}
			} else if (button == PICO4_TOUR_NEXT) {
				submit_entry();
			}
			dirty = true;
			return;
		}
		if (button == PICO4_TOUR_SKIP) {
			if (u.page == PICO4_SETUP_WELCOME) {
				hide(true);
			} else {
				enter(pico4_setup_back(u.page));
			}
		} else if (button == PICO4_TOUR_NEXT) {
			const enum pico4_setup_page next = pico4_setup_next(u.page);
			if (next == PICO4_SETUP_PAGES) {
				hide(true);
			} else {
				enter(next);
			}
		} else if (button == PICO4_TOUR_PAIR) {
			if (u.page == PICO4_SETUP_WIFI && !u.wifi_on) {
				u.radio_job.start({"nmcli", "radio", "wifi", "on"});
			} else if (u.page == PICO4_SETUP_WIFI) {
				scan(true);
			} else if (u.page == PICO4_SETUP_CONTROLLERS) {
				s.hands_free = false;
				start_pairing(pico4_tour_pair_button(s.left, s.right));
			}
		}
		dirty = true;
	};
	auto choose_row = [&](int r) {
		const int i = u.list_page * PICO4_SETUP_ROWS + r;
		if (i >= list_count(u)) {
			return;
		}
		if (u.page == PICO4_SETUP_LANGUAGE && !u.language_job.running()) {
			u.language = i;
			const char *lang = pico4_setup_languages[i].steam;
			if (texts.language != lang) {
				u.language_job.start({"/usr/libexec/pico4/steam-language", lang});
				texts.load(lang);
				texts_language = texts.language;
				overlay->SetOverlayName(tab, texts.our("start_title").c_str());
				u.message = texts.our("setup_language_note");
			}
		} else if (u.page == PICO4_SETUP_TIMEZONE && !u.zone_job.running()) {
			u.zone = i;
			u.zone_job.start({"/usr/bin/steamos-polkit-helpers/steamos-set-timezone", pico4_setup_zones[i].zone});
		} else if (u.page == PICO4_SETUP_WIFI && !u.connect_job.running()) {
			u.retry = false;
			u.wifi_message.clear();
			if (i == (int)u.nets.size()) {
				u.hidden = true;
				u.secured = false; // unknown until it answers: the password may stay empty
				u.ssid.clear();
				wifi_step(PICO4_WIFI_NAME);
				return;
			}
			const pico4_setup_network &n = u.nets[(size_t)i];
			if (n.active) {
				return;
			}
			u.hidden = false;
			u.secured = n.secured;
			u.ssid = n.ssid;
			if (n.secured) {
				wifi_step(PICO4_WIFI_PASSWORD);
			} else {
				connect_network();
			}
		}
		dirty = true;
	};
	auto lens = [&](int steps) {
		if (u.ipd < 0) {
			return;
		}
		u.ipd_target = pico4_setup_ipd_step(u.ipd_target >= 0 ? u.ipd_target : u.ipd, steps);
		dirty = true;
	};

	while (running && !g_stop) {
		const double now = now_s();
		vr::VREvent_t e;
		for (vr::VROverlayHandle_t h : {front, tab}) {
			while (overlay->PollNextOverlayEvent(h, &e, sizeof(e))) {
				const bool typing = u.active && (u.wifi == PICO4_WIFI_NAME || u.wifi == PICO4_WIFI_PASSWORD);
				if (e.eventType == vr::VREvent_KeyboardCharInput) {
					char input[sizeof(e.data.keyboard.cNewInput) + 1] = {0};
					memcpy(input, e.data.keyboard.cNewInput, sizeof(e.data.keyboard.cNewInput));
					if (typing && pico4_setup_text_input(u.entry, sizeof(u.entry), input)) {
						submit_entry();
					}
					u.typed++;
					memset(input, 0, sizeof(input));
					memset(e.data.keyboard.cNewInput, 0, sizeof(e.data.keyboard.cNewInput));
					dirty = true;
					continue;
				}
				if (e.eventType == vr::VREvent_KeyboardDone) {
					if (typing && entry_ok(u)) {
						submit_entry();
					} else {
						keyboard(false);
					}
					dirty = true;
					continue;
				}
				if (e.eventType == vr::VREvent_KeyboardClosed) {
					if (u.keyboard) {
						printf("pico4-onboarding: keyboard closed, %d characters typed\n", u.typed);
						fflush(stdout);
					}
					u.keyboard = false;
					dirty = true;
					continue;
				}
				const double x = e.data.mouse.x, y = PICO4_TOUR_H - e.data.mouse.y; // SteamVR's y points up
				const bool middle_shown =
				    u.active ? (u.page == PICO4_SETUP_WIFI && u.wifi == PICO4_WIFI_LIST) ||
				                   (u.page == PICO4_SETUP_CONTROLLERS && s.pairing == 0 && pico4_tour_pair_button(s.left, s.right) != 0)
				             : s.page == PICO4_TOUR_CONTROLLERS && s.pairing == 0 && pico4_tour_pair_button(s.left, s.right) != 0;
				// The tab's own start page; once started there, the pages run in the tab (OpenVR has no
				// call that closes the dashboard, which would cover the panel in front).
				if (h == tab && !s.in_tab) {
					const bool on = x >= PICO4_TOUR_NEXT_X - 330 && x < PICO4_TOUR_NEXT_X - 330 + PICO4_TOUR_BUTTON_W &&
					                y >= PICO4_TOUR_BUTTON_Y && y < PICO4_TOUR_BUTTON_Y + PICO4_TOUR_BUTTON_H;
					if (e.eventType == vr::VREvent_MouseButtonDown && on) {
						tab_pressed = true;
					} else if (e.eventType == vr::VREvent_MouseButtonUp && tab_pressed) {
						tab_pressed = false;
						if (on && !s.shown) {
							s.in_tab = true;
							s.tour = true;
							s.page = PICO4_TOUR_CONTROLLERS;
							s.message.clear();
							dirty = true;
						}
					}
					continue;
				}
				if ((h == tab) != s.in_tab) {
					continue;
				}
				const bool lists = u.active && list_count(u) > 0;
				if (e.eventType == vr::VREvent_MouseButtonDown) {
					s.pressed = pico4_tour_hit(x, y, middle_shown);
					if (u.active) {
						u.pressed_row = lists ? pico4_setup_hit_row(x, y) : -1;
						u.pressed_arrow = lists ? pico4_setup_hit_arrow(x, y) : 0;
						u.pressed_ipd = u.page == PICO4_SETUP_IPD ? pico4_setup_hit_ipd(x, y) : 0;
						u.pressed_field = typing ? pico4_setup_hit_field(x, y) : 0;
						if (u.pressed_field == 2 && u.wifi != PICO4_WIFI_PASSWORD) {
							u.pressed_field = 0; // a name has no switch to show it
						}
					}
					dirty = true;
				} else if (e.eventType == vr::VREvent_MouseButtonUp) {
					const int was = s.pressed, was_row = u.pressed_row, was_arrow = u.pressed_arrow, was_ipd = u.pressed_ipd;
					const int was_field = u.pressed_field;
					s.pressed = PICO4_TOUR_NONE;
					u.pressed_row = -1;
					u.pressed_arrow = 0;
					u.pressed_ipd = 0;
					u.pressed_field = 0;
					dirty = true;
					if (u.active) {
						if (was_field != 0 && typing && was_field == pico4_setup_hit_field(x, y)) {
							if (was_field == 1 && !u.keyboard) {
								keyboard(true);
							} else if (was_field == 2) {
								u.reveal = !u.reveal;
							}
						} else if (was != PICO4_TOUR_NONE && was == pico4_tour_hit(x, y, middle_shown)) {
							press_setup(was);
						} else if (was_row >= 0 && lists && was_row == pico4_setup_hit_row(x, y)) {
							choose_row(was_row);
						} else if (was_arrow != 0 && lists && was_arrow == pico4_setup_hit_arrow(x, y)) {
							u.list_page = pico4_setup_list_turn(u.list_page, was_arrow, list_count(u));
						} else if (was_ipd != 0 && was_ipd == pico4_setup_hit_ipd(x, y)) {
							lens(was_ipd);
						}
						continue;
					}
					if (was != pico4_tour_hit(x, y, middle_shown)) {
						continue;
					}
					if (was == PICO4_TOUR_SKIP) {
						hide(true);
					} else if (was == PICO4_TOUR_PAIR) {
						s.hands_free = false;
						start_pairing(pico4_tour_pair_button(s.left, s.right));
					} else if (was == PICO4_TOUR_NEXT) {
						const enum pico4_tour_page next = pico4_tour_next(s.page);
						if (!s.tour || next == PICO4_TOUR_PAGES) {
							hide(true);
						} else {
							s.page = next;
						}
					}
				}
			}
		}
		while (system->PollNextEvent(&e, sizeof(e))) {
			if (e.eventType == vr::VREvent_Quit) {
				system->AcknowledgeQuit_Exiting();
				running = false;
			} else if (e.eventType == vr::VREvent_TrackedDeviceUserInteractionStarted &&
			           e.trackedDeviceIndex == vr::k_unTrackedDeviceIndex_Hmd && s.shown) {
				// Put on (the wear sensor): the panel came up wherever the headset lay; in front again.
				place_in_front(overlay, system, front, placement);
				printf("pico4-onboarding: put on, the panel in front\n");
				fflush(stdout);
			}
		}

		// The setup's jobs.
		if (u.language_job.poll() && u.language_job.status != 0) {
			fprintf(stderr, "pico4-onboarding: steam-language: %s\n", u.language_job.output.c_str());
		}
		if (u.zone_job.poll() && u.zone_job.status != 0) {
			fprintf(stderr, "pico4-onboarding: steamos-set-timezone: %s\n", u.zone_job.output.c_str());
			u.message = texts.our("setup_timezone_failed");
			u.message_error = true;
			dirty = true;
		}
		if (u.radio_job.poll()) {
			const bool was = u.wifi_on;
			u.wifi_on = u.radio_job.output.rfind("disabled", 0) != 0;
			if (u.wifi_on && !was) {
				scan(true);
			}
			dirty = true;
		}
		if (u.scan_job.poll()) {
			if (u.scan_job.status == 0) {
				pico4_setup_network found[32];
				u.nets.assign(found, found + pico4_setup_parse_networks(u.scan_job.output.c_str(), found, 32));
				u.list_page = pico4_setup_list_turn(u.list_page, 0, list_count(u));
			}
			u.scanned = true;
			dirty = true;
		}
		if (u.connect_job.poll()) {
			u.join = pico4_setup_join_result(u.connect_job.status, u.connect_job.output.c_str());
			if (u.join == PICO4_JOIN_OK) {
				wifi_step(PICO4_WIFI_JOINED);
			} else {
				const std::string &out = u.connect_job.output;
				fprintf(stderr, "pico4-onboarding: nmcli connect %s failed (%d): %s\n", u.ssid.c_str(), u.connect_job.status,
				        out.substr(0, out.find('\n')).c_str());
				wifi_step(PICO4_WIFI_FAILED);
				if (u.join == PICO4_JOIN_PASSWORD && !u.forget_job.running()) {
					// nmcli kept the refused password in a new profile, which the next attempt would
					// try first: gone, the next password starts afresh.
					u.forget_job.start({"nmcli", "connection", "delete", "id", u.ssid});
				}
			}
			scan(false);
			dirty = true;
		}
		u.forget_job.poll();
		if (u.wifi == PICO4_WIFI_JOINED && now - u.step_at >= 1.5) {
			// Joined: the list again, the network marked as in use.
			u.wifi_message = texts.steam_text("Settings_Internet_Connection_Successful", "Connection successful!");
			wifi_step(PICO4_WIFI_LIST);
		}
		if (u.active && (u.wifi == PICO4_WIFI_JOINING || u.keyboard) && now >= next_frame_draw) {
			// The spinner's and the caret's motion.
			next_frame_draw = now + 0.05;
			dirty = true;
		}
		if (u.active && u.page == PICO4_SETUP_WIFI && u.wifi_on && now >= u.next_scan && !u.connect_job.running()) {
			scan(false);
		}
		if (u.ipd_job.poll()) {
			const int ipd = pico4_setup_parse_ipd(u.ipd_job.output.c_str());
			if (ipd >= 0) {
				u.ipd = ipd;
			}
			if (u.ipd_target == u.ipd || (u.ipd_target >= 0 && ipd < 0)) {
				u.ipd_target = -1;
			}
			dirty = true;
		}
		if (u.ipd_target >= 0 && u.ipd >= 0 && u.ipd_target != u.ipd && !u.ipd_job.running()) {
			char mm[16];
			snprintf(mm, sizeof(mm), "%d.%02d", u.ipd_target / 100, u.ipd_target % 100);
			u.ipd_job.start({"pico4-lens", "set", mm});
			u.ipd = u.ipd_target; // shown at once; the answer corrects it
		}

		// The controllers' state from the driver, while no pairing holds its socket.
		if (!pair.active() && !status.active() && now >= next_status) {
			next_status = now + 2;
			status.start("status");
		}
		if (status.poll() && status.done) {
			bool left, right;
			if (!status.answers.empty() && pico4_tour_parse_status(status.answers[0].c_str(), &left, &right)) {
				dirty = dirty || !s.known || left != s.left || right != s.right;
				s.known = true;
				s.left = left;
				s.right = right;
			}
			status.answers.clear();
		}
		if (pair.poll() && pair.done) {
			const int result = pair.answers.empty() ? -1 : pico4_tour_pair_answer(pair.answers.back().c_str());
			if (result != 1 && !s.hands_free) {
				s.message = texts.our("controller_pair_failed");
			}
			s.pairing = 0;
			next_status = 0; // the new state, now
			dirty = true;
		}
		// Hands-free pairing: nothing points at the panel while no controller is there.
		if (s.known && !s.left && !s.right && s.pairing == 0 && !s.in_tab && now - started >= PICO4_TOUR_CONTROLLER_WAIT_S) {
			if (!s.shown) {
				show(s.tour);
			}
			s.hands_free = true;
			s.last_hands_free = pico4_tour_hands_free(false, false, s.last_hands_free);
			start_pairing(s.last_hands_free);
		}
		if (now >= next_oobe) {
			next_oobe = now + 5;
			const std::string vdf = registry();
			oobe = pico4_tour_oobe_done(vdf.c_str());
		}
		if (now >= next_texts && !texts_steam) {
			// The client's files arrive with its first start.
			next_texts = now + 10;
			texts.load(texts_language);
			texts_steam = texts.steam_found;
			dirty = dirty || texts_steam;
		}
		// The setup, before the first sign-in: as soon as the driver says where the controllers are.
		if (scene && s.known && !s.in_tab && !u.active && pico4_setup_starts(oobe, setup_mark)) {
			if (!s.shown) {
				show(false);
			}
			u.active = true;
			u.language = std::max(0, pico4_setup_language_index(texts.language.c_str()));
			enter(PICO4_SETUP_WELCOME);
			printf("pico4-onboarding: setup started in %s\n", texts.language.c_str());
			fflush(stdout);
			// SteamVR waits for nobody meanwhile; the room view follows once its system UI is up.
			steamvr_switch("expect-steam", false);
			view_pending = true;
		}
		// SteamVR's system UI sets the room view as it starts and opens its dashboard: the wearer's
		// view is switched only after it, once its mailboxes are open (a few seconds on).
		if (u.active && view_pending && now >= next_view_check) {
			next_view_check = now + 0.5;
			vr::VROverlayHandle_t systemui = vr::k_ulOverlayHandleInvalid;
			if (systemui_since == 0 && overlay->FindOverlay("system.systemui", &systemui) == vr::VROverlayError_None) {
				systemui_since = now;
			}
			if (systemui_since != 0 && now - systemui_since >= kSystemUiSettle) {
				steamvr_switch("dashboard", false);
				steamvr_switch("roomview", true);
				steamvr_switch("keyboard", true); // ready, its page loaded, before the password page
				view_pending = false;
				keyboard_warm_at = now + kKeyboardLoad;
			}
		}
		// SteamVR's keyboard loads its keys' layout at its first showing and passes on no key during
		// that one: it is shown once, early (the headset is seldom worn yet), so the field gets the
		// second showing on.
		if (keyboard_warm_at != 0 && now >= keyboard_warm_at) {
			keyboard_warm_at = 0;
			if (!u.keyboard && overlay->ShowKeyboardForOverlay(front, vr::k_EGamepadTextInputModeNormal,
			                                                   vr::k_EGamepadTextInputLineModeSingleLine,
			                                                   vr::KeyboardFlag_Minimal, "", 1, "", 0) == vr::VROverlayError_None) {
				keyboard_hide_at = now + kKeyboardLoad;
			}
		}
		if (keyboard_hide_at != 0 && now >= keyboard_hide_at) {
			keyboard_hide_at = 0;
			if (!u.keyboard) {
				overlay->HideKeyboard();
				printf("pico4-onboarding: keyboard shown once, its layout loaded\n");
				fflush(stdout);
			}
		}
		if (u.active && oobe) {
			// Signed in meanwhile (from Steam's window): the setup is over, the introduction follows.
			hide(true);
		}
		if (s.known && (s.left || s.right) && !s.in_tab && !u.active && pico4_tour_starts(oobe, mark)) {
			if (!s.shown) {
				show(true);
			} else if (!s.tour) {
				s.tour = true; // the controllers came hands-free: the tour goes on from here
				dirty = true;
			}
		}
		// The panel follows the view as Steam's window does, a second after it left it.
		if (s.shown && out_of_view(system, placement)) {
			if (away_since == 0) {
				away_since = now;
			} else if (now - away_since >= 1.0) {
				place_in_front(overlay, system, front, placement);
				away_since = 0;
			}
		} else {
			away_since = 0;
		}
		if (dirty && (s.shown || s.in_tab)) {
			draw(surface, s, texts, now);
			if (!texture_show(gpu, s.in_tab ? tab_texture : front_texture, surface, overlay, s.in_tab ? tab : front)) {
				fprintf(stderr, "pico4-onboarding: image not accepted\n");
			}
			dirty = false;
		}
		if (scene && room.ready) {
			room.frame(gpu, vrs.compositor);
		} else {
			usleep(s.shown || s.in_tab ? 16000 : 200000);
		}
	}
	pair.finish();
	status.finish();
	cairo_surface_destroy(surface);
	overlay->DestroyOverlay(front);
	overlay->DestroyOverlay(tab);
	vrs.shutdown();
	if (again && !g_stop) {
		printf("pico4-onboarding: the setup is done; again as a background application\n");
		fflush(stdout);
		execv("/proc/self/exe", argv);
		perror("pico4-onboarding: execv");
		return 1;
	}
	return 0;
}
