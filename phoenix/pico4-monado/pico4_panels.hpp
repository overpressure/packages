// Copyright 2026, overpressure contributors.
// SPDX-License-Identifier: BSL-1.0
/*
 * What the setup (pico4-onboarding) and the settings (overpressure-settings) share past
 * pico4_overlay.hpp: their texts in Steam's language, the driver's pairing socket, the state
 * files in the home directory, and a panel that stands in front of the wearer.
 */
#pragma once

#include "pico4_overlay.hpp"
#include "pico4_setup.h"
#include "pico4_text.hpp"

#include <dirent.h>
#include <errno.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/un.h>

#include <fstream>
#include <sstream>

namespace pico4_panels {

inline std::string
home_path(const char *rest)
{
	const char *home = getenv("HOME");
	return std::string(home != nullptr ? home : "/tmp") + rest;
}

inline std::string
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

//! A mark below ~/.local/share/pico4 (the setup's "setup-done").
inline bool
mark_exists(const char *name)
{
	return access(home_path((std::string("/.local/share/pico4/") + name).c_str()).c_str(), F_OK) == 0;
}

//! The Steam client's registry.vdf: its language and whether its out-of-box experience is done.
inline std::string
registry()
{
	return read_file(home_path("/.steam/registry.vdf"));
}

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
inline std::string
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

//! How far in front of the wearer a panel stands: before Steam's window, which follows the view.
constexpr double kPanelDistance = 1.3;

//! Where a panel stands: its centre, in the standing universe; valid once placed while tracked.
struct Placement
{
	double x = 0, y = 0, z = 0;
	bool valid = false;
};

//! The panel in front of the wearer, level, facing them, where they look now.
inline void
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
inline bool
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

/*!
 * A panel that follows the wearer's view as Steam's window does: placed in front when shown, and
 * again a second after it left the view.
 */
struct Front
{
	Placement at;
	double away_since = 0; //!< when the panel left the view, 0 while in it

	void place(vr::IVROverlay *overlay, vr::IVRSystem *system, vr::VROverlayHandle_t handle)
	{
		place_in_front(overlay, system, handle, at);
		away_since = 0;
	}

	void follow(vr::IVROverlay *overlay, vr::IVRSystem *system, vr::VROverlayHandle_t handle, double now)
	{
		if (!out_of_view(system, at)) {
			away_since = 0;
		} else if (away_since == 0) {
			away_since = now;
		} else if (now - away_since >= 1.0) {
			place(overlay, system, handle);
		}
	}
};

} // namespace pico4_panels
