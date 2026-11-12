// Copyright 2026, overpressure contributors.
// SPDX-License-Identifier: BSL-1.0
/*
 * pico4-steamvr-switch: SteamVR's switches for the first-run setup (docs/session.md#first-run-setup),
 * from a background application, since its scene application may throw none of them:
 *
 *   pico4-steamvr-switch roomview on|off       the room view, as the dashboard's camera button
 *                                              switches it (the compositor's system layer, through
 *                                              vrclient's mailbox client)
 *   pico4-steamvr-switch dashboard off         closes the dashboard, as its system button does (the
 *                                              system UI's dashboard mailbox)
 *   pico4-steamvr-switch keyboard on           starts SteamVR's keyboard, as the open dashboard does
 *   pico4-steamvr-switch expect-steam on|off   /settings/dashboard/expectSteam: SteamVR's loading
 *                                              spinner and, after 120 s, its error while Steam is not up
 */
#include "pico4_overlay.hpp"

#include <cstring>

namespace {

using namespace pico4_overlay;

volatile sig_atomic_t g_stop;

/*
 * vrclient's mailbox client (IVRMailbox_002), as SteamVR's own processes use it: open a mailbox,
 * close it, send a JSON message to another one. open takes a third string, which vrclient copies
 * when it is set; nullptr opens the mailbox (a stray pointer there crashed it in strncpy).
 */
struct Mailbox
{
	virtual uint32_t open(const char *name, uint64_t *handle, const char *third) = 0;
	virtual uint32_t close(uint64_t handle) = 0;
	virtual uint32_t send(uint64_t handle, const char *to, const char *json) = 0;
};

//! One message to one of SteamVR's mailboxes; 0 when it left.
int
send(const Vr &vrs, const char *to, const char *json)
{
	vr::EVRInitError err = vr::VRInitError_None;
	auto *mailbox = static_cast<Mailbox *>(vrs.get_interface("IVRMailbox_002", &err));
	if (mailbox == nullptr) {
		fprintf(stderr, "pico4-steamvr-switch: no mailbox client: %d\n", err);
		return 1;
	}
	uint64_t handle = 0;
	const uint32_t opened = mailbox->open("pico4switch", &handle, nullptr);
	if (opened != 0) {
		fprintf(stderr, "pico4-steamvr-switch: mailbox not opened: %u\n", opened);
	}
	const uint32_t sent = mailbox->send(handle, to, json);
	if (handle != 0) {
		mailbox->close(handle);
	}
	return sent == 0 ? 0 : 1;
}

int
room_view(const Vr &vrs, bool on)
{
	const int r = send(vrs, "vrcompositor_systemlayer",
	                   on ? "{\"type\":\"set_camera_roomview_enabled\",\"enabled\":true}"
	                      : "{\"type\":\"set_camera_roomview_enabled\",\"enabled\":false}");
	printf("pico4-steamvr-switch: room view %s%s\n", on ? "on" : "off", r == 0 ? "" : " (not sent)");
	return r;
}

int
close_dashboard(const Vr &vrs)
{
	const int r = send(vrs, "vrwebui_dashboard", "{\"type\":\"hide_dashboard_requested\",\"reason\":\"pico4_setup\"}");
	printf("pico4-steamvr-switch: dashboard closed%s\n", r == 0 ? "" : " (not sent)");
	return r;
}

/*
 * SteamVR's keyboard is a page of its web helper, started on the first message to its mailbox; the
 * system UI sends one while its own dashboard is open, so with the dashboard closed from the start
 * there is no keyboard until this.
 */
int
start_keyboard(const Vr &vrs)
{
	const int r = send(vrs, "keyboard_vrwebui/main", "{\"type\":\"dummy\"}");
	printf("pico4-steamvr-switch: keyboard started%s\n", r == 0 ? "" : " (not sent)");
	return r;
}

int
expect_steam(const Vr &vrs, bool on)
{
	if (vrs.settings == nullptr) {
		fprintf(stderr, "pico4-steamvr-switch: no settings interface\n");
		return 1;
	}
	vr::EVRSettingsError err = vr::VRSettingsError_None;
	vrs.settings->SetBool("dashboard", "expectSteam", on, &err);
	printf("pico4-steamvr-switch: SteamVR %s Steam%s\n", on ? "expects" : "does not wait for",
	       err == vr::VRSettingsError_None ? "" : " (not set)");
	return err == vr::VRSettingsError_None ? 0 : 1;
}

} // namespace

int
main(int argc, char **argv)
{
	const char *what = argc == 3 ? argv[1] : "";
	const bool on = argc == 3 && strcmp(argv[2], "on") == 0;
	const bool off = argc == 3 && strcmp(argv[2], "off") == 0;
	const bool roomview = strcmp(what, "roomview") == 0 && (on || off);
	const bool dashboard = strcmp(what, "dashboard") == 0 && off;
	const bool keyboard = strcmp(what, "keyboard") == 0 && on;
	const bool expect = strcmp(what, "expect-steam") == 0 && (on || off);
	if (!roomview && !dashboard && !keyboard && !expect) {
		fprintf(stderr, "usage: pico4-steamvr-switch roomview|expect-steam on|off, dashboard off, keyboard on\n");
		return 2;
	}
	Vr vrs;
	const int opened = vr_open("pico4-steamvr-switch", vrs, &g_stop);
	if (opened != 0) {
		return 1;
	}
	const int r = roomview    ? room_view(vrs, on)
	              : dashboard ? close_dashboard(vrs)
	              : keyboard  ? start_keyboard(vrs)
	                          : expect_steam(vrs, on);
	fflush(stdout);
	// The message leaves with the connection; a moment for it before letting go.
	usleep(300000);
	vrs.shutdown();
	return r;
}
