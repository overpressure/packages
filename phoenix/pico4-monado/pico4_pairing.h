// Copyright 2026, overpressure contributors.
// SPDX-License-Identifier: BSL-1.0
/*
 * The controllers' pairing as the setup (pico4-onboarding.cpp) and the settings
 * (overpressure-settings.cpp) run it through the driver's pairing socket (pico4_station.h), and
 * Steam's sign-in, apart from SteamVR and drawing so that the tests can reach them.
 */
#pragma once

#include <stdbool.h>
#include <string.h>

//! Seconds after the start before missing controllers count as missing (they reconnect by themselves).
#define PICO4_PAIRING_WAIT_S 20.0
//! How long each pairing listens, as `pico4-station pair` by default.
#define PICO4_PAIRING_LISTEN_S 60

/*!
 * Whether Steam's out-of-box experience is done: `CompletedOOBE` set to 1 in the client's
 * `registry.vdf` (`~/.steam/registry.vdf`), as the client keeps it (docs/session.md).
 */
static inline bool
pico4_pairing_oobe_done(const char *vdf)
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
 * The controller to pair hands-free: with no controller connected the laser cannot reach a
 * panel, so the station listens for one hand after the other. 1 left, 2 right, 0 none (a
 * controller is connected). @p last is the hand tried last (0 none yet).
 */
static inline int
pico4_pairing_hands_free(bool left, bool right, int last)
{
	if (left || right) {
		return 0;
	}
	return last == 1 ? 2 : 1;
}

//! The controller a Pair button pairs: the missing one when the other one points; 0 for none.
static inline int
pico4_pairing_missing(bool left, bool right)
{
	return left == right ? 0 : left ? 2 : 1;
}

/*!
 * One answer of the driver's pairing socket for a hand ("left" or "right"): 1 paired, 0 still
 * pairing, -1 failed (refused, no answer, timeout, an error).
 */
static inline int
pico4_pairing_answer(const char *answer)
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
pico4_pairing_parse_status(const char *answer, bool *left, bool *right)
{
	const char *r = strstr(answer, "\nright ");
	if (strncmp(answer, "left ", 5) != 0 || r == NULL) {
		return false;
	}
	*left = strncmp(answer + 5, "connected", 9) == 0;
	*right = strncmp(r + 7, "connected", 9) == 0;
	return true;
}
