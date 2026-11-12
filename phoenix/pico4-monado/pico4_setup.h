// Copyright 2026, overpressure contributors.
// SPDX-License-Identifier: BSL-1.0
/*
 * The first-run setup before the Steam sign-in (pico4-onboarding.cpp, docs/session.md#first-run-setup):
 * its pages, lists and decisions, the tables it offers, and the readers for the Steam client's own
 * texts and nmcli's output, apart from SteamVR and drawing so that the tests can reach them.
 *
 * Steam runs its own setup only on Valve's factory image; on an installed system with no saved
 * account it goes to the sign-in. These pages cover what its setup does, with the client's own
 * words where it has them: the language and time zone it offers, the network, the controllers and
 * the lens distance.
 */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

enum pico4_setup_page
{
	PICO4_SETUP_WELCOME,
	PICO4_SETUP_LANGUAGE,
	PICO4_SETUP_TIMEZONE,
	PICO4_SETUP_WIFI,
	PICO4_SETUP_CONTROLLERS,
	PICO4_SETUP_IPD,
	PICO4_SETUP_READY,
	PICO4_SETUP_PAGES,
};

//! The page after @p page; PICO4_SETUP_PAGES after the last.
static inline enum pico4_setup_page
pico4_setup_next(enum pico4_setup_page page)
{
	return page + 1 < PICO4_SETUP_PAGES ? (enum pico4_setup_page)(page + 1) : PICO4_SETUP_PAGES;
}

//! The page before @p page; the welcome page stays.
static inline enum pico4_setup_page
pico4_setup_back(enum pico4_setup_page page)
{
	return page > PICO4_SETUP_WELCOME ? (enum pico4_setup_page)(page - 1) : PICO4_SETUP_WELCOME;
}

/*!
 * Whether the setup comes up by itself at this start: before anyone signed in to Steam
 * (`CompletedOOBE`, which the client sets at the first sign-in) and until it was finished once (its
 * mark on the data partition). A headset that was set up before this setup existed has signed in.
 */
static inline bool
pico4_setup_starts(bool signed_in, bool done_mark)
{
	return !signed_in && !done_mark;
}

// -- The languages and time zones Steam offers ------------------------------------------------------

//! A language as the Steam client names it (`registry.vdf`'s `language`), with its English name.
struct pico4_setup_language
{
	const char *steam;
	const char *english;
};

#define PICO4_SETUP_LANGUAGES 31

//! The client's 31 interface languages (its localization files), English first as its default.
static const struct pico4_setup_language pico4_setup_languages[PICO4_SETUP_LANGUAGES] = {
    {"english", "English"},
    {"arabic", "Arabic"},
    {"brazilian", "Portuguese (Brazil)"},
    {"bulgarian", "Bulgarian"},
    {"czech", "Czech"},
    {"danish", "Danish"},
    {"dutch", "Dutch"},
    {"finnish", "Finnish"},
    {"french", "French"},
    {"german", "German"},
    {"greek", "Greek"},
    {"hungarian", "Hungarian"},
    {"indonesian", "Indonesian"},
    {"italian", "Italian"},
    {"japanese", "Japanese"},
    {"koreana", "Korean"},
    {"latam", "Spanish (Latin America)"},
    {"malay", "Malay"},
    {"norwegian", "Norwegian"},
    {"polish", "Polish"},
    {"portuguese", "Portuguese (Portugal)"},
    {"romanian", "Romanian"},
    {"russian", "Russian"},
    {"schinese", "Chinese (Simplified)"},
    {"spanish", "Spanish (Spain)"},
    {"swedish", "Swedish"},
    {"tchinese", "Chinese (Traditional)"},
    {"thai", "Thai"},
    {"turkish", "Turkish"},
    {"ukrainian", "Ukrainian"},
    {"vietnamese", "Vietnamese"},
};

//! The index of the Steam language @p steam; -1 for none.
static inline int
pico4_setup_language_index(const char *steam)
{
	for (int i = 0; steam != NULL && i < PICO4_SETUP_LANGUAGES; i++) {
		if (strcmp(pico4_setup_languages[i].steam, steam) == 0) {
			return i;
		}
	}
	return -1;
}

//! A time zone as Steam offers it: its UTC offset (hhmm), the system zone, its texts' token names.
struct pico4_setup_zone
{
	int offset;
	const char *zone;
	const char *region; //!< #TimeZoneRegion_<region>
	const char *cities; //!< #TimeZoneCities_<cities>
};

#define PICO4_SETUP_ZONES 62

//! The client's time zones, in its order (by offset), each with the zone it sets for the system.
static const struct pico4_setup_zone pico4_setup_zones[PICO4_SETUP_ZONES] = {
    {-1100, "Pacific/Pago_Pago", "Samoa", "Samoa"},
    {-1000, "Pacific/Honolulu", "Hawaii", "Hawaii"},
    {-900, "America/Anchorage", "Alaska", "Alaska"},
    {-800, "America/Los_Angeles", "Pacific", "Pacific"},
    {-700, "America/Denver", "Mountain", "Mountain"},
    {-700, "America/Chihuahua", "MountainMexico", "MountainMexico"},
    {-700, "America/Phoenix", "MountainArizona", "MountainArizona"},
    {-600, "America/Chicago", "Central", "Central"},
    {-600, "America/Costa_Rica", "CentralAmerica", "CentralAmerica"},
    {-600, "Canada/Saskatchewan", "Central", "Saskatchewan"},
    {-500, "America/New_York", "Eastern", "Eastern"},
    {-500, "America/Indiana/Indianapolis", "Indiana", "Indiana"},
    {-500, "America/Bogota", "SAmericaPacific", "SAmericaPacific"},
    {-400, "America/Moncton", "Atlantic", "Atlantic"},
    {-400, "America/Caracas", "SAmericaWestern", "SAmericaWestern"},
    {-400, "America/Santiago", "Chile", "Chile"},
    {-330, "America/St_Johns", "EastCanada", "EastCanada"},
    {-300, "America/Sao_Paulo", "Brazil", "Brazil"},
    {-300, "America/Argentina/Buenos_Aires", "SAmericaEast", "SAmericaEast"},
    {-300, "America/Godthab", "Greenland", "Greenland"},
    {-100, "Atlantic/Azores", "Azores", "Azores"},
    {-100, "Atlantic/Cape_Verde", "CapeVerde", "CapeVerde"},
    {0, "Europe/Dublin", "GMT", "GMT"},
    {100, "Europe/Paris", "CentralEurope", "CentralEurope"},
    {100, "Africa/Lagos", "WestAfrica", "WestAfrica"},
    {100, "Africa/Harare", "CentralAfrica", "CentralAfrica"},
    {200, "Europe/Bucharest", "EasternEurope", "EasternEurope"},
    {200, "Asia/Jerusalem", "Israel", "Israel"},
    {300, "Europe/Moscow", "Moscow", "Moscow"},
    {300, "Asia/Kuwait", "Arabia", "Arabia"},
    {300, "Africa/Nairobi", "EastAfrica", "EastAfrica"},
    {330, "Asia/Tehran", "Iran", "Iran"},
    {400, "Asia/Muscat", "Arabian", "Arabian"},
    {400, "Asia/Baku", "Caucasus", "Caucasus"},
    {430, "Asia/Kabul", "Afghanistan", "Afghanistan"},
    {500, "Asia/Yekaterinburg", "Yekaterinburg", "Yekaterinburg"},
    {500, "Asia/Karachi", "WestAsia", "WestAsia"},
    {530, "Asia/Kolkata", "India", "India"},
    {545, "Asia/Kathmandu", "Nepal", "Nepal"},
    {600, "Asia/Dhaka", "CentralAsia", "CentralAsia"},
    {600, "Asia/Novosibirsk", "NorthCentralAsia", "NorthCentralAsia"},
    {630, "Asia/Rangoon", "Myanmar", "Myanmar"},
    {700, "Asia/Bangkok", "SoutheastAsia", "SoutheastAsia"},
    {700, "Asia/Krasnoyarsk", "NorthAsia", "NorthAsia"},
    {800, "Asia/Shanghai", "China", "China"},
    {800, "Asia/Singapore", "Singapore", "Singapore"},
    {800, "Asia/Taipei", "Taipei", "Taipei"},
    {800, "Australia/Perth", "WestAustralia", "WestAustralia"},
    {800, "Asia/Irkutsk", "NorthAsiaEast", "NorthAsiaEast"},
    {900, "Asia/Seoul", "Korea", "Korea"},
    {900, "Asia/Tokyo", "Japan", "Japan"},
    {900, "Asia/Yakutsk", "Yakutsk", "Yakutsk"},
    {930, "Australia/Darwin", "CentralAustraliaD", "CentralAustraliaD"},
    {930, "Australia/Adelaide", "CentralAustraliaA", "CentralAustraliaA"},
    {1000, "Australia/Sydney", "EasternAustralia", "EasternAustralia"},
    {1000, "Australia/Brisbane", "EasternAustraliaB", "EasternAustraliaB"},
    {1000, "Asia/Vladivostok", "Vladivostok", "Vladivostok"},
    {1000, "Pacific/Guam", "WestPacific", "WestPacific"},
    {1100, "Pacific/Guadalcanal", "CentralPacific", "CentralPacific"},
    {1200, "Pacific/Fiji", "Fiji", "Fiji"},
    {1200, "Pacific/Auckland", "NewZealand", "NewZealand"},
    {1300, "Pacific/Tongatapu", "Tonga", "Tonga"},
};

/*!
 * The zone to preselect for the system's @p zone (`/etc/localtime`'s target below zoneinfo, or ""
 * when it has none): the matching entry, else GMT, as Steam shows an unset zone.
 */
static inline int
pico4_setup_zone_index(const char *zone)
{
	for (int i = 0; zone != NULL && i < PICO4_SETUP_ZONES; i++) {
		if (strcmp(pico4_setup_zones[i].zone, zone) == 0) {
			return i;
		}
	}
	for (int i = 0; i < PICO4_SETUP_ZONES; i++) {
		if (pico4_setup_zones[i].offset == 0) {
			return i;
		}
	}
	return 0;
}

//! "UTC+05:30" for @p offset hhmm, into @p out.
static inline void
pico4_setup_offset_text(int offset, char *out, size_t n)
{
	const int a = offset < 0 ? -offset : offset;
	snprintf(out, n, "UTC%c%02d:%02d", offset < 0 ? '-' : '+', a / 100, a % 100);
}

// -- Lists ------------------------------------------------------------------------------------------

//! Rows a list page shows.
#define PICO4_SETUP_ROWS 5

// The list's place on the 1000 x 600 page, y down: rows on the left, the page arrows on the right.
#define PICO4_SETUP_LIST_X 40.0
#define PICO4_SETUP_LIST_Y 130.0
#define PICO4_SETUP_LIST_W 820.0
#define PICO4_SETUP_ROW_H 60.0
#define PICO4_SETUP_ARROW_X 880.0
#define PICO4_SETUP_ARROW_W 80.0
#define PICO4_SETUP_ARROW_H 140.0

//! The row of the list's page at (@p x, @p y): 0 to PICO4_SETUP_ROWS - 1, or -1.
static inline int
pico4_setup_hit_row(double x, double y)
{
	if (x < PICO4_SETUP_LIST_X || x >= PICO4_SETUP_LIST_X + PICO4_SETUP_LIST_W || y < PICO4_SETUP_LIST_Y) {
		return -1;
	}
	const int row = (int)((y - PICO4_SETUP_LIST_Y) / PICO4_SETUP_ROW_H);
	return row < PICO4_SETUP_ROWS ? row : -1;
}

//! The page arrow at (@p x, @p y): -1 up, 1 down, 0 none.
static inline int
pico4_setup_hit_arrow(double x, double y)
{
	if (x < PICO4_SETUP_ARROW_X || x >= PICO4_SETUP_ARROW_X + PICO4_SETUP_ARROW_W) {
		return 0;
	}
	if (y >= PICO4_SETUP_LIST_Y && y < PICO4_SETUP_LIST_Y + PICO4_SETUP_ARROW_H) {
		return -1;
	}
	const double down = PICO4_SETUP_LIST_Y + PICO4_SETUP_ROWS * PICO4_SETUP_ROW_H - PICO4_SETUP_ARROW_H;
	return y >= down && y < down + PICO4_SETUP_ARROW_H ? 1 : 0;
}

// The lens page's two buttons: closer and wider, either side of the distance.
#define PICO4_SETUP_IPD_Y 230.0
#define PICO4_SETUP_IPD_H 120.0
#define PICO4_SETUP_IPD_W 140.0
#define PICO4_SETUP_IPD_LESS_X 180.0
#define PICO4_SETUP_IPD_MORE_X 680.0

//! The lens button at (@p x, @p y): -1 closer, 1 wider, 0 none.
static inline int
pico4_setup_hit_ipd(double x, double y)
{
	if (y < PICO4_SETUP_IPD_Y || y >= PICO4_SETUP_IPD_Y + PICO4_SETUP_IPD_H) {
		return 0;
	}
	if (x >= PICO4_SETUP_IPD_LESS_X && x < PICO4_SETUP_IPD_LESS_X + PICO4_SETUP_IPD_W) {
		return -1;
	}
	return x >= PICO4_SETUP_IPD_MORE_X && x < PICO4_SETUP_IPD_MORE_X + PICO4_SETUP_IPD_W ? 1 : 0;
}

/*!
 * The Steam client's interface language in its `registry.vdf` (@p vdf): the first `"language"`
 * value, into @p out; false when it has none (before the client's first start).
 */
static inline bool
pico4_setup_registry_language(const char *vdf, char *out, size_t n)
{
	const char *key = "\"language\"";
	const char *p = vdf != NULL ? strstr(vdf, key) : NULL;
	if (p == NULL) {
		return false;
	}
	p += strlen(key);
	while (*p == ' ' || *p == '\t') {
		p++;
	}
	if (*p != '"') {
		return false;
	}
	size_t k = 0;
	for (p++; *p != '\0' && *p != '"' && *p != '\n' && k + 1 < n; p++) {
		out[k++] = *p;
	}
	out[k] = '\0';
	return *p == '"' && k > 0;
}

//! The page of rows that holds row @p index.
static inline int
pico4_setup_list_page(int index)
{
	return index < 0 ? 0 : index / PICO4_SETUP_ROWS;
}

//! How many pages @p count rows take; at least one.
static inline int
pico4_setup_list_pages(int count)
{
	return count <= 0 ? 1 : (count + PICO4_SETUP_ROWS - 1) / PICO4_SETUP_ROWS;
}

//! Page @p page moved by @p delta, kept within the list.
static inline int
pico4_setup_list_turn(int page, int delta, int count)
{
	int p = page + delta;
	const int last = pico4_setup_list_pages(count) - 1;
	return p < 0 ? 0 : p > last ? last : p;
}

// -- The lens distance --------------------------------------------------------------------------------

//! The lens distance's range and step in 0.01 mm: the PICO 4's specified 62-72 mm, PICO OS's 0.5 mm.
#define PICO4_SETUP_IPD_MIN 6200
#define PICO4_SETUP_IPD_MAX 7200
#define PICO4_SETUP_IPD_STEP 50

//! @p ipd (0.01 mm) moved by @p steps, on the 0.5 mm grid, within the range.
static inline int
pico4_setup_ipd_step(int ipd, int steps)
{
	int v = (ipd + PICO4_SETUP_IPD_STEP / 2) / PICO4_SETUP_IPD_STEP * PICO4_SETUP_IPD_STEP;
	v += steps * PICO4_SETUP_IPD_STEP;
	return v < PICO4_SETUP_IPD_MIN ? PICO4_SETUP_IPD_MIN : v > PICO4_SETUP_IPD_MAX ? PICO4_SETUP_IPD_MAX : v;
}

//! `pico4-lens`'s answer ("63.48") in 0.01 mm; -1 for anything else.
static inline int
pico4_setup_parse_ipd(const char *text)
{
	char *end;
	const double mm = strtod(text, &end);
	if (end == text || mm < 50 || mm > 80) {
		return -1;
	}
	return (int)(mm * 100 + 0.5);
}

// -- Wi-Fi ------------------------------------------------------------------------------------------

#define PICO4_SETUP_SSID 33 //!< 32 bytes and the end

struct pico4_setup_network
{
	char ssid[PICO4_SETUP_SSID];
	int signal;  //!< 0-100
	bool secured;
	bool active; //!< in use
};

/*!
 * One field of a `nmcli -t` line from @p p (fields end at ':', escaped as "\:"; a backslash as
 * "\\"), into @p out; returns where the next field starts, NULL after the last.
 */
static inline const char *
pico4_setup_nmcli_field(const char *p, char *out, size_t n)
{
	size_t k = 0;
	for (; *p != '\0' && *p != '\n'; p++) {
		if (*p == '\\' && (p[1] == ':' || p[1] == '\\')) {
			p++;
		} else if (*p == ':') {
			break;
		}
		if (k + 1 < n) {
			out[k++] = *p;
		}
	}
	out[k] = '\0';
	return *p == ':' ? p + 1 : NULL;
}

/*!
 * The networks in `nmcli -t -f IN-USE,SSID,SIGNAL,SECURITY device wifi list` (@p text), strongest
 * first, one entry per name (hidden networks without one left out); returns how many, at most @p max.
 */
static inline int
pico4_setup_parse_networks(const char *text, struct pico4_setup_network *nets, int max)
{
	int count = 0;
	for (const char *line = text; line != NULL && *line != '\0';) {
		char use[8], ssid[PICO4_SETUP_SSID], signal[8], security[64];
		const char *p = pico4_setup_nmcli_field(line, use, sizeof(use));
		p = p != NULL ? pico4_setup_nmcli_field(p, ssid, sizeof(ssid)) : NULL;
		p = p != NULL ? pico4_setup_nmcli_field(p, signal, sizeof(signal)) : NULL;
		if (p != NULL) {
			pico4_setup_nmcli_field(p, security, sizeof(security));
		}
		const char *next = strchr(line, '\n');
		line = next != NULL ? next + 1 : NULL;
		if (p == NULL || ssid[0] == '\0') {
			continue;
		}
		struct pico4_setup_network n = {{0}, atoi(signal), security[0] != '\0' && strcmp(security, "--") != 0,
		                                strcmp(use, "*") == 0};
		snprintf(n.ssid, sizeof(n.ssid), "%s", ssid);
		int at = -1;
		for (int i = 0; i < count; i++) {
			if (strcmp(nets[i].ssid, n.ssid) == 0) {
				at = i;
			}
		}
		if (at >= 0) {
			// The same name from several access points: the strongest, in use if any is.
			n.active = n.active || nets[at].active;
			if (n.signal <= nets[at].signal) {
				nets[at].active = n.active;
				continue;
			}
			memmove(&nets[at], &nets[at + 1], (size_t)(count - at - 1) * sizeof(*nets));
			count--;
		}
		if (count == max && (max == 0 || n.signal <= nets[count - 1].signal)) {
			continue;
		}
		int i = count < max ? count++ : count - 1;
		for (; i > 0 && nets[i - 1].signal < n.signal; i--) {
			nets[i] = nets[i - 1];
		}
		nets[i] = n;
	}
	return count;
}

//! Bars (0-4) for a signal of @p signal percent.
static inline int
pico4_setup_bars(int signal)
{
	return signal >= 75 ? 4 : signal >= 50 ? 3 : signal >= 25 ? 2 : signal > 0 ? 1 : 0;
}

/*!
 * The Wi-Fi page's steps, as Steam's network setup goes: the list, then for a hidden network its
 * name, the password for one that asks for it, and the attempt, which ends joined (back to the
 * list) or failed (its reason, then the password again or the list).
 */
enum pico4_setup_wifi_step
{
	PICO4_WIFI_LIST,
	PICO4_WIFI_NAME,
	PICO4_WIFI_PASSWORD,
	PICO4_WIFI_JOINING,
	PICO4_WIFI_JOINED,
	PICO4_WIFI_FAILED,
};

//! What an attempt to join ended in.
enum pico4_setup_join
{
	PICO4_JOIN_OK,
	PICO4_JOIN_PASSWORD,  //!< the network refused the password, or wanted one
	PICO4_JOIN_NOT_FOUND, //!< no network of that name in range
	PICO4_JOIN_FAILED,    //!< anything else
};

/*!
 * The end of `nmcli device wifi connect` from its exit @p status and its @p output: nmcli reports a
 * refused password as missing secrets ("Secrets were required, but not provided"), a name not in
 * range as "No network with SSID ... found" (status 10).
 */
static inline enum pico4_setup_join
pico4_setup_join_result(int status, const char *output)
{
	if (status == 0) {
		return PICO4_JOIN_OK;
	}
	if (output != NULL && (strstr(output, "Secrets were required") != NULL || strstr(output, "psk: property is invalid") != NULL)) {
		return PICO4_JOIN_PASSWORD;
	}
	if (status == 10 || (output != NULL && strstr(output, "No network with SSID") != NULL)) {
		return PICO4_JOIN_NOT_FOUND;
	}
	return PICO4_JOIN_FAILED;
}

/*!
 * Whether @p password can be offered: a WPA passphrase of 8 to 63 characters or 64 hexadecimal
 * digits; for a hidden network (@p hidden), none for an open one too.
 */
static inline bool
pico4_setup_password_ok(const char *password, bool hidden)
{
	const size_t n = strlen(password);
	if (n == 0) {
		return hidden;
	}
	return n >= 8 && n <= 64;
}

#define PICO4_SETUP_TEXT 128 //!< a typed name or password, with the end

/*!
 * One of the keyboard's inputs (@p input, SteamVR's minimal mode: a character as UTF-8, "\b" to
 * delete, "\n" to send) applied to @p text (at most @p n bytes with the end): the last character
 * goes as a whole, what does not fit is left out. Returns true for "\n".
 */
static inline bool
pico4_setup_text_input(char *text, size_t n, const char *input)
{
	size_t k = strlen(text);
	if (strcmp(input, "\n") == 0 || strcmp(input, "\r") == 0) {
		return true;
	}
	if (strcmp(input, "\b") == 0) {
		while (k > 0 && ((unsigned char)text[k - 1] & 0xC0) == 0x80) {
			k--; // continuation bytes of the last character
		}
		if (k > 0) {
			k--;
		}
		text[k] = '\0';
		return false;
	}
	const size_t add = strlen(input);
	if (add > 0 && (unsigned char)input[0] >= 0x20 && input[0] != 0x7F && k + add < n) {
		memcpy(text + k, input, add + 1);
	}
	return false;
}

//! The number of characters (not bytes) in UTF-8 @p text, for the dots that stand for a password.
static inline size_t
pico4_setup_text_chars(const char *text)
{
	size_t chars = 0;
	for (const unsigned char *p = (const unsigned char *)text; *p != '\0'; p++) {
		chars += (*p & 0xC0) != 0x80;
	}
	return chars;
}

// The input page's field, its show-password switch under it, and the attempt's sign.
#define PICO4_SETUP_FIELD_X 120.0
#define PICO4_SETUP_FIELD_Y 170.0
#define PICO4_SETUP_FIELD_W 760.0
#define PICO4_SETUP_FIELD_H 80.0
#define PICO4_SETUP_SHOW_Y 270.0
#define PICO4_SETUP_SHOW_H 50.0
#define PICO4_SETUP_SHOW_W 320.0

//! The field (1) or the show-password switch (2) at (@p x, @p y); 0 for neither.
static inline int
pico4_setup_hit_field(double x, double y)
{
	if (x >= PICO4_SETUP_FIELD_X && x < PICO4_SETUP_FIELD_X + PICO4_SETUP_FIELD_W && y >= PICO4_SETUP_FIELD_Y &&
	    y < PICO4_SETUP_FIELD_Y + PICO4_SETUP_FIELD_H) {
		return 1;
	}
	if (x >= PICO4_SETUP_FIELD_X && x < PICO4_SETUP_FIELD_X + PICO4_SETUP_SHOW_W && y >= PICO4_SETUP_SHOW_Y &&
	    y < PICO4_SETUP_SHOW_Y + PICO4_SETUP_SHOW_H) {
		return 2;
	}
	return 0;
}

// -- The Steam client's texts -------------------------------------------------------------------------

/*!
 * Append the UTF-8 encoding of @p cp to @p out (at @p *k, at most @p n with the end); false when full.
 */
static inline bool
pico4_setup_put_utf8(uint32_t cp, char *out, size_t n, size_t *k)
{
	char b[4];
	size_t len;
	if (cp < 0x80) {
		b[0] = (char)cp, len = 1;
	} else if (cp < 0x800) {
		b[0] = (char)(0xC0 | cp >> 6), b[1] = (char)(0x80 | (cp & 0x3F)), len = 2;
	} else if (cp < 0x10000) {
		b[0] = (char)(0xE0 | cp >> 12), b[1] = (char)(0x80 | (cp >> 6 & 0x3F)), b[2] = (char)(0x80 | (cp & 0x3F));
		len = 3;
	} else {
		b[0] = (char)(0xF0 | cp >> 18), b[1] = (char)(0x80 | (cp >> 12 & 0x3F));
		b[2] = (char)(0x80 | (cp >> 6 & 0x3F)), b[3] = (char)(0x80 | (cp & 0x3F)), len = 4;
	}
	if (*k + len >= n) {
		return false;
	}
	memcpy(out + *k, b, len);
	*k += len;
	return true;
}

static inline int
pico4_setup_hex(const char *p, int digits, uint32_t *v)
{
	*v = 0;
	for (int i = 0; i < digits; i++) {
		const char c = p[i];
		const int d = c >= '0' && c <= '9' ? c - '0' : c >= 'a' && c <= 'f' ? c - 'a' + 10 : c >= 'A' && c <= 'F' ? c - 'A' + 10 : -1;
		if (d < 0) {
			return 0;
		}
		*v = *v << 4 | (uint32_t)d;
	}
	return digits;
}

/*!
 * A JavaScript string from @p p, just after its opening quote, decoded into UTF-8 (\uXXXX with
 * surrogate pairs, \xHH, \n, \t, \" and \\); @p layers is 2 for a JSON text inside a template
 * literal, where JSON's own escapes arrive doubled. Returns false without its closing quote.
 */
static inline bool
pico4_setup_js_decode(const char *p, int layers, char *out, size_t n)
{
	size_t k = 0;
	out[0] = '\0';
	for (; *p != '\0'; p++) {
		if (*p == '"') {
			out[k] = '\0';
			return true;
		}
		uint32_t cp = (unsigned char)*p;
		if (*p == '\\') {
			const char *e = p + 1;
			if (layers == 2 && *e == '\\') {
				e++; // JSON's escape, doubled for the template literal
			}
			uint32_t v;
			switch (*e) {
			case 'n': cp = '\n'; break;
			case 't': cp = '\t'; break;
			case '"':
			case '\\':
			case '/':
			case '\'': cp = (unsigned char)*e; break;
			case 'x':
				if (pico4_setup_hex(e + 1, 2, &v) == 0) {
					return false;
				}
				cp = v, e += 2;
				break;
			case 'u':
				if (pico4_setup_hex(e + 1, 4, &v) == 0) {
					return false;
				}
				e += 4;
				cp = v;
				if (v >= 0xD800 && v < 0xDC00 && e[1] == '\\') {
					const char *f = e + 2;
					if (layers == 2 && *f == '\\') {
						f++;
					}
					uint32_t lo;
					if (*f == 'u' && pico4_setup_hex(f + 1, 4, &lo) == 4 && lo >= 0xDC00 && lo < 0xE000) {
						cp = 0x10000 + ((v - 0xD800) << 10) + (lo - 0xDC00);
						e = f + 4;
					}
				}
				break;
			default: return false;
			}
			p = e;
		} else if (cp >= 0x80) {
			// UTF-8 as it is.
			if (k + 1 >= n) {
				return false;
			}
			out[k++] = *p;
			continue;
		}
		if (!pico4_setup_put_utf8(cp, out, n, &k)) {
			return false;
		}
	}
	return false;
}

/*!
 * The value of @p key in one of the client's localization files (`steamui_<language>-json.js`: a
 * JSON object inside a template literal, `"Key":"value"`), into @p out; false when it has none.
 */
static inline bool
pico4_setup_steam_token(const char *file, const char *key, char *out, size_t n)
{
	char pattern[96];
	snprintf(pattern, sizeof(pattern), "\"%s\":\"", key);
	const char *p = file != NULL ? strstr(file, pattern) : NULL;
	return p != NULL && pico4_setup_js_decode(p + strlen(pattern), 2, out, n) && out[0] != '\0';
}

/*!
 * One language's entry @p key in the client's language-screen module (an object literal of
 * `language:{Key:"value",...}`, the texts its setup shows each language in), into @p out.
 */
static inline bool
pico4_setup_language_text(const char *module, const char *language, const char *key, char *out, size_t n)
{
	char pattern[64];
	snprintf(pattern, sizeof(pattern), "%s:{", language);
	const char *p = module != NULL ? strstr(module, pattern) : NULL;
	// The language's own object: the key before its closing brace (values hold no braces).
	while (p != NULL && p > module && (p[-1] == '_' || (p[-1] >= 'a' && p[-1] <= 'z'))) {
		p = strstr(p + 1, pattern); // a longer name ending in this one
	}
	if (p == NULL) {
		return false;
	}
	const char *end = strchr(p, '}');
	snprintf(pattern, sizeof(pattern), "%s:\"", key);
	const char *v = strstr(p, pattern);
	return v != NULL && (end == NULL || v < end) && pico4_setup_js_decode(v + strlen(pattern), 1, out, n) &&
	       out[0] != '\0';
}

// -- Our own texts ------------------------------------------------------------------------------------

/*!
 * One text of ours in @p language from the strings file (@p file: `[language]` sections of
 * `key = value` lines, `\n` for a line break), into @p out; false when that language lacks it.
 */
static inline bool
pico4_setup_string(const char *file, const char *language, const char *key, char *out, size_t n)
{
	char section[64];
	snprintf(section, sizeof(section), "[%s]", language);
	const char *p = file;
	while (p != NULL && (p = strstr(p, section)) != NULL && !(p == file || p[-1] == '\n')) {
		p++;
	}
	if (p == NULL) {
		return false;
	}
	const size_t key_len = strlen(key);
	for (const char *line = strchr(p, '\n'); line != NULL; line = strchr(line, '\n')) {
		line++;
		if (*line == '[') {
			return false; // the next section
		}
		if (strncmp(line, key, key_len) != 0) {
			continue;
		}
		const char *v = line + key_len;
		while (*v == ' ') {
			v++;
		}
		if (*v != '=') {
			continue;
		}
		v++;
		while (*v == ' ') {
			v++;
		}
		size_t k = 0;
		for (; *v != '\0' && *v != '\n' && *v != '\r' && k + 1 < n; v++) {
			if (v[0] == '\\' && v[1] == 'n') {
				out[k++] = '\n';
				v++;
			} else {
				out[k++] = *v;
			}
		}
		out[k] = '\0';
		return k > 0;
	}
	return false;
}
