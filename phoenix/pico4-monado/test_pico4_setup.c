// Copyright 2026, overpressure contributors.
// SPDX-License-Identifier: BSL-1.0
/*
 * The first-run setup's decisions, tables and readers (pico4_setup.h), with made-up samples in the
 * formats of the Steam client's files and nmcli's output.
 */
#include "pico4_setup.h"

#include <stdio.h>

static int failures;

#define EXPECT(cond)                                                                                           \
	do {                                                                                                       \
		if (!(cond)) {                                                                                         \
			fprintf(stderr, "%s:%d: %s\n", __FILE__, __LINE__, #cond);                                       \
			failures++;                                                                                        \
		}                                                                                                      \
	} while (0)

static void
test_pages(void)
{
	EXPECT(pico4_setup_next(PICO4_SETUP_WELCOME) == PICO4_SETUP_LANGUAGE);
	EXPECT(pico4_setup_next(PICO4_SETUP_READY) == PICO4_SETUP_PAGES);
	EXPECT(pico4_setup_back(PICO4_SETUP_LANGUAGE) == PICO4_SETUP_WELCOME);
	EXPECT(pico4_setup_back(PICO4_SETUP_WELCOME) == PICO4_SETUP_WELCOME);
	// Before the first sign-in, until finished once; never on a headset someone signed in on.
	EXPECT(pico4_setup_starts(false, false));
	EXPECT(!pico4_setup_starts(false, true));
	EXPECT(!pico4_setup_starts(true, false));
}

static void
test_tables(void)
{
	// The client's languages: unique, English first (its default), each found by name.
	EXPECT(strcmp(pico4_setup_languages[0].steam, "english") == 0);
	for (int i = 0; i < PICO4_SETUP_LANGUAGES; i++) {
		EXPECT(pico4_setup_language_index(pico4_setup_languages[i].steam) == i);
	}
	EXPECT(pico4_setup_language_index("klingon") == -1);
	EXPECT(pico4_setup_language_index(NULL) == -1);
	// The zones: ordered by offset, each zone once, an unset or unknown zone shown as GMT.
	for (int i = 1; i < PICO4_SETUP_ZONES; i++) {
		EXPECT(pico4_setup_zones[i - 1].offset <= pico4_setup_zones[i].offset);
		for (int j = 0; j < i; j++) {
			EXPECT(strcmp(pico4_setup_zones[i].zone, pico4_setup_zones[j].zone) != 0);
		}
	}
	EXPECT(strcmp(pico4_setup_zones[pico4_setup_zone_index("Europe/Paris")].zone, "Europe/Paris") == 0);
	EXPECT(pico4_setup_zones[pico4_setup_zone_index("")].offset == 0);
	EXPECT(pico4_setup_zones[pico4_setup_zone_index("Mars/Olympus")].offset == 0);
	char text[16];
	pico4_setup_offset_text(-330, text, sizeof(text));
	EXPECT(strcmp(text, "UTC-03:30") == 0);
	pico4_setup_offset_text(545, text, sizeof(text));
	EXPECT(strcmp(text, "UTC+05:45") == 0);
}

static void
test_lists(void)
{
	EXPECT(pico4_setup_list_pages(0) == 1);
	EXPECT(pico4_setup_list_pages(PICO4_SETUP_ROWS) == 1);
	EXPECT(pico4_setup_list_pages(PICO4_SETUP_ROWS + 1) == 2);
	EXPECT(pico4_setup_list_page(PICO4_SETUP_ROWS) == 1);
	EXPECT(pico4_setup_list_turn(0, -1, 31) == 0);
	EXPECT(pico4_setup_list_turn(4, 1, 31) == 5);
	EXPECT(pico4_setup_list_turn(6, 1, 31) == 6); // the last of 7
	// Rows above the page's note (y 440) and the buttons, arrows beside them.
	EXPECT(PICO4_SETUP_LIST_Y + PICO4_SETUP_ROWS * PICO4_SETUP_ROW_H <= 440.0);
	EXPECT(pico4_setup_hit_row(100, PICO4_SETUP_LIST_Y + 1) == 0);
	EXPECT(pico4_setup_hit_row(100, PICO4_SETUP_LIST_Y + PICO4_SETUP_ROW_H * (PICO4_SETUP_ROWS - 1) + 1) == PICO4_SETUP_ROWS - 1);
	EXPECT(pico4_setup_hit_row(100, PICO4_SETUP_LIST_Y + PICO4_SETUP_ROW_H * PICO4_SETUP_ROWS + 1) == -1);
	EXPECT(pico4_setup_hit_row(900, PICO4_SETUP_LIST_Y + 1) == -1);
	EXPECT(pico4_setup_hit_arrow(900, PICO4_SETUP_LIST_Y + 10) == -1);
	EXPECT(pico4_setup_hit_arrow(900, PICO4_SETUP_LIST_Y + PICO4_SETUP_ROWS * PICO4_SETUP_ROW_H - 10) == 1);
	EXPECT(pico4_setup_hit_arrow(100, PICO4_SETUP_LIST_Y + 10) == 0);
}

static void
test_ipd(void)
{
	EXPECT(pico4_setup_ipd_step(6348, 0) == 6350);
	EXPECT(pico4_setup_ipd_step(6348, 1) == 6400);
	EXPECT(pico4_setup_ipd_step(6348, -1) == 6300);
	EXPECT(pico4_setup_ipd_step(6200, -1) == 6200);
	EXPECT(pico4_setup_ipd_step(7200, 1) == 7200);
	EXPECT(pico4_setup_parse_ipd("63.48\n") == 6348);
	EXPECT(pico4_setup_parse_ipd("error") == -1);
	EXPECT(pico4_setup_parse_ipd("9.0") == -1);
	EXPECT(pico4_setup_hit_ipd(PICO4_SETUP_IPD_LESS_X + 1, PICO4_SETUP_IPD_Y + 1) == -1);
	EXPECT(pico4_setup_hit_ipd(PICO4_SETUP_IPD_MORE_X + 1, PICO4_SETUP_IPD_Y + 1) == 1);
	EXPECT(pico4_setup_hit_ipd(500, PICO4_SETUP_IPD_Y + 1) == 0);
}

static void
test_networks(void)
{
	// nmcli -t: "\:" in a name, the same name from two access points, a hidden one, an open one.
	const char *text = " :Home\\:5G:54:WPA2\n"
	                   "*:Home\\:5G:81:WPA2\n"
	                   " ::40:WPA2\n"
	                   " :Cafe:90:\n"
	                   " :Office:20:WPA2 WPA3\n";
	struct pico4_setup_network nets[8];
	const int n = pico4_setup_parse_networks(text, nets, 8);
	EXPECT(n == 3);
	EXPECT(strcmp(nets[0].ssid, "Cafe") == 0 && !nets[0].secured && !nets[0].active);
	EXPECT(strcmp(nets[1].ssid, "Home:5G") == 0 && nets[1].signal == 81 && nets[1].secured && nets[1].active);
	EXPECT(strcmp(nets[2].ssid, "Office") == 0 && nets[2].secured);
	// "--" is nmcli's empty security; a full list keeps the strongest.
	EXPECT(pico4_setup_parse_networks(" :Open:50:--\n", nets, 8) == 1 && !nets[0].secured);
	EXPECT(pico4_setup_parse_networks(" :A:10:\n :B:90:\n :C:50:\n", nets, 2) == 2);
	EXPECT(strcmp(nets[0].ssid, "B") == 0 && strcmp(nets[1].ssid, "C") == 0);
	EXPECT(pico4_setup_parse_networks("", nets, 8) == 0);
	EXPECT(pico4_setup_bars(81) == 4 && pico4_setup_bars(30) == 2 && pico4_setup_bars(0) == 0);
}

static void
test_joining(void)
{
	// nmcli's words for a refused password, a name out of range, and anything else.
	EXPECT(pico4_setup_join_result(0, "Device 'wlan0' successfully activated") == PICO4_JOIN_OK);
	EXPECT(pico4_setup_join_result(4, "Error: Connection activation failed: Secrets were required, but not provided.\n") ==
	       PICO4_JOIN_PASSWORD);
	EXPECT(pico4_setup_join_result(4, "Error: 802-11-wireless-security.psk: property is invalid.") == PICO4_JOIN_PASSWORD);
	EXPECT(pico4_setup_join_result(10, "Error: No network with SSID 'Cafe' found.") == PICO4_JOIN_NOT_FOUND);
	EXPECT(pico4_setup_join_result(4, "Error: Connection activation failed: IP configuration could not be reserved") ==
	       PICO4_JOIN_FAILED);
	EXPECT(pico4_setup_join_result(1, NULL) == PICO4_JOIN_FAILED);

	// WPA's passphrases: 8 to 63 characters or 64 hexadecimal digits; none only for a hidden open network.
	EXPECT(!pico4_setup_password_ok("", false) && pico4_setup_password_ok("", true));
	EXPECT(!pico4_setup_password_ok("1234567", false) && pico4_setup_password_ok("12345678", false));
	char longest[66];
	memset(longest, 'a', 64);
	longest[64] = '\0';
	EXPECT(pico4_setup_password_ok(longest, false));
	longest[64] = 'a', longest[65] = '\0';
	EXPECT(!pico4_setup_password_ok(longest, false));

	// The keyboard's minimal mode: characters as UTF-8, "\b" deletes a whole character, "\n" sends.
	char text[8] = "";
	EXPECT(!pico4_setup_text_input(text, sizeof(text), "a"));
	EXPECT(!pico4_setup_text_input(text, sizeof(text), "\xC3\xBC")); // ü, two bytes
	EXPECT(strcmp(text, "a\xC3\xBC") == 0 && pico4_setup_text_chars(text) == 2);
	EXPECT(!pico4_setup_text_input(text, sizeof(text), "\b") && strcmp(text, "a") == 0);
	EXPECT(!pico4_setup_text_input(text, sizeof(text), "\b") && strcmp(text, "") == 0);
	EXPECT(!pico4_setup_text_input(text, sizeof(text), "\b") && strcmp(text, "") == 0);
	EXPECT(pico4_setup_text_input(text, sizeof(text), "\n") && pico4_setup_text_input(text, sizeof(text), "\r"));
	// Control characters stay out; what does not fit is left out, whole.
	EXPECT(!pico4_setup_text_input(text, sizeof(text), "\t") && strcmp(text, "") == 0);
	for (int i = 0; i < 10; i++) {
		pico4_setup_text_input(text, sizeof(text), "x");
	}
	EXPECT(strcmp(text, "xxxxxxx") == 0);
	EXPECT(!pico4_setup_text_input(text, sizeof(text), "\xE2\x82\xAC") && strcmp(text, "xxxxxxx") == 0);

	// The field and the show-password switch under it.
	EXPECT(pico4_setup_hit_field(PICO4_SETUP_FIELD_X + 10, PICO4_SETUP_FIELD_Y + 10) == 1);
	EXPECT(pico4_setup_hit_field(PICO4_SETUP_FIELD_X + 10, PICO4_SETUP_SHOW_Y + 10) == 2);
	EXPECT(pico4_setup_hit_field(PICO4_SETUP_FIELD_X - 1, PICO4_SETUP_FIELD_Y + 10) == 0);
	EXPECT(pico4_setup_hit_field(PICO4_SETUP_FIELD_X + PICO4_SETUP_SHOW_W + 1, PICO4_SETUP_SHOW_Y + 10) == 0);
}

static void
test_steam_texts(void)
{
	// The localization file: JSON in a template literal, so JSON's escapes arrive doubled.
	const char *file = "\"use strict\";(()=>{e.exports=JSON.parse(`{\"Generic_Close\":\"Schlie\\xDFen\","
	                   "\"Settings_Internet_Connect\":\"Verbinden\",\"Quote\":\"say \\\\\"hi\\\\\"\","
	                   "\"Lines\":\"a\\\\nb\",\"Emoji\":\"\\uD83D\\uDE00\",\"Empty\":\"\"}`)})";
	char out[64];
	EXPECT(pico4_setup_steam_token(file, "Settings_Internet_Connect", out, sizeof(out)) && strcmp(out, "Verbinden") == 0);
	EXPECT(pico4_setup_steam_token(file, "Generic_Close", out, sizeof(out)) && strcmp(out, "Schlie\xC3\x9F" "en") == 0);
	EXPECT(pico4_setup_steam_token(file, "Quote", out, sizeof(out)) && strcmp(out, "say \"hi\"") == 0);
	EXPECT(pico4_setup_steam_token(file, "Lines", out, sizeof(out)) && strcmp(out, "a\nb") == 0);
	EXPECT(pico4_setup_steam_token(file, "Emoji", out, sizeof(out)) && strcmp(out, "\xF0\x9F\x98\x80") == 0);
	EXPECT(!pico4_setup_steam_token(file, "Empty", out, sizeof(out)));
	EXPECT(!pico4_setup_steam_token(file, "Missing", out, sizeof(out)));
	EXPECT(!pico4_setup_steam_token(NULL, "Missing", out, sizeof(out)));
	char small[6];
	EXPECT(!pico4_setup_steam_token(file, "Settings_Internet_Connect", small, sizeof(small)));

	// The language-screen module: an object literal per language, single escapes.
	const char *module = "const n={latam:{Steam_Welcome:\"Te damos la bienvenida\",Language_Name:\"Espa\\xF1ol\"},"
	                     "spanish:{Steam_Welcome:\"Bienvenido\",Language_Name:\"Espa\\xF1ol - Espa\\xF1a\"},"
	                     "russian:{Language_Name:\"\\u0420\\u0443\\u0441\\u0441\\u043A\\u0438\\u0439\"}}";
	EXPECT(pico4_setup_language_text(module, "spanish", "Steam_Welcome", out, sizeof(out)) && strcmp(out, "Bienvenido") == 0);
	EXPECT(pico4_setup_language_text(module, "latam", "Language_Name", out, sizeof(out)) &&
	       strcmp(out, "Espa\xC3\xB1ol") == 0);
	EXPECT(pico4_setup_language_text(module, "russian", "Language_Name", out, sizeof(out)) &&
	       strcmp(out, "\xD0\xA0\xD1\x83\xD1\x81\xD1\x81\xD0\xBA\xD0\xB8\xD0\xB9") == 0);
	// Russian has no welcome: none from the next language's object either.
	EXPECT(!pico4_setup_language_text(module, "russian", "Steam_Welcome", out, sizeof(out)));
	EXPECT(!pico4_setup_language_text(module, "thai", "Language_Name", out, sizeof(out)));
	EXPECT(!pico4_setup_language_text(NULL, "thai", "Language_Name", out, sizeof(out)));
}

static void
test_registry(void)
{
	const char *vdf = "\"Registry\"\n{\n\t\"HKCU\"\n\t{\n\t\t\"Steam\"\n\t\t{\n\t\t\t\"language\"\t\t\"german\"\n";
	char out[32];
	EXPECT(pico4_setup_registry_language(vdf, out, sizeof(out)) && strcmp(out, "german") == 0);
	EXPECT(!pico4_setup_registry_language("\"Registry\"\n{\n}\n", out, sizeof(out)));
	EXPECT(!pico4_setup_registry_language(NULL, out, sizeof(out)));
}

static void
test_strings(void)
{
	const char *file = "# comment\n[english]\nskip = Skip\nlines = one\\ntwo\nsetup_wifi_password = Password for %s\n"
	                   "[german]\nskip = \xC3\x9C" "berspringen\n[english_extra]\nonly = x\n";
	char out[64];
	EXPECT(pico4_setup_string(file, "english", "skip", out, sizeof(out)) && strcmp(out, "Skip") == 0);
	EXPECT(pico4_setup_string(file, "english", "lines", out, sizeof(out)) && strcmp(out, "one\ntwo") == 0);
	EXPECT(pico4_setup_string(file, "german", "skip", out, sizeof(out)) && strcmp(out, "\xC3\x9C" "berspringen") == 0);
	// A key missing from German is not taken from the next section; a prefix is not the key.
	EXPECT(!pico4_setup_string(file, "german", "lines", out, sizeof(out)));
	EXPECT(!pico4_setup_string(file, "english", "ski", out, sizeof(out)));
	EXPECT(!pico4_setup_string(file, "thai", "skip", out, sizeof(out)));
	EXPECT(!pico4_setup_string(file, "english", "only", out, sizeof(out)));
}

int
main(void)
{
	test_pages();
	test_tables();
	test_lists();
	test_ipd();
	test_networks();
	test_joining();
	test_steam_texts();
	test_registry();
	test_strings();
	if (failures != 0) {
		fprintf(stderr, "test_pico4_setup: %d failures\n", failures);
		return 1;
	}
	printf("test_pico4_setup: all checks passed\n");
	return 0;
}
