// Copyright 2026, overpressure contributors.
// SPDX-License-Identifier: BSL-1.0
/*
 * Overpressure Settings' menu, its controllers' page and when that page comes up in front of the
 * wearer.
 */
#include "pico4_pairing.h"
#include "pico4_settings.h"

#include <stdio.h>

static int failures;

#define EXPECT(c)                                                                                                      \
	do {                                                                                                           \
		if (!(c)) {                                                                                            \
			printf("line %d: %s\n", __LINE__, #c);                                                         \
			failures++;                                                                                    \
		}                                                                                                      \
	} while (0)

int
main(void)
{
	// The menu: one entry per page, nothing between or below them, nothing on the page side.
	EXPECT(pico4_settings_menu_hit(100, 120) == PICO4_SETTINGS_LENSES);
	EXPECT(pico4_settings_menu_hit(100, 200) == PICO4_SETTINGS_CONTROLLERS);
	EXPECT(pico4_settings_menu_hit(100, 180) == PICO4_SETTINGS_NONE);
	EXPECT(pico4_settings_menu_hit(100, 300) == PICO4_SETTINGS_NONE);
	EXPECT(pico4_settings_menu_hit(100, 60) == PICO4_SETTINGS_NONE);
	EXPECT(pico4_settings_menu_hit(400, 120) == PICO4_SETTINGS_NONE);
	// Both pages fit beside the menu.
	EXPECT(PICO4_SETTINGS_PAGE_X + PICO4_PANEL_W <= PICO4_SETTINGS_W);
	EXPECT(PICO4_SETTINGS_PAGE_Y + PICO4_PANEL_H <= PICO4_SETTINGS_H);
	EXPECT(PICO4_CONTROLLERS_PAIR_X + PICO4_CONTROLLERS_PAIR_W <= PICO4_CONTROLLERS_W);

	// A hand's Pair button only where it is shown; Close only on the panel in front.
	EXPECT(pico4_controllers_hit(600, 40, true, true, false) == PICO4_CONTROLLERS_PAIR_LEFT);
	EXPECT(pico4_controllers_hit(600, 140, true, true, false) == PICO4_CONTROLLERS_PAIR_RIGHT);
	EXPECT(pico4_controllers_hit(600, 40, false, true, false) == PICO4_CONTROLLERS_NONE);
	EXPECT(pico4_controllers_hit(600, 92, true, true, false) == PICO4_CONTROLLERS_NONE); // between the rows
	EXPECT(pico4_controllers_hit(300, 40, true, true, false) == PICO4_CONTROLLERS_NONE);
	EXPECT(pico4_controllers_hit(600, 440, false, false, true) == PICO4_CONTROLLERS_CLOSE);
	EXPECT(pico4_controllers_hit(600, 440, false, false, false) == PICO4_CONTROLLERS_NONE);

	// In front of the wearer: no controller, no pairing, the setup over, after the wait.
	EXPECT(pico4_settings_rescue(false, true, false, false, 0, 25, PICO4_PAIRING_WAIT_S));
	EXPECT(!pico4_settings_rescue(true, true, false, false, 0, 25, PICO4_PAIRING_WAIT_S)); // the setup pairs
	EXPECT(!pico4_settings_rescue(false, false, false, false, 0, 25, PICO4_PAIRING_WAIT_S)); // no status yet
	EXPECT(!pico4_settings_rescue(false, true, true, false, 0, 25, PICO4_PAIRING_WAIT_S));
	EXPECT(!pico4_settings_rescue(false, true, false, false, 1, 25, PICO4_PAIRING_WAIT_S));
	EXPECT(!pico4_settings_rescue(false, true, false, false, 0, 5, PICO4_PAIRING_WAIT_S));

	if (failures != 0) {
		printf("%d failure(s)\n", failures);
		return 1;
	}
	printf("test_pico4_settings: all checks passed\n");
	return 0;
}
