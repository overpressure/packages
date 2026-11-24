// Copyright 2026, overpressure contributors.
// SPDX-License-Identifier: BSL-1.0
/*
 * Overpressure Settings' lens distance page: its layout and presets.
 *
 *   cc -std=gnu11 -Isrc/xr src/xr/test_pico4_lens_panel.c -lm -o test_pico4_lens_panel
 */
#include "pico4_lens_panel.h"

#include <math.h>
#include <stdlib.h>
#include <unistd.h>

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
	// What a press hits.
	EXPECT(pico4_panel_hit(50, 200, false) == PICO4_PANEL_MINUS);
	EXPECT(pico4_panel_hit(750, 200, false) == PICO4_PANEL_PLUS);
	EXPECT(pico4_panel_hit(400, 230, false) == PICO4_PANEL_SLIDER);
	EXPECT(pico4_panel_hit(100, 350, false) == PICO4_PANEL_PRESET0);
	EXPECT(pico4_panel_hit(300, 350, false) == PICO4_PANEL_PRESET0 + 1);
	EXPECT(pico4_panel_hit(700, 350, false) == PICO4_PANEL_PRESET0 + 3);
	EXPECT(pico4_panel_hit(205, 350, false) == PICO4_PANEL_NONE); // between two presets
	EXPECT(pico4_panel_hit(780, 350, false) == PICO4_PANEL_NONE);
	EXPECT(pico4_panel_hit(400, 100, false) == PICO4_PANEL_NONE);
	// The warning takes the presets' row while it shows; the row above stays.
	EXPECT(pico4_panel_hit(100, 350, true) == PICO4_PANEL_WARNING);
	EXPECT(pico4_panel_hit(205, 350, true) == PICO4_PANEL_WARNING);
	EXPECT(pico4_panel_hit(780, 350, true) == PICO4_PANEL_NONE);
	EXPECT(pico4_panel_hit(50, 200, true) == PICO4_PANEL_MINUS);

	// The slider's scale: PICO OS's 58 to 72 mm.
	EXPECT(pico4_panel_slider_mm(PICO4_PANEL_SLIDER_X0) == 58.0);
	EXPECT(pico4_panel_slider_mm(PICO4_PANEL_SLIDER_X1) == 72.0);
	EXPECT(pico4_panel_slider_mm(0) == 58.0 && pico4_panel_slider_mm(800) == 72.0);
	EXPECT(fabs(pico4_panel_slider_x(65.0) - (PICO4_PANEL_SLIDER_X0 + PICO4_PANEL_SLIDER_X1) / 2) < 1e-9);
	EXPECT(pico4_panel_clamp(50) == 58.0 && pico4_panel_clamp(80) == 72.0 && pico4_panel_clamp(63.5) == 63.5);

	// Closer than 62 mm only with the warning's box ticked; wider always goes.
	bool refused;
	EXPECT(pico4_panel_gate(64.0, 63.5, false, &refused) == 63.5 && !refused);
	EXPECT(pico4_panel_gate(62.5, 62.0, false, &refused) == 62.0 && !refused);
	EXPECT(pico4_panel_gate(62.0, 61.5, false, &refused) == 62.0 && refused);  // held at 62, warned
	EXPECT(pico4_panel_gate(65.0, 59.0, false, &refused) == 62.0 && refused);  // a preset or the slider: stops at 62
	EXPECT(pico4_panel_gate(60.0, 59.5, false, &refused) == 60.0 && refused);  // closer already: stays, never wider
	EXPECT(pico4_panel_gate(60.0, 60.5, false, &refused) == 60.5 && !refused); // wider from below 62
	EXPECT(pico4_panel_gate(62.0, 61.5, true, &refused) == 61.5 && !refused);
	EXPECT(pico4_panel_gate(58.5, 57.0, true, &refused) == 58.0 && !refused); // the range's end
	// The warning: from a refused move until the lenses go beyond 62 mm.
	EXPECT(pico4_panel_warning_after(false, true, 62.0));
	EXPECT(pico4_panel_warning_after(true, false, 61.5));
	EXPECT(pico4_panel_warning_after(true, false, 62.0));
	EXPECT(!pico4_panel_warning_after(true, false, 62.5));
	EXPECT(!pico4_panel_warning_after(false, false, 61.0));

	// Presets across a restart; anything out of range is an empty one.
	char path[] = "test_pico4_presets_XXXXXX";
	close(mkstemp(path));
	double presets[PICO4_PANEL_PRESETS] = {63.5, 0, 71.9, 0}, loaded[PICO4_PANEL_PRESETS];
	EXPECT(pico4_panel_presets_save(path, presets));
	pico4_panel_presets_load(path, loaded);
	EXPECT(loaded[0] == 63.5 && loaded[1] == 0 && loaded[2] == 71.9 && loaded[3] == 0);
	FILE *f = fopen(path, "w");
	fputs("10\n65.5\n99\n", f);
	fclose(f);
	pico4_panel_presets_load(path, loaded);
	EXPECT(loaded[0] == 0 && loaded[1] == 65.5 && loaded[2] == 0 && loaded[3] == 0);
	unlink(path);
	pico4_panel_presets_load("no-such-file", loaded);
	EXPECT(loaded[0] == 0 && loaded[3] == 0);

	if (failures == 0) {
		printf("test_pico4_lens_panel: all checks passed\n");
	}
	return failures == 0 ? 0 : 1;
}
