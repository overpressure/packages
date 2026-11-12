// Copyright 2026, overpressure contributors.
// SPDX-License-Identifier: BSL-1.0
/*
 * The onboarding's decisions and layout.
 *
 *   cc -std=gnu11 -Isrc/xr src/xr/test_pico4_onboarding.c -o test_pico4_onboarding
 */
#include "pico4_onboarding.h"

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
	// Steam's OOBE: CompletedOOBE "1" in registry.vdf, whatever the tabs; "0" or missing is not done.
	EXPECT(pico4_tour_oobe_done("\"Steam\"\n{\n\t\t\"CompletedOOBEStage1\"\t\t\"1\"\n\t\t\"CompletedOOBE\"\t\t\"1\"\n}"));
	EXPECT(!pico4_tour_oobe_done("\t\"CompletedOOBEStage1\"\t\t\"1\"\n"));
	EXPECT(!pico4_tour_oobe_done("\t\"CompletedOOBE\"\t\t\"0\"\n"));
	EXPECT(!pico4_tour_oobe_done(""));
	EXPECT(!pico4_tour_oobe_done(NULL));

	// The tour starts once after the OOBE; the mark (done or skipped) ends that.
	EXPECT(pico4_tour_starts(true, false));
	EXPECT(!pico4_tour_starts(false, false));
	EXPECT(!pico4_tour_starts(true, true));

	// Hands-free pairing only without any controller, one hand after the other.
	EXPECT(pico4_tour_hands_free(false, false, 0) == 1);
	EXPECT(pico4_tour_hands_free(false, false, 1) == 2);
	EXPECT(pico4_tour_hands_free(false, false, 2) == 1);
	EXPECT(pico4_tour_hands_free(true, false, 1) == 0);
	EXPECT(pico4_tour_hands_free(false, true, 0) == 0);

	// The Pair button pairs the missing controller; none with both or neither.
	EXPECT(pico4_tour_pair_button(true, false) == 2);
	EXPECT(pico4_tour_pair_button(false, true) == 1);
	EXPECT(pico4_tour_pair_button(true, true) == 0);
	EXPECT(pico4_tour_pair_button(false, false) == 0);

	// Pages in order, then the end.
	EXPECT(pico4_tour_next(PICO4_TOUR_CONTROLLERS) == PICO4_TOUR_SAFETY);
	EXPECT(pico4_tour_next(PICO4_TOUR_PLAYSPACE) == PICO4_TOUR_DONE);
	EXPECT(pico4_tour_next(PICO4_TOUR_DONE) == PICO4_TOUR_PAGES);
	// The pages' texts are in pico4-onboarding.strings (tests/test_onboarding_strings.py).

	// The driver's answers.
	EXPECT(pico4_tour_pair_answer("pairing left") == 0);
	EXPECT(pico4_tour_pair_answer("paired left TESTSERIAL0000001") == 1);
	EXPECT(pico4_tour_pair_answer("timeout left") == -1);
	EXPECT(pico4_tour_pair_answer("refused right") == -1);
	EXPECT(pico4_tour_pair_answer("paired left TESTSERIAL0000001 (pairing not stopped)") == 1);
	bool left = false, right = true;
	EXPECT(pico4_tour_parse_status("left connected TESTSERIAL0000001\nright disconnected", &left, &right));
	EXPECT(left && !right);
	EXPECT(pico4_tour_parse_status("left disconnected\nright connected TESTSERIAL0000002", &left, &right));
	EXPECT(!left && right);
	EXPECT(!pico4_tour_parse_status("error request", &left, &right));

	// The buttons: Pair only when shown.
	EXPECT(pico4_tour_hit(100, 500, false) == PICO4_TOUR_SKIP);
	EXPECT(pico4_tour_hit(450, 500, true) == PICO4_TOUR_PAIR);
	EXPECT(pico4_tour_hit(450, 500, false) == PICO4_TOUR_NONE);
	EXPECT(pico4_tour_hit(800, 520, false) == PICO4_TOUR_NEXT);
	EXPECT(pico4_tour_hit(800, 300, false) == PICO4_TOUR_NONE);
	EXPECT(pico4_tour_hit(320, 500, true) == PICO4_TOUR_NONE); // between Skip and Pair

	if (failures != 0) {
		printf("%d failure(s)\n", failures);
		return 1;
	}
	printf("test_pico4_onboarding: all checks passed\n");
	return 0;
}
