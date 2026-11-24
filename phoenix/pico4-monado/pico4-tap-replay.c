// Copyright 2026, overpressure contributors.
// SPDX-License-Identifier: BSL-1.0
/*
 * pico4-tap-replay: the double-tap detector over a recording of the head accelerometer.
 *
 *   pico4-imu stream --rate 1000 --seconds 120 --csv taps.csv
 *   pico4-tap-replay [--set NAME=VALUE ...] [--trace FILE] [taps.csv]
 *
 * takes what pico4-imu stream --csv wrote (its accelerometer rows; the other sensors' are
 * skipped) or the same columns on standard input, and lists every spike the detector looks at
 * and whether it counts: knocks, the double taps they make, and the spikes it refuses. --set
 * changes a setting from pico4_tap.h's defaults (spike, release and quiet in m/s²; gravity
 * and envelope in s; width, rise, gap-min, gap-max and lockout in ms; ratio), to find them
 * for a recording. --trace writes what the detector makes of each sample: its time, the
 * linear acceleration, the motion envelope and the state.
 *
 * Runs on the headset and on a development host alike.
 */
#define _GNU_SOURCE
#include "pico4_tap.h"

#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

//! The Qualcomm timer the DSP stamps its samples with.
#define TICKS_PER_MS (19200000 / 1000)
#define MS_NS (1000 * 1000)
#define GAP_MS 10 //!< Between two samples: a dropout, reported.

//! The settings by the names --set takes, in the order of set_setting().
static const char *const SETTINGS[] = {"spike",    "release", "quiet",   "gravity", "envelope", "width",
                                       "rise",     "gap-min", "gap-max", "lockout", "ratio"};
#define SETTINGS_COUNT (sizeof(SETTINGS) / sizeof(SETTINGS[0]))

struct options
{
	struct pico4_tap_config cfg;
	const char *path; //!< The recording; NULL: standard input.
	FILE *trace;      //!< Where --trace writes; NULL: nowhere.
};

static int
usage(void)
{
	fprintf(stderr,
	        "usage: pico4-tap-replay [--set NAME=VALUE ...] [--trace FILE] [FILE.csv]\n"
	        "  NAME: spike, release, quiet (m/s²); gravity, envelope (s); width, rise, gap-min,\n"
	        "        gap-max, lockout (ms); ratio\n");
	return 2;
}

//! --set's NAME=VALUE into @p cfg; false if NAME is none of the settings.
static bool
set_setting(struct pico4_tap_config *cfg, const char *name, double value)
{
	size_t index = SETTINGS_COUNT;
	for (size_t i = 0; i < SETTINGS_COUNT; i++) {
		if (strcmp(name, SETTINGS[i]) == 0) {
			index = i;
		}
	}
	switch (index) {
	case 0: cfg->spike_mpss = (float)value; break;
	case 1: cfg->release_mpss = (float)value; break;
	case 2: cfg->quiet_mpss = (float)value; break;
	case 3: cfg->gravity_s = (float)value; break;
	case 4: cfg->envelope_s = (float)value; break;
	case 5: cfg->width_ns = (int64_t)(value * MS_NS); break;
	case 6: cfg->rise_ns = (int64_t)(value * MS_NS); break;
	case 7: cfg->gap_min_ns = (int64_t)(value * MS_NS); break;
	case 8: cfg->gap_max_ns = (int64_t)(value * MS_NS); break;
	case 9: cfg->lockout_ns = (int64_t)(value * MS_NS); break;
	case 10: cfg->peak_ratio = (float)value; break;
	default: return false;
	}
	return true;
}

static const char *
state_name(enum pico4_tap_state state)
{
	switch (state) {
	case PICO4_TAP_IDLE: return "idle";
	case PICO4_TAP_FIRST: return "first";
	case PICO4_TAP_SECOND: return "second";
	case PICO4_TAP_LAST: return "last";
	case PICO4_TAP_LOCKOUT: return "lockout";
	}
	return "?";
}

static bool
judging(enum pico4_tap_state state)
{
	return state == PICO4_TAP_FIRST || state == PICO4_TAP_LAST;
}

//! A spike the detector judges; closed once the state has left it.
struct spike
{
	bool open;         //!< The detector judges it.
	bool last;         //!< It may be the second tap.
	int64_t start_ns;  //!< Its first sample.
};

//! What the detector made of the spike that has just closed.
static void
report(const struct pico4_tap *tap, const struct spike *spike, int64_t now_ns, bool found)
{
	const double start_s = (double)spike->start_ns / 1e9;
	const double length_ms = (double)(now_ns - spike->start_ns) / 1e6;
	if (spike->last) {
		const double gap_ms = (double)(spike->start_ns - tap->first_ns) / 1e6;
		if (found) {
			printf("%10.3f s  knock   %6.1f m/s² %5.1f ms   double tap, %.0f ms after the first\n", start_s,
			       tap->peak, length_ms, gap_ms);
		} else {
			printf("%10.3f s  spike   %6.1f m/s² %5.1f ms   unlike the first: %.1f times its %.1f m/s²\n",
			       start_s, tap->peak, length_ms, tap->peak / tap->first_peak, tap->first_peak);
		}
	} else if (tap->state == PICO4_TAP_SECOND) {
		printf("%10.3f s  knock   %6.1f m/s² %5.1f ms\n", start_s, tap->first_peak, length_ms);
	} else {
		const char *why = length_ms > (double)tap->cfg.width_ns / 1e6 ? "too long" : "its top comes too late";
		printf("%10.3f s  spike   %6.1f m/s² %5.1f ms   %s\n", start_s, tap->peak, length_ms, why);
	}
}

int
main(int argc, char **argv)
{
	struct options opt = {.cfg = pico4_tap_default_config, .path = NULL, .trace = NULL};
	for (int i = 1; i < argc; i++) {
		if (strcmp(argv[i], "--trace") == 0) {
			if (++i >= argc) {
				return usage();
			}
			opt.trace = fopen(argv[i], "w");
			if (!opt.trace) {
				perror(argv[i]);
				return 1;
			}
		} else if (strcmp(argv[i], "--set") == 0) {
			if (++i >= argc) {
				return usage();
			}
			char *value = strchr(argv[i], '=');
			if (!value) {
				return usage();
			}
			*value++ = '\0';
			char *end = NULL;
			const double number = strtod(value, &end);
			if (*end != '\0' || end == value || !(number > 0.0) || !set_setting(&opt.cfg, argv[i], number)) {
				fprintf(stderr, "--set %s=%s: no such setting, or no number above zero\n", argv[i], value);
				return usage();
			}
		} else if (argv[i][0] == '-' && argv[i][1] != '\0') {
			return usage();
		} else if (!opt.path) {
			opt.path = argv[i];
		} else {
			return usage();
		}
	}
	if (!(opt.cfg.release_mpss < opt.cfg.spike_mpss) || !(opt.cfg.gap_min_ns < opt.cfg.gap_max_ns)) {
		fprintf(stderr, "--set: release lies under spike, and gap-min under gap-max\n");
		return usage();
	}

	FILE *in = stdin;
	if (opt.path && strcmp(opt.path, "-") != 0) {
		in = fopen(opt.path, "r");
		if (!in) {
			perror(opt.path);
			return 1;
		}
	}
	const char *name = opt.path ? opt.path : "standard input";

	struct pico4_tap tap;
	pico4_tap_init(&tap, &opt.cfg);
	struct spike spike = {.open = false, .last = false, .start_ns = 0};
	bool settled = false;
	int64_t first_ticks = -1, last_ns = 0, largest_gap_ns = 0;
	long samples = 0, dropouts = 0;
	int found = 0;
	const char *error = NULL;
	char *line = NULL;
	size_t capacity = 0;
	while (getline(&line, &capacity, in) > 0) {
		if (strncmp(line, "accel,", 6) != 0) {
			continue; // The header, and the other sensors' rows.
		}
		char *end = NULL;
		const long long ticks = strtoll(line + 6, &end, 10);
		float v[3];
		bool ok = *end == ',';
		for (int a = 0; ok && a < 3; a++) {
			v[a] = (float)strtod(end + 1, &end);
			ok = a < 2 ? *end == ',' : (*end == '\n' || *end == '\0');
		}
		if (!ok) {
			error = "a malformed accelerometer row";
			break;
		}
		if (first_ticks < 0) {
			first_ticks = ticks;
		}
		const int64_t ts_ns = (ticks - first_ticks) * MS_NS / TICKS_PER_MS;
		if (samples > 0 && ts_ns - last_ns > GAP_MS * MS_NS) {
			dropouts++;
			largest_gap_ns = ts_ns - last_ns > largest_gap_ns ? ts_ns - last_ns : largest_gap_ns;
		}
		last_ns = ts_ns;

		const enum pico4_tap_state before = tap.state;
		const bool tap_found = pico4_tap_feed(&tap, ts_ns, v[0], v[1], v[2]);
		const float lx = v[0] - tap.gravity[0], ly = v[1] - tap.gravity[1], lz = v[2] - tap.gravity[2];
		const float linear = sqrtf(lx * lx + ly * ly + lz * lz);
		if (opt.trace) {
			fprintf(opt.trace, "%lld,%.3f,%.3f,%s\n", (long long)ts_ns, linear, tap.envelope, state_name(tap.state));
		}

		if (judging(tap.state) && !judging(before)) {
			// Out of rest, or out of a first tap that waited too long for its second.
			spike.open = true;
			spike.last = tap.state == PICO4_TAP_LAST;
			spike.start_ns = ts_ns;
		} else if (!judging(tap.state) && judging(before) && spike.open) {
			spike.open = false;
			report(&tap, &spike, ts_ns, tap_found);
			found += tap_found;
		} else if (before == PICO4_TAP_SECOND && tap.state == PICO4_TAP_SECOND && settled && !tap.settled) {
			// Above the spike level again too soon after the first knock: its ringing or a bounce.
			printf("%10.3f s  bounce  %6.1f m/s²          %.0f ms after the first\n", (double)ts_ns / 1e9, linear,
			       (double)(ts_ns - tap.first_ns) / 1e6);
		} else if (before == PICO4_TAP_SECOND && tap.state == PICO4_TAP_IDLE) {
			printf("           dropped: the headset moves (%.1f m/s²)\n", tap.envelope);
		}
		settled = tap.settled;
		samples++;
	}
	free(line);
	if (ferror(in)) {
		perror(name);
		return 1;
	}
	if (error != NULL || samples == 0) {
		fprintf(stderr, "%s: %s\n", name, error != NULL ? error : "no accelerometer rows");
		return 1;
	}
	printf("{\n  \"samples\": %ld,\n  \"seconds\": %.3f,\n  \"rate_hz\": %.2f,\n  \"dropouts\": %ld,\n"
	       "  \"largest_gap_ms\": %.2f,\n  \"double_taps\": %d\n}\n",
	       samples, (double)last_ns / 1e9, samples > 1 ? (double)(samples - 1) * 1e9 / (double)last_ns : 0.0,
	       dropouts, (double)largest_gap_ns / 1e6, found);
	if (in != stdin) {
		fclose(in);
	}
	if (opt.trace) {
		if (fclose(opt.trace) != 0) {
			perror("--trace");
			return 1;
		}
	}
	return 0;
}
