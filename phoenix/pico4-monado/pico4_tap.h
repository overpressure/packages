// Copyright 2026, overpressure contributors.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  PICO 4: the wearer's double tap on the headset, from its accelerometer (see pico4_tap.c).
 *
 * On PICO OS a double tap on the headset's side switches the passthrough. This
 * finds it in the head IMU's acceleration, whatever way the headset is turned:
 * gravity is taken out, and what remains must show two short sharp spikes close
 * together while the headset is otherwise at rest. Walking, shaking the head,
 * taking the headset off and setting it down give none.
 *
 * Plain C without Monado's math library, so that it is tested on its own.
 *
 * @ingroup drv_pico4
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

//! The detector's settings; pico4_tap_default_config holds the values the driver uses.
struct pico4_tap_config
{
	float gravity_s;     //!< Time constant of the gravity estimate, s.
	float envelope_s;   //!< Time constant of the motion envelope, s.
	float spike_mpss;   //!< Linear acceleration that starts a spike, m/s².
	float release_mpss; //!< Below which a spike has ended, m/s².
	float quiet_mpss;   //!< Motion envelope above which the headset moves: a tap waiting for its second ends.
	int64_t width_ns;   //!< A spike longer than this is a sustained acceleration, not a tap.
	int64_t rise_ns;    //!< Its largest value comes this early, counted from its first sample.
	int64_t gap_min_ns; //!< A second spike sooner than this after the first's start is its ringing.
	int64_t gap_max_ns; //!< A second spike must start within this of the first's start.
	int64_t lockout_ns; //!< After a double tap, or a too long spike, nothing counts for this long.
	float peak_ratio;   //!< The two spikes' peaks differ at most by this factor.
};

enum pico4_tap_state
{
	PICO4_TAP_IDLE = 0,      //!< Waiting for a spike.
	PICO4_TAP_FIRST = 1,     //!< A spike that may be the first tap.
	PICO4_TAP_SECOND = 2,    //!< The first tap ended; waiting for the second.
	PICO4_TAP_LAST = 3,      //!< A spike that may be the second tap.
	PICO4_TAP_LOCKOUT = 4,   //!< Nothing counts until until_ns.
};

struct pico4_tap
{
	struct pico4_tap_config cfg;
	float gravity[3];   //!< Gravity in the accelerometer's frame, low-passed while nothing spikes.
	float envelope;    //!< The linear acceleration's magnitude, low-passed: how much the headset moves.
	enum pico4_tap_state state;
	bool started;      //!< A sample was taken; the next one is relative to it.
	bool settled;      //!< After the first tap: the signal has come down since the last spike.
	int64_t last_ns;   //!< The last sample's time.
	int64_t start_ns;  //!< The spike's first sample's time.
	int64_t first_ns;  //!< The first tap's start.
	int64_t until_ns;  //!< The lockout's end.
	float peak;        //!< The spike's largest magnitude so far, m/s².
	float first_peak; //!< The first tap's largest magnitude, m/s².
};

extern const struct pico4_tap_config pico4_tap_default_config;

//! A detector with the settings @p cfg (pico4_tap_default_config for the driver's).
void
pico4_tap_init(struct pico4_tap *t, const struct pico4_tap_config *cfg);

//! Forget the samples so far: the next sample starts from rest, as after a suspend or a gap.
void
pico4_tap_reset(struct pico4_tap *t);

/*!
 * The acceleration at @p ts_ns, m/s² in any fixed frame and at a steady rate of about 1 kHz.
 * Returns true once per double tap, at the end of its second spike. A sample more than
 * gap_max_ns after the last, or not after it, starts the detector over.
 */
bool
pico4_tap_feed(struct pico4_tap *t, int64_t ts_ns, float x, float y, float z);

#ifdef __cplusplus
}
#endif
