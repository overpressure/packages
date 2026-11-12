// Copyright 2026, overpressure contributors.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  PICO 4 lens model: per-eye radial polynomial with chromatic aberration terms.
 *
 * The factory file OpticalCalibrationData.json gives, per eye, the scale F
 * (metres on the panel per tan-angle at the centre) and coefficients K0..K10 of
 * a polynomial in r². A panel point p (metres from the lens centre) is seen
 * at tan-angle t·s(|t|²) with t = p / F and s(x) = Σ Kᵢ xⁱ. Red and blue scale
 * that by (1 + CA0 + CA1·r² + CA4·r) and (1 + CA2 + CA3·r² + CA5·r); the six CA
 * terms come from the stock lens configuration. docs/xr.md has the details.
 *
 * @ingroup drv_pico4
 */

#pragma once

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define PICO4_LENS_COEFFS 11

//! Width and height of one eye's panel in metres (2160 px at 21.3 µm).
#define PICO4_LENS_EYE_M 0.046

struct pico4_lens
{
	double f;                    //!< Metres per tan-angle at the lens centre.
	double k[PICO4_LENS_COEFFS]; //!< Polynomial in r².
	double cx, cy;               //!< Lens centre on the panel, metres right and down of the panel centre.
};

struct pico4_lens_ca
{
	double ca[6];
};

/*!
 * Parse both eyes (0 left, 1 right) from OpticalCalibrationData.json text.
 * The lens centre offsets (lens_center_gos, mm) are optional.
 */
bool
pico4_lens_parse_optical(const char *json_text, struct pico4_lens out_lenses[2]);

//! Parse the six chromatic aberration terms from the stock lens configuration (JSON text).
bool
pico4_lens_parse_ca(const char *json_text, struct pico4_lens_ca *out_ca);

//! s(rsq) = Σ Kᵢ rsqⁱ.
double
pico4_lens_scale(const struct pico4_lens *lens, double rsq);

/*!
 * Tan-angles seen through the lens at panel point (x, y), metres from the panel
 * centre (x right, y down), for red, green and blue: out[channel][0 = x, 1 = y].
 * Returns false outside the radius the polynomial was fitted for (the panel's
 * inscribed circle); there the tan-angles grow steeply from their value on that
 * circle, so the image ends smoothly at the edge of the rendered view.
 */
bool
pico4_lens_tan(const struct pico4_lens *lens, const struct pico4_lens_ca *ca, double x, double y, double out[3][2]);

/*!
 * The inverse of pico4_lens_tan for one channel (0 red, 1 green, 2 blue): the panel
 * point (metres from the panel centre, x right, y down) seen at tan-angles (tx, ty).
 * The model is radial about the lens centre, so the point lies on the ray from the
 * centre in the direction of (tx, ty); within the fitted radius its distance is found
 * by bisection. Beyond it the inverse continues at its slope there rather than
 * inverting the steep continuation of pico4_lens_tan, so directions far outside the
 * view land far outside the panel. False for an unknown channel.
 */
bool
pico4_lens_inverse(const struct pico4_lens *lens,
                   const struct pico4_lens_ca *ca,
                   int channel,
                   double tx,
                   double ty,
                   double *out_x,
                   double *out_y);

/*!
 * Field of view of one eye in radians, from the panel edges through the green
 * channel: left and down negative, as in xrt_fov.
 */
void
pico4_lens_fov(const struct pico4_lens *lens,
               double *out_left,
               double *out_right,
               double *out_up,
               double *out_down);

#ifdef __cplusplus
}
#endif
