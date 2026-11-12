// Copyright 2026, overpressure contributors.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Factory calibration of the PICO 4 tracking cameras (see pico4_tcam.h).
 * @ingroup drv_pico4
 */

#include "pico4_tcam.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define NPARAM 6 //!< fx, fy, k1..k4; the principal point stays the factory one.

//! Parse "[a, b, ...]" after @p key (searched from @p from); returns the count read.
static int
parse_list(const char *text, const char *from_key, const char *key, double *out, int max)
{
	const char *p = text;
	if (from_key != NULL && (p = strstr(text, from_key)) == NULL) {
		return 0;
	}
	if ((p = strstr(p, key)) == NULL || (p = strchr(p, '[')) == NULL) {
		return 0;
	}
	p++;
	int n = 0;
	while (n < max) {
		char *end;
		double v = strtod(p, &end);
		if (end == p) {
			break;
		}
		out[n++] = v;
		p = end;
		while (*p == ' ' || *p == ',' || *p == '\n' || *p == '\r' || *p == '\t') {
			p++;
		}
		if (*p == ']') {
			break;
		}
	}
	return n;
}

static bool
has_value(const char *text, const char *key, const char *value)
{
	const char *p = strstr(text, key);
	if (p == NULL) {
		return false;
	}
	p += strlen(key);
	while (*p == ' ' || *p == ':') {
		p++;
	}
	return strncmp(p, value, strlen(value)) == 0;
}

bool
pico4_tcam_parse_yaml(const char *text, struct pico4_tcam_omni *out)
{
	double res[2], intr[5], dist[8] = {0};
	memset(out, 0, sizeof(*out));
	// PICO OS's loader takes "fisheye62" with k1..k6, p1, p2; any other model as radtan.
	out->distortion = has_value(text, "distortion_model", "fisheye62") ? PICO4_TCAM_FISHEYE62 : PICO4_TCAM_RADTAN;
	const int coefficients = out->distortion == PICO4_TCAM_FISHEYE62 ? 8 : 4;
	if (!has_value(text, "camera_model", "omni") ||
	    (out->distortion == PICO4_TCAM_RADTAN && !has_value(text, "distortion_model", "radtan")) ||
	    parse_list(text, NULL, "resolution:", res, 2) != 2 || parse_list(text, NULL, "intrinsics:", intr, 5) != 5 ||
	    parse_list(text, NULL, "distortion_coefficients:", dist, coefficients) != coefficients ||
	    parse_list(text, "T_BS:", "data:", out->T_BS, 16) != 16) {
		return false;
	}
	out->width = (int)res[0];
	out->height = (int)res[1];
	out->xi = intr[0];
	out->fu = intr[1];
	out->fv = intr[2];
	out->pu = intr[3];
	out->pv = intr[4];
	if (out->distortion == PICO4_TCAM_FISHEYE62) {
		out->k1 = dist[0];
		out->k2 = dist[1];
		out->k3 = dist[2];
		out->k4 = dist[3];
		out->k5 = dist[4];
		out->k6 = dist[5];
		out->p1 = dist[6];
		out->p2 = dist[7];
	} else {
		out->k1 = dist[0];
		out->k2 = dist[1];
		out->p1 = dist[2];
		out->p2 = dist[3];
	}
	return out->width > 0 && out->height > 0 && out->fu > 0 && out->fv > 0;
}

bool
pico4_tcam_omni_project(const struct pico4_tcam_omni *c, const double p[3], double uv[2])
{
	double n = sqrt(p[0] * p[0] + p[1] * p[1] + p[2] * p[2]);
	if (n <= 0) {
		return false;
	}
	double den = p[2] / n + c->xi;
	if (den <= 1e-9) {
		return false;
	}
	double mx = p[0] / n / den, my = p[1] / n / den;
	double r2 = mx * mx + my * my;
	double radial = 1 + c->k1 * r2 + c->k2 * r2 * r2;
	if (c->distortion == PICO4_TCAM_FISHEYE62) {
		// PICO OS's fisheye62: the angle of the omni-normalised point (not of the ray), its
		// polynomial in theta^2 up to theta^12, and the tangential terms on the undistorted point.
		double rho = sqrt(r2), t = atan(rho), t2 = t * t;
		double td = t * (1 + t2 * (c->k1 + t2 * (c->k2 + t2 * (c->k3 + t2 * (c->k4 + t2 * (c->k5 + t2 * c->k6))))));
		radial = rho > 1e-8 ? td / rho : 1;
	}
	double dx = mx * radial + 2 * c->p1 * mx * my + c->p2 * (r2 + 2 * mx * mx);
	double dy = my * radial + c->p1 * (r2 + 2 * my * my) + 2 * c->p2 * mx * my;
	uv[0] = c->fu * dx + c->pu;
	uv[1] = c->fv * dy + c->pv;
	return true;
}

void
pico4_tcam_kb4_project(const struct pico4_tcam_kb4 *c, const double p[3], double uv[2])
{
	double r = sqrt(p[0] * p[0] + p[1] * p[1]);
	if (r < 1e-12) {
		uv[0] = c->cx;
		uv[1] = c->cy;
		return;
	}
	double t = atan2(r, p[2]), t2 = t * t;
	double d = t * (1 + t2 * (c->k[0] + t2 * (c->k[1] + t2 * (c->k[2] + t2 * c->k[3]))));
	uv[0] = c->fx * d * p[0] / r + c->cx;
	uv[1] = c->fy * d * p[1] / r + c->cy;
}

struct sample
{
	double p[3], uv[2];
};

/*!
 * Rays on a polar grid, kept while the factory projection stays inside the
 * image and moves outwards (the radtan polynomial folds back far out).
 */
static int
make_samples(const struct pico4_tcam_omni *c, struct sample *s, int max)
{
	const int azimuths = 72, steps = 120;
	int n = 0;
	for (int a = 0; a < azimuths; a++) {
		double phi = 2 * M_PI * a / azimuths;
		double last_r = -1;
		for (int i = 1; i <= steps && n < max; i++) {
			double theta = (M_PI * 0.75) * i / steps; // up to 135 degrees off axis
			double p[3] = {sin(theta) * cos(phi), sin(theta) * sin(phi), cos(theta)};
			double uv[2];
			if (!pico4_tcam_omni_project(c, p, uv)) {
				break;
			}
			double r = hypot(uv[0] - c->pu, uv[1] - c->pv);
			if (r <= last_r || uv[0] < 0 || uv[1] < 0 || uv[0] > c->width || uv[1] > c->height) {
				break;
			}
			last_r = r;
			memcpy(s[n].p, p, sizeof(p));
			memcpy(s[n].uv, uv, sizeof(uv));
			n++;
		}
	}
	return n;
}

static void
unpack(const double x[NPARAM], double cx, double cy, struct pico4_tcam_kb4 *m)
{
	m->fx = x[0];
	m->fy = x[1];
	m->cx = cx;
	m->cy = cy;
	for (int i = 0; i < 4; i++) {
		m->k[i] = x[2 + i];
	}
}

static double
cost(const double x[NPARAM], double cx, double cy, const struct sample *s, int n)
{
	struct pico4_tcam_kb4 m;
	double sum = 0;
	unpack(x, cx, cy, &m);
	for (int i = 0; i < n; i++) {
		double uv[2];
		pico4_tcam_kb4_project(&m, s[i].p, uv);
		sum += (uv[0] - s[i].uv[0]) * (uv[0] - s[i].uv[0]) + (uv[1] - s[i].uv[1]) * (uv[1] - s[i].uv[1]);
	}
	return sum;
}

//! Solve A x = b (NPARAM x NPARAM) by Gaussian elimination with partial pivoting.
static bool
solve(double a[NPARAM][NPARAM], double b[NPARAM], double x[NPARAM])
{
	for (int c = 0; c < NPARAM; c++) {
		int piv = c;
		for (int r = c + 1; r < NPARAM; r++) {
			if (fabs(a[r][c]) > fabs(a[piv][c])) {
				piv = r;
			}
		}
		if (fabs(a[piv][c]) < 1e-300) {
			return false;
		}
		for (int k = 0; k < NPARAM; k++) {
			double t = a[c][k];
			a[c][k] = a[piv][k];
			a[piv][k] = t;
		}
		double t = b[c];
		b[c] = b[piv];
		b[piv] = t;
		for (int r = c + 1; r < NPARAM; r++) {
			double f = a[r][c] / a[c][c];
			for (int k = c; k < NPARAM; k++) {
				a[r][k] -= f * a[c][k];
			}
			b[r] -= f * b[c];
		}
	}
	for (int r = NPARAM - 1; r >= 0; r--) {
		double sum = b[r];
		for (int k = r + 1; k < NPARAM; k++) {
			sum -= a[r][k] * x[k];
		}
		x[r] = sum / a[r][r];
	}
	return true;
}

bool
pico4_tcam_fit_kb4(const struct pico4_tcam_omni *c, struct pico4_tcam_kb4 *out)
{
	const int max = 72 * 120;
	struct sample *s = calloc(max, sizeof(*s));
	if (s == NULL) {
		return false;
	}
	int n = make_samples(c, s, max);
	// Near the axis the omni model scales tan-angles by f / (1 + xi).
	double x[NPARAM] = {c->fu / (1 + c->xi), c->fv / (1 + c->xi), 0, 0, 0, 0};
	double lambda = 1e-3, f = cost(x, c->pu, c->pv, s, n);
	bool ok = n >= 100;

	for (int it = 0; ok && it < 200; it++) {
		double jtj[NPARAM][NPARAM] = {{0}}, jtr[NPARAM] = {0};
		struct pico4_tcam_kb4 m;
		unpack(x, c->pu, c->pv, &m);
		for (int i = 0; i < n; i++) {
			double uv[2], jac[2][NPARAM];
			pico4_tcam_kb4_project(&m, s[i].p, uv);
			for (int k = 0; k < NPARAM; k++) {
				double xk[NPARAM], h = 1e-7 * fmax(1.0, fabs(x[k])), uvk[2];
				struct pico4_tcam_kb4 mk;
				memcpy(xk, x, sizeof(xk));
				xk[k] += h;
				unpack(xk, c->pu, c->pv, &mk);
				pico4_tcam_kb4_project(&mk, s[i].p, uvk);
				jac[0][k] = (uvk[0] - uv[0]) / h;
				jac[1][k] = (uvk[1] - uv[1]) / h;
			}
			double r[2] = {uv[0] - s[i].uv[0], uv[1] - s[i].uv[1]};
			for (int a = 0; a < NPARAM; a++) {
				for (int b = 0; b < NPARAM; b++) {
					jtj[a][b] += jac[0][a] * jac[0][b] + jac[1][a] * jac[1][b];
				}
				jtr[a] -= jac[0][a] * r[0] + jac[1][a] * r[1];
			}
		}
		bool improved = false;
		for (int tries = 0; tries < 10 && !improved; tries++) {
			double a[NPARAM][NPARAM], b[NPARAM], d[NPARAM], xn[NPARAM];
			memcpy(a, jtj, sizeof(a));
			memcpy(b, jtr, sizeof(b));
			for (int k = 0; k < NPARAM; k++) {
				a[k][k] *= 1 + lambda;
			}
			if (!solve(a, b, d)) {
				lambda *= 10;
				continue;
			}
			for (int k = 0; k < NPARAM; k++) {
				xn[k] = x[k] + d[k];
			}
			double fn = cost(xn, c->pu, c->pv, s, n);
			if (fn < f) {
				bool converged = f - fn < 1e-10 * f;
				memcpy(x, xn, sizeof(x));
				lambda = fmax(lambda / 10, 1e-9);
				f = fn;
				improved = true;
				if (converged) {
					it = 1000;
				}
			} else {
				lambda *= 10;
			}
		}
		if (!improved) {
			break;
		}
	}

	unpack(x, c->pu, c->pv, out);
	out->samples = n;
	out->rms_px = n > 0 ? sqrt(f / n) : INFINITY;
	out->max_px = 0;
	for (int i = 0; i < n; i++) {
		double uv[2];
		pico4_tcam_kb4_project(out, s[i].p, uv);
		out->max_px = fmax(out->max_px, hypot(uv[0] - s[i].uv[0], uv[1] - s[i].uv[1]));
	}
	free(s);
	return ok && isfinite(out->rms_px) && out->fx > 0 && out->fy > 0;
}

void
pico4_tcam_rotate_180(struct pico4_tcam_kb4 *kb4, double T_BS[16], int width, int height)
{
	// (u, v) -> (w - 1 - u, h - 1 - v) is the camera turned about its z axis: x and y negate.
	kb4->cx = (width - 1) - kb4->cx;
	kb4->cy = (height - 1) - kb4->cy;
	for (int row = 0; row < 3; row++) {
		T_BS[row * 4 + 0] = -T_BS[row * 4 + 0];
		T_BS[row * 4 + 1] = -T_BS[row * 4 + 1];
	}
}

int
pico4_tcam_upright(struct pico4_tcam_kb4 *kb4, double T_BS[16], int width, int height, const double down_body[3],
                   int *out_w, int *out_h)
{
	// The image's down axis after each turn, in the body frame: y, x, -y, -x of the camera.
	double x_dot = 0, y_dot = 0;
	for (int row = 0; row < 3; row++) {
		x_dot += T_BS[row * 4 + 0] * down_body[row];
		y_dot += T_BS[row * 4 + 1] * down_body[row];
	}
	const double dots[4] = {y_dot, x_dot, -y_dot, -x_dot};
	int turn = 0;
	for (int t = 1; t < 4; t++) {
		turn = dots[t] > dots[turn] ? t : turn;
	}
	*out_w = turn % 2 == 0 ? width : height;
	*out_h = turn % 2 == 0 ? height : width;
	if (turn == 2) {
		pico4_tcam_rotate_180(kb4, T_BS, width, height);
		return turn;
	}
	if (turn == 0) {
		return turn;
	}
	// Turn 1: u' = h - 1 - v, v' = u; the new x axis is -y, the new y axis x. Turn 3 the mirror.
	double fx = kb4->fx, cx = kb4->cx;
	kb4->fx = kb4->fy;
	kb4->fy = fx;
	kb4->cx = turn == 1 ? (height - 1) - kb4->cy : kb4->cy;
	kb4->cy = turn == 1 ? cx : (width - 1) - cx;
	for (int row = 0; row < 3; row++) {
		double x = T_BS[row * 4 + 0], y = T_BS[row * 4 + 1];
		T_BS[row * 4 + 0] = turn == 1 ? -y : y;
		T_BS[row * 4 + 1] = turn == 1 ? x : -x;
	}
	return turn;
}

bool
pico4_tcam_upside_down(const double a_T_BS[16], const double b_T_BS[16])
{
	// Column 1 of the rotation: each image's "down" direction in the body frame.
	double dot = a_T_BS[1] * b_T_BS[1] + a_T_BS[5] * b_T_BS[5] + a_T_BS[9] * b_T_BS[9];
	return dot < 0;
}
