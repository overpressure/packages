// SPDX-License-Identifier: BSL-1.0
/*
 * pico4-openxr-pose: read the head pose through the OpenXR loader and the
 * active runtime, without graphics (XR_MND_headless), and print a JSON summary.
 *
 *   pico4-openxr-pose [SECONDS] [INTERVAL_MS]
 *
 * VIEW is located in LOCAL every INTERVAL_MS at the current time. The summary
 * (tilt measured against STAGE, the gravity-aligned tracking origin)
 * reports the system, how many locations had a valid/tracked orientation, the
 * rotation between the first (after one second) and the last pose, the heading
 * change about the up axis (+Y) and its rate, and how far the view's up vector
 * is from vertical. Exit status 0 means every location after the first second
 * had a valid orientation.
 */

#define _DEFAULT_SOURCE // clock_gettime, nanosleep, M_PI under -std=c11
#define XR_USE_TIMESPEC
#include <time.h>

#include <openxr/openxr.h>
#include <openxr/openxr_platform.h>

#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#ifndef XR_MND_HEADLESS_EXTENSION_NAME
#define XR_MND_HEADLESS_EXTENSION_NAME "XR_MND_headless"
#endif

//! The JSON report; stdout itself goes to stderr while the in-process runtime runs.
static FILE *g_report;

static FILE *
report(void)
{
	return g_report != NULL ? g_report : stdout;
}

/*!
 * The runtime runs in this process and also writes to stdout (Basalt, for one).
 * Keep the real stdout for the report and point file descriptor 1 at stderr.
 */
static void
report_open(void)
{
	fflush(stdout);
	int fd = dup(STDOUT_FILENO);
	FILE *f = fd >= 0 ? fdopen(fd, "w") : NULL;
	if (f != NULL && dup2(STDERR_FILENO, STDOUT_FILENO) >= 0) {
		g_report = f;
	}
}

static void
fail(const char *what, XrResult res)
{
	fprintf(report(), "{\"schema\": 1, \"status\": \"fail\", \"error\": \"%s failed (%d)\"}\n", what, (int)res);
	exit(1);
}

#define CHECK(call)                                                                                                    \
	do {                                                                                                           \
		XrResult r_ = (call);                                                                                  \
		if (XR_FAILED(r_)) {                                                                                   \
			fail(#call, r_);                                                                               \
		}                                                                                                      \
	} while (0)

struct quat
{
	double x, y, z, w;
};

static struct quat
qmul(struct quat a, struct quat b)
{
	return (struct quat){
	    a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
	    a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
	    a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
	    a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
	};
}

static struct quat
qconj(struct quat q)
{
	return (struct quat){-q.x, -q.y, -q.z, q.w};
}

int
main(int argc, char **argv)
{
	report_open();
	double seconds = argc > 1 ? atof(argv[1]) : 5.0;
	int interval_ms = argc > 2 ? atoi(argv[2]) : 10;

	const char *extensions[] = {XR_MND_HEADLESS_EXTENSION_NAME, XR_KHR_CONVERT_TIMESPEC_TIME_EXTENSION_NAME};
	XrInstanceCreateInfo ici = {.type = XR_TYPE_INSTANCE_CREATE_INFO};
	strcpy(ici.applicationInfo.applicationName, "pico4-openxr-pose");
	ici.applicationInfo.apiVersion = XR_API_VERSION_1_0;
	ici.enabledExtensionCount = 2;
	ici.enabledExtensionNames = extensions;
	XrInstance instance;
	CHECK(xrCreateInstance(&ici, &instance));

	XrInstanceProperties ip = {.type = XR_TYPE_INSTANCE_PROPERTIES};
	CHECK(xrGetInstanceProperties(instance, &ip));

	PFN_xrConvertTimespecTimeToTimeKHR to_xr_time;
	CHECK(xrGetInstanceProcAddr(instance, "xrConvertTimespecTimeToTimeKHR", (PFN_xrVoidFunction *)&to_xr_time));

	XrSystemGetInfo sgi = {.type = XR_TYPE_SYSTEM_GET_INFO, .formFactor = XR_FORM_FACTOR_HEAD_MOUNTED_DISPLAY};
	XrSystemId system;
	CHECK(xrGetSystem(instance, &sgi, &system));
	XrSystemProperties sp = {.type = XR_TYPE_SYSTEM_PROPERTIES};
	CHECK(xrGetSystemProperties(instance, system, &sp));

	// Headless: no graphics binding.
	XrSessionCreateInfo sci = {.type = XR_TYPE_SESSION_CREATE_INFO, .systemId = system};
	XrSession session;
	CHECK(xrCreateSession(instance, &sci, &session));

	XrReferenceSpaceCreateInfo rci = {.type = XR_TYPE_REFERENCE_SPACE_CREATE_INFO};
	rci.poseInReferenceSpace.orientation.w = 1.0f;
	XrSpace local, view, stage;
	rci.referenceSpaceType = XR_REFERENCE_SPACE_TYPE_LOCAL;
	CHECK(xrCreateReferenceSpace(session, &rci, &local));
	rci.referenceSpaceType = XR_REFERENCE_SPACE_TYPE_VIEW;
	CHECK(xrCreateReferenceSpace(session, &rci, &view));
	rci.referenceSpaceType = XR_REFERENCE_SPACE_TYPE_STAGE;
	CHECK(xrCreateReferenceSpace(session, &rci, &stage));

	struct timespec start, now;
	clock_gettime(CLOCK_MONOTONIC, &start);
	long located = 0, valid = 0, tracked = 0, invalid_after_warmup = 0;
	struct quat first = {0, 0, 0, 1}, last = {0, 0, 0, 1};
	bool have_first = false;
	double max_tilt = 0, tilt = 0, first_elapsed = 0, last_elapsed = 0;
	// Position (6DoF): samples with a tracked position, their path and spread.
	long pos_tracked = 0;
	XrVector3f pos_first = {0}, pos_prev = {0}, pos_last = {0};
	double path_m = 0, max_dist_m = 0;

	for (;;) {
		XrEventDataBuffer ev = {.type = XR_TYPE_EVENT_DATA_BUFFER};
		while (xrPollEvent(instance, &ev) == XR_SUCCESS) {
			if (ev.type == XR_TYPE_EVENT_DATA_SESSION_STATE_CHANGED &&
			    ((XrEventDataSessionStateChanged *)&ev)->state == XR_SESSION_STATE_READY) {
				XrSessionBeginInfo sbi = {.type = XR_TYPE_SESSION_BEGIN_INFO,
				                          .primaryViewConfigurationType =
				                              XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO};
				CHECK(xrBeginSession(session, &sbi));
			}
			ev = (XrEventDataBuffer){.type = XR_TYPE_EVENT_DATA_BUFFER};
		}

		clock_gettime(CLOCK_MONOTONIC, &now);
		double elapsed = (now.tv_sec - start.tv_sec) + (now.tv_nsec - start.tv_nsec) / 1e9;
		if (elapsed >= seconds) {
			break;
		}
		XrTime t;
		CHECK(to_xr_time(instance, &now, &t));
		XrSpaceLocation loc = {.type = XR_TYPE_SPACE_LOCATION};
		CHECK(xrLocateSpace(view, local, t, &loc));
		located++;
		bool ok = (loc.locationFlags & XR_SPACE_LOCATION_ORIENTATION_VALID_BIT) != 0;
		if (ok) {
			valid++;
			if (loc.locationFlags & XR_SPACE_LOCATION_ORIENTATION_TRACKED_BIT) {
				tracked++;
			}
			if (loc.locationFlags & XR_SPACE_LOCATION_POSITION_TRACKED_BIT) {
				XrVector3f p = loc.pose.position;
				if (pos_tracked == 0) {
					pos_first = pos_prev = p;
				}
				path_m += sqrt((p.x - pos_prev.x) * (p.x - pos_prev.x) + (p.y - pos_prev.y) * (p.y - pos_prev.y) +
				               (p.z - pos_prev.z) * (p.z - pos_prev.z));
				double d = sqrt((p.x - pos_first.x) * (p.x - pos_first.x) +
				                (p.y - pos_first.y) * (p.y - pos_first.y) +
				                (p.z - pos_first.z) * (p.z - pos_first.z));
				max_dist_m = d > max_dist_m ? d : max_dist_m;
				pos_prev = pos_last = p;
				pos_tracked++;
			}
			XrQuaternionf o = loc.pose.orientation;
			last = (struct quat){o.x, o.y, o.z, o.w};
			last_elapsed = elapsed;
			if (!have_first && elapsed >= 1.0) {
				first = last;
				first_elapsed = elapsed;
				have_first = true;
			}
			// Up vector of the view in STAGE: rotate (0, 1, 0). LOCAL keeps only the
			// start yaw of a tilted head and need not be exactly level.
			XrSpaceLocation sl = {.type = XR_TYPE_SPACE_LOCATION};
			CHECK(xrLocateSpace(view, stage, t, &sl));
			if (sl.locationFlags & XR_SPACE_LOCATION_ORIENTATION_VALID_BIT) {
				o = sl.pose.orientation;
			}
			double ux = 2 * (o.x * o.y - o.w * o.z);
			double uy = 1 - 2 * (o.x * o.x + o.z * o.z);
			double uz = 2 * (o.y * o.z + o.w * o.x);
			tilt = acos(fmin(1.0, uy / sqrt(ux * ux + uy * uy + uz * uz))) * 180.0 / M_PI;
			if (tilt > max_tilt) {
				max_tilt = tilt;
			}
		} else if (elapsed >= 1.0) {
			invalid_after_warmup++;
		}
		struct timespec sleep = {0, interval_ms * 1000000L};
		nanosleep(&sleep, NULL);
	}

	struct quat rel = qmul(last, qconj(first));
	double rotation = 2 * acos(fmin(1.0, fabs(rel.w))) * 180.0 / M_PI;
	double heading = 2 * atan2(rel.y, rel.w) * 180.0 / M_PI;
	if (heading > 180) {
		heading -= 360;
	} else if (heading < -180) {
		heading += 360;
	}
	bool pass = valid > 0 && invalid_after_warmup == 0 && have_first;
	double span = last_elapsed - first_elapsed;

	fprintf(report(), "{\n  \"schema\": 1,\n  \"runtime\": \"%s\",\n  \"runtime_version\": \"%u.%u.%u\",\n", ip.runtimeName,
	                  XR_VERSION_MAJOR(ip.runtimeVersion), XR_VERSION_MINOR(ip.runtimeVersion),
	                  XR_VERSION_PATCH(ip.runtimeVersion));
	fprintf(report(), "  \"system\": \"%s\",\n  \"orientation_tracking\": %s,\n  \"position_tracking\": %s,\n",
	                  sp.systemName, sp.trackingProperties.orientationTracking ? "true" : "false",
	                  sp.trackingProperties.positionTracking ? "true" : "false");
	fprintf(report(), "  \"seconds\": %.1f,\n  \"located\": %ld,\n  \"orientation_valid\": %ld,\n  \"orientation_tracked\": %ld,\n",
	                  seconds, located, valid, tracked);
	fprintf(report(), "  \"invalid_after_warmup\": %ld,\n  \"rotation_deg\": %.3f,\n  \"heading_change_deg\": %.3f,\n",
	                  invalid_after_warmup, rotation, heading);
	fprintf(report(), "  \"heading_rate_deg_per_s\": %.4f,\n  \"final_tilt_from_vertical_deg\": %.3f,\n",
	                  span > 0 ? heading / span : 0.0, tilt);
	fprintf(report(), "  \"max_tilt_from_vertical_deg\": %.3f,\n  \"last_orientation_xyzw\": [%.6f, %.6f, %.6f, %.6f],\n",
	                  max_tilt, last.x, last.y, last.z, last.w);
	fprintf(report(), "  \"position_tracked\": %ld,\n  \"position_path_m\": %.4f,\n  \"position_max_distance_m\": %.4f,\n",
	                  pos_tracked, path_m, max_dist_m);
	fprintf(report(), "  \"last_position_m\": [%.4f, %.4f, %.4f],\n", pos_last.x, pos_last.y, pos_last.z);
	fprintf(report(), "  \"status\": \"%s\"\n}\n", pass ? "pass" : "fail");

	xrDestroySpace(stage);
	xrDestroySpace(view);
	xrDestroySpace(local);
	xrDestroySession(session);
	xrDestroyInstance(instance);
	return pass ? 0 : 1;
}
