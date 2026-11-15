// SPDX-License-Identifier: BSL-1.0
/*
 * pico4-vrlink-compat against a fake OpenVR: built once with -DFAKE_OPENVR as the library the layer
 * finds behind it (RTLD_NEXT, as vrlink's libopenvr_api), once as the test, linked to the layer
 * first. Device 0 is the headset, 1 and 2 our left and right controllers.
 */
#define _GNU_SOURCE

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>

void *VR_GetGenericInterface(const char *version, int32_t *error);

#ifdef FAKE_OPENVR

//! Each table on a page of its own: the layer write-protects the page of the slot it replaces.
static void *g_system_table[512] __attribute__((aligned(4096)));
static void *g_paths_table[512] __attribute__((aligned(4096)));
static void *g_client_table[512] __attribute__((aligned(4096)));
static void *g_system = g_system_table;
static void *g_paths = g_paths_table;
static void *g_client = g_client_table;
//! The path the last StringToHandle got.
char g_last_path[256];

static uint32_t
get_string(void *self, uint32_t device, int32_t prop, char *value, uint32_t size, int32_t *error)
{
	(void)self;
	static const char *const types[] = {"", "pico_controller", "pico_controller"};
	static const char *const models[] = {"PICO 4", "PICO 4 left controller", "PICO 4 right controller"};
	const char *text = NULL;
	if (device < 3 && prop == 7000) {
		text = types[device];
	} else if (device < 3 && prop == 1001) {
		text = models[device];
	} else if (device < 3 && prop == 1002) {
		text = "PICO 4 serial";
	}
	*error = text == NULL ? 4 : 0;
	if (text == NULL) {
		return 0;
	}
	snprintf(value, size, "%s", text);
	return (uint32_t)strlen(text) + 1;
}

static int32_t
string_to_handle(void *self, uint64_t *handle, const char *path)
{
	(void)self;
	snprintf(g_last_path, sizeof(g_last_path), "%s", path);
	*handle = 1;
	return 0;
}

/*
 * IVRClientInternal's driver pose: an 8-byte header, then DriverPose_t, whose driver-from-head
 * transform is a quarter turn about z and 0.1 m along x.
 */
static uint64_t
get_driver_pose(void *self, uint32_t device, uint8_t *out)
{
	(void)self;
	double pose[16] = {0};
	pose[1] = 1;                // world-from-driver rotation w
	pose[8] = 0.70710678118655; // driver-from-head rotation w
	pose[11] = 0.70710678118655; // ... z
	pose[12] = 0.1;             // driver-from-head translation x
	memcpy(out + 8, pose, sizeof(pose));
	return device;
}

void *
VR_GetGenericInterface(const char *version, int32_t *error)
{
	*error = 0;
	if (g_system_table[28] == NULL) { // once: the layer write-protects the tables' pages then
		g_system_table[28] = (void *)get_string;
		g_paths_table[2] = (void *)string_to_handle;
		g_client_table[12] = (void *)get_driver_pose;
	}
	if (strcmp(version, "IVRSystem_026") == 0) {
		return &g_system;
	}
	if (strcmp(version, "IVRClientInternal_XXX") == 0) {
		return &g_client;
	}
	return strcmp(version, "IVRPaths_002") == 0 ? &g_paths : NULL;
}

#else

typedef uint32_t (*get_string_fn)(void *, uint32_t, int32_t, char *, uint32_t, int32_t *);
typedef int32_t (*to_handle_fn)(void *, uint64_t *, const char *);
typedef uint64_t (*get_pose_fn)(void *, uint32_t, uint8_t *);
extern char g_last_path[256];
static int g_failed;

static void
expect(const char *what, const char *got, const char *want)
{
	if (strcmp(got, want) != 0) {
		fprintf(stderr, "FAIL %s: %s, not %s\n", what, got, want);
		g_failed = 1;
	}
}

static const char *
prop(void *system, uint32_t device, int32_t id, uint32_t size)
{
	static char value[1024];
	int32_t error = 0;
	value[0] = '\0';
	(*(get_string_fn **)system)[28](system, device, id, value, size, &error);
	return value;
}

static const char *
path(void *paths, const char *frame)
{
	uint64_t handle = 0;
	(*(to_handle_fn **)paths)[2](paths, &handle, frame);
	return g_last_path;
}

int
main(void)
{
	// The calibration the layer reads, made up: grip 30 degrees about x from the aim, both hands.
	char dir[] = "/tmp/pico4-vrlink-test-XXXXXX";
	if (mkdtemp(dir) == NULL) {
		perror("mkdtemp");
		return 1;
	}
	char file[128];
	snprintf(file, sizeof(file), "%s/controllers", dir);
	mkdir(file, 0755);
	snprintf(file, sizeof(file), "%s/controllers/aim-to-grip.json", dir);
	FILE *f = fopen(file, "w");
	static const char hand[] = "{\"position\": [0, 0.01, 0.05], \"orientation\": [0.96592583, 0.25881905, 0, 0]}";
	fprintf(f, "{\"left\": %s, \"right\": %s}", hand, hand);
	fclose(f);
	setenv("PICO4_CALIBRATION_DIR", dir, 1);

	int32_t error = 0;
	void *system = VR_GetGenericInterface("IVRSystem_026", &error);
	void *paths = VR_GetGenericInterface("IVRPaths_002", &error);

	// The static properties (128-byte reads): Valve's PICO 4 controllers; the headset as it is.
	expect("left type", prop(system, 1, 7000, 128), "pico_controller");
	expect("left model", prop(system, 1, 1001, 128), "PICO 4 (Left Controller)");
	expect("right model", prop(system, 2, 1001, 128), "PICO 4 (Right Controller)");
	expect("right render model", prop(system, 2, 1003, 128), "pico_4_controller_right");
	expect("left serial", prop(system, 1, 1002, 128), "VRLINKP4_Controller_Left");
	expect("left profile", prop(system, 1, 1037, 128), "{vrlink}/input/pico_controller_profile.json");
	expect("headset model", prop(system, 0, 1001, 128), "PICO 4");
	// The input packing (1024 bytes) packs only the Frame's controllers.
	expect("packing type", prop(system, 1, 7000, 1024), "frame_controller");
	expect("packing serial", prop(system, 1, 1002, 1024), "PICO 4 serial");

	// Each Frame slot the PICO layout reads carries that slot's PICO input.
	expect("left click 1", path(paths, "/user/hand/left/input/dpad_right/click"), "/user/hand/left/input/y/click");
	expect("left click 2", path(paths, "/user/hand/left/input/dpad_left/click"), "/user/hand/left/input/system/click");
	expect("right click 2", path(paths, "/user/hand/right/input/x/click"), "/user/hand/right/input/none/click");
	expect("right touch 2", path(paths, "/user/hand/right/input/x/touch"), "/user/hand/right/input/trigger/touch");
	expect("analog 0", path(paths, "/user/hand/right/input/thumbstick/x"), "/user/hand/right/input/trigger/value");
	expect("analog 2", path(paths, "/user/hand/left/input/trigger/value"), "/user/hand/left/input/joystick/y");
	expect("right click 0", path(paths, "/user/hand/right/input/a/click"), "/user/hand/right/input/a/click");
	expect("analog 3", path(paths, "/user/hand/left/input/grip/value"), "/user/hand/left/input/grip/value");

	// Poses: a made-up aim-to-grip (0, 0.01, 0.05), 30 degrees about x, then Valve's offset, composed
	// onto the driver-from-head transform of our controllers only.
	void *client = VR_GetGenericInterface("IVRClientInternal_XXX", &error);
	const double expected[2][7] = {
	    {0.0863055, -0.007, -0.0587991, 0.6963642, 0.1227878, 0.1227878, 0.6963642}, // a controller
	    {0.1, 0, 0, 0.70710678, 0, 0, 0.70710678},                                     // the headset
	};
	for (uint32_t device = 0; device < 3; device++) {
		uint8_t out[8 + 16 * sizeof(double)];
		(*(get_pose_fn **)client)[12](client, device, out);
		double pose[16];
		memcpy(pose, out + 8, sizeof(pose));
		const double *want = expected[device == 0];
		const double got[7] = {pose[12], pose[13], pose[14], pose[8], pose[9], pose[10], pose[11]};
		for (int i = 0; i < 7; i++) {
			if (fabs(got[i] - want[i]) > 1e-6) {
				fprintf(stderr, "FAIL device %u driver-from-head [%d]: %.7f, not %.7f\n", device, i, got[i], want[i]);
				g_failed = 1;
			}
		}
	}
	if (!g_failed) {
		printf("pico4-vrlink-compat: ok\n");
	}
	return g_failed;
}

#endif
