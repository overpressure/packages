// SPDX-License-Identifier: BSL-1.0
/*
 * pico4-vrlink-compat: preloaded into SteamVR's Steam Link client (vrlink, through its
 * run_vrlink.sh; docs/session.md#steam-link) so that the PC sees the PICO 4's controllers as
 * PICO 4 controllers, as Valve's own Steam Link client on a PICO headset sends them.
 */
#define _GNU_SOURCE

#include <dlfcn.h>
#include <pthread.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <time.h>
#include <unistd.h>

#define EXPORT __attribute__((visibility("default")))

/*
 * vrlink is the Frame's client: it reads a controller's type twice, through
 * IVRSystem::GetStringTrackedDeviceProperty. Its input packing (with a 1024-byte buffer) packs
 * input only for frame_controller; its static properties for the PC (128-byte buffers, every
 * string property) are what the PC's driver sets the controller up from. So the packing gets
 * frame_controller, and the properties get what Valve's Android client sends for a PICO 4
 * controller: the PC then makes it a pico_controller, with Valve's PICO bindings and render model
 * (it keys those on the model number).
 *
 * The PC decodes the input by slots whose meaning depends on that type: per hand a click mask, a
 * touch mask and analog bytes. vrlink fills the slots from the Frame's input paths (IVRPaths), so
 * each Frame path that fills a slot the PICO layout uses is turned into our input with that
 * slot's PICO meaning (g_input_paths). Frame paths of slots the PICO layout leaves unused stay as
 * they are: the PC binds nothing to them. SteamVR on the headset keeps our type and bindings: only
 * this process's interfaces change.
 *
 * Poses: vrlink reads each device's driver pose through IVRClientInternal (slot 12: an 8-byte
 * header, then a DriverPose_t) and sends it with its driver-from-head transform, which places the
 * device at world-from-driver * pose * driver-from-head; for our controllers that is the aim pose,
 * the render models' frame. Valve's Android client sends a PICO 4 controller's OpenXR grip pose
 * times a fixed offset, and the PC places its PICO 4 model there. So for our controllers the
 * driver-from-head transform gets aim-to-grip (PICO OS's calibration, our driver's grip pose) times
 * Valve's offset composed onto it; SteamVR derives the sent point's velocity from it.
 */
#define VR_SLOT_GET_STRING_PROPERTY 28 // IVRSystem_026::GetStringTrackedDeviceProperty
#define VR_SLOT_STRING_TO_HANDLE 2     // IVRPaths_002::StringToHandle
#define VR_SLOT_GET_DRIVER_POSE 12     // IVRClientInternal_XXX: a device's driver pose, as vrlink reads it
//! Where the DriverPose_t starts in what IVRClientInternal's slot 12 writes.
#define VR_DRIVER_POSE_OFFSET 8
#define VR_MAX_DEVICES 64 // k_unMaxTrackedDeviceCount
//! How long a device's hand stays cached before it is asked again (seconds).
#define HAND_CACHE_S 1.0
#define VR_PROP_TRACKING_SYSTEM 1000   // Prop_TrackingSystemName_String
#define VR_PROP_MODEL_NUMBER 1001      // Prop_ModelNumber_String
#define VR_PROP_SERIAL_NUMBER 1002     // Prop_SerialNumber_String
#define VR_PROP_RENDER_MODEL 1003      // Prop_RenderModelName_String
#define VR_PROP_MANUFACTURER 1005      // Prop_ManufacturerName_String
#define VR_PROP_RESOURCE_ROOT 1035     // Prop_ResourceRoot_String
#define VR_PROP_DEVICE_TYPE 1036       // Prop_RegisteredDeviceType_String
#define VR_PROP_INPUT_PROFILE 1037     // Prop_InputProfilePath_String
#define VR_PROP_CONTROLLER_TYPE 7000   // Prop_ControllerType_String
#define VR_PROP_ERROR_NOT_SET 4        // TrackedProp_UnknownProperty
//! The buffer vrlink's static properties read with; its input packing uses 1024 bytes.
#define VRLINK_STATIC_PROPS_SIZE 128

typedef uint32_t (*vr_get_string_property)(void *self, uint32_t device, int32_t prop, char *value, uint32_t size,
                                           int32_t *error);
typedef int32_t (*vr_string_to_handle)(void *self, uint64_t *handle, const char *path);
typedef uint64_t (*vr_get_driver_pose)(void *self, uint32_t device, uint8_t *out);

//! The start of DriverPose_t: quaternions are w, x, y, z.
typedef struct
{
	double pose_time_offset;
	double world_from_driver_rotation[4];
	double world_from_driver_translation[3];
	double driver_from_head_rotation[4];
	double driver_from_head_translation[3];
} vr_driver_pose_start;

static vr_get_string_property real_get_string_property;
static vr_string_to_handle real_string_to_handle;
static vr_get_driver_pose real_get_driver_pose;
//! vrlink's IVRSystem, which the pose hook asks for a device's type.
static void *g_system;

/*
 * Valve's offset from a PICO 4 controller's OpenXR grip pose to the pose it sends (its Android
 * client's controller_config.json, /interaction_profiles/bytedance/pico4_controller): a position in
 * the grip frame and a rotation of -10 degrees about x, as a quaternion (w, x, y, z).
 */
static const double g_valve_offset_position[3] = {-0.007, -0.0512, -0.09607};
static const double g_valve_offset_rotation[4] = {0.9961946980917455, -0.08715574274765817, 0.0, 0.0};
//! Where our driver finds PICO OS's calibration (PICO4_CALIBRATION_DIR overrides it, as there).
#define CALIBRATION_DIR "/usr/share/qcom/pico4/calibration"

//! A rigid transform: unit quaternion (w, x, y, z) and translation.
typedef struct
{
	double q[4];
	double t[3];
} transform;

static pthread_once_t g_offsets_once = PTHREAD_ONCE_INIT;
//! Per hand (0 left, 1 right) aim to sent pose; valid only when g_have_offsets.
static transform g_offsets[2];
static bool g_have_offsets;

static pthread_mutex_t g_hand_lock = PTHREAD_MUTEX_INITIALIZER;
static char g_hand[VR_MAX_DEVICES];
static double g_hand_checked[VR_MAX_DEVICES];

//! Valve's Android client's static properties of the PICO 4's controllers (its controller_config.json).
static const struct
{
	int32_t prop;
	const char *left;
	const char *right;
} g_pico4_props[] = {
    {VR_PROP_CONTROLLER_TYPE, "pico_controller", "pico_controller"},
    {VR_PROP_TRACKING_SYSTEM, "pico", "pico"},
    {VR_PROP_SERIAL_NUMBER, "VRLINKP4_Controller_Left", "VRLINKP4_Controller_Right"},
    {VR_PROP_MANUFACTURER, "PICO", "PICO"},
    {VR_PROP_MODEL_NUMBER, "PICO 4 (Left Controller)", "PICO 4 (Right Controller)"},
    {VR_PROP_RENDER_MODEL, "pico_4_controller_left", "pico_4_controller_right"},
    {VR_PROP_RESOURCE_ROOT, "vrlink", "vrlink"},
    {VR_PROP_INPUT_PROFILE, "{vrlink}/input/pico_controller_profile.json", "{vrlink}/input/pico_controller_profile.json"},
    {VR_PROP_DEVICE_TYPE, "PICO/VRLINKP4_Controller_Left", "PICO/VRLINKP4_Controller_Right"},
};

/*
 * The Frame paths whose slots the PC's pico_controller reads, and our input with that slot's
 * meaning. The slots: click 0 A/X, 1 B/Y, 2 system, 3 joystick; touch 0 A/X, 1 B/Y, 2 trigger,
 * 3 joystick; analog 0 trigger, 1 joystick x, 2 joystick y, 3 grip (the Frame's: right A, B, X, Y
 * and left d-pad down, right, left, up; analog thumbstick x, y, trigger, grip). As Valve's Android
 * client, only the left menu button goes out as system: the right one is the headset's home.
 */
static const struct
{
	const char *frame;
	const char *ours;
} g_input_paths[] = {
    {"/user/hand/right/input/x/click", "/user/hand/right/input/none/click"},
    {"/user/hand/right/input/x/touch", "/user/hand/right/input/trigger/touch"},
    {"/user/hand/right/input/y/click", "/user/hand/right/input/joystick/click"},
    {"/user/hand/right/input/y/touch", "/user/hand/right/input/joystick/touch"},
    {"/user/hand/right/input/thumbstick/x", "/user/hand/right/input/trigger/value"},
    {"/user/hand/right/input/thumbstick/y", "/user/hand/right/input/joystick/x"},
    {"/user/hand/right/input/trigger/value", "/user/hand/right/input/joystick/y"},
    {"/user/hand/left/input/dpad_down/click", "/user/hand/left/input/x/click"},
    {"/user/hand/left/input/dpad_down/touch", "/user/hand/left/input/x/touch"},
    {"/user/hand/left/input/dpad_right/click", "/user/hand/left/input/y/click"},
    {"/user/hand/left/input/dpad_right/touch", "/user/hand/left/input/y/touch"},
    {"/user/hand/left/input/dpad_left/click", "/user/hand/left/input/system/click"},
    {"/user/hand/left/input/dpad_left/touch", "/user/hand/left/input/trigger/touch"},
    {"/user/hand/left/input/dpad_up/click", "/user/hand/left/input/joystick/click"},
    {"/user/hand/left/input/dpad_up/touch", "/user/hand/left/input/joystick/touch"},
    {"/user/hand/left/input/thumbstick/x", "/user/hand/left/input/trigger/value"},
    {"/user/hand/left/input/thumbstick/y", "/user/hand/left/input/joystick/x"},
    {"/user/hand/left/input/trigger/value", "/user/hand/left/input/joystick/y"},
};

static uint32_t
vr_string_result(const char *text, char *value, uint32_t size, int32_t *error)
{
	uint32_t needed = (uint32_t)strlen(text) + 1;
	if (value != NULL && size >= needed) {
		memcpy(value, text, needed);
	}
	if (error != NULL) {
		*error = 0;
	}
	return needed;
}

//! Tell which hand @p device is when it is one of our controllers: 'l', 'r', or 0 for any other device.
static char
our_hand(void *self, uint32_t device)
{
	char text[128] = "";
	int32_t error = 0;
	real_get_string_property(self, device, VR_PROP_CONTROLLER_TYPE, text, sizeof(text), &error);
	if (error != 0 || strcmp(text, "pico_controller") != 0) {
		return 0;
	}
	real_get_string_property(self, device, VR_PROP_MODEL_NUMBER, text, sizeof(text), &error);
	if (error != 0) {
		return 0;
	}
	return strstr(text, "left") != NULL ? 'l' : strstr(text, "right") != NULL ? 'r' : 0;
}

static uint32_t
hook_get_string_property(void *self, uint32_t device, int32_t prop, char *value, uint32_t size, int32_t *error)
{
	if (size == VRLINK_STATIC_PROPS_SIZE) {
		char hand = our_hand(self, device);
		for (size_t i = 0; hand != 0 && i < sizeof(g_pico4_props) / sizeof(g_pico4_props[0]); i++) {
			if (g_pico4_props[i].prop == prop) {
				return vr_string_result(hand == 'l' ? g_pico4_props[i].left : g_pico4_props[i].right, value, size,
				                        error);
			}
		}
		return real_get_string_property(self, device, prop, value, size, error);
	}
	// The input packing, every frame: one call, as without this library.
	int32_t real_error = 0;
	uint32_t n = real_get_string_property(self, device, prop, value, size, &real_error);
	if (prop == VR_PROP_CONTROLLER_TYPE && real_error == 0 && value != NULL && strcmp(value, "pico_controller") == 0) {
		return vr_string_result("frame_controller", value, size, error);
	}
	if (error != NULL) {
		*error = real_error;
	}
	return n;
}

static int32_t
hook_string_to_handle(void *self, uint64_t *handle, const char *path)
{
	for (size_t i = 0; i < sizeof(g_input_paths) / sizeof(g_input_paths[0]); i++) {
		if (strcmp(path, g_input_paths[i].frame) == 0) {
			return real_string_to_handle(self, handle, g_input_paths[i].ours);
		}
	}
	return real_string_to_handle(self, handle, path);
}

//! The quaternion product @p a * @p b (w, x, y, z) into @p out.
static void
quat_mul(const double a[4], const double b[4], double out[4])
{
	double r[4] = {
	    a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3],
	    a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2],
	    a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1],
	    a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0],
	};
	memcpy(out, r, sizeof(r));
}

//! @p v rotated by the unit quaternion @p q (w, x, y, z) into @p out.
static void
quat_rotate(const double q[4], const double v[3], double out[3])
{
	double p[4] = {0, v[0], v[1], v[2]}, conj[4] = {q[0], -q[1], -q[2], -q[3]}, t[4];
	quat_mul(q, p, t);
	quat_mul(t, conj, t);
	memcpy(out, t + 1, 3 * sizeof(double));
}

//! The @p count numbers of the JSON array after key @p key from @p from on; false without them.
static bool
json_numbers(const char *from, const char *key, double *out, int count)
{
	const char *p = from == NULL ? NULL : strstr(from, key);
	p = p == NULL ? NULL : strchr(p, '[');
	for (int i = 0; p != NULL && i < count; i++) {
		char *end;
		out[i] = strtod(p + 1, &end);
		if (end == p + 1) {
			return false;
		}
		p = strpbrk(end, i + 1 < count ? "," : "]");
	}
	return p != NULL;
}

/*
 * Each hand's aim-to-grip from calibration/controllers/aim-to-grip.json ({"left"|"right":
 * {"position": [x, y, z], "orientation": [w, x, y, z]}}, grip = aim * transform), times Valve's offset.
 */
static void
load_offsets(void)
{
	const char *dir = getenv("PICO4_CALIBRATION_DIR");
	char path[512];
	snprintf(path, sizeof(path), "%s/controllers/aim-to-grip.json", dir != NULL && *dir ? dir : CALIBRATION_DIR);
	char text[8192] = "";
	FILE *f = fopen(path, "r");
	if (f != NULL) {
		text[fread(text, 1, sizeof(text) - 1, f)] = '\0';
		fclose(f);
	}
	static const char *const hands[] = {"\"left\"", "\"right\""};
	for (int h = 0; h < 2; h++) {
		double p[3], q[4];
		const char *at = strstr(text, hands[h]);
		if (!json_numbers(at, "\"position\"", p, 3) || !json_numbers(at, "\"orientation\"", q, 4)) {
			fprintf(stderr, "pico4-vrlink-compat: no aim-to-grip in %s: controller poses stay aim poses\n", path);
			return;
		}
		transform *t = &g_offsets[h];
		quat_rotate(q, g_valve_offset_position, t->t);
		for (int i = 0; i < 3; i++) {
			t->t[i] += p[i];
		}
		quat_mul(q, g_valve_offset_rotation, t->q);
	}
	g_have_offsets = true;
}

static double
now_s(void)
{
	struct timespec ts;
	clock_gettime(CLOCK_MONOTONIC, &ts);
	return (double)ts.tv_sec + (double)ts.tv_nsec / 1e9;
}

//! 0 left, 1 right for one of our controllers, -1 otherwise; asked at most every HAND_CACHE_S.
static int
hand_index(uint32_t device)
{
	if (device >= VR_MAX_DEVICES || g_system == NULL || real_get_string_property == NULL) {
		return -1;
	}
	pthread_mutex_lock(&g_hand_lock);
	double now = now_s();
	if (g_hand_checked[device] == 0 || now - g_hand_checked[device] > HAND_CACHE_S) {
		g_hand[device] = our_hand(g_system, device);
		g_hand_checked[device] = now;
	}
	char hand = g_hand[device];
	pthread_mutex_unlock(&g_hand_lock);
	return hand == 'l' ? 0 : hand == 'r' ? 1 : -1;
}

//! Compose the hand's offset onto the driver-from-head transform of @p device's driver pose @p pose.
static void
send_pose(uint32_t device, vr_driver_pose_start *pose)
{
	pthread_once(&g_offsets_once, load_offsets);
	int hand = g_have_offsets ? hand_index(device) : -1;
	if (hand < 0) {
		return;
	}
	const transform *t = &g_offsets[hand];
	double moved[3];
	quat_rotate(pose->driver_from_head_rotation, t->t, moved);
	for (int i = 0; i < 3; i++) {
		pose->driver_from_head_translation[i] += moved[i];
	}
	quat_mul(pose->driver_from_head_rotation, t->q, pose->driver_from_head_rotation);
}

static uint64_t
hook_get_driver_pose(void *self, uint32_t device, uint8_t *out)
{
	uint64_t result = real_get_driver_pose(self, device, out);
	if (out != NULL) {
		vr_driver_pose_start pose;
		memcpy(&pose, out + VR_DRIVER_POSE_OFFSET, sizeof(pose));
		send_pose(device, &pose);
		memcpy(out + VR_DRIVER_POSE_OFFSET, &pose, sizeof(pose));
	}
	return result;
}

//! Replaces a slot of an interface's table in this process; returns what it held.
static void *
patch_slot(void *iface, size_t slot, void *hook)
{
	void **entry = *(void ***)iface + slot;
	if (*entry == hook) {
		return NULL;
	}
	long page = sysconf(_SC_PAGESIZE);
	void *start = (void *)((uintptr_t)entry & ~((uintptr_t)page - 1));
	size_t len = (size_t)((uintptr_t)(entry + 1) - (uintptr_t)start);
	if (mprotect(start, len, PROT_READ | PROT_WRITE) != 0) {
		perror("pico4-vrlink-compat: mprotect");
		return NULL;
	}
	void *old = *entry;
	*entry = hook;
	mprotect(start, len, PROT_READ);
	return old;
}

EXPORT void *
VR_GetGenericInterface(const char *version, int32_t *error)
{
	static void *(*real)(const char *, int32_t *);
	if (real == NULL) {
		real = (void *(*)(const char *, int32_t *))dlsym(RTLD_NEXT, "VR_GetGenericInterface");
	}
	void *iface = real(version, error);
	if (iface == NULL) {
		return NULL;
	}
	if (strcmp(version, "IVRSystem_026") == 0) {
		void *old = patch_slot(iface, VR_SLOT_GET_STRING_PROPERTY, (void *)hook_get_string_property);
		if (old != NULL) {
			real_get_string_property = (vr_get_string_property)old;
		}
		g_system = iface;
	} else if (strcmp(version, "IVRClientInternal_XXX") == 0) {
		void *old = patch_slot(iface, VR_SLOT_GET_DRIVER_POSE, (void *)hook_get_driver_pose);
		if (old != NULL) {
			real_get_driver_pose = (vr_get_driver_pose)old;
		}
	} else if (strcmp(version, "IVRPaths_002") == 0) {
		void *old = patch_slot(iface, VR_SLOT_STRING_TO_HANDLE, (void *)hook_string_to_handle);
		if (old != NULL) {
			real_string_to_handle = (vr_string_to_handle)old;
		}
	}
	return iface;
}
