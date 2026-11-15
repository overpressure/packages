// SPDX-License-Identifier: BSL-1.0
/*
 * pico4-vrlink-compat: preloaded into SteamVR's Steam Link client (vrlink, through its
 * run_vrlink.sh; docs/session.md#steam-link) so that the PC sees the PICO 4's controllers as
 * PICO 4 controllers, as Valve's own Steam Link client on a PICO headset sends them.
 */
#define _GNU_SOURCE

#include <dlfcn.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/mman.h>
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
 */
#define VR_SLOT_GET_STRING_PROPERTY 28 // IVRSystem_026::GetStringTrackedDeviceProperty
#define VR_SLOT_STRING_TO_HANDLE 2     // IVRPaths_002::StringToHandle
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

static vr_get_string_property real_get_string_property;
static vr_string_to_handle real_string_to_handle;

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
	} else if (strcmp(version, "IVRPaths_002") == 0) {
		void *old = patch_slot(iface, VR_SLOT_STRING_TO_HANDLE, (void *)hook_string_to_handle);
		if (old != NULL) {
			real_string_to_handle = (vr_string_to_handle)old;
		}
	}
	return iface;
}
