// SPDX-License-Identifier: BSL-1.0
/*
 * pico4-vrlink-compat: preloaded into SteamVR's Steam Link client (vrlink, through its
 * run_vrlink.sh; docs/session.md#steam-link) so that it takes the PICO 4's controllers for the
 * Steam Frame's.
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
 * The controllers as vrlink sees them: the Frame's, the only ones it sends input for. On the PC,
 * Valve's driver activates a controller only with a serial, a manufacturer and a type in its
 * properties, and reads the input vrlink sends only for the type frame_controller. vrlink reads
 * the input by the Frame's paths (IVRPaths), so they are turned into ours, the reverse of
 * frame-controller-remapping.json. SteamVR on the headset keeps our type and bindings: only this
 * process's interfaces change.
 */
#define VR_SLOT_GET_STRING_PROPERTY 28 // IVRSystem_026::GetStringTrackedDeviceProperty
#define VR_SLOT_STRING_TO_HANDLE 2     // IVRPaths_002::StringToHandle
#define VR_PROP_MANUFACTURER 1005      // Prop_ManufacturerName_String
#define VR_PROP_CONTROLLER_TYPE 7000   // Prop_ControllerType_String
#define VR_PROP_ERROR_NOT_SET 4        // TrackedProp_UnknownProperty

typedef uint32_t (*vr_get_string_property)(void *self, uint32_t device, int32_t prop, char *value, uint32_t size,
                                           int32_t *error);
typedef int32_t (*vr_string_to_handle)(void *self, uint64_t *handle, const char *path);

static vr_get_string_property real_get_string_property;
static vr_string_to_handle real_string_to_handle;

static const struct
{
	const char *frame;
	const char *ours;
} g_input_paths[] = {
    {"/user/hand/left/input/thumbstick/", "/user/hand/left/input/joystick/"},
    {"/user/hand/right/input/thumbstick/", "/user/hand/right/input/joystick/"},
    {"/user/hand/left/input/dpad_down/", "/user/hand/left/input/x/"},
    {"/user/hand/left/input/dpad_up/", "/user/hand/left/input/y/"},
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

static uint32_t
hook_get_string_property(void *self, uint32_t device, int32_t prop, char *value, uint32_t size, int32_t *error)
{
	int32_t real_error = 0;
	uint32_t n = real_get_string_property(self, device, prop, value, size, &real_error);
	if (prop == VR_PROP_CONTROLLER_TYPE && real_error == 0 && value != NULL &&
	    strcmp(value, "pico_controller") == 0) {
		return vr_string_result("frame_controller", value, size, error);
	}
	if (prop == VR_PROP_MANUFACTURER && real_error == VR_PROP_ERROR_NOT_SET) {
		return vr_string_result("PICO", value, size, error);
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
		size_t len = strlen(g_input_paths[i].frame);
		if (strncmp(path, g_input_paths[i].frame, len) == 0) {
			char ours[128];
			snprintf(ours, sizeof(ours), "%s%s", g_input_paths[i].ours, path + len);
			return real_string_to_handle(self, handle, ours);
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
