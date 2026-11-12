// Copyright 2026, overpressure contributors.
// SPDX-License-Identifier: BSL-1.0
/*
 * pico4-roomview: the producer of SteamVR's room view (docs/xr.md#passthrough).
 *
 * SteamVR 2.17's compositor shows the cameras only from a client of the private
 * IVRCameraPassthroughInternal_001 in vrclient.so, as the Frame's XR service is: the client
 * sets up dma-buf frames for two cameras and publishes each frame with its metadata. The
 * PICO 4 driver in vrserver owns the cameras and shares the forward pair's upright 8-bit
 * images, the cameras' geometry and the head pose at each exposure (pico4_roomview.h). This
 * program allocates R8 images on the GPU that it can share as dma-bufs (Turnip, linear),
 * hands them to SteamVR, and copies each new frame into a free one while the compositor
 * asks for frames. The interface's layout is this project's own description of it
 * (docs/xr.md#passthrough); nothing of Valve's is linked: libopenvr_api.so is opened at run time.
 *
 *   pico4-roomview [--frames N] [--test-pattern]
 *
 * --test-pattern publishes colour bars as NV12 in place of the cameras' images, with the left
 * camera's geometry and the frames' poses: a check of the colour path into the compositor.
 */
#define _GNU_SOURCE
#include "pico4_roomview.h"

#include <dirent.h>
#include <dlfcn.h>
#include <errno.h>
#include <fcntl.h>
#include <math.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <time.h>
#include <unistd.h>
#include <vulkan/vulkan.h>

#define OPENVR_API "/opt/steamvr/bin/linuxarm64/libopenvr_api.so"
#define INTERFACE "IVRCameraPassthroughInternal_001"
#define COMPOSITOR_INTERFACE "IVRCompositor_029"
#define APPLICATION_BACKGROUND 3
#define GRAPHICS_VULKAN 2
#define BLOCK 0 //!< The frame block of the plain camera view (the other one is for a second mode).
#define MAX_FRAMES 16
#define DRM_FORMAT_R8 0x20203852u // fourcc 'R', '8', ' ', ' '
#define DRM_FORMAT_NV12 0x3231564eu // fourcc 'N', 'V', '1', '2'
#define DRM_FORMAT_MOD_LINEAR 0ull

// The interface's slots, in vtable order; every call takes the object first.
enum slot
{
	SLOT_SET_GRAPHICS_DEVICE = 0,
	SLOT_SETUP_CAMERA_FRAMES = 1,
	SLOT_IS_REQUESTED = 2,
	SLOT_GET_FRAME_STATUSES = 3,
	SLOT_PUBLISH_FRAME = 4,
	SLOT_SET_FRAME_METADATA = 5,
	SLOT_SET_STATE = 10,
	SLOT_SET_PAUSED = 12,
	SLOT_RELEASE = 14,
};

enum frame_status
{
	FRAME_UNUSED = -1,
	FRAME_FREE = 0,
	FRAME_NEWEST = 1,
	FRAME_HELD = 2, //!< The compositor renders from it.
};

struct vr_vulkan_device
{
	VkInstance instance;
	VkDevice device;
	VkPhysicalDevice physical_device;
	VkQueue queue;
	uint32_t queue_family_index;
};

struct graphics_device
{
	uint32_t type;
	uint32_t pad;
	struct vr_vulkan_device *vulkan;
};

//! The frame description SetupCameraFrames takes (0x38 bytes).
struct frames_desc
{
	uint32_t frames;   //!< Per camera, at most 16.
	uint32_t width, height;
	uint8_t nv12;       //!< Two-plane YUV input.
	uint8_t monochrome; //!< One 8-bit channel.
	uint8_t pad[0x22];
	uint32_t spline_width, spline_height; //!< The distortion map SteamVR always allocates.
};

struct dmabuf_plane
{
	uint32_t offset, stride;
	int32_t fd;
};

//! OpenVR's DmabufAttributes_t.
struct dmabuf_attributes
{
	void *next;
	uint32_t width, height, depth, mip_levels, array_layers, sample_count;
	uint32_t format; //!< DRM fourcc.
	uint32_t pad;
	uint64_t modifier;
	uint32_t plane_count;
	struct dmabuf_plane planes[4];
};

//! The view state slot 10 sets: available, the camera chosen per eye, the other block, -, sharpen.
struct view_state
{
	uint8_t available, per_eye_camera, second_block, unused, sharpen, pad[3];
};

//! Per-frame metadata (0xf8 bytes), the fields the compositor reads for the plain camera view.
struct frame_metadata
{
	uint8_t bytes[0xf8];
};
#define META_TIME 0x08      // double, seconds
#define META_INTRINSICS 0x10 // float fx, fy, cx, cy in pixels
#define META_DISTORTION 0x20 // float k1..k4 (fisheye)
#define META_CAMERA 0x30     // HmdMatrix34_t, the camera in raw tracking space at its exposure
#define META_CAMERA_PICK 0x60 // HmdMatrix34_t, the camera in the head frame: which camera serves which eye
#define META_HEAD 0x90       // HmdMatrix34_t, a head-frame calibration (identity)
#define META_VALID 0xcc      // float u0, v0, u1, v1: the image's valid rectangle (normalized)
#define META_SPLINE 0xe0     // u8, 1: distortion from the spline map instead of k1..k4

_Static_assert(sizeof(struct frames_desc) == 0x38, "frames_desc");
_Static_assert(sizeof(struct dmabuf_attributes) == 0x68, "dmabuf_attributes");

typedef uint32_t (*set_graphics_device_fn)(void *, const struct graphics_device *);
typedef uint32_t (*setup_camera_frames_fn)(void *, uint32_t, const struct frames_desc *,
                                            const struct dmabuf_attributes *);
typedef uint32_t (*is_requested_fn)(void *, uint32_t, bool *);
typedef uint32_t (*get_frame_statuses_fn)(void *, uint32_t, uint32_t, int32_t *, uint32_t);
typedef uint32_t (*publish_frame_fn)(void *, uint32_t, uint32_t, uint32_t);
typedef uint32_t (*set_frame_metadata_fn)(void *, uint32_t, uint32_t, uint32_t, const struct frame_metadata *);
typedef void (*set_state_fn)(void *, const struct view_state *);
typedef void (*set_paused_fn)(void *, bool);
typedef void (*release_fn)(void *);

#define CALL(obj, slot, type) ((type)((*(void ***)(obj))[slot]))

struct image
{
	VkImage image;
	VkDeviceMemory memory;
	uint8_t *data;
	VkSubresourceLayout layout; //!< The (luma) plane.
	VkSubresourceLayout uv;     //!< NV12: the interleaved Cb, Cr plane.
	int fd;
};

static volatile sig_atomic_t stop;

static void
on_signal(int sig)
{
	(void)sig;
	stop = 1;
}

static int64_t
monotonic_ns(void)
{
	struct timespec ts;
	clock_gettime(CLOCK_MONOTONIC, &ts);
	return (int64_t)ts.tv_sec * 1000000000ll + ts.tv_nsec;
}

static void
sleep_ms(int ms)
{
	struct timespec ts = {ms / 1000, (long)(ms % 1000) * 1000000L};
	nanosleep(&ts, NULL);
}

//! The oldest process of this user named @p name (its comm), or 0.
static pid_t
find_process(const char *name)
{
	pid_t found = 0;
	DIR *proc = opendir("/proc");
	if (proc == NULL) {
		return 0;
	}
	struct dirent *e;
	while ((e = readdir(proc)) != NULL) {
		char *end;
		long pid = strtol(e->d_name, &end, 10);
		if (*end != '\0' || pid <= 0) {
			continue;
		}
		char path[64], comm[64] = {0};
		snprintf(path, sizeof(path), "/proc/%ld/comm", pid);
		int fd = open(path, O_RDONLY | O_CLOEXEC);
		if (fd < 0) {
			continue;
		}
		ssize_t n = read(fd, comm, sizeof(comm) - 1);
		close(fd);
		struct stat st;
		snprintf(path, sizeof(path), "/proc/%ld", pid);
		if (n <= 0 || stat(path, &st) != 0 || st.st_uid != getuid()) {
			continue;
		}
		comm[strcspn(comm, "\n")] = '\0';
		if (strcmp(comm, name) == 0 && (found == 0 || pid < found)) {
			found = (pid_t)pid;
		}
	}
	closedir(proc);
	return found;
}

//! The driver's shared memory, once vrserver's driver has created it.
static struct pico4_roomview *
map_roomview(void)
{
	int fd = open(PICO4_ROOMVIEW_PATH, O_RDWR | O_CLOEXEC);
	if (fd < 0) {
		return NULL;
	}
	struct pico4_roomview *rv = mmap(NULL, sizeof(*rv), PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
	close(fd);
	return rv == MAP_FAILED ? NULL : rv;
}

struct vulkan
{
	VkInstance instance;
	VkPhysicalDevice physical;
	VkDevice device;
	VkQueue queue;
	uint32_t family;
	PFN_vkGetMemoryFdKHR get_memory_fd;
};

/*!
 * The extensions of @p wanted that the instance (@p physical NULL) or device offers, into @p out.
 * vrclient calls their entry points (KHR names) on our device: the shared semaphore it imports
 * from the compositor (an OPAQUE_FD timeline), the
 * distortion images it creates and exports.
 */
static uint32_t
offered(VkPhysicalDevice physical, const char *const *wanted, uint32_t n, const char **out)
{
	VkExtensionProperties props[512];
	uint32_t count = 512, used = 0;
	if (physical == NULL) {
		vkEnumerateInstanceExtensionProperties(NULL, &count, props);
	} else {
		vkEnumerateDeviceExtensionProperties(physical, NULL, &count, props);
	}
	for (uint32_t i = 0; i < n; i++) {
		for (uint32_t j = 0; j < count; j++) {
			if (strcmp(wanted[i], props[j].extensionName) == 0) {
				out[used++] = wanted[i];
				break;
			}
		}
	}
	return used;
}

static bool
vulkan_init(struct vulkan *vk)
{
	VkApplicationInfo app = {
	    .sType = VK_STRUCTURE_TYPE_APPLICATION_INFO,
	    .pApplicationName = "pico4-roomview",
	    .apiVersion = VK_API_VERSION_1_2,
	};
	static const char *const instance_wanted[] = {
	    VK_KHR_GET_PHYSICAL_DEVICE_PROPERTIES_2_EXTENSION_NAME,
	    VK_KHR_EXTERNAL_MEMORY_CAPABILITIES_EXTENSION_NAME,
	    VK_KHR_EXTERNAL_SEMAPHORE_CAPABILITIES_EXTENSION_NAME,
	    VK_KHR_EXTERNAL_FENCE_CAPABILITIES_EXTENSION_NAME,
	};
	const char *instance_extensions[4];
	VkInstanceCreateInfo ici = {
	    .sType = VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO,
	    .pApplicationInfo = &app,
	    .enabledExtensionCount = offered(NULL, instance_wanted, 4, instance_extensions),
	    .ppEnabledExtensionNames = instance_extensions,
	};
	if (vkCreateInstance(&ici, NULL, &vk->instance) != VK_SUCCESS) {
		fprintf(stderr, "pico4-roomview: no Vulkan instance\n");
		return false;
	}
	uint32_t count = 1;
	VkResult r = vkEnumeratePhysicalDevices(vk->instance, &count, &vk->physical);
	if ((r != VK_SUCCESS && r != VK_INCOMPLETE) || count == 0) {
		fprintf(stderr, "pico4-roomview: no GPU\n");
		return false;
	}
	VkQueueFamilyProperties families[8];
	uint32_t nf = 8;
	vkGetPhysicalDeviceQueueFamilyProperties(vk->physical, &nf, families);
	vk->family = UINT32_MAX;
	for (uint32_t i = 0; i < nf; i++) {
		if (families[i].queueFlags & VK_QUEUE_GRAPHICS_BIT) {
			vk->family = i;
			break;
		}
	}
	if (vk->family == UINT32_MAX) {
		return false;
	}
	const float priority = 1.0f;
	VkDeviceQueueCreateInfo q = {
	    .sType = VK_STRUCTURE_TYPE_DEVICE_QUEUE_CREATE_INFO,
	    .queueFamilyIndex = vk->family,
	    .queueCount = 1,
	    .pQueuePriorities = &priority,
	};
	// Shared images: dma-bufs with an explicit (linear) layout, as the compositor imports them;
	// and what vrclient uses on this device.
	static const char *const required[] = {
	    VK_KHR_EXTERNAL_MEMORY_FD_EXTENSION_NAME,
	    VK_EXT_EXTERNAL_MEMORY_DMA_BUF_EXTENSION_NAME,
	    VK_EXT_IMAGE_DRM_FORMAT_MODIFIER_EXTENSION_NAME,
	    VK_KHR_EXTERNAL_SEMAPHORE_FD_EXTENSION_NAME,
	};
	static const char *const wanted[] = {
	    VK_KHR_EXTERNAL_MEMORY_FD_EXTENSION_NAME,
	    VK_EXT_EXTERNAL_MEMORY_DMA_BUF_EXTENSION_NAME,
	    VK_EXT_IMAGE_DRM_FORMAT_MODIFIER_EXTENSION_NAME,
	    VK_KHR_EXTERNAL_SEMAPHORE_FD_EXTENSION_NAME,
	    VK_KHR_EXTERNAL_MEMORY_EXTENSION_NAME,
	    VK_KHR_EXTERNAL_SEMAPHORE_EXTENSION_NAME,
	    VK_KHR_EXTERNAL_FENCE_FD_EXTENSION_NAME,
	    VK_KHR_TIMELINE_SEMAPHORE_EXTENSION_NAME,
	    VK_KHR_DEDICATED_ALLOCATION_EXTENSION_NAME,
	    VK_KHR_GET_MEMORY_REQUIREMENTS_2_EXTENSION_NAME,
	    VK_KHR_BIND_MEMORY_2_EXTENSION_NAME,
	    VK_KHR_IMAGE_FORMAT_LIST_EXTENSION_NAME,
	    VK_KHR_SAMPLER_YCBCR_CONVERSION_EXTENSION_NAME,
	    VK_EXT_QUEUE_FAMILY_FOREIGN_EXTENSION_NAME,
	};
	const uint32_t n_wanted = sizeof(wanted) / sizeof(wanted[0]);
	const char *extensions[sizeof(wanted) / sizeof(wanted[0])];
	uint32_t n = offered(vk->physical, wanted, n_wanted, extensions);
	const char *have[4];
	if (offered(vk->physical, required, 4, have) != 4) {
		fprintf(stderr, "pico4-roomview: the GPU lacks dma-buf images or shared semaphores\n");
		return false;
	}
	// The timeline semaphores vrclient imports.
	VkPhysicalDeviceTimelineSemaphoreFeatures timeline = {
	    .sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_TIMELINE_SEMAPHORE_FEATURES,
	    .timelineSemaphore = VK_TRUE,
	};
	VkDeviceCreateInfo dci = {
	    .sType = VK_STRUCTURE_TYPE_DEVICE_CREATE_INFO,
	    .pNext = &timeline,
	    .queueCreateInfoCount = 1,
	    .pQueueCreateInfos = &q,
	    .enabledExtensionCount = n,
	    .ppEnabledExtensionNames = extensions,
	};
	if (vkCreateDevice(vk->physical, &dci, NULL, &vk->device) != VK_SUCCESS) {
		fprintf(stderr, "pico4-roomview: no Vulkan device with dma-buf images\n");
		return false;
	}
	vkGetDeviceQueue(vk->device, vk->family, 0, &vk->queue);
	vk->get_memory_fd = (PFN_vkGetMemoryFdKHR)vkGetDeviceProcAddr(vk->device, "vkGetMemoryFdKHR");
	return vk->get_memory_fd != NULL;
}

/*!
 * One frame the CPU writes and the compositor imports: linear, mapped, exported as a dma-buf.
 * R8 for the grey cameras; NV12 (two planes in one allocation) for colour, the only colour
 * input of the compositor's room view.
 */
static bool
image_create(struct vulkan *vk, uint32_t width, uint32_t height, bool nv12, struct image *out)
{
	const uint64_t modifier = DRM_FORMAT_MOD_LINEAR;
	VkImageDrmFormatModifierListCreateInfoEXT mods = {
	    .sType = VK_STRUCTURE_TYPE_IMAGE_DRM_FORMAT_MODIFIER_LIST_CREATE_INFO_EXT,
	    .drmFormatModifierCount = 1,
	    .pDrmFormatModifiers = &modifier,
	};
	VkExternalMemoryImageCreateInfo external = {
	    .sType = VK_STRUCTURE_TYPE_EXTERNAL_MEMORY_IMAGE_CREATE_INFO,
	    .pNext = &mods,
	    .handleTypes = VK_EXTERNAL_MEMORY_HANDLE_TYPE_DMA_BUF_BIT_EXT,
	};
	VkImageCreateInfo ici = {
	    .sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO,
	    .pNext = &external,
	    .imageType = VK_IMAGE_TYPE_2D,
	    .format = nv12 ? VK_FORMAT_G8_B8R8_2PLANE_420_UNORM : VK_FORMAT_R8_UNORM,
	    .extent = {width, height, 1},
	    .mipLevels = 1,
	    .arrayLayers = 1,
	    .samples = VK_SAMPLE_COUNT_1_BIT,
	    .tiling = VK_IMAGE_TILING_DRM_FORMAT_MODIFIER_EXT,
	    .usage = VK_IMAGE_USAGE_SAMPLED_BIT | VK_IMAGE_USAGE_TRANSFER_SRC_BIT | VK_IMAGE_USAGE_TRANSFER_DST_BIT,
	    .sharingMode = VK_SHARING_MODE_EXCLUSIVE,
	    .initialLayout = VK_IMAGE_LAYOUT_UNDEFINED,
	};
	if (vkCreateImage(vk->device, &ici, NULL, &out->image) != VK_SUCCESS) {
		fprintf(stderr, "pico4-roomview: no linear %s dma-buf image\n", nv12 ? "NV12" : "R8");
		return false;
	}
	VkMemoryRequirements req;
	vkGetImageMemoryRequirements(vk->device, out->image, &req);
	VkPhysicalDeviceMemoryProperties props;
	vkGetPhysicalDeviceMemoryProperties(vk->physical, &props);
	const VkMemoryPropertyFlags want = VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT;
	uint32_t type = UINT32_MAX;
	for (uint32_t i = 0; i < props.memoryTypeCount; i++) {
		if ((req.memoryTypeBits & (1u << i)) && (props.memoryTypes[i].propertyFlags & want) == want) {
			type = i;
			break;
		}
	}
	if (type == UINT32_MAX) {
		fprintf(stderr, "pico4-roomview: no mappable memory for the shared images\n");
		return false;
	}
	VkExportMemoryAllocateInfo export = {
	    .sType = VK_STRUCTURE_TYPE_EXPORT_MEMORY_ALLOCATE_INFO,
	    .handleTypes = VK_EXTERNAL_MEMORY_HANDLE_TYPE_DMA_BUF_BIT_EXT,
	};
	VkMemoryDedicatedAllocateInfo dedicated = {
	    .sType = VK_STRUCTURE_TYPE_MEMORY_DEDICATED_ALLOCATE_INFO,
	    .pNext = &export,
	    .image = out->image,
	};
	VkMemoryAllocateInfo mai = {
	    .sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO,
	    .pNext = &dedicated,
	    .allocationSize = req.size,
	    .memoryTypeIndex = type,
	};
	if (vkAllocateMemory(vk->device, &mai, NULL, &out->memory) != VK_SUCCESS ||
	    vkBindImageMemory(vk->device, out->image, out->memory, 0) != VK_SUCCESS ||
	    vkMapMemory(vk->device, out->memory, 0, VK_WHOLE_SIZE, 0, (void **)&out->data) != VK_SUCCESS) {
		fprintf(stderr, "pico4-roomview: shared image memory failed\n");
		return false;
	}
	VkImageSubresource sub = {.aspectMask = VK_IMAGE_ASPECT_MEMORY_PLANE_0_BIT_EXT};
	vkGetImageSubresourceLayout(vk->device, out->image, &sub, &out->layout);
	if (nv12) {
		VkImageSubresource uv = {.aspectMask = VK_IMAGE_ASPECT_MEMORY_PLANE_1_BIT_EXT};
		vkGetImageSubresourceLayout(vk->device, out->image, &uv, &out->uv);
	}
	VkMemoryGetFdInfoKHR fdi = {
	    .sType = VK_STRUCTURE_TYPE_MEMORY_GET_FD_INFO_KHR,
	    .memory = out->memory,
	    .handleType = VK_EXTERNAL_MEMORY_HANDLE_TYPE_DMA_BUF_BIT_EXT,
	};
	if (vk->get_memory_fd(vk->device, &fdi, &out->fd) != VK_SUCCESS) {
		fprintf(stderr, "pico4-roomview: no dma-buf of the shared image\n");
		return false;
	}
	memset(out->data + out->layout.offset, 0, (size_t)out->layout.rowPitch * height);
	if (nv12) {
		memset(out->data + out->uv.offset, 128, (size_t)out->uv.rowPitch * (height / 2)); // no colour
	}
	return true;
}

/*!
 * --test-pattern: eight colour bars (white, yellow, cyan, green, magenta, red, blue, black) as
 * NV12, sRGB-encoded and full range (BT.601), as the compositor's colour shader decodes them.
 */
static void
fill_bars(struct image *im, uint32_t width, uint32_t height)
{
	static const uint8_t bars[8][3] = {{255, 255, 255}, {255, 255, 0}, {0, 255, 255}, {0, 255, 0},
	                                   {255, 0, 255},   {255, 0, 0},   {0, 0, 255},   {0, 0, 0}};
	for (uint32_t y = 0; y < height; y++) {
		uint8_t *luma = im->data + im->layout.offset + (size_t)y * im->layout.rowPitch;
		uint8_t *uv = im->data + im->uv.offset + (size_t)(y / 2) * im->uv.rowPitch;
		for (uint32_t x = 0; x < width; x++) {
			const uint8_t *c = bars[x * 8 / width];
			double r = c[0], g = c[1], b = c[2];
			luma[x] = (uint8_t)lround(0.299 * r + 0.587 * g + 0.114 * b);
			if (y % 2 == 0 && x % 2 == 0) {
				double cb = 128 - 0.168736 * r - 0.331264 * g + 0.5 * b;
				double cr = 128 + 0.5 * r - 0.418688 * g - 0.081312 * b;
				uv[x] = (uint8_t)lround(cb < 0 ? 0 : cb > 255 ? 255 : cb);
				uv[x + 1] = (uint8_t)lround(cr < 0 ? 0 : cr > 255 ? 255 : cr);
			}
		}
	}
}

/*!
 * The driver's luma is linear in the light (sensor counts less the black level); the room view's
 * pixel shader squares each sample, so the frames carry its square root.
 */
static void
encode_table(uint8_t table[256])
{
	for (int i = 0; i < 256; i++) {
		table[i] = (uint8_t)(sqrt(i / 255.0) * 255.0 + 0.5);
	}
}

static void
put_matrix(struct frame_metadata *m, size_t at, const double v[12])
{
	float f[12];
	for (int i = 0; i < 12; i++) {
		f[i] = (float)v[i];
	}
	memcpy(m->bytes + at, f, sizeof(f));
}

//! CLOCK_MONOTONIC_RAW less CLOCK_MONOTONIC: the compositor's clock is the raw one.
static int64_t
raw_offset_ns(void)
{
	struct timespec raw, mono;
	clock_gettime(CLOCK_MONOTONIC_RAW, &raw);
	clock_gettime(CLOCK_MONOTONIC, &mono);
	return ((int64_t)raw.tv_sec - mono.tv_sec) * 1000000000ll + (raw.tv_nsec - mono.tv_nsec);
}

static void
metadata(const struct pico4_roomview *rv, const struct pico4_roomview_frame *f, int cam, int64_t raw_offset,
         struct frame_metadata *m)
{
	memset(m, 0, sizeof(*m));
	// A time after the compositor's now shows nothing; an older one fades the view.
	const double t = (double)(f->timestamp_ns + raw_offset) * 1e-9;
	memcpy(m->bytes + META_TIME, &t, sizeof(t));
	const float intrinsics[4] = {(float)rv->fx[cam], (float)rv->fy[cam], (float)rv->cx[cam], (float)rv->cy[cam]};
	memcpy(m->bytes + META_INTRINSICS, intrinsics, sizeof(intrinsics));
	const float k[4] = {(float)rv->k[cam][0], (float)rv->k[cam][1], (float)rv->k[cam][2], (float)rv->k[cam][3]};
	memcpy(m->bytes + META_DISTORTION, k, sizeof(k));
	// The compositor projects in the computer-vision camera frame (x right, y down, looking along
	// +z); the driver shares OpenVR's camera axes (y up, looking along -z).
	double cv[12];
	memcpy(cv, rv->camera_to_head[cam], sizeof(cv));
	for (int r = 0; r < 3; r++) {
		cv[r * 4 + 1] = -cv[r * 4 + 1];
		cv[r * 4 + 2] = -cv[r * 4 + 2];
	}
	// The compositor projects into the camera with the inverse of +0x30 from raw tracking space,
	// and its view is inverse(eye from head) x +0x90 x inverse(render head): +0x30 is the camera
	// in tracking space at its exposure, +0x90 a head-frame calibration (none here). The head's
	// motion since the exposure is then carried about the head, not about the tracking origin.
	static const double IDENTITY[12] = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0};
	const double *head = f->pose_valid ? f->tracking_from_head : IDENTITY;
	double camera[12];
	for (int r = 0; r < 3; r++) {
		for (int c = 0; c < 4; c++) {
			double v = c == 3 ? head[r * 4 + 3] : 0;
			for (int k = 0; k < 3; k++) {
				v += head[r * 4 + k] * cv[k * 4 + c];
			}
			camera[r * 4 + c] = v;
		}
	}
	put_matrix(m, META_CAMERA, camera);
	put_matrix(m, META_CAMERA_PICK, cv);
	put_matrix(m, META_HEAD, IDENTITY);
	// The valid part of the image: the pixel shader clamps its coordinate to it.
	const float valid[4] = {0, 0, 1, 1};
	memcpy(m->bytes + META_VALID, valid, sizeof(valid));
	m->bytes[META_SPLINE] = 0;
}

//! Copies the newest complete frame out of the ring; false if there is none (or it changed meanwhile).
static bool
newest_frame(struct pico4_roomview *rv, uint64_t after, struct pico4_roomview_frame *out)
{
	uint64_t n = __atomic_load_n(&rv->frames_written, __ATOMIC_ACQUIRE);
	if (n == 0) {
		return false;
	}
	const struct pico4_roomview_frame *f = &rv->frames[(n - 1) % PICO4_ROOMVIEW_FRAMES];
	uint64_t seq = __atomic_load_n(&f->sequence, __ATOMIC_ACQUIRE);
	if (seq == 0 || seq <= after) {
		return false;
	}
	// Grey: both cameras' images; NV12: one image with its half-height colour rows.
	size_t pixels = (size_t)rv->width * rv->height;
	pixels = rv->format == PICO4_ROOMVIEW_FORMAT_NV12 ? pixels * 3 / 2 : pixels * PICO4_ROOMVIEW_CAMS;
	out->timestamp_ns = f->timestamp_ns;
	out->exposure_us = f->exposure_us;
	out->pose_valid = f->pose_valid;
	memcpy(out->tracking_from_head, f->tracking_from_head, sizeof(out->tracking_from_head));
	memcpy(out->pixels, f->pixels, pixels);
	__atomic_thread_fence(__ATOMIC_ACQUIRE);
	if (__atomic_load_n(&f->sequence, __ATOMIC_RELAXED) != seq) {
		return false;
	}
	out->sequence = seq;
	return true;
}

//! Sets the frames up with the compositor; false if it does not take them (yet).
static bool
setup_frames(void *pt, const struct frames_desc *desc, const struct dmabuf_attributes *attrs)
{
	uint32_t r = CALL(pt, SLOT_SETUP_CAMERA_FRAMES, setup_camera_frames_fn)(pt, BLOCK, desc, attrs);
	if (r != 0) {
		fprintf(stderr, "pico4-roomview: SetupCameraFrames: %u, retrying\n", r);
		return false;
	}
	struct view_state state = {.available = 1, .per_eye_camera = 1};
	CALL(pt, SLOT_SET_STATE, set_state_fn)(pt, &state);
	CALL(pt, SLOT_SET_PAUSED, set_paused_fn)(pt, false);
	fprintf(stderr, "pico4-roomview: %u frames of %ux%u per camera set up (stride %u)\n", desc->frames, desc->width,
	        desc->height, attrs[0].planes[0].stride);
	return true;
}

int
main(int argc, char **argv)
{
	uint32_t frames = 3;
	bool test_pattern = false;
	for (int i = 1; i < argc; i++) {
		if (strcmp(argv[i], "--frames") == 0 && i + 1 < argc) {
			frames = (uint32_t)atoi(argv[++i]);
		} else if (strcmp(argv[i], "--test-pattern") == 0) {
			test_pattern = true;
		} else {
			fprintf(stderr, "usage: pico4-roomview [--frames N] [--test-pattern]\n");
			return 2;
		}
	}
	if (frames < 2 || frames > MAX_FRAMES) {
		fprintf(stderr, "pico4-roomview: 2 to %d frames per camera\n", MAX_FRAMES);
		return 2;
	}
	signal(SIGTERM, on_signal);
	signal(SIGINT, on_signal);
	// A compositor that goes away closes its pipe under us; the frames are set up again with the next.
	signal(SIGPIPE, SIG_IGN);
	setvbuf(stderr, NULL, _IOLBF, 0);

	// The driver's frames: vrserver loads the driver before any client can connect.
	struct pico4_roomview *rv = NULL;
	for (int i = 0; !stop && (rv = map_roomview()) == NULL; i++) {
		if (i == 0) {
			fprintf(stderr, "pico4-roomview: waiting for %s\n", PICO4_ROOMVIEW_PATH);
		}
		sleep_ms(500);
	}
	while (!stop && __atomic_load_n(&rv->magic, __ATOMIC_ACQUIRE) != PICO4_ROOMVIEW_MAGIC) {
		sleep_ms(100);
	}
	if (stop) {
		return 0;
	}
	const uint32_t width = rv->width, height = rv->height;
	const bool colour = rv->format == PICO4_ROOMVIEW_FORMAT_NV12;
	const size_t frame_bytes = colour ? (size_t)width * height * 3 / 2 : (size_t)width * height * PICO4_ROOMVIEW_CAMS;
	if (frame_bytes > PICO4_ROOMVIEW_BYTES || width == 0 || height == 0 || (colour && (width % 2 || height % 2))) {
		fprintf(stderr, "pico4-roomview: bad frame size %ux%u\n", width, height);
		return 1;
	}

	void *lib = dlopen(OPENVR_API, RTLD_NOW);
	if (lib == NULL) {
		fprintf(stderr, "pico4-roomview: %s\n", dlerror());
		return 1;
	}
	uint32_t (*vr_init)(int *, int, const char *) = (uint32_t(*)(int *, int, const char *))dlsym(lib, "VR_InitInternal2");
	void *(*vr_interface)(const char *, int *) = (void *(*)(const char *, int *))dlsym(lib, "VR_GetGenericInterface");
	void (*vr_shutdown)(void) = (void (*)(void))dlsym(lib, "VR_ShutdownInternal");
	if (vr_init == NULL || vr_interface == NULL || vr_shutdown == NULL) {
		fprintf(stderr, "pico4-roomview: %s lacks the client entry points\n", OPENVR_API);
		return 1;
	}
	int err = 0;
	// A background application: SteamVR's server must be running; it is not started for us.
	for (int i = 0; !stop; i++) {
		vr_init(&err, APPLICATION_BACKGROUND, NULL);
		if (err == 0) {
			break;
		}
		if (i == 0) {
			fprintf(stderr, "pico4-roomview: VR_Init: %d, retrying\n", err);
		}
		sleep_ms(1000);
	}
	if (stop) {
		return 0;
	}
	// The frames' import goes over vrclient's connection to the compositor, which only asking for
	// a compositor interface opens.
	if (vr_interface(COMPOSITOR_INTERFACE, &err) == NULL || err != 0) {
		fprintf(stderr, "pico4-roomview: no %s: %d\n", COMPOSITOR_INTERFACE, err);
		vr_shutdown();
		return 1;
	}
	void *pt = vr_interface(INTERFACE, &err);
	if (pt == NULL) {
		fprintf(stderr, "pico4-roomview: no %s: %d\n", INTERFACE, err);
		vr_shutdown();
		return 1;
	}

	struct vulkan vk = {0};
	if (!vulkan_init(&vk)) {
		vr_shutdown();
		return 1;
	}
	struct vr_vulkan_device vrdev = {vk.instance, vk.device, vk.physical, vk.queue, vk.family};
	struct graphics_device gd = {.type = GRAPHICS_VULKAN, .vulkan = &vrdev};
	uint32_t r = CALL(pt, SLOT_SET_GRAPHICS_DEVICE, set_graphics_device_fn)(pt, &gd);
	if (r != 0) {
		fprintf(stderr, "pico4-roomview: SetGraphicsDevice: %u\n", r);
		vr_shutdown();
		return 1;
	}

	/*
	 * Colour (NV12) has one camera image: both camera slots are backed by the same frames, and
	 * each frame is published to both, so the compositor's per-eye pick finds the same image.
	 * The compositor imports only plane 0's dma-buf; plane 1 names the same one.
	 */
	const bool nv12 = test_pattern || colour;
	fprintf(stderr, "pico4-roomview: %s frames of %ux%u\n", test_pattern ? "colour test pattern" : colour ? "colour" : "grey",
	        width, height);
	static struct image images[PICO4_ROOMVIEW_CAMS][MAX_FRAMES];
	static struct dmabuf_attributes attrs[PICO4_ROOMVIEW_CAMS][MAX_FRAMES];
	for (int c = 0; c < PICO4_ROOMVIEW_CAMS; c++) {
		for (uint32_t i = 0; i < frames; i++) {
			struct image *im = &images[c][i];
			if (nv12 && c > 0) {
				images[c][i] = images[0][i];
				attrs[c][i] = attrs[0][i];
				continue;
			}
			if (!image_create(&vk, width, height, nv12, im)) {
				vr_shutdown();
				return 1;
			}
			attrs[c][i] = (struct dmabuf_attributes){
			    .width = width,
			    .height = height,
			    .depth = 1,
			    .mip_levels = 1,
			    .array_layers = 1,
			    .sample_count = 1,
			    .format = nv12 ? DRM_FORMAT_NV12 : DRM_FORMAT_R8,
			    .modifier = DRM_FORMAT_MOD_LINEAR,
			    .plane_count = nv12 ? 2 : 1,
			    .planes = {{(uint32_t)im->layout.offset, (uint32_t)im->layout.rowPitch, im->fd},
			               {(uint32_t)im->uv.offset, (uint32_t)im->uv.rowPitch, im->fd}},
			};
			if (test_pattern) {
				fill_bars(im, width, height);
			}
		}
	}
	// Fisheye coefficients; the distortion map SteamVR always allocates stays unused (metadata
	// META_SPLINE 0) and small.
	struct frames_desc desc = {
	    .frames = frames,
	    .width = width,
	    .height = height,
	    .nv12 = nv12 ? 1 : 0,
	    .monochrome = nv12 ? 0 : 1,
	    .spline_width = 16,
	    .spline_height = 16,
	};
	uint8_t encode[256];
	encode_table(encode);
	const pid_t server = find_process("vrserver");

	uint64_t last = 0, published = 0, skipped = 0;
	bool was_requested = false;
	pid_t compositor = 0;
	int64_t report_at = monotonic_ns() + 10000000000ll, check_at = 0, raw_offset = raw_offset_ns();
	static struct pico4_roomview_frame frame;
	while (!stop) {
		// The compositor imports the frames: set them up once it runs, and again after it restarted.
		if (monotonic_ns() >= check_at) {
			check_at = monotonic_ns() + 1000000000ll;
			raw_offset = raw_offset_ns();
			if (server != 0 && find_process("vrserver") != server) {
				fprintf(stderr, "pico4-roomview: vrserver is gone\n");
				break;
			}
			pid_t now = find_process("vrcompositor");
			if (now != compositor) {
				if (compositor != 0) {
					CALL(pt, SLOT_RELEASE, release_fn)(pt);
					fprintf(stderr, "pico4-roomview: the compositor changed, setting up again\n");
				}
				compositor = 0;
				if (now != 0) {
					if (setup_frames(pt, &desc, &attrs[0][0])) {
						compositor = now;
					} else {
						check_at = monotonic_ns() + 2000000000ll;
					}
				}
			}
		}
		if (compositor == 0) {
			sleep_ms(100);
			continue;
		}
		bool requested = false;
		CALL(pt, SLOT_IS_REQUESTED, is_requested_fn)(pt, BLOCK, &requested);
		__atomic_store_n(&rv->producer_ns, monotonic_ns(), __ATOMIC_RELEASE);
		__atomic_store_n(&rv->requested, requested ? 1u : 0u, __ATOMIC_RELEASE);
		if (requested != was_requested) {
			fprintf(stderr, "pico4-roomview: room view %s\n", requested ? "requested" : "no longer requested");
			was_requested = requested;
		}
		if (!requested || !newest_frame(rv, last, &frame)) {
			sleep_ms(requested ? 2 : 50);
			continue;
		}
		last = frame.sequence;
		if (nv12) {
			// One frame index that both cameras have free, the same image and pose for both.
			int32_t status[PICO4_ROOMVIEW_CAMS][MAX_FRAMES];
			bool ok = true;
			for (int c = 0; c < PICO4_ROOMVIEW_CAMS; c++) {
				ok = ok && CALL(pt, SLOT_GET_FRAME_STATUSES, get_frame_statuses_fn)(pt, BLOCK, (uint32_t)c, status[c],
				                                                                    sizeof(status[c])) == 0;
			}
			uint32_t idx = UINT32_MAX;
			for (uint32_t i = 0; ok && i < frames && idx == UINT32_MAX; i++) {
				if (status[0][i] == FRAME_FREE && status[1][i] == FRAME_FREE) {
					idx = i;
				}
			}
			if (idx == UINT32_MAX) {
				skipped++;
				continue;
			}
			struct image *im = &images[0][idx];
			if (colour && !test_pattern) {
				// The luma rows, then the interleaved Cb, Cr rows, each into its plane.
				for (uint32_t row = 0; row < height; row++) {
					memcpy(im->data + im->layout.offset + (size_t)row * im->layout.rowPitch,
					       frame.pixels + (size_t)row * width, width);
				}
				const uint8_t *uv = frame.pixels + (size_t)width * height;
				for (uint32_t row = 0; row < height / 2; row++) {
					memcpy(im->data + im->uv.offset + (size_t)row * im->uv.rowPitch, uv + (size_t)row * width, width);
				}
			}
			for (int c = 0; c < PICO4_ROOMVIEW_CAMS; c++) {
				struct frame_metadata meta;
				metadata(rv, &frame, 0, raw_offset, &meta);
				CALL(pt, SLOT_SET_FRAME_METADATA, set_frame_metadata_fn)(pt, BLOCK, (uint32_t)c, idx, &meta);
			}
			for (int c = 0; c < PICO4_ROOMVIEW_CAMS; c++) {
				if (CALL(pt, SLOT_PUBLISH_FRAME, publish_frame_fn)(pt, BLOCK, (uint32_t)c, idx) == 0) {
					published++;
				}
			}
		}
		for (int c = 0; c < PICO4_ROOMVIEW_CAMS && !nv12; c++) {
			int32_t status[MAX_FRAMES];
			if (CALL(pt, SLOT_GET_FRAME_STATUSES, get_frame_statuses_fn)(pt, BLOCK, (uint32_t)c, status,
			                                                               sizeof(status)) != 0) {
				continue;
			}
			uint32_t idx = UINT32_MAX;
			for (uint32_t i = 0; i < frames; i++) {
				if (status[i] == FRAME_FREE) {
					idx = i;
					break;
				}
			}
			if (idx == UINT32_MAX) {
				skipped++; // the compositor holds every frame: skip this one
				continue;
			}
			struct image *im = &images[c][idx];
			const uint8_t *src = frame.pixels + (size_t)c * width * height;
			for (uint32_t row = 0; row < height; row++) {
				uint8_t *dst = im->data + im->layout.offset + (size_t)row * im->layout.rowPitch;
				const uint8_t *in = src + (size_t)row * width;
				for (uint32_t x = 0; x < width; x++) {
					dst[x] = encode[in[x]];
				}
			}
			struct frame_metadata meta;
			metadata(rv, &frame, c, raw_offset, &meta);
			CALL(pt, SLOT_SET_FRAME_METADATA, set_frame_metadata_fn)(pt, BLOCK, (uint32_t)c, idx, &meta);
			if (CALL(pt, SLOT_PUBLISH_FRAME, publish_frame_fn)(pt, BLOCK, (uint32_t)c, idx) == 0) {
				published++;
			}
		}
		if (monotonic_ns() >= report_at) {
			fprintf(stderr, "pico4-roomview: %llu frames published, %llu skipped (all held)\n",
			        (unsigned long long)published, (unsigned long long)skipped);
			report_at = monotonic_ns() + 10000000000ll;
		}
	}
	__atomic_store_n(&rv->requested, 0u, __ATOMIC_RELEASE);
	CALL(pt, SLOT_RELEASE, release_fn)(pt);
	vr_shutdown();
	// Without SteamVR's server the unit starts again with SteamVR (pico4-roomview.service).
	return stop ? 0 : 1;
}
