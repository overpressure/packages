// Copyright 2026, overpressure contributors.
// SPDX-License-Identifier: BSL-1.0
/*
 * What the project's SteamVR overlays share (pico4-lens-panel, pico4-onboarding): OpenVR opened at
 * run time (nothing of Valve's is linked), cairo images handed to SteamVR as Vulkan textures, and
 * a few drawing helpers.
 *
 * The images go to SteamVR as Vulkan textures: with one through SetOverlayRaw or SetOverlayFromFile
 * alive, the compositor crashes in its cleanup at every stop (docs/session.md).
 */
#pragma once

#include <cairo/cairo.h>
#include <dlfcn.h>
#include <signal.h>
#include <time.h>
#include <unistd.h>
#include <vulkan/vulkan.h>

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#include "openvr.h"

namespace pico4_overlay {

constexpr const char *kOpenvrApi = "/opt/steamvr/bin/linuxarm64/libopenvr_api.so";
constexpr const char *kFont = "Adwaita Sans";

inline double
now_s()
{
	timespec t;
	clock_gettime(CLOCK_MONOTONIC, &t);
	return t.tv_sec + t.tv_nsec / 1e9;
}

//! SteamVR's interfaces for an overlay application, and how to let go of them.
struct Vr
{
	vr::IVROverlay *overlay = nullptr;
	vr::IVRSystem *system = nullptr;
	vr::IVRCompositor *compositor = nullptr;
	vr::IVRSettings *settings = nullptr; //!< may stay null: only some tools change settings
	void *(*get_interface)(const char *, vr::EVRInitError *) = nullptr; //!< any other of SteamVR's interfaces
	void (*shutdown)() = nullptr;
};

/*!
 * Open libopenvr_api.so and connect as @p type (a background application by default), trying for
 * two minutes while SteamVR starts. Returns 0, 1 on an error (printed with @p tool), or 2 when
 * @p stop was set first.
 */
inline int
vr_open(const char *tool, Vr &out, volatile sig_atomic_t *stop, vr::EVRApplicationType type = vr::VRApplication_Background)
{
	void *lib = dlopen(kOpenvrApi, RTLD_NOW);
	if (lib == nullptr) {
		fprintf(stderr, "%s: %s\n", tool, dlerror());
		return 1;
	}
	// openvr.h's own entry point, with its startup information: the older two-argument one would
	// leave that argument to whatever the register holds, and vrclient copies it (strncpy) later.
	auto vr_init = (uint32_t(*)(vr::EVRInitError *, vr::EVRApplicationType, const char *))dlsym(lib, "VR_InitInternal2");
	auto vr_interface = (void *(*)(const char *, vr::EVRInitError *))dlsym(lib, "VR_GetGenericInterface");
	out.shutdown = (void (*)())dlsym(lib, "VR_ShutdownInternal");
	out.get_interface = vr_interface;
	if (vr_init == nullptr || vr_interface == nullptr || out.shutdown == nullptr) {
		fprintf(stderr, "%s: not an OpenVR library\n", tool);
		return 1;
	}
	vr::EVRInitError err = vr::VRInitError_Driver_Failed;
	for (int i = 0; i < 60 && err != vr::VRInitError_None && !*stop; i++) {
		vr_init(&err, type, nullptr);
		if (err != vr::VRInitError_None && !*stop) {
			sleep(2);
		}
	}
	if (*stop && err != vr::VRInitError_None) {
		return 2;
	}
	if (err != vr::VRInitError_None) {
		fprintf(stderr, "%s: VR_Init: %d\n", tool, err);
		return 1;
	}
	out.overlay = static_cast<vr::IVROverlay *>(vr_interface(vr::IVROverlay_Version, &err));
	out.system = static_cast<vr::IVRSystem *>(vr_interface(vr::IVRSystem_Version, &err));
	out.compositor = static_cast<vr::IVRCompositor *>(vr_interface(vr::IVRCompositor_Version, &err));
	vr::EVRInitError settings_err = vr::VRInitError_None;
	out.settings = static_cast<vr::IVRSettings *>(vr_interface(vr::IVRSettings_Version, &settings_err));
	if (out.overlay == nullptr || out.system == nullptr || out.compositor == nullptr) {
		fprintf(stderr, "%s: no overlay interface: %d\n", tool, err);
		return 1;
	}
	return 0;
}

inline void
rounded(cairo_t *c, double x, double y, double w, double h, double r)
{
	cairo_new_sub_path(c);
	cairo_arc(c, x + w - r, y + r, r, -M_PI / 2, 0);
	cairo_arc(c, x + w - r, y + h - r, r, 0, M_PI / 2);
	cairo_arc(c, x + r, y + h - r, r, M_PI / 2, M_PI);
	cairo_arc(c, x + r, y + r, r, M_PI, 1.5 * M_PI);
	cairo_close_path(c);
}

inline void
text_centered(cairo_t *c, const char *text, double x, double y, double size, cairo_font_weight_t weight)
{
	cairo_select_font_face(c, kFont, CAIRO_FONT_SLANT_NORMAL, weight);
	cairo_set_font_size(c, size);
	cairo_text_extents_t e;
	cairo_text_extents(c, text, &e);
	cairo_move_to(c, x - e.width / 2 - e.x_bearing, y);
	cairo_show_text(c, text);
}

//! Cairo's premultiplied native ARGB as SteamVR's RGBA bytes.
inline void
to_rgba(cairo_surface_t *surface, std::vector<uint8_t> &out)
{
	const int w = cairo_image_surface_get_width(surface), h = cairo_image_surface_get_height(surface);
	const int stride = cairo_image_surface_get_stride(surface);
	const uint8_t *data = cairo_image_surface_get_data(surface);
	out.resize((size_t)w * h * 4);
	for (int y = 0; y < h; y++) {
		const uint32_t *in = reinterpret_cast<const uint32_t *>(data + y * stride);
		uint8_t *o = out.data() + (size_t)y * w * 4;
		for (int x = 0; x < w; x++, o += 4) {
			o[0] = (in[x] >> 16) & 255;
			o[1] = (in[x] >> 8) & 255;
			o[2] = in[x] & 255;
			o[3] = in[x] >> 24;
		}
	}
}

inline std::vector<std::string>
split_words(const char *text)
{
	std::vector<std::string> words;
	std::string word;
	for (const char *c = text;; c++) {
		if (*c == ' ' || *c == '\0') {
			if (!word.empty()) {
				words.push_back(word);
			}
			word.clear();
			if (*c == '\0') {
				return words;
			}
		} else {
			word += *c;
		}
	}
}

//! What every overlay image shares: the device SteamVR asks for, and a command buffer.
struct Gpu
{
	VkInstance instance = VK_NULL_HANDLE;
	VkPhysicalDevice physical = VK_NULL_HANDLE;
	VkDevice device = VK_NULL_HANDLE;
	VkQueue queue = VK_NULL_HANDLE;
	uint32_t family = 0;
	VkCommandBuffer commands = VK_NULL_HANDLE;
};

inline bool
gpu_create(Gpu &g, vr::IVRCompositor *compositor, const char *application)
{
	char names[4096] = "";
	compositor->GetVulkanInstanceExtensionsRequired(names, sizeof(names));
	const std::vector<std::string> instance_words = split_words(names);
	std::vector<const char *> instance_extensions;
	for (const std::string &w : instance_words) {
		instance_extensions.push_back(w.c_str());
	}
	VkApplicationInfo app{VK_STRUCTURE_TYPE_APPLICATION_INFO};
	app.pApplicationName = application;
	app.apiVersion = VK_API_VERSION_1_1;
	VkInstanceCreateInfo instance_info{VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO};
	instance_info.pApplicationInfo = &app;
	instance_info.enabledExtensionCount = (uint32_t)instance_extensions.size();
	instance_info.ppEnabledExtensionNames = instance_extensions.data();
	if (vkCreateInstance(&instance_info, nullptr, &g.instance) != VK_SUCCESS) {
		return false;
	}
	uint32_t count = 1;
	if (vkEnumeratePhysicalDevices(g.instance, &count, &g.physical) < 0 || count == 0) {
		return false;
	}
	uint32_t families = 0;
	vkGetPhysicalDeviceQueueFamilyProperties(g.physical, &families, nullptr);
	std::vector<VkQueueFamilyProperties> family_properties(families);
	vkGetPhysicalDeviceQueueFamilyProperties(g.physical, &families, family_properties.data());
	while (g.family < families && !(family_properties[g.family].queueFlags & VK_QUEUE_GRAPHICS_BIT)) {
		g.family++;
	}
	if (g.family == families) {
		return false;
	}
	names[0] = '\0';
	compositor->GetVulkanDeviceExtensionsRequired(g.physical, names, sizeof(names));
	const std::vector<std::string> device_words = split_words(names);
	std::vector<const char *> device_extensions;
	for (const std::string &w : device_words) {
		device_extensions.push_back(w.c_str());
	}
	const float priority = 0;
	VkDeviceQueueCreateInfo queue_info{VK_STRUCTURE_TYPE_DEVICE_QUEUE_CREATE_INFO};
	queue_info.queueFamilyIndex = g.family;
	queue_info.queueCount = 1;
	queue_info.pQueuePriorities = &priority;
	VkDeviceCreateInfo device_info{VK_STRUCTURE_TYPE_DEVICE_CREATE_INFO};
	device_info.queueCreateInfoCount = 1;
	device_info.pQueueCreateInfos = &queue_info;
	device_info.enabledExtensionCount = (uint32_t)device_extensions.size();
	device_info.ppEnabledExtensionNames = device_extensions.data();
	if (vkCreateDevice(g.physical, &device_info, nullptr, &g.device) != VK_SUCCESS) {
		return false;
	}
	vkGetDeviceQueue(g.device, g.family, 0, &g.queue);
	VkCommandPoolCreateInfo pool_info{VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO};
	pool_info.flags = VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT;
	pool_info.queueFamilyIndex = g.family;
	VkCommandPool pool;
	if (vkCreateCommandPool(g.device, &pool_info, nullptr, &pool) != VK_SUCCESS) {
		return false;
	}
	VkCommandBufferAllocateInfo command_info{VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO};
	command_info.commandPool = pool;
	command_info.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
	command_info.commandBufferCount = 1;
	return vkAllocateCommandBuffers(g.device, &command_info, &g.commands) == VK_SUCCESS;
}

inline bool
allocate(const Gpu &g, const VkMemoryRequirements &needs, VkMemoryPropertyFlags flags, VkDeviceMemory *out)
{
	VkPhysicalDeviceMemoryProperties properties;
	vkGetPhysicalDeviceMemoryProperties(g.physical, &properties);
	for (uint32_t i = 0; i < properties.memoryTypeCount; i++) {
		if ((needs.memoryTypeBits & (1u << i)) && (properties.memoryTypes[i].propertyFlags & flags) == flags) {
			VkMemoryAllocateInfo info{VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO};
			info.allocationSize = needs.size;
			info.memoryTypeIndex = i;
			return vkAllocateMemory(g.device, &info, nullptr, out) == VK_SUCCESS;
		}
	}
	return false;
}

//! One overlay's image on the GPU and the buffer each redraw is copied through.
struct Texture
{
	uint32_t width = 0, height = 0;
	VkImage image = VK_NULL_HANDLE;
	VkBuffer staging = VK_NULL_HANDLE;
	void *mapped = nullptr;
	bool written = false;
};

inline bool
texture_create(const Gpu &g, Texture &t, uint32_t width, uint32_t height)
{
	t.width = width;
	t.height = height;
	VkImageCreateInfo image_info{VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO};
	image_info.imageType = VK_IMAGE_TYPE_2D;
	image_info.format = VK_FORMAT_R8G8B8A8_UNORM;
	image_info.extent = {width, height, 1};
	image_info.mipLevels = 1;
	image_info.arrayLayers = 1;
	image_info.samples = VK_SAMPLE_COUNT_1_BIT;
	image_info.tiling = VK_IMAGE_TILING_OPTIMAL;
	image_info.usage = VK_IMAGE_USAGE_TRANSFER_DST_BIT | VK_IMAGE_USAGE_TRANSFER_SRC_BIT | VK_IMAGE_USAGE_SAMPLED_BIT;
	if (vkCreateImage(g.device, &image_info, nullptr, &t.image) != VK_SUCCESS) {
		return false;
	}
	VkMemoryRequirements needs;
	vkGetImageMemoryRequirements(g.device, t.image, &needs);
	VkDeviceMemory image_memory, staging_memory;
	if (!allocate(g, needs, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT, &image_memory) ||
	    vkBindImageMemory(g.device, t.image, image_memory, 0) != VK_SUCCESS) {
		return false;
	}
	VkBufferCreateInfo buffer_info{VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO};
	buffer_info.size = (VkDeviceSize)width * height * 4;
	buffer_info.usage = VK_BUFFER_USAGE_TRANSFER_SRC_BIT;
	if (vkCreateBuffer(g.device, &buffer_info, nullptr, &t.staging) != VK_SUCCESS) {
		return false;
	}
	vkGetBufferMemoryRequirements(g.device, t.staging, &needs);
	return allocate(g, needs, VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT, &staging_memory) &&
	       vkBindBufferMemory(g.device, t.staging, staging_memory, 0) == VK_SUCCESS &&
	       vkMapMemory(g.device, staging_memory, 0, VK_WHOLE_SIZE, 0, &t.mapped) == VK_SUCCESS;
}

inline void
image_barrier(VkCommandBuffer commands, VkImage image, VkImageLayout from, VkImageLayout to, VkAccessFlags src, VkAccessFlags dst)
{
	VkImageMemoryBarrier barrier{VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER};
	barrier.srcAccessMask = src;
	barrier.dstAccessMask = dst;
	barrier.oldLayout = from;
	barrier.newLayout = to;
	barrier.srcQueueFamilyIndex = barrier.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
	barrier.image = image;
	barrier.subresourceRange = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1};
	vkCmdPipelineBarrier(commands, VK_PIPELINE_STAGE_TRANSFER_BIT, VK_PIPELINE_STAGE_TRANSFER_BIT, 0, 0, nullptr, 0, nullptr, 1, &barrier);
}

//! The surface's pixels into the texture, left in the layout SteamVR takes it in.
inline bool
texture_upload(const Gpu &g, Texture &t, cairo_surface_t *surface)
{
	std::vector<uint8_t> rgba;
	to_rgba(surface, rgba);
	memcpy(t.mapped, rgba.data(), rgba.size());
	VkCommandBufferBeginInfo begin{VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO};
	begin.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
	vkBeginCommandBuffer(g.commands, &begin);
	image_barrier(g.commands, t.image, t.written ? VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL : VK_IMAGE_LAYOUT_UNDEFINED,
	              VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, 0, VK_ACCESS_TRANSFER_WRITE_BIT);
	VkBufferImageCopy copy{};
	copy.imageSubresource = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 0, 1};
	copy.imageExtent = {t.width, t.height, 1};
	vkCmdCopyBufferToImage(g.commands, t.staging, t.image, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, 1, &copy);
	image_barrier(g.commands, t.image, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL,
	              VK_ACCESS_TRANSFER_WRITE_BIT, VK_ACCESS_TRANSFER_READ_BIT);
	vkEndCommandBuffer(g.commands);
	VkSubmitInfo submit{VK_STRUCTURE_TYPE_SUBMIT_INFO};
	submit.commandBufferCount = 1;
	submit.pCommandBuffers = &g.commands;
	if (vkQueueSubmit(g.queue, 1, &submit, VK_NULL_HANDLE) != VK_SUCCESS || vkQueueWaitIdle(g.queue) != VK_SUCCESS) {
		return false;
	}
	t.written = true;
	return true;
}

//! The texture as SteamVR takes a Vulkan image: its device, its queue and its size.
inline vr::VRVulkanTextureData_t
vulkan_data(const Gpu &g, const Texture &t)
{
	vr::VRVulkanTextureData_t data{};
	data.m_nImage = (uint64_t)t.image;
	data.m_pDevice = g.device;
	data.m_pPhysicalDevice = g.physical;
	data.m_pInstance = g.instance;
	data.m_pQueue = g.queue;
	data.m_nQueueFamilyIndex = g.family;
	data.m_nWidth = t.width;
	data.m_nHeight = t.height;
	data.m_nFormat = VK_FORMAT_R8G8B8A8_UNORM;
	data.m_nSampleCount = 1;
	return data;
}

//! The surface's pixels into the texture, then the texture to SteamVR as the overlay's image.
inline bool
texture_show(const Gpu &g, Texture &t, cairo_surface_t *surface, vr::IVROverlay *overlay, vr::VROverlayHandle_t handle)
{
	if (!texture_upload(g, t, surface)) {
		return false;
	}
	vr::VRVulkanTextureData_t data = vulkan_data(g, t);
	vr::Texture_t texture{&data, vr::TextureType_Vulkan, vr::ColorSpace_Auto};
	return overlay->SetOverlayTexture(handle, &texture) == vr::VROverlayError_None;
}

} // namespace pico4_overlay
