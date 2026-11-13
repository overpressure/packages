// Copyright 2026, overpressure contributors.
// SPDX-License-Identifier: Unlicense
/*
 * The tracker root's kernel interfaces on a mainline kernel (docs/xr.md#head-tracking): PICO OS's
 * libcdsprpc speaks the vendor kernel's FastRPC ioctls on /dev/adsprpc-smd and allocates its
 * buffers through ION on /dev/ion; a mainline kernel has the upstream FastRPC driver
 * (/dev/fastrpc-<dsp>) and DMA-BUF heaps (/dev/dma_heap/system) instead. This translates one
 * vendor call into the upstream one. The calls and their sizes are the ones the stock
 * libcdsprpc.so and libion.so build; the remote protocol on the DSP is the same, so
 * only the ioctl layer changes.
 *
 * Header only, without a C library: the bionic preload (fastrpc-bridge.c) and the host test
 * (test_fastrpc_bridge.c) supply the system calls through struct fb_ops.
 */
#pragma once
#include <stddef.h>
#include <stdint.h>

#define FB_IOC(dir, type, nr, size) (((uint32_t)(dir) << 30) | ((uint32_t)(size) << 16) | ((uint32_t)(type) << 8) | (uint32_t)(nr))
#define FB_IOWR(type, nr, size) FB_IOC(3, type, nr, size)
#define FB_IO(type, nr) FB_IOC(0, type, nr, 0)

#define FB_EINVAL 22
#define FB_ENOTTY 25

/* The vendor kernel's FastRPC ABI, as the stock libcdsprpc uses it (64-bit layouts). */
union fb_ds_arg
{
	struct
	{
		uint64_t pv;
		uint64_t len;
	} buf;
	struct
	{
		int32_t fd;
		uint32_t offset;
	} dma;
	uint32_t h;
};

struct fb_ds_invoke
{
	uint32_t handle;
	uint32_t sc;
	uint64_t pra; // union fb_ds_arg *
};

struct fb_ds_invoke_crc
{
	struct fb_ds_invoke inv;
	uint64_t fds;   // int *, may be 0
	uint64_t attrs; // unsigned *, may be 0
	uint64_t crc;   // unsigned *, ignored
};

struct fb_ds_init_attrs
{
	uint32_t flags;
	uint32_t pad0;
	uint64_t file;
	uint32_t filelen;
	int32_t filefd;
	uint64_t mem;
	uint32_t memlen;
	int32_t memfd;
	int32_t attrs;
	uint32_t siglen;
};

struct fb_ds_mmap
{
	int32_t fd;
	uint32_t flags;
	uint64_t vaddrin;
	uint64_t size;
	uint64_t vaddrout;
};

struct fb_ds_munmap
{
	uint64_t vaddrout;
	uint64_t size;
};

#define FB_DS_INVOKE FB_IOWR('R', 1, 16)
#define FB_DS_MMAP FB_IOWR('R', 2, 32)
#define FB_DS_MUNMAP FB_IOWR('R', 3, 16)
#define FB_DS_INVOKE_FD FB_IOWR('R', 4, 24)
#define FB_DS_SETMODE FB_IOWR('R', 5, 4)
#define FB_DS_INIT FB_IOWR('R', 6, 40)
#define FB_DS_INVOKE_ATTRS FB_IOWR('R', 7, 32)
#define FB_DS_GETINFO FB_IOWR('R', 8, 4)
#define FB_DS_GETPERF FB_IOWR('R', 9, 24)
#define FB_DS_INIT_ATTRS FB_IOWR('R', 10, 48)
#define FB_DS_INVOKE_CRC FB_IOWR('R', 11, 40)
#define FB_DS_CONTROL FB_IOWR('R', 12, 12)
#define FB_DS_MUNMAP_FD FB_IOWR('R', 13, 24)

#define FB_DS_CONTROL_KALLOC 3

#define FB_DS_INIT_ATTACH 0
#define FB_DS_INIT_CREATE 1
#define FB_DS_INIT_CREATE_STATIC 2
#define FB_DS_INIT_ATTACH_SENSORS 3

/* The upstream driver's ABI (include/uapi/misc/fastrpc.h). */
struct fb_up_invoke_args
{
	uint64_t ptr;
	uint64_t length;
	int32_t fd;
	uint32_t attr;
};

struct fb_up_invoke
{
	uint32_t handle;
	uint32_t sc;
	uint64_t args;
};

struct fb_up_init_create
{
	uint32_t filelen;
	int32_t filefd;
	uint32_t attrs;
	uint32_t siglen;
	uint64_t file;
};

struct fb_up_init_create_static
{
	uint32_t namelen;
	uint32_t memlen;
	uint64_t name;
};

#define FB_UP_INVOKE FB_IOWR('R', 3, 16)
#define FB_UP_INIT_ATTACH FB_IO('R', 4)
#define FB_UP_INIT_CREATE FB_IOWR('R', 5, 24)
#define FB_UP_MMAP FB_IOWR('R', 6, 32)
#define FB_UP_MUNMAP FB_IOWR('R', 7, 16)
#define FB_UP_INIT_ATTACH_SNS FB_IO('R', 8)
#define FB_UP_INIT_CREATE_STATIC FB_IOWR('R', 9, 16)

/* ION as the stock libion and libcdsprpc call it, and the DMA-BUF heap that replaces it. */
struct fb_ion_alloc
{
	uint64_t len;
	uint32_t heap_id_mask;
	uint32_t flags;
	uint32_t fd;
	uint32_t unused;
};

struct fb_ion_heap_data
{
	char name[32];
	uint32_t type;
	uint32_t heap_id;
	uint32_t reserved0, reserved1, reserved2;
};

struct fb_ion_heap_query
{
	uint32_t cnt;
	uint32_t reserved0;
	uint64_t heaps; // struct fb_ion_heap_data *
	uint32_t reserved1, reserved2;
};

#define FB_ION_ALLOC FB_IOWR('I', 0, 24)
#define FB_ION_FREE FB_IOWR('I', 1, 4)
#define FB_ION_HEAP_QUERY FB_IOWR('I', 8, 24)

struct fb_heap_alloc
{
	uint64_t len;
	uint32_t fd;
	uint32_t fd_flags;
	uint64_t heap_flags;
};

#define FB_HEAP_ALLOC FB_IOWR('H', 0, 24)

/* The system heap's id on the stock kernel (ION_SYSTEM_HEAP_ID), as the DSP libraries ask for it. */
#define FB_ION_SYSTEM_HEAP_ID 25
#define FB_ION_HEAP_TYPE_SYSTEM 0

#define FB_O_RDWR 02
#define FB_O_CLOEXEC 02000000

/* The FastRPC device of each domain (the vendor GETINFO argument). */
static inline const char *
fb_domain_node(uint32_t domain)
{
	switch (domain) {
	case 0: return "/dev/fastrpc-adsp";
	case 2: return "/dev/fastrpc-sdsp";
	case 3: return "/dev/fastrpc-cdsp";
	default: return NULL;
	}
}

/* What the bridge needs from the system: the raw calls, so that it never calls itself. */
struct fb_ops
{
	long (*ioctl)(int fd, uint32_t req, void *arg);
	int (*open)(const char *path, int flags);
	int (*dup3)(int from, int to, int flags);
	int (*close)(int fd);
	void *ctx;
};

/* Number of arguments an invocation carries: in and out buffers, in and out handles. */
static inline uint32_t
fb_nargs(uint32_t sc)
{
	return ((sc >> 16) & 0xff) + ((sc >> 8) & 0xff) + ((sc >> 4) & 0x0f) + (sc & 0x0f);
}

#define FB_MAX_ARGS 255

/*
 * One vendor FastRPC call on an upstream descriptor. GETINFO opens the domain's device onto the
 * same descriptor number (the vendor library opened one node for every DSP). Returns the
 * upstream call's result, or a negative errno.
 */
static inline long
fb_fastrpc(const struct fb_ops *ops, int fd, uint32_t req, void *arg)
{
	switch (req) {
	case FB_DS_GETINFO: {
		uint32_t *info = arg;
		const char *node = fb_domain_node(*info);
		if (node == NULL) {
			return -FB_EINVAL;
		}
		int real = ops->open(node, FB_O_RDWR | FB_O_CLOEXEC);
		if (real < 0) {
			return real;
		}
		if (real != fd) {
			int r = ops->dup3(real, fd, FB_O_CLOEXEC);
			ops->close(real);
			if (r < 0) {
				return r;
			}
		}
		*info = 1; // the session sits behind the SMMU, as on the stock kernel
		return 0;
	}
	case FB_DS_INIT:
	case FB_DS_INIT_ATTRS: {
		const struct fb_ds_init_attrs *in = arg;
		switch (in->flags) {
		case FB_DS_INIT_ATTACH: return ops->ioctl(fd, FB_UP_INIT_ATTACH, NULL);
		case FB_DS_INIT_ATTACH_SENSORS: return ops->ioctl(fd, FB_UP_INIT_ATTACH_SNS, NULL);
		case FB_DS_INIT_CREATE: {
			struct fb_up_init_create c = {
			    .filelen = in->filelen,
			    .filefd = in->filefd,
			    .attrs = req == FB_DS_INIT_ATTRS ? (uint32_t)in->attrs : 0,
			    .siglen = req == FB_DS_INIT_ATTRS ? in->siglen : 0,
			    .file = in->file,
			};
			return ops->ioctl(fd, FB_UP_INIT_CREATE, &c);
		}
		case FB_DS_INIT_CREATE_STATIC: {
			struct fb_up_init_create_static c = {
			    .namelen = in->filelen,
			    .memlen = in->memlen,
			    .name = in->file,
			};
			return ops->ioctl(fd, FB_UP_INIT_CREATE_STATIC, &c);
		}
		default: return -FB_EINVAL;
		}
	}
	case FB_DS_INVOKE:
	case FB_DS_INVOKE_FD:
	case FB_DS_INVOKE_ATTRS:
	case FB_DS_INVOKE_CRC: {
		// The four vendor forms share their leading fields; the later ones add fds and attrs.
		const struct fb_ds_invoke_crc *in = arg;
		const union fb_ds_arg *pra = (const union fb_ds_arg *)(uintptr_t)in->inv.pra;
		const int32_t *fds = req == FB_DS_INVOKE ? NULL : (const int32_t *)(uintptr_t)in->fds;
		const uint32_t *attrs =
		    req == FB_DS_INVOKE_ATTRS || req == FB_DS_INVOKE_CRC ? (const uint32_t *)(uintptr_t)in->attrs : NULL;
		uint32_t n = fb_nargs(in->inv.sc);
		struct fb_up_invoke_args args[FB_MAX_ARGS];
		for (uint32_t i = 0; i < n; i++) {
			args[i].ptr = pra[i].buf.pv;
			args[i].length = pra[i].buf.len;
			args[i].fd = fds != NULL ? fds[i] : -1;
			args[i].attr = attrs != NULL ? attrs[i] : 0;
		}
		struct fb_up_invoke up = {
		    .handle = in->inv.handle,
		    .sc = in->inv.sc,
		    .args = (uint64_t)(uintptr_t)args,
		};
		return ops->ioctl(fd, FB_UP_INVOKE, &up);
	}
	case FB_DS_MMAP:
		// Same layout upstream, under another number.
		return ops->ioctl(fd, FB_UP_MMAP, arg);
	case FB_DS_MUNMAP:
		return ops->ioctl(fd, FB_UP_MUNMAP, arg);
	case FB_DS_CONTROL: {
		// The library asks whether the kernel allocates what it maps for the DSP (the vendor driver's
		// FASTRPC_CONTROL_KALLOC). It does, here too: upstream's MMAP allocates by itself, and
		// refuses the user-allocated pages the library would map otherwise.
		uint32_t *control = (uint32_t *)arg;
		if (control[0] == FB_DS_CONTROL_KALLOC) {
			control[1] = 1;
		}
		return 0;
	}
	case FB_DS_SETMODE:
	case FB_DS_MUNMAP_FD:
		// Session modes, latency and wake-lock hints, and unmapping by fd, which the upstream
		// driver handles on its own (mappings go with their buffers).
		return 0;
	default:
		return -FB_ENOTTY;
	}
}

/*
 * One ION call on the bridge's ION descriptor. The legacy FREE probe fails as on a kernel with
 * the new ION ABI, so libion takes that ABI; every allocation comes from the system DMA-BUF heap.
 */
static inline long
fb_ion(const struct fb_ops *ops, uint32_t req, void *arg)
{
	switch (req) {
	case FB_ION_ALLOC: {
		struct fb_ion_alloc *a = arg;
		int heap = ops->open("/dev/dma_heap/system", FB_O_RDWR | FB_O_CLOEXEC);
		if (heap < 0) {
			return heap;
		}
		struct fb_heap_alloc h = {.len = a->len, .fd_flags = FB_O_RDWR | FB_O_CLOEXEC};
		long r = ops->ioctl(heap, FB_HEAP_ALLOC, &h);
		ops->close(heap);
		if (r < 0) {
			return r;
		}
		a->fd = h.fd;
		return 0;
	}
	case FB_ION_HEAP_QUERY: {
		struct fb_ion_heap_query *q = arg;
		if (q->heaps != 0 && q->cnt >= 1) {
			struct fb_ion_heap_data *d = (struct fb_ion_heap_data *)(uintptr_t)q->heaps;
			for (size_t i = 0; i < sizeof(d->name); i++) {
				d->name[i] = "system"[i < 6 ? i : 6];
			}
			d->type = FB_ION_HEAP_TYPE_SYSTEM;
			d->heap_id = FB_ION_SYSTEM_HEAP_ID;
			d->reserved0 = d->reserved1 = d->reserved2 = 0;
		}
		q->cnt = 1;
		return 0;
	}
	default:
		// FREE (the legacy probe), MAP, SHARE, IMPORT, SYNC: the legacy ABI, absent.
		return -FB_ENOTTY;
	}
}
