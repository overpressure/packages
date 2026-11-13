// Copyright 2026, overpressure contributors.
// SPDX-License-Identifier: Unlicense
/*
 * Native test of fastrpc_bridge.h: the vendor requests are the constants the stock libcdsprpc.so
 * and libion.so build, and each one becomes the upstream call with the same fields.
 */
#include <assert.h>
#include <stdio.h>
#include <string.h>

#include "fastrpc_bridge.h"

static struct
{
	int calls;
	uint32_t req;
	unsigned char arg[256];
	int ioctl_fd;
	char opened[64];
	int dup_from, dup_to;
	int closed;
	long heap_fd;
	struct fb_up_invoke_args args[8];
} rec;

static long
fake_ioctl(int fd, uint32_t req, void *arg)
{
	rec.calls++;
	rec.req = req;
	rec.ioctl_fd = fd;
	if (arg != NULL) {
		memcpy(rec.arg, arg, (req >> 16) & 0x3fff);
	}
	if (req == FB_UP_INVOKE) {
		// The arguments live on the bridge's stack: copy them while the call runs.
		const struct fb_up_invoke *up = arg;
		uint32_t n = fb_nargs(up->sc);
		memcpy(rec.args, (const void *)(uintptr_t)up->args, (n < 8 ? n : 8) * sizeof rec.args[0]);
	}
	if (req == FB_HEAP_ALLOC) {
		((struct fb_heap_alloc *)arg)->fd = (uint32_t)rec.heap_fd;
	}
	return 0;
}

static int
fake_open(const char *path, int flags)
{
	(void)flags;
	snprintf(rec.opened, sizeof rec.opened, "%s", path);
	return 40;
}

static int
fake_dup3(int from, int to, int flags)
{
	(void)flags;
	rec.dup_from = from;
	rec.dup_to = to;
	return to;
}

static int
fake_close(int fd)
{
	rec.closed = fd;
	return 0;
}

static const struct fb_ops ops = {fake_ioctl, fake_open, fake_dup3, fake_close, NULL};

static void
reset(void)
{
	memset(&rec, 0, sizeof rec);
}

int
main(void)
{
	// The requests as the stock libraries build them.
	assert(FB_DS_INVOKE == 0xc0105201u && FB_DS_MMAP == 0xc0205202u && FB_DS_MUNMAP == 0xc0105203u);
	assert(FB_DS_INVOKE_FD == 0xc0185204u && FB_DS_SETMODE == 0xc0045205u && FB_DS_INIT == 0xc0285206u);
	assert(FB_DS_INVOKE_ATTRS == 0xc0205207u && FB_DS_GETINFO == 0xc0045208u && FB_DS_GETPERF == 0xc0185209u);
	assert(FB_DS_INIT_ATTRS == 0xc030520au && FB_DS_INVOKE_CRC == 0xc028520bu && FB_DS_CONTROL == 0xc00c520cu);
	assert(FB_DS_MUNMAP_FD == 0xc018520du);
	assert(FB_ION_ALLOC == 0xc0184900u && FB_ION_FREE == 0xc0044901u && FB_ION_HEAP_QUERY == 0xc0184908u);
	// The upstream requests (include/uapi/misc/fastrpc.h, dma-heap.h).
	assert(FB_UP_INVOKE == 0xc0105203u && FB_UP_INIT_ATTACH == 0x5204u && FB_UP_INIT_CREATE == 0xc0185205u);
	assert(FB_UP_MMAP == 0xc0205206u && FB_UP_MUNMAP == 0xc0105207u && FB_UP_INIT_ATTACH_SNS == 0x5208u);
	assert(FB_UP_INIT_CREATE_STATIC == 0xc0105209u && FB_HEAP_ALLOC == 0xc0184800u);
	assert(sizeof(struct fb_ds_init_attrs) == 48 && sizeof(struct fb_ds_invoke_crc) == 40);
	assert(sizeof(struct fb_ds_mmap) == 32 && sizeof(struct fb_up_invoke_args) == 24);
	assert(sizeof(union fb_ds_arg) == 16 && sizeof(struct fb_ion_heap_query) == 24);

	// GETINFO: the domain's upstream node onto the same descriptor; the SMMU flag back.
	reset();
	uint32_t info = 3;
	assert(fb_fastrpc(&ops, 7, FB_DS_GETINFO, &info) == 0);
	assert(strcmp(rec.opened, "/dev/fastrpc-cdsp") == 0 && rec.dup_from == 40 && rec.dup_to == 7);
	assert(rec.closed == 40 && info == 1 && rec.calls == 0);
	info = 9;
	assert(fb_fastrpc(&ops, 7, FB_DS_GETINFO, &info) == -FB_EINVAL);

	// INIT_ATTRS creating the process from its shell, with attributes and signature length.
	reset();
	struct fb_ds_init_attrs init = {.flags = FB_DS_INIT_CREATE, .file = 0x1000, .filelen = 4096, .filefd = 5,
	                                .attrs = 8, .siglen = 0x40};
	assert(fb_fastrpc(&ops, 7, FB_DS_INIT_ATTRS, &init) == 0);
	struct fb_up_init_create c;
	memcpy(&c, rec.arg, sizeof c);
	assert(rec.req == FB_UP_INIT_CREATE && c.filelen == 4096 && c.filefd == 5 && c.attrs == 8 && c.siglen == 0x40);
	assert(c.file == 0x1000);
	// The plain INIT has no attributes.
	reset();
	assert(fb_fastrpc(&ops, 7, FB_DS_INIT, &init) == 0);
	memcpy(&c, rec.arg, sizeof c);
	assert(c.attrs == 0 && c.siglen == 0);
	// Attaching, to the guest OS or the sensors process, and a static process.
	reset();
	init.flags = FB_DS_INIT_ATTACH;
	assert(fb_fastrpc(&ops, 7, FB_DS_INIT_ATTRS, &init) == 0 && rec.req == FB_UP_INIT_ATTACH);
	init.flags = FB_DS_INIT_ATTACH_SENSORS;
	assert(fb_fastrpc(&ops, 7, FB_DS_INIT, &init) == 0 && rec.req == FB_UP_INIT_ATTACH_SNS);
	struct fb_ds_init_attrs st = {.flags = FB_DS_INIT_CREATE_STATIC, .file = 0x2000, .filelen = 8, .memlen = 0x200000};
	assert(fb_fastrpc(&ops, 7, FB_DS_INIT, &st) == 0 && rec.req == FB_UP_INIT_CREATE_STATIC);
	struct fb_up_init_create_static s;
	memcpy(&s, rec.arg, sizeof s);
	assert(s.namelen == 8 && s.memlen == 0x200000 && s.name == 0x2000);

	// An invocation with two in buffers, one out buffer and one in handle, its fds and attributes.
	reset();
	union fb_ds_arg pra[4] = {
	    {.buf = {0x100, 16}}, {.buf = {0x200, 32}}, {.buf = {0x300, 64}}, {.h = 77},
	};
	int32_t fds[4] = {-1, 9, -1, -1};
	uint32_t attrs[4] = {0, 1, 0, 0};
	uint32_t sc = (2u << 16) | (1u << 8) | (1u << 4);
	struct fb_ds_invoke_crc inv = {.inv = {.handle = 3, .sc = sc, .pra = (uintptr_t)pra}, .fds = (uintptr_t)fds,
	                               .attrs = (uintptr_t)attrs};
	assert(fb_nargs(sc) == 4);
	assert(fb_fastrpc(&ops, 7, FB_DS_INVOKE_ATTRS, &inv) == 0 && rec.req == FB_UP_INVOKE);
	struct fb_up_invoke up;
	memcpy(&up, rec.arg, sizeof up);
	assert(up.handle == 3 && up.sc == sc);
	assert(rec.args[0].ptr == 0x100 && rec.args[0].length == 16 && rec.args[0].fd == -1 && rec.args[0].attr == 0);
	assert(rec.args[1].ptr == 0x200 && rec.args[1].length == 32 && rec.args[1].fd == 9 && rec.args[1].attr == 1);
	assert(rec.args[2].ptr == 0x300 && rec.args[2].length == 64 && rec.args[2].fd == -1);
	assert((uint32_t)rec.args[3].ptr == 77 && rec.args[3].fd == -1);
	// The plain form has neither fds nor attributes.
	reset();
	struct fb_ds_invoke plain = {.handle = 1, .sc = (1u << 16), .pra = (uintptr_t)pra};
	assert(fb_fastrpc(&ops, 7, FB_DS_INVOKE, &plain) == 0 && rec.req == FB_UP_INVOKE);
	assert(rec.args[0].ptr == 0x100 && rec.args[0].fd == -1 && rec.args[0].attr == 0);

	// mmap and munmap keep their layouts under the upstream numbers.
	reset();
	struct fb_ds_mmap mm = {.fd = 9, .flags = 0, .vaddrin = 0x4000, .size = 4096};
	assert(fb_fastrpc(&ops, 7, FB_DS_MMAP, &mm) == 0 && rec.req == FB_UP_MMAP);
	assert(memcmp(rec.arg, &mm, sizeof mm) == 0);
	struct fb_ds_munmap um = {.vaddrout = 0x5000, .size = 4096};
	assert(fb_fastrpc(&ops, 7, FB_DS_MUNMAP, &um) == 0 && rec.req == FB_UP_MUNMAP);

	// Hints the upstream driver has no use for; and a request it has no counterpart for.
	reset();
	uint32_t ctl[6] = {0};
	assert(fb_fastrpc(&ops, 7, FB_DS_CONTROL, ctl) == 0 && rec.calls == 0);
	assert(fb_fastrpc(&ops, 7, FB_DS_SETMODE, ctl) == 0 && rec.calls == 0);
	assert(fb_fastrpc(&ops, 7, FB_DS_MUNMAP_FD, ctl) == 0 && rec.calls == 0);
	// Asked whether the kernel allocates what it maps: yes, so the library sends no user pages.
	uint32_t kalloc[3] = {FB_DS_CONTROL_KALLOC, 0, 0};
	assert(fb_fastrpc(&ops, 7, FB_DS_CONTROL, kalloc) == 0 && kalloc[1] == 1 && rec.calls == 0);
	uint32_t latency[3] = {1, 0, 0};
	assert(fb_fastrpc(&ops, 7, FB_DS_CONTROL, latency) == 0 && latency[1] == 0);
	assert(fb_fastrpc(&ops, 7, FB_DS_GETPERF, ctl) == -FB_ENOTTY);

	// ION: the legacy probe fails, allocations come from the system heap, one heap is listed.
	reset();
	uint32_t handle = 0;
	assert(fb_ion(&ops, FB_ION_FREE, &handle) == -FB_ENOTTY);
	rec.heap_fd = 33;
	struct fb_ion_alloc a = {.len = 65536, .heap_id_mask = 1u << FB_ION_SYSTEM_HEAP_ID};
	assert(fb_ion(&ops, FB_ION_ALLOC, &a) == 0 && a.fd == 33);
	assert(strcmp(rec.opened, "/dev/dma_heap/system") == 0 && rec.req == FB_HEAP_ALLOC && rec.closed == 40);
	struct fb_heap_alloc h;
	memcpy(&h, rec.arg, sizeof h);
	assert(h.len == 65536 && h.fd_flags == (FB_O_RDWR | FB_O_CLOEXEC));
	struct fb_ion_heap_query q = {0};
	assert(fb_ion(&ops, FB_ION_HEAP_QUERY, &q) == 0 && q.cnt == 1);
	struct fb_ion_heap_data d;
	q.heaps = (uintptr_t)&d;
	assert(fb_ion(&ops, FB_ION_HEAP_QUERY, &q) == 0);
	assert(strcmp(d.name, "system") == 0 && d.heap_id == FB_ION_SYSTEM_HEAP_ID);

	printf("fastrpc bridge: all translations as expected\n");
	return 0;
}
