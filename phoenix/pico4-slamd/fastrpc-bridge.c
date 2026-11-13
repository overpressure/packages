// Copyright 2026, overpressure contributors.
// SPDX-License-Identifier: Unlicense
/*
 * libpico4-fastrpc-bridge.so, preloaded into the tracker root's processes on a mainline kernel
 * (docs/xr.md#head-tracking): the vendor FastRPC node (/dev/adsprpc-smd, -secure) and /dev/ion
 * the stock libraries open become descriptors whose ioctls fastrpc_bridge.h translates onto the
 * upstream FastRPC driver and the system DMA-BUF heap. Every other path and descriptor goes to
 * the kernel unchanged. The bridge calls the kernel directly (syscall), never itself.
 */
#include "bionic.h"
#include "fastrpc_bridge.h"

long syscall(long, ...);
int *__errno(void);

#define NR_DUP3 24
#define NR_IOCTL 29
#define NR_OPENAT 56
#define NR_CLOSE 57
#define NR_MEMFD_CREATE 279
#define AT_FDCWD (-100)
#define MFD_CLOEXEC 1
#define O_CREAT_OR_TMPFILE (O_CREAT | 020000000)

enum kind
{
	KIND_NONE,
	KIND_FASTRPC,
	KIND_ION,
};

#define MAX_FD 4096
static unsigned char kinds[MAX_FD];

static long
sys_result(long r)
{
	return r < 0 ? -(long)*__errno() : r;
}

static long
op_ioctl(int fd, uint32_t req, void *arg)
{
	return sys_result(syscall(NR_IOCTL, fd, (unsigned long)req, arg));
}

static int
op_open(const char *path, int flags)
{
	return (int)sys_result(syscall(NR_OPENAT, AT_FDCWD, path, flags, 0));
}

static int
op_dup3(int from, int to, int flags)
{
	return (int)sys_result(syscall(NR_DUP3, from, to, flags));
}

static int
op_close(int fd)
{
	return (int)sys_result(syscall(NR_CLOSE, fd));
}

static const struct fb_ops ops = {op_ioctl, op_open, op_dup3, op_close, NULL};

static enum kind
kind_of(const char *path)
{
	if (strcmp(path, "/dev/adsprpc-smd") == 0 || strcmp(path, "/dev/adsprpc-smd-secure") == 0) {
		return KIND_FASTRPC;
	}
	if (strcmp(path, "/dev/ion") == 0) {
		return KIND_ION;
	}
	return KIND_NONE;
}

/* A descriptor that stands for the vendor node until GETINFO (or ION) gives it a meaning. */
static int
placeholder(const char *path, int flags, enum kind k)
{
	long fd = syscall(NR_MEMFD_CREATE, path, (flags & O_CLOEXEC) ? MFD_CLOEXEC : 0);
	if (fd >= 0 && fd < MAX_FD) {
		kinds[fd] = (unsigned char)k;
	}
	return (int)fd;
}

static int
bridge_open(int dirfd, const char *path, int flags, mode_t mode)
{
	enum kind k = path != NULL ? kind_of(path) : KIND_NONE;
	if (k != KIND_NONE) {
		return placeholder(path, flags, k);
	}
	return (int)syscall(NR_OPENAT, dirfd, path, flags, mode);
}

int
open(const char *path, int flags, ...)
{
	mode_t mode = 0;
	if (flags & O_CREAT_OR_TMPFILE) {
		va_list ap;
		va_start(ap, flags);
		mode = va_arg(ap, mode_t);
		va_end(ap);
	}
	return bridge_open(AT_FDCWD, path, flags, mode);
}

int
__open_2(const char *path, int flags)
{
	return bridge_open(AT_FDCWD, path, flags, 0);
}

int
openat(int dirfd, const char *path, int flags, ...)
{
	mode_t mode = 0;
	if (flags & O_CREAT_OR_TMPFILE) {
		va_list ap;
		va_start(ap, flags);
		mode = va_arg(ap, mode_t);
		va_end(ap);
	}
	return bridge_open(dirfd, path, flags, mode);
}

int
__openat_2(int dirfd, const char *path, int flags)
{
	return bridge_open(dirfd, path, flags, 0);
}

int
ioctl(int fd, int request, ...)
{
	va_list ap;
	va_start(ap, request);
	void *arg = va_arg(ap, void *);
	va_end(ap);
	enum kind k = fd >= 0 && fd < MAX_FD ? (enum kind)kinds[fd] : KIND_NONE;
	long r;
	switch (k) {
	case KIND_FASTRPC: r = fb_fastrpc(&ops, fd, (uint32_t)request, arg); break;
	case KIND_ION: r = fb_ion(&ops, (uint32_t)request, arg); break;
	default: return (int)syscall(NR_IOCTL, fd, (unsigned long)(uint32_t)request, arg);
	}
	if (r < 0) {
		*__errno() = (int)-r;
		return -1;
	}
	return (int)r;
}

int
close(int fd)
{
	if (fd >= 0 && fd < MAX_FD) {
		kinds[fd] = KIND_NONE;
	}
	return (int)syscall(NR_CLOSE, fd);
}
