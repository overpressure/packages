// Copyright 2026, overpressure contributors.
// SPDX-License-Identifier: Unlicense
/*
 * libandroid.so for PICO OS's tracker: it needs only these two trace calls from it, and the
 * real library would pull in most of Android. The auto exposure's shared-memory library
 * (libtianti_shm_dynamic.so) binds the hardware-buffer calls at load time but uses them only
 * between processes, which the auto exposure does not: they fail.
 */
#include <stdint.h>

#define NOT_SUPPORTED (-38) // -ENOSYS

void
ATrace_beginSection(const char *name)
{
	(void)name;
}

void
ATrace_endSection(void)
{
}

int
AHardwareBuffer_allocate(const void *desc, void **out)
{
	(void)desc;
	*out = 0;
	return NOT_SUPPORTED;
}

int
AHardwareBuffer_lock(void *buffer, uint64_t usage, int32_t fence, const void *rect, void **out)
{
	(void)buffer;
	(void)usage;
	(void)fence;
	(void)rect;
	*out = 0;
	return NOT_SUPPORTED;
}

int
AHardwareBuffer_unlock(void *buffer, int32_t *fence)
{
	(void)buffer;
	(void)fence;
	return NOT_SUPPORTED;
}

void
AHardwareBuffer_release(void *buffer)
{
	(void)buffer;
}

int
AHardwareBuffer_sendHandleToUnixSocket(const void *buffer, int socket)
{
	(void)buffer;
	(void)socket;
	return NOT_SUPPORTED;
}

int
AHardwareBuffer_recvHandleFromUnixSocket(int socket, void **out)
{
	(void)socket;
	*out = 0;
	return NOT_SUPPORTED;
}
