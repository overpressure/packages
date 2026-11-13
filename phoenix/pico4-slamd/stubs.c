// Copyright 2026, overpressure contributors.
// SPDX-License-Identifier: Unlicense
/*
 * Preloaded into PICO OS's tracker processes (libpico4-slam-stubs.so): there is no logd and no
 * property area in the tracker's root, so Android's log calls go to stderr (the journal) and
 * property lookups find nothing, which the tracker's defaults cover. Verbose and debug log
 * lines are dropped unless PICO4_SLAMD_DEBUG is set. libphotoid's CVP session is answered here
 * too (below): the pipeline opens one but takes its depth from a network on the Compute DSP.
 */
#include "bionic.h"

static const char prio[] = "??VDIWEFS";

int
__android_log_vprint(int p, const char *tag, const char *fmt, va_list ap)
{
	static int debug = -1;
	if (debug < 0) {
		const char *d = getenv("PICO4_SLAMD_DEBUG");
		debug = d != NULL && *d != '\0' && *d != '0';
	}
	if (p < 4 && !debug) { // ANDROID_LOG_INFO
		return 0;
	}
	char line[1024];
	vsnprintf(line, sizeof line, fmt, ap);
	fprintf(stderr, "%c/%s: %s\n", p >= 0 && p < 9 ? prio[p] : '?', tag ? tag : "", line);
	return 1;
}

int
__android_log_print(int p, const char *tag, const char *fmt, ...)
{
	va_list ap;
	va_start(ap, fmt);
	int r = __android_log_vprint(p, tag, fmt, ap);
	va_end(ap);
	return r;
}

int
__android_log_write(int p, const char *tag, const char *msg)
{
	return __android_log_print(p, tag, "%s", msg);
}

int
__android_log_buf_print(int buf, int p, const char *tag, const char *fmt, ...)
{
	(void)buf;
	va_list ap;
	va_start(ap, fmt);
	int r = __android_log_vprint(p, tag, fmt, ap);
	va_end(ap);
	return r;
}

// Properties: none set; the tracker's own defaults apply.
int
__system_property_get(const char *name, char *value)
{
	(void)name;
	value[0] = 0;
	return 0;
}

const void *
__system_property_find(const char *name)
{
	(void)name;
	return 0;
}

int
__system_property_set(const char *name, const char *value)
{
	(void)name;
	(void)value;
	return 0;
}

/*
 * The CVP session of libphotoid's stereo node (docs/xr.md#passthrough). vst_releaseA computes
 * depth with FLNet on the Compute DSP: the node sets up a depth-from-stereo session and its
 * buffers, and never runs it. Without these it would need /dev/cvp, which Valve's kernel has not.
 * cvpQueryImageInfo stays libcvp2's own: it only calculates.
 */
struct cvp_mem
{
	int32_t fd;
	uint32_t size;
	uint64_t reserved;
	void *data; //!< libphotoid reads the address at 0x10.
};

struct cvp_dfs_config
{
	uint32_t fps, operational_fps, format, width, height;
};

struct cvp_dfs_requirements
{
	uint32_t disparity_bytes, confidence_bytes;
};

void *
cvpCreateSession(void *callback, void *event_callback, void *user)
{
	(void)callback, (void)event_callback, (void)user;
	return calloc(1, 8);
}

int
cvpStartSession(void *session)
{
	(void)session;
	return 0;
}

int
cvpStopSession(void *session)
{
	(void)session;
	return 0;
}

int
cvpDeleteSession(void *session)
{
	free(session);
	return 0;
}

void *
cvpInitDfs(void *session, const struct cvp_dfs_config *config, struct cvp_dfs_requirements *out, void *callback,
           void *user)
{
	(void)session, (void)callback, (void)user;
	// As the node reads them: 32 bits per pixel, and one confidence bit.
	out->disparity_bytes = config->width * config->height * 4;
	out->confidence_bytes = config->width * config->height / 8;
	return calloc(1, 8);
}

int
cvpDeInitDfs(void *dfs)
{
	free(dfs);
	return 0;
}

int
cvpDfs_Sync(void *dfs, const void *left, const void *right, uint32_t a, uint32_t b, void *out)
{
	(void)dfs, (void)left, (void)right, (void)a, (void)b, (void)out;
	fprintf(stderr, "pico4-slamd: libphotoid asked for the CVP's depth, which is not there\n");
	return -1;
}

int
cvpMemAlloc(void *session, uint32_t size, uint32_t secure, struct cvp_mem **out)
{
	(void)session, (void)secure;
	struct cvp_mem *mem = calloc(1, sizeof(*mem));
	void *data = calloc(1, size);
	if (mem == NULL || data == NULL) {
		free(mem);
		free(data);
		return -1;
	}
	mem->fd = -1;
	mem->size = size;
	mem->data = data;
	*out = mem;
	return 0;
}

int
cvpMemFree(void *session, struct cvp_mem *mem)
{
	(void)session;
	if (mem != NULL) {
		free(mem->data);
		free(mem);
	}
	return 0;
}
