// Copyright 2026, overpressure contributors.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Capture from the OV7251 tracking cameras and the IMX471 RGB camera through V4L2 and CAMSS
 *         (see pico4_cam.h).
 *
 * The sensors run in FSIN slave mode: they only send a frame when the controller radio
 * ("station") pulses FSIN, which it does after its camera init/stream commands
 * (pico4_station.h), sent once all sensors stream. The station also sets the exposure.
 *
 * The OV7251s' control port belongs to the station, so the kernel's camera driver (pico4-ov7251)
 * has no registers to set: it powers and clocks a sensor while its stream runs. Each camera has its
 * own pipeline, CSIPHY -> CSID -> VFE RDI -> video node, with PICO OS's PHYs and lanes (the tree
 * holds those). Buffers are stamped with the CSID's hardware start of frame on CLOCK_MONOTONIC,
 * the stamp PICO OS's camera stack reports. The frames come as MIPI RAW10 and are unpacked into the
 * 16-bit samples PICO OS's RDI writes, the module's format for callers.
 *
 * The RGB camera has the same kind of pipeline on the XR variant's third IFE-lite. Its driver
 * (pico4-imx471) programs the sensor and takes the exposure as controls. Its CSID writes 16-bit
 * samples, as PICO's stack has it: the bus cannot write a line of packed RAW10 that is no whole
 * number of its bursts (2910 bytes), and nothing has to unpack 4 million samples.
 *
 * @ingroup drv_pico4
 */

#ifndef _GNU_SOURCE // Monado defines it on the command line
#define _GNU_SOURCE // clock_gettime(CLOCK_BOOTTIME), mmap and friends under -std=c11
#endif

#include "pico4_cam.h"

#include <stddef.h>
#include <stdint.h>
#include <sys/time.h>
#include <time.h>

#include <linux/media.h>
#include <linux/v4l2-subdev.h>
#include <linux/videodev2.h>

#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>

#define IMX471_ANALOG_GAIN_MAX 960 // 16x: gain = 1024 / (1024 - register)


/*
 *
 * State.
 *
 */

struct pico4_cam
{
	struct pico4_cam_config cfg;
	bool started;
	struct pico4_cam_frame last;
	bool have_last;
	struct v4l_capture *v4l;
};


/*
 *
 * Logging.
 *
 */

__attribute__((format(printf, 3, 4))) static void
cam_log(const struct pico4_cam_config *cfg, enum pico4_cam_log_level level, const char *fmt, ...)
{
	char msg[512];
	va_list ap;

	if (cfg != NULL && (int)level > cfg->verbose) {
		return;
	}
	if (cfg == NULL && level != PICO4_CAM_LOG_ERROR) {
		return;
	}
	va_start(ap, fmt);
	vsnprintf(msg, sizeof(msg), fmt, ap);
	va_end(ap);
	if (cfg != NULL && cfg->log != NULL) {
		cfg->log(cfg->log_user, level, msg);
	} else {
		fprintf(stderr, "pico4_cam: %s\n", msg);
	}
}

#define LOG_E(c, ...) cam_log(&(c)->cfg, PICO4_CAM_LOG_ERROR, __VA_ARGS__)
#define LOG_I(c, ...) cam_log(&(c)->cfg, PICO4_CAM_LOG_INFO, __VA_ARGS__)

static int64_t
ts_ns(const struct timespec *t)
{
	return (int64_t)t->tv_sec * 1000000000 + t->tv_nsec;
}

static int64_t
now_ns(clockid_t id);

int64_t
pico4_slept_ns(void)
{
	int64_t b = now_ns(CLOCK_BOOTTIME), m = now_ns(CLOCK_MONOTONIC);
	return b - m;
}

static int64_t
now_ns(clockid_t id)
{
	struct timespec t;
	clock_gettime(id, &t);
	return ts_ns(&t);
}


/*
 *
 * V4L2 through CAMSS.
 *
 */

#define V4L_LINES 481 // the sensor's metadata line (pico4_cam_meta_decode) and the 480 scene rows
#define V4L_STRIDE (PICO4_CAM_WIDTH * 2)
#define V4L_MBUS_Y10 0x200a // MEDIA_BUS_FMT_Y10_1X10
#define V4L_MBUS_SRGGB10 0x300f // MEDIA_BUS_FMT_SRGGB10_1X10
#define V4L_MBUS_SRGGB16 0x3020 // MEDIA_BUS_FMT_SRGGB16_1X16: what the CSID makes of it
#define V4L_CID_RGB_PAUSED (V4L2_CID_USER_BASE | 0x1000) // pico4-imx471's: the sensor's stream off

struct v4l_pipe
{
	int phy, phy_source_pad, csid, vfe;
};

//! Each slot's pipeline: the second sensor on CSIPHY 3 is on the PHY's second pads (combo mode).
static const struct v4l_pipe v4l_pipes[PICO4_CAM_SLOTS] = {
    {5, 1, 0, 0},
    {3, 1, 1, 1},
    {3, 3, 2, 2},
    {4, 1, 3, 3},
    {1, 1, 4, 4}, // the RGB camera
};

struct v4l_buffer
{
	const uint8_t *raw;
	size_t length;
	uint16_t *samples;
	uint32_t sequence;
	int64_t sof_monotonic_ns;
};

struct v4l_cam
{
	bool enabled;
	bool rgb;
	int fd;
	unsigned count;
	uint32_t raw_stride;
	struct v4l_buffer buffers[PICO4_CAM_RING_MAX];
	uint64_t errors;
};

struct v4l_done
{
	uint8_t cam, buffer;
};

struct v4l_capture
{
	struct v4l_cam cams[PICO4_CAM_SLOTS];
	struct v4l_done done[PICO4_CAM_SLOTS * PICO4_CAM_RING_MAX];
	int rgb_sensor; //!< The IMX471's subdevice, -1 if none.
	unsigned done_head, done_count;
	int64_t qtimer_offset_ns, qtimer_measured_ns;
};

//! The node of a media entity, from its device numbers.
static int
v4l_node(const struct media_entity_desc *e, char *out, size_t size)
{
	char path[64], line[48];
	snprintf(path, sizeof(path), "/sys/dev/char/%u:%u/uevent", e->dev.major, e->dev.minor);
	FILE *f = fopen(path, "r");
	if (f == NULL) {
		return -errno;
	}
	int r = -ENOENT;
	while (fgets(line, sizeof(line), f) != NULL) {
		if (strncmp(line, "DEVNAME=", 8) == 0) {
			line[strcspn(line, "\n")] = '\0';
			snprintf(out, size, "/dev/%s", line + 8);
			r = 0;
		}
	}
	fclose(f);
	return r;
}

static int
v4l_entity(int media, const char *name, struct media_entity_desc *out)
{
	memset(out, 0, sizeof(*out));
	out->id = MEDIA_ENT_ID_FLAG_NEXT;
	while (ioctl(media, MEDIA_IOC_ENUM_ENTITIES, out) == 0) {
		if (strcmp(out->name, name) == 0) {
			return 0;
		}
		out->id |= MEDIA_ENT_ID_FLAG_NEXT;
	}
	return -ENOENT;
}

//! The node of the entity whose name starts with @p prefix (an I2C sensor's ends in its address).
static int
v4l_entity_node(int media, const char *prefix, char *node, size_t size)
{
	struct media_entity_desc e;
	memset(&e, 0, sizeof(e));
	e.id = MEDIA_ENT_ID_FLAG_NEXT;
	while (ioctl(media, MEDIA_IOC_ENUM_ENTITIES, &e) == 0) {
		if (strncmp(e.name, prefix, strlen(prefix)) == 0) {
			return v4l_node(&e, node, size);
		}
		e.id |= MEDIA_ENT_ID_FLAG_NEXT;
	}
	return -ENOENT;
}

static int
v4l_link(int media, const char *source, unsigned source_pad, const char *sink, unsigned sink_pad)
{
	struct media_entity_desc a, b;
	int r = v4l_entity(media, source, &a);
	if (r == 0) {
		r = v4l_entity(media, sink, &b);
	}
	if (r < 0) {
		return r;
	}
	struct media_link_desc link;
	memset(&link, 0, sizeof(link));
	link.source.entity = a.id;
	link.source.index = (uint16_t)source_pad;
	link.source.flags = MEDIA_PAD_FL_SOURCE;
	link.sink.entity = b.id;
	link.sink.index = (uint16_t)sink_pad;
	link.sink.flags = MEDIA_PAD_FL_SINK;
	link.flags = MEDIA_LNK_FL_ENABLED;
	return ioctl(media, MEDIA_IOC_SETUP_LINK, &link) < 0 ? -errno : 0;
}

static int
v4l_subdev_format(int media, const char *name, unsigned pad, bool rgb, uint32_t code)
{
	struct media_entity_desc e;
	char node[64];
	int r = v4l_entity(media, name, &e);
	if (r == 0) {
		r = v4l_node(&e, node, sizeof(node));
	}
	if (r < 0) {
		return r;
	}
	int fd = open(node, O_RDWR | O_CLOEXEC);
	if (fd < 0) {
		return -errno;
	}
	struct v4l2_subdev_format f;
	memset(&f, 0, sizeof(f));
	f.which = V4L2_SUBDEV_FORMAT_ACTIVE;
	f.pad = pad;
	f.format.width = rgb ? PICO4_CAM_RGB_WIDTH : PICO4_CAM_WIDTH;
	f.format.height = rgb ? PICO4_CAM_RGB_HEIGHT : V4L_LINES;
	f.format.code = code;
	f.format.field = V4L2_FIELD_NONE;
	r = ioctl(fd, VIDIOC_SUBDEV_S_FMT, &f) < 0 ? -errno : 0;
	close(fd);
	return r;
}

static int
v4l_setup_pipe(int media, const struct v4l_pipe *p, bool rgb)
{
	char phy[32], csid[32], rdi[32];
	snprintf(phy, sizeof(phy), "msm_csiphy%d", p->phy);
	snprintf(csid, sizeof(csid), "msm_csid%d", p->csid);
	snprintf(rdi, sizeof(rdi), "msm_vfe%d_rdi0", p->vfe);
	int r = v4l_link(media, phy, (unsigned)p->phy_source_pad, csid, 0);
	if (r == 0) {
		r = v4l_link(media, csid, 1, rdi, 0);
	}
	if (r == 0) {
		r = v4l_subdev_format(media, phy, (unsigned)p->phy_source_pad - 1, rgb, rgb ? V4L_MBUS_SRGGB10 : V4L_MBUS_Y10);
	}
	if (r == 0) {
		r = v4l_subdev_format(media, csid, 0, rgb, rgb ? V4L_MBUS_SRGGB10 : V4L_MBUS_Y10);
	}
	if (r == 0 && rgb) {
		r = v4l_subdev_format(media, csid, 1, rgb, V4L_MBUS_SRGGB16);
	}
	if (r == 0) {
		r = v4l_subdev_format(media, rdi, 0, rgb, rgb ? V4L_MBUS_SRGGB16 : V4L_MBUS_Y10);
	}
	return r;
}

static int
v4l_queue(struct v4l_cam *k, unsigned index)
{
	struct v4l2_plane plane;
	struct v4l2_buffer b;
	memset(&plane, 0, sizeof(plane));
	memset(&b, 0, sizeof(b));
	b.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
	b.memory = V4L2_MEMORY_MMAP;
	b.index = index;
	b.length = 1;
	b.m.planes = &plane;
	if (ioctl(k->fd, VIDIOC_QBUF, &b) < 0) {
		return -errno;
	}
	return 0;
}

static int
v4l_open_cam(struct pico4_cam *c, struct v4l_cam *k, int media, int slot)
{
	const struct v4l_pipe *p = &v4l_pipes[slot];
	struct media_entity_desc e;
	char name[32], node[64];
	const bool rgb = slot == PICO4_CAM_RGB_SLOT;
	const unsigned width = rgb ? PICO4_CAM_RGB_WIDTH : PICO4_CAM_WIDTH;
	const unsigned lines = rgb ? PICO4_CAM_RGB_HEIGHT : V4L_LINES;
	const uint32_t format = rgb ? V4L2_PIX_FMT_SRGGB10 : V4L2_PIX_FMT_Y10P;
	k->rgb = rgb;
	int r = v4l_setup_pipe(media, p, rgb);
	if (r < 0) {
		LOG_E(c, "slot %d: no pipeline: %s", slot, strerror(-r));
		return r;
	}
	snprintf(name, sizeof(name), "msm_vfe%d_video0", p->vfe);
	r = v4l_entity(media, name, &e);
	if (r == 0) {
		r = v4l_node(&e, node, sizeof(node));
	}
	if (r < 0) {
		return r;
	}
	k->fd = open(node, O_RDWR | O_NONBLOCK | O_CLOEXEC);
	if (k->fd < 0) {
		return -errno;
	}
	struct v4l2_format f;
	memset(&f, 0, sizeof(f));
	f.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
	f.fmt.pix_mp.width = width;
	f.fmt.pix_mp.height = lines;
	f.fmt.pix_mp.pixelformat = format;
	f.fmt.pix_mp.field = V4L2_FIELD_NONE;
	f.fmt.pix_mp.num_planes = 1;
	if (ioctl(k->fd, VIDIOC_S_FMT, &f) < 0) {
		return -errno;
	}
	k->raw_stride = f.fmt.pix_mp.plane_fmt[0].bytesperline;
	if (f.fmt.pix_mp.pixelformat != format || k->raw_stride < (rgb ? width * 2 : width * 5 / 4)) {
		LOG_E(c, "slot %d: unexpected format %08x, %u bytes per line", slot, f.fmt.pix_mp.pixelformat, k->raw_stride);
		return -EINVAL;
	}
	struct v4l2_requestbuffers req;
	memset(&req, 0, sizeof(req));
	req.count = c->cfg.ring;
	req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
	req.memory = V4L2_MEMORY_MMAP;
	if (ioctl(k->fd, VIDIOC_REQBUFS, &req) < 0) {
		return -errno;
	}
	k->count = req.count < PICO4_CAM_RING_MAX ? req.count : PICO4_CAM_RING_MAX;
	for (unsigned i = 0; i < k->count; i++) {
		struct v4l_buffer *vb = &k->buffers[i];
		struct v4l2_plane plane;
		struct v4l2_buffer b;
		memset(&plane, 0, sizeof(plane));
		memset(&b, 0, sizeof(b));
		b.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
		b.memory = V4L2_MEMORY_MMAP;
		b.index = i;
		b.length = 1;
		b.m.planes = &plane;
		if (ioctl(k->fd, VIDIOC_QUERYBUF, &b) < 0) {
			return -errno;
		}
		void *map = mmap(NULL, plane.length, PROT_READ, MAP_SHARED, k->fd, plane.m.mem_offset);
		if (map == MAP_FAILED) {
			return -errno;
		}
		vb->raw = map;
		vb->length = plane.length;
		if (!rgb) {
			vb->samples = malloc((size_t)V4L_STRIDE * V4L_LINES);
			if (vb->samples == NULL) {
				return -ENOMEM;
			}
		}
		if ((r = v4l_queue(k, i)) < 0) {
			return r;
		}
	}
	LOG_I(c, "slot %d: %s, CSIPHY %d, CSID %d, %u buffers", slot, node, p->phy, p->csid, k->count);
	return 0;
}

//! The IMX471's exposure (lines) and analog gain (16 = 1x), one frame takes both or neither.
static int
v4l_rgb_exposure(struct v4l_capture *v, unsigned lines, unsigned gain)
{
	unsigned again = 1024u - 1024u * 16u / gain;
	struct v4l2_ext_control ctrl[2];
	struct v4l2_ext_controls ctrls;
	memset(ctrl, 0, sizeof(ctrl));
	memset(&ctrls, 0, sizeof(ctrls));
	ctrl[0].id = V4L2_CID_EXPOSURE;
	ctrl[0].value = (int32_t)lines;
	ctrl[1].id = V4L2_CID_ANALOGUE_GAIN;
	ctrl[1].value = (int32_t)(again > IMX471_ANALOG_GAIN_MAX ? IMX471_ANALOG_GAIN_MAX : again);
	ctrls.which = V4L2_CTRL_WHICH_CUR_VAL;
	ctrls.count = 2;
	ctrls.controls = ctrl;
	return ioctl(v->rgb_sensor, VIDIOC_S_EXT_CTRLS, &ctrls) < 0 ? -errno : 0;
}

static void
v4l_close(struct v4l_capture *v)
{
	if (v->rgb_sensor >= 0) {
		close(v->rgb_sensor);
	}
	for (int i = 0; i < PICO4_CAM_SLOTS; i++) {
		struct v4l_cam *k = &v->cams[i];
		for (unsigned b = 0; b < k->count; b++) {
			if (k->buffers[b].raw != NULL) {
				munmap((void *)k->buffers[b].raw, k->buffers[b].length);
			}
			free(k->buffers[b].samples);
		}
		if (k->fd >= 0) {
			close(k->fd);
		}
	}
	free(v);
}

static int
v4l_open(struct pico4_cam *c)
{
	int media = open("/dev/media0", O_RDWR | O_CLOEXEC);
	if (media < 0) {
		return -errno;
	}
	struct v4l_capture *v = calloc(1, sizeof(*v));
	if (v == NULL) {
		close(media);
		return -ENOMEM;
	}
	v->rgb_sensor = -1;
	for (int i = 0; i < PICO4_CAM_SLOTS; i++) {
		v->cams[i].fd = -1;
	}
	int r = 0;
	for (int i = 0; i < PICO4_CAM_SLOTS && r == 0; i++) {
		v->cams[i].enabled = (c->cfg.slots >> i) & 1;
		if (v->cams[i].enabled) {
			r = v4l_open_cam(c, &v->cams[i], media, i);
		}
	}
	if (r == 0 && v->cams[PICO4_CAM_RGB_SLOT].enabled) {
		char node[64];
		r = v4l_entity_node(media, "pico4-imx471", node, sizeof(node));
		if (r == 0) {
			v->rgb_sensor = open(node, O_RDWR | O_CLOEXEC);
			r = v->rgb_sensor < 0 ? -errno : 0;
		}
		if (r == 0) {
			r = v4l_rgb_exposure(v, c->cfg.rgb_exposure_lines != 0 ? c->cfg.rgb_exposure_lines : 10000000u / PICO4_CAM_RGB_LINE_NS,
			                     c->cfg.rgb_gain != 0 ? c->cfg.rgb_gain : 16);
		}
	}
	close(media);
	if (r < 0) {
		v4l_close(v);
		return r;
	}
	c->v4l = v;
	return 0;
}

static int
v4l_stream(struct pico4_cam *c, bool on)
{
	struct v4l_capture *v = c->v4l;
	int type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
	for (int i = 0; i < PICO4_CAM_SLOTS; i++) {
		struct v4l_cam *k = &v->cams[i];
		if (k->enabled && ioctl(k->fd, on ? VIDIOC_STREAMON : VIDIOC_STREAMOFF, &type) < 0) {
			int r = -errno;
			LOG_E(c, "slot %d: stream %s: %s", i, on ? "on" : "off", strerror(-r));
			if (on) {
				return r;
			}
		}
	}
	return 0;
}

//! One stamp of the QTimer on CLOCK_MONOTONIC, the module's callers take it the same way.
static int64_t
v4l_qtimer_offset(void)
{
	int64_t best = INT64_MAX, offset = 0;
	for (int i = 0; i < 8; i++) {
		uint64_t ticks = 0;
		int64_t before = now_ns(CLOCK_MONOTONIC);
#if defined(__aarch64__)
		__asm__ volatile("isb; mrs %0, cntvct_el0" : "=r"(ticks)::"memory");
#endif
		int64_t after = now_ns(CLOCK_MONOTONIC);
		if (after - before < best) {
			best = after - before;
			offset = before + (after - before) / 2 - (int64_t)(ticks / 192u * 10000u + ticks % 192u * 10000u / 192u);
		}
	}
	return offset;
}

static void
v4l_unpack(const struct v4l_cam *k, struct v4l_buffer *vb)
{
	for (unsigned y = 0; y < V4L_LINES; y++) {
		const uint8_t *in = vb->raw + (size_t)y * k->raw_stride;
		uint16_t *out = vb->samples + (size_t)y * PICO4_CAM_WIDTH;
		for (unsigned x = 0; x < PICO4_CAM_WIDTH; x += 4, in += 5, out += 4) {
			const unsigned low = in[4];
			out[0] = (uint16_t)((in[0] << 2) | (low & 3));
			out[1] = (uint16_t)((in[1] << 2) | ((low >> 2) & 3));
			out[2] = (uint16_t)((in[2] << 2) | ((low >> 4) & 3));
			out[3] = (uint16_t)((in[3] << 2) | ((low >> 6) & 3));
		}
	}
}

static void
v4l_fill_frame(struct pico4_cam *c, int slot, unsigned index, struct pico4_cam_frame *f)
{
	struct v4l_capture *v = c->v4l;
	struct v4l_cam *k = &v->cams[slot];
	struct v4l_buffer *vb = &k->buffers[index];
	if (!k->rgb) {
		v4l_unpack(k, vb);
	}
	int64_t now = now_ns(CLOCK_MONOTONIC);
	if (now - v->qtimer_measured_ns >= 1000000000) {
		v->qtimer_offset_ns = v4l_qtimer_offset();
		v->qtimer_measured_ns = now;
	}
	memset(f, 0, sizeof(*f));
	f->slot = slot;
	f->frame_id = vb->sequence;
	f->sof_monotonic_ns = vb->sof_monotonic_ns;
	f->sof_boottime_ns = vb->sof_monotonic_ns + pico4_slept_ns();
	f->sof_qtimer_ns = vb->sof_monotonic_ns - v->qtimer_offset_ns;
	f->buffer = index;
	if (k->rgb) {
		f->width = PICO4_CAM_RGB_WIDTH;
		f->height = PICO4_CAM_RGB_HEIGHT;
		f->lines = PICO4_CAM_RGB_HEIGHT;
		f->stride = k->raw_stride;
		f->meta = vb->raw;
		f->data = vb->raw;
		f->size = vb->length;
		return;
	}
	f->width = PICO4_CAM_WIDTH;
	f->height = PICO4_CAM_HEIGHT;
	f->lines = V4L_LINES;
	f->stride = V4L_STRIDE;
	f->meta = vb->samples;
	f->data = (const uint8_t *)vb->samples + V4L_STRIDE;
	f->size = (size_t)V4L_STRIDE * V4L_LINES;
}

//! Takes the finished buffers of a camera; false if the camera failed.
static bool
v4l_dequeue(struct pico4_cam *c, int slot)
{
	struct v4l_capture *v = c->v4l;
	struct v4l_cam *k = &v->cams[slot];
	for (;;) {
		struct v4l2_plane plane;
		struct v4l2_buffer b;
		memset(&plane, 0, sizeof(plane));
		memset(&b, 0, sizeof(b));
		b.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
		b.memory = V4L2_MEMORY_MMAP;
		b.length = 1;
		b.m.planes = &plane;
		if (ioctl(k->fd, VIDIOC_DQBUF, &b) < 0) {
			if (errno != EAGAIN) {
				LOG_E(c, "slot %d: VIDIOC_DQBUF: %s", slot, strerror(errno));
			}
			return errno == EAGAIN;
		}
		struct v4l_buffer *vb = &k->buffers[b.index];
		vb->sequence = b.sequence;
		vb->sof_monotonic_ns = (int64_t)b.timestamp.tv_sec * 1000000000 + (int64_t)b.timestamp.tv_usec * 1000;
		if ((b.flags & V4L2_BUF_FLAG_ERROR) != 0) {
			k->errors++;
			v4l_queue(k, b.index);
			continue;
		}
		unsigned tail = (v->done_head + v->done_count) % (unsigned)(sizeof(v->done) / sizeof(v->done[0]));
		v->done[tail].cam = (uint8_t)slot;
		v->done[tail].buffer = (uint8_t)b.index;
		v->done_count++;
	}
}

static int
v4l_next(struct pico4_cam *c, struct pico4_cam_frame *out, int timeout_ms)
{
	struct v4l_capture *v = c->v4l;
	int64_t deadline = timeout_ms >= 0 ? now_ns(CLOCK_MONOTONIC) + (int64_t)timeout_ms * 1000000 : -1;
	if (!c->cfg.hold_frames && c->have_last) {
		c->have_last = false;
		pico4_cam_release(c, &c->last);
	}
	for (;;) {
		if (v->done_count != 0) {
			struct v4l_done e = v->done[v->done_head];
			v->done_head = (v->done_head + 1) % (unsigned)(sizeof(v->done) / sizeof(v->done[0]));
			v->done_count--;
			v4l_fill_frame(c, e.cam, e.buffer, out);
			c->last = *out;
			c->have_last = true;
			return 0;
		}
		struct pollfd pf[PICO4_CAM_SLOTS];
		int slots[PICO4_CAM_SLOTS], n = 0, wait = -1;
		for (int i = 0; i < PICO4_CAM_SLOTS; i++) {
			if (v->cams[i].enabled) {
				pf[n].fd = v->cams[i].fd;
				pf[n].events = POLLIN;
				slots[n++] = i;
			}
		}
		if (deadline >= 0) {
			int64_t left = deadline - now_ns(CLOCK_MONOTONIC);
			if (left <= 0) {
				return -ETIMEDOUT;
			}
			wait = (int)((left + 999999) / 1000000);
		}
		int r = poll(pf, (nfds_t)n, wait);
		if (r < 0) {
			return -errno;
		}
		if (r == 0) {
			return -ETIMEDOUT;
		}
		for (int i = 0; i < n; i++) {
			if ((pf[i].revents & POLLIN) != 0 && !v4l_dequeue(c, slots[i])) {
				return -EIO;
			}
			if ((pf[i].revents & (POLLERR | POLLHUP | POLLNVAL)) != 0 && (pf[i].revents & POLLIN) == 0) {
				// No buffer queued: all held by the caller. It gives them back with release.
				struct timespec ts = {0, 1000000};
				nanosleep(&ts, NULL);
			}
		}
	}
}


/*
 *
 * Public API.
 *
 */

void
pico4_cam_config_init(struct pico4_cam_config *cfg)
{
	memset(cfg, 0, sizeof(*cfg));
	cfg->slots = (1u << PICO4_CAM_NUM) - 1;
	cfg->ring = PICO4_CAM_RING_MAX;
	cfg->verbose = PICO4_CAM_LOG_ERROR;
}

static int
check_config(const struct pico4_cam_config *cfg)
{
	if (cfg->slots == 0 || (cfg->slots >> PICO4_CAM_SLOTS) != 0) {
		cam_log(cfg, PICO4_CAM_LOG_ERROR, "slots must be a non-empty mask of 0-%d", PICO4_CAM_SLOTS - 1);
		return -EINVAL;
	}
	if (cfg->rgb_exposure_lines != 0 &&
	    (cfg->rgb_exposure_lines < 4 || cfg->rgb_exposure_lines > PICO4_CAM_RGB_MAX_LINES)) {
		cam_log(cfg, PICO4_CAM_LOG_ERROR, "RGB exposure must be 4-%d lines", PICO4_CAM_RGB_MAX_LINES);
		return -EINVAL;
	}
	if (cfg->rgb_gain != 0 && (cfg->rgb_gain < 16 || cfg->rgb_gain > 256)) {
		cam_log(cfg, PICO4_CAM_LOG_ERROR, "RGB gain must be 16-256 (16 = 1x)");
		return -EINVAL;
	}
	if (cfg->ring < PICO4_CAM_RING_MIN || cfg->ring > PICO4_CAM_RING_MAX) {
		cam_log(cfg, PICO4_CAM_LOG_ERROR, "ring must be %d-%d", PICO4_CAM_RING_MIN, PICO4_CAM_RING_MAX);
		return -EINVAL;
	}
	return 0;
}

int
pico4_cam_open(struct pico4_cam **out, const struct pico4_cam_config *cfg)
{
	struct pico4_cam *c;
	int r;

	*out = NULL;
	r = check_config(cfg);
	if (r < 0) {
		return r;
	}
	c = calloc(1, sizeof(*c));
	if (c == NULL) {
		return -ENOMEM;
	}
	c->cfg = *cfg;
	r = v4l_open(c);
	if (r < 0) {
		free(c);
		return r;
	}
	*out = c;
	return 0;
}

int
pico4_cam_start(struct pico4_cam *c)
{
	int r;

	if (c->started) {
		return -EINVAL;
	}
	r = v4l_stream(c, true);
	c->started = r == 0;
	return r;
}

int
pico4_cam_next(struct pico4_cam *c, struct pico4_cam_frame *out, int timeout_ms)
{
	if (!c->started) {
		return -EINVAL;
	}
	return v4l_next(c, out, timeout_ms);
}

int
pico4_cam_release(struct pico4_cam *c, const struct pico4_cam_frame *f)
{
	if (f->slot < 0 || f->slot >= PICO4_CAM_SLOTS || !c->v4l->cams[f->slot].enabled ||
	    f->buffer >= c->v4l->cams[f->slot].count) {
		return -EINVAL;
	}
	if (c->have_last && c->last.slot == f->slot && c->last.buffer == f->buffer) {
		c->have_last = false;
	}
	return c->started ? v4l_queue(&c->v4l->cams[f->slot], f->buffer) : 0;
}

int
pico4_cam_stop(struct pico4_cam *c)
{
	if (!c->started) {
		return 0;
	}
	c->started = false;
	c->have_last = false;
	return v4l_stream(c, false);
}

void
pico4_cam_close(struct pico4_cam *c)
{
	if (c == NULL) {
		return;
	}
	pico4_cam_stop(c);
	v4l_close(c->v4l);
	free(c);
}

int
pico4_cam_set_rgb_streaming(struct pico4_cam *c, bool on)
{
	if (c->v4l->rgb_sensor < 0) {
		return -EINVAL;
	}
	struct v4l2_control ctrl = {.id = V4L_CID_RGB_PAUSED, .value = !on};
	return ioctl(c->v4l->rgb_sensor, VIDIOC_S_CTRL, &ctrl) < 0 ? -errno : 0;
}

int
pico4_cam_set_rgb_exposure(struct pico4_cam *c, unsigned lines, unsigned gain)
{
	if (c->v4l->rgb_sensor < 0 || lines < 4 || lines > PICO4_CAM_RGB_MAX_LINES || gain < 16 || gain > 256) {
		return -EINVAL;
	}
	return v4l_rgb_exposure(c->v4l, lines, gain);
}

int
pico4_cam_get_slot_info(const struct pico4_cam *c, int slot, struct pico4_cam_slot_info *out)
{
	if (slot < 0 || slot >= PICO4_CAM_SLOTS) {
		return -EINVAL;
	}
	const struct v4l_cam *v = &c->v4l->cams[slot];
	memset(out, 0, sizeof(*out));
	out->phy = v4l_pipes[slot].phy;
	out->lane_assign = v4l_pipes[slot].phy_source_pad == 3 ? 2 : 0;
	out->errors = v->errors;
	return 0;
}
