// Copyright 2026, overpressure contributors.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Minimal client of the Qualcomm Sensor Core (SSC) SNS client service.
 * @ingroup drv_pico4
 */

#ifndef _GNU_SOURCE // Monado and the tests define it on the command line
#define _GNU_SOURCE // clock_gettime, poll and QRTR sockets under -std=c11 (pico4-cam)
#endif

#include "pico4_ssc.h"

#include <errno.h>
#include <poll.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include <sys/socket.h>
#include <linux/qrtr.h>

#ifndef AF_QIPCRTR
#define AF_QIPCRTR 42
#endif

#define SNS_CLIENT_SERVICE 0x190
#define QRTR_CMD_NEW_SERVER 4
#define QRTR_CMD_DEL_SERVER 5
#define QRTR_CMD_DEL_CLIENT 6
#define QRTR_CMD_NEW_LOOKUP 10

#define QMI_REQUEST 0
#define QMI_INDICATION 4
#define SSC_MSG_CONTROL 0x20
#define SSC_IND_SMALL 0x21
#define SSC_IND_LARGE 0x22

#define MSG_DISABLE 10
#define MSG_SUID_REQ 512
#define MSG_SUID_EVENT 768
#define MSG_CONFIG 513
//! On-change sensors (proximity): an empty request, as PICO OS sends it. 513 crashed the SLPI.
#define MSG_ON_CHANGE_CONFIG 514
#define MSG_SENSOR_EVENT 1025
//! sns_proximity_event: 1 type (0 far, 1 near), 2 raw ADC, 3 status.
#define MSG_PROXIMITY_EVENT 769

static const struct pico4_ssc_suid suid_lookup = {0xABABABABABABABABULL, 0xABABABABABABABABULL};

struct pico4_ssc
{
	int fd;
	uint32_t node, port; //!< The SNS client service.
	uint16_t txn;
};


/*
 *
 * Protobuf (proto2 wire format), writer and reader.
 *
 */

static void
pb_put(struct pico4_pb *pb, const void *data, size_t len)
{
	if (pb->overflow || pb->len + len > pb->cap) {
		pb->overflow = true;
		return;
	}
	memcpy(pb->data + pb->len, data, len);
	pb->len += len;
}

static void
pb_raw_varint(struct pico4_pb *pb, uint64_t value)
{
	uint8_t tmp[10];
	size_t n = 0;
	do {
		tmp[n] = value & 0x7f;
		value >>= 7;
		if (value) {
			tmp[n] |= 0x80;
		}
		n++;
	} while (value);
	pb_put(pb, tmp, n);
}

static void
pb_key(struct pico4_pb *pb, uint32_t field, uint32_t wire)
{
	pb_raw_varint(pb, ((uint64_t)field << 3) | wire);
}

void
pico4_pb_varint(struct pico4_pb *pb, uint32_t field, uint64_t value)
{
	pb_key(pb, field, 0);
	pb_raw_varint(pb, value);
}

void
pico4_pb_fixed32(struct pico4_pb *pb, uint32_t field, uint32_t value)
{
	uint8_t b[4] = {value & 0xff, (value >> 8) & 0xff, (value >> 16) & 0xff, value >> 24};
	pb_key(pb, field, 5);
	pb_put(pb, b, 4);
}

void
pico4_pb_fixed64(struct pico4_pb *pb, uint32_t field, uint64_t value)
{
	uint8_t b[8];
	for (int i = 0; i < 8; i++) {
		b[i] = (value >> (8 * i)) & 0xff;
	}
	pb_key(pb, field, 1);
	pb_put(pb, b, 8);
}

void
pico4_pb_float(struct pico4_pb *pb, uint32_t field, float value)
{
	uint32_t bits;
	memcpy(&bits, &value, sizeof(bits));
	pico4_pb_fixed32(pb, field, bits);
}

void
pico4_pb_bytes(struct pico4_pb *pb, uint32_t field, const void *data, size_t len)
{
	pb_key(pb, field, 2);
	pb_raw_varint(pb, len);
	pb_put(pb, data, len);
}

//! One field of a protobuf message; for wire type 2 @p data/@p len is the payload.
struct pb_field
{
	uint32_t field;
	uint32_t wire;
	uint64_t value;
	const uint8_t *data;
	size_t len;
};

static bool
pb_read_varint(const uint8_t **p, const uint8_t *end, uint64_t *out)
{
	uint64_t result = 0;
	for (int shift = 0; shift < 64; shift += 7) {
		if (*p >= end) {
			return false;
		}
		uint8_t byte = *(*p)++;
		result |= (uint64_t)(byte & 0x7f) << shift;
		if (!(byte & 0x80)) {
			*out = result;
			return true;
		}
	}
	return false;
}

static uint64_t
le(const uint8_t *p, int bytes)
{
	uint64_t v = 0;
	for (int i = bytes - 1; i >= 0; i--) {
		v = (v << 8) | p[i];
	}
	return v;
}

//! Next field of the message at *p; returns false at the end, -1 in *err when malformed.
static bool
pb_next(const uint8_t **p, const uint8_t *end, struct pb_field *f, int *err)
{
	uint64_t key;
	if (*p >= end) {
		return false;
	}
	if (!pb_read_varint(p, end, &key)) {
		*err = -EINVAL;
		return false;
	}
	f->field = (uint32_t)(key >> 3);
	f->wire = key & 7;
	f->data = NULL;
	f->len = 0;
	switch (f->wire) {
	case 0:
		if (!pb_read_varint(p, end, &f->value)) {
			*err = -EINVAL;
			return false;
		}
		return true;
	case 1:
	case 5: {
		size_t n = f->wire == 1 ? 8 : 4;
		if ((size_t)(end - *p) < n) {
			*err = -EINVAL;
			return false;
		}
		f->value = le(*p, (int)n);
		f->data = *p;
		f->len = n;
		*p += n;
		return true;
	}
	case 2: {
		uint64_t n;
		if (!pb_read_varint(p, end, &n) || (uint64_t)(end - *p) < n) {
			*err = -EINVAL;
			return false;
		}
		f->data = *p;
		f->len = (size_t)n;
		*p += n;
		return true;
	}
	default: *err = -EINVAL; return false;
	}
}

static bool
decode_suid(const uint8_t *data, size_t len, struct pico4_ssc_suid *out)
{
	const uint8_t *p = data, *end = data + len;
	struct pb_field f;
	int err = 0;
	bool low = false, high = false;
	while (pb_next(&p, end, &f, &err)) {
		if (f.field == 1 && f.wire == 1) {
			out->low = f.value;
			low = true;
		} else if (f.field == 2 && f.wire == 1) {
			out->high = f.value;
			high = true;
		}
	}
	return err == 0 && low && high;
}

static void
encode_suid(struct pico4_pb *pb, uint32_t field, const struct pico4_ssc_suid *suid)
{
	uint8_t buf[32];
	struct pico4_pb inner = {buf, 0, sizeof(buf), false};
	pico4_pb_fixed64(&inner, 1, suid->low);
	pico4_pb_fixed64(&inner, 2, suid->high);
	pico4_pb_bytes(pb, field, inner.data, inner.len);
}


/*
 *
 * SSC and QMI messages.
 *
 */

size_t
pico4_ssc_encode_request(uint8_t *buf,
                         size_t cap,
                         const struct pico4_ssc_suid *suid,
                         uint32_t msg_id,
                         const uint8_t *payload,
                         size_t payload_len)
{
	struct pico4_pb pb = {buf, 0, cap, false};
	uint8_t susp_buf[8], body_buf[256];
	struct pico4_pb susp = {susp_buf, 0, sizeof(susp_buf), false};
	struct pico4_pb body = {body_buf, 0, sizeof(body_buf), false};

	// Applications processor, wakeup delivery.
	pico4_pb_varint(&susp, 1, 1);
	pico4_pb_varint(&susp, 2, 0);
	if (payload_len > 0) {
		pico4_pb_bytes(&body, 2, payload, payload_len);
	}

	encode_suid(&pb, 1, suid);
	pico4_pb_fixed32(&pb, 2, msg_id);
	pico4_pb_bytes(&pb, 3, susp.data, susp.len);
	pico4_pb_bytes(&pb, 4, body.data, body.len);
	return pb.overflow || susp.overflow || body.overflow ? 0 : pb.len;
}

size_t
pico4_ssc_encode_qmi(uint8_t *buf, size_t cap, uint16_t txn, const uint8_t *request, size_t request_len)
{
	// Header, TLV 0x01 (u16 length + request), TLV 0x10 (report type: small).
	size_t tlv_len = 3 + 2 + request_len + 3 + 1;
	size_t total = 7 + tlv_len;
	if (total > cap || request_len > 0xffff - 2) {
		return 0;
	}
	uint8_t *p = buf;
	*p++ = QMI_REQUEST;
	*p++ = txn & 0xff;
	*p++ = txn >> 8;
	*p++ = SSC_MSG_CONTROL & 0xff;
	*p++ = SSC_MSG_CONTROL >> 8;
	*p++ = tlv_len & 0xff;
	*p++ = tlv_len >> 8;
	*p++ = 0x01;
	*p++ = (request_len + 2) & 0xff;
	*p++ = (request_len + 2) >> 8;
	*p++ = request_len & 0xff;
	*p++ = request_len >> 8;
	memcpy(p, request, request_len);
	p += request_len;
	*p++ = 0x10;
	*p++ = 1;
	*p++ = 0;
	*p++ = 0;
	return total;
}

//! Find TLV @p type in a QMI message; returns its value or NULL.
static const uint8_t *
qmi_tlv(const uint8_t *packet, size_t len, uint8_t type, size_t *out_len)
{
	if (len < 7) {
		return NULL;
	}
	size_t msg_len = packet[5] | (packet[6] << 8);
	size_t end = 7 + msg_len < len ? 7 + msg_len : len;
	size_t pos = 7;
	while (pos + 3 <= end) {
		uint8_t t = packet[pos];
		size_t l = packet[pos + 1] | (packet[pos + 2] << 8);
		pos += 3;
		if (pos + l > end) {
			return NULL;
		}
		if (t == type) {
			*out_len = l;
			return packet + pos;
		}
		pos += l;
	}
	return NULL;
}

static int
decode_floats(const uint8_t *data, size_t len, float *out, int max)
{
	const uint8_t *p = data, *end = data + len;
	struct pb_field f;
	int err = 0, n = 0;
	while (pb_next(&p, end, &f, &err)) {
		if (f.field != 1) {
			continue;
		}
		if (f.wire == 5 && n < max) {
			uint32_t bits = (uint32_t)f.value;
			memcpy(&out[n++], &bits, sizeof(float));
		} else if (f.wire == 2) {
			// Packed repeated floats.
			for (size_t i = 0; i + 4 <= f.len && n < max; i += 4) {
				uint32_t bits = (uint32_t)le(f.data + i, 4);
				memcpy(&out[n++], &bits, sizeof(float));
			}
		}
	}
	return err ? err : n;
}

//! A proximity event's fields 1-3 (varints or floats) as values[0..2]; how many were found.
static int
decode_proximity(const uint8_t *data, size_t len, float *out)
{
	const uint8_t *p = data, *end = data + len;
	struct pb_field f;
	int err = 0, n = 0;
	while (pb_next(&p, end, &f, &err)) {
		if (f.field < 1 || f.field > 3) {
			continue;
		}
		if (f.wire == 0) {
			out[f.field - 1] = (float)f.value;
		} else if (f.wire == 5) {
			uint32_t bits = (uint32_t)f.value;
			memcpy(&out[f.field - 1], &bits, sizeof(float));
		} else {
			continue;
		}
		n++;
	}
	return err ? err : n;
}

//! The first length-delimited field 1 of an event payload, as opaque bytes.
static uint16_t
copy_raw(const uint8_t *data, size_t len, uint8_t *out)
{
	const uint8_t *p = data, *end = data + len;
	struct pb_field f;
	int err = 0;
	while (pb_next(&p, end, &f, &err)) {
		if (f.field == 1 && f.wire == 2) {
			size_t n = f.len < PICO4_SSC_RAW_MAX ? f.len : PICO4_SSC_RAW_MAX;
			memcpy(out, f.data, n);
			return (uint16_t)n;
		}
	}
	return 0;
}

static bool
suid_lookup_match(const uint8_t *data, size_t len, const char *data_type, struct pico4_ssc_suid *out)
{
	const uint8_t *p = data, *end = data + len;
	struct pb_field f;
	int err = 0;
	bool type_ok = false, found = false;
	struct pico4_ssc_suid first = {0, 0};
	while (pb_next(&p, end, &f, &err)) {
		if (f.field == 1 && f.wire == 2) {
			type_ok = f.len == strlen(data_type) && memcmp(f.data, data_type, f.len) == 0;
		} else if (f.field == 2 && f.wire == 2 && !found) {
			found = decode_suid(f.data, f.len, &first);
		}
	}
	if (err || !type_ok || !found) {
		return false;
	}
	*out = first;
	return true;
}

int
pico4_ssc_decode_indication(const uint8_t *packet,
                            size_t len,
                            struct pico4_ssc_sample *out,
                            int max,
                            const char *suid_data_type,
                            struct pico4_ssc_suid *out_suid_event,
                            bool *out_suid_found)
{
	if (len < 7) {
		return -EINVAL;
	}
	uint16_t msg_id = packet[3] | (packet[4] << 8);
	if (packet[0] != QMI_INDICATION || (msg_id != SSC_IND_SMALL && msg_id != SSC_IND_LARGE)) {
		return 0;
	}
	size_t tlv_len = 0;
	const uint8_t *tlv = qmi_tlv(packet, len, 0x02, &tlv_len);
	if (tlv == NULL || tlv_len < 2) {
		return -EINVAL;
	}
	size_t data_len = tlv[0] | (tlv[1] << 8);
	if (data_len + 2 > tlv_len) {
		return -EINVAL;
	}

	const uint8_t *p = tlv + 2, *end = tlv + 2 + data_len;
	struct pb_field f;
	int err = 0, n = 0;
	struct pico4_ssc_suid suid = {0, 0};
	while (pb_next(&p, end, &f, &err)) {
		if (f.field == 1 && f.wire == 2) {
			if (!decode_suid(f.data, f.len, &suid)) {
				return -EINVAL;
			}
			continue;
		}
		if (f.field != 2 || f.wire != 2) {
			continue;
		}
		// sns_client_event: msg_id (fixed32), timestamp (fixed64), payload.
		const uint8_t *q = f.data, *qend = f.data + f.len;
		struct pb_field e;
		uint32_t ev_id = 0;
		uint64_t ts = 0;
		const uint8_t *payload = NULL;
		size_t payload_len = 0;
		while (pb_next(&q, qend, &e, &err)) {
			if (e.field == 1 && e.wire == 5) {
				ev_id = (uint32_t)e.value;
			} else if (e.field == 2 && e.wire == 1) {
				ts = e.value;
			} else if (e.field == 3 && e.wire == 2) {
				payload = e.data;
				payload_len = e.len;
			}
		}
		if (err) {
			return err;
		}
		if (ev_id == MSG_SENSOR_EVENT && payload != NULL && n < max) {
			int count = decode_floats(payload, payload_len, out[n].values, 3);
			if (count < 0) {
				return count;
			}
			out[n].suid = suid;
			out[n].ticks = ts;
			out[n].count = count;
			out[n].raw_len = copy_raw(payload, payload_len, out[n].raw);
			out[n].proximity = false;
			n++;
		} else if (ev_id == MSG_PROXIMITY_EVENT && payload != NULL && n < max) {
			memset(&out[n], 0, sizeof(out[n]));
			int count = decode_proximity(payload, payload_len, out[n].values);
			if (count < 0) {
				return count;
			}
			out[n].suid = suid;
			out[n].ticks = ts;
			out[n].count = count;
			out[n].proximity = true;
			n++;
		} else if (ev_id == MSG_SUID_EVENT && payload != NULL && out_suid_event != NULL &&
		           suid.low == suid_lookup.low && suid.high == suid_lookup.high && suid_data_type != NULL) {
			if (suid_lookup_match(payload, payload_len, suid_data_type, out_suid_event)) {
				*out_suid_found = true;
			}
		}
	}
	return err ? err : n;
}


/*
 *
 * Socket handling.
 *
 */

//! QRTR datagrams (the default transport).
static int
qrtr_open(uint32_t *out_node)
{
	int fd = socket(AF_QIPCRTR, SOCK_DGRAM, 0);
	if (fd < 0) {
		return -errno;
	}
	struct sockaddr_qrtr local;
	socklen_t local_len = sizeof(local);
	if (getsockname(fd, (struct sockaddr *)&local, &local_len) < 0) {
		int err = -errno;
		close(fd);
		return err;
	}
	*out_node = local.sq_node;
	return fd;
}

static long
qrtr_send(int fd, const void *buf, size_t len, uint32_t node, uint32_t port)
{
	struct sockaddr_qrtr to = {AF_QIPCRTR, node, port};
	return sendto(fd, buf, len, 0, (struct sockaddr *)&to, sizeof(to));
}

static long
qrtr_recv(int fd, void *buf, size_t cap, uint32_t *out_node, uint32_t *out_port)
{
	struct sockaddr_qrtr from = {0};
	socklen_t from_len = sizeof(from);
	ssize_t n = recvfrom(fd, buf, cap, 0, (struct sockaddr *)&from, &from_len);
	*out_node = from.sq_node;
	*out_port = from.sq_port;
	return n;
}

static const struct pico4_ssc_transport qrtr = {qrtr_open, qrtr_send, qrtr_recv};
static const struct pico4_ssc_transport *transport = &qrtr;

void
pico4_ssc_set_transport(const struct pico4_ssc_transport *t)
{
	transport = t != NULL ? t : &qrtr;
}

static int64_t
now_ms(void)
{
	struct timespec ts;
	clock_gettime(CLOCK_MONOTONIC, &ts);
	return (int64_t)ts.tv_sec * 1000 + ts.tv_nsec / 1000000;
}

bool
pico4_ssc_control_gone(const uint8_t *packet, size_t len, uint32_t node, uint32_t port)
{
	uint32_t pkt[5];
	if (len < sizeof(pkt)) {
		return false;
	}
	memcpy(pkt, packet, sizeof(pkt)); // little-endian on the wire and on the AP
	if (pkt[0] == QRTR_CMD_DEL_SERVER) {
		return pkt[1] == SNS_CLIENT_SERVICE && pkt[3] == node && pkt[4] == port;
	}
	if (pkt[0] == QRTR_CMD_DEL_CLIENT) {
		return pkt[1] == node && pkt[2] == port;
	}
	return false;
}

/*!
 * Receive one packet from the SNS client service. Other packets are skipped, except the
 * name service's notice that the service went away (the sensor DSP restarted): -ECONNRESET.
 */
static int
recv_packet(struct pico4_ssc *ssc, uint8_t *buf, size_t cap, int timeout_ms)
{
	int64_t deadline = now_ms() + timeout_ms;
	for (;;) {
		int left = (int)(deadline - now_ms());
		struct pollfd pfd = {ssc->fd, POLLIN, 0};
		int ret = poll(&pfd, 1, left > 0 ? left : 0);
		if (ret < 0) {
			return errno == EINTR ? 0 : -errno;
		}
		if (ret == 0) {
			return 0;
		}
		uint32_t node, port;
		ssize_t n = transport->recv(ssc->fd, buf, cap, &node, &port);
		if (n < 0) {
			return -errno;
		}
		if (node == ssc->node && port == ssc->port) {
			return (int)n;
		}
		if (port == QRTR_PORT_CTRL && pico4_ssc_control_gone(buf, (size_t)n, ssc->node, ssc->port)) {
			return -ECONNRESET;
		}
		// Other name-service updates and anything else: not ours.
	}
}

int
pico4_ssc_open(struct pico4_ssc **out_ssc, int timeout_ms)
{
	struct pico4_ssc *ssc = calloc(1, sizeof(*ssc));
	if (ssc == NULL) {
		return -ENOMEM;
	}
	uint32_t local_node = 0;
	ssc->fd = transport->open(&local_node);
	if (ssc->fd < 0) {
		int err = ssc->fd;
		free(ssc);
		return err;
	}

	uint32_t lookup[5] = {QRTR_CMD_NEW_LOOKUP, SNS_CLIENT_SERVICE, 0, 0, 0};
	if (transport->send(ssc->fd, lookup, sizeof(lookup), local_node, QRTR_PORT_CTRL) < 0) {
		int err = -errno;
		pico4_ssc_close(&ssc);
		return err;
	}

	int64_t deadline = now_ms() + timeout_ms;
	while (now_ms() < deadline) {
		struct pollfd pfd = {ssc->fd, POLLIN, 0};
		if (poll(&pfd, 1, (int)(deadline - now_ms())) <= 0) {
			break;
		}
		uint32_t pkt[8], node, port;
		ssize_t n = transport->recv(ssc->fd, pkt, sizeof(pkt), &node, &port);
		if (n < 20) {
			continue;
		}
		if (pkt[0] == QRTR_CMD_NEW_SERVER && pkt[1] == SNS_CLIENT_SERVICE && (pkt[3] || pkt[4])) {
			ssc->node = pkt[3];
			ssc->port = pkt[4];
			*out_ssc = ssc;
			return 0;
		}
	}
	pico4_ssc_close(&ssc);
	return -ETIMEDOUT;
}

void
pico4_ssc_close(struct pico4_ssc **ssc_ptr)
{
	struct pico4_ssc *ssc = *ssc_ptr;
	if (ssc == NULL) {
		return;
	}
	if (ssc->fd >= 0) {
		close(ssc->fd);
	}
	free(ssc);
	*ssc_ptr = NULL;
}

static int
send_request(struct pico4_ssc *ssc, const struct pico4_ssc_suid *suid, uint32_t msg_id, const uint8_t *payload, size_t len)
{
	uint8_t request[512], frame[600];
	size_t request_len = pico4_ssc_encode_request(request, sizeof(request), suid, msg_id, payload, len);
	ssc->txn = (uint16_t)(ssc->txn + 1) ? (uint16_t)(ssc->txn + 1) : 1;
	size_t frame_len = pico4_ssc_encode_qmi(frame, sizeof(frame), ssc->txn, request, request_len);
	if (request_len == 0 || frame_len == 0) {
		return -EOVERFLOW;
	}
	if (transport->send(ssc->fd, frame, frame_len, ssc->node, ssc->port) < 0) {
		return -errno;
	}
	return 0;
}

int
pico4_ssc_find(struct pico4_ssc *ssc, const char *data_type, struct pico4_ssc_suid *out_suid, int timeout_ms)
{
	uint8_t payload[64];
	struct pico4_pb pb = {payload, 0, sizeof(payload), false};
	pico4_pb_bytes(&pb, 1, data_type, strlen(data_type));
	pico4_pb_varint(&pb, 2, 0);
	pico4_pb_varint(&pb, 3, 0);
	if (pb.overflow) {
		return -EINVAL;
	}
	int ret = send_request(ssc, &suid_lookup, MSG_SUID_REQ, pb.data, pb.len);
	if (ret < 0) {
		return ret;
	}

	int64_t deadline = now_ms() + timeout_ms;
	uint8_t buf[4096];
	while (now_ms() < deadline) {
		int n = recv_packet(ssc, buf, sizeof(buf), (int)(deadline - now_ms()));
		if (n < 0) {
			return n;
		}
		if (n == 0) {
			break;
		}
		struct pico4_ssc_sample unused[1];
		bool found = false;
		pico4_ssc_decode_indication(buf, (size_t)n, unused, 0, data_type, out_suid, &found);
		if (found) {
			return 0;
		}
	}
	return -ENOENT;
}

int
pico4_ssc_enable(struct pico4_ssc *ssc, const struct pico4_ssc_suid *suid, float rate_hz)
{
	uint8_t payload[8];
	struct pico4_pb pb = {payload, 0, sizeof(payload), false};
	pico4_pb_float(&pb, 1, rate_hz);
	return send_request(ssc, suid, MSG_CONFIG, pb.data, pb.len);
}

int
pico4_ssc_enable_on_change(struct pico4_ssc *ssc, const struct pico4_ssc_suid *suid)
{
	return send_request(ssc, suid, MSG_ON_CHANGE_CONFIG, NULL, 0);
}

int
pico4_ssc_disable(struct pico4_ssc *ssc, const struct pico4_ssc_suid *suid)
{
	return send_request(ssc, suid, MSG_DISABLE, NULL, 0);
}

int
pico4_ssc_subscribe(struct pico4_ssc **out_ssc,
                    const char *const *data_types,
                    struct pico4_ssc_suid *out_suids,
                    int count,
                    float rate_hz,
                    int timeout_ms)
{
	int ret = pico4_ssc_open(out_ssc, timeout_ms);
	for (int i = 0; ret == 0 && i < count; i++) {
		ret = pico4_ssc_find(*out_ssc, data_types[i], &out_suids[i], timeout_ms);
	}
	for (int i = 0; ret == 0 && i < count; i++) {
		ret = pico4_ssc_enable(*out_ssc, &out_suids[i], rate_hz);
	}
	if (ret < 0) {
		pico4_ssc_close(out_ssc);
	}
	return ret;
}

int
pico4_ssc_read(struct pico4_ssc *ssc, int timeout_ms, struct pico4_ssc_sample *out, int max)
{
	uint8_t buf[65536];
	int n = recv_packet(ssc, buf, sizeof(buf), timeout_ms);
	if (n <= 0) {
		return n;
	}
	int ret = pico4_ssc_decode_indication(buf, (size_t)n, out, max, NULL, NULL, NULL);
	return ret < 0 ? 0 : ret; // A malformed indication is dropped, not fatal.
}
