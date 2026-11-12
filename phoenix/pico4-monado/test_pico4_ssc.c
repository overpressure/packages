// Copyright 2026, overpressure contributors.
// SPDX-License-Identifier: BSL-1.0
/*
 * Unit test of the SSC client encoding. The golden messages were produced by
 * the hardware-verified Python client (userspace/rootfs/usr/bin/pico4-imu).
 */

#include "pico4_ssc.h"

#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>

// clang-format off
static const uint8_t golden_enable_request[] = {0x0a, 0x12, 0x09, 0x44, 0x44, 0x33, 0x33, 0x22, 0x22, 0x11, 0x11, 0x11, 0xd0, 0xd0, 0xc0, 0xc0, 0xb0, 0xb0, 0xa0, 0xa0, 0x15, 0x01, 0x02, 0x00, 0x00, 0x1a, 0x04, 0x08, 0x01, 0x10, 0x00, 0x22, 0x07, 0x12, 0x05, 0x0d, 0x00, 0x00, 0x7a, 0x44};
static const uint8_t golden_enable_qmi[] = {0x00, 0x07, 0x00, 0x20, 0x00, 0x31, 0x00, 0x01, 0x2a, 0x00, 0x28, 0x00, 0x0a, 0x12, 0x09, 0x44, 0x44, 0x33, 0x33, 0x22, 0x22, 0x11, 0x11, 0x11, 0xd0, 0xd0, 0xc0, 0xc0, 0xb0, 0xb0, 0xa0, 0xa0, 0x15, 0x01, 0x02, 0x00, 0x00, 0x1a, 0x04, 0x08, 0x01, 0x10, 0x00, 0x22, 0x07, 0x12, 0x05, 0x0d, 0x00, 0x00, 0x7a, 0x44, 0x10, 0x01, 0x00, 0x00};
static const uint8_t golden_disable_request[] = {0x0a, 0x12, 0x09, 0x44, 0x44, 0x33, 0x33, 0x22, 0x22, 0x11, 0x11, 0x11, 0xd0, 0xd0, 0xc0, 0xc0, 0xb0, 0xb0, 0xa0, 0xa0, 0x15, 0x0a, 0x00, 0x00, 0x00, 0x1a, 0x04, 0x08, 0x01, 0x10, 0x00, 0x22, 0x00};
static const uint8_t golden_suid_request[] = {0x0a, 0x12, 0x09, 0xab, 0xab, 0xab, 0xab, 0xab, 0xab, 0xab, 0xab, 0x11, 0xab, 0xab, 0xab, 0xab, 0xab, 0xab, 0xab, 0xab, 0x15, 0x00, 0x02, 0x00, 0x00, 0x1a, 0x04, 0x08, 0x01, 0x10, 0x00, 0x22, 0x0c, 0x12, 0x0a, 0x0a, 0x04, 0x67, 0x79, 0x72, 0x6f, 0x10, 0x00, 0x18, 0x00};
static const uint8_t golden_samples[] = {0x04, 0x00, 0x00, 0x21, 0x00, 0x80, 0x00, 0x01, 0x08, 0x00, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x72, 0x00, 0x70, 0x00, 0x0a, 0x12, 0x09, 0x44, 0x44, 0x33, 0x33, 0x22, 0x22, 0x11, 0x11, 0x11, 0xd0, 0xd0, 0xc0, 0xc0, 0xb0, 0xb0, 0xa0, 0xa0, 0x12, 0x21, 0x0d, 0x01, 0x04, 0x00, 0x00, 0x11, 0x15, 0xcd, 0x5b, 0x07, 0x00, 0x00, 0x00, 0x00, 0x1a, 0x11, 0x0d, 0x00, 0x00, 0xc0, 0x3f, 0x0d, 0x00, 0x00, 0x10, 0xc0, 0x0d, 0x00, 0x00, 0x1c, 0x41, 0x10, 0x03, 0x12, 0x15, 0x0d, 0x00, 0x03, 0x00, 0x00, 0x11, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1a, 0x05, 0x0d, 0x00, 0x00, 0x7a, 0x44, 0x12, 0x20, 0x0d, 0x01, 0x04, 0x00, 0x00, 0x11, 0x35, 0x1b, 0x5c, 0x07, 0x00, 0x00, 0x00, 0x00, 0x1a, 0x10, 0x0a, 0x0c, 0x00, 0x00, 0x00, 0x3f, 0x00, 0x00, 0x80, 0x3e, 0x00, 0x00, 0x00, 0xbe, 0x10, 0x03};
// The ICM-42688's sensor_temperature event as the headset sends it: one packed float, 43.36 C.
static const uint8_t golden_temperature[] = {0x04, 0x00, 0x00, 0x21, 0x00, 0x3e, 0x00, 0x01, 0x08, 0x00, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x30, 0x00, 0x2e, 0x00, 0x0a, 0x12, 0x09, 0x08, 0x07, 0x06, 0x05, 0x04, 0x03, 0x02, 0x01, 0x11, 0x18, 0x17, 0x16, 0x15, 0x14, 0x13, 0x12, 0x11, 0x12, 0x18, 0x0d, 0x01, 0x04, 0x00, 0x00, 0x11, 0xe8, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1a, 0x08, 0x0a, 0x04, 0x12, 0x6e, 0x2d, 0x42, 0x10, 0x03};
// A proximity event (msg 769) as PICO OS's HAL decodes it: near (1), raw ADC 210, status 3.
static const uint8_t golden_proximity[] = {0x04, 0x00, 0x00, 0x21, 0x00, 0x3d, 0x00, 0x01, 0x08, 0x00, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x2f, 0x00, 0x2d, 0x00, 0x0a, 0x12, 0x09, 0x08, 0x07, 0x06, 0x05, 0x04, 0x03, 0x02, 0x01, 0x11, 0x18, 0x17, 0x16, 0x15, 0x14, 0x13, 0x12, 0x11, 0x12, 0x17, 0x0d, 0x01, 0x03, 0x00, 0x00, 0x11, 0xd0, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1a, 0x07, 0x08, 0x01, 0x10, 0xd2, 0x01, 0x18, 0x03};
static const uint8_t golden_suid_event[] = {0x04, 0x00, 0x00, 0x22, 0x00, 0x64, 0x00, 0x01, 0x08, 0x00, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x56, 0x00, 0x54, 0x00, 0x0a, 0x12, 0x09, 0xab, 0xab, 0xab, 0xab, 0xab, 0xab, 0xab, 0xab, 0x11, 0xab, 0xab, 0xab, 0xab, 0xab, 0xab, 0xab, 0xab, 0x12, 0x3e, 0x0d, 0x00, 0x03, 0x00, 0x00, 0x11, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1a, 0x2e, 0x0a, 0x04, 0x67, 0x79, 0x72, 0x6f, 0x12, 0x12, 0x09, 0x88, 0x88, 0x77, 0x77, 0x66, 0x66, 0x55, 0x55, 0x11, 0xcc, 0xcc, 0xbb, 0xbb, 0xaa, 0xaa, 0x99, 0x99, 0x12, 0x12, 0x09, 0x44, 0x44, 0x33, 0x33, 0x22, 0x22, 0x11, 0x11, 0x11, 0xd0, 0xd0, 0xc0, 0xc0, 0xb0, 0xb0, 0xa0, 0xa0};
// clang-format on

static int failures;

#define EXPECT(cond)                                                                                                   \
	do {                                                                                                           \
		if (!(cond)) {                                                                                         \
			fprintf(stderr, "%s:%d: expected %s\n", __FILE__, __LINE__, #cond);                            \
			failures++;                                                                                    \
		}                                                                                                      \
	} while (0)

static bool
same(const uint8_t *a, size_t a_len, const uint8_t *b, size_t b_len)
{
	return a_len == b_len && memcmp(a, b, a_len) == 0;
}

/*
 * A fake SNS client service behind the client's transport: a datagram socket pair per
 * client, each datagram prefixed with the sender's node and port. It answers the name
 * service lookup, SUID lookups and enables, sends samples of the enabled sensors, and can
 * go away (the name service's DEL_SERVER) and come back on a new port with new UIDs, as
 * after a sensor DSP restart.
 */
#define PORT_CTRL 0xfffffffeu
#define LOCAL_NODE 7u

static struct
{
	bool up;
	uint32_t node, port;
	struct pico4_ssc_suid accel, gyro;
	bool accel_on, gyro_on;
	int client, peer; //!< The newest client's socket pair.
} fake;

static void
deliver(uint32_t node, uint32_t port, const void *data, size_t len)
{
	uint8_t buf[1024];
	memcpy(buf, &node, 4);
	memcpy(buf + 4, &port, 4);
	memcpy(buf + 8, data, len);
	(void)!send(fake.peer, buf, len + 8, 0);
}

static int
fake_open(uint32_t *out_node)
{
	int sv[2];
	if (socketpair(AF_UNIX, SOCK_DGRAM, 0, sv) != 0) {
		return -errno;
	}
	fake.client = sv[0];
	fake.peer = sv[1];
	*out_node = LOCAL_NODE;
	return sv[0];
}

static long
fake_recv(int fd, void *buf, size_t cap, uint32_t *out_node, uint32_t *out_port)
{
	uint8_t tmp[1024];
	ssize_t n = recv(fd, tmp, sizeof(tmp), 0);
	if (n < 8) {
		return n < 0 ? n : 0;
	}
	memcpy(out_node, tmp, 4);
	memcpy(out_port, tmp + 4, 4);
	size_t len = (size_t)n - 8 < cap ? (size_t)n - 8 : cap;
	memcpy(buf, tmp + 8, len);
	return (long)len;
}

//! A QMI indication from the service: @p suid's event @p ev_id with @p payload.
static void
indicate(const struct pico4_ssc_suid *suid, uint32_t ev_id, const uint8_t *payload, size_t payload_len)
{
	uint8_t s_buf[32], e_buf[256], m_buf[400], q[512];
	struct pico4_pb s = {s_buf, 0, sizeof(s_buf), false}, e = {e_buf, 0, sizeof(e_buf), false};
	struct pico4_pb m = {m_buf, 0, sizeof(m_buf), false};
	pico4_pb_fixed64(&s, 1, suid->low);
	pico4_pb_fixed64(&s, 2, suid->high);
	pico4_pb_fixed32(&e, 1, ev_id);
	pico4_pb_fixed64(&e, 2, 123456789);
	pico4_pb_bytes(&e, 3, payload, payload_len);
	pico4_pb_bytes(&m, 1, s.data, s.len);
	pico4_pb_bytes(&m, 2, e.data, e.len);
	size_t tlv = 2 + m.len, body = 3 + tlv;
	uint8_t head[] = {4, 0, 0, 0x21, 0, body & 0xff, body >> 8, 0x02, tlv & 0xff, tlv >> 8, m.len & 0xff, m.len >> 8};
	memcpy(q, head, sizeof(head));
	memcpy(q + sizeof(head), m.data, m.len);
	deliver(fake.node, fake.port, q, sizeof(head) + m.len);
}

static long
fake_send(int fd, const void *buf, size_t len, uint32_t node, uint32_t port)
{
	const uint8_t *b = buf;
	uint32_t cmd = 0;
	if (port == PORT_CTRL && len >= 4) {
		memcpy(&cmd, b, 4);
		if (cmd == 10 && fake.up) { // NEW_LOOKUP: the server, if it runs
			uint32_t server[5] = {4, 0x190, 1, fake.node, fake.port};
			deliver(LOCAL_NODE, PORT_CTRL, server, sizeof(server));
		}
		return (long)len;
	}
	if (!fake.up || node != fake.node || port != fake.port || len < 12 + 25) {
		return (long)len; // a request to a server that is gone is lost
	}
	// QMI header (7), TLV 0x01 header (3), request length (2); then the request: SUID, msg id.
	const uint8_t *r = b + 12;
	struct pico4_ssc_suid suid;
	uint32_t msg;
	memcpy(&suid.low, r + 3, 8);
	memcpy(&suid.high, r + 12, 8);
	memcpy(&msg, r + 21, 4);
	if (msg == 512) { // SUID lookup: answer for the data type in the payload
		bool accel = memmem(b, len, "accel", 5) != NULL;
		const struct pico4_ssc_suid *found = accel ? &fake.accel : &fake.gyro;
		uint8_t i_buf[32], p_buf[64];
		struct pico4_pb inner = {i_buf, 0, sizeof(i_buf), false}, p = {p_buf, 0, sizeof(p_buf), false};
		pico4_pb_fixed64(&inner, 1, found->low);
		pico4_pb_fixed64(&inner, 2, found->high);
		pico4_pb_bytes(&p, 1, accel ? "accel" : "gyro", accel ? 5 : 4);
		pico4_pb_bytes(&p, 2, inner.data, inner.len);
		const struct pico4_ssc_suid lookup = {0xABABABABABABABABULL, 0xABABABABABABABABULL};
		indicate(&lookup, 768, p.data, p.len);
	} else if (msg == 513) { // enable
		fake.accel_on |= suid.low == fake.accel.low && suid.high == fake.accel.high;
		fake.gyro_on |= suid.low == fake.gyro.low && suid.high == fake.gyro.high;
	}
	(void)fd;
	return (long)len;
}

static void
emit_samples(void)
{
	uint8_t p_buf[32];
	struct pico4_pb p = {p_buf, 0, sizeof(p_buf), false};
	pico4_pb_float(&p, 1, 0.5f);
	pico4_pb_float(&p, 1, -9.8f);
	pico4_pb_float(&p, 1, 0.25f);
	if (fake.accel_on) {
		indicate(&fake.accel, 1025, p.data, p.len);
	}
	if (fake.gyro_on) {
		indicate(&fake.gyro, 1025, p.data, p.len);
	}
}

static int
read_samples(struct pico4_ssc *ssc, struct pico4_ssc_sample *s, int max)
{
	int total = 0;
	for (int i = 0; i < 4; i++) {
		int n = pico4_ssc_read(ssc, 50, s + total, max - total);
		if (n < 0) {
			return n;
		}
		total += n;
	}
	return total;
}

static void
test_drop_and_return(void)
{
	static const struct pico4_ssc_transport transport = {fake_open, fake_send, fake_recv};
	static const char *const types[2] = {"accel", "gyro"};
	fake.up = true;
	fake.node = 9;
	fake.port = 0x4000;
	fake.accel = (struct pico4_ssc_suid){0x1111, 0xA0A0};
	fake.gyro = (struct pico4_ssc_suid){0x5555, 0x9999};
	pico4_ssc_set_transport(&transport);

	struct pico4_ssc *ssc = NULL;
	struct pico4_ssc_suid suids[2];
	struct pico4_ssc_sample s[8];
	EXPECT(pico4_ssc_subscribe(&ssc, types, suids, 2, 1000.0f, 200) == 0 && ssc != NULL);
	EXPECT(suids[0].low == 0x1111 && suids[1].low == 0x5555 && fake.accel_on && fake.gyro_on);
	emit_samples();
	EXPECT(read_samples(ssc, s, 8) == 2 && s[0].suid.low == 0x1111 && s[0].count == 3 && s[0].values[1] == -9.8f);

	// The DSP restarts: the name service reports the server gone, and the read says so.
	fake.up = false;
	fake.accel_on = fake.gyro_on = false;
	uint32_t gone[5] = {5, 0x190, 1, fake.node, fake.port};
	deliver(LOCAL_NODE, PORT_CTRL, gone, sizeof(gone));
	EXPECT(pico4_ssc_read(ssc, 200, s, 8) == -ECONNRESET);
	pico4_ssc_close(&ssc);

	// While it is down a new client finds no service, and nothing stays open.
	EXPECT(pico4_ssc_subscribe(&ssc, types, suids, 2, 1000.0f, 100) == -ETIMEDOUT && ssc == NULL);

	// Back on a new port with new UIDs: found, enabled and streaming again.
	fake.up = true;
	fake.port = 0x4001;
	fake.accel.low = 0x2222;
	fake.gyro.low = 0x6666;
	EXPECT(pico4_ssc_subscribe(&ssc, types, suids, 2, 1000.0f, 200) == 0);
	EXPECT(suids[0].low == 0x2222 && suids[1].low == 0x6666 && fake.accel_on && fake.gyro_on);
	emit_samples();
	EXPECT(read_samples(ssc, s, 8) == 2 && s[0].suid.low == 0x2222 && s[1].suid.low == 0x6666);
	pico4_ssc_close(&ssc);
	pico4_ssc_set_transport(NULL);
}

int
main(void)
{
	const struct pico4_ssc_suid accel = {0x1111222233334444ULL, 0xA0A0B0B0C0C0D0D0ULL};
	const struct pico4_ssc_suid gyro = {0x5555666677778888ULL, 0x9999AAAABBBBCCCCULL};
	const struct pico4_ssc_suid lookup = {0xABABABABABABABABULL, 0xABABABABABABABABULL};
	uint8_t buf[512], frame[600], payload[64];
	size_t n;

	// Requests: byte-identical to the hardware-verified Python client.
	struct pico4_pb pb = {payload, 0, sizeof(payload), false};
	pico4_pb_float(&pb, 1, 1000.0f);
	n = pico4_ssc_encode_request(buf, sizeof(buf), &accel, 513, pb.data, pb.len);
	EXPECT(same(buf, n, golden_enable_request, sizeof(golden_enable_request)));
	size_t f = pico4_ssc_encode_qmi(frame, sizeof(frame), 7, buf, n);
	EXPECT(same(frame, f, golden_enable_qmi, sizeof(golden_enable_qmi)));

	n = pico4_ssc_encode_request(buf, sizeof(buf), &accel, 10, NULL, 0);
	EXPECT(same(buf, n, golden_disable_request, sizeof(golden_disable_request)));

	pb = (struct pico4_pb){payload, 0, sizeof(payload), false};
	pico4_pb_bytes(&pb, 1, "gyro", 4);
	pico4_pb_varint(&pb, 2, 0);
	pico4_pb_varint(&pb, 3, 0);
	n = pico4_ssc_encode_request(buf, sizeof(buf), &lookup, 512, pb.data, pb.len);
	EXPECT(same(buf, n, golden_suid_request, sizeof(golden_suid_request)));

	// Buffers that are too small are reported, not overrun.
	EXPECT(pico4_ssc_encode_request(buf, 8, &accel, 513, payload, 5) == 0);
	EXPECT(pico4_ssc_encode_qmi(frame, 10, 7, buf, 40) == 0);

	// Sensor events: unpacked and packed floats; the config event is skipped.
	struct pico4_ssc_sample s[4];
	int count = pico4_ssc_decode_indication(golden_samples, sizeof(golden_samples), s, 4, NULL, NULL, NULL);
	EXPECT(count == 2);
	EXPECT(s[0].suid.low == accel.low && s[0].suid.high == accel.high);
	EXPECT(s[0].ticks == 123456789 && s[0].count == 3);
	EXPECT(s[0].values[0] == 1.5f && s[0].values[1] == -2.25f && s[0].values[2] == 9.75f);
	EXPECT(s[1].ticks == 123476789 && s[1].count == 3);
	EXPECT(s[1].values[0] == 0.5f && s[1].values[1] == 0.25f && s[1].values[2] == -0.125f);
	// Opaque payload bytes (as ble_mcu sends them): only a length-delimited field 1 is kept.
	EXPECT(s[0].raw_len == 0);
	EXPECT(s[1].raw_len == 12 && s[1].raw[3] == 0x3f && s[1].raw[11] == 0xbe);

	// The IMU's temperature: a single value (handle_sample takes it before the 3-axis check).
	EXPECT(pico4_ssc_decode_indication(golden_temperature, sizeof(golden_temperature), s, 4, NULL, NULL, NULL) == 1);
	EXPECT(s[0].suid.low == 0x0102030405060708ULL && s[0].suid.high == 0x1112131415161718ULL);
	EXPECT(s[0].ticks == 1000 && s[0].count == 1);
	EXPECT(s[0].values[0] > 43.35f && s[0].values[0] < 43.37f);

	// Proximity: an event of its own (769), {type, raw, status}.
	EXPECT(pico4_ssc_decode_indication(golden_proximity, sizeof(golden_proximity), s, 4, NULL, NULL, NULL) == 1);
	EXPECT(s[0].proximity && s[0].count == 3 && s[0].ticks == 2000);
	EXPECT(s[0].values[0] == 1.0f && s[0].values[1] == 210.0f && s[0].values[2] == 3.0f);
	// Sensor samples are not proximity events.
	pico4_ssc_decode_indication(golden_temperature, sizeof(golden_temperature), s, 4, NULL, NULL, NULL);
	EXPECT(!s[0].proximity);

	// Output space is respected.
	EXPECT(pico4_ssc_decode_indication(golden_samples, sizeof(golden_samples), s, 1, NULL, NULL, NULL) == 1);

	// SUID lookup result: first SUID of the requested type only.
	struct pico4_ssc_suid found = {0, 0};
	bool ok = false;
	pico4_ssc_decode_indication(golden_suid_event, sizeof(golden_suid_event), s, 0, "gyro", &found, &ok);
	EXPECT(ok && found.low == gyro.low && found.high == gyro.high);
	ok = false;
	pico4_ssc_decode_indication(golden_suid_event, sizeof(golden_suid_event), s, 0, "accel", &found, &ok);
	EXPECT(!ok);

	// Truncated or corrupted input is rejected.
	for (size_t cut = 7; cut < sizeof(golden_samples); cut += 9) {
		int r = pico4_ssc_decode_indication(golden_samples, cut, s, 4, NULL, NULL, NULL);
		EXPECT(r == -EINVAL || (r >= 0 && r <= 2));
	}
	uint8_t bad[sizeof(golden_samples)];
	memcpy(bad, golden_samples, sizeof(bad));
	bad[21] = 0xff; // data length (u16 in TLV 0x02) beyond the TLV
	bad[22] = 0xff;
	EXPECT(pico4_ssc_decode_indication(bad, sizeof(bad), s, 4, NULL, NULL, NULL) == -EINVAL);

	// Responses and other messages carry no samples.
	uint8_t response[] = {0x02, 0x01, 0x00, 0x20, 0x00, 0x07, 0x00, 0x02, 0x04, 0x00, 0, 0, 0, 0};
	EXPECT(pico4_ssc_decode_indication(response, sizeof(response), s, 4, NULL, NULL, NULL) == 0);

	// The name service's notices when the sensor DSP restarts: the service at node 9 port
	// 0x4000 is gone (DEL_SERVER for it, DEL_CLIENT of its port); other services and
	// ports, a new server and short packets are not.
	const uint32_t del_server[5] = {5, 0x190, 1, 9, 0x4000}, other_service[5] = {5, 0x191, 1, 9, 0x4000};
	const uint32_t other_port[5] = {5, 0x190, 1, 9, 0x4001}, del_client[5] = {6, 9, 0x4000, 0, 0};
	const uint32_t new_server[5] = {4, 0x190, 1, 9, 0x4000};
	EXPECT(pico4_ssc_control_gone((const uint8_t *)del_server, 20, 9, 0x4000));
	EXPECT(pico4_ssc_control_gone((const uint8_t *)del_client, 20, 9, 0x4000));
	EXPECT(!pico4_ssc_control_gone((const uint8_t *)other_service, 20, 9, 0x4000));
	EXPECT(!pico4_ssc_control_gone((const uint8_t *)other_port, 20, 9, 0x4000));
	EXPECT(!pico4_ssc_control_gone((const uint8_t *)new_server, 20, 9, 0x4000));
	EXPECT(!pico4_ssc_control_gone((const uint8_t *)del_server, 16, 9, 0x4000));

	// A fake service that drops and returns, through the real client code.
	test_drop_and_return();

	if (failures == 0) {
		printf("pico4_ssc: ok\n");
	}
	return failures ? 1 : 0;
}
