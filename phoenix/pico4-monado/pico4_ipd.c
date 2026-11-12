// Copyright 2026, overpressure contributors.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Read the PICO 4 lens separation from the GD32 MCU (see pico4_ipd.h).
 * @ingroup drv_pico4
 */

#include "pico4_ipd.h"

#include <errno.h>
#include <fcntl.h>
#include <string.h>
#include <sys/file.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

const uint8_t pico4_ipd_get_position[6] = {0xA0, 0x08, 0x00, 0x00, 0x00, 0xA0 ^ 0x08};

bool
pico4_ipd_parse_position(const uint8_t *reply, size_t size, double *out_ipd_m)
{
	if (size < 6 || reply[0] != 0x79) {
		return false;
	}
	uint8_t x = 0;
	for (int i = 0; i < 5; i++) {
		x ^= reply[i];
	}
	if (x != reply[5]) {
		return false;
	}
	unsigned v[2] = {reply[1] | (unsigned)reply[2] << 8, reply[3] | (unsigned)reply[4] << 8};
	for (int i = 0; i < 2; i++) {
		if (v[i] >= 5700 && v[i] <= 7300) {
			*out_ipd_m = v[i] / 100000.0;
			return true;
		}
	}
	return false;
}

int
pico4_ipd_read(const char *device, double *out_ipd_m)
{
	int fd = open(device, O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
	if (fd < 0) {
		return -errno;
	}
	int ret = 0;
	// pico4-fan sends the fan's frames on the same port (docs/cooling.md). Each side
	// holds an exclusive lock for its transaction, about 0.1 s; wait for it up to 1 s.
	for (int i = 0; flock(fd, LOCK_EX | LOCK_NB) != 0; i++) {
		if (errno != EWOULDBLOCK || i == 200) {
			ret = errno == EWOULDBLOCK ? -EBUSY : -errno;
			goto out;
		}
		const struct timespec wait = {0, 5 * 1000 * 1000};
		nanosleep(&wait, NULL);
	}
	struct termios t;
	if (tcgetattr(fd, &t) != 0) {
		ret = -errno;
		goto out;
	}
	cfmakeraw(&t);
	cfsetispeed(&t, B115200);
	cfsetospeed(&t, B115200);
	t.c_cflag |= CLOCAL | CREAD;
	t.c_cc[VMIN] = 0;
	t.c_cc[VTIME] = 0;
	if (tcsetattr(fd, TCSANOW, &t) != 0) {
		ret = -errno;
		goto out;
	}
	tcflush(fd, TCIOFLUSH);
	if (write(fd, pico4_ipd_get_position, sizeof(pico4_ipd_get_position)) != sizeof(pico4_ipd_get_position)) {
		ret = -EIO;
		goto out;
	}

	uint8_t reply[16];
	size_t n = 0;
	ret = -ETIMEDOUT;
	for (int i = 0; i < 200 && ret == -ETIMEDOUT; i++) {
		const struct timespec ms = {0, 1000 * 1000};
		nanosleep(&ms, NULL);
		ssize_t r = read(fd, reply + n, sizeof(reply) - n);
		if (r > 0) {
			n += (size_t)r;
		}
		// Skip anything before the reply marker.
		while (n > 0 && reply[0] != 0x79) {
			memmove(reply, reply + 1, --n);
		}
		if (n >= 6) {
			ret = pico4_ipd_parse_position(reply, n, out_ipd_m) ? 0 : -EPROTO;
		}
	}
out:
	close(fd);
	return ret;
}
