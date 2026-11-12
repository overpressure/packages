// Copyright 2026, overpressure contributors.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Tracking-camera frame sync through the controller radio (see pico4_station.h).
 * @ingroup drv_pico4
 */

#ifndef _GNU_SOURCE // Monado defines it on the command line
#define _GNU_SOURCE // usleep, O_CLOEXEC and clock_gettime under -std=c11
#endif

#include "pico4_station.h"

#include <errno.h>
#include <fcntl.h>
#include <linux/spi/spidev.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/file.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/un.h>
#include <time.h>
#include <unistd.h>

#define HEADER_SEND 0xAA
#define HEADER_REPLY 0xBB
#define HEADER_NOTIFY 0xA5
#define TARGET_STATION 0x01
#define TARGET_LEFT 0x02  //!< Controller targets: StationService::SetMotor 0x72054.
#define TARGET_RIGHT 0x03
#define MOTOR_LEN 4       //!< SetMotor's payload (packetSendData at 0x72070).
#define NOTIFY_STREAM 0x01   //!< A5 01: streamed data, not acknowledged.
#define NOTIFY_CONNECT 0x02  //!< A5 02: a controller connected (repeated a few times).
#define NOTIFY_DISCONNECT 0x03 //!< A5 03: a controller disconnected.
#define NOTIFY_ANNOUNCE 0x06 //!< A5 06: the station announces itself after power-on.
#define CONNECT_OK 0x01      //!< Status (byte 9, low four bits) of a valid connection.
#define CALIBRATION_NOT_READY 0xFF
#define CALIBRATION_ERROR 0xEE
#define CALIBRATION_TRIES 30     //!< GetCalibrationFromStation 0x9f158.
#define CALIBRATION_WAIT_MS 100  //!< waitForDataCommon(100) at 0x9f124.
#define CALIBRATION_RETRY_MS 50  //!< usleep(50000) after a "not ready", 0x9f0e4.
//! The switch of the station's supply (regulator-output in the device tree).
#define STATION_POWER "/sys/devices/platform/station-power/state"
#define SPIDEV_QUEUE 8         //!< Frames that came back with commands, kept for the next read.
#define SPIDEV_POLL_US 2000    //!< How often the spidev is asked for the station's next frame.

#define CAMERA_INIT 0x01
#define CAMERA_STREAM 0x02
#define CAMERA_SET_PARA 0x03 //!< Exposure and gain; 28-byte payload.
#define CAMERA_SET_PARA_LEN 28
//! Camera op 05: the modules' OTP ids (StationService::GetCameraOTPInfo, mode 0: sub-ops 01, 02).
#define CAMERA_OTP 0x05
#define CAMERA_OTP_READ 0x01
#define CAMERA_OTP_RESULT 0x02
#define CAMERA_OTP_WAIT_MS 2000
#define CAMERA_ON 0x01
#define CAMERA_OFF 0x02
#define CAMERA_ALL 0x00
#define FPS_50 0x01        //!< The stock setting (sys_station_camera_fps 50).
#define FPS_60 0x02        //!< PICO OS's other rate (sys_station_camera_fps 60).
#define DEVICE_TYPE 0x00   //!< DeviceTypeGet() on the PICO 4.
#define EXPOSURE_FIRST_ROW 0x02 //!< As PICO OS initialises the cameras.

struct pico4_station
{
	int fd;
	//! Commands and polling may come from different threads; a command keeps
	//! the lock until its reply arrived, so a poller cannot take the reply.
	pthread_mutex_t lock;
	bool announced;
	//! The cameras' frame rate the announcement is answered with: 50 or 60 Hz.
	unsigned camera_hz;
	//! The last reply to a camera command: op, status.
	int camera_op, camera_status;
	int slow_adv_status; //!< 0 no reply yet, 1 confirmed, -1 refused.
	//! The last answer to a pairing command: the command (-1 none yet) and byte 4.
	int pair_cmd, pair_status;
	//! The cameras' module ids from the OTP replies (a zero byte keeps the value before).
	uint8_t otp[4];
	//! CameraSetPara's event index of the last command (0-9).
	unsigned para_index;
	//! Connection state per hand (index hand - 1).
	struct pico4_station_controller controllers[2];
	//! The last calibration reply: target, echoed index, data.
	int calib_target, calib_index;
	uint8_t calib_data[PICO4_STATION_CALIBRATION_CHUNK];
	//! The frames that came back with transfers.
	uint8_t queue[SPIDEV_QUEUE][PICO4_STATION_FRAME];
	unsigned queued;
	//! The last frame that came back: the station repeats it on every transfer until it has
	//! another one.
	uint8_t last[PICO4_STATION_FRAME];
};

uint8_t
pico4_station_crc8(const uint8_t *data, size_t size)
{
	uint8_t c = 0;
	for (size_t i = 0; i < size; i++) {
		c ^= data[i];
		for (int b = 0; b < 8; b++) {
			c = (c & 1) ? (uint8_t)((c >> 1) ^ 0x8C) : (uint8_t)(c >> 1);
		}
	}
	return c;
}

static bool
allowed(uint8_t cmd, const uint8_t *p, size_t n)
{
	switch (cmd) {
	case PICO4_STATION_CMD_ACK: return n == 2;
	case PICO4_STATION_CMD_CAMERA_FPS: return n == 3 && (p[0] == 1 || p[0] == 2) && p[2] == 1;
	case PICO4_STATION_CMD_SLOW_ADV: return n == 2 && p[0] == 0 && p[1] == 0; // the controllers; never the trackers
	case PICO4_STATION_PAIR:
	case PICO4_STATION_UNPAIR:
	case PICO4_STATION_PAIR_STOP: // one controller's mask; never a tracker's bits or a radio address
		return n == 1 && (p[0] == PICO4_STATION_HAND_LEFT || p[0] == PICO4_STATION_HAND_RIGHT);
	case PICO4_STATION_CMD_CAMERA:
		if (n == 4 && p[0] == CAMERA_INIT) {
			return p[2] <= 2 && p[3] <= 1;
		}
		if (n == CAMERA_SET_PARA_LEN && p[0] == CAMERA_SET_PARA) {
			return true;
		}
		if (n == 2 && p[0] == CAMERA_OTP) {
			return p[1] == CAMERA_OTP_READ || p[1] == CAMERA_OTP_RESULT; // never sub-op 03 or others
		}
		return n == 3 && p[0] == CAMERA_STREAM && p[1] <= 4 && (p[2] == CAMERA_ON || p[2] == CAMERA_OFF);
	default: return false;
	}
}

//! Frames to a controller: the plain motor command (play flag 0 or 1) and the calibration query.
static bool
allowed_controller(uint8_t target, uint8_t cmd, const uint8_t *p, size_t n)
{
	if (target != TARGET_LEFT && target != TARGET_RIGHT) {
		return false;
	}
	switch (cmd) {
	case PICO4_STATION_CMD_MOTOR: return n == MOTOR_LEN && p[0] <= 1;
	case PICO4_STATION_CMD_CALIBRATION: return n == 1 && p[0] >= 1 && p[0] <= PICO4_STATION_CALIBRATION_CHUNKS;
	default: return false;
	}
}

static void
build(uint8_t out[PICO4_STATION_FRAME], uint8_t target, uint8_t cmd, const uint8_t *payload, size_t size)
{
	memset(out, 0, PICO4_STATION_FRAME);
	out[0] = HEADER_SEND;
	out[1] = target;
	out[2] = cmd;
	memcpy(out + 3, payload, size);
	out[39] = pico4_station_crc8(out, 39);
}

bool
pico4_station_frame(uint8_t out[PICO4_STATION_FRAME], uint8_t cmd, const uint8_t *payload, size_t size)
{
	if (!allowed(cmd, payload, size)) {
		return false;
	}
	build(out, TARGET_STATION, cmd, payload, size);
	return true;
}

bool
pico4_station_vibrate(uint8_t out[PICO4_STATION_FRAME],
                      enum pico4_station_hand hand,
                      unsigned level,
                      unsigned scale_percent,
                      unsigned duration_ms)
{
	if ((hand != PICO4_STATION_HAND_LEFT && hand != PICO4_STATION_HAND_RIGHT) || scale_percent > 100) {
		return false;
	}
	const uint8_t target = hand == PICO4_STATION_HAND_LEFT ? TARGET_LEFT : TARGET_RIGHT;
	uint8_t p[MOTOR_LEN] = {0};

	// Level 0 stops: the whole payload stays zero (SetMotor 0x72050).
	if (level != 0) {
		if (level > 255) { // StationClient 0x37518
			level = 255;
		}
		unsigned amplitude = level * scale_percent / 100; // 0x72000..0x7202c
		if (duration_ms > 0xFFFF) {
			duration_ms = 0xFFFF;
		}
		p[0] = 1;
		p[1] = (uint8_t)(amplitude < 255 ? amplitude : 255);
		p[2] = (uint8_t)(duration_ms & 0xff);
		p[3] = (uint8_t)(duration_ms >> 8);
	}
	if (!allowed_controller(target, PICO4_STATION_CMD_MOTOR, p, MOTOR_LEN)) {
		return false;
	}
	build(out, target, PICO4_STATION_CMD_MOTOR, p, MOTOR_LEN);
	return true;
}

bool
pico4_station_calibration_query(uint8_t out[PICO4_STATION_FRAME], enum pico4_station_hand hand, unsigned index)
{
	if ((hand != PICO4_STATION_HAND_LEFT && hand != PICO4_STATION_HAND_RIGHT) || index > 255) {
		return false;
	}
	const uint8_t target = hand == PICO4_STATION_HAND_LEFT ? TARGET_LEFT : TARGET_RIGHT;
	const uint8_t p[1] = {(uint8_t)index};
	if (!allowed_controller(target, PICO4_STATION_CMD_CALIBRATION, p, 1)) {
		return false;
	}
	build(out, target, PICO4_STATION_CMD_CALIBRATION, p, 1);
	return true;
}

bool
pico4_station_pairing_frame(uint8_t out[PICO4_STATION_FRAME], enum pico4_station_pairing op, enum pico4_station_hand hand)
{
	if (op != PICO4_STATION_PAIR && op != PICO4_STATION_UNPAIR && op != PICO4_STATION_PAIR_STOP) {
		return false;
	}
	const uint8_t mask[1] = {(uint8_t)hand};
	return pico4_station_frame(out, (uint8_t)op, mask, sizeof(mask));
}

//! Bytes [from, to) as a string up to the first NUL, printable ASCII only.
static void
copy_text(char *out, size_t size, const uint8_t *f, int from, int to)
{
	size_t n = 0;
	for (int i = from; i < to && n + 1 < size && f[i] != 0; i++) {
		out[n++] = (f[i] >= 0x20 && f[i] < 0x7f) ? (char)f[i] : '?';
	}
	out[n] = '\0';
}

int
pico4_station_parse_connection(const uint8_t f[PICO4_STATION_FRAME],
                               enum pico4_station_hand *hand,
                               struct pico4_station_controller *info)
{
	if (f[0] != HEADER_NOTIFY || (f[1] != NOTIFY_CONNECT && f[1] != NOTIFY_DISCONNECT) ||
	    (f[2] != PICO4_STATION_HAND_LEFT && f[2] != PICO4_STATION_HAND_RIGHT)) {
		return -1;
	}
	*hand = (enum pico4_station_hand)f[2];
	if (f[1] == NOTIFY_DISCONNECT) {
		return 0;
	}
	if ((f[9] & 0x0f) != CONNECT_OK) {
		return -1;
	}
	memset(info, 0, sizeof(*info));
	info->connected = true;
	copy_text(info->software, sizeof(info->software), f, 3, 9);
	copy_text(info->serial, sizeof(info->serial), f, 13, 13 + PICO4_STATION_SERIAL_LEN);
	return 1;
}

void
pico4_station_haptic_map(float amplitude, int64_t duration_ns, unsigned *out_level, unsigned *out_duration_ms)
{
	int64_t ms = duration_ns == -1 ? 500 : duration_ns / 1000000; // set_output 0xa9d5c..0xa9da8
	if (!(amplitude > 0.0f) || ms == 0) {                          // also NaN
		*out_level = 0;
		*out_duration_ms = 0;
		return;
	}
	if (amplitude > 1.0f) {
		amplitude = 1.0f;
	}
	if (ms < 10) { // SetPhyControllerVibrate 0x28d68..0x28d7c, also negative
		ms = 10;
	} else if (ms > 0xFFFF) {
		ms = 0xFFFF;
	}
	*out_level = (unsigned)(amplitude * 255.0f); // 0x28d94..0x28da4
	*out_duration_ms = (unsigned)ms;
}

bool
pico4_station_valid(const uint8_t f[PICO4_STATION_FRAME])
{
	return (f[0] == HEADER_REPLY || f[0] == HEADER_NOTIFY) && pico4_station_crc8(f, 39) == f[39];
}

static int64_t
now_ms(void)
{
	struct timespec t;
	clock_gettime(CLOCK_MONOTONIC, &t);
	return (int64_t)t.tv_sec * 1000 + t.tv_nsec / 1000000;
}

/*
 * The spidev moves no frames on its own: each transfer is full duplex, and the station's next
 * frame comes back with whatever goes out, a command or zeros, which carry no header and which the station
 * ignores. Frames that come back keep their order in the queue until a read takes them.
 */
static int
spidev_transfer(struct pico4_station *st, const uint8_t tx[PICO4_STATION_FRAME])
{
	uint8_t rx[PICO4_STATION_FRAME] = {0};
	struct spi_ioc_transfer t = {
	    .tx_buf = (uintptr_t)tx,
	    .rx_buf = (uintptr_t)rx,
	    .len = PICO4_STATION_FRAME,
	};
	if (ioctl(st->fd, SPI_IOC_MESSAGE(1), &t) < 0) {
		return -errno;
	}
	if (pico4_station_valid(rx) && memcmp(rx, st->last, sizeof(rx)) != 0) {
		memcpy(st->last, rx, sizeof(rx));
		if (st->queued == SPIDEV_QUEUE) { // the oldest goes
			memmove(st->queue[0], st->queue[1], sizeof(st->queue) - sizeof(st->queue[0]));
			st->queued--;
		}
		memcpy(st->queue[st->queued++], rx, PICO4_STATION_FRAME);
	}
	return 0;
}

//! Ask the spidev until a frame is queued or @p timeout_ms passed: 1, 0 or a negative errno.
static int
spidev_wait(struct pico4_station *st, int timeout_ms)
{
	static const uint8_t idle[PICO4_STATION_FRAME];
	int64_t end = now_ms() + timeout_ms;
	for (;;) {
		if (st->queued > 0) {
			return 1;
		}
		int r = spidev_transfer(st, idle);
		if (r < 0) {
			return r;
		}
		if (st->queued > 0) {
			return 1;
		}
		if (now_ms() >= end) {
			return 0;
		}
		usleep(SPIDEV_POLL_US);
	}
}

//! Send one frame: a transfer.
static int
send_frame(struct pico4_station *st, const uint8_t f[PICO4_STATION_FRAME])
{
	return spidev_transfer(st, f);
}

static int
send_cmd(struct pico4_station *st, uint8_t cmd, const uint8_t *payload, size_t size)
{
	uint8_t f[PICO4_STATION_FRAME];
	if (!pico4_station_frame(f, cmd, payload, size)) {
		return -EINVAL;
	}
	return send_frame(st, f);
}

//! One frame within @p timeout_ms: 1 handled, 0 none, negative errno.
static int
read_one(struct pico4_station *st, int timeout_ms)
{
	uint8_t f[PICO4_STATION_FRAME];
	int r;

	if ((r = spidev_wait(st, timeout_ms)) <= 0) {
		return r;
	}
	memcpy(f, st->queue[0], sizeof(f));
	memmove(st->queue[0], st->queue[1], --st->queued * sizeof(st->queue[0]));
	if (!(f[0] == HEADER_NOTIFY && f[1] == NOTIFY_STREAM)) {
		send_cmd(st, PICO4_STATION_CMD_ACK, f + 37, 2);
	}
	if (f[0] == HEADER_NOTIFY && f[1] == NOTIFY_ANNOUNCE && !st->announced) {
		// As the stock service: the camera frame rate, after which camera commands work.
		const uint8_t fps[3] = {st->camera_hz == 60 ? FPS_60 : FPS_50, DEVICE_TYPE, 1};
		st->announced = true;
		send_cmd(st, PICO4_STATION_CMD_CAMERA_FPS, fps, sizeof(fps));
	}
	if (f[0] == HEADER_REPLY && f[2] == PICO4_STATION_CMD_CAMERA) {
		st->camera_op = f[3];
		st->camera_status = f[4];
		if (f[1] == TARGET_STATION && f[3] == CAMERA_OTP) {
			pico4_station_parse_otp(f, st->otp);
		}
	}
	if (f[0] == HEADER_REPLY && f[1] == TARGET_STATION && f[2] == PICO4_STATION_CMD_SLOW_ADV) {
		st->slow_adv_status = f[4] == 1 ? 1 : -1;
	}
	if (f[0] == HEADER_REPLY && f[1] == TARGET_STATION &&
	    (f[2] == PICO4_STATION_PAIR || f[2] == PICO4_STATION_UNPAIR || f[2] == PICO4_STATION_PAIR_STOP)) {
		st->pair_cmd = f[2];
		st->pair_status = f[4];
	}
	if (f[0] == HEADER_REPLY && (f[1] == TARGET_LEFT || f[1] == TARGET_RIGHT) &&
	    f[2] == PICO4_STATION_CMD_CALIBRATION) {
		st->calib_target = f[1];
		st->calib_index = f[3];
		memcpy(st->calib_data, f + 4, sizeof(st->calib_data));
	}
	enum pico4_station_hand hand;
	struct pico4_station_controller info;
	int c = pico4_station_parse_connection(f, &hand, &info);
	if (c >= 0) {
		struct pico4_station_controller *s = &st->controllers[hand - 1];
		if (c == 0) {
			s->connected = false;
		} else {
			if (!s->connected) {
				s->connects++;
			}
			info.connects = s->connects;
			*s = info;
		}
	}
	return 1;
}

void
pico4_station_set_camera_hz(struct pico4_station *st, unsigned hz)
{
	st->camera_hz = hz == 60 ? 60 : 50;
}

int
pico4_station_open(struct pico4_station **out, const char *device)
{
	struct pico4_station *st = calloc(1, sizeof(*st));
	if (st == NULL) {
		return -ENOMEM;
	}
	st->fd = open(device != NULL ? device : PICO4_STATION_DEVICE, O_RDWR | O_CLOEXEC);
	/*
	 * One owner at a time: a second runtime (Monado's service beside SteamVR's driver_monado)
	 * would reset the station under the first one and read its frames. The lock goes with the
	 * file, also when the process dies.
	 */
	if (st->fd >= 0 && flock(st->fd, LOCK_EX | LOCK_NB) < 0) {
		int r = errno == EWOULDBLOCK ? -EBUSY : -errno;
		close(st->fd);
		free(st);
		return r;
	}
	if (st->fd < 0) {
		int r = -errno;
		free(st);
		return r;
	}
	pthread_mutex_init(&st->lock, NULL);
	st->camera_hz = 50;
	*out = st;
	return 0;
}

void
pico4_station_close(struct pico4_station *st)
{
	if (st != NULL) {
		close(st->fd);
		pthread_mutex_destroy(&st->lock);
		free(st);
	}
}

static int
power(struct pico4_station *st, int on)
{
	const char *state = on ? "enabled" : "disabled";
	int fd = open(STATION_POWER, O_WRONLY | O_CLOEXEC);
	if (fd < 0) {
		return -errno;
	}
	ssize_t w = write(fd, state, strlen(state));
	int r = w < 0 ? -errno : 0;
	close(fd);
	st->queued = 0; // frames from before the power cycle
	memset(st->last, 0, sizeof(st->last));
	return r;
}

static int
reset_locked(struct pico4_station *st)
{
	int r;
	if ((r = power(st, 0)) < 0) {
		return r;
	}
	usleep(100 * 1000);
	st->announced = false;
	// The controllers reconnect after the power cycle, each with a connect notification.
	st->controllers[0].connected = st->controllers[1].connected = false;
	if ((r = power(st, 1)) < 0) {
		return r;
	}
	for (int64_t end = now_ms() + 3000; !st->announced && now_ms() < end;) {
		if ((r = read_one(st, 100)) < 0) {
			return r;
		}
	}
	if (!st->announced) {
		return -ETIMEDOUT;
	}
	// Take the replies to the frame-rate answer.
	for (int64_t end = now_ms() + 200; now_ms() < end;) {
		if ((r = read_one(st, 50)) < 0) {
			return r;
		}
	}
	return 0;
}

int
pico4_station_sleep(struct pico4_station *st)
{
	if (st == NULL) {
		return 0;
	}
	const uint8_t controllers[2] = {0, 0};
	pthread_mutex_lock(&st->lock);
	// As the stock service's goToSleep: two tries of 100 ms each, then the power goes either way.
	st->slow_adv_status = 0;
	for (int attempt = 0; attempt < 2 && st->slow_adv_status != 1; attempt++) {
		st->slow_adv_status = 0;
		int sent = send_cmd(st, PICO4_STATION_CMD_SLOW_ADV, controllers, sizeof(controllers));
		for (int64_t end = now_ms() + 100; sent == 0 && st->slow_adv_status == 0 && now_ms() < end;) {
			if (read_one(st, 10) < 0) {
				break;
			}
		}
	}
	int status = st->slow_adv_status;
	int r = power(st, 0);
	pthread_mutex_unlock(&st->lock);
	return r < 0 ? r : status == 1 ? 0 : -ETIMEDOUT;
}

int
pico4_station_reset(struct pico4_station *st)
{
	pthread_mutex_lock(&st->lock);
	int r = reset_locked(st);
	pthread_mutex_unlock(&st->lock);
	return r;
}

int
pico4_station_poll(struct pico4_station *st, int timeout_ms)
{
	int64_t end = now_ms() + timeout_ms;
	int n = 0;
	// Every question to the spidev is a transfer, which needs the lock: ask, then sleep
	// without it, so that a command can take its turn.
	do {
		pthread_mutex_lock(&st->lock);
		int r = read_one(st, 0);
		pthread_mutex_unlock(&st->lock);
		if (r < 0) {
			return r;
		}
		n += r;
		if (r == 0) {
			usleep(SPIDEV_POLL_US);
		}
	} while (now_ms() < end);
	return n;
}

//! Send a camera command and wait for its reply; the status byte is 1 on success.
static int
camera_cmd(struct pico4_station *st, const uint8_t *payload, size_t size, int timeout_ms)
{
	int r;
	st->camera_op = -1;
	if ((r = send_cmd(st, PICO4_STATION_CMD_CAMERA, payload, size)) < 0) {
		return r;
	}
	for (int64_t end = now_ms() + timeout_ms; st->camera_op != payload[0] && now_ms() < end;) {
		if ((r = read_one(st, 50)) < 0) {
			return r;
		}
	}
	if (st->camera_op != payload[0]) {
		return -ETIMEDOUT;
	}
	return st->camera_status == 1 ? 0 : -EIO;
}

int
pico4_station_camera_start(struct pico4_station *st)
{
	const uint8_t init[4] = {CAMERA_INIT, 0, EXPOSURE_FIRST_ROW, 0};
	const uint8_t on[3] = {CAMERA_STREAM, CAMERA_ALL, CAMERA_ON};
	pthread_mutex_lock(&st->lock);
	int r = camera_cmd(st, init, sizeof(init), 2000);
	if (r == 0) {
		r = camera_cmd(st, on, sizeof(on), 1000);
	}
	pthread_mutex_unlock(&st->lock);
	return r;
}

void
pico4_station_camera_para_each(uint8_t out[CAMERA_SET_PARA_LEN],
                               const struct pico4_station_exposure scene[4],
                               struct pico4_station_exposure leds,
                               unsigned index)
{
	if (leds.lines > PICO4_STATION_LED_MAX_LINES) {
		leds.lines = PICO4_STATION_LED_MAX_LINES;
	}
	if (leds.gain > PICO4_STATION_LED_MAX_GAIN) {
		leds.gain = PICO4_STATION_LED_MAX_GAIN;
	}
	memset(out, 0, CAMERA_SET_PARA_LEN);
	out[0] = CAMERA_SET_PARA;
	for (int i = 0; i < 8; i++) {
		const struct pico4_station_exposure *x = i < 4 ? &scene[i] : &leds;
		uint8_t *e = out + 1 + 3 * i + (i >= 4 ? 3 : 0);
		e[0] = (uint8_t)(x->lines & 0xff);
		e[1] = (uint8_t)(x->lines >> 8);
		e[2] = (uint8_t)x->gain;
	}
	out[13] = (uint8_t)(PICO4_STATION_PARA_SYNC_50HZ & 0xff);
	out[14] = (uint8_t)(PICO4_STATION_PARA_SYNC_50HZ >> 8);
	out[15] = (uint8_t)(index % 10);
}

static int
send_camera_para(struct pico4_station *st,
                 const struct pico4_station_exposure scene[4],
                 struct pico4_station_exposure leds)
{
	// The station answers nothing.
	uint8_t p[CAMERA_SET_PARA_LEN];
	pthread_mutex_lock(&st->lock);
	st->para_index = (st->para_index + 1) % 10; // counted before each command, as PICO OS does
	pico4_station_camera_para_each(p, scene, leds, st->para_index);
	if (st->camera_hz != 50) {
		p[13] = (uint8_t)(PICO4_STATION_PARA_SYNC_OTHER & 0xff);
		p[14] = (uint8_t)(PICO4_STATION_PARA_SYNC_OTHER >> 8);
	}
	int r = send_cmd(st, PICO4_STATION_CMD_CAMERA, p, sizeof(p));
	pthread_mutex_unlock(&st->lock);
	return r;
}

int
pico4_station_camera_set_each(struct pico4_station *st, const struct pico4_station_exposure scene[4])
{
	const struct pico4_station_exposure leds = {4, 26};
	return send_camera_para(st, scene, leds);
}

int
pico4_station_camera_set_exposure(struct pico4_station *st, unsigned exposure_lines, unsigned gain)
{
	// The second set as PICO OS fills it: the station's own LED setting (4 lines, gain 26).
	const struct pico4_station_exposure scene = {exposure_lines, gain}, leds = {4, 26};
	const struct pico4_station_exposure each[4] = {scene, scene, scene, scene};
	return send_camera_para(st, each, leds);
}

int
pico4_station_motor(struct pico4_station *st, enum pico4_station_hand hand, unsigned level, unsigned duration_ms)
{
	uint8_t f[PICO4_STATION_FRAME];
	if (!pico4_station_vibrate(f, hand, level, PICO4_STATION_MOTOR_SCALE_PERCENT, duration_ms)) {
		return -EINVAL;
	}
	pthread_mutex_lock(&st->lock);
	int r = send_frame(st, f);
	pthread_mutex_unlock(&st->lock);
	return r;
}

bool
pico4_station_parse_otp(const uint8_t f[PICO4_STATION_FRAME], uint8_t ids[4])
{
	if (f[0] != HEADER_REPLY || f[1] != TARGET_STATION || f[2] != PICO4_STATION_CMD_CAMERA || f[3] != CAMERA_OTP) {
		return false;
	}
	for (int k = 0; k < 4; k++) {
		if (f[4 + k] != 0) { // as PICO OS's reply handler: a zero keeps the value before
			ids[k] = f[4 + k];
		}
	}
	return true;
}

int
pico4_station_camera_otp(struct pico4_station *st, uint8_t out[4])
{
	const uint8_t subops[2] = {CAMERA_OTP_READ, CAMERA_OTP_RESULT};
	int r = 0;
	pthread_mutex_lock(&st->lock);
	memset(st->otp, 0, sizeof(st->otp));
	for (int i = 0; i < 2 && r == 0; i++) {
		const uint8_t p[2] = {CAMERA_OTP, subops[i]};
		st->camera_op = -1;
		if ((r = send_cmd(st, PICO4_STATION_CMD_CAMERA, p, sizeof(p))) < 0) {
			break;
		}
		for (int64_t end = now_ms() + CAMERA_OTP_WAIT_MS; st->camera_op != CAMERA_OTP && now_ms() < end;) {
			if ((r = read_one(st, 50)) < 0) {
				break;
			}
			r = 0;
		}
		if (r == 0 && st->camera_op != CAMERA_OTP) {
			r = -ETIMEDOUT;
		}
	}
	memcpy(out, st->otp, sizeof(st->otp));
	pthread_mutex_unlock(&st->lock);
	return r;
}

int
pico4_station_camera_stop(struct pico4_station *st)
{
	const uint8_t off[3] = {CAMERA_STREAM, CAMERA_ALL, CAMERA_OFF};
	pthread_mutex_lock(&st->lock);
	int r = camera_cmd(st, off, sizeof(off), 1000);
	pthread_mutex_unlock(&st->lock);
	return r;
}

void
pico4_station_get_controller(struct pico4_station *st,
                             enum pico4_station_hand hand,
                             struct pico4_station_controller *out)
{
	memset(out, 0, sizeof(*out));
	if (hand != PICO4_STATION_HAND_LEFT && hand != PICO4_STATION_HAND_RIGHT) {
		return;
	}
	pthread_mutex_lock(&st->lock);
	*out = st->controllers[hand - 1];
	pthread_mutex_unlock(&st->lock);
}

//! One chunk: the reply's data, or a negative errno (-EIO error reply, -ETIMEDOUT).
static int
calibration_chunk(struct pico4_station *st, const uint8_t query[PICO4_STATION_FRAME], unsigned index, uint8_t *out)
{
	for (int attempt = 0; attempt < CALIBRATION_TRIES; attempt++) {
		int r;
		st->calib_index = -1;
		if ((r = send_frame(st, query)) < 0) {
			return r;
		}
		for (int64_t end = now_ms() + CALIBRATION_WAIT_MS; now_ms() < end;) {
			if ((r = read_one(st, (int)(end - now_ms()))) < 0) {
				return r;
			}
			if (st->calib_target != query[1] || st->calib_index < 0) {
				continue;
			}
			if (st->calib_index == CALIBRATION_ERROR) {
				return -EIO;
			}
			if (st->calib_index == CALIBRATION_NOT_READY) {
				usleep(CALIBRATION_RETRY_MS * 1000);
				break;
			}
			if (st->calib_index == (int)index) {
				memcpy(out, st->calib_data, PICO4_STATION_CALIBRATION_CHUNK);
				return 0;
			}
			st->calib_index = -1; // a late reply for an earlier chunk
		}
	}
	return -ETIMEDOUT;
}

int
pico4_station_get_calibration(struct pico4_station *st,
                              enum pico4_station_hand hand,
                              uint8_t out[PICO4_STATION_CALIBRATION_CHUNKS][PICO4_STATION_CALIBRATION_CHUNK])
{
	int r = 0;
	pthread_mutex_lock(&st->lock);
	for (unsigned i = 1; i <= PICO4_STATION_CALIBRATION_CHUNKS && r == 0; i++) {
		uint8_t q[PICO4_STATION_FRAME];
		if (!pico4_station_calibration_query(q, hand, i)) {
			r = -EINVAL;
			break;
		}
		r = calibration_chunk(st, q, i, out[i - 1]);
	}
	pthread_mutex_unlock(&st->lock);
	return r;
}

int
pico4_station_pairing(struct pico4_station *st, enum pico4_station_pairing op, enum pico4_station_hand hand)
{
	uint8_t f[PICO4_STATION_FRAME];
	if (!pico4_station_pairing_frame(f, op, hand)) {
		return -EINVAL;
	}
	pthread_mutex_lock(&st->lock);
	st->pair_cmd = -1;
	int r = send_frame(st, f);
	for (int64_t end = now_ms() + PICO4_STATION_PAIRING_ACK_MS; r == 0 && st->pair_cmd != (int)op && now_ms() < end;) {
		int n = read_one(st, 50);
		r = n < 0 ? n : 0;
	}
	if (r == 0) {
		r = st->pair_cmd != (int)op ? -ETIMEDOUT : st->pair_status == 1 ? 0 : -EIO;
	}
	pthread_mutex_unlock(&st->lock);
	return r;
}

static int
station_ops_pairing(void *ctx, enum pico4_station_pairing op, enum pico4_station_hand hand)
{
	return pico4_station_pairing((struct pico4_station *)ctx, op, hand);
}

static void
station_ops_get_controller(void *ctx, enum pico4_station_hand hand, struct pico4_station_controller *out)
{
	pico4_station_get_controller((struct pico4_station *)ctx, hand, out);
}

struct pico4_station_ops
pico4_station_ops(struct pico4_station *st)
{
	struct pico4_station_ops ops = {station_ops_pairing, station_ops_get_controller, st};
	return ops;
}

//! The next word of the text (spaces separate words) into @p word; false at its end or for a long word.
static bool
next_word(const char **text, const char *end, char *word, size_t size)
{
	const char *p = *text;
	while (p < end && *p == ' ') {
		p++;
	}
	size_t n = 0;
	while (p < end && *p != ' ') {
		if (n + 1 >= size) {
			return false;
		}
		word[n++] = *p++;
	}
	word[n] = '\0';
	*text = p;
	return n > 0;
}

bool
pico4_station_parse_request(const char *text, size_t size, struct pico4_station_request *out)
{
	const char *p = text, *end = text + size;
	while (end > p && (end[-1] == '\n' || end[-1] == '\0')) {
		end--;
	}
	for (const char *q = p; q < end; q++) {
		if (*q < 0x20 || *q > 0x7e) {
			return false;
		}
	}
	char verb[8], hand[8], seconds[8], extra[2];
	memset(out, 0, sizeof(*out));
	if (!next_word(&p, end, verb, sizeof(verb))) {
		return false;
	}
	if (strcmp(verb, "status") == 0) {
		out->kind = PICO4_STATION_REQUEST_STATUS;
		return !next_word(&p, end, extra, sizeof(extra)) && p == end;
	}
	if (strcmp(verb, "pair") == 0) {
		out->kind = PICO4_STATION_REQUEST_PAIR;
	} else if (strcmp(verb, "unpair") == 0) {
		out->kind = PICO4_STATION_REQUEST_UNPAIR;
	} else {
		return false;
	}
	if (!next_word(&p, end, hand, sizeof(hand))) {
		return false;
	}
	if (strcmp(hand, "left") == 0) {
		out->hand = PICO4_STATION_HAND_LEFT;
	} else if (strcmp(hand, "right") == 0) {
		out->hand = PICO4_STATION_HAND_RIGHT;
	} else {
		return false;
	}
	if (out->kind == PICO4_STATION_REQUEST_PAIR) {
		out->seconds = PICO4_STATION_PAIR_DEFAULT_S;
		if (next_word(&p, end, seconds, sizeof(seconds))) {
			char *rest;
			unsigned long s = strtoul(seconds, &rest, 10);
			if (*rest != '\0' || seconds[0] < '0' || seconds[0] > '9' || s < 1 || s > PICO4_STATION_PAIR_MAX_S) {
				return false;
			}
			out->seconds = (unsigned)s;
		}
	}
	return !next_word(&p, end, extra, sizeof(extra)) && p == end;
}

struct pico4_station_control
{
	int listen_fd, client_fd;
	struct pico4_station_ops ops;
	struct sockaddr_un addr;
	int64_t accepted_ms;
	//! A pairing in progress: the hand, its connects when it started, when it ends.
	bool pairing;
	enum pico4_station_hand hand;
	uint32_t connects;
	int64_t deadline_ms;
};

static const char *
hand_name(enum pico4_station_hand hand)
{
	return hand == PICO4_STATION_HAND_LEFT ? "left" : "right";
}

int
pico4_station_control_open(struct pico4_station_control **out, const char *path, struct pico4_station_ops ops)
{
	struct pico4_station_control *c = calloc(1, sizeof(*c));
	if (c == NULL) {
		return -ENOMEM;
	}
	c->addr.sun_family = AF_UNIX;
	const char *dir = getenv("XDG_RUNTIME_DIR");
	int n;
	if (path != NULL) {
		n = snprintf(c->addr.sun_path, sizeof(c->addr.sun_path), "%s", path);
	} else if (dir != NULL && dir[0] == '/') {
		n = snprintf(c->addr.sun_path, sizeof(c->addr.sun_path), "%s/%s", dir, PICO4_STATION_CONTROL_NAME);
	} else {
		n = snprintf(c->addr.sun_path, sizeof(c->addr.sun_path), "/run/user/%u/%s", (unsigned)getuid(),
		             PICO4_STATION_CONTROL_NAME);
	}
	if (n < 0 || (size_t)n >= sizeof(c->addr.sun_path)) {
		free(c);
		return -ENAMETOOLONG;
	}
	c->ops = ops;
	c->client_fd = -1;
	c->listen_fd = socket(AF_UNIX, SOCK_SEQPACKET | SOCK_NONBLOCK | SOCK_CLOEXEC, 0);
	if (c->listen_fd < 0) {
		int r = -errno;
		free(c);
		return r;
	}
	// A runtime that ended without closing (killed) leaves its socket behind.
	unlink(c->addr.sun_path);
	if (bind(c->listen_fd, (const struct sockaddr *)&c->addr, sizeof(c->addr)) < 0 ||
	    chmod(c->addr.sun_path, 0600) < 0 || listen(c->listen_fd, 2) < 0) {
		int r = -errno;
		close(c->listen_fd);
		unlink(c->addr.sun_path);
		free(c);
		return r;
	}
	*out = c;
	return 0;
}

static void
hang_up(struct pico4_station_control *c)
{
	if (c->client_fd >= 0) {
		close(c->client_fd);
	}
	c->client_fd = -1;
	c->pairing = false;
}

//! Send @p line to the client and copy it into the log.
static size_t
answer(struct pico4_station_control *c, const char *line, char *log, size_t size)
{
	(void)send(c->client_fd, line, strlen(line), MSG_NOSIGNAL | MSG_DONTWAIT);
	return (size_t)snprintf(log, size, "%s", line);
}

static const char *
result_word(int r, const char *done)
{
	return r == 0 ? done : r == -EIO ? "refused" : r == -ETIMEDOUT ? "noreply" : "error";
}

//! A pairing in progress: has the controller connected, has the time run out, has the client gone?
static size_t
follow_pairing(struct pico4_station_control *c, int64_t now_ms, char *log, size_t size)
{
	char probe[1];
	ssize_t n = recv(c->client_fd, probe, sizeof(probe), MSG_DONTWAIT);
	// A pairing takes no second request: anything but "nothing yet" ends it.
	bool gone = n >= 0 || (errno != EAGAIN && errno != EWOULDBLOCK);
	struct pico4_station_controller info;
	c->ops.get_controller(c->ops.ctx, c->hand, &info);
	char line[96];
	if (info.connected && info.connects != c->connects) {
		snprintf(line, sizeof(line), "paired %s %s", hand_name(c->hand), info.serial);
	} else if (now_ms >= c->deadline_ms) {
		snprintf(line, sizeof(line), "timeout %s", hand_name(c->hand));
	} else if (gone) {
		snprintf(line, sizeof(line), "cancelled %s", hand_name(c->hand));
	} else {
		return 0;
	}
	// Also after a connect: the station must not stay open to other controllers.
	if (c->ops.pairing(c->ops.ctx, PICO4_STATION_PAIR_STOP, c->hand) != 0) {
		size_t used = strlen(line);
		snprintf(line + used, sizeof(line) - used, " (pairing not stopped)");
	}
	size_t r = gone ? (size_t)snprintf(log, size, "%s", line) : answer(c, line, log, size);
	hang_up(c);
	return r;
}

size_t
pico4_station_control_step(struct pico4_station_control *c, int64_t now_ms, char *log, size_t size)
{
	if (c == NULL) {
		return 0;
	}
	if (c->client_fd < 0) {
		c->client_fd = accept4(c->listen_fd, NULL, NULL, SOCK_NONBLOCK | SOCK_CLOEXEC);
		if (c->client_fd < 0) {
			return 0;
		}
		c->accepted_ms = now_ms;
	}
	if (c->pairing) {
		return follow_pairing(c, now_ms, log, size);
	}
	char text[64];
	ssize_t n = recv(c->client_fd, text, sizeof(text), MSG_DONTWAIT);
	if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK) && now_ms - c->accepted_ms < PICO4_STATION_REQUEST_MS) {
		return 0;
	}
	if (n <= 0) {
		hang_up(c);
		return 0;
	}
	struct pico4_station_request req;
	if (!pico4_station_parse_request(text, (size_t)n, &req)) {
		size_t r = answer(c, "error request", log, size);
		hang_up(c);
		return r;
	}
	char line[96];
	if (req.kind == PICO4_STATION_REQUEST_STATUS) {
		struct pico4_station_controller left, right;
		c->ops.get_controller(c->ops.ctx, PICO4_STATION_HAND_LEFT, &left);
		c->ops.get_controller(c->ops.ctx, PICO4_STATION_HAND_RIGHT, &right);
		snprintf(line, sizeof(line), "left %s%s%s\nright %s%s%s", left.connected ? "connected" : "disconnected",
		         left.connected ? " " : "", left.connected ? left.serial : "",
		         right.connected ? "connected" : "disconnected", right.connected ? " " : "",
		         right.connected ? right.serial : "");
	} else if (req.kind == PICO4_STATION_REQUEST_UNPAIR) {
		int r = c->ops.pairing(c->ops.ctx, PICO4_STATION_UNPAIR, req.hand);
		snprintf(line, sizeof(line), "%s %s", result_word(r, "unpaired"), hand_name(req.hand));
	} else {
		struct pico4_station_controller info;
		c->ops.get_controller(c->ops.ctx, req.hand, &info);
		int r = c->ops.pairing(c->ops.ctx, PICO4_STATION_PAIR, req.hand);
		snprintf(line, sizeof(line), "%s %s", result_word(r, "pairing"), hand_name(req.hand));
		if (r == 0) {
			c->pairing = true;
			c->hand = req.hand;
			c->connects = info.connects;
			c->deadline_ms = now_ms + (int64_t)req.seconds * 1000;
			return answer(c, line, log, size);
		}
	}
	size_t r = answer(c, line, log, size);
	hang_up(c);
	// A status changes nothing: clients ask every few seconds (the onboarding), the log keeps pairings only.
	if (req.kind == PICO4_STATION_REQUEST_STATUS) {
		log[0] = '\0';
		return 0;
	}
	return r;
}

void
pico4_station_control_close(struct pico4_station_control *c)
{
	if (c == NULL) {
		return;
	}
	if (c->pairing) {
		c->ops.pairing(c->ops.ctx, PICO4_STATION_PAIR_STOP, c->hand);
	}
	hang_up(c);
	close(c->listen_fd);
	unlink(c->addr.sun_path);
	free(c);
}
