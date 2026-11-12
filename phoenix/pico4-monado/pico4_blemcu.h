// Copyright 2026, overpressure contributors.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  PICO 4 controller packets as the sensor DSP's "ble_mcu" sensor reports them.
 *
 * The controller radio (station) forwards the controllers' data to the sensor
 * DSP, which reports each 200-byte radio packet as one ble_mcu event. Layout
 * (from the stock tracking service's parser, SlpiDataParserV2):
 *
 *   0       0x5A
 *   1       format version
 *   4..11   station time, microseconds (little endian)
 *   12      length of the records that follow
 *   13..    records: [length][type][id][time offset][data]
 *
 * The id's low six bits name the device (1 left controller, 2 right, 3-5
 * trackers). Bit 7 adds a time offset before the data (2 bytes, or 4 with bit
 * 6), in microseconds before the packet's station time. Record types: 0 IMU,
 * 1 keys, 2 battery; others belong to trackers and diagnostics.
 *
 * @ingroup drv_pico4
 */

#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define PICO4_BLEMCU_LEFT 1
#define PICO4_BLEMCU_RIGHT 2
#define PICO4_BLEMCU_MAX_IMU 8

//! Accelerometer: 1024 LSB per g (the controllers' 32 g range).
#define PICO4_BLEMCU_ACCEL_LSB_PER_G 1024.0
/*!
 * Gyroscope: 8.192 LSB per deg/s (4000 deg/s range). PICO OS scales gyro and accelerometer
 * together (2000 deg/s and 16 g over 32768) and doubles both for the PICO 4's controllers;
 * the accelerometer above is the doubled scale too.
 */
#define PICO4_BLEMCU_GYRO_LSB_PER_DPS 8.192

//! Key bits (data byte 4).
enum pico4_blemcu_button
{
	PICO4_BLEMCU_BTN_HOME = 1 << 0,
	PICO4_BLEMCU_BTN_APP = 1 << 1,
	PICO4_BLEMCU_BTN_AX = 1 << 2, //!< A on the right controller, X on the left.
	PICO4_BLEMCU_BTN_BY = 1 << 3, //!< B on the right controller, Y on the left.
	PICO4_BLEMCU_BTN_GRIP = 1 << 4,
	PICO4_BLEMCU_BTN_STICK = 1 << 5,
	PICO4_BLEMCU_BTN_TRIGGER = 1 << 7,
};

//! Touch bits (data byte 5).
enum pico4_blemcu_touch
{
	PICO4_BLEMCU_TOUCH_AX = 1 << 0,
	PICO4_BLEMCU_TOUCH_BY = 1 << 1,
	PICO4_BLEMCU_TOUCH_STICK = 1 << 2,
	PICO4_BLEMCU_TOUCH_THUMBREST = 1 << 3,
	PICO4_BLEMCU_TOUCH_TRIGGER = 1 << 4,
};

struct pico4_blemcu_imu
{
	int device;              //!< PICO4_BLEMCU_LEFT or _RIGHT.
	uint32_t offset_us;      //!< Before the packet's station time.
	int16_t gyro[3];         //!< Raw sensor axes, PICO4_BLEMCU_GYRO_LSB_PER_DPS.
	int16_t accel[3];        //!< Raw sensor axes, PICO4_BLEMCU_ACCEL_LSB_PER_G.
	int16_t temperature;     //!< raw / 132.48 + 25 = degrees Celsius.
	uint16_t counter;
};

struct pico4_blemcu_keys
{
	int device;
	uint8_t trigger, grip;   //!< 0-255.
	uint8_t stick_x, stick_y; //!< 0-255, centre 0x80; x grows rightwards, y upwards (y is sent first).
	uint8_t buttons;         //!< enum pico4_blemcu_button bits.
	uint8_t touch;           //!< enum pico4_blemcu_touch bits.
};

struct pico4_blemcu_packet
{
	uint64_t station_time_us;
	int imu_count;
	struct pico4_blemcu_imu imu[PICO4_BLEMCU_MAX_IMU];
	int key_count;
	struct pico4_blemcu_keys keys[2];
	int battery_count;
	struct
	{
		int device;
		uint8_t level;
	} battery[2];
};

/*!
 * A controller's newest IMU sample is this old: the controller has stopped sending (switched
 * off, out of range). PICO OS's tracker starts over when its newest sample is older than an
 * image by 0.2 s.
 */
#define PICO4_BLEMCU_STALE_NS 200000000LL

//! A controller's newest IMU sample time, for order and freshness.
struct pico4_blemcu_order
{
	int64_t newest_ns; //!< 0 before the first sample.
	uint64_t rejected; //!< Duplicates and backward steps.
};

//! Parse one ble_mcu payload; false if it is not a controller-radio packet.
bool
pico4_blemcu_parse(const uint8_t *data, size_t size, struct pico4_blemcu_packet *out);

/*!
 * Drop the IMU samples stamped at or after @p now_ns, as PICO OS's controller service does
 * ("[TS-ERROR]"): a sample is the packet's time @p packet_ns minus its offset. Returns how
 * many were dropped.
 */
int
pico4_blemcu_drop_future(struct pico4_blemcu_packet *p, int64_t packet_ns, int64_t now_ns);

//! True if a sample at @p ts_ns is newer than the newest so far, which it then becomes.
bool
pico4_blemcu_order_accept(struct pico4_blemcu_order *o, int64_t ts_ns);

//! True while the newest sample is less than PICO4_BLEMCU_STALE_NS older than @p at_ns.
bool
pico4_blemcu_order_fresh(const struct pico4_blemcu_order *o, int64_t at_ns);

#ifdef __cplusplus
}
#endif
