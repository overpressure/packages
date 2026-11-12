// Copyright 2026, overpressure contributors.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Tracking-camera frame sync through the PICO 4 controller radio ("station").
 *
 * The station, an nRF52832 on /dev/spidev0.0, pulses the OV7251s' FSIN line;
 * the sensors run in FSIN slave mode and send nothing without it. Frames are
 * 40 bytes: header (AA to the station, BB replies, A5 notifications), target,
 * command, payload, and a CRC-8 (Dallas/Maxim) over bytes 0..38. Each spidev
 * transfer is full duplex: a frame goes out and the station's next one comes back.
 *
 * All calls may come from different threads (one internal lock).
 *
 * Safety: the station and controller firmware can be updated over this link.
 * This module builds only the frames in pico4_station_frame()'s allowlist
 * (acknowledge, camera frame rate, camera init/stream/exposure), the
 * vibration frame to a controller (pico4_station_vibrate()), the read-only
 * calibration query (pico4_station_calibration_query()), the controller pairing
 * commands with one controller's mask (pico4_station_pairing_frame()) and the supply's switch; it
 * has no way to send firmware, bootloader or flash commands and never touches
 * the boot pin.
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

#define PICO4_STATION_DEVICE "/dev/spidev0.0"
#define PICO4_STATION_FRAME 40

/*!
 * Commands this module may build. Byte 1 of a frame names the target: the station
 * (1) or a controller (2 left, 3 right), and a command byte means different things
 * for each; to the station 01 reads the firmware version and 07 switches the IMU
 * stream. The motor command is therefore allowed only towards a controller.
 */
enum pico4_station_cmd
{
	PICO4_STATION_CMD_CAMERA = 0x04,     //!< Payload [op, ...]: 01 init, 02 stream.
	PICO4_STATION_CMD_CAMERA_FPS = 0x19, //!< Answer to the power-on announcement.
	PICO4_STATION_CMD_SLOW_ADV = 0x0D,   //!< Payload 00 00: the controllers advertise slowly.
	PICO4_STATION_CMD_ACK = 0xFA,        //!< Payload: the acknowledged frame's bytes 37..38.
	//! To a controller only: [play, amplitude, u16 duration ms].
	PICO4_STATION_CMD_MOTOR = 0x01,
	//! To a controller only, read-only: [chunk index 1-12] of its calibration record.
	PICO4_STATION_CMD_CALIBRATION = 0x02,
};

//! CRC-8, reflected polynomial 0x8C, initial value 0.
uint8_t
pico4_station_crc8(const uint8_t *data, size_t size);

/*!
 * Build a request into @p out (40 bytes). Returns false, building nothing, for
 * any command or payload outside the allowlist.
 */
bool
pico4_station_frame(uint8_t out[PICO4_STATION_FRAME], uint8_t cmd, const uint8_t *payload, size_t size);

//! A frame from the station with a valid header and CRC.
bool
pico4_station_valid(const uint8_t frame[PICO4_STATION_FRAME]);

/*!
 * Controllers as PICO OS's station service numbers them (controller_type_t); its
 * frames target 2 for the left and 3 for the right one.
 */
enum pico4_station_hand
{
	PICO4_STATION_HAND_LEFT = 1,
	PICO4_STATION_HAND_RIGHT = 2,
};

//! PICO OS's default vibration strength, ctrl_motor_vibration_amplitude (StationService::SetMotor 0x72000).
#define PICO4_STATION_MOTOR_SCALE_PERCENT 75

/*!
 * Build a vibration request for one controller into @p out (40 bytes), laid out as
 * PICO OS's stationservice does (libstationopticsservice.so, StationService::SetMotor
 * 0x71eb4; the level clamp is StationClient's, libstationclient.pxr.so 0x374e4):
 *
 * - @p level (0-255, larger values count as 255) is PICO OS's motor level; 0 stops the
 *   motor and zeroes the whole payload.
 * - @p scale_percent (0-100) is the station service's amplitude setting; the amplitude
 *   byte is level × scale / 100, at most 255.
 * - @p duration_ms goes out as a u16 (larger values are clamped).
 *
 * The station sends nothing back. Returns false, building nothing, for a
 * hand other than left or right or a scale above 100.
 */
bool
pico4_station_vibrate(uint8_t out[PICO4_STATION_FRAME],
                      enum pico4_station_hand hand,
                      unsigned level,
                      unsigned scale_percent,
                      unsigned duration_ms);

/*!
 * PICO OS's mapping of an OpenXR haptic vibration onto pico4_station_vibrate()
 * (libpxrruntime.so controller set_output 0xa9d20/0xaa4c8, libpxrinputmanager.so
 * PhyController::SetPhyControllerVibrate 0x28d20): the duration in whole ms
 * (XR_MIN_HAPTIC_DURATION, -1, becomes 500 ms), at least 10 ms; level = amplitude ×
 * 255, truncated. A duration of 0 ms or an amplitude of 0 stops (level 0, 0 ms). The
 * frequency is not used: PICO OS sends the plain motor command. Unlike PICO OS, the
 * amplitude is clamped to 0-1 and the duration to 65535 ms.
 */
void
pico4_station_haptic_map(float amplitude, int64_t duration_ns, unsigned *out_level, unsigned *out_duration_ms);

//! The calibration record's chunks for the 20-LED layout: 1-3 IMU, 4-11 LEDs, 12 imu_IR.
#define PICO4_STATION_CALIBRATION_CHUNKS 12
#define PICO4_STATION_CALIBRATION_CHUNK 32
#define PICO4_STATION_SERIAL_LEN 17

/*!
 * Build the query for chunk @p index (1-12) of a controller's calibration record, as PICO OS's
 * station service asks for it on every connect (CalibrationData::GetCalibrationFromStation
 * 0x9eeb4): command 02 to the controller, the index as the only payload byte. The reply (BB,
 * target, 02) echoes the index in byte 3 (0xFF: not ready yet, 0xEE: error) and carries 32
 * bytes of the record in bytes 4-35. False, building nothing, for another hand or index.
 */
bool
pico4_station_calibration_query(uint8_t out[PICO4_STATION_FRAME], enum pico4_station_hand hand, unsigned index);

/*!
 * Controller pairing, as PICO OS's station service sends it (StationService::EnterPairing,
 * StopPairing and UnbondController, libstationopticsservice.so): to the station, one byte, the
 * controller's mask (1 left, 2 right, the hand's number). The station answers BB 01 with the
 * command and byte 4 1 for done. The trackers' mask bits (4 and up) and the USB pairing (03 with a
 * radio address) are never built.
 */
enum pico4_station_pairing
{
	PICO4_STATION_PAIR = 0x02,      //!< Enter pairing: the controller pairs while its Trigger and Home are held 6 s.
	PICO4_STATION_UNPAIR = 0x03,    //!< Forget the controller; it connects again only after pairing.
	PICO4_STATION_PAIR_STOP = 0x08, //!< Stop pairing.
};

//! How long a pairing command waits for the station's answer (the stock service: at least 300 ms).
#define PICO4_STATION_PAIRING_ACK_MS 500

//! Build a pairing command for one controller into @p out; false, building nothing, for another hand.
bool
pico4_station_pairing_frame(uint8_t out[PICO4_STATION_FRAME], enum pico4_station_pairing op, enum pico4_station_hand hand);

//! A controller's connection as the station reports it.
struct pico4_station_controller
{
	bool connected;
	//! Connections so far: the station repeats each connect notification, which counts once.
	uint32_t connects;
	char software[7];
	char serial[PICO4_STATION_SERIAL_LEN + 1]; //!< Picks PICO OS's per-controller files.
};

/*!
 * A controller's connect (A5 02) or disconnect (A5 03) notification: byte 2 is the hand. A connect
 * carries the software version in bytes 3-8, the status in byte 9 (low four bits: 1 connected,
 * 2 connected but invalid), the hardware in 10-11, the serial in 13-29 and the radio address in
 * 30-35, the layout of PICO OS's connect data (StationService::MCUCallBackDataProcess 0x6cc70).
 * Returns 1 for a valid connect (@p info filled: connected, software, serial), 0 for a
 * disconnect, -1 for anything else.
 */
int
pico4_station_parse_connection(const uint8_t f[PICO4_STATION_FRAME],
                               enum pico4_station_hand *hand,
                               struct pico4_station_controller *info);

struct pico4_station;

//! Opens the station's device (NULL: the headset's) and locks it; -EBUSY if another process has it.
int
pico4_station_open(struct pico4_station **out, const char *device);

void
pico4_station_close(struct pico4_station *st);

/*!
 * The cameras' frame rate the next announcement is answered with: 50 Hz (the stock default)
 * or 60 Hz (PICO OS's other rate); exposure commands then carry that rate's sync value. At
 * 60 Hz an exposure plus the 9 ms readout must fit 16.7 ms, about 400 lines.
 */
void
pico4_station_set_camera_hz(struct pico4_station *st, unsigned hz);

/*!
 * Power-cycle the station and answer its announcement with the camera frame
 * rate (50 Hz unless set otherwise), after which it accepts camera commands. Controllers reconnect
 * on their own afterwards. Returns 0 or a negative errno (-ETIMEDOUT if no
 * announcement came).
 */
int
pico4_station_reset(struct pico4_station *st);

/*!
 * Put the station to sleep as PICO OS does: the controllers are told to advertise slowly, which
 * leaves them reachable, then its power goes off. The next pico4_station_reset() brings it back
 * and the controllers reconnect without a button press. Returns 0, -ETIMEDOUT if the station did
 * not confirm the controllers' mode (the power is off all the same), or a negative errno.
 */
int
pico4_station_sleep(struct pico4_station *st);

/*!
 * Read frames for up to @p timeout_ms, acknowledging them as the stock service
 * does. Returns the number of valid frames or a negative errno.
 */
int
pico4_station_poll(struct pico4_station *st, int timeout_ms);

/*!
 * Start FSIN: camera init, then stream on for all cameras. Call once all
 * sensors stream (pico4_cam_start()); the station initialises them over its own
 * I2C and fails if they are powered down. Returns 0, -EIO if the station
 * refused, or another negative errno.
 */
int
pico4_station_camera_start(struct pico4_station *st);

/*!
 * Exposure of all cameras in sensor lines (about 19.3 us each; with the 9 ms
 * readout it must fit the 20 ms frame period, so at most about 570) and gain
 * (16 = 1x, up to 248). In FSIN slave mode the station sets the exposure; the
 * sensors' own registers are ignored.
 */
int
pico4_station_camera_set_exposure(struct pico4_station *st, unsigned exposure_lines, unsigned gain);

//! One exposure phase's setting: sensor lines and gain (16 = 1x).
struct pico4_station_exposure
{
	unsigned lines, gain;
};

//! PICO OS's limits for controller-LED exposures: at most 10 lines, gain below 33.
#define PICO4_STATION_LED_MAX_LINES 10
#define PICO4_STATION_LED_MAX_GAIN 32

/*!
 * The u16 PICO OS puts in bytes 13-14 of every CameraSetPara at a 50 Hz camera rate (160 at
 * other rates; its tracking service's gFPSInfo). With 0 there instead, the controller-LED
 * exposures come out about 27 times brighter (saturated samples per frame) than with no
 * command at all, and neighbouring LEDs merge into one blob; with 52 they stay as without one.
 */
#define PICO4_STATION_PARA_SYNC_50HZ 52
//! The same value at a 60 Hz camera rate (PICO OS's gFPSInfo at other rates).
#define PICO4_STATION_PARA_SYNC_OTHER 160

/*!
 * CameraSetPara's 28-byte payload (u16 lines, u8 gain per camera): the scene phase's setting
 * for cameras 0-3, in camera order, in bytes 1-12, PICO4_STATION_PARA_SYNC_50HZ in bytes 13-14, the event index
 * in byte 15 (PICO OS counts it 0-9, one per command), a second set in bytes 16-27. By the
 * frames' metadata the first set drives the scene phase; the second changes neither phase on
 * this station (the LED phase stays at 4 lines, gain 26, whatever it holds). Here it carries
 * @p leds, clamped to PICO OS's LED limits.
 */
void
pico4_station_camera_para_each(uint8_t out[28],
                               const struct pico4_station_exposure scene[4],
                               struct pico4_station_exposure leds,
                               unsigned index);

/*!
 * The scene phase's exposure per camera (camera order 0-3), as PICO OS's auto exposure sends
 * it; the second set as pico4_station_camera_set_exposure fills it.
 */
int
pico4_station_camera_set_each(struct pico4_station *st, const struct pico4_station_exposure scene[4]);

/*!
 * The tracking cameras' module (vendor) ids, as PICO OS's station service reads them from the
 * modules' OTP when it has none saved: camera op 05 sub-op 01, then 02, each answered by
 * BB 01 04 05 with the ids of cameras 0-3 in bytes 4-7 (a zero keeps the value before). Call
 * with the cameras started; exposure commands wait meanwhile. 0 and @p out (zeros for ids the
 * station did not give), or a negative errno.
 */
int
pico4_station_camera_otp(struct pico4_station *st, uint8_t out[4]);

//! The ids of an OTP reply (BB 01 04 05 v0-v3) into @p ids, keeping a byte where the reply has 0.
bool
pico4_station_parse_otp(const uint8_t f[PICO4_STATION_FRAME], uint8_t ids[4]);

//! Stop FSIN for all cameras.
int
pico4_station_camera_stop(struct pico4_station *st);

/*!
 * Vibrate a controller at @p level (0-255, 0 stops) for @p duration_ms with PICO OS's default
 * strength: one plain motor frame, as PICO OS sends per request. The controller ends the
 * vibration itself; the station answers nothing.
 */
int
pico4_station_motor(struct pico4_station *st, enum pico4_station_hand hand, unsigned level, unsigned duration_ms);

//! A controller's connection state from the notifications read so far.
void
pico4_station_get_controller(struct pico4_station *st,
                             enum pico4_station_hand hand,
                             struct pico4_station_controller *out);

/*!
 * Fetch a controller's calibration record, chunk by chunk as PICO OS does: up to 30 tries per
 * chunk with 100 ms for the reply, 50 ms after a "not ready". Late replies for another chunk
 * are skipped. Holds the station for the whole fetch (normally well under a second). Returns 0,
 * -EIO on the controller's error reply, -ETIMEDOUT, or another negative errno.
 */
int
pico4_station_get_calibration(struct pico4_station *st,
                              enum pico4_station_hand hand,
                              uint8_t out[PICO4_STATION_CALIBRATION_CHUNKS][PICO4_STATION_CALIBRATION_CHUNK]);

/*!
 * Send a pairing command for one controller and wait up to PICO4_STATION_PAIRING_ACK_MS for the
 * station's answer. Returns 0 when it is done, -EIO when the station refused, -ETIMEDOUT without
 * an answer, or another negative errno.
 */
int
pico4_station_pairing(struct pico4_station *st, enum pico4_station_pairing op, enum pico4_station_hand hand);

/*!
 * Pairing while a runtime owns the station (the station's lock keeps pico4-station out): a
 * SOCK_SEQPACKET socket, $XDG_RUNTIME_DIR/pico4-station by default, which only the session's user
 * and root reach. One request per connection, as text:
 *
 * - `pair left|right [SECONDS]`: enter pairing for 1-600 s (60 unless given); answers
 *   `pairing HAND`, then, once pairing has stopped, `paired HAND SERIAL` or `timeout HAND`. A
 *   client that hangs up meanwhile stops the pairing.
 * - `unpair left|right`: answers `unpaired HAND`.
 * - `status`: answers `left connected SERIAL` or `left disconnected`, a newline, the same for right.
 *
 * A command the station refused answers `refused HAND`, one it did not answer `noreply HAND`, a
 * request that is none of these `error request`. A stop pairing that failed adds
 * ` (pairing not stopped)`.
 */
#define PICO4_STATION_CONTROL_NAME "pico4-station"
#define PICO4_STATION_PAIR_DEFAULT_S 60
#define PICO4_STATION_PAIR_MAX_S 600
//! How long an accepted connection may take to send its request.
#define PICO4_STATION_REQUEST_MS 2000

enum pico4_station_request_kind
{
	PICO4_STATION_REQUEST_PAIR,
	PICO4_STATION_REQUEST_UNPAIR,
	PICO4_STATION_REQUEST_STATUS,
};

struct pico4_station_request
{
	enum pico4_station_request_kind kind;
	enum pico4_station_hand hand; //!< Pair and unpair.
	unsigned seconds;             //!< Pair.
};

//! Parse one request (@p size bytes, no terminator needed); false for anything else.
bool
pico4_station_parse_request(const char *text, size_t size, struct pico4_station_request *out);

//! What the control socket needs from a station; the tests put fakes in.
struct pico4_station_ops
{
	int (*pairing)(void *ctx, enum pico4_station_pairing op, enum pico4_station_hand hand);
	void (*get_controller)(void *ctx, enum pico4_station_hand hand, struct pico4_station_controller *out);
	void *ctx;
};

//! The ops of a station opened with pico4_station_open().
struct pico4_station_ops
pico4_station_ops(struct pico4_station *st);

struct pico4_station_control;

/*!
 * Listen on @p path (NULL: $XDG_RUNTIME_DIR/pico4-station, else /run/user/UID/pico4-station),
 * replacing a socket a runtime before left there. Returns 0 or a negative errno.
 */
int
pico4_station_control_open(struct pico4_station_control **out, const char *path, struct pico4_station_ops ops);

//! Stop listening and remove the socket; a pairing still running is stopped.
void
pico4_station_control_close(struct pico4_station_control *c);

/*!
 * Serve the socket without waiting for clients (only a station command waits, up to
 * PICO4_STATION_PAIRING_ACK_MS): take a connection and its request, answer it, follow a pairing
 * to its end. Call it every 100 ms or so; @p now_ms is CLOCK_MONOTONIC in ms. Writes what
 * happened (the line sent, or `cancelled HAND`) into @p log and returns its length, 0 when
 * nothing happened or only a status was answered.
 */
size_t
pico4_station_control_step(struct pico4_station_control *c, int64_t now_ms, char *log, size_t size);

#ifdef __cplusplus
}
#endif
