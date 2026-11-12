# SPDX-License-Identifier: Unlicense
"""pico4-station: the station's SPI frames, the controller queries, mainline spidev transfers and the device lock."""

from __future__ import annotations

import ctypes
import importlib.machinery
import importlib.util
import struct
import tempfile
import types
import unittest
from pathlib import Path
from unittest import mock

PATH = Path(__file__).resolve().parent / "pico4-station"
_loader = importlib.machinery.SourceFileLoader("pico4_station_tool", str(PATH))
_spec = importlib.util.spec_from_loader("pico4_station_tool", _loader)
assert _spec is not None
station = importlib.util.module_from_spec(_spec)
_loader.exec_module(station)


def reply(target: int, command: int, body: bytes) -> bytes:
    """Build a frame the station sends: header, target, command, body, CRC."""
    buf = bytearray(40)
    buf[0], buf[1], buf[2] = 0xBB, target, command
    buf[3 : 3 + len(body)] = body
    buf[39] = station.crc8(bytes(buf[:39]))
    return bytes(buf)


class FakeStation(station.Station):
    """Answers the controller queries from a script instead of /dev/spidev0.0."""

    def __init__(self, answers: dict[tuple[int, int], list[bytes]]):
        self.answers = answers  # (command, payload) -> list of replies, one per request
        self.sent, self.pending, self.log, self.seq, self.announced = [], [], [], 0, False

    def send(self, data: bytes) -> None:
        self.sent.append(data)
        if data[2] != station.CMD_ACK:
            queue = self.answers.get((data[2], data[3]), [])
            if queue:
                self.pending.append(queue.pop(0))

    def read(self, timeout: float) -> bytes | None:  # noqa: ARG002  # Station.read's signature
        if not self.pending:
            return None
        buf = self.pending.pop(0)
        self.send(station.frame(station.CMD_ACK, buf[37:39]))
        return buf


class StationToolTests(unittest.TestCase):
    def test_slow_advertising_frame_as_the_stock_sleep(self):
        buf = station.frame(station.CMD_SLOW_ADV, bytes([0, 0]))
        self.assertEqual(buf[:5], bytes([0xAA, 0x01, 0x0D, 0, 0]))
        for payload in (b"", bytes([4, 0]), bytes([0, 1]), bytes([0, 0, 0])):
            with self.assertRaises(ValueError):
                station.frame(station.CMD_SLOW_ADV, payload)

    def test_slow_advertising_reads_the_status_and_tries_twice(self):
        confirmed = reply(1, station.CMD_SLOW_ADV, bytes([0, 1]))
        self.assertTrue(FakeStation({(station.CMD_SLOW_ADV, 0): [confirmed]}).controllers_slow_adv())
        refused = reply(1, station.CMD_SLOW_ADV, bytes([0, 0]))
        self.assertFalse(FakeStation({(station.CMD_SLOW_ADV, 0): [refused]}).controllers_slow_adv())
        silent = FakeStation({})
        self.assertFalse(silent.controllers_slow_adv())
        self.assertEqual(sum(1 for s in silent.sent if s[2] == station.CMD_SLOW_ADV), 2)

    def test_controller_queries_exactly_as_pico_os(self):
        info = station.controller_query_frame("left", station.CMD_CONTROLLER_INFO)
        self.assertEqual(info[:4], bytes([0xAA, 0x02, 0x03, 0x00]))
        self.assertEqual(info[3:39], bytes(36))
        self.assertEqual(info[39], station.crc8(info[:39]))
        calib = station.controller_query_frame("right", station.CMD_CONTROLLER_CALIBRATION, bytes([12]))
        self.assertEqual(calib[:4], bytes([0xAA, 0x03, 0x02, 12]))
        self.assertEqual(calib[4:39], bytes(35))

    def test_anything_else_to_a_controller_is_refused(self):
        refused = [
            ("left", station.CMD_CONTROLLER_INFO, b"\x01"),
            ("left", station.CMD_CONTROLLER_CALIBRATION, b""),
            ("left", station.CMD_CONTROLLER_CALIBRATION, b"\x00"),
            ("left", station.CMD_CONTROLLER_CALIBRATION, b"\x0d"),
            ("left", station.CMD_CONTROLLER_CALIBRATION, b"\x01\x00"),
            ("right", 0x04, b""),
            ("station", station.CMD_CONTROLLER_INFO, b""),
        ]
        for hand, command, payload in refused:
            with self.assertRaises(ValueError, msg=(hand, command, payload)):
                station.controller_query_frame(hand, command, payload)

    def test_info_reply_fields(self):
        body = bytearray(36)
        body[0:6] = b"3.26.0"
        body[6] = 0x21  # byte 9: connect type 2, status 1
        body[7:9] = bytes([0x01, 0x02])
        body[10:27] = b"TESTSERIAL0000001"
        body[27:33] = bytes([1, 2, 3, 4, 5, 6])
        fake = FakeStation({(station.CMD_CONTROLLER_INFO, 0): [reply(0x03, 0x03, bytes(body))]})
        info = fake.controller_info("right")
        self.assertEqual(info["status"], 1)
        self.assertEqual(info["connect_type"], 2)
        self.assertEqual(info["software"], "3.26.0")
        self.assertEqual(info["hardware"], "0102")
        self.assertEqual(info["serial"], "TESTSERIAL0000001")
        self.assertEqual(info["address"], "01:02:03:04:05:06")
        self.assertEqual(fake.sent[0][:3], bytes([0xAA, 0x03, 0x03]))
        self.assertEqual(fake.sent[1][2], station.CMD_ACK)  # replies are acknowledged

    def test_calibration_retries_a_chunk_that_is_not_ready(self):
        answers = {}
        for index in range(1, 13):
            data = struct.pack("<8f", *[index + k / 10 for k in range(8)])
            answers[(station.CMD_CONTROLLER_CALIBRATION, index)] = [reply(0x02, 0x02, bytes([index]) + data)]
        answers[(station.CMD_CONTROLLER_CALIBRATION, 2)].insert(0, reply(0x02, 0x02, bytes([0xFF])))
        fake = FakeStation(answers)
        chunks = fake.controller_calibration("left")
        self.assertEqual(sorted(chunks), list(range(1, 13)))
        record = station.calibration_floats(chunks)
        self.assertEqual(len(record["imu"]), 24)
        self.assertEqual(len(record["leds"]), 64)
        self.assertEqual(len(record["imu_ir"]), 8)
        self.assertAlmostEqual(record["imu"][8], 2.0, places=5)
        requests = [f[3] for f in fake.sent if f[2] == station.CMD_CONTROLLER_CALIBRATION]
        self.assertEqual(requests, [1, 2, 2, *list(range(3, 13))])

    def test_late_replies_for_an_earlier_chunk_are_skipped(self):
        answers = {}
        for index in range(1, 13):
            data = bytes([index]) * 32
            answers[(station.CMD_CONTROLLER_CALIBRATION, index)] = [reply(0x02, 0x02, bytes([index]) + data)]
        # The request for chunk 4 is answered first by a late reply for chunk 3.
        late = reply(0x02, 0x02, bytes([3]) + bytes([3]) * 32)
        answers[(station.CMD_CONTROLLER_CALIBRATION, 4)].insert(0, late)
        fake = FakeStation(answers)
        real_send = fake.send

        def send(data: bytes) -> None:
            real_send(data)
            if data[2] == station.CMD_CONTROLLER_CALIBRATION and data[3] == 4:
                fake.pending.extend(answers[(station.CMD_CONTROLLER_CALIBRATION, 4)])

        fake.send = send
        chunks = fake.controller_calibration("left")
        self.assertEqual(chunks[4], bytes([4]) * 32)
        requests = [f[3] for f in fake.sent if f[2] == station.CMD_CONTROLLER_CALIBRATION]
        self.assertEqual(requests, list(range(1, 13)))  # waited instead of asking again

    def test_calibration_error_stops(self):
        fake = FakeStation({(station.CMD_CONTROLLER_CALIBRATION, 1): [reply(0x02, 0x02, bytes([0xEE]))]})
        with self.assertRaises(OSError):
            fake.controller_calibration("left")


def station_frame(kind: int, body: bytes) -> bytes:
    """Build one of the station's own frames: A5, kind, body from byte 2, CRC."""
    buf = bytearray(40)
    buf[0], buf[1] = 0xA5, kind
    buf[2 : 2 + len(body)] = body
    buf[39] = station.crc8(bytes(buf[:39]))
    return bytes(buf)


def connect(hand: int, status: int = 1) -> bytes:
    """Build the station's report that a controller connected: hand, software, status, serial, address."""
    body = bytearray(34)
    body[0] = hand
    body[1:7] = b"3.26.0"
    body[7] = 0x20 | status
    body[11:28] = b"TESTSERIAL0000002"
    body[28:34] = bytes([6, 5, 4, 3, 2, 1])
    return station_frame(station.STATION_CONNECT, bytes(body))


class PairingTests(unittest.TestCase):
    """Pairing as the stock EnterPairing, StopPairing and UnbondController: AA 01 02/08/03 with one mask byte."""

    def test_pairing_frames_as_the_stock_service(self):
        golden = {
            (station.CMD_ENTER_PAIRING, 1): ("aa010201", 0x32),
            (station.CMD_ENTER_PAIRING, 2): ("aa010202", 0x10),
            (station.CMD_UNBOND, 1): ("aa010301", 0x47),
            (station.CMD_STOP_PAIRING, 2): ("aa010802", 0x79),
        }
        for (command, mask), (head, crc) in golden.items():
            buf = station.frame(command, bytes([mask]))
            self.assertEqual(buf[:4].hex(), head)
            self.assertEqual(buf[4:39], bytes(35))
            self.assertEqual(buf[39], crc)

    def test_only_one_controller_mask_is_allowed(self):
        refused = [
            (station.CMD_ENTER_PAIRING, b""),
            (station.CMD_ENTER_PAIRING, b"\x00"),
            (station.CMD_ENTER_PAIRING, b"\x03"),  # both at once: not what PICO OS sends
            (station.CMD_ENTER_PAIRING, b"\x04"),  # the trackers' bits
            (station.CMD_ENTER_PAIRING, b"\x01\x00"),
            (station.CMD_UNBOND, bytes([1, 6, 5, 4, 3, 2, 1])),  # USB pairing: 03 with a radio address
            (station.CMD_STOP_PAIRING, b"\x10"),
            (0x09, bytes([1, 0, 0, 0])),  # the controllers' pair mode
            (0x0A, b""),  # the station updater's command
        ]
        for command, payload in refused:
            with self.assertRaises(ValueError, msg=(command, payload)):
                station.frame(command, payload)

    def fake(self, entered: int, after: list[bytes]) -> FakeStation:
        fake = FakeStation(
            {
                (station.CMD_ENTER_PAIRING, 1): [reply(1, station.CMD_ENTER_PAIRING, bytes([0, entered]))],
                (station.CMD_STOP_PAIRING, 1): [reply(1, station.CMD_STOP_PAIRING, bytes([0, 1]))],
            }
        )
        real_send = fake.send

        def send(data: bytes) -> None:
            real_send(data)
            if data[2] == station.CMD_ENTER_PAIRING:
                fake.pending.extend(after)  # what the station reports while it listens

        fake.send = send
        return fake

    def test_pair_waits_for_that_controller_then_stops(self):
        other = connect(2)
        invalid = connect(1, status=2)
        fake = self.fake(1, [other, invalid, connect(1)])
        out = fake.pair("left", 5.0)
        self.assertTrue(out["entered"])
        self.assertEqual(out["connected"]["serial"], "TESTSERIAL0000002")
        self.assertEqual(out["connected"]["address"], "06:05:04:03:02:01")
        self.assertTrue(out["stopped"])
        commands = [f[:4].hex() for f in fake.sent if f[2] != station.CMD_ACK]
        self.assertEqual(commands, ["aa010201", "aa010801"])

    def test_pair_stops_after_the_time_without_a_controller(self):
        fake = self.fake(1, [connect(2)])
        out = fake.pair("left", 0.05)
        self.assertIsNone(out["connected"])
        self.assertTrue(out["stopped"])

    def test_a_refused_pairing_waits_for_nothing(self):
        fake = self.fake(0, [connect(1)])
        out = fake.pair("left", 5.0)
        self.assertFalse(out["entered"])
        self.assertIsNone(out["connected"])
        self.assertNotIn("stopped", out)

    def test_unpair_reads_the_station_answer(self):
        done = reply(1, station.CMD_UNBOND, bytes([0, 1]))
        fake = FakeStation({(station.CMD_UNBOND, 2): [done]})
        self.assertTrue(fake.unpair("right"))
        self.assertEqual(fake.sent[0][:4].hex(), "aa010302")
        self.assertIsNone(FakeStation({}).unpair("left"))


class DriverSocketTests(unittest.TestCase):
    """While a session runs, pair, unpair and status go to the driver's pairing socket (pico4_station.h)."""

    class Sock:
        """A SOCK_SEQPACKET client that answers from a script."""

        def __init__(self, answers: list[bytes]):
            self.answers, self.sent, self.connected, self.timeout = answers, [], None, None

        def settimeout(self, seconds: float) -> None:
            self.timeout = seconds

        def connect(self, path: str) -> None:
            self.connected = path

        def send(self, data: bytes) -> int:
            self.sent.append(data)
            return len(data)

        def recv(self, size: int) -> bytes:  # noqa: ARG002  # socket.recv's signature
            return self.answers.pop(0) if self.answers else b""

        def close(self) -> None:
            pass

    def fake_socket(self, answers: list[bytes]):
        sock = self.Sock(answers)
        module = types.SimpleNamespace(AF_UNIX=1, SOCK_SEQPACKET=5, socket=lambda *_args: sock)
        return sock, mock.patch.object(station, "socket", module)

    def test_requests_and_waits(self):
        parser = station._parser()  # noqa: SLF001  # the tool's own command line
        cases = {
            ("pair", "left"): ("pair left 60", 65.0),
            ("pair", "right", "--seconds", "90"): ("pair right 90", 95.0),
            ("unpair", "left"): ("unpair left", station.CONTROL_ANSWER_S),
            ("status",): ("status", station.CONTROL_ANSWER_S),
        }
        for argv, expected in cases.items():
            self.assertEqual(station.control_command(parser.parse_args(list(argv))), expected)

    def test_answers_until_the_driver_hangs_up(self):
        sock, patch = self.fake_socket([b"pairing left", b"paired left TESTSERIAL0000001"])
        with patch:
            answers = station.control_request("pair left 60", 65.0, "/run/user/1000/pico4-station")
        self.assertEqual(answers, ["pairing left", "paired left TESTSERIAL0000001"])
        self.assertEqual(sock.sent, [b"pair left 60"])
        self.assertEqual((sock.connected, sock.timeout), ("/run/user/1000/pico4-station", 65.0))

    def test_a_busy_station_goes_through_the_driver(self):
        busy = station.StationBusyError("in use")
        with (
            mock.patch.object(station, "Station", side_effect=busy),
            mock.patch.object(station, "control_request", return_value=["unpaired right"]) as request,
            mock.patch("builtins.print") as printed,
        ):
            self.assertEqual(station.main(["unpair", "right"]), 0)
            request.assert_called_once_with("unpair right", station.CONTROL_ANSWER_S, None)
            self.assertIn('"through": "driver"', printed.call_args[0][0])
            # Status never opens the station; other commands do not go through the driver.
            self.assertEqual(station.main(["status"]), 0)
            self.assertEqual(station.Station.call_count, 1)
            self.assertEqual(station.main(["version"]), 1)

    def test_the_socket_path(self):
        with mock.patch.dict(station.os.environ, {"PICO4_STATION_SOCKET": "/run/test/pico4-station"}):
            self.assertEqual(station.control_path(), "/run/test/pico4-station")


class PlainSpidevTests(unittest.TestCase):
    """A mainline kernel: full-duplex transfers instead of the vendor spidev's reads and writes."""

    def station(self, replies: list[bytes | None]):
        st = station.Station.__new__(station.Station)
        st.spidev, st.queue, st.sent, st.seq, st.log, st.announced = True, [], [], 0, [], False

        def transfer(data: bytes) -> None:
            # Each transfer brings the station's next frame back, or nothing (None).
            st.sent.append(data)
            if replies:
                frame = replies.pop(0)
                if frame is not None:
                    st.queue.append(frame)

        st._transfer = transfer  # noqa: SLF001  # the fake replaces the private transfer
        return st

    def test_reads_send_zeros_until_a_frame_comes_back(self):
        frame = reply(1, station.CMD_VERSION, bytes([2, 0x60]))
        st = self.station([None, None, frame])
        self.assertEqual(st._next(1.0), frame)  # noqa: SLF001  # tests the private read
        self.assertEqual(st.sent, [bytes(40)] * 3)

    def test_commands_go_out_as_transfers(self):
        st = self.station([])
        st.send(station.frame(station.CMD_VERSION, b"", 1))
        self.assertEqual(len(st.sent), 1)
        self.assertEqual(st.sent[0][0], station.HEADER_SEND)

    def test_a_repeated_frame_is_no_new_one(self):
        # The station returns its last frame on every transfer until it has another.
        st = station.Station.__new__(station.Station)
        st.fd, st.spidev, st.queue, st.last = 3, True, [], None
        st._tx, st._rx = ctypes.create_string_buffer(40), ctypes.create_string_buffer(40)  # noqa: SLF001  # the transfer's buffers
        frames = [
            reply(1, station.CMD_VERSION, bytes([1])),
            reply(1, station.CMD_VERSION, bytes([1])),
            reply(1, station.CMD_VERSION, bytes([2])),
        ]

        def ioctl(*_: object) -> None:
            ctypes.memmove(st._rx, frames.pop(0), 40)  # noqa: SLF001  # the transfer's receive buffer

        with mock.patch.object(station.fcntl, "ioctl", ioctl, create=True):
            for _ in range(3):
                st._transfer(bytes(40))  # noqa: SLF001  # tests the private transfer
        self.assertEqual([f[3] for f in st.queue], [1, 2])

    def test_nothing_within_the_timeout(self):
        st = self.station([])
        self.assertIsNone(st._next(0.01))  # noqa: SLF001  # tests the private read


class StationLockTests(unittest.TestCase):
    """One owner of the station: a second runtime must not reset it under the first."""

    def test_second_owner_is_refused(self):
        with tempfile.NamedTemporaryFile() as f:
            first = station.Station(f.name)
            try:
                with self.assertRaisesRegex(OSError, "in use"):
                    station.Station(f.name)
            finally:
                first.close()
            station.Station(f.name).close()  # free again once the owner closed it


if __name__ == "__main__":
    unittest.main()
