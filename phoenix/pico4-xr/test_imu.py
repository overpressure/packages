# SPDX-License-Identifier: Unlicense
"""pico4-imu: the head IMU through the sensor DSP's client service, against a fake service."""

from __future__ import annotations

import contextlib
import importlib.machinery
import importlib.util
import io
import json
import math
import struct
import unittest
from pathlib import Path
from typing import Any
from unittest import mock

PATH = Path(__file__).resolve().parent / "pico4-imu"
_loader = importlib.machinery.SourceFileLoader("pico4_imu", str(PATH))
_spec = importlib.util.spec_from_loader("pico4_imu", _loader)
assert _spec is not None
imu = importlib.util.module_from_spec(_spec)
_loader.exec_module(imu)

ACCEL = (0x1111, 0xA0A0)
GYRO = (0x2222, 0xB0B0)
SERVER = (9, 77)


class FakeDsp:
    """The SNS client service and QRTR name service, as seen through a socket."""

    def __init__(
        self,
        rate_scale: float = 1.0,
        gyro: tuple[float, float, float] = (0.001, -0.002, 0.0005),
        gravity: tuple[float, float, float] = (0.0, 0.0, 9.80665),
    ):
        self.queue = []
        self.enabled = {}
        self.ticks = {ACCEL: 1_000_000, GYRO: 1_000_000}
        self.rate_scale = rate_scale
        self.values = {ACCEL: gravity, GYRO: gyro}
        self.requests = []

    # socket interface
    def getsockname(self) -> tuple[int, int]:
        return (1, 4242)

    def sendto(self, data: bytes, addr: tuple[int, int]) -> None:
        if addr == (1, imu.QRTR_PORT_CTRL):
            cmd, service = struct.unpack_from("<II", data)
            assert cmd == imu.QRTR_TYPE_NEW_LOOKUP
            assert service == imu.SNS_CLIENT_SERVICE
            self.queue.append(struct.pack("<IIIII", imu.QRTR_TYPE_NEW_SERVER, service, 1, *SERVER))
            self.queue.append(struct.pack("<IIIII", imu.QRTR_TYPE_NEW_SERVER, 0, 0, 0, 0))
            return
        assert addr == SERVER
        msg_type, txn, msg_id, tlvs = imu.qmi_parse(data)
        assert msg_type == imu.QMI_REQUEST
        assert msg_id == imu.SSC_MSG_CONTROL
        (length,) = struct.unpack_from("<H", tlvs[0x01])
        request = imu.pb_parse(tlvs[0x01][2 : 2 + length])
        suid = imu.parse_suid(imu.pb_first(request, 1))
        req_id = imu.u32(imu.pb_first(request, 2))
        body = imu.pb_parse(imu.pb_first(request, 4))
        payload = imu.pb_first(body, 2, b"")
        self.requests.append((suid, req_id))
        result = imu.qmi_tlv(0x02, struct.pack("<HH", 0, 0)) + imu.qmi_tlv(0x10, struct.pack("<Q", 5))
        self.queue.append(imu.qmi_message(imu.QMI_RESPONSE, txn, imu.SSC_MSG_CONTROL, result))
        if suid == imu.SUID_SENSOR and req_id == imu.MSG_SUID_REQ:
            data_type = imu.pb_first(imu.pb_parse(payload), 1).decode()
            found = {"accel": [ACCEL], "gyro": [GYRO]}.get(data_type, [])
            ev = imu.pb_bytes(1, data_type.encode()) + b"".join(imu.pb_bytes(2, imu.suid_bytes(s)) for s in found)
            self._indicate(imu.SUID_SENSOR, [(imu.MSG_SUID_EVENT, 0, ev)])
        elif req_id == imu.MSG_ATTR_REQ:
            name = imu.pb_bytes(1, imu.pb_bytes(2, b"icm4x6xx"))
            rates = imu.pb_bytes(1, imu.pb_bytes(3, struct.pack("<f", 500.0)))
            attrs = imu.pb_bytes(1, imu.pb_varint(1, 0) + imu.pb_bytes(2, name)) + imu.pb_bytes(
                1, imu.pb_varint(1, 6) + imu.pb_bytes(2, rates)
            )
            self._indicate(suid, [(imu.MSG_ATTR_EVENT, 0, attrs)])
        elif req_id == imu.MSG_CONFIG:
            rate = struct.unpack("<f", imu.pb_first(imu.pb_parse(payload), 1))[0]
            self.enabled[suid] = rate * self.rate_scale
            self._indicate(suid, [(imu.MSG_PHYSICAL_CONFIG_EVENT, 0, imu.pb_float(1, rate))])
        elif req_id == imu.MSG_DISABLE:
            self.enabled.pop(suid, None)

    def recvfrom(self, _size: int) -> tuple[bytes, tuple[int, int]]:
        if not self.queue:
            self._samples()
        return self.queue.pop(0), SERVER

    def ready(self) -> bool:
        return bool(self.queue or self.enabled)

    # helpers
    def _indicate(self, suid: tuple[int, int], events: list[tuple[int, int, bytes]], *, large: bool = False) -> None:
        msg = imu.pb_bytes(1, imu.suid_bytes(suid))
        for msg_id, ts, payload in events:
            msg += imu.pb_bytes(2, imu.pb_fixed32(1, msg_id) + imu.pb_fixed64(2, ts) + imu.pb_bytes(3, payload))
        tlvs = imu.qmi_tlv(0x01, struct.pack("<Q", 5)) + imu.qmi_tlv(0x02, struct.pack("<H", len(msg)) + msg)
        self.queue.append(
            imu.qmi_message(imu.QMI_INDICATION, 0, imu.SSC_IND_LARGE if large else imu.SSC_IND_SMALL, tlvs)
        )

    def _samples(self) -> None:
        for suid, rate in self.enabled.items():
            events = []
            for _i in range(10):
                self.ticks[suid] += int(imu.QTIMER_HZ / rate)
                x, y, z = self.values[suid]
                noise = 0.001 * math.sin(self.ticks[suid])
                data = b"".join(imu.pb_float(1, v + noise) for v in (x, y, z)) + imu.pb_varint(2, 3)
                events.append((imu.MSG_SENSOR_EVENT, self.ticks[suid], data))
            self._indicate(suid, events, large=True)


def run(dsp: FakeDsp, argv: list[str]) -> tuple[int, dict[str, Any]]:
    """Run pico4-imu against the fake service; return its exit code and its JSON output."""

    def fake_select(
        rlist: list[object], _w: list[object], _x: list[object], _timeout: float | None = None
    ) -> tuple[list[object], list[object], list[object]]:
        return (rlist, [], []) if dsp.ready() else ([], [], [])

    clock = iter(range(10**9))
    out = io.StringIO()
    with (
        mock.patch.object(imu.socket, "socket", return_value=dsp),
        mock.patch.object(imu.select, "select", fake_select),
        mock.patch.object(imu.time, "monotonic", side_effect=lambda: next(clock) * 0.0005),
        contextlib.redirect_stdout(out),
    ):
        code = imu.main(argv)
    return code, json.loads(out.getvalue())


class ProtobufTests(unittest.TestCase):
    def test_wire_format(self):
        self.assertEqual(imu.pb_varint(1, 300), b"\x08\xac\x02")
        self.assertEqual(imu.pb_fixed32(2, 513), b"\x15\x01\x02\x00\x00")
        self.assertEqual(imu.pb_bytes(4, b"ab"), b"\x22\x02ab")
        self.assertEqual(imu.pb_varint(1, -1)[1:], b"\xff" * 9 + b"\x01")

    def test_repeated_floats_packed_and_unpacked(self):
        unpacked = imu.pb_float(1, 1.5) + imu.pb_float(1, -2.0) + imu.pb_varint(2, 3)
        packed = imu.pb_bytes(1, struct.pack("<2f", 1.5, -2.0))
        for data in (unpacked, packed):
            self.assertEqual(imu.pb_floats(imu.pb_parse(data), 1), [1.5, -2.0])

    def test_truncated_input_is_rejected(self):
        with self.assertRaises(ValueError):
            imu.pb_parse(b"\x22\x05ab")

    def test_client_request_layout(self):
        msg = imu.pb_parse(imu.client_request(ACCEL, imu.MSG_CONFIG, imu.pb_float(1, 500.0)))
        self.assertEqual([f for f, _w, _v in msg], [1, 2, 3, 4])
        self.assertEqual(imu.parse_suid(imu.pb_first(msg, 1)), ACCEL)
        self.assertEqual(imu.u32(imu.pb_first(msg, 2)), 513)
        self.assertEqual(imu.pb_parse(imu.pb_first(msg, 3)), [(1, 0, 1), (2, 0, 0)])
        body = imu.pb_parse(imu.pb_first(msg, 4))
        self.assertEqual(imu.pb_first(body, 2), imu.pb_float(1, 500.0))

    def test_qmi_control_frame(self):
        frame = imu.ssc_control(7, b"xyz")
        self.assertEqual(frame[:7], struct.pack("<BHHH", 0, 7, 0x20, len(frame) - 7))
        _t, txn, msg_id, tlvs = imu.qmi_parse(frame)
        self.assertEqual((txn, msg_id), (7, 0x20))
        self.assertEqual(tlvs[0x01], b"\x03\x00xyz")
        self.assertEqual(tlvs[0x10], b"\x00")


class StreamTests(unittest.TestCase):
    def test_list_finds_sensors_with_attributes(self):
        dsp = FakeDsp()
        code, out = run(dsp, ["list", "accel", "gyro", "mag"])
        self.assertEqual(code, 0)
        self.assertEqual(out["accel"][0]["name"], "icm4x6xx")
        self.assertEqual(out["accel"][0]["rates"], 500.0)
        self.assertEqual(out["mag"], [])

    def test_still_headset_passes(self):
        dsp = FakeDsp()
        code, out = run(dsp, ["stream", "--rate", "500", "--seconds", "1", "--check", "--orientation"])
        self.assertEqual(code, 0, out["failures"])
        self.assertEqual(out["status"], "pass")
        accel = out["streams"]["accel"]
        self.assertAlmostEqual(accel["rate_hz"], 500, delta=5)
        self.assertAlmostEqual(accel["norm_mean"], 9.80665, places=2)
        self.assertEqual(accel["config"], {"sample_rate_hz": 500.0})
        self.assertEqual(accel["non_increasing_timestamps"], 0)
        self.assertEqual(accel["gaps"], 0)
        self.assertLess(abs(out["orientation"]["roll_deg"]), 1)
        self.assertLess(abs(out["orientation"]["pitch_deg"]), 1)
        # Both streams were switched off again.
        self.assertIn((ACCEL, imu.MSG_DISABLE), dsp.requests)
        self.assertIn((GYRO, imu.MSG_DISABLE), dsp.requests)
        self.assertEqual(dsp.enabled, {})

    def test_orientation_follows_gravity(self):
        g = 9.80665
        tilt = math.radians(30)
        dsp = FakeDsp(gravity=(0.0, g * math.sin(tilt), g * math.cos(tilt)), gyro=(0.0, 0.0, 0.0))
        _code, out = run(dsp, ["stream", "--seconds", "1", "--orientation"])
        self.assertEqual(out["status"], "collected")
        self.assertAlmostEqual(out["orientation"]["roll_deg"], 30, delta=0.5)
        self.assertAlmostEqual(out["orientation"]["accel_roll_deg"], 30, delta=0.5)

    def test_turn_about_gravity_is_measured(self):
        # Level headset turned by 90 degrees about the vertical in 2 s at 512 Hz.
        rate, seconds = 512, 2.0
        accel, gyro = imu.Stream("accel"), imu.Stream("gyro")
        n = int(rate * seconds)
        for i in range(n + 1):
            ts = 1_000_000 + i * imu.QTIMER_HZ // rate
            accel.add(ts, [0.0, 0.0, 9.80665])
            gyro.add(ts, [0.0, 0.0, math.radians(90) / seconds])
        out = imu.orientation_from(accel, gyro)
        self.assertAlmostEqual(out["heading_change_deg"], 90, delta=0.5)
        self.assertAlmostEqual(out["rotation_deg"], 90, delta=0.5)
        self.assertLess(abs(out["roll_deg"]), 0.1)

    def test_wrong_rate_and_moving_gyro_fail(self):
        dsp = FakeDsp(rate_scale=0.5, gyro=(0.2, 0.0, 0.0))
        code, out = run(dsp, ["stream", "--seconds", "1", "--check"])
        self.assertEqual(code, 1)
        self.assertTrue(any("rate" in f for f in out["failures"]))
        self.assertTrue(any(f.startswith("gyro: mean") for f in out["failures"]))

    def test_missing_service_is_reported(self):
        dsp = FakeDsp()
        dsp.sendto = mock.Mock(return_value=None)
        code, out = run(dsp, ["stream"])
        self.assertEqual(code, 1)
        self.assertIn("SNS client service", out["error"])


if __name__ == "__main__":
    unittest.main()
