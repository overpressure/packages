# SPDX-License-Identifier: Unlicense
"""pico4-audiocal: looking up PICO OS's audio calibration in its ACDB files (synthetic data here)."""

import importlib.machinery
import importlib.util
import struct
import tempfile
import unittest
from pathlib import Path
from types import ModuleType

TOOL = Path(__file__).resolve().parent / "pico4-audiocal"


def load_tool() -> ModuleType:
    """Load pico4-audiocal, a program without a suffix, as a module."""
    loader = importlib.machinery.SourceFileLoader("pico4_audiocal", str(TOOL))
    spec = importlib.util.spec_from_loader(loader.name, loader)
    assert spec is not None
    module = importlib.util.module_from_spec(spec)
    loader.exec_module(module)
    return module


audiocal = load_tool()


def table(rows: list[list[int]]) -> bytes:
    """Pack a u32 table: the row count, then the rows."""
    words = [len(rows)] + [w for row in rows for w in row]
    return struct.pack(f"<{len(words)}I", *words)


class Pool:
    """DATAPOOL records {u32 length; data}, addressed by the offset of their length."""

    def __init__(self):
        self.data = b""

    def add(self, payload: bytes) -> int:
        off = len(self.data)
        self.data += struct.pack("<I", len(payload)) + payload
        return off


def database(kind: str, chunks: dict[str, bytes]) -> bytes:
    """Build an ACDB file of this kind from its named chunks."""
    body = b"".join(
        name.encode("latin1").ljust(8, b"\0") + struct.pack("<I", len(data)) + data for name, data in chunks.items()
    )
    return b"QCMSNDDB" + bytes(8) + kind.encode().ljust(8, b"\0") + struct.pack("<II", len(body), len(body)) + body


def u32(*values: int) -> bytes:
    """Pack little-endian u32 words."""
    return struct.pack(f"<{len(values)}I", *values)


def write_fake_acdb(directory: Path) -> None:
    """Write a global file and one device file holding both paths' devices, laid out as PICO OS's."""
    pool = Pool()
    custom = pool.add(b"TOPOLOGIES-BLOB!")
    instance = pool.add(u32(1))
    asm = pool.add(u32(0x11130, 0x1000FFFF, 0x11132, 0x10000018))
    (directory / "Global_cal.acdb").write_bytes(
        database(
            "GCDB",
            {
                "GPROPLUT": table([[0x132AC, custom], [0x1327F, instance], [0x113AE, asm]]),
                "DATAPOOL": pool.data,
            },
        )
    )

    pool = Pool()
    rows = []
    for dev, app, topo, afe in ((15, 0x11130, 0x11000003, 0x112FC), (4, 0x11132, 0x10010315, 0x112FB)):
        rows.append([dev, 0x113AD, pool.add(u32(0x99999, 1, app, topo))])
        rows.append([dev, 0x13150, pool.add(u32(afe))])
    # Audproc of the speaker: two parameters, one of 3 bytes (padded to 4); of the mic: one.
    p1, p2, p3 = pool.add(b"\x01\x02\x03"), pool.add(u32(7, 8)), pool.add(u32(9))
    vol = pool.add(u32(5))
    ids = u32(2, 0x10C2A, 1, 0x10C2B, 0x10C35, 0, 0x10C36) + u32(1, 0x10719, 0, 0x10911)
    offsets = u32(2, p1, p2) + u32(1, p3)
    (directory / "Device_cal.acdb").write_bytes(
        database(
            "AVDB",
            {
                "DPROPLUT": table(rows),
                "AGIILUT0": table([[15, 48000, 0x11130, 0, 0], [4, 48000, 0x11132, 28, 12]]),
                "AGIICDFT": ids,
                "AGIICDOT": offsets,
                "AVIILUT0": table([[15, 0x11130, 0, 0, 0]]),
                "AVIICDFT": u32(1, 0x1031F, 0, 0x1031C),
                "AVIICDOT": u32(1, vol),
                "DATAPOOL": pool.data,
            },
        )
    )


class AudioCalLookupTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.dir = Path(self.tmp.name)
        write_fake_acdb(self.dir)

    def tearDown(self):
        self.tmp.cleanup()

    def test_topologies_per_path(self):
        cal = audiocal.lookup(str(self.dir))
        self.assertEqual(cal["custom_topologies"], b"TOPOLOGIES-BLOB!")
        speaker, mic = cal["paths"]
        self.assertEqual(
            (speaker["adm_topology"], speaker["afe_topology"], speaker["asm_topology"]),
            (0x11000003, 0x112FC, 0x1000FFFF),
        )
        self.assertEqual((mic["adm_topology"], mic["afe_topology"]), (0x10010315, 0x112FB))

    def test_parameters_are_packed_with_instance_headers(self):
        speaker, mic = audiocal.lookup(str(self.dir))["paths"]
        expected = (
            struct.pack("<IHHII", 0x10C2A, 1, 0, 0x10C2B, 4)
            + b"\x01\x02\x03\x00"
            + struct.pack("<IHHII", 0x10C35, 0, 0, 0x10C36, 8)
            + u32(7, 8)
        )
        self.assertEqual(speaker["audproc"], expected)
        self.assertEqual(speaker["audvol"], struct.pack("<IHHII", 0x1031F, 0, 0, 0x1031C, 4) + u32(5))
        self.assertEqual(mic["audproc"], struct.pack("<IHHII", 0x10719, 0, 0, 0x10911, 4) + u32(9))
        # Volume tables are for playback only.
        self.assertIsNone(mic["audvol"])

    def test_mainline_file_layout(self):
        # What q6calibration-pico4.c parses: header, topologies, then per path port, ADM path,
        # device and port topologies and the processing blocks followed by the volume table, each padded to 4.
        cal = audiocal.lookup(str(self.dir))
        data = audiocal.mainline_file(cal)
        self.assertEqual(data[:4], b"P4AC")
        version, topo_size, npaths = struct.unpack_from("<III", data, 4)
        self.assertEqual((version, topo_size, npaths), (2, 16, 2))
        self.assertEqual(data[16:32], b"TOPOLOGIES-BLOB!")
        off, paths = 32, []
        for _ in range(npaths):
            port, path, topology, afe, size = struct.unpack_from("<HHIII", data, off)
            params = data[off + 16 : off + 16 + size]
            paths.append((port, path, topology, afe, params))
            off += 16 + (size + 3) // 4 * 4
        self.assertEqual(off, len(data))
        speaker, mic = cal["paths"]
        self.assertEqual(
            paths[0], (0xB000, 1, 0x11000003, speaker["afe_topology"], speaker["audproc"] + speaker["audvol"])
        )
        self.assertEqual(paths[1], (0xB037, 2, 0x10010315, mic["afe_topology"], mic["audproc"]))
        self.assertTrue(audiocal.MAINLINE_FILE.endswith("/firmware/qcom/sm8250/pico4/audio-calibration.bin"))

    def test_missing_device_is_an_error(self):
        (self.dir / "Device_cal.acdb").unlink()
        with self.assertRaises(ValueError):
            audiocal.lookup(str(self.dir))

    def test_paths_match_pico_os(self):
        # audio_platform_info.xml and audio_io_policy.conf: speaker 15/deep buffer, handset mic 4/record.
        by_name = {p["name"]: p for p in audiocal.PATHS}
        self.assertEqual((by_name["speaker"]["acdb_id"], by_name["speaker"]["app_type"]), (15, 0x11130))
        self.assertEqual((by_name["handset-mic"]["acdb_id"], by_name["handset-mic"]["app_type"]), (4, 0x11132))
        self.assertEqual(by_name["speaker"]["backend"], "WSA_CDC_DMA_RX_0")
        self.assertEqual(by_name["handset-mic"]["backend"], "TX_CDC_DMA_TX_3")


if __name__ == "__main__":
    unittest.main()
