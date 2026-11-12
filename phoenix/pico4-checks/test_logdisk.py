# SPDX-License-Identifier: Unlicense
"""The installed system's log drive: a journal larger than the drive keeps its newest whole lines."""

import subprocess
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent
SCRIPT = HERE / "userspace-logdisk"
NOTE = "-- the journal's older lines do not fit on the drive"


def tail(text: str, room: int) -> str:
    """Run the drive's journal step on TEXT with ROOM bytes."""
    return subprocess.run(  # noqa: S603  # bash from PATH runs the script under test
        ["bash", str(SCRIPT), "--tail", str(room)],  # noqa: S607  # bash from PATH
        input=text,
        capture_output=True,
        text=True,
        timeout=10,
        check=True,
    ).stdout


class LogdiskTests(unittest.TestCase):
    def test_a_journal_that_fits_stays_whole(self) -> None:
        journal = "".join(f"[{i:6d}.000000] line {i}\n" for i in range(100))
        self.assertEqual(tail(journal, len(journal)), journal)

    def test_a_larger_journal_keeps_its_newest_whole_lines(self) -> None:
        journal = "".join(f"[{i:6d}.000000] line {i}\n" for i in range(10000))
        room = 4000
        out = tail(journal, room)
        lines = out.splitlines()
        self.assertTrue(lines[0].startswith(NOTE))
        self.assertTrue(journal.endswith("\n".join(lines[1:]) + "\n"))
        # Whole lines only, and no more than the room and the note.
        self.assertRegex(lines[1], r"^\[ *\d+\.000000\] line \d+$")
        self.assertLessEqual(len(out), room + len(lines[0]) + 1)


if __name__ == "__main__":
    unittest.main()
