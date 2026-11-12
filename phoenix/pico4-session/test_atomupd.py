# SPDX-License-Identifier: Unlicense
"""The atomupd manager: the answers to the queries Valve's branch and update tools make."""

import subprocess
import unittest
from pathlib import Path

MANAGER = Path(__file__).resolve().parent / "atomupd-manager"


def manager(*args: str) -> subprocess.CompletedProcess[str]:
    """Run the recipe's atomupd-manager under bash."""
    return subprocess.run(  # noqa: S603  # the recipe's script under bash, no shell
        ["bash", str(MANAGER), *args],  # noqa: S607  # bash from PATH
        capture_output=True,
        text=True,
        check=False,
    )


class AtomupdManagerTests(unittest.TestCase):
    """The queries Valve's steamos-select-branch and steamos-update make."""

    def test_queries_describe_an_up_to_date_stable_system(self):
        for command, answer in (
            # steamos-select-branch -c maps "stable" to the client's "rel".
            ("tracked-branch", "stable"),
            ("list-branches", "stable"),
            # steamos-update exits 7 ("No update available") on exactly this line.
            ("check", "No update available"),
            # "successful" would make the client ask for a reboot.
            ("get-update-status", "idle"),
        ):
            r = manager(command)
            self.assertEqual((r.returncode, r.stdout), (0, answer + "\n"), command)

    def test_changes_are_refused(self):
        for command in ("switch-branch", "update", "cancel", "pause", "resume"):
            r = manager(command, "beta")
            self.assertEqual((r.returncode, r.stdout), (1, ""), command)
            self.assertIn("installing a new image", r.stderr)
        self.assertEqual(manager().returncode, 2)


if __name__ == "__main__":
    unittest.main()
