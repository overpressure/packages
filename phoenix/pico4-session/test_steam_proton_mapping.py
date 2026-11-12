# SPDX-License-Identifier: Unlicense
"""steam-proton-mapping: games Valve runs with Proton Experimental get Proton stable."""

import importlib.machinery
import importlib.util
import tempfile
import unittest
from pathlib import Path

PATH = Path(__file__).resolve().parent / "steam-proton-mapping"
_loader = importlib.machinery.SourceFileLoader("steam_proton_mapping", str(PATH))
_spec = importlib.util.spec_from_loader("steam_proton_mapping", _loader)
assert _spec is not None
tool = importlib.util.module_from_spec(_spec)
_loader.exec_module(tool)

# The client's own lines (compat_log.txt, 2026-10-28).
LOG = """[2026-10-28 13:12:18] Recording non-user mapping for 450390 at priority 85 to tool proton-experimental-arm64
[2026-10-28 13:12:18] Mapping AppID 450390 to tool "proton-experimental-arm64" with priority 85
[2026-10-28 13:12:18] Recording non-user mapping for 620980 at priority 85 to tool proton-stable
[2026-10-28 13:12:18] Mapping AppID 620980 to tool "proton-stable" with priority 85
[2026-10-28 13:18:55] StartSession: appID 450390 session 0123456789abcdef
"""

# config.vdf as the client writes it: tabs, quoted keys and values.
CONFIG = """"InstallConfigStore"
{
\t"Software"
\t{
\t\t"Valve"
\t\t{
\t\t\t"Steam"
\t\t\t{
\t\t\t\t"AutoUpdateWindowEnabled"\t\t"0"
\t\t\t\t"ShaderCacheManager"
\t\t\t\t{
\t\t\t\t\t"HasCurrentBucket"\t\t"1"
\t\t\t\t}
\t\t\t\t"Path"\t\t"a \\"quoted\\" {brace}"
\t\t\t}
\t\t}
\t}
}
"""

MAPPED = """\t\t\t\t"CompatToolMapping"
\t\t\t\t{
\t\t\t\t\t"2663530"
\t\t\t\t\t{
\t\t\t\t\t\t"name"\t\t"proton-experimental-arm64"
\t\t\t\t\t\t"config"\t\t""
\t\t\t\t\t\t"priority"\t\t"250"
\t\t\t\t\t}
\t\t\t\t}
"""


class ProtonMappingTests(unittest.TestCase):
    def setUp(self):
        self.dir = tempfile.TemporaryDirectory()
        self.config = Path(self.dir.name) / "config.vdf"
        self.log = Path(self.dir.name) / "compat_log.txt"
        with self.log.open("w", encoding="utf-8") as f:
            f.write(LOG)

    def tearDown(self):
        self.dir.cleanup()

    def write_config(self, text: str) -> None:
        with self.config.open("w", encoding="utf-8", newline="") as f:
            f.write(text)

    def run_tool(self) -> str:
        tool.main(["steam-proton-mapping", str(self.config), str(self.log)])
        with self.config.open(encoding="utf-8", newline="") as f:
            return f.read()

    def test_only_experimental_recommendations(self):
        self.assertEqual(tool.recommended([str(self.log)]), {"450390": tool.EXPERIMENTAL, "620980": "proton-stable"})
        # The latest recommendation counts: a later line for the same game replaces the first.
        with self.log.open("a", encoding="utf-8") as f:
            f.write(
                "[2026-10-29 10:00:00] Recording non-user mapping for 450390 at priority 85 to tool proton-stable\n"
            )
        self.assertEqual(tool.recommended([str(self.log)])["450390"], "proton-stable")

    def test_new_mapping_block_and_the_rest_kept(self):
        self.write_config(CONFIG)
        text = self.run_tool()
        steam_open = '\t\t\t"Steam"\n\t\t\t{\n'
        self.assertIn(steam_open + '\t\t\t\t"CompatToolMapping"\n\t\t\t\t{\n\t\t\t\t\t"450390"\n', text)
        self.assertIn(
            '\t\t\t\t\t\t"name"\t\t"proton-stable"\n\t\t\t\t\t\t"config"\t\t""\n\t\t\t\t\t\t"priority"\t\t"250"\n', text
        )
        self.assertNotIn('"620980"', text)
        # Everything else byte for byte: removing the block gives the original back.
        block_end = text.index("\n\t\t\t\t}\n", text.index('"CompatToolMapping"')) + len("\n\t\t\t\t}\n")
        self.assertEqual(text[: text.index(steam_open) + len(steam_open)] + text[block_end:], CONFIG)
        tool.parse(text)  # still well formed

    def test_existing_block_gets_the_game_and_the_users_choice_stays(self):
        steam_open = '\t\t\t"Steam"\n\t\t\t{\n'
        self.write_config(CONFIG.replace(steam_open, steam_open + MAPPED))
        with self.log.open("a", encoding="utf-8") as f:
            f.write(
                "[2026-10-28 13:12:18] Recording non-user mapping for 2663530 at priority 85 to tool "
                "proton-experimental-arm64\n"
            )
        text = self.run_tool()
        self.assertEqual(text.count('"CompatToolMapping"'), 1)
        self.assertIn('\t\t\t\t\t"450390"\n\t\t\t\t\t{\n\t\t\t\t\t\t"name"\t\t"proton-stable"', text)
        # The user's own mapping (here to Experimental) is not touched.
        self.assertIn('"2663530"\n\t\t\t\t\t{\n\t\t\t\t\t\t"name"\t\t"proton-experimental-arm64"', text)
        self.assertLess(text.index('"2663530"'), text.index('"450390"'))
        tool.parse(text)
        # A second run changes nothing.
        before = text
        self.assertEqual(self.run_tool(), before)

    def test_a_file_that_does_not_parse_is_left_alone(self):
        broken = CONFIG.replace("\t\t\t}\n\t\t}\n", "\t\t}\n", 1)
        self.write_config(broken)
        self.assertEqual(self.run_tool(), broken)
        with self.assertRaises(ValueError):
            tool.parse('"a"\n{\n"b" x\n}\n')


if __name__ == "__main__":
    unittest.main()
