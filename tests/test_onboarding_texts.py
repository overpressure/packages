# SPDX-License-Identifier: Unlicense
"""pico4-monado's first-run setup panel: every text it shows exists in every language it offers."""

from __future__ import annotations

import re
import unittest
from pathlib import Path

RECIPE = Path(__file__).resolve().parent.parent / "phoenix/pico4-monado"
STRINGS = RECIPE / "pico4-onboarding.strings"


def sections(text: str) -> dict[str, dict[str, str]]:
    """Read the strings file as {language: {key: value}}."""
    out: dict[str, dict[str, str]] = {}
    current: dict[str, str] | None = None
    for line in text.splitlines():
        if not line.strip() or line.startswith("#"):
            continue
        m = re.fullmatch(r"\[([a-z]+)\]", line)
        if m:
            current = out.setdefault(m.group(1), {})
            continue
        key, sep, value = line.partition(" = ")
        assert current is not None, line
        assert sep, line
        assert key not in current, f"{key} twice"
        current[key] = value
    return out


class Texts(unittest.TestCase):
    def setUp(self) -> None:
        self.languages = sections(STRINGS.read_text(encoding="utf-8"))

    def test_every_key_the_panel_uses_is_in_english_and_every_text_is_used(self) -> None:
        source = (RECIPE / "pico4-onboarding.cpp").read_text(encoding="utf-8")
        english = self.languages["english"]
        self.assertFalse(set(re.findall(r'our\("([a-z_]+)"\)', source)) - set(english), "keys without English text")
        # Each text is named in the source, also where a condition picks one of two.
        self.assertFalse({key for key in english if f'"{key}"' not in source}, "texts nothing uses")

    def test_every_language_the_panel_offers_has_every_text_with_its_placeholders(self) -> None:
        header = (RECIPE / "pico4_setup.h").read_text(encoding="utf-8")
        table = header[header.index("pico4_setup_languages[") :]
        offered = re.findall(r'\{"([a-z]+)", "[^"]+"\},', table[: table.index("};")])
        self.assertEqual(sorted(self.languages), sorted(offered))
        english = self.languages["english"]
        for language, texts in self.languages.items():
            self.assertEqual(sorted(texts), sorted(english), language)
            for key, value in texts.items():
                self.assertEqual(value.count("%s"), english[key].count("%s"), f"{language} {key}")
                self.assertTrue(value.strip(), f"{language} {key}")


if __name__ == "__main__":
    unittest.main()
