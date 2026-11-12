# SPDX-License-Identifier: Unlicense
"""pico4-lens: only stock's lens frames can be built, and a move follows the stock service."""

import importlib.machinery
import importlib.util
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent
_loader = importlib.machinery.SourceFileLoader("pico4_lens", str(HERE / "pico4-lens"))
_spec = importlib.util.spec_from_loader("pico4_lens", _loader)
assert _spec is not None
lens = importlib.util.module_from_spec(_spec)
_loader.exec_module(lens)


def position_reply(value: int, hall: int = 893) -> bytes:
    """Build the lens controller's position reply: the position and the hall sensor's reading."""
    body = bytes((0x79, value & 255, value >> 8, hall & 255, hall >> 8))
    return body + bytes((lens.xor(body),))


def state_reply(state: int) -> bytes:
    """Build the lens controller's move state reply."""
    return bytes((0x79, state, 0x79 ^ state))


class FakePort:
    def __init__(self, states: list[int], end: int):
        self.states, self.end, self.sent = list(states), end, []

    def __call__(self, tx: bytes, _length: int) -> bytes:
        self.sent.append(tx)
        if tx[1] == lens.GET_MOVE_STATE:
            return state_reply(self.states.pop(0))
        if tx[1] == lens.GET_POSITION:
            return position_reply(self.end)
        return bytes((0x79,))


class FrameTests(unittest.TestCase):
    def test_frames_as_the_stock_service(self):
        self.assertEqual(lens.frame(lens.GET_POSITION).hex(), "a008000000a8")
        self.assertEqual(lens.frame(lens.GET_MOVE_STATE).hex(), "a011000000b1")
        self.assertEqual(lens.frame(lens.REPORT_ENABLE).hex(), "a02200000082")
        # 63.50 mm: A0 07 00 lo hi xor.
        self.assertEqual(lens.frame(lens.MOVE, 6350).hex(), "a00700ce1871")

    def test_nothing_else_can_be_built(self):
        for position in (6199, 7201, 5800, 0, 63.5, None):
            with self.assertRaises(ValueError):
                lens.frame(lens.MOVE, position)
        for command in set(range(256)) - {lens.GET_POSITION, lens.GET_MOVE_STATE, lens.REPORT_ENABLE}:
            with self.assertRaises(ValueError):
                lens.frame(command)
        with self.assertRaises(ValueError):
            lens.frame(lens.REPORT_ENABLE, 1)

    def test_replies(self):
        self.assertEqual(lens.parse_position(position_reply(7195)), 7195)
        self.assertEqual(lens.parse_position(position_reply(893, 7195)), 7195)
        for bad in (position_reply(7195)[:5], b"\x78" + position_reply(7195)[1:], position_reply(100, 200)):
            with self.assertRaises(ValueError):
                lens.parse_position(bad)
        self.assertEqual(lens.parse_state(state_reply(1)), 1)
        with self.assertRaises(ValueError):
            lens.parse_state(bytes((0x79, 1, 0)))


class MoveTests(unittest.TestCase):
    def test_move_sends_stocks_sequence_and_waits(self):
        port = FakePort([0, 1, 1, 0], 6351)
        self.assertEqual(lens.move(port, 6350, sleep=lambda _: None), 6351)
        self.assertEqual(
            [tx[1] for tx in port.sent],
            [
                lens.GET_MOVE_STATE,
                lens.REPORT_ENABLE,
                lens.MOVE,
                lens.GET_MOVE_STATE,
                lens.GET_MOVE_STATE,
                lens.GET_MOVE_STATE,
                lens.GET_POSITION,
            ],
        )

    def test_refused_while_moving(self):
        port = FakePort([1], 6350)
        with self.assertRaises(RuntimeError):
            lens.move(port, 6350, sleep=lambda _: None)
        self.assertEqual(len(port.sent), 1)

    def test_bad_target_sends_nothing(self):
        port = FakePort([0], 6350)
        with self.assertRaises(ValueError):
            lens.move(port, 5000, sleep=lambda _: None)
        self.assertEqual(port.sent, [])

    def test_end_of_range_or_wrong_position_is_an_error(self):
        with self.assertRaises(RuntimeError):
            lens.move(FakePort([0, 3], 6350), 6350, sleep=lambda _: None)
        with self.assertRaises(RuntimeError):
            lens.move(FakePort([0, 0], 6500), 6350, sleep=lambda _: None)

    def test_a_move_that_never_stops_times_out(self):
        clock = iter(range(0, 1000, 10))
        with self.assertRaises(TimeoutError):
            lens.move(FakePort([0] + [1] * 50, 6350), 6350, sleep=lambda _: None, now=lambda: next(clock))


if __name__ == "__main__":
    unittest.main()
