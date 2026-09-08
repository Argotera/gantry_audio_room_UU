#!/usr/bin/env python3
"""Unit tests for oes.py and move.py against a fake serial port. No hardware
and no extra packages needed:

    python3 -m unittest -v

The fake replies like the real board: the reset banner on RTS release, 'X+0'
to RX, a status word to RSTS, silence to set commands. It records every write
so the tests can assert exactly which commands reached the "controller".
"""

from __future__ import annotations

import contextlib
import io
import sys
import time
import types
import unittest


class FakeSerial:
    """Stand-in for serial.Serial with the board's reply habits."""

    def __init__(self, port: str, baud: int, **kwargs: object) -> None:
        self.port = port
        self.timeout = float(kwargs.get("timeout", 0.05))  # type: ignore[arg-type]
        self.writes: list[str] = []
        self._pending = b""
        self._rts = False
        self.dtr = False
        self.moving_polls = 0  # RSTS reports "moving" this many more times
        self.garbage_polls = 0  # RSTS replies with junk this many more times
        self.closed = False

    @property
    def rts(self) -> bool:
        return self._rts

    @rts.setter
    def rts(self, value: bool) -> None:
        if self._rts and not value:  # reset released -> the board boots and prints
            self._pending = b"Version 11.27\r\nJoystick is on\r\n"
        self._rts = value

    def read(self, size: int = 1) -> bytes:
        time.sleep(self.timeout)  # a read with nothing waiting costs one timeout
        out, self._pending = self._pending, b""
        return out

    def write(self, data: bytes) -> None:
        cmd = data.strip().upper().decode()
        self.writes.append(cmd)
        if cmd.startswith("RSTS"):
            if self.garbage_polls > 0:
                self.garbage_polls -= 1
                self._pending = b"???\r\n"
            else:
                self._pending = b"1\r\n" if self.moving_polls > 0 else b"0\r\n"
                self.moving_polls = max(self.moving_polls - 1, 0)
        elif cmd in ("RX", "RY", "RZ"):
            self._pending = cmd[1].encode() + b"+0\r\n"
        elif cmd == "JOFF":
            self._pending = b"Joystick is off\r\n"
        elif cmd.startswith("MOV"):
            self.moving_polls = 3

    def flush(self) -> None:
        pass

    def reset_input_buffer(self) -> None:
        self._pending = b""

    def close(self) -> None:
        self.closed = True


def install_fake_serial() -> None:
    module = types.ModuleType("serial")
    module.Serial = FakeSerial  # type: ignore[attr-defined]
    module.SerialException = type("SerialException", (OSError,), {})  # type: ignore[attr-defined]
    module.EIGHTBITS, module.PARITY_NONE, module.STOPBITS_ONE = 8, "N", 1  # type: ignore[attr-defined]
    sys.modules["serial"] = module


install_fake_serial()
import move  # noqa: E402  (must come after the fake is installed)
import oes  # noqa: E402


class ControllerTestCase(unittest.TestCase):
    def setUp(self) -> None:
        self.ctl = oes.OESController(verbose=False).open()
        self.port: FakeSerial = self.ctl.ser  # type: ignore[assignment]

    def tearDown(self) -> None:
        self.ctl.close()

    def sent_since(self, mark: int) -> list[str]:
        return self.port.writes[mark:]


class ConnectTests(ControllerTestCase):
    def test_banner_and_version_parsed_from_reset(self) -> None:
        self.assertIn("Version 11.27", self.ctl.banner)
        self.assertEqual(self.ctl.version, "11.27")

    def test_close_releases_port(self) -> None:
        self.ctl.close()
        self.assertTrue(self.port.closed)
        self.assertIsNone(self.ctl.ser)


class GuardTests(ControllerTestCase):
    def test_destructive_refused_without_force(self) -> None:
        for cmd in ("RUN", "NEW", "SAVE", "CONT", "run"):
            with self.assertRaises(oes.OESError):
                self.ctl.raw(cmd)
        self.assertNotIn("RUN", self.port.writes)

    def test_destructive_sent_with_force(self) -> None:
        self.ctl.raw("NEW", force=True)
        self.assertEqual(self.port.writes[-1], "NEW")

    def test_home_refused_even_with_force(self) -> None:
        with self.assertRaises(oes.OESError):
            self.ctl.raw("HOMEX", force=True)
        with self.assertRaises(oes.OESError):
            self.ctl.home("Z")
        self.assertFalse(any(w.startswith("HOME") for w in self.port.writes))

    def test_home_planned_when_switches_declared(self) -> None:
        ctl = oes.OESController(verbose=False, home_switches=True).open()
        result = ctl.home("Z")  # still dry run: planned, not sent
        self.assertTrue(result.dry_run)
        self.assertEqual(result.commands, ["MONZ", "HOMEZ"])
        self.assertNotIn("HOMEZ", ctl.ser.writes)  # type: ignore[union-attr]

    def test_actuator_simulated_until_armed(self) -> None:
        mark = len(self.port.writes)
        self.ctl.raw("MONX")
        self.assertEqual(self.sent_since(mark), [])
        self.ctl.arm(confirm=True)
        self.ctl.raw("MONX")
        self.assertEqual(self.sent_since(mark), ["MONX"])

    def test_arm_requires_confirm(self) -> None:
        with self.assertRaises(oes.OESError):
            self.ctl.arm()
        self.assertTrue(self.ctl.dry_run)

    def test_config_and_reads_always_sent(self) -> None:
        mark = len(self.port.writes)
        self.ctl.set_velocity("X", 4000)
        self.ctl.redefine_position("Y", 0)
        self.ctl.report_position("Z")
        self.ctl.stop()
        self.assertEqual(self.sent_since(mark), ["VELX 4000", "SPOSY 0", "RZ", "STOPALL"])

    def test_unknown_axis_refused(self) -> None:
        with self.assertRaises(oes.OESError):
            self.ctl.report_position("W")
        with self.assertRaises(oes.OESError):
            oes.OESController(axes=("X", "Q"))

    def test_out_of_range_values_refused_before_sending(self) -> None:
        mark = len(self.port.writes)
        with self.assertRaises(oes.OESError):
            self.ctl.set_velocity("X", 50)
        with self.assertRaises(oes.OESError):
            self.ctl.set_acceleration("X", 1000)
        self.assertEqual(self.sent_since(mark), [])


class MoveTests(ControllerTestCase):
    def setUp(self) -> None:
        super().setUp()
        self.ctl.arm(confirm=True)

    def test_relative_move_sequence(self) -> None:
        mark = len(self.port.writes)
        result = self.ctl.move_relative("X", 100, vel=4000, acc=40000)
        self.assertFalse(result.dry_run)
        self.assertEqual(
            self.sent_since(mark), ["MONX", "ACCX 40000", "VELX 4000", "POSX 100", "MOVRX"]
        )

    def test_absolute_move_without_vel_acc(self) -> None:
        mark = len(self.port.writes)
        self.ctl.move_absolute("Y", -5)
        self.assertEqual(self.sent_since(mark), ["MONY", "POSY -5", "MOVAY"])

    def test_dry_run_move_sends_nothing(self) -> None:
        self.ctl.disarm()
        mark = len(self.port.writes)
        result = self.ctl.move_relative("Z", 10, vel=1000)
        self.assertTrue(result.dry_run)
        self.assertIsNone(result.responses)
        self.assertEqual(self.sent_since(mark), [])


class WaitStoppedTests(ControllerTestCase):
    def setUp(self) -> None:
        super().setUp()
        self.ctl.arm(confirm=True)

    def test_moving_then_idle_returns_true(self) -> None:
        self.ctl.move_relative("Y", 10)  # the fake now reports moving for 3 polls
        polls = []
        self.assertTrue(
            self.ctl.wait_stopped(
                "Y", timeout=10, on_poll=lambda: polls.append(1) is None and False
            )
        )
        self.assertEqual(self.port.writes.count("RSTSY"), 4)
        self.assertEqual(len(polls), 3)

    def test_idle_from_start_waits_for_grace(self) -> None:
        start = time.monotonic()
        self.assertTrue(self.ctl.wait_stopped("X", timeout=10, grace=0.5))
        self.assertGreater(time.monotonic() - start, 0.5)

    def test_on_poll_can_break_out(self) -> None:
        self.assertFalse(self.ctl.wait_stopped("X", timeout=10, on_poll=lambda: True))

    def test_timeout_returns_false(self) -> None:
        self.port.moving_polls = 10_000
        self.assertFalse(self.ctl.wait_stopped("X", timeout=0.4))

    def test_read_failures_raise_after_limit(self) -> None:
        self.port.garbage_polls = 10_000
        with self.assertRaises(oes.OESError):
            self.ctl.wait_stopped("X", timeout=30, max_read_failures=2)


class ParsingTests(unittest.TestCase):
    def test_parse_signed(self) -> None:
        parse = oes.OESController._parse_signed
        self.assertEqual(parse("X+1000", "X"), 1000)
        self.assertEqual(parse("X-7", "X"), -7)
        self.assertEqual(parse("42\r\nJoystick is off"), 42)
        with self.assertRaises(oes.OESError):
            parse("", "X")
        with self.assertRaises(oes.OESError):
            parse("Joystick is off", "X")

    def test_classify_command(self) -> None:
        expected = {
            "movrx": "actuator",
            "MON X": "actuator",
            "home": "actuator",
            "STOPALL": "safe-stop",
            "moffz": "safe-stop",
            "run": "destructive",
            "SAVE": "destructive",
            "outx": "output",
            "setbit 2": "output",
            "RSTSX": "other",
            "velz 5000": "other",
            "spos 0": "other",
        }
        for cmd, kind in expected.items():
            self.assertEqual(oes.classify_command(cmd), kind, cmd)


class MoveScriptTests(unittest.TestCase):
    def setUp(self) -> None:
        self._quiet = contextlib.redirect_stderr(io.StringIO())  # argparse usage text
        self._quiet.__enter__()

    def tearDown(self) -> None:
        self._quiet.__exit__(None, None, None)

    def test_mm_rounds_to_whole_steps(self) -> None:
        self.assertEqual(move.resolve_steps("X", 1.0, None), 105)
        self.assertEqual(move.resolve_steps("Y", -1.0, None), -211)
        self.assertEqual(move.resolve_steps("Z", None, 5000), 5000)
        self.assertEqual(move.resolve_steps("Z", None, None), 0)

    def test_mm_below_one_step_is_an_error(self) -> None:
        with self.assertRaises(ValueError):
            move.resolve_steps("X", 0.001, None)

    def test_steps_beyond_register_is_an_error(self) -> None:
        with self.assertRaises(ValueError):
            move.resolve_steps("X", None, move.POS_LIMIT + 1)

    def test_axis_is_required_and_validated(self) -> None:
        with self.assertRaises(SystemExit):
            move.parse_args(["--mm", "1"])
        with self.assertRaises(SystemExit):
            move.parse_args(["--axis", "Q", "--mm", "1"])
        args = move.parse_args(["--axis", "z", "--mm", "-2.5"])
        self.assertEqual((args.axis, args.step_count), ("Z", -263))

    def test_velocity_ceiling_needs_force(self) -> None:
        with self.assertRaises(SystemExit):
            move.parse_args(["--axis", "X", "--mm", "1", "--vel", "6000"])
        args = move.parse_args(["--axis", "X", "--mm", "1", "--vel", "6000", "--force"])
        self.assertEqual(args.vel, 6000)

    def test_mm_and_steps_are_exclusive(self) -> None:
        with self.assertRaises(SystemExit):
            move.parse_args(["--axis", "X", "--mm", "1", "--steps", "5"])


if __name__ == "__main__":
    unittest.main()
