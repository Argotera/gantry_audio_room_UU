#!/usr/bin/env python3
"""
oes.py - Driver for the OES (Optimal Engineering Systems) Allegra/ICAD-series
motion controller driving this 3-axis Cartesian gantry over RS-232.

Verified hardware/protocol (firmware v11.27):
  * Prolific PL2303 USB-RS232 adapter -> /dev/ttyUSB0
  * 19200 baud, 8 data bits, no parity, 1 stop bit, NO flow control
  * Commands are ASCII, terminated with CR ('\\r'); responses end with CR/LF
  * DB9 pin 7 = hardware RESET, driven by the host RTS line.
      Init handshake: RTS SET (>=10 ms) then CLEAR, held CLEAR the whole session.
      (Holding RTS asserted keeps the board in reset -> total silence.)
  * Allow ~100 ms processing time per command.
  * Straight-through cable into the port labeled RS-232 (not COMMAND/INPUT/etc).

SAFETY MODEL
  Every command passes through raw(), which is the single guard:
  * RUN/NEW/SAVE/CONT are refused unless force=True. They run or destroy the
    program stored in the controller's non-volatile memory.
  * HOME is refused unless the controller is constructed with
    home_switches=True. This machine has no home switches.
  * Commands that energize or move a motor (MON, MOVA, MOVR, JOG, HOME) and
    output-port commands are only logged, not sent, while dry_run is True.
    The driver starts in dry run; call arm(confirm=True) to transmit them.
  * Reads, VEL/ACC/POS/SPOS, STOP and MOFF are always sent.

Move model (per OES reference):
  MON x                enable the axis driver
  ACC x <steps/s^2>    set acceleration   (40,000 .. 40,000,000)
  VEL x <steps/s>      set slew speed     (200 .. 200,000)
  POS x <value>        set target (for MOVA) / distance (for MOVR), +-2147483647
  MOVA x  |  MOVR x    begin absolute / relative move  -> "X Abs. Move" then "Done X"

Progress messages go to the `oes` logger. Configure `logging` in the
application to see them; with verbose=False they are emitted at DEBUG.
"""

from __future__ import annotations

import contextlib
import logging
import time
from dataclasses import dataclass
from typing import Callable

try:
    import serial
except ImportError as e:
    raise SystemExit("pyserial is required: pip install pyserial") from e

log = logging.getLogger("oes")


class OESError(Exception):
    """Protocol / usage error talking to the controller."""


@dataclass
class MoveResult:
    """What a guarded motion call did. In dry run nothing was sent and
    `responses` is None."""

    dry_run: bool
    description: str
    commands: list[str]
    responses: list[str] | None


# Command prefixes that can energize or move a motor: simulated until arm().
_ACTUATOR_PREFIXES = ("MON", "MOVA", "MOVR", "JOG", "HOME")
# Stop / de-energize: always sent, even in dry run.
_ALWAYS_SAFE_PREFIXES = ("STOP", "MOFF")
# Commands that run or destroy the stored program. Refused unless raw(..., force=True).
#   RUN  : executes the stored program -> real motion AND output actuation
#   NEW  : erases program memory ("Purging the memory")
#   SAVE : overwrites non-volatile memory with the current RAM program
#   CONT : resumes a PAUSEd program -> can resume motion
_DESTRUCTIVE_PREFIXES = ("RUN", "NEW", "SAVE", "CONT")
# Commands that drive the physical output port (solenoid/relay/tool). The previous
# owner's program used output bit 2 for an unidentified end-effector.
_OUTPUT_PREFIXES = ("OUT", "SETBIT", "CLRBIT", "PWM")

VEL_MIN, VEL_MAX = 200, 200_000  # steps/s; the firmware clamps silently outside this
ACC_MIN, ACC_MAX = 40_000, 40_000_000  # steps/s^2
POS_MIN, POS_MAX = -2_147_483_647, 2_147_483_647

# Serial timing. The controller needs ~100 ms per command and drops replies when
# polled faster than ~20 Hz.
SERIAL_TIMEOUT_S = 0.05  # one read() call blocks at most this long
RESET_PULSE_S = 0.05  # RTS held asserted; the manual asks for >= 10 ms
BANNER_TIMEOUT_S = 2.5  # max wait for the reset banner
BANNER_QUIET_S = 0.3  # banner is complete this long after its last byte
REPLY_QUIET_S = 0.12  # a reply is complete this long after sending, once bytes arrived
MIN_POLL_S = 0.05  # floor for status polling (~20 Hz)
READ_CHUNK = 256


class OESController:
    """One controller on one serial port. Use it as a context manager. Opening
    pulses RESET, which zeroes every step counter."""

    AXES = ("X", "Y", "Z", "W")  # what the board can drive; W is absent on this machine

    def __init__(
        self,
        port: str = "/dev/ttyUSB0",
        baud: int = 19200,
        dry_run: bool = True,
        axes: tuple[str, ...] = ("X", "Y", "Z"),
        cmd_wait: float = 0.20,
        verbose: bool = True,
        home_switches: bool = False,
    ) -> None:
        """
        dry_run: start with motion commands simulated; see arm().
        axes: the axes this machine actually has. Anything else is refused.
        cmd_wait: longest wait for the reply to one command, in seconds.
        verbose: emit progress at INFO (True) or DEBUG (False) on the `oes` logger.
        home_switches: whether home switches are wired to the controller's HOME
            inputs. This machine has NONE, so HOME is refused: it would drive
            the axis into a hard stop. Pass True only after switches are fitted
            and verified. The HOME code path itself is complete.
        """
        self.port = port
        self.baud = baud
        self.dry_run = dry_run
        self.axes = tuple(axis.upper() for axis in axes)
        unknown = set(self.axes) - set(self.AXES)
        if unknown:
            raise OESError(f"unknown axes {sorted(unknown)}; the controller has {self.AXES}")
        self.cmd_wait = cmd_wait
        self.verbose = verbose
        self.home_switches = home_switches
        self.ser: serial.Serial | None = None
        self.banner = ""
        self.version: str | None = None

    # ------------------------------------------------------------- lifecycle
    def open(self, tries: int = 3) -> OESController:
        """Open the port and run the RESET handshake.

        The PL2303 intermittently throws 'device reports readiness to read but
        returned no data' on open -- a transient, not a real disconnect (the
        adapter stays enumerated at the same bus address). Retry rather than
        failing the whole run.
        """
        last_error: Exception | None = None
        for attempt in range(1, tries + 1):
            try:
                self.ser = serial.Serial(
                    self.port,
                    self.baud,
                    bytesize=serial.EIGHTBITS,
                    parity=serial.PARITY_NONE,
                    stopbits=serial.STOPBITS_ONE,
                    timeout=SERIAL_TIMEOUT_S,
                    rtscts=False,
                    dsrdtr=False,
                )
                self.ser.dtr = False  # DB9 pin 4 is NC on the controller
                self.reset()
                return self
            except (serial.SerialException, OSError) as e:
                last_error = e
                log.warning("open attempt %d/%d failed: %s", attempt, tries, e)
                if self.ser is not None:
                    with contextlib.suppress(Exception):
                        self.ser.close()
                self.ser = None
                time.sleep(attempt)
        raise OESError(f"could not open {self.port} after {tries} tries: {last_error}")

    def reset(self) -> None:
        """Documented RESET/init pulse on RTS (pin 7): SET >= 10 ms, then CLEAR.

        NOTE: open() calls this, so every OESController() invocation resets the
        board and ZEROES all step counters. (A bare port close does NOT reset --
        verified -- but our open deliberately does.) On this open-loop machine
        the counters therefore read 0 at the start of every script run,
        regardless of where the axes physically are. Never infer physical
        position from a counter across script invocations.
        """
        ser = self._require_port()
        ser.rts = True
        time.sleep(RESET_PULSE_S)
        ser.rts = False  # release reset; stays clear for the whole session
        # The firmware prints "Version xx.yy" then "Joystick is on" after every
        # reset, including the pulse just sent. Wait up to BANNER_TIMEOUT_S for
        # it, but stop BANNER_QUIET_S after the last byte so a normal boot does
        # not stall.
        buf = bytearray()
        start = time.monotonic()
        last_byte_at = start
        while time.monotonic() - start < BANNER_TIMEOUT_S:
            chunk = ser.read(READ_CHUNK)
            if chunk:
                buf.extend(chunk)
                last_byte_at = time.monotonic()
                if b"Version" in buf and buf.count(b"\n") >= 2:
                    break
            elif buf and time.monotonic() - last_byte_at > BANNER_QUIET_S:
                break
        self.banner = buf.decode("latin-1", "replace").strip()
        self.version = None
        if "Version" in self.banner:
            words = self.banner.split("Version", 1)[1].split()
            self.version = words[0] if words else None

    def close(self) -> None:
        if self.ser is not None:
            self.ser.close()
            self.ser = None

    def __enter__(self) -> OESController:
        return self.open()

    def __exit__(self, *exc: object) -> None:
        self.close()

    def _require_port(self) -> serial.Serial:
        if self.ser is None:
            raise OESError("port not open")
        return self.ser

    def _log(self, msg: str, *args: object) -> None:
        log.log(logging.INFO if self.verbose else logging.DEBUG, msg, *args)

    # ------------------------------------------------------------- low level io
    def raw(self, cmd: str, wait: float | None = None, force: bool = False) -> str:
        """Send one CR-terminated command and return the decoded reply.

        This is the single guard for everything that reaches the controller:
          * RUN/NEW/SAVE/CONT are refused unless force=True: they run or
            destroy the stored program;
          * HOME is refused unless the controller was constructed with
            home_switches=True (this machine has none), force or not;
          * motion and output commands (MON, MOVA, MOVR, JOG, HOME, OUT, ...)
            are logged instead of sent while dry_run is True -- see arm();
          * reads, VEL/ACC/POS/SPOS, STOP and MOFF are always sent.
        An empty reply is normal for most set commands and means success.
        """
        ser = self._require_port()
        head = _command_head(cmd)
        kind = classify_command(cmd)
        if kind == "destructive" and not force:
            raise OESError(
                f"{head!r} is refused: it runs or destroys the stored program. "
                f"Pass force=True only with a deliberate reason."
            )
        if head.startswith("HOME") and not self.home_switches:
            raise OESError(
                f"{head!r} is refused: this machine has no home switches, the "
                f"axis would run into a hard stop. Construct "
                f"OESController(home_switches=True) only once they are fitted."
            )
        if kind in ("actuator", "output") and self.dry_run:
            self._log("[DRY-RUN] would send %r", cmd.strip())
            return ""
        wait = self.cmd_wait if wait is None else wait
        ser.reset_input_buffer()
        ser.write((cmd.strip() + "\r").encode("latin-1"))
        ser.flush()
        buf = bytearray()
        start = time.monotonic()
        while time.monotonic() - start < wait:
            chunk = ser.read(READ_CHUNK)
            if chunk:
                buf.extend(chunk)
            elif buf and time.monotonic() - start > REPLY_QUIET_S:
                break
        return buf.decode("latin-1", "replace").strip()

    def _check_axis(self, axis: str) -> str:
        axis = axis.upper()
        if axis not in self.axes:
            raise OESError(f"invalid axis {axis!r}; this controller drives {self.axes}")
        return axis

    # ------------------------------------------------------------------- guard
    def arm(self, confirm: bool = False) -> None:
        """Enable real motion. Requires confirm=True as a deliberate step."""
        if not confirm:
            raise OESError("arm(confirm=True) required to enable motion")
        self.dry_run = False
        self._log("*** ARMED: motion commands will now be TRANSMITTED ***")

    def disarm(self) -> None:
        self.dry_run = True
        self._log("controller disarmed (dry-run); motion commands are simulated")

    def _run_sequence(self, commands: list[str], description: str) -> MoveResult:
        """Send a motion command sequence, or in dry run only log it."""
        if self.dry_run:
            self._log("[DRY-RUN] %s: would send -> %s", description, commands)
            return MoveResult(True, description, commands, None)
        responses = []
        for command in commands:
            reply = self.raw(command)
            responses.append(reply)
            self._log("  -> %r: %r", command, reply)
        return MoveResult(False, description, commands, responses)

    # --------------------------------------------------------------- read-only
    def _read_int(self, cmd: str, prefix: str | None = None, tries: int = 4) -> int:
        """Read a numeric reply, retrying transient empty/garbled responses.

        The controller occasionally returns nothing if polled faster than it can
        answer (~100 ms/command). A single dropped reply must NOT abort a
        move-completion poll, so retry before giving up.
        """
        last_error: OESError | None = None
        for attempt in range(1, tries + 1):
            reply = self.raw(cmd)
            try:
                return self._parse_signed(reply, expect_prefix=prefix)
            except OESError as e:
                last_error = e
                time.sleep(MIN_POLL_S * attempt)  # back off, let the board catch up
        raise OESError(f"{cmd}: no parseable reply after {tries} tries ({last_error})")

    def report_position(self, axis: str) -> int:
        """R<axis> -> current step counter. Reply like 'X+0'."""
        axis = self._check_axis(axis)
        return self._read_int(f"R{axis}", prefix=axis)

    def get_positions(self) -> dict[str, int]:
        return {axis: self.report_position(axis) for axis in self.axes}

    def axis_status(self, axis: str) -> int:
        """RSTS<axis> -> 32-bit axis status word. Bit 0 = 1 moving / 0 idle;
        bits 16-23 = analog channel value; RSTSX bits 8-11 = joystick keys."""
        axis = self._check_axis(axis)
        return self._read_int(f"RSTS{axis}")

    def is_moving(self, axis: str) -> bool:
        """True if the axis is in MOVE mode (status-word bit 0)."""
        return bool(self.axis_status(axis) & 0x1)

    def analog(self, axis: str) -> int:
        """Analog channel value for the axis (status-word bits 16-23)."""
        return (self.axis_status(axis) >> 16) & 0xFF

    def joystick(self, on: bool) -> str:
        return self.raw("JON" if on else "JOFF")

    def messages(self, on: bool) -> str:
        return self.raw("MSGON" if on else "MSGOFF")

    @staticmethod
    def _parse_signed(reply: str, expect_prefix: str | None = None) -> int:
        """'X+1000' -> 1000 (with expect_prefix 'X'), '-5' -> -5."""
        text = reply.strip()
        if expect_prefix and text[:1].upper() == expect_prefix.upper():
            text = text[1:]
        words = text.split()
        first = words[0] if words else text
        try:
            return int(first.replace("+", ""))
        except ValueError as e:
            raise OESError(f"could not parse numeric response {reply!r}") from e

    # --------------------------------------------- non-motion configuration
    def set_velocity(self, axis: str, steps_per_sec: int) -> str:
        axis = self._check_axis(axis)
        self._validate_vel(steps_per_sec)
        return self.raw(f"VEL{axis} {int(steps_per_sec)}")

    def set_acceleration(self, axis: str, steps_per_sec2: int) -> str:
        axis = self._check_axis(axis)
        self._validate_acc(steps_per_sec2)
        return self.raw(f"ACC{axis} {int(steps_per_sec2)}")

    def set_move_operand(self, axis: str, value: int) -> str:
        """POS<axis>: target position (for MOVA) or distance (for MOVR)."""
        axis = self._check_axis(axis)
        self._validate_pos(value, "position/distance")
        return self.raw(f"POS{axis} {int(value)}")

    def redefine_position(self, axis: str, value: int = 0) -> str:
        """SPOS<axis>: redefine the current step counter (like G92). No motion."""
        axis = self._check_axis(axis)
        self._validate_pos(value, "value")
        return self.raw(f"SPOS{axis} {int(value)}")

    # ---------------------------------------------------- guarded motion API
    def motor_on(self, axis: str) -> MoveResult:
        axis = self._check_axis(axis)
        return self._run_sequence([f"MON{axis}"], f"enable {axis} motor")

    def motor_off(self, axis: str) -> str:
        """Always executes (de-energizing is safe)."""
        axis = self._check_axis(axis)
        return self.raw(f"MOFF{axis}")

    def _move(
        self,
        axis: str,
        mnemonic: str,
        value: int,
        vel: int | None,
        acc: int | None,
        enable: bool,
        description: str,
    ) -> MoveResult:
        """Shared body of move_absolute / move_relative: MON, ACC, VEL, POS, then
        the move mnemonic. Returns as soon as the controller acknowledges."""
        self._validate_pos(value, description)
        commands = []
        if enable:
            commands.append(f"MON{axis}")
        if acc is not None:
            self._validate_acc(acc)
            commands.append(f"ACC{axis} {int(acc)}")
        if vel is not None:
            self._validate_vel(vel)
            commands.append(f"VEL{axis} {int(vel)}")
        commands += [f"POS{axis} {int(value)}", f"{mnemonic}{axis}"]
        return self._run_sequence(commands, description)

    def move_absolute(
        self,
        axis: str,
        target: int,
        vel: int | None = None,
        acc: int | None = None,
        enable: bool = True,
    ) -> MoveResult:
        """MOVA: go to `target` steps from the current origin. Does not block;
        call wait_stopped() to wait for the move to end."""
        axis = self._check_axis(axis)
        return self._move(
            axis, "MOVA", target, vel, acc, enable, f"absolute move {axis} -> {target}"
        )

    def move_relative(
        self,
        axis: str,
        distance: int,
        vel: int | None = None,
        acc: int | None = None,
        enable: bool = True,
    ) -> MoveResult:
        """MOVR: move by `distance` steps (signed). Does not block; call
        wait_stopped() to wait for the move to end."""
        axis = self._check_axis(axis)
        return self._move(
            axis, "MOVR", distance, vel, acc, enable, f"relative move {axis} by {distance}"
        )

    def jog(self, axis: str, vel: int | None = None, enable: bool = True) -> MoveResult:
        """Continuous jog. Direction is the sign of the velocity. Stop with stop()."""
        axis = self._check_axis(axis)
        commands = []
        if enable:
            commands.append(f"MON{axis}")
        if vel is not None:
            self._validate_vel(vel, allow_negative=True)
            commands.append(f"VEL{axis} {int(vel)}")
        commands.append(f"JOG{axis}")
        return self._run_sequence(commands, f"jog {axis}")

    def home(self, axis: str, enable: bool = True) -> MoveResult:
        """Homing sequence. Refused unless constructed with home_switches=True:
        this machine has no switches and the axis would drive into a hard stop
        with the gearbox multiplying the motor torque."""
        axis = self._check_axis(axis)
        if not self.home_switches:
            raise OESError(
                f"HOME{axis} refused: no home switches on this machine. Construct "
                f"OESController(home_switches=True) only once they are fitted."
            )
        commands = ([f"MON{axis}"] if enable else []) + [f"HOME{axis}"]
        return self._run_sequence(commands, f"home {axis}")

    def stop(self, axis: str | None = None) -> str:
        """STOP<axis> or STOPALL. ALWAYS executes -- stopping is always safe."""
        if axis is None:
            return self.raw("STOPALL")
        return self.raw(f"STOP{self._check_axis(axis)}")

    def wait_stopped(
        self,
        axis: str,
        timeout: float = 60.0,
        poll: float = 0.1,
        on_poll: Callable[[], bool] | None = None,
        grace: float = 1.0,
        max_read_failures: int = 5,
    ) -> bool:
        """Poll RSTS<axis> bit 0 until the axis is idle. Works with MSGOFF.
        Returns True once the axis is idle, False on timeout or when on_poll
        asked to break out.

        Start-up grace: an idle reading counts only after the axis has been
        seen moving, or after `grace` seconds. That covers both a short move
        that finishes before the first poll and a controller that has not
        started the move yet.

        on_poll is called between polls; return True to break out (the caller
        decides whether to STOP). A GUI uses it to check its abort flag and
        refresh the position display.

        The poll interval is floored at MIN_POLL_S: the controller needs ~100 ms
        per command and drops replies when polled faster than ~20 Hz. Up to
        `max_read_failures` consecutive unreadable replies are tolerated, then
        the OESError propagates.
        """
        poll = max(poll, MIN_POLL_S)
        axis = self._check_axis(axis)
        start = time.monotonic()
        seen_moving = False
        failures = 0
        while time.monotonic() - start < timeout:
            try:
                moving = bool(self.axis_status(axis) & 0x1)
                failures = 0
            except OESError:
                failures += 1
                if failures >= max_read_failures:
                    raise
                continue
            if moving:
                seen_moving = True
            elif seen_moving or time.monotonic() - start > grace:
                return True
            if on_poll is not None and on_poll():
                return False
            time.sleep(poll)
        return False

    # -------------------------------------------------------------- validation
    @staticmethod
    def _validate_vel(vel: int, allow_negative: bool = False) -> None:
        magnitude = abs(vel) if allow_negative else vel
        if not VEL_MIN <= magnitude <= VEL_MAX:
            raise OESError(f"velocity {vel} out of range [{VEL_MIN}, {VEL_MAX}] steps/s")

    @staticmethod
    def _validate_acc(acc: int) -> None:
        if not ACC_MIN <= acc <= ACC_MAX:
            raise OESError(f"acceleration {acc} out of range [{ACC_MIN}, {ACC_MAX}] steps/s^2")

    @staticmethod
    def _validate_pos(value: int, what: str) -> None:
        if not POS_MIN <= value <= POS_MAX:
            raise OESError(f"{what}: {value} out of 32-bit range")


def _command_head(cmd: str) -> str:
    """Mnemonic of a command line: first token, upper-cased, digits stripped.
    'velz 5000' -> 'VELZ', 'STOPALL' -> 'STOPALL', 'setbit 2' -> 'SETBIT'."""
    head = "".join(ch for ch in cmd.strip().upper() if not ch.isdigit()).strip()
    words = head.split()
    return words[0] if words else head


def classify_command(cmd: str) -> str:
    """Return 'destructive' (refused unless forced), 'actuator' / 'output'
    (simulated unless armed), 'safe-stop' (always sent) or 'other' (reads and
    non-motion configuration, always sent). raw() applies this to everything."""
    head = _command_head(cmd)
    for kind, prefixes in (
        ("safe-stop", _ALWAYS_SAFE_PREFIXES),
        ("destructive", _DESTRUCTIVE_PREFIXES),
        ("actuator", _ACTUATOR_PREFIXES),
        ("output", _OUTPUT_PREFIXES),
    ):
        if head.startswith(prefixes):
            return kind
    return "other"


if __name__ == "__main__":
    # Quick self-test: connect, print version + positions (read-only), then show
    # a dry-run motion plan without sending it.
    logging.basicConfig(level=logging.INFO, format="%(message)s")
    with OESController() as controller:
        print(f"banner : {controller.banner!r}")
        print(f"version: {controller.version}")
        print(f"joystick off: {controller.joystick(False)!r}")
        print(f"positions   : {controller.get_positions()}")
        print("\n--- dry-run motion plan (nothing sent) ---")
        print(controller.move_relative("X", 2000, vel=5000, acc=200000))
