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
  This driver defaults to dry_run=True. All commands that can ENERGIZE or MOVE a
  motor (MON, MOVA, MOVR, JOG, HOME) are refused unless you explicitly arm() the
  controller. In dry_run mode those methods return the exact command sequence they
  WOULD send, so you can inspect motion plans without any risk. Read-only reports
  and non-motion configuration (VEL/ACC/POS/SPOS) always execute. STOP/MOFF always
  execute (stopping/de-energizing is always safe).
  HOME is refused regardless of arming unless the controller is constructed
  with home_switches=True. This machine has no home switches.

Move model (per OES reference):
  MON x                enable the axis driver
  ACC x <steps/s^2>    set acceleration   (40,000 .. 40,000,000)
  VEL x <steps/s>      set slew speed     (200 .. 200,000)
  POS x <value>        set target (for MOVA) / distance (for MOVR), +-2147483647
  MOVA x  |  MOVR x    begin absolute / relative move  -> "X Abs. Move" then "Done X"
"""

from __future__ import annotations
import time
from typing import Callable

try:
    import serial
except ImportError as e:
    raise SystemExit("pyserial is required: pip install pyserial") from e


class OESError(Exception):
    """Protocol / usage error talking to the controller."""


# Command prefixes that can energize or move a motor -> gated behind arm().
_ACTUATOR_PREFIXES = ("MON", "MOVA", "MOVR", "JOG", "HOME")
# Always-safe commands (execute even in dry_run): stop / de-energize.
_ALWAYS_SAFE_PREFIXES = ("STOP", "MOFF")
# Commands that destroy stored state or run the stored program. These are
# refused ALWAYS (arming does not unlock them) -- only raw(..., force=True).
#   RUN  : executes the stored program -> real motion AND output actuation
#   NEW  : erases program memory ("Purging the memory")
#   SAVE : overwrites non-volatile memory with current RAM program
#   CONT : resumes a PAUSEd program -> can resume motion
_DESTRUCTIVE_PREFIXES = ("RUN", "NEW", "SAVE", "CONT")
# Commands that drive the physical output port (solenoid/relay/tool). Gated
# behind arm() -- the previous owner's program uses output bit 2 for an
# unidentified end-effector.
_OUTPUT_PREFIXES = ("OUT", "SETBIT", "CLRBIT", "PWM")

VEL_MIN, VEL_MAX = 200, 200_000  # steps / sec
ACC_MIN, ACC_MAX = 40_000, 40_000_000  # steps / sec^2
POS_MIN, POS_MAX = -2_147_483_647, 2_147_483_647


class OESController:
    AXES = ("X", "Y", "Z", "W")

    def __init__(
        self,
        port: str = "/dev/ttyUSB0",
        baud: int = 19200,
        dry_run: bool = True,
        axes=("X", "Y", "Z"),
        cmd_wait: float = 0.20,
        verbose: bool = True,
        home_switches: bool = False,
    ):
        """
        home_switches: whether home switches are wired to the controller's HOME
            inputs. This machine has NONE, so HOME is refused: it would drive
            the axis into a hard stop. Pass True only after switches are fitted
            and verified. The HOME code path itself is complete.
        """
        self.port = port
        self.baud = baud
        self.dry_run = dry_run
        self.axes = tuple(a.upper() for a in axes)
        unknown = set(self.axes) - set(self.AXES)
        if unknown:
            raise OESError(f"unknown axes {sorted(unknown)}; the controller has {self.AXES}")
        self.cmd_wait = cmd_wait
        self.verbose = verbose
        self.home_switches = home_switches
        self.ser: serial.Serial | None = None
        self.banner = ""
        self.version = None

    def open(self, tries: int = 3) -> OESController:
        """Open the port and run the RESET handshake.

        The PL2303 intermittently throws 'device reports readiness to read but
        returned no data' on open -- a transient, not a real disconnect (the
        adapter stays enumerated at the same bus address). Retry rather than
        failing the whole run.
        """
        last = None
        for i in range(tries):
            try:
                self.ser = serial.Serial(
                    self.port,
                    self.baud,
                    bytesize=serial.EIGHTBITS,
                    parity=serial.PARITY_NONE,
                    stopbits=serial.STOPBITS_ONE,
                    timeout=0.05,
                    rtscts=False,
                    dsrdtr=False,
                )
                self.ser.dtr = False  # DB9 pin 4 is NC on the controller
                self.reset()
                return self
            except (serial.SerialException, OSError) as e:
                last = e
                self._log(f"open attempt {i + 1}/{tries} failed: {e}")
                try:
                    if self.ser:
                        self.ser.close()
                except Exception:
                    pass
                self.ser = None
                time.sleep(1.0 + i)
        raise OESError(f"could not open {self.port} after {tries} tries: {last}")

    def reset(self) -> None:
        """Documented RESET/init pulse on RTS (pin 7): SET >=10ms, then CLEAR.
        NOTE: open() calls this, so every OESController() invocation resets the
        board and ZEROES all step counters.
        """

        if not self.ser:
            raise OESError("port not open")
        self.ser.rts = True  # SET
        time.sleep(0.05)  # >= 10 ms
        self.ser.rts = False  # CLEAR (release reset; hold clear)
        # The firmware prints "Version xx.yy" then "Joystick is on" after every
        # reset, including the pulse just sent. Wait up to ~2.5 s for it, but
        # stop ~0.3 s after the last byte so a normal boot does not stall.
        buf = bytearray()
        t0 = time.time()
        last = t0
        while time.time() - t0 < 2.5:
            c = self.ser.read(256)
            if c:
                buf.extend(c)
                last = time.time()
                if b"Version" in buf and buf.count(b"\n") >= 2:
                    break
            elif buf and (time.time() - last) > 0.3:
                break
        self.banner = buf.decode("latin-1", "replace").strip()
        if "Version" in self.banner:
            try:
                self.version = self.banner.split("Version", 1)[1].split()[0]
            except IndexError:
                self.version = None

    def close(self) -> None:
        if self.ser:
            self.ser.close()
            self.ser = None

    def __enter__(self):
        return self.open()

    def __exit__(self, *exc):
        self.close()

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
        if not self.ser:
            raise OESError("port not open")
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
            self._log(f"[DRY-RUN] would send {cmd.strip()!r}")
            return ""
        wait = self.cmd_wait if wait is None else wait
        payload = (cmd.strip() + "\r").encode("latin-1")
        self.ser.reset_input_buffer()
        self.ser.write(payload)
        self.ser.flush()
        buf = bytearray()
        t0 = time.time()
        while time.time() - t0 < wait:
            c = self.ser.read(256)
            if c:
                buf.extend(c)
            elif buf and (time.time() - t0) > 0.12:
                break
        return buf.decode("latin-1", "replace").strip()

    def _check_axis(self, axis: str) -> str:
        a = axis.upper()
        if a not in self.axes:
            raise OESError(f"invalid axis {axis!r}; this controller drives {self.axes}")
        return a

    def _log(self, msg: str) -> None:
        if self.verbose:
            print(msg)

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

    def _run_sequence(self, commands: list[str], desc: str) -> dict:
        """Execute (or, in dry_run, simulate) a motion command sequence."""
        if self.dry_run:
            self._log(f"[DRY-RUN] {desc}: would send -> {commands}")
            return {"dry_run": True, "desc": desc, "commands": commands, "responses": None}
        responses = []
        for c in commands:
            r = self.raw(c)
            responses.append(r)
            self._log(f"  -> {c!r}: {r!r}")
        return {"dry_run": False, "desc": desc, "commands": commands, "responses": responses}

    # --------------------------------------------------------------- read-only
    def _read_int(self, cmd: str, prefix: str | None = None, tries: int = 4) -> int:
        """Read a numeric reply, retrying transient empty/garbled responses.

        The controller occasionally returns nothing if polled faster than it can
        answer (~100 ms/command). A single dropped reply must NOT abort a
        move-completion poll, so retry before giving up.
        """
        last = None
        for i in range(tries):
            resp = self.raw(cmd)
            try:
                return self._parse_signed(resp, expect_prefix=prefix)
            except OESError as e:
                last = e
                time.sleep(0.05 * (i + 1))  # let the board catch up
        raise OESError(f"{cmd}: no parseable reply after {tries} tries ({last})")

    def report_position(self, axis: str) -> int:
        """R<axis> -> current step counter. Reply like 'X+0'."""
        a = self._check_axis(axis)
        return self._read_int(f"R{a}", prefix=a)

    def get_positions(self) -> dict:
        return {a: self.report_position(a) for a in self.axes}

    def axis_status(self, axis: str) -> int:
        """RSTS<axis> -> 32-bit axis status word. Bit 0 = 1 moving / 0 idle;
        bits 16-23 = analog channel value; RSTSX bits 8-11 = joystick keys."""
        a = self._check_axis(axis)
        return self._read_int(f"RSTS{a}")

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
    def _parse_signed(resp: str, expect_prefix: str | None = None) -> int:
        s = resp.strip()
        if expect_prefix and s[:1].upper() == expect_prefix.upper():
            s = s[1:]
        s = s.split()[0] if s.split() else s
        try:
            return int(s.replace("+", ""))
        except ValueError as e:
            raise OESError(f"could not parse numeric response {resp!r}") from e

    # --------------------------------------------- non-motion configuration
    def set_velocity(self, axis: str, steps_per_sec: int) -> str:
        a = self._check_axis(axis)
        if not (VEL_MIN <= steps_per_sec <= VEL_MAX):
            raise OESError(f"velocity {steps_per_sec} out of range [{VEL_MIN}, {VEL_MAX}] steps/s")
        return self.raw(f"VEL{a} {int(steps_per_sec)}")

    def set_acceleration(self, axis: str, steps_per_sec2: int) -> str:
        a = self._check_axis(axis)
        if not (ACC_MIN <= steps_per_sec2 <= ACC_MAX):
            raise OESError(
                f"acceleration {steps_per_sec2} out of range [{ACC_MIN}, {ACC_MAX}] steps/s^2"
            )
        return self.raw(f"ACC{a} {int(steps_per_sec2)}")

    def set_move_operand(self, axis: str, value: int) -> str:
        """POS<axis>: target position (for MOVA) or distance (for MOVR)."""
        a = self._check_axis(axis)
        if not (POS_MIN <= value <= POS_MAX):
            raise OESError(f"position/distance {value} out of 32-bit range")
        return self.raw(f"POS{a} {int(value)}")

    def redefine_position(self, axis: str, value: int = 0) -> str:
        """SPOS<axis>: redefine the current step counter (like G92). No motion."""
        a = self._check_axis(axis)
        if not (POS_MIN <= value <= POS_MAX):
            raise OESError(f"value {value} out of 32-bit range")
        return self.raw(f"SPOS{a} {int(value)}")

    # ---------------------------------------------------- guarded motion API
    def motor_on(self, axis: str) -> dict:
        a = self._check_axis(axis)
        return self._run_sequence([f"MON{a}"], f"enable {a} motor")

    def motor_off(self, axis: str) -> str:
        """Always executes (de-energizing is safe)."""
        a = self._check_axis(axis)
        return self.raw(f"MOFF{a}")

    def _move(
        self,
        a: str,
        mnemonic: str,
        value: int,
        vel: int | None,
        acc: int | None,
        enable: bool,
        desc: str,
    ) -> dict:
        """Shared body of move_absolute / move_relative: MON, ACC, VEL, POS, then
        the move mnemonic. Returns as soon as the controller acknowledges."""
        if not (POS_MIN <= value <= POS_MAX):
            raise OESError(f"{desc}: {value} out of 32-bit range")
        seq = []
        if enable:
            seq.append(f"MON{a}")
        if acc is not None:
            self._validate_acc(acc)
            seq.append(f"ACC{a} {int(acc)}")
        if vel is not None:
            self._validate_vel(vel)
            seq.append(f"VEL{a} {int(vel)}")
        seq += [f"POS{a} {int(value)}", f"{mnemonic}{a}"]
        return self._run_sequence(seq, desc)

    def move_absolute(
        self,
        axis: str,
        target: int,
        vel: int | None = None,
        acc: int | None = None,
        enable: bool = True,
    ) -> dict:
        """MOVA: go to `target` steps from the current origin. Does not block;
        call wait_stopped() to wait for the move to end."""
        a = self._check_axis(axis)
        return self._move(a, "MOVA", target, vel, acc, enable, f"absolute move {a} -> {target}")

    def move_relative(
        self,
        axis: str,
        distance: int,
        vel: int | None = None,
        acc: int | None = None,
        enable: bool = True,
    ) -> dict:
        """MOVR: move by `distance` steps (signed). Does not block; call
        wait_stopped() to wait for the move to end."""
        a = self._check_axis(axis)
        return self._move(a, "MOVR", distance, vel, acc, enable, f"relative move {a} by {distance}")

    def jog(self, axis: str, vel: int | None = None, enable: bool = True) -> dict:
        """Continuous jog. Direction is the sign of the velocity. Stop with stop()."""
        a = self._check_axis(axis)
        seq = []
        if enable:
            seq.append(f"MON{a}")
        if vel is not None:
            self._validate_vel(abs(vel))
            seq.append(f"VEL{a} {int(vel)}")
        seq.append(f"JOG{a}")
        return self._run_sequence(seq, f"jog {a}")

    def home(self, axis: str, enable: bool = True) -> dict:
        """Homing sequence. Refused unless constructed with home_switches=True:
        this machine has no switches and the axis would drive into a hard stop
        with the gearbox multiplying the motor torque."""
        a = self._check_axis(axis)
        if not self.home_switches:
            raise OESError(
                f"HOME{a} refused: no home switches on this machine. Construct "
                f"OESController(home_switches=True) only once they are fitted."
            )
        seq = ([f"MON{a}"] if enable else []) + [f"HOME{a}"]
        return self._run_sequence(seq, f"home {a}")

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

        The poll interval is floored at 50 ms: the controller needs ~100 ms per
        command and drops replies when polled faster than ~20 Hz. Up to
        `max_read_failures` consecutive unreadable replies are tolerated, then
        the OESError propagates.
        """
        poll = max(poll, 0.05)
        a = self._check_axis(axis)
        t0 = time.time()
        seen_moving = False
        failures = 0
        while time.time() - t0 < timeout:
            try:
                moving = bool(self.axis_status(a) & 0x1)
                failures = 0
            except OESError:
                failures += 1
                if failures >= max_read_failures:
                    raise
                continue
            if moving:
                seen_moving = True
            elif seen_moving or time.time() - t0 > grace:
                return True
            if on_poll is not None and on_poll():
                return False
            time.sleep(poll)
        return False

    def _validate_vel(self, v):
        if not (VEL_MIN <= abs(v) <= VEL_MAX):
            raise OESError(f"velocity {v} out of range [{VEL_MIN}, {VEL_MAX}]")

    def _validate_acc(self, a):
        if not (ACC_MIN <= a <= ACC_MAX):
            raise OESError(f"acceleration {a} out of range [{ACC_MIN}, {ACC_MAX}]")


def _command_head(cmd: str) -> str:
    """Mnemonic of a command line: first token, upper-cased, digits stripped.
    'velz 5000' -> 'VELZ', 'STOPALL' -> 'STOPALL', 'setbit 2' -> 'SETBIT'."""
    head = "".join(ch for ch in cmd.strip().upper() if not ch.isdigit()).strip()
    return head.split()[0] if head.split() else head


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
    # Quick self-test: connect, print version + positions (read-only).
    with OESController() as oes:
        print(f"banner : {oes.banner!r}")
        print(f"version: {oes.version}")
        print(f"joystick off: {oes.joystick(False)!r}")
        print(f"positions   : {oes.get_positions()}")
        print("\n--- dry-run motion plan (nothing sent) ---")
        oes.move_relative("X", 2000, vel=5000, acc=200000)
