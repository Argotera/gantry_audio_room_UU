#!/usr/bin/env python3
"""
move.py -- self-contained one-axis mover for the gantry robot in the robot lab.

Needs only Python 3.7+ and pyserial. On purpose it imports nothing else from the
repository, so this one file can be copied to any computer with the USB adapter
and used on its own. It is a DIY machine. BE VERY CAREFUL. Ask lab for details and
pdf with extensive hardware description, details, and gotchas.

THE MACHINE
  Three-axis Cartesian belt gantry in room 73133, working envelope roughly
  3.4 m (X) x 4.3 m (Y) x 1.0 m (Z).
    X  moves the trolley along the bridge    +X = towards the office doors (north)
    Y  moves the bridge along the frame      +Y = towards the curtain wall (west)
    Z  moves the carriage along the leg      +Z = up
    Most set commands reply with nothing. Empty means success.
    Motion is ALWAYS LIVE in this script. There is no dry run.
    Ctrl+C sends STOPALL, but the link adds about 0.2 s of lag.
    THE ONLY REAL EMERGENCY STOP IS THE POWER CORD.
    Exactly one process may hold the serial port. Close every other tool first.

USAGE
    ./move.py --axis x --mm 250               # move +250 mm and stay there
    ./move.py --axis x --mm -250              # send the negative to come back
    ./move.py --axis y --steps -10000 --vel 2000
    ./move.py --axis Z --steps 5000           # raw steps still work
    ./move.py --axis X                        # neither: just report the counters
    ./move.py --axis Z --home-counter         # SPOSZ 0: redefine here as 0, no motion

FLAGS
  --axis             X, Y or Z. Required. There is no default, so a forgotten
                     flag cannot move the wrong axis.
  --mm / --steps     The same relative move in millimetres or in raw steps. Give
                     one, never both. --mm is converted with the axis's steps/mm
                     and rounded to whole steps; the residual is printed.
  --vel              steps/s, default 1000. Refused above the proven ceiling
                     unless --force is given.
  --acc              steps/s^2, default 40000, which is also the firmware floor.
  --home-counter     SPOS<axis> 0: redefine the current position as zero. No motion.
  --leave-energized  Skip the closing MOFF so the axis keeps holding torque.
  --force            Allow --vel above the proven ceiling. Deliberate testing only.
  --port             Serial port, default /dev/ttyUSB0. List candidates with
                     `python3 -m serial.tools.list_ports`.

It prints the firmware version, the counter before and after, the commanded
distance in both units, the rounding residual, the elapsed time against the
theoretical one, and the exact flags that reverse the move.
"""

from __future__ import annotations

import argparse
import contextlib
import socket
import threading
import time

try:
    import serial
except ImportError as e:
    raise SystemExit("pyserial is required: pip install pyserial") from e

DEFAULT_PORT = "/dev/ttyUSB0"
BAUD = 19200
AXES = ("X", "Y", "Z")  # W is absent on this machine

VEL_MIN, VEL_MAX = 200, 200_000  # firmware range; outside it the value is clamped silently
ACC_MIN, ACC_MAX = 40_000, 40_000_000
POS_LIMIT = 2_147_483_647

# Highest velocity PROVEN clean on each axis, steps/s. The firmware accepts up to
# 200000.
VEL_CEILING = {"X": 5000, "Y": 5000, "Z": 5000}
# Measured for the complete drive train: microsteps -> gearbox -> cog -> belt.
STEPS_PER_MM = {"X": 105.2632, "Y": 210.5263, "Z": 105.2632}

# Serial timing: ~100 ms per command; replies are dropped when polled faster than ~20 Hz.
SERIAL_TIMEOUT_S = 0.05  # one read() call blocks at most this long
RESET_PULSE_S = 0.05  # RTS held asserted; the manual asks for >= 10 ms
BANNER_TIMEOUT_S = 2.5  # max wait for the reset banner
BANNER_QUIET_S = 0.3  # banner is complete this long after its last byte
REPLY_QUIET_S = 0.12  # a reply is complete this long after sending, once bytes arrived
POLL_S = 0.05  # status poll interval (~20 Hz)
START_GRACE_S = 1.0  # an idle reading before this only counts if the axis was seen moving
MOVE_TIMEOUT_MARGIN_S = 30.0  # added to the theoretical move time before giving up
READ_CHUNK = 256


class GantryError(Exception):
    """Could not talk to the controller."""


class _Reads:
    """The read vocabulary, shared by the local and remote transports below.

    Both need exactly these four, and both get them from raw(), so they live
    here once rather than being written twice in the same file.
    """

    def raw(self, cmd: str, wait: float | None = None) -> str:  # pragma: no cover
        raise NotImplementedError

    def _read_int(self, cmd: str, prefix: str | None = None, tries: int = 4) -> int:
        """Numeric read with retry. A transient empty reply is normal and must
        never abort a move-completion poll."""
        last_error: Exception | None = None
        for attempt in range(1, tries + 1):
            text = self.raw(cmd).strip()
            if prefix and text[:1].upper() == prefix.upper():
                text = text[1:]
            words = text.split()
            first = words[0] if words else text
            try:
                return int(first.replace("+", ""))
            except ValueError as e:
                last_error = e
                time.sleep(POLL_S * attempt)
        raise GantryError(f"{cmd}: no parseable reply after {tries} tries ({last_error})")

    def position(self, axis: str) -> int:
        """R<axis> -> commanded step counter (NOT where the axis actually is)."""
        return self._read_int(f"R{axis}", prefix=axis)

    def positions(self) -> dict[str, int]:
        return {axis: self.position(axis) for axis in AXES}

    def is_moving(self, axis: str) -> bool:
        """RSTS<axis> bit 0. Works with MSGOFF. Do not poll faster than ~20 Hz."""
        return bool(self._read_int(f"RSTS{axis}") & 0x1)


class Gantry(_Reads):
    """Minimal RS-232 client for the OES controller.

    A deliberate copy of the transport layer in oes.py, so that this file stays
    self-contained. Keep the two in sync.
    """

    def __init__(self, port: str = DEFAULT_PORT, baud: int = BAUD, cmd_wait: float = 0.20) -> None:
        self.port = port
        self.baud = baud
        self.cmd_wait = cmd_wait
        self.ser: serial.Serial | None = None
        self.version: str | None = None

    def open(self, tries: int = 3) -> Gantry:
        """Open the port and run the RESET handshake. The PL2303 throws a
        transient SerialException on open now and then: retry rather than fail."""
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
                self.ser.dtr = False  # DB9 pin 4 is not connected on the controller
                self._reset()
                return self
            except (serial.SerialException, OSError) as e:
                last_error = e
                print(f"open attempt {attempt}/{tries} failed: {e}")
                if self.ser is not None:
                    with contextlib.suppress(Exception):
                        self.ser.close()
                self.ser = None
                time.sleep(attempt)
        raise GantryError(f"could not open {self.port} after {tries} tries: {last_error}")

    def _reset(self) -> None:
        """RTS SET >= 10 ms, then CLEAR and hold clear for the whole session. Leave
        RTS asserted and the board is mute at every baud rate.
        WARNING: this ZEROES every step counter."""
        ser = self._require_port()
        ser.rts = True
        time.sleep(RESET_PULSE_S)
        ser.rts = False
        # The board prints "Version 11.27" and "Joystick is on" after every reset.
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
        banner = buf.decode("latin-1", "replace").strip()
        if "Version" in banner:
            words = banner.split("Version", 1)[1].split()
            self.version = words[0] if words else None

    def close(self) -> None:
        if self.ser is not None:
            self.ser.close()
            self.ser = None

    def __enter__(self) -> Gantry:
        return self.open()

    def __exit__(self, *exc: object) -> None:
        self.close()

    def _require_port(self) -> serial.Serial:
        if self.ser is None:
            raise GantryError("port not open")
        return self.ser

    def raw(self, cmd: str, wait: float | None = None) -> str:
        """Send one CR-terminated command and return the decoded reply.
        An empty reply means success for most set commands."""
        ser = self._require_port()
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

    def stop_all(self) -> str:
        return self.raw("STOPALL")


# --------------------------------------------------------------------------
# Remote operation: the same machine reached through a gantry_server that owns
# the serial port, normally over a Bluetooth PAN.
#
# This client is deliberately a SECOND implementation rather than an import.
# move.py's whole point is that one file can be copied anywhere. The cost is
# that the two constants below are duplicated from gantry_link.py -- but the
# server refuses a protocol-version mismatch outright, so getting it wrong
# fails loudly on connect rather than silently doing the wrong thing.
# --------------------------------------------------------------------------

PROTOCOL_VERSION = 1  # keep in step with gantry_link.PROTOCOL_VERSION
ARM_ACKNOWLEDGEMENT = "workspace clear; power cutoff reachable"
PING_INTERVAL_S = 1.0  # keeps the server's watchdog happy and the link out of sniff
LATENCY_BUDGET_S = 2.0  # added to each command's wait; a sniffing link is slow to wake


def _escape(text: str) -> str:
    return text.replace("\\", "\\\\").replace("\r", "\\r").replace("\n", "\\n")


def _unescape(text: str) -> str:
    out, i = [], 0
    while i < len(text):
        if text[i] == "\\" and i + 1 < len(text):
            nxt = text[i + 1]
            if nxt in "\\rn":
                out.append({"\\": "\\", "r": "\r", "n": "\n"}[nxt])
                i += 2
                continue
        out.append(text[i])
        i += 1
    return "".join(out)


class RemoteGantry(_Reads):
    """Same surface as Gantry, but the serial port lives on another machine.

    ⚠️ Opening does NOT reset the controller -- the server holds the port open
    so that step counters survive a dropped link. The counter you read is
    whatever the server has been tracking, not "zero, here".

    ⚠️ The server stops the machine if the link goes quiet, so this keeps a
    heartbeat running for as long as it is connected.
    """

    def __init__(self, url: str, cmd_wait: float = 0.20, connect_timeout: float = 10.0) -> None:
        self.url = url
        self.cmd_wait = cmd_wait
        self.connect_timeout = connect_timeout
        self.sock: socket.socket | None = None
        self.version: str | None = None
        self.info: dict[str, str] = {}
        self._buf = bytearray()
        self._next_id = 0
        self._lock = threading.RLock()
        self._last_activity = 0.0
        self._closing = threading.Event()
        self._pinger: threading.Thread | None = None

    # -- lifecycle ----------------------------------------------------------
    def open(self) -> RemoteGantry:
        scheme, _, rest = self.url.partition("://")
        if scheme.lower() != "tcp":
            raise GantryError(f"expected tcp://host:port, got {self.url!r}")
        host, _, port = rest.rpartition(":")
        if not host or not port.isdigit():
            raise GantryError(f"expected tcp://host:port, got {self.url!r}")
        try:
            self.sock = socket.create_connection((host, int(port)), timeout=self.connect_timeout)
        except OSError as e:
            raise GantryError(f"cannot reach {self.url}: {e}") from e
        self._last_activity = time.monotonic()
        payload = self._request("HELLO", str(PROTOCOL_VERSION), "move.py")
        self.info = dict(item.split("=", 1) for item in payload.split(" ") if "=" in item)
        self.version = self.info.get("firmware")
        self._closing.clear()
        self._pinger = threading.Thread(target=self._ping_loop, daemon=True)
        self._pinger.start()
        return self

    def close(self) -> None:
        self._closing.set()
        if self._pinger is not None:
            self._pinger.join(timeout=2.0)
            self._pinger = None
        if self.sock is not None:
            with contextlib.suppress(OSError):
                self.sock.shutdown(socket.SHUT_RDWR)
            with contextlib.suppress(OSError):
                self.sock.close()
            self.sock = None

    def __enter__(self) -> RemoteGantry:
        return self.open()

    def __exit__(self, *exc: object) -> None:
        self.close()

    # -- the wire -----------------------------------------------------------
    def _read_line(self) -> str:
        assert self.sock is not None
        while True:
            index = self._buf.find(b"\n")
            if index >= 0:
                line = bytes(self._buf[:index])
                del self._buf[: index + 1]
                return line.decode("latin-1", "replace").rstrip("\r")
            chunk = self.sock.recv(4096)
            if not chunk:
                # An unterminated remainder is dropped on purpose: a truncated
                # line can still parse as a valid but DIFFERENT message.
                raise GantryError(
                    f"{self.url} closed the link. If the server's watchdog fired it has "
                    f"sent STOPALL and ended the session."
                )
            self._buf.extend(chunk)

    def _request(self, verb: str, *args: str, wait: float = 0.0) -> str:
        if self.sock is None:
            raise GantryError(f"not connected to {self.url}")
        timeout = wait + LATENCY_BUDGET_S
        with self._lock:
            self._next_id += 1
            request_id = self._next_id
            # Only the trailing field is escaped, which keeps a command with a
            # space in it ("VELX 5000") readable in the server's log.
            fields = [verb, str(request_id)]
            if args:
                fields += list(args[:-1]) + [_escape(args[-1])]
            line = " ".join(fields)
            try:
                self.sock.settimeout(timeout)
                self.sock.sendall((line + "\n").encode("latin-1"))
            except OSError as e:
                raise GantryError(f"link to {self.url} failed: {e}") from e
            while True:
                try:
                    reply = self._read_line()
                except OSError as e:
                    raise GantryError(f"link to {self.url} failed: {e}") from e
                self._last_activity = time.monotonic()
                pieces = reply.split(" ")
                if pieces[0].upper() == "EVT":
                    print(f"  ** server event: {' '.join(pieces[1:])}")
                    continue
                if pieces[0].upper() != "REP" or len(pieces) < 3:
                    raise GantryError(f"unintelligible reply from {self.url}: {reply!r}")
                if pieces[1] != str(request_id):
                    continue  # a late answer to something we already gave up on
                if pieces[2].upper() == "OK":
                    return _unescape(" ".join(pieces[3:]))
                reason = _unescape(" ".join(pieces[4:]))
                raise GantryError(f"{verb} refused [{pieces[3]}]: {reason}")

    def _ping_loop(self) -> None:
        while not self._closing.wait(PING_INTERVAL_S / 2.0):
            if time.monotonic() - self._last_activity < PING_INTERVAL_S:
                continue
            try:
                self._request("PING")
            except (GantryError, OSError):
                return

    # -- the same vocabulary as Gantry --------------------------------------
    def raw(self, cmd: str, wait: float | None = None) -> str:
        wait = self.cmd_wait if wait is None else wait
        return self._request("CMD", str(int(wait * 1000)), cmd, wait=wait)

    def arm(self) -> None:
        """Unlock motion on the server for this session."""
        self._request("ARM", ARM_ACKNOWLEDGEMENT)

    def stop_all(self) -> str:
        """Priority STOP: the server runs it ahead of anything else queued."""
        self._request("STOP")
        return ""


def micron_per_step(axis: str) -> float:
    return 1000.0 / STEPS_PER_MM[axis]


def resolve_steps(axis: str, mm: float | None, steps: int | None) -> int:
    """Turn --mm or --steps into whole steps. 0 means no move. Raises ValueError
    for a move that rounds to nothing or exceeds the 32-bit register."""
    if mm is not None:
        result = int(round(mm * STEPS_PER_MM[axis]))
        if mm and result == 0:
            raise ValueError(
                f"--mm {mm} on {axis} rounds to 0 steps "
                f"(one step is {micron_per_step(axis):.2f} um)"
            )
    else:
        result = steps or 0
    if abs(result) > POS_LIMIT:
        raise ValueError(f"{result} steps out of 32-bit range")
    return result


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Move one gantry axis by a relative distance (--mm) or step count (--steps)."
    )
    parser.add_argument(
        "--axis",
        required=True,
        type=str.upper,
        choices=AXES,
        help="X, Y or Z. Required: no default, so nothing moves by accident.",
    )
    distance = parser.add_mutually_exclusive_group()
    distance.add_argument("--steps", type=int, help="relative move in steps (signed)")
    distance.add_argument(
        "--mm", type=float, help="relative move in mm (signed), converted with the axis's steps/mm"
    )
    parser.add_argument("--vel", type=int, default=1000, help="steps/s (default 1000)")
    parser.add_argument(
        "--acc", type=int, default=40000, help="steps/s^2 (default 40000, the firmware floor)"
    )
    parser.add_argument(
        "--home-counter", action="store_true", help="SPOS<axis> 0: redefine here as 0 (NO motion)"
    )
    parser.add_argument(
        "--leave-energized",
        action="store_true",
        help="skip MOFF at the end (the axis keeps holding torque)",
    )
    parser.add_argument(
        "--force", action="store_true", help="allow --vel above the proven-clean ceiling"
    )
    where = parser.add_mutually_exclusive_group()
    where.add_argument(
        "--remote",
        metavar="URL",
        help="drive the machine through a gantry_server instead of a local port, "
        "e.g. tcp://10.73.133.1:7313 over a Bluetooth PAN. Requires --acknowledge "
        "for anything that moves.",
    )
    parser.add_argument(
        "--acknowledge",
        action="store_true",
        help="REMOTE MOTION ONLY. Asserts that the workspace is clear and that you or "
        "somebody else can physically reach the power cutoff. Over a remote link you "
        "may not be near the machine, and the power switch is the only real emergency "
        "stop. There is no way to move remotely without this.",
    )
    where.add_argument("--port", default=DEFAULT_PORT, help=f"serial port (default {DEFAULT_PORT})")
    args = parser.parse_args(argv)

    if not VEL_MIN <= args.vel <= VEL_MAX:
        parser.error(f"--vel {args.vel} out of range [{VEL_MIN}, {VEL_MAX}]")
    if not ACC_MIN <= args.acc <= ACC_MAX:
        parser.error(f"--acc {args.acc} out of range [{ACC_MIN}, {ACC_MAX}]")
    ceiling = VEL_CEILING[args.axis]
    if args.vel > ceiling and not args.force:
        parser.error(
            f"--vel {args.vel} is above the proven-clean ceiling for {args.axis} ({ceiling}). "
            f"Too fast stalls the motor and loses position SILENTLY. "
            f"Pass --force if that is deliberate."
        )
    try:
        args.step_count = resolve_steps(args.axis, args.mm, args.steps)
    except ValueError as e:
        parser.error(str(e))
    return args


def wait_until_idle(gantry: Gantry, axis: str, timeout_s: float) -> bool:
    """Poll status bit 0 until the axis is idle. An idle reading during the
    start-up grace only counts if the axis was already seen moving, so a move
    that ends between two polls is not confused with one that never started.
    Returns False on timeout."""
    start = time.monotonic()
    seen_moving = False
    while time.monotonic() - start < timeout_s:
        if gantry.is_moving(axis):
            seen_moving = True
        elif seen_moving or time.monotonic() - start > START_GRACE_S:
            return True
        time.sleep(POLL_S)
    return False


def run_move(gantry: Gantry, args: argparse.Namespace) -> None:
    axis, steps, vel = args.axis, args.step_count, args.vel
    steps_per_mm = STEPS_PER_MM[axis]
    mm = steps / steps_per_mm
    expected_s = abs(steps) / vel

    print(
        f"MON{axis}/ACC/VEL -> {gantry.raw(f'MON{axis}')!r} "
        f"{gantry.raw(f'ACC{axis} {args.acc}')!r} {gantry.raw(f'VEL{axis} {vel}')!r}"
    )
    counter_before = gantry.position(axis)
    print(
        f"\nmoving {axis} by {steps:+d} steps ({mm:+.3f} mm) "
        f"at {vel} steps/s = {vel / steps_per_mm:.1f} mm/s (~{expected_s:.1f}s)"
    )
    if args.mm is not None:
        # Steps are integers, so the commanded distance is almost never exactly
        # the one asked for. Show the residual instead of quietly absorbing it.
        print(
            f"  requested {args.mm:+.3f} mm -> {args.mm * steps_per_mm:+.2f} steps, rounded "
            f"to {steps:+d} ({mm - args.mm:+.4f} mm off; one step on {axis} "
            f"is {micron_per_step(axis):.2f} um)"
        )
    print(f"  counter before: {counter_before}")
    print(f"  POS{axis} -> {gantry.raw(f'POS{axis} {steps}')!r}")

    start = time.monotonic()
    try:
        print(f"  MOVR{axis} -> {gantry.raw(f'MOVR{axis}', wait=0.15)!r}")
        if not wait_until_idle(gantry, axis, expected_s + MOVE_TIMEOUT_MARGIN_S):
            print("  TIMEOUT waiting for the move to finish -> STOPALL")
            gantry.stop_all()
    except KeyboardInterrupt:
        print("\n  ^C -> STOPALL (0.2 s of link lag! The real emergency stop is the power cord.)")
        gantry.stop_all()

    elapsed_s = time.monotonic() - start
    counter_after = gantry.position(axis)
    print(f"  counter after : {counter_after}  (delta {counter_after - counter_before:+d})")
    print(f"  elapsed       : {elapsed_s:.2f}s  (theoretical {expected_s:.2f}s)")
    if not args.leave_energized:
        gantry.raw(f"MOFF{axis}")
        print(f"  MOFF{axis} sent")
    print(f"  positions: {gantry.positions()}")
    back = f"--mm {-args.mm:g}" if args.mm is not None else f"--steps {-steps}"
    print(f"\n  to return: --axis {axis} {back}")


def main() -> None:
    args = parse_args()
    axis = args.axis
    if args.remote and args.step_count and not args.acknowledge:
        raise SystemExit(
            "refusing to move over a remote link without --acknowledge.\n"
            "It asserts that the workspace is clear and that somebody can reach the\n"
            "power cutoff. This machine has no limit switches and no encoders, and\n"
            "over Bluetooth STOP lags 0.3-0.5 s (14-24 mm at VEL 5000 on X/Z).\n"
            "The power switch is the only real emergency stop."
        )
    gantry = RemoteGantry(args.remote) if args.remote else Gantry(args.port)
    with gantry:
        if args.remote:
            print(f"connected to {args.remote}: firmware {gantry.version}")
            print("  the server holds the port: counters were NOT zeroed by connecting")
            print("  position below is what the server has been tracking, not 'zero, here'")
        else:
            print(f"connected: firmware {gantry.version}")
        print(f"JOFF -> {gantry.raw('JOFF')!r}")  # the board boots with the joystick enabled
        if args.remote and args.step_count:
            gantry.arm()
            print("  server session ARMED (workspace clear; power cutoff reachable)")

        if args.home_counter:
            gantry.raw(f"SPOS{axis} 0")
            print(f"SPOS{axis} 0 -> counter here is now 0 (NO motion)")
            print(f"positions: {gantry.positions()}")
            return
        if not args.step_count:
            print(f"positions: {gantry.positions()}")
            return
        run_move(gantry, args)


if __name__ == "__main__":
    try:
        main()
    except GantryError as e:
        raise SystemExit(f"error: {e}") from None
