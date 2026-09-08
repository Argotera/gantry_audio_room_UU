#!/usr/bin/env python3
"""
move.py -- self-contained mover for the gantry in the audio signal room.

It is a DIY machine.  BE VERY CAREFUL!

Brief description, info and gotchas about the machine. Ask Vassilis for details.


Stepper motors: Five identical MAE/AMETEK HY200-2220-072-A4 — NEMA 23, 1.8° per full step, 0.72 A per phase
Gearboxes: Apex Dynamics AN023/PN023 planetary — 10:1 on Y,  5:1 on X and Z.
All gearboxes output to a 19-tooth cog. 5mm pitch belt on all axes.
The steps/mm below are measured values for the complete drive train.

So it is:

X   105.2632 steps per mm
Y   210.5263 steps per mm
Z   105.2632 steps per mm


Controller:
An OES Allegra/ICAD, board ASSY 160005, firmware v11.27, running on an Analog Devices ADSP-2181.
Four-axis capable,fourth axis (W) simply absent. It carries its own microstep driver stage, and its own non-volatile program memory


X axis move the wagon on the bridge (shelves to office doors axis,  North to South)
Y axis move the bridge along th frame ( curtains to anechoic wedges, West to East)
Z axis moves along th eleg, (up - down)


It uses 19200 baudrate

NO encoder ---> therefore the machine cannot report absolute position,
only relative compared to when it was powered on. If it stalls or hits the end, it loses
track.

NO switches at the end of the rails. It will just stall.

the firmware silently CLAMPS out-of-range VEL/ACC instead of rejecting them; most
set commands reply with nothing (empty == success).

Speed should be restricted to <5000, preferably ~4000.

Motion is ALWAYS LIVE! BE CAREFUL.
CTRl+C kills the process but there is a ~0.2 sec delay!
ONLY REAL KILL SWITCH IS YANKING THE POWER CORD!


This script sends only: JOFF, MON, ACC, VEL, POS, MOVR, R, RSTS, MOFF, STOPALL.
Never HOME (no home switches). Never RUN/NEW/SAVE/CONT.


CLI:

    ./move.py --axis x --mm 250                # move +250 mm and STAY
    ./move.py --axis x --mm -250               # send the negative to come back
    ./move.py --axis y --steps -10000 --vel 2000
    ./move.py --axis Z --steps 5000            # raw steps still work
    ./move.py --axis X                         # neither: just report counters
    ./move.py --axis Z --home-counter          # SPOS<ax> 0, redefine here as 0

--axis is required. There is no default, so a forgotten flag cannot move the
wrong axis.
--mm and --steps are the same relative move in different units; give one, not
both. --mm converts with this axis's measured steps/mm (X and Z 105.2632,
Y 210.5263) and rounds to whole steps -- the residual is printed.
 --vel and --acc stay in steps/s; the mm/s equivalent is printed.


"""

import argparse
import time

try:
    import serial
except ImportError as e:
    raise SystemExit("pyserial is required: pip install pyserial") from e

PORT = "/dev/ttyUSB0"  # change to whatever serial is used

# list serials by ls -l /sys/class/tty/*/device/driver

BAUD = 19200  # Bausrate used by the machine
AXES = ("X", "Y", "Z")  # W is absent on this machine

VEL_MIN, VEL_MAX = 200, 200_000  # firmware range (clamped silently)
ACC_MIN, ACC_MAX = 40_000, 40_000_000
POS_LIMIT = 2_147_483_647

# Highest velocity PROVEN clean on each axis.
# Override with --force if you are deliberately testing above these.
VEL_CEILING = {"X": 5000, "Y": 5000, "Z": 5000}

STEPS_PER_MM = {"X": 105.2632, "Y": 210.5263, "Z": 105.2632}
# calculated motor steps to actual physical movement


class GantryError(Exception):
    pass


class Gantry:
    """Minimal RS-232 client: 19200 8N1, CR-terminated, ~100 ms/command,
    RTS (DB9 pin 7) is a hardware RESET that must be pulsed then held CLEAR."""

    def __init__(self, port=PORT, baud=BAUD, cmd_wait=0.20):
        self.port = port
        self.baud = baud
        self.cmd_wait = cmd_wait
        self.ser = None
        self.version = None

    # ----------------------------------------------------------
    def open(self, tries=3):
        """Open the port and run the RESET handshake. The PL2303 throws a
        transient SerialException on open -- retry rather than fail."""
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
                self.ser.dtr = False  # pin 4 is NC on the controller
                self._reset()
                return self
            except (serial.SerialException, OSError) as e:
                last = e
                print(f"open attempt {i + 1}/{tries} failed: {e}")
                try:
                    if self.ser:
                        self.ser.close()
                except Exception:
                    pass
                self.ser = None
                time.sleep(1.0 + i)
        raise GantryError(f"could not open {self.port} after {tries} tries: {last}")

    def _reset(self):
        """RTS SET >=10 ms, then CLEAR and hold clear all session. Leave RTS
        asserted and the board is mute at every baud.
        WARNING: this ZEROES every step counter."""
        self.ser.rts = True
        time.sleep(0.05)
        self.ser.rts = False
        buf = bytearray()  # drain the boot banner
        t0 = last = time.time()
        while time.time() - t0 < 2.5:
            c = self.ser.read(256)
            if c:
                buf.extend(c)
                last = time.time()
                if b"Version" in buf and buf.count(b"\n") >= 2:
                    break
            elif buf and (time.time() - last) > 0.3:
                break
        banner = buf.decode("latin-1", "replace").strip()
        if "Version" in banner:
            try:
                self.version = banner.split("Version", 1)[1].split()[0]
            except IndexError:
                pass

    def close(self):
        if self.ser:
            self.ser.close()
            self.ser = None

    def __enter__(self):
        return self.open()

    def __exit__(self, *exc):
        self.close()

    # ------------------------------------------------------------- low level
    def raw(self, cmd, wait=None):
        """Send one CR-terminated command, return the decoded reply.
        Empty reply == success for most set commands."""
        if not self.ser:
            raise GantryError("port not open")
        wait = self.cmd_wait if wait is None else wait
        self.ser.reset_input_buffer()
        self.ser.write((cmd.strip() + "\r").encode("latin-1"))
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

    def _read_int(self, cmd, prefix=None, tries=4):
        """Numeric read with retry -- a transient empty reply is normal and
        must never abort a move-completion poll."""
        last = None
        for i in range(tries):
            s = self.raw(cmd).strip()
            if prefix and s[:1].upper() == prefix.upper():
                s = s[1:]
            s = s.split()[0] if s.split() else s
            try:
                return int(s.replace("+", ""))
            except ValueError as e:
                last = e
                time.sleep(0.05 * (i + 1))
        raise GantryError(f"{cmd}: no parseable reply after {tries} tries ({last})")

    # -------------------------------------------------------------- commands
    def position(self, axis):
        """R<axis> -> commanded step counter (NOT where the axis actually is)."""
        return self._read_int(f"R{axis}", prefix=axis)

    def positions(self):
        return {a: self.position(a) for a in AXES}

    def is_moving(self, axis):
        """RSTS<axis> bit 0. Works with MSGOFF. Do not poll faster than ~20 Hz."""
        return bool(self._read_int(f"RSTS{axis}") & 0x1)

    def stop_all(self):
        return self.raw("STOPALL")


def main():
    ap = argparse.ArgumentParser(
        description="Move one gantry axis by a relative distance (--mm) or step count (--steps)."
    )
    ap.add_argument(
        "--axis",
        required=True,
        type=str.upper,
        choices=AXES,
        help="X, Y or Z. Required: no default, so nothing moves by accident.",
    )
    ap.add_argument("--steps", type=int, default=0, help="relative steps (signed)")
    ap.add_argument(
        "--mm",
        type=float,
        default=None,
        help="relative distance in mm (signed) -- converted with this "
        "axis's steps/mm. Use instead of --steps, not with it.",
    )
    ap.add_argument("--vel", type=int, default=1000, help="steps/s")
    ap.add_argument("--acc", type=int, default=40000, help="steps/s^2")
    ap.add_argument(
        "--home-counter", action="store_true", help="SPOS<axis> 0: redefine here as 0 (NO motion)"
    )
    ap.add_argument(
        "--leave-energized",
        action="store_true",
        help="skip MOFF at the end (axis keeps holding torque)",
    )
    ap.add_argument(
        "--force", action="store_true", help="allow --vel above the proven-clean ceiling"
    )
    ap.add_argument("--port", default=PORT)
    a = ap.parse_args()

    AX = a.axis

    # --mm is the same relative move expressed in millimetres. Resolve it to
    # steps here so everything downstream deals in steps only.
    if a.mm is not None and a.steps:
        raise SystemExit("give either --steps or --mm, not both")
    if a.mm is not None:
        exact = a.mm * STEPS_PER_MM[AX]
        steps = int(round(exact))
        if a.mm and steps == 0:
            raise SystemExit(
                f"--mm {a.mm} on {AX} rounds to 0 steps "
                f"(one step is {1000 / STEPS_PER_MM[AX]:.2f} um)"
            )
    else:
        steps = a.steps

    if abs(steps) > POS_LIMIT:
        raise SystemExit(f"{steps} steps out of 32-bit range")
    if not (VEL_MIN <= a.vel <= VEL_MAX):
        raise SystemExit(f"--vel {a.vel} out of range [{VEL_MIN}, {VEL_MAX}]")
    if not (ACC_MIN <= a.acc <= ACC_MAX):
        raise SystemExit(f"--acc {a.acc} out of range [{ACC_MIN}, {ACC_MAX}]")
    cap = VEL_CEILING[AX]
    if a.vel > cap and not a.force:
        raise SystemExit(
            f"--vel {a.vel} is above the proven-clean ceiling for {AX} ({cap}).\n"
            f"Too fast stalls the motor and loses position SILENTLY. "
            f"Pass --force if that is deliberate."
        )

    with Gantry(a.port) as g:
        print(f"connected: firmware {g.version}")
        print(f"JOFF -> {g.raw('JOFF')!r}")  # board boots joystick-ENABLED

        if a.home_counter:
            g.raw(f"SPOS{AX} 0")
            print(f"SPOS{AX} 0 -> counter here is now 0 (NO motion)")
            print(f"positions: {g.positions()}")
            return

        if not steps:
            print(f"positions: {g.positions()}")
            return

        mm = steps / STEPS_PER_MM[AX]
        est = abs(steps) / a.vel
        print(
            f"MON{AX}/ACC/VEL -> {g.raw(f'MON{AX}')!r} "
            f"{g.raw(f'ACC{AX} {a.acc}')!r} {g.raw(f'VEL{AX} {a.vel}')!r}"
        )
        p0 = g.position(AX)
        print(
            f"\nmoving {AX} by {steps:+d} steps ({mm:+.3f} mm) "
            f"at {a.vel} steps/s = {a.vel / STEPS_PER_MM[AX]:.1f} mm/s "
            f"(~{est:.1f}s)"
        )
        if a.mm is not None:
            # Steps are integers, so the commanded distance is almost never
            # exactly the one asked for. Show the residual instead of quietly
            # absorbing it.
            print(
                f"  requested {a.mm:+.3f} mm -> {exact:+.2f} steps, rounded "
                f"to {steps:+d} ({mm - a.mm:+.4f} mm off; one step on {AX} "
                f"is {1000 / STEPS_PER_MM[AX]:.2f} um)"
            )
        print(f"  counter before: {p0}")
        print(f"  POS{AX} -> {g.raw(f'POS{AX} {steps}')!r}")

        t0 = time.time()
        try:
            print(f"  MOVR{AX} -> {g.raw(f'MOVR{AX}', wait=0.15)!r}")
            # Poll bit 0 until idle. Wait out a start-up grace period first so a
            # move that finishes between polls isn't mistaken for one that never
            # started. Then break on the first idle reading.
            seen = False
            while time.time() - t0 < est + 30:
                if g.is_moving(AX):
                    seen = True
                elif seen or (time.time() - t0) > 1.0:
                    break
                time.sleep(0.05)  # >=20 Hz or replies get dropped
            else:
                print("  TIMEOUT waiting for the move to finish -> STOPALL")
                g.stop_all()
        except KeyboardInterrupt:
            print(
                "\n  ^C -> STOPALL "
                "(0.2s of link latency! The real Emergency stop is the power switch!!!)"
            )
            g.stop_all()

        dt = time.time() - t0
        p1 = g.position(AX)
        print(f"  counter after : {p1}  (delta {p1 - p0:+d})")
        print(f"  elapsed       : {dt:.2f}s  (theoretical {est:.2f}s)")
        if not a.leave_energized:
            g.raw(f"MOFF{AX}")
            print(f"  MOFF{AX} sent")
        print(f"  positions: {g.positions()}")
        back = f"--mm {-a.mm:g}" if a.mm is not None else f"--steps {-steps}"
        print(f"\n  to return: --axis {AX} {back}")


if __name__ == "__main__":
    try:
        main()
    except GantryError as e:
        raise SystemExit(f"error: {e}") from None
