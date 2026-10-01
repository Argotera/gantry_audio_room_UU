#!/usr/bin/env python3
"""
gantry_gui.py -- manual control panel for the OES gantry robot in the robot lab.

  python3 gantry_gui.py                                      (asks where to connect)
  python3 gantry_gui.py --port /dev/ttyUSB0                  default for a linux
  python3 gantry_gui.py --remote tcp://10.73.133.1:7313      (over Bluetooth to the rpi in the lab)

With neither flag, a chooser lists the USB serial adapters it can see, plus a
"Bluetooth gantry" entry when this machine is actually on the Pi's Bluetooth
network. Pass a flag to skip it. The chooser runs once, before connecting:
there is no way to change target without restarting, on purpose -- see
TargetChooser.

Position display, typed mm moves, set-origin-here, a big red STOP, and a
polar (r, phi) move panel for the X/Y plane.

NOTES
* ONE thread owns the serial port (the worker). The Tk thread never touches it.
  Two processes or two threads on the port corrupt the protocol.
* Connecting to a LOCAL port RESETS the controller and ZEROES all three
  counters, so the origin is wherever the machine happens to be standing.
  Connecting to a REMOTE server does NOT: the server holds the port open, so
  the counters survive a reconnect. Check the log line at start-up, which says
  which of the two happened.

REMOTE MODE
* Starts DISARMED. Untick "dry run" to arm, which asks you to confirm that the
  workspace is clear and somebody can reach the power cutoff. That question is
  the point: over Bluetooth you may not be standing next to the machine, and
  the power switch is the only real emergency stop.
* If the link drops, the server stops the machine by itself and ends the
  session. The GUI latches STOP when that happens.
* The machine has no limit switches, no home switches and no encoders, so
  "position" only ever means "distance from where the machine stood at connect,
  or from where SET ORIGIN was last pressed".
* The STOP button is NOT an emergency stop. STOP is slower over Bluetooth ~0.3-0.5 s
  rather than 0.2 s, which at VEL 5000 on X or Z is 14-24 mm of travel.
* HOME is never sent. There are no home switches. oes.py refuses it
  (home_switches=False) and RUN/NEW/SAVE/CONT as well.

POLAR MOVES (the panel on the right)
  r   distance from the origin, mm (>= 0)
  phi angle in degrees, -90 .. +90

  x = r * sin(phi)        phi = 0   -> straight along -Y
  y = -r * cos(phi)       phi > 0   -> toward +X (north)
                          phi < 0   -> toward -X
  so tan(phi) = x / |y|, and every target lies on the -Y side (y <= 0).

* The polar target is ABSOLUTE from the origin (SET ORIGIN HERE or the connect
  reset), not relative to where the machine stands.
* Path: Y first, then X -- two single-axis MOVA moves, an L-shaped path, NOT
  a straight line. The top-view plot draws it before you press GO.
  Future option: start X and Y together (needs two-axis polling, untested).
* The X leg only runs if the Y counter reached its target. A STOP, a timeout or
  a lost reply between the legs cancels X.
* Z is never commanded by a polar move.




USAGE:

Connect the raspberry pi to the USB-RS232 adapter connected to the controller's box
Power up the Pi and the controller. The gantry server on the Pi should start by itself at boot.
Make sure your computer has a working python environment with packages: pyserial, tkinter

Linux:
Find and pair bluetooth device named rpi-gen1
join the Pi's Bluetooth network:

nmcli con up rpi-gen1-bt

then launch gui

python3 gantry_gui.py --remote tcp://10.73.133.1:7313
or just
python3 gantry_gui.py and pick up gantry from start-up window

When you're done, close the GUI, then run:
nmcli con down rpi-gen1-bt



Windows:

Pair with the Pi:
Settings → Bluetooth & devices → Add device → Bluetooth → rpi-gen1.

Join the Pi's network:
   a. Go to Settings → Bluetooth & devices → Devices.
   b. Under "Other devices", expand rpi-gen1.
   c. Next to Personal Area Network (PAN), click Join.
   d. Keep Access Point selected, click Connect, and wait for "Connection successful".

py gantry_gui.py --remote tcp://10.73.133.1:7313
or just
py gantry_gui.py and pick up gantry from the start-up window

When you're done, close the GUI, then go back to the same Settings screen and click Disconnect PAN.



"""

from __future__ import annotations

import argparse
import contextlib
import logging
import math
import queue
import socket
import threading
import time
import tkinter as tk
from dataclasses import dataclass
from tkinter import messagebox, ttk
from typing import Callable

from oes import MoveResult, OESController, OESError

log = logging.getLogger("gantry_gui")

# --- machine constants (measured; see the machine documentation)
STEPS_PER_MM = {"X": 105.2632, "Y": 210.5263, "Z": 105.2632}
MAX_VEL = {"X": 5000, "Y": 5000, "Z": 5000}
DEFAULT_VEL = 4000
ACC = 40000  # firmware floor; anything lower is silently clamped
CONFIRM_OVER_MM = 500.0  # moves longer than this ask for confirmation
AXES = ("X", "Y", "Z")

CLOSE_TIMEOUT_S = 6.0  # max wait for STOPALL + MOFF x3 + port close on exit
MOVE_TIMEOUT_S = 600.0  # give up waiting for a move after this
IDLE_REFRESH_S = 1.0  # position refresh interval while nothing is queued
DRAIN_INTERVAL_MS = 50  # how often the Tk thread empties the event queue

# Poll interval while waiting for a move to finish. Every poll is a round trip,
# which is ~1 ms on a local UART and tens of milliseconds over Bluetooth and
# several hundred if the link has dropped into sniff mode. Polling a remote link
# as hard as a local one just floods it.
MOVE_POLL_S = 0.1
MOVE_POLL_REMOTE_S = 0.25

# --- the Bluetooth gantry server, as deployed ------------------------------
# Hard-coded on purpose: there is one Pi, at a fixed address on its own PAN.
#  These MUST match the Pi's bt-pan-nap address and its gantry_server.py
# (DEFAULT_TCP_PORT). Change one and you must change the others.
BT_GANTRY_HOST = "10.73.133.1"
BT_GANTRY_PORT = 7313
BT_GANTRY_LABEL = f"Bluetooth gantry ({BT_GANTRY_HOST}:{BT_GANTRY_PORT})"

#: Shown before arming a remote session. Not a formality: over Bluetooth the
#: operator may be nowhere near the machine, and the power switch is the only
#: real emergency stop there is.
REMOTE_ARM_QUESTION = (
    "You are about to arm the gantry over a REMOTE link.\n\n"
    "This machine has no limit switches, no home switches and no encoders. "
    "Nothing detects an overrun, a stall or a crash.\n\n"
    "STOP is not an emergency stop, and over Bluetooth it is slower: "
    "0.3-0.5 s, about 14-24 mm at VEL 5000.\n"
    "THE ONLY EMERGENCY STOP IS THE POWER SWITCH.\n\n"
    "Confirm BOTH of the following:\n"
    "  \u2022 the workspace is clear, and\n"
    "  \u2022 you or somebody else can physically reach the power cutoff.\n\n"
    "Arm the machine?"
)

BG, FG, DIM = "#1e1e1e", "#e8e8e8", "#8a8a8a"
RED, RED_HOT, GREEN, AMBER = "#c0392b", "#e74c3c", "#27ae60", "#d4952a"
ENTRY_BG, LOG_BG, STOPPED_RED = "#2b2b2b", "#141414", "#7a1c12"

# --- polar panel
PHI_MIN, PHI_MAX = -90.0, 90.0
PLOT_W, PLOT_H = 380, 320
PLOT_MIN_EXTENT_MM = 100.0  # smallest half-width the plot zooms in to


# ------------------------------------------------------------------ polar ----
def polar_to_xy(r_mm: float, phi_deg: float) -> tuple[float, float]:
    """(r, phi) -> (x, y) in mm. phi = 0 on -Y, positive toward +X."""
    phi = math.radians(phi_deg)
    return r_mm * math.sin(phi), -r_mm * math.cos(phi)


def xy_to_polar(x_mm: float, y_mm: float) -> tuple[float, float]:
    """(x, y) -> (r, phi_deg). Inverse of polar_to_xy; |phi| > 90 means y > 0."""
    return math.hypot(x_mm, y_mm), math.degrees(math.atan2(x_mm, -y_mm))


# ------------------------------------------------------------- connection ----
@dataclass(frozen=True)
class Target:
    """One entry in the startup chooser."""

    label: str  # what the operator sees
    target: str  # what OESController is given
    usb: bool = False  # a USB serial adapter, as opposed to a motherboard port


def gantry_pan_is_up(host: str = BT_GANTRY_HOST, port: int = BT_GANTRY_PORT) -> bool:
    """True if this machine is on the gantry Pi's Bluetooth PAN.

    Plain reachability is NOT the test, and getting this wrong would offer
    the Bluetooth entry on every machine. With a default route present the
    kernel will happily claim it can reach 10.73.133.1 through the ordinary
    gateway.
    """
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as probe:
            probe.connect((host, port))
            local = probe.getsockname()[0]
    except OSError:
        return False
    subnet = host.rsplit(".", 1)[0] + "."  # "10.73.133.1" -> "10.73.133."
    return local.startswith(subnet)


def serial_ports() -> list[object]:
    """Serial ports pyserial can see. Never raises: a machine with no working
    pyserial should still be offered the Bluetooth entry."""
    try:
        from serial.tools import list_ports

        return list(list_ports.comports())
    except Exception as e:  # the remote entry still matters
        log.warning("could not enumerate serial ports: %s", e)
        return []


def list_targets(
    all_ports: bool = False,
    ports: list[object] | None = None,
    pan_up: bool | None = None,
) -> list[Target]:
    """Everything the panel could connect to right now.

    Non-USB ports are hidden unless `all_ports`. The gantry is always on a
    USB-RS232 adapter.
    """
    ports = serial_ports() if ports is None else list(ports)
    pan = gantry_pan_is_up() if pan_up is None else pan_up

    usb: list[Target] = []
    other: list[Target] = []
    for port in sorted(ports, key=lambda p: p.device):
        description = (port.description or "").strip()
        label = port.device
        if description and description.lower() != "n/a":
            label = f"{port.device}  --  {description}"
        is_usb = port.vid is not None
        (usb if is_usb else other).append(Target(label, port.device, usb=is_usb))

    targets = usb
    if pan:
        targets.append(Target(BT_GANTRY_LABEL, f"tcp://{BT_GANTRY_HOST}:{BT_GANTRY_PORT}"))
    return targets + (other if all_ports else [])


def default_target(targets: list[Target]) -> int:
    """Which entry to pre-select.
    Already sketchy machine, at least prefer USB
    """
    for index, target in enumerate(targets):
        if target.usb:
            return index
    return 0


class TargetChooser:
    """Startup dialog: pick a serial port, or the Bluetooth gantry.
    """

    def __init__(self, root: tk.Tk) -> None:
        self.root = root
        self.choice: str | None = None
        self.targets: list[Target] = []

        root.title("Connect to the gantry")
        root.configure(bg=BG)
        frame = tk.Frame(root, bg=BG)
        frame.pack(padx=16, pady=14, fill="both", expand=True)

        tk.Label(
            frame,
            text="Where is the gantry?",
            bg=BG,
            fg=FG,
            font=("TkDefaultFont", 12, "bold"),
        ).pack(anchor="w")

        self.combo = ttk.Combobox(frame, state="readonly", width=52, font=("TkFixedFont", 10))
        self.combo.pack(pady=(10, 4), fill="x")
        self.combo.bind("<<ComboboxSelected>>", lambda _e: self._show_target())
        self.combo.bind("<Return>", lambda _e: self.on_connect())

        self.detail = tk.Label(frame, text="", bg=BG, fg=DIM, anchor="w", font=("TkFixedFont", 9))
        self.detail.pack(fill="x")

        self.all_ports = tk.BooleanVar(value=False)
        tk.Checkbutton(
            frame,
            text="show every serial port (most are motherboard ports with nothing on them)",
            variable=self.all_ports,
            command=self.refresh,
            bg=BG,
            fg=DIM,
            selectcolor=ENTRY_BG,
            activebackground=BG,
            activeforeground=DIM,
            highlightthickness=0,
            anchor="w",
        ).pack(fill="x", pady=(8, 0))

        buttons = tk.Frame(frame, bg=BG)
        buttons.pack(fill="x", pady=(14, 0))
        ttk.Button(buttons, text="Rescan", command=self.refresh).pack(side="left")
        ttk.Button(buttons, text="Cancel", command=self.on_cancel).pack(side="right")
        self.connect_btn = ttk.Button(buttons, text="Connect", command=self.on_connect)
        self.connect_btn.pack(side="right", padx=6)

        root.bind("<Escape>", lambda _e: self.on_cancel())
        root.protocol("WM_DELETE_WINDOW", self.on_cancel)
        self.refresh()

    def refresh(self) -> None:
        self.targets = list_targets(all_ports=self.all_ports.get())
        self.combo["values"] = [t.label for t in self.targets] or ["(nothing found)"]
        self.combo.current(default_target(self.targets) if self.targets else 0)
        self.connect_btn.state(["!disabled"] if self.targets else ["disabled"])
        self._show_target()

    def _show_target(self) -> None:
        if not self.targets:
            hint = "" if self.all_ports.get() else "  Tick the box below to see every port."
            self.detail.configure(
                text=f"No USB adapter, and not on the gantry's Bluetooth network.{hint}"
            )
            return
        target = self.targets[self.combo.current()]
        self.detail.configure(text=f"opens: {target.target}")

    def on_connect(self) -> None:
        if self.targets:
            self.choice = self.targets[self.combo.current()].target
            self.root.quit()

    def on_cancel(self) -> None:
        self.choice = None
        self.root.quit()


def ask_for_target(root: tk.Tk) -> str | None:
    """Run the chooser on `root` and return the selected target, or None."""
    chooser = TargetChooser(root)
    root.mainloop()
    for child in root.winfo_children():  # clear the dialog, keep the window
        child.destroy()
    for sequence in ("<Escape>",):
        root.unbind(sequence)
    return chooser.choice


# ----------------------------------------------------------------- worker ----
class Worker(threading.Thread):
    """Owns the serial port. Everything else talks to it through two queues:
    `commands` in from the Tk thread, `events` out to the Tk thread."""

    def __init__(self, events: queue.Queue, port: str) -> None:
        super().__init__(daemon=True)
        self.events = events
        self.port = port
        #: A transport URL rather than a device path means we reach the machine
        #: through a gantry_server, normally over Bluetooth.
        self.remote = "://" in port
        self.commands: queue.Queue = queue.Queue()
        self.abort = threading.Event()  # set by STOP, from any thread
        self.quit = threading.Event()
        self.controller: OESController | None = None
        # ACC and VEL persist in the controller until the next reset (verified on
        # hardware 2026-09-08), so send ACC once per axis and VEL only when it
        # changes. Cleared on every STOP as cheap insurance.
        self.acc_sent: set[str] = set()
        self.vel_sent: dict[str, int] = {}
        self._handlers: dict[str, Callable[..., None]] = {
            "stop": self._do_stop,
            "arm": self._do_arm,
            "zero": self._do_zero,
            "move": self._do_move,
            "polar": self._do_polar,
        }

    # -- helpers ------------------------------------------------------------
    @property
    def ctl(self) -> OESController:
        if self.controller is None:
            raise OESError("not connected")
        return self.controller

    def emit(self, kind: str, payload: object) -> None:
        self.events.put((kind, payload))

    def log(self, msg: str, tag: str = "info") -> None:
        self.emit("log", (msg, tag))

    def push_positions(self) -> None:
        started = time.monotonic()
        try:
            self.emit("pos", {axis: self.ctl.report_position(axis) for axis in AXES})
        except OESError as e:
            self.log(f"position read failed: {e}", "warn")
            return
        if self.remote:
            # Three round trips; report the per-command average so the number
            # means "what one command costs", which is what STOP lag depends on.
            self.emit("link", (time.monotonic() - started) / len(AXES))

    def on_server_event(self, name: str, payload: str) -> None:
        """Unsolicited news from the server. Called on whichever thread is
        reading the link, so it only ever posts to the event queue.
        The important one is the watchdog.
        """
        self.log(f"SERVER {name}: {payload}", "err")
        if name in ("watchdog", "goodbye"):
            self.abort.set()
            self.emit("linkdown", f"{name}: {payload}")

    # -- public API (called from the Tk thread) -----------------------------
    def submit(self, kind: str, **kwargs: object) -> None:
        self.commands.put((kind, kwargs))

    def kill(self) -> None:
        """Set the abort flag and queue a "stop". Safe from any thread.

        The queue is FIFO, so "stop" does not overtake queued moves. The abort
        flag is what makes queued moves refuse to run and makes a move in
        progress break out of its poll loop and STOP."""
        self.abort.set()
        self.commands.put(("stop", {}))

    # -- main loop ----------------------------------------------------------
    def run(self) -> None:
        logging.getLogger("oes").addHandler(_LogToGui(self))
        try:
            options: dict[str, object] = {}
            if self.remote:
                options = {"client_name": "gantry_gui", "on_event": self.on_server_event}
            self.controller = OESController(
                port=self.port, verbose=False, transport_options=options
            ).open()
            self.ctl.joystick(False)
            self.log(f"connected to {self.ctl.target}, firmware {self.ctl.version}")
            if self.ctl.transport.resets_on_open:
                self.log("counters zeroed by the connect reset -- origin is HERE")
            else:
                self.log("server held the port open: counters were NOT zeroed", "ok")
                self.log("position is whatever the server has been tracking, not 'here'", "warn")
            if self.remote:
                # Remote sessions start disarmed.
                self.emit("status", ("DRY RUN  ·  NOT ARMED -- untick 'dry run' to arm", AMBER))
                self.log("remote session: DISARMED until you untick 'dry run'", "warn")
            else:
                # The driver starts in dry run. The local panel is live from the start.
                self.ctl.arm(confirm=True)
                self.emit("status", ("ARMED   ·  motion is LIVE", GREEN))
                self.log("ARMED -- motion commands are transmitted for real", "ok")
            self.push_positions()
        except Exception as e:
            self.emit("status", (f"NOT CONNECTED: {e}", RED_HOT))
            self.log(f"connect failed: {e}", "err")
            self.log(f"is another process holding {self.port}?", "warn")
            return

        last_refresh = 0.0
        while not self.quit.is_set():
            try:
                kind, kwargs = self.commands.get(timeout=0.25)
            except queue.Empty:
                if time.monotonic() - last_refresh > IDLE_REFRESH_S:
                    self.push_positions()
                    last_refresh = time.monotonic()
                continue
            try:
                self._handlers[kind](**kwargs)
            except OESError as e:
                self.log(f"{kind} failed: {e}", "err")
            except Exception as e:
                self.log(f"{kind} crashed: {type(e).__name__}: {e}", "err")
            last_refresh = 0.0

        with contextlib.suppress(Exception):
            self._shutdown()

    def _shutdown(self) -> None:
        """STOPALL, de-energize every axis, release the port."""
        self.ctl.stop()
        for axis in AXES:
            self.ctl.motor_off(axis)
        self.ctl.close()

    def poll_interval(self) -> float:
        return MOVE_POLL_REMOTE_S if self.remote else MOVE_POLL_S

    # -- commands -----------------------------------------------------------
    def _do_stop(self) -> None:
        """STOPALL + de-energize. Always safe, always allowed."""
        try:
            self.ctl.stop()
        finally:
            for axis in AXES:
                with contextlib.suppress(OESError):
                    self.ctl.motor_off(axis)
        while not self.commands.empty():  # drop queued motion
            try:
                self.commands.get_nowait()
            except queue.Empty:
                break
        self.acc_sent.clear()
        self.vel_sent.clear()
        self.log("STOP: STOPALL sent, all axes de-energized", "err")
        self.push_positions()
        self.emit("busy", False)

    def _do_arm(self, live: bool) -> None:
        firmware = self.ctl.version
        if live:
            self.ctl.arm(confirm=True)
            self.emit("status", (f"ARMED  ·  firmware {firmware}  ·  motion is LIVE", GREEN))
            self.log("ARMED -- motion commands are transmitted for real", "ok")
        else:
            self.ctl.disarm()
            self.emit("status", (f"DRY RUN  ·  firmware {firmware}  ·  motion is SIMULATED", AMBER))
            self.log("DRY RUN -- moves are simulated, the machine will NOT move", "warn")

    def _do_zero(self, axes: list[str]) -> None:
        for axis in axes:
            self.ctl.redefine_position(axis, 0)
        self.log(f"origin set here for {'/'.join(axes)} (SPOS 0 -- no motion)", "ok")
        self.push_positions()

    def _do_move(self, axis: str, steps: int, vel: int, absolute: bool) -> None:
        if self.abort.is_set():
            self.log("move refused: STOP is latched -- press RESET STOP", "warn")
            self.emit("busy", False)
            return
        mm = steps / STEPS_PER_MM[axis]
        self.emit("busy", True)
        try:
            result = self._send_move(axis, steps, vel, absolute)
            if absolute:
                self.log(f"{axis} -> {mm:+.3f} mm  (abs, VEL {vel})")
            else:
                self.log(f"{axis} {mm:+.3f} mm  (rel, VEL {vel})")
            if result.dry_run:
                self.log("DRY RUN -- nothing was sent; the machine will not move", "warn")
                return
            self._wait_move(axis)
        finally:
            self.emit("busy", False)

    def _do_polar(self, x_steps: int, y_steps: int, vel_x: int, vel_y: int) -> None:
        """MOVA Y, wait, check the counter, then MOVA X, wait. X only if Y landed."""
        if self.abort.is_set():
            self.log("polar move refused: STOP is latched -- press RESET STOP", "warn")
            self.emit("busy", False)
            return
        self.emit("busy", True)
        try:
            for axis, target, vel in (("Y", y_steps, vel_y), ("X", x_steps, vel_x)):
                if self.abort.is_set():
                    self.log(f"polar move: STOP pressed, {axis} leg not started", "err")
                    return
                if not self._polar_leg(axis, target, vel):
                    if axis == "Y":
                        self.log("polar move: X leg cancelled", "err")
                    return
            self.log("polar move complete", "ok")
        finally:
            self.emit("busy", False)

    def _polar_leg(self, axis: str, target: int, vel: int) -> bool:
        """One absolute single-axis move. True if the counter ended on target
        (or in dry run, where nothing moves)."""
        result = self._send_move(axis, target, vel, absolute=True)
        self.log(f"{axis} -> {target / STEPS_PER_MM[axis]:+.3f} mm  (abs, VEL {vel})")
        if result.dry_run:
            self.log("DRY RUN -- nothing was sent; the machine will not move", "warn")
            return True
        self._wait_move(axis)
        if self.abort.is_set():
            return False
        now = self.ctl.report_position(axis)
        if now != target:
            self.log(f"{axis} counter reads {now}, expected {target} -- not on target", "err")
            return False
        return True

    def _send_move(self, axis: str, steps: int, vel: int, absolute: bool) -> MoveResult:
        """MOVA or MOVR, with ACC sent once per axis and VEL only when it changed.
        The cache is updated only when the move was transmitted, not in dry run."""
        acc = None if axis in self.acc_sent else ACC
        vel_to_send = None if self.vel_sent.get(axis) == vel else vel
        move = self.ctl.move_absolute if absolute else self.ctl.move_relative
        result = move(axis, steps, vel=vel_to_send, acc=acc)
        if not result.dry_run:
            self.acc_sent.add(axis)
            self.vel_sent[axis] = vel
        return result

    def _wait_move(self, axis: str, timeout: float = MOVE_TIMEOUT_S) -> None:
        """Block until the move ends, through the driver's wait loop. Between
        polls the callback checks the STOP flag and refreshes the position."""

        def between_polls() -> bool:
            if self.abort.is_set():
                self.ctl.stop()
                self.log(f"{axis} aborted mid-move", "err")
                return True
            with contextlib.suppress(OESError):  # transient; the status poll decides
                self.emit("pos", {axis: self.ctl.report_position(axis)})
            return False

        try:
            done = self.ctl.wait_stopped(
                axis, timeout=timeout, poll=self.poll_interval(), on_poll=between_polls
            )
        except OESError as e:
            self.log(f"{axis}: lost contact during the move ({e}) -- STOPALL", "err")
            self.ctl.stop()
            done = False
        if not done and not self.abort.is_set():
            self.log(f"{axis} still moving after {timeout:.0f}s -- stopping", "err")
            self.ctl.stop()
        self.push_positions()


class _LogToGui(logging.Handler):
    """Forwards the driver's warnings into the GUI log box."""

    def __init__(self, worker: Worker) -> None:
        super().__init__(level=logging.WARNING)
        self.worker = worker

    def emit(self, record: logging.LogRecord) -> None:
        self.worker.log(f"driver: {record.getMessage()}", "warn")


# --------------------------------------------------------------------- UI ----
class App:
    def __init__(self, root: tk.Tk, port: str) -> None:
        self.root = root
        self.events: queue.Queue = queue.Queue()
        self.worker = Worker(self.events, port)
        self.remote = self.worker.remote
        self.steps = {axis: 0 for axis in AXES}  # last known counters
        self.busy = False
        self.status_base = "connecting…"
        self.link_text = ""

        root.title("OES Gantry — remote" if self.remote else "OES Gantry — manual + polar control")
        root.configure(bg=BG)
        root.protocol("WM_DELETE_WINDOW", self.on_close)
        root.bind("<Escape>", lambda _event: self.on_kill())

        self._build()
        self._build_polar()
        self._redraw_polar()
        self.worker.start()
        self.root.after(DRAIN_INTERVAL_MS, self._drain)

    # -- layout -------------------------------------------------------------
    def _build(self) -> None:
        pad = dict(padx=8, pady=4)
        root = self.root

        self.status = tk.Label(
            root,
            text="connecting…",
            bg=BG,
            fg=AMBER,
            font=("TkDefaultFont", 11, "bold"),
            anchor="w",
        )
        self.status.grid(row=0, column=0, columnspan=6, sticky="we", **pad)

        headers = ("axis", "position", "steps", "mm", "", "")
        for col, title in enumerate(headers):
            tk.Label(root, text=title, bg=BG, fg=DIM).grid(row=1, column=col, sticky="w", padx=8)

        self.pos_label: dict[str, tk.Label] = {}
        self.step_label: dict[str, tk.Label] = {}
        self.entry: dict[str, tk.Entry] = {}
        for row, axis in enumerate(AXES, start=2):
            tk.Label(root, text=axis, bg=BG, fg=FG, font=("TkFixedFont", 16, "bold")).grid(
                row=row, column=0, **pad
            )

            self.pos_label[axis] = tk.Label(
                root,
                text="+0.000 mm",
                bg=BG,
                fg=GREEN,
                font=("TkFixedFont", 18),
                width=13,
                anchor="e",
            )
            self.pos_label[axis].grid(row=row, column=1, **pad)

            self.step_label[axis] = tk.Label(
                root, text="0", bg=BG, fg=DIM, font=("TkFixedFont", 10), width=10, anchor="e"
            )
            self.step_label[axis].grid(row=row, column=2, **pad)

            entry = tk.Entry(
                root,
                width=10,
                justify="right",
                bg=ENTRY_BG,
                fg=FG,
                insertbackground=FG,
                font=("TkFixedFont", 13),
            )
            entry.grid(row=row, column=3, **pad)
            entry.bind("<Return>", lambda _event, ax=axis: self.on_move(ax, absolute=False))
            self.entry[axis] = entry

            ttk.Button(
                root,
                text="Move ±",
                width=8,
                command=lambda ax=axis: self.on_move(ax, absolute=False),
            ).grid(row=row, column=4, **pad)
            ttk.Button(
                root, text="Go to", width=7, command=lambda ax=axis: self.on_move(ax, absolute=True)
            ).grid(row=row, column=5, **pad)

        # velocity + dry run
        bar = tk.Frame(root, bg=BG)
        bar.grid(row=5, column=0, columnspan=6, sticky="we", padx=8, pady=(10, 2))
        tk.Label(bar, text="velocity", bg=BG, fg=DIM).pack(side="left")
        self.vel = tk.Entry(
            bar,
            width=7,
            justify="right",
            bg=ENTRY_BG,
            fg=FG,
            insertbackground=FG,
            font=("TkFixedFont", 12),
        )
        self.vel.insert(0, str(DEFAULT_VEL))
        self.vel.pack(side="left", padx=6)
        tk.Label(
            bar,
            text=f"steps/s   proven max  X {MAX_VEL['X']} · Y {MAX_VEL['Y']} · Z {MAX_VEL['Z']}",
            bg=BG,
            fg=DIM,
        ).pack(side="left")

        # Remote sessions start DISARMED.
        self.simulate = tk.BooleanVar(value=self.remote)
        tk.Checkbutton(
            bar,
            text="dry run (simulate, do not move)",
            variable=self.simulate,
            command=self.on_toggle_dry,
            bg=BG,
            fg=AMBER,
            selectcolor=ENTRY_BG,
            activebackground=BG,
            activeforeground=AMBER,
            highlightthickness=0,
        ).pack(side="right")

        ttk.Button(root, text="SET ORIGIN HERE  (zero all axes)", command=self.on_zero_all).grid(
            row=6, column=0, columnspan=6, sticky="we", padx=8, pady=6
        )

        # kill switch
        self.kill_btn = tk.Button(
            root,
            text="■  S T O P  ■",
            command=self.on_kill,
            bg=RED,
            fg="white",
            activebackground=RED_HOT,
            activeforeground="white",
            relief="raised",
            bd=5,
            font=("TkDefaultFont", 22, "bold"),
            height=2,
        )
        self.kill_btn.grid(row=7, column=0, columnspan=6, sticky="we", padx=8, pady=(12, 2))

        tk.Label(root, text=self.stop_note(), bg=BG, fg=AMBER, justify="left", anchor="w").grid(
            row=8, column=0, columnspan=6, sticky="we", padx=8
        )

        self.reset_btn = ttk.Button(
            root, text="reset STOP latch", command=self.on_reset_latch, state="disabled"
        )
        self.reset_btn.grid(row=9, column=0, columnspan=6, sticky="we", padx=8, pady=4)

        self.log_box = tk.Text(
            root,
            height=11,
            bg=LOG_BG,
            fg=FG,
            wrap="word",
            font=("TkFixedFont", 10),
            relief="flat",
        )
        self.log_box.grid(row=10, column=0, columnspan=6, sticky="nsew", padx=8, pady=8)
        for tag, colour in (("info", DIM), ("ok", GREEN), ("warn", AMBER), ("err", RED_HOT)):
            self.log_box.tag_config(tag, foreground=colour)
        self.log_box.configure(state="disabled")
        root.grid_rowconfigure(10, weight=1)
        root.grid_columnconfigure(1, weight=1)

    def stop_note(self) -> str:
        if self.remote:
            return (
                "NOT an emergency stop!!!  (Bluetooth link + 19200 baud serial)\n"
                "~0.3-0.5 s lag (~14-24 mm at VEL 5000 on X/Z).\n"
                "For a real emergency, CUT THE POWER — make sure somebody can reach it.\n"
                "[Esc] also triggers STOP."
            )
        return (
            "NOT an emergency stop!!!  (19200 baud serial link)\n"
            "~0.2 s lag (~10 mm at VEL 5000).\n"
            "For a real emergency, CUT THE POWER.   [Esc] also triggers STOP."
        )

    def _polar_entry(self, parent: tk.Widget, width: int = 9) -> tk.Entry:
        return tk.Entry(
            parent,
            width=width,
            justify="right",
            bg=ENTRY_BG,
            fg=FG,
            insertbackground=FG,
            font=("TkFixedFont", 13),
        )

    def _build_polar(self) -> None:
        panel = tk.Frame(self.root, bg=BG, highlightthickness=1, highlightbackground=DIM)
        panel.grid(row=0, column=6, rowspan=11, sticky="nsew", padx=8, pady=8)

        tk.Label(
            panel,
            text="POLAR MOVE  (from origin, X/Y only)",
            bg=BG,
            fg=FG,
            font=("TkDefaultFont", 11, "bold"),
        ).grid(row=0, column=0, columnspan=4, sticky="w", padx=6, pady=(6, 2))
        tk.Label(
            panel,
            text="φ = 0 along −Y,  +φ toward +X (north),  −90 … +90°",
            bg=BG,
            fg=DIM,
        ).grid(row=1, column=0, columnspan=4, sticky="w", padx=6)

        tk.Label(panel, text="r  [mm]", bg=BG, fg=DIM).grid(row=2, column=0, sticky="e", padx=6)
        self.r_entry = self._polar_entry(panel)
        self.r_entry.grid(row=2, column=1, sticky="w", pady=4)
        tk.Label(panel, text="φ  [deg]", bg=BG, fg=DIM).grid(row=2, column=2, sticky="e", padx=6)
        self.phi_entry = self._polar_entry(panel)
        self.phi_entry.grid(row=2, column=3, sticky="w", pady=4)
        for entry in (self.r_entry, self.phi_entry):
            entry.bind("<KeyRelease>", lambda _event: self._redraw_polar())
            entry.bind("<Return>", lambda _event: self.on_polar())

        self.polar_target = tk.Label(panel, text="", bg=BG, fg=DIM, font=("TkFixedFont", 10))
        self.polar_target.grid(row=3, column=0, columnspan=4, sticky="w", padx=6)

        ttk.Button(panel, text="GO  (Y first, then X)", command=self.on_polar).grid(
            row=4, column=0, columnspan=4, sticky="we", padx=6, pady=4
        )

        self.polar_now = tk.Label(
            panel, text="", bg=BG, fg=GREEN, font=("TkFixedFont", 14), anchor="w"
        )
        self.polar_now.grid(row=5, column=0, columnspan=4, sticky="we", padx=6, pady=(6, 0))

        self.plot = tk.Canvas(panel, width=PLOT_W, height=PLOT_H, bg=LOG_BG, highlightthickness=0)
        self.plot.grid(row=6, column=0, columnspan=4, padx=6, pady=6)
        tk.Label(
            panel,
            text="top view · green = now · red = target · amber = path\n"
            "Z is never moved by a polar move.",
            bg=BG,
            fg=DIM,
            justify="left",
        ).grid(row=7, column=0, columnspan=4, sticky="w", padx=6, pady=(0, 6))

    # -- events -------------------------------------------------------------
    def _vel_for(self, axis: str) -> int | None:
        """The velocity entry as an int, after the over-ceiling confirmation.
        None means: do not move."""
        try:
            vel = int(float(self.vel.get()))
        except ValueError:
            self._log(f"velocity {self.vel.get()!r} is not a number", "err")
            return None
        ceiling = MAX_VEL[axis]
        if vel > ceiling and not messagebox.askyesno(
            "Velocity above proven maximum",
            f"VEL {vel} on {axis} exceeds the highest value proven clean "
            f"on that axis ({ceiling}).\n\nAbove the machine may stall. "
            f"When it stalls it loses position silently (no encoder to detect it).\n\n"
            f"Proceed anyway?",
        ):
            return None
        return vel

    def on_move(self, axis: str, absolute: bool) -> None:
        if not self.worker.is_alive():
            self._log("not connected: nothing to send to", "err")
            return
        if self.worker.abort.is_set():
            self._log("STOP is latched — press 'reset STOP latch' first", "warn")
            return
        if self.busy:
            self._log("a move is already running", "warn")
            return
        text = self.entry[axis].get().strip()
        try:
            mm = float(text)
        except ValueError:
            self._log(f"{axis}: {text!r} is not a number", "err")
            return
        vel = self._vel_for(axis)
        if vel is None:
            return

        steps_per_mm = STEPS_PER_MM[axis]
        target = round(mm * steps_per_mm)
        delta_mm = mm - self.steps[axis] / steps_per_mm if absolute else mm
        if abs(delta_mm) > CONFIRM_OVER_MM and not messagebox.askyesno(
            "Large move",
            f"This will move on the {axis} axis by {delta_mm:+.1f} mm.\n\n"
            f"There are no limit switches on this machine and no encoder! "
            f"If there is no space it will run into a hard stop!\n\nProceed?",
        ):
            return
        # Mark busy here, on the Tk thread. Waiting for the worker's own "busy"
        # message leaves a window in which a second click queues a second move.
        self._set_busy(True)
        self.worker.submit("move", axis=axis, steps=target, vel=vel, absolute=absolute)

    def _read_polar(self) -> tuple[float, float] | None:
        """(r, phi) from the entries, or None if either is missing or invalid."""
        try:
            r = float(self.r_entry.get())
            phi = float(self.phi_entry.get())
        except ValueError:
            return None
        if r < 0 or not PHI_MIN <= phi <= PHI_MAX or not math.isfinite(r):
            return None
        return r, phi

    def on_polar(self) -> None:
        if not self.worker.is_alive():
            self._log("not connected: nothing to send to", "err")
            return
        if self.worker.abort.is_set():
            self._log("STOP is latched — press 'reset STOP latch' first", "warn")
            return
        if self.busy:
            self._log("a move is already running", "warn")
            return
        polar = self._read_polar()
        if polar is None:
            self._log(f"polar: need r >= 0 mm and {PHI_MIN:.0f} <= φ <= {PHI_MAX:.0f} deg", "err")
            return
        r, phi = polar
        x_mm, y_mm = polar_to_xy(r, phi)
        x_steps = round(x_mm * STEPS_PER_MM["X"])
        y_steps = round(y_mm * STEPS_PER_MM["Y"])

        vel_y = self._vel_for("Y")
        if vel_y is None:
            return
        vel_x = self._vel_for("X")
        if vel_x is None:
            return

        dy = y_mm - self.steps["Y"] / STEPS_PER_MM["Y"]
        dx = x_mm - self.steps["X"] / STEPS_PER_MM["X"]
        if max(abs(dx), abs(dy)) > CONFIRM_OVER_MM and not messagebox.askyesno(
            "Large move",
            f"Polar move to r = {r:.1f} mm, φ = {phi:+.2f}°\n\n"
            f"  1) Y by {dy:+.1f} mm\n  2) X by {dx:+.1f} mm\n\n"
            f"There are no limit switches on this machine and no encoder! "
            f"If there is no space it will run into a hard stop!\n\nProceed?",
        ):
            return
        self._log(f"polar: r {r:.1f} mm, φ {phi:+.2f}° -> x {x_mm:+.3f}, y {y_mm:+.3f} mm")
        self._set_busy(True)  # on the Tk thread, closes the double-click window
        self.worker.submit("polar", x_steps=x_steps, y_steps=y_steps, vel_x=vel_x, vel_y=vel_y)

    def on_toggle_dry(self) -> None:
        going_live = not self.simulate.get()
        if going_live and self.remote and not self._acknowledge_remote_arming():
            self.simulate.set(True)  # put the tick back; nothing was armed
            self._log("arming cancelled — still in dry run", "warn")
            return
        self.worker.submit("arm", live=going_live)

    def _acknowledge_remote_arming(self) -> bool:
        """The per-session acknowledgement required before a remote link may
        move the machine. Deliberate friction."""
        return bool(
            messagebox.askyesno("Arm the gantry remotely?", REMOTE_ARM_QUESTION, icon="warning")
        )

    def on_zero_all(self) -> None:
        self.worker.submit("zero", axes=list(AXES))

    def on_kill(self) -> None:
        self.worker.kill()
        self.kill_btn.configure(text="■  STOPPED  ■", bg=STOPPED_RED)
        self.reset_btn.configure(state="normal")
        self._log("STOP pressed", "err")

    def on_reset_latch(self) -> None:
        self.worker.abort.clear()
        self.kill_btn.configure(text="■  S T O P  ■", bg=RED)
        self.reset_btn.configure(state="disabled")
        self._log("STOP latch cleared — motion re-enabled", "ok")

    def on_close(self) -> None:
        """STOP, de-energize, release the port, then destroy the window.

        The worker is a daemon thread. Destroying the window on a fixed timer
        let the interpreter exit while STOPALL/MOFF were still being sent, so
        axes stayed energized. Wait for the worker to finish, with a cap.
        """
        self.worker.kill()
        self.worker.quit.set()
        self.status.configure(text="closing: STOPALL, de-energizing, releasing port…", fg=AMBER)
        self._destroy_when_worker_done(time.monotonic() + CLOSE_TIMEOUT_S)

    def _destroy_when_worker_done(self, deadline: float) -> None:
        if self.worker.is_alive() and time.monotonic() < deadline:
            self.root.after(100, self._destroy_when_worker_done, deadline)
            return
        if self.worker.is_alive():
            self._log(f"worker still busy after {CLOSE_TIMEOUT_S:.0f} s; closing anyway", "err")
        self.root.destroy()

    # -- plumbing -----------------------------------------------------------
    def _render_status(self) -> None:
        text = self.status_base
        if self.link_text:
            text = f"{text}   ·   {self.link_text}"
        self.status.configure(text=text, fg=getattr(self, "status_colour", AMBER))

    def _on_link_down(self, message: str) -> None:
        """The server stopped the machine and ended the session on its own.

        Latch STOP so nothing can be sent into a session that no longer exists,
        and make it loud: the machine moved, or was moving, without us.
        """
        self.kill_btn.configure(text="■  LINK LOST  ■", bg=STOPPED_RED)
        self.reset_btn.configure(state="normal")
        self.simulate.set(True)
        self.status_base = "LINK LOST — the server stopped the machine"
        self.status_colour = RED_HOT
        self.link_text = ""
        self._render_status()
        self._log(f"LINK LOST: {message}", "err")
        self._log("the server sent STOPALL and ended the session", "err")
        self._log("reconnect by restarting this panel; it will start disarmed", "warn")

    def _set_busy(self, busy: bool) -> None:
        self.busy = busy
        for axis in AXES:
            self.pos_label[axis].configure(fg=AMBER if busy else GREEN)

    def _log(self, msg: str, tag: str = "info") -> None:
        self.log_box.configure(state="normal")
        self.log_box.insert("end", f"{time.strftime('%H:%M:%S')}  {msg}\n", tag)
        self.log_box.see("end")
        self.log_box.configure(state="disabled")

    def _drain(self) -> None:
        """Move worker events into the widgets. Runs on the Tk thread."""
        try:
            while True:
                kind, payload = self.events.get_nowait()
                if kind == "pos":
                    for axis, steps in payload.items():
                        self.steps[axis] = steps
                        self.pos_label[axis].configure(text=f"{steps / STEPS_PER_MM[axis]:+.3f} mm")
                        self.step_label[axis].configure(text=f"{steps}")
                elif kind == "log":
                    self._log(*payload)
                elif kind == "status":
                    self.status_base, self.status_colour = payload
                    self._render_status()
                elif kind == "link":
                    self.link_text = f"link {payload * 1000:.0f} ms"
                    self._render_status()
                elif kind == "linkdown":
                    self._on_link_down(payload)
                elif kind == "busy":
                    self._set_busy(payload)
        except queue.Empty:
            pass
        self.root.after(DRAIN_INTERVAL_MS, self._drain)
        self._redraw_polar()

    # -- polar display ------------------------------------------------------
    def _now_mm(self) -> tuple[float, float]:
        return self.steps["X"] / STEPS_PER_MM["X"], self.steps["Y"] / STEPS_PER_MM["Y"]

    def _redraw_polar(self) -> None:
        x_now, y_now = self._now_mm()
        r_now, phi_now = xy_to_polar(x_now, y_now)
        if r_now < 0.005:
            self.polar_now.configure(text="now   r   0.000 mm   φ    —", fg=GREEN)
        else:
            outside = abs(phi_now) > PHI_MAX
            self.polar_now.configure(
                text=f"now   r {r_now:9.3f} mm   φ {phi_now:+8.3f}°"
                + ("   (+Y side)" if outside else ""),
                fg=AMBER if outside else GREEN,
            )

        polar = self._read_polar()
        target = polar_to_xy(*polar) if polar else None
        if target:
            self.polar_target.configure(
                text=f"target  x {target[0]:+10.3f} mm   y {target[1]:+10.3f} mm", fg=DIM
            )
        elif self.r_entry.get().strip() or self.phi_entry.get().strip():
            self.polar_target.configure(text="target  (invalid r or φ)", fg=RED_HOT)
        else:
            self.polar_target.configure(text="target  —", fg=DIM)

        self._draw_plot((x_now, y_now), target, polar[0] if polar else None)

    def _draw_plot(
        self,
        now: tuple[float, float],
        target: tuple[float, float] | None,
        r_target: float | None,
    ) -> None:
        c = self.plot
        c.delete("all")
        extent = max(PLOT_MIN_EXTENT_MM, abs(now[0]), abs(now[1]))
        if target:
            extent = max(extent, abs(target[0]), abs(target[1]), r_target or 0.0)
        extent *= 1.15
        cx, cy = PLOT_W / 2, PLOT_H / 2
        scale = (min(PLOT_W, PLOT_H) / 2 - 12) / extent

        def px(x: float, y: float) -> tuple[float, float]:
            return cx + x * scale, cy - y * scale  # +X right, +Y up

        # axes
        c.create_line(0, cy, PLOT_W, cy, fill="#3a3a3a")
        c.create_line(cx, 0, cx, PLOT_H, fill="#3a3a3a")
        c.create_text(PLOT_W - 4, cy - 8, text="+X (N)", fill=DIM, anchor="e")
        c.create_text(cx + 4, PLOT_H - 4, text="−Y  φ=0", fill=DIM, anchor="sw")
        c.create_text(cx + 4, 4, text="+Y (W)", fill=DIM, anchor="nw")
        c.create_text(4, 4, text=f"±{extent:.0f} mm", fill=DIM, anchor="nw")

        if target and r_target:
            rr = r_target * scale
            c.create_arc(
                cx - rr,
                cy - rr,
                cx + rr,
                cy + rr,
                start=180,
                extent=180,
                style="arc",
                outline="#555555",
                dash=(3, 3),
            )
            # planned L path: Y leg first, then X leg
            p0, p1, p2 = px(*now), px(now[0], target[1]), px(*target)
            c.create_line(*p0, *p1, *p2, fill=AMBER, dash=(5, 3), width=2, arrow="last")
            c.create_line(cx, cy, *p2, fill="#555555")
            x, y = p2
            c.create_line(x - 6, y - 6, x + 6, y + 6, fill=RED_HOT, width=2)
            c.create_line(x - 6, y + 6, x + 6, y - 6, fill=RED_HOT, width=2)

        c.create_oval(cx - 3, cy - 3, cx + 3, cy + 3, fill=FG, outline="")
        x, y = px(*now)
        c.create_oval(x - 5, y - 5, x + 5, y + 5, fill=GREEN, outline="")


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(
        description="Manual control panel for the OES gantry.",
        epilog="Use --remote to drive the machine through a gantry_server, "
        "normally over a Bluetooth PAN. A remote panel starts disarmed.",
    )
    where = parser.add_mutually_exclusive_group()
    where.add_argument(
        "--port",
        help="serial port, e.g. /dev/ttyUSB0. Omit both this and --remote to be asked.",
    )
    where.add_argument(
        "--remote",
        metavar="URL",
        help="reach the machine through a server, e.g. tcp://10.73.133.1:7313",
    )
    args = parser.parse_args(argv)

    window = tk.Tk()
    target = args.remote or args.port
    if target is None:
        target = ask_for_target(window)
        if target is None:  # cancelled
            window.destroy()
            return
    App(window, target)
    window.mainloop()


if __name__ == "__main__":
    main()
