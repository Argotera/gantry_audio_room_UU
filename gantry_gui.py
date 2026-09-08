#!/usr/bin/env python3
"""
gantry_gui.py -- manual control panel for the OES gantry robot in the audio lab.

  python3 gantry_gui.py [--port /dev/ttyUSB0]

Position display, typed mm moves, set-origin-here, and a big red STOP.

NOTES
* ONE thread owns the serial port (the worker). The Tk thread never touches it.
  Two processes or two threads on the port corrupt the protocol.
* Connecting RESETS the controller and ZEROES all three counters, so on startup
  the origin is wherever the machine happens to be standing.
* The machine has no limit switches, no home switches and no encoders, so
  "position" only ever means "distance from where the machine stood at connect,
  or from where SET ORIGIN was last pressed".
* The STOP button is NOT an emergency stop. 19200 baud, so about 0.2 s of lag.
* HOME is never sent. There are no home switches. oes.py refuses it
  (home_switches=False) and RUN/NEW/SAVE/CONT as well.
"""

from __future__ import annotations

import argparse
import contextlib
import logging
import queue
import threading
import time
import tkinter as tk
from tkinter import messagebox, ttk
from typing import Callable

from oes import OESController, OESError

# --- machine constants (measured; see the machine documentation)
STEPS_PER_MM = {"X": 105.2632, "Y": 210.5263, "Z": 105.2632}
# Highest velocity PROVEN clean on each axis. Not the firmware limit (200000),
# which needs 6000 motor RPM and stalls the motor.
MAX_VEL = {"X": 5000, "Y": 5000, "Z": 5000}
DEFAULT_VEL = 4000
ACC = 40000  # firmware floor; anything lower is silently clamped
CONFIRM_OVER_MM = 500.0  # moves longer than this ask for confirmation
AXES = ("X", "Y", "Z")

CLOSE_TIMEOUT_S = 6.0  # max wait for STOPALL + MOFF x3 + port close on exit
MOVE_TIMEOUT_S = 600.0  # give up waiting for a move after this
IDLE_REFRESH_S = 1.0  # position refresh interval while nothing is queued
DRAIN_INTERVAL_MS = 50  # how often the Tk thread empties the event queue

BG, FG, DIM = "#1e1e1e", "#e8e8e8", "#8a8a8a"
RED, RED_HOT, GREEN, AMBER = "#c0392b", "#e74c3c", "#27ae60", "#d4952a"
ENTRY_BG, LOG_BG, STOPPED_RED = "#2b2b2b", "#141414", "#7a1c12"


# ----------------------------------------------------------------- worker ----
class Worker(threading.Thread):
    """Owns the serial port. Everything else talks to it through two queues:
    `commands` in from the Tk thread, `events` out to the Tk thread."""

    def __init__(self, events: queue.Queue, port: str) -> None:
        super().__init__(daemon=True)
        self.events = events
        self.port = port
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
        try:
            self.emit("pos", {axis: self.ctl.report_position(axis) for axis in AXES})
        except OESError as e:
            self.log(f"position read failed: {e}", "warn")

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
            self.controller = OESController(port=self.port, verbose=False).open()
            self.ctl.joystick(False)
            # The driver starts in dry run. This panel is live from the start.
            self.ctl.arm(confirm=True)
            self.emit("status", ("ARMED   ·  motion is LIVE", GREEN))
            self.log(f"connected on {self.ctl.port}, firmware {self.ctl.version}")
            self.log("ARMED -- motion commands are transmitted for real", "ok")
            self.log("counters zeroed by the connect reset -- origin is HERE")
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
            acc = None if axis in self.acc_sent else ACC
            vel_to_send = None if self.vel_sent.get(axis) == vel else vel
            if absolute:
                result = self.ctl.move_absolute(axis, steps, vel=vel_to_send, acc=acc)
                self.log(f"{axis} -> {mm:+.3f} mm  (abs, VEL {vel})")
            else:
                result = self.ctl.move_relative(axis, steps, vel=vel_to_send, acc=acc)
                self.log(f"{axis} {mm:+.3f} mm  (rel, VEL {vel})")
            if result.dry_run:
                self.log("DRY RUN -- nothing was sent; the machine will not move", "warn")
                return
            self.acc_sent.add(axis)
            self.vel_sent[axis] = vel
            self._wait_move(axis)
        finally:
            self.emit("busy", False)

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
            done = self.ctl.wait_stopped(axis, timeout=timeout, on_poll=between_polls)
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
        self.steps = {axis: 0 for axis in AXES}  # last known counters
        self.busy = False

        root.title("OES Gantry — manual control")
        root.configure(bg=BG)
        root.protocol("WM_DELETE_WINDOW", self.on_close)
        root.bind("<Escape>", lambda _event: self.on_kill())

        self._build()
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

        self.simulate = tk.BooleanVar(value=False)
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

    @staticmethod
    def stop_note() -> str:
        return (
            "NOT an emergency stop!!!  (19200 baud serial link)\n"
            "~0.2 s lag (~10 mm at VEL 5000).\n"
            "For a real emergency, CUT THE POWER.   [Esc] also triggers STOP."
        )

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

    def on_toggle_dry(self) -> None:
        self.worker.submit("arm", live=not self.simulate.get())

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
                    self.status.configure(text=payload[0], fg=payload[1])
                elif kind == "busy":
                    self._set_busy(payload)
        except queue.Empty:
            pass
        self.root.after(DRAIN_INTERVAL_MS, self._drain)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Manual control panel for the OES gantry.")
    parser.add_argument("--port", default="/dev/ttyUSB0", help="serial port (default /dev/ttyUSB0)")
    args = parser.parse_args()
    window = tk.Tk()
    App(window, args.port)
    window.mainloop()
