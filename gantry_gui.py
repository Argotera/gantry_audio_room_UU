#!/usr/bin/env python3
"""
gantry_gui.py -- manual control panel for the OES gantry in signal and audio room.

  python3 gantry_gui.py

Position display, typed mm moves, set-origin-here, and a big nice red STOP.

DESIGN NOTES
------------
* ONE thread owns the serial port (the worker). The Tk thread never touches it.
  Two processes or two threads on /dev/ttyUSB0 corrupts the protocol.
* Connecting RESETS the controller and ZEROES all three counters, so on startup
  the origin is wherever the machine happens to be standing.
* This machine is DIY and has no limit switches, no home switches and no encoders, so
  "position" only ever means "distance from when booted machine or where you last pressed SET ORIGIN".
* The STOP button is NOT an emergency stop. 19200 baud, so about 0.2 s of lag.
* HOME is never sent -- there are no home switches and it would drive an axis
  into a hard stop. oes.py refuses it (home_switches=False) and RUN/NEW/SAVE/CONT.
"""

# Keeps `OESController | None` below from being evaluated at runtime, so the
# panel still starts on earlier  Python 3.7-3.9 (oes.py already does the same).
from __future__ import annotations

import argparse
import queue, threading, time, tkinter as tk
from tkinter import ttk, messagebox

from oes import OESController, OESError

# --- machine constants: measured + derived
STEPS_PER_MM = {"X": 105.2632, "Y": 210.5263, "Z": 105.2632}

# Highest velocity actually PROVEN clean on each axis.
# Not the firmware limit (200000) -- that needs 6000 motor RPM and would stall.
MAX_VEL = {"X": 5000, "Y": 5000, "Z": 5000}
DEFAULT_VEL = 4000
ACC = 40000                 # firmware floor; anything lower is silently clamped
CONFIRM_OVER_MM = 500.0     # guard for large movements. 
AXES = ("X", "Y", "Z")
CLOSE_TIMEOUT_S = 6.0       # max wait for STOPALL + MOFF x3 + port close on exit

BG, FG, DIM = "#1e1e1e", "#e8e8e8", "#8a8a8a"
RED, RED_HOT, GREEN, AMBER = "#c0392b", "#e74c3c", "#27ae60", "#d4952a"


# ----------------------------------------------------------------- worker ----
class Worker(threading.Thread):
    """Owns the serial port. Everything else talks to it through queues."""

    def __init__(self, out: queue.Queue, port: str):
        super().__init__(daemon=True)
        self.out = out
        self.port = port
        self.cmds: queue.Queue = queue.Queue()
        self.abort = threading.Event()      # set by STOP, from any thread
        self.quit = threading.Event()
        self.g: OESController | None = None
        # ACC and VEL persist in the controller until the next reset (verified
        # on hardware 2026-09-08), so send ACC once per axis and VEL only when
        # it changes. Cleared on every STOP as cheap insurance.
        self.acc_sent: set = set()
        self.vel_sent: dict = {}

    # -- helpers ------------------------------------------------------------
    def emit(self, kind, payload):
        self.out.put((kind, payload))

    def log(self, msg, tag="info"):
        self.emit("log", (msg, tag))

    def push_positions(self):
        try:
            self.emit("pos", {a: self.g.report_position(a) for a in AXES})
        except OESError as e:
            self.log(f"position read failed: {e}", "warn")

    # -- public API (called from the Tk thread) -----------------------------
    def submit(self, kind, **kw):
        self.cmds.put((kind, kw))

    def kill(self):
        """Set the abort flag and queue a "stop". Safe from any thread.

        The queue is FIFO, so "stop" does not overtake queued moves. The abort
        flag is what makes queued moves refuse to run and makes a move in
        progress break out of its poll loop and STOP."""
        self.abort.set()
        self.cmds.put(("stop", {}))

    # -- main loop ----------------------------------------------------------
    def run(self):
        try:
            self.g = OESController(port=self.port, verbose=False).open()
            self.g.joystick(False)
            # oes.py defaults to dry_run=True: motion is SIMULATED until armed.
            # This is a manual control panel, so arm it.
            self.g.arm(confirm=True)
            self.emit("status", ("ARMED   ·  motion is LIVE", GREEN))
            self.log(f"connected on {self.g.port}, firmware {self.g.version}")
            self.log("ARMED -- motion commands are transmitted for real", "ok")
            self.log("counters zeroed by the connect reset -- origin is HERE")
            self.push_positions()
        except Exception as e:
            self.emit("status", (f"NOT CONNECTED: {e}", RED_HOT))
            self.log(f"connect failed: {e}", "err")
            self.log(f"is another process holding {self.port}?", "warn")
            return

        last_poll = 0.0
        while not self.quit.is_set():
            try:
                kind, kw = self.cmds.get(timeout=0.25)
            except queue.Empty:
                if time.time() - last_poll > 1.0:       # idle position refresh
                    self.push_positions(); last_poll = time.time()
                continue
            try:
                getattr(self, f"_do_{kind}")(**kw)
            except OESError as e:
                self.log(f"{kind} failed: {e}", "err")
            except Exception as e:                       # noqa: BLE001
                self.log(f"{kind} crashed: {type(e).__name__}: {e}", "err")
            last_poll = 0.0

        try:
            self.g.stop()
            for a in AXES:
                self.g.motor_off(a)
            self.g.close()
        except Exception:                                # noqa: BLE001
            pass

    # -- commands -----------------------------------------------------------
    def _do_stop(self):
        """STOPALL + de-energize. Always safe, always allowed."""
        try:
            self.g.stop()
        finally:
            for a in AXES:
                try:
                    self.g.motor_off(a)
                except OESError:
                    pass
        while not self.cmds.empty():                     # drop queued motion
            try:
                self.cmds.get_nowait()
            except queue.Empty:
                break
        self.acc_sent.clear()
        self.vel_sent.clear()
        self.log("STOP: STOPALL sent, all axes de-energized", "err")
        self.push_positions()
        self.emit("busy", False)

    def _do_arm(self, live):
        if live:
            self.g.arm(confirm=True)
            self.emit("status", (f"ARMED  ·  firmware {self.g.version}  ·  motion is LIVE", GREEN))
            self.log("ARMED -- motion commands are transmitted for real", "ok")
        else:
            self.g.disarm()
            self.emit("status", (f"DRY RUN  ·  firmware {self.g.version}  ·  motion is SIMULATED", AMBER))
            self.log("DRY RUN -- moves are simulated, the machine will NOT move", "warn")

    def _do_zero(self, axes):
        for a in axes:
            self.g.redefine_position(a, 0)
        self.log(f"origin set here for {'/'.join(axes)} (SPOS 0 -- no motion)", "ok")
        self.push_positions()

    def _do_move(self, axis, steps, vel, absolute):
        if self.abort.is_set():
            self.log("move refused: STOP is latched -- press RESET STOP", "warn")
            self.emit("busy", False)
            return
        spmm = STEPS_PER_MM[axis]
        self.emit("busy", True)
        try:
            acc = None if axis in self.acc_sent else ACC
            vel_arg = None if self.vel_sent.get(axis) == vel else vel
            if absolute:
                res = self.g.move_absolute(axis, steps, vel=vel_arg, acc=acc)
                self.log(f"{axis} -> {steps/spmm:+.3f} mm  (abs, VEL {vel})")
            else:
                res = self.g.move_relative(axis, steps, vel=vel_arg, acc=acc)
                self.log(f"{axis} {steps/spmm:+.3f} mm  (rel, VEL {vel})")
            if res.get("dry_run"):
                self.log("DRY RUN -- nothing was sent; the machine will not move", "warn")
                return
            self.acc_sent.add(axis)
            self.vel_sent[axis] = vel
            self._wait_move(axis)
        finally:
            self.emit("busy", False)

    def _wait_move(self, axis, timeout=600.0):
        """Block until the move ends, through the driver's wait loop. Between
        polls the callback checks the STOP flag and refreshes the position."""
        def between_polls():
            if self.abort.is_set():
                self.g.stop()
                self.log(f"{axis} aborted mid-move", "err")
                return True
            try:
                self.emit("pos", {axis: self.g.report_position(axis)})
            except OESError:
                pass                                      # transient; status poll decides
            return False

        try:
            done = self.g.wait_stopped(axis, timeout=timeout, on_poll=between_polls)
        except OESError as e:
            self.log(f"{axis}: lost contact during the move ({e}) -- STOPALL", "err")
            self.g.stop()
            done = False
        if not done and not self.abort.is_set():
            self.log(f"{axis} still moving after {timeout:.0f}s -- stopping", "err")
            self.g.stop()
        self.push_positions()


# --------------------------------------------------------------------- UI ----
class App:
    def __init__(self, root: tk.Tk, port: str):
        self.root = root
        self.q: queue.Queue = queue.Queue()
        self.worker = Worker(self.q, port)
        self.steps = {a: 0 for a in AXES}
        self.busy = False

        root.title("OES Gantry — manual control")
        root.configure(bg=BG)
        root.protocol("WM_DELETE_WINDOW", self.on_close)
        root.bind("<Escape>", lambda _e: self.on_kill())

        self._build()
        self.worker.start()
        self.root.after(50, self._drain)

    # -- layout -------------------------------------------------------------
    def _build(self):
        pad = dict(padx=8, pady=4)
        root = self.root

        self.status = tk.Label(root, text="connecting…", bg=BG, fg=AMBER,
                               font=("TkDefaultFont", 11, "bold"), anchor="w")
        self.status.grid(row=0, column=0, columnspan=6, sticky="we", **pad)

        hdr = ("axis", "position", "steps", "mm", "", "")
        for c, t in enumerate(hdr):
            tk.Label(root, text=t, bg=BG, fg=DIM).grid(row=1, column=c, sticky="w", padx=8)

        self.pos_lbl, self.step_lbl, self.entry = {}, {}, {}
        for r, a in enumerate(AXES, start=2):
            tk.Label(root, text=a, bg=BG, fg=FG,
                     font=("TkFixedFont", 16, "bold")).grid(row=r, column=0, **pad)

            self.pos_lbl[a] = tk.Label(root, text="+0.000 mm", bg=BG, fg=GREEN,
                                       font=("TkFixedFont", 18), width=13, anchor="e")
            self.pos_lbl[a].grid(row=r, column=1, **pad)

            self.step_lbl[a] = tk.Label(root, text="0", bg=BG, fg=DIM,
                                        font=("TkFixedFont", 10), width=10, anchor="e")
            self.step_lbl[a].grid(row=r, column=2, **pad)

            e = tk.Entry(root, width=10, justify="right", bg="#2b2b2b", fg=FG,
                         insertbackground=FG, font=("TkFixedFont", 13))
            e.grid(row=r, column=3, **pad)
            e.bind("<Return>", lambda _e, ax=a: self.on_move(ax, absolute=False))
            self.entry[a] = e

            ttk.Button(root, text="Move ±", width=8,
                       command=lambda ax=a: self.on_move(ax, absolute=False)
                       ).grid(row=r, column=4, **pad)
            ttk.Button(root, text="Go to", width=7,
                       command=lambda ax=a: self.on_move(ax, absolute=True)
                       ).grid(row=r, column=5, **pad)

        # velocity + origin
        bar = tk.Frame(root, bg=BG)
        bar.grid(row=5, column=0, columnspan=6, sticky="we", padx=8, pady=(10, 2))
        tk.Label(bar, text="velocity", bg=BG, fg=DIM).pack(side="left")
        self.vel = tk.Entry(bar, width=7, justify="right", bg="#2b2b2b", fg=FG,
                            insertbackground=FG, font=("TkFixedFont", 12))
        self.vel.insert(0, str(DEFAULT_VEL))
        self.vel.pack(side="left", padx=6)
        tk.Label(bar, text=f"steps/s   proven max  X {MAX_VEL['X']} · "
                           f"Y {MAX_VEL['Y']} · Z {MAX_VEL['Z']}",
                 bg=BG, fg=DIM).pack(side="left")

        self.simulate = tk.BooleanVar(value=False)
        tk.Checkbutton(bar, text="dry run (simulate, do not move)",
                       variable=self.simulate, command=self.on_toggle_dry,
                       bg=BG, fg=AMBER, selectcolor="#2b2b2b",
                       activebackground=BG, activeforeground=AMBER,
                       highlightthickness=0).pack(side="right")

        ttk.Button(root, text="SET ORIGIN HERE  (zero all axes)",
                   command=self.on_zero_all
                   ).grid(row=6, column=0, columnspan=6, sticky="we", padx=8, pady=6)

        # kill switch
        self.kill_btn = tk.Button(root, text="■  S T O P  ■", command=self.on_kill,
                                  bg=RED, fg="white", activebackground=RED_HOT,
                                  activeforeground="white", relief="raised", bd=5,
                                  font=("TkDefaultFont", 22, "bold"), height=2)
        self.kill_btn.grid(row=7, column=0, columnspan=6, sticky="we", padx=8, pady=(12, 2))

        tk.Label(root, text=self.stop_note(), bg=BG, fg=AMBER,
                 justify="left", anchor="w").grid(row=8, column=0, columnspan=6,
                                                  sticky="we", padx=8)

        self.reset_btn = ttk.Button(root, text="reset STOP latch",
                                    command=self.on_reset_latch, state="disabled")
        self.reset_btn.grid(row=9, column=0, columnspan=6, sticky="we", padx=8, pady=4)

        self.log = tk.Text(root, height=11, bg="#141414", fg=FG, wrap="word",
                           font=("TkFixedFont", 10), relief="flat")
        self.log.grid(row=10, column=0, columnspan=6, sticky="nsew", padx=8, pady=8)
        for tag, col in (("info", DIM), ("ok", GREEN), ("warn", AMBER), ("err", RED_HOT)):
            self.log.tag_config(tag, foreground=col)
        self.log.configure(state="disabled")
        root.grid_rowconfigure(10, weight=1)
        root.grid_columnconfigure(1, weight=1)

    @staticmethod
    def stop_note():
        return ("NOT an emergency stop!!!  (19200 baud serial link)\n"
                "~0.2 s lag (~10 mm at VEL 5000).\n"
                "For a real emergency, CUT THE POWER.   [Esc] also triggers STOP.")

    # -- events -------------------------------------------------------------
    def _vel_for(self, axis):
        try:
            v = int(float(self.vel.get()))
        except ValueError:
            self._log(f"velocity {self.vel.get()!r} is not a number", "err")
            return None
        cap = MAX_VEL[axis]
        if v > cap:
            if not messagebox.askyesno(
                    "Velocity above proven maximum",
                    f"VEL {v} on {axis} exceeds the highest value proven clean "
                    f"on that axis ({cap}).\n\nAbove the machine may stall. "
                    f"When it stalls it loses position silently (no encoder to detect it).\n\n"
                    f"Proceed anyway?"):
                return None
        return v

    def on_move(self, axis, absolute):
        if not self.worker.is_alive():
            self._log("not connected: nothing to send to", "err")
            return
        if self.worker.abort.is_set():
            self._log("STOP is latched — press 'reset STOP latch' first", "warn")
            return
        if self.busy:
            self._log("a move is already running", "warn")
            return
        txt = self.entry[axis].get().strip()
        try:
            mm = float(txt)
        except ValueError:
            self._log(f"{axis}: {txt!r} is not a number", "err")
            return
        vel = self._vel_for(axis)
        if vel is None:
            return

        spmm = STEPS_PER_MM[axis]
        target = round(mm * spmm)
        delta_mm = mm if not absolute else mm - self.steps[axis] / spmm
        if abs(delta_mm) > CONFIRM_OVER_MM:
            if not messagebox.askyesno(
                    "Large move",
                    f"This will move on the {axis} axis by {delta_mm:+.1f} mm.\n\n"
                    f"There are no limit switches on this machine and no encoder! "
                    f"If there is no space it will run into a hard stop!\n\nProceed?"):
                return
        # Mark busy here, on the Tk thread. Waiting for the worker's own "busy"
        # message leaves a window in which a second click queues a second move.
        self._set_busy(True)
        self.worker.submit("move", axis=axis, steps=target, vel=vel, absolute=absolute)

    def on_toggle_dry(self):
        self.worker.submit("arm", live=not self.simulate.get())

    def on_zero_all(self):
        self.worker.submit("zero", axes=list(AXES))

    def on_kill(self):
        self.worker.kill()
        self.kill_btn.configure(text="■  STOPPED  ■", bg="#7a1c12")
        self.reset_btn.configure(state="normal")
        self._log("STOP pressed", "err")

    def on_reset_latch(self):
        self.worker.abort.clear()
        self.kill_btn.configure(text="■  S T O P  ■", bg=RED)
        self.reset_btn.configure(state="disabled")
        self._log("STOP latch cleared — motion re-enabled", "ok")

    def on_close(self):
        """STOP, de-energize, release the port, then destroy the window.

        The worker is a daemon thread. Destroying the window on a fixed timer
        let the interpreter exit while STOPALL/MOFF were still being sent, so
        axes stayed energized. Wait for the worker to finish, with a cap.
        """
        self.worker.kill()
        self.worker.quit.set()
        self.status.configure(text="closing: STOPALL, de-energizing, releasing port…", fg=AMBER)
        self._destroy_when_worker_done(time.time() + CLOSE_TIMEOUT_S)

    def _destroy_when_worker_done(self, deadline):
        if self.worker.is_alive() and time.time() < deadline:
            self.root.after(100, self._destroy_when_worker_done, deadline)
            return
        if self.worker.is_alive():
            self._log(f"worker still busy after {CLOSE_TIMEOUT_S:.0f} s; closing anyway", "err")
        self.root.destroy()

    # -- plumbing -----------------------------------------------------------
    def _set_busy(self, busy):
        self.busy = busy
        for a in AXES:
            self.pos_lbl[a].configure(fg=AMBER if busy else GREEN)

    def _log(self, msg, tag="info"):
        self.log.configure(state="normal")
        self.log.insert("end", f"{time.strftime('%H:%M:%S')}  {msg}\n", tag)
        self.log.see("end")
        self.log.configure(state="disabled")

    def _drain(self):
        try:
            while True:
                kind, payload = self.q.get_nowait()
                if kind == "pos":
                    for a, st in payload.items():
                        self.steps[a] = st
                        self.pos_lbl[a].configure(text=f"{st/STEPS_PER_MM[a]:+.3f} mm")
                        self.step_lbl[a].configure(text=f"{st}")
                elif kind == "log":
                    self._log(*payload)
                elif kind == "status":
                    self.status.configure(text=payload[0], fg=payload[1])
                elif kind == "busy":
                    self._set_busy(payload)
        except queue.Empty:
            pass
        self.root.after(50, self._drain)


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description="Manual control panel for the OES gantry.")
    ap.add_argument("--port", default="/dev/ttyUSB0", help="serial port (default /dev/ttyUSB0)")
    args = ap.parse_args()
    r = tk.Tk()
    App(r, args.port)
    r.mainloop()
