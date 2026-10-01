# Gantry robot (room 73133)

Python tools for the DIY three-axis belt gantry in the robot lab.

| File | What it is |
|---|---|
| `oes.py` | Driver: serial protocol, reset handshake, guarded motion API. |
| `gantry_gui.py` | Manual control GUI, including polar (r, φ) moves in the X/Y plane. |
| `move.py` | Cli for use in scripts. Self contained, needs nothing else from this repo. |

The full machine description, wiring, calibration along with a complete list of
details and gotchas are in a separate PDF. Ask the lab for it.

## The Gantry is a DIY machine. Read this and the accompanying pdf before first use.

**The machine has no idea where it is.** There are no limit switches, no home
switch and no encoders. The step counter records what was commanded, not its actual position.
A stall, a slipped belt or a crash produces exactly the same output as a perfect move.

- **Never send `HOME`.** Hardware does not support it.
- **Never send `RUN`, `NEW`, `SAVE` or `CONT`.** The previous owner's (defective) program
  is still in the controller's NVM. The driver refuses all four.
- **Connecting resets the controller and zeroes every counter.** Origin is
  wherever the machine was when a tool starts. Position does not survive
  a reconnect or reboot.
- **Out-of-range values are clamped silently by the contoller, not rejected.** `VEL` floor 200,
  `ACC` floor 40000.
- **Proven velocity ceiling: 5000 steps/s.** Above that the
  motors may stall and lose position without any error.
- **STOP is not an emergency stop.**  ~0.2 s lag. **The emergency stop is the power cord.**
- The controller boots with its joystick enabled. Every tool here sends `JOFF` first.


## Setup

You need Python 3.7 or newer, `pyserial`, and `tkinter` for the GUI.
Examples for uv and conda below. Run the commands from the repository folder.

**With uv**

```bash
uv venv                     # creates .venv/ in the current folder
source .venv/bin/activate            # Windows: .venv\Scripts\activate
uv pip install -r requirements.txt
```

The Python that uv downloads already includes tkinter.

**With conda**

```bash
conda create -n gantry -c conda-forge python pyserial
conda activate gantry
```

Conda's Python already includes tkinter.



**Serial port access (Linux).** Your user must be in the `dialout` group, so
that no `sudo` is needed. Add yourself once, then log out and back in:

```bash
sudo usermod -aG dialout $USER
```

## The GUI

```bash
python3 gantry_gui.py
```

Opens the port, which resets the controller and zeroes the counters, sends
`JOFF`. Tick *dry run* to rehearse without motion. Moves over 500 mm and velocities above the proven ceiling ask
for confirmation. `Esc` is STOP. STOP latches: press *reset STOP latch* before
the next move.

The panel on the left one can either input an absolute position(), or amount of displacement
The panel on the right moves X and Y to a polar target measured from the
origin: `r` in mm and `φ` in degrees, with φ = 0 along −Y and +φ toward +X
(−90° to +90°). It moves Y first and then X, an L-shaped path rather than a
straight line, and draws that path before you press GO. X only runs if Y
arrived on target. Z is never moved.

## The command line mover

```bash
./move.py --axis X --mm -250         # -250 mm on X and stay there
./move.py --axis Z --home-counter    # SPOSZ 0: redefine here as zero, no motion
./move.py --axis Y                   # only report the counters
./move.py --help                     # help and remaining flags, read this before using move.py
```

Motion is always live. `Ctrl+C` sends `STOPALL`.

## Driver in your own script

```python
from oes import OESController

with OESController() as g:          # resets the board, zeroes counters
    g.joystick(False)
    g.arm(confirm=True)             # until this, motion commands are simulated
    g.move_relative("X", 1000, vel=4000, acc=40000)
    g.wait_stopped("X")
    g.motor_off("X")
```

## License

MIT, see `LICENSE`.
