# Gantry robot, audio lab (room 73133)

Python tools for the DIY three-axis belt gantry in the audio lab.

| File | What it is |
|---|---|
| `oes.py` | Driver: serial protocol, reset handshake, guarded motion API. |
| `gantry_gui.py` | Manual control GUI. |
| `move.py` | Command line mover for use in scripts. Self contained, needs nothing else from this repo. |

The full machine description, wiring, calibration along with a complete list of
details and gotchas are in a separate PDF. Ask the lab for it.

## Read this before anything moves

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

Python 3.7+, pyserial, tkinter for the GUI. The user must be in the
`dialout` group so that no `sudo` is needed.

```bash
pip install -r requirements.txt
```

The unit tests run against a fake serial port, so no hardware is needed:

```bash
python3 -m unittest -v
```

## The GUI

```bash
python3 gantry_gui.py
```

Opens the port, which resets the controller and zeroes the counters, sends
`JOFF`. Tick *dry run* to rehearse without motion. Moves over 500 mm and velocities above the proven ceiling ask
for confirmation. `Esc` is STOP. STOP latches: press *reset STOP latch* before
the next move.

## The command line mover

```bash
./move.py --axis X --mm -250         # -250 mm on X and stay there
./move.py --axis Z --home-counter    # SPOSZ 0: redefine here as zero, no motion
./move.py --axis Y                   # only report the counters
./move.py --help                     # help and remaining flags
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
