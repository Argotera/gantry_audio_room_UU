# Gantry robot (room 73133)
## Branch to use remotely ( bluetooth -> raspberry pi -> gantry controller)


```
  laptop                      Raspberry Pi 1                  gantry
  ------                      --------------                  ------
  gantry_gui.py  ---BT--->  gantry_server.py  ---USB-RS232--->  OES Allegra
  move.py                   (owns the port)                     controller
  oes.py
```

An old first gen Pi is cabled to the controller by the existing PL2303 USB-RS232 adapter and
carries a USB Bluetooth dongle. A laptop discovers the Pi, pairs with it,
connects, and then runs the existing tools — GUI and scripts.

```bash
# on your laptop, once paired and joined to the Pi's Bluetooth network
python3 gantry_gui.py --remote tcp://10.73.133.1:7313
./move.py --remote tcp://10.73.133.1:7313 --axis X --mm -50 --acknowledge
```

Three things differ from working at the bench:

- **A remote panel starts disarmed.** Arming asks you to confirm that the
  workspace is clear and that somebody can reach the power cutoff. Over a radio
  link you may not be standing next to the machine.
- **STOP is slower** — budget 0.3-0.5 s rather than 0.2 s, which is 14-24 mm at
  VEL 5000 on X or Z instead of 10 mm. These are estimates until measured.
- **Connecting does not reset the controller.** The server holds the port open,
  so the step counters survive a dropped link — unlike a local tool, which
  zeroes them every time it starts. Restarting the *server* still zeroes them.

If the link drops, the Pi stops the machine by itself. It sends `STOPALL` and
then de-energizes every axis, exactly as the STOP button does.


---


## What changes in each file

| File | Change |
|---|---|
| `oes.py` | Extract `Transport`; add `SerialTransport` (verbatim move) and `RemoteTransport` (TCP only); `OESController` accepts a URL. No change to the guard layer. |
| `gantry_link.py` | **New.** Protocol encode/decode, escaping, request ids. Pure, no I/O. Shared by client and server. |
| `gantry_gui.py` | `--remote <url>`; a startup chooser listing USB adapters and, when this machine is on the Pi's PAN, the Bluetooth gantry; connection and latency indicator; acknowledgement dialog; slower idle poll when remote. |
| `move.py` | `--remote <url>` with its own ~60-line client. Stays self-contained by decision. |


## License

MIT, see `LICENSE`.
