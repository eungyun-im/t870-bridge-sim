<div align="center">

# t870-vehicle-bridge

**Low-level control of a HENES T870 ride-on car: Arduino Mega firmware for the motors and sensors,<br>and a ROS node on the PC that closes the steering loop over a binary serial link.**

![Arduino](https://img.shields.io/badge/Arduino_Mega_2560-00878F?style=flat-square&logo=arduino&logoColor=white)
![C++](https://img.shields.io/badge/C++-00599C?style=flat-square&logo=cplusplus&logoColor=white)
![Python](https://img.shields.io/badge/Python-3776AB?style=flat-square&logo=python&logoColor=white)
![ROS](https://img.shields.io/badge/ROS_1-22314E?style=flat-square&logo=ros&logoColor=white)
![Serial](https://img.shields.io/badge/115200_8N1-20_Hz-555555?style=flat-square)

HL FMA (Future Mobility Award) 2026 Autonomous Driving Competition · Encouragement Award · Team KUUVe

[Architecture](#architecture) · [Firmware](#firmware) · [Protocol](#protocol) · [Steering loop](#steering-loop) · [Safety](#safety) · [Bring-up](#bring-up) · [Limitations](#limitations)

</div>

---

## Architecture

```text
 PC (HLC, ROS node)                                Arduino Mega (LLC)
 ───────────────────                                ──────────────────
 joystick / autonomy cmd                            drive motor  MD20A   PWM 4, DIR 22
 steering PID on pot feedback   ── 13 B @ 20 Hz ─▶ steer motor  MD10C   PWM 5, DIR 24
 drive ramp, gear interlock     ◀─ 17 B @ 20 Hz ── encoder A/B          INT1 / D2
 command / encoder watchdogs                        steering pot          A8 (10-bit ADC)
```

The first version ran a P controller for steering inside the Arduino. Every gain change needed a reflash, and the ADC noise during motor motion made the loop hard to tune there. The steering loop was moved to the PC: the Arduino now passes a signed PWM through, keeps only hardware safety guards, and reports the raw potentiometer value back.

## Firmware

[`arduino/t870_llc/t870_llc.ino`](arduino/t870_llc/t870_llc.ino)

| Part | Implementation |
|---|---|
| Drive motor | Direction pin plus `analogWrite` PWM; command 0 to 1000 mapped linearly to PWM 0 to 255 |
| Steering motor | Signed PWM from the PC: sign sets the direction pin, magnitude is the PWM, capped at 220 |
| Encoder | Quadrature on an interrupt: on A rising, the level of B gives the direction. The 32-bit count is copied with interrupts disabled so the main loop never reads a half-updated value |
| Steering potentiometer | 8-sample average for the end-stop guard; a wrap guard pins the value to the nearest edge when the pot jumps across its electrical boundary |
| Control period | 50 ms, scheduled with `last += period` so the loop does not drift |
| Receive | Byte-wise state machine that resynchronizes on the `STX` header and accepts a frame only with the correct `CR LF` tail |

Hardware fixes made on the car: the potentiometer line on A8 picked up motor noise, which was reduced with a capacitor filter, and the wiring was rebuilt with WAGO connectors and a star ground.

## Protocol

| Direction | Size | Byte order | Fields |
|---|---|---|---|
| PC → Arduino | 13 B | big-endian | `STX` (3) · auto/manual · E-stop · gear · speed u16 · steer PWM i16 · alive · `CR LF` |
| Arduino → PC | 17 B | little-endian | `STX` (3) · auto/manual · E-stop · gear · pot ADC u16 · applied steer PWM u16 · encoder i32 · alive · `CR LF` |

The Arduino echoes the steer PWM it actually applied (after the cap and the end-stop guard), so the PC can see when a guard overrode its command.

## Steering loop

[`scripts/vehicle_bridge_node.py`](scripts/vehicle_bridge_node.py), parameters in [`config/vehicle_bridge.yaml`](config/vehicle_bridge.yaml)

| Stage | What it does |
|---|---|
| Filter | Median of 5 samples rejects spikes, then EMA (α 0.32) smooths |
| Setpoint | Degrees mapped to a pot target with two segments, so 0° is always the measured center even with an asymmetric pot |
| Early stop | Stops when the unfiltered value reaches the target, because the filtered value lags and would drive through center into the opposite end stop |
| PID | Pot-count error to signed PWM; integral clamped against windup; minimum PWM 18 to overcome static friction; deadband 19 counts |
| Output | PWM capped at 200 (below the firmware cap of 220) and slew-limited to 22 per cycle |

Drive commands are ramped (accelerate 700, decelerate 2000 raw/s), and a change of direction is allowed only after the encoder confirms standstill.

## Safety

| Where | Condition | Reaction |
|---|---|---|
| Arduino | Power on | Starts in E-stop until the PC clears it |
| Arduino | No valid frame for 300 ms | Drive and steering motors off, feedback reports E-stop |
| Arduino | Pot at the left or right end stop | Refuses to push further in that direction |
| PC | Joystick or autonomy command older than 0.5 s | Steering motor off, no drive |
| PC | Encoder feedback older than 0.5 s | Treated as unknown, gear change waits on time only |
| PC | Drive commanded, no encoder motion for 2 s | Drive cut and latched (only after the encoder has been seen working) |
| PC | Serial port lost | Reconnect loop; the Arduino watchdog stops the car meanwhile |

## Bring-up

[`FIRST_SETUP.md`](FIRST_SETUP.md) is the checklist used on the car: wheels off the ground and E-stop in reach, link check before wiring the motors, pot calibration, direction polarity, low PWM cap first, then watchdog and boot E-stop tests before driving.

Helper scripts: `adc_raw_monitor.py` (pot noise), `steer_monitor.py`, `joystick_check.py`, `speed_calib.py` (command to m/s from a bag, encoder scale checked against GPS distance).

```bash
roslaunch vehicle_bridge vehicle_bridge.launch serial_port:=/dev/ttyACM0
```

## Limitations

- The frame has no checksum. A corrupted byte between the header and the tail is accepted; a CRC would close this.
- The firmware currently uses the USB serial port (`#define HLC Serial`); switch to `Serial2` for the RS232 wiring.
- The end-stop guard reads the pot while the motor is running, so it is best effort with margin, not precise.
- The RC transmitter override uses blocking `pulseIn` because A0 to A5 have no pin-change interrupt; it is disabled by default.
- No automated tests; verification was done on the bench and on the car.

## License and authorship

- License: not chosen yet.
- Team KUUVe, Kookmin University. AI tools (Claude Code) were used for implementation and analysis.
