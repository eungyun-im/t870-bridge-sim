# T870 LLC (Arduino Mega) — `t870_llc.ino`

Low-Level Controller firmware for the HENES T870. Pairs with the ROS `vehicle_bridge`
HLC node over RS232.

## Wiring (PDF "Low Level Controller Layout")

| Function | Arduino Mega pin |
|---|---|
| Drive motor PWM (Cytron MD20A) | **4** |
| Drive motor DIR | **22** |
| Steer motor PWM (MD10C R3) | **5** |
| Steer motor DIR | **24** |
| Encoder A / B | **3 (INT1) / 2 (INT0)** |
| Steering potentiometer | **A8** |
| RC receiver THR/STR/RUDD/ELEV/GEAR/AorM | A0/A1/A2/A3/A4/A5 |
| RS232 to PC (HLC) | **Serial2: RX2=17, TX2=16** |

> RS232 needs a TTL↔RS232 transceiver (the MAX3232 board in the PDF) between the
> Mega's Serial2 and the PC. 115200 8N1.

## Protocol (matches `vehicle_bridge`)

- **HLC→LLC** 13 B, **Big-Endian**: `STX | A/M | ESTOP | GEAR | SPEED u16(0..1000) | STEER i16(-2000..2000) | ALIVE | 0x0D 0x0A`
- **LLC→HLC** 17 B, **Little-Endian**: `STX | A/M | ESTOP | GEAR | SPEED u16(safe A8 ADC 0..1000) | STEER u16(applied PWM 0..255) | ENC i32 | ALIVE | 0x0D 0x0A`
- 20 Hz (50 ms). GEAR 0=Fwd 1=Neu 2=Bwd. STEER deg*100, negative=left.

> **PHASE B (HLC-owned steering loop):** the steering control loop now lives on the HLC; the
> LLC is a dumb actuator. Field meanings change:
> - **HLC→LLC STEER** (i16): signed **steer-motor PWM** (-255..+255). sign = direction, magnitude = PWM.
> - **LLC→HLC SPEED** (u16): safe-clamped `analogRead(A8)` (0..1000). The HLC closes the loop on this
>   (`/vehicle/steer_adc_raw`).
> - **LLC→HLC STEER** (u16): the steer PWM the LLC actually drove (0..255, after the safety cap +
>   end-stop guard).
>
> The LLC keeps two safety guards only: a `STEER_PWM_MAX` cap, and a mechanical end-stop guard
> (`POT_FULL_LEFT/RIGHT`) that refuses to push past the steering limits. Drive motor, E-STOP, comm
> watchdog and boot-ESTOP are unchanged. All steering tuning (PID gains, filter, pot calibration)
> is now ROS params in `config/vehicle_bridge.yaml` — no re-flash needed.

## Calibrate before driving

1. **Steering pot** — `rostopic echo /vehicle/steer_adc_raw` (or `/vehicle/status`), turn the
   wheels fully and read the raw A8. Set the HLC params `pot_left` / `pot_center` / `pot_right`
   in `config/vehicle_bridge.yaml`, and the LLC end-stop guards `POT_FULL_LEFT` / `POT_FULL_RIGHT`
   in the `.ino`.
2. **Direction polarity** (`.ino`) — if a wheel drives or steers the wrong way, flip
   `DRIVE_FWD_LEVEL` / `STEER_POS_LEVEL`.
3. **Steer PID** (HLC, `config/vehicle_bridge.yaml`) — tune `steer_kp` / `steer_ki` / `steer_kd`,
   `steer_pwm_min/max`, `steer_deadband`, and the EMI filter `adc_median_window` / `adc_ema_alpha`
   so the wheel reaches target without oscillating. No re-flash — just restart the node.
4. **Speed cap** (`.ino`) — for first tests lower `DRIVE_PWM_MAX` (e.g. 120) to limit speed.

## Safety built in

- Boots in **ESTOP** (motors off) until the HLC sends `ESTOP=0`.
- **Comm watchdog**: no valid HLC packet for `COMM_TIMEOUT_MS` (300 ms) → motors stop,
  feedback reports `ESTOP=1`.
- E-STOP / Neutral → drive PWM 0.

## RC transmitter override (optional, default OFF)

Set `#define USE_RC 1` to let a physical RC transmitter (AR620 on A0–A5) take over as
a hardware manual override. **Caveat:** Mega pins A0–A5 have no pin-change interrupt,
so channels are read with blocking `pulseIn` — keep it minimal. Left disabled, the car
is driven purely by the HLC (joystick/auto).

## Build / upload

Arduino IDE: open `arduino/t870_llc/t870_llc.ino`, board = *Arduino Mega 2560*,
select port, Upload. (Close any ROS node holding the port first.)
Or `arduino-cli compile --fqbn arduino:avr:mega t870_llc && arduino-cli upload -p <port> --fqbn arduino:avr:mega t870_llc`.
