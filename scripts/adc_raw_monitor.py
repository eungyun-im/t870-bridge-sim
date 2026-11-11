#!/usr/bin/env python3
import argparse
import struct
import sys
import time

import serial


STX = b"STX"
ETX = b"\r\n"
FRAME_LEN = 17


def read_frame(ser):
    buf = bytearray()
    while True:
        b = ser.read(1)
        if not b:
            return None
        buf += b
        if len(buf) > FRAME_LEN:
            del buf[:-FRAME_LEN]
        start = buf.find(STX)
        if start < 0:
            if len(buf) > 2:
                del buf[:-2]
            continue
        if start:
            del buf[:start]
        if len(buf) < FRAME_LEN:
            continue
        frame = bytes(buf[:FRAME_LEN])
        del buf[:FRAME_LEN]
        if frame[-2:] == ETX:
            return frame


def parse_frame(frame):
    _, mode, estop, gear, adc_raw, applied_pwm, enc, alive = struct.unpack(
        "<3sBBBHHiB", frame[:15]
    )
    return {
        "mode": mode,
        "estop": estop,
        "gear": gear,
        "adc_raw": adc_raw,
        "applied_pwm": applied_pwm,
        "enc": enc,
        "alive": alive,
    }


def main():
    parser = argparse.ArgumentParser(
        description="Read T870 LLC feedback directly and print raw A8 ADC."
    )
    parser.add_argument("--port", default="/dev/ttyACM0")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--rate", type=float, default=10.0, help="max print rate in Hz")
    args = parser.parse_args()

    try:
        with serial.Serial(args.port, args.baud, timeout=1.0) as ser:
            time.sleep(2.0)
            ser.reset_input_buffer()
            print(f"listening on {args.port} @ {args.baud}; Ctrl-C to stop", flush=True)
            last_print = 0.0
            min_adc = None
            max_adc = None
            while True:
                frame = read_frame(ser)
                if frame is None:
                    print("timeout: no LLC feedback frame", flush=True)
                    continue
                fb = parse_frame(frame)
                adc = fb["adc_raw"]
                min_adc = adc if min_adc is None else min(min_adc, adc)
                max_adc = adc if max_adc is None else max(max_adc, adc)
                now = time.monotonic()
                if now - last_print >= 1.0 / max(args.rate, 0.1):
                    last_print = now
                    mode = "AUTO" if fb["mode"] == 1 else "MANUAL"
                    print(
                        "adc_raw={adc:<4d} min={mn:<4d} max={mx:<4d} "
                        "appliedPWM={pwm:<3d} estop={estop} gear={gear} "
                        "mode={mode} enc={enc} alive={alive}".format(
                            adc=adc,
                            mn=min_adc,
                            mx=max_adc,
                            pwm=fb["applied_pwm"],
                            estop=fb["estop"],
                            gear=fb["gear"],
                            mode=mode,
                            enc=fb["enc"],
                            alive=fb["alive"],
                        ),
                        flush=True,
                    )
    except serial.SerialException as exc:
        sys.stderr.write(f"serial open/read failed on {args.port}: {exc}\n")
        sys.stderr.write("Close ROS vehicle_bridge or any serial monitor using the same port.\n")
        return 1
    except KeyboardInterrupt:
        print("\nstopped", flush=True)
        return 0


if __name__ == "__main__":
    raise SystemExit(main())
