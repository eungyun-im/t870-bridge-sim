#!/usr/bin/env python3
import argparse
import os
import struct
import time


JS_EVENT_BUTTON = 0x01
JS_EVENT_AXIS = 0x02
JS_EVENT_INIT = 0x80


def norm_axis(value):
    return max(-1.0, min(1.0, value / 32767.0))


def main():
    parser = argparse.ArgumentParser(description="Print Linux joystick events.")
    parser.add_argument("--dev", default="/dev/input/js0")
    parser.add_argument("--seconds", type=float, default=0.0,
                        help="0 means run until Ctrl-C")
    args = parser.parse_args()

    fd = os.open(args.dev, os.O_RDONLY | os.O_NONBLOCK)
    print(f"opened {args.dev}; move sticks or press buttons; Ctrl-C to stop", flush=True)

    end = None if args.seconds <= 0 else time.time() + args.seconds
    try:
        while end is None or time.time() < end:
            try:
                data = os.read(fd, 8)
            except BlockingIOError:
                time.sleep(0.02)
                continue
            if len(data) != 8:
                continue

            _, value, event_type, number = struct.unpack("IhBB", data)
            init = bool(event_type & JS_EVENT_INIT)
            event_type &= ~JS_EVENT_INIT

            if event_type == JS_EVENT_AXIS:
                print(f"axis {number}: raw={value:6d} norm={norm_axis(value): .3f} init={init}",
                      flush=True)
            elif event_type == JS_EVENT_BUTTON:
                print(f"button {number}: {value} init={init}", flush=True)
            else:
                print(f"type {event_type} num={number} value={value} init={init}", flush=True)
    except KeyboardInterrupt:
        print("\nstopped", flush=True)
    finally:
        os.close(fd)


if __name__ == "__main__":
    main()
