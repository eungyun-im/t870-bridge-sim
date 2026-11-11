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


class JoystickReader:
    def __init__(self, dev):
        self.dev = dev
        self.fd = os.open(dev, os.O_RDONLY | os.O_NONBLOCK)
        self.axes = [0.0] * 16
        self.buttons = [0] * 32
        self.poll(0.8)

    def close(self):
        os.close(self.fd)

    def poll(self, seconds):
        end = time.time() + seconds
        events = []
        while time.time() < end:
            try:
                data = os.read(self.fd, 8)
            except BlockingIOError:
                time.sleep(0.01)
                continue
            if len(data) != 8:
                continue

            _, value, event_type, number = struct.unpack("IhBB", data)
            init = bool(event_type & JS_EVENT_INIT)
            event_type &= ~JS_EVENT_INIT
            if event_type == JS_EVENT_AXIS and number < len(self.axes):
                val = norm_axis(value)
                self.axes[number] = val
                events.append(("axis", number, val, init))
            elif event_type == JS_EVENT_BUTTON and number < len(self.buttons):
                self.buttons[number] = 1 if value else 0
                events.append(("button", number, self.buttons[number], init))
        return events

    def detect_motion(self, label, seconds):
        baseline = list(self.axes)
        print(f"\n[{label}] Move that control now, then release. Recording {seconds:.1f}s...")
        events = self.poll(seconds)

        axis_hits = {}
        button_hits = {}
        for kind, number, value, init in events:
            if init:
                continue
            if kind == "axis":
                delta = abs(value - baseline[number])
                prev = axis_hits.get(number)
                if prev is None or delta > prev[0]:
                    axis_hits[number] = (delta, value)
            elif kind == "button" and value == 1:
                button_hits[number] = button_hits.get(number, 0) + 1

        best_axis = None
        if axis_hits:
            number, (delta, value) = max(axis_hits.items(), key=lambda item: item[1][0])
            if delta >= 0.25:
                best_axis = (number, value, delta)

        best_button = None
        if button_hits:
            best_button = max(button_hits.items(), key=lambda item: item[1])[0]

        if best_axis is not None:
            print("  axis %d value=% .3f delta=%.3f" % best_axis)
        if best_button is not None:
            print("  button %d" % best_button)
        if best_axis is None and best_button is None:
            print("  no clear input detected")
        return {"axis": best_axis, "button": best_button}


def yes(value):
    return "true" if value else "false"


def main():
    parser = argparse.ArgumentParser(description="Interactive joystick mapping helper.")
    parser.add_argument("--dev", default="/dev/input/js0")
    parser.add_argument("--seconds", type=float, default=3.0)
    parser.add_argument("--buttons", action="store_true",
                        help="also map buttons after axes")
    args = parser.parse_args()

    js = JoystickReader(args.dev)
    try:
        print(f"opened {args.dev}")
        input("Release all sticks/triggers, then press Enter...")
        js.poll(0.5)

        forward = js.detect_motion("FORWARD throttle", args.seconds)
        reverse = js.detect_motion("REVERSE throttle", args.seconds)
        left = js.detect_motion("LEFT steering", args.seconds)
        right = js.detect_motion("RIGHT steering", args.seconds)

        button_map = {}
        if args.buttons:
            for name in ("estop_button", "estop_reset_button",
                         "estop_toggle_button", "mode_toggle_button"):
                hit = js.detect_motion(name, args.seconds)
                button_map[name] = hit["button"]

        print("\nDetected:")
        print("  forward:", forward)
        print("  reverse:", reverse)
        print("  left:   ", left)
        print("  right:  ", right)
        if button_map:
            print("  buttons:", button_map)

        f_axis = forward["axis"]
        r_axis = reverse["axis"]
        l_axis = left["axis"]
        rr_axis = right["axis"]

        print("\nSuggested config/vehicle_bridge.yaml snippet:")
        if f_axis and r_axis and f_axis[0] == r_axis[0]:
            invert_throttle = f_axis[1] > 0.0
            print('throttle_mode: "axis"')
            print(f"throttle_axis: {f_axis[0]}")
            print(f"invert_throttle: {yes(invert_throttle)}")
        elif f_axis and r_axis:
            print('throttle_mode: "trigger_pair"')
            print(f"forward_trigger_axis: {f_axis[0]}")
            print(f"reverse_trigger_axis: {r_axis[0]}")
        else:
            print("# throttle axis not detected clearly")

        if l_axis and rr_axis and l_axis[0] == rr_axis[0]:
            invert_steer = rr_axis[1] > 0.0
            print(f"steer_axis: {rr_axis[0]}")
            print(f"invert_steer: {yes(invert_steer)}")
        else:
            print("# steer axis not detected clearly")

        for name, value in button_map.items():
            if value is not None:
                print(f"{name}: {value}")
    finally:
        js.close()


if __name__ == "__main__":
    main()
