#!/usr/bin/env python3
import os
import struct
import time

import rospy
from sensor_msgs.msg import Joy


JS_EVENT_BUTTON = 0x01
JS_EVENT_AXIS = 0x02
JS_EVENT_INIT = 0x80


def axis_value(raw, deadzone):
    value = max(-1.0, min(1.0, raw / 32767.0))
    return 0.0 if abs(value) < deadzone else value


def main():
    rospy.init_node("simple_joy_node")
    dev = rospy.get_param("~dev", "/dev/input/js0")
    rate_hz = float(rospy.get_param("~autorepeat_rate", 20.0))
    deadzone = float(rospy.get_param("~deadzone", 0.05))
    axes = [0.0] * int(rospy.get_param("~num_axes", 8))
    buttons = [0] * int(rospy.get_param("~num_buttons", 12))

    pub = rospy.Publisher("/joy", Joy, queue_size=10)

    fd = os.open(dev, os.O_RDONLY | os.O_NONBLOCK)
    rospy.loginfo("[simple_joy_node] opened %s", dev)

    rate = rospy.Rate(rate_hz)
    while not rospy.is_shutdown():
        while True:
            try:
                data = os.read(fd, 8)
            except BlockingIOError:
                break
            if len(data) != 8:
                break

            _, value, event_type, number = struct.unpack("IhBB", data)
            event_type &= ~JS_EVENT_INIT
            if event_type == JS_EVENT_AXIS and number < len(axes):
                axes[number] = axis_value(value, deadzone)
            elif event_type == JS_EVENT_BUTTON and number < len(buttons):
                buttons[number] = 1 if value else 0

        msg = Joy()
        msg.header.stamp = rospy.Time.now()
        msg.axes = list(axes)
        msg.buttons = list(buttons)
        pub.publish(msg)
        rate.sleep()

    os.close(fd)


if __name__ == "__main__":
    main()
