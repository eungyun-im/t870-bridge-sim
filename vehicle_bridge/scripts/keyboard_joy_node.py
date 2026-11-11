#!/usr/bin/env python3
import select
import sys
import termios
import tty

import rospy
from sensor_msgs.msg import Joy


HELP = """
keyboard_joy_node
  w: forward
  s: reverse
  a: steer left
  d: steer right
  q: toggle MANUAL / AUTO mode
  e: toggle E-STOP
  x: quit
"""


def read_key(timeout):
    rlist, _, _ = select.select([sys.stdin], [], [], timeout)
    if not rlist:
        return None
    return sys.stdin.read(1)


def main():
    rospy.init_node("keyboard_joy_node")
    rate_hz = float(rospy.get_param("~autorepeat_rate", 20.0))
    # NOTE: vehicle_bridge.yaml's throttle_deadzone is currently 0.50 -- this must stay
    # above that or w/s will be silently swallowed by the bridge's deadzone filter.
    throttle_step = float(rospy.get_param("~throttle", 0.8))
    steer_step = float(rospy.get_param("~steer", 0.65))
    # must match vehicle_bridge.yaml: throttle_axis, steer_axis, estop_toggle_button,
    # mode_toggle_button -- these are the vehicle_bridge_node.py defaults / current yaml.
    ax_throttle = int(rospy.get_param("~throttle_axis", 1))
    ax_steer = int(rospy.get_param("~steer_axis", 3))
    btn_estop_toggle = int(rospy.get_param("~estop_toggle_button", 5))
    btn_mode_toggle = int(rospy.get_param("~mode_toggle_button", 7))

    num_axes = max(8, ax_throttle + 1, ax_steer + 1)
    num_buttons = max(12, btn_estop_toggle + 1, btn_mode_toggle + 1)
    axes = [0.0] * num_axes
    buttons = [0] * num_buttons

    pub = rospy.Publisher("/joy", Joy, queue_size=10)
    rate = rospy.Rate(rate_hz)
    old_settings = termios.tcgetattr(sys.stdin)

    print(HELP, flush=True)
    try:
        tty.setcbreak(sys.stdin.fileno())
        while not rospy.is_shutdown():
            key = read_key(1.0 / rate_hz)

            # wasd are momentary: no key-up event from a terminal, so each axis
            # resets every cycle and only holds nonzero while autorepeat keeps
            # feeding the same key.
            axes[ax_throttle] = 0.0
            axes[ax_steer] = 0.0
            # button pulses (mode/estop toggle) are single-shot too.
            buttons[btn_estop_toggle] = 0
            buttons[btn_mode_toggle] = 0

            if key == "x":
                break
            elif key == "w":
                axes[ax_throttle] = -throttle_step
            elif key == "s":
                axes[ax_throttle] = throttle_step
            elif key == "a":
                axes[ax_steer] = -steer_step
            elif key == "d":
                axes[ax_steer] = steer_step
            elif key == "q":
                buttons[btn_mode_toggle] = 1
            elif key == "e":
                buttons[btn_estop_toggle] = 1

            msg = Joy()
            msg.header.stamp = rospy.Time.now()
            msg.axes = list(axes)
            msg.buttons = list(buttons)
            pub.publish(msg)
            rate.sleep()
    finally:
        neutral = Joy()
        neutral.header.stamp = rospy.Time.now()
        neutral.axes = [0.0] * num_axes
        neutral.buttons = [0] * num_buttons
        for _ in range(5):
            pub.publish(neutral)
            rate.sleep()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        print("\nstopped", flush=True)


if __name__ == "__main__":
    main()
