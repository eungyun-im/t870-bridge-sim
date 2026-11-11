#!/usr/bin/env python3
import sys

try:
    import rospy
    from std_msgs.msg import String
except ImportError as exc:
    sys.stderr.write(
        "ROS1 rospy import failed. This usually means ROS2 Humble is still in "
        "PYTHONPATH while running a ROS1 script.\n\n"
        "Run this in a clean ROS1 terminal:\n"
        "  unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH ROS_DISTRO ROS_PYTHON_VERSION\n"
        "  export PYTHONPATH=$(python3 - <<'PY'\n"
        "import sys\n"
        "print(':'.join(p for p in sys.path if 'ros/humble' not in p))\n"
        "PY\n"
        ")\n"
        "  source /opt/ros/noetic/setup.bash\n"
        "  python3 /home/kuuve/Downloads/vehicle_bridge_shit/vehicle_bridge/scripts/steer_monitor.py\n\n"
        "For direct Arduino ADC without ROS, use:\n"
        "  python3 /home/kuuve/Downloads/vehicle_bridge_shit/vehicle_bridge/scripts/adc_raw_monitor.py --port /dev/ttyACM0\n\n"
        f"Original error: {exc}\n"
    )
    sys.exit(1)


def parse_field(msg, key):
    for token in msg.split():
        if token.startswith(key + "="):
            return token.split("=")[1]
    return None


def cb(msg):
    adc  = parse_field(msg.data, "adc_raw")     # raw unfiltered A8 ADC (0..1023)
    filt = parse_field(msg.data, "adc_filt")    # HLC-filtered feedback (PID input)
    pwm  = parse_field(msg.data, "appliedPWM")  # steer PWM the LLC actually drove
    estop = parse_field(msg.data, "estop")
    if adc is not None:
        rospy.loginfo("adc_raw=%-5s  adc_filt=%-6s  appliedPWM=%-4s  estop=%s", adc, filt, pwm, estop)


rospy.init_node("steer_monitor")
rospy.Subscriber("/vehicle/status", String, cb, queue_size=10)
rospy.loginfo("[steer_monitor] 구독 시작: /vehicle/status")
rospy.spin()
