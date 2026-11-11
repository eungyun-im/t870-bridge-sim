cmake_minimum_required(VERSION 3.0.2)
project(vehicle_bridge)

find_package(catkin REQUIRED)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/vehicle_bridge_node.py
  scripts/joystick_check.py
  scripts/joystick_mapper.py
  scripts/keyboard_joy_node.py
  scripts/simple_joy_node.py
  scripts/steer_monitor.py
  scripts/adc_raw_monitor.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
