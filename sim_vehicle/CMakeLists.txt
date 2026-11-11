cmake_minimum_required(VERSION 3.0.2)
project(sim_vehicle)

find_package(catkin REQUIRED)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/fake_vehicle_node.py
  scripts/eval_metrics.py
  scripts/fake_obstacles_node.py
  scripts/fake_traffic_light_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch rviz config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
