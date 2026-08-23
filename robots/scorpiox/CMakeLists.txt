cmake_minimum_required(VERSION 3.0.2)
project(scorpiox)

find_package(catkin REQUIRED)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/door_tag_marker_publisher.py
  scripts/kortex_arm_motion.py
  scripts/kortex_keyboard_control.py
  scripts/kortex_open_door_demo.py
  scripts/spot_kortex_teleop.py
  scripts/wait_for_controllers_and_unpause.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY
  config
  launch
  urdf
  worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
