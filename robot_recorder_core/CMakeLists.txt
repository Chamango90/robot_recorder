cmake_minimum_required(VERSION 2.8.3)
project(robot_recorder)

find_package(catkin REQUIRED)
catkin_package()
catkin_python_setup()

catkin_install_python(PROGRAMS scripts/robot_recorder_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

catkin_install_python(PROGRAMS scripts/throttle_joints_by_change
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
