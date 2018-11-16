cmake_minimum_required(VERSION 2.8.3)
project(recordit)

find_package(catkin REQUIRED)
catkin_package()
catkin_python_setup()

catkin_install_python(PROGRAMS scripts/recordit
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

catkin_install_python(PROGRAMS scripts/throttle_joints_by_change
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

if(CATKIN_ENABLE_TESTING)
   find_package(rostest REQUIRED)
   add_rostest(test/recordit.test)
endif()