cmake_minimum_required(VERSION 2.8.3)
project(jog_launch)

find_package(catkin REQUIRED COMPONENTS
  fake_joint_launch
  jog_controller
  roslaunch
  rostest
)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  file(GLOB LAUNCH_FILES launch/*.launch)
  foreach(LAUNCH_FILE ${LAUNCH_FILES})
    message(STATUS "Testing ${LAUNCH_FILE}")
    roslaunch_add_file_check(${LAUNCH_FILE})
  endforeach()
endif()
