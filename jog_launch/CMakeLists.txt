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
  set(LAUNCH_FILES
    launch/ur3.launch
    launch/ur5.launch
    launch/tra1.launch
    launch/nextage.launch
  )
  foreach(LAUNCH_FILE ${LAUNCH_FILES})
    message(STATUS "Testing ${LAUNCH_FILE}")
    roslaunch_add_file_check(${LAUNCH_FILE})
  endforeach()

  add_rostest(test/ur3_jog.test)
  add_rostest(test/ur5_jog.test)
  add_rostest(test/tra1_jog.test)
  # add_rostest(test/nextage_jog.test)
  
endif()
