cmake_minimum_required(VERSION 2.8.3)
project(jog_controller)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  jog_msgs
  moveit_msgs
  roscpp
)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

catkin_package(
 CATKIN_DEPENDS actionlib jog_msgs moveit_msgs roscpp
 INCLUDE_DIRS
 LIBRARIES
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(jog_frame_node src/jog_frame_node.cpp)
target_link_libraries(jog_frame_node ${catkin_LIBRARIES})

install(TARGETS jog_frame_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
