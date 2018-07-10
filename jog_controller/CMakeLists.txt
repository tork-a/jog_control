cmake_minimum_required(VERSION 2.8.3)
project(jog_controller)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  control_msgs
  jog_msgs
  moveit_core
  moveit_ros_planning
  moveit_visual_tools
  moveit_msgs
  roscpp
  rviz
)

find_package(Boost REQUIRED system filesystem date_time thread)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS actionlib control_msgs jog_msgs
  moveit_core
  moveit_ros_planning
  moveit_visual_tools
  moveit_msgs roscpp
  INCLUDE_DIRS
  LIBRARIES
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

if(rviz_QT_VERSION VERSION_LESS "5")
  find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
  include(${QT_USE_FILE})
else()
  find_package(Qt5Widgets REQUIRED)
endif()
add_definitions(-DQT_NO_KEYWORDS -g)

include_directories(${CMAKE_CURRENT_BINARY_DIR})

set(HEADER_FILES
  src/jog_frame_panel.h src/jog_joint_panel.h src/jog_slider.h
)

if(rviz_QT_VERSION VERSION_LESS "5")
  qt4_wrap_cpp(MOC_FILES ${HEADER_FILES})
else()
  qt5_wrap_cpp(MOC_FILES ${HEADER_FILES})
endif()

set(SOURCE_FILES
  src/jog_frame_panel.cpp
  src/jog_joint_panel.cpp
  ${MOC_FILES}
  )

add_library(jog_controller ${SOURCE_FILES} ${UIC_FILES})

if(rviz_QT_VERSION VERSION_LESS "5")
  target_link_libraries(jog_controller ${QT_LIBRARIES} ${catkin_LIBRARIES} ${rviz_DEFAULT_PLUGIN_LIBRARIES})
else()
  target_link_libraries(jog_controller Qt5::Widgets ${catkin_LIBRARIES} ${rviz_DEFAULT_PLUGIN_LIBRARIES})
endif()

if("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
  set_target_properties(jog_controller PROPERTIES LINK_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -z defs")
endif()

add_executable(jog_frame_node src/jog_frame_node.cpp)
target_link_libraries(jog_frame_node ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(jog_joint_node src/jog_joint_node.cpp)
target_link_libraries(jog_joint_node ${catkin_LIBRARIES} ${Boost_LIBRARIES})

install(TARGETS jog_frame_node jog_joint_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(TARGETS jog_controller
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

# Tests
if(CATKIN_ENABLE_TESTING)
  find_package(roslint REQUIRED)
  set(ROSLINT_CPP_OPTS "--filter=-runtime/references,-runtime/int")
  # roslint_cpp(src/melfa_hardware_interface.cpp src/melfa_driver_node.cpp src/melfa_loopback_node.cpp)
  set(ROSLINT_PYTHON_OPTS "--max-line-length=180")
  # roslint_python(test/test_joint_trajectory_controller.py)
  # add_dependencies(tests roslint)

  find_package(roslaunch REQUIRED)
  find_package(rostest REQUIRED)
  # Remove tempolary for travis
  # file(GLOB LAUNCH_FILES launch/*.launch)
  # foreach(LAUNCH_FILE ${LAUNCH_FILES})
  #   message(STATUS "Testing ${LAUNCH_FILE}")
  #   roslaunch_add_file_check(${LAUNCH_FILE})
  # endforeach()

  add_rostest(test/ur5_jog.test ARGS use_gui:=false)
  add_rostest(test/nextage_jog.test ARGS use_rviz:=false)
  # add_rostest(test/baxter_jog.test ARGS use_rviz:=false)
endif()
