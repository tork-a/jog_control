cmake_minimum_required(VERSION 2.8.3)
project(jog_controller)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  jog_msgs
  moveit_msgs
  roscpp
  rviz
)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

catkin_package(
  DEPENDS rviz
  CATKIN_DEPENDS actionlib jog_msgs moveit_msgs roscpp
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
target_link_libraries(jog_frame_node ${catkin_LIBRARIES})

add_executable(jog_joint_node src/jog_joint_node.cpp)
target_link_libraries(jog_joint_node ${catkin_LIBRARIES})

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
