################################################################################
# CMake
################################################################################
cmake_minimum_required(VERSION 2.8.3)
project(op3_demo)

################################################################################
# Packages
################################################################################
find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  sensor_msgs
  ball_detector
  op3_walking_module_msgs
  robotis_controller_msgs
  cmake_modules
  robotis_math
)

find_package(Eigen REQUIRED)


################################################################################
# Declare ROS messages, services and actions
################################################################################

################################################################################
# Declare ROS dynamic reconfigure parameters
################################################################################

################################################################################
# Catkin specific configuration
################################################################################
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp sensor_msgs
)

################################################################################
# Build
################################################################################
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

add_executable(op_demo_node
  src/demo_node.cpp
  src/soccer/soccer_demo.cpp
  src/soccer/keeper_demo.cpp
  src/soccer/pk_demo.cpp
  src/soccer/ball_tracker.cpp
  src/soccer/ball_follower.cpp
  src/action/action_demo.cpp
  src/vision/vision_demo.cpp
  src/vision/face_tracker.cpp
)

add_dependencies(op_demo_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(op_demo_node
  ${catkin_LIBRARIES}
  yaml-cpp
)

add_executable(self_test_node
  src/test_node.cpp
  src/soccer/soccer_demo.cpp
  src/soccer/keeper_demo.cpp
  src/soccer/pk_demo.cpp
  src/soccer/ball_tracker.cpp
  src/soccer/ball_follower.cpp
  src/action/action_demo.cpp
  src/vision/vision_demo.cpp
  src/vision/face_tracker.cpp
  src/test/button_test.cpp
  src/test/mic_test.cpp
)

add_dependencies(self_test_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(self_test_node
  ${catkin_LIBRARIES}
  yaml-cpp
)

################################################################################
# Install
################################################################################

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS ball_tracking ball_tracking_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

################################################################################
# Test
################################################################################
