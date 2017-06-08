################################################################################
# CMake
################################################################################
cmake_minimum_required(VERSION 2.8.3)
project(ball_detector)

################################################################################
# Packages
################################################################################
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  image_transport
  roscpp
  rospy
  std_msgs
  dynamic_reconfigure
  message_generation
)

################################################################################
# Declare ROS messages, services and actions
################################################################################
add_message_files(
  FILES
  circleSetStamped.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs std_msgs
)

################################################################################
# Declare ROS dynamic reconfigure parameters
################################################################################
generate_dynamic_reconfigure_options(cfg/detector_params.cfg)


################################################################################
# Catkin specific configuration
################################################################################
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS cv_bridge geometry_msgs image_transport roscpp rospy std_msgs dynamic_reconfigure
)

################################################################################
# Build
################################################################################
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(ball_detector_node
  src/ball_detector.cpp
  src/ball_detector_node.cpp)

add_dependencies(ball_detector_node ${PROJECT_NAME}_gencfg)

target_link_libraries(ball_detector_node
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
# install(TARGETS ball_detector ball_detector_node
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