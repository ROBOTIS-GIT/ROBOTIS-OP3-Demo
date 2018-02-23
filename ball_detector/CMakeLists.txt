################################################################################
# CMake
################################################################################
cmake_minimum_required(VERSION 2.8.3)
project(ball_detector)

################################################################################
# Packages
################################################################################
find_package(catkin REQUIRED COMPONENTS
  roslib
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
  BallDetectorParams.msg
)

add_service_files(
  FILES
  GetParameters.srv
  SetParameters.srv
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

add_dependencies(ball_detector_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(ball_detector_node
  ${catkin_LIBRARIES}
  yaml-cpp
)

################################################################################
# Install
################################################################################

################################################################################
# Test
################################################################################
