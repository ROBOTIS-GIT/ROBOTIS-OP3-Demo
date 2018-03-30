################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 2.8.3)
project(ball_detector)

################################################################################
# Find catkin packages and libraries for catkin and system dependencies
################################################################################
find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  std_msgs
  sensor_msgs
  geometry_msgs
  dynamic_reconfigure
  cv_bridge
  image_transport
  message_generation
)

################################################################################
# Setup for python modules and scripts
################################################################################

################################################################################
# Declare ROS messages, services and actions
################################################################################
add_message_files(
  FILES
  CircleSetStamped.msg
  BallDetectorParams.msg
)

add_service_files(
  FILES
  GetParameters.srv
  SetParameters.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs 
)

################################################################################
# Declare ROS dynamic reconfigure parameters
################################################################################
generate_dynamic_reconfigure_options(
  cfg/DetectorParams.cfg
)


################################################################################
# Declare catkin specific configuration to be passed to dependent projects
##################################################################################
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    roscpp
    roslib
    std_msgs
    sensor_msgs
    geometry_msgs
    dynamic_reconfigure
    cv_bridge
    image_transport
    message_runtime
  DEPENDS Boost OpenCV 
)

################################################################################
# Build
################################################################################
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
)

add_executable(ball_detector_node
  src/ball_detector.cpp
  src/ball_detector_node.cpp
)

add_dependencies(ball_detector_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(ball_detector_node
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
)

################################################################################
# Install
################################################################################
install(TARGETS ball_detector_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY config launch rviz 
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

################################################################################
# Test
################################################################################
