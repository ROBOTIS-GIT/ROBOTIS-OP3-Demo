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
  cmake_modules
  roscpp
  rospy
  std_msgs
  dynamic_reconfigure
  message_generation
)

## Resolve system dependency on yaml-cpp, which apparently does not
## provide a CMake find_package() module.
find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)
find_path(YAML_CPP_INCLUDE_DIR
  NAMES yaml_cpp.h
  PATHS ${YAML_CPP_INCLUDE_DIRS}
)
find_library(YAML_CPP_LIBRARY
  NAMES YAML_CPP
  PATHS ${YAML_CPP_LIBRARY_DIRS}
)
link_directories(${YAML_CPP_LIBRARY_DIRS})

if(NOT ${YAML_CPP_VERSION} VERSION_LESS "0.5")
add_definitions(-DHAVE_NEW_YAMLCPP)
endif(NOT ${YAML_CPP_VERSION} VERSION_LESS "0.5")
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
  CATKIN_DEPENDS cv_bridge cmake_modules geometry_msgs image_transport roscpp rospy std_msgs dynamic_reconfigure
)

################################################################################
# Build
################################################################################
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
)

add_executable(ball_detector_node
  src/ball_detector.cpp
  src/ball_detector_node.cpp)

add_dependencies(ball_detector_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(ball_detector_node
  ${catkin_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
)

################################################################################
# Install
################################################################################

################################################################################
# Test
################################################################################
