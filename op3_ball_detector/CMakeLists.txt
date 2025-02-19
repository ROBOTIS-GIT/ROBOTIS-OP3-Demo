cmake_minimum_required(VERSION 3.5)
project(op3_ball_detector)

set(ROS_PACKAGE_NAME ${PROJECT_NAME})

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(usb_cam 0.8.2 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)
find_package(op3_ball_detector_msgs REQUIRED)
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

include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
)

add_executable(ball_detector_node
  src/ball_detector.cpp
  src/ball_detector_node.cpp
)

ament_target_dependencies(ball_detector_node
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  ament_index_cpp 
  cv_bridge
  image_transport
  OpenCV
  Boost
  op3_ball_detector_msgs
)
target_link_libraries(ball_detector_node ${YAML_CPP_LIBRARIES})

target_compile_definitions(ball_detector_node PUBLIC "ROS_PACKAGE_NAME=\"${ROS_PACKAGE_NAME}\"")

install(TARGETS ball_detector_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

install(DIRECTORY config launch rviz 
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
