/*******************************************************************************
* Copyright 2017 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/* Author: Kayman Jung */

#ifndef VISION_DEMO_H_
#define VISION_DEMO_H_

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <boost/thread.hpp>

#include "robotis_controller_msgs/msg/sync_write_item.hpp"
#include "robotis_controller_msgs/srv/set_module.hpp"

#include "op3_demo/op_demo.h"
#include "op3_demo/face_tracker.h"

namespace robotis_op
{

class VisionDemo : public OPDemo
{
 public:
  VisionDemo();
  ~VisionDemo();

  void setDemoEnable();
  void setDemoDisable();

  void process();

  rclcpp::Node::SharedPtr node_;
  void setNode(rclcpp::Node::SharedPtr node);

 protected:
  const int SPIN_RATE;
  const int TIME_TO_INIT;

  // void processThread();
  // void callbackThread();

  void playMotion(int motion_index);
  void setRGBLED(int blue, int green, int red);

  // void buttonHandlerCallback(const std_msgs::msg::String::SharedPtr msg);
  void facePositionCallback(const std_msgs::msg::Int32MultiArray::SharedPtr msg);
  // void demoCommandCallback(const std_msgs::msg::String::SharedPtr msg);

  void setModuleToDemo(const std::string &module_name);
  void callServiceSettingModule(const std::string &module_name);

  FaceTracker face_tracker_;

  // rclcpp::Publisher<std_msgs::msg::String>::SharedPtr module_control_pub_;
  // rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr motion_index_pub_;
  // rclcpp::Publisher<robotis_controller_msgs::msg::SyncWriteItem>::SharedPtr rgb_led_pub_;  
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr face_tracking_command_pub_;

  // rclcpp::Subscription<std_msgs::msg::String>::SharedPtr button_sub_;
  // rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr faceCoord_sub_;

  // rclcpp::Client<robotis_controller_msgs::srv::SetModule>::SharedPtr set_joint_module_client_;
  rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr faceCoord_sub_;
  geometry_msgs::msg::Point face_position_;

  rclcpp::Time prev_time_;
  // rclcpp::CallbackGroup::SharedPtr callback_group_;

  int tracking_status_;
};

} /* namespace robotis_op */

#endif /* VISION_DEMO_H_ */
