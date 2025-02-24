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

#include "op3_demo/vision_demo.h"

namespace robotis_op
{

VisionDemo::VisionDemo()
  : // Node("vision_demo"),
    SPIN_RATE(30),
    TIME_TO_INIT(10),
    tracking_status_(FaceTracker::Waiting)
{
  enable_ = false;

  // // subscriber & publisher
  // module_control_pub_ = this->create_publisher<std_msgs::msg::String>("/robotis/enable_ctrl_module", 10);
  // motion_index_pub_ = this->create_publisher<std_msgs::msg::Int32>("/robotis/action/page_num", 10);
  // rgb_led_pub_ = this->create_publisher<robotis_controller_msgs::msg::SyncWriteItem>("/robotis/sync_write_item", 10);
  // face_tracking_command_pub_ = this->create_publisher<std_msgs::msg::Bool>("/face_tracking/command", 10);

  // button_sub_ = this->create_subscription<std_msgs::msg::String>("/robotis/open_cr/button", 10, std::bind(&VisionDemo::buttonHandlerCallback, this, std::placeholders::_1));
  // faceCoord_sub_ = this->create_subscription<std_msgs::msg::Int32MultiArray>("/faceCoord", 10, std::bind(&VisionDemo::facePositionCallback, this, std::placeholders::_1));

  // set_joint_module_client_ = this->create_client<robotis_controller_msgs::srv::SetModule>("/robotis/set_present_ctrl_modules");

  // // Use timer for loop rate
  // create_wall_timer(std::chrono::duration<double>(1.0 / SPIN_RATE), std::bind(&VisionDemo::process, this));
}

VisionDemo::~VisionDemo()
{
  // TODO Auto-generated destructor stub
}

void VisionDemo::setNode(rclcpp::Node::SharedPtr node)
{
  node_ = node;
  if (node_ != nullptr)
  {
    face_tracking_command_pub_ = node_->create_publisher<std_msgs::msg::Bool>("/face_tracking/command", 10);
    faceCoord_sub_ = node_->create_subscription<std_msgs::msg::Int32MultiArray>("/faceCoord", 10, std::bind(&VisionDemo::facePositionCallback, this, std::placeholders::_1));
  }
  else
  {
    RCLCPP_ERROR(rclcpp::get_logger("VisionDemo"), "Node is not set");
  }
}

void VisionDemo::setDemoEnable()
{
  // set prev time for timer
  prev_time_ = rclcpp::Clock().now();

  if (node_ == nullptr)
  {
    RCLCPP_ERROR(rclcpp::get_logger("VisionDemo"), "Node is not set, cannot set demo enable");
    return;
  }

  // change to motion module
  setModuleToDemo("action_module");

  playMotion(InitPose);

  std::this_thread::sleep_for(std::chrono::milliseconds(1500));

  setModuleToDemo("head_control_module");

  enable_ = true;

  // send command to start face_tracking
  std_msgs::msg::Bool command;
  command.data = enable_;
  face_tracking_command_pub_->publish(command);

  face_tracker_.startTracking();

  RCLCPP_INFO(rclcpp::get_logger("VisionDemo"), "Start Vision Demo");
}

void VisionDemo::setDemoDisable()
{
  face_tracker_.stopTracking();
  tracking_status_ = FaceTracker::Waiting;
  enable_ = false;

  if (node_ == nullptr)
  {
    RCLCPP_ERROR(rclcpp::get_logger("VisionDemo"), "Node is not set, cannot set demo disable");
    return;
  }

  std_msgs::msg::Bool command;
  command.data = enable_;
  face_tracking_command_pub_->publish(command);
}

void VisionDemo::process()
{
  if (enable_ == false)
    return;

  int tracking_status = face_tracker_.processTracking();

  switch(tracking_status)
  {
  case FaceTracker::Found:
    if(tracking_status_ != tracking_status)
      setRGBLED(0x1F, 0x1F, 0x1F);
    prev_time_ = rclcpp::Clock().now();
    break;

  case FaceTracker::NotFound:
  {
    if(tracking_status_ != tracking_status)
      setRGBLED(0, 0, 0);

    rclcpp::Time curr_time = rclcpp::Clock().now();
    rclcpp::Duration dur = curr_time - prev_time_;
    if(dur.seconds() > TIME_TO_INIT)
    {
      face_tracker_.goInit(0,0);
      prev_time_ = curr_time;
    }
    break;
  }
  default:
    break;
  }

  if(tracking_status != FaceTracker::Waiting)
    tracking_status_ = tracking_status;
}

// void VisionDemo::buttonHandlerCallback(const std_msgs::msg::String::SharedPtr msg)
// {
//   if (enable_ == false)
//     return;

//   if (msg->data == "start")
//   {

//   }
//   else if (msg->data == "mode")
//   {

//   }
// }

// void VisionDemo::demoCommandCallback(const std_msgs::msg::String::SharedPtr msg)
// {
//   if (enable_ == false)
//     return;

//   if (msg->data == "start")
//   {

//   }
//   else if (msg->data == "stop")
//   {

//   }
// }

void VisionDemo::setModuleToDemo(const std::string &module_name)
{
  callServiceSettingModule(module_name);
  RCLCPP_INFO(rclcpp::get_logger("VisionDemo"), "VisionDemo::setModuleToDemo - enable module : %s", module_name.c_str());
}

void VisionDemo::callServiceSettingModule(const std::string &module_name)
{
  auto temp_node = std::make_shared<rclcpp::Node>("vision_call_service");
  auto set_joint_module_client_ = temp_node->create_client<robotis_controller_msgs::srv::SetModule>("/robotis/set_present_ctrl_modules");
  auto request = std::make_shared<robotis_controller_msgs::srv::SetModule::Request>();
  request->module_name = module_name;

  if (!set_joint_module_client_->wait_for_service(std::chrono::seconds(1)))
  {
    RCLCPP_ERROR(rclcpp::get_logger("VisionDemo"), "VisionDemo::callServiceSettingModule - Service not available");
    return;
  }

  auto future = set_joint_module_client_->async_send_request(request);
  if (rclcpp::spin_until_future_complete(temp_node, future) == rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_INFO(rclcpp::get_logger("VisionDemo"), "VisionDemo::callServiceSettingModule - result : %d", future.get()->result);
  }
}

void VisionDemo::facePositionCallback(const std_msgs::msg::Int32MultiArray::SharedPtr msg)
{
  if (enable_ == false)
    return;

  // face is detected
  if (msg->data.size() >= 10)
  {
    // center of face
    face_position_.x = (msg->data[6] + msg->data[8] * 0.5) / msg->data[2] * 2 - 1;
    face_position_.y = (msg->data[7] + msg->data[9] * 0.5) / msg->data[3] * 2 - 1;
    face_position_.z = msg->data[8] * 0.5 + msg->data[9] * 0.5;

    face_tracker_.setFacePosition(face_position_);
  }
  else
  {
    face_position_.x = 0;
    face_position_.y = 0;
    face_position_.z = 0;
    return;
  }
}

void VisionDemo::playMotion(int motion_index)
{
  if (node_ == nullptr)
  {
    RCLCPP_ERROR(rclcpp::get_logger("VisionDemo"), "Node is not set, cannot play motion");
    return;
  }
  auto motion_index_pub_ = node_->create_publisher<std_msgs::msg::Int32>("/robotis/action/page_num", 10);
  std_msgs::msg::Int32 motion_msg;
  motion_msg.data = motion_index;

  motion_index_pub_->publish(motion_msg);
}

void VisionDemo::setRGBLED(int blue, int green, int red)
{
  if (node_ == nullptr)
  {
    RCLCPP_ERROR(rclcpp::get_logger("VisionDemo"), "Node is not set, cannot set RGB LED");
    return;
  }
  auto rgb_led_pub_ = node_->create_publisher<robotis_controller_msgs::msg::SyncWriteItem>("/robotis/sync_write_item", 10);
  int led_full_unit = 0x1F;
  int led_value = (blue & led_full_unit) << 10 | (green & led_full_unit) << 5 | (red & led_full_unit);
  robotis_controller_msgs::msg::SyncWriteItem syncwrite_msg;
  syncwrite_msg.item_name = "LED_RGB";
  syncwrite_msg.joint_name.push_back("open-cr");
  syncwrite_msg.value.push_back(led_value);

  rgb_led_pub_->publish(syncwrite_msg);
}

} /* namespace robotis_op */
