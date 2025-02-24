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

#include "op3_demo/face_tracker.h"

namespace robotis_op
{

FaceTracker::FaceTracker()
    : Node("face_tracker"),
      FOV_WIDTH(35.2 * M_PI / 180),
      FOV_HEIGHT(21.6 * M_PI / 180),
      NOT_FOUND_THRESHOLD(50),
      use_head_scan_(false),
      count_not_found_(0),
      on_tracking_(false)
{
  head_joint_offset_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/robotis/head_control/set_joint_states_offset", 10);
  head_joint_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/robotis/head_control/set_joint_states", 10);
  head_scan_pub_ = this->create_publisher<std_msgs::msg::String>("/robotis/head_control/scan_command", 10);

  face_position_sub_ = this->create_subscription<geometry_msgs::msg::Point>("/face_position", 10, std::bind(&FaceTracker::facePositionCallback, this, std::placeholders::_1));
  //face_tracking_command_sub_ = this->create_subscription<std_msgs::msg::String>("/robotis/demo_command", 10, std::bind(&FaceTracker::faceTrackerCommandCallback, this, std::placeholders::_1));
}

FaceTracker::~FaceTracker()
{

}

void FaceTracker::facePositionCallback(const geometry_msgs::msg::Point::SharedPtr msg)
{
  if (msg->z < 0)
    return;

  face_position_ = *msg;
}

void FaceTracker::faceTrackerCommandCallback(const std_msgs::msg::String::SharedPtr msg)
{
  if (msg->data == "start")
  {
    startTracking();
  }
  else if (msg->data == "stop")
  {
    stopTracking();
  }
  else if (msg->data == "toggle_start")
  {
    if (on_tracking_ == false)
      startTracking();
    else
      stopTracking();
  }
}

void FaceTracker::startTracking()
{
  on_tracking_ = true;

  RCLCPP_INFO(this->get_logger(), "Start Face tracking");
}

void FaceTracker::stopTracking()
{
  on_tracking_ = false;

  RCLCPP_INFO(this->get_logger(), "Stop Face tracking");
}

void FaceTracker::setUsingHeadScan(bool use_scan)
{
  use_head_scan_ = use_scan;
}

void FaceTracker::setFacePosition(geometry_msgs::msg::Point &face_position)
{
  if (face_position.z > 0)
  {
    face_position_ = face_position;
  }
}

void FaceTracker::goInit(double init_pan, double init_tile)
{
  sensor_msgs::msg::JointState head_angle_msg;

  head_angle_msg.name.push_back("head_pan");
  head_angle_msg.name.push_back("head_tilt");

  head_angle_msg.position.push_back(init_pan);
  head_angle_msg.position.push_back(init_tile);

  head_joint_pub_->publish(head_angle_msg);
}

int FaceTracker::processTracking()
{
  if (on_tracking_ == false)
  {
    face_position_.z = 0;
    count_not_found_ = 0;
    //return false;
    return Waiting;
  }

  // check ball position
  if (face_position_.z <= 0)
  {
    count_not_found_++;

    if (count_not_found_ == NOT_FOUND_THRESHOLD)
    {
      scanFace();
      //count_not_found_ = 0;
      return NotFound;
    }
    else if (count_not_found_ > NOT_FOUND_THRESHOLD)
    {
      return NotFound;
    }
    else
    {
      return Waiting;
    }

    //return false;
  }

  // if face is detected
  double x_error = -atan(face_position_.x * tan(FOV_WIDTH));
  double y_error = -atan(face_position_.y * tan(FOV_HEIGHT));

  face_position_.z = 0;
  count_not_found_ = 0;

  double p_gain = 0.5, d_gain = 0.1;
  double x_error_diff = x_error - current_face_pan_;
  double y_error_diff = y_error - current_face_tilt_;
  double x_error_target = x_error * p_gain + x_error_diff * d_gain;
  double y_error_target = y_error * p_gain + y_error_diff * d_gain;

  // move head joint
  publishHeadJoint(x_error_target, y_error_target);

  current_face_pan_ = x_error;
  current_face_tilt_ = y_error;

  // return true;
  return Found;
}

void FaceTracker::publishHeadJoint(double pan, double tilt)
{
  double min_angle = 1 * M_PI / 180;
  if (fabs(pan) < min_angle && fabs(tilt) < min_angle)
  {
    dismissed_count_ += 1;
    return;
  }
  std::cout << "Target angle[" << dismissed_count_ << "] : " << pan << " | " << tilt << std::endl;

  dismissed_count_ = 0;

  sensor_msgs::msg::JointState head_angle_msg;

  head_angle_msg.name.push_back("head_pan");
  head_angle_msg.name.push_back("head_tilt");

  head_angle_msg.position.push_back(pan);
  head_angle_msg.position.push_back(tilt);

  head_joint_offset_pub_->publish(head_angle_msg);
}

void FaceTracker::scanFace()
{
  if (use_head_scan_ == false)
    return;

  // check head control module enabled
  // ...

  // send message to head control module
  std_msgs::msg::String scan_msg;
  scan_msg.data = "scan";

  head_scan_pub_->publish(scan_msg);
  // RCLCPP_INFO(this->get_logger(), "Scan the ball");
}

}
