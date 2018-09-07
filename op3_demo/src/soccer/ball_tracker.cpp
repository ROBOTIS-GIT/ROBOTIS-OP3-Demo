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

#include "op3_demo/ball_tracker.h"

namespace robotis_op
{

BallTracker::BallTracker()
  : nh_(ros::this_node::getName()),
    FOV_WIDTH(35.2 * M_PI / 180),
    FOV_HEIGHT(21.6 * M_PI / 180),
    NOT_FOUND_THRESHOLD(50),
    WAITING_THRESHOLD(5),
    use_head_scan_(true),
    count_not_found_(0),
    on_tracking_(false),
    current_ball_pan_(0),
    current_ball_tilt_(0),
    x_error_sum_(0),
    y_error_sum_(0),
    current_ball_bottom_(0),
    tracking_status_(NotFound),
    DEBUG_PRINT(false)
{
  ros::NodeHandle param_nh("~");
  p_gain_ = param_nh.param("p_gain", 0.4);
  i_gain_ = param_nh.param("i_gain", 0.0);
  d_gain_ = param_nh.param("d_gain", 0.0);

  ROS_INFO_STREAM("Ball tracking Gain : " << p_gain_ << ", " << i_gain_ << ", " << d_gain_);

  head_joint_offset_pub_ = nh_.advertise<sensor_msgs::JointState>("/robotis/head_control/set_joint_states_offset", 0);
  head_joint_pub_ = nh_.advertise<sensor_msgs::JointState>("/robotis/head_control/set_joint_states", 0);
  head_scan_pub_ = nh_.advertise<std_msgs::String>("/robotis/head_control/scan_command", 0);
  //  error_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("/ball_tracker/errors", 0);

  ball_position_sub_ = nh_.subscribe("/ball_detector_node/circle_set", 1, &BallTracker::ballPositionCallback, this);
  ball_tracking_command_sub_ = nh_.subscribe("/ball_tracker/command", 1, &BallTracker::ballTrackerCommandCallback,
                                             this);
}

BallTracker::~BallTracker()
{

}

void BallTracker::ballPositionCallback(const op3_ball_detector::CircleSetStamped::ConstPtr &msg)
{
  for (int idx = 0; idx < msg->circles.size(); idx++)
  {
    if (ball_position_.z >= msg->circles[idx].z)
      continue;

    ball_position_ = msg->circles[idx];
  }
}

void BallTracker::ballTrackerCommandCallback(const std_msgs::String::ConstPtr &msg)
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

void BallTracker::startTracking()
{
  on_tracking_ = true;
  ROS_INFO_COND(DEBUG_PRINT, "Start Ball tracking");
}

void BallTracker::stopTracking()
{
  goInit();

  on_tracking_ = false;
  ROS_INFO_COND(DEBUG_PRINT, "Stop Ball tracking");

  current_ball_pan_ = 0;
  current_ball_tilt_ = 0;
  x_error_sum_ = 0;
  y_error_sum_ = 0;
}

void BallTracker::setUsingHeadScan(bool use_scan)
{
  use_head_scan_ = use_scan;
}

int BallTracker::processTracking()
{
  int tracking_status = Found;

  if (on_tracking_ == false)
  {
    ball_position_.z = 0;
    count_not_found_ = 0;
    return NotFound;
  }

  // check ball position
  if (ball_position_.z <= 0)
  {
    count_not_found_++;

    if (count_not_found_ < WAITING_THRESHOLD)
    {
      if(tracking_status_ == Found || tracking_status_ == Waiting)
        tracking_status = Waiting;
      else
        tracking_status = NotFound;
    }
    else if (count_not_found_ > NOT_FOUND_THRESHOLD)
    {
      scanBall();
      count_not_found_ = 0;
      tracking_status = NotFound;
    }
    else
    {
      tracking_status = NotFound;
    }
  }
  else
  {
    count_not_found_ = 0;
  }

  // if ball is found
  // convert ball position to desired angle(rad) of head
  // ball_position : top-left is (-1, -1), bottom-right is (+1, +1)
  // offset_rad : top-left(+, +), bottom-right(-, -)
  double x_error = 0.0, y_error = 0.0, ball_size = 0.0;

  switch (tracking_status)
  {
  case NotFound:
    tracking_status_ = tracking_status;
    current_ball_pan_ = 0;
    current_ball_tilt_ = 0;
    x_error_sum_ = 0;
    y_error_sum_ = 0;
    return tracking_status;

  case Waiting:
    tracking_status_ = tracking_status;
    return tracking_status;

  case Found:
    x_error = -atan(ball_position_.x * tan(FOV_WIDTH));
    y_error = -atan(ball_position_.y * tan(FOV_HEIGHT));
    ball_size = ball_position_.z;
    break;

  default:
    break;
  }

  ROS_INFO_STREAM_COND(DEBUG_PRINT, "--------------------------------------------------------------");
  ROS_INFO_STREAM_COND(DEBUG_PRINT, "Ball position : " << ball_position_.x << " | " << ball_position_.y);
  ROS_INFO_STREAM_COND(DEBUG_PRINT, "Target angle : " << (x_error * 180 / M_PI) << " | " << (y_error * 180 / M_PI));

  ros::Time curr_time = ros::Time::now();
  ros::Duration dur = curr_time - prev_time_;
  double delta_time = dur.nsec * 0.000000001 + dur.sec;
  prev_time_ = curr_time;

  double x_error_diff = (x_error - current_ball_pan_) / delta_time;
  double y_error_diff = (y_error - current_ball_tilt_) / delta_time;
  x_error_sum_ += x_error;
  y_error_sum_ += y_error;
  double x_error_target = x_error * p_gain_ + x_error_diff * d_gain_ + x_error_sum_ * i_gain_;
  double y_error_target = y_error * p_gain_ + y_error_diff * d_gain_ + y_error_sum_ * i_gain_;

  //  std_msgs::Float64MultiArray x_error_msg;
  //  x_error_msg.data.push_back(x_error);
  //  x_error_msg.data.push_back(x_error_diff);
  //  x_error_msg.data.push_back(x_error_sum_);
  //  x_error_msg.data.push_back(x_error * p_gain_);
  //  x_error_msg.data.push_back(x_error_diff * d_gain_);
  //  x_error_msg.data.push_back(x_error_sum_ * i_gain_);
  //  x_error_msg.data.push_back(x_error_target);
  //  error_pub_.publish(x_error_msg);

  ROS_INFO_STREAM_COND(DEBUG_PRINT, "------------------------  " << tracking_status << "  --------------------------------------");
  ROS_INFO_STREAM_COND(DEBUG_PRINT, "error         : " << (x_error * 180 / M_PI) << " | " << (y_error * 180 / M_PI));
  ROS_INFO_STREAM_COND(
        DEBUG_PRINT,
        "error_diff    : " << (x_error_diff * 180 / M_PI) << " | " << (y_error_diff * 180 / M_PI) << " | " << delta_time);
  ROS_INFO_STREAM_COND(
        DEBUG_PRINT,
        "error_sum    : " << (x_error_sum_ * 180 / M_PI) << " | " << (y_error_sum_ * 180 / M_PI));
  ROS_INFO_STREAM_COND(
        DEBUG_PRINT,
        "error_target  : " << (x_error_target * 180 / M_PI) << " | " << (y_error_target * 180 / M_PI) << " | P : " << p_gain_ << " | D : " << d_gain_ << " | time : " << delta_time);

  // move head joint
  publishHeadJoint(x_error_target, y_error_target);

  // args for following ball
  current_ball_pan_ = x_error;
  current_ball_tilt_ = y_error;
  current_ball_bottom_ = ball_size;

  ball_position_.z = 0;

  tracking_status_ = tracking_status;
  return tracking_status;
}

void BallTracker::publishHeadJoint(double pan, double tilt)
{
  double min_angle = 1 * M_PI / 180;
  if (fabs(pan) < min_angle && fabs(tilt) < min_angle)
    return;

  sensor_msgs::JointState head_angle_msg;

  head_angle_msg.name.push_back("head_pan");
  head_angle_msg.name.push_back("head_tilt");

  head_angle_msg.position.push_back(pan);
  head_angle_msg.position.push_back(tilt);

  head_joint_offset_pub_.publish(head_angle_msg);
}

void BallTracker::goInit()
{
  sensor_msgs::JointState head_angle_msg;

  head_angle_msg.name.push_back("head_pan");
  head_angle_msg.name.push_back("head_tilt");

  head_angle_msg.position.push_back(0.0);
  head_angle_msg.position.push_back(0.0);

  head_joint_pub_.publish(head_angle_msg);
}

void BallTracker::scanBall()
{
  if (use_head_scan_ == false)
    return;

  // check head control module enabled
  // ...

  // send message to head control module
  std_msgs::String scan_msg;
  scan_msg.data = "scan";

  head_scan_pub_.publish(scan_msg);
}

}

