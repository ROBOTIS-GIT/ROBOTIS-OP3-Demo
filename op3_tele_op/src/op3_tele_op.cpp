/*******************************************************************************
 * Copyright (c) 2016, ROBOTIS CO., LTD.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of ROBOTIS nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************/

/* Author: Kayman Jung */
#include "op3_tele_op/op3_tele_op.h"

namespace robotis_op
{

OP3Teleop::OP3Teleop()
    : OP3Teleop(false)
{

}

OP3Teleop::OP3Teleop(const bool debug_print)
    : nh_(),
      DEBUG_PRINT(debug_print),
      SPIN_RATE(30)
{
  //ros::NodeHandle nh(ros::this_node::getName());

  init_pose_pub_ = nh_.advertise<std_msgs::String>("/robotis/base/ini_pose", 0);
  play_sound_pub_ = nh_.advertise<std_msgs::String>("/play_sound_file", 0);
  led_pub_ = nh_.advertise<robotis_controller_msgs::SyncWriteItem>("/robotis/sync_write_item", 0);

  set_walking_command_pub_ = nh_.advertise<std_msgs::String>("/robotis/walking/command", 0);
  set_walking_param_pub_ = nh_.advertise<op3_walking_module_msgs::WalkingParam>("/robotis/walking/set_params", 0);

  get_walking_param_client_ = nh_.serviceClient<op3_walking_module_msgs::GetWalkingParam>(
      "/robotis/walking/get_params");

  ros::Subscriber joy_sub = nh_.subscribe("/robotis/open_cr/button", 1, &OP3Teleop::joyHandlerCallback, this);

  default_mp3_path_ = ros::package::getPath("op3_demo") + "/Data/mp3/";
}

OP3Teleop::~OP3Teleop()
{
  // TODO Auto-generated destructor stub
}

void OP3Teleop::goInitPose()
{
  std_msgs::String init_msg;
  init_msg.data = "ini_pose";

  init_pose_pub_.publish(init_msg);
}

void OP3Teleop::playSound(const std::string &path)
{
  std_msgs::String sound_msg;
  sound_msg.data = path;

  play_sound_pub_.publish(sound_msg);
}

void OP3Teleop::setLED(int led)
{
  robotis_controller_msgs::SyncWriteItem syncwrite_msg;
  syncwrite_msg.item_name = "LED";
  syncwrite_msg.joint_name.push_back("open-cr");
  syncwrite_msg.value.push_back(led);

  led_pub_.publish(syncwrite_msg);
}

void OP3Teleop::joyHandlerCallback(const sensor_msgs::Joy::ConstPtr& msg)
{
  //msg->buttons
  //msg->axes

  // check axis and publish movement command
  handleWalking(msg->axes[PS3_AXIS_STICK_LEFT_UPWARDS], msg->axes[PS3_AXIS_STICK_LEFT_LEFTWARDS]);

  // check button and handle action
  handleAction();
}

void OP3Teleop::setWalkingCommand(const std::string &command)
{
  // get param
  if (command == "start")
  {
    getWalkingParam();
    setWalkingParam(0.0, 0, 0, true);
  }

  std_msgs::String _command_msg;
  _command_msg.data = command;
  set_walking_command_pub_.publish(_command_msg);

  ROS_INFO_STREAM_COND(DEBUG_PRINT, "Send Walking command : " << command);
}

void OP3Teleop::setWalkingParam(double x_move, double y_move, double rotation_angle, bool balance)
{
  current_walking_param_.balance_enable = balance;
  current_walking_param_.x_move_amplitude = x_move + 0.0;
  current_walking_param_.y_move_amplitude = y_move + 0.0;
  current_walking_param_.angle_move_amplitude = rotation_angle + 0.0;

  set_walking_param_pub_.publish(current_walking_param_);
}

bool OP3Teleop::getWalkingParam()
{
  op3_walking_module_msgs::GetWalkingParam walking_param_msg;

  if (get_walking_param_client_.call(walking_param_msg))
  {
    current_walking_param_ = walking_param_msg.response.parameters;

    // update ui
    ROS_INFO_COND(DEBUG_PRINT, "Get walking parameters");

    return true;
  }
  else
  {
    ROS_ERROR("Fail to get walking parameters.");

    return false;
  }

}

void OP3Teleop::handleWalking(double fb_value, double rl_value)
{

}

void OP3Teleop::handleAction()
{

}

} /* namespace robotis_op */
