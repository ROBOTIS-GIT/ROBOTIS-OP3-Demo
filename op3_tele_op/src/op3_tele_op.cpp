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
    : nh_(ros::this_node::getName()),
      MAX_FB_STEP(40.0 * 0.001),
      MAX_RL_STEP(25 * 0.001),
      MAX_RL_TURN(15.0 * M_PI / 180),
      is_walking_(false),
      is_playing_action_(false),
      current_status(NONE),
      DEBUG_PRINT(debug_print),
      SPIN_RATE(30)
{
  //ros::NodeHandle nh(ros::this_node::getName());

  init_pose_pub_ = nh_.advertise<std_msgs::String>("/robotis/base/ini_pose", 0);
  play_sound_pub_ = nh_.advertise<std_msgs::String>("/play_sound_file", 0);
  led_pub_ = nh_.advertise<robotis_controller_msgs::SyncWriteItem>("/robotis/sync_write_item", 0);
  motion_index_pub_ = nh_.advertise<std_msgs::Int32>("/robotis/action/page_num", 0);

  set_walking_command_pub_ = nh_.advertise<std_msgs::String>("/robotis/walking/command", 0);
  set_walking_param_pub_ = nh_.advertise<op3_walking_module_msgs::WalkingParam>("/robotis/walking/set_params", 0);
  module_control_preset_pub_ = nh_.advertise<std_msgs::String>("/robotis/enable_ctrl_module", 0);

  get_walking_param_client_ = nh_.serviceClient<op3_walking_module_msgs::GetWalkingParam>(
      "/robotis/walking/get_params");
  get_module_control_client_ = nh_.serviceClient<robotis_controller_msgs::GetJointModule>(
      "/robotis/get_present_joint_ctrl_modules");

  joy_sub_ = nh_.subscribe("/joy", 1, &OP3Teleop::joyHandlerCallback, this);

  default_mp3_path_ = ros::package::getPath("op3_demo") + "/Data/mp3/";

  std::cout << "OP3Teleop initialzed " << std::endl;
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
  // check current status and button
  // ...
  // set module
  if (msg->buttons[PS3_BUTTON_REAR_LEFT_2] && current_status != WALKING)
  {
    // set walking Module
    setControlMode("walking_module");
    current_status = WALKING;
  }
  else if (msg->buttons[PS3_AXIS_BUTTON_REAR_RIGHT_2] && current_status != ACTION)
  {
    // set action Module
    setControlMode("action_module");
    current_status = ACTION;
  }
  else
  {
    switch (current_status)
    {
      case WALKING:
      {
        // walking module
        // check axis and publish movement command
        handleWalking(msg->axes[PS3_AXIS_STICK_LEFT_UPWARDS], msg->axes[PS3_AXIS_STICK_RIGHT_LEFTWARDS],
                      msg->axes[PS3_AXIS_STICK_LEFT_LEFTWARDS]);

        // check button and handle action
        // handleAction();

        break;
      }

      case ACTION:
      {
        // action module
        // check button and handle action
        handleAction(msg->buttons);

        break;
      }

      default:
      {

        break;
      }
    }
  }
}

void OP3Teleop::setWalkingCommand(const std::string &command)
{
  // get param
  if (command == "start")
  {
    getWalkingParam();
    setWalkingParam(0.0, 0, 0, true);
    is_walking_ = true;
  }
  else
    is_walking_ = false;

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

void OP3Teleop::handleWalking(double fb_value, double rl_value, double rot_value)
{
  // check whether status is walking
  // ....
  ROS_INFO_STREAM("FB : " << fb_value << ", RL : " << rl_value << ", ROT : " << rot_value);

  if (fb_value == 0.0 && rot_value == 0.0 && rl_value == 0.0)
  {
    setWalkingCommand("stop");

    return;
  }

  // make walking parameter
  double fb_length = 0.0, rl_length = 0.0, rot_angle = 0.0;
  getWalkingParam();

  if (rl_value != 0)
  {
    rl_length = rl_value * MAX_RL_STEP;
  }
  else
  {
    fb_length = fb_value * MAX_FB_STEP;
    rot_angle = rot_value * MAX_RL_TURN;

    if (fb_value < 0)
      rot_angle *= -1;
  }

  // set walking parameter
  if (is_walking_ == true)
  {
    setWalkingParam(fb_length, rl_length, rot_angle, true);
  }
  else
  {
    setWalkingCommand("start");
  }
}

void OP3Teleop::handleAction(std::vector<int> buttons)
{
  // check whether status is action
  // ....
  if (buttons[PS3_AXIS_BUTTON_ACTION_SQUARE])
  {
    // left kick
    playMotion(LeftKick);
  }
  else if (buttons[PS3_BUTTON_ACTION_CIRCLE])
  {
    // right kick
    playMotion(RightKick);
  }
  else if (buttons[PS3_BUTTON_ACTION_TRIANGLE])
  {
    playMotion(GetUpFront);
  }
  else if (buttons[PS3_BUTTON_ACTION_CROSS])
  {
    playMotion(GetUpBack);
  }
}

void OP3Teleop::setControlMode(const std::string &mode)
{
  std_msgs::String set_module_msg;
  set_module_msg.data = mode;

  module_control_preset_pub_.publish(set_module_msg);

  ROS_INFO_STREAM("Set Mode : " << mode);
}

void OP3Teleop::playMotion(int motion_index)
{
  std_msgs::Int32 motion_msg;
  motion_msg.data = motion_index;

  motion_index_pub_.publish(motion_msg);
}

} /* namespace robotis_op */
