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

#ifndef OP3_TELE_OP_H_
#define OP3_TELE_OP_H_

#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/Joy.h>

#include "robotis_controller_msgs/SyncWriteItem.h"
#include "robotis_controller_msgs/GetJointModule.h"
#include "op3_walking_module_msgs/WalkingParam.h"
#include "op3_walking_module_msgs/GetWalkingParam.h"

namespace robotis_op
{

class OP3Teleop
{
 public:
  enum PS3_BUTTON
  {
    PS3_BUTTON_SELECT          = 0,
    PS3_BUTTON_STICK_LEFT      = 1,
    PS3_BUTTON_STICK_RIGHT     = 2,
    PS3_BUTTON_START           = 3,
    PS3_BUTTON_CROSS_UP        = 4,
    PS3_BUTTON_CROSS_RIGHT     = 5,
    PS3_BUTTON_CROSS_DOWN      = 6,
    PS3_BUTTON_CROSS_LEFT      = 7,
    PS3_BUTTON_REAR_LEFT_2     = 8,
    PS3_BUTTON_REAR_RIGHT_2    = 9,
    PS3_BUTTON_REAR_LEFT_1     = 10,
    PS3_BUTTON_REAR_RIGHT_1    = 11,
    PS3_BUTTON_ACTION_TRIANGLE = 12,
    PS3_BUTTON_ACTION_CIRCLE   = 13,
    PS3_BUTTON_ACTION_CROSS    = 14,
    PS3_BUTTON_ACTION_SQUARE   = 15,
    PS3_BUTTON_PAIRING         = 16,
  };

  enum PS3_AXIS
  {
    PS3_AXIS_STICK_LEFT_LEFTWARDS   = 0,
    PS3_AXIS_STICK_LEFT_UPWARDS     = 1,
    PS3_AXIS_STICK_RIGHT_LEFTWARDS  = 2,
    PS3_AXIS_STICK_RIGHT_UPWARDS    = 3,
    PS3_AXIS_BUTTON_CROSS_UP        = 4,
    PS3_AXIS_BUTTON_CROSS_RIGHT     = 5,
    PS3_AXIS_BUTTON_CROSS_DOWN      = 6,
    PS3_AXIS_BUTTON_CROSS_LEFT      = 7,
    PS3_AXIS_BUTTON_REAR_LEFT_2     = 8,
    PS3_AXIS_BUTTON_REAR_RIGHT_2    = 9,
    PS3_AXIS_BUTTON_REAR_LEFT_1     = 10,
    PS3_AXIS_BUTTON_REAR_RIGHT_1    = 11,
    PS3_AXIS_BUTTON_ACTION_TRIANGLE = 12,
    PS3_AXIS_BUTTON_ACTION_CIRCLE   = 13,
    PS3_AXIS_BUTTON_ACTION_CROSS    = 14,
    PS3_AXIS_BUTTON_ACTION_SQUARE   = 15,
    PS3_AXIS_ACCELEROMETER_LEFT     = 16,
    PS3_AXIS_ACCELEROMETER_FORWARD  = 17,
    PS3_AXIS_ACCELEROMETER_UP       = 18,
    PS3_AXIS_GYRO_YAW               = 19,
  };

  enum CONTROL_STATUS
  {
    NONE = 0,
    BASE = 1,
    WALKING = 2,
    ACTION = 3,
  };

  enum Motion_Index
  {
    InitPose = 1,
    WalkingReady = 9,
    GetUpFront = 122,
    GetUpBack = 123,
    RightKick = 121,
    LeftKick = 120,
    Ceremony = 27,
    ForGrass = 20,
  };

  const bool DEBUG_PRINT;

  OP3Teleop();
  OP3Teleop(const bool debug_print);
  ~OP3Teleop();

  void goInitPose();
  void playSound(const std::string &path);
  void setLED(int led);

  int current_status;

 private:
  const double MAX_FB_STEP;
  const double MAX_RL_STEP;
  const double MAX_RL_TURN;
  const int SPIN_RATE;

  void joyHandlerCallback(const sensor_msgs::Joy::ConstPtr& msg);
  bool checkManagerRunning(std::string& manager_name);
  void handleWalking(double fb_value, double rl_value, double rot_value);
  void handleAction(std::vector<int> buttons);

  void setWalkingCommand(const std::string &command);
  void setWalkingParam(double x_move, double y_move, double rotation_angle, bool balance);
  bool getWalkingParam();

  void setControlMode(const std::string &mode);
  void playMotion(int motion_index);

  bool is_walking_;
  bool is_playing_action_;

  ros::NodeHandle nh_;

  ros::Publisher init_pose_pub_;
  ros::Publisher play_sound_pub_;
  ros::Publisher led_pub_;
  ros::Publisher motion_index_pub_;

  ros::Publisher module_control_preset_pub_;
  ros::Publisher set_walking_command_pub_;
  ros::Publisher set_walking_param_pub_;
  ros::Subscriber joy_sub_;
  ros::ServiceClient get_module_control_client_;
  ros::ServiceClient get_walking_param_client_;
  op3_walking_module_msgs::WalkingParam current_walking_param_;

  std::string default_mp3_path_ = "";
};

} /* namespace robotis_op */

#endif
