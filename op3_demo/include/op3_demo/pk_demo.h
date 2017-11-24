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

#ifndef PK_DEMO_H
#define PK_DEMO_H

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Imu.h>
#include <boost/thread.hpp>

#include "op3_demo/op_demo.h"
#include "op3_demo/ball_tracker.h"
#include "op3_demo/ball_follower.h"
#include "robotis_math/robotis_linear_algebra.h"
#include "op3_action_module_msgs/IsRunning.h"
#include "robotis_controller_msgs/SyncWriteItem.h"

namespace robotis_op
{

class PKDemo : public OPDemo
{
 public:
  enum Stand_Status
  {
    Stand = 0,
    Fallen_Forward = 1,
    Fallen_Behind = 2,
  };

  enum Robot_Status
  {
    Waited = 0,
    TrackingAndFollowing = 1,
    ReadyToKick = 2,
    ReadyToCeremony = 3,
    ReadyToGetup = 4,
  };

  PKDemo();
  ~PKDemo();

  void setDemoEnable();
  void setDemoDisable();

 protected:
  const double FALL_FORWARD_LIMIT;
  const double FALL_BACK_LIMIT;
  const int SPIN_RATE;
  const bool DEBUG_PRINT;

  void processThread();
  void callbackThread();

  void setBodyModuleToDemo(const std::string &body_module, bool with_head_control = true);
  void setModuleToDemo(const std::string &module_name);
  void parseJointNameFromYaml(const std::string &path);
  bool getJointNameFromID(const int &id, std::string &joint_name);
  bool getIDFromJointName(const std::string &joint_name, int &id);
  int getJointCount();
  bool isHeadJoint(const int &id);
  void buttonHandlerCallback(const std_msgs::String::ConstPtr& msg);
  void demoCommandCallback(const std_msgs::String::ConstPtr& msg);
  void imuDataCallback(const sensor_msgs::Imu::ConstPtr& msg);

  void startSoccerMode();
  void stopSoccerMode();

  void process();
  void handleKick(int ball_position);
  bool handleFallen(int fallen_status);

  void playMotion(int motion_index);
  void setRGBLED(int blue, int green, int red);
  bool isActionRunning();

  BallTracker ball_tracker_;
  BallFollower ball_follower_;
  OPDemo demo_node_;

  ros::Publisher module_control_pub_;
  ros::Publisher motion_index_pub_;
  ros::Publisher rgb_led_pub_;
  ros::Subscriber buttuon_sub_;
  ros::Subscriber demo_command_sub_;
  ros::Subscriber imu_data_sub_;

  ros::ServiceClient is_running_client_;

  std::map<int, std::string> id_joint_table_;
  std::map<std::string, int> joint_id_table_;

  bool is_grass_;
  int wait_count_;
  bool on_following_ball_;
  bool on_tracking_ball_;
  bool after_shooting_;
  bool restart_soccer_;
  bool start_following_;
  bool stop_following_;
  bool start_tracking_;
  bool stop_tracking_;
  bool stop_fallen_check_;
  int robot_status_;
  int tracking_status_;
  int stand_state_;
  int count;
  double present_pitch_;
};

}  // namespace robotis_op
#endif // PK_DEMO_H
