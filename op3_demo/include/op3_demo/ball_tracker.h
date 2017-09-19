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

#ifndef BALL_TRACKING_H_
#define BALL_TRACKING_H_

#include <math.h>
#include <yaml-cpp/yaml.h>

#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/JointState.h>

#include "robotis_controller_msgs/JointCtrlModule.h"
#include "ball_detector/circleSetStamped.h"
#include "op3_walking_module_msgs/WalkingParam.h"
#include "op3_walking_module_msgs/GetWalkingParam.h"

namespace robotis_op
{

// head tracking for looking the ball
class BallTracker
{
 public:
  enum TrackingStatus
  {
    NotFound = -1,
    Waiting = 0,
    Found = 1,

  };
  BallTracker();
  ~BallTracker();

  int processTracking();

  void startTracking();
  void stopTracking();

  void setUsingHeadScan(bool use_scan);

  double getPanOfBall()
  {
    // left (+) ~ right (-)
    return current_ball_pan_;
  }
  double getTiltOfBall()
  {
    // top (+) ~ bottom (-)
    return current_ball_tilt_;
  }
  double getBallSize()
  {
    return current_ball_bottom_;
  }

 protected:
  const double FOV_WIDTH;
  const double FOV_HEIGHT;
  const int NOT_FOUND_THRESHOLD;
  const int WAITING_THRESHOLD;
  const bool DEBUG_PRINT;

  void ballPositionCallback(const ball_detector::circleSetStamped::ConstPtr &msg);
  void ballTrackerCommandCallback(const std_msgs::String::ConstPtr &msg);
  void presentJointStatesCallback(const sensor_msgs::JointState::ConstPtr &msg);
  void publishHeadJoint(double pan, double tilt);
  void scanBall();

  //ros node handle
  ros::NodeHandle nh_;

  //image publisher/subscriber
  ros::Publisher module_control_pub_;
  ros::Publisher head_joint_pub_;
  ros::Publisher head_scan_pub_;

  ros::Publisher motion_index_pub_;

  ros::Subscriber ball_position_sub_;
  ros::Subscriber ball_tracking_command_sub_;
  ros::Subscriber present_joint_states_sub_;

  // (x, y) is the center position of the ball in image coordinates
  // z is the ball radius
  geometry_msgs::Point ball_position_;

  double head_pan_, head_tilt_;

  int tracking_status_;
  bool use_head_scan_;
  int count_not_found_;
  bool on_tracking_;
  double current_ball_pan_, current_ball_tilt_;
  double current_ball_bottom_;
  ros::Time prev_time_;

};
}

#endif /* BALL_TRACKING_H_ */
