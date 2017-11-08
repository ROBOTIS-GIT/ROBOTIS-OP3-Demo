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

  // (x, y) is the center position of the ball in image coordinates
  // z is the ball radius
  geometry_msgs::Point ball_position_;

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
