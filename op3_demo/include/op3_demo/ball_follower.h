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

#ifndef BALL_FOLLOWER_H_
#define BALL_FOLLOWER_H_

#include <math.h>
#include <numeric>
#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Point.h>
#include <yaml-cpp/yaml.h>

#include "robotis_controller_msgs/JointCtrlModule.h"
#include "op3_ball_detector/CircleSetStamped.h"
#include "op3_walking_module_msgs/WalkingParam.h"
#include "op3_walking_module_msgs/GetWalkingParam.h"

namespace robotis_op
{

// following the ball using walking
class BallFollower
{
 public:
  enum
  {
    NotFound = 0,
    OutOfRange = 1,
    OnRight = 2,
    OnLeft = 3,
  };

  BallFollower();
  ~BallFollower();

  bool processFollowing(double x_angle, double y_angle, double ball_size);
  void decideBallPositin(double x_angle, double y_angle);
  void waitFollowing();
  void startFollowing();
  void stopFollowing();
  void clearBallPosition()
  {
    approach_ball_position_ = NotFound;
  }

  int getBallPosition()
  {
    return approach_ball_position_;
  }

  bool isBallInRange()
  {
    return (approach_ball_position_ == OnRight || approach_ball_position_ == OnLeft);
  }

 protected:
  const bool DEBUG_PRINT;
  const double CAMERA_HEIGHT;
  const int NOT_FOUND_THRESHOLD;
  const double FOV_WIDTH;
  const double FOV_HEIGHT;
  const double MAX_FB_STEP;
  const double MAX_RL_TURN;
  const double IN_PLACE_FB_STEP;
  const double MIN_FB_STEP;
  const double MIN_RL_TURN;
  const double UNIT_FB_STEP;
  const double UNIT_RL_TURN;

  const double SPOT_FB_OFFSET;
  const double SPOT_RL_OFFSET;
  const double SPOT_ANGLE_OFFSET;

  void currentJointStatesCallback(const sensor_msgs::JointState::ConstPtr &msg);
  void setWalkingCommand(const std::string &command);
  void setWalkingParam(double x_move, double y_move, double rotation_angle, bool balance = true);
  bool getWalkingParam();
  void calcFootstep(double target_distance, double target_angle, double delta_time,
                    double& fb_move, double& rl_angle);

  //ros node handle
  ros::NodeHandle nh_;

  //image publisher/subscriber
  ros::Publisher module_control_pub_;
  ros::Publisher head_joint_pub_;
  ros::Publisher head_scan_pub_;
  ros::Publisher set_walking_command_pub_;
  ros::Publisher set_walking_param_pub_;

  ros::Publisher motion_index_pub_;
  ros::ServiceClient get_walking_param_client_;

  ros::Subscriber ball_position_sub_;
  ros::Subscriber ball_tracking_command_sub_;
  ros::Subscriber current_joint_states_sub_;

  // (x, y) is the center position of the ball in image coordinates
  // z is the ball radius
  geometry_msgs::Point ball_position_;
  op3_walking_module_msgs::WalkingParam current_walking_param_;

  int count_not_found_;
  int count_to_kick_;
  bool on_tracking_;
  int approach_ball_position_;
  double current_pan_, current_tilt_;
  double current_x_move_, current_r_angle_;
  int kick_motion_index_;
  double hip_pitch_offset_;
  ros::Time prev_time_;

  double curr_period_time_;
  double accum_period_time_;

};
}

#endif /* BALL_FOLLOWER_H_ */
