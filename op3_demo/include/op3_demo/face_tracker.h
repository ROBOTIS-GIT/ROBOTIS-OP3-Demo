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

#ifndef FACE_TRACKING_H_
#define FACE_TRACKING_H_

#include <math.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Point.h>
#include <yaml-cpp/yaml.h>

namespace robotis_op
{

// head tracking for looking the ball
class FaceTracker
{
 public:
  enum TrackingStatus
  {
    NotFound = -1,
    Waiting = 0,
    Found = 1,

  };

  FaceTracker();
  ~FaceTracker();

  int processTracking();

  void startTracking();
  void stopTracking();

  void setUsingHeadScan(bool use_scan);
  void setFacePosition(geometry_msgs::Point &face_position);
  void goInit(double init_pan, double init_tile);

  double getPanOfFace()
  {
    return current_face_pan_;
  }
  double getTiltOfFace()
  {
    return current_face_tilt_;
  }

 protected:
  const double FOV_WIDTH;
  const double FOV_HEIGHT;
  const int NOT_FOUND_THRESHOLD;

  void facePositionCallback(const geometry_msgs::Point::ConstPtr &msg);
  void faceTrackerCommandCallback(const std_msgs::String::ConstPtr &msg);
  void publishHeadJoint(double pan, double tilt);
  void scanFace();

  //ros node handle
  ros::NodeHandle nh_;

  //image publisher/subscriber
  ros::Publisher module_control_pub_;
  ros::Publisher head_joint_offset_pub_;
  ros::Publisher head_joint_pub_;
  ros::Publisher head_scan_pub_;

  ros::Subscriber face_position_sub_;
  ros::Subscriber face_tracking_command_sub_;

  // (x, y) is the center position of the ball in image coordinates
  // z is the face size
  geometry_msgs::Point face_position_;

  bool use_head_scan_;
  int count_not_found_;
  bool on_tracking_;
  double current_face_pan_, current_face_tilt_;

  int dismissed_count_;

};
}

#endif /* FACE_TRACKING_H_ */
