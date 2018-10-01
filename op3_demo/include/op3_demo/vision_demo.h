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

#ifndef VISION_DEMO_H_
#define VISION_DEMO_H_

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32MultiArray.h>
#include <geometry_msgs/Point.h>
#include <boost/thread.hpp>

#include "robotis_controller_msgs/SyncWriteItem.h"
#include "robotis_controller_msgs/SetModule.h"

#include "op3_demo/op_demo.h"
#include "op3_demo/face_tracker.h"

namespace robotis_op
{

class VisionDemo : public OPDemo
{
 public:
  VisionDemo();
  ~VisionDemo();

  void setDemoEnable();
  void setDemoDisable();

 protected:
  const int SPIN_RATE;
  const int TIME_TO_INIT;

  void processThread();
  void callbackThread();

  void process();

  void playMotion(int motion_index);
  void setRGBLED(int blue, int green, int red);

  void buttonHandlerCallback(const std_msgs::String::ConstPtr& msg);
  void facePositionCallback(const std_msgs::Int32MultiArray::ConstPtr &msg);
  void demoCommandCallback(const std_msgs::String::ConstPtr &msg);

  void setModuleToDemo(const std::string &module_name);
  void callServiceSettingModule(const std::string &module_name);

  FaceTracker face_tracker_;

  ros::Publisher module_control_pub_;
  ros::Publisher motion_index_pub_;
  ros::Publisher rgb_led_pub_;  
  ros::Publisher face_tracking_command_pub_;

  ros::Subscriber buttuon_sub_;
  ros::Subscriber faceCoord_sub_;

  ros::ServiceClient set_joint_module_client_;
  geometry_msgs::Point face_position_;

  ros::Time prev_time_;

  int tracking_status_;
};

} /* namespace robotis_op */

#endif /* VISION_DEMO_H_ */
