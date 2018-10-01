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

#include "op3_demo/vision_demo.h"

namespace robotis_op
{

VisionDemo::VisionDemo()
  : SPIN_RATE(30),
    TIME_TO_INIT(10),
    tracking_status_(FaceTracker::Waiting)
{
  enable_ = false;

  ros::NodeHandle nh(ros::this_node::getName());

  boost::thread queue_thread = boost::thread(boost::bind(&VisionDemo::callbackThread, this));
  boost::thread process_thread = boost::thread(boost::bind(&VisionDemo::processThread, this));
}

VisionDemo::~VisionDemo()
{
  // TODO Auto-generated destructor stub
}

void VisionDemo::setDemoEnable()
{
  // set prev time for timer
  prev_time_ = ros::Time::now();

  // change to motion module
  setModuleToDemo("action_module");

  playMotion(InitPose);

  usleep(1500 * 1000);

  setModuleToDemo("head_control_module");

  enable_ = true;

  // send command to start face_tracking
  std_msgs::Bool command;
  command.data = enable_;
  face_tracking_command_pub_.publish(command);

  face_tracker_.startTracking();

  ROS_INFO("Start Vision Demo");

}

void VisionDemo::setDemoDisable()
{

  face_tracker_.stopTracking();
  tracking_status_ = FaceTracker::Waiting;
  enable_ = false;

  std_msgs::Bool command;
  command.data = enable_;
  face_tracking_command_pub_.publish(command);
}

void VisionDemo::process()
{
  int tracking_status = face_tracker_.processTracking();



  switch(tracking_status)
  {
  case FaceTracker::Found:
    if(tracking_status_ != tracking_status)
      setRGBLED(0x1F, 0x1F, 0x1F);
    prev_time_ = ros::Time::now();
    break;

  case FaceTracker::NotFound:
  {
    if(tracking_status_ != tracking_status)
      setRGBLED(0, 0, 0);

    ros::Time curr_time = ros::Time::now();
    ros::Duration dur = curr_time - prev_time_;
    if(dur.sec > TIME_TO_INIT)
    {
      face_tracker_.goInit(0,0);
      prev_time_ = curr_time;
    }
    break;
  }
  default:
    break;
  }


  if(tracking_status != FaceTracker::Waiting)
    tracking_status_ = tracking_status;
}

void VisionDemo::processThread()
{
  //set node loop rate
  ros::Rate loop_rate(SPIN_RATE);

  //node loop
  while (ros::ok())
  {
    if (enable_ == true)
      process();

    //relax to fit output rate
    loop_rate.sleep();
  }
}

void VisionDemo::callbackThread()
{
  ros::NodeHandle nh(ros::this_node::getName());

  // subscriber & publisher
  module_control_pub_ = nh.advertise<std_msgs::String>("/robotis/enable_ctrl_module", 0);
  motion_index_pub_ = nh.advertise<std_msgs::Int32>("/robotis/action/page_num", 0);
  rgb_led_pub_ = nh.advertise<robotis_controller_msgs::SyncWriteItem>("/robotis/sync_write_item", 0);
  face_tracking_command_pub_ = nh.advertise<std_msgs::Bool>("/face_tracking/command", 0);

  buttuon_sub_ = nh.subscribe("/robotis/open_cr/button", 1, &VisionDemo::buttonHandlerCallback, this);
  faceCoord_sub_ = nh.subscribe("/faceCoord", 1, &VisionDemo::facePositionCallback, this);

  set_joint_module_client_ = nh.serviceClient<robotis_controller_msgs::SetModule>("/robotis/set_present_ctrl_modules");

  while (nh.ok())
  {
    ros::spinOnce();

    usleep(1 * 1000);
  }
}

void VisionDemo::buttonHandlerCallback(const std_msgs::String::ConstPtr& msg)
{
  if (enable_ == false)
    return;

  if (msg->data == "start")
  {

  }
  else if (msg->data == "mode")
  {

  }
}

void VisionDemo::demoCommandCallback(const std_msgs::String::ConstPtr &msg)
{
  if (enable_ == false)
    return;

  if (msg->data == "start")
  {

  }
  else if (msg->data == "stop")
  {

  }
}

void VisionDemo::setModuleToDemo(const std::string &module_name)
{
  callServiceSettingModule(module_name);
  ROS_INFO_STREAM("enable module : " << module_name);
}

void VisionDemo::callServiceSettingModule(const std::string &module_name)
{
  robotis_controller_msgs::SetModule set_module_srv;
  set_module_srv.request.module_name = module_name;

  if (set_joint_module_client_.call(set_module_srv) == false)
  {
    ROS_ERROR("Failed to set module");
    return;
  }

  return ;
}

void VisionDemo::facePositionCallback(const std_msgs::Int32MultiArray::ConstPtr &msg)
{
  if (enable_ == false)
    return;

  // face is detected
  if (msg->data.size() >= 10)
  {
    // center of face
    face_position_.x = (msg->data[6] + msg->data[8] * 0.5) / msg->data[2] * 2 - 1;
    face_position_.y = (msg->data[7] + msg->data[9] * 0.5) / msg->data[3] * 2 - 1;
    face_position_.z = msg->data[8] * 0.5 + msg->data[9] * 0.5;

    face_tracker_.setFacePosition(face_position_);
  }
  else
  {
    face_position_.x = 0;
    face_position_.y = 0;
    face_position_.z = 0;
    return;
  }
}

void VisionDemo::playMotion(int motion_index)
{
  std_msgs::Int32 motion_msg;
  motion_msg.data = motion_index;

  motion_index_pub_.publish(motion_msg);
}

void VisionDemo::setRGBLED(int blue, int green, int red)
{
  int led_full_unit = 0x1F;
  int led_value = (blue & led_full_unit) << 10 | (green & led_full_unit) << 5 | (red & led_full_unit);
  robotis_controller_msgs::SyncWriteItem syncwrite_msg;
  syncwrite_msg.item_name = "LED_RGB";
  syncwrite_msg.joint_name.push_back("open-cr");
  syncwrite_msg.value.push_back(led_value);

  rgb_led_pub_.publish(syncwrite_msg);
}

} /* namespace robotis_op */
