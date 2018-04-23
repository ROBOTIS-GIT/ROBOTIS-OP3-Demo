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

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/JointState.h>

#include "robotis_controller_msgs/SetModule.h"
#include "robotis_controller_msgs/SyncWriteItem.h"

void buttonHandlerCallback(const std_msgs::String::ConstPtr& msg);
void jointstatesCallback(const sensor_msgs::JointState::ConstPtr& msg);
void setModule(const std::string& module_name);
void goInitPose();
void setLED(int led);
bool checkManagerRunning(std::string& manager_name);
void torqueOnAll();
void torqueOff(const std::string& body_side);

const int SPIN_RATE = 30;
const bool DEBUG_PRINT = false;

ros::Publisher init_pose_pub;
//ros::Publisher play_sound_pub;
ros::Publisher sync_write_pub;
ros::Publisher dxl_torque_pub;
ros::Publisher write_joint_pub;
ros::Subscriber buttuon_sub;
ros::Subscriber read_joint_sub;

ros::ServiceClient set_joint_module_client;

//std::string default_mp3_path = "";
bool demo_ready = false;

//node main
int main(int argc, char **argv)
{
  //init ros
  ros::init(argc, argv, "read_write");

  ros::NodeHandle nh(ros::this_node::getName());

  init_pose_pub = nh.advertise<std_msgs::String>("/robotis/base/ini_pose", 0);
  //  play_sound_pub = nh.advertise<std_msgs::String>("/play_sound_file", 0);
  sync_write_pub = nh.advertise<robotis_controller_msgs::SyncWriteItem>("/robotis/sync_write_item", 0);
  dxl_torque_pub = nh.advertise<std_msgs::String>("/robotis/dxl_torque", 0);
  write_joint_pub = nh.advertise<sensor_msgs::JointState>("/robotis/direct_control/set_joint_states", 0);

  read_joint_sub = nh.subscribe("/robotis/present_joint_ctrl_modules", 1, jointstatesCallback);
  buttuon_sub = nh.subscribe("/robotis/open_cr/button", 1, buttonHandlerCallback);

  //  default_mp3_path = ros::package::getPath("op3_demo") + "/data/mp3/";

  // service
  set_joint_module_client = nh.serviceClient<robotis_controller_msgs::SetModule>("/robotis/set_present_ctrl_modules");

  ros::start();

  //set node loop rate
  ros::Rate loop_rate(SPIN_RATE);

  // wait for starting of manager
  std::string manager_name = "/op3_manager";
  while (ros::ok())
  {
    ros::Duration(1.0).sleep();

    if (checkManagerRunning(manager_name) == true)
    {
      break;
      ROS_INFO_COND(DEBUG_PRINT, "Succeed to connect");
    }
    ROS_WARN("Waiting for op3 manager");
  }

  // init procedure
  goInitPose();

  // turn on R/G/B LED [0x01 | 0x02 | 0x04]
  setLED(0x01);

  // change the module to direct_control for demo
  setModule("direct_control");

  // torque off : right arm
  torqueOff("right");

//  demo_ready = true;

  //node loop
  while (ros::ok())
  {
    // process


    //execute pending callbacks
    ros::spinOnce();

    //relax to fit output rate
    loop_rate.sleep();
  }

  //exit program
  return 0;
}

void buttonHandlerCallback(const std_msgs::String::ConstPtr& msg)
{
  //    if (msg->data == "mode_long")
  //    else if (msg->data == "user_long")
      if (msg->data == "start")
        demo_ready = false;
      else if (msg->data == "mode")
        demo_ready = true;
}

void jointstatesCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
  if(demo_ready == false)
    return;

  sensor_msgs::JointState write_msg;
  for(int ix = 0; ix < msg->name.size(); ix++)
  {
    std::string joint_name = msg->name[ix];
    double joint_position = msg->position[ix];

    if(joint_name == "r_sho_pitch")
    {
      write_msg.name.push_back("r_sho_pitch");
      write_msg.position.push_back(-joint_position);
    }
    if(joint_name == "r_sho_roll")
    {
      write_msg.name.push_back("l_sho_roll");
      write_msg.position.push_back(-joint_position);
    }
    if(joint_name == "r_el")
    {
      write_msg.name.push_back("l_el");
      write_msg.position.push_back(-joint_position);
    }
  }

  write_joint_pub.publish(write_msg);
}

void goInitPose()
{
  std_msgs::String init_msg;
  init_msg.data = "ini_pose";

  init_pose_pub.publish(init_msg);
}

void setLED(int led)
{
  robotis_controller_msgs::SyncWriteItem syncwrite_msg;
  syncwrite_msg.item_name = "LED";
  syncwrite_msg.joint_name.push_back("open-cr");
  syncwrite_msg.value.push_back(led);

  sync_write_pub.publish(syncwrite_msg);
}

bool checkManagerRunning(std::string& manager_name)
{
  std::vector<std::string> node_list;
  ros::master::getNodes(node_list);

  for (unsigned int node_list_idx = 0; node_list_idx < node_list.size(); node_list_idx++)
  {
    if (node_list[node_list_idx] == manager_name)
      return true;
  }

  ROS_ERROR("Can't find op3_manager");
  return false;
}

void setModule(const std::string& module_name)
{
  robotis_controller_msgs::SetModule set_module_srv;
  set_module_srv.request.module_name = module_name;

  if (set_joint_module_client.call(set_module_srv) == false)
  {
    ROS_ERROR("Failed to set module");
    return;
  }

  return ;
}

void torqueOnAll()
{
  std_msgs::String check_msg;
  check_msg.data = "check";

  dxl_torque_pub.publish(check_msg);
}

void torqueOff(const std::string& body_side)
{
  robotis_controller_msgs::SyncWriteItem syncwrite_msg;
  int torque_value = 0;
  syncwrite_msg.item_name = "torque_enable";

  if(body_side == "right")
  {
    syncwrite_msg.joint_name.push_back("r_sho_pitch");
    syncwrite_msg.value.push_back(torque_value);
    syncwrite_msg.joint_name.push_back("r_sho_roll");
    syncwrite_msg.value.push_back(torque_value);
    syncwrite_msg.joint_name.push_back("r_el");
    syncwrite_msg.value.push_back(torque_value);
  }
  else if(body_side == "left")
  {
    syncwrite_msg.joint_name.push_back("l_sho_pitch");
    syncwrite_msg.value.push_back(torque_value);
    syncwrite_msg.joint_name.push_back("l_sho_roll");
    syncwrite_msg.value.push_back(torque_value);
    syncwrite_msg.joint_name.push_back("l_el");
    syncwrite_msg.value.push_back(torque_value);
  }
  else
    return;

  sync_write_pub.publish(syncwrite_msg);
}
