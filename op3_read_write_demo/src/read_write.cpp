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
void readyToDemo();
void setModule(const std::string& module_name);
void goInitPose();
void setLED(int led);
bool checkManagerRunning(std::string& manager_name);
void torqueOnAll();
void torqueOff(const std::string& body_side);

enum ControlModule
{
  None = 0,
  DirectControlModule = 1,
  Framework = 2,
};

const int SPIN_RATE = 30;
const bool DEBUG_PRINT = false;

ros::Publisher init_pose_pub;
ros::Publisher sync_write_pub;
ros::Publisher dxl_torque_pub;
ros::Publisher write_joint_pub;
ros::Publisher write_joint_pub2;
ros::Subscriber buttuon_sub;
ros::Subscriber read_joint_sub;

ros::ServiceClient set_joint_module_client;

int control_module = None;
bool demo_ready = false;

//node main
int main(int argc, char **argv)
{
  //init ros
  ros::init(argc, argv, "read_write");

  ros::NodeHandle nh(ros::this_node::getName());

  init_pose_pub = nh.advertise<std_msgs::String>("/robotis/base/ini_pose", 0);
  sync_write_pub = nh.advertise<robotis_controller_msgs::SyncWriteItem>("/robotis/sync_write_item", 0);
  dxl_torque_pub = nh.advertise<std_msgs::String>("/robotis/dxl_torque", 0);
  write_joint_pub = nh.advertise<sensor_msgs::JointState>("/robotis/set_joint_states", 0);
  write_joint_pub2 = nh.advertise<sensor_msgs::JointState>("/robotis/direct_control/set_joint_states", 0);

  read_joint_sub = nh.subscribe("/robotis/present_joint_states", 1, jointstatesCallback);
  buttuon_sub = nh.subscribe("/robotis/open_cr/button", 1, buttonHandlerCallback);

  // service
  set_joint_module_client = nh.serviceClient<robotis_controller_msgs::SetModule>("/robotis/set_present_ctrl_modules");

  ros::start();

  //set node loop rate
  ros::Rate loop_rate(SPIN_RATE);

  // wait for starting of op3_manager
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

  readyToDemo();

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
  // starting demo using robotis_controller
  if (msg->data == "mode")
  {
    control_module = Framework;
    ROS_INFO("Button : mode | Framework");
    readyToDemo();
  }
  // starting demo using direct_control_module
  else if (msg->data == "start")
  {
    control_module = DirectControlModule;
    ROS_INFO("Button : start | Direct control module");
    readyToDemo();
  }
  // torque on all joints of ROBOTIS-OP3
  else if (msg->data == "user")
  {
    torqueOnAll();
    control_module = None;
  }
}

void jointstatesCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
  if(control_module == None)
    return;

  sensor_msgs::JointState write_msg;
  write_msg.header = msg->header;

  for(int ix = 0; ix < msg->name.size(); ix++)
  {
    std::string joint_name = msg->name[ix];
    double joint_position = msg->position[ix];

    // mirror and copy joint angles from right to left
    if(joint_name == "r_sho_pitch")
    {
      write_msg.name.push_back("r_sho_pitch");
      write_msg.position.push_back(joint_position);
      write_msg.name.push_back("l_sho_pitch");
      write_msg.position.push_back(-joint_position);
    }
    if(joint_name == "r_sho_roll")
    {
      write_msg.name.push_back("r_sho_roll");
      write_msg.position.push_back(joint_position);
      write_msg.name.push_back("l_sho_roll");
      write_msg.position.push_back(-joint_position);
    }
    if(joint_name == "r_el")
    {
      write_msg.name.push_back("r_el");
      write_msg.position.push_back(joint_position);
      write_msg.name.push_back("l_el");
      write_msg.position.push_back(-joint_position);
    }
  }

  // publish a message to set the joint angles
  if(control_module == Framework)
    write_joint_pub.publish(write_msg);
  else if(control_module == DirectControlModule)
    write_joint_pub2.publish(write_msg);
}

void readyToDemo()
{
  ROS_INFO("Start Read-Write Demo");
  // turn off LED
  setLED(0x04);

  torqueOnAll();
  ROS_INFO("Torque on All joints");

  // send message for going init posture
  goInitPose();
  ROS_INFO("Go Init pose");

  // wait while ROBOTIS-OP3 goes to the init posture.
  ros::Duration(4.0).sleep();

  // turn on R/G/B LED [0x01 | 0x02 | 0x04]
  setLED(control_module);

  // change the module for demo
  if(control_module == Framework)
  {
    setModule("none");
    ROS_INFO("Change module to none");
  }
  else if(control_module == DirectControlModule)
  {
    setModule("direct_control_module");
    ROS_INFO("Change module to direct_control_module");
  }
  else
    return;

  // torque off : right arm
  torqueOff("right");
  ROS_INFO("Torque off");
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
