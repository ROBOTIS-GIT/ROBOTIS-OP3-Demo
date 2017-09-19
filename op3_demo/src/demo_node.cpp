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

#include <ros/ros.h>
#include <std_msgs/String.h>

#include "op3_demo/soccer_demo.h"
#include "op3_demo/keeper_demo.h"
#include "op3_demo/pk_demo.h"
#include "op3_demo/action_demo.h"
#include "op3_demo/vision_demo.h"
#include "robotis_math/robotis_linear_algebra.h"
#include "robotis_controller_msgs/SyncWriteItem.h"

enum Demo_Status
{
    Ready = 0,
    SoccerDemo = 1,
    KeeperDemo = 2,
    PKDemo = 3,
    VisionDemo = 4,
    ActionDemo = 5,
    DemoCount = 6,
};

void buttonHandlerCallback(const std_msgs::String::ConstPtr& msg);
void goInitPose();
void playSound(const std::string &path);
void setLED(int led);
bool checkManagerRunning(std::string& manager_name);
void dxlTorqueChecker();

const int SPIN_RATE = 30;
const bool DEBUG_PRINT = false;

ros::Publisher init_pose_pub;
ros::Publisher play_sound_pub;
ros::Publisher led_pub;
ros::Publisher dxl_torque_pub;

std::string default_mp3_path = "";
int current_status = Ready;
int desired_status = Ready;
bool apply_desired = false;

//node main
int main(int argc, char **argv)
{
  //init ros
  ros::init(argc, argv, "demo_node");

  //create ros wrapper object
  robotis_op::OPDemo *current_demo = NULL;
  robotis_op::SoccerDemo *soccer_demo = new robotis_op::SoccerDemo();
  robotis_op::KeeperDemo *keeper_demo = new robotis_op::KeeperDemo();
  robotis_op::PKDemo *pk_demo = new robotis_op::PKDemo();
  robotis_op::ActionDemo *action_demo = new robotis_op::ActionDemo();
  robotis_op::VisionDemo *vision_demo = new robotis_op::VisionDemo();

  ros::NodeHandle nh(ros::this_node::getName());

  init_pose_pub = nh.advertise<std_msgs::String>("/robotis/base/ini_pose", 0);
  play_sound_pub = nh.advertise<std_msgs::String>("/play_sound_file", 0);
  led_pub = nh.advertise<robotis_controller_msgs::SyncWriteItem>("/robotis/sync_write_item", 0);
  dxl_torque_pub = nh.advertise<std_msgs::String>("/robotis/dxl_torque", 0);
  ros::Subscriber buttuon_sub = nh.subscribe("/robotis/open_cr/button", 1, buttonHandlerCallback);

  default_mp3_path = ros::package::getPath("op3_demo") + "/Data/mp3/";

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
  playSound(default_mp3_path + "Demonstration ready mode.mp3");
  // turn on R/G/B LED
  setLED(0x01 | 0x02 | 0x04);

  //node loop
  while (ros::ok())
  {
    // process
    if (apply_desired == true)
    {
      switch (desired_status)
      {
        case Ready:
        {

          if (current_demo != NULL)
            current_demo->setDemoDisable();

          current_demo = NULL;

          goInitPose();

          ROS_INFO_COND(DEBUG_PRINT, "[Go to Demo READY!]");
          break;
        }

        case SoccerDemo:
        {
          if (current_demo != NULL)
            current_demo->setDemoDisable();

          current_demo = soccer_demo;
          current_demo->setDemoEnable();

          ROS_INFO_COND(DEBUG_PRINT, "[Start] Soccer Demo");
          break;
        }

        case KeeperDemo:
        {
          if (current_demo != NULL)
            current_demo->setDemoDisable();

          current_demo = keeper_demo;
          current_demo->setDemoEnable();

          ROS_INFO_COND(DEBUG_PRINT, "[Start] Keeper Demo");
          break;
        }

        case PKDemo:
        {
          if (current_demo != NULL)
            current_demo->setDemoDisable();

          current_demo = pk_demo;
          current_demo->setDemoEnable();

          ROS_INFO_COND(DEBUG_PRINT, "[Start] PK Demo");
          break;
        }


        case VisionDemo:
        {
          if (current_demo != NULL)
            current_demo->setDemoDisable();

          current_demo = vision_demo;
          current_demo->setDemoEnable();
          ROS_INFO_COND(DEBUG_PRINT, "[Start] Vision Demo");
          break;
        }
        case ActionDemo:
        {
          if (current_demo != NULL)
            current_demo->setDemoDisable();

          current_demo = action_demo;
          current_demo->setDemoEnable();
          ROS_INFO_COND(DEBUG_PRINT, "[Start] Action Demo");
          break;
        }

        default:
        {
          break;
        }
      }

      apply_desired = false;
      current_status = desired_status;
    }

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
  if(apply_desired == true)
    return;

  // in the middle of playing demo
  if (current_status != Ready)
  {
    if (msg->data == "mode_long")
    {
      // go to mode selection status
      desired_status = Ready;
      apply_desired = true;

      playSound(default_mp3_path + "Demonstration ready mode.mp3");
      setLED(0x01 | 0x02 | 0x04);
    }
    else if (msg->data == "user_long")
    {
      // it's using in op3_manager
      // torque on and going to init pose
    }
  }
  // ready to start demo
  else
  {
    if (msg->data == "start")
    {
      // select current demo
      desired_status = (desired_status == Ready) ? desired_status + 1 : desired_status;
      apply_desired = true;

      // sound out desired status
      switch (desired_status)
      {
        case SoccerDemo:
          dxlTorqueChecker();
          playSound(default_mp3_path + "Start soccer demonstration.mp3");
          break;

        case KeeperDemo:
          dxlTorqueChecker();
          playSound(default_mp3_path + "Start soccer demonstration.mp3");
          break;

        case PKDemo:
          dxlTorqueChecker();
          playSound(default_mp3_path + "Start soccer demonstration.mp3");
          break;

        case VisionDemo:
          dxlTorqueChecker();
          playSound(default_mp3_path + "Start vision processing demonstration.mp3");
          break;

        case ActionDemo:
          dxlTorqueChecker();
          playSound(default_mp3_path + "Start motion demonstration.mp3");
          break;

        default:
          break;
      }

      ROS_INFO_COND(DEBUG_PRINT, "= Start Demo Mode : %d", desired_status);
    }
    else if (msg->data == "mode")
    {
      // change to next demo
      desired_status = (desired_status + 1) % DemoCount;
      desired_status = (desired_status == Ready) ? desired_status + 1 : desired_status;

      // sound out desired status and changing LED
      switch (desired_status)
      {
        case SoccerDemo:
          playSound(default_mp3_path + "Autonomous soccer mode.mp3");
          setLED(0x01);
          break;

        case KeeperDemo:
          playSound(default_mp3_path + "Autonomous soccer mode.mp3");
          setLED(0x02);
          break;

        case PKDemo:
          playSound(default_mp3_path + "Autonomous soccer mode.mp3");
          setLED(0x04);
          break;

        case VisionDemo:
          playSound(default_mp3_path + "Vision processing mode.mp3");
          setLED(0x10);
          break;

        case ActionDemo:
          playSound(default_mp3_path + "Interactive motion mode.mp3");
          setLED(0x11);
          break;

        default:
          break;
      }

      ROS_INFO_COND(DEBUG_PRINT, "= Demo Mode : %d", desired_status);
    }
  }
}

void goInitPose()
{
  std_msgs::String init_msg;
  init_msg.data = "ini_pose";

  init_pose_pub.publish(init_msg);
}

void playSound(const std::string &path)
{
  std_msgs::String sound_msg;
  sound_msg.data = path;

  play_sound_pub.publish(sound_msg);
}

void setLED(int led)
{
  robotis_controller_msgs::SyncWriteItem syncwrite_msg;
  syncwrite_msg.item_name = "LED";
  syncwrite_msg.joint_name.push_back("open-cr");
  syncwrite_msg.value.push_back(led);

  led_pub.publish(syncwrite_msg);
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

void dxlTorqueChecker()
{
  std_msgs::String check_msg;
  check_msg.data = "check";

  dxl_torque_pub.publish(check_msg);
}
