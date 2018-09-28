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
#include <ros/package.h>
#include <std_msgs/String.h>

#include "op3_demo/soccer_demo.h"
#include "op3_demo/action_demo.h"
#include "op3_demo/vision_demo.h"
#include "op3_demo/button_test.h"
#include "op3_demo/mic_test.h"
#include "robotis_math/robotis_linear_algebra.h"
#include "robotis_controller_msgs/SyncWriteItem.h"

enum Demo_Status
{
  Ready = 0,
  ButtonTest = 1,
  MicTest = 2,
  SoccerDemo = 3,
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

void demoModeCommandCallback(const std_msgs::String::ConstPtr &msg);

const int SPIN_RATE = 30;
const bool DEBUG_PRINT = true;

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
  ros::init(argc, argv, "self_test_node");

  //create ros wrapper object
  robotis_op::OPDemo *current_demo = NULL;
  robotis_op::SoccerDemo *soccer_demo = new robotis_op::SoccerDemo();
  robotis_op::ActionDemo *action_demo = new robotis_op::ActionDemo();
  robotis_op::VisionDemo *vision_demo = new robotis_op::VisionDemo();
  robotis_op::ButtonTest *button_test = new robotis_op::ButtonTest();
  robotis_op::MicTest *mic_test = new robotis_op::MicTest();

  ros::NodeHandle nh(ros::this_node::getName());

  init_pose_pub = nh.advertise<std_msgs::String>("/robotis/base/ini_pose", 0);
  play_sound_pub = nh.advertise<std_msgs::String>("/play_sound_file", 0);
  led_pub = nh.advertise<robotis_controller_msgs::SyncWriteItem>("/robotis/sync_write_item", 0);
  dxl_torque_pub = nh.advertise<std_msgs::String>("/robotis/dxl_torque", 0);
  ros::Subscriber buttuon_sub = nh.subscribe("/robotis/open_cr/button", 1, buttonHandlerCallback);
  ros::Subscriber mode_command_sub = nh.subscribe("/robotis/mode_command", 1, demoModeCommandCallback);

  default_mp3_path = ros::package::getPath("op3_demo") + "/data/mp3/";

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
  playSound(default_mp3_path + "test/Self test ready mode.mp3");
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
        case ButtonTest:
        {
          if (current_demo != NULL)
            current_demo->setDemoDisable();

          current_demo = button_test;
          current_demo->setDemoEnable();
          ROS_INFO_COND(DEBUG_PRINT, "[Start] Button Test");
          break;
        }
        case MicTest:
        {
          if (current_demo != NULL)
            current_demo->setDemoDisable();

          current_demo = mic_test;
          current_demo->setDemoEnable();
          ROS_INFO_COND(DEBUG_PRINT, "[Start] Mic Test");
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

    // for debug
    if (checkManagerRunning(manager_name) == false)
      return 0;
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

      playSound(default_mp3_path + "test/Self test ready mode.mp3");
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

        case VisionDemo:
          dxlTorqueChecker();
          playSound(default_mp3_path + "Start vision processing demonstration.mp3");
          break;

        case ActionDemo:
          dxlTorqueChecker();
          playSound(default_mp3_path + "Start motion demonstration.mp3");
          break;

        case ButtonTest:
          dxlTorqueChecker();
          playSound(default_mp3_path + "test/Start button test mode.mp3");
          break;

        case MicTest:
          dxlTorqueChecker();
          playSound(default_mp3_path + "test/Start mic test mode.mp3");
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

      // sound out desired status and changign LED
      switch (desired_status)
      {
        case SoccerDemo:
          playSound(default_mp3_path + "Autonomous soccer mode.mp3");
          setLED(0x01);
          break;

        case VisionDemo:
          playSound(default_mp3_path + "Vision processing mode.mp3");
          setLED(0x02);
          break;

        case ActionDemo:
          playSound(default_mp3_path + "Interactive motion mode.mp3");
          setLED(0x04);
          break;

        case ButtonTest:
          playSound(default_mp3_path + "test/Button test mode.mp3");
          setLED(0x01 | 0x02);
          break;

        case MicTest:
          playSound(default_mp3_path + "test/Mic test mode.mp3");
          setLED(0x01 | 0x04);
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

void demoModeCommandCallback(const std_msgs::String::ConstPtr &msg)
{
  // In demo mode
  if (current_status != Ready)
  {
    if (msg->data == "ready")
    {
      // go to mode selection status
      desired_status = Ready;
      apply_desired = true;

      playSound(default_mp3_path + "Demonstration ready mode.mp3");
      setLED(0x01 | 0x02 | 0x04);
    }
  }
  // In ready mode
  else
  {
    if(msg->data == "soccer")
    {
      desired_status = SoccerDemo;
      apply_desired = true;

      // play sound
      dxlTorqueChecker();
      playSound(default_mp3_path + "Start soccer demonstration.mp3");
      ROS_INFO_COND(DEBUG_PRINT, "= Start Demo Mode : %d", desired_status);
    }
    else if(msg->data == "vision")
    {
      desired_status = VisionDemo;
      apply_desired = true;

      // play sound
      dxlTorqueChecker();
      playSound(default_mp3_path + "Start vision processing demonstration.mp3");
      ROS_INFO_COND(DEBUG_PRINT, "= Start Demo Mode : %d", desired_status);
    }
    else if(msg->data == "action")
    {
      desired_status = ActionDemo;
      apply_desired = true;

      // play sound
      dxlTorqueChecker();
      playSound(default_mp3_path + "Start motion demonstration.mp3");
      ROS_INFO_COND(DEBUG_PRINT, "= Start Demo Mode : %d", desired_status);
    }
  }
}

