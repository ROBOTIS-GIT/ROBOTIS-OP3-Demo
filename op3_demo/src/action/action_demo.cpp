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

#include "op3_demo/action_demo.h"

namespace robotis_op
{

ActionDemo::ActionDemo()
    : SPIN_RATE(30),
      DEBUG_PRINT(false),
      play_index_(0),
      play_status_(StopAction)
{
  enable_ = false;

  ros::NodeHandle nh(ros::this_node::getName());

  std::string default_path = ros::package::getPath("op3_demo") + "/list/action_script.yaml";
  script_path_ = nh.param<std::string>("action_script", default_path);

  std::string default_play_list = "default";
  play_list_name_ = nh.param<std::string>("action_script_play_list", default_play_list);

  demo_command_sub_ = nh.subscribe("/robotis/demo_command", 1, &ActionDemo::demoCommandCallback, this);

  parseActionScript (script_path_);

  boost::thread queue_thread = boost::thread(boost::bind(&ActionDemo::callbackThread, this));
  boost::thread process_thread = boost::thread(boost::bind(&ActionDemo::processThread, this));
}

ActionDemo::~ActionDemo()
{
}

void ActionDemo::setDemoEnable()
{
  setModuleToDemo("action_module");

  enable_ = true;

  ROS_INFO_COND(DEBUG_PRINT, "Start ActionScript Demo");

  playAction(InitPose);

  startProcess(play_list_name_);
}

void ActionDemo::setDemoDisable()
{
  stopProcess();

  enable_ = false;
  ROS_WARN("Set Action demo disable");
  play_list_.resize(0);
}

void ActionDemo::process()
{
  switch (play_status_)
  {
    case PlayAction:
    {
      if (play_list_.size() == 0)
      {
        ROS_WARN("Play List is empty.");
        return;
      }

      // action is not running
      if (isActionRunning() == false)
      {
        // play
        bool result_play = playActionWithSound(play_list_.at(play_index_));

        ROS_INFO_COND(!result_play, "Fail to play action script.");

        // add play index
        int index_to_play = (play_index_ + 1) % play_list_.size();
        play_index_ = index_to_play;
      }
      else
      {
        // wait
        return;
      }
      break;
    }

    case PauseAction:
    {
      stopMP3();
      brakeAction();

      play_status_ = ReadyAction;

      break;
    }

    case StopAction:
    {
      stopMP3();
      stopAction();

      play_status_ = ReadyAction;

      break;
    }

    default:
      break;
  }
}

void ActionDemo::startProcess(const std::string &set_name)
{
  parseActionScriptSetName(script_path_, set_name);

  play_status_ = PlayAction;
}

void ActionDemo::resumeProcess()
{
  play_status_ = PlayAction;
}

void ActionDemo::pauseProcess()
{
  play_status_ = PauseAction;
}

void ActionDemo::stopProcess()
{
  play_index_ = 0;
  play_status_ = StopAction;
}

void ActionDemo::processThread()
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

void ActionDemo::callbackThread()
{
  ros::NodeHandle nh(ros::this_node::getName());

  // subscriber & publisher
  module_control_pub_ = nh.advertise<std_msgs::String>("/robotis/enable_ctrl_module", 0);
  motion_index_pub_ = nh.advertise<std_msgs::Int32>("/robotis/action/page_num", 0);
  play_sound_pub_ = nh.advertise<std_msgs::String>("/play_sound_file", 0);

  buttuon_sub_ = nh.subscribe("/robotis/open_cr/button", 1, &ActionDemo::buttonHandlerCallback, this);

  is_running_client_ = nh.serviceClient<op3_action_module_msgs::IsRunning>("/robotis/action/is_running");
  set_joint_module_client_ = nh.serviceClient<robotis_controller_msgs::SetModule>("/robotis/set_present_ctrl_modules");

  while (nh.ok())
  {
    ros::spinOnce();

    usleep(1000);
  }
}

void ActionDemo::parseActionScript(const std::string &path)
{
  YAML::Node doc;

  try
  {
    // load yaml
    doc = YAML::LoadFile(path.c_str());
  } catch (const std::exception& e)
  {
    ROS_ERROR_STREAM("Fail to load action script yaml. - " << e.what());
    ROS_ERROR_STREAM("Script Path : " << path);
    return;
  }

  // parse action_sound table
  YAML::Node sub_node = doc["action_and_sound"];
  for (YAML::iterator yaml_it = sub_node.begin(); yaml_it != sub_node.end(); ++yaml_it)
  {
    int action_index = yaml_it->first.as<int>();
    std::string mp3_path = yaml_it->second.as<std::string>();

    action_sound_table_[action_index] = mp3_path;
  }

  // default action set
  if (doc["default"])
    play_list_ = doc["default"].as<std::vector<int> >();
}

bool ActionDemo::parseActionScriptSetName(const std::string &path, const std::string &set_name)
{

  YAML::Node doc;

  try
  {
    // load yaml
    doc = YAML::LoadFile(path.c_str());
  } catch (const std::exception& e)
  {
    ROS_ERROR("Fail to load yaml.");
    return false;
  }

  // parse action_sound table
  if (doc[set_name])
  {
    play_list_ = doc[set_name].as<std::vector<int> >();
    return true;
  }
  else
    return false;
}

bool ActionDemo::playActionWithSound(int motion_index)
{
  std::map<int, std::string>::iterator map_it = action_sound_table_.find(motion_index);
  if (map_it == action_sound_table_.end())
    return false;

  playAction(motion_index);
  playMP3(map_it->second);

  ROS_INFO_STREAM_COND(DEBUG_PRINT, "action : " << motion_index << ", mp3 path : " << map_it->second);

  return true;
}

void ActionDemo::playMP3(std::string &path)
{
  std_msgs::String sound_msg;
  sound_msg.data = path;

  play_sound_pub_.publish(sound_msg);
}

void ActionDemo::stopMP3()
{
  std_msgs::String sound_msg;
  sound_msg.data = "";

  play_sound_pub_.publish(sound_msg);
}

void ActionDemo::playAction(int motion_index)
{
  std_msgs::Int32 motion_msg;
  motion_msg.data = motion_index;

  motion_index_pub_.publish(motion_msg);
}

void ActionDemo::stopAction()
{
  std_msgs::Int32 motion_msg;
  motion_msg.data = StopActionCommand;

  motion_index_pub_.publish(motion_msg);
}

void ActionDemo::brakeAction()
{
  std_msgs::Int32 motion_msg;
  motion_msg.data = BrakeActionCommand;

  motion_index_pub_.publish(motion_msg);
}

// check running of action
bool ActionDemo::isActionRunning()
{
  op3_action_module_msgs::IsRunning is_running_srv;

  if (is_running_client_.call(is_running_srv) == false)
  {
    ROS_ERROR("Failed to get action status");
    return true;
  }
  else
  {
    if (is_running_srv.response.is_running == true)
    {
      return true;
    }
  }

  return false;
}

void ActionDemo::buttonHandlerCallback(const std_msgs::String::ConstPtr& msg)
{
  if (enable_ == false)
    return;

  if (msg->data == "start")
  {
    switch (play_status_)
      {
        case PlayAction:
        {
          pauseProcess();
          break;
        }

        case PauseAction:
        {
          resumeProcess();
          break;
        }

        case StopAction:
        {
          resumeProcess();
          break;
        }

        default:
          break;
      }
  }
  else if (msg->data == "mode")
  {

  }
}

void ActionDemo::setModuleToDemo(const std::string &module_name)
{
  callServiceSettingModule(module_name);
  ROS_INFO_STREAM("enable module : " << module_name);
}

void ActionDemo::callServiceSettingModule(const std::string &module_name)
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

void ActionDemo::demoCommandCallback(const std_msgs::String::ConstPtr &msg)
{
  if (enable_ == false)
    return;

  if (msg->data == "start")
  {
    resumeProcess();
  }
  else if (msg->data == "stop")
  {
    pauseProcess();
  }
}

} /* namespace robotis_op */
