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

#include <ament_index_cpp/get_package_share_directory.hpp>
#include "op3_demo/action_demo.h"

namespace robotis_op
{

ActionDemo::ActionDemo()
  : // Node("action_demo"),
    SPIN_RATE(30),
    DEBUG_PRINT(false),
    play_index_(0),
    play_status_(ActionStatus::StopAction)
{
  enable_ = false;
  voice_control_mode_ = false;  // Initialize voice control mode
  stop_action_executed_ = false;  // Initialize stop action flag

  std::string default_path = ament_index_cpp::get_package_share_directory("op3_demo") + "/list/action_script.yaml";
  // this->declare_parameter<std::string>("action_script", default_path);
  // this->get_parameter<std::string>("action_script", script_path_);
  script_path_ = default_path;

  std::string default_play_list = "default";
  // this->declare_parameter<std::string>("action_script_play_list", default_play_list);
  // this->get_parameter<std::string>("action_script_play_list", play_list_name_);
  play_list_name_ = default_play_list;

  // demo_command_sub_ = this->create_subscription<std_msgs::msg::String>("/robotis/demo_command", 1, std::bind(&ActionDemo::demoCommandCallback, this, std::placeholders::_1));

  parseActionScript(script_path_);

  // std::thread queue_thread(&ActionDemo::callbackThread, this);
  // std::thread process_thread(&ActionDemo::processThread, this);
  // queue_thread.detach();
  // process_thread.detach();
}

ActionDemo::~ActionDemo()
{
}

void ActionDemo::setNode(rclcpp::Node::SharedPtr node)
{
  node_ = node;
}

void ActionDemo::setDemoEnable()
{
  setModuleToDemo("action_module");

  enable_ = true;

  if (DEBUG_PRINT)
    RCLCPP_INFO(rclcpp::get_logger("ActionDemo"), "Start ActionScript Demo");

  // Only start process if not in voice control mode
  if (!voice_control_mode_)
  {
    playAction(InitPose);
    startProcess(play_list_name_);
    RCLCPP_INFO(rclcpp::get_logger("ActionDemo"), "Started default action process");
  }
  else
  {
    playAction(InitPose);
    RCLCPP_INFO(rclcpp::get_logger("ActionDemo"), "Voice control mode - waiting for voice commands");
  }
}

void ActionDemo::setDemoDisable()
{
  stopProcess();

  enable_ = false;
  voice_control_mode_ = false;  // Reset voice control mode
  RCLCPP_WARN(rclcpp::get_logger("ActionDemo"), "Set Action demo disable");
  play_list_.resize(0);
}

void ActionDemo::process()
{
  if (enable_ == false || node_ == nullptr)
    return;

  // Handle StopAction state even in voice control mode to prevent continuous stopAction calls
  if (play_status_ == StopAction)
  {
    if (!stop_action_executed_)
    {
      stopMP3();
      stopAction();
      stop_action_executed_ = true;  // Mark as executed
      RCLCPP_DEBUG(rclcpp::get_logger("ActionDemo"), "StopAction executed once");
    }
    
    play_status_ = ReadyAction;
    return;
  }

  // Skip default action sequence processing in voice control mode
  if (voice_control_mode_)
  {
    RCLCPP_DEBUG(rclcpp::get_logger("ActionDemo"), "Voice control mode active - skipping default action sequence");
    return;
  }

  switch (play_status_)
  {
    case PlayAction:
    {
      if (play_list_.size() == 0)
      {
        RCLCPP_WARN(rclcpp::get_logger("ActionDemo"), "Play List is empty.");
        return;
      }

      // action is not running
      if (isActionRunning() == false)
      {
        // play
        bool result_play = playActionWithSound(play_list_.at(play_index_));

        if (result_play == false)
          RCLCPP_INFO(rclcpp::get_logger("ActionDemo"), "Fail to play action script.");

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

    default:
      break;
  }
}

void ActionDemo::startProcess(const std::string &set_name)
{
  parseActionScriptSetName(script_path_, set_name);

  play_status_ = PlayAction;
  stop_action_executed_ = false;  // Reset flag when starting a new process
}

void ActionDemo::resumeProcess()
{
  play_status_ = PlayAction;
  stop_action_executed_ = false;  // Reset flag when resuming
}

void ActionDemo::pauseProcess()
{
  play_status_ = PauseAction;
}

void ActionDemo::stopProcess()
{
  play_index_ = 0;
  play_status_ = ActionStatus::StopAction;
  stop_action_executed_ = false;  // Reset flag when entering StopAction state
}

// void ActionDemo::processThread()
// {
//   //set node loop rate
//   rclcpp::Rate loop_rate(SPIN_RATE);

//   //node loop
//   while (rclcpp::ok())
//   {
//     if (enable_ == true)
//       process();

//     //relax to fit output rate
//     loop_rate.sleep();
//   }
// }

// void ActionDemo::callbackThread()
// {
//   // subscriber & publisher
//   module_control_pub_ = this->create_publisher<std_msgs::msg::String>("/robotis/enable_ctrl_module", 10);
//   motion_index_pub_ = this->create_publisher<std_msgs::msg::Int32>("/robotis/action/page_num", 10);
//   play_sound_pub_ = this->create_publisher<std_msgs::msg::String>("/play_sound_file", 10);

//   button_sub_ = this->create_subscription<std_msgs::msg::String>("/robotis/open_cr/button", 1, std::bind(&ActionDemo::buttonHandlerCallback, this, std::placeholders::_1));

//   is_running_client_ = this->create_client<op3_action_module_msgs::srv::IsRunning>("/robotis/action/is_running");
//   set_joint_module_client_ = this->create_client<robotis_controller_msgs::srv::SetModule>("/robotis/set_present_ctrl_modules");

//   while (rclcpp::ok())
//   {
//     rclcpp::spin_some(this->get_node_base_interface());

//     std::this_thread::sleep_for(std::chrono::milliseconds(1));
//   }
// }

void ActionDemo::parseActionScript(const std::string &path)
{
  YAML::Node doc;

  try
  {
    // load yaml
    doc = YAML::LoadFile(path.c_str());
  } catch (const std::exception& e)
  {
    RCLCPP_ERROR(rclcpp::get_logger("ActionDemo"), "Fail to load action script yaml. - %s", e.what());
    RCLCPP_ERROR(rclcpp::get_logger("ActionDemo"), "Script Path : %s", path.c_str());
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
    RCLCPP_ERROR(rclcpp::get_logger("ActionDemo"), "Fail to load yaml.");
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

  RCLCPP_INFO(rclcpp::get_logger("ActionDemo"), "action : %d, mp3 path : %s", motion_index, map_it->second.c_str());

  return true;
}

void ActionDemo::playMP3(std::string &path)
{
  if (node_ == nullptr)
  {
    RCLCPP_ERROR(rclcpp::get_logger("ActionDemo"), "Node is not set, cannot play mp3");
    return;
  }
  auto play_sound_pub_ = node_->create_publisher<std_msgs::msg::String>("/play_sound_file", 10);
  std_msgs::msg::String sound_msg;
  sound_msg.data = path;

  play_sound_pub_->publish(sound_msg);
}

void ActionDemo::stopMP3()
{
  if (node_ == nullptr)
  {
    RCLCPP_ERROR(rclcpp::get_logger("ActionDemo"), "Node is not set, cannot stop mp3");
    return;
  }
  auto play_sound_pub_ = node_->create_publisher<std_msgs::msg::String>("/play_sound_file", 10);
  std_msgs::msg::String sound_msg;
  sound_msg.data = "";

  play_sound_pub_->publish(sound_msg);
}

void ActionDemo::playAction(int motion_index)
{
  if (node_ == nullptr)
  {
    RCLCPP_ERROR(rclcpp::get_logger("ActionDemo"), "Node is not set, cannot play motion");
    return;
  }
  
  // Reset stop action flag when starting a new action
  stop_action_executed_ = false;
  
  auto motion_index_pub_ = node_->create_publisher<std_msgs::msg::Int32>("/robotis/action/page_num", 10);
  std_msgs::msg::Int32 motion_msg;
  motion_msg.data = motion_index;

  motion_index_pub_->publish(motion_msg);
}

void ActionDemo::stopAction()
{
  if (node_ == nullptr)
  {
    RCLCPP_ERROR(rclcpp::get_logger("ActionDemo"), "Node is not set, cannot stop motion");
    return;
  }
  auto motion_index_pub_ = node_->create_publisher<std_msgs::msg::Int32>("/robotis/action/page_num", 10);
  std_msgs::msg::Int32 motion_msg;
  motion_msg.data = StopActionCommand;

  motion_index_pub_->publish(motion_msg);
}

void ActionDemo::brakeAction()
{
  if (node_ == nullptr)
  {
    RCLCPP_ERROR(rclcpp::get_logger("ActionDemo"), "Node is not set, cannot brake motion");
    return;
  }
  auto motion_index_pub_ = node_->create_publisher<std_msgs::msg::Int32>("/robotis/action/page_num", 10);
  std_msgs::msg::Int32 motion_msg;
  motion_msg.data = BrakeActionCommand;

  motion_index_pub_->publish(motion_msg);
}

// check running of action
bool ActionDemo::isActionRunning()
{
  auto temp_node = rclcpp::Node::make_shared("action_is_running");

  auto temp_is_running_client_ = temp_node->create_client<op3_action_module_msgs::srv::IsRunning>("/robotis/action/is_running");
  auto request = std::make_shared<op3_action_module_msgs::srv::IsRunning::Request>();
  bool request_result = true;

  if (!temp_is_running_client_->wait_for_service(std::chrono::seconds(1)))
  {
    RCLCPP_ERROR(rclcpp::get_logger("ActionDemo"), "Failed to get action status: Service not available");
    return request_result;
  }

  auto future = temp_is_running_client_->async_send_request(request);
  if (rclcpp::spin_until_future_complete(temp_node, future) == rclcpp::FutureReturnCode::SUCCESS)
  {
    auto result = future.get();
    RCLCPP_INFO(rclcpp::get_logger("ActionDemo"), "ActionDemo::isActionRunning - is running : %d", result->is_running);
    return result->is_running;
  }
  else
  {
    RCLCPP_ERROR(rclcpp::get_logger("ActionDemo"), "Failed to get action status: Service call failed");
    return request_result;
  }

  return request_result;
}

void ActionDemo::buttonHandlerCallback(const std_msgs::msg::String::SharedPtr msg)
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
  RCLCPP_INFO(rclcpp::get_logger("ActionDemo"), "ActionDemo::setModuleToDemo - enable module : %s", module_name.c_str());
}

void ActionDemo::callServiceSettingModule(const std::string &module_name)
{
  auto temp_node = rclcpp::Node::make_shared("action_call_service");
  auto temp_set_joint_module_client_ = temp_node->create_client<robotis_controller_msgs::srv::SetModule>("/robotis/set_present_ctrl_modules");
  auto set_module_srv = std::make_shared<robotis_controller_msgs::srv::SetModule::Request>();
  set_module_srv->module_name = module_name;

  if (!temp_set_joint_module_client_->wait_for_service(std::chrono::seconds(1)))
  {
    RCLCPP_ERROR(rclcpp::get_logger("ActionDemo"), "Failed to set module");
    return;
  }

  auto future = temp_set_joint_module_client_->async_send_request(set_module_srv);
  if (rclcpp::spin_until_future_complete(temp_node, future) == rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_INFO(rclcpp::get_logger("ActionDemo"), "ActionDemo::callServiceSettingModule(%s) : result : %d", module_name.c_str(), future.get()->result);
  }
}

void ActionDemo::demoCommandCallback(const std_msgs::msg::String::SharedPtr msg)
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

void ActionDemo::setVoiceControlMode(bool voice_mode)
{
  voice_control_mode_ = voice_mode;
  RCLCPP_INFO(rclcpp::get_logger("ActionDemo"), "Voice control mode set to: %s", 
              voice_mode ? "true" : "false");
}

} /* namespace robotis_op */
