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

#include "op3_demo/soccer_demo.h"

namespace robotis_op
{

SoccerDemo::SoccerDemo()
  : FALL_FORWARD_LIMIT(60),
    FALL_BACK_LIMIT(-60),
    SPIN_RATE(30),
    DEBUG_PRINT(false),
    wait_count_(0),
    on_following_ball_(false),
    on_tracking_ball_(false),
    restart_soccer_(false),
    start_following_(false),
    stop_following_(false),
    stop_fallen_check_(false),
    robot_status_(Waited),
    stand_state_(Stand),
    tracking_status_(BallTracker::Waiting),
    present_pitch_(0)
{
  //init ros
  enable_ = false;

  ros::NodeHandle nh(ros::this_node::getName());

  std::string default_path = ros::package::getPath("op3_gui_demo") + "/config/gui_config.yaml";
  std::string path = nh.param<std::string>("demo_config", default_path);
  parseJointNameFromYaml(path);

  boost::thread queue_thread = boost::thread(boost::bind(&SoccerDemo::callbackThread, this));
  boost::thread process_thread = boost::thread(boost::bind(&SoccerDemo::processThread, this));
  boost::thread tracking_thread = boost::thread(boost::bind(&SoccerDemo::trackingThread, this));

  is_grass_ = nh.param<bool>("grass_demo", false);
}

SoccerDemo::~SoccerDemo()
{

}

void SoccerDemo::setDemoEnable()
{
  enable_ = true;

  startSoccerMode();
}

void SoccerDemo::setDemoDisable()
{
  // handle disable procedure
  ball_tracker_.stopTracking();
  ball_follower_.stopFollowing();

  enable_ = false;
  wait_count_ = 0;
  on_following_ball_ = false;
  on_tracking_ball_ = false;
  restart_soccer_ = false;
  start_following_ = false;
  stop_following_ = false;
  stop_fallen_check_ = false;

  tracking_status_ = BallTracker::Waiting;
}

void SoccerDemo::process()
{
  if(enable_ == false)
    return;

  // check to start
  if (start_following_ == true)
  {
    ball_tracker_.startTracking();
    ball_follower_.startFollowing();
    start_following_ = false;

    wait_count_ = 1 * SPIN_RATE;
  }

  // check to stop
  if (stop_following_ == true)
  {
    ball_tracker_.stopTracking();
    ball_follower_.stopFollowing();
    stop_following_ = false;

    wait_count_ = 0;
  }

  if (wait_count_ <= 0)
  {
    // ball following
    if (on_following_ball_ == true)
    {
      switch(tracking_status_)
      {
      case BallTracker::Found:
        ball_follower_.processFollowing(ball_tracker_.getPanOfBall(), ball_tracker_.getTiltOfBall(), 0.0);
        break;

      case BallTracker::NotFound:
        ball_follower_.waitFollowing();
        break;

      default:
        break;
      }
    }

    // check fallen states
    switch (stand_state_)
    {
    case Stand:
    {
      // check restart soccer
      if (restart_soccer_ == true)
      {
        restart_soccer_ = false;
        startSoccerMode();
        break;
      }

      // check states for kick
//      int ball_position = ball_follower_.getBallPosition();
      bool in_range = ball_follower_.isBallInRange();

      if(in_range == true)
      {
        ball_follower_.stopFollowing();
        handleKick();
      }
      break;
    }
      // fallen state : Fallen_Forward, Fallen_Behind
    default:
    {
      ball_follower_.stopFollowing();
      handleFallen(stand_state_);
      break;
    }
    }
  }
  else
  {
    wait_count_ -= 1;
  }
}

void SoccerDemo::processThread()
{
  bool result = false;

  //set node loop rate
  ros::Rate loop_rate(SPIN_RATE);

  ball_tracker_.startTracking();

  //node loop
  while (ros::ok())
  {
    if (enable_ == true)
      process();

    //relax to fit output rate
    loop_rate.sleep();
  }
}

void SoccerDemo::callbackThread()
{
  ros::NodeHandle nh(ros::this_node::getName());

  // subscriber & publisher
  module_control_pub_ = nh.advertise<robotis_controller_msgs::JointCtrlModule>("/robotis/set_joint_ctrl_modules", 0);
  motion_index_pub_ = nh.advertise<std_msgs::Int32>("/robotis/action/page_num", 0);
  rgb_led_pub_ = nh.advertise<robotis_controller_msgs::SyncWriteItem>("/robotis/sync_write_item", 0);

  buttuon_sub_ = nh.subscribe("/robotis/open_cr/button", 1, &SoccerDemo::buttonHandlerCallback, this);
  demo_command_sub_ = nh.subscribe("/robotis/demo_command", 1, &SoccerDemo::demoCommandCallback, this);
  imu_data_sub_ = nh.subscribe("/robotis/open_cr/imu", 1, &SoccerDemo::imuDataCallback, this);

  is_running_client_ = nh.serviceClient<op3_action_module_msgs::IsRunning>("/robotis/action/is_running");
  set_joint_module_client_ = nh.serviceClient<robotis_controller_msgs::SetJointModule>("/robotis/set_present_joint_ctrl_modules");

  test_pub_ = nh.advertise<std_msgs::String>("/debug_text", 0);

  while (nh.ok())
  {
    ros::spinOnce();

    usleep(1000);
  }
}

void SoccerDemo::trackingThread()
{

  //set node loop rate
  ros::Rate loop_rate(SPIN_RATE);

  ball_tracker_.startTracking();

  //node loop
  while (ros::ok())
  {

    if(enable_ == true && on_tracking_ball_ == true)
    {
      // ball tracking
      int tracking_status;

      tracking_status = ball_tracker_.processTracking();

      // set led
      switch(tracking_status)
      {
      case BallTracker::Found:
        if(tracking_status_ != tracking_status)
          setRGBLED(0x1F, 0x1F, 0x1F);
        break;

      case BallTracker::NotFound:
        if(tracking_status_ != tracking_status)
          setRGBLED(0, 0, 0);
        break;

      default:
        break;
      }

      if(tracking_status != tracking_status_)
        tracking_status_ = tracking_status;
    }
    //relax to fit output rate
    loop_rate.sleep();
  }
}

void SoccerDemo::setBodyModuleToDemo(const std::string &body_module, bool with_head_control)
{
  robotis_controller_msgs::JointCtrlModule control_msg;

  std::string head_module = "head_control_module";
  std::map<int, std::string>::iterator joint_iter;

  for (joint_iter = id_joint_table_.begin(); joint_iter != id_joint_table_.end(); ++joint_iter)
  {
    // check whether joint name contains "head"
    if (joint_iter->second.find("head") != std::string::npos)
    {
      if (with_head_control == true)
      {
        control_msg.joint_name.push_back(joint_iter->second);
        control_msg.module_name.push_back(head_module);
      }
      else
        continue;
    }
    else
    {
      control_msg.joint_name.push_back(joint_iter->second);
      control_msg.module_name.push_back(body_module);
    }
  }

  // no control
  if (control_msg.joint_name.size() == 0)
    return;

  callServiceSettingModule(control_msg);
  std::cout << "enable module of body : " << body_module << std::endl;
}

void SoccerDemo::setModuleToDemo(const std::string &module_name)
{
  if(enable_ == false)
    return;

  robotis_controller_msgs::JointCtrlModule control_msg;
  std::map<int, std::string>::iterator joint_iter;

  for (joint_iter = id_joint_table_.begin(); joint_iter != id_joint_table_.end(); ++joint_iter)
  {
    control_msg.joint_name.push_back(joint_iter->second);
    control_msg.module_name.push_back(module_name);
  }

  // no control
  if (control_msg.joint_name.size() == 0)
    return;

  callServiceSettingModule(control_msg);
  std::cout << "enable module : " << module_name << std::endl;
}

void SoccerDemo::callServiceSettingModule(const robotis_controller_msgs::JointCtrlModule &modules)
{
  robotis_controller_msgs::SetJointModule set_joint_srv;
  set_joint_srv.request.joint_name = modules.joint_name;
  set_joint_srv.request.module_name = modules.module_name;

  if (set_joint_module_client_.call(set_joint_srv) == false)
  {
    ROS_ERROR("Failed to set module");
    return;
  }

  return ;
}

void SoccerDemo::parseJointNameFromYaml(const std::string &path)
{
  YAML::Node doc;
  try
  {
    // load yaml
    doc = YAML::LoadFile(path.c_str());
  } catch (const std::exception& e)
  {
    ROS_ERROR("Fail to load id_joint table yaml.");
    return;
  }

  // parse id_joint table
  YAML::Node _id_sub_node = doc["id_joint"];
  for (YAML::iterator _it = _id_sub_node.begin(); _it != _id_sub_node.end(); ++_it)
  {
    int _id;
    std::string _joint_name;

    _id = _it->first.as<int>();
    _joint_name = _it->second.as<std::string>();

    id_joint_table_[_id] = _joint_name;
    joint_id_table_[_joint_name] = _id;
  }
}

// joint id -> joint name
bool SoccerDemo::getJointNameFromID(const int &id, std::string &joint_name)
{
  std::map<int, std::string>::iterator _iter;

  _iter = id_joint_table_.find(id);
  if (_iter == id_joint_table_.end())
    return false;

  joint_name = _iter->second;
  return true;
}

// joint name -> joint id
bool SoccerDemo::getIDFromJointName(const std::string &joint_name, int &id)
{
  std::map<std::string, int>::iterator _iter;

  _iter = joint_id_table_.find(joint_name);
  if (_iter == joint_id_table_.end())
    return false;

  id = _iter->second;
  return true;
}

int SoccerDemo::getJointCount()
{
  return joint_id_table_.size();
}

bool SoccerDemo::isHeadJoint(const int &id)
{
  std::map<std::string, int>::iterator _iter;

  for (_iter = joint_id_table_.begin(); _iter != joint_id_table_.end(); ++_iter)
  {
    if (_iter->first.find("head") != std::string::npos)
      return true;
  }

  return false;
}

void SoccerDemo::buttonHandlerCallback(const std_msgs::String::ConstPtr& msg)
{
  if (enable_ == false)
    return;

  if (msg->data == "start")
  {
    if (on_following_ball_ == true)
      stopSoccerMode();
    else
      startSoccerMode();
  }
}

void SoccerDemo::demoCommandCallback(const std_msgs::String::ConstPtr &msg)
{
  if (enable_ == false)
    return;

  if (msg->data == "start")
  {
    if (on_following_ball_ == true)
      stopSoccerMode();
    else
      startSoccerMode();
  }
  else if (msg->data == "stop")
  {
    stopSoccerMode();
  }
}

// check fallen states
void SoccerDemo::imuDataCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
  if (enable_ == false)
    return;

  if (stop_fallen_check_ == true)
    return;

  Eigen::Quaterniond orientation(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);
  Eigen::MatrixXd rpy_orientation = robotis_framework::convertQuaternionToRPY(orientation);
  rpy_orientation *= (180 / M_PI);

  ROS_INFO_COND(DEBUG_PRINT, "Roll : %3.2f, Pitch : %2.2f", rpy_orientation.coeff(0, 0), rpy_orientation.coeff(1, 0));

  double pitch = rpy_orientation.coeff(1, 0);

  double alpha = 0.4;
  if (present_pitch_ == 0)
    present_pitch_ = pitch;
  else
    present_pitch_ = present_pitch_ * (1 - alpha) + pitch * alpha;

  if (present_pitch_ > FALL_FORWARD_LIMIT)
    stand_state_ = Fallen_Forward;
  else if (present_pitch_ < FALL_BACK_LIMIT)
    stand_state_ = Fallen_Behind;
  else
    stand_state_ = Stand;
}

void SoccerDemo::startSoccerMode()
{
  setModuleToDemo("action_module");

  playMotion(WalkingReady);

  setBodyModuleToDemo("walking_module");

  ROS_INFO("Start Soccer Demo");
  on_following_ball_ = true;
  on_tracking_ball_ = true;
  start_following_ = true;
}

void SoccerDemo::stopSoccerMode()
{
  ROS_INFO("Stop Soccer Demo");
  on_following_ball_ = false;
  on_tracking_ball_ = false;
  stop_following_ = true;
}

void SoccerDemo::handleKick(int ball_position)
{
  usleep(1500 * 1000);

  // change to motion module
  setModuleToDemo("action_module");

  if (handleFallen(stand_state_) == true || enable_ == false)
    return;

  // kick motion
  switch (ball_position)
  {
  case robotis_op::BallFollower::OnRight:
    std::cout << "Kick Motion [R]: " << ball_position << std::endl;
    playMotion(is_grass_ ? RightKick + ForGrass : RightKick);
    break;

  case robotis_op::BallFollower::OnLeft:
    std::cout << "Kick Motion [L]: " << ball_position << std::endl;
    playMotion(is_grass_ ? LeftKick + ForGrass : LeftKick);
    break;

  default:
    break;
  }

  on_following_ball_ = false;
  restart_soccer_ = true;
  tracking_status_ = BallTracker::NotFound;
  ball_follower_.clearBallPosition();

  usleep(2000 * 1000);

  if (handleFallen(stand_state_) == true)
    return;

  // ceremony
  //playMotion(Ceremony);
}

void SoccerDemo::handleKick()
{
  usleep(1500 * 1000);

  // change to motion module
  setModuleToDemo("action_module");

  if (handleFallen(stand_state_) == true || enable_ == false)
    return;

  // kick motion
  ball_follower_.decideBallPositin(ball_tracker_.getPanOfBall(), ball_tracker_.getTiltOfBall());
  int ball_position = ball_follower_.getBallPosition();
  if(ball_position == BallFollower::NotFound || ball_position == BallFollower::OutOfRange)
  {
    on_following_ball_ = false;
    restart_soccer_ = true;
    tracking_status_ = BallTracker::NotFound;
    ball_follower_.clearBallPosition();
    return;
  }

  switch (ball_position)
  {
  case robotis_op::BallFollower::OnRight:
    std::cout << "Kick Motion [R]: " << ball_position << std::endl;
    sendDebugTopic("Kick the ball using Right foot");
    playMotion(is_grass_ ? RightKick + ForGrass : RightKick);
    break;

  case robotis_op::BallFollower::OnLeft:
    std::cout << "Kick Motion [L]: " << ball_position << std::endl;
    sendDebugTopic("Kick the ball using Left foot");
    playMotion(is_grass_ ? LeftKick + ForGrass : LeftKick);
    break;

  default:
    break;
  }

  on_following_ball_ = false;
  restart_soccer_ = true;
  tracking_status_ = BallTracker::NotFound;
  ball_follower_.clearBallPosition();

  usleep(2000 * 1000);

  if (handleFallen(stand_state_) == true)
    return;

  // ceremony
  //playMotion(Ceremony);
}

bool SoccerDemo::handleFallen(int fallen_status)
{
  if (fallen_status == Stand)
  {
    return false;
  }

  // change to motion module
  setModuleToDemo("action_module");

  // getup motion
  switch (fallen_status)
  {
  case Fallen_Forward:
    std::cout << "Getup Motion [F]: " << std::endl;
    playMotion(is_grass_ ? GetUpFront + ForGrass : GetUpFront);
    break;

  case Fallen_Behind:
    std::cout << "Getup Motion [B]: " << std::endl;
    playMotion(is_grass_ ? GetUpBack + ForGrass : GetUpBack);
    break;

  default:
    break;
  }

  while(isActionRunning() == true)
    usleep(100 * 1000);

  usleep(650 * 1000);

  if (on_following_ball_ == true)
    restart_soccer_ = true;

  // reset state
  on_following_ball_ = false;

  return true;
}

void SoccerDemo::playMotion(int motion_index)
{
  std_msgs::Int32 motion_msg;
  motion_msg.data = motion_index;

  motion_index_pub_.publish(motion_msg);
}

void SoccerDemo::setRGBLED(int blue, int green, int red)
{
  int led_full_unit = 0x1F;
  int led_value = (blue & led_full_unit) << 10 | (green & led_full_unit) << 5 | (red & led_full_unit);
  robotis_controller_msgs::SyncWriteItem syncwrite_msg;
  syncwrite_msg.item_name = "LED_RGB";
  syncwrite_msg.joint_name.push_back("open-cr");
  syncwrite_msg.value.push_back(led_value);

  rgb_led_pub_.publish(syncwrite_msg);
}

// check running of action
bool SoccerDemo::isActionRunning()
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

void SoccerDemo::sendDebugTopic(const std::string &msgs)
{
  std_msgs::String debug_msg;
  debug_msg.data = msgs;

  test_pub_.publish(debug_msg);
}

}
