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

#include "op3_demo/pk_demo.h"

namespace robotis_op
{

PKDemo::PKDemo()
    : FALL_FORWARD_LIMIT(60),
      FALL_BACK_LIMIT(-60),
      SPIN_RATE(30),
      DEBUG_PRINT(false),
      wait_count_(0),
      on_following_ball_(false),
      on_tracking_ball_(false),
      restart_soccer_(false),
      after_shooting_(false),
      start_following_(false),
      stop_following_(false),
      start_tracking_(false),
      stop_tracking_(false),
      stop_fallen_check_(false),
      robot_status_(Waited),
      stand_state_(Stand),
      tracking_status_(BallTracker::Waiting),
      present_pitch_(0),
      count(0)
{
  //init ros
  enable_ = false;

  ros::NodeHandle nh(ros::this_node::getName());

  std::string default_path = ros::package::getPath("op3_gui_demo") + "/config/demo_config.yaml";
  std::string path = nh.param<std::string>("demo_config", default_path);
  parseJointNameFromYaml(path);

  boost::thread queue_thread = boost::thread(boost::bind(&PKDemo::callbackThread, this));
  boost::thread process_thread = boost::thread(boost::bind(&PKDemo::processThread, this));

  is_grass_ = nh.param<bool>("grass_demo", false);
}

PKDemo::~PKDemo()
{

}

void PKDemo::setDemoEnable()
{
  enable_ = true;

  startSoccerMode();
}

void PKDemo::setDemoDisable()
{
  setModuleToDemo("base_module");

  // handle disable procedure
  ball_tracker_.stopTracking();
  ball_follower_.stopFollowing();

  enable_ = false;
  wait_count_ = 0;
  on_tracking_ball_ = false;
  on_following_ball_ = false;
  after_shooting_ = false;
  restart_soccer_ = false;
  start_tracking_ = false;
  stop_tracking_ = false;
  start_following_ = false;
  stop_following_ = false;
  stop_fallen_check_ = false;
  count = 0;

  tracking_status_ = BallTracker::Waiting;
}

void PKDemo::process()
{
  // ball tracking
  int tracking_status;

  tracking_status = ball_tracker_.processTracking();

  // check to start
  if (start_tracking_ == true)
  {
    ball_tracker_.startTracking();
    start_tracking_ = false;
    on_tracking_ball_ = true;

    wait_count_ = 1 * SPIN_RATE;
  }
  if (start_following_ == true)
  {
    ball_follower_.startFollowing();
    start_following_ = false;
    on_following_ball_ = true;

    wait_count_ = 1 * SPIN_RATE;
  }

  //for debug
  //return;

  // check to stop
  if (stop_tracking_ == true)
  {
    ball_tracker_.stopTracking();
    stop_tracking_ = false;

    wait_count_ = 0;
  }
  if (stop_following_ == true)
  {
    ball_follower_.stopFollowing();
    stop_following_ = false;

    wait_count_ = 0;
  }

  if (wait_count_ <= 0)
  {
    // ball tracking
    if (on_tracking_ball_ == true)
    {
      switch(tracking_status)
      {
        case BallTracker::Found:
        if(on_following_ball_ == true)
          ball_follower_.processFollowing(ball_tracker_.getPanOfBall(), ball_tracker_.getTiltOfBall(), 0.0);

        else if(on_following_ball_ == false && after_shooting_ == false)
          start_following_ = true;

        else if(after_shooting_ == true)
        {
          int ball_position = ball_follower_.getBallPosition();
          if(ball_position != robotis_op::BallFollower::NotFound)
            count++;
          else
            count--;

          if(count > 200)
            handleKick(ball_position);
          else if(count < -200)
          {
            after_shooting_ = false;
            usleep(1000 * 1000);
            setModuleToDemo("action_module");
            usleep(1500 * 1000);

            playMotion(InitPose);
            setDemoDisable();
          }
        }

        if(tracking_status_ != tracking_status)
            setRGBLED(0x1F, 0x1F, 0x1F);
          break;

        case BallTracker::NotFound:
        if(on_following_ball_ == true)
          ball_follower_.waitFollowing();
        else if(on_following_ball_ == false && after_shooting_ == true)
        {
          count--;
        }
        if(count < -200)
        {
          after_shooting_ = false;
          usleep(1000 * 1000);
          setModuleToDemo("action_module");
          usleep(1500 * 1000);

          playMotion(InitPose);
          setDemoDisable();
        }

        if(tracking_status_ != tracking_status)
          setRGBLED(0, 0, 0);
        break;

        default:
          break;
      }

      if(tracking_status != tracking_status_)
        tracking_status_ = tracking_status;
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
        int ball_position = ball_follower_.getBallPosition();
        if (ball_position != robotis_op::BallFollower::NotFound)
        {
          ball_follower_.stopFollowing();
          handleKick(ball_position);
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

void PKDemo::processThread()
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

void PKDemo::callbackThread()
{
  ros::NodeHandle nh(ros::this_node::getName());

  // subscriber & publisher
  module_control_pub_ = nh.advertise<robotis_controller_msgs::JointCtrlModule>("/robotis/set_joint_ctrl_modules", 0);
  motion_index_pub_ = nh.advertise<std_msgs::Int32>("/robotis/action/page_num", 0);
  rgb_led_pub_ = nh.advertise<robotis_controller_msgs::SyncWriteItem>("/robotis/sync_write_item", 0);

  buttuon_sub_ = nh.subscribe("/robotis/open_cr/button", 1, &PKDemo::buttonHandlerCallback, this);
  demo_command_sub_ = nh.subscribe("/ball_tracker/command", 1, &PKDemo::demoCommandCallback, this);
  imu_data_sub_ = nh.subscribe("/robotis/open_cr/imu", 1, &PKDemo::imuDataCallback, this);

  is_running_client_ = nh.serviceClient<op3_action_module_msgs::IsRunning>("/robotis/action/is_running");

  while (nh.ok())
  {
    ros::spinOnce();

    usleep(1000);
  }
}

void PKDemo::setBodyModuleToDemo(const std::string &body_module, bool with_head_control)
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

  module_control_pub_.publish(control_msg);
  std::cout << "enable module of body : " << body_module << std::endl;
}

void PKDemo::setModuleToDemo(const std::string &module_name)
{
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

  module_control_pub_.publish(control_msg);
  std::cout << "enable module : " << module_name << std::endl;
}

void PKDemo::parseJointNameFromYaml(const std::string &path)
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
bool PKDemo::getJointNameFromID(const int &id, std::string &joint_name)
{
  std::map<int, std::string>::iterator _iter;

  _iter = id_joint_table_.find(id);
  if (_iter == id_joint_table_.end())
    return false;

  joint_name = _iter->second;
  return true;
}

// joint name -> joint id
bool PKDemo::getIDFromJointName(const std::string &joint_name, int &id)
{
  std::map<std::string, int>::iterator _iter;

  _iter = joint_id_table_.find(joint_name);
  if (_iter == joint_id_table_.end())
    return false;

  id = _iter->second;
  return true;
}

int PKDemo::getJointCount()
{
  return joint_id_table_.size();
}

bool PKDemo::isHeadJoint(const int &id)
{
  std::map<std::string, int>::iterator _iter;

  for (_iter = joint_id_table_.begin(); _iter != joint_id_table_.end(); ++_iter)
  {
    if (_iter->first.find("head") != std::string::npos)
      return true;
  }

  return false;
}

void PKDemo::buttonHandlerCallback(const std_msgs::String::ConstPtr& msg)
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

void PKDemo::demoCommandCallback(const std_msgs::String::ConstPtr &msg)
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
void PKDemo::imuDataCallback(const sensor_msgs::Imu::ConstPtr& msg)
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

void PKDemo::startSoccerMode()
{
  setModuleToDemo("action_module");

  usleep(10 * 1000);

  playMotion(WalkingReady);

  usleep(1500 * 1000);

  setBodyModuleToDemo("walking_module");

  usleep(10 * 1000);

  ROS_INFO("Start Soccer Demo");
  start_tracking_ = true;
}

void PKDemo::stopSoccerMode()
{
  ROS_INFO("Stop Soccer Demo");
  on_following_ball_ = false;
  stop_tracking_ = true;
  stop_following_ = true;
  after_shooting_ = false;
  count = 0;
}

void PKDemo::handleKick(int ball_position)
{
  usleep(1000 * 1000);

  // change to motion module
  setModuleToDemo("action_module");

  usleep(1500 * 1000);

  if (handleFallen(stand_state_) == true)
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
  after_shooting_ = true;

  usleep(2000 * 1000);

  if (handleFallen(stand_state_) == true)
    return;

  // ceremony
  //playMotion(Ceremony);
}

bool PKDemo::handleFallen(int fallen_status)
{
  if (fallen_status == Stand)
  {
    return false;
  }

  // change to motion module
  setModuleToDemo("action_module");

  usleep(600 * 1000);

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

void PKDemo::playMotion(int motion_index)
{
  std_msgs::Int32 motion_msg;
  motion_msg.data = motion_index;

  motion_index_pub_.publish(motion_msg);
}

void PKDemo::setRGBLED(int blue, int green, int red)
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
bool PKDemo::isActionRunning()
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

}
