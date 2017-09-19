#include "op3_demo/keeper_demo.h"

namespace robotis_op
{

KeeperDemo::KeeperDemo()
  : FALL_FORWARD_LIMIT(60),
    FALL_BACK_LIMIT(-60),
    SPIN_RATE(45), //Hz
    DEBUG_PRINT(false),
    wait_count_(0),
    restart_keeper_(false),
    start_tracking_(false),
    stop_tracking_(false),
    stop_fallen_check_(false),
    on_tracking_ball_(false),
    on_following_ball_(false),
    after_shooting(false),
    PK_mode(true),
    following_count_(0),
    robot_status_(Waited),
    stand_state_(Stand),
    tracking_status_(BallTracker::Waiting),
    present_pitch_(0),
    ball_position(0),
    count(0)
{
  //init ros
  enable_ = false;

  ros::NodeHandle nh(ros::this_node::getName());

  std::string default_path = ros::package::getPath("op3_gui_demo") + "/config/demo_config.yaml";
  std::string path = nh.param<std::string>("demo_config", default_path);
  parseJointNameFromYaml(path);

  boost::thread queue_thread = boost::thread(boost::bind(&KeeperDemo::callbackThread, this));
  boost::thread process_thread = boost::thread(boost::bind(&KeeperDemo::processThread, this));
}

KeeperDemo::~KeeperDemo()
{

}

void KeeperDemo::setDemoEnable()
{
  enable_ = true;
  setBodyModuleToDemo("action_module");
  usleep(100 * 1000);
  startKeeperMode();
}

void KeeperDemo::setDemoDisable()
{
  setModuleToDemo("base_module");

  // handle disable procedure
  ball_tracker_.stopTracking();
  ball_follower_.stopFollowing();

  enable_ = false;
  wait_count_ = 0;
  following_count_ = 0;
  on_tracking_ball_ = false;
  restart_keeper_ = false;
  start_tracking_ = false;
  stop_tracking_ = false;
  stop_fallen_check_ = false;

  tracking_status_ = BallTracker::Waiting;
}

void KeeperDemo::process()
{
  int tracking_status;

  tracking_status = ball_tracker_.processTracking();

  // check to start tracking
  if (start_tracking_ == true)
  {
    ball_tracker_.startTracking();
    start_tracking_ = false;
    on_tracking_ball_ = true;

    count_to_follow = 0;
    deffence_count_ = 0;
    deffence_count_1_ = 0;
    action_count_ = 0;

    wait_count_ = 1 * SPIN_RATE;
  }

  // check to stop tracking & following
  if (stop_tracking_ == true)
  {
    ball_tracker_.stopTracking();
    //ball_follower_.stopFollowing();
    stop_tracking_ = false;
    on_tracking_ball_ = false;
    on_following_ball_ = false;

    wait_count_ = 0;
    count_to_follow = 0;
    deffence_count_ = 0;
    action_count_ = 0;
  }

  //// keeper algorithm ////
  if (wait_count_ <= 0)
  {
    if (on_tracking_ball_ == true)
    {
      switch(tracking_status)
      {

      case BallTracker::Found:
      {
        action_count_ = 0;
        getDistance();

        /////For PK/////
        if(PK_mode == true)
        {
          double flag = ball_distance_x -  ball_distance_x_1;
          if(flag<0)
          {
            deffence_count_1_ ++;
            printf("deffence_count_PK = %d   Dx = %6.4f  \n", deffence_count_1_, ball_distance_x);
          }
          else
            deffence_count_1_ = 0;          

          ball_distance_x_1 = ball_distance_x;

          if ( (deffence_count_1_ > 4 && ball_distance_x < 0.75) ||
              (deffence_count_1_ > 9 && ball_distance_x < 0.85) )
          {
            deffence_count_1_ = 0;
            deffence_count_ = 0;

            // check states for Deffence Action
            if (ball_distance_y > 0)
              ball_position = OnLeft;
            else
              ball_position = OnRight;
            handleDeffence(ball_position);
          }

        }
        ///////

        count++;
        printf("count: %d || Dx : %6.4f, Dy : %6.4f, V : %6.4f      %d \n", count, ball_distance_x, ball_distance_y, ball_velocity, deffence_count_);


        if (
            (ball_distance_x > 0.00) &&
            (ball_velocity >= 0.00) &&
            (ball_velocity <= 7.00) &&
            (ball_distance_x < 0.30) ||
            (ball_distance_x < 0.60 && ball_velocity >= 0.30) ||
            (ball_distance_x < 0.75 && ball_velocity >= 0.65) ||
            (ball_distance_x < 0.95 && ball_velocity >= 0.90)
            )//m
        {
          deffence_count_ ++;
          //printf("count: %d || Dx : %6.4f, Dy : %6.4f, V : %6.4f  count: %d\n", deffence_count_, ball_distance_x, ball_distance_y, ball_velocity, count);

          if(deffence_count_ > 2)
          {
            deffence_count_ = 0;
            deffence_count_1_ = 0;

            // check states for Deffence Action
            if (ball_distance_y > 0)
              ball_position = OnLeft;
            else
              ball_position = OnRight;
            handleDeffence(ball_position);
          }
        }

        else
        {
          deffence_count_ = 0;
          //printf("TOO FAR  || Dx : %6.4f, Dy : %6.4f, V : %6.4f \n", ball_x, ball_y, ball_velocity);
        }
        if(tracking_status_ != tracking_status)
          setRGBLED(0x1F, 0x1F, 0x1F);
        break;
      }

      case BallTracker::NotFound:
      {
        // can't see the ball & following ball is running
        if(on_following_ball_ == true)
        {
          following_count_ ++;
          if (following_count_ >= 90)
            BallTrackerOnly();
        }
        else
        {
          ball_position = NotFound;
          following_count_ = 0;
          action_count_ ++;
          if(action_count_ > 400)
          {
            handleDeffence(ball_position);
            action_count_ = 0;
          }
        deffence_count_ = 0;
        }
        break;
      }

      default:
        break;

      //      //need to consider the robot's movement for velocity value
      //            if (ball_distance < 0.20 && ball_velocity < 0.20) //m
      //            {
      //              count_to_follow ++;
      //              if(on_following_ball_ == false && count_to_follow >30)
      //              {
      //                printf("Time to BallFollow\n");
      //                startBallFollower();
      //                following_count_ = 0;
      //              }
      //            }

      }
    }

    // check fallen states
    switch (stand_state_)
    {
    case Stand:
    {
      // check restart soccer
      if (restart_keeper_ == true)
      {
        restart_keeper_ = false;
        startKeeperMode();
        break;
      }
      break;
    }
      // fallen state : Fallen_Forward, Fallen_Behind
    default:
    {
      ball_follower_.stopFollowing();
      //ball_tracker_.stopTracking();
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

void KeeperDemo::processThread()
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

void KeeperDemo::callbackThread()
{
  ros::NodeHandle nh(ros::this_node::getName());

  // subscriber & publisher
  module_control_pub_ = nh.advertise<robotis_controller_msgs::JointCtrlModule>("/robotis/set_joint_ctrl_modules", 0);
  motion_index_pub_ = nh.advertise<std_msgs::Int32>("/robotis/action/page_num", 0);
  rgb_led_pub_ = nh.advertise<robotis_controller_msgs::SyncWriteItem>("/robotis/sync_write_item", 0);

  button_sub_ = nh.subscribe("/robotis/open_cr/button", 1, &KeeperDemo::buttonHandlerCallback, this);
  demo_command_sub_ = nh.subscribe("/ball_tracker/command", 1, &KeeperDemo::demoCommandCallback, this);
  imu_data_sub_ = nh.subscribe("/robotis/open_cr/imu", 1, &KeeperDemo::imuDataCallback, this);
  present_joint_states_sub_ = nh.subscribe("/robotis/present_joint_states", 10, &KeeperDemo::presentJointStatesCallback, this);

  is_running_client_ = nh.serviceClient<op3_action_module_msgs::IsRunning>("/robotis/action/is_running");


  while (nh.ok())
  {
    ros::spinOnce();

    usleep(1000);
  }
}

void KeeperDemo::getDistance()
{
  ros::Time curr_time = ros::Time::now();
  ros::Duration dur = curr_time - prev_time_;
  double delta_time = dur.nsec * 0.000000001 + dur.sec;
  prev_time_ = curr_time;

  //double ball_radius = 0.06;
  double ground_to_headtilt = 0.43; //ank_to_headtilt_z(0.386956) + ground_to_ank(0.045)
  double ank_to_headpan_p = 0.20; //ankle to headpan pitch: 0.207614 (rad)
  double ank_to_headtilt_x = 0.0619; //0.061948
  double headtilt_to_cam = 0.05;
  //Height that consider the ball_radius
  //ground_to_headtilt(including spike) + headtilt_to_cam - ball_radius
  double cam_height = ground_to_headtilt + headtilt_to_cam * cos(-head_tilt_); //- ball_radius;
  //ROS_INFO("CAM_HEIGHT: %f", cam_height - ball_radius);

  double x_error = ball_tracker_.getPanOfBall();
  double y_error = ball_tracker_.getTiltOfBall();

  //ball_distance from head position
  ball_distance_x = cam_height * tan(M_PI *0.5 - ank_to_headpan_p + head_tilt_ + y_error)
      + headtilt_to_cam * sin(-head_tilt_) + ank_to_headtilt_x;
  ball_distance_y = sqrt(pow(cam_height,2) + pow(ball_distance_x,2)) * tan(x_error);

  //ball_distance from body position
  ball_distance_x = ball_distance_x * cos(head_pan_) + ball_distance_y * sin(head_pan_);
  ball_distance_y = ball_distance_x * sin(head_pan_) + ball_distance_y * cos(head_pan_);

  ball_velocity = -(ball_distance_x - past_distance) / delta_time;
  past_distance = ball_distance_x;
}

void KeeperDemo::setBodyModuleToDemo(const std::string &body_module, bool with_head_control)
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

void KeeperDemo::setModuleToDemo(const std::string &module_name)
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

void KeeperDemo::parseJointNameFromYaml(const std::string &path)
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
bool KeeperDemo::getJointNameFromID(const int &id, std::string &joint_name)
{
  std::map<int, std::string>::iterator _iter;

  _iter = id_joint_table_.find(id);
  if (_iter == id_joint_table_.end())
    return false;

  joint_name = _iter->second;
  return true;
}

// joint name -> joint id
bool KeeperDemo::getIDFromJointName(const std::string &joint_name, int &id)
{
  std::map<std::string, int>::iterator _iter;

  _iter = joint_id_table_.find(joint_name);
  if (_iter == joint_id_table_.end())
    return false;

  id = _iter->second;
  return true;
}

int KeeperDemo::getJointCount()
{
  return joint_id_table_.size();
}

bool KeeperDemo::isHeadJoint(const int &id)
{
  std::map<std::string, int>::iterator _iter;

  for (_iter = joint_id_table_.begin(); _iter != joint_id_table_.end(); ++_iter)
  {
    if (_iter->first.find("head") != std::string::npos)
      return true;
  }

  return false;
}

void KeeperDemo::buttonHandlerCallback(const std_msgs::String::ConstPtr& msg)
{
  if (enable_ == false)
    return;

  if (msg->data == "start")
  {
    if (on_tracking_ball_ == true)
      stopKeeperMode();
    else
      startKeeperMode();
  }
}

void KeeperDemo::demoCommandCallback(const std_msgs::String::ConstPtr &msg)
{
  if (enable_ == false)
    return;

  if (msg->data == "start")
  {
    if (on_tracking_ball_ == true)
      stopKeeperMode();
    else
      startKeeperMode();
  }
  else if (msg->data == "stop")
  {
    stopKeeperMode();
  }
}

// check fallen states
void KeeperDemo::imuDataCallback(const sensor_msgs::Imu::ConstPtr& msg)
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

void KeeperDemo::presentJointStatesCallback(const sensor_msgs::JointState::ConstPtr &msg)
{
  double pan, tilt;
  int get_count = 0;

  for (int ix = 0; ix < msg->name.size(); ix++)
  {
    if (msg->name[ix] == "head_pan")
    {
      pan = msg->position[ix];
      get_count += 1;
    }
    else if (msg->name[ix] == "head_tilt")
    {
      tilt = msg->position[ix];
      get_count += 1;
    }

    if (get_count == 2)
      break;
  }
  head_tilt_ = tilt;
  head_pan_ = pan;
}

void KeeperDemo::startKeeperMode()
{
  usleep(500 * 1000);
  ROS_INFO("Start Keeper Demo");
//  if(after_shooting == true)
//    playMotion(KeeperReReady);
//  else
    playMotion(KeeperReady);
  while(isActionRunning() == true)
    usleep(100 * 1000);
  usleep(800 * 1000);
  start_tracking_ = true;
}

void KeeperDemo::stopKeeperMode()
{
  ROS_INFO("Stop Keeper Demo");
  on_tracking_ball_ = false;
  on_following_ball_ = false;
  stop_tracking_ = true;
}

void KeeperDemo::startBallFollower()
{
  ROS_INFO("Start Keeper Following");
  setBodyModuleToDemo("walking_module");


  on_following_ball_ = true;

  ball_follower_.startFollowing();
  ball_follower_.processFollowing(ball_tracker_.getPanOfBall(), ball_tracker_.getTiltOfBall(), ball_tracker_.getBallSize());
}

void KeeperDemo::BallTrackerOnly()
{
  ROS_INFO("Start Keeper Tracking Only");
  ball_follower_.stopFollowing();
  on_following_ball_ = false;
  following_count_ = 0;
  //////////
  setBodyModuleToDemo("action_module");
  usleep(10 * 1000);
  playMotion(KeeperReady);
  usleep(1000 * 1000);
}
void KeeperDemo::handleDeffence(int ball_position)
{
  ROS_INFO("Deffence Motion");
//  if (on_following_ball_ == true)
//    ball_follower_.stopFollowing();
//  on_following_ball_ = false;

  // change to motion module
//  usleep(50 * 1000);
//  setBodyModuleToDemo("action_module");
//  usleep(100 * 1000);

  if (handleFallen(stand_state_) == true)
    return;

  // Deffence motion
  switch (ball_position)
  {
  case OnRight:
    playMotion(KeeperRight);
    ROS_INFO ("RIGHT");
    break;

  case OnLeft:
    playMotion(KeeperLeft);
    ROS_INFO ("LEFT");
    break;

  case NotFound:
    playMotion(KeeperAction);
    ROS_INFO ("NO BALL ACTION");
    break;

  default:
    break;
  }

  if (ball_position == OnRight || ball_position == OnLeft)
  {
    restart_keeper_ = true;
    after_shooting = true;
  }

  usleep(1500 * 1000);

  if (handleFallen(stand_state_) == true)
    return;
}

bool KeeperDemo::handleFallen(int fallen_status)
{
  if (fallen_status == Stand)
  {
    return false;
  }

  // change to motion module
//  setBodyModuleToDemo("action_module");

//  usleep(100 * 1000);

  // getup motion
  switch (fallen_status)
  {
  case Fallen_Forward:
    std::cout << "Getup Motion [F]: " << std::endl;
    playMotion(GetUpFront);
    break;

  case Fallen_Behind:
    std::cout << "Getup Motion [B]: " << std::endl;
    playMotion(GetUpBack);
    break;

  default:
    break;
  }

  while(isActionRunning() == true)
    usleep(100 * 1000);

  usleep(650 * 1000);

  restart_keeper_ = true;
  // reset state
  on_following_ball_ = false;

  return true;
}

void KeeperDemo::playMotion(int motion_index)
{
  std_msgs::Int32 motion_msg;
  motion_msg.data = motion_index;

  motion_index_pub_.publish(motion_msg);
}

void KeeperDemo::setRGBLED(int blue, int green, int red)
{
  int led_full_unit = 0x1F;
  int led_value = (blue & led_full_unit) << 10 | (green & led_full_unit) << 5 | (red & led_full_unit);
  robotis_controller_msgs::SyncWriteItem syncwrite_msg;
  syncwrite_msg.item_name = "LED_RGB";
  syncwrite_msg.joint_name.push_back("open-cr");
  syncwrite_msg.value.push_back(led_value);

  rgb_led_pub_.publish(syncwrite_msg);
}

bool KeeperDemo::isActionRunning()
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
