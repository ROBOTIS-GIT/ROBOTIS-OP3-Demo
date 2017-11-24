#ifndef KEEPER_DEMO_H
#define KEEPER_DEMO_H

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Imu.h>
#include <boost/thread.hpp>

#include "op3_demo/op_demo.h"
#include "op3_demo/ball_tracker.h"
#include "op3_demo/ball_follower.h"
#include "robotis_math/robotis_linear_algebra.h"
#include "op3_action_module_msgs/IsRunning.h"
#include "robotis_controller_msgs/SyncWriteItem.h"

namespace robotis_op
{

class KeeperDemo : public OPDemo
{
 public:
  enum Stand_Status
  {
    Stand = 0,
    Fallen_Forward = 1,
    Fallen_Behind = 2,
  };

  enum
  {
    NotFound = 0,
    OnRight = 1,
    OnLeft = 2,
  };

  enum Robot_Status
  {
    Waited = 0,
    TrackingAndFollowing = 1,
    ReadyToKick = 2,
    ReadyToCeremony = 3,
    ReadyToGetup = 4,
  };

  KeeperDemo();
  ~KeeperDemo();

  void setDemoEnable();
  void setDemoDisable();

 protected:
  const double FALL_FORWARD_LIMIT;
  const double FALL_BACK_LIMIT;
  const int SPIN_RATE;
  const bool DEBUG_PRINT;

  void processThread();
  void callbackThread();

  void getDistance();
  void setBodyModuleToDemo(const std::string &body_module, bool with_head_control = true);
  void setModuleToDemo(const std::string &module_name);
  void parseJointNameFromYaml(const std::string &path);
  bool getJointNameFromID(const int &id, std::string &joint_name);
  bool getIDFromJointName(const std::string &joint_name, int &id);
  int getJointCount();
  bool isHeadJoint(const int &id);
  void buttonHandlerCallback(const std_msgs::String::ConstPtr& msg);
  void demoCommandCallback(const std_msgs::String::ConstPtr& msg);
  void imuDataCallback(const sensor_msgs::Imu::ConstPtr& msg);
  void presentJointStatesCallback(const sensor_msgs::JointState::ConstPtr &msg);

  void startKeeperMode();
  void stopKeeperMode();
  void startBallFollower();
  void BallTrackerOnly();

  void process();
  void handleDeffence(int ball_position);
  bool handleFallen(int fallen_status);

  void playMotion(int motion_index);
  void setRGBLED(int blue, int green, int red);
  bool isActionRunning();

  BallTracker ball_tracker_;
  BallTracker ball_position_;
  BallFollower ball_follower_;

  ros::Publisher module_control_pub_;
  ros::Publisher motion_index_pub_;
  ros::Publisher rgb_led_pub_;
  ros::Subscriber button_sub_;
  ros::Subscriber demo_command_sub_;
  ros::Subscriber present_joint_states_sub_;
  ros::Subscriber imu_data_sub_;

  ros::ServiceClient is_running_client_;

  std::map<int, std::string> id_joint_table_;
  std::map<std::string, int> joint_id_table_;

  int wait_count_;
  int count_to_follow;
  int deffence_count_;
  int deffence_count_1_;
  int action_count_;
  bool on_tracking_ball_;
  bool on_following_ball_;
  bool after_shooting;
  bool restart_keeper_;
  bool start_tracking_;
  bool stop_tracking_;
  bool stop_fallen_check_;
  bool PK_mode;
  int robot_status_;
  int tracking_status_;
  int stand_state_;
  int ball_position;
  int following_count_;
  int count;
  double present_pitch_;
  double head_pan_, head_tilt_;
  double ball_distance_x, ball_distance_y;
  double ball_distance_x_1;
  double ball_velocity;
  double past_distance;
  ros::Time prev_time_;
};

}  // namespace robotis_op
#endif // KEEPER_DEMO_H
