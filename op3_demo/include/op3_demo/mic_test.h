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

#ifndef MIC_TEST_H_
#define MIC_TEST_H_

#include <signal.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/String.h>
#include <boost/thread.hpp>

#include "op3_demo/op_demo.h"
#include "robotis_controller_msgs/SyncWriteItem.h"

namespace robotis_op
{

class MicTest : public OPDemo
{
 public:
  enum Mic_Test_Status
  {
    Ready = 0,
    AnnounceRecording = 1,
    MicRecording = 2,
    PlayingSound = 3,
    DeleteTempFile = 4,
    DemoCount = 5
  };

  MicTest();
  ~MicTest();

  void setDemoEnable();
  void setDemoDisable();

 protected:
  const int SPIN_RATE;

  void processThread();
  void callbackThread();

  void process();

  void announceTest();
  void recordSound(int recording_time);
  void recordSound();
  void playTestSound(const std::string &path);
  void playSound(const std::string &file_path);
  void deleteSoundFile(const std::string &file_path);

  void startTimer(double wait_time);
  void finishTimer();

  void buttonHandlerCallback(const std_msgs::String::ConstPtr& msg);

  std::string recording_file_name_;
  std::string default_mp3_path_;

  ros::Publisher play_sound_pub_;
  ros::Subscriber buttuon_sub_;

  ros::Time start_time_;
  double wait_time_;
  bool is_wait_;
  int record_pid_;
  int play_pid_;
  int test_status_;
};

} /* namespace robotis_op */

#endif /* MIC_TEST_H_ */
