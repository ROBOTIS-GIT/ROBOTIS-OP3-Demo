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

#include "op3_demo/mic_test.h"

namespace robotis_op
{

MicTest::MicTest()
    : SPIN_RATE(30),
      is_wait_(false),
      wait_time_(-1),
      test_status_(Ready),
      record_pid_(-1),
      play_pid_(-1)
{
  enable_ = false;

  ros::NodeHandle nh(ros::this_node::getName());

  boost::thread queue_thread = boost::thread(boost::bind(&MicTest::callbackThread, this));
  boost::thread process_thread = boost::thread(boost::bind(&MicTest::processThread, this));

  recording_file_name_ = ros::package::getPath("op3_demo") + "/data/mp3/test/mic-test.wav";
  default_mp3_path_ = ros::package::getPath("op3_demo") + "/data/mp3/test/";

  start_time_ = ros::Time::now();
}

MicTest::~MicTest()
{
}

void MicTest::setDemoEnable()
{
  wait_time_ = -1;
  test_status_ = AnnounceRecording;
  enable_ = true;

  ROS_INFO("Start Mic test Demo");
}

void MicTest::setDemoDisable()
{
  finishTimer();

  test_status_ = Ready;
  enable_ = false;
}

void MicTest::process()
{
  // check status
  // timer
  if (wait_time_ > 0)
  {
    ros::Duration dur = ros::Time::now() - start_time_;

    // check timer
    if (dur.sec >= wait_time_)
    {
      finishTimer();
    }
  }
  else if (wait_time_ == -1.0)
  {
    // handle test process
    switch (test_status_)
    {
      case Ready:
        // do nothing
        break;

      case AnnounceRecording:
        announceTest();
        test_status_ = MicRecording;
        break;

      case MicRecording:
        recordSound();
        test_status_ = PlayingSound;
        break;

      case PlayingSound:
        playTestSound(recording_file_name_);
        test_status_ = DeleteTempFile;
        break;

      case DeleteTempFile:
        deleteSoundFile(recording_file_name_);
        test_status_ = Ready;
        break;

      default:
        break;
    }
  }

}

void MicTest::processThread()
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

void MicTest::callbackThread()
{
  ros::NodeHandle nh(ros::this_node::getName());

  // subscriber & publisher
  buttuon_sub_ = nh.subscribe("/robotis/open_cr/button", 1, &MicTest::buttonHandlerCallback, this);
  play_sound_pub_ = nh.advertise<std_msgs::String>("/play_sound_file", 0);

  while (nh.ok())
  {
    ros::spinOnce();

    usleep(1 * 1000);
  }
}

void MicTest::buttonHandlerCallback(const std_msgs::String::ConstPtr& msg)
{
  if (enable_ == false)
    return;

  if (msg->data == "start")
  {
    // restart mic test
    if (test_status_ != Ready)
      return;

    test_status_ = AnnounceRecording;
  }
  else if(msg->data == "user")
  {
    is_wait_ = true;
  }
}

void MicTest::announceTest()
{
  // play mic test sound
  playSound(default_mp3_path_ + "Announce mic test.mp3");

  usleep(3.4 * 1000 * 1000);
}

void MicTest::recordSound(int recording_time)
{
  ROS_INFO("Start to record");

  playSound(default_mp3_path_ + "Start recording.mp3");

  usleep(1.5 * 1000 * 1000);

  if (record_pid_ != -1)
    kill(record_pid_, SIGKILL);

  record_pid_ = fork();

  switch (record_pid_)
  {
    case -1:
      fprintf(stderr, "Fork Failed!! \n");
      ROS_WARN("Fork Failed!! \n");
      break;

    case 0:
    {
      std::stringstream ss;
      ss << "-d " << recording_time;
      execl("/usr/bin/arecord", "arecord", "-Dplughw:1,0", "-fS16_LE", "-c1", "-r22050", "-twav", ss.str().c_str(),
            recording_file_name_.c_str(), (char*) 0);
      break;
    }

    default:
      break;
  }

  startTimer(recording_time);
}

void MicTest::recordSound()
{
  recordSound(5);
}

void MicTest::playTestSound(const std::string &path)
{
  ROS_INFO("Start to play recording sound");

  playSound(default_mp3_path_ + "Start playing.mp3");

  usleep(1.3 * 1000 * 1000);

  if (play_pid_ != -1)
    kill(play_pid_, SIGKILL);

  play_pid_ = fork();

  switch (play_pid_)
  {
    case -1:
      fprintf(stderr, "Fork Failed!! \n");
      ROS_WARN("Fork Failed!! \n");
      break;

    case 0:
      execl("/usr/bin/aplay", "aplay", path.c_str(), (char*) 0);
      break;

    default:
      break;
  }

  startTimer(5);
}

void MicTest::playSound(const std::string &path)
{
  std_msgs::String sound_msg;
  sound_msg.data = path;

  play_sound_pub_.publish(sound_msg);
}

void MicTest::deleteSoundFile(const std::string &file_path)
{
  remove(file_path.c_str());
  ROS_INFO("Delete temporary file");
}

void MicTest::startTimer(double wait_time)
{
  start_time_ = ros::Time::now();
  wait_time_ = wait_time;
}

void MicTest::finishTimer()
{
  wait_time_ = -1;
}

} /* namespace robotis_op */
