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

#ifndef BUTTON_TEST_H_
#define BUTTON_TEST_H_

#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/String.h>
#include <boost/thread.hpp>

#include "op3_demo/op_demo.h"
#include "robotis_controller_msgs/SyncWriteItem.h"

namespace robotis_op
{

class ButtonTest : public OPDemo
{
 public:
  ButtonTest();
  ~ButtonTest();

  void setDemoEnable();
  void setDemoDisable();

 protected:
  const int SPIN_RATE;

  void processThread();
  void callbackThread();

  void process();

  void setRGBLED(int blue, int green, int red);
  void setLED(int led);

  void playSound(const std::string &path);

  void buttonHandlerCallback(const std_msgs::String::ConstPtr& msg);

  ros::Publisher rgb_led_pub_;
  ros::Publisher play_sound_pub_;
  ros::Subscriber buttuon_sub_;

  std::string default_mp3_path_;
  int led_count_;
  int rgb_led_count_;
};

} /* namespace robotis_op */

#endif /* BUTTON_TEST_H_ */
