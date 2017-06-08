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

#include "op3_demo/button_test.h"

namespace robotis_op
{

ButtonTest::ButtonTest()
    : SPIN_RATE(30),
      led_count_(0),
      rgb_led_count_(0)
{
  enable_ = false;

  ros::NodeHandle nh(ros::this_node::getName());

  boost::thread queue_thread = boost::thread(boost::bind(&ButtonTest::callbackThread, this));
  boost::thread process_thread = boost::thread(boost::bind(&ButtonTest::processThread, this));

  default_mp3_path_ = ros::package::getPath("op3_demo") + "/Data/mp3/test/";
}

ButtonTest::~ButtonTest()
{
}

void ButtonTest::setDemoEnable()
{
  enable_ = true;

  ROS_INFO("Start Button Test");
}

void ButtonTest::setDemoDisable()
{
  enable_ = false;
}

void ButtonTest::process()
{

}

void ButtonTest::processThread()
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

void ButtonTest::callbackThread()
{
  ros::NodeHandle nh(ros::this_node::getName());

  // subscriber & publisher
  rgb_led_pub_ = nh.advertise<robotis_controller_msgs::SyncWriteItem>("/robotis/sync_write_item", 0);
  play_sound_pub_ = nh.advertise<std_msgs::String>("/play_sound_file", 0);

  buttuon_sub_ = nh.subscribe("/robotis/open_cr/button", 1, &ButtonTest::buttonHandlerCallback, this);

  while (nh.ok())
  {
    ros::spinOnce();

    usleep(1 * 1000);
  }
}

// button test
void ButtonTest::buttonHandlerCallback(const std_msgs::String::ConstPtr& msg)
{
  if (enable_ == false)
    return;

  if (msg->data == "mode")
  {
    playSound(default_mp3_path_ + "Mode button pressed.mp3");
  }
  else if (msg->data == "start")
  {
    // RGB led color test
    playSound(default_mp3_path_ + "Start button pressed.mp3");
    int rgb_selector[3] = { 1, 0, 0 };
    setRGBLED((0x1F * rgb_selector[rgb_led_count_ % 3]), (0x1F * rgb_selector[(rgb_led_count_ + 1) % 3]),
              (0x1F * rgb_selector[(rgb_led_count_ + 2) % 3]));
    rgb_led_count_ += 1;
  }
  else if (msg->data == "user")
  {
    // Monochromatic led color test
    playSound(default_mp3_path_ + "User button pressed.mp3");
    setLED(0x01 << (led_count_++ % 3));
  }
  else if (msg->data == "mode_long")
  {
    playSound(default_mp3_path_ + "Mode button long pressed.mp3");
  }
  else if (msg->data == "start_long")
  {
    playSound(default_mp3_path_ + "Start button long pressed.mp3");
  }
  else if (msg->data == "user_long")
  {
    playSound(default_mp3_path_ + "User button long pressed.mp3");
  }
}

void ButtonTest::setRGBLED(int blue, int green, int red)
{
  int led_full_unit = 0x1F;
  int led_value = (blue & led_full_unit) << 10 | (green & led_full_unit) << 5 | (red & led_full_unit);
  robotis_controller_msgs::SyncWriteItem syncwrite_msg;
  syncwrite_msg.item_name = "LED_RGB";
  syncwrite_msg.joint_name.push_back("open-cr");
  syncwrite_msg.value.push_back(led_value);

  rgb_led_pub_.publish(syncwrite_msg);
}

void ButtonTest::setLED(int led)
{
  robotis_controller_msgs::SyncWriteItem syncwrite_msg;
  syncwrite_msg.item_name = "LED";
  syncwrite_msg.joint_name.push_back("open-cr");
  syncwrite_msg.value.push_back(led);

  rgb_led_pub_.publish(syncwrite_msg);
}

void ButtonTest::playSound(const std::string &path)
{
  std_msgs::String sound_msg;
  sound_msg.data = path;

  play_sound_pub_.publish(sound_msg);
}

} /* namespace robotis_op */
