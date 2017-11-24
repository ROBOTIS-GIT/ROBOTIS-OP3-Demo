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

#include "op3_tele_op/op3_tele_op.h"

bool const DEBUG_PRINT = false;
const int SPIN_RATE = 30;

bool checkManagerRunning(std::string& manager_name)
{
  std::vector<std::string> node_list;
  ros::master::getNodes(node_list);

  for (unsigned int node_list_idx = 0; node_list_idx < node_list.size(); node_list_idx++)
  {
    if (node_list[node_list_idx] == manager_name)
      return true;
  }

  ROS_ERROR("Can't find op3_manager");
  return false;
}

//node main
int main(int argc, char **argv)
{
  //init ros
  ros::init(argc, argv, "op3_tele_op_node");

  ros::start();

  //set node loop rate
  ros::Rate loop_rate(SPIN_RATE);

  // wait for starting of manager
  std::string manager_name = "/op3_manager";
  while (ros::ok())
  {
    ros::Duration(1.0).sleep();

    if (checkManagerRunning(manager_name) == true)
    {
      break;
      ROS_INFO_COND(DEBUG_PRINT, "Succeed to connect");
    }
    ROS_WARN("Waiting for op3 manager");
  }

  robotis_op::OP3Teleop tele_op(DEBUG_PRINT);

  // init procedure
  //tele_op.playSound(default_mp3_path + "Demonstration ready mode.mp3");
  // turn on R/G/B LED
  tele_op.setLED(0x01 | 0x02 | 0x04);

  //node loop
  while (ros::ok())
  {
    //execute pending callbacks
    ros::spinOnce();

    //relax to fit output rate
    loop_rate.sleep();
  }

  //exit program
  return 0;
}




