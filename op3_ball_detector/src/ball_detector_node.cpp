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

#include "op3_ball_detector/ball_detector.h"

//node main
int main(int argc, char **argv)
{
  //init ros
  ros::init(argc, argv, "ball_detector_node");

  //create ros wrapper object
  robotis_op::BallDetector detector;

  //set node loop rate
  ros::Rate loop_rate(30);

  //node loop
  while (ros::ok())
  {
    //if new image , do things
    if (detector.newImage())
    {
      detector.process();
      detector.publishImage();
      detector.publishCircles();

    }

    //execute pending callbacks
    ros::spinOnce();

    //relax to fit output rate
    loop_rate.sleep();
  }

  //exit program
  return 0;
}

