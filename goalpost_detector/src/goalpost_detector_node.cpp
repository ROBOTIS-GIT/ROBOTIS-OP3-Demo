/* Author: Seri Lee */

#include "goalpost_detector/goalpost_detector.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "goalpost_detector_node");

  //create ros wrapper object
  robotis_op::GoalpostDetector postdetector;

  //set node loop rate
  ros::Rate loop_rate(30);

  //node loop
  while (ros::ok())
  {
    //if new image , do things
    if (postdetector.newImage())
    {
      postdetector.process();
      postdetector.publishImage();
      postdetector.publishLines();
    }

    //execute pending callbacks
    ros::spinOnce();

    //relax to fit output rate
    loop_rate.sleep();
  }

  //exit program
  return 0;
}
