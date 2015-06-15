#include "ros/ros.h"


int main(int argc, char** argv)
{
  ros::init(argc, argv, "camera_effector_node");

  int rate;
  //ros::NodeHandle("/camera_effector").param<int>(
  //    "interface_rate", rate, 27);
  rate = 10;
  ros::Rate loop_rate(rate);

  while(ros::ok())
  {
    loop_rate.sleep();
  }

  return 0;
}
