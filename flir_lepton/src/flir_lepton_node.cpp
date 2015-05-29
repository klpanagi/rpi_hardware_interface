#include "ros/ros.h"

#include "flir_lepton/flir_lepton.h"

using namespace flir_lepton;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "flir_lepton_node");
  FlirLeptonHardwareInterface flirLepron("/flir_lepton");

  int rate;
  ros::NodeHandle("/flir_lepton").param<int>(
      "interface_rate", rate, 27);
  ros::Rate loop_rate(rate);

  while(ros::ok())
  {
    flirLepron.run();
    loop_rate.sleep();
  }

  return 0;
}
