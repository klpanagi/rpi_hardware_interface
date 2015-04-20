#include "ros/ros.h"
#include "flir_lepton_hardware_interface/flir_lepton_hardware_interface.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "flir_lepton_hardware_interface_node");
  FlirLeptonHardwareInterface flirLepron;

  // TODO Get flir lepton's interface's rate from config file
  ros::Rate loop_rate(27);

  while(ros::ok())
  {
    flirLepron.run();
    loop_rate.sleep();
  }

  return 0;
}
