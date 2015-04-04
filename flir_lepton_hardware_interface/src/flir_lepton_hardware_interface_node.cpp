#include "flir_lepton_hardware_interface/flir_lepton_hardware_interface.h"


int main(int argc, char** argv)
{
  ros::init(argc, argv, "flir_lepton_hardware_interface_node");
  FlirLeptonHardwareInterface flirLepron;  

  ros::Rate loop_rate(10);

  while(ros::ok())
  {
    flirLepron.run();
    loop_rate.sleep();
  }

  return 0;
}
