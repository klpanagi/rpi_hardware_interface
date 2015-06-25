#include "ros/ros.h"
#include "std_srvs/Empty.h"


int main(int argc, char** argv)
{
  
  ros::init(argc, argv, "raspicam_wrapper_node");
  ros::NodeHandle nh("/raspi_cam_wrapper");

  ros::ServiceClient client = nh.serviceClient<std_srvs::Empty::Request>( \
    "/raspicam/start_capture");
  std_srvs::Empty start_capture_srv;

  if (client.call(start_capture_srv))
  {
    ROS_INFO("PICAMERA started frame capturing");
  }
  else
  {
    ROS_FATAL("Failed to call camera/start_capture service");
  }

}
