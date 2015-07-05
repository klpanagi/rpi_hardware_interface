cmake_minimum_required(VERSION 2.8.3)
project(ledstrip_controller)


find_package(catkin REQUIRED
  rospy
  std_msgs
  state_manager_msgs
)

catkin_python_setup()
