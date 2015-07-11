cmake_minimum_required(VERSION 2.8.3)
project(ledstrip_controller)


find_package(catkin REQUIRED
  rospy
  dynamic_reconfigure
  std_msgs
  state_manager_msgs
)

#zzinclude_directories(${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake)

catkin_python_setup()

include_directories(
  ${catkin_INCLUDE_DIRS}
  )


generate_dynamic_reconfigure_options(cfg/ledstrip_controller.cfg)

