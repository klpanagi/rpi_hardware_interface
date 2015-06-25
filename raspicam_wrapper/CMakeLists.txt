cmake_minimum_required(VERSION 2.8.3)
project(raspicam_wrapper)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_srvs
)

#add_message_files(
  #FILES
  #flirLeptonMsg.msg
  #)

  #generate_messages(
  #DEPENDENCIES
  #std_msgs
  #sensor_msgs
  #)

catkin_package(
  INCLUDE_DIRS
    include
#  LIBRARIES flir_lepton_hardware_interface
  CATKIN_DEPENDS
    message_runtime
    roscpp
    std_srvs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node
  src/raspicam_wrapper_node.cpp
)

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)
