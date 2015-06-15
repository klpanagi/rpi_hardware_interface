cmake_minimum_required(VERSION 2.8.3)
project(camera_effector)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  std_msgs
  sensor_msgs
)

#add_message_files(
#  FILES
#)
#
#generate_messages(
#  DEPENDENCIES
#  std_msgs
#  sensor_msgs
#)

catkin_package(
  INCLUDE_DIRS
    include
#  LIBRARIES flir_lepton_hardware_interface
  CATKIN_DEPENDS
    message_runtime
    roscpp
    std_msgs
    sensor_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


################################################################################
#                      Flir Lepton Hardware Interface Lib                      #
################################################################################

add_library(${PROJECT_NAME}_lib
  src/camera_effector.cpp
  )
target_link_libraries(${PROJECT_NAME}_lib
  wiringPi
  pthread
  ${catkin_LIBRARIES}
  )

################################################################################
#                  Flir Lepton Hardware Interface Executable                   #
################################################################################

add_executable(${PROJECT_NAME}_node
  src/camera_effector_node.cpp
  )
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}_lib
  )
