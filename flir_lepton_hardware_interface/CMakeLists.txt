cmake_minimum_required(VERSION 2.8.3)
project(flir_lepton_hardware_interface)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
)

catkin_package(
  INCLUDE_DIRS
    include
#  LIBRARIES flir_lepton_hardware_interface
  CATKIN_DEPENDS
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
  src/flir_lepton_hardware_interface.cpp
  )
target_link_libraries(flir_lepton_hardware_interface_node
  ${catkin_LIBRARIES}
  )

################################################################################
#                  Flir Lepton Hardware Interface Executable                   #
################################################################################

add_executable(${PROJECT_NAME}_node
  src/flir_lepton_hardware_interface_node.cpp
  )
target_link_libraries(flir_lepton_hardware_interface_node
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}_lib
  )
