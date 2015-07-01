cmake_minimum_required(VERSION 2.8.3)
project(camera_effector)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
)

# --- For python camera_effector package --- #
catkin_python_setup()


#catkin_package(
  #INCLUDE_DIRS
    #include
  #CATKIN_DEPENDS
    #message_runtime
    #rospy
    #std_msgs
  #DEPENDS system_lib
#)

#include_directories(
  #include
  #${catkin_INCLUDE_DIRS}
#)


#add_library(${PROJECT_NAME}_lib
  #src/camera_effector.cpp
  #)
#target_link_libraries(${PROJECT_NAME}_lib
  #pthread
  #${catkin_LIBRARIES}
#)


#add_executable(${PROJECT_NAME}_node
  #src/camera_effector_node.cpp
  #)
#target_link_libraries(${PROJECT_NAME}_node
  #${catkin_LIBRARIES}
  #${PROJECT_NAME}_lib
  #)
