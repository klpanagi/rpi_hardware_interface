cmake_minimum_required(VERSION 2.8.3)
project(camera_effector)

find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  std_msgs
  sensor_msgs
)

# --- For python camera_effector package --- #
catkin_python_setup()

#######################################################
#              Camera Effector Teleop                 #
#######################################################

find_package(Curses)
set(CURSES_LIBRARY "/usr/lib/arm-linux-gnueabihf/libcurses.so")
set(CURSES_INCLUDE_PATH "/usr/include")
#find_package(PkgConfig REQUIRED)
#pkg_check_modules(NCURSES REQUIRED ncurses)
#set( CMAKE_INCLUDE_PATH "/usr/include")
#set( CMAKE_LIBRARY_PATH "/usr/lib/arm-linux-gnueabihf")

catkin_package(
  CATKIN_DEPENDS
    message_runtime
    roscpp
    std_msgs
    geometry_msgs
  DEPENDS
    system_lib
    Curses
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


add_executable(${PROJECT_NAME}_teleop
  src/camera_effector_teleop/camera_effector_teleop.cpp
)


target_link_libraries(${PROJECT_NAME}_teleop
  ${catkin_LIBRARIES}
  ${CURSES_LIBRARY}
)

