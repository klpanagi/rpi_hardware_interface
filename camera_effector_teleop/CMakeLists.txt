cmake_minimum_required(VERSION 2.8.3)
project(camera_effector_teleop)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  std_msgs
  roslint
)

find_package(Boost REQUIRED COMPONENTS thread)
# --- For python camera_effector package --- #
#catkin_python_setup()


catkin_package(
  CATKIN_DEPENDS
    message_runtime
    roscpp
    std_msgs
    geometry_msgs
  DEPENDS system_lib
)


if(Boost_FOUND)
  include_directories(${Boost_INCLUDE_DIRS})
endif()


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


add_executable(${PROJECT_NAME}_teleop
  src/camera_effector_teleop.cpp
)


target_link_libraries(${PROJECT_NAME}_teleop
  ${catkin_LIBRARIES}
)
