cmake_minimum_required(VERSION 2.8.3)
project(raspicam_wrapper)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_srvs
)


catkin_package(
  CATKIN_DEPENDS
    message_runtime
    roscpp
    std_srvs
#  DEPENDS system_lib
)


include_directories(
  ${catkin_INCLUDE_DIRS}
)


add_executable(${PROJECT_NAME}_node
  src/raspicam_wrapper_node.cpp
)


target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)
