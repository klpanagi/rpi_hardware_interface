#!/bin/bash

#export ROS_IP=155.207.33.186
export ROS_IP=10.0.1.1
export ROS_MASTER_URI=http://10.0.1.2:11311
#export ROS_MASTER_URI=http://155.207.33.185

source /home/pandora/rpi_ws/devel/setup.bash --extend

exec "$@"
