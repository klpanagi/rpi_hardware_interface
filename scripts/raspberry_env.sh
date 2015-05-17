#!/bin/bash

export ROS_IP=155.207.33.186

source /home/ubuntu/rpi_ws/devel/setup.bash --extend

exec "$@"
