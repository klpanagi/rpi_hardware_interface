#!/bin/bash

export ROS_IP=10.0.1.2

source /home/ubuntu/rpi_ws/devel/setup.bash --extend

exec "$@"
