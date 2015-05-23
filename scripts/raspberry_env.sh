#!/bin/bash

ETH_IP=$(ifconfig eth1 | grep "inet " | awk -F'[: ]+' '{ print $4 }')
echo $ETH_IP
export ROS_IP=$ETH_IP

source /home/ubuntu/rpi_ws/devel/setup.bash --extend

exec "$@"
