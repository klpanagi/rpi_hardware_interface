#!/bin/bash

MASTER_IP=$1
MY_IP=$(ifconfig eth0 | grep "inet " | awk -F'[: ]+' '{ print $4 }')
export ROS_MASTER_URI=http://$MASTER_IP:11311
export ROS_IP=$MY_IP
echo "ROS_MASTER_URI = $ROS_MASTER_URI"
echo "ROS_IP = $ROS_IP"

