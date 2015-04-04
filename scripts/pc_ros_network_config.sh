#!/bin/bash

MY_IP=$(ifconfig wlan0 | grep "inet " | awk -F'[: ]+' '{ print $4 }')
echo $MY_IP
export ROS_MASTER_URI=http://$MY_IP:11311
export ROS_IP=$MY_IP


