#!/bin/bash

set -x

ETH_IP=$(ifconfig eth0 | grep "inet " | awk -F'[: ]+' '{ print $4 }')
echo $ETH_IP
export ROS_IP=$ETH_IP

source /home/$USER/pandora_ws/devel/setup.bash --extend

#source /home/$USER/scripts/launch_config_params.sh
#sleep 10

exec "$@"
