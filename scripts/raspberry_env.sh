#!/bin/bash

set -x
echo -e "\033[0m\n"

ETH_IP=$(ifconfig eth0 | grep "inet " | awk -F'[: ]+' '{ print $4 }')
echo $ETH_IP
export ROS_IP=$ETH_IP

#export PATH=$PATH:/opt/vc/bin:/opt/vc/sbin
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/opt/vc/lib
source /home/$USER/pandora_ws/devel/setup.bash --extend


#source /home/$USER/scripts/launch_config_params.sh
#sleep 10

exec "$@"
