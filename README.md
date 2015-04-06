rpi_hardware_interface <image src="https://github.com/klpanagi/Pandora_Wiki/blob/master/random_pngs/914495.jpg">
=====================

Contains packages used on raspberry PI 2 for interfacing with hardware

Connect raspberry PI 2 to a host-PC running ROS 
------------------------------
These instructions allows raspberry to connect to the host PC running ROS through
ROS over ethernet communication.
Both the raspberryPI2 and the host PC must have ros-hydro distro installed locally.
ROS_MASTER is initiated and runs on the remote PC.
The raspberryPI2 device will be configured to expose to ROS_MASTER running 
on the host PC. (Slave)

We assume that ROS_MASTER is initiated on the remote PC, on the wlan interface.
This allows more devices to connect to ROS_MASTER, on the host PC, through wlan
interface **(e.g. ROS_MASTER_URI=https://192.168.0.117:11311)**.

The raspberryPI2 connects to the host PC through an ethernet port (eth interface). 

###Prepare the host-PC
In order to allow devices connect to the ROS_MASTER through either the eth or wlan
interface, we need to provide configurations for ip-forwarding and postrouting.

####IP-Fowrwarding configurations

1. Check if IP-Forwarding is enabled on the host PC:
`sysctl net.ipv4.ip_forward`

2. Enable IP-Forwarding on the fly:
`sysctl -w net.ipv4.ip_forward=1`

This will allow ip forwarding on the curennt shell session. If u want to enable 
IP-Forwarding permanent, proceed with the following steps.

1. Add the following line in **/etc/sysclt.conf**:
`net.ipv4.ip_forward = 1`

2. To enable the changes made in sysctl.conf you will need to run the command:
`sysctl -p /etc/sysctl.conf`

####POSTROUTING configurations
In order to enable NAT in the kernel, run the following commands:
```
sudo iptables -t nat -A POSTROUTING -o wlan0 -j MASQUERADE
```

To make this permanent so you don't have to run the commands after each reboot,
run the following command:
```
sudo sh -c "iptables-save > /etc/iptables.ipv4.nat"
```

Now edit the file ***/etc/network/interfaces*** and add the following line to 
the bottom of the file:
```
up iptables-restore < /etc/iptables.ipv4.nat
```

These configurations allows raspberry to also connect to the internet provided 
by the host-PC's wlan0 interface.

####Configure ROS_MASTER to run on wlan0 interface - Host PC
Source the **pc_ros_network_config.sh** bash script, located in the scripts/ directory,
in order to configure the ROS_MASTER_URI and ROS_IP env-variables.
```bash
source pc_ros_network_config.sh
```

The current script automatically finds the pc's IP address on wlan0 interface
and applies it to the ROS_MASTER_URI and ROS_IP environmental variables.

###ROS over ethernet configurations on raspberry-PI2 
Source the **rpi_ros_network_config.sh** bash script, located in the scripts/ directory, 
with the host-PC's IP-address on wlan0 interface as input parameter.
For example, if the host-PC has the IP-address, **192.168.0.117**, for the wlan0 
interface, then run the following command on the raspberry-PI2 device:
```
source rpi_ros_network_config 192.168.0.117
```
