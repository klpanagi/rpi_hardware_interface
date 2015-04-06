rpi_hardware_interface <image src="https://github.com/klpanagi/Pandora_Wiki/blob/master/random_pngs/914495.jpg">
=====================

Contains packages used on raspberry PI 2 for interfacing with hardware

Connect raspberry PI 2 to a PC 
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

###Prepare remote PC
In order to allow devices connect to the ROS_MASTER through either the eth or wlan
interface, we need to provide configurations for ip-forwarding and postrouting.

1. Check if IP-Forwarding is enabled on the host PC:
`sysctl net.ipv4.ip_forward`

2. Enable IP-Forwarding on the fly:
`sysctl -w net.ipv4.ip_forward=1`

This will allow ip forwarding on the curennt shell session. If u want to enable 
IP-Forwarding permanent, proceed to step 3.

3. Add the following line in **/etc/sysclt.conf**:
`net.ipv4.ip_forward = 1`

4. To enable the changes made in sysctl.conf you will need to run the command:
`sysctl -p /etc/sysctl.conf`
