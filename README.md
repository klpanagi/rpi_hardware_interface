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
```bash
sysctl net.ipv4.ip_forward
```

2. Enable IP-Forwarding on the fly:
```bash
sysctl -w net.ipv4.ip_forward=1
```

This will allow ip forwarding on the curennt shell session. If u want to enable 
IP-Forwarding permanent, proceed with the following steps.

1. Add the following line in **/etc/sysclt.conf**:
```bash
net.ipv4.ip_forward = 1
```

2. To enable the changes made in sysctl.conf you will need to run the command:
```bash
sysctl -p /etc/sysctl.conf
```

#### POSTROUTING configurations
In order to enable NAT in the kernel, run the following commands:
```bash
sudo iptables -t nat -A POSTROUTING -o wlan0 -j MASQUERADE
```

To make this permanent so you don't have to run the commands after each reboot,
run the following command:
```bash
sudo sh -c "iptables-save > /etc/iptables.ipv4.nat"
```

Now edit the file ***/etc/network/interfaces*** and add the following line to 
the bottom of the file:
```bash
up iptables-restore < /etc/iptables.ipv4.nat
```

These configurations allows raspberry to also connect to the internet provided 
by the host-PC's wlan0 interface.

##### Install pigpio
camera_effector ros package dependency.

- Fetch pigpio sources:

```bash
wget abyz.co.uk/rpi/pigpio/pigpio.zip
```

- Unzip it:

```bash
unzip pigpio.zip
```

- Build:

```bash
cd PIGPIO && make
```

- Install:

```bash
make install
```
