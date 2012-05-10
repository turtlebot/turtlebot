#!/bin/bash

# Run this as root, from the directory containing it!
#
# USAGE: sudo ./install.bash
#  or
# sudo ./install.bash usb0
#  or
# sudo ./install.bash usb0 fuerte
#
# where usb0 is whatever network interface you want to set the robot
# up for.  wlan0 is the default.
# and fuerte is the specified version of ROS to use.
# Default is the latest installed.

interface=wlan0

if [ $# -gt 0 ]; then
    if [ "$1" != "" ]; then
        interface=$1
    fi
fi

release=$(ls /opt/ros/ | tail -n1)

if [ $# -gt 1 ]; then
    if [ "$2" != "" ]; then
        release=$2
    fi
fi

source /opt/ros/$release/setup.bash

echo "Installing using network interface $interface."

sed "s/wlan0/$interface/g" < turtlebot-start | sed "s/electric/$release/"g > /usr/sbin/turtlebot-start
chmod +x /usr/sbin/turtlebot-start
sed "s/wlan0/$interface/g" < turtlebot-stop | sed "s/electric/$release/"g > /usr/sbin/turtlebot-stop
chmod +x /usr/sbin/turtlebot-stop
sed "s/wlan0/$interface/g" < turtlebot.conf > /etc/init/turtlebot.conf

# Copy files into /etc/ros/$release/turtlebot
mkdir -p /etc/ros
mkdir -p /etc/ros/$release
cat turtlebot.launch > /etc/ros/$release/turtlebot.launch

echo ". /opt/ros/$release/setup.bash; export ROS_PACKAGE_PATH=/home/turtlebot/ros:${ROS_PACKAGE_PATH}" > /etc/ros/setup.bash

