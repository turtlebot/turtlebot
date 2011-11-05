#!/bin/bash

# Run this as root, from the directory containing it!
#
# USAGE: sudo ./install.bash
#  or
# sudo ./install.bash usb0
#
# where usb0 is whatever network interface you want to set the robot
# up for.  wlan0 is the default.

interface=wlan0

if [ $# -gt 0 ]; then
    if [ "$1" != "" ]; then
        interface=$1
    fi
fi

echo "Installing using network interface $interface."

sed "s/wlan0/$interface/g" < turtlebot-start > /usr/sbin/turtlebot-start
chmod +x /usr/sbin/turtlebot-start
sed "s/wlan0/$interface/g" < turtlebot-stop > /usr/sbin/turtlebot-stop
chmod +x /usr/sbin/turtlebot-stop
sed "s/wlan0/$interface/g" < turtlebot.conf > /etc/init/turtlebot.conf

# Copy files into /etc/ros/electric/turtlebot
mkdir -p /etc/ros
mkdir -p /etc/ros/electric
cat turtlebot.launch > /etc/ros/electric/turtlebot.launch

echo '. /opt/ros/electric/setup.bash; export ROS_PACKAGE_PATH=/home/turtlebot/ros:${ROS_PACKAGE_PATH}' > /etc/ros/setup.bash

