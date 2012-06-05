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
# up for.
# and fuerte is the specified version of ROS to use.
# Default is the latest installed.

interface=$(iwconfig 2>/dev/null | awk '{print $1}' | head -n1)

#stackPath=/opt/ros/fuerte/stacks/turtlebot/turtlebot_bringup/upstart
stackPath=./

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
OLD_DIR=$(pwd)
cd `rospack find turtlebot_bringup`/upstart

# checks if turtlebot user+group exists, if it doesn't, then it creates a turtlebot daemon.

if ! grep "^turtlebot:" /etc/group >/dev/null 2>&1; then
    echo "Group turtlebot does not exist, creating."
    groupadd turtlebot
fi

if ! id -u turtlebot >/dev/null 2>&1; then
    echo "User turtlebot does not exist, creating and adding it to groups turtlebot and sudo."
    useradd -g turtlebot turtlebot
    usermod turtlebot -G sudo
    if [ ! -e /home/turtlebot ]; then
        echo "Turtlebot home directory was not created, creating."
        mkdir /home/turtlebot
        chown turtlebot:turtlebot /home/turtlebot
    fi
fi

cp $stackPath/52-turtlebot.rules /etc/udev/rules.d/

source /opt/ros/$release/setup.bash

echo "Installing using network interface $interface."

sed "s/wlan0/$interface/g" < $stackPath/turtlebot-start | sed "s/electric/$release/"g > /usr/sbin/turtlebot-start
chmod +x /usr/sbin/turtlebot-start
sed "s/wlan0/$interface/g" < $stackPath/turtlebot-stop | sed "s/electric/$release/"g > /usr/sbin/turtlebot-stop
chmod +x /usr/sbin/turtlebot-stop
sed "s/wlan0/$interface/g" < $stackPath/turtlebot.conf > /etc/init/turtlebot.conf

# Copy files into /etc/ros/$release/turtlebot
mkdir -p /etc/ros
mkdir -p /etc/ros/$release
cat $stackPath/turtlebot.launch > /etc/ros/$release/turtlebot.launch

echo ". /opt/ros/$release/setup.bash;" > /etc/ros/setup.bash

cd $OLD_DIR
