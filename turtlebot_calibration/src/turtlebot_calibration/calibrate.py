#! /usr/bin/python
#***********************************************************
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of the Willow Garage nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#

# Author: Wim Meeussen

from __future__ import with_statement

import roslib; roslib.load_manifest('turtlebot_calibration')
import yaml
import PyKDL
import rospy
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from turtlebot_calibration.msg import ScanAngle
from math import *
import threading


def quat_to_angle(quat):
    rot = PyKDL.Rotation.Quaternion(quat.x, quat.y, quat.z, quat.w)
    return rot.GetRPY()[2]
        


class ScanToAngle:
    def __init__(self):
        self.lock = threading.Lock()
        self.sub_imu  = rospy.Subscriber('imu', Imu, self.imu_cb)
        self.sub_odom = rospy.Subscriber('odom', Odometry, self.odom_cb)
        self.sub_scan = rospy.Subscriber('scan_angle', ScanAngle, self.scan_cb)
        self.cmd_pub = rospy.Publisher('cmd_vel', Twist)
        self.imu_time = rospy.Time()
        self.odom_time = rospy.Time()
        self.scan_time = rospy.Time()
        
        # params
        self.inital_wall_angle = rospy.get_param("inital_wall_angle", 0.1)
        self.imu_calibrate_time = rospy.get_param("imu_calibrate_time", 10.0)
        

    def calibrate(self, speed):
        # wait for all sensor to start
        rospy.loginfo('Waiting to receive imu, odometry and scan angle.')
        started = False
        self.wait_for(rospy.Time.now())
        
        # estimate imu drift
        rospy.loginfo('Estimating imu drift')
        with self.lock:
            imu_start_time = self.imu_time
            imu_start_angle = self.imu_angle
        rospy.sleep(self.imu_calibrate_time)
        with self.lock:
            imu_end_time = self.imu_time
            imu_end_angle = self.imu_angle
        imu_drift = (imu_end_angle - imu_start_angle) / ((imu_end_time - imu_start_time).to_sec())
        rospy.loginfo(' ... imu drift is %f degrees per second'%(imu_drift*180.0/pi))


        # rotate 360 degrees
        (imu_start_angle, odom_start_angle, scan_start_angle) = self.wait_for(rospy.Time.now())
        last_angle = odom_start_angle
        turn_angle = 0
        while turn_angle < 2*pi:
            if rospy.is_shutdown():
                return
            cmd = Twist()
            cmd.angular.z = speed
            self.cmd_pub.publish(cmd)
            rospy.sleep(0.1)
            with self.lock:
                delta_angle = self.odom_angle - last_angle
            if delta_angle < 0:
                delta_angle += 2*pi
            turn_angle += delta_angle
            last_angle = self.odom_angle
        self.cmd_pub.publish(Twist())

        (imu_end_angle, odom_end_angle, scan_end_angle) = self.wait_for(rospy.Time.now())
        imu_delta = 2*pi + (imu_end_angle - imu_start_angle)
        odom_delta = 2*pi + (odom_end_angle - odom_start_angle)
        scan_delta = 2*pi + (scan_end_angle - scan_start_angle)

        print 'Imu correction: %f'%(imu_delta/scan_delta)
        print 'Odom correction: %f'%(odom_delta/scan_delta) 





    def align(self):
        rospy.loginfo("Aligning base with wall")
        with self.lock:
            angle = self.scan_angle
        cmd = Twist()

        while angle < -self.inital_wall_angle or angle > self.inital_wall_angle:
            if angle > 0:
                cmd.angular.z = -0.3
            else:
                cmd.angular.z = 0.3
            self.cmd_pub.publish(cmd)
            rospy.sleep(0.05)
            with self.lock:
                angle = self.scan_angle





    def wait_for(self, start_time):
        while not rospy.is_shutdown():
            rospy.sleep(0.3)
            with self.lock:
                if self.imu_time < start_time:
                    rospy.loginfo("Still waiting for imu")
                elif self.odom_time < start_time:
                    rospy.loginfo("Still waiting for odom")
                elif self.scan_time < start_time:
                    rospy.loginfo("Still waiting for scan")
                else:
                    return (self.imu_angle, self.odom_angle, self.scan_angle)
        exit(0)
        

    def imu_cb(self, msg):
        with self.lock:
            angle = quat_to_angle(msg.orientation)
            self.imu_angle = angle
            self.imu_time = msg.header.stamp

    def odom_cb(self, msg):
        with self.lock:
            angle = quat_to_angle(msg.pose.pose.orientation)
            self.odom_angle = angle
            self.odom_time = msg.header.stamp

    def scan_cb(self, msg):
        with self.lock:
            angle = msg.scan_angle
            self.scan_angle = angle
            self.scan_time = msg.header.stamp



def main():
    rospy.init_node('scan_to_angle')
    robot = ScanToAngle()
    
    robot.wait_for(rospy.Time.now())
    for speed in (0.3, 0.7, 1.0, 1.5):
        robot.align()
        robot.calibrate(speed)


if __name__ == '__main__':
    main()
