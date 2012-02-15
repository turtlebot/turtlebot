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
import rospy
from sensor_msgs.msg import LaserScan
from turtlebot_calibration.msg import ScanAngle
from math import *


class ScanToAngle:
    def __init__(self):
        self.min_angle = rospy.get_param('min_angle', -0.3)
        self.max_angle = rospy.get_param('max_angle', 0.3)
        self.pub = rospy.Publisher('scan_angle', ScanAngle)
        self.sub = rospy.Subscriber('scan', LaserScan, self.scan_cb)


    def scan_cb(self, msg):
        angle = msg.angle_min
        d_angle = msg.angle_increment
        sum_x = 0
        sum_y = 0
        sum_xx = 0
        sum_xy = 0
        num = 0
        for r in msg.ranges:
            if angle > self.min_angle and angle < self.max_angle and r < msg.range_max:
                x = sin(angle) * r
                y = cos(angle) * r
                sum_x += x
                sum_y += y
                sum_xx += x*x
                sum_xy += x*y
                num += 1
            angle += d_angle
        if num > 0:
            angle=atan2((-sum_x*sum_y+num*sum_xy)/(num*sum_xx-sum_x*sum_x), 1)
            res = ScanAngle()
            res.header = msg.header
            res.scan_angle = angle
            self.pub.publish(res)
        else:
            rospy.logerr("Please point me at a wall.")

def main():
    rospy.init_node('scan_to_angle')
    s = ScanToAngle()
    rospy.spin()


if __name__ == '__main__':
    main()
