#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2011, Willow Garage, Inc.
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
#  * Neither the name of Willow Garage, Inc. nor the names of its
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
# Revision $Id: roslogging.py 11745 2010-10-25 02:23:13Z kwc $
# $Author: kwc $

"""
Simplified interface for controlling a mobile robot base.
"""

import roslib; roslib.load_manifest('turtle_drive')

import math
import actionlib

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from turtle_drive.msg import TurnAction, TurnFeedback, TurnResult

M_PI = math.pi

def normalize_angle_positive(angle):
    return (angle % (2.0*M_PI) + 2.0*M_PI) % (2.0*M_PI)

# utilities for calculating angles
def normalize_angle(angle):
    a = normalize_angle_positive(angle)
    if a > M_PI:
        a = a- 2.0 * M_PI
    return a
    
def shortest_angular_distance(from_angle, to_angle):
    result = normalize_angle_positive(normalize_angle_positive(to_angle) - normalize_angle_positive(from_angle))
	
    if result > M_PI:
      # If the result > 180,
      # It's shorter the other way.
      result = -(2.0*M_PI - result)
	
    return normalize_angle(result)

def yaw_delta(q1, q2):
    e1 = tr.euler_from_quaternion([q1.x, q1.y, q1.z, q1.w]);
    e2 = tr.euler_from_quaternion([q1.x, q2.y, q2.z, q2.w]);
    return math.abs(shortest_angular_distance(e1[2], e2[2]))
    
class TurtleDrive(object):

    def __init__(self):
        self.server = actionlib.SimpleActionServer('turn',
                                                   TurnAction,
                                                   execute_cb=self.turn)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist)
        self.state_sub = rospy.Subscriber(create_odom_topic,
                                          Odometry,
                                          self.sensor_cb)
        self.delta_angle = 0
        self.last_orientation = None

    def sensor_cb(self, odom_data):
        if self.last_orientation is None:
            self.delta_angle = 0
        else:
            new_orientation = odom_data.pose.pose.orientation
            self.delta_angle += yaw_delta(self.last_orientation, new_orientation)
            #TODO: remove
            print "da", self.delta_angle
            
        self.last_orientation = new_orientation

    def turn(self, goal):
        try:
            goal_radians = goal.radians
            if goal.clockwise:
                twist_turn = Twist(angular=Vector3(z=1.))
            else:
                twist_turn = Twist(angular=Vector3(z=-1.))

            server = self.server
            self.delta_angle = 0

            feedback = TurnFeedback()
            r = rospy.Rate(10)
            while not stop:
                delta_angle = float(abs(last_capture - curr_angle))
                # First: check for stop conditions
                if server.is_preempt_requested():
                    rospy.loginfo('%s: Preempted' % self._action_name)
                    server.set_preempted()
                    stop = True
                elif total_angle >= goal_radians or rospy.is_shutdown():
                    stop = True
                else:
                    feedback.radians_completed = math.abs(self.delta_angle)
                    self.cmd_vel_pub.publish(create_twist_stop)
                    self.server.publish_feedback(feedback)
                    r.sleep()
                    
            if result:
                self.server.set_succeeded(TurnResult(radians_completed=feedback.radians_completed))

        finally:
            # stop the base
            self.cmd_vel_pub.publish(twist_stop)
