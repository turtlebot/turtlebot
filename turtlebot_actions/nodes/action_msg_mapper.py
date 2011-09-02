#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2010, Willow Garage, Inc.
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

# Melonee Wise <mwise@willowgarage.com>

import roslib; roslib.load_manifest('turtlebot_actions')

import rospy
import string
import rosgraph.masterapi
import actionlib
import turtlebot_actions.msg 

class ActionMsgMapper(object): 
    _result   = turtlebot_actions.msg.ActionMsgMapperResult()

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, turtlebot_actions.msg.ActionMsgMapperAction, execute_cb=self.init_cb)

    
    def init_cb(self, goal):
        self.goal = goal

        self.input_sub = rospy.Subscriber(goal.input_topic, roslib.message.get_message_class(goal.input_type), self.input_cb)
        self.output_pub = rospy.Publisher(goal.output_topic, roslib.message.get_message_class(goal.output_type))

        if len(self.goal.input_list)!=len(self.goal.input_list):
            rospy.logerror('%s: remap lists not the smae length' % self._action_name)
            self._as.set_aborted(self._result)

        self._as.set_succeeded()

    def input_cb(self, input_msg):

        output_msg = roslib.message.get_message_class(self.goal.output_type)()

        for i in range(len(self.goal.input_list)):
            #get the input attribute by steping down through the attributes   
            in_attributes = string.split(self.goal.input_list[i],'.')
            in_att = getattr(input_msg, in_attributes[0])
            if len(in_attributes)>1:
                for a in in_attributes[1:]:
                    in_att = getattr(in_att,a)
            #get the output attribute by steping down through the attributes
            out_attributes = string.split(self.goal.output_list[i],'.')
            out_att = getattr(output_msg, out_attributes[0])
            if len(out_attributes)>1:
                for a in out_attributes[1:len(out_attributes)-1]:
                    out_att = getattr(out_att,a)
                setattr(out_att, out_attributes[len(out_attributes)-1], in_att)
            else:
                setattr(output_msg, out_attributes[0], in_att)
        self.output_pub.publish(output_msg)    


if __name__ == '__main__':
    rospy.init_node('remapper')
    ActionMsgMapper(rospy.get_name())
    rospy.spin()

