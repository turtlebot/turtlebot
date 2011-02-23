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

#Melonee Wise mwise@willowgarage.com

import roslib
roslib.load_manifest('create_node')
import rospy
import math
import copy
import sensor_msgs.msg
import PyKDL

class TurtlebotGyro():
    def __init__(self):
        self.cal_offset = 0.0
        self.orientation = 0.0
        self.cal_buffer =[]
        self.imu_data = sensor_msgs.msg.Imu()
        self.imu_data = sensor_msgs.msg.Imu(header=rospy.Header(frame_id="gyro_link"))
        self.imu_data.orientation_covariance = [1e6, 0, 0, 0, 1e6, 0, 0, 0, 1e-6]
        self.imu_data.angular_velocity_covariance = [1e6, 0, 0, 0, 1e6, 0, 0, 0, 1e-6]
        self.imu_data.linear_acceleration_covariance = [-1,0,0,0,0,0,0,0,0]

        self.imu_pub = rospy.Publisher('imu/data', sensor_msgs.msg.Imu)

    def compute_cal_offset(self):
        self.cal_offset = sum(self.cal_buffer)/len(self.cal_buffer)
        return self.cal_offset, self.cal_buffer

    def update_calibration(self, sensor_state):
        self.cal_buffer.append(sensor_state.user_analog_input)
        if len(self.cal_buffer) > 100:
            del self.cal_buffer[:-100]
        
    def publish_imu_data(self, sensor_state, last_time):
        current_time = sensor_state.header.stamp
        dt = (current_time - last_time).to_sec()
        self.imu_data.header.stamp =  sensor_state.header.stamp
        self.imu_data.angular_velocity.z  = (float(sensor_state.user_analog_input)-self.cal_offset)/self.cal_offset*150.0*(math.pi/180.0)*1.35
        #sign change
        self.imu_data.angular_velocity.z = -1.0*self.imu_data.angular_velocity.z
        if(sensor_state.requested_right_velocity == 0 and sensor_state.requested_left_velocity == 0 and sensor_state.distance ==0):
            self.imu_data.angular_velocity.z  = 0.0
        self.orientation += self.imu_data.angular_velocity.z * dt
        #print orientation
        (self.imu_data.orientation.x, self.imu_data.orientation.y, self.imu_data.orientation.z, self.imu_data.orientation.w) = PyKDL.Rotation.RotZ(self.orientation).GetQuaternion()
        self.imu_pub.publish(self.imu_data)
