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

import roslib.message
import struct
import logging
import time
import rospy

from math import radians
from turtlebot_driver import SENSOR_GROUP_PACKET_LENGTHS

#_struct_I = roslib.message.struct_I
_struct_BI = struct.Struct(">BI")
_struct_12B2hBHhb7HBH5B4h = struct.Struct(">12B2hBHhb7HBH5B4h")

def deserialize(msg, buff, timestamp):
    """
    unpack serialized message in str into this message instance
    @param buff: byte array of serialized message
    @type  buff: str
    """
    try:
        _x = msg
        (_x.bumps_wheeldrops, _x.wall, _x.cliff_left, _x.cliff_front_left, _x.cliff_front_right, _x.cliff_right, _x.virtual_wall, _x.motor_overcurrents, _x.dirt_detector_left, _x.dirt_detector_right, _x.remote_opcode, _x.buttons, _x.distance, _x.angle, _x.charging_state, _x.voltage, _x.current, _x.temperature, _x.charge, _x.capacity, _x.wall_signal, _x.cliff_left_signal, _x.cliff_front_left_signal, _x.cliff_front_right_signal, _x.cliff_right_signal, _x.user_digital_inputs, _x.user_analog_input, _x.charging_sources_available, _x.oi_mode, _x.song_number, _x.song_playing, _x.number_of_stream_packets, _x.requested_velocity, _x.requested_radius, _x.requested_right_velocity, _x.requested_left_velocity,) = _struct_12B2hBHhb7HBH5B4h.unpack(buff[0:52])

        msg.wall = bool(msg.wall)
        msg.cliff_left = bool(msg.cliff_left)
        msg.cliff_front_left = bool(msg.cliff_front_left)
        msg.cliff_front_right = bool(msg.cliff_front_right)
        msg.cliff_right = bool(msg.cliff_right)
        msg.virtual_wall = bool(msg.virtual_wall)
        msg.song_playing = bool(msg.song_playing)

        # do unit conversions
        msg.angle = radians(msg.angle)
        msg.header.stamp = rospy.Time.from_seconds(timestamp)        
        msg.distance     = float(msg.distance) / 1000.
        
        msg.requested_velocity       = float(msg.requested_velocity) / 1000.
        msg.requested_radius         = float(msg.requested_radius) / 1000.
        msg.requested_right_velocity = float(msg.requested_right_velocity) / 1000.
        msg.requested_left_velocity  = float(msg.requested_left_velocity) / 1000.

        return msg
    except struct.error, e:
      raise roslib.message.DeserializationError(e)

class CreateSensorHandler(object):
    
    def __init__(self, robot):
        self.robot = robot    
    def request_packet(self, packet_id):
        """Reqeust a sensor packet."""
        with self.robot.sci.lock:
            self.robot.sci.flush_input()
            self.robot.sci.sensors(packet_id)
            #kwc: there appears to be a 10-20ms latency between sending the
            #sensor request and fully reading the packet.  Based on
            #observation, we are preferring the 'before' stamp rather than
            #after.
            stamp = time.time()
            length = SENSOR_GROUP_PACKET_LENGTHS[packet_id]
            return self.robot.sci.read(length), stamp

    def get_all(self, sensor_state):
        buff, timestamp = self.request_packet(6)
        if buff is not None:
            deserialize(sensor_state, buff, timestamp)
