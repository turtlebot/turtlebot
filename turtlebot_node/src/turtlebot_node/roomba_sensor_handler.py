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

import struct
import logging
import time
import math

import rospy
import turtlebot_driver

import std_msgs.msg as std_msgs

import robot_types

class RoombaSensorHandler(object):

  ROOMBA_PULSES_TO_M = 0.000445558279992234
    
  def __init__(self, robot):
    self._robot = robot
    self._sensor_state_struct = struct.Struct(">12B2hBHhb7HBH5B4h2HB6H2B4hb")
    self._last_encoder_counts = None

  def _request_packet(self, packet_id):
    """Reqeust a sensor packet."""
    with self._robot.sci.lock:
      self._robot.sci.flush_input()
      self._robot.sci.sensors(packet_id)
      #kwc: there appears to be a 10-20ms latency between sending the
      #sensor request and fully reading the packet.  Based on
      #observation, we are preferring the 'before' stamp rather than
      #after.
      stamp = time.time()
      length = turtlebot_driver.SENSOR_GROUP_PACKET_LENGTHS[packet_id]
      return self._robot.sci.read(length), stamp

  def _deserialize(self, buffer, timestamp):
    try:
      (self.bumps_wheeldrops,
       self.wall,
       self.cliff_left, self.cliff_front_left, self.cliff_front_right, self.cliff_right,
       self.virtual_wall,
       self.motor_overcurrents,
       self.dirt_detector_left, self.dirt_detector_right,
       self.remote_opcode, self.buttons,
       self.distance, self.angle,
       self.charging_state,
       self.voltage, self.current, self.temperature, self.charge, self.capacity,
       self.wall_signal, self.cliff_left_signal, self.cliff_front_left_signal,
       self.cliff_front_right_signal, self.cliff_right_signal,
       self.user_digital_inputs, self.user_analog_input,
       self.charging_sources_available,
       self.oi_mode,
       self.song_number, self.song_playing,
       self.number_of_stream_packets,
       self.requested_velocity, self.requested_radius,
       self.requested_right_velocity, self.requested_left_velocity,
       self.encoder_counts_left, self.encoder_counts_right,
       self.light_bumper,
       self.light_bump_left, self.light_bump_front_left, self.light_bump_center_left,
       self.light_bump_center_right, self.light_bump_front_right, self.light_bump_right,
       self.ir_opcode_left, self.ir_opcode_right,
       self.left_motor_current, self.right_motor_current,
       self.main_brish_current, self.side_brush_current,
       self.statis, ) = self._sensor_state_struct.unpack(buffer[0:80])
    except struct.error, e:
      raise roslib.message.DeserializationError(e)

    self.wall = bool(self.wall)
    self.cliff_left = bool(self.cliff_left)
    self.cliff_front_left = bool(self.cliff_front_left)
    self.cliff_front_right = bool(self.cliff_front_right)
    self.cliff_right = bool(self.cliff_right)
    self.virtual_wall = bool(self.virtual_wall)
    self.song_playing = bool(self.song_playing)
 
    # do unit conversions
    self.header = std_msgs.Header(stamp=rospy.Time.from_seconds(timestamp))
    self.requested_velocity = float(self.requested_velocity) / 1000.
    self.requested_radius = float(self.requested_radius) / 1000.
    self.requested_right_velocity = float(self.requested_right_velocity) / 1000.
    self.requested_left_velocity = float(self.requested_left_velocity) / 1000.

    # The distance and angle calculation sent by the robot seems to
    # be really bad. Re-calculate the values using the raw enconder
    # counts.
    if self._last_encoder_counts:
      count_delta_left = self._normalize_encoder_count(
          self.encoder_counts_left - self._last_encoder_counts[0], 0xffff)
      count_delta_right = self._normalize_encoder_count(
          self.encoder_counts_right - self._last_encoder_counts[1], 0xffff)
      distance_left = count_delta_left * self.ROOMBA_PULSES_TO_M
      distance_right = count_delta_right * self.ROOMBA_PULSES_TO_M
      self.distance = (distance_left + distance_right) / 2.0
      self.angle = (distance_right - distance_left) / robot_types.ROBOT_TYPES['roomba'].wheel_separation
    else:
      self.disance = 0
      self.angle = 0
    self._last_encoder_counts = (self.encoder_counts_left, self.encoder_counts_right)

  def _normalize_encoder_count(self, count_delta, maximal_count):
    if count_delta >= maximal_count / 2:
      return count_delta - maximal_count + 1
    elif count_delta <= -maximal_count / 2:
      return count_delta + maximal_count + 1
    return count_delta

  def _encode_message(self, message):
    message.header = self.header
    message.bumps_wheeldrops = self.bumps_wheeldrops
    message.wall = self.wall
    message.cliff_left = self.cliff_left
    message.cliff_front_left = self.cliff_front_left
    message.cliff_front_right = self.cliff_front_right
    message.cliff_right = self.cliff_right
    message.virtual_wall = self.virtual_wall
    message.motor_overcurrents = self.motor_overcurrents
    message.dirt_detector_left = self.dirt_detector_left
    message.dirt_detector_right = self.dirt_detector_right
    message.remote_opcode = self.remote_opcode
    message.buttons = self.buttons
    message.distance = self.distance
    message.angle = self.angle
    message.charging_state = self.charging_state
    message.voltage = self.voltage
    message.current = self.current
    message.temperature = self.temperature
    message.charge = self.charge
    message.capacity = self.capacity
    message.wall_signal = self.wall_signal
    message.cliff_left_signal = self.cliff_left_signal
    message.cliff_front_left_signal = self.cliff_front_left_signal
    message.cliff_front_right_signal = self.cliff_front_right_signal
    message.cliff_right_signal, = self.cliff_right_signal,
    message.user_digital_inputs = self.user_digital_inputs
    message.user_analog_input, = self.user_analog_input,
    message.charging_sources_available = self.charging_sources_available
    message.oi_mode = self.oi_mode
    message.song_number = self.song_number
    message.song_playing = self.song_playing
    message.number_of_stream_packets, = self.number_of_stream_packets,
    message.requested_velocity = self.requested_velocity
    message.requested_radius, = self.requested_radius,
    message.requested_right_velocity = self.requested_right_velocity
    message.requested_left_velocity = self.requested_left_velocity
    return message
    
  def get_all(self, sensor_state):
    buff, timestamp = self._request_packet(100)
    if buff is not None:
      self._deserialize(buff, timestamp)
    self._encode_message(sensor_state)
