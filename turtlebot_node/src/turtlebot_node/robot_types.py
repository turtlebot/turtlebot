# Copyright (c) 2011, Lorenz Moesenlechner <moesenle@in.tum.de>
# All rights reserved.
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# 
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Intelligent Autonomous Systems Group/
#       Technische Universitaet Muenchen nor the names of its contributors 
#       may be used to endorse or promote products derived from this software 
#       without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import create_sensor_handler
import roomba_sensor_handler

__author__ = 'moesenle@in.tum.de (Lorenz Moesenlechner)'

class RobotType(object):

  def __init__(self, name, baudrate, sensor_handler, wheel_separation):
    self.name = name
    self.baudrate = baudrate
    self.sensor_handler = sensor_handler
    self.wheel_separation = wheel_separation

    
ROBOT_TYPES = {
    'create': RobotType('create', 57600, create_sensor_handler.CreateSensorHandler,
                        wheel_separation=0.26),
    'roomba': RobotType('roomba', 115200, roomba_sensor_handler.RoombaSensorHandler,
                        wheel_separation=0.235),
    }
