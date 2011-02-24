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

#Melonee Wise mwise@willowgarage.com

import roslib
roslib.load_manifest('turtlebot_node')
import rospy
import diagnostic_msgs.msg


class TurtlebotDiagnostics():
    def __init__(self):
        self.charging_state =  {0:"Not Charging",
                                1:"Reconditioning Charging",
                                2:"Full Charging",
                                3:"Trickle Charging",
                                4:"Waiting",
                                5:"Charging Fault Condition"}
        self.charging_source = {0:"None",
                                1:"Internal Charger",
                                2:"Base Dock"}
        self.digital_outputs = {0:"OFF",
                                1:"ON"}
        self.oi_mode = {1:"Passive",
                        2:"Safe",
                        3:"Full"}
        self.diag_pub = rospy.Publisher('/diagnostics', diagnostic_msgs.msg.DiagnosticArray)

    def publish_diagnostics(self, sensor_state, cal_offset, has_gyro, cal_buffer):
        diag = diagnostic_msgs.msg.DiagnosticArray()
        diag.header.stamp = sensor_state.header.stamp
        #mode info
        stat = diagnostic_msgs.msg.DiagnosticStatus()
        stat.name = "Operating Mode"
        stat.level = diagnostic_msgs.msg.DiagnosticStatus.OK
        stat.message = self.oi_mode[sensor_state.oi_mode]
        diag.status.append(stat)
        #battery info
        stat = diagnostic_msgs.msg.DiagnosticStatus()
        stat.name = "Battery"
        stat.level = diagnostic_msgs.msg.DiagnosticStatus.OK
        stat.message = "OK"
        if sensor_state.charging_state == 5:
            stat.level = diagnostic_msgs.msg.DiagnosticStatus.ERROR
            stat.message = "Charging Fault Condition"
            stat.values.append(diagnostic_msgs.msg.KeyValue("Charging State", self.charging_state[sensor_state.charging_state]))
        stat.values.append(diagnostic_msgs.msg.KeyValue("Voltage (V)", str(sensor_state.voltage/1000.0)))
        stat.values.append(diagnostic_msgs.msg.KeyValue("Current (A)", str(sensor_state.current/1000.0)))
        stat.values.append(diagnostic_msgs.msg.KeyValue("Temperature (C)",str(sensor_state.temperature)))
        stat.values.append(diagnostic_msgs.msg.KeyValue("Charge (Ah)", str(sensor_state.charge/1000.0)))
        stat.values.append(diagnostic_msgs.msg.KeyValue("Capacity (Ah)", str(sensor_state.capacity/1000.0)))
        diag.status.append(stat)
        #charging source
        stat = diagnostic_msgs.msg.DiagnosticStatus()
        stat.name = "Charging Sources"
        stat.level = diagnostic_msgs.msg.DiagnosticStatus.OK
        stat.message = self.charging_source[sensor_state.charging_sources_available]
        diag.status.append(stat)
        #cliff sensors
        stat = diagnostic_msgs.msg.DiagnosticStatus()
        stat.name = "Cliff Sensor"
        stat.level = diagnostic_msgs.msg.DiagnosticStatus.OK
        stat.message = "OK"
        if sensor_state.cliff_left or sensor_state.cliff_front_left or sensor_state.cliff_right or sensor_state.cliff_front_right:
            stat.level = diagnostic_msgs.msg.DiagnosticStatus.ERROR
            stat.message = "Near Cliff"
        stat.values.append(diagnostic_msgs.msg.KeyValue("Left", str(sensor_state.cliff_left)))
        stat.values.append(diagnostic_msgs.msg.KeyValue("Left Signal", str(sensor_state.cliff_left_signal)))
        stat.values.append(diagnostic_msgs.msg.KeyValue("Front Left", str(sensor_state.cliff_front_left)))
        stat.values.append(diagnostic_msgs.msg.KeyValue("Front Left Signal", str(sensor_state.cliff_front_left_signal)))
        stat.values.append(diagnostic_msgs.msg.KeyValue("Front Right", str(sensor_state.cliff_right)))
        stat.values.append(diagnostic_msgs.msg.KeyValue("Front Right Signal", str(sensor_state.cliff_right_signal)))
        stat.values.append(diagnostic_msgs.msg.KeyValue("Right", str(sensor_state.cliff_front_right)))
        stat.values.append(diagnostic_msgs.msg.KeyValue("Right Signal", str(sensor_state.cliff_front_right_signal)))
        diag.status.append(stat)
        #Wall sensors
        stat = diagnostic_msgs.msg.DiagnosticStatus()
        stat.name = "Wall Sensor"
        stat.level = diagnostic_msgs.msg.DiagnosticStatus.OK
        stat.message = "OK"
        #wall always seems to be false??? 
        if sensor_state.wall:
            stat.level = diagnostic_msgs.msg.DiagnosticStatus.ERROR
            stat.message = "Near Wall"
        stat.values.append(diagnostic_msgs.msg.KeyValue("Wall", str(sensor_state.wall)))
        stat.values.append(diagnostic_msgs.msg.KeyValue("Wall Signal", str(sensor_state.wall_signal)))
        stat.values.append(diagnostic_msgs.msg.KeyValue("Virtual Wall", str(sensor_state.virtual_wall)))
        diag.status.append(stat)
        #Gyro
        stat = diagnostic_msgs.msg.DiagnosticStatus()
        stat.name = "Gyro Sensor"
        stat.level = diagnostic_msgs.msg.DiagnosticStatus.OK
        stat.message = "OK"
        if not has_gyro:
            stat.level = diagnostic_msgs.msg.DiagnosticStatus.WARN
            stat.message = "Gyro Not Enabled"
        elif cal_offset > 520.0 or cal_offset < 480.0:
            stat.level = diagnostic_msgs.msg.DiagnosticStatus.ERROR
            stat.message = "Bad Gyro Calibration"
        stat.values.append(diagnostic_msgs.msg.KeyValue("Gyro Enabled", str(has_gyro)))
        stat.values.append(diagnostic_msgs.msg.KeyValue("Raw Gyro Rate", str(sensor_state.user_analog_input)))
        stat.values.append(diagnostic_msgs.msg.KeyValue("Calibration Offset", str(cal_offset)))
        stat.values.append(diagnostic_msgs.msg.KeyValue("Calibration Buffer", str(cal_buffer)))
        diag.status.append(stat)
        #Digital IO
        stat = diagnostic_msgs.msg.DiagnosticStatus()
        stat.name = "Digital Outputs"
        stat.level = diagnostic_msgs.msg.DiagnosticStatus.OK
        stat.message = "OK"
        out_byte = sensor_state.user_digital_outputs
        stat.values.append(diagnostic_msgs.msg.KeyValue("Raw Byte", str(out_byte)))
        stat.values.append(diagnostic_msgs.msg.KeyValue("Digital Out 2", self.digital_outputs[out_byte%2]))
        out_byte = out_byte >>1
        stat.values.append(diagnostic_msgs.msg.KeyValue("Digital Out 1", self.digital_outputs[out_byte%2]))
        out_byte = out_byte >>1
        stat.values.append(diagnostic_msgs.msg.KeyValue("Digital Out 0", self.digital_outputs[out_byte%2]))
        diag.status.append(stat)
        #publish
        self.diag_pub.publish(diag)

