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
#
# Revision $Id: __init__.py 11217 2010-09-23 21:08:11Z kwc $

import roslib; roslib.load_manifest('turtlebot_node')

"""
ROS Turtlebot node for ROS built on top of turtlebot_driver.
This driver is based on otl_roomba by OTL (otl-ros-pkg).

turtlebot_driver is based on Damon Kohler's pyrobot.py. 
"""

import os
import sys
import select
import threading
import serial
import termios

from math import sin, cos, radians, pi

from turtlebot_driver import Turtlebot, WHEEL_SEPARATION, MAX_WHEEL_SPEED, DriverError
from turtlebot_node.msg import TurtlebotSensorState, Drive, Turtle
from turtlebot_node.srv import SetTurtlebotMode,SetTurtlebotModeResponse, SetDigitalOutputs, SetDigitalOutputsResponse
from turtlebot_node.diagnostics import TurtlebotDiagnostics
from turtlebot_node.gyro import TurtlebotGyro
import rospy

from geometry_msgs.msg import Point, Pose, Pose2D, PoseWithCovariance, \
    Quaternion, Twist, TwistWithCovariance, Vector3
from nav_msgs.msg import Odometry 
from tf.broadcaster import TransformBroadcaster


class TurtlebotNode(object):

    def __init__(self, default_port='/dev/ttyUSB0'):
        """
        @param default_port: default tty port to use for establishing
            connection to Turtlebot.  This will be overriden by ~port ROS
            param if available.
        """
        rospy.init_node('turtlebot')

        self.port = rospy.get_param('~port', default_port)
        rospy.loginfo("serial port: %s"%(self.port))
        # determine cmd_vel mode
        self.drive_mode = rospy.get_param('~drive_mode', 'twist')
        rospy.loginfo("drive mode: %s"%(self.drive_mode))

        self.has_gyro = rospy.get_param('~has_gyro', True)
        rospy.loginfo("has gyro: %s"%(self.has_gyro))

        self.odom_angular_scale_correction = rospy.get_param('~odom_angular_scale_correction', 1.0)
        self.odom_linear_scale_correction = rospy.get_param('~odom_linear_scale_correction', 1.0)
        self.cmd_vel_timeout = rospy.Duration(rospy.get_param('~cmd_vel_timeout', 0.6))
        self.stop_motors_on_bump = rospy.get_param('~stop_motors_on_bump', False)
        self.min_abs_yaw_vel = rospy.get_param('~min_abs_yaw_vel', None)
        

        self.lock  = threading.RLock()
        self.robot = None
        while not rospy.is_shutdown():
            try:
                self.robot = Turtlebot(self.port)
                break
            except serial.serialutil.SerialException, ex:
                rospy.logerr("Failed to open port %s.  Please make sure the Create cable is plugged into the computer. "%self.port)
                rospy.sleep(3.0)
        self.robot.safe = False

        if rospy.get_param('~bonus', False):
            bonus(self.robot)

        self.robot.control()

        self.sensor_state_pub = rospy.Publisher('~sensor_state', TurtlebotSensorState)
        self.operating_mode_srv = rospy.Service('~set_operation_mode', SetTurtlebotMode, self.set_operation_mode)
        self.digital_output_srv = rospy.Service('~set_digital_outputs', SetDigitalOutputs, self.set_digital_outputs)
        if self.drive_mode == 'twist':
            self.cmd_vel_sub = rospy.Subscriber('~cmd_vel', Twist, self.cmd_vel)
            self.drive_cmd = self.robot.direct_drive            
        elif self.drive_mode == 'drive':
            self.cmd_vel_sub = rospy.Subscriber('~cmd_vel', Drive, self.cmd_vel)
            self.drive_cmd = self.robot.drive 
        elif self.drive_mode == 'turtle':
            self.cmd_vel_sub = rospy.Subscriber('~cmd_vel', Turtle, self.cmd_vel)
            self.drive_cmd = self.robot.direct_drive
        else:
            rospy.logerr("unknown drive mode :%s"%(self.drive_mode))
        
        if(self.has_gyro):    
            self._gyro = TurtlebotGyro()
            
        self.odom_broadcaster = TransformBroadcaster()
        self.odom_pub = rospy.Publisher('odom', Odometry)
        self._diagnostics = TurtlebotDiagnostics()

 
        self.req_cmd_vel = None
        self.cal_offset = 0
        self.cal_buffer = []
        # TODO: optionally publish raw sensor state 
        # TODO: roomba backend
        # TODO: songs, led
        # TODO: reset
        # TODO: stop/brake

    def cmd_vel(self, msg):
        # Clamp to min abs yaw velocity, to avoid trying to rotate at low
        # speeds, which doesn't work well.
        if self.min_abs_yaw_vel is not None and msg.angular.z != 0.0 and abs(msg.angular.z) < self.min_abs_yaw_vel:
            msg.angular.z = self.min_abs_yaw_vel if msg.angular.z > 0.0 else -self.min_abs_yaw_vel
        if self.drive_mode == 'twist':
            # convert twist to direct_drive args
            ts  = msg.linear.x * 1000 # m -> mm
            # TODO: pyrobot says 298mm, otl has 225mm
            tw  = msg.angular.z  * (WHEEL_SEPARATION / 2)
            # Prevent saturation at max wheel speed when a compound command is sent.
            if ts > 0:
                ts = min(ts, MAX_WHEEL_SPEED - tw)
            else:
                ts = max(ts, -MAX_WHEEL_SPEED + tw)
            self.req_cmd_vel = int(ts - tw), int(ts + tw)
        elif self.drive_mode == 'turtle':
            # convert to direct_drive args
            ts  = msg.linear * 1000 # m -> mm
            tw  = msg.angular  * (WHEEL_SEPARATION / 2)
            self.req_cmd_vel = int(ts - tw), int(ts + tw)
        elif self.drive_mode == 'drive':
            # convert twist to drive args, m->mm (velocity, radius)
            self.req_cmd_vel = msg.velocity * 1000, msg.radius * 1000

    def set_operation_mode(self,req):
        if req.mode == 1: #passive
            rospy.logdebug("Setting turtlebot to passive mode.")
            #setting all the digital outputs to 0
            outputs = [0, 0, 0]
            self.robot.set_digital_outputs(outputs)
            self.robot.passive()
        elif req.mode == 2: #safe
            rospy.logdebug("Setting turtlebot to safe mode.")
            self.robot.safe = True
            self.robot.control()
        elif req.mode == 3: #full
            rospy.logdebug("Setting turtlebot to full mode.")
            self.robot.safe = False
            self.robot.control()
        else:
            rospy.logerr("Requested an invalid mode.")
            return SetTurtlebotModeResponse(False)
        return SetTurtlebotModeResponse(True)

    def set_digital_outputs(self,req):
        outputs = [req.digital_out_0,req.digital_out_1, req.digital_out_2]
        self.robot.set_digital_outputs(outputs)
        return SetDigitalOutputsResponse(True)

    def sense(self, sensor_state):
        self.robot.sensors.get_all() 
        convert_sensor_state(self.robot.sensors, sensor_state)
        
    def spin(self):
        
        # de-self vars
        state_pub = self.sensor_state_pub
        odom_pub = self.odom_pub
        odom_broadcaster = self.odom_broadcaster
        drive_cmd = self.drive_cmd

        # state
        pos2d = Pose2D()
        s = TurtlebotSensorState()
        odom = Odometry(header=rospy.Header(frame_id="odom"), child_frame_id='base_footprint')
        last_cmd_vel = 0, 0
        last_cmd_vel_time = rospy.get_rostime()
        last_diagnostics_time = rospy.get_rostime()

        while not rospy.is_shutdown():
            try:
                if(self.has_gyro):
                    self.sense(s) # make sure to use the port before 
                    rospy.loginfo("Calibrating Gyro. Don't move the robot now")
                    start_time = rospy.Time.now()
                    cal_duration = rospy.Duration(2.0)
                    tmp_time = rospy.Time.now()
                    offset = 0
                    while rospy.Time.now() < start_time + cal_duration:
                        rospy.sleep(0.01)
                        self.sense(s)
                        self._gyro.update_calibration(s) 
                    self.cal_offset, self.cal_buffer = self._gyro.compute_cal_offset()
                    rospy.loginfo("Gyro calibrated with offset %f"%self.cal_offset)  

                # loop rate, not yet tuned
                r = rospy.Rate(10)
                while not rospy.is_shutdown():
                    last_time = s.header.stamp

                    # sense/compute state
                    try:
                        self.sense(s)
                        transform = self.compute_odom(s, pos2d, last_time, odom)
                        #check if we're not moving and update the calibration offset
                        #to account for any calibration drift due to temperature
                        if(self.has_gyro and s.requested_right_velocity == 0 and s.requested_left_velocity == 0 and s.distance == 0):
                            self._gyro.update_calibration(s)
                            self.cal_offset, self.cal_buffer = self._gyro.compute_cal_offset()
                        if(self.has_gyro and self.cal_offset !=0):  
                            self._gyro.publish_imu_data(s, last_time)
                        if (s.header.stamp-last_diagnostics_time).to_sec() > 1.0:
                            last_diagnostics_time = s.header.stamp
                            self._diagnostics.publish_diagnostics(s, self.cal_offset, self.has_gyro, self.cal_buffer)
                    except select.error:
                        # packet read can get interrupted, restart loop to
                        # check for exit conditions
                        continue

                    # publish state
                    state_pub.publish(s)
                    odom_pub.publish(odom)
                    #odom_broadcaster.sendTransform(transform[0], transform[1],
                    #    s.header.stamp, "base_footprint", "odom")

                    # act
                    if self.req_cmd_vel is not None:
                        # consume cmd_vel msg and store for timeout
                        last_cmd_vel = self.req_cmd_vel
                        self.req_cmd_vel = None 
                        last_cmd_vel_time = last_time
                        last_cmd_vel = self.check_bumpers(s, last_cmd_vel)
                        drive_cmd(*last_cmd_vel)
                    else:
                        if last_time - last_cmd_vel_time > self.cmd_vel_timeout:
                            last_cmd_vel = 0, 0
                        last_cmd_vel = self.check_bumpers(s, last_cmd_vel)
                        drive_cmd(*last_cmd_vel)
                    r.sleep()
                self.robot.set_digital_outputs([0, 0, 0])

            except DriverError, ex:
                #self.robot.sci.wake()
                #try:
                #    self.sense(s)
                #    rospy.logdebug("Successfully woke robot")
                #except DriverError, ex:
                rospy.logerr("Failed to contact device with error: [%s]. Please check that the Create is powered on and that the connector is plugged into the Create."%ex)
                rospy.sleep(3.0)
            except termios.error, ex:
                rospy.logfatal("Write to port %s failed.  Did the usb cable become unplugged?"%self.port)
                break

    def check_bumpers(self, s, cmd_vel):
        # Safety: disallow forward motion if bumpers or wheeldrops 
        # are activated.
        # TODO: check bumps_wheeldrops flags more thoroughly, and disable
        # all motion (not just forward motion) when wheeldrops are activated
        forward = (cmd_vel[0] + cmd_vel[1]) > 0
        if self.stop_motors_on_bump and s.bumps_wheeldrops > 0 and forward:
            return (0,0)
        else:
            return cmd_vel


    def compute_odom(self, sensor_state, pos2d, last_time, odom):
        """
        Compute current odometry.  Updates odom instance and returns tf
        transform. compute_odom() does not set frame ids or covariances in
        Odometry instance.  It will only set stamp, pose, and twist.

        @param sensor_state: Current sensor reading
        @type  sensor_state: TurtlebotSensorState
        @param pos2d: Current position
        @type  pos2d: geometry_msgs.msg.Pose2D
        @param last_time: time of last sensor reading
        @type  last_time: rospy.Time
        @param odom: Odometry instance to update. 
        @type  odom: nav_msgs.msg.Odometry

        @return: transform
        @rtype: ( (float, float, float), (float, float, float, float) )
        """
        # based on otl_roomba by OTL <t.ogura@gmail.com>

        current_time = sensor_state.header.stamp
        dt = (current_time - last_time).to_sec()

        # this is really delta_distance, delta_angle
        d  = sensor_state.distance * self.odom_linear_scale_correction #correction factor from calibration
        angle = sensor_state.angle * self.odom_angular_scale_correction #correction factor from calibration

        x = cos(angle) * d
        y = -sin(angle) * d

        last_angle = pos2d.theta
        pos2d.x += cos(last_angle)*x - sin(last_angle)*y
        pos2d.y += sin(last_angle)*x + cos(last_angle)*y
        pos2d.theta += angle

        # Turtlebot quaternion from yaw. simplified version of tf.transformations.quaternion_about_axis
        odom_quat = (0., 0., sin(pos2d.theta/2.), cos(pos2d.theta/2.))
        #rospy.logerr("theta: %f odom_quat %s"%(pos2d.theta, str(odom_quat)))

        # construct the transform
        transform = (pos2d.x, pos2d.y, 0.), odom_quat

        # update the odometry state
        odom.header.stamp = current_time
        odom.pose.pose   = Pose(Point(pos2d.x, pos2d.y, 0.), Quaternion(*odom_quat))
        odom.pose.covariance = [1e-3, 0, 0, 0, 0, 0, 
                                0, 1e-3, 0, 0, 0, 0,
                                0, 0, 1e6, 0, 0, 0,
                                0, 0, 0, 1e6, 0, 0,
                                0, 0, 0, 0, 1e6, 0,
                                0, 0, 0, 0, 0, 1e3]
        odom.twist.twist = Twist(Vector3(d/dt, 0, 0), Vector3(0, 0, angle/dt))
        odom.twist.covariance = [1e-3, 0, 0, 0, 0, 0, 
                                0, 1e-3, 0, 0, 0, 0,
                                0, 0, 1e6, 0, 0, 0,
                                0, 0, 0, 1e6, 0, 0,
                                0, 0, 0, 0, 1e6, 0,
                                0, 0, 0, 0, 0, 1e3]
        if(sensor_state.requested_right_velocity == 0 and sensor_state.requested_left_velocity == 0 and sensor_state.distance ==0):
            odom.pose.covariance[0] = 1e-9
            odom.pose.covariance[8] = 1e-9
            odom.pose.covariance[35] = 1e-9
            odom.twist.covariance[0] = 1e-9
            odom.twist.covariance[8] = 1e-9
            odom.twist.covariance[35] = 1e-9
        # return the transform
        return transform
                
#TODO: this method is temporary. In the future, we should be able to
#direct deserialize into RawTurtlebotSensorState and fix conversions
def convert_sensor_state(robot_sensors, state_msg):
    d = robot_sensors.data
    state_msg.header.stamp = rospy.Time.from_seconds(d['timestamp'])
    
    state_msg.bumps_wheeldrops   = d['bumps-wheeldrops']
    state_msg.wall               = d['wall']
    state_msg.cliff_left         = d['cliff-left']
    state_msg.cliff_front_left   = d['cliff-front-left']
    state_msg.cliff_front_right  = d['cliff-front-right']
    state_msg.cliff_right        = d['cliff-right']
    state_msg.virtual_wall       = d['virtual-wall']
    state_msg.motor_overcurrents = d['motor-overcurrents']
    state_msg.remote_opcode      = d['remote-opcode']
    state_msg.buttons            = d['buttons']

    state_msg.distance           = float(d['distance']) / 1000.
    #rospy.logerr(state_msg.distance)
    state_msg.angle              = radians(d['angle'])
    state_msg.charging_state     = d['charging-state']
    state_msg.voltage            = d['voltage']  # mV
    state_msg.current            = d['current']  # mA
    state_msg.temperature        = d['temperature']  # C
    state_msg.charge             = d['charge']  # mAh
    state_msg.capacity           = d['capacity']  # mAh

    state_msg.wall_signal                = d['wall-signal']
    state_msg.cliff_left_signal          = d['cliff-left-signal']
    state_msg.cliff_front_left_signal    = d['cliff-front-left-signal']
    state_msg.cliff_front_right_signal   = d['cliff-front-right-signal']
    state_msg.cliff_right_signal         = d['cliff-right-signal']
    state_msg.user_digital_outputs       = d['user-digital-outputs']
    state_msg.user_digital_inputs        = d['user-digital-inputs']
    state_msg.user_analog_input          = d['user-analog-input']
    state_msg.charging_sources_available = d['charging-sources-available']
    
    state_msg.oi_mode      = d['oi-mode']
    state_msg.song_number  = d['song-number']
    state_msg.song_playing = d['song-playing']
    
    state_msg.number_of_stream_packets = d['number-of-stream-packets']
    # convert mm to m
    state_msg.requested_velocity       = float(d['requested-velocity']) / 1000.
    state_msg.requested_radius         = float(d['requested-radius']) / 1000.
    state_msg.requested_right_velocity = float(d['requested-right-velocity']) / 1000.
    state_msg.requested_left_velocity  = float(d['requested-left-velocity']) / 1000.
    
def bonus(robot):
    # a nice bit of goodness from the turtlebot driver by Xuwen Cao and
    # Morgan Quigley
    song = (
        (76, 16), (76, 16), (72, 8),  (76, 16), 
        (79, 32), (67, 32), (72, 24), (67, 24), 	
        (64, 24), (69, 16), (71, 16), (70, 8), 
        (69, 16), (79, 16), (76, 16), (72, 8), 
        (74, 16), (71, 24), (60, 16), (79, 8), 
        (78, 8),  (77, 8),  (74, 8),  (75, 8), 
        (76, 16), (67, 8),  (69, 8),  (72, 16), 
        (69, 8),  (72, 8),  (74, 8),  (60, 16), 	
        (79, 8),  (78, 8),  (77, 8),  (74, 8), 
        (75, 8),  (76, 16), (76, 4),  (78, 4), 
        (84, 16), (84, 8),  (84, 16), (84, 16), 
        (60, 16), (79, 8),  (78, 8),  (77, 8), 
        (74, 8),  (75, 8),  (76, 16), (67, 8), 
        (69, 8),  (72, 16), (69, 8),  (72, 8), 
        (74, 16), (70, 4),  (72, 4),  (75, 16), 
        (69, 4),  (71, 4),  (74, 16), (67, 4), 
        (69, 4),  (72, 16), (67, 8),  (67, 16), 
        (60, 24),
         )
    # have to make sure robot is in full mode
    robot.sci.send([128, 132])
    robot.sci.send([140, 1, len(song)])
    for note in song:
        robot.sci.send(note)
    robot.sci.play_song(1)
    

def turtlebot_main():
    c = TurtlebotNode()
    c.spin()

if __name__ == '__main__':
    turtlebot_main()
