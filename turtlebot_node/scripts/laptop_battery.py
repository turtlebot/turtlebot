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


import roslib
roslib.load_manifest('turtlebot_node')
import rospy
import diagnostic_msgs.msg
import subprocess

def laptop_battery_monitor():
    diag_pub = rospy.Publisher('/diagnostics', diagnostic_msgs.msg.DiagnosticArray)
    rospy.init_node('laptop_battery_monitor')
    while not rospy.is_shutdown():
        
        #Main header
        diag = diagnostic_msgs.msg.DiagnosticArray()
        diag.header.stamp = rospy.Time.now()

        
        #battery info                                                                                                                              
        stat = diagnostic_msgs.msg.DiagnosticStatus()
        stat.name = "Laptop Battery"
        stat.level = diagnostic_msgs.msg.DiagnosticStatus.OK
        stat.message = "OK"


        # get values from upower
        cmd = ['upower', '-d']
        command_line = subprocess.Popen(cmd, stdout=subprocess.PIPE,  stderr=subprocess.STDOUT)
        (pstd_out, pstd_err) = command_line.communicate() # pstd_err should be None due to pipe above 
        raw_battery_stats = {}
        for l in pstd_out.split('\n'):
            l_vec = l.split(':')
            l_stripped = [l.strip() for l in l_vec]
            if len(l_stripped) == 2:
                k, v = l_stripped
                if k == 'voltage':
                    raw_battery_stats['voltage'] = v.split()[0]
                elif k == 'energy-rate':
                    raw_battery_stats['watts'] = v.split()[0]
                elif k == 'energy-full':
                    raw_battery_stats['energy_full_wh'] = v.split()[0]
                elif k == 'percentage':
                    raw_battery_stats['percentage'] = v.rstrip('%')
                elif k == 'on-battery':
                    raw_battery_stats['on_battery'] = v
                    

                #else:
                #    print "not processing", l_vec


        voltage = float(raw_battery_stats['voltage'])
        watts = float(raw_battery_stats['watts'])
        if raw_battery_stats['on_battery'] == 'no':
            current_direction = 1
        else:
            current_direction = -1
        current = current_direction * watts / voltage
        capacity_fraction  = float(raw_battery_stats['percentage'])/100.0
        energy_full = float(raw_battery_stats['energy_full_wh'])
        capacity = energy_full / voltage
        charge = capacity_fraction * capacity
        

        stat.values.append(diagnostic_msgs.msg.KeyValue("Voltage (V)", str(voltage)))
        stat.values.append(diagnostic_msgs.msg.KeyValue("Current (A)", str(current)))
        stat.values.append(diagnostic_msgs.msg.KeyValue("Charge (Ah)", str(charge)))
        stat.values.append(diagnostic_msgs.msg.KeyValue("Capacity (Ah)", str(capacity)))

        #append
        diag.status.append(stat)
        
        #publish
        diag_pub.publish(diag)
        rospy.sleep(1.0)


if __name__ == '__main__':
    try:
        laptop_battery_monitor()
    except rospy.ROSInterruptException: pass
    except IOError: pass
    except KeyError: pass
