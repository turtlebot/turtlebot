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

##\author Kevin Watts
##\brief Monitors laptop battery status

from __future__ import division

import roslib; roslib.load_manifest('turtlebot_node')

from   collections import deque
import threading
import copy
import yaml
import math
import rospy

from turtlebot_node.msg import LaptopChargeStatus
from diagnostic_msgs.msg import DiagnosticStatus, DiagnosticArray, KeyValue

def _strip_Ah(raw_val):
    if 'mAh' in raw_val:
        rv = float(raw_val.rstrip('mAh').strip()) / 1000.0
    elif 'Ah' in raw_val:
        rv = float(raw_val.rstrip('Ah').strip())
    elif 'mWh' in raw_val:
        rv = float(raw_val.rstrip('mWh').strip()) / 1000.0
    elif 'Wh' in raw_val:
        rv = float(raw_val.rstrip('Wh').strip())
    else:
        raise Exception('Value %s did not have supported units. (mAh,Ah,mWh,Wh)' % raw_val)
    return rv

def _strip_V(raw_val):
    if 'mV' in raw_val:
        rv = float(raw_val.rstrip('mV').strip()) / 1000.0
    elif 'V' in raw_val:
        rv = float(raw_val.rstrip('V').strip())
    else:
        raise Exception('Value %s did not have "V" or "mV"' % raw_val)
    return rv

def _strip_A(raw_val):
    if 'mA' in raw_val:
        rv = float(raw_val.rstrip('mA').strip()) / 1000.0
    elif 'A' in raw_val:
        rv = float(raw_val.rstrip('A').strip())
    elif 'mW' in raw_val:
        rv = float(raw_val.rstrip('mW').strip()) / 1000.0
    elif 'W' in raw_val:
        rv = float(raw_val.rstrip('W').strip())
    else:
        raise Exception('Value %s did not have supported units. (A,mA,W,mW)' % raw_val)
    return rv

def slerp(filename):
    f = open(filename, 'r')
    data = f.read()
    f.close()
    data = data.replace('\t', '  ')
    return data

#/proc/acpi/battery/BAT0/state
def _check_battery_info(_battery_acpi_path):
    o = slerp(_battery_acpi_path+'/info')

    batt_info = yaml.load(o)
    design_capacity    = _strip_Ah(batt_info.get('design capacity',    '0 mAh'))
    last_full_capacity = _strip_Ah(batt_info.get('last full capacity', '0 mAh'))
    
    return (design_capacity, last_full_capacity)

state_to_val = {'charged':     LaptopChargeStatus.CHARGED, 
                'charging':    LaptopChargeStatus.CHARGING, 
                'discharging': LaptopChargeStatus.DISCHARGING }

diag_level_to_msg = { DiagnosticStatus.OK:    'OK', 
                      DiagnosticStatus.WARN:  'Warning',
                      DiagnosticStatus.ERROR: 'Error'    }

def _check_battery_state(_battery_acpi_path):
    """
    @return LaptopChargeStatus
    """
    o = slerp(_battery_acpi_path+'/state')

    batt_info = yaml.load(o)

    rv = LaptopChargeStatus()

    state = batt_info.get('charging state', 'discharging')
    rv.charge_state = state_to_val.get(state, 0)
    rv.rate     = _strip_A(batt_info.get('present rate',        '-1 mA'))
    if rv.charge_state == LaptopChargeStatus.DISCHARGING:
        rv.rate = math.copysign(rv.rate, -1) # Need to set discharging rate to negative
    
    rv.charge   = _strip_Ah(batt_info.get('remaining capacity', '-1 mAh'))
    rv.voltage  = _strip_V(batt_info.get('present voltage',     '-1 mV'))
    rv.present  = batt_info.get('present', False)

    rv.header.stamp = rospy.get_rostime()

    return rv

def _laptop_charge_to_diag(laptop_msg):
    rv = DiagnosticStatus()
    rv.level   = DiagnosticStatus.OK
    rv.message = 'OK'
    rv.name    = 'Laptop Battery'

    if not laptop_msg.present:
        rv.level = DiagnosticStatus.ERROR
        rv.message = 'Laptop battery missing'

    rv.values.append(KeyValue('Voltage (V)',          str(laptop_msg.voltage)))
    rv.values.append(KeyValue('Current (A)',          str(laptop_msg.rate)))
    rv.values.append(KeyValue('Charge (Ah)',          str(laptop_msg.charge)))
    rv.values.append(KeyValue('Capacity (Ah)',        str(laptop_msg.capacity)))
    rv.values.append(KeyValue('Design Capacity (Ah)', str(laptop_msg.design_capacity)))

    return rv

class LaptopBatteryMonitor(object):
    def __init__(self):
        self._mutex = threading.Lock()

        self._last_info_update  = 0
        self._last_state_update = 0
        self._msg = LaptopChargeStatus()
        
        self._power_pub = rospy.Publisher('laptop_charge', LaptopChargeStatus, latch=True)
        self._diag_pub  = rospy.Publisher('/diagnostics', DiagnosticArray)
        
        # Battery info
        self._batt_acpi_path = rospy.get_param('~acpi_path', "/proc/acpi/battery/BAT0")
        self._batt_design_capacity = 0
        self._batt_last_full_capacity = 0
        self._last_info_update = 0

        self._batt_info_rate = 1 / 60.0
        self._batt_info_thread = threading.Thread(target=self._check_batt_info)
        self._batt_info_thread.daemon = True
        self._batt_info_thread.start()

        # Battery state
        self._batt_state_rate = 1 / 5.0
        self._batt_state_thread = threading.Thread(target=self._check_batt_state)
        self._batt_state_thread.daemon = True
        self._batt_state_thread.start()

    def _check_batt_info(self):
        rate = rospy.Rate(self._batt_info_rate)
        while not rospy.is_shutdown():
            try:
                design_cap, last_full_cap = _check_battery_info(self._batt_acpi_path)
                with self._mutex:
                    self._batt_last_full_capacity = last_full_cap
                    self._batt_design_capacity    = design_cap
                    self._last_info_update        = rospy.get_time()
            except Exception, e:
                rospy.logwarn('Unable to check laptop battery info. Exception: %s' % e)
                
            rate.sleep()

    def _check_batt_state(self):
        rate = rospy.Rate(self._batt_state_rate)
        while not rospy.is_shutdown():
            try:
                msg = _check_battery_state(self._batt_acpi_path)
                with self._mutex:
                    self._msg = msg
                    self._last_state_update = rospy.get_time()
            except Exception, e:
                rospy.logwarn('Unable to check laptop battery state. Exception: %s' % e)
                
            rate.sleep()

    def update(self):
        with self._mutex:
            diag = DiagnosticArray()
            diag.header.stamp = rospy.get_rostime()
            
            info_update_ok  = rospy.get_time() - self._last_info_update  < 5.0 / self._batt_info_rate
            state_update_ok = rospy.get_time() - self._last_state_update < 5.0 / self._batt_state_rate

            if info_update_ok:
                self._msg.design_capacity = self._batt_design_capacity
                self._msg.capacity        = self._batt_last_full_capacity
            else:
                self._msg.design_capacity = 0.0
                self._msg.capacity        = 0.0
                
            if info_update_ok and state_update_ok and self._msg.capacity != 0:
                self._msg.percentage = int(self._msg.charge / self._msg.capacity * 100.0)

            diag_stat = _laptop_charge_to_diag(self._msg)
            if not info_update_ok or not state_update_ok:
                diag_stat.level   = DiagnosticStatus.ERROR
                diag_stat.message = 'Laptop battery data stale'

            diag.status.append(diag_stat)

            self._diag_pub.publish(diag)
            self._power_pub.publish(self._msg)

if __name__ == '__main__':
    rospy.init_node('laptop_battery')

    bm = LaptopBatteryMonitor()
    try:
        r = rospy.Rate(1.0)
        while not rospy.is_shutdown():
            bm.update()
            r.sleep()
    except KeyboardInterrupt:
        pass
    except Exception:
        import traceback
        traceback.print_exc()

