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
PKG = 'turtlebot_app_manager'
import roslib; roslib.load_manifest(PKG)

import os
import sys
import unittest

import rospkg
import rosunit

def touch(filename):
    os.utime(filename, None)

class AppListTest(unittest.TestCase):
  
    def test_AppList(self):
        import app_manager
        rospack = rospkg.RosPack()
        path = rospack.get_path(PKG)
        test_dir = os.path.join(path, 'test')

        app_list = turtlebot_app_manager.AppList([os.path.join(test_dir, 'applist0')])
        self.assertEquals([], app_list.get_app_list())

        filename = os.path.join(test_dir, 'applist1')
        app_list = turtlebot_app_manager.AppList([filename])
        al = app_list.get_app_list()
        self.assertEquals([], app_list.invalid_installed_files)
        #self.assertEquals(1, len(al), al.invalid_installed_files)
        self.assertEquals('Android Joystick', al[0].display_name )
      
        #Had to be commented out, see app_list.py
        #mtime = app_list._applist_directory_mtime
        #app_list.update()
        #self.assertEquals(mtime, app_list._applist_directory_mtime)
        #touch(filename)
        #app_list.update()
        #self.assertNotEquals(mtime, app_list._applist_directory_mtime)

        filename = os.path.join(test_dir, 'applistbad')
        app_list = turtlebot_app_manager.AppList([filename])
        al = app_list.get_app_list()
        self.assertEquals([], al)
        self.assertEquals(2, len(app_list.invalid_installed_files))

    def test_get_default_applist_directory(self):
        import turtlebot_app_manager.app_list
        self.assertEquals('/etc/robot/apps', turtlebot_app_manager.app_list.get_default_applist_directory())
      
    def test_InstalledFile(self):
        from turtlebot_app_manager import InvalidAppException
        from turtlebot_app_manager.app import find_resource
        from turtlebot_app_manager.app_list import InstalledFile

        filename = find_resource('turtlebot_app_manager/apps1.installed')
        inf = InstalledFile(filename)
        self.failIf(inf._file_mtime is None)
        self.assertEquals(filename, inf.filename)
        self.assertEquals(1, len(inf.available_apps))
        self.assertEquals('Android Joystick', inf.available_apps[0].display_name)
                          
        #Had to be commented out, see app_list.py
        #mtime = inf._file_mtime
        #inf.update()
        #self.assertEquals(mtime, inf._file_mtime)
        #touch(filename)
        #inf.update()
        #self.assertNotEquals(mtime, inf._file_mtime)      
      
        for bad in ['turtlebot_app_manager/bad.installed', 'turtlebot_app_manager/bad2.installed']:
            filename = find_resource(bad)
            try:
                inf = InstalledFile(filename)
                self.fail("should have thrown")
            except InvalidAppException: pass
        
    def test_dict_to_KeyValue(self):
        from turtlebot_app_manager.msg import KeyValue
        from turtlebot_app_manager.app_list import dict_to_KeyValue

        v = dict_to_KeyValue({})
        self.assertEquals([], v)

        v = dict_to_KeyValue({'a': 'b'})
        self.assertEquals([KeyValue('a', 'b')], v)

        v = dict_to_KeyValue({'a': 'b', 'c': 'd'})
        for ve in [KeyValue('a', 'b'), KeyValue('c', 'd')]:
            self.assert_(ve in v)

        # make sure that types convert
        v = dict_to_KeyValue({'a': 1})
        self.assertEquals([KeyValue('a', '1')], v)

    def test_AppDefinition_to_App(self):
        from turtlebot_app_manager.msg import App, ClientApp, KeyValue
        from turtlebot_app_manager.app import AppDefinition, Client
        from turtlebot_app_manager.app_list import AppDefinition_to_App, dict_to_KeyValue

        ad = AppDefinition(name="appname", display_name="An App", 
                           description="Does something", platform="fakebot",
                           launch="file.launch", interface="file.interface", clients=[])
        a = AppDefinition_to_App(ad)
        self.assertEquals(a.name, 'appname')
        self.assertEquals(a.display_name, 'An App')
        self.assertEquals([], a.client_apps)

        client1 = Client('android',
                         {'manager1': 'data1'},
                         {'app1': 'data1'})
        ca = ClientApp('android', [KeyValue('manager1', 'data1')], [KeyValue('app1', 'data1')])
        ad = AppDefinition(name="appname", display_name="An App", 
                           description="Does something", platform="fakebot",
                           launch="file.launch", interface="file.interface", clients=[client1])
        a = AppDefinition_to_App(ad)
        self.assertEquals([ca], a.client_apps)

        client1 = Client('android',
                         {'manager1': 'data1', 'manager2': 'data2'},
                         {'app1': 'data1', 'app2': 'data2'})
        ca = ClientApp('android', dict_to_KeyValue(client1.manager_data), dict_to_KeyValue(client1.app_data))    
        ad = AppDefinition(name="appname", display_name="An App", 
                           description="Does something", platform="fakebot",
                           launch="file.launch", interface="file.interface", clients=[client1])
        a = AppDefinition_to_App(ad)
        self.assertEquals([ca], a.client_apps)

        client2 = Client('web', {},
                         {'app2': 'data2', 'app2b': 'data2b'})
        ca2 = ClientApp('web', [], dict_to_KeyValue(client2.app_data))  
        ad = AppDefinition(name="appname", display_name="An App", 
                           description="Does something", platform="fakebot",
                           launch="file.launch", interface="file.interface", clients=[client1, client2])
        a = AppDefinition_to_App(ad)
        self.assertEquals([ca, ca2], a.client_apps)

      
if __name__ == '__main__':
    rosunit.unitrun(PKG, 'test_app_list', AppListTest, coverage_packages=['turtlebot_app_manager.app_list'])

