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

class AppTest(unittest.TestCase):
  
  
    def test_Interface(self):
        from turtlebot_app_manager.app import Interface
        s1 = {'/chatter': 'std_msgs/String'}
        p1 = {'/chatter': 'std_msgs/String'}
        i1 = Interface(s1, p1)
        self.assertEquals(i1, i1)
        self.assertEquals(s1, i1.subscribed_topics)
        self.assertEquals(s1, i1.published_topics)
        
        i2 = Interface({'/chatter': 'std_msgs/String'}, {'/chatter': 'std_msgs/String'})
        self.assertEquals(i1, i2)
        i3 = Interface({'/chatter2': 'std_msgs/String'}, {'/chatter': 'std_msgs/String'})
        self.assertNotEquals(i1, i3)
        i4 = Interface({'/chatter': 'std_msgs/String'}, {'/chatter2': 'std_msgs/String'})
        self.assertNotEquals(i1, i4)
        
    def test_Client(self):
        from turtlebot_app_manager.app import Client
        c1 = Client(client_type="android",
                    manager_data={"manager1": "dataA"},
                    app_data={"app1": "dataB"})
        self.assertEquals("android", c1.client_type)
        self.assertEquals({"manager1": "dataA"}, c1.manager_data)
        self.assertEquals({"app1": "dataB"}, c1.app_data)
        self.assertEquals(c1, c1)
        c2 = Client(client_type="android",
                    manager_data={"manager1": "dataA"},
                    app_data={"app1": "dataB"})
        self.assertEquals(c1, c2)
        noteq = [
            Client("androidB", {"manager1": "dataA"},
                   {"app1": "dataB"}),
            Client("android", {"manager2": "dataA"},
                   {"app1": "dataB"}),
            Client("android", {"manager1": "dataA"},
                   {"app2": "dataB"}),
            ]
        for c3 in noteq:
            self.assertNotEquals(c1, c3)
            
    def test_AppDefinition(self):
        # name, display_name, description, platform, launch, interface, clients
        from turtlebot_app_manager.app import AppDefinition, Client
        ad1 = AppDefinition('foo', 'Foo', 'Is Foo', 'android', 'foo.launch', 'foo.interface', [Client("android", {}, {})])
        self.assertEquals(ad1, ad1)
        ad2 = AppDefinition('foo', 'Foo', 'Is Foo', 'android', 'foo.launch', 'foo.interface', [Client("android", {}, {})])
        self.assertEquals(ad1, ad2)
        ad3 = AppDefinition('foo', 'Foo', 'Is Foo', 'android', 'foo.launch', 'foo.interface', [Client("android", {}, {})], None)
        self.assertEquals(ad1, ad3)

        ad1b = AppDefinition('foo', 'Foo', 'Is Foo', 'android', 'foo.launch', 'foo.interface', [Client("android", {}, {})], 'app_manager/icon.png')
        self.assertEquals(ad1, ad1)
        ad2b = AppDefinition('foo', 'Foo', 'Is Foo', 'android', 'foo.launch', 'foo.interface', [Client("android", {}, {})], 'app_manager/icon.png')
        self.assertEquals(ad1b, ad2b)
        nad2b = AppDefinition('foo', 'Foo', 'Is Foo', 'android', 'foo.launch', 'foo.interface', [Client("android", {}, {})], 'app_manager/icon2.png')
        self.assertNotEquals(ad1, nad2b)
        
        noteq = [
            AppDefinition('bar', 'Foo', 'Is Foo', 'android', 'foo.launch', 'foo.interface', [Client("android", {}, {})]),
            AppDefinition('foo', 'Bar', 'Is Foo', 'android', 'foo.launch', 'foo.interface', [Client("android", {}, {})]),
            AppDefinition('foo', 'Foo', 'Is Bar', 'android', 'foo.launch', 'foo.interface', [Client("android", {}, {})]),
            AppDefinition('foo', 'Foo', 'Is Foo', 'ios', 'foo.launch', 'foo.interface', [Client("android", {}, {})]),
            AppDefinition('foo', 'Foo', 'Is Foo', 'android', 'bar.launch', 'foo.interface', [Client("android", {}, {})]),
            AppDefinition('foo', 'Foo', 'Is Foo', 'android', 'foo.launch', 'bar.interface', [Client("android", {}, {})]),
            AppDefinition('foo', 'Foo', 'Is Foo', 'android', 'foo.launch', 'foo.interface', [Client("ios", {}, {})]),
            ]
        for nad in noteq:
            self.assertNotEquals(ad1, nad)
        
    def test_find_resource(self):
        from turtlebot_app_manager.app import find_resource
        rospack = rospkg.RosPack()
        path = rospack.get_path(PKG)
        test_dir = os.path.join(path, 'test')

        e = os.path.join(test_dir, 'empty.interface')
        self.assertEquals(e, find_resource('%s/empty.interface'%(PKG)))

        e = os.path.join(test_dir, 'applist1', 'apps1.installed')
        self.assertEquals(e, find_resource('%s/apps1.installed'%(PKG)))

        try:
            find_resource('empty.interface')
            self.fail("should have thrown ValueError: no package name")
        except ValueError:
            pass
        try:
            find_resource('turtlebot_app_manager')
            self.fail("should have thrown ValueError: no resource name")
        except ValueError:
            pass
        
    def test_load_AppDefinition_by_name(self):
        rospack = rospkg.RosPack()
        rl_dir = rospack.get_path('roslaunch')
        from app_manager import NotFoundException, InternalAppException
        from app_manager.app import load_AppDefinition_by_name, Interface
        try:
            load_AppDefinition_by_name(None)
            self.fail("should fail")
        except ValueError: pass
        try:
            load_AppDefinition_by_name("fake_pkg/appA")
            self.fail("should fail")
        except NotFoundException: pass
        
        ad = load_AppDefinition_by_name('app_manager/appA')
        self.assertEquals("turtlebot_app_manager/appA", ad.name)
        self.assertEquals("Android Joystick", ad.display_name)
        self.assertEquals("Control the TurtleBot with an Android device", ad.description)
        self.assertEquals("turtlebot", ad.platform)
        self.assertEquals(os.path.join(rl_dir, 'example-min.launch'), ad.launch)
        self.assert_(isinstance(ad.interface, Interface))
        self.assertEquals({}, ad.interface.subscribed_topics)
        self.assertEquals({}, ad.interface.published_topics)
        self.assertEquals([], ad.clients)

        #monkey patch in for coverage
        import errno
        def fake_load_enoent(*args):
            raise IOError(errno.ENOENT, "fnf")
        def fake_load_gen(*args):
            raise IOError()
        import turtlebot_app_manager.app
        load_actual = turtlebot_app_manager.app.load_AppDefinition_from_file
        try:
            turtlebot_app_manager.app.load_AppDefinition_from_file = fake_load_enoent
            try:
                load_AppDefinition_by_name('turtlebot_app_manager/appA')
                self.fail("should have raised")
            except NotFoundException: pass

            turtlebot_app_manager.app.load_AppDefinition_from_file = fake_load_gen
            try:
                load_AppDefinition_by_name('turtlebot_app_manager/appA')
                self.fail("should have raised")
            except InternalAppException: pass
        finally:
            turtlebot_app_manager.app.load_AppDefinition_from_file = load_actual
        
    def test_load_AppDefinition_from_file(self):
        rospack = rospkg.RosPack()
        rl_dir = rospack.get_path('roslaunch')
        path = rospack.get_path(PKG)
        test_dir = os.path.join(path, 'test')

        from app_manager.app import load_AppDefinition_from_file, Interface
        ad = load_AppDefinition_from_file(os.path.join(test_dir, 'appA.app'), 'foo/AppA')
        self.assertEquals("foo/AppA", ad.name)
        self.assertEquals("Android Joystick", ad.display_name)
        self.assertEquals("Control the TurtleBot with an Android device", ad.description)
        self.assertEquals("turtlebot", ad.platform)
        self.assertEquals(os.path.join(rl_dir, 'example-min.launch'), ad.launch)
        self.assert_(isinstance(ad.interface, Interface))
        self.assertEquals({}, ad.interface.subscribed_topics)
        self.assertEquals({}, ad.interface.published_topics)
        self.assertEquals([], ad.clients)
        #self.assertEquals('app_manager/empty.interface', ad.interface)
        
    def test_load_Interface_from_file(self):
        from turtlebot_app_manager.app import load_Interface_from_file
        rospack = rospkg.RosPack()
        path = rospack.get_path(PKG)
        test_dir = os.path.join(path, 'test')

        empty = load_Interface_from_file(os.path.join(test_dir, 'empty.interface'))
        self.assertEquals({}, empty.subscribed_topics)
        self.assertEquals({}, empty.published_topics)

        test1 = load_Interface_from_file(os.path.join(test_dir, 'test1.interface'))
        self.assertEquals({'/camera/rgb/image_color/compressed': 'sensor_msgs/CompressedImage'}, test1.published_topics)
        self.assertEquals({'/turtlebot_node/cmd_vel': 'geometry_msgs/Twist'}, test1.subscribed_topics)
      
      
if __name__ == '__main__':
  rosunit.unitrun(PKG, 'test_app', AppTest, coverage_packages=['app_manager.app'])

