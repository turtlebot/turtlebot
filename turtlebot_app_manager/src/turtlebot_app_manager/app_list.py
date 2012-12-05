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
#
# Revision $Id: topics.py 11753 2010-10-25 06:23:19Z kwc $

# author: kwc

"""
Implements applist part of app_manager, which handles listing of
currently installed applications.
"""

import os
import sys
import yaml

from .app import load_AppDefinition_by_name
from app_manager.msg import App, ClientApp, KeyValue, Icon
from .exceptions import AppException, InvalidAppException

import roslib
roslib.load_manifest('turtlebot_app_manager')
import rospy

def get_default_applist_directory():
    """
    Default directory where applist configuration is stored.
    """
    return "/etc/robot/apps"

def dict_to_KeyValue(d):
    l = []
    for k, v in d.iteritems():
        l.append(KeyValue(k, str(v)))
    return l
        
def read_Icon_file(filename):
    icon = Icon()
    if filename == None or filename == "":
        return icon
    basename, extension = os.path.splitext(filename)
    if extension.lower() == ".jpg" or extension.lower() == ".jpeg":
        icon.format = "jpeg"
    elif extension.lower() == ".png":
        icon.format = "png"
    else:
        icon.format = ""
        return icon
    icon.data = open(filename, "rb").read()
    return icon

def AppDefinition_to_App(app_definition):
    a = App(name=app_definition.name, display_name=app_definition.display_name, icon=read_Icon_file(app_definition.icon))
    a.client_apps = []
    for c in app_definition.clients:
        a.client_apps.append(ClientApp(c.client_type,
                                       dict_to_KeyValue(c.manager_data),
                                       dict_to_KeyValue(c.app_data)))
    return a

class InstalledFile(object):
    """
    Models data stored in a .installed file.  These files are used to
    track installation of apps.
    """
    
    def __init__(self, filename):
        self.filename = filename
        # list of App
        self.available_apps = []

        self._file_mtime = None
        self.update()

    def _load(self):
        available_apps = []
        with open(self.filename) as f:
            installed_data = yaml.load(f)
            for reqd in ['apps']:
                if not reqd in installed_data:
                    raise InvalidAppException("installed file [%s] is missing required key [%s]"%(self.filename, reqd))
            for app in installed_data['apps']:
                for areqd in ['app']:
                    if not areqd in app:
                        raise InvalidAppException("installed file [%s] app definition is missing required key [%s]"%(self.filename, areqd))
                available_apps.append(load_AppDefinition_by_name(app['app']))
                
        self.available_apps = available_apps

    def update(self):
        """
        Update app list
        """
        s = os.stat(self.filename)
        if s.st_mtime != self._file_mtime:
            self._load()
            self._file_mtime = s.st_mtime
    
class AppList(object):
    
    def __init__(self, applist_directories):
        self.applist_directories = applist_directories
        self.installed_files = {}
        self.invalid_installed_files = []
        self.app_list = []
        
        self._applist_directory_mtime = None
        self.update()
        
    def _load(self):
        app_list = []
        invalid_installed_files = []
        
        dir_list = [] 
        for i in self.applist_directories:
            for k in os.listdir(i):
                dir_list.append(k)
            
        
        for f in set(self.installed_files.keys()) - set(dir_list):
            rospy.loginfo("App Manager: deleting installation data for [%s]"%(f))
            del self.installed_files[f]
        
        for i in self.applist_directories:
            for f in os.listdir(i):
                rospy.loginfo("App Manager: found installed app list [%s]."%f)
                if not f.endswith('.installed'):
                    continue
                try:
                    if f in self.installed_files:
                        installed_file = self.installed_files[f]
                        installed_file.update()
                    else:
                        rospy.loginfo("App Manager: loading installation data for [%s]"%(f))
                        filename = os.path.join(i, f)
                        installed_file = InstalledFile(filename)
                        self.installed_files[f] = installed_file
                    
                    app_list.extend(installed_file.available_apps)

                except AppException as ae:
                    print >> sys.stderr, "ERROR: %s"%(str(ae))
                    invalid_installed_files.append((filename, ae))
                except Exception as e:
                    print >> sys.stderr, "ERROR: %s"%(str(e))
                    invalid_installed_files.append((filename, e))

        self.app_list = app_list
        self.invalid_installed_files = invalid_installed_files
        
    def get_app_list(self):
        return [AppDefinition_to_App(ad) for ad in self.app_list]
    
    def add_directory(self, directory):
        self.applist_directories.append(directory)

    def update(self):
        """
        Update app list
        """
        
        bad = True
        #TODO: this detects when the directories are actually modified.
        #It does not work because os.stat(i).st_mtime does not change if
        #only a file is modified.
        #s = []
        #for i in self.applist_directories:
        #    s.append(os.stat(i).st_mtime)
        #
        #bad = True
        #if self._applist_directory_mtime != None:
        #    if len(s) == len(self._applist_directory_mtime):
        #        bad = False
        #        for i in range(0, len(s)):
        #            if s[i] != self._applist_directory_mtime[i]:
        #                bad = True
        
        if (bad):
            self._load()
            #self._applist_directory_mtime = s
