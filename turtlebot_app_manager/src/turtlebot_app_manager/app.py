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
# Revision $Id: app.py 14667 2011-08-12 23:55:04Z pratkanis $

# author: leibs, kwc

import os
import errno
import yaml

import roslib.names
from rospkg import ResourceNotFound
from .exceptions import AppException, InvalidAppException, NotFoundException, InternalAppException

class Interface(object):
    def __init__(self, subscribed_topics, published_topics):
        self.subscribed_topics = subscribed_topics
        self.published_topics = published_topics

    def __eq__(self, other):
        if not isinstance(other, Interface):
            return False
        return self.subscribed_topics == other.subscribed_topics and \
               self.published_topics == other.published_topics
               
class Client(object):
    __slots__ = ['client_type', 'manager_data', 'app_data']
    def __init__(self, client_type, manager_data, app_data):
        self.client_type = client_type
        self.manager_data = manager_data
        self.app_data = app_data
        
    def as_dict(self):
        return {'client_type': self.client_type, 'manager_data': self.manager_data, 'app_data': self.app_data}
    
    def __eq__(self, other):
        if not isinstance(other, Client):
            return False
        return self.client_type == other.client_type and \
               self.manager_data == other.manager_data and \
               self.app_data == other.app_data

    def __repr__(self):
        return yaml.dump(self.as_dict())
               
class AppDefinition(object):
    __slots__ = ['name', 'display_name', 'description', 'platform',
                 'launch', 'interface', 'clients', 'icon']
    def __init__(self, name, display_name, description, platform,
                 launch, interface, clients, icon=None):
        self.name = name
        self.display_name = display_name
        self.description = description
        self.platform=platform
        self.launch = launch
        self.interface = interface
        self.clients = clients
        self.icon = icon

    def __repr__(self):
        d = {}
        for s in self.__slots__:
            if s == 'clients':
                d[s] = [c.as_dict() for c in self.clients]
            else:
                d[s] = getattr(self, s)
        return yaml.dump(d)
    #    return "name: %s\ndisplay: %s\ndescription: %s\nplatform: %s\nlaunch: %s\ninterface: %s\nclients: %s"%(self.name, self.display_name, self.description, self.platform, self.launch, self.interface, self.clients)
    
    def __eq__(self, other):
        if not isinstance(other, AppDefinition):
            return False
        return self.name == other.name and \
               self.display_name == other.display_name and \
               self.description == other.description and \
               self.platform == other.platform and \
               self.launch == other.launch and \
               self.interface == other.interface and \
               self.clients == other.clients and \
               self.icon == other.icon
               
def find_resource(resource):
    """
    @return: filepath of resource.  Does not validate if filepath actually exists.
    
    @raise ValueError: if resource is not a valid resource name.
    @raise rospkg.ResourceNotFound: if package referred
        to in resource name cannot be found.
    @raise NotFoundException: if resource does not exist.
    """
    p, a = roslib.names.package_resource_name(resource)
    if not p:
        raise ValueError("Resource is missing package name: %s"%(resource))
    matches = roslib.packages.find_resource(p, a)
    # TODO: convert ValueError to better type for better error messages
    if len(matches) == 1:
        return matches[0]
    elif not matches:
        raise NotFoundException("No resource [%s]"%(resource))        
    else:
        raise ValueError("Multiple resources named [%s]"%(resource))        

def load_Interface_from_file(filename):
    """
    @raise IOError: I/O error reading file (e.g. does not exist)
    @raise InvalidAppException: if app file is invalid
    """
    with open(filename,'r') as f:
        y = yaml.load(f.read())
        y = y or {} #coerce to dict
        try:
            subscribed_topics = y.get('subscribed_topics', {})
            published_topics = y.get('published_topics', {})
        except KeyError:
            raise InvalidAppException("Malformed interface, missing keys")
    return Interface(published_topics=published_topics, subscribed_topics=subscribed_topics)

def _AppDefinition_load_icon_entry(app_data, appfile="UNKNOWN"):
    """
    @raise InvalidAppExcetion: if app definition is invalid.
    """
    # load/validate launch entry
    try:
        icon_resource = app_data.get('icon', '')
        if icon_resource == '':
            return None
        icon_filename = find_resource(icon_resource)
        if not icon_filename or not os.path.exists(icon_filename):
            return None
        return icon_filename
    except ValueError as e:
        raise InvalidAppException("Malformed appfile [%s]: bad icon entry: %s"%(appfile, e))
    except NotFoundException:
        # TODO: make this a soft fail?
        raise InvalidAppException("App file [%s] refers to icon that cannot be found"%(appfile))
    except ResourceNotFound as e:
        raise InvalidAppException("App file [%s] refers to package that is not installed: %s"%(appfile, str(e)))

def _AppDefinition_load_launch_entry(app_data, appfile="UNKNOWN"):
    """
    @raise InvalidAppExcetion: if app definition is invalid.
    """
    # load/validate launch entry
    try:
        launch = find_resource(app_data['launch'])
        if not os.path.exists(launch):
            raise InvalidAppException("Malformed appfile [%s]: refers to launch that does not exist."%(appfile))
        return launch
    except ValueError as e:
        raise InvalidAppException("Malformed appfile [%s]: bad launch entry: %s"%(appfile, e))
    except NotFoundException:
        raise InvalidAppException("App file [%s] refers to launch that is not installed"%(appfile))
    except ResourceNotFound as e:
        raise InvalidAppException("App file [%s] refers to package that is not installed: %s"%(appfile, str(e)))

def _AppDefinition_load_interface_entry(app_data, appfile="UNKNOWN"):
    """
    @raise InvalidAppExcetion: if app definition is invalid.
    """
    # load/validate interface entry
    try:
        return load_Interface_from_file(find_resource(app_data['interface']))
    except IOError as e:
        if e.errno == errno.ENOENT:
            raise InvalidAppException("Malformed appfile [%s]: refers to interface file that does not exist"%(appfile))
        else:
            raise InvalidAppException("Error with appfile [%s]: cannot read interface file"%(appfile))
    except ValueError:
        raise InvalidAppException("Malformed appfile [%s]: bad interface entry"%(appfile))
    except ResourceNotFound as e:
        raise InvalidAppException("App file [%s] refers to package that is not installed: %s"%(appfile, str(e)))
    
def _AppDefinition_load_clients_entry(app_data, appfile="UNKNOWN"):
    """
    @raise InvalidAppExcetion: if app definition is invalid.
    """
    clients_data = app_data.get('clients', [])
    clients = []
    for c in clients_data:
        for reqd in ['type', 'manager']:
            if not reqd in c:
                raise InvalidAppException("Malformed appfile [%s], missing required key [%s]"%(appfile, reqd))
        client_type = c['type']
        manager_data = c['manager']
        if not type(manager_data) == dict:
            raise InvalidAppException("Malformed appfile [%s]: manager data must be a map"%(appfile))

        app_data = c.get('app', {})
        if not type(app_data) == dict:
            raise InvalidAppException("Malformed appfile [%s]: app data must be a map"%(appfile))

        clients.append(Client(client_type, manager_data, app_data))
    return clients

def load_AppDefinition_from_file(appfile, appname):
    """
    @raise InvalidAppExcetion: if app definition is invalid.
    @raise IOError: I/O error reading appfile (e.g. file does not exist).
    """
    with open(appfile,'r') as f:
        app_data = yaml.load(f.read())
        for reqd in ['launch', 'interface', 'platform']:
            if not reqd in app_data:
                raise InvalidAppException("Malformed appfile [%s], missing required key [%s]"%(appfile, reqd))

        display_name = app_data.get('display', appname)
        description = app_data.get('description', '')        
        platform = app_data['platform']


        launch = _AppDefinition_load_launch_entry(app_data, appfile)
        interface = _AppDefinition_load_interface_entry(app_data, appfile)
        clients = _AppDefinition_load_clients_entry(app_data, appfile)
        icon = _AppDefinition_load_icon_entry(app_data, appfile)

    return AppDefinition(appname, display_name, description, platform,
                         launch, interface, clients, icon)
    
def load_AppDefinition_by_name(appname):
    """
    @raise InvalidAppExcetion: if app definition is invalid.
    @raise NotFoundExcetion: if app definition is not installed.
    @raise ValueError: if appname is invalid.
    """
    if not appname:
        raise ValueError("app name is empty")

    try:
        appfile = find_resource(appname + '.app')
    except ResourceNotFound as e:
        raise NotFoundException("Cannot locate app file for %s: package is not installed."%(appname))

    try:
        return load_AppDefinition_from_file(appfile, appname)
    except IOError as e:
        if e.errno == errno.ENOENT:
            raise NotFoundException("Cannot locate app file for %s."%(appname))
        else:
            raise InternalAppException("I/O error loading AppDefinition file: %s."%(e.errno))
