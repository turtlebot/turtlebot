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
# Revision $Id: app_manager.py 14948 2011-09-07 19:25:54Z pratkanis $

# author: leibs

import roslib.names
import rospy
import thread
import time

import roslaunch.parent
import roslaunch.pmon

from .app import AppDefinition, load_AppDefinition_by_name
from .exceptions import LaunchException, AppException, InvalidAppException, NotFoundException
from .master_sync import MasterSync
from app_manager.msg import App, AppList, StatusCodes, AppStatus, AppInstallationState, ExchangeApp
from app_manager.srv import StartApp, StopApp, ListApps, ListAppsResponse, StartAppResponse, StopAppResponse, InstallApp, UninstallApp, GetInstallationState, UninstallAppResponse, InstallAppResponse, GetInstallationStateResponse, GetAppDetails, GetAppDetailsResponse

class AppManager(object):

    def __init__(self, robot_name, interface_master, app_list, exchange, single_app):
        self._robot_name = robot_name
        self._interface_master = interface_master
        self._app_list = app_list
        self._single_app = single_app
        self._current_apps = []
        self._current_app_definitions = []
        self._exchange = exchange
            
        rospy.loginfo("Starting app manager for %s"%self._robot_name)

        self._app_interface = self.scoped_name('application')

        # note: we publish into the application namespace
        self._status_pub = rospy.Publisher(self.scoped_name('application/app_status'), AppStatus, latch=True)
        self._list_apps_pub = rospy.Publisher(self.scoped_name('app_list'), AppList, latch=True)
        
        self._list_apps_srv  = rospy.Service(self.scoped_name('list_apps'),  ListApps,  self.handle_list_apps)
        self._start_app_srv = rospy.Service(self.scoped_name('start_app'), StartApp, self.handle_start_app)
        self._stop_app_srv   = rospy.Service(self.scoped_name('stop_app'),   StopApp,   self.handle_stop_app)
        if (self._exchange):
            self._exchange_list_apps_pub = rospy.Publisher(self.scoped_name('exchange_app_list'), AppInstallationState, latch=True)
            self._list_exchange_apps_srv = rospy.Service(self.scoped_name('list_exchange_apps'), GetInstallationState, self.handle_list_exchange_apps)
            self._get_app_details_srv = rospy.Service(self.scoped_name('get_app_details'), GetAppDetails, self.handle_get_app_details)
            self._install_app_srv = rospy.Service(self.scoped_name('install_app'), InstallApp, self.handle_install_app)
            self._uninstall_app_srv = rospy.Service(self.scoped_name('uninstall_app'), UninstallApp, self.handle_uninstall_app)
        
            pub_names = [x.resolved_name for x in [self._list_apps_pub, self._status_pub, self._exchange_list_apps_pub]]
            service_names = [x.resolved_name for x in [self._list_apps_srv, self._start_app_srv, self._stop_app_srv, self._get_app_details_srv, self._list_exchange_apps_srv, self._install_app_srv, self._uninstall_app_srv]]
        else:
            pub_names = [x.resolved_name for x in [self._list_apps_pub, self._status_pub]]
            service_names = [x.resolved_name for x in [self._list_apps_srv, self._start_app_srv, self._stop_app_srv]]
        
        self._api_sync = MasterSync(self._interface_master,
                                    local_service_names=service_names,
                                    local_pub_names=pub_names)

        self._launch = None
        self._launches = {}
        self._interface_sync = None

        roslaunch.pmon._init_signal_handlers()

        if (self._exchange):
            self._exchange.update_local()

        self._app_list.update()
        self.publish_exchange_list_apps()
        self.publish_list_apps()
        
    def shutdown(self):
        if self._api_sync:
            self._api_sync.stop()
        if self._launch:
            self._launch.shutdown()
            self._interface_sync.stop()

    def _set_current_apps(self, apps, app_definitions):
        self._current_apps = apps
        self._current_app_definitions = app_definitions

        if self._list_apps_pub:
            self._list_apps_pub.publish(apps, self._app_list.get_app_list())
          
    def scoped_name(self, name):
        return roslib.names.canonicalize_name('/%s/%s'%(self._robot_name, rospy.remap_name(name)))

    def handle_get_app_details(self, req):
        return GetAppDetailsResponse(app=self._exchange.get_app_details(req.name))
    
    def handle_list_exchange_apps(self, req):
        if (self._exchange == None):
            return None
        if (req.remote_update):
            print "UPDATE"
            if (not self._exchange.update()):
                return None
        i_apps = self._exchange.get_installed_apps()
        a_apps = self._exchange.get_available_apps()
        return GetInstallationStateResponse(installed_apps=i_apps, available_apps=a_apps)

    def publish_list_apps(self):
        self._list_apps_pub.publish(self._current_apps, self._app_list.get_app_list())

    def publish_exchange_list_apps(self):
        if (self._exchange == None):
            return
        i_apps = self._exchange.get_installed_apps()
        a_apps = self._exchange.get_available_apps()
        self._exchange_list_apps_pub.publish(i_apps, a_apps)
    
    def handle_install_app(self, req):
        appname = req.name
        if (self._exchange.install_app(appname)):
            self._app_list.update()
            self.publish_list_apps()
            self.publish_exchange_list_apps()
            return InstallAppResponse(installed=True, message="app [%s] installed"%(appname))
        else:
            return InstallAppResponse(installed=False, message="app [%s] could not be installed"%(appname))

    def handle_uninstall_app(self, req):
        appname = req.name
        if (self._exchange.uninstall_app(appname)):
            self._app_list.update()
            self.publish_list_apps()
            self.publish_exchange_list_apps()
            return UninstallAppResponse(uninstalled=True, message="app [%s] uninstalled"%(appname))
        else:
            return UninstallAppResponse(uninstalled=False, message="app [%s] could not be uninstalled"%(appname))

    def handle_list_apps(self, req):
        rospy.loginfo("Listing apps")
        current = self._current_apps
        running_apps = current
        self._app_list.update()
        rospy.loginfo("done listing apps")
        return ListAppsResponse(running_apps=running_apps, available_apps=self._app_list.get_app_list())

    def handle_start_app(self, req):
        rospy.loginfo("start_app: %s"%(req.name))
        if len(self._current_apps) > 0:
            for app in self._current_app_definitions:
                if app.name == req.name:
                    return StartAppResponse(started=True, message="app [%s] already started"%(req.name), namespace=self._app_interface)
                elif (self._single_app):
                    return StartAppResponse(started=False, message="Please stop the running app before starting another app.", error_code=StatusCodes.MULTIAPP_NOT_SUPPORTED)

        # TODO: the app list has already loaded the App data.  We should use that instead for consistency

        appname = req.name
        rospy.loginfo("App Manager : loading app: %s"%(appname))
        try:
            app = load_AppDefinition_by_name(appname)
        except ValueError as e:
            return StartAppResponse(started=False, message=str(e), error_code=StatusCodes.BAD_REQUEST)
        except InvalidAppException as e:
            return StartAppResponse(started=False, message=str(e), error_code=StatusCodes.INTERNAL_ERROR)
        except NotFoundException as e:
            return StartAppResponse(started=False, message=str(e), error_code=StatusCodes.NOT_FOUND)

        try:
            app_definitions = self._current_app_definitions
            app_definitions.append(app)
            current_apps = self._current_apps
            current_apps.append(App(name=appname))
            self._set_current_apps(current_apps, app_definitions)

            rospy.loginfo("Launching: %s"%(app.launch))
            self._status_pub.publish(AppStatus(AppStatus.INFO, 'launching %s'%(app.display_name)))

            #TODO:XXX This is a roslaunch-caller-like abomination.  Should leverage a true roslaunch API when it exists.
            launch = roslaunch.parent.ROSLaunchParent(rospy.get_param("/run_id"),
                                                            [app.launch], is_core=False,
                                                            process_listeners=())

            self._launches[appname] = launch #if that doesn't work just do it the way they were doing it before. STOP TRYING TO BE CLEVER.
            launch._load_config()

            #TODO: coNvert to method
            for N in launch.config.nodes:
                for t in app.interface.published_topics.keys():
                    N.remap_args.append((t, self._app_interface + '/' + t))
                for t in app.interface.subscribed_topics.keys():
                    N.remap_args.append((t, self._app_interface + '/' + t))
            launch.start()

            fp = [self._app_interface + '/' + x for x in app.interface.subscribed_topics.keys()]
            lp = [self._app_interface + '/' + x for x in app.interface.published_topics.keys()]

            self._interface_sync = MasterSync(self._interface_master, foreign_pub_names=fp, local_pub_names=lp)

            thread.start_new_thread(self.app_monitor,())

            return StartAppResponse(started=True, message="app [%s] started"%(appname), namespace=self._app_interface)
        
        except Exception as e:
            try:
                # attempt to kill any launched resources
                #self._stop_current()
                try:
                    if (self._launches.has_key(appname)):
                        launch = self._launches[appname]
                        launch.shutdown()
                finally:
                    del self._launches[appname]
                try:
                    self._interface_sync.stop()
                finally:
                    self._interface_sync = None
            except:
                pass
            self._status_pub.publish(AppStatus(AppStatus.INFO, 'app start failed'))
            rospy.logerr("App Manager : app start failed")
            return StartAppResponse(started=False, message="internal error [%s]"%(str(e)), error_code=StatusCodes.INTERNAL_ERROR)

    def handle_stop_app(self, req):
        rospy.loginfo("AppManager : request received to stop application [%s]"%(req.name))
        return self.stop_app(req.name)

    def app_monitor(self):
        while self._launch:
            time.sleep(0.1)
            launch = self._launch
            if launch:
                pm = launch.pm
                if pm:
                    if pm.done:
                        time.sleep(1.0)
                        self.stop_apps()
                        break

    def stop_app(self, appname):
        resp = StopAppResponse(stopped=False)
        current_app_definitions = self._current_app_definitions
        current_apps = self._current_apps
        launches = self._launches
        try:
            for app in current_app_definitions:
                if app.name == appname:
                    launches[appname].shutdown()
                    app_to_stop = current_apps[current_app_definitions.index(app)]
                    current_apps.remove(app_to_stop)
                    current_app_definitions.remove(app)
                    self._set_current_apps(current_apps, current_app_definitions)
                    resp.stopped =True
        except Exception as e:
            rospy.logerr("handle stop app: internal error %s"%(e))
            resp.error_code = StatusCodes.INTERNAL_ERROR
            resp.message = "internal error: %s"%(str(e))
            
        return resp

    def stop_apps(self):
        resp = StopAppResponse(stopped=False)
        current_app_definitions = self._current_app_definitions
        current_apps = self._current_apps
        launches = self._launches
        try:
            for app in current_app_definitions:
                launches[app.name].shutdown()
                app_to_stop = current_apps[current_app_definitions.index(app)]
                current_apps.remove(app_to_stop)
                current_app_definitions.remove(app)
                self._set_current_apps(current_apps, current_app_definitions)
                resp.stopped =True
        except Exception as e:
            rospy.logerr("handle stop app: internal error %s"%(e))
            resp.error_code = StatusCodes.INTERNAL_ERROR
            resp.message = "internal error: %s"%(str(e))

        return resp
