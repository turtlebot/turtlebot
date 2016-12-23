^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package turtlebot_bringup
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.3.14 (2016-12-22)
-------------------
* Modify R200 launch file.
  Modified the R200 3D sensor launch file
  to conform more closely to the other
  open_ni2-style sensor launch files. This
  allows the data processing arguments passed
  in to correctly pass through and control
  image processing.
  This allows the turtlebot_follower package,
  for example, to set "depth_processing" to
  true to enable the /camera/depth/depth_rect/
  topic with the R200 as it can with other
  cameras.
* Refactor cmd_vel_mux.
  Moved mobile_base_nodelet_manager and cmd_vel_mux to a
  common, higher-level launch file.
  Currently each base-specific mobile_base.launch.xml
  loaded the same cmd_vel_mux node with the same configuration.
  To reduce this redundancy, launching this nodelet has been
  moved to the common, higher-level mobile_base.launch.xml.
  The mobile_base_nodelet_manager has been moved up to facilitate
  this change.
* Remove concert reference to mobile_base.launch
  Added mobile_base.launch.xml back in for backward compatability
  with deprecation message.
* Remove unnecessary mobile_base.launch.xml layer
  mobile_base.launch.xml currently is just a pass-through to
  include the TURTLEBOT_BASE-specific launch file.  Simply
  removed this unnecessary layer.
* Contributors: Kevin C. Wells

2.3.13 (2016-11-01)
-------------------
* Add support for Intel R200 camera
  Added necessary launch, urdf, etc. files to
  add support for the R200 camera in Turtlebot.
  Updated r200 URDF to inclue proper mounting.
  Added runtime dependency on realsense_camera package.
* [bringup] remove outdated broken comment
* Contributors: Daniel Stonier, Kevin C. Wells

2.3.12 (2016-06-27)
-------------------
* update xacro usage for jade deprecations
  comment out unused arguments generating errors
* add support for Orbbec Astra
* add the teleop switch script as a default input to support joystick interactive follower
* use rocon_apps/make_a_map for make a map interaction closes `#210 <https://github.com/turtlebot/turtlebot/issues/210>`_
* Fix icon for interactions.
* Contributors: Jihoon Lee, Tully Foote, commaster90

2.3.11 (2015-04-15)
-------------------
* interaction fix closes `#208 <https://github.com/turtlebot/turtlebot/issues/208>`_
* Contributors: Jihoon Lee

2.3.10 (2015-04-02)
-------------------
* update interaction name regarding android pairing as change of android apps name
* Contributors: dwlee

2.3.9 (2015-03-30)
------------------
* add chirp in android pairing closes `#205 <https://github.com/turtlebot/turtlebot/issues/205>`_
* Contributors: Jihoon Lee

2.3.8 (2015-03-23)
------------------
* update compatibility for upgrade turtlebot android apps
* Merge pull request `#201 <https://github.com/turtlebot/turtlebot/issues/201>`_ from dwlee/indigo_teleop
  Update interaction
* [turtlebot_bringup] fix conflicts in the turtlebot base launchers
* [turtlebot_bringup] doc strings for roslaunch args.
* update android teleop remapping rule and add kitkat in compatibility
* add rapp preferred configuration
* expose the interactions list in a way that conforms to the rest of the turtlebot environment settings.
* doc'ified some url args.
* bugfixes incorrect concert_whitelist arg default, fixes `#199 <https://github.com/turtlebot/turtlebot/issues/199>`_.
* Merge branch 'indigo' of https://github.com/turtlebot/turtlebot into indigo
* visualisation interactions
* add ps3 joystick interactions closes `#196 <https://github.com/turtlebot/turtlebot/issues/196>`_
* Contributors: Daniel Stonier, Jihoon Lee, dwlee

2.3.7 (2015-03-02)
------------------
* switch openni driver to openni2 for asus xtion pro
* Contributors: Jihoon Lee

2.3.6 (2015-02-27)
------------------
* Merge pull request `#194 <https://github.com/turtlebot/turtlebot/issues/194>`_ from turtlebot/asus_center
  Configurable 3d sensor
* update urdf. now new position uses asus_xtion_pro. Old position is asus_xtion_pro_offset
* update env hook to  to use centered asus
* add 3dsensor aluncher
* asus is now default
* updates
* Merge branch 'indigo' into 3dsensor_config
* separate launchers for kinect and asus
* Contributors: Daniel Stonier, Jihoon Lee

2.3.5 (2015-01-12)
------------------
* bringup depend on capabilities. capabilities should not depend on bringup `#185 <https://github.com/turtlebot/turtlebot/issues/185>`_
* Contributors: Jihoon Lee

2.3.4 (2015-01-07)
------------------
* remove turtlebot_capabilities from run_depend closes `#185 <https://github.com/turtlebot/turtlebot/issues/185>`_
* Contributors: Jihoon Lee

2.3.3 (2015-01-05)
------------------
* add kobuki_capabilities and turtlebot_capabilities as run_depend in turtlebot_bring fixes `#184 <https://github.com/turtlebot/turtlebot/issues/184>`_
* Contributors: Jihoon Lee

2.3.2 (2014-12-30)
------------------

2.3.1 (2014-12-30)
------------------
* use env for rapp parsing
* use turtlebot as envinroment variable prefixes
* install interactions directory closes `#176 <https://github.com/turtlebot/turtlebot/issues/176>`_
* Contributors: Jihoon Lee

2.3.0 (2014-11-30)
------------------
* removing unused args
* move out turtlebot map file environment variable to turtlebot_navigation, refs `#163 <https://github.com/turtlebot/turtlebot/issues/163>`_.
* fixing typo in concert_client.launch
* add interaction icons to fix `#162 <https://github.com/turtlebot/turtlebot/issues/162>`_
* migrate linux_hardware as linux_peripheral_interfaces repo
* Revert "Adding the rosbridge setting for using rosbridge on pairing mode"
* align the arg
* add the rosbridge setting for using rosbridge on pairing mode
* Change env-vars to not overwrite already set vars
* concert version turtlebot
* proper remappings and use video_teleop virtual rapp
* Merge pull request `#148 <https://github.com/turtlebot/turtlebot/issues/148>`_ from turtlebot/irdevel
  Update to use environment hooks for turtlebot_bringup / Android pairing updates
* Move robot name and type to environment variable as mentioned in `#134 <https://github.com/turtlebot/turtlebot/issues/134>`_
* Fix compatibility uri's to filter out PC interactions on Android
* Split Android and PC Pairing into seperate roles
* Update to use env-hooks for Turtlebot
* Initial Android fixes
* Update to account for turtlebot_rapps/teleop change to implement rocon_apps/teleop
* cleanup legacy install rule. and remove concert directory which is not valid anymore
* Rolled android and qt make_a_map and map_nav into one
* Remove unncessary launchers as app manager and capabilities are rolled into minimal now
* Enable capabilities server for turtlebot on indigo
* Fix mapping for Qt teleop
* Bugfix changes - small
* Added more interactions
* Added interactions to Turtlebot on indigo, collapsed minimal_with_appmanager into just minimal
* better acceleration defaults after experimental observations.
* refactor turtlebot_core_apps -> turtlebot_rapps
* Remap cmd_vel for the calibration script
  It needs to match the turtlebot node in order to monitor for changes
* Load calibration on turtlebot bringup
* add depth argument to configure scan_processing. With this configuration scan works for both depth_regratation false and true
* add blacklist argument
* compatible with new app manager
* rapp exporting for new rocon_app_manager
* patches to keep the consistency of arguments `#114 <https://github.com/turtlebot/turtlebot/issues/114>`_
* Merge pull request `#114 <https://github.com/turtlebot/turtlebot/issues/114>`_ from mayrjohannes/hydro-devel
  Added serial port as parameter to launch files (Issue https://github.com/turtlebot/turtlebot/issues/111)
* Fixing "Error with diagnostics.yaml for roomba `#110 <https://github.com/turtlebot/turtlebot/issues/110>`_"
* updates capabilities-specific rosinstaller
* adds turtlebot_capabilities package and related changes
* Trivial comment spelling fix rhoomba -> roomba
* turtlebot_bringup: adds capabilities (server + default provider configs)
* adding name for rapp list
* Added serial port as parameter to launch files
  modified:   create/mobile_base.launch.xml
  modified:   kobuki/mobile_base.launch.xml
  modified:   mobile_base.launch.xml
  modified:   roomba/mobile_base.launch.xml
  modified:   ../minimal.launch
  Committer: mayrjohannes <joh.mayr@jku.at>
  Author: mayrjohannes <joh.mayr@jku.at>
* Contributors: Daniel Stonier, DongWook Lee, Jihoon Lee, Kenneth Bogert, Luka Čehovin, Marcus Liebhardt, Yujin, kentsommer, wheeled_robin

2.2.2 (2013-10-14)
------------------
* Rename cmd_vel_mux as yocs_cmd_vel_mux.
* Temporary fix for hub whitelists so pairing doesn't get distracted by concerts.

2.2.1 (2013-09-14)
------------------
* remove cmake install rule for now obsoleted upstart files.

2.2.0 (2013-08-29)
------------------
* convenient paired launchers (no uuids, auto-invitations).
* Robot description in paired master.
* Modularising robot description to use with paired masters.
* Remove outdated upstart directory.
* Rename include launchers to xyz.launch.xml.
* Changelogs at package level.
* Remove _mobile_base_soft.launch
* Do not use robot_pose_ekf for kobuki base. Instead, use imu for heading and encoders por x and y.
* A bunch of fixes on absolute and application namespaces
* Depend on turtlebot_description rather than the specific instances kobuki, create.
* Update dependency to openni_launch and remove component dependencies.


2.1.x - hydro, unstable
=======================

2.1.1 (2013-08-06)
------------------
* Fix TurtleBot name
* Add map manager rapp and rapp-related namespace changes to 3dsensor.launch
* Change 3dsensor.launch so we maximize use of openni_launch
* Use the new app manager
* Use the new app manager app list format. Remove turtlebot_sounds, as it's already included on rocon apps
* Public master for android app is 11311, and private master is 11312
* Add turtlebot2 icons

2.1.0 (2013-07-15)
------------------
* Catkinized
* Use more aggressive acceleration limits
* Use the new Rocon app manager: http://www.ros.org/wiki/rocon_app_manager


Previous versions, bugfixing
============================

Available in ROS wiki: http://ros.org/wiki/turtlebot/ChangeList
