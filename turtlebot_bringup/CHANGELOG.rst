^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package turtlebot_bringup
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
