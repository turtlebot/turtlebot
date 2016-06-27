^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package turtlebot_description
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.3.12 (2016-06-27)
-------------------
* update xacro usage for jade deprecations
  comment out unused arguments generating errors
* [turtlebot_description] adds orbbec astra urdfs and mesh
* Contributors: Marcus Liebhardt, Tully Foote

2.3.11 (2015-04-15)
-------------------

2.3.10 (2015-04-02)
-------------------

2.3.9 (2015-03-30)
------------------

2.3.8 (2015-03-23)
------------------
* replace fbx model to open collada model closes `#198 <https://github.com/turtlebot/turtlebot/issues/198>`_
* Contributors: Jihoon Lee

2.3.7 (2015-03-02)
------------------

2.3.6 (2015-02-27)
------------------
* add reasons for comment `#194 <https://github.com/turtlebot/turtlebot/issues/194>`_
* update urdf. now new position uses asus_xtion_pro. Old position is asus_xtion_pro_offset
* add asus, mount, and new pole
* add urdf for asus center located version
* Contributors: Jihoon Lee

2.3.5 (2015-01-12)
------------------

2.3.4 (2015-01-07)
------------------

2.3.3 (2015-01-05)
------------------

2.3.2 (2014-12-30)
------------------

2.3.1 (2014-12-30)
------------------
* Issue `#172 <https://github.com/turtlebot/turtlebot/issues/172>`_: Provide mass and inertia real values for 3D camera, stacks and poles
* Contributors: corot

2.3.0 (2014-11-30)
------------------
* Check if unit-testing is enabled
  CATKIN_ENABLE_TESTING is a late addtion to catkin. This updates the
  package to comply with the strict checking rule.
* Contributors: Adam Lee

2.2.2 (2013-10-14)
------------------

2.2.1 (2013-09-14)
------------------
* prefix test target to not collide with other targets

2.2.0 (2013-08-29)
------------------
* Fix kinect's collision model misplacement.
* Change hexagon stack plates collision models to cylinders.
* Add bugtracker and repo info URLs.
* CLip camera view at 8 m, more closely replicating real kinect.
* Changelogs at package level.


2.1.x - hydro, unstable
=======================

2.1.1 (2013-08-06)
------------------
* Update Kinect Gazebo simulation parameters
* Remove graveyard folders
* Remove forgotten create meshes (now in create_description)

2.1.0 (2013-07-15)
------------------
* Catkinized
* Add roomba xacro files (same as create, since now roomba mesh is available)
* Add updates to xacros and urdfs for Gazebo simulation
* Fix broken turtlebot 1 visualisation (scaling and units in collada mesh)
* Add eclipse project files


Previous versions, bugfixing
============================

Available in ROS wiki: http://ros.org/wiki/turtlebot/ChangeList
