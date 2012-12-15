== Testing Model Views ==

Test the robot descriptions via the turtlebot_viz/turtlebot_rviz_launchers
and reconfiguring your environment variables.

== Turtlebot 2 ==

This is the default, so you don't need to set the variables, but for 
purposes of illustration, it is shown below.

> export TURTLEBOT_BASE=kobuki
> export TURTLEBOT_STACKS=hexagons
> export TURTLEBOT_3D_SENSOR=kinect
> roslaunch turtlebot_rviz_launchers view_model.launch

== Turtlebot 1 ==

> export TURTLEBOT_BASE=create
> export TURTLEBOT_STACKS=circles
> export TURTLEBOT_3D_SENSOR=kinect
> roslaunch turtlebot_rviz_launchers view_model.launch

== Switch 3d Sensor ==

Switch the 3d sensor from the kinect to the asus xtion pro:

> export TURTLEBOT_3D_SENSOR=asus_xtion_pro
> roslaunch turtlebot_rviz_launchers view_model.launch
