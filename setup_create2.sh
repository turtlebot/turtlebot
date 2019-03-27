# This configures the environment variables for a create based turtlebot
# running the Turtlebot 2.0 software. This is necessary to run after
# setup.bash to ensure the create drivers and nodes are all correctly launched.
# Note that Create  and Create 2 have different baud rates. Create 2 is based on
# a Roomba 600 series, thus we use TURTLEBOT_BASE=roomba for Create 2 to avoid
# baudrate sci cable connection errors.
#
# You may wish to set the 3d sensor to asus_xtion_pro if you do not have a kinect
# though. While the kinect settings work for the asus in terms of 3d sensing (openni
# handles the abstraction) the asus setting makes sure the mesh shown in rviz/gazebo 
# is the asus.

export TURTLEBOT_BASE=roomba
export TURTLEBOT_STACKS=circles
export TURTLEBOT_3D_SENSOR=kinect
export TURTLEBOT_SIMULATION=false
