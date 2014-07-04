# Set some sane defaults for the turtlebot launch environment
export TURTLEBOT_BASE=kobuki                           # create, roomba
export TURTLEBOT_BATTERY=/sys/class/power_supply/BAT0  # /proc/acpi/battery/BAT0 in 2.6 or earlier kernels,  /sys/class/power_supply/ (kernels 3.0+) 
export TURTLEBOT_STACKS=hexagons                       # circles, hexagons
export TURTLEBOT_3D_SENSOR=kinect                      # kinect, asus_xtion_pro
export TURTLEBOT_SIMULATION=false
export TURTLEBOT_SERIAL_PORT=/dev/kobuki               # /dev/ttyUSB0, /dev/ttyS0


export ROBOT_NAME=turtlebot
export ROBOT_TYPE=turtlebot



export TURTLEBOT_MAP_FILE=`rospack find turtlebot_navigation`/maps/willow-2010-02-18-0.10.yaml