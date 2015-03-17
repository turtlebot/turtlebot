# Set some sane defaults for the turtlebot launch environment

##Documentation: 
#  The colon command simply has its arguments evaluated and then succeeds. 
#   It is the original shell comment notation (before '#' to end of line). For a long time, Bourne shell scripts had a colon as the first character. 
#   The C Shell would read a script and use the first character to determine whether it was for the C Shell (a '#' hash) or the Bourne shell (a ':' colon).
#   Then the kernel got in on the act and added support for '#!/path/to/program' and the Bourne shell got '#' comments, and the colon convention went by the wayside. 
#   But if you come across a script that starts with a colon (Like this one), now you will know why. ~ Jonathan Leffler

: ${TURTLEBOT_BASE:=kobuki}                           # create, roomba
: ${TURTLEBOT_BATTERY:=/sys/class/power_supply/BAT0}  # /proc/acpi/battery/BAT0 in 2.6 or earlier kernels,  /sys/class/power_supply/ (kernels 3.0+) 
: ${TURTLEBOT_STACKS:=hexagons}                       # circles, hexagons
: ${TURTLEBOT_3D_SENSOR:=asus_xtion_pro}              # kinect, asus_xtion_pro, asus_xtion_pro_offset
: ${TURTLEBOT_SIMULATION:=false}
: ${TURTLEBOT_SERIAL_PORT:=/dev/kobuki}               # /dev/ttyUSB0, /dev/ttyS0

: ${TURTLEBOT_NAME:=turtlebot}
: ${TURTLEBOT_TYPE:=turtlebot}
: ${TURTLEBOT_RAPP_PACKAGE_WHITELIST:=[rocon_apps, turtlebot_rapps]}
: ${TURTLEBOT_RAPP_PACKAGE_BLACKLIST:=[]}
: ${TURTLEBOT_INTERACTIONS_LIST:=[turtlebot_bringup/admin.interactions, turtlebot_bringup/documentation.interactions, turtlebot_bringup/pairing.interactions, turtlebot_bringup/visualisation.interactions]}

# Exports
export TURTLEBOT_BASE
export TURTLEBOT_BATTERY
export TURTLEBOT_STACKS
export TURTLEBOT_3D_SENSOR
export TURTLEBOT_SIMULATION
export TURTLEBOT_SERIAL_PORT
export TURTLEBOT_NAME
export TURTLEBOT_TYPE
export TURTLEBOT_RAPP_PACKAGE_WHITELIST
export TURTLEBOT_RAPP_PACKAGE_BLACKLIST
export TURTLEBOT_INTERACTIONS_LIST
