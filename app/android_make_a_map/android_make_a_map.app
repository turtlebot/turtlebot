display: Make a Map
description: Make a map by driving a Turtlebot from an Android device.
platform: turtlebot
launch: turtlebot_teleop/android_make_a_map.launch
interface: turtlebot_teleop/android_teleop.interface
icon: turtlebot_teleop/map.jpg
clients:
 - type: android
   manager:
     api-level: 9
     intent-action: ros.android.makeamap.MakeAMap
   app: 
     gravityMode: 0
     base_control_topic: /cmd_vel 
