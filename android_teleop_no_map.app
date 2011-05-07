display: Android Teleop
description: Drive TurtleBot with an Android device with a camera feed
platform: turtlebot
launch: turtlebot_teleop/android_teleop_no_map.launch
interface: turtlebot_teleop/android_teleop_no_map.interface
icon: turtlebot_teleop/android-lightning-turtlebot.png
clients:
 - type: android
   manager:
     api-level: 9
     intent-action: ros.android.teleop.Teleop
   app: 
     gravityMode: 0
